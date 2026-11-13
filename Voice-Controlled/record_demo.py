#!/usr/bin/env python3
"""Record real ROS telemetry and WAV -> Whisper -> Nav2 as an annotated MP4.

Run with the voice venv after sourcing ROS/workspace. The isolated simulation
is started and stopped by this script. No desktop or ambient audio is captured.
"""

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import wave

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from nav_msgs.msg import Path as PathMessage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from robot_node.semantic_map import RegionCatalog, default_regions_file


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
COLORS = [(99, 218, 163), (102, 177, 255), (245, 137, 140), (190, 135, 245)]


def scenario_audio(output, scenario, language='en'):
    """Cache by the actual command so a renamed region never reuses old speech."""
    text = scenario['spoken_command']
    digest = hashlib.sha256((language+':'+text).encode()).hexdigest()[:12]
    assets = output / 'assets'
    assets.mkdir(exist_ok=True)
    filename = assets / f'{scenario["key"]}_{digest}.wav'
    if not filename.exists():
        from gtts import gTTS
        mp3 = filename.with_suffix('.mp3')
        gTTS(text=text, lang=language, timeout=30).save(str(mp3))
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', str(mp3),
                        '-ar', '16000', '-ac', '1', str(filename)], check=True)
    return filename


def font(size, bold=False):
    return ImageFont.truetype(BOLD if bold else FONT, size)


def text_wrap(draw, text, x, y, width, size=24, color='#e9edf7', bold=False, spacing=8):
    face = font(size, bold)
    line = ''
    for word in text.split():
        test = (line + ' ' + word).strip()
        if draw.textlength(test, font=face) > width and line:
            draw.text((x, y), line, font=face, fill=color)
            y += size + spacing
            line = word
        else:
            line = test
    if line:
        draw.text((x, y), line, font=face, fill=color)
        y += size + spacing
    return y


class Recorder(Node):
    def __init__(self, output, regions_file=None):
        super().__init__('voice_demo_recorder')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.catalog = RegionCatalog(regions_file or default_regions_file())
        self.grid = self.catalog.grid
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.status_text = 'Đang chuẩn bị hệ thống'
        self.speech_text = ''
        self.route = []
        self.active = None
        self.planned = []
        self.local_plan = []
        self.trail = []
        self.pose = None
        self.events = []
        self.results = []
        self.ready = False
        self.started = None
        self.phase = 'Chuẩn bị'
        self.chapter = 'TỔNG QUAN HỆ THỐNG'
        self.command_started = None
        self.audio = np.zeros(1)
        self.audio_duration = 0
        self.command_received = False
        self.audio_source_label = 'Giọng đọc mẫu tổng hợp · tiếng Anh'
        self.spoken_prompt = ''
        self.output = output
        self.journal = (output / 'events.jsonl').open('w')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/voice_navigation/status', self.on_status, latched)
        self.create_subscription(String, '/voice_command', self.on_command, 10)
        self.create_subscription(String, '/speech_recognition/status', self.on_speech, 10)
        self.create_subscription(PathMessage, '/voice_navigation/planned_path',
                                 lambda m: setattr(self, 'planned', m.poses), latched)
        self.create_subscription(PathMessage, '/plan',
                                 lambda m: setattr(self, 'local_plan', m.poses), 10)
        self.base_map = self.make_map()

    def elapsed(self):
        return 0.0 if self.started is None else time.monotonic() - self.started

    def log(self, kind, value):
        event = {'time_sec': round(self.elapsed(), 3), 'kind': kind, 'value': value}
        self.journal.write(json.dumps(event, ensure_ascii=False) + '\n')
        self.journal.flush()
        print(json.dumps(event, ensure_ascii=False), flush=True)

    def on_status(self, message):
        self.status_text = message.data
        self.events.append((self.elapsed(), message.data))
        self.log('navigation_status', message.data)
        if message.data.startswith('READY'):
            self.ready = True
        if message.data.startswith('NAVIGATING'):
            self.phase = 'ROBOT ĐANG DI CHUYỂN'
            self.active = message.data.rsplit('→', 1)[-1].strip()
        if message.data.startswith('PLANNING'):
            self.phase = 'NAV2 ĐANG LẬP KẾ HOẠCH'
        if message.data.startswith('SUCCEEDED'):
            self.phase = 'ĐÃ ĐẾN ĐÍCH'
            self.active = None
        if message.data.startswith('INVALID_COMMAND'):
            self.phase = 'TỪ CHỐI KHU VỰC KHÔNG TỒN TẠI'

    def on_command(self, message):
        self.speech_text = message.data
        self.command_received = True
        self.log('whisper_transcript', message.data)
        try:
            self.route = self.catalog.parse(message.data)
        except ValueError:
            self.route = []

    def on_speech(self, message):
        if message.data == 'TRANSCRIBING':
            self.phase = 'WHISPER ĐANG NHẬN DẠNG'
        self.log('speech_status', message.data)

    def map_point(self, x, y):
        u, v = self.grid.world_to_pixel(float(x), float(y))
        return (48 + (u + 0.5)*720/self.grid.width,
                124 + (v + 0.5)*720/self.grid.height)

    def make_map(self):
        labels = np.load(self.catalog.filename.parent / 'labels.npy', allow_pickle=False)
        if labels.shape != self.grid.free.shape or labels.max() > len(self.catalog.regions):
            raise ValueError('labels.npy không khớp catalog của bản đồ.')
        image = np.repeat(self.grid.image[:, :, None], 3, axis=2)
        for index in range(1, len(self.catalog.regions)+1):
            color = COLORS[(index-1) % len(COLORS)]
            mask = labels == index
            image[mask] = (image[mask]*0.45 + np.array(color)*0.55).astype(np.uint8)
        return Image.fromarray(image).resize(
            (720, 720), getattr(Image, 'Resampling', Image).NEAREST)

    def update_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return
        p, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        self.pose = (p.x, p.y, yaw)
        if not self.trail or math.hypot(p.x-self.trail[-1][0], p.y-self.trail[-1][1]) > 0.025:
            self.trail.append((p.x, p.y))
            if self.started is not None:
                self.journal.write(json.dumps({'time_sec': round(self.elapsed(), 3),
                                              'kind': 'robot_pose',
                                              'value': list(self.pose)})+'\n')

    def draw(self):
        self.update_pose()
        canvas = Image.new('RGB', (1600, 900), '#0c1220')
        draw = ImageDraw.Draw(canvas)
        draw.text((40, 22), 'TURTLEBOT4  /  ĐIỀU HƯỚNG GIỌNG NÓI',
                  font=font(29, True), fill='#ffffff')
        draw.text((40, 67), self.chapter, font=font(19), fill='#a7b8ce')
        draw.rounded_rectangle((1390, 25, 1555, 69), 12, fill='#1a2940')
        elapsed = int(self.elapsed())
        draw.text((1412, 32), f'{elapsed//60:02d}:{elapsed%60:02d}  LIVE',
                  font=font(19, True), fill='#67dda8')
        canvas.paste(self.base_map, (48, 124))
        draw.rectangle((47, 123, 769, 845), outline='#42516b', width=2)
        for poses, color, width in [(self.planned, '#f0be45', 5),
                                     (self.local_plan, '#ffffff', 3)]:
            points = [self.map_point(p.pose.position.x, p.pose.position.y) for p in poses]
            if len(points) > 1:
                draw.line(points, fill=color, width=width)
        if len(self.trail) > 1:
            draw.line([self.map_point(x, y) for x, y in self.trail], fill='#06a8b7', width=4)
        for index, region in enumerate(self.catalog.regions):
            goal = region['goal']
            x, y = self.map_point(goal['x'], goal['y'])
            color = COLORS[index % len(COLORS)]
            if region['id'] == self.active:
                r = 22 + int(3*math.sin(self.elapsed()*5))
                draw.ellipse((x-r, y-r, x+r, y+r), outline='#ffffff', width=3)
            draw.ellipse((x-9, y-9, x+9, y+9), fill=color, outline='#10202c', width=3)
            label = (region['id'] if region['id'] == region['name'] else
                     f'{region["id"]} · {region["name"]}')
            w = draw.textlength(label, font=font(17, True))
            lx = min(x+13, 760-w-14)
            ly = y-17
            draw.rounded_rectangle((lx, ly, lx+13+w, ly+32), 6, fill='#101c2c')
            draw.text((lx+6, ly+3), label, font=font(17, True), fill='#ffffff')
        if self.pose:
            px, py = self.map_point(*self.pose[:2])
            yaw = self.pose[2] - self.grid.origin[2]
            draw.ellipse((px-14, py-14, px+14, py+14), fill='#152943',
                         outline='#ffffff', width=3)
            tip = (px+25*math.cos(yaw), py-25*math.sin(yaw))
            draw.line([(px, py), tip], fill='#ffffff', width=5)
            draw.text((56, 803), f'Robot: x={self.pose[0]:+.2f} m   y={self.pose[1]:+.2f} m',
                      font=font(16, True), fill='#0c1220', stroke_width=2, stroke_fill='#ffffff')
        # Right-side cards display received data, never assumed recognition/results.
        for top, bottom in [(120, 310), (328, 475), (493, 625), (643, 844)]:
            draw.rounded_rectangle((810, top, 1554, bottom), 16, fill='#162236')
        draw.text((835, 139), '01  ÂM THANH ĐẦU VÀO', font=font(18, True), fill='#9eb5d4')
        draw.text((835, 176), self.audio_source_label, font=font(21), fill='#ffffff')
        if self.spoken_prompt:
            text_wrap(draw, self.spoken_prompt, 835, 209, 680, size=16)
        center = 280 if self.spoken_prompt else 251
        progress = 0.0 if self.command_started is None else time.monotonic()-self.command_started
        if self.audio_duration > 0 and 0 <= progress < self.audio_duration:
            sample = int(progress*16000)
            sample_slice = self.audio[max(0, sample-5000):sample+5000]
            if len(sample_slice):
                bins = np.array_split(sample_slice, 100)
                for i, chunk in enumerate(bins):
                    amplitude = max(3, min(18 if self.spoken_prompt else 32,
                                           float(np.max(np.abs(chunk)))*65))
                    x = 839 + i*6.6
                    draw.line((x, center-amplitude, x, center+amplitude), fill='#68dfb4', width=3)
        else:
            draw.line((839, center, 1509, center), fill='#42516b', width=2)
        draw.text((835, 346), '02  VĂN BẢN WHISPER NHẬN DẠNG', font=font(18, True), fill='#9eb5d4')
        text_wrap(draw, self.speech_text or 'Đang chờ kết quả nhận dạng…',
                  835, 383, 680, size=24, color='#ffffff')
        draw.text((835, 510), '03  CÁC ĐÍCH ĐÃ XÁC ĐỊNH', font=font(18, True), fill='#9eb5d4')
        if self.route:
            draw.text((835, 548), ' → '.join(self.route), font=font(33, True), fill='#f0be45')
            details = '   |   '.join(f'{i+1}. {self.catalog.by_id[r]["name"]}'
                                    for i, r in enumerate(self.route))
            text_wrap(draw, details, 835, 593, 680, size=15)
        else:
            draw.text((835, 555), 'Chưa có đích hợp lệ', font=font(24), fill='#b5c4d8')
        draw.text((835, 660), '04  ĐIỀU HƯỚNG NAV2', font=font(18, True), fill='#9eb5d4')
        text_wrap(draw, self.phase, 835, 695, 680, size=24, color='#69dfaf', bold=True)
        text_wrap(draw, self.status_text, 835, 744, 680, size=17)
        draw.text((48, 864), 'Vàng: đường dự kiến   ·   Trắng: kế hoạch hiện tại   ·   Xanh: quỹ đạo đã đi',
                  font=font(15), fill='#a7b8ce')
        draw.text((820, 864), 'Whisper local → ROS 2 Humble → Nav2 / Gazebo · Dữ liệu ROS trực tiếp',
                  font=font(14), fill='#a7b8ce')
        return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'Voice-Controlled/demo_current')
    parser.add_argument('--regions-file', default=default_regions_file())
    parser.add_argument('--model', default='small')
    parser.add_argument('--language', choices=['en', 'vi'], default='en')
    parser.add_argument('--preview-only', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    (output / 'logs').mkdir(exist_ok=True)
    if not os.environ.get('ROS_DOMAIN_ID') or os.environ['ROS_DOMAIN_ID'] == '69':
        raise RuntimeError('Use a separate ROS_DOMAIN_ID, for example 89, to isolate the demo.')
    rclpy.init()
    node = Recorder(output, args.regions_file)
    scenarios = node.catalog.demo_scenarios(args.language)
    node.audio_source_label = 'Giọng đọc mẫu tổng hợp · ' + args.language
    processes, logs = [], []
    writer = None
    fps = 15
    frames = 0
    audio_tracks = []
    results = []
    poster_saved = False

    def tick():
        nonlocal frames, poster_saved
        rclpy.spin_once(node, timeout_sec=0.005)
        if writer is not None:
            expected = int(node.elapsed()*fps)
            if frames <= expected:
                picture = node.draw()
                if (len(node.route) >= 2 and node.active == node.route[len(node.route)//2]
                        and not poster_saved):
                    picture.save(output / 'poster.png')
                    poster_saved = True
                frame = picture.tobytes()
                while frames <= expected:
                    writer.stdin.write(frame)
                    frames += 1

    def wait(predicate, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            tick()
            if predicate():
                return True
            time.sleep(0.005)
        return False

    def launch(name, command):
        stream = (output / 'logs' / f'{name}.log').open('w')
        logs.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    try:
        if args.preview_only:
            node.draw().save(output / 'preview.png')
            return
        audio_files = {s['key']: scenario_audio(output, s, args.language) for s in scenarios}
        launch('gazebo', ['ros2', 'launch', 'robot_sim', 'sim.launch.py',
                          'headless:=true', 'enable_camera:=false'])
        launch('localization', ['ros2', 'launch', 'robot_navigation',
                                'localization.launch.py', 'rviz:=false', 'set_initial_pose:=true',
                                f'map:={node.catalog.grid.filename}'])
        launch('nav2', ['ros2', 'launch', 'robot_navigation', 'navigation.launch.py'])
        launch('voice_navigation', ['ros2', 'launch', 'robot_node', 'voice_navigation.launch.py',
                                   f'regions_file:={node.catalog.filename}'])
        if not wait(lambda: node.ready, 70):
            raise RuntimeError('Semantic map not ready; inspect demo/logs.')
        wait(lambda: False, 15)
        node.update_pose()
        if node.pose is None:
            raise RuntimeError('No live map->base_link transform.')
        writer = subprocess.Popen([
            'ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
            '-s', '1600x900', '-r', str(fps), '-i', '-', '-an', '-c:v', 'libx264',
            '-preset', 'veryfast', '-crf', '20', '-pix_fmt', 'yuv420p',
            str(output / 'demo_silent.mp4')], stdin=subprocess.PIPE)
        node.started = time.monotonic()
        node.phase = 'SẴN SÀNG NHẬN LỆNH'
        node.log('recording_started', {'audio_source': f'synthetic {args.language} speech',
                                     'pose_source': 'live map->base_link TF',
                                     'resolution': '1600x900', 'fps': fps})
        wait(lambda: False, 3)
        for index, scenario in enumerate(scenarios, 1):
            name, title, invalid = scenario['key'], scenario['title'], scenario['invalid']
            node.chapter = f'DEMO {index}/{len(scenarios)}  ·  {title}'
            node.phase = 'ĐANG NHẬN ÂM THANH MẪU'
            node.speech_text = ''
            node.route = []
            node.active = None
            node.command_received = False
            filename = audio_files[name]
            with wave.open(str(filename)) as audio:
                if audio.getframerate() != 16000 or audio.getnchannels() != 1:
                    raise ValueError('Demo WAVs must be 16 kHz mono PCM16.')
                node.audio = np.frombuffer(audio.readframes(audio.getnframes()),
                                           dtype=np.int16).astype(float)/32768
            node.audio_duration = len(node.audio)/16000
            node.command_started = time.monotonic()
            audio_tracks.append((node.elapsed(), str(filename)))
            node.log('audio_input', {'file': filename.name, 'source': 'synthetic speech'})
            wait(lambda: False, node.audio_duration)
            node.phase = 'WHISPER ĐANG NHẬN DẠNG'
            # Use the actual project speech node; the recorder never publishes /voice_command.
            speech = launch(name, [str(WORKSPACE / '.venv-voice/bin/python'), '-m',
                                   'robot_node.speech_recognition', '--ros-args',
                                   '-p', f'audio_file:={filename}', '-p', f'model:={args.model}',
                                   '-p', f'language:={args.language}',
                                   '-p', f'regions_file:={node.catalog.filename}'])
            before = len(node.events)
            terminal = ('INVALID_COMMAND',) if invalid else (
                'SUCCEEDED', 'FAILED', 'REJECTED', 'NOT_READY', 'STOPPING', 'INVALID_COMMAND')
            def completed():
                if speech.poll() not in (None, 0):
                    raise RuntimeError(f'Speech node failed: {name}; inspect demo/logs.')
                # Status and transcript arrive on independent ROS subscriptions.
                return node.command_received and any(
                    s.startswith(terminal) for _, s in node.events[before:])
            if not wait(completed, 240):
                raise RuntimeError(f'Timed out: {name}; speech exit={speech.poll()}')
            actual = next(s for _, s in node.events[before:] if s.startswith(terminal))
            expected = 'INVALID_COMMAND' if invalid else 'SUCCEEDED'
            if (not actual.startswith(expected) or not node.command_received or
                    node.route != scenario['route']):
                raise RuntimeError(f'Unexpected result in {name}: {actual}')
            result = {'scenario': name, 'transcript': node.speech_text,
                      'route': list(node.route), 'result': actual,
                      'time_sec': round(node.elapsed(), 2)}
            results.append(result)
            node.log('scenario_completed', result)
            wait(lambda: False, 4)
        node.chapter = f'HOÀN THÀNH  ·  {len(scenarios)-1} LỆNH HỢP LỆ + 1 LỆNH KHU VỰC LẠ'
        node.phase = 'DEMO ĐÃ HOÀN THÀNH'
        node.status_text = 'Whisper → xác định đích → Nav2 di chuyển: đạt. Vùng lạ được từ chối.'
        wait(lambda: False, 6)
        if not poster_saved:
            node.draw().save(output / 'poster.png')
        writer.stdin.close()
        if writer.wait(timeout=30) != 0:
            raise RuntimeError('Video encoding failed.')
        writer = None
        duration = frames/fps
        command = ['ffmpeg', '-y', '-loglevel', 'error', '-i', str(output / 'demo_silent.mp4')]
        filters, names = [], []
        for index, (start, filename) in enumerate(audio_tracks, 1):
            command += ['-i', filename]
            filters.append(f'[{index}:a]adelay={round(start*1000)}:all=1[a{index}]')
            names.append(f'[a{index}]')
        filters.append(''.join(names)+f'amix=inputs={len(names)}:normalize=0,apad[audio]')
        command += ['-filter_complex', ';'.join(filters), '-map', '0:v', '-map', '[audio]',
                    '-c:v', 'copy', '-c:a', 'aac', '-b:a', '128k', '-t', str(duration),
                    '-movflags', '+faststart', str(output / 'turtlebot4_voice_demo.mp4')]
        subprocess.run(command, check=True)
        (output / 'manifest.json').write_text(json.dumps({
            'created': datetime.now().astimezone().isoformat(), 'duration_sec': duration,
            'fps': fps, 'resolution': [1600, 900], 'audio_source': 'synthetic speech',
            'simulation': 'Gazebo Fortress small_room', 'asr': f'Whisper {args.model} local CPU',
            'navigation': 'real Nav2 actions', 'scenarios': results,
            'regions_file': str(node.catalog.filename), 'catalog_revision': node.catalog.revision,
            'language': args.language,
            'audio_tracks': audio_tracks,
        }, ensure_ascii=False, indent=2))
        (output / 'demo_silent.mp4').unlink()
        print(f'VIDEO_READY: {output / "turtlebot4_voice_demo.mp4"}', flush=True)
    finally:
        if writer is not None:
            writer.stdin.close()
            writer.wait(timeout=20)
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
        for stream in logs:
            stream.close()
        node.journal.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
