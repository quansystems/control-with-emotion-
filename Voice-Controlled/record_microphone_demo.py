#!/usr/bin/env python3
"""Interactive microphone recording. Capture starts only after the user clicks."""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import wave

import numpy as np
from PyQt5.QtCore import QLibraryInfo, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QComboBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget
import rclpy
import sounddevice as sd

from record_demo import Recorder, ROOT, WORKSPACE
from robot_node.semantic_map import default_regions_file


class Panel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('TurtleBot4 — Quay demo bằng microphone của bạn')
        self.requested = False
        self.closed = False
        self.image = QLabel()
        self.image.setAlignment(Qt.AlignCenter)
        self.hint = QLabel('Đang chuẩn bị Gazebo và Nav2. Microphone chưa thu.')
        self.hint.setWordWrap(True)
        self.hint.setStyleSheet('font-size: 18px; color: white; padding: 8px;')
        self.button = QPushButton('Thu lệnh')
        self.button.setEnabled(False)
        self.button.setMinimumHeight(45)
        self.button.setStyleSheet('font-size: 18px; padding: 8px;')
        self.button.clicked.connect(self.clicked)
        self.device = QComboBox()
        self.device.addItem('Microphone mặc định của hệ thống', -1)
        for index, device in enumerate(sd.query_devices()):
            if device['max_input_channels'] > 0:
                self.device.addItem(f'{index}: {device["name"]}', index)
        row = QHBoxLayout()
        row.addWidget(self.device, 2)
        row.addWidget(self.button, 1)
        layout = QVBoxLayout(self)
        layout.addWidget(self.image)
        layout.addWidget(self.hint)
        layout.addLayout(row)
        self.setStyleSheet('background: #0c1220; color: white;')
        self.resize(1300, 850)
        self.show()

    def clicked(self):
        self.requested = True
        self.button.setEnabled(False)

    def closeEvent(self, event):
        self.closed = True
        event.accept()

    def show_frame(self, picture):
        data = picture.tobytes()
        image = QImage(data, picture.width, picture.height,
                       picture.width*3, QImage.Format_RGB888).copy()
        self.image.setPixmap(QPixmap.fromImage(image).scaled(
            1280, 720, Qt.KeepAspectRatio, Qt.SmoothTransformation))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'Voice-Controlled/demo_microphone_current')
    parser.add_argument('--regions-file', default=default_regions_file())
    parser.add_argument('--model', default='small')
    parser.add_argument('--language', choices=['en', 'vi'], default='en')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--preview-only', action='store_true')
    options = parser.parse_args()
    if not 1 <= options.seconds <= 30:
        raise ValueError('seconds must be in [1,30].')
    if not os.environ.get('ROS_DOMAIN_ID') or os.environ['ROS_DOMAIN_ID'] == '69':
        raise RuntimeError('Use an isolated ROS_DOMAIN_ID, e.g. 90.')
    output = options.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    for directory in ('assets', 'logs'):
        (output / directory).mkdir(exist_ok=True)
    # cv2 can inject its private Qt plugin directory when semantic_map imports it.
    # The preview uses system PyQt5, so select the matching platform plugins.
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = QLibraryInfo.location(QLibraryInfo.PluginsPath)
    os.environ.pop('QT_QPA_FONTDIR', None)
    app = QApplication([])
    panel = Panel()
    rclpy.init()
    node = Recorder(output, options.regions_file)
    scenarios = node.catalog.demo_scenarios(options.language)
    node.audio_source_label = 'Microphone trực tiếp · giọng nói của bạn'
    node.spoken_prompt = 'Nhấn Thu lệnh khi bạn sẵn sàng.'
    processes, streams, audio_tracks, results = [], [], [], []
    writer = None
    frames, fps = 0, 15
    last_frame = 0.0
    recording_finished = False

    def launch(name, command):
        stream = (output / 'logs' / f'{name}.log').open('w')
        streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    def tick():
        nonlocal frames, last_frame
        app.processEvents()
        if panel.closed:
            raise KeyboardInterrupt
        rclpy.spin_once(node, timeout_sec=0.005)
        now = time.monotonic()
        if now-last_frame < 1/fps:
            return
        picture = node.draw()
        panel.show_frame(picture)
        last_frame = now
        if writer is not None:
            expected = int(node.elapsed()*fps)
            while frames <= expected:
                writer.stdin.write(picture.tobytes())
                frames += 1

    def wait(predicate, timeout=None):
        deadline = float('inf') if timeout is None else time.monotonic()+timeout
        while time.monotonic() < deadline:
            tick()
            if predicate():
                return True
            time.sleep(0.005)
        return False

    def open_video():
        nonlocal writer
        writer = subprocess.Popen([
            'ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
            '-s', '1600x900', '-r', str(fps), '-i', '-', '-an', '-c:v', 'libx264',
            '-preset', 'veryfast', '-crf', '20', '-pix_fmt', 'yuv420p',
            str(output / 'microphone_silent.mp4')], stdin=subprocess.PIPE)
        node.started = time.monotonic()
        node.log('recording_started', {'audio_source': 'live user microphone'})

    try:
        if options.preview_only:
            node.draw().save(output / 'preview.png')
            return
        launch('gazebo', ['ros2', 'launch', 'robot_sim', 'sim.launch.py',
                          'headless:=true', 'enable_camera:=false'])
        launch('localization', ['ros2', 'launch', 'robot_navigation',
                                'localization.launch.py', 'rviz:=false', 'set_initial_pose:=true',
                                f'map:={node.catalog.grid.filename}'])
        launch('nav2', ['ros2', 'launch', 'robot_navigation', 'navigation.launch.py'])
        launch('voice_navigation', ['ros2', 'launch', 'robot_node', 'voice_navigation.launch.py',
                                   f'regions_file:={node.catalog.filename}'])
        if not wait(lambda: node.ready, 75):
            raise RuntimeError('Map chưa sẵn sàng; kiểm tra logs.')
        wait(lambda: False, 15)
        node.update_pose()
        if node.pose is None:
            raise RuntimeError('Chưa có pose robot từ TF.')
        node.phase = 'SẴN SÀNG — MICROPHONE CHƯA THU'
        node.log('waiting_for_user', 'Click Thu lệnh in the desktop window to begin.')
        for index, scenario in enumerate(scenarios, 1):
            title, prompt, invalid = scenario['title'], scenario['spoken_command'], scenario['invalid']
            attempt = 0
            while True:
                attempt += 1
                name = f'{index:02d}_take_{attempt:02d}'
                node.chapter = f'MICROPHONE {index}/{len(scenarios)}  ·  {title}'
                node.spoken_prompt = f'Câu cần đọc: {prompt}'
                node.phase = 'NHẤN THU LỆNH KHI BẠN SẴN SÀNG'
                panel.hint.setText(f'Lượt {index}/{len(scenarios)} — {prompt}\n'
                                   f'Nhấn Thu lệnh; đợi đếm ngược rồi đọc trong {options.seconds:g} giây.')
                panel.requested = False
                panel.button.setEnabled(True)
                wait(lambda: panel.requested)
                if writer is None:
                    open_video()
                for countdown in (3, 2, 1):
                    node.phase = f'CHUẨN BỊ ĐỌC SAU {countdown} GIÂY'
                    panel.hint.setText(f'{countdown}… Chuẩn bị: {prompt}')
                    wait(lambda: False, 1)
                node.command_received = False
                node.speech_text = ''
                node.route = []
                node.active = None
                node.phase = 'ĐANG THU MICROPHONE — MỜI BẠN ĐỌC'
                panel.hint.setText(f'ĐANG THU — Hãy đọc: {prompt}')
                chunks = []
                audio_errors = []
                def capture(data, count, timing, status):
                    if status:
                        audio_errors.append(str(status))
                    chunks.append(data[:, 0].copy())
                device = panel.device.currentData()
                node.audio_duration = options.seconds
                with sd.InputStream(samplerate=16000, channels=1, dtype='float32',
                                    device=None if device < 0 else device,
                                    callback=capture):
                    node.command_started = time.monotonic()
                    start = node.elapsed()
                    node.log('microphone_started', {'device': device, 'take': name})
                    deadline = time.monotonic()+options.seconds
                    while time.monotonic() < deadline:
                        node.audio = np.concatenate(list(chunks)) if chunks else np.zeros(1)
                        tick()
                        time.sleep(0.005)
                node.audio = np.concatenate(chunks) if chunks else np.zeros(1)
                filename = output / 'assets' / f'{name}.wav'
                with wave.open(str(filename), 'wb') as audio:
                    audio.setnchannels(1)
                    audio.setsampwidth(2)
                    audio.setframerate(16000)
                    audio.writeframes((np.clip(node.audio, -1, 1)*32767).astype('<i2').tobytes())
                audio_tracks.append((start, str(filename)))
                node.log('microphone_saved', {'file': filename.name, 'warnings': audio_errors})
                if float(np.sqrt(np.mean(node.audio**2))) < 0.005:
                    panel.hint.setText('Âm thanh quá nhỏ. Kiểm tra/chọn microphone rồi thu lại.')
                    node.phase = 'ÂM THANH QUÁ NHỎ — CẦN THU LẠI'
                    wait(lambda: False, 3)
                    continue
                panel.hint.setText('Đang nhận dạng và điều hướng. Microphone đã ngừng thu.')
                node.phase = 'WHISPER ĐANG NHẬN DẠNG'
                before = len(node.events)
                speech = launch(name, [str(WORKSPACE / '.venv-voice/bin/python'), '-m',
                    'robot_node.speech_recognition', '--ros-args', '-p', f'audio_file:={filename}',
                    '-p', f'model:={options.model}', '-p', f'language:={options.language}',
                    '-p', f'regions_file:={node.catalog.filename}'])
                speech_started = time.monotonic()
                terminal = ('SUCCEEDED', 'INVALID_COMMAND', 'FAILED', 'REJECTED',
                            'NOT_READY', 'STOPPED', 'ACTION_FAULT')
                def complete():
                    if speech.poll() not in (None, 0):
                        raise RuntimeError('Node giọng nói gặp lỗi; xem logs.')
                    if (speech.poll() == 0 and not node.command_received and
                            time.monotonic()-speech_started > 15):
                        return True
                    return node.command_received and any(
                        s.startswith(terminal) for _, s in node.events[before:])
                if not wait(complete, 240):
                    raise RuntimeError('Chưa có kết quả giọng nói/Nav2; xem logs.')
                if not node.command_received:
                    node.phase = 'CHƯA NHẬN RÕ GIỌNG NÓI — CẦN THU LẠI'
                    panel.hint.setText('Whisper chưa nhận rõ giọng nói. Nhấn Thu lệnh để thử lại.')
                    wait(lambda: False, 3)
                    continue
                actual = next(s for _, s in node.events[before:] if s.startswith(terminal))
                expected = 'INVALID_COMMAND' if invalid else 'SUCCEEDED'
                result = {'scenario': index, 'take': attempt, 'transcript': node.speech_text,
                          'route': list(node.route), 'result': actual, 'time_sec': node.elapsed()}
                results.append(result)
                node.log('scenario_completed', result)
                if actual.startswith(expected) and node.route == scenario['route']:
                    node.draw().save(output / 'poster.png')
                    wait(lambda: False, 3)
                    break
                panel.hint.setText(f'Kết quả: {actual}\nCó thể đọc lại câu gợi ý.')
                wait(lambda: False, 3)
        node.chapter = 'DEMO BẰNG GIỌNG NÓI CỦA BẠN — HOÀN THÀNH'
        node.phase = 'ĐÃ HOÀN THÀNH CÁC LƯỢT THỬ'
        panel.hint.setText('Đã quay xong. Đang xuất video có tiếng…')
        wait(lambda: False, 5)
        writer.stdin.close()
        if writer.wait(timeout=30) != 0:
            raise RuntimeError('Mã hóa video thất bại.')
        writer = None
        duration = frames/fps
        command = ['ffmpeg', '-y', '-loglevel', 'error', '-i', str(output/'microphone_silent.mp4')]
        filters, names = [], []
        for index, (start, filename) in enumerate(audio_tracks, 1):
            command += ['-i', filename]
            filters.append(f'[{index}:a]adelay={round(start*1000)}:all=1[a{index}]')
            names.append(f'[a{index}]')
        filters.append(''.join(names)+f'amix=inputs={len(names)}:normalize=0,apad[audio]')
        video = output / 'turtlebot4_voice_microphone_demo.mp4'
        command += ['-filter_complex', ';'.join(filters), '-map', '0:v', '-map', '[audio]',
                    '-c:v', 'copy', '-c:a', 'aac', '-t', str(duration),
                    '-movflags', '+faststart', str(video)]
        subprocess.run(command, check=True)
        (output/'manifest.json').write_text(json.dumps({
            'created': datetime.now().astimezone().isoformat(), 'duration_sec': duration,
            'audio_source': 'live user microphone', 'scenarios': results,
            'regions_file': str(node.catalog.filename), 'catalog_revision': node.catalog.revision,
            'language': options.language,
            'audio_tracks': audio_tracks, 'fps': fps, 'resolution': [1600, 900],
        }, ensure_ascii=False, indent=2))
        (output/'microphone_silent.mp4').unlink()
        recording_finished = True
        panel.hint.setText(f'ĐÃ LƯU VIDEO: {video}')
        node.log('video_ready', str(video))
        wait(lambda: False, 5)
    except KeyboardInterrupt:
        print('Đã đóng phiên thu.', flush=True)
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
        for stream in streams:
            stream.close()
        node.journal.close()
        node.destroy_node()
        rclpy.shutdown()
        panel.close()
        if not recording_finished:
            print('Chưa có video microphone hoàn chỉnh.', flush=True)


if __name__ == '__main__':
    main()
