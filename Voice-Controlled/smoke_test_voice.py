#!/usr/bin/env python3
"""Check audio -> project Whisper node -> real Nav2/Gazebo in an isolated domain."""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from robot_node.semantic_map import RegionCatalog, default_regions_file


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--regions-file', default=default_regions_file())
    parser.add_argument('--audio-file', type=Path)
    parser.add_argument('--audio-source', choices=['file', 'synthetic'], default='file')
    parser.add_argument('--model', default='small')
    parser.add_argument('--language', choices=['en', 'vi'], default='en')
    parser.add_argument('--output', type=Path, default=ROOT/'checks/latest')
    parser.add_argument('--expected-route', nargs='+')
    options = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None, '0', '69'):
        raise ValueError('Use an isolated domain, for example ROS_DOMAIN_ID=92.')
    catalog = RegionCatalog(options.regions_file)
    scenarios = catalog.demo_scenarios(options.language)
    scenario = next((s for s in scenarios if s['key'] == 'multi_stop'), scenarios[0])
    expected = options.expected_route or scenario['route']
    output = options.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    logs_directory = output/'logs'
    logs_directory.mkdir(exist_ok=True)
    audio_file = options.audio_file
    if audio_file is None:
        from gtts import gTTS
        audio_file = output/'navigation_command.mp3'
        gTTS(text=scenario['spoken_command'], lang=options.language, timeout=30).save(str(audio_file))
        options.audio_source = 'synthetic'
    audio_file = audio_file.resolve()
    if not audio_file.is_file():
        raise ValueError(f'Audio file not found: {audio_file}')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('voice_pipeline_smoke_test')
    processes, streams, events = [], [], []
    transcript = None
    started = time.monotonic()
    report = {'created': datetime.now().astimezone().isoformat(),
              'catalog': str(catalog.filename), 'catalog_revision': catalog.revision,
              'audio_file': str(audio_file), 'audio_source': options.audio_source,
              'model': options.model, 'language': options.language,
              'expected_route': expected, 'success': False}

    def event(kind, value):
        events.append({'time_sec': round(time.monotonic()-started, 3),
                       'kind': kind, 'value': value})
        print(f'{kind}: {value}', flush=True)

    def recognized(message):
        nonlocal transcript
        transcript = message.data
        event('transcript', transcript)

    qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/voice_navigation/status',
                             lambda m: event('navigation_status', m.data), qos)
    node.create_subscription(String, '/voice_command', recognized, 10)

    def wait(predicate, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def launch(name, command):
        stream = (logs_directory/f'{name}.log').open('w')
        streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    try:
        launch('gazebo', ['ros2', 'launch', 'robot_sim', 'sim.launch.py',
                          'headless:=true', 'enable_camera:=false'])
        launch('localization', ['ros2', 'launch', 'robot_navigation',
                                'localization.launch.py', 'rviz:=false',
                                'set_initial_pose:=true', f'map:={catalog.grid.filename}'])
        launch('nav2', ['ros2', 'launch', 'robot_navigation', 'navigation.launch.py'])
        launch('voice_navigation', ['ros2', 'launch', 'robot_node',
                                   'voice_navigation.launch.py', f'regions_file:={catalog.filename}'])
        if not wait(lambda: any(e['value'].startswith('READY') for e in events
                                if e['kind'] == 'navigation_status'), 75):
            raise RuntimeError('Map not ready; see logs.')
        lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
        if not wait(lifecycle.service_is_ready, 60):
            raise RuntimeError('Nav2 lifecycle service not ready.')
        active = False
        deadline = time.monotonic()+75
        while time.monotonic() < deadline:
            future = lifecycle.call_async(GetState.Request())
            if wait(future.done, 5) and future.result().current_state.id == 3:
                active = True
                break
        if not active:
            raise RuntimeError('Nav2 is not active.')
        speech = launch('speech_recognition', [str(WORKSPACE/'.venv-voice/bin/python'),
            '-m', 'robot_node.speech_recognition', '--ros-args',
            '-p', f'audio_file:={audio_file}', '-p', f'model:={options.model}',
            '-p', f'language:={options.language}',
            '-p', f'regions_file:={catalog.filename}'])
        terminal = ('SUCCEEDED', 'FAILED', 'REJECTED', 'INVALID_COMMAND',
                    'NOT_READY', 'STOPPING', 'ACTION_FAULT', 'CONFIG_ERROR')
        def completed():
            if speech.poll() not in (None, 0):
                raise RuntimeError('Speech process failed; see logs/speech_recognition.log.')
            return transcript is not None and any(
                e['value'].startswith(terminal) for e in events if e['kind'] == 'navigation_status')
        if not wait(completed, 300):
            raise RuntimeError('Voice/navigation pipeline timed out.')
        parsed = catalog.parse(transcript)
        statuses = [e['value'] for e in events if e['kind'] == 'navigation_status']
        visited = [s.rsplit('→', 1)[-1].strip() for s in statuses if s.startswith('NAVIGATING')]
        result = next(s for s in statuses if s.startswith(terminal))
        report.update(transcript=transcript, parsed_route=parsed, visited=visited, result=result)
        if parsed != expected or visited != expected or not result.startswith('SUCCEEDED'):
            raise RuntimeError(f'Wrong route/result: parsed={parsed}, visited={visited}, result={result}')
        if RegionCatalog(catalog.filename).revision != catalog.revision:
            raise RuntimeError('Catalog changed during this check.')
        report['success'] = True
        event('check_passed', expected)
        return 0
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        report['traceback'] = traceback.format_exc()
        event('check_failed', report['error'])
        return 1
    finally:
        report['events'] = events
        report['duration_sec'] = round(time.monotonic()-started, 2)
        (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
