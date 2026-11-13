"""Push-to-talk English Whisper transcription, published as ROS text commands."""

import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_node.semantic_map import RegionCatalog, default_regions_file


def main(args=None):
    rclpy.init(args=args)
    node = Node('speech_recognition')
    try:
        node.declare_parameter('model', 'small')
        node.declare_parameter('language', 'en')
        node.declare_parameter('record_seconds', 6.0)
        node.declare_parameter('input_device', -1)
        node.declare_parameter('audio_file', '')
        node.declare_parameter('silence_rms', 0.005)
        node.declare_parameter('initial_prompt', '')
        node.declare_parameter('regions_file', default_regions_file())
        seconds = float(node.get_parameter('record_seconds').value)
        threshold = float(node.get_parameter('silence_rms').value)
        if not math.isfinite(seconds) or not 0 < seconds <= 30:
            raise ValueError('record_seconds must be in (0, 30].')
        if not math.isfinite(threshold) or threshold < 0:
            raise ValueError('silence_rms must be finite and nonnegative.')
        # Optional heavy dependencies are not required by the map/navigation nodes.
        import whisper

        publisher = node.create_publisher(String, '/voice_command', 10)
        status = node.create_publisher(String, '/speech_recognition/status', 10)
        model_name = node.get_parameter('model').value
        language = node.get_parameter('language').value
        if language not in ('en', 'vi'):
            raise ValueError('language must be en or vi.')
        if language != 'en' and str(model_name).endswith('.en'):
            raise ValueError('Use a multilingual Whisper model for Vietnamese.')
        catalog = RegionCatalog(node.get_parameter('regions_file').value)
        node.get_logger().info('Các vùng hiện có: ' + ', '.join(
            f'{r["id"]} ({r["name"]})' for r in catalog.regions))
        node.get_logger().info(f'Loading Whisper {model_name} on CPU...')
        model = whisper.load_model(model_name, device='cpu')
        filename = node.get_parameter('audio_file').value
        while rclpy.ok():
            if filename:
                if not Path(filename).is_file():
                    raise ValueError(f'Audio file not found: {filename}')
                audio = whisper.load_audio(filename)
            else:
                import sounddevice as sd

                input('Nhấn Enter để thu âm, Ctrl+C để thoát: ')
                device = int(node.get_parameter('input_device').value)
                status.publish(String(data='RECORDING'))
                print(f'Đang thu {seconds:g} giây...')
                audio = sd.rec(int(seconds*16000), samplerate=16000, channels=1,
                               dtype='float32', device=None if device < 0 else device)
                try:
                    sd.wait()
                finally:
                    sd.stop()
                audio = audio.reshape(-1)
            if not audio.size or not np.isfinite(audio).all():
                raise ValueError('Audio contains no samples or invalid values.')
            if float(np.sqrt(np.mean(audio**2))) < threshold:
                node.get_logger().info('SILENCE: không gửi câu lệnh.')
            else:
                status.publish(String(data='TRANSCRIBING'))
                # Re-read saved names/aliases before each utterance, so edits do not
                # require changing Python code or a manually maintained prompt.
                catalog = RegionCatalog(node.get_parameter('regions_file').value)
                prompt = node.get_parameter('initial_prompt').value or catalog.speech_prompt(language)
                result = model.transcribe(audio, language=language, task='transcribe',
                                          fp16=False, temperature=0,
                                          beam_size=5,
                                          condition_on_previous_text=False,
                                          initial_prompt=prompt)
                text = ' '.join(segment['text'].strip()
                                for segment in result.get('segments', [])
                                if segment.get('no_speech_prob', 0) < 0.6).strip()
                if text:
                    # Give discovery a bounded chance before publishing a WAV once.
                    import time
                    deadline = time.monotonic() + 2.0
                    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.1)
                    node.get_logger().info(f'RECOGNIZED: {text}')
                    publisher.publish(String(data=text))
                    rclpy.spin_once(node, timeout_sec=0.2)
                else:
                    node.get_logger().info('NO_SPEECH: không gửi câu lệnh.')
            status.publish(String(data='IDLE'))
            if filename:
                break
    except (KeyboardInterrupt, EOFError):
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
