# SPDX-License-Identifier: Apache-2.0
"""USB webcam -> MediaPipe Gesture Recognizer -> /detected_gesture."""

import math
import os
from pathlib import Path
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from robot_node.gesture_logic import GESTURE_LABELS, GestureInterpreter
from robot_node.runtime import stop_on_signals


class GestureRecognitionNode(Node):
    WINDOW = 'Gesture recognition (Q / Esc to quit)'

    def __init__(self):
        super().__init__('gesture_recognition_node')
        self.capture = None
        self.recognizer = None
        self.show_image = False
        self.stop_requested = False
        self.last_timestamp_ms = -1
        self.read_failures = 0
        self.last_gesture = None
        try:
            self._configure()
        except BaseException:
            self.destroy_node()
            raise

    def _configure(self):
        defaults = {
            'camera_index': 0,
            'model_path': str(Path.home() / '.cache/turtlebot4/gesture/gesture_recognizer.task'),
            'confidence_threshold': 0.6,
            'max_fps': 15.0,
            'frame_width': 640,
            'frame_height': 480,
            'show_image': True,
            'mirror_image': True,
            'wave_window_sec': 2.0,
            'wave_min_motion': 0.035,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        params = {name: self.get_parameter(name).value for name in defaults}
        for name in ('max_fps', 'wave_window_sec', 'wave_min_motion'):
            if not math.isfinite(params[name]) or params[name] <= 0:
                raise ValueError(f'{name} must be positive and finite.')
        confidence = params['confidence_threshold']
        if not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError('confidence_threshold must be between 0 and 1.')
        if params['camera_index'] < 0 or min(params['frame_width'], params['frame_height']) <= 0:
            raise ValueError('camera_index must be >= 0; frame dimensions must be > 0.')
        if params['wave_min_motion'] > 1 or params['wave_window_sec'] < 0.35:
            raise ValueError('wave_min_motion must be <= 1; wave_window_sec must be >= 0.35.')
        self.show_image = params['show_image']
        self.mirror_image = params['mirror_image']
        if self.show_image and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            self.get_logger().warning('No desktop display; disabling webcam window.')
            self.show_image = False
        try:
            import mediapipe as mp
            from mediapipe.tasks import python
            from mediapipe.tasks.python import vision
        except ImportError as exc:
            raise RuntimeError('MediaPipe missing. Install requirements-gesture.txt and run '
                               'with .venv-gesture/bin/python; see README section 7.') from exc
        self.mp = mp
        model_path = Path(params['model_path']).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f'Model not found: {model_path}. '
                                    'Download gesture_recognizer.task as in README section 7.')
        options = vision.GestureRecognizerOptions(
            base_options=python.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        # Keep the original top class, including rejected classes. An allowlist
        # could promote a low-ranked allowed class for an unsupported gesture.
        self.recognizer = vision.GestureRecognizer.create_from_options(options)
        self.interpreter = GestureInterpreter(confidence, params['wave_window_sec'],
                                              params['wave_min_motion'])
        self.capture = cv2.VideoCapture(params['camera_index'], cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(f"Cannot open /dev/video{params['camera_index']}. "
                               'Check camera_index, permissions and camera use by other apps.')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, params['frame_width'])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, params['frame_height'])
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.publisher = self.create_publisher(String, '/detected_gesture', 10)
        if self.show_image:
            cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)
        self.timer = self.create_timer(1.0 / params['max_fps'], self.process_frame)
        self.get_logger().info(f"Webcam /dev/video{params['camera_index']} -> /detected_gesture")

    def process_frame(self):
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.read_failures += 1
            self.interpreter.reset_wave()
            self.publisher.publish(String(data='None'))
            self.get_logger().error('Cannot read webcam frame.', throttle_duration_sec=3.0)
            if self.read_failures >= 10:
                raise RuntimeError('Webcam failed to deliver 10 consecutive frames.')
            return
        self.read_failures = 0
        if self.mirror_image:
            frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        timestamp_ms = max(self.last_timestamp_ms + 1, time.monotonic_ns() // 1_000_000)
        self.last_timestamp_ms = timestamp_ms
        result = self.recognizer.recognize_for_video(
            self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), timestamp_ms)
        landmarks = result.hand_landmarks[0] if result.hand_landmarks else []
        categories = result.gestures[0] if result.gestures else []
        top = max(categories, key=lambda item: item.score) if categories else None
        hand = result.handedness[0][0].category_name if result.handedness else None
        height, width = frame.shape[:2]
        gesture, score, source = self.interpreter.classify(
            top.category_name if top else 'None', top.score if top else 0.0,
            landmarks, timestamp_ms / 1000.0, hand, width / height)
        # MediaPipe uses Thumb_Up; expose the requested ROS label thumb_up.
        topic_label = 'thumb_up' if gesture == 'Thumb_Up' else gesture
        self.publisher.publish(String(data=topic_label))
        if gesture != self.last_gesture:
            self.get_logger().info(f'{topic_label}: {score:.1%} ({source})')
            self.last_gesture = gesture
        if self.show_image:
            for point in landmarks:
                cv2.circle(frame, (int(point.x * width), int(point.y * height)),
                           3, (0, 220, 0), -1)
            # Hershey fonts support ASCII; Vietnamese labels are transliterated.
            label = GESTURE_LABELS.get(gesture, 'Khong nhan dien')
            cv2.rectangle(frame, (0, 0), (width, 80), (30, 30, 30), -1)
            cv2.putText(frame, f'{label}: {score:.1%}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            cv2.putText(frame, source, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (220, 220, 220), 1)
            cv2.imshow(self.WINDOW, frame)
            if (cv2.waitKey(1) & 0xFF in (ord('q'), 27)
                    or cv2.getWindowProperty(self.WINDOW, cv2.WND_PROP_VISIBLE) < 1):
                self.stop_requested = True

    def destroy_node(self):
        try:
            if self.capture is not None:
                self.capture.release()
            if self.recognizer is not None:
                self.recognizer.close()
        finally:
            if self.show_image:
                cv2.destroyAllWindows()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    exit_code = 0
    try:
        with stop_on_signals():
            node = GestureRecognitionNode()
            while rclpy.ok() and not node.stop_requested:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger('gesture_recognition_node').error(str(exc))
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
