"""Webcam and DeepFace workers feeding a nonblocking ROS velocity timer."""

import math
import os
import signal
import threading
import time

import cv2
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from emotion_control.detector import Detector
from emotion_control.logic import MotionController, Settings


class EmotionControlNode(Node):
    WINDOW = 'TurtleBot4 emotions - Q / Esc: stop and quit'

    def __init__(self):
        super().__init__('emotion_control')
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.frame = None
        self.result = None
        self.status = 'Loading DeepFace model...'
        self.workers = []
        self.publisher = None
        self.show_image = False
        self.quit_requested = False
        try:
            self._configure()
        except BaseException:
            self.close()
            self.destroy_node()
            raise

    def _configure(self):
        defaults = dict(vars(Settings()))
        defaults.update(camera_index=0, frame_width=640, frame_height=480,
                        show_image=True, mirror_image=True,
                        cmd_vel_topic='/cmd_vel', odom_topic='/odom')
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.controller = MotionController(Settings(**{k: p[k] for k in vars(Settings())}))
        if p['camera_index'] < 0 or min(p['frame_width'], p['frame_height']) <= 0:
            raise ValueError('Invalid camera index or frame dimensions')
        self.params = p
        self.show_image = p['show_image'] and bool(
            os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        if p['show_image'] and not self.show_image:
            self.get_logger().warning('No display available; running without camera window.')
        self.publisher = self.create_publisher(Twist, p['cmd_vel_topic'], 1)
        self.emotion_pub = self.create_publisher(String, '/detected_emotion', 1)
        self.subscription = self.create_subscription(
            Odometry, p['odom_topic'], self.on_odometry, qos_profile_sensor_data)
        # Wall time keeps the stop watchdog alive when simulation time pauses.
        self.timer = self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        for target in (self.capture_loop, self.inference_loop):
            worker = threading.Thread(target=target, daemon=True)
            self.workers.append(worker)
            worker.start()
        self.get_logger().info(f"Camera {p['camera_index']} -> {p['cmd_vel_topic']}; "
                               'waiting for stable expression. Surprise requires odometry.')

    def capture_loop(self):
        capture = None
        try:
            capture = cv2.VideoCapture(self.params['camera_index'], cv2.CAP_V4L2)
            if not capture.isOpened():
                raise RuntimeError('Cannot open camera; check camera_index and permissions')
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.params['frame_width'])
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.params['frame_height'])
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            while not self.stop_event.is_set():
                ok, frame = capture.read()
                captured_at = time.monotonic()
                if not ok or frame is None:
                    raise RuntimeError('Camera disconnected or failed to read a frame')
                if self.params['mirror_image']:
                    frame = cv2.flip(frame, 1)
                with self.lock:
                    self.frame = (captured_at, frame)
                self.stop_event.wait(0.01)
        except Exception as exc:
            with self.lock:
                self.status = str(exc)
                self.frame = None
                self.result = (time.monotonic(), None, 0.0, None)
            self.get_logger().error(str(exc))
        finally:
            if capture is not None:
                capture.release()

    def inference_loop(self):
        try:
            detector = Detector()
            processed = -1.0
            while not self.stop_event.is_set():
                with self.lock:
                    current = self.frame
                if current is None or current[0] == processed:
                    self.stop_event.wait(0.02)
                    continue
                stamp, frame = current
                processed = stamp
                label, confidence, region = detector.analyze(frame)
                with self.lock:
                    # Never let a finished inference override a camera failure.
                    if self.frame is not None:
                        self.result = (stamp, label, confidence, region)
                        self.status = f'{label or "No face"} ({confidence:.0%})'
        except Exception as exc:
            with self.lock:
                self.result = (time.monotonic(), None, 0.0, None)
                self.status = f'DeepFace error: {exc}'
            self.get_logger().error(f'{self.status}. See emotion_control/README.md.')

    def on_odometry(self, msg):
        q = msg.pose.pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-6:
            return
        x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        self.controller.odometry(yaw, time.monotonic())

    def tick(self):
        now = time.monotonic()
        with self.lock:
            result, current, status = self.result, self.frame, self.status
        if result is not None:
            stamp, label, confidence, region = result
            if stamp > self.controller.last_sample:
                self.controller.observe(label, confidence, stamp, now)
                self.emotion_pub.publish(String(data=label or 'unknown'))
        linear, angular = self.controller.command(now)
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.publisher.publish(msg)
        if self.show_image and current is not None:
            frame = current[1].copy()
            if result is not None and now - result[0] <= self.params['detection_timeout']:
                region = result[3]
                if region:
                    x, y, w, h = (int(region[k]) for k in ('x', 'y', 'w', 'h'))
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            else:
                status = 'Waiting for fresh recognition - STOP'
            cv2.putText(frame, status[:90], (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 255), 2)
            action = self.controller.active or 'STOP'
            cv2.putText(frame, f'{action}: v={linear:.2f} w={angular:.2f}',
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.imshow(self.WINDOW, frame)
            if (cv2.waitKey(1) & 0xff) in (27, ord('q')):
                self.quit_requested = True
            if cv2.getWindowProperty(self.WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                self.quit_requested = True

    def close(self):
        self.stop_event.set()
        if self.publisher is not None and rclpy.ok():
            for _ in range(3):
                self.publisher.publish(Twist())
        for worker in self.workers:
            worker.join(timeout=0.5)
        if self.show_image:
            cv2.destroyAllWindows()


def main(args=None):
    # Keep the ROS context alive until the final zero velocity is published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = EmotionControlNode()
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
