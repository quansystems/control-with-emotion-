# SPDX-License-Identifier: Apache-2.0
"""ROS RGB image -> cv_bridge -> YOLO -> annotated image and terminal log."""

import math
import os
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

from robot_node.runtime import stop_on_signals


class ObjectDetector(Node):
    WINDOW = 'TurtleBot4 - YOLO detections (Q to quit)'

    def __init__(self):
        super().__init__('object_detector')
        default_model = str(Path.home() / '.cache/turtlebot4/yolo/yolov8n.pt')
        defaults = {
            'image_topic': '/camera/image',
            'output_topic': '/detections/image',
            'model_file': default_model,
            'device': 'cpu',
            'confidence': 0.35,
            'image_size': 640,
            'max_fps': 3.0,
            'log_interval_sec': 1.0,
            'show_image': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.confidence = float(self.get_parameter('confidence').value)
        self.image_size = self.get_parameter('image_size').value
        fps = float(self.get_parameter('max_fps').value)
        self.log_interval = float(self.get_parameter('log_interval_sec').value)
        if not math.isfinite(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError('confidence must be between 0 and 1.')
        for name, value in [('max_fps', fps), ('log_interval_sec', self.log_interval)]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite.')
        if self.image_size <= 0:
            raise ValueError('image_size must be positive.')
        source = self.get_parameter('image_topic').value
        output = self.get_parameter('output_topic').value
        if source == output:
            raise ValueError('image_topic and output_topic must differ.')
        self.device = self.get_parameter('device').value
        self.show_image = self.get_parameter('show_image').value
        if self.show_image and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            self.get_logger().warning('No display available; use /detections/image in RViz.')
            self.show_image = False
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError('YOLO is missing. Run with .venv-yolo/bin/python; '
                               'see README section 6 for installation.') from exc
        model_file = Path(self.get_parameter('model_file').value).expanduser()
        # Ultralytics downloads known pretrained weights on their first use.
        model_file.parent.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Loading YOLO: {model_file} (device={self.device})')
        self.model = YOLO(str(model_file), task='detect')
        self.bridge = CvBridge()
        self.latest_image = None
        self.stop_requested = False
        self.last_log = float('-inf')
        self.last_image_time = time.monotonic()
        self.publisher = self.create_publisher(Image, output, 1)
        # Queue only one image; do not build a backlog while inference runs.
        qos = qos_profile_sensor_data
        camera_qos = QoSProfile(depth=1, reliability=qos.reliability,
                                durability=qos.durability)
        self.subscription = self.create_subscription(Image, source, self.receive_image, camera_qos)
        self.timer = self.create_timer(1.0 / fps, self.process_image)
        self.get_logger().info(f'Listening on {source}; annotated output: {output}; max {fps:g} FPS.')

    def receive_image(self, message):
        self.latest_image = message
        self.last_image_time = time.monotonic()

    def process_image(self):
        message = self.latest_image
        if message is None:
            if time.monotonic() - self.last_image_time > 5.0:
                self.get_logger().warning('No new camera image. Check camera topic and simulation.',
                                          throttle_duration_sec=5.0)
            return
        self.latest_image = None
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            result = self.model.predict(source=image, conf=self.confidence,
                                        imgsz=self.image_size, device=self.device,
                                        verbose=False)[0]
            annotated = result.plot(labels=True, conf=True, boxes=True)
            output = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            output.header = message.header
            self.publisher.publish(output)
            if time.monotonic() - self.last_log >= self.log_interval:
                self.log_detections(result)
                self.last_log = time.monotonic()
            if self.show_image:
                cv2.imshow(self.WINDOW, annotated)
                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                    self.stop_requested = True
        except Exception as exc:
            self.get_logger().error(f'Image processing failed: {exc}', throttle_duration_sec=5.0)

    def log_detections(self, result):
        if result.boxes is None or len(result.boxes) == 0:
            self.get_logger().info('No objects detected above confidence threshold.')
            return
        for box in result.boxes:
            class_id = int(box.cls.item())
            confidence = float(box.conf.item())
            coordinates = [round(float(v), 1) for v in box.xyxy[0].tolist()]
            self.get_logger().info(
                f'{result.names[class_id]} confidence={confidence:.2f} bbox={coordinates}')

    def destroy_node(self):
        if getattr(self, 'show_image', False):
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    exit_code = 0
    try:
        with stop_on_signals():
            node = ObjectDetector()
            while rclpy.ok() and not node.stop_requested:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger('object_detector').error(str(exc))
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
