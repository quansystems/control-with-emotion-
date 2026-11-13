"""Exercise real ROS messages on an isolated domain, without camera or motors."""

import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from emotion_control.node import EmotionControlNode


def test_velocity_topic_watchdog_and_odometry(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '137')
    monkeypatch.setattr(EmotionControlNode, 'capture_loop', lambda self: None)
    monkeypatch.setattr(EmotionControlNode, 'inference_loop', lambda self: None)
    rclpy.init(args=['--ros-args', '-p', 'show_image:=false', '-p',
                    'cmd_vel_topic:=/emotion_control_test/cmd_vel'])
    node = observer = None
    try:
        node = EmotionControlNode()
        observer = Node('emotion_test_observer')
        received = []
        subscription = observer.create_subscription(
            Twist, '/emotion_control_test/cmd_vel', received.append, 10)
        deadline = time.monotonic() + 3
        while node.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.02)
        assert node.publisher.get_subscription_count() > 0
        now = time.monotonic()
        for offset in (0.2, 0.1, 0.0):
            node.result = (now-offset, 'happy', 0.95, None)
            node.tick()
        deadline = time.monotonic() + 2
        while not any(m.linear.x > 0 for m in received) and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.02)
        assert any(m.linear.x == 0.1 and m.angular.z == 0.5 for m in received)
        received.clear()
        node.controller.last_detection = time.monotonic() - 3
        node.tick()
        deadline = time.monotonic() + 2
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.02)
        assert received[-1].linear.x == received[-1].angular.z == 0
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        node.on_odometry(odom)
        assert node.controller.yaw == 0
        assert node.controller.odom_time > now
        assert subscription is not None
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if observer is not None:
            observer.destroy_node()
        rclpy.shutdown()
