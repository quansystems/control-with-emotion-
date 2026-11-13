# SPDX-License-Identifier: Apache-2.0
"""ROS transport tests use private command topics and never drive a robot."""

import math
import time
import uuid

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from robot_node.robot_behavior_node import RobotBehaviorNode


@pytest.fixture
def harness():
    prefix = '/behavior_test_' + uuid.uuid4().hex
    rclpy.init(args=['--ros-args', '-p', f'cmd_vel_topic:={prefix}/cmd',
                     '-p', f'gesture_topic:={prefix}/gesture',
                     '-p', f'scan_topic:={prefix}/scan', '-p', f'odom_topic:={prefix}/odom',
                     '-p', 'use_sim_time:=true'])
    behavior = RobotBehaviorNode()
    driver = Node('behavior_test_driver', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(behavior)
    executor.add_node(driver)
    gestures = driver.create_publisher(String, prefix + '/gesture', 1)
    odometry = driver.create_publisher(Odometry, prefix + '/odom', 1)
    scans = driver.create_publisher(LaserScan, prefix + '/scan', 1)
    commands = []
    subscription = driver.create_subscription(
        Twist, prefix + '/cmd', lambda msg: commands.append((msg.linear.x, msg.angular.z)), 20)
    fixed_stamp = time.monotonic_ns()

    def pump(duration, label=None, freeze_stamps=False):
        end = time.monotonic() + duration
        next_send = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_send:
                stamp = (behavior.sensor_stamps.get('odometry', fixed_stamp)
                         if freeze_stamps else time.monotonic_ns())
                odom = Odometry()
                odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                odom.pose.pose.orientation.w = 1.0
                odometry.publish(odom)
                scan = LaserScan()
                scan.header.stamp = odom.header.stamp
                scan.angle_increment = 2 * math.pi / 359
                scan.range_min, scan.range_max = 0.164, 12.0
                scan.ranges = [5.0] * 360
                scans.publish(scan)
                if label is not None:
                    gestures.publish(String(data=label))
                next_send = now + 0.04
            executor.spin_once(timeout_sec=0.005)

    try:
        pump(0.2)  # DDS discovery; no /clock is published in these tests.
        yield behavior, commands, pump, gestures
    finally:
        executor.shutdown()
        driver.destroy_subscription(subscription)
        driver.destroy_node()
        behavior.destroy_node()
        rclpy.shutdown()


def test_ros_queue_levels_unknown_input_and_topic_disconnect_without_sim_clock(harness):
    behavior, commands, pump, gestures = harness
    pump(0.4, 'Pointing_Up')
    assert behavior.controller.state == 'SELECTED'
    assert behavior.controller.pending_command == 'forward'
    assert all(command == (0.0, 0.0) for command in commands)
    pump(0.4, 'Victory')
    pump(0.4, 'Waving')
    assert behavior.controller.pending_command == 'circle'
    assert behavior.controller.active_behavior is None
    pump(0.4, 'thumb_up')
    assert behavior.controller.active_behavior == 'circle'
    assert not behavior.controller.command_queue
    assert any(linear > 0 and angular > 0 for linear, angular in commands)
    pump(0.4, 'Closed_Fist')
    pump(1.0, 'None')
    assert behavior.controller.active_behavior == 'circle'
    assert behavior.controller.pending_command == 'cancel'
    pump(0.4, 'thumb_up')
    assert behavior.controller.state == 'IDLE'
    assert not behavior.controller.command_queue
    pump(0.4, 'Waving')
    assert behavior.controller.active_behavior == 'wave'
    gestures.publish(String(data='Open_Palm'))  # One message must stop immediately.
    pump(0.08)
    assert behavior.controller.state == 'STOPPED'
    assert commands[-1] == (0.0, 0.0)
    pump(0.4, 'Pointing_Up')
    pump(0.4, 'thumb_up')
    assert behavior.controller.state == 'RUNNING'
    pump(1.0, 'None')  # Live recognizer, no recognized gesture: keep moving.
    assert behavior.controller.state == 'RUNNING'
    pump(0.4, 'Pointing_Down')
    assert behavior.controller.state == 'RUNNING'
    assert behavior.controller.pending_command == 'backward'
    pump(0.4, 'thumb_up')
    assert behavior.controller.active_behavior == 'backward'
    assert commands[-1][0] < 0
    gestures.publish(String(data='Open_Palm'))
    pump(0.08)
    assert behavior.controller.state == 'STOPPED'
    assert commands[-1] == (0.0, 0.0)
    pump(0.4, 'Pointing_Up')
    pump(0.4, 'thumb_up')
    pump(1.0)  # No topic messages at all: deadman still stops the robot.
    assert behavior.controller.state == 'IDLE'
    assert behavior.controller.reason == 'gesture topic disconnected'
    assert commands[-1] == (0.0, 0.0)


def test_ros_level_1_interrupts_wave_before_confirm_and_wave_cannot_interrupt_level_1(harness):
    behavior, commands, pump, gestures = harness
    pump(0.4, 'Waving')
    assert behavior.controller.active_behavior == 'wave'
    assert any(angular > 0 for linear, angular in commands)
    pump(0.4, 'Pointing_Down')
    assert behavior.controller.state == 'SELECTED'
    assert behavior.controller.pending_command == 'backward'
    assert behavior.controller.active_behavior is None
    assert commands[-1] == (0.0, 0.0)
    pump(0.4, 'Waving')
    assert behavior.controller.active_behavior is None
    pump(0.4, 'thumb_up')
    assert behavior.controller.active_behavior == 'backward'
    assert not behavior.controller.command_queue
    started = behavior.controller.started_at
    pump(0.4, 'Waving')
    assert behavior.controller.active_behavior == 'backward'
    assert behavior.controller.started_at == started
    assert commands[-1][0] < 0
    gestures.publish(String(data='Open_Palm'))
    pump(0.08)
    assert behavior.controller.state == 'STOPPED'
    assert commands[-1] == (0.0, 0.0)


def test_replayed_sensor_stamps_do_not_refresh_watchdog(harness):
    behavior, commands, pump, _ = harness
    pump(0.4, 'Victory')
    pump(0.4, 'thumb_up')
    assert behavior.controller.state == 'RUNNING'
    # Use the last accepted stamp repeatedly; a frozen sensor stream is stale.
    pump(0.8, 'thumb_up', freeze_stamps=True)
    assert behavior.controller.state == 'IDLE'
    assert 'stale' in behavior.controller.reason
    assert commands[-1] == (0.0, 0.0)
