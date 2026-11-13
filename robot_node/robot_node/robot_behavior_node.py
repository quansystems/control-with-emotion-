# SPDX-License-Identifier: Apache-2.0
"""Gesture levels and a latest-command queue with odometry/lidar interlocks."""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from robot_node.behavior_logic import BehaviorController, scan_clearance
from robot_node.runtime import publish_stop, stop_on_signals
from robot_node.velocity_filter import positive_finite


class RobotBehaviorNode(Node):
    def __init__(self):
        super().__init__('robot_behavior_node')
        defaults = dict(BehaviorController.DEFAULTS, publish_rate=20.0,
                        gesture_topic='/detected_gesture', cmd_vel_topic='/cmd_vel',
                        scan_topic='/scan', odom_topic='/odom')
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        params = {name: self.get_parameter(name).value for name in defaults}
        self.controller = BehaviorController(**{name: params[name] for name in BehaviorController.DEFAULTS})
        rate = positive_finite('publish_rate', params['publish_rate'])
        self.publisher = self.create_publisher(Twist, params['cmd_vel_topic'], 1)
        self.status_publisher = self.create_publisher(String, '/robot_behavior/status', 10)
        # Depth 1 prevents executing old gestures/sensor samples from a backlog.
        self.gesture_subscription = self.create_subscription(
            String, params['gesture_topic'], self.receive_gesture, 1)
        qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability,
                         durability=qos_profile_sensor_data.durability)
        self.scan_subscription = self.create_subscription(
            LaserScan, params['scan_topic'], self.receive_scan, qos)
        self.odom_subscription = self.create_subscription(
            Odometry, params['odom_topic'], self.receive_odometry, qos)
        self.sensor_stamps = {}
        self.last_status = None
        # Safety watchdog keeps firing when /clock freezes or simulation pauses.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / rate, self.tick, clock=self.wall_clock)
        self.publish_velocity((0.0, 0.0))
        self.publish_status()

    def publish_velocity(self, command):
        message = Twist()
        message.linear.x, message.angular.z = command
        self.publisher.publish(message)

    def publish_status(self):
        c = self.controller
        status = (f'{c.state}; queued={c.pending_command or "none"}; '
                  f'active={c.active_behavior or "none"}; '
                  f'level={c.active_level if c.active_level is not None else "none"}; {c.reason}')
        if status != self.last_status:
            self.status_publisher.publish(String(data=status))
            self.get_logger().info(status)
            self.last_status = status

    def receive_gesture(self, message):
        previous_motion = (self.controller.active_behavior, self.controller.started_at)
        self.controller.receive_gesture(message.data, time.monotonic())
        current_motion = (self.controller.active_behavior, self.controller.started_at)
        if self.controller.state != 'RUNNING' or previous_motion != current_motion:
            # Stop the previous command immediately before a confirmed/preempting
            # behavior starts publishing its velocity on the next timer tick.
            self.publish_velocity((0.0, 0.0))
        self.publish_status()

    def _new_sensor_sample(self, name, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        previous = self.sensor_stamps.get(name)
        if previous is not None and stamp <= previous:
            if stamp < previous:
                self.controller.abort(f'{name} timestamp moved backwards')
                self.sensor_stamps[name] = stamp
                self.publish_velocity((0.0, 0.0))
                if name == 'odometry':
                    self.controller.last_odom = None
                else:
                    self.controller.last_scan = None
            return False
        self.sensor_stamps[name] = stamp
        return True

    def receive_odometry(self, message):
        if not self._new_sensor_sample('odometry', message):
            return
        position, orientation = message.pose.pose.position, message.pose.pose.orientation
        q = (orientation.x, orientation.y, orientation.z, orientation.w)
        norm = math.sqrt(sum(v * v for v in q))
        if not math.isfinite(norm) or norm < 1e-6:
            self.controller.last_odom = None
            self.controller.abort('invalid odometry quaternion')
        else:
            x, y, z, w = (v / norm for v in q)
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            self.controller.receive_odometry(position.x, position.y, yaw, time.monotonic())
        if self.controller.state != 'RUNNING':
            self.publish_velocity((0.0, 0.0))

    def receive_scan(self, message):
        if not self._new_sensor_sample('scan', message):
            return
        clearance = scan_clearance(message.ranges, message.angle_increment,
                                   message.range_min, message.range_max)
        self.controller.receive_scan(clearance, time.monotonic())
        if self.controller.state != 'RUNNING':
            self.publish_velocity((0.0, 0.0))

    def tick(self):
        self.publish_velocity(self.controller.sample(time.monotonic()))
        self.publish_status()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    exit_code = 0
    with stop_on_signals():
        try:
            node = RobotBehaviorNode()
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        except Exception as exc:
            rclpy.logging.get_logger('robot_behavior_node').error(str(exc))
            exit_code = 1
        finally:
            if node is not None:
                publish_stop(node, node.publisher)
                node.destroy_node()
            rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
