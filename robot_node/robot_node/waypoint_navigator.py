# SPDX-License-Identifier: Apache-2.0
"""Navigate through map-frame goals, advancing only after successful arrival."""

import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.signals import SignalHandlerOptions
import yaml

from robot_node.runtime import stop_on_signals
from robot_node.semantic_map import RegionCatalog, default_regions_file


def load_goals(filename):
    with Path(filename).open() as stream:
        config = yaml.safe_load(stream)
    goals = config.get('goals') if isinstance(config, dict) else None
    if not isinstance(goals, list) or not goals:
        raise ValueError('The goals file must contain a non-empty goals list.')
    result = []
    for index, goal in enumerate(goals, start=1):
        if not isinstance(goal, dict) or not all(k in goal for k in ('x', 'y', 'yaw')):
            raise ValueError(f'Goal {index} requires x, y and yaw.')
        values = tuple(float(goal[k]) for k in ('x', 'y', 'yaw'))
        if not all(math.isfinite(v) for v in values):
            raise ValueError(f'Goal {index} contains a non-finite coordinate.')
        result.append(values)
    return result


def load_region_goals(filename):
    catalog = RegionCatalog(filename)
    return [tuple(float(catalog.by_id[r]['goal'][key]) for key in ('x', 'y', 'yaw'))
            for r in sorted(catalog.by_id)]


def make_pose(navigator, coordinates):
    x, y, yaw = coordinates
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    # Navigation is planar; Publish Point's small surface z offset is ignored.
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def navigate_goals(navigator, goals, goal_timeout):
    """Return False immediately on rejection, failed result or wall-clock timeout."""
    for index, coordinates in enumerate(goals, start=1):
        navigator.info(f'Goal {index}/{len(goals)}: {coordinates}')
        if not navigator.goToPose(make_pose(navigator, coordinates)):
            navigator.error(f'Goal {index} rejected. Route stopped.')
            return False
        started = time.monotonic()
        while not navigator.isTaskComplete():
            if time.monotonic() - started >= goal_timeout:
                navigator.cancelTask()
                navigator.error(f'Goal {index} timed out; cancellation requested. Route stopped.')
                return False
            time.sleep(0.05)
        result = navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            navigator.error(f'Goal {index}: {result.name}. Route stopped.')
            return False
        navigator.info(f'Goal {index} succeeded.')
    navigator.info(f'Completed all {len(goals)} goals in order.')
    return True


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    navigator = None
    exit_code = 1
    try:
        with stop_on_signals():
            navigator = BasicNavigator(node_name='waypoint_navigator')
            navigator.declare_parameter('goals_file', '')
            navigator.declare_parameter('regions_file', default_regions_file())
            navigator.declare_parameter('goal_timeout_sec', 180.0)
            goals_file = navigator.get_parameter('goals_file').value
            goals = (load_goals(goals_file) if goals_file else
                     load_region_goals(navigator.get_parameter('regions_file').value))
            timeout = float(navigator.get_parameter('goal_timeout_sec').value)
            if not math.isfinite(timeout) or timeout <= 0.0:
                raise ValueError('goal_timeout_sec must be positive and finite.')
            navigator.info(f'Loaded {len(goals)} goals. Waiting for AMCL pose; '
                           'set 2D Pose Estimate in RViz if needed.')
            # Humble's waitUntilNav2Active publishes its default initial pose if
            # no AMCL pose has arrived. Wait first to preserve RViz localization.
            while rclpy.ok() and not navigator.initial_pose_received:
                rclpy.spin_once(navigator, timeout_sec=0.5)
            if rclpy.ok():
                navigator.info('AMCL pose received. Waiting for Nav2 to become active...')
                navigator.waitUntilNav2Active()
                exit_code = 0 if navigate_goals(navigator, goals, timeout) else 1
    except KeyboardInterrupt:
        if navigator is not None:
            navigator.warn('Interrupted. Canceling current navigation goal.')
        exit_code = 130
    except Exception as exc:
        if navigator is not None:
            navigator.error(str(exc))
        else:
            print(f'waypoint_navigator: {exc}')
    finally:
        if navigator is not None:
            try:
                if (rclpy.ok() and navigator.goal_handle is not None and
                        navigator.result_future is not None and
                        not navigator.result_future.done()):
                    navigator.cancelTask()
            finally:
                navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
