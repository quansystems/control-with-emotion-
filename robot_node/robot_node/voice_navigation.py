"""Nonblocking semantic navigation: plan a route, then visit every named region."""

import math
import hashlib
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import ComputePathThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path as RosPath
import numpy as np
import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from robot_node.semantic_map import RegionCatalog, default_regions_file


class VoiceNavigation(Node):
    def __init__(self):
        super().__init__('voice_navigation')
        self.declare_parameter('regions_file', default_regions_file())
        self.declare_parameter('goal_timeout_sec', 180.0)
        self.declare_parameter('planning_timeout_sec', 30.0)
        self.regions_file = Path(self.get_parameter('regions_file').value).resolve()
        self.catalog = RegionCatalog(self.regions_file)
        self.goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self.plan_timeout = float(self.get_parameter('planning_timeout_sec').value)
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.goal_timeout, self.plan_timeout)):
            raise ValueError('Timeouts must be positive and finite.')
        self.route = []
        self.index = 0
        self.phase = 'idle'
        self.goal_handle = None
        self.deadline = 0.0
        self.map_ready = False
        self.last_map = None
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planner = ActionClient(self, ComputePathThroughPoses, 'compute_path_through_poses')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/voice_navigation/status', latched)
        self.region_pub = self.create_publisher(MarkerArray, '/semantic_regions', latched)
        self.route_pub = self.create_publisher(MarkerArray, '/voice_navigation/route', latched)
        self.path_pub = self.create_publisher(RosPath, '/voice_navigation/planned_path', latched)
        self.create_subscription(String, '/voice_command', self.command, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        # A wall-clock watchdog must run even when Gazebo /clock is paused.
        self.create_timer(0.1, self.watchdog,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_regions()
        self.status('WAITING_MAP: chờ /map khớp bản đồ phân vùng.')

    def refresh_catalog(self):
        """Adopt saved regions between routes; never change an in-flight goal."""
        try:
            revision = hashlib.sha256(self.regions_file.read_bytes()).hexdigest()
            if revision == self.catalog.revision:
                return True
            catalog = RegionCatalog(self.regions_file)
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            self.status(f'CONFIG_ERROR: không thể đọc vùng đã lưu: {exc}')
            return False
        self.catalog = catalog
        self.route, self.index = [], 0
        self.map_ready = False
        self.publish_regions()
        self.publish_route()
        self.clear_path()
        if self.last_map is not None:
            self.on_map(self.last_map)
        self.status('REGIONS_UPDATED: ' + ', '.join(catalog.by_id))
        return True

    def status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def on_map(self, message):
        self.last_map = message
        grid = self.catalog.grid
        origin = message.info.origin
        yaw = grid.origin[2]
        expected_z, expected_w = math.sin(yaw/2), math.cos(yaw/2)
        orientation = origin.orientation
        # Quaternions q and -q describe the same rotation.
        orientation_ok = (abs(orientation.x) < 1e-5 and abs(orientation.y) < 1e-5 and
                          abs(abs(orientation.z*expected_z +
                                  orientation.w*expected_w) - 1) < 1e-5)
        valid = (message.header.frame_id == 'map' and
                 message.info.width == grid.width and message.info.height == grid.height and
                 abs(message.info.resolution-grid.resolution) < 1e-6 and
                 abs(origin.position.x-grid.origin[0]) < 1e-5 and
                 abs(origin.position.y-grid.origin[1]) < 1e-5 and orientation_ok)
        if valid and len(message.data) == grid.width*grid.height:
            live = np.flipud(np.asarray(message.data).reshape(grid.height, grid.width))
            valid = np.array_equal(live == 0, grid.free)
        else:
            valid = False
        previous = self.map_ready
        self.map_ready = valid
        if not valid:
            if self.phase not in ('idle', 'stopping', 'fault'):
                self.stop('MAP_MISMATCH')
            elif previous:
                self.status('MAP_MISMATCH: hãy phân vùng lại bản đồ đang chạy.')
        elif not previous:
            self.status('READY: nhận câu lệnh tại /voice_command.')

    def pose(self, region_id):
        goal = self.catalog.by_id[region_id]['goal']
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(goal['x'])
        pose.pose.position.y = float(goal['y'])
        pose.pose.orientation.z = math.sin(float(goal['yaw']) / 2)
        pose.pose.orientation.w = math.cos(float(goal['yaw']) / 2)
        return pose

    def command(self, message):
        if self.phase == 'idle' and not self.refresh_catalog():
            return
        try:
            route = self.catalog.parse(message.data)
        except ValueError as exc:
            self.status(f'INVALID_COMMAND: {exc}')
            return
        if not route:
            if self.phase != 'idle':
                self.stop('USER_CANCELED')
            else:
                self.status('IDLE: không có hành trình đang chạy.')
            return
        if self.phase != 'idle':
            self.status('BUSY: chờ hoàn tất hoặc nói Dừng lại trước lệnh mới.')
            return
        if not self.map_ready:
            self.status('NOT_READY: /map chưa khớp bản đồ đã phân vùng.')
            return
        if not self.planner.server_is_ready() or not self.nav.server_is_ready():
            self.status('NOT_READY: Nav2 chưa active hoặc chưa có action server.')
            return
        self.route, self.index = route, 0
        self.publish_route()
        self.clear_path()
        self.status('PLANNING: ' + ' → '.join(route))
        goal = ComputePathThroughPoses.Goal()
        goal.goals = [self.pose(region_id) for region_id in route]
        goal.use_start = False  # Nav2 uses the current map->base_link pose.
        self.send(self.planner, goal, 'planning', self.plan_timeout)

    def send(self, client, goal, phase, timeout):
        self.phase = phase
        self.goal_handle = None
        self.deadline = time.monotonic() + timeout
        try:
            client.send_goal_async(goal).add_done_callback(self.accepted)
        except Exception as exc:
            self.finish(f'SEND_FAILED: {exc}')

    def accepted(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            # An unknown remote state must not allow another route to start.
            self.phase = 'fault'
            self.status(f'ACTION_FAULT: {exc}; kiểm tra Nav2 rồi khởi động lại node.')
            return
        if not handle.accepted:
            self.finish('REJECTED: Nav2 từ chối mục tiêu.')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.completed)
        if self.phase == 'stopping':
            handle.cancel_goal_async()

    def completed(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.phase = 'fault'
            self.status(f'ACTION_FAULT: {exc}; kiểm tra Nav2 rồi khởi động lại node.')
            return
        self.goal_handle = None
        if self.phase == 'stopping':
            self.finish('STOPPED: hành trình đã kết thúc/hủy trên Nav2.')
            return
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self.finish(f'FAILED: Nav2 status={response.status}; dừng chuỗi mục tiêu.')
            return
        if self.phase == 'planning':
            if not response.result.path.poses:
                self.finish('FAILED: planner trả đường rỗng.')
                return
            self.path_pub.publish(response.result.path)
        else:
            self.index += 1
        if self.index >= len(self.route):
            self.finish('SUCCEEDED: đã đến tất cả khu vực theo thứ tự.')
            return
        region_id = self.route[self.index]
        self.status(f'NAVIGATING: {self.index+1}/{len(self.route)} → {region_id}')
        goal = NavigateToPose.Goal()
        goal.pose = self.pose(region_id)
        self.send(self.nav, goal, 'navigating', self.goal_timeout)

    def watchdog(self):
        if self.phase in ('planning', 'navigating') and time.monotonic() >= self.deadline:
            self.stop('TIMEOUT')

    def stop(self, reason):
        if self.phase in ('stopping', 'fault'):
            return
        self.phase = 'stopping'
        self.clear_path()
        self.status(f'STOPPING: {reason}; chờ Nav2 xác nhận kết thúc.')
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        # If acceptance is pending, accepted() cancels the late goal.

    def finish(self, message):
        self.phase = 'idle'
        self.goal_handle = None
        self.status(message)
        if not message.startswith('SUCCEEDED'):
            self.clear_path()

    def clear_path(self):
        path = RosPath()
        path.header.frame_id = 'map'
        self.path_pub.publish(path)

    def marker(self, namespace, index, kind, color):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type = namespace, index, kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def publish_regions(self):
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear]
        palette = [(0.2, 0.8, 0.3, 1.0), (0.2, 0.5, 1.0, 1.0),
                   (1.0, 0.3, 0.3, 1.0), (0.8, 0.4, 1.0, 1.0)]
        for index, region in enumerate(self.catalog.regions):
            color = palette[index % len(palette)]
            for outline in region.get('outlines', []):
                line = self.marker('boundaries', len(markers), Marker.LINE_STRIP, color)
                line.scale.x = 0.04
                line.points = [Point(x=float(x), y=float(y), z=0.03)
                               for x, y in [*outline, outline[0]]]
                markers.append(line)
            label = self.marker('labels', index, Marker.TEXT_VIEW_FACING, color)
            label.pose = self.pose(region['id']).pose
            label.pose.position.z = 0.35
            label.scale.z = 0.22
            # RViz font support varies; IDs are always readable.
            label.text = (region['id'] if region['id'] == region['name'] else
                          f'{region["id"]}: {region["name"]}')
            markers.append(label)
            target = self.marker('targets', index, Marker.SPHERE, color)
            target.pose = self.pose(region['id']).pose
            target.scale.x = target.scale.y = target.scale.z = 0.15
            markers.append(target)
        self.region_pub.publish(MarkerArray(markers=markers))

    def publish_route(self):
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear]
        for index, region_id in enumerate(self.route):
            label = self.marker('visit_order', index, Marker.TEXT_VIEW_FACING,
                                (1.0, 0.8, 0.0, 1.0))
            label.pose = self.pose(region_id).pose
            label.pose.position.z = 0.65 + 0.08*index
            label.scale.z = 0.2
            label.text = f'{index+1}: {region_id}'
            markers.append(label)
        self.route_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = VoiceNavigation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.phase not in ('idle', 'fault') and rclpy.ok():
                node.stop('SHUTDOWN')
                deadline = time.monotonic() + 3
                while node.phase == 'stopping' and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
