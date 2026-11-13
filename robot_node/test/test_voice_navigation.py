"""Real ROS action transport with fake planner/navigation, without robot motion."""

import time

import numpy as np
import pytest
import rclpy
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from nav2_msgs.action import ComputePathThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from robot_node.voice_navigation import VoiceNavigation


def spin_until(executor, condition, timeout=5):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert condition(), 'ROS condition timed out'


@pytest.fixture
def system(sample_catalog_file):
    path = sample_catalog_file
    rclpy.init(args=['--ros-args', '-p', f'regions_file:={path}'])
    node = VoiceNavigation()
    fake = Node('fake_nav2')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(fake)
    state = {'sent': [], 'planned': [], 'fail_at': None, 'reject': False,
             'gate': None, 'plan_gate': None}

    async def plan(handle):
        state['planned'] = handle.request.goals
        if state['plan_gate'] is not None:
            await state['plan_gate']
        result = ComputePathThroughPoses.Result()
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            result.path.header.frame_id = 'map'
            result.path.poses = handle.request.goals
            handle.succeed()
        return result

    async def navigate(handle):
        state['sent'].append(handle.request.pose)
        if state['gate'] is not None:
            await state['gate']
        if handle.is_cancel_requested:
            handle.canceled()
        elif len(state['sent']) == state['fail_at']:
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    planner = ActionServer(fake, ComputePathThroughPoses, 'compute_path_through_poses', plan,
                           cancel_callback=lambda _: CancelResponse.ACCEPT)
    navigation = ActionServer(fake, NavigateToPose, 'navigate_to_pose', navigate,
                              goal_callback=lambda _: GoalResponse.REJECT if state['reject']
                              else GoalResponse.ACCEPT,
                              cancel_callback=lambda _: CancelResponse.ACCEPT)
    grid = node.catalog.grid
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.width, message.info.height = grid.width, grid.height
    message.info.resolution = grid.resolution
    message.info.origin.position.x, message.info.origin.position.y = grid.origin[:2]
    message.info.origin.orientation.w = 1.0
    message.data = np.flipud(np.where(grid.free, 0, -1)).ravel().tolist()
    node.on_map(message)
    spin_until(executor, lambda: node.planner.server_is_ready() and node.nav.server_is_ready())
    try:
        yield node, state, executor, message
    finally:
        for key in ('gate', 'plan_gate'):
            if state[key] is not None and not state[key].done():
                state[key].set_result(None)
        navigation.destroy()
        planner.destroy()
        executor.shutdown()
        fake.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_ros_actions_visit_start_via_destination_in_order(system):
    node, state, executor, _ = system
    node.command(String(data='Đi từ phòng A qua hành lang C rồi đến phòng B'))
    spin_until(executor, lambda: node.phase == 'idle')
    assert len(state['planned']) == 3
    assert len(state['sent']) == 3
    expected = [node.pose(i).pose.position for i in ['A', 'C', 'B']]
    assert [(p.pose.position.x, p.pose.position.y) for p in state['sent']] == [
        (p.x, p.y) for p in expected]


@pytest.mark.parametrize('mode', ['failed', 'rejected'])
def test_failed_leg_never_sends_remaining_goal(system, mode):
    node, state, executor, _ = system
    state['fail_at'] = 2 if mode == 'failed' else None
    state['reject'] = mode == 'rejected'
    node.command(String(data='Đi từ A qua C rồi đến B'))
    spin_until(executor, lambda: node.phase == 'idle')
    assert len(state['sent']) == (2 if mode == 'failed' else 0)


def test_unknown_waypoint_and_map_mismatch_do_not_send_actions(system):
    node, state, executor, message = system
    node.command(String(data='Đi từ A qua Z rồi đến B'))
    assert node.phase == 'idle'
    assert not state['planned']
    message.info.origin.position.x += 1
    node.on_map(message)
    node.command(String(data='Đi đến A'))
    assert not node.map_ready
    assert node.phase == 'idle'
    assert not state['planned']


@pytest.mark.parametrize('reason', ['cancel', 'timeout', 'map_changed'])
def test_cancel_or_timeout_waits_for_terminal_result_and_stops_route(system, reason):
    node, state, executor, message = system
    state['gate'] = Future()
    node.command(String(data='Đi từ A đến B'))
    spin_until(executor, lambda: len(state['sent']) == 1 and node.goal_handle is not None)
    if reason == 'cancel':
        node.command(String(data='Dừng lại'))
    elif reason == 'timeout':
        node.deadline = time.monotonic() - 1
        node.watchdog()
    else:
        message.data[0] = 0 if message.data[0] == -1 else -1
        node.on_map(message)
    assert node.phase == 'stopping'
    node.command(String(data='Đi đến C'))
    assert node.phase == 'stopping'
    # Process cancellation request before releasing fake action execution.
    for _ in range(15):
        executor.spin_once(timeout_sec=0.01)
    state['gate'].set_result(None)
    spin_until(executor, lambda: node.phase == 'idle')
    assert len(state['sent']) == 1


def test_cancel_during_planning_never_starts_navigation(system):
    node, state, executor, _ = system
    state['plan_gate'] = Future()
    node.command(String(data='Đi đến A'))
    spin_until(executor, lambda: bool(state['planned']) and node.goal_handle is not None)
    node.command(String(data='Dừng lại'))
    for _ in range(15):
        executor.spin_once(timeout_sec=0.01)
    state['plan_gate'].set_result(None)
    spin_until(executor, lambda: node.phase == 'idle')
    assert not state['sent']


def test_updated_catalog_is_loaded_before_next_route(system):
    node, state, executor, _ = system
    data = yaml.safe_load(node.regions_file.read_text())
    data['regions'][0].update(id='X', name='Vùng mới', aliases=[])
    data['regions'][0]['goal']['x'] += 0.2
    node.regions_file.write_text(yaml.safe_dump(data, allow_unicode=True))
    node.command(String(data='Đi đến vùng X'))
    spin_until(executor, lambda: node.phase == 'idle')
    assert len(state['sent']) == 1
    assert state['sent'][0].pose.position.x == data['regions'][0]['goal']['x']
    assert 'A' not in node.catalog.by_id
    assert 'X' in node.catalog.by_id


def test_invalid_updated_catalog_blocks_old_goals(system):
    node, state, executor, _ = system
    node.regions_file.write_text('regions: [broken YAML')
    node.command(String(data='Đi đến A'))
    assert node.phase == 'idle'
    assert not state['sent']
    assert not state['planned']


def test_saved_changes_do_not_replace_active_route(system):
    node, state, executor, _ = system
    state['gate'] = Future()
    node.command(String(data='Đi đến A'))
    spin_until(executor, lambda: len(state['sent']) == 1 and node.goal_handle is not None)
    old_revision = node.catalog.revision
    data = yaml.safe_load(node.regions_file.read_text())
    data['regions'][0].update(id='X', name='Vùng mới', aliases=[])
    node.regions_file.write_text(yaml.safe_dump(data, allow_unicode=True))
    node.command(String(data='Đi đến B'))
    assert node.catalog.revision == old_revision
    assert node.phase == 'navigating'
    state['gate'].set_result(None)
    spin_until(executor, lambda: node.phase == 'idle')
    node.command(String(data='Đi đến X'))
    spin_until(executor, lambda: node.phase == 'idle')
    assert node.catalog.revision != old_revision
    assert len(state['sent']) == 2
