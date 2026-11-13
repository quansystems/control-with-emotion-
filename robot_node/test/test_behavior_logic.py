# SPDX-License-Identifier: Apache-2.0
import math

import pytest

from robot_node.behavior_logic import BehaviorController, GestureGate, scan_clearance


def sensors(c, now, x=0.0, y=0.0, yaw=0.0, clearance=5.0):
    c.receive_odometry(x, y, yaw, now)
    c.receive_scan(clearance, now)


def gesture(c, label, start, duration=0.35):
    for i in range(round(duration / 0.05) + 1):
        c.receive_gesture(label, start + i * 0.05)


def running(label='Pointing_Up', **options):
    c = BehaviorController(**options)
    sensors(c, 0.0)
    gesture(c, label, 0.0)
    sensors(c, 0.4)
    if label.casefold() not in ('wave', 'waving'):
        gesture(c, 'thumb_up', 0.4)
    assert c.state == 'RUNNING'
    assert not c.command_queue
    return c


@pytest.mark.parametrize('label, command', [('Pointing_Up', 'forward'),
    ('Pointing_Down', 'backward'), ('Victory', 'circle'), ('Closed_Fist', 'cancel')])
def test_level_1_only_queues_until_confirmation(label, command):
    c = BehaviorController()
    gesture(c, label, 0.0)
    assert c.state == 'SELECTED'
    assert list(c.command_queue) == [command]
    assert c.active_behavior is None
    assert c.sample(0.4) == (0.0, 0.0)


def test_latest_level_1_replaces_old_and_confirmation_consumes_once():
    c = BehaviorController()
    for i, label in enumerate(['Pointing_Up', 'Pointing_Down', 'Closed_Fist', 'Victory']):
        gesture(c, label, i * 0.4)
    assert list(c.command_queue) == ['circle']
    sensors(c, 1.6)
    gesture(c, 'thumb_up', 1.6)
    assert c.active_behavior == 'circle'
    assert not c.command_queue
    started = c.started_at
    gesture(c, 'thumb_up', 2.0)
    assert c.started_at == started


def test_confirm_alone_and_single_frame_noise_never_start():
    c = BehaviorController()
    sensors(c, 0.0)
    gesture(c, 'thumb_up', 0.0)
    assert c.state == 'IDLE'
    for i, label in enumerate(['Pointing_Up', 'thumb_up'] * 10):
        c.receive_gesture(label, 0.4 + i * 0.05)
        assert c.sample(0.4 + i * 0.05) == (0.0, 0.0)


@pytest.mark.parametrize('label', ['Waving', 'Wave', 'wave'])
def test_wave_automatic_only_with_empty_queue(label):
    c = BehaviorController()
    sensors(c, 0.0)
    gesture(c, 'Victory', 0.0)
    gesture(c, label, 0.4)
    assert c.active_behavior is None
    assert list(c.command_queue) == ['circle']
    c.receive_gesture('Open_Palm', 0.8)
    sensors(c, 0.9)
    gesture(c, label, 0.9)
    assert c.active_behavior == 'wave'
    assert not c.command_queue


@pytest.mark.parametrize('label', ['Pointing_Up', 'Pointing_Down', 'Victory'])
def test_lower_urgency_wave_cannot_interrupt_level_1_even_with_empty_queue(label):
    c = running(label)
    original_motion = (c.active_behavior, c.started_at, c.plan)
    assert c.active_level == 1 and not c.command_queue
    sensors(c, 0.8)
    gesture(c, 'Waving', 0.8)
    assert (c.active_behavior, c.started_at, c.plan) == original_motion


def test_running_wave_does_not_repeat_while_held():
    c = running('Waving')
    started = c.started_at
    gesture(c, 'Waving', 0.8)
    assert c.started_at == started


@pytest.mark.parametrize('label, queued', [('Pointing_Up', 'forward'),
    ('Pointing_Down', 'backward'), ('Victory', 'circle'), ('Closed_Fist', 'cancel')])
def test_level_1_interrupts_level_2_before_confirmation(label, queued):
    c = running('Waving')
    assert c.active_level == 2
    gesture(c, label, 0.8)
    assert c.active_behavior is None and c.active_level is None
    assert c.state == 'SELECTED' and c.pending_command == queued
    assert c.sample(1.15) == (0.0, 0.0)
    assert not c.plan
    c.receive_gesture('None', 1.2)
    assert c.pending_command == queued and c.active_behavior is None
    sensors(c, 1.3)
    gesture(c, 'thumb_up', 1.3)
    assert not c.command_queue
    assert c.active_behavior == (None if queued == 'cancel' else queued)


def test_queue_updates_while_running_and_confirm_preempts():
    c = running()
    old_plan = c.plan
    gesture(c, 'Victory', 0.8)
    assert c.pending_command == 'circle'
    assert c.active_behavior == 'forward' and c.plan is old_plan
    gesture(c, 'Waving', 1.2)
    assert c.active_behavior == 'forward' and c.pending_command == 'circle'
    sensors(c, 1.6)
    gesture(c, 'thumb_up', 1.6)
    assert c.active_behavior == 'circle' and not c.command_queue


def test_level_1_cancel_needs_confirmation_to_interrupt_same_priority():
    c = running()
    gesture(c, 'Closed_Fist', 0.8)
    assert c.state == 'RUNNING' and c.active_behavior == 'forward'
    gesture(c, 'thumb_up', 1.2)
    assert c.state == 'IDLE' and c.active_behavior is None
    assert not c.command_queue and c.sample(1.55) == (0.0, 0.0)


def test_pointing_down_replaces_forward_only_after_confirmation():
    c = running()
    gesture(c, 'Pointing_Down', 0.8)
    assert c.active_behavior == 'forward' and c.pending_command == 'backward'
    sensors(c, 1.2)
    gesture(c, 'thumb_up', 1.2)
    assert c.active_behavior == 'backward' and c.active_level == 1
    assert not c.command_queue and c.sample(1.55)[0] < 0


@pytest.mark.parametrize('motion', [None, 'Pointing_Up', 'Pointing_Down', 'Victory', 'Waving'])
def test_open_palm_immediately_stops_every_priority_and_clears_queue(motion):
    c = running(motion) if motion else BehaviorController()
    gesture(c, 'None' if motion == 'Waving' else 'Victory', 0.8)
    c.receive_gesture('Open_Palm', 1.2)
    assert c.state == 'STOPPED' and c.active_behavior is None
    assert not c.command_queue and c.sample(1.2) == (0.0, 0.0)
    gesture(c, 'thumb_up', 1.3)
    assert c.active_behavior is None


@pytest.mark.parametrize('invalid', ['None', '', 'garbage', 'Unknown'])
def test_unknown_is_noop_and_does_not_expire_queue_or_motion(invalid):
    c = running()
    gesture(c, 'Victory', 0.8)
    snapshot = (c.state, c.pending_command, c.active_behavior, c.reason, c.started_at, c.gate.latched)
    for i in range(120):
        now = 1.2 + i * 0.1
        sensors(c, now)
        c.receive_gesture(invalid, now)
        assert c.sample(now)[0] > 0
        assert (c.state, c.pending_command, c.active_behavior, c.reason,
                c.started_at, c.gate.latched) == snapshot


def test_pending_queue_does_not_expire_with_time_or_disconnected_topic():
    c = BehaviorController()
    gesture(c, 'Pointing_Up', 0.0)
    assert c.sample(100.0) == (0.0, 0.0) and c.pending_command == 'forward'
    sensors(c, 100.0)
    gesture(c, 'thumb_up', 100.0)
    assert c.state == 'RUNNING' and not c.command_queue


def test_topic_disconnect_stops_motion_but_preserves_pending_command():
    c = running()
    gesture(c, 'Victory', 0.8)
    sensors(c, 2.0)
    assert c.sample(2.0) == (0.0, 0.0)
    assert c.reason == 'gesture topic disconnected'
    assert c.state == 'SELECTED' and c.pending_command == 'circle'
    c.receive_gesture('None', 2.1)
    assert c.active_behavior is None


@pytest.mark.parametrize('missing', ['scan', 'odometry'])
def test_sensor_timeout_still_stops_with_live_unknown_messages(missing):
    c = running()
    for now in (0.8, 1.0, 1.2):
        if missing == 'scan':
            c.receive_odometry(0.0, 0.0, 0.0, now)
        else:
            c.receive_scan(5.0, now)
        c.receive_gesture('None', now)
        c.sample(now)
    assert c.active_behavior is None and missing in c.reason


def test_no_sensors_consumes_confirmed_command_without_moving():
    c = BehaviorController()
    gesture(c, 'Pointing_Up', 0.0)
    gesture(c, 'thumb_up', 0.4)
    assert not c.command_queue and c.state == 'IDLE'
    assert c.sample(0.8) == (0.0, 0.0)


def test_obstacle_stops_motion_and_preserves_pending_work():
    c = running()
    gesture(c, 'Victory', 0.8)
    c.receive_scan(0.5, 1.2)
    assert c.sample(1.2) == (0.0, 0.0)
    assert c.reason == 'obstacle or invalid scan' and c.pending_command == 'circle'


@pytest.mark.parametrize('label, clearance', [('Victory', 1.0), ('Pointing_Up', 1.5), ('Pointing_Down', 1.5)])
def test_insufficient_space_blocks_motion(label, clearance):
    c = BehaviorController()
    gesture(c, label, 0.0)
    sensors(c, 0.4, clearance=clearance)
    gesture(c, 'thumb_up', 0.4)
    assert c.active_behavior is None and not c.command_queue
    assert 'free space' in c.reason


def test_gate_rejects_sparse_frames_and_single_frame_rearming():
    gate = GestureGate()
    outputs = [gate.receive('confirm', t) for t in (0.0, 0.1, 0.2, 0.3, 0.4)]
    assert outputs.count('confirm') == 1
    gate.receive('wave', 0.5)
    for t in (0.6, 0.7, 0.8, 0.9):
        assert gate.receive('confirm', t) is None
    c = BehaviorController()
    for t in (0.0, 2.0, 4.0, 6.0):
        c.receive_gesture('Pointing_Up', t)
        assert c.state == 'IDLE'


@pytest.mark.parametrize('confirm', ['thumb_up', 'Thumb_Up', 'Thumbs_Up'])
def test_confirmation_aliases(confirm):
    c = BehaviorController()
    gesture(c, 'Victory', 0.0)
    sensors(c, 0.4)
    gesture(c, confirm, 0.4)
    linear, angular = c.sample(0.75)
    assert c.active_behavior == 'circle' and not c.command_queue
    assert linear / angular == pytest.approx(0.5)


@pytest.mark.parametrize('label, segments', [('Pointing_Up', 4), ('Pointing_Down', 4), ('Wave', 4), ('Victory', 1)])
def test_complete_motion_and_one_second_pauses_with_unknown_input(label, segments):
    c = running(label)
    assert len(c.plan) == segments
    x = y = yaw = 0.0
    now, previous_index = 0.75, 0
    pause_starts = []
    for _ in range(3000):
        now += 0.05
        sensors(c, now, x, y, yaw)
        c.receive_gesture('None', now)
        linear, angular = c.sample(now)
        if c.segment_index != previous_index:
            previous_index = c.segment_index
            if c.active_behavior is not None:
                pause_starts.append(now)
        if pause_starts and now - pause_starts[-1] < 1.0 - 1e-9:
            assert (linear, angular) == (0.0, 0.0)
        x += linear * math.cos(yaw + angular * 0.025) * 0.05
        y += linear * math.sin(yaw + angular * 0.025) * 0.05
        yaw += angular * 0.05
        if c.active_behavior is None:
            break
    assert c.reason == 'behavior completed' and c.segment_index == segments
    expected_x = 4 * c.travel_distance if label == 'Pointing_Up' else 0.0
    if label == 'Pointing_Down':
        expected_x = -4 * c.travel_distance
    assert abs(x - expected_x) < 0.05 and abs(y) < 0.04
    assert abs(math.remainder(yaw, 2 * math.pi)) < 0.04
    assert len(pause_starts) == segments - 1
    gesture(c, 'thumb_up', now + 0.1)
    assert c.active_behavior is None


def test_completion_preserves_pending_command_without_executing_it():
    c = running('Victory')
    gesture(c, 'Closed_Fist', 0.8)
    for i in range(1, 33):
        now = 1.2 + i * 0.01
        sensors(c, now, yaw=i * math.pi / 16)
    c.receive_gesture('None', now)
    assert c.sample(now) == (0.0, 0.0)
    assert c.pending_command == 'cancel' and c.state == 'SELECTED'


def test_odometry_discontinuity_invalid_data_and_demo_boundary():
    controller = running()
    controller.receive_odometry(10.0, 0.0, 0.0, 0.8)
    assert controller.sample(0.8) == (0.0, 0.0)
    assert 'discontinuity' in controller.reason
    controller = running()
    controller.receive_odometry(math.nan, 0.0, 0.0, 0.8)
    assert controller.sample(0.8) == (0.0, 0.0)
    controller = running()
    for i in range(1, 12):
        controller.receive_odometry(i * 0.2, 0.0, 0.0, 0.8 + i * 0.1)
    assert controller.reason == 'outside demo area'


@pytest.mark.parametrize('ranges, expected', [([5.0] * 360, 5.0),
                                            ([math.inf] * 360, 12.0),
                                            ([5.0] * 359 + [0.3], 0.3),
                                            ([math.nan] + [5.0] * 359, None),
                                            ([0.0] + [5.0] * 359, None),
                                            ([-math.inf] + [5.0] * 359, None),
                                            ([13.0] + [5.0] * 359, None)])
def test_scan_clearance(ranges, expected):
    assert scan_clearance(ranges, 2 * math.pi / 359, 0.164, 12.0) == expected


def test_partial_empty_or_malformed_scan_is_not_clear():
    assert scan_clearance([], 0.01, 0.1, 12.0) is None
    assert scan_clearance([5.0] * 180, math.pi / 179, 0.1, 12.0) is None
    assert scan_clearance([5.0] * 360, math.nan, 0.1, 12.0) is None


@pytest.mark.parametrize('options', [{'linear_speed': 1.0}, {'angular_speed': 2.0},
                                    {'circle_radius': 0.1}, {'circle_radius': 2.0},
                                    {'obstacle_distance': 0.1}, {'stable_sec': 1.0},
                                    {'control_timeout_sec': math.nan}])
def test_reject_unsafe_configuration(options):
    with pytest.raises(ValueError):
        BehaviorController(**options)
