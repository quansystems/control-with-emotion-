# SPDX-License-Identifier: Apache-2.0
"""Single-slot gesture queue, prioritized reactions and motion interlocks."""

from collections import deque
from dataclasses import dataclass
import math

from robot_node.velocity_filter import positive_finite


ALIASES = {
    'wave': 'wave', 'waving': 'wave', 'pointing_up': 'forward',
    'pointing_down': 'backward',
    'victory': 'circle', 'thumb_up': 'confirm', 'thumbs_up': 'confirm',
    'open_palm': 'stop', 'closed_fist': 'cancel',
}

COMMAND_LEVELS = {
    'forward': 1, 'backward': 1, 'circle': 1, 'cancel': 1,
    'wave': 2, 'stop': 0,
}


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_clearance(ranges, angle_increment, range_min, range_max):
    """Conservative all-around clearance; None means unusable scan.

    Require full 360-degree coverage for reversing/turning. Positive infinity
    means no return within range_max; NaN/negative/below-min data fail closed.
    """
    if (len(ranges) < 90 or not all(math.isfinite(v) for v in
                                  (angle_increment, range_min, range_max))
            or range_min < 0 or range_max <= range_min
            or not 2 * math.pi - 0.05 <= abs(angle_increment) * (len(ranges) - 1)
            <= 2 * math.pi + 0.1):
        return None
    nearest = range_max
    for distance in ranges:
        if distance == math.inf:
            continue
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            return None
        nearest = min(nearest, distance)
    return nearest


class GestureGate:
    """One event per sustained gesture; changing poses must also be stable."""

    def __init__(self, stable_sec=0.25, min_samples=3, max_gap_sec=0.8):
        self.stable_sec = stable_sec
        self.min_samples = min_samples
        self.max_gap_sec = max_gap_sec
        self.reset()

    def reset(self):
        self.candidate = None
        self.since = None
        self.count = 0
        self.latched = None
        self.last_received = None

    def receive(self, gesture, now):
        if self.last_received is not None and not 0 <= now - self.last_received < self.max_gap_sec:
            self.candidate, self.since, self.count = None, None, 0
        self.last_received = now
        if gesture != self.candidate:
            self.candidate, self.since, self.count = gesture, now, 1
        else:
            self.count += 1
        if (gesture is not None and self.count >= self.min_samples
                and now - self.since >= self.stable_sec and gesture != self.latched):
            self.latched = gesture
            return gesture
        return None


@dataclass(frozen=True)
class Segment:
    linear: float
    angular: float
    target: float


class BehaviorController:
    DEFAULTS = {
        'stable_sec': 0.25,
        'control_timeout_sec': 0.8,
        'sensor_timeout_sec': 0.6,
        'linear_speed': 0.12,
        'angular_speed': 0.5,
        'travel_distance': 0.35,
        'circle_radius': 0.5,
        'greeting_angle': 0.45,
        'segment_pause_sec': 1.0,
        'obstacle_distance': 0.55,
        'max_demo_radius': 2.0,
        'motion_timeout_sec': 90.0,
    }

    def __init__(self, **options):
        unknown = options.keys() - self.DEFAULTS.keys()
        if unknown:
            raise ValueError(f'Unknown options: {unknown}')
        for name, default in self.DEFAULTS.items():
            setattr(self, name, positive_finite(name, options.get(name, default)))
        if self.linear_speed > 0.2 or self.angular_speed > 0.8:
            raise ValueError('Demo limits: linear_speed <= 0.2, angular_speed <= 0.8.')
        if self.linear_speed / self.circle_radius > self.angular_speed:
            raise ValueError('circle_radius too small for configured speeds.')
        if max(2 * self.circle_radius, 4 * self.travel_distance) + 0.2 >= self.max_demo_radius:
            raise ValueError('Motion footprint must fit inside max_demo_radius with margin.')
        if self.obstacle_distance < 0.4:
            raise ValueError('obstacle_distance must be >= 0.4 m for TurtleBot 4.')
        if self.stable_sec >= self.control_timeout_sec:
            raise ValueError('stable_sec must be less than control_timeout_sec.')
        self.gate = GestureGate(self.stable_sec, max_gap_sec=self.control_timeout_sec)
        self.state = 'IDLE'
        self.command_queue = deque(maxlen=1)
        self.active_behavior = None
        self.reason = 'waiting for command'
        self.last_input = None
        self.last_odom = None
        self.last_scan = None
        self.pose = None
        self.anchor = None
        self.total_yaw = 0.0
        self.clearance = None
        self.plan = []
        self.segment_index = 0
        self.segment_origin = None
        self.pause_until = None
        self.started_at = None

    @property
    def pending_command(self):
        return self.command_queue[0] if self.command_queue else None

    @property
    def active_level(self):
        return COMMAND_LEVELS.get(self.active_behavior)

    def abort(self, reason, state=None):
        """Stop active motion; pending work is only cleared by level 0/confirm."""
        self.state = state or ('SELECTED' if self.command_queue else 'IDLE')
        self.reason = reason
        self.active_behavior = None
        self.plan = []
        self.pause_until = None
        self.segment_origin = None
        self.started_at = None
        # Keep the last accepted pose latched: a held gesture must never rearm
        # itself after completion, obstacle stop or a disconnected topic.
        latched = self.gate.latched
        self.gate.reset()
        self.gate.latched = latched

    def receive_odometry(self, x, y, yaw, now):
        if not all(math.isfinite(v) for v in (x, y, yaw, now)):
            self.last_odom = None
            self.abort('invalid odometry')
            return
        if self.pose is not None:
            delta_yaw = wrap_angle(yaw - self.pose[2])
            if math.hypot(x - self.pose[0], y - self.pose[1]) > 0.4 or abs(delta_yaw) > 1.0:
                self.last_odom = None
                self.pose = None
                self.abort('odometry discontinuity; restart in demo area')
                return
            self.total_yaw += delta_yaw
        self.pose = (x, y, yaw)
        self.last_odom = now
        if self.anchor is None:
            self.anchor = (x, y)
        if math.hypot(x - self.anchor[0], y - self.anchor[1]) >= self.max_demo_radius:
            self.abort('outside demo area')

    def receive_scan(self, clearance, now):
        self.clearance = clearance
        self.last_scan = now if clearance is not None else None
        if self.state == 'RUNNING' and (clearance is None or clearance <= self.obstacle_distance):
            self.abort('obstacle or invalid scan')

    def _expire(self, now):
        if self.state == 'RUNNING':
            if self.last_input is None or not 0 <= now - self.last_input < self.control_timeout_sec:
                self.abort('gesture topic disconnected')

    def receive_gesture(self, label, now):
        # A live recognizer sending None is different from a disconnected topic.
        # Unknown labels must not alter the queue, latch or running behavior.
        self.last_input = now
        gesture = ALIASES.get(label.strip().casefold())
        if gesture is None:
            return
        if gesture == 'stop':
            self.command_queue.clear()
            self.abort('level 0 open palm stop; queue cleared', 'STOPPED')
            self.gate.latched = gesture
            return
        event = self.gate.receive(gesture, now)
        if event is None:
            return
        if COMMAND_LEVELS.get(event) == 1:
            self.command_queue.append(event)  # deque(maxlen=1): latest replaces old.
            if self.active_level is not None and COMMAND_LEVELS[event] < self.active_level:
                # Higher urgency stops the lower-urgency motion now; starting
                # queued level-1 work still requires a separate thumb_up.
                self.abort('level 1 interrupted level 2; waiting for thumb_up')
                return
            if self.state != 'RUNNING':
                self.state = 'SELECTED'
            self.reason = 'level 1 queued; waiting for thumb_up'
        elif event == 'confirm' and self.command_queue:
            command = self.command_queue.popleft()
            self._execute(command, now)
        elif event == 'wave' and not self.command_queue and self.active_behavior is None:
            self._execute('wave', now)

    def _execute(self, command, now):
        # Numeric priority is inverse urgency. Low-urgency work cannot replace
        # higher-urgency active motion, even after its queue entry was consumed.
        if self.active_level is not None and COMMAND_LEVELS[command] > self.active_level:
            return
        if command == 'cancel':
            self.abort('confirmed cancel', 'IDLE')
            return
        fault = self._sensor_fault(now)
        if fault:
            self.abort(fault)
            return
        required = self.obstacle_distance
        if command == 'circle':
            required += 2 * self.circle_radius
        elif command in ('forward', 'backward'):
            required += 4 * self.travel_distance
        if self.clearance <= required:
            self.abort('not enough free space for requested behavior')
            return
        self.plan = self._make_plan(command)
        self.active_behavior = command
        self.state = 'RUNNING'
        self.reason = 'automatic level 2 reply' if command == 'wave' else 'level 1 confirmed; queue consumed'
        self.started_at, self.segment_index = now, 0
        self._start_segment()

    def _sensor_fault(self, now):
        for name, timestamp in (('odometry', self.last_odom), ('scan', self.last_scan)):
            if timestamp is None or not 0 <= now - timestamp < self.sensor_timeout_sec:
                return f'{name} missing or stale'
        if self.clearance is None or self.clearance <= self.obstacle_distance:
            return 'obstacle or invalid scan'
        if self.anchor is None or self.pose is None:
            return 'odometry missing'
        if math.hypot(self.pose[0] - self.anchor[0], self.pose[1] - self.anchor[1]) >= self.max_demo_radius:
            return 'outside demo area'
        return None

    def _make_plan(self, name):
        if name == 'wave':
            angle = self.greeting_angle
            return [Segment(0.0, self.angular_speed * sign, angle * factor)
                    for sign, factor in ((1, 1), (-1, 2), (1, 2), (-1, 1))]
        if name in ('forward', 'backward'):
            speed = self.linear_speed if name == 'forward' else -self.linear_speed
            return [Segment(speed, 0.0, self.travel_distance) for _ in range(4)]
        if name == 'circle':
            return [Segment(self.linear_speed, self.linear_speed / self.circle_radius, 2 * math.pi)]
        raise ValueError(f'Unsupported motion: {name}')

    def _start_segment(self):
        self.segment_origin = (*self.pose, self.total_yaw)
        self.pause_until = None

    def sample(self, now):
        """Called by a wall-clock timer. Always returns a bounded planar Twist."""
        self._expire(now)
        if self.state != 'RUNNING':
            return 0.0, 0.0
        fault = self._sensor_fault(now)
        if fault or now - self.started_at >= self.motion_timeout_sec:
            self.abort(fault or 'motion timed out')
            return 0.0, 0.0
        if self.pause_until is not None:
            if now < self.pause_until:
                return 0.0, 0.0
            self._start_segment()
        segment = self.plan[self.segment_index]
        x0, y0, yaw0, total0 = self.segment_origin
        if segment.angular:
            progress = (self.total_yaw - total0) * math.copysign(1.0, segment.angular)
        else:
            progress = ((self.pose[0] - x0) * math.cos(yaw0)
                        + (self.pose[1] - y0) * math.sin(yaw0)) * math.copysign(1.0, segment.linear)
        remaining = segment.target - progress
        if remaining <= (0.02 if segment.angular else 0.01):
            self.segment_index += 1
            if self.segment_index == len(self.plan):
                self.abort('behavior completed')
            else:
                self.pause_until = now + self.segment_pause_sec
            return 0.0, 0.0
        if segment.angular:
            angular = math.copysign(min(abs(segment.angular), max(0.1, remaining)), segment.angular)
            linear = angular * self.circle_radius if segment.linear else 0.0
            return linear, angular
        linear = math.copysign(min(abs(segment.linear), max(0.04, remaining)), segment.linear)
        heading_error = wrap_angle(yaw0 - self.pose[2])
        correction_limit = min(0.2, self.angular_speed)
        return linear, max(-correction_limit, min(correction_limit, heading_error))
