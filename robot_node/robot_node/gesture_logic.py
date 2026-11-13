# SPDX-License-Identifier: Apache-2.0
"""Gesture postprocessing independent of ROS, OpenCV and MediaPipe runtimes."""

from collections import deque
import math


GESTURE_LABELS = {
    'Waving': 'Vay tay',
    'Pointing_Up': 'Gio 1 ngon tay chi len',
    'Pointing_Down': 'Gio 1 ngon tay chi xuong',
    'Victory': 'Gio 2 ngon tay',
    'Thumb_Up': 'Gio ngon cai',
    'Open_Palm': 'Xoe ban tay',
    'Closed_Fist': 'Nam tay',
}


def pointing_direction(landmarks):
    """Return direction and geometric score, not a calibrated ML probability.

    Coordinates must have equal x/y scales (pixels or aspect-corrected units).
    Require a straight index finger, folded middle/ring/pinky, and vertical aim.
    """
    def distance(a, b):
        return math.hypot(a.x - b.x, a.y - b.y)

    wrist = landmarks[0]
    palm = distance(wrist, landmarks[9])
    if palm < 1e-6:
        return None
    base, joint, tip = (landmarks[i] for i in (5, 6, 8))
    ax, ay = base.x - joint.x, base.y - joint.y
    bx, by = tip.x - joint.x, tip.y - joint.y
    norm = math.hypot(ax, ay) * math.hypot(bx, by)
    if norm < 1e-9:
        return None
    straightness = -(ax * bx + ay * by) / norm
    if straightness < 0.85 or distance(tip, wrist) < distance(joint, wrist) + 0.15 * palm:
        return None
    for pip, end in ((10, 12), (14, 16), (18, 20)):
        if distance(landmarks[end], wrist) > distance(landmarks[pip], wrist) + 0.1 * palm:
            return None
    dx, dy = tip.x - base.x, tip.y - base.y
    length = math.hypot(dx, dy)
    if length < 0.3 * palm or abs(dy) < 1.5 * abs(dx):
        return None
    return ('Pointing_Up' if dy < 0 else 'Pointing_Down',
            min(straightness, abs(dy) / length))


def open_hand_score(landmarks, aspect_ratio):
    """Geometric support for an open hand during blurred/rotated wave frames."""
    def delta(a, b):
        return ((a.x - b.x) * aspect_ratio, a.y - b.y)

    def length(vector):
        return math.hypot(*vector)

    wrist = landmarks[0]
    palm = length(delta(landmarks[9], wrist))
    if palm < 1e-6:
        return 0.0
    extended = []
    for base, pip, tip in ((5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20)):
        a = delta(landmarks[base], landmarks[pip])
        b = delta(landmarks[tip], landmarks[pip])
        norm = length(a) * length(b)
        straightness = -(a[0] * b[0] + a[1] * b[1]) / norm if norm > 1e-9 else 0.0
        if (straightness >= 0.75 and length(delta(landmarks[tip], wrist))
                > length(delta(landmarks[pip], wrist)) + 0.15 * palm):
            extended.append(straightness)
    # Three extended fingers tolerate an occluded/bent finger while waving.
    return min(extended) if len(extended) >= 3 else 0.0


class WaveDetector:
    """Require an open palm with two horizontal reversals inside a time window."""

    def __init__(self, window_sec=2.0, min_motion=0.035):
        self.window_sec = window_sec
        self.min_motion = min_motion
        self.samples = deque()
        self.hand = None

    def reset(self):
        self.samples.clear()
        self.hand = None

    def update(self, now, x=None, score=0.0, hand=None):
        if x is None:
            self.reset()
            return None
        if self.samples and (now - self.samples[-1][0] > 0.45
                             or now <= self.samples[-1][0]
                             or hand != self.hand
                             or abs(x - self.samples[-1][1]) > 0.3):
            self.reset()
        self.hand = hand
        self.samples.append((now, x, score))
        while self.samples and now - self.samples[0][0] > self.window_sec:
            self.samples.popleft()
        anchor = self.samples[0][1]
        extreme = anchor
        direction = 0
        reversals = 0
        last_turn = None
        for timestamp, position, _ in self.samples:
            if direction == 0:
                if abs(position - anchor) >= self.min_motion:
                    direction = 1 if position > anchor else -1
                    extreme = position
            elif direction * (position - extreme) >= 0:
                extreme = position
            elif direction * (position - extreme) <= -self.min_motion:
                direction *= -1
                extreme = position
                reversals += 1
                last_turn = timestamp
        if (reversals >= 2 and now - self.samples[0][0] >= 0.35
                and last_turn is not None and now - last_turn <= 0.4):
            # Lowest accepted pose score (model or geometry) in this window.
            return min(sample[2] for sample in self.samples)
        return None

    def pause(self, now):
        """Preserve a short classification dropout, without emitting a stale wave."""
        if self.samples and now - self.samples[-1][0] > 0.45:
            self.reset()


class GestureInterpreter:
    def __init__(self, confidence=0.6, wave_window_sec=2.0, wave_min_motion=0.035):
        self.confidence = confidence
        self.wave = WaveDetector(wave_window_sec, wave_min_motion)
        # 0.1 * palm-length units: detect wrist rotation at any distance.
        self.tilt_wave = WaveDetector(wave_window_sec, min_motion=0.035)

    def reset_wave(self):
        self.wave.reset()
        self.tilt_wave.reset()

    def classify(self, category, score, landmarks, now, hand, aspect_ratio=1.0):
        """Return (stable topic label, score, score source); only seven labels."""
        if not landmarks:
            self.reset_wave()
            return 'None', 0.0, 'none'
        geometry_score = open_hand_score(landmarks, aspect_ratio)
        model_open = category == 'Open_Palm' and score >= self.confidence
        geometry_open = geometry_score >= 0.75 and (
            category in ('None', '', 'Open_Palm') or score < self.confidence)
        if model_open or geometry_open:
            pose_score = score if model_open else geometry_score
            center_x = sum(landmarks[i].x for i in (0, 5, 9, 13, 17)) / 5
            wave_score = self.wave.update(now, center_x, pose_score, hand)
            palm_length = math.hypot((landmarks[9].x - landmarks[0].x) * aspect_ratio,
                                     landmarks[9].y - landmarks[0].y)
            tilt_score = None
            if palm_length > 1e-6:
                finger_x = sum(landmarks[i].x for i in (8, 12, 16, 20)) / 4
                tilt = (finger_x - landmarks[0].x) * aspect_ratio / palm_length
                # WaveDetector's jump check is in signal units. Scale to keep
                # ordinary wrist swings below its tracking-jump threshold.
                tilt_score = self.tilt_wave.update(now, tilt * 0.1, pose_score, hand)
            else:
                self.tilt_wave.reset()
            if wave_score is not None or tilt_score is not None:
                return ('Waving', max(s for s in (wave_score, tilt_score) if s is not None),
                        'open-hand pose + motion rule')
        elif category in GESTURE_LABELS and score >= self.confidence:
            self.reset_wave()
        else:
            self.wave.pause(now)
            self.tilt_wave.pause(now)
        # Pointing_Up can be rotation invariant; determine actual image direction.
        if category in ('None', '', 'Pointing_Up'):
            from types import SimpleNamespace
            points = [SimpleNamespace(x=p.x * aspect_ratio, y=p.y) for p in landmarks]
            pointing = pointing_direction(points)
            if pointing is not None and pointing[1] >= self.confidence:
                return pointing[0], pointing[1], 'geometry rule'
            return 'None', 0.0, 'none'
        if category in GESTURE_LABELS and score >= self.confidence:
            return category, score, 'model'
        return 'None', 0.0, 'none'
