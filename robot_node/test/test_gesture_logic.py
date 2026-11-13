# SPDX-License-Identifier: Apache-2.0
from types import SimpleNamespace
import math

import pytest

from robot_node.gesture_logic import GestureInterpreter, WaveDetector


def pointing_hand(down=False, horizontal=False, aspect=1.0):
    points = [SimpleNamespace(x=0.5, y=0.5) for _ in range(21)]
    positions = {
        0: (0.5, 0.8), 5: (0.45, 0.6), 6: (0.45, 0.45), 8: (0.45, 0.2),
        9: (0.5, 0.6), 10: (0.5, 0.5), 12: (0.5, 0.65),
        14: (0.55, 0.5), 16: (0.55, 0.65), 18: (0.6, 0.55), 20: (0.6, 0.7),
    }
    for i, (x, y) in positions.items():
        if down:
            y = 1.0 - y
        if horizontal:
            x, y = y, x
        points[i] = SimpleNamespace(x=x / aspect, y=y)
    return points


@pytest.mark.parametrize('down, expected', [(False, 'Pointing_Up'), (True, 'Pointing_Down')])
@pytest.mark.parametrize('aspect', [1.0, 16 / 9])
def test_pointing_orientation(down, expected, aspect):
    output = GestureInterpreter().classify('None', 0.1, pointing_hand(down, aspect=aspect),
                                            0.0, 'Left', aspect)
    assert output[0] == expected
    assert output[1] > 0.9
    assert output[2] == 'geometry rule'


def test_reject_sideways_pointing_and_extended_middle_finger():
    interpreter = GestureInterpreter()
    assert interpreter.classify('Pointing_Up', 0.99, pointing_hand(horizontal=True),
                                0.0, 'Left')[0] == 'None'
    points = pointing_hand()
    points[12].y = 0.2
    assert interpreter.classify('None', 0.0, points, 0.0, 'Left')[0] == 'None'


@pytest.mark.parametrize('category', ['Thumb_Down', 'ILoveYou', 'Unknown'])
def test_reject_unsupported_model_classes(category):
    assert GestureInterpreter().classify(category, 0.99, pointing_hand(),
                                         0.0, 'Left')[0] == 'None'


@pytest.mark.parametrize('category', ['Victory', 'Thumb_Up', 'Open_Palm', 'Closed_Fist'])
def test_static_gestures_and_confidence(category):
    interpreter = GestureInterpreter()
    assert interpreter.classify(category, 0.9, pointing_hand(), 0.0, 'Left')[:2] == (category, 0.9)
    assert interpreter.classify(category, 0.2, pointing_hand(), 0.1, 'Left')[0] == 'None'


def test_wave_requires_reversals_not_stationary_palm_or_one_sweep():
    detector = WaveDetector()
    for i, x in enumerate([0.3, 0.31, 0.3, 0.32, 0.3]):
        assert detector.update(i * 0.15, x, 0.9, 'Left') is None
    detector.reset()
    for i, x in enumerate([0.3, 0.35, 0.4, 0.45, 0.5]):
        assert detector.update(i * 0.15, x, 0.9, 'Left') is None
    detector.reset()
    scores = [detector.update(i * 0.15, x, 0.9, 'Left')
              for i, x in enumerate([0.3, 0.4, 0.5, 0.4, 0.3, 0.4, 0.5])]
    assert scores[-1] == 0.9
    assert detector.update(1.25, 0.5, 0.9, 'Left') is None


@pytest.mark.parametrize('reset_reason', ['missing', 'hand_change', 'gap'])
def test_wave_does_not_join_separate_hands_or_tracking_gaps(reset_reason):
    detector = WaveDetector()
    for i, x in enumerate([0.3, 0.4, 0.5, 0.4, 0.3]):
        detector.update(i * 0.15, x, 0.9, 'Left')
    if reset_reason == 'missing':
        detector.update(0.7)
        result = detector.update(0.75, 0.4, 0.9, 'Left')
    elif reset_reason == 'hand_change':
        result = detector.update(0.75, 0.4, 0.9, 'Right')
    else:
        result = detector.update(1.5, 0.4, 0.9, 'Left')
    assert result is None


def test_wave_priority_and_hand_loss():
    interpreter = GestureInterpreter()
    for i, x in enumerate([0.3, 0.4, 0.5, 0.4, 0.3, 0.4, 0.5]):
        points = [SimpleNamespace(x=x, y=0.5) for _ in range(21)]
        gesture, score, _ = interpreter.classify('Open_Palm', 0.9, points, i * 0.15, 'Left')
    assert (gesture, score) == ('Waving', 0.9)
    assert interpreter.classify('None', 0.0, [], 1.0, None)[0] == 'None'
    assert not interpreter.wave.samples


def open_hand(angle=0.0, scale=1.0, shift=0.0):
    """Rotate a realistic open hand about a fixed wrist in square image units."""
    positions = [(0.0, 0.0)] * 21
    for base, x in ((5, -0.04), (9, 0.0), (13, 0.035), (17, 0.065)):
        for offset, y in enumerate((-0.1, -0.16, -0.20, -0.24)):
            positions[base + offset] = (x, y)
    return [SimpleNamespace(
        x=0.5 + shift + scale * (x * math.cos(angle) - y * math.sin(angle)),
        y=0.75 + scale * (x * math.sin(angle) + y * math.cos(angle)))
        for x, y in positions]


@pytest.mark.parametrize('scale', [0.4, 1.0, 1.4])
@pytest.mark.parametrize('aspect', [1.0, 16 / 9])
def test_wrist_wave_without_large_palm_translation(scale, aspect):
    # Disable the translation channel to isolate distance-independent wrist tilt.
    interpreter = GestureInterpreter(wave_min_motion=1.0)
    outputs = []
    for i, angle in enumerate([-0.4, 0.0, 0.4, 0.0, -0.4, 0.0, 0.4]):
        points = open_hand(angle, scale)
        for p in points:
            p.x /= aspect
        outputs.append(interpreter.classify('Open_Palm', 0.9, points,
                                            i * 0.12, 'Left', aspect)[0])
    assert outputs[-1] == 'Waving'


def test_small_translating_wave_with_default_threshold():
    interpreter = GestureInterpreter()
    outputs = [interpreter.classify('Open_Palm', 0.9, open_hand(shift=shift),
                                    i * 0.12, 'Left')[0]
               for i, shift in enumerate([-0.025, 0.0, 0.025, 0.0, -0.025, 0.0, 0.025])]
    assert outputs[-1] == 'Waving'


def test_wrist_wave_with_low_or_missing_open_palm_classification():
    interpreter = GestureInterpreter()
    outputs = [interpreter.classify(category, score, open_hand(angle),
                                    i * 0.12, 'Left')[0]
               for i, (angle, category, score) in enumerate([
                   (-0.4, 'Open_Palm', 0.9), (0.0, 'Open_Palm', 0.4),
                   (0.4, 'None', 0.1), (0.0, 'Open_Palm', 0.9),
                   (-0.4, 'None', 0.1), (0.0, 'Open_Palm', 0.4),
                   (0.4, 'Open_Palm', 0.9)])]
    assert outputs[-1] == 'Waving'


def test_short_uncertain_pose_does_not_erase_wave_history():
    interpreter = GestureInterpreter(wave_min_motion=1.0)
    for i, angle in enumerate([-0.4, 0.0, 0.4, 0.0, -0.4]):
        interpreter.classify('Open_Palm', 0.9, open_hand(angle), i * 0.1, 'Left')
    assert interpreter.classify('None', 0.0, pointing_hand(), 0.5, 'Left')[0] != 'Waving'
    assert interpreter.classify('Open_Palm', 0.9, open_hand(0.4), 0.6, 'Left')[0] == 'Waving'


def test_static_palm_and_jitter_do_not_become_wave():
    interpreter = GestureInterpreter()
    for i in range(60):
        angle = 0.01 if i % 2 else -0.01
        assert interpreter.classify('Open_Palm', 0.9, open_hand(angle),
                                     i * 0.1, 'Left')[0] == 'Open_Palm'


def test_switching_to_fist_clears_both_motion_channels():
    interpreter = GestureInterpreter()
    for i, angle in enumerate([-0.4, 0.0, 0.4, 0.0, -0.4]):
        interpreter.classify('Open_Palm', 0.9, open_hand(angle), i * 0.1, 'Left')
    assert interpreter.classify('Closed_Fist', 0.95, pointing_hand(), 0.5, 'Left')[0] == 'Closed_Fist'
    assert not interpreter.wave.samples
    assert not interpreter.tilt_wave.samples
