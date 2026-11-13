import math

import pytest

from emotion_control.detector import select_face
from emotion_control.logic import MotionController, Settings


def detect(controller, label, start=0.0, confidence=0.9):
    for i in range(3):
        now = start + i * 0.1
        controller.observe(label, confidence, now, now)
    return now


@pytest.mark.parametrize('label,expected', [
    ('happy', (0.1, 0.5)), ('sad', (0.1, -0.5)),
    ('angry', (-0.1, 0.0)), ('fear', (0.1, 0.0)),
    ('neutral', (0.0, 0.0)), ('surprise', (0.0, 0.5)),
])
def test_six_emotions(label, expected):
    c = MotionController()
    c.odometry(0, 0.2)
    now = detect(c, label)
    assert c.command(now) == expected


def test_debounce_and_repeated_frame():
    c = MotionController()
    for _ in range(10):
        c.observe('happy', 0.9, 0, 0)
    assert c.command(0) == (0, 0)
    assert c.samples == 1


@pytest.mark.parametrize('label,confidence', [
    (None, 0), ('disgust', 0.99), ('happy', 0.2), ('happy', float('nan')),
    ('neutral', 0.9),
])
def test_invalid_or_neutral_stops_immediately(label, confidence):
    c = MotionController()
    detect(c, 'happy')
    c.observe(label, confidence, 0.3, 0.3)
    assert c.command(0.3) == (0, 0)


def test_stale_detection_and_recovery_require_new_samples():
    c = MotionController()
    detect(c, 'happy')
    assert c.command(2.3) == (0, 0)
    c.observe('fear', 0.9, 3, 3)
    assert c.command(3) == (0, 0)
    c.observe('fear', 0.9, 3.1, 6)
    assert c.command(6) == (0, 0)


def test_step_finishes_and_requires_changed_expression():
    c = MotionController()
    detect(c, 'fear')
    assert c.command(1.3) == (0, 0)
    detect(c, 'fear', 1.4)
    assert c.command(1.6) == (0, 0)
    detect(c, 'neutral', 1.7)
    detect(c, 'fear', 2.0)
    assert c.command(2.2) == (0.1, 0)


def test_turn_one_revolution_across_pi_boundary():
    c = MotionController()
    c.odometry(3.0, 0.2)
    detect(c, 'surprise')
    for i in range(1, 65):
        now = 0.2 + i * 0.1
        angle = 3.0 + i * 0.1
        c.odometry(math.atan2(math.sin(angle), math.cos(angle)), now)
        c.observe('surprise', 0.9, now, now)
        command = c.command(now)
        if i < 62:
            assert command == (0, 0.5)
    assert command == (0, 0)
    assert c.rotation >= 2 * math.pi - 0.03


def test_turn_requires_fresh_odometry_and_has_timeout():
    c = MotionController()
    detect(c, 'surprise')
    assert c.command(0.2) == (0, 0)
    c = MotionController(Settings(turn_timeout=0.5))
    c.odometry(0, 0.2)
    detect(c, 'surprise')
    c.odometry(0.1, 0.8)
    assert c.command(0.8) == (0, 0)
    c = MotionController()
    c.odometry(0, 0.2)
    detect(c, 'surprise')
    assert c.command(0.8) == (0, 0)


@pytest.mark.parametrize('kwargs', [
    {'linear_speed': -1}, {'detection_timeout': 0},
    {'angular_speed': float('nan')}, {'confidence_threshold': 1.1},
    {'stable_samples': 1.5},
])
def test_invalid_settings(kwargs):
    with pytest.raises(ValueError):
        Settings(**kwargs)


def test_largest_face_and_unsupported_top_class():
    small = {'region': {'w': 10, 'h': 10}, 'emotion': {'happy': 99}}
    large = {'region': {'w': 30, 'h': 30}, 'emotion': {'disgust': 80, 'happy': 20}}
    assert select_face([small, large])[:2] == ('disgust', 0.8)
    assert select_face([]) == (None, 0, None)
