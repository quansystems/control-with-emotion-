"""Motion policy independent of ROS, camera and neural network dependencies."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Settings:
    confidence_threshold: float = 0.6
    stable_samples: int = 3
    detection_timeout: float = 2.0
    linear_speed: float = 0.10
    angular_speed: float = 0.50
    arc_duration: float = 3.0
    step_duration: float = 1.0
    turn_timeout: float = 20.0
    odom_timeout: float = 0.5

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if self.confidence_threshold > 1:
            raise ValueError('confidence_threshold must be <= 1')
        if not isinstance(self.stable_samples, int):
            raise ValueError('stable_samples must be an integer')


class MotionController:
    EMOTIONS = {'happy', 'sad', 'neutral', 'angry', 'surprise', 'fear'}

    def __init__(self, settings=None):
        self.settings = settings or Settings()
        self.candidate = None
        self.samples = 0
        self.latched = None
        self.active = None
        self.last_detection = -math.inf
        self.last_sample = -math.inf
        self.started = 0.0
        self.yaw = None
        self.odom_time = -math.inf
        self.rotation = 0.0

    def odometry(self, yaw, now):
        if not math.isfinite(yaw):
            return
        if self.active == 'surprise' and self.yaw is not None:
            # Signed increments handle crossing +/-pi without losing a revolution.
            self.rotation += math.atan2(math.sin(yaw - self.yaw),
                                        math.cos(yaw - self.yaw))
        self.yaw = yaw
        self.odom_time = now

    def observe(self, emotion, confidence, captured_at, now):
        if captured_at <= self.last_sample:
            return
        self.last_sample = captured_at
        valid = (emotion in self.EMOTIONS and math.isfinite(confidence)
                 and confidence >= self.settings.confidence_threshold
                 and 0 <= now - captured_at <= self.settings.detection_timeout)
        if not valid:
            self.active = None
            self.candidate = None
            self.samples = 0
            return
        if captured_at - self.last_detection > self.settings.detection_timeout:
            self.candidate, self.samples = None, 0
        self.last_detection = captured_at
        if emotion != self.candidate:
            self.candidate, self.samples = emotion, 0
            self.active = None
        self.samples += 1
        # A neutral observation stops immediately, even before debounce finishes.
        if emotion == 'neutral':
            self.active = None
        if self.samples < self.settings.stable_samples or emotion == self.latched:
            return
        self.latched = emotion
        self.active = None if emotion == 'neutral' else emotion
        self.started = now
        self.rotation = 0.0

    def command(self, now):
        s = self.settings
        if now - self.last_detection > s.detection_timeout:
            self.active = None
            self.candidate, self.samples = None, 0
        if self.active is None:
            return 0.0, 0.0
        age = now - self.started
        duration = s.arc_duration if self.active in ('happy', 'sad') else s.step_duration
        if self.active == 'surprise':
            if (now - self.odom_time > s.odom_timeout
                    or self.yaw is None or age >= s.turn_timeout
                    or self.rotation >= 2 * math.pi - 0.03):
                self.active = None
                return 0.0, 0.0
            return 0.0, s.angular_speed
        if age >= duration:
            self.active = None
            return 0.0, 0.0
        return {
            'happy': (s.linear_speed, s.angular_speed),
            'sad': (s.linear_speed, -s.angular_speed),
            'angry': (-s.linear_speed, 0.0),
            'fear': (s.linear_speed, 0.0),
        }[self.active]
