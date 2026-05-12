import time
from dataclasses import dataclass
import math


@dataclass
class PidGains:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float = 0.0


class PidAxis:
    """Single-axis PID helper used by the chassis controller."""

    def __init__(self, gains: PidGains):
        self.gains = gains
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

    def update(self, error, now=None):
        now = time.monotonic() if now is None else now
        error = float(error)
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = max(now - self.previous_time, 1e-3)

        if dt > 0.0:
            self.integral += error * dt
            if self.gains.integral_limit > 0.0:
                self.integral = _clamp(
                    self.integral,
                    -self.gains.integral_limit,
                    self.gains.integral_limit,
                )
            derivative = self._error_delta(error) / dt
        else:
            derivative = 0.0

        self.previous_error = error
        self.previous_time = now

        output = (
            self.gains.kp * error +
            self.gains.ki * self.integral +
            self.gains.kd * derivative
        )
        return _clamp(output, -self.gains.output_limit, self.gains.output_limit)

    def _error_delta(self, error):
        return error - self.previous_error


class AnglePidAxis(PidAxis):
    """PID helper for wrapped radian errors such as yaw."""

    def update(self, error, now=None):
        error = float(error)
        if self.previous_time is None:
            continuous_error = wrap_angle(error)
        else:
            continuous_error = self.previous_error + wrap_angle(error - self.previous_error)
        return super().update(continuous_error, now)

    def _error_delta(self, error):
        return wrap_angle(error - self.previous_error)


def wrap_angle(angle):
    """Wrap a radian angle into [-pi, pi) so yaw error stays continuous."""
    wrapped = (float(angle) + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped == -math.pi and float(angle) > 0.0:
        return math.pi
    return wrapped


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))
