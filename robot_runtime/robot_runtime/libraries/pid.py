import time
from dataclasses import dataclass


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
            derivative = (error - self.previous_error) / dt
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


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))

