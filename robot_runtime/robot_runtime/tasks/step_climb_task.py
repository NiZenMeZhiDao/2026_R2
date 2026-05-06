from dataclasses import dataclass, field

from robot_runtime.libraries.pid import PidAxis, PidGains


@dataclass
class StepClimbConfig:
    """Config for driving while the active suspension state machine runs."""

    move_direction: int = 0
    base_vx: float = 0.12
    base_vy: float = 0.0
    base_wz: float = 0.0
    correct_y: bool = True
    correct_wz: bool = True
    y_gains: PidGains = field(
        default_factory=lambda: PidGains(0.8, 0.0, 0.05, 0.5, 0.5)
    )
    wz_gains: PidGains = field(
        default_factory=lambda: PidGains(1.2, 0.0, 0.08, 1.2, 0.5)
    )

    @classmethod
    def forward(cls, speed=0.12, **kwargs):
        return cls(move_direction=0, base_vx=speed, **kwargs)

    @classmethod
    def left(cls, speed=0.12, **kwargs):
        return cls(move_direction=-1, base_vy=speed, **kwargs)

    @classmethod
    def right(cls, speed=0.12, **kwargs):
        return cls(move_direction=1, base_vy=-speed, **kwargs)


class StepClimbTask:
    """Task-side helper for chassis motion plus suspension climbing."""

    def __init__(self, core, config=None):
        self.core = core
        self.config = config or StepClimbConfig()
        self._pid_y = PidAxis(self.config.y_gains)
        self._pid_wz = PidAxis(self.config.wz_gains)

    def reset(self):
        self._pid_y.reset()
        self._pid_wz.reset()
        self.core.reset_suspension_math()

    def tick(self, pose_error=(0.0, 0.0, 0.0), now=None):
        """Run one task cycle and return the published command data."""
        _, y_error, yaw_error = _read_pose_error(pose_error)

        vx = self.config.base_vx
        vy = self.config.base_vy
        wz = self.config.base_wz

        if self.config.correct_y:
            vy += self._pid_y.update(y_error, now)
        if self.config.correct_wz:
            wz += self._pid_wz.update(yaw_error, now)

        self.core.update_direction(self.config.move_direction)
        chassis_msg = self.core.set_chassis_velocity(vx, vy, wz)
        suspension_result = self.core.run_suspension_math_once()

        return {
            'chassis': chassis_msg,
            'suspension': suspension_result,
            'velocity': [float(vx), float(vy), float(wz)],
        }


def _read_pose_error(pose_error):
    if hasattr(pose_error, 'linear') and hasattr(pose_error, 'angular'):
        return (
            float(pose_error.linear.x),
            float(pose_error.linear.y),
            float(pose_error.angular.z),
        )

    if hasattr(pose_error, 'x') and hasattr(pose_error, 'y'):
        yaw = getattr(pose_error, 'yaw', 0.0)
        return float(pose_error.x), float(pose_error.y), float(yaw)

    values = list(pose_error)
    if len(values) < 3:
        raise ValueError('pose_error must contain x, y and yaw')
    return float(values[0]), float(values[1]), float(values[2])
