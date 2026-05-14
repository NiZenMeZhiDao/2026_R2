from dataclasses import dataclass, field

from robot_runtime.libraries.pid import AnglePidAxis, PidAxis, PidGains
from robot_runtime.libraries.suspension_math import SuspensionPhase
from robot_runtime.pid_config import pid_gains


@dataclass
class StepClimbConfig:
    """Config for driving while the active suspension state machine runs."""

    move_direction: int = 0
    base_vx: float = 0.12
    base_vy: float = 0.0
    base_wz: float = 0.0
    control_period: float = 0.005
    timeout: float = 10.0
    correct_y: bool = True
    correct_wz: bool = True
    y_gains: PidGains = field(
        default_factory=lambda: pid_gains('step_climb', 'y')
    )
    wz_gains: PidGains = field(
        default_factory=lambda: pid_gains('step_climb', 'wz')
    )

    @classmethod
    def forward(cls, speed=0.12, **kwargs):
        return cls(move_direction=0, base_vx=speed, **kwargs)

    @classmethod
    def left(cls, speed=0.12, **kwargs):
        kwargs.setdefault('base_vx', 0.0)
        return cls(move_direction=1, base_vy=speed, **kwargs)

    @classmethod
    def right(cls, speed=0.12, **kwargs):
        kwargs.setdefault('base_vx', 0.0)
        return cls(move_direction=-1, base_vy=-speed, **kwargs)


class StepClimbTask:
    """Task-side helper for chassis motion plus suspension climbing."""

    def __init__(self, core, config=None):
        self.core = core
        self.config = config or StepClimbConfig()
        self._pid_y = PidAxis(self.config.y_gains)
        self._pid_wz = AnglePidAxis(self.config.wz_gains)
        self._has_started_sequence = False
        self._last_result = None
        self._stepmode_enabled = False

    def reset(self):
        self._pid_y.reset()
        self._pid_wz.reset()
        self._has_started_sequence = False
        self._last_result = None
        self._stepmode_enabled = False
        self.core.set_stepmode(False)

    def tick(self, pose_error=(0.0, 0.0, 0.0), now=None):
        """Run one task cycle and return the published command data."""
        forward_error, left_error, yaw_error = _read_pose_error(pose_error)

        vx = self.config.base_vx
        vy = self.config.base_vy
        wz = self.config.base_wz

        if self.config.correct_y:
            correction = self._pid_y.update(
                _cross_track_error(
                    self.config.move_direction,
                    forward_error,
                    left_error,
                ),
                now,
            )
            if int(self.config.move_direction) == 0:
                vy += correction
            else:
                vx += correction
        if self.config.correct_wz:
            wz += self._pid_wz.update(yaw_error, now)

        if not self._stepmode_enabled:
            self.core.set_stepmode(True, self.config.move_direction)
            self._stepmode_enabled = True

        chassis_skill = self.core.move(vx, vy, wz, 0.0)
        if not _core_has_background_tick(self.core):
            self.core.tick_skills()
        suspension_result = self.core.last_suspension_result or {
            'phase': _current_suspension_phase(self.core),
            'wheel_targets': list(self.core.context.suspension_target),
            'target_height': 0.0,
        }
        phase = suspension_result['phase']
        if phase.name != 'IDLE':
            self._has_started_sequence = True

        self._last_result = {
            'chassis': chassis_skill,
            'suspension': suspension_result,
            'velocity': [float(vx), float(vy), float(wz)],
        }
        return self._last_result

    def is_done(self):
        if not self._has_started_sequence or self._last_result is None:
            return False
        return self._last_result['suspension']['phase'].name == 'IDLE'

    def stop(self):
        self.core.stop()


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


def _cross_track_error(move_direction, forward_error, left_error):
    if int(move_direction) == 0:
        return left_error
    return forward_error


def _core_has_background_tick(core):
    return getattr(core, '_skill_timer', None) is not None


def _current_suspension_phase(core):
    suspension_math = getattr(core, 'suspension_math', None)
    state = getattr(suspension_math, 'state', None)
    return getattr(state, 'phase', SuspensionPhase.IDLE)
