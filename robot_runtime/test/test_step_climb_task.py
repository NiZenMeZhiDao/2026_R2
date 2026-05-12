from types import SimpleNamespace
import math

from robot_runtime.libraries.suspension_math import SuspensionPhase
from robot_runtime.tasks.step_climb_task import StepClimbConfig, StepClimbTask


class FakeCore:
    def __init__(self):
        self.directions = []
        self.velocities = []
        self.reset_count = 0
        self.suspension_count = 0
        self.stop_count = 0
        self.stepmode_commands = []
        self.last_suspension_result = None
        self.context = SimpleNamespace(suspension_target=[])
        self.next_phase = SuspensionPhase.UP_1_PREPARE

    def set_stepmode(self, enabled, direction=0):
        self.stepmode_commands.append([bool(enabled), int(direction)])
        if enabled:
            self.directions.append(int(direction))

    def move(self, vx, vy=0.0, wz=0.0, duration=0.0):
        velocity = [float(vx), float(vy), float(wz)]
        self.velocities.append(velocity)
        return {'kind': 'move', 'velocity': velocity, 'duration': float(duration)}

    def tick_skills(self):
        self.suspension_count += 1
        self.last_suspension_result = {
            'phase': self.next_phase,
            'wheel_targets': [205.0] * 4,
            'target_height': 205.0,
        }

    def stop(self):
        self.stop_count += 1


def test_forward_step_climb_keeps_base_speed_and_corrects_y_wz():
    core = FakeCore()
    task = StepClimbTask(core, StepClimbConfig.forward(speed=0.2))

    result = task.tick((0.0, 0.1, -0.2), now=1.0)

    assert core.directions == [0]
    assert core.suspension_count == 1
    assert _close_list(result['velocity'], [0.2, 0.08, -0.24])
    assert core.velocities == [result['velocity']]
    assert result['suspension']['phase'] == SuspensionPhase.UP_1_PREPARE


def test_step_climb_can_extend_to_left_direction_and_custom_speed():
    core = FakeCore()
    config = StepClimbConfig.left(speed=0.15, base_vx=0.01, correct_y=False)
    task = StepClimbTask(core, config)

    result = task.tick((0.0, 0.3, 0.1), now=1.0)

    assert core.directions == [1]
    assert _close_list(result['velocity'], [0.01, 0.15, 0.12])
    assert core.suspension_count == 1


def test_step_climb_can_extend_to_right_direction_and_disable_wz_pid():
    core = FakeCore()
    config = StepClimbConfig.right(speed=0.18, base_wz=0.03, correct_wz=False)
    task = StepClimbTask(core, config)

    result = task.tick((0.0, -0.1, 0.2), now=1.0)

    assert core.directions == [-1]
    assert _close_list(result['velocity'], [0.12, -0.26, 0.03])
    assert core.suspension_count == 1


def test_step_climb_does_not_double_tick_when_core_has_background_timer():
    core = FakeCore()
    core._skill_timer = object()
    task = StepClimbTask(core)

    result = task.tick((0.0, 0.0, 0.0), now=1.0)

    assert core.suspension_count == 0
    assert result['suspension']['phase'] == SuspensionPhase.IDLE


def test_step_climb_wz_pid_wraps_yaw_error_across_pi():
    core = FakeCore()
    config = StepClimbConfig.forward(
        speed=0.0,
        correct_y=False,
        wz_gains=SimpleNamespace(
            kp=0.0,
            ki=0.0,
            kd=1.0,
            output_limit=10.0,
            integral_limit=0.0,
        ),
    )
    task = StepClimbTask(core, config)

    task.tick((0.0, 0.0, math.radians(179.0)), now=1.0)
    result = task.tick((0.0, 0.0, math.radians(-179.0)), now=2.0)

    assert abs(result['velocity'][2] - math.radians(2.0)) < 1e-9


def test_step_climb_reset_resets_suspension_math():
    core = FakeCore()
    task = StepClimbTask(core)

    task.reset()

    assert core.stepmode_commands == [[False, 0]]


def test_step_climb_is_done_after_sequence_returns_to_idle():
    core = FakeCore()
    task = StepClimbTask(core)

    task.tick((0.0, 0.0, 0.0), now=1.0)
    assert task.is_done() is False

    core.next_phase = SuspensionPhase.IDLE
    task.tick((0.0, 0.0, 0.0), now=1.1)

    assert task.is_done() is True


def test_step_climb_stop_calls_core_stop():
    core = FakeCore()
    task = StepClimbTask(core)

    task.stop()

    assert core.stop_count == 1


def _close_list(values, expected, tolerance=1e-9):
    return all(
        abs(float(value) - float(target)) <= tolerance
        for value, target in zip(values, expected)
    )
