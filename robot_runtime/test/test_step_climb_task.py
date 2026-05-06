from types import SimpleNamespace

from robot_runtime.libraries.suspension_math import SuspensionPhase
from robot_runtime.tasks.step_climb_task import StepClimbConfig, StepClimbTask


class FakeCore:
    def __init__(self):
        self.directions = []
        self.velocities = []
        self.reset_count = 0
        self.suspension_count = 0

    def update_direction(self, value):
        self.directions.append(int(value))

    def set_chassis_velocity(self, vx, vy=0.0, wz=0.0):
        velocity = [float(vx), float(vy), float(wz)]
        self.velocities.append(velocity)
        return SimpleNamespace(data=velocity)

    def run_suspension_math_once(self):
        self.suspension_count += 1
        return {
            'phase': SuspensionPhase.UP_1_PREPARE,
            'wheel_targets': [205.0] * 4,
            'target_height': 205.0,
        }

    def reset_suspension_math(self):
        self.reset_count += 1


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

    assert core.directions == [-1]
    assert _close_list(result['velocity'], [0.01, 0.15, 0.12])
    assert core.suspension_count == 1


def test_step_climb_can_extend_to_right_direction_and_disable_wz_pid():
    core = FakeCore()
    config = StepClimbConfig.right(speed=0.18, base_wz=0.03, correct_wz=False)
    task = StepClimbTask(core, config)

    result = task.tick((0.0, -0.1, 0.2), now=1.0)

    assert core.directions == [1]
    assert _close_list(result['velocity'], [0.12, -0.26, 0.03])
    assert core.suspension_count == 1


def test_step_climb_reset_resets_suspension_math():
    core = FakeCore()
    task = StepClimbTask(core)

    task.reset()

    assert core.reset_count == 1


def _close_list(values, expected, tolerance=1e-9):
    return all(
        abs(float(value) - float(target)) <= tolerance
        for value, target in zip(values, expected)
    )
