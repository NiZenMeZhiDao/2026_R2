from types import SimpleNamespace

from robot_runtime.libraries.suspension_math import SuspensionPhase
from robot_runtime.step_climb_forward import (
    _pose_error_from_reference,
    climb_step,
    run_task,
)
from robot_runtime.tasks import StepClimbConfig


class FakeCore:
    def __init__(self):
        self.calls = []
        self.context = SimpleNamespace(
            relative_pose_error=[0.0, 0.0, 0.0],
            robot_pose_map_xytheta=[0.0, 0.0, 0.0],
            robot_pose_odom_xytheta=[],
            robot_x=0.0,
            robot_y=0.0,
            robot_theta=0.0,
            odom_x=0.0,
            odom_y=0.0,
            odom_theta=0.0,
            suspension_target=[],
            localization_ready=True,
            odom_ready=True,
        )
        self.last_suspension_result = None
        self.pose_after_first_tick = None

    def move_to(self, x, y, theta, timeout=10.0, **kwargs):
        self.calls.append(('move_to', float(x), float(y), float(theta), float(timeout)))
        return {'reached': True}

    def set_stepmode(self, enabled, direction=0):
        self.calls.append(('set_stepmode', bool(enabled), int(direction)))
        if enabled:
            self._step_ticks_remaining = 1

    def set_height(self, height):
        self.calls.append(('set_height', float(height)))

    def move(self, vx, vy=0.0, wz=0.0, duration=0.0):
        self.calls.append(('move', float(vx), float(vy), float(wz)))
        return {'kind': 'move'}

    def tick_skills(self):
        phase = SuspensionPhase.IDLE
        if getattr(self, '_step_ticks_remaining', 0) > 0:
            phase = SuspensionPhase.UP_1_PREPARE
            self._step_ticks_remaining -= 1
        self.last_suspension_result = {
            'phase': phase,
            'wheel_targets': [205.0] * 4,
            'target_height': 205.0,
        }
        if self.pose_after_first_tick is not None:
            self.context.robot_pose_map_xytheta = list(self.pose_after_first_tick)
            self.pose_after_first_tick = None

    def stop(self):
        self.calls.append(('stop',))


def test_run_task_is_easy_to_rewrite_top_level_skill_sequence():
    core = FakeCore()

    run_task(core)

    assert _find_calls(core, 'move_to') == [
        ('move_to', 1.2, 0.0, 1.5708, 10.0),
        ('move_to', 1.2, 1.2, 0.0, 10.0),
    ]
    assert ('set_stepmode', False, 0) in core.calls
    assert ('set_height', 30.0) in core.calls
    assert _find_calls(core, 'set_stepmode').count(('set_stepmode', True, 0)) == 3


def test_run_task_stops_after_first_climb_when_localization_is_not_ready():
    core = FakeCore()
    core.context.localization_ready = False
    core.context.odom_ready = False

    run_task(core)

    assert _find_calls(core, 'move_to') == []
    assert _find_calls(core, 'stop')
    assert _find_calls(core, 'set_stepmode').count(('set_stepmode', True, 0)) == 1


def test_climb_step_holds_forward_reference_y_and_theta():
    core = FakeCore()
    core.context.robot_pose_map_xytheta = [1.0, 2.0, 0.0]
    core.pose_after_first_tick = [1.2, 2.1, 0.1]

    climb_step(core, StepClimbConfig.forward(speed=0.0, timeout=0.05))

    move_calls = _find_calls(core, 'move')
    assert move_calls[-1][1] == 0.0
    assert move_calls[-1][2] < 0.0
    assert move_calls[-1][3] < 0.0


def test_climb_step_holds_side_reference_x_and_theta():
    core = FakeCore()
    core.context.robot_pose_map_xytheta = [1.0, 2.0, 0.0]
    core.pose_after_first_tick = [1.1, 2.0, -0.1]

    climb_step(core, StepClimbConfig.left(speed=0.0, base_vx=0.0, timeout=0.05))

    move_calls = _find_calls(core, 'move')
    assert move_calls[-1][1] < 0.0
    assert move_calls[-1][2] == 0.0
    assert move_calls[-1][3] > 0.0


def test_pose_error_from_reference_uses_body_frame_for_forward_motion():
    core = FakeCore()
    core.context.robot_pose_map_xytheta = [1.1, 2.0, 1.57079632679]

    error = _pose_error_from_reference(core, (1.0, 2.0, 1.57079632679), 0)

    assert abs(error[0]) < 1e-6
    assert abs(error[1] - 0.1) < 1e-6


def test_pose_error_from_reference_uses_current_heading_for_body_frame():
    core = FakeCore()
    core.context.robot_pose_map_xytheta = [0.1, 0.0, 1.57079632679]

    error = _pose_error_from_reference(core, (0.0, 0.0, 1.57079632679), 0)

    assert abs(error[0]) < 1e-6
    assert abs(error[1] - 0.1) < 1e-6


def test_pose_error_from_reference_skips_correction_until_localization_ready():
    core = FakeCore()
    core.context.localization_ready = False
    core.context.robot_pose_map_xytheta = [3.0, 4.0, 1.0]

    error = _pose_error_from_reference(core, None, 0)

    assert error == (0.0, 0.0, 0.0)


def _find_calls(core, name):
    return [call for call in core.calls if call[0] == name]
