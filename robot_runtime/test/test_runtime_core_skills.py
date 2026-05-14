from types import SimpleNamespace
import math

from robot_runtime.libraries.suspension_math import SuspensionPhase
from robot_runtime.runtime_core import RuntimeCore


class FakeNode:
    def create_timer(self, period, callback):
        self.period = period
        self.callback = callback
        return SimpleNamespace(period=period, callback=callback)


class FakeChassis:
    def __init__(self):
        self.commands = []

    def acquire(self, owner):
        return True

    def release(self, owner):
        return True

    def set_velocity(self, vx, vy=0.0, wz=0.0, owner=None):
        command = [float(vx), float(vy), float(wz)]
        self.commands.append(command)
        return SimpleNamespace(data=command)

    def set_twist(self, twist, owner=None):
        return self.set_velocity(twist.linear.x, twist.linear.y, twist.angular.z, owner)

    def stop(self, owner):
        return self.set_velocity(0.0, 0.0, 0.0, owner)


class FakeSuspension:
    def __init__(self, context):
        self.context = context
        self.commands = []

    def acquire(self, owner):
        return True

    def release(self, owner):
        return True

    def set_wheel_heights(self, heights, owner=None):
        command = [float(value) for value in heights[:4]]
        self.commands.append(command)
        self.context.suspension_target = command
        return SimpleNamespace(data=command)

    def set_all_height(self, height, owner=None):
        return self.set_wheel_heights([height] * 4, owner)

    def stop(self, owner):
        if self.context.wheel_heights:
            return self.set_wheel_heights(self.context.wheel_heights[:4], owner)
        return None


class FakeRobotBody:
    def __init__(self, node, context):
        self.chassis = FakeChassis()
        self.suspension = FakeSuspension(context)

    def stop_all(self, owner):
        self.chassis.stop(owner)
        self.suspension.stop(owner)


def test_move_skill_overwrites_previous_motion(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(FakeNode())

    core.move(0.1, 0.0, 0.0)
    core.move(0.2, 0.3, 0.4)
    core.tick_skills()

    assert core.body.chassis.commands == [[0.32, 0.42, 0.55]]


def test_runtime_core_uses_200hz_skill_timer_for_step_sequence(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    node = FakeNode()

    core = RuntimeCore(node)

    assert abs(core._skill_period - 0.005) < 1e-9
    assert abs(node.period - 0.005) < 1e-9


def test_stop_clears_active_skills_and_publishes_zero(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(FakeNode())

    core.move(0.2, 0.0, 0.0)
    core.set_height(40.0)
    core.stop()

    assert core.active_motion_skill is None
    assert core.active_suspension_skill is None
    assert core.body.chassis.commands[-1] == [0.0, 0.0, 0.0]


def test_set_height_overwrites_stepmode(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(FakeNode())

    core.set_stepmode(True, direction=1)
    core.set_height(55.0)
    core.tick_skills()

    assert core.active_suspension_skill['kind'] == 'height'
    assert core.body.suspension.commands == [[55.0, 55.0, 55.0, 55.0]]


def test_stepmode_runs_suspension_math_as_background_skill(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(FakeNode())
    core.context.filtered_distances = [100.0] * 8
    core.context.wheel_heights = [30.0] * 4
    core.context.suspension_target = [30.0] * 4

    core.set_stepmode(True, direction=0)
    for _ in range(6):
        core.tick_skills()

    assert core.last_suspension_result['phase'] != SuspensionPhase.IDLE
    assert core.body.suspension.commands


def test_move_to_can_still_be_started_as_async_skill(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(FakeNode())

    skill = core.move_to(1.0, 2.0, 0.3, wait=False)

    assert skill['kind'] == 'move_to'
    assert core.active_motion_skill['kind'] == 'move_to'
    assert core.context.move_to_target == [1.0, 2.0, 0.3]


def test_move_to_waits_until_pose_reaches_target(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(FakeNode())
    core.update_robot_pose(_pose('map', 1.0, 2.0, 0.3))

    result = core.move_to(1.0, 2.0, 0.3)

    assert result['reached'] is True
    assert core.active_motion_skill is None
    assert core.body.chassis.commands[-1] == [0.0, 0.0, 0.0]


def test_move_to_timeout_stops_motion(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(FakeNode())

    try:
        core.move_to(1.0, 0.0, 0.0, timeout=0.0)
        raised = False
    except TimeoutError:
        raised = True

    assert raised is True
    assert core.active_motion_skill is None
    assert core.body.chassis.commands[-1] == [0.0, 0.0, 0.0]


def test_move_to_velocity_uses_body_frame_left_positive(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(FakeNode())
    core.update_robot_pose(_pose('map', 0.0, 0.0, 0.0))
    core.move_to(0.0, 1.0, 0.0, wait=False)

    core.tick_skills()

    assert core.context.move_to_error == [0.0, 1.0, 0.0]
    assert core.context.move_to_body_error == [0.0, 1.0, 0.0]
    assert core.body.chassis.commands[-1] == [0.0, 0.32, 0.0]


def test_move_to_velocity_rotates_map_error_into_body_frame(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(FakeNode())
    core.update_robot_pose(_pose('map', 0.0, 0.0, math.pi / 2.0))
    core.move_to(1.0, 0.0, math.pi / 2.0, wait=False)

    core.tick_skills()

    assert abs(core.context.move_to_body_error[0]) < 1e-9
    assert abs(core.context.move_to_body_error[1] + 1.0) < 1e-9
    assert abs(core.body.chassis.commands[-1][0]) < 1e-9
    assert core.body.chassis.commands[-1][1] == -0.32
    assert abs(core.body.chassis.commands[-1][2]) < 1e-9


def test_chassis_deadzone_compensation_keeps_zero_commands_zero(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(FakeNode())

    core.move(0.0, -0.1, 0.2)
    core.tick_skills()
    core.stop()

    assert core.body.chassis.commands[0] == [0.0, -0.22, 0.35]
    assert core.body.chassis.commands[-1] == [0.0, 0.0, 0.0]


def _pose(frame_id, x, y, yaw):
    half = yaw / 2.0
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=math.sin(half),
                w=math.cos(half),
            ),
        ),
    )
