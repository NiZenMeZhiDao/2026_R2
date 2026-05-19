from types import SimpleNamespace
import math

from robot_runtime.runtime_core import RuntimeCore


class FakeRobotBody:
    def __init__(self, node, context):
        self.node = node
        self.context = context


def test_runtime_core_keeps_map_pose_alias(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(node=SimpleNamespace())
    pose = _pose('map', 1.0, 2.0, math.pi / 2.0)

    core.update_robot_pose(pose)

    assert core.context.robot_pose is pose
    assert core.context.robot_pose_map is pose
    assert core.context.robot_pose_map_xytheta[:2] == [1.0, 2.0]
    assert core.context.robot_x == 1.0
    assert core.context.robot_y == 2.0
    assert abs(core.context.robot_theta - math.pi / 2.0) < 1e-9
    assert core.context.robot_pose_frame == 'map'
    assert core.context.map_ready is True
    assert core.context.localization_ready is True


def test_runtime_core_keeps_odom_pose_and_localization_status(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(node=SimpleNamespace())
    pose = _pose('odom', 3.0, 4.0, -math.pi / 2.0)

    core.update_robot_pose_odom(pose)
    core.update_localization_status('localized')

    assert core.context.robot_pose_odom is pose
    assert core.context.robot_pose_odom_xytheta[:2] == [3.0, 4.0]
    assert core.context.odom_x == 3.0
    assert core.context.odom_y == 4.0
    assert abs(core.context.odom_theta + math.pi / 2.0) < 1e-9
    assert core.context.odom_pose_frame == 'odom'
    assert core.context.odom_ready is True
    assert core.context.localization_status == 'localized'
    assert core.context.map_ready is True
    assert core.context.localization_ready is True


def test_runtime_core_keeps_odom_only_separate_from_map_ready(monkeypatch):
    monkeypatch.setattr('robot_runtime.runtime_core.RobotBody', FakeRobotBody)
    core = RuntimeCore(node=SimpleNamespace())
    pose = _pose('odom', 3.0, 4.0, -math.pi / 2.0)

    core.update_robot_pose_odom(pose)
    core.update_localization_status('odom_only')

    assert core.context.odom_ready is True
    assert core.context.map_ready is False
    assert core.context.localization_ready is False
    assert core.context.localization_status == 'odom_only'


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
