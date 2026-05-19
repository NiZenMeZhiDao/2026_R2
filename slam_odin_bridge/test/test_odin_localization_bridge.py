from types import SimpleNamespace
import math

from slam_odin_bridge.pose_math import relative_pose_from_reference


def test_relative_pose_zeroes_initial_odin_center_pose():
    initial_pose = _pose('odom', 0.0, 0.0, math.pi)
    moved_pose = _pose('odom', 1.0, 0.0, math.pi)

    zeroed_initial = relative_pose_from_reference(initial_pose, initial_pose)
    zeroed_moved = relative_pose_from_reference(moved_pose, initial_pose)

    assert abs(zeroed_initial.pose.position.x) < 1e-9
    assert abs(zeroed_initial.pose.position.y) < 1e-9
    assert abs(_yaw(zeroed_initial)) < 1e-9
    assert abs(zeroed_moved.pose.position.x + 1.0) < 1e-9
    assert abs(zeroed_moved.pose.position.y) < 1e-9
    assert abs(_yaw(zeroed_moved)) < 1e-9


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


def _yaw(pose):
    orientation = pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
