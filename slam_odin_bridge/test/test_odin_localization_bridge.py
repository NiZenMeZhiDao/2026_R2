from types import SimpleNamespace
import math

from slam_odin_bridge.pose_math import correct_mount_pose


def test_correct_mount_pose_reverses_xy_and_adds_pi_to_yaw():
    pose = _pose('map', 1.5, -2.0, math.pi / 2.0)

    corrected = correct_mount_pose(pose, reverse_xy=True, reverse_yaw=True)

    assert corrected is not pose
    assert corrected.pose.position.x == -1.5
    assert corrected.pose.position.y == 2.0
    assert abs(_yaw(corrected) + math.pi / 2.0) < 1e-9


def test_correct_mount_pose_can_leave_pose_unchanged():
    pose = _pose('odom', 1.5, -2.0, 0.25)

    corrected = correct_mount_pose(pose)

    assert corrected.pose.position.x == 1.5
    assert corrected.pose.position.y == -2.0
    assert abs(_yaw(corrected) - 0.25) < 1e-9


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
