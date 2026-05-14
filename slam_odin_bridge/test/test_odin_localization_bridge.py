from types import SimpleNamespace
import math

from slam_odin_bridge.pose_math import (
    correct_mount_pose,
    relative_pose_from_reference,
    sensor_pose_to_base_pose,
)


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


def test_sensor_pose_to_base_pose_removes_rotating_sensor_offset():
    sensor_pose = _pose('odom', 1.0, 0.5, math.pi / 2.0)

    base_pose = sensor_pose_to_base_pose(
        sensor_pose,
        base_to_sensor_x=0.2,
        base_to_sensor_y=0.0,
        base_to_sensor_yaw=0.0,
    )

    assert abs(base_pose.pose.position.x - 1.0) < 1e-9
    assert abs(base_pose.pose.position.y - 0.3) < 1e-9
    assert abs(_yaw(base_pose) - math.pi / 2.0) < 1e-9


def test_sensor_pose_to_base_pose_handles_backward_mount():
    sensor_pose = _pose('odom', 1.0, 0.0, math.pi)

    base_pose = sensor_pose_to_base_pose(
        sensor_pose,
        base_to_sensor_x=0.2,
        base_to_sensor_y=0.0,
        base_to_sensor_yaw=math.pi,
    )

    assert abs(base_pose.pose.position.x - 0.8) < 1e-9
    assert abs(base_pose.pose.position.y) < 1e-9
    assert abs(_yaw(base_pose)) < 1e-9


def test_relative_pose_zeroes_initial_backward_mount_base_pose():
    initial_sensor_pose = _pose('odom', 0.0, 0.0, math.pi)
    moved_sensor_pose = _pose('odom', 1.0, 0.0, math.pi)

    initial_base_pose = sensor_pose_to_base_pose(
        initial_sensor_pose,
        base_to_sensor_x=-0.35,
        base_to_sensor_yaw=math.pi,
    )
    moved_base_pose = sensor_pose_to_base_pose(
        moved_sensor_pose,
        base_to_sensor_x=-0.35,
        base_to_sensor_yaw=math.pi,
    )

    zeroed_initial = relative_pose_from_reference(initial_base_pose, initial_base_pose)
    zeroed_moved = relative_pose_from_reference(moved_base_pose, initial_base_pose)

    assert abs(zeroed_initial.pose.position.x) < 1e-9
    assert abs(zeroed_initial.pose.position.y) < 1e-9
    assert abs(_yaw(zeroed_initial)) < 1e-9
    assert abs(zeroed_moved.pose.position.x - 1.0) < 1e-9
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
