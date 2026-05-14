from copy import deepcopy
import math


def correct_mount_pose(
    pose_stamped,
    reverse_xy=False,
    reverse_yaw=False,
    base_to_sensor_x=0.0,
    base_to_sensor_y=0.0,
    base_to_sensor_z=0.0,
    base_to_sensor_yaw=0.0,
    output_frame_yaw=0.0,
):
    corrected = deepcopy(pose_stamped)
    if (
        base_to_sensor_x != 0.0 or
        base_to_sensor_y != 0.0 or
        base_to_sensor_z != 0.0 or
        base_to_sensor_yaw != 0.0
    ):
        corrected = sensor_pose_to_base_pose(
            corrected,
            base_to_sensor_x=base_to_sensor_x,
            base_to_sensor_y=base_to_sensor_y,
            base_to_sensor_z=base_to_sensor_z,
            base_to_sensor_yaw=base_to_sensor_yaw,
        )

    if reverse_xy:
        corrected.pose.position.x = -float(corrected.pose.position.x)
        corrected.pose.position.y = -float(corrected.pose.position.y)

    yaw_offset = float(output_frame_yaw)
    if reverse_yaw:
        yaw_offset += math.pi

    if yaw_offset != 0.0:
        orientation = corrected.pose.orientation
        q = quat_multiply(
            (orientation.x, orientation.y, orientation.z, orientation.w),
            yaw_to_quaternion(yaw_offset),
        )
        q = quat_normalize(q)
        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]
    return corrected


def sensor_pose_to_base_pose(
    pose_stamped,
    base_to_sensor_x=0.0,
    base_to_sensor_y=0.0,
    base_to_sensor_z=0.0,
    base_to_sensor_yaw=0.0,
):
    """Convert a mounted sensor pose into the robot-center pose."""
    base_pose = deepcopy(pose_stamped)
    sensor_orientation = pose_stamped.pose.orientation
    sensor_q = (
        sensor_orientation.x,
        sensor_orientation.y,
        sensor_orientation.z,
        sensor_orientation.w,
    )
    base_to_sensor_q = yaw_to_quaternion(base_to_sensor_yaw)
    sensor_to_base_q = quat_conjugate(base_to_sensor_q)
    base_q = quat_normalize(quat_multiply(sensor_q, sensor_to_base_q))

    offset_in_base = (
        float(base_to_sensor_x),
        float(base_to_sensor_y),
        float(base_to_sensor_z),
    )
    offset_in_world = rotate_vector(offset_in_base, base_q)

    base_pose.pose.position.x = float(pose_stamped.pose.position.x) - offset_in_world[0]
    base_pose.pose.position.y = float(pose_stamped.pose.position.y) - offset_in_world[1]
    base_pose.pose.position.z = float(pose_stamped.pose.position.z) - offset_in_world[2]
    base_pose.pose.orientation.x = base_q[0]
    base_pose.pose.orientation.y = base_q[1]
    base_pose.pose.orientation.z = base_q[2]
    base_pose.pose.orientation.w = base_q[3]
    return base_pose


def yaw_to_quaternion(yaw):
    half_yaw = float(yaw) / 2.0
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def rotate_vector(vector, quaternion):
    q_vector = (vector[0], vector[1], vector[2], 0.0)
    q_conjugate = quat_conjugate(quaternion)
    rotated = quat_multiply(quat_multiply(quaternion, q_vector), q_conjugate)
    return rotated[:3]


def quat_conjugate(quaternion):
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def quat_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quat_normalize(quaternion):
    length = math.sqrt(sum(component * component for component in quaternion))
    if length == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / length for component in quaternion)
