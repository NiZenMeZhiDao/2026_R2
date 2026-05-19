from copy import deepcopy
import math


def relative_pose_from_reference(current_pose, reference_pose):
    """Express the current pose in the reference pose coordinate frame."""
    relative_pose = deepcopy(current_pose)

    current_orientation = current_pose.pose.orientation
    reference_orientation = reference_pose.pose.orientation
    current_q = (
        current_orientation.x,
        current_orientation.y,
        current_orientation.z,
        current_orientation.w,
    )
    reference_q = (
        reference_orientation.x,
        reference_orientation.y,
        reference_orientation.z,
        reference_orientation.w,
    )
    reference_inv_q = quat_conjugate(reference_q)

    dx = float(current_pose.pose.position.x) - float(reference_pose.pose.position.x)
    dy = float(current_pose.pose.position.y) - float(reference_pose.pose.position.y)
    dz = float(current_pose.pose.position.z) - float(reference_pose.pose.position.z)
    relative_position = rotate_vector((dx, dy, dz), reference_inv_q)
    relative_orientation = quat_normalize(quat_multiply(reference_inv_q, current_q))

    relative_pose.pose.position.x = relative_position[0]
    relative_pose.pose.position.y = relative_position[1]
    relative_pose.pose.position.z = relative_position[2]
    relative_pose.pose.orientation.x = relative_orientation[0]
    relative_pose.pose.orientation.y = relative_orientation[1]
    relative_pose.pose.orientation.z = relative_orientation[2]
    relative_pose.pose.orientation.w = relative_orientation[3]
    return relative_pose


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
