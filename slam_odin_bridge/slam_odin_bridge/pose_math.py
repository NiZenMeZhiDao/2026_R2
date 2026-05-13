from copy import deepcopy
import math


def correct_mount_pose(pose_stamped, reverse_xy=False, reverse_yaw=False):
    corrected = deepcopy(pose_stamped)
    if reverse_xy:
        corrected.pose.position.x = -float(corrected.pose.position.x)
        corrected.pose.position.y = -float(corrected.pose.position.y)

    if reverse_yaw:
        orientation = corrected.pose.orientation
        q = quat_multiply(
            (orientation.x, orientation.y, orientation.z, orientation.w),
            (0.0, 0.0, 1.0, 0.0),
        )
        q = quat_normalize(q)
        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]
    return corrected


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
