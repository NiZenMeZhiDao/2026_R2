from geometry_msgs.msg import Twist

from robot_runtime.libraries.pid import AnglePidAxis, PidAxis, PidGains
from robot_runtime.pid_config import pid_gains


class ChassisPidTask:
    """Task-side PID calculator for relative chassis pose correction."""

    def __init__(self, x_gains=None, y_gains=None, yaw_gains=None):
        self._pid_x = PidAxis(x_gains or pid_gains('chassis_pose', 'x'))
        self._pid_y = PidAxis(y_gains or pid_gains('chassis_pose', 'y'))
        self._pid_yaw = AnglePidAxis(yaw_gains or pid_gains('chassis_pose', 'yaw'))

    def reset(self):
        self._pid_x.reset()
        self._pid_y.reset()
        self._pid_yaw.reset()

    def compute_cmd(self, pose_error, now=None):
        x_error, y_error, yaw_error = _read_relative_error(pose_error)
        cmd = Twist()
        cmd.linear.x = self._pid_x.update(x_error, now)
        cmd.linear.y = self._pid_y.update(y_error, now)
        cmd.angular.z = self._pid_yaw.update(yaw_error, now)
        return cmd


def _read_relative_error(pose_error):
    if hasattr(pose_error, 'linear') and hasattr(pose_error, 'angular'):
        return (
            float(pose_error.linear.x),
            float(pose_error.linear.y),
            float(pose_error.angular.z),
        )

    if hasattr(pose_error, 'x') and hasattr(pose_error, 'y'):
        yaw = getattr(pose_error, 'yaw', 0.0)
        return float(pose_error.x), float(pose_error.y), float(yaw)

    values = list(pose_error)
    if len(values) < 3:
        raise ValueError('relative pose error must contain x, y, yaw')
    return float(values[0]), float(values[1]), float(values[2])
