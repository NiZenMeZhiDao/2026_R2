from std_msgs.msg import Float32MultiArray

from robot_runtime.controllers.exclusive_controller import ExclusiveController


class ChassisController(ExclusiveController):
    """Bottom-layer chassis command publisher."""

    def __init__(self, node):
        super().__init__('chassis')
        self._node = node
        self._cmd_vel_pub = node.create_publisher(
            Float32MultiArray,
            't0x0101_cmdvel',
            10,
        )

    def stop(self, owner):
        self.set_velocity(0.0, 0.0, 0.0, owner)

    def set_velocity(self, vx, vy=0.0, wz=0.0, owner=None):
        self.require_owner(owner)
        msg = Float32MultiArray()
        msg.data = [float(vx), float(vy), float(wz)]
        self._cmd_vel_pub.publish(msg)
        return msg

    def set_twist(self, twist, owner=None):
        return self.set_velocity(
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
            owner,
        )

    def slow_forward(self, speed, owner=None):
        return self.set_velocity(speed, 0.0, 0.0, owner)
