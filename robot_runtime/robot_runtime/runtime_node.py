import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray, Int32, String

from robot_runtime.runtime_core import RuntimeCore


class RobotRuntimeNode(Node):
    """Thin ROS bridge for USB, sensor and navigation driver data."""

    def __init__(self):
        super().__init__('robot_runtime_node')
        self.core = RuntimeCore(self)
        self.context = self.core.context
        self.body = self.core.body

        self._create_external_driver_subscriptions()
        self.get_logger().info(
            'Robot runtime bridge ready. Control is by Python function calls.'
        )

    def _create_external_driver_subscriptions(self):
        self.create_subscription(
            Float32MultiArray,
            'sensor_distances',
            self._sensor_distances_cb,
            10,
        )
        self.create_subscription(Float32MultiArray, 'r0x0201', self._hw_status_cb, 10)
        self.create_subscription(Int32, 'current_state', self._suspension_state_cb, 10)
        self.create_subscription(String, 'suspension/status', self._suspension_status_cb, 10)
        self.create_subscription(PoseStamped, 'robot_pose', self._robot_pose_cb, 10)
        self.create_subscription(Imu, 'imu/data', self._imu_cb, 10)
        self.create_subscription(String, 'nav/status', self._nav_status_cb, 10)
        self.create_subscription(Int32, 'direction', self._direction_cb, 10)
        self.create_subscription(Bool, 'emergency_stop', self._emergency_stop_cb, 10)

    def _sensor_distances_cb(self, msg):
        self.core.update_distances(msg.data)

    def _hw_status_cb(self, msg):
        self.core.update_lower_machine(msg.data)

    def _suspension_state_cb(self, msg):
        self.core.update_suspension_state(msg.data)

    def _suspension_status_cb(self, msg):
        self.core.update_suspension_status(msg.data)

    def _robot_pose_cb(self, msg):
        self.core.update_robot_pose(msg)

    def _imu_cb(self, msg):
        self.core.update_imu(msg)

    def _nav_status_cb(self, msg):
        self.core.update_nav_status(msg.data)

    def _direction_cb(self, msg):
        self.core.update_direction(msg.data)

    def _emergency_stop_cb(self, msg):
        self.core.set_emergency_stop(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = RobotRuntimeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.core.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

