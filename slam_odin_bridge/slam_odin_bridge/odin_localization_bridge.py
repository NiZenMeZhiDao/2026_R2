from copy import deepcopy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu
from slam_odin_bridge.pose_math import correct_mount_pose, relative_pose_from_reference
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class OdinLocalizationBridge(Node):
    """Adapt Odin localization outputs to the runtime's stable topic contract."""

    def __init__(self):
        super().__init__('odin_localization_bridge')

        self.declare_parameter('odometry_topic', '/odin1/odometry')
        self.declare_parameter('imu_topic', '/odin1/imu')
        self.declare_parameter('output_pose_map_topic', '/robot_pose')
        self.declare_parameter('output_pose_odom_topic', '/robot_pose_odom')
        self.declare_parameter('output_imu_topic', '/imu/data')
        self.declare_parameter('localization_status_topic', '/localization/status')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'odin1_base_link')
        self.declare_parameter('publish_odom_pose', True)
        self.declare_parameter('republish_imu', True)
        self.declare_parameter('require_map_transform', True)
        self.declare_parameter('reverse_mount_xy', False)
        self.declare_parameter('reverse_mount_yaw', False)
        self.declare_parameter('zero_pose_on_start', True)
        self.declare_parameter('mount_base_to_odin_x', 0.0)
        self.declare_parameter('mount_base_to_odin_y', 0.0)
        self.declare_parameter('mount_base_to_odin_z', 0.0)
        self.declare_parameter('mount_base_to_odin_yaw', 0.0)
        self.declare_parameter('runtime_frame_yaw', 0.0)
        self.declare_parameter('tf_lookup_timeout_sec', 0.05)
        self.declare_parameter('status_period_sec', 0.5)

        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.output_pose_map_topic = self.get_parameter('output_pose_map_topic').value
        self.output_pose_odom_topic = self.get_parameter('output_pose_odom_topic').value
        self.output_imu_topic = self.get_parameter('output_imu_topic').value
        self.localization_status_topic = self.get_parameter('localization_status_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_odom_pose = bool(self.get_parameter('publish_odom_pose').value)
        self.republish_imu = bool(self.get_parameter('republish_imu').value)
        self.require_map_transform = bool(self.get_parameter('require_map_transform').value)
        self.reverse_mount_xy = bool(self.get_parameter('reverse_mount_xy').value)
        self.reverse_mount_yaw = bool(self.get_parameter('reverse_mount_yaw').value)
        self.zero_pose_on_start = bool(self.get_parameter('zero_pose_on_start').value)
        self.mount_base_to_odin_x = float(self.get_parameter('mount_base_to_odin_x').value)
        self.mount_base_to_odin_y = float(self.get_parameter('mount_base_to_odin_y').value)
        self.mount_base_to_odin_z = float(self.get_parameter('mount_base_to_odin_z').value)
        self.mount_base_to_odin_yaw = float(
            self.get_parameter('mount_base_to_odin_yaw').value
        )
        self.runtime_frame_yaw = float(self.get_parameter('runtime_frame_yaw').value)
        self.tf_lookup_timeout = Duration(
            seconds=float(self.get_parameter('tf_lookup_timeout_sec').value)
        )
        status_period = float(self.get_parameter('status_period_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_pose_pub = self.create_publisher(PoseStamped, self.output_pose_map_topic, 10)
        self.odom_pose_pub = self.create_publisher(PoseStamped, self.output_pose_odom_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.output_imu_topic, 10)
        self.status_pub = self.create_publisher(String, self.localization_status_topic, 10)

        self.create_subscription(Odometry, self.odometry_topic, self._odometry_cb, 10)
        if self.republish_imu:
            self.create_subscription(Imu, self.imu_topic, self._imu_cb, 10)

        self._status = 'waiting_for_odom'
        self._last_logged_status = None
        self._map_reference_pose = None
        self._odom_reference_pose = None
        self.create_timer(status_period, self._publish_status)

        self.get_logger().info(
            'Odin localization bridge ready: '
            f'{self.odometry_topic} -> {self.output_pose_map_topic}'
        )

    def _odometry_cb(self, msg):
        odom_pose = PoseStamped()
        odom_pose.header = deepcopy(msg.header)
        odom_pose.pose = deepcopy(msg.pose.pose)
        if not odom_pose.header.frame_id:
            odom_pose.header.frame_id = self.odom_frame

        if self.publish_odom_pose:
            self.odom_pose_pub.publish(self._runtime_pose(odom_pose, 'odom'))

        try:
            if odom_pose.header.frame_id == self.map_frame:
                map_pose = odom_pose
            else:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    odom_pose.header.frame_id,
                    odom_pose.header.stamp,
                    timeout=self.tf_lookup_timeout,
                )
                map_pose = _transform_pose(odom_pose, transform)

            map_pose.header.frame_id = self.map_frame
            self.map_pose_pub.publish(self._runtime_pose(map_pose, 'map'))
            self._set_status('localized')
        except TransformException as exc:
            if self.require_map_transform:
                self._set_status('odom_only')
            else:
                self._set_status('waiting_for_map')
            self.get_logger().debug(
                f'Cannot transform {odom_pose.header.frame_id} -> {self.map_frame}: {exc}'
            )

    def _runtime_pose(self, sensor_pose, reference_key):
        base_pose = correct_mount_pose(
            sensor_pose,
            reverse_xy=self.reverse_mount_xy,
            reverse_yaw=self.reverse_mount_yaw,
            base_to_sensor_x=self.mount_base_to_odin_x,
            base_to_sensor_y=self.mount_base_to_odin_y,
            base_to_sensor_z=self.mount_base_to_odin_z,
            base_to_sensor_yaw=self.mount_base_to_odin_yaw,
            output_frame_yaw=self.runtime_frame_yaw,
        )
        if not self.zero_pose_on_start:
            return base_pose

        reference_attr = '_%s_reference_pose' % reference_key
        reference_pose = getattr(self, reference_attr)
        if reference_pose is None:
            reference_pose = deepcopy(base_pose)
            setattr(self, reference_attr, reference_pose)

        runtime_pose = relative_pose_from_reference(base_pose, reference_pose)
        runtime_pose.header.frame_id = base_pose.header.frame_id
        return runtime_pose

    def _imu_cb(self, msg):
        self.imu_pub.publish(msg)

    def _publish_status(self):
        status_msg = String()
        status_msg.data = self._status
        self.status_pub.publish(status_msg)
        if status_msg.data != self._last_logged_status:
            self.get_logger().info(f'Localization status: {status_msg.data}')
            self._last_logged_status = status_msg.data

    def _set_status(self, status):
        self._status = str(status)


def _transform_pose(pose_stamped, transform_stamped):
    transformed = PoseStamped()
    transformed.header = deepcopy(transform_stamped.header)

    translation = transform_stamped.transform.translation
    rotation = transform_stamped.transform.rotation
    point = pose_stamped.pose.position
    orientation = pose_stamped.pose.orientation

    rotated_point = _rotate_vector(
        (point.x, point.y, point.z),
        (rotation.x, rotation.y, rotation.z, rotation.w),
    )
    transformed.pose.position.x = rotated_point[0] + translation.x
    transformed.pose.position.y = rotated_point[1] + translation.y
    transformed.pose.position.z = rotated_point[2] + translation.z

    q = _quat_multiply(
        (rotation.x, rotation.y, rotation.z, rotation.w),
        (orientation.x, orientation.y, orientation.z, orientation.w),
    )
    q = _quat_normalize(q)
    transformed.pose.orientation.x = q[0]
    transformed.pose.orientation.y = q[1]
    transformed.pose.orientation.z = q[2]
    transformed.pose.orientation.w = q[3]
    return transformed


def _rotate_vector(vector, quaternion):
    q_vector = (vector[0], vector[1], vector[2], 0.0)
    q_conjugate = _quat_conjugate(quaternion)
    rotated = _quat_multiply(_quat_multiply(quaternion, q_vector), q_conjugate)
    return rotated[:3]


def _quat_conjugate(quaternion):
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def _quat_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quat_normalize(quaternion):
    length = math.sqrt(sum(component * component for component in quaternion))
    if length == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / length for component in quaternion)


def main(args=None):
    rclpy.init(args=args)
    node = OdinLocalizationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
