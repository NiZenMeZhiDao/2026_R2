import math
import os
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class PcdMapPublisher(Node):
    """Publish a static PCD file as a latched PointCloud2 map."""

    def __init__(self):
        super().__init__('pcd_map_publisher')

        self.declare_parameter('pcd_path', '')
        self.declare_parameter('map_topic', '/odin1/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_period_sec', 2.0)

        self.pcd_path = self.get_parameter('pcd_path').value
        self.map_topic = self.get_parameter('map_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        publish_period = float(self.get_parameter('publish_period_sec').value)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(PointCloud2, self.map_topic, qos)

        self.map_msg = self._load_map()
        self.create_timer(publish_period, self._publish_map)
        self._publish_map()

    def _load_map(self):
        if not self.pcd_path:
            self.get_logger().warning('pcd_path is empty, publishing an empty map')
            return self._build_pointcloud2([])
        if not os.path.exists(self.pcd_path):
            self.get_logger().warning(
                f'PCD file does not exist: {self.pcd_path}, publishing an empty map'
            )
            return self._build_pointcloud2([])

        header, data_offset = _read_pcd_header(self.pcd_path)
        points = _read_pcd_points(self.pcd_path, header, data_offset)

        msg = self._build_pointcloud2(points)
        self.get_logger().info(
            f'Loaded {msg.width} map points from {self.pcd_path}; publishing on {self.map_topic}'
        )
        return msg

    def _build_pointcloud2(self, points):
        msg = PointCloud2()
        msg.header.frame_id = self.map_frame
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.is_dense = all(math.isfinite(value) for point in points for value in point[:3])
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = b''.join(struct.pack('<ffff', *point) for point in points)
        return msg

    def _publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)


def _read_pcd_header(path):
    header = {}
    with open(path, 'rb') as pcd_file:
        while True:
            line = pcd_file.readline()
            if not line:
                raise RuntimeError('Invalid PCD file: missing DATA line')
            text = line.decode('utf-8', errors='replace').strip()
            if not text or text.startswith('#'):
                continue
            key, *values = text.split()
            key = key.upper()
            header[key] = values
            if key == 'DATA':
                return header, pcd_file.tell()


def _read_pcd_points(path, header, data_offset):
    fields = header.get('FIELDS', [])
    sizes = [int(value) for value in header.get('SIZE', [])]
    types = header.get('TYPE', [])
    counts = [int(value) for value in header.get('COUNT', ['1'] * len(fields))]
    data_format = header.get('DATA', [''])[0].lower()
    point_count = int(header.get('POINTS', header.get('WIDTH', ['0']))[0])

    if not fields or not sizes or not types:
        raise RuntimeError('Invalid PCD file: FIELDS, SIZE and TYPE are required')
    if any(count != 1 for count in counts):
        raise RuntimeError('PCD fields with COUNT other than 1 are not supported')

    field_offsets = {}
    offset = 0
    for field, size in zip(fields, sizes):
        field_offsets[field] = offset
        offset += size
    point_step = offset

    if not {'x', 'y', 'z'}.issubset(field_offsets):
        raise RuntimeError('PCD file must contain x, y and z fields')

    if data_format == 'ascii':
        return _read_ascii_points(path, data_offset, fields, point_count)
    if data_format == 'binary':
        return _read_binary_points(
            path,
            data_offset,
            point_count,
            point_step,
            fields,
            sizes,
            types,
            field_offsets,
        )
    raise RuntimeError(f'Unsupported PCD DATA format: {data_format}')


def _read_ascii_points(path, data_offset, fields, point_count):
    index = {field: fields.index(field) for field in fields}
    points = []
    with open(path, 'rb') as pcd_file:
        pcd_file.seek(data_offset)
        for raw_line in pcd_file:
            if len(points) >= point_count:
                break
            values = raw_line.decode('utf-8', errors='replace').split()
            if not values:
                continue
            intensity = float(values[index['intensity']]) if 'intensity' in index else 0.0
            points.append((
                float(values[index['x']]),
                float(values[index['y']]),
                float(values[index['z']]),
                intensity,
            ))
    return points


def _read_binary_points(path, data_offset, point_count, point_step, fields, sizes, types, offsets):
    points = []
    with open(path, 'rb') as pcd_file:
        pcd_file.seek(data_offset)
        data = pcd_file.read(point_count * point_step)

    for point_index in range(point_count):
        base = point_index * point_step
        points.append((
            _unpack_field(data, base + offsets['x'], sizes[fields.index('x')], types[fields.index('x')]),
            _unpack_field(data, base + offsets['y'], sizes[fields.index('y')], types[fields.index('y')]),
            _unpack_field(data, base + offsets['z'], sizes[fields.index('z')], types[fields.index('z')]),
            _unpack_optional_field(data, base, fields, sizes, types, offsets, 'intensity'),
        ))
    return points


def _unpack_optional_field(data, base, fields, sizes, types, offsets, name):
    if name not in offsets:
        return 0.0
    field_index = fields.index(name)
    return _unpack_field(data, base + offsets[name], sizes[field_index], types[field_index])


def _unpack_field(data, offset, size, field_type):
    if field_type == 'F' and size == 4:
        return struct.unpack_from('<f', data, offset)[0]
    if field_type == 'F' and size == 8:
        return float(struct.unpack_from('<d', data, offset)[0])
    if field_type == 'U' and size == 1:
        return float(struct.unpack_from('<B', data, offset)[0])
    if field_type == 'U' and size == 2:
        return float(struct.unpack_from('<H', data, offset)[0])
    if field_type == 'U' and size == 4:
        return float(struct.unpack_from('<I', data, offset)[0])
    if field_type == 'I' and size == 1:
        return float(struct.unpack_from('<b', data, offset)[0])
    if field_type == 'I' and size == 2:
        return float(struct.unpack_from('<h', data, offset)[0])
    if field_type == 'I' and size == 4:
        return float(struct.unpack_from('<i', data, offset)[0])
    raise RuntimeError(f'Unsupported PCD field type/size: {field_type}{size}')


def main(args=None):
    rclpy.init(args=args)
    node = PcdMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
