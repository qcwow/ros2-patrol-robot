"""ROS 2 rolling local voxel map for RGB-D navigation and visualisation."""

from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from patrol_robot_camera.rolling_voxel import RollingVoxelMap


def _rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError('TF quaternion has zero length')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float32)


def _decode_xyzrgb(message: PointCloud2) -> tuple[np.ndarray, np.ndarray]:
    fields = {field.name: field for field in message.fields}
    missing = sorted({'x', 'y', 'z'} - fields.keys())
    if missing:
        raise ValueError(f'point cloud is missing fields: {missing}')

    endian = '>' if message.is_bigendian else '<'
    formats = [endian + 'f4', endian + 'f4', endian + 'f4']
    offsets = [fields[name].offset for name in ('x', 'y', 'z')]
    names = ['x', 'y', 'z']
    if 'rgb' in fields:
        names.append('rgb')
        formats.append(endian + 'u4')
        offsets.append(fields['rgb'].offset)
    dtype = np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': int(message.point_step),
    })
    width = int(message.width)
    height = int(message.height)
    required = 0 if height == 0 else (height - 1) * int(message.row_step) \
        + width * int(message.point_step)
    if len(message.data) < required:
        raise ValueError('point cloud data is shorter than its metadata')
    records = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=message.data,
        strides=(int(message.row_step), int(message.point_step)),
    ).reshape(-1)
    points = np.column_stack(
        (records['x'], records['y'], records['z'])
    ).astype(np.float32, copy=False)
    if 'rgb' not in fields:
        return points, np.zeros((points.shape[0], 3), dtype=np.uint8)
    packed = records['rgb'].astype(np.uint32, copy=False)
    colors = np.column_stack((packed >> 16, packed >> 8, packed)).astype(
        np.uint8,
        copy=False,
    )
    return points, colors


class LocalVoxelMapper(Node):
    """Accumulate a bounded point map in the odometry frame."""

    def __init__(self) -> None:
        super().__init__('local_voxel_mapper')
        self.declare_parameter('input_topic', '/camera/points/filtered')
        self.declare_parameter('output_topic', '/local_grid/occupied')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('half_extent', 2.0)
        self.declare_parameter('max_voxels', 80000)
        self.declare_parameter('min_observations', 2)
        self.declare_parameter('stale_seconds', 12.0)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('transform_timeout', 0.08)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._target_frame = str(
            self.get_parameter('target_frame').value
        ).lstrip('/')
        self._transform_timeout = float(
            self.get_parameter('transform_timeout').value
        )
        publish_rate = float(self.get_parameter('publish_rate').value)
        if not self._target_frame:
            raise ValueError('target_frame must not be empty')
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        if self._transform_timeout < 0.0:
            raise ValueError('transform_timeout must not be negative')

        self._map = RollingVoxelMap(
            voxel_size=float(self.get_parameter('voxel_size').value),
            half_extent=float(self.get_parameter('half_extent').value),
            max_voxels=int(self.get_parameter('max_voxels').value),
            min_observations=int(
                self.get_parameter('min_observations').value),
            stale_seconds=float(self.get_parameter('stale_seconds').value),
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data)
        self._status_publisher = self.create_publisher(
            String, '/local_grid/status', 1)
        self.create_subscription(
            PointCloud2,
            input_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / publish_rate, self._publish)
        self.create_timer(1.0, self._publish_status)
        self.create_service(Empty, '/local_grid/reset', self._reset)

        self._last_update = 0.0
        self._last_error_log = 0.0
        self._last_source_frame = ''
        self._published_points = 0
        self._failed_transforms = 0
        self.get_logger().info(
            'Rolling local grid started: '
            f'{input_topic} -> {output_topic}, frame={self._target_frame}, '
            f'window={2.0 * self._map.half_extent:g} m cube, '
            f'voxel={self._map.voxel_size:g} m')

    def _reset(self, _request: Empty.Request, response: Empty.Response):
        self._map.clear()
        self._published_points = 0
        self._last_update = 0.0
        return response

    def _on_cloud(self, message: PointCloud2) -> None:
        source_frame = str(message.header.frame_id).lstrip('/')
        if not source_frame:
            self._report_input_error('point cloud frame_id is empty')
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self._transform_timeout),
            ).transform
            points, colors = _decode_xyzrgb(message)
            rotation = transform.rotation
            matrix = _rotation_matrix(
                rotation.x, rotation.y, rotation.z, rotation.w)
            translation = transform.translation
            origin = np.asarray(
                [translation.x, translation.y, translation.z],
                dtype=np.float32,
            )
            transformed = points @ matrix.T + origin
            stamp = float(message.header.stamp.sec) \
                + float(message.header.stamp.nanosec) * 1e-9
            self._map.insert(transformed, colors, origin, stamp)
            self._last_source_frame = source_frame
            self._last_update = time.monotonic()
        except (TransformException, TypeError, ValueError) as error:
            self._failed_transforms += 1
            self._report_input_error(str(error))

    def _report_input_error(self, detail: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log >= 2.0:
            self.get_logger().warning(f'Local grid skipped a frame: {detail}')
            self._last_error_log = now

    def _publish(self) -> None:
        points, colors = self._map.snapshot()
        count = int(points.shape[0])
        packed_colors = (
            (colors[:, 0].astype(np.uint32) << 16)
            | (colors[:, 1].astype(np.uint32) << 8)
            | colors[:, 2].astype(np.uint32)
        )
        records = np.empty(count, dtype=np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4'),
        ]))
        records['x'], records['y'], records['z'] = (
            points[:, 0], points[:, 1], points[:, 2])
        records['rgb'] = packed_colors

        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._target_frame
        message.height = 1
        message.width = count
        message.fields = [
            PointField(
                name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 16
        message.row_step = 16 * count
        message.data = records.tobytes()
        message.is_dense = True
        self._publisher.publish(message)
        self._published_points = count

    def _publish_status(self) -> None:
        age = None if self._last_update == 0.0 else \
            time.monotonic() - self._last_update
        message = String()
        message.data = json.dumps({
            'ready': age is not None and age < 2.0,
            'target_frame': self._target_frame,
            'source_frame': self._last_source_frame,
            'points': self._published_points,
            'candidate_voxels': self._map.candidate_voxel_count,
            'voxel_size': self._map.voxel_size,
            'half_extent': self._map.half_extent,
            'age_seconds': None if age is None else round(age, 3),
            'failed_transforms': self._failed_transforms,
        }, ensure_ascii=False)
        self._status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalVoxelMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
