"""ROS 2 node that converts synchronized RGB-D frames into a point cloud."""

from __future__ import annotations

import time

import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from patrol_robot_camera.geometry import (
    decode_color_image,
    decode_depth_image,
    project_rgbd,
)


class RgbdProcessor(Node):
    """Synchronize registered RGB-D frames and publish a filtered XYZRGB cloud."""

    def __init__(self) -> None:
        super().__init__('rgbd_processor')

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('point_cloud_topic', '/camera/points/filtered')
        self.declare_parameter(
            'mapping_point_cloud_topic',
            '/camera/points/mapping',
        )
        self.declare_parameter('mapping_publish_rate', 3.0)
        self.declare_parameter('point_cloud_publish_rate', 0.0)
        self.declare_parameter('use_color', True)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_depth', 0.25)
        self.declare_parameter('max_depth', 8.0)
        self.declare_parameter('sampling_stride', 2)
        self.declare_parameter('voxel_size', 0.04)
        self.declare_parameter('max_points', 120000)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop_seconds', 0.08)

        self._use_color = bool(self.get_parameter('use_color').value)
        self._depth_scale = float(self.get_parameter('depth_scale').value)
        self._min_depth = float(self.get_parameter('min_depth').value)
        self._max_depth = float(self.get_parameter('max_depth').value)
        self._sampling_stride = int(
            self.get_parameter('sampling_stride').value
        )
        self._voxel_size = float(self.get_parameter('voxel_size').value)
        self._max_points = int(self.get_parameter('max_points').value)
        self._mapping_publish_rate = float(
            self.get_parameter('mapping_publish_rate').value
        )
        self._point_cloud_publish_rate = float(
            self.get_parameter('point_cloud_publish_rate').value
        )
        queue_size = int(self.get_parameter('sync_queue_size').value)
        sync_slop = float(self.get_parameter('sync_slop_seconds').value)

        self._validate_parameters(queue_size, sync_slop)

        point_cloud_topic = str(
            self.get_parameter('point_cloud_topic').value
        )
        self._publisher = self.create_publisher(
            PointCloud2,
            point_cloud_topic,
            qos_profile_sensor_data,
        )
        mapping_point_cloud_topic = str(
            self.get_parameter('mapping_point_cloud_topic').value
        )
        self._mapping_publisher = (
            self.create_publisher(
                PointCloud2,
                mapping_point_cloud_topic,
                qos_profile_sensor_data,
            )
            if mapping_point_cloud_topic and self._mapping_publish_rate > 0.0
            else None
        )
        self._mapping_publish_period = (
            1.0 / self._mapping_publish_rate
            if self._mapping_publisher is not None
            else 0.0
        )
        self._last_mapping_publish = float('-inf')
        self._last_point_cloud_publish = float('-inf')
        mapping_status = (
            f'{mapping_point_cloud_topic}@{self._mapping_publish_rate:g}Hz'
            if self._mapping_publisher is not None
            else '(关闭)'
        )

        depth_topic = str(self.get_parameter('depth_topic').value)
        camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self._depth_subscriber = message_filters.Subscriber(
            self,
            Image,
            depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self._info_subscriber = message_filters.Subscriber(
            self,
            CameraInfo,
            camera_info_topic,
            qos_profile=qos_profile_sensor_data,
        )

        filters = [self._depth_subscriber, self._info_subscriber]
        if self._use_color:
            color_topic = str(self.get_parameter('color_topic').value)
            self._color_subscriber = message_filters.Subscriber(
                self,
                Image,
                color_topic,
                qos_profile=qos_profile_sensor_data,
            )
            filters.insert(0, self._color_subscriber)
        else:
            color_topic = '(关闭)'
            self._color_subscriber = None

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            filters,
            queue_size=queue_size,
            slop=sync_slop,
        )
        if self._use_color:
            self._synchronizer.registerCallback(self._on_rgbd)
        else:
            self._synchronizer.registerCallback(self._on_depth)

        self._processed_frames = 0
        self._failed_frames = 0
        self._last_error_log = 0.0
        self._last_status_log = time.monotonic()

        self.get_logger().info(
            'RGB-D 处理器已启动：'
            f'彩色={color_topic}，深度={depth_topic}，内参={camera_info_topic}，'
            f'避障点云={point_cloud_topic}，'
            f'建图点云={mapping_status}'
        )

    def _validate_parameters(self, queue_size: int, sync_slop: float) -> None:
        if self._depth_scale <= 0.0:
            raise ValueError('depth_scale 必须大于 0')
        if self._min_depth < 0.0 or self._max_depth <= self._min_depth:
            raise ValueError('深度范围必须满足 0 <= min_depth < max_depth')
        if self._sampling_stride < 1:
            raise ValueError('sampling_stride 必须大于等于 1')
        if self._voxel_size < 0.0:
            raise ValueError('voxel_size 不能小于 0')
        if self._max_points < 1:
            raise ValueError('max_points 必须大于等于 1')
        if self._mapping_publish_rate < 0.0:
            raise ValueError('mapping_publish_rate 不能小于 0')
        if self._point_cloud_publish_rate < 0.0:
            raise ValueError('point_cloud_publish_rate 不能小于 0')
        if queue_size < 2:
            raise ValueError('sync_queue_size 必须大于等于 2')
        if sync_slop < 0.0:
            raise ValueError('sync_slop_seconds 不能小于 0')

    def _on_rgbd(
        self,
        color_message: Image,
        depth_message: Image,
        camera_info: CameraInfo,
    ) -> None:
        self._process(depth_message, camera_info, color_message)

    def _on_depth(
        self,
        depth_message: Image,
        camera_info: CameraInfo,
    ) -> None:
        self._process(depth_message, camera_info, None)

    def _process(
        self,
        depth_message: Image,
        camera_info: CameraInfo,
        color_message: Image | None,
    ) -> None:
        started_at = time.monotonic()
        if (
            self._point_cloud_publish_rate > 0.0
            and started_at - self._last_point_cloud_publish
            < 1.0 / self._point_cloud_publish_rate
        ):
            return
        try:
            depth = decode_depth_image(depth_message, self._depth_scale)
            color = (
                decode_color_image(color_message)
                if color_message is not None
                else None
            )
            result = project_rgbd(
                depth,
                color,
                camera_info.k,
                min_depth=self._min_depth,
                max_depth=self._max_depth,
                sampling_stride=self._sampling_stride,
                voxel_size=self._voxel_size,
                max_points=self._max_points,
            )
            cloud = self._make_cloud(depth_message, result.points, result.colors)
            self._publisher.publish(cloud)
            now = time.monotonic()
            self._last_point_cloud_publish = now
            if (
                self._mapping_publisher is not None
                and now - self._last_mapping_publish
                >= self._mapping_publish_period
            ):
                self._mapping_publisher.publish(cloud)
                self._last_mapping_publish = now
            self._processed_frames += 1
            self._log_status_if_due(result.points.shape[0])
        except (TypeError, ValueError) as error:
            self._failed_frames += 1
            now = time.monotonic()
            if now - self._last_error_log >= 2.0:
                self.get_logger().error(f'跳过无效 RGB-D 帧：{error}')
                self._last_error_log = now

    @staticmethod
    def _make_cloud(
        source_image: Image,
        points: np.ndarray,
        colors: np.ndarray,
    ) -> PointCloud2:
        count = int(points.shape[0])
        packed_colors = (
            (colors[:, 0].astype(np.uint32) << 16)
            | (colors[:, 1].astype(np.uint32) << 8)
            | colors[:, 2].astype(np.uint32)
        )
        records = np.empty(
            count,
            dtype=np.dtype([
                ('x', '<f4'),
                ('y', '<f4'),
                ('z', '<f4'),
                ('rgb', '<u4'),
            ]),
        )
        records['x'] = points[:, 0]
        records['y'] = points[:, 1]
        records['z'] = points[:, 2]
        records['rgb'] = packed_colors

        cloud = PointCloud2()
        cloud.header = source_image.header
        cloud.height = 1
        cloud.width = count
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            # PCL / RViz convention: RGB bytes packed into a FLOAT32 field.
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * count
        cloud.data = records.tobytes()
        cloud.is_dense = True
        return cloud

    def _log_status_if_due(self, points: int) -> None:
        now = time.monotonic()
        if now - self._last_status_log < 10.0:
            return
        self.get_logger().info(
            f'相机处理正常：累计 {self._processed_frames} 帧，'
            f'当前点云 {points} 点，丢弃 {self._failed_frames} 帧'
        )
        self._last_status_log = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RgbdProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
