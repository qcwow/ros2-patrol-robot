"""Pure RGB-D image decoding and point projection helpers.

This module deliberately has no ROS imports so the geometry can be unit-tested
without a running ROS graph.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ProjectionResult:
    """Filtered 3D points and their RGB colours."""

    points: np.ndarray
    colors: np.ndarray


def _image_view(message, dtype: np.dtype, channels: int = 1) -> np.ndarray:
    """Return an image view while respecting row padding and byte order."""

    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    dtype = np.dtype(dtype)
    pixel_bytes = dtype.itemsize * channels

    if height <= 0 or width <= 0:
        raise ValueError('图像宽度和高度必须大于 0')
    if step < width * pixel_bytes:
        raise ValueError(
            f'图像 step={step} 小于每行所需字节数 {width * pixel_bytes}'
        )
    if len(message.data) < height * step:
        raise ValueError(
            f'图像数据不完整：需要 {height * step} 字节，'
            f'实际 {len(message.data)} 字节'
        )

    byte_order = '>' if bool(message.is_bigendian) else '<'
    ordered_dtype = dtype.newbyteorder(byte_order)
    if channels == 1:
        shape = (height, width)
        strides = (step, dtype.itemsize)
    else:
        shape = (height, width, channels)
        strides = (step, pixel_bytes, dtype.itemsize)

    return np.ndarray(
        shape=shape,
        dtype=ordered_dtype,
        buffer=message.data,
        strides=strides,
    )


def decode_depth_image(message, depth_scale: float = 0.001) -> np.ndarray:
    """Decode a ROS Image depth payload into metres as float32."""

    encoding = str(message.encoding).lower()
    if encoding in ('16uc1', 'mono16'):
        if depth_scale <= 0.0:
            raise ValueError('depth_scale 必须大于 0')
        depth = _image_view(message, np.dtype('u2')).astype(np.float32)
        depth *= np.float32(depth_scale)
        return depth
    if encoding == '32fc1':
        return _image_view(message, np.dtype('f4')).astype(np.float32)
    raise ValueError(
        f'不支持深度图编码 {message.encoding!r}；'
        '当前支持 16UC1、mono16 和 32FC1'
    )


def decode_color_image(message) -> np.ndarray:
    """Decode common ROS colour encodings into contiguous RGB8."""

    encoding = str(message.encoding).lower()
    if encoding == 'rgb8':
        rgb = _image_view(message, np.dtype('u1'), 3)
    elif encoding == 'bgr8':
        rgb = _image_view(message, np.dtype('u1'), 3)[..., ::-1]
    elif encoding in ('rgba8', 'bgra8'):
        rgba = _image_view(message, np.dtype('u1'), 4)
        rgb = rgba[..., :3]
        if encoding == 'bgra8':
            rgb = rgb[..., ::-1]
    elif encoding in ('mono8', '8uc1'):
        gray = _image_view(message, np.dtype('u1'))
        rgb = np.repeat(gray[..., None], 3, axis=2)
    else:
        raise ValueError(
            f'不支持彩色图编码 {message.encoding!r}；当前支持 '
            'rgb8、bgr8、rgba8、bgra8、mono8 和 8UC1'
        )
    return np.ascontiguousarray(rgb, dtype=np.uint8)


def project_rgbd(
    depth_m: np.ndarray,
    color_rgb: np.ndarray | None,
    camera_matrix: list[float] | tuple[float, ...] | np.ndarray,
    *,
    min_depth: float,
    max_depth: float,
    sampling_stride: int = 1,
    voxel_size: float = 0.0,
    max_points: int = 200_000,
) -> ProjectionResult:
    """Project a registered depth image to a filtered coloured point cloud."""

    if depth_m.ndim != 2:
        raise ValueError('深度图必须是二维单通道图像')
    if color_rgb is not None:
        if color_rgb.ndim != 3 or color_rgb.shape[2] != 3:
            raise ValueError('彩色图必须是 HxWx3 的 RGB 图像')
        if color_rgb.shape[:2] != depth_m.shape:
            raise ValueError(
                f'彩色图尺寸 {color_rgb.shape[:2]} 与深度图尺寸 '
                f'{depth_m.shape} 不一致；请使用驱动提供的对齐深度图'
            )
    if min_depth < 0.0 or max_depth <= min_depth:
        raise ValueError('深度范围必须满足 0 <= min_depth < max_depth')
    if sampling_stride < 1:
        raise ValueError('sampling_stride 必须大于等于 1')
    if voxel_size < 0.0:
        raise ValueError('voxel_size 不能小于 0')
    if max_points < 1:
        raise ValueError('max_points 必须大于等于 1')

    matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(-1)
    if matrix.size != 9:
        raise ValueError('相机内参 K 必须包含 9 个数值')
    fx, fy = float(matrix[0]), float(matrix[4])
    cx, cy = float(matrix[2]), float(matrix[5])
    if not np.isfinite([fx, fy, cx, cy]).all() or fx <= 0.0 or fy <= 0.0:
        raise ValueError('相机内参无效：fx、fy 必须是有限正数')

    sampled_depth = depth_m[::sampling_stride, ::sampling_stride]
    rows = np.arange(0, depth_m.shape[0], sampling_stride, dtype=np.float32)
    columns = np.arange(0, depth_m.shape[1], sampling_stride, dtype=np.float32)
    pixel_u, pixel_v = np.meshgrid(columns, rows)

    valid = (
        np.isfinite(sampled_depth)
        & (sampled_depth >= min_depth)
        & (sampled_depth <= max_depth)
    )
    z = sampled_depth[valid].astype(np.float32, copy=False)
    x = ((pixel_u[valid] - cx) * z / fx).astype(np.float32, copy=False)
    y = ((pixel_v[valid] - cy) * z / fy).astype(np.float32, copy=False)
    points = np.column_stack((x, y, z)).astype(np.float32, copy=False)

    if color_rgb is None:
        colors = np.full((points.shape[0], 3), 180, dtype=np.uint8)
    else:
        colors = color_rgb[::sampling_stride, ::sampling_stride][valid]
        colors = np.ascontiguousarray(colors, dtype=np.uint8)

    if voxel_size > 0.0 and points.shape[0] > 1:
        voxel_indices = np.floor(points / voxel_size).astype(np.int64)
        _, keep = np.unique(voxel_indices, axis=0, return_index=True)
        keep.sort()
        points = points[keep]
        colors = colors[keep]

    if points.shape[0] > max_points:
        keep = np.linspace(0, points.shape[0] - 1, max_points, dtype=np.int64)
        points = points[keep]
        colors = colors[keep]

    return ProjectionResult(
        points=np.ascontiguousarray(points, dtype=np.float32),
        colors=np.ascontiguousarray(colors, dtype=np.uint8),
    )
