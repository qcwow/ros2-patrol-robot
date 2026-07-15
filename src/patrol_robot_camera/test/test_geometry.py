import unittest
from types import SimpleNamespace

import numpy as np

from patrol_robot_camera.geometry import (
    decode_color_image,
    decode_depth_image,
    project_rgbd,
)


def image_message(array, encoding, step=None, is_bigendian=False):
    height, width = array.shape[:2]
    return SimpleNamespace(
        height=height,
        width=width,
        encoding=encoding,
        is_bigendian=is_bigendian,
        step=step if step is not None else array.strides[0],
        data=array.tobytes(),
    )


class GeometryTest(unittest.TestCase):
    def test_decodes_uint16_depth_in_metres(self):
        raw = np.array([[1000, 2500], [0, 65535]], dtype='<u2')
        depth = decode_depth_image(image_message(raw, '16UC1'))

        np.testing.assert_allclose(depth, [[1.0, 2.5], [0.0, 65.535]])

    def test_decodes_bgr_as_rgb(self):
        bgr = np.array([[[3, 2, 1], [30, 20, 10]]], dtype=np.uint8)
        rgb = decode_color_image(image_message(bgr, 'bgr8'))

        np.testing.assert_array_equal(rgb, [[[1, 2, 3], [10, 20, 30]]])

    def test_decodes_float_depth_with_row_padding(self):
        first_row = np.array([1.0, 2.0], dtype='<f4').tobytes() + b'pad!'
        second_row = np.array([3.0, 4.0], dtype='<f4').tobytes() + b'pad!'
        message = SimpleNamespace(
            height=2,
            width=2,
            encoding='32FC1',
            is_bigendian=False,
            step=12,
            data=first_row + second_row,
        )

        depth = decode_depth_image(message)

        np.testing.assert_array_equal(depth, [[1.0, 2.0], [3.0, 4.0]])

    def test_projects_registered_rgbd_and_filters_invalid_depth(self):
        depth = np.array([[1.0, 2.0], [0.0, 1.0]], dtype=np.float32)
        color = np.array(
            [
                [[255, 0, 0], [0, 255, 0]],
                [[0, 0, 255], [1, 2, 3]],
            ],
            dtype=np.uint8,
        )
        result = project_rgbd(
            depth,
            color,
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            min_depth=0.1,
            max_depth=3.0,
        )

        np.testing.assert_allclose(
            result.points,
            [
                [0.0, 0.0, 1.0],
                [2.0, 0.0, 2.0],
                [1.0, 1.0, 1.0],
            ],
        )
        np.testing.assert_array_equal(
            result.colors,
            [[255, 0, 0], [0, 255, 0], [1, 2, 3]],
        )

    def test_voxel_filter_keeps_one_point_per_voxel(self):
        depth = np.ones((2, 2), dtype=np.float32)
        result = project_rgbd(
            depth,
            None,
            [100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 1.0],
            min_depth=0.1,
            max_depth=2.0,
            voxel_size=0.1,
        )

        self.assertEqual(result.points.shape, (1, 3))
        np.testing.assert_array_equal(result.colors, [[180, 180, 180]])

    def test_rejects_unregistered_color_dimensions(self):
        with self.assertRaisesRegex(ValueError, '对齐深度图'):
            project_rgbd(
                np.ones((2, 2), dtype=np.float32),
                np.ones((3, 2, 3), dtype=np.uint8),
                [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                min_depth=0.1,
                max_depth=2.0,
            )


if __name__ == '__main__':
    unittest.main()
