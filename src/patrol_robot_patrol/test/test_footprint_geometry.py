import math
import unittest

from patrol_robot_patrol.footprint_geometry import (
    quaternion_to_yaw,
    rectangle_overlaps_lethal_cell,
)


class FootprintGeometryTest(unittest.TestCase):
    def setUp(self):
        self.width = 40
        self.height = 40
        self.resolution = 0.05
        self.data = [0] * (self.width * self.height)

    def _set_cost(self, x: float, y: float, cost: int) -> None:
        column = math.floor(x / self.resolution)
        row = math.floor(y / self.resolution)
        self.data[row * self.width + column] = cost

    def _overlaps(self, yaw: float = 0.0) -> bool:
        return rectangle_overlaps_lethal_cell(
            self.data,
            grid_width=self.width,
            grid_height=self.height,
            resolution=self.resolution,
            origin_x=0.0,
            origin_y=0.0,
            robot_x=1.0,
            robot_y=1.0,
            robot_yaw=yaw,
            footprint_length=0.52,
            footprint_width=0.42,
            safety_margin=0.02,
        )

    def test_inflated_cost_99_is_not_a_physical_collision(self):
        self._set_cost(1.0, 1.0, 99)
        self.assertFalse(self._overlaps())

    def test_lethal_cost_inside_body_is_a_collision(self):
        self._set_cost(1.2, 1.0, 100)
        self.assertTrue(self._overlaps())

    def test_cell_inside_old_circle_but_outside_rectangle_is_clear(self):
        self._set_cost(1.0, 1.35, 100)
        self.assertFalse(self._overlaps())

    def test_rectangle_rotates_with_robot(self):
        self._set_cost(1.0, 1.25, 100)
        self.assertFalse(self._overlaps(yaw=0.0))
        self.assertTrue(self._overlaps(yaw=math.pi / 2.0))

    def test_quaternion_yaw(self):
        yaw = math.pi / 3.0
        self.assertAlmostEqual(
            quaternion_to_yaw(
                0.0,
                0.0,
                math.sin(yaw / 2.0),
                math.cos(yaw / 2.0),
            ),
            yaw,
        )

    def test_rectangular_body_near_map_edge_does_not_use_circular_bounds(self):
        self.assertFalse(rectangle_overlaps_lethal_cell(
            self.data,
            grid_width=self.width,
            grid_height=self.height,
            resolution=self.resolution,
            origin_x=0.0,
            origin_y=0.0,
            robot_x=0.30,
            robot_y=1.0,
            robot_yaw=0.0,
            footprint_length=0.52,
            footprint_width=0.42,
            safety_margin=0.02,
        ))


if __name__ == '__main__':
    unittest.main()
