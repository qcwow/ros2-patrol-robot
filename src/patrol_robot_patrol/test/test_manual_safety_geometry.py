import math
import unittest

from patrol_robot_patrol.manual_safety_geometry import evaluate_scan_safety


def scan_with_point(angle, distance, count=360):
    ranges = [math.inf] * count
    angle_min = -math.pi
    increment = 2.0 * math.pi / count
    index = round((angle - angle_min) / increment) % count
    ranges[index] = distance
    return ranges, angle_min, increment


class ManualSafetyGeometryTest(unittest.TestCase):

    def evaluate(self, angle, distance, vx=0.0, vy=0.0, wz=0.0):
        ranges, angle_min, increment = scan_with_point(angle, distance)
        return evaluate_scan_safety(
            ranges, angle_min, increment, vx, vy, wz,
            maximum_valid_range=12.0)

    def test_clear_scan_keeps_command(self):
        decision = evaluate_scan_safety(
            [math.inf] * 360, -math.pi, 2.0 * math.pi / 360,
            0.12, 0.0, 0.3)
        self.assertEqual(1.0, decision.linear_scale)
        self.assertTrue(decision.rotation_allowed)

    def test_front_wall_stops_forward_motion(self):
        self.assertEqual(0.0, self.evaluate(0.0, 0.27, vx=0.12).linear_scale)

    def test_rear_wall_stops_reverse_motion(self):
        self.assertEqual(
            0.0, self.evaluate(math.pi, 0.27, vx=-0.12).linear_scale)

    def test_side_wall_stops_mecanum_translation(self):
        self.assertEqual(
            0.0,
            self.evaluate(math.pi / 2.0, 0.24, vy=0.12).linear_scale,
        )

    def test_point_outside_swept_corridor_is_ignored(self):
        angle = math.atan2(0.35, 0.25)
        decision = self.evaluate(
            angle, math.hypot(0.25, 0.35), vx=0.12)
        self.assertEqual(1.0, decision.linear_scale)

    def test_intermediate_clearance_slows_translation(self):
        decision = self.evaluate(0.0, 0.40, vx=0.12)
        self.assertGreater(decision.linear_scale, 0.0)
        self.assertLess(decision.linear_scale, 1.0)

    def test_close_obstacle_blocks_rotation(self):
        self.assertFalse(
            self.evaluate(math.pi / 3.0, 0.25, wz=0.3).rotation_allowed)

    def test_far_obstacle_allows_rotation(self):
        self.assertTrue(
            self.evaluate(math.pi / 3.0, 0.60, wz=0.3).rotation_allowed)

    def test_invalid_ranges_are_ignored(self):
        decision = evaluate_scan_safety(
            [math.nan, math.inf, 0.01], -0.1, 0.1, 0.12, 0.0, 0.0)
        self.assertEqual(1.0, decision.linear_scale)
