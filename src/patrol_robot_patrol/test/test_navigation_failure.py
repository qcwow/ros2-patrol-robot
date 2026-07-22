import unittest

from patrol_robot_patrol.navigation_failure import (
    is_route_failure,
    navigation_error_label,
)


class NavigationFailureTest(unittest.TestCase):
    def test_controller_path_failures_require_a_different_route(self):
        for code in (103, 104, 105, 106):
            self.assertTrue(is_route_failure(code))

    def test_planner_geometry_failures_require_a_different_route(self):
        for code in (203, 204, 205, 206, 208):
            self.assertTrue(is_route_failure(code))

    def test_infrastructure_and_timing_faults_do_not_blacklist_a_route(self):
        for code in (None, 0, 100, 101, 102, 107, 200, 201, 202, 207):
            self.assertFalse(is_route_failure(code))

    def test_error_label_is_operator_readable(self):
        self.assertEqual(
            navigation_error_label(106),
            '控制器找不到有效控制指令',
        )
        self.assertEqual(navigation_error_label(999), '未知错误码 999')


if __name__ == '__main__':
    unittest.main()
