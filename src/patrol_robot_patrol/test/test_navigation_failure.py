import unittest

from patrol_robot_patrol.navigation_failure import (
    is_route_failure,
    navigation_error_label,
    should_blacklist_health_failure,
    should_blacklist_route,
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

    def test_humble_aborted_result_blacklists_verified_candidate(self):
        self.assertTrue(
            should_blacklist_route(
                None,
                action_aborted=True,
                has_candidate_path=True,
            )
        )

    def test_humble_failure_without_candidate_does_not_blacklist(self):
        self.assertFalse(
            should_blacklist_route(
                None,
                action_aborted=True,
                has_candidate_path=False,
            )
        )
        self.assertFalse(
            should_blacklist_route(
                None,
                action_aborted=False,
                has_candidate_path=True,
            )
        )

    def test_confirmed_footprint_collision_blacklists_frontier(self):
        checks = {
            'nav2_active': True,
            'scan_ok': True,
            'odom_ok': True,
            'map_to_odom_tf_ok': True,
            'odom_to_base_tf_ok': True,
            'localization_ok': True,
            'costmap_fresh': True,
            'footprint_raw_clear': False,
            'footprint_clear': False,
        }
        self.assertTrue(should_blacklist_health_failure(checks))

    def test_infrastructure_health_failure_never_blacklists_frontier(self):
        base = {
            'nav2_active': True,
            'scan_ok': True,
            'odom_ok': True,
            'map_to_odom_tf_ok': True,
            'odom_to_base_tf_ok': True,
            'localization_ok': True,
            'costmap_fresh': True,
            'footprint_raw_clear': False,
            'footprint_clear': False,
        }
        for key in (
            'nav2_active',
            'scan_ok',
            'odom_ok',
            'map_to_odom_tf_ok',
            'odom_to_base_tf_ok',
            'localization_ok',
            'costmap_fresh',
        ):
            checks = dict(base)
            checks[key] = False
            self.assertFalse(should_blacklist_health_failure(checks), key)
        self.assertFalse(should_blacklist_health_failure(None))


if __name__ == '__main__':
    unittest.main()
