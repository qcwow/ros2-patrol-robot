import math
import unittest

from patrol_robot_patrol.navigation_metrics import NavigationRunMetrics


class NavigationRunMetricsTest(unittest.TestCase):
    def test_records_path_clearance_retries_and_final_error(self):
        metrics = NavigationRunMetrics('normal', started_at=10.0)
        metrics.update_pose(0.0, 0.0)
        metrics.update_pose(0.3, 0.4)
        metrics.update_scan([math.inf, 1.2, 0.7])
        metrics.update_status({'state': 'NAVIGATING', 'retry_count': 1})
        status = {
            'state': 'COMPLETE',
            'retry_count': 0,
            'current_goal': {'x': 1.0, 'y': 2.0, 'yaw': 0.0},
            'completed_loops': 1,
        }
        metrics.update_status(status)

        result = metrics.summary(15.0, status, (1.06, 2.08, 0.05))

        self.assertTrue(result['success'])
        self.assertAlmostEqual(result['path_length_meters'], 0.5)
        self.assertAlmostEqual(result['minimum_obstacle_distance_meters'], 0.7)
        self.assertAlmostEqual(result['final_position_error_meters'], 0.1)
        self.assertEqual(result['maximum_retry_count'], 1)
        self.assertEqual(result['state_history'], ['NAVIGATING', 'COMPLETE'])

    def test_ignores_unusable_measurements_and_large_pose_jumps(self):
        metrics = NavigationRunMetrics('fault', started_at=0.0)
        metrics.update_pose(0.0, 0.0)
        metrics.update_pose(10.0, 10.0)
        metrics.update_scan([math.inf, math.nan])

        result = metrics.summary(1.0, {'state': 'BLOCKED'}, None)

        self.assertEqual(result['path_length_meters'], 0.0)
        self.assertIsNone(result['minimum_obstacle_distance_meters'])
        self.assertFalse(result['success'])
