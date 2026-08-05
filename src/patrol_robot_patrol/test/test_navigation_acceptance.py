import unittest

from patrol_robot_patrol.navigation_acceptance import (
    NavigationAcceptanceLimits,
    completed_inspection_count,
    evaluate_navigation_acceptance,
)


class NavigationAcceptanceTest(unittest.TestCase):
    def test_rejects_invalid_limits(self):
        with self.assertRaises(ValueError):
            NavigationAcceptanceLimits.from_mapping({
                'maximum_linear_speed': 0.0,
            })

    def test_counts_only_completed_inspection_visits(self):
        self.assertEqual(completed_inspection_count({
            'waypoint_tasks': [
                {'count_as_task': False, 'completed_visits': 4},
                {'count_as_task': True, 'completed_visits': 2},
                {'count_as_task': True, 'completed_visits': None},
            ],
        }), 2)

    def test_accepts_report_inside_every_limit(self):
        limits = NavigationAcceptanceLimits()
        result = evaluate_navigation_acceptance({
            'terminal_state': 'COMPLETE',
            'elapsed_seconds': 100.0,
            'maximum_linear_speed': 0.08,
            'maximum_angular_speed': 0.20,
            'maximum_retry_count': 1,
            'health_false_events': 0,
            'maximum_map_correction_translation': 0.01,
            'maximum_map_correction_yaw_degrees': 1.0,
            'completed_inspections': 0,
            'final_stop_verified': True,
        }, limits)
        self.assertTrue(result['passed'])

    def test_fails_overspeed_even_if_route_completes(self):
        limits = NavigationAcceptanceLimits()
        result = evaluate_navigation_acceptance({
            'terminal_state': 'COMPLETE',
            'elapsed_seconds': 100.0,
            'maximum_linear_speed': 0.081,
            'maximum_angular_speed': 0.20,
            'maximum_retry_count': 0,
            'health_false_events': 0,
            'maximum_map_correction_translation': 0.0,
            'maximum_map_correction_yaw_degrees': 0.0,
            'completed_inspections': 0,
            'final_stop_verified': True,
        }, limits)
        self.assertFalse(result['passed'])
        self.assertFalse(
            result['checks']['maximum_linear_speed']['passed']
        )

    def test_blocked_can_be_expected_for_fault_scenario(self):
        limits = NavigationAcceptanceLimits.from_mapping({
            'expected_terminal_state': 'BLOCKED',
        })
        report = {
            'terminal_state': 'BLOCKED',
            'elapsed_seconds': 20.0,
            'maximum_linear_speed': 0.0,
            'maximum_angular_speed': 0.0,
            'maximum_retry_count': 0,
            'health_false_events': 0,
            'maximum_map_correction_translation': 0.0,
            'maximum_map_correction_yaw_degrees': 0.0,
            'completed_inspections': 0,
            'final_stop_verified': True,
        }
        self.assertTrue(
            evaluate_navigation_acceptance(report, limits)['passed']
        )


if __name__ == '__main__':
    unittest.main()
