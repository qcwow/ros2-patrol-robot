import unittest

from patrol_robot_patrol.route_model import parse_route


class RouteModelTest(unittest.TestCase):
    def test_legacy_route_is_migrated_without_breaking_existing_files(self):
        route = parse_route({
            'frame_id': 'map',
            'waypoints': [
                {'name': '基地', 'x': 0, 'y': 0},
                {'name': '设备', 'x': 1, 'y': 0},
            ],
        })

        self.assertEqual(route.waypoints[0].waypoint_type, 'HOME')
        self.assertFalse(route.waypoints[0].count_as_task)
        self.assertEqual(route.waypoints[1].waypoint_type, 'INSPECTION')
        self.assertTrue(route.waypoints[1].count_as_task)
        self.assertEqual(route.task_indices, (1,))

    def test_semantic_route_rotates_execution_order_from_explicit_home(self):
        route = parse_route({
            'route_id': 'line_a',
            'waypoints': [
                {'id': 'inspection', 'type': 'INSPECTION', 'x': 1, 'y': 0},
                {'id': 'gate', 'type': 'TRANSIT', 'x': 2, 'y': 0},
                {'id': 'base', 'type': 'HOME', 'x': 0, 'y': 0},
            ],
        })

        self.assertEqual(route.home_index, 2)
        self.assertEqual(route.ordered_indices, (2, 0, 1))
        self.assertEqual(route.task_indices, (0,))
        self.assertEqual(route.waypoints[1].recovery_policy, 'no_spin')

    def test_route_requires_exactly_one_explicit_home(self):
        with self.assertRaisesRegex(ValueError, '只能包含一个 HOME'):
            parse_route({
                'waypoints': [
                    {'type': 'INSPECTION', 'x': 0, 'y': 0},
                ],
            })

    def test_transit_cannot_be_counted_as_inspection_task(self):
        with self.assertRaisesRegex(ValueError, '只有 INSPECTION'):
            parse_route({
                'waypoints': [
                    {'type': 'HOME', 'x': 0, 'y': 0},
                    {
                        'type': 'TRANSIT',
                        'x': 1,
                        'y': 0,
                        'count_as_task': True,
                    },
                ],
            })

    def test_per_waypoint_policy_fields_are_loaded(self):
        route = parse_route({
            'route_id': 'precision',
            'waypoints': [
                {'type': 'HOME', 'x': 0, 'y': 0},
                {
                    'id': 'meter',
                    'type': 'INSPECTION',
                    'x': 1,
                    'y': 2,
                    'tolerance': {'position': 0.06, 'yaw': 0.07},
                    'speed_limit': 0.12,
                    'recovery_policy': 'restricted',
                    'required_sensor': 'rgbd',
                },
            ],
        })

        waypoint = route.waypoints[1]
        self.assertEqual(waypoint.position_tolerance, 0.06)
        self.assertEqual(waypoint.yaw_tolerance, 0.07)
        self.assertEqual(waypoint.speed_limit, 0.12)
        self.assertEqual(waypoint.required_sensor, 'rgbd')

    def test_rejects_unknown_required_sensor(self):
        with self.assertRaisesRegex(ValueError, 'required_sensor'):
            parse_route({
                'waypoints': [
                    {'type': 'HOME', 'x': 0, 'y': 0},
                    {
                        'type': 'INSPECTION',
                        'x': 1,
                        'y': 0,
                        'required_sensor': 'magic_camera',
                    },
                ],
            })


if __name__ == '__main__':
    unittest.main()
