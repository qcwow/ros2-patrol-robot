import math
import unittest

from patrol_robot_patrol.rotation_diagnostics import (
    accumulated_yaw,
    compose_planar_transforms,
    integrate_series,
    summarize_rotation_event,
)


def _sample(elapsed, odom_yaw, correction_yaw, command=0.0, imu=0.0):
    return {
        'elapsed_seconds': elapsed,
        'odom_to_base': {'yaw': odom_yaw},
        'map_to_odom': {'yaw': correction_yaw},
        'map_to_base': {'yaw': odom_yaw + correction_yaw},
        'commands': {'cmd_vel_safety_checked_angular_z': command},
        'imu': {'angular_velocity_z': imu},
    }


class RotationDiagnosticsTest(unittest.TestCase):
    def test_composes_planar_transforms(self):
        result = compose_planar_transforms(
            {'x': 1.0, 'y': 2.0, 'yaw': math.pi / 2.0},
            {'x': 2.0, 'y': 1.0, 'yaw': -math.pi / 4.0},
        )
        self.assertAlmostEqual(result['x'], 0.0)
        self.assertAlmostEqual(result['y'], 4.0)
        self.assertAlmostEqual(result['yaw'], math.pi / 4.0)

    def test_accumulated_yaw_unwraps_pi_boundary(self):
        result = accumulated_yaw([
            math.radians(179.0),
            math.radians(-179.0),
            math.radians(-170.0),
        ])
        self.assertAlmostEqual(math.degrees(result), 11.0)

    def test_integrate_series_ignores_large_gaps(self):
        self.assertEqual(
            integrate_series([(0.0, 1.0), (0.5, 1.0), (2.0, 1.0)]),
            0.5,
        )

    def test_classifies_physical_chassis_rotation(self):
        result = summarize_rotation_event([
            _sample(0.0, 0.0, 0.0, 0.4, 0.4),
            _sample(1.0, 0.4, 0.02, 0.4, 0.4),
        ])
        self.assertEqual(
            result['classification'], 'physical_chassis_rotation'
        )
        self.assertAlmostEqual(
            result['odom_to_base_yaw_degrees'], 22.918, places=3
        )

    def test_classifies_localization_correction(self):
        result = summarize_rotation_event([
            _sample(0.0, 0.0, 0.0),
            _sample(1.0, 0.02, 0.5),
        ])
        self.assertEqual(result['classification'], 'localization_correction')

    def test_classifies_mixed_rotation(self):
        result = summarize_rotation_event([
            _sample(0.0, 0.0, 0.0),
            _sample(1.0, 0.4, 0.3),
        ])
        self.assertEqual(
            result['classification'],
            'mixed_physical_and_localization_rotation',
        )

    def test_reports_missing_tf_data(self):
        result = summarize_rotation_event([{'elapsed_seconds': 0.0}])
        self.assertEqual(result['classification'], 'insufficient_tf_data')

    def test_reports_sensor_timestamp_alignment(self):
        sample = _sample(0.0, 0.0, 0.0)
        sample['imu']['stamp_ns'] = 1_020_000_000
        sample['sensors'] = {
            'rgb': {'stamp_ns': 1_000_000_000},
            'depth': {'stamp_ns': 1_010_000_000},
            'odom_raw': {'stamp_ns': 1_015_000_000},
        }
        result = summarize_rotation_event([sample])
        alignment = result['timestamp_alignment']
        self.assertEqual(
            alignment['rgb_to_depth']['maximum_absolute_ms'], 10.0
        )
        self.assertEqual(
            alignment['imu_to_raw_odom']['maximum_absolute_ms'], 5.0
        )

    def test_reports_each_available_command_stage(self):
        sample = _sample(0.0, 0.0, 0.0, command=-0.3)
        sample['commands']['controller_cmd_vel_linear_x'] = -0.1
        sample['commands']['controller_cmd_vel_angular_z'] = -0.2
        result = summarize_rotation_event([sample])
        stages = result['command_stage_peaks']
        self.assertEqual(
            stages['cmd_vel_safety_checked']['maximum_absolute_angular_z'],
            0.3,
        )
        self.assertEqual(
            stages['controller_cmd_vel']['maximum_absolute_linear_x'], 0.1
        )


if __name__ == '__main__':
    unittest.main()
