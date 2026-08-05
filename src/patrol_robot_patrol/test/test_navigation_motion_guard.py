import math
import unittest

from patrol_robot_patrol.navigation_motion_guard import (
    NavigationMotionGuard,
    PoseStabilityGate,
)


class PoseStabilityGateTest(unittest.TestCase):
    def test_requires_continuous_stability_window(self):
        gate = PoseStabilityGate(5.0, 0.08, math.radians(5.0))
        self.assertFalse(gate.observe(0.0, 0.0, 0.0, 0.0).ready)
        self.assertFalse(gate.observe(4.9, 0.02, 0.0, 0.01).ready)
        self.assertTrue(gate.observe(5.1, 0.02, 0.0, 0.01).ready)

    def test_pose_jump_restarts_stability_window(self):
        gate = PoseStabilityGate(5.0, 0.08, math.radians(5.0))
        gate.observe(0.0, 0.0, 0.0, 0.0)
        status = gate.observe(4.0, 0.10, 0.0, 0.0)
        self.assertFalse(status.ready)
        self.assertEqual(status.stable_for, 0.0)
        self.assertFalse(gate.observe(8.9, 0.10, 0.0, 0.0).ready)
        self.assertTrue(gate.observe(9.1, 0.10, 0.0, 0.0).ready)


class NavigationMotionGuardTest(unittest.TestCase):
    def test_trips_when_yaw_grows_without_progress(self):
        guard = NavigationMotionGuard(
            2.0, 0.08, math.radians(20.0), 0.02
        )
        guard.observe(0.0, 0.0, 0.0, 0.0, 0.30)
        status = guard.observe(
            2.1, 0.05, 0.0, math.radians(28.0), 0.48
        )
        self.assertTrue(status.tripped)

    def test_allows_curved_motion_with_goal_progress(self):
        guard = NavigationMotionGuard(
            2.0, 0.08, math.radians(20.0), 0.02
        )
        guard.observe(0.0, 0.0, 0.0, 0.0, 1.00)
        status = guard.observe(
            2.1, 0.05, 0.0, math.radians(28.0), 0.90
        )
        self.assertFalse(status.tripped)

    def test_handles_yaw_wraparound(self):
        guard = NavigationMotionGuard(
            2.0, 0.08, math.radians(20.0), 0.02
        )
        guard.observe(0.0, 0.0, 0.0, math.radians(175.0), 0.30)
        status = guard.observe(
            2.1, 0.01, 0.0, math.radians(-155.0), 0.31
        )
        self.assertTrue(status.tripped)


if __name__ == '__main__':
    unittest.main()
