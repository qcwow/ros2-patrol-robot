import unittest

from patrol_robot_patrol.health_recovery_progress import HealthRecoveryProgress


class HealthRecoveryProgressTest(unittest.TestCase):
    def test_requires_both_stable_time_and_forward_progress(self):
        progress = HealthRecoveryProgress(5.0, 0.5)
        progress.arm()

        self.assertFalse(progress.observe(10.0, 4.0, True))
        self.assertFalse(progress.observe(16.0, 3.6, True))
        self.assertTrue(progress.observe(17.0, 3.4, True))
        self.assertAlmostEqual(progress.progress_meters, 0.6)

    def test_new_fault_discards_partial_safe_progress(self):
        progress = HealthRecoveryProgress(5.0, 0.5)
        progress.arm()
        progress.observe(10.0, 4.0, True)
        progress.observe(13.0, 3.6, True)

        self.assertFalse(progress.observe(14.0, 3.6, False))
        self.assertEqual(progress.progress_meters, 0.0)
        self.assertFalse(progress.observe(20.0, 3.5, True))
        self.assertFalse(progress.observe(24.0, 3.0, True))
        self.assertTrue(progress.observe(25.0, 3.0, True))

    def test_rearming_requires_fresh_progress(self):
        progress = HealthRecoveryProgress(2.0, 0.2)
        progress.arm()
        progress.observe(0.0, 2.0, True)
        self.assertTrue(progress.observe(2.0, 1.7, True))

        progress.arm()
        self.assertFalse(progress.observe(5.0, 1.7, True))
        self.assertFalse(progress.observe(8.0, 1.6, True))

    def test_clear_disables_validation(self):
        progress = HealthRecoveryProgress(0.0, 0.0)
        progress.arm()
        progress.clear()

        self.assertFalse(progress.observe(1.0, 1.0, True))
        self.assertFalse(progress.armed)


if __name__ == '__main__':
    unittest.main()
