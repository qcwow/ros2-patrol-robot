import unittest

from patrol_robot_patrol.task_ledger import PatrolTaskLedger


class PatrolTaskLedgerTest(unittest.TestCase):
    def test_home_has_no_counter_and_inspections_start_at_loop_count(self):
        ledger = PatrolTaskLedger(4, 10)

        self.assertEqual(ledger.remaining, [0, 10, 10, 10])
        self.assertEqual(ledger.completed_visits(0), 0)

    def test_round_requires_every_inspection_task(self):
        ledger = PatrolTaskLedger(4, 2)

        ledger.complete_inspection(1)
        ledger.complete_inspection(2)
        self.assertFalse(ledger.round_ready(0))
        ledger.complete_inspection(3)

        self.assertTrue(ledger.round_ready(0))
        self.assertEqual(ledger.remaining, [0, 1, 1, 1])

    def test_each_completed_round_decrements_every_task_once(self):
        ledger = PatrolTaskLedger(3, 2)

        for index in (1, 2):
            ledger.complete_inspection(index)
        self.assertTrue(ledger.round_ready(0))
        for index in (1, 2):
            ledger.complete_inspection(index)

        self.assertTrue(ledger.round_ready(1))
        self.assertEqual(ledger.remaining, [0, 0, 0])

    def test_cannot_complete_home_as_an_inspection(self):
        ledger = PatrolTaskLedger(2, 1)

        with self.assertRaises(IndexError):
            ledger.complete_inspection(0)

    def test_transit_points_do_not_receive_task_counters(self):
        ledger = PatrolTaskLedger(5, 2, task_indices=(1, 3))

        self.assertEqual(ledger.remaining, [0, 2, 0, 2, 0])
        ledger.complete_inspection(1)
        ledger.complete_inspection(3)

        self.assertTrue(ledger.round_ready(0))
        with self.assertRaises(IndexError):
            ledger.complete_inspection(2)


if __name__ == '__main__':
    unittest.main()
