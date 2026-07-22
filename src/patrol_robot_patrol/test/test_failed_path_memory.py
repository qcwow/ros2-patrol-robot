import unittest

from patrol_robot_patrol.failed_path_memory import FailedPathMemory


class FailedPathMemoryTest(unittest.TestCase):
    def test_remembers_full_route_but_preserves_start_and_goal(self):
        memory = FailedPathMemory()

        added = memory.remember([(0.0, 0.0), (4.0, 0.0)], (0.0, 0.0))

        self.assertGreater(added, 0)
        self.assertEqual(memory.route_count, 1)
        self.assertGreater(min(x for x, _ in memory.points), 0.55)
        self.assertLess(max(x for x, _ in memory.points), 3.45)

    def test_rejects_same_route_and_small_parallel_shift(self):
        memory = FailedPathMemory()
        memory.remember([(0.0, 0.0), (4.0, 0.0)], (0.0, 0.0))

        same, same_ratio = memory.is_similar(
            [(0.0, 0.0), (4.0, 0.0)]
        )
        shifted, shifted_ratio = memory.is_similar(
            [(0.0, 0.3), (4.0, 0.3)]
        )

        self.assertTrue(same)
        self.assertAlmostEqual(same_ratio, 1.0)
        self.assertTrue(shifted)
        self.assertGreaterEqual(shifted_ratio, memory.similarity_ratio)

    def test_accepts_a_topologically_different_route(self):
        memory = FailedPathMemory()
        memory.remember([(0.0, 0.0), (4.0, 0.0)], (0.0, 0.0))
        alternate = [
            (0.0, 0.0),
            (0.0, 2.0),
            (4.0, 2.0),
            (4.0, 0.0),
        ]

        similar, ratio = memory.is_similar(alternate)

        self.assertFalse(similar)
        self.assertLess(ratio, memory.similarity_ratio)

    def test_accumulates_rejected_candidate_routes(self):
        memory = FailedPathMemory()
        first = memory.remember(
            [(0.0, 0.0), (4.0, 0.0)], (0.0, 0.0)
        )
        second = memory.remember(
            [(0.0, 0.5), (4.0, 0.5)], (0.0, 0.5)
        )

        self.assertGreater(first, 0)
        self.assertGreater(second, 0)
        self.assertEqual(memory.route_count, 2)

    def test_does_not_block_a_nearby_goal(self):
        memory = FailedPathMemory()

        added = memory.remember([(0.0, 0.0), (1.0, 0.0)], (0.0, 0.0))

        self.assertEqual(added, 0)
        self.assertEqual(memory.points, ())

    def test_clear_removes_routes_and_cells(self):
        memory = FailedPathMemory()
        memory.remember([(0.0, 0.0), (4.0, 0.0)], (0.0, 0.0))

        self.assertTrue(memory.clear())
        self.assertEqual(memory.route_count, 0)
        self.assertEqual(memory.points, ())
        self.assertFalse(memory.clear())


if __name__ == '__main__':
    unittest.main()
