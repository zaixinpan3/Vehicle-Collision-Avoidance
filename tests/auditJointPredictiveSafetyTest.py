"""Behavior checks for strict collision geometry with zero added buffer."""

import sys
import math
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from auditJointPredictiveSafety import clearance_passes, distance, rectangle, target_state


class CollisionAuditTest(unittest.TestCase):
    def test_constant_acceleration_changes_velocity_and_heading_rate(self):
        initial = [0, 0, .2, 6, 1.5, .1, 1.6, 2.4, .95, 0, 0]
        future = target_state(initial, 2)
        self.assertAlmostEqual(future[3], 9)
        self.assertAlmostEqual(future[2], .2 + 15 * math.sin(.1) / 1.6)

    def test_braking_continues_with_signed_velocity(self):
        initial = [0, 0, 0, 2, -2, 0, 1.6, 2.4, .95, 0, 0]
        self.assertEqual(target_state(initial, 1), (1, 0, 0, 0))
        self.assertEqual(target_state(initial, 3), (-3, 0, 0, -4))

    def test_touching_and_overlapping_rectangles_fail_without_buffer(self):
        ego = rectangle(0, 0, 0)
        for x in (0, 2, 4.8):
            with self.subTest(target_x=x):
                gap = distance(ego, rectangle(x, 0, 0))
                self.assertEqual(gap, 0)
                self.assertFalse(clearance_passes(gap, 0))

    def test_positive_gap_below_ten_centimeters_passes_without_buffer(self):
        gap = distance(rectangle(0, 0, 0), rectangle(4.801, 0, 0))
        self.assertGreater(gap, 0)
        self.assertTrue(clearance_passes(gap, 0))
        self.assertFalse(clearance_passes(gap, 0.1))

    def test_collision_is_not_accepted_by_a_numerical_tolerance(self):
        self.assertFalse(clearance_passes(0, 0))
        self.assertFalse(clearance_passes(-1e-12, 0))
        self.assertTrue(clearance_passes(1e-12, 0))


if __name__ == '__main__':
    unittest.main()
