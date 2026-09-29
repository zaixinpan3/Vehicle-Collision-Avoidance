"""Behavior checks for strict collision geometry with zero added buffer."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from auditJointPredictiveSafety import clearance_passes, distance, rectangle


class CollisionAuditTest(unittest.TestCase):
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
