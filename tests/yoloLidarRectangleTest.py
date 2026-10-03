"""Visible-face bootstrap geometry for predicted-pose rectangle fitting."""
from pathlib import Path
import math
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "perception"))
import yolo_lidar_target_position as perception


class YoloLidarRectangleTest(unittest.TestCase):
    @staticmethod
    def fit(points, mode="symmetric", heading=0.0):
        return perception.fit_known_size_rectangle(
            points, 5.0, 1.9, heading, 0.0, 1.0, 0.0, 0.0, 2.0,
            placement_mode=mode,
        )[0]

    @staticmethod
    def rear_face(width=1.7):
        tangent = np.linspace(-width / 2, width / 2, 41)
        return np.column_stack([np.full(41, 10.0), tangent, np.ones(41)])

    def testNarrowRearBodyDoesNotPullCenterTowardOneBoundingBoxEdge(self):
        points = self.rear_face()
        np.testing.assert_allclose(self.fit(points), [12.5, 0.0], atol=1e-12)
        self.assertAlmostEqual(abs(self.fit(points, "edge")[1]), 0.1)

    def testSideFacePreservesNormalOffsetAndCentersTheObservedLength(self):
        tangent = np.linspace(8.0, 12.0, 41)
        points = np.column_stack([tangent, np.full(41, 3.0), np.ones(41)])
        np.testing.assert_allclose(self.fit(points), [10.0, 3.95], atol=1e-12)

    def testRigidRotationAndTranslationPreserveRearFaceCenter(self):
        angle = math.radians(32.0)
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        shift = np.array([4.0, -2.0])
        points = self.rear_face()
        # A short adjacent face resolves the front/rear ambiguity of a flat line.
        points = np.vstack([points, [[10.1, 0.85, 1.0], [10.3, 0.85, 1.0]]])
        points[:, :2] = points[:, :2] @ rotation.T + shift
        expected = rotation @ np.array([12.5, 0.0]) + shift
        np.testing.assert_allclose(self.fit(points, heading=angle), expected, atol=1e-12)

    def testUnevenPointDensityDoesNotReplaceGeometricCenterWithCentroid(self):
        points = self.rear_face()
        points = np.vstack([points, np.repeat(points[:8], 12, axis=0)])
        self.assertGreater(abs(np.mean(points[:, 1])), 0.3)
        np.testing.assert_allclose(self.fit(points), [12.5, 0.0], atol=1e-12)

    def testWeakSupportKeepsLegacyFaceAnchoring(self):
        points = self.rear_face(width=1.2)
        np.testing.assert_array_equal(self.fit(points), self.fit(points, "edge"))

    def testFullRectangleKeepsTheSameCenter(self):
        points = np.array([[10.0, -0.95, 1.0], [10.0, 0.95, 1.0],
                           [15.0, -0.95, 1.0], [15.0, 0.95, 1.0]])
        np.testing.assert_allclose(self.fit(points), [12.5, 0.0], atol=1e-12)
        np.testing.assert_array_equal(self.fit(points), self.fit(points, "edge"))

    def testFirstAcquisitionBootstrapsTheCenterBeforeFreePoseRefinement(self):
        args = perception.parse_args([
            "--dataset", ".", "--model", "unused.pt",
            "--vehicle-length", "5", "--vehicle-width", "1.9",
        ])
        detection = perception.Detection("front", (0.0, 0.0, 10.0, 10.0), 0.9, 2, "car")
        prior = perception.PosePrior(None, 0.0, 0.0, None, "initialization")
        estimate = perception.estimate_from_cluster(detection, self.rear_face(), args, prior)
        self.assertIsNotNone(estimate)
        np.testing.assert_allclose(estimate.relative_position, [12.5, 0.0], atol=1e-12)
        self.assertEqual(estimate.rectangle["heading_constraint"], "free")

    def testInvalidSupportFractionIsRejected(self):
        for support in (0.0, -0.1, 1.1, float("nan")):
            with self.subTest(support=support), self.assertRaises(ValueError):
                perception.fit_known_size_rectangle(
                    self.rear_face(), 5.0, 1.9, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0,
                    symmetry_minimum_support=support,
                )


if __name__ == "__main__":
    unittest.main()
