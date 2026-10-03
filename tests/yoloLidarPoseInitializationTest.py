"""Predicted poses initialize a free geometric fit without becoming observations."""
from pathlib import Path
import copy
import math
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "perception"))
import yolo_lidar_target_position as perception
from rectangle_pose_fit import fit_rectangle_pose


def rectangle_points(center, heading):
    rear = np.column_stack([np.full(51, -2.5), np.linspace(-0.95, 0.95, 51)])
    side = np.column_stack([np.linspace(-2.5, 2.5, 81), np.full(81, -0.95)])
    rotation = np.asarray([[math.cos(heading), -math.sin(heading)],
                           [math.sin(heading), math.cos(heading)]])
    return np.column_stack([np.vstack([rear, side]) @ rotation.T + center, np.ones(132)])


class PoseFeedback:
    def __init__(self):
        self.last_time = None
        self.events = []

    def predict(self, frame, time):
        self.events.append(("predict", frame))
        return perception.PosePrior((12.8 + 0.1 * frame, 0.7), 0.25,
                                    time, self.last_time,
                                    "initialization" if self.last_time is None else "prediction")

    def update(self, frame, time, position):
        self.events.append(("update", frame, None if position is None else position.copy()))
        self.last_time = time
        if position is not None:
            position[:] = -999.0


class YoloLidarPoseInitializationTest(unittest.TestCase):
    def setUp(self):
        self.args = perception.parse_args([
            "--dataset", ".", "--model", "unused.pt", "--causal",
            "--center-mode", "rectangle", "--vehicle-length", "5", "--vehicle-width", "1.9",
        ])
        self.det = perception.Detection("front", (0.0, 0.0, 10.0, 10.0), 0.9, 2, "car")

    def records(self, count):
        return [dict(frame_number=i, time=i * 0.02, ego_yaw=1.1,
                     clusters=[(self.det, rectangle_points([12.5 + 0.1 * i, 0.2], 0.1))])
                for i in range(count)]

    def testBothPositionAndHeadingMoveFromAnIncorrectSeed(self):
        points = rectangle_points([12.5, 0.2], 0.1)
        initial = np.asarray([12.8, 0.7, 0.25])
        original = initial.copy()
        center, fit = fit_rectangle_pose(points, initial, 5.0, 1.9)
        np.testing.assert_allclose(center, [12.5, 0.2], atol=2e-4)
        self.assertAlmostEqual(fit["heading_rad"], 0.1, delta=2e-4)
        self.assertLess(fit["objective"], fit["seed_objective"] * 1e-4)
        np.testing.assert_array_equal(initial, original)

    def testSeveralNearbySeedsReachTheSameObservedPose(self):
        points = rectangle_points([15.0, -1.0], -0.2)
        for seed in [[15.4, -0.6, -0.05], [14.7, -1.3, -0.35]]:
            with self.subTest(seed=seed):
                center, fit = fit_rectangle_pose(points, np.asarray(seed), 5.0, 1.9)
                np.testing.assert_allclose(center, [15.0, -1.0], atol=2e-4)
                self.assertAlmostEqual(fit["heading_rad"], -0.2, delta=2e-4)

    def testUnobservedFaceTangentRetainsSeedAmbiguity(self):
        points = np.column_stack([np.full(31, 10.0), np.linspace(-0.2, 0.2, 31), np.ones(31)])
        centers = [fit_rectangle_pose(points, np.asarray([12.5, y, 0.0]), 5.0, 1.9)[0]
                   for y in [-0.3, 0.3]]
        self.assertAlmostEqual(centers[0][1], -0.3)
        self.assertAlmostEqual(centers[1][1], 0.3)

    def testCurrentBodyPoseIsUsedWithoutAnExtraEgoRotation(self):
        records = self.records(1)
        result, _, _ = perception.estimator_track_estimates(
            records, self.args, PoseFeedback(), pose_initialization=True)
        fit = result[0][0].rectangle
        np.testing.assert_allclose(fit["prediction_initial_pose"], [12.8, 0.7, 0.25])
        np.testing.assert_allclose(result[0][0].relative_position, [12.5, 0.2], atol=3e-4)
        self.assertEqual(fit["heading_constraint"], "free")

    def testMissingDetectionsDeliverNoPredictedPositionAsMeasurement(self):
        records = self.records(3)
        records[1]["clusters"] = []
        feedback = PoseFeedback()
        result, _, _ = perception.estimator_track_estimates(
            records, self.args, feedback, pose_initialization=True)
        self.assertEqual([(e[0], e[1]) for e in feedback.events],
                         [("predict", 0), ("update", 0), ("predict", 1), ("update", 1),
                          ("predict", 2), ("update", 2)])
        self.assertIsNone(feedback.events[3][2])
        self.assertEqual(result[1], [])
        self.assertGreater(result[0][0].relative_position[0], 0.0)

    def testPoseModePreservesThePrefixWithoutLookingAtFutureMeasurements(self):
        short_records = self.records(3)
        records = self.records(6)
        records[3:][0]["clusters"] = []
        short = perception.estimator_track_estimates(short_records, self.args, PoseFeedback(), pose_initialization=True)
        long = perception.estimator_track_estimates(records, self.args, PoseFeedback(), pose_initialization=True)
        for before, after in zip(short[0], long[0]):
            np.testing.assert_array_equal(before[0].relative_position, after[0].relative_position)

    def testInitializationCanAcquireItsCenterFromTheCurrentCloud(self):
        feedback = PoseFeedback()
        feedback.predict = lambda frame, time: perception.PosePrior(None, 0.12, time, None, "initialization")
        result, _, _ = perception.estimator_track_estimates(
            self.records(1), self.args, feedback, pose_initialization=True)
        np.testing.assert_allclose(result[0][0].relative_position, [12.5, 0.2], atol=3e-4)

    def testMalformedStaleAndAlreadyUpdatedPriorsAreRejectedBeforeUpdate(self):
        priors = [
            perception.PosePrior((1.0, float("nan")), 0.0, 0.0, None, "initialization"),
            perception.PosePrior((1.0, 2.0), 0.0, -0.02, -0.04, "prediction"),
            perception.PosePrior((1.0, 2.0), 0.0, 0.0, 0.0, "prediction"),
            perception.PosePrior(None, 0.0, 0.0, -0.02, "prediction"),
        ]
        for prior in priors:
            feedback = PoseFeedback()
            feedback.predict = lambda frame, time: prior
            with self.subTest(prior=prior), self.assertRaises(ValueError):
                perception.estimator_track_estimates(self.records(1), self.args, feedback, pose_initialization=True)
            self.assertEqual(feedback.events, [])

    def testHeadingLockAndPoseInitializationCannotBeRequestedTogether(self):
        with self.assertRaisesRegex(ValueError, "either"):
            perception.run_pipeline(self.args, heading_feedback=PoseFeedback(), pose_feedback=PoseFeedback())


if __name__ == "__main__":
    unittest.main()
