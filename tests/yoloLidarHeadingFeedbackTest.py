"""Known-orientation geometry and estimator/perception feedback timing."""
from pathlib import Path
import copy
import math
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "perception"))
import yolo_lidar_target_position as perception


class Feedback:
    def __init__(self, ego_heading=0.0):
        self.ego_heading = ego_heading
        self.last_time = None
        self.events = []

    def predict(self, frame, time):
        self.events.append(("predict", frame))
        return perception.HeadingPrior(self.ego_heading, time, self.last_time,
                                       "initialization" if self.last_time is None else "prediction")

    def update(self, frame, time, position):
        self.events.append(("update", frame, position is not None))
        self.last_time = time
        if position is not None:
            self.ego_heading += 0.001 * position[1]
            position[:] = 999.0  # Adapters must not be able to mutate published outputs.


class YoloLidarHeadingFeedbackTest(unittest.TestCase):
    def setUp(self):
        self.args = perception.parse_args([
            "--dataset", ".", "--model", "unused.pt", "--causal",
            "--center-mode", "rectangle", "--vehicle-length", "5",
            "--vehicle-width", "1.9", "--rectangle-placement", "symmetric",
        ])
        self.det = perception.Detection("front", (0.0, 0.0, 10.0, 10.0), 0.9, 2, "car")

    def records(self, count):
        points = np.column_stack([np.full(41, 10.0), np.linspace(-0.85, 0.85, 41), np.ones(41)])
        return [dict(frame_number=i, time=i * 0.02, ego_yaw=0.0,
                     clusters=[(self.det, points.copy())]) for i in range(count)]

    def testBothRectangleFitStagesKeepTheSuppliedOrientation(self):
        points = self.records(1)[0]["clusters"][0][1]
        heading = 0.12
        estimate = perception.estimate_from_cluster(self.det, points, self.args, heading, fixed_heading=True)
        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(math.sin(estimate.rectangle["heading_rad"] - heading), 0.0, places=12)
        self.assertEqual(estimate.rectangle["heading_constraint"], "fixed")

    def testFixedOrientationRequiresAnActualFinitePrior(self):
        for heading in [None, float("nan"), float("inf")]:
            with self.subTest(heading=heading), self.assertRaises(ValueError):
                perception.estimate_from_cluster(self.det, self.records(1)[0]["clusters"][0][1],
                                                 self.args, heading, fixed_heading=True)

    def testPredictionPrecedesEachUpdateAndDropoutsStillAdvanceFeedback(self):
        records = self.records(3)
        records[1]["clusters"] = []
        feedback = Feedback()
        estimates, _, _ = perception.estimator_track_estimates(records, self.args, feedback)
        self.assertEqual(feedback.events, [("predict", 0), ("update", 0, True),
                                          ("predict", 1), ("update", 1, False),
                                          ("predict", 2), ("update", 2, True)])
        self.assertEqual(estimates[1], [])
        np.testing.assert_allclose(estimates[0][0].relative_position, [12.5, 0.0], atol=1e-12)

    def testRelativeHeadingDoesNotAcquireAnExtraWorldYawRotation(self):
        records = self.records(1)
        angle = 0.3
        records[0]["ego_yaw"] = 1.2
        rotation = np.array([[math.cos(angle), math.sin(angle)],
                             [-math.sin(angle), math.cos(angle)]])
        records[0]["clusters"][0][1][:, :2] = records[0]["clusters"][0][1][:, :2] @ rotation.T
        result = perception.estimator_track_estimates(records, self.args, Feedback(-angle))
        self.assertAlmostEqual(result[1][0], -angle)
        self.assertAlmostEqual(math.sin(result[0][0][0].rectangle["heading_rad"] + angle), 0.0)

    def testCurrentFutureStaleAndNonfinitePriorsAreRejected(self):
        invalid = [perception.HeadingPrior(0.0, 0.0, 0.0, "prediction"),
                   perception.HeadingPrior(0.0, 0.0, 1.0, "prediction"),
                   perception.HeadingPrior(0.0, -0.02, -0.04, "prediction"),
                   perception.HeadingPrior(float("nan"), 0.0, -0.02, "prediction"),
                   perception.HeadingPrior(0.0, 0.0, None, "prediction")]
        for prior in invalid:
            feedback = Feedback()
            feedback.predict = lambda frame, time: prior
            with self.subTest(prior=prior), self.assertRaises(ValueError):
                perception.estimator_track_estimates(self.records(1), self.args, feedback)
            self.assertEqual(feedback.events, [])

    def testFutureMeasurementsCannotReviseTheFeedbackPrefix(self):
        records = self.records(8)
        short = perception.estimator_track_estimates(copy.deepcopy(records[:4]), self.args, Feedback())
        for row in records[4:]:
            row["clusters"][0][1][:, 1] += 5.0
        long = perception.estimator_track_estimates(records, self.args, Feedback())
        self.assertEqual(short[1], long[1][:4])
        for expected, actual in zip(short[0], long[0][:4]):
            np.testing.assert_array_equal(expected[0].relative_position, actual[0].relative_position)

    def testMissingPriorFallsBackToGeometricSearch(self):
        feedback = Feedback()
        feedback.predict = lambda frame, time: None
        records = self.records(1)
        points = records[0]["clusters"][0][1]
        side = np.column_stack([np.linspace(10.0, 15.0, 41), np.full(41, 0.85), np.ones(41)])
        records[0]["clusters"] = [(self.det, np.vstack([points, side]))]
        estimates, headings, count = perception.estimator_track_estimates(records, self.args, feedback)
        self.assertEqual((headings, count), ([None], 0))
        self.assertEqual(estimates[0][0].rectangle["heading_constraint"], "search")
        self.assertEqual(feedback.events, [("update", 0, True)])

    def testRepeatedTimestampsCannotUpdateTheEstimatorTwice(self):
        records = self.records(2)
        records[1]["time"] = records[0]["time"]
        feedback = Feedback()
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            perception.estimator_track_estimates(records, self.args, feedback)
        self.assertEqual(feedback.events, [("predict", 0), ("update", 0, True)])

    def testFeedbackCannotRunInAnOfflineOrNonRectanglePipeline(self):
        for causal, center in [(False, "rectangle"), (True, "capsule")]:
            self.args.causal, self.args.center_mode = causal, center
            with self.assertRaisesRegex(ValueError, "causal rectangle"):
                perception.run_pipeline(self.args, heading_feedback=Feedback())


if __name__ == "__main__":
    unittest.main()
