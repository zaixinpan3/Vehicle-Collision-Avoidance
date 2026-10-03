"""Predicted poses initialize a free geometric fit without becoming observations."""
from pathlib import Path
import contextlib
import io
import math
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

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
            "--dataset", ".", "--model", "unused.pt",
            "--vehicle-length", "5", "--vehicle-width", "1.9",
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
        center, fit = fit_rectangle_pose(points, initial, 5.0, 1.9, support_quantile=0.0)
        np.testing.assert_allclose(center, [12.5, 0.2], atol=2e-4)
        self.assertAlmostEqual(fit["heading_rad"], 0.1, delta=2e-4)
        self.assertLess(fit["objective"], fit["seed_objective"] * 1e-4)
        np.testing.assert_array_equal(initial, original)

    def testSeveralNearbySeedsReachTheSameObservedPose(self):
        points = rectangle_points([15.0, -1.0], -0.2)
        for seed in [[15.4, -0.6, -0.05], [14.7, -1.3, -0.35]]:
            with self.subTest(seed=seed):
                center, fit = fit_rectangle_pose(points, np.asarray(seed), 5.0, 1.9, support_quantile=0.0)
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
            records, self.args, PoseFeedback())
        fit = result[0][0].rectangle
        np.testing.assert_allclose(fit["prediction_initial_pose"], [12.8, 0.7, 0.25])
        np.testing.assert_allclose(result[0][0].relative_position, [12.5, 0.2], atol=3e-4)
        self.assertEqual(fit["heading_constraint"], "free")

    def testMissingDetectionsDeliverNoPredictedPositionAsMeasurement(self):
        records = self.records(3)
        records[1]["clusters"] = []
        feedback = PoseFeedback()
        result, _, _ = perception.estimator_track_estimates(
            records, self.args, feedback)
        self.assertEqual([(e[0], e[1]) for e in feedback.events],
                         [("predict", 0), ("update", 0), ("predict", 1), ("update", 1),
                          ("predict", 2), ("update", 2)])
        self.assertIsNone(feedback.events[3][2])
        self.assertEqual(result[1], [])
        self.assertGreater(result[0][0].relative_position[0], 0.0)

    def testPoseModePreservesThePrefixWithoutLookingAtFutureMeasurements(self):
        short_records = self.records(3)
        records = self.records(6)
        records[3]["clusters"] = []
        short = perception.estimator_track_estimates(short_records, self.args, PoseFeedback())
        long = perception.estimator_track_estimates(records, self.args, PoseFeedback())
        for before, after in zip(short[0], long[0]):
            np.testing.assert_array_equal(before[0].relative_position, after[0].relative_position)

    def testInitializationCanAcquireItsCenterFromTheCurrentCloud(self):
        feedback = PoseFeedback()
        feedback.predict = lambda frame, time: perception.PosePrior(None, 0.12, time, None, "initialization")
        result, _, _ = perception.estimator_track_estimates(
            self.records(1), self.args, feedback)
        np.testing.assert_allclose(result[0][0].relative_position, [12.5, 0.2], atol=3e-4)

    def testMalformedStaleAndAlreadyUpdatedPriorsAreRejectedBeforeUpdate(self):
        priors = [
            perception.PosePrior((1.0, float("nan")), 0.0, 0.0, None, "initialization"),
            perception.PosePrior((1.0, 2.0), 0.0, -0.02, -0.04, "prediction"),
            perception.PosePrior((1.0, 2.0), 0.0, 0.0, 0.0, "prediction"),
            perception.PosePrior(None, 0.0, 0.0, -0.02, "prediction"),
            perception.PosePrior((1.0, 2.0), float("nan"), 0.0, -0.02, "prediction"),
            perception.PosePrior((1.0, 2.0), 0.0, 0.0, None, "prediction"),
            None,
        ]
        for prior in priors:
            feedback = PoseFeedback()
            feedback.predict = lambda frame, time: prior
            with self.subTest(prior=prior), self.assertRaises(ValueError):
                perception.estimator_track_estimates(self.records(1), self.args, feedback)
            self.assertEqual(feedback.events, [])

    def testThePipelineCannotRunWithoutItsEstimator(self):
        with self.assertRaises(TypeError):
            perception.run_pipeline(self.args)
        with self.assertRaisesRegex(ValueError, "requires an estimator"):
            perception.run_pipeline(self.args, pose_feedback=None)

    def testRemovedHeadingLockAndModeFlagsAreRejected(self):
        with self.assertRaises(TypeError):
            perception.run_pipeline(self.args, heading_feedback=PoseFeedback(), pose_feedback=PoseFeedback())
        for flag in ["--causal", "--heading-iterations", "--center-mode", "--rectangle-placement"]:
            with self.subTest(flag=flag), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                perception.parse_args(["--dataset", ".", "--model", "unused.pt", flag])

    def testRepeatedTimestampsCannotAdvanceTheEstimatorTwice(self):
        records = self.records(2)
        records[1]["time"] = records[0]["time"]
        feedback = PoseFeedback()
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            perception.estimator_track_estimates(records, self.args, feedback)
        self.assertEqual(len(feedback.events), 2)

    def testPointsCanOverrideBothComponentsOfThePredictedCenter(self):
        records = self.records(1)
        records[0]["clusters"] = [(self.det, rectangle_points([13.2, -0.1], 0.3))]
        result, _, _ = perception.estimator_track_estimates(records, self.args, PoseFeedback())
        np.testing.assert_allclose(result[0][0].relative_position, [13.2, -0.1], atol=3e-4)
        self.assertAlmostEqual(result[0][0].rectangle["heading_rad"], 0.3, delta=3e-4)

    def testCurrentGeometryDeterminesSupportWhenSeedAndFitSeeDifferentAxes(self):
        points = rectangle_points([12.5, 0.2], 0.1)
        _, fit = fit_rectangle_pose(points, np.asarray([12.7, 0.3, -0.2]), 5.0, 1.9,
                                    support_quantile=0.0)
        # At the observed pose both full axes have support, whatever axis the seed suggested.
        np.testing.assert_allclose(fit["symmetry_weights"], [1.0, 1.0], atol=1e-3)
        self.assertAlmostEqual(fit["heading_rad"], 0.1, delta=3e-4)

    def testTheCliRequiresEstimatorWiringAndClosesItOnFailure(self):
        base = ["--dataset", ".", "--model", "unused.pt"]
        with self.assertRaisesRegex(SystemExit, "estimator-factory"):
            perception.main(base)
        backend = PoseFeedback()
        closed = []
        backend.close = lambda: closed.append(True)
        module = types.SimpleNamespace(create=lambda args: backend)
        with patch.object(perception.importlib, "import_module", return_value=module), \
                patch.object(perception, "run_pipeline", side_effect=RuntimeError("sensor failed")), \
                self.assertRaisesRegex(RuntimeError, "sensor failed"):
            perception.main(base + ["--estimator-factory", "test_adapter:create"])
        self.assertEqual(closed, [True])

    def testDefaultPipelineRefinesThePredictedPoseWithoutSelectingAMethod(self):
        transform = {"matrix": np.eye(4).tolist()}
        metadata = {"sensors": {"cameras": [{"id": "front", "width": 100, "height": 100,
                    "fov": 90, "transform": transform}], "lidar": {"transform": transform}}}
        frame = {"frame": 0, "time": 0.0, "ego_transform": transform,
                 "sensors": {"lidar": {"transform": transform, "path": "unused.npy"},
                             "cameras": {"front": {"path": "unused.png"}}}}
        points = rectangle_points([12.5, 0.2], 0.1)
        detector = types.SimpleNamespace(detect=lambda *args: [self.det])
        backend = PoseFeedback()
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(perception, "load_json", return_value=metadata), \
                patch.object(perception, "iter_jsonl", return_value=[frame]), \
                patch.object(perception, "load_lidar_points", return_value=points), \
                patch.object(perception, "associate_detection_points", return_value=points), \
                patch.object(perception, "UltralyticsCarDetector", return_value=detector), \
                contextlib.redirect_stdout(io.StringIO()):
            self.args.output = Path(folder)
            summary = perception.run_pipeline(self.args, pose_feedback=backend)
        self.assertEqual(summary["tracking_method"], "predicted pose initialization")
        self.assertEqual(summary["valid_count"], 1)
        np.testing.assert_allclose(backend.events[-1][2], [12.5, 0.2], atol=3e-4)


if __name__ == "__main__":
    unittest.main()
