"""Behavior tests for persistent single-camera target identity selection."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'perception'))
import yolo_lidar_target_position as perception


class YoloLidarAssociationTest(unittest.TestCase):
    @staticmethod
    def car(x, confidence=.8, camera='front'):
        return perception.Detection(camera, (x, 0., x+20., 20.), confidence, 2, 'car')

    def testHigherConfidenceDistractorDoesNotReplaceOverlappingTarget(self):
        selector = perception.PersistentTargetSelector()
        first = self.car(0.)
        self.assertEqual(selector.select([first, self.car(50., .7)], 0.), [first])
        continued = self.car(1., .4)
        self.assertEqual(selector.select([self.car(50., .99), continued], .02), [continued])

    def testShortMissingIntervalRetainsIdentityAndReacquiresByOverlap(self):
        selector = perception.PersistentTargetSelector()
        selector.select([self.car(0.)], 0.)
        self.assertEqual(selector.select([], .1), [])
        self.assertEqual(selector.select([self.car(80., .99)], .2), [])
        continued = self.car(3., .4)
        self.assertEqual(selector.select([continued, self.car(80., .99)], .32), [continued])

    def testExpiredTrackNeverSilentlyReacquiresTheSameOrAnotherBox(self):
        selector = perception.PersistentTargetSelector(maximum_gap=.5)
        selector.select([self.car(0.)], 0.)
        self.assertEqual(selector.select([self.car(0.)], .6), [])
        self.assertEqual(selector.select([self.car(80., .99)], .7), [])
        self.assertEqual(selector.select([self.car(0.)], .8), [])

    def testMissingStartupDoesNotPreventFirstAcquisition(self):
        selector = perception.PersistentTargetSelector()
        self.assertEqual(selector.select([], 0.), [])
        first = self.car(0.)
        self.assertEqual(selector.select([first], 10.), [first])

    def testBoxesFromAnotherCameraCannotMatchTheTrack(self):
        selector = perception.PersistentTargetSelector()
        selector.select([self.car(0.)], 0.)
        self.assertEqual(selector.select([self.car(0., camera='rear')], .02), [])

    def testNonIncreasingAndNonfiniteTimestampsAreRejected(self):
        for invalid in [0., -.1, float('nan'), float('inf')]:
            with self.subTest(timestamp=invalid):
                selector = perception.PersistentTargetSelector()
                selector.select([self.car(0.)], 0.)
                with self.assertRaisesRegex(ValueError, 'strictly increasing'):
                    selector.select([], invalid)

    def testInvalidGatesAreRejected(self):
        for value in [0., -1., 1.1, float('nan'), float('inf')]:
            with self.subTest(iou=value), self.assertRaises(ValueError):
                perception.PersistentTargetSelector(minimum_iou=value)
        for value in [0., -1., float('nan'), float('inf')]:
            with self.subTest(gap=value), self.assertRaises(ValueError):
                perception.PersistentTargetSelector(maximum_gap=value)

    def testExistingConfidenceDefaultRequiresAnExplicitOptIn(self):
        base = ['--dataset', '.', '--model', 'unused.pt']
        self.assertEqual(perception.parse_args(base).target_association, 'confidence')
        self.assertEqual(perception.parse_args(base+['--target-association', 'persistent']).target_association, 'persistent')

    def testPersistentPipelineRejectsUnresolvedCrossCameraIdentity(self):
        args = perception.parse_args(['--dataset', '.', '--model', 'unused.pt',
                                      '--target-association', 'persistent'])
        metadata = {'sensors': {'cameras': [{'id': 'front'}, {'id': 'rear'}]}}
        with patch.object(perception, 'load_json', return_value=metadata), \
                patch.object(perception, 'iter_jsonl', return_value=[]), \
                self.assertRaisesRegex(ValueError, 'exactly one selected camera'):
            perception.run_pipeline(args, pose_feedback=SimpleNamespace(predict=lambda *x: None, update=lambda *x: None))


if __name__ == '__main__':
    unittest.main()
