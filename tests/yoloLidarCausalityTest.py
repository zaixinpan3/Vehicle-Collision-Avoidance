"""Behavior checks for the optional causal target-motion perception path."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'perception'))
import yolo_lidar_target_position as perception


class YoloLidarCausalityTest(unittest.TestCase):
    def setUp(self):
        self.args = perception.parse_args(['--dataset', '.', '--model', 'unused.pt', '--causal'])
        self.args.motion_heading_minimum_displacement = .5
        self.args.heading_iterations = 2
        self.det = perception.Detection('front', (0., 0., 10., 10.), .9, 2, 'car')
        # Isolate temporal behavior from already-existing rectangle fitting.
        self.fitter = patch.object(perception, 'estimate_from_cluster', side_effect=self.fit)
        self.fitter.start()
        self.addCleanup(self.fitter.stop)

    @staticmethod
    def fit(detection, cluster, args, heading):
        return perception.CameraEstimate('front', detection, len(cluster), cluster[0, :2], {}, None)

    def records(self, count, moving=True):
        return [dict(time=i*.1, ego_yaw=0., ego_to_world=np.eye(4),
                     clusters=[(self.det, np.array([[20.+(i if moving else 0), 2., 1.]]))])
                for i in range(count)]

    def testFutureFramesCannotChangePreviouslyPublishedPositionsOrHeadings(self):
        prefix = self.records(7)
        short = perception.causal_track_estimates(prefix, self.args)
        suffix = self.records(12)[7:]
        for row in suffix:
            row['clusters'][0][1][:] = [1000., -1000., 1.]
        long = perception.causal_track_estimates(prefix+suffix, self.args)
        self.assertEqual(short[1], long[1][:7])
        for expected, actual in zip(short[0], long[0][:7]):
            np.testing.assert_array_equal(expected[0].relative_position, actual[0].relative_position)

    def testStationaryStartupIsNotBackfilledByLaterMotion(self):
        stationary = self.records(5, moving=False)
        result = perception.causal_track_estimates(stationary+self.records(12)[5:], self.args)
        self.assertEqual(result[1][:5], [None]*5)
        self.assertTrue(any(h is not None for h in result[1][5:]))

    def testCarriedHeadingRotatesWithTheCurrentEgoFrameDuringDropout(self):
        rows = self.records(5)
        rows.append(dict(time=.5, ego_yaw=np.pi/2, ego_to_world=np.eye(4), clusters=[]))
        result = perception.causal_track_estimates(rows, self.args)
        self.assertAlmostEqual(result[1][-1], -np.pi/2)
        self.assertEqual(result[0][-1], [])

    def testDuplicateTimestampsAreRejected(self):
        rows = self.records(3)
        rows[2]['time'] = rows[1]['time']
        with self.assertRaisesRegex(ValueError, 'strictly increasing'):
            perception.causal_track_estimates(rows, self.args)

    def testCausalityIsExplicitAndOfflineDefaultIsPreserved(self):
        self.assertTrue(self.args.causal)
        self.assertFalse(perception.parse_args(['--dataset', '.', '--model', 'unused.pt']).causal)


if __name__ == '__main__':
    unittest.main()
