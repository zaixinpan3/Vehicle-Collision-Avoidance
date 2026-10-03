"""Lower-body fitting keeps interior upper returns out of the edge objective."""
from pathlib import Path
import math
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "perception"))
from rectangle_pose_fit import fit_rectangle_pose


def rotation(heading):
    return np.array([[math.cos(heading), -math.sin(heading)],
                     [math.sin(heading), math.cos(heading)]])


def surface_cloud(center, heading, upper_repetitions=1):
    rear = np.column_stack([np.full(51, -2.5), np.linspace(-0.95, 0.95, 51)])
    side = np.column_stack([np.linspace(-2.5, 2.5, 81), np.full(81, -0.95)])
    faces = np.column_stack([np.vstack([rear, side]), np.full(132, 0.4)])
    # Dense asymmetric interior returns would bias an all-points edge fit.
    x, y = np.meshgrid(np.linspace(-2.0, 0.3, 17), np.linspace(-0.6, 0.1, 13))
    upper = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, 1.4)])
    cloud = np.vstack([faces, np.tile(upper, (upper_repetitions, 1))])
    cloud[:, :2] = cloud[:, :2] @ rotation(heading).T + center
    return cloud


class YoloLidarSurfaceFitTest(unittest.TestCase):
    def testInteriorUpperSurfacesDoNotPullTheRectangleOntoTheRoof(self):
        center, heading = np.array([9.0, -0.3]), 0.25
        for density in [1, 4]:
            for offset in [[0.3, -0.2, 0.12], [-0.2, 0.3, -0.1]]:
                cloud = surface_cloud(center, heading, density)
                with self.subTest(density=density, offset=offset):
                    result, fit = fit_rectangle_pose(
                        cloud, np.r_[center, heading] + offset, 5.0, 1.9, 0.0)
                    np.testing.assert_allclose(result, center, atol=3e-4)
                    self.assertAlmostEqual(fit["heading_rad"], heading, delta=3e-4)

    def testUpperReturnsStillConstrainContainment(self):
        rear = np.column_stack([np.full(61, 10.0), np.linspace(-0.95, 0.95, 61),
                                np.full(61, 0.4)])
        upper = np.column_stack([np.full(21, 15.4), np.linspace(-0.95, 0.95, 21),
                                 np.full(21, 1.4)])
        result, fit = fit_rectangle_pose(np.vstack([rear, upper]),
                                         np.array([12.5, 0.0, 0.0]), 5.0, 1.9, 0.0)
        # The inconsistent footprint cannot contain both ends: lower returns
        # also resist displacement through their own containment residuals.
        self.assertGreater(result[0], 12.55)
        self.assertLess(fit["maximum_containment_violation_m"], 0.35)
        self.assertLess(fit["objective"], fit["seed_objective"])

    def testPlanarRigidMotionAndHeightDatumDoNotChangeTheFit(self):
        cloud = surface_cloud(np.array([9.0, -0.3]), 0.25)
        seed = np.array([9.3, -0.5, 0.37])
        result, fit = fit_rectangle_pose(cloud, seed, 5.0, 1.9, 0.0)
        angle, shift = 1.8, np.array([-4.0, 6.0])
        transformed = cloud.copy()
        transformed[:, :2] = cloud[:, :2] @ rotation(angle).T + shift
        transformed[:, 2] += 7.0
        moved_seed = np.r_[seed[:2] @ rotation(angle).T + shift, seed[2] + angle]
        moved, moved_fit = fit_rectangle_pose(transformed, moved_seed, 5.0, 1.9, 0.0)
        np.testing.assert_allclose(moved, result @ rotation(angle).T + shift, atol=3e-4)
        error = moved_fit["heading_rad"] - fit["heading_rad"] - angle
        self.assertAlmostEqual(math.atan2(math.sin(error), math.cos(error)), 0.0, delta=3e-4)

    def testFullSideSupportCanRecoverPositionAndYawWithInteriorUpperReturns(self):
        center, heading = np.array([8.0, 2.0]), -1.1
        side = np.column_stack([np.linspace(-2.5, 2.5, 81), np.full(81, -0.95),
                                np.full(81, 0.4)])
        x, y = np.meshgrid(np.linspace(-2.0, 2.0, 15), np.linspace(-0.6, 0.5, 11))
        upper = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, 1.4)])
        cloud = np.vstack([side, upper])
        cloud[:, :2] = cloud[:, :2] @ rotation(heading).T + center
        result, fit = fit_rectangle_pose(cloud, np.r_[center + [0.2, 0.3], heading + 0.1],
                                         5.0, 1.9, 0.0)
        np.testing.assert_allclose(result, center, atol=3e-4)
        self.assertAlmostEqual(fit["heading_rad"], heading, delta=3e-4)

    def testInteriorUpperExtentDoesNotInventLowerFaceTangentSupport(self):
        rear = np.column_stack([np.full(31, 10.0), np.linspace(-0.2, 0.2, 31),
                                np.full(31, 0.4)])
        upper = np.column_stack([np.full(31, 11.0), np.linspace(-0.3, 0.3, 31),
                                 np.full(31, 1.4)])
        for y in [-0.3, 0.3]:
            with self.subTest(seedY=y):
                result, _ = fit_rectangle_pose(np.vstack([rear, upper]),
                                               np.array([12.5, y, 0.0]), 5.0, 1.9)
                np.testing.assert_allclose(result, [12.5, y], atol=1e-8)

    def testNonfiniteHeightAndMissingHeightAreRejected(self):
        cloud = surface_cloud(np.array([9.0, -0.3]), 0.25)
        invalid = cloud.copy()
        invalid[0, 2] = np.nan
        for points in [cloud[:, :2], invalid, cloud[0]]:
            with self.subTest(shape=points.shape), self.assertRaisesRegex(ValueError, "XYZ"):
                fit_rectangle_pose(points, np.array([9.0, -0.3, 0.25]), 5.0, 1.9)


if __name__ == "__main__":
    unittest.main()
