"""Behavior checks of exact joint mismatch and terminal specializations."""

from fractions import Fraction as F
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import checkJointMismatchDerivations as derivation


class JointMismatchDerivationsTest(unittest.TestCase):
    def testOnePhysicalMismatchPreservesTheObserverUpdateIdentity(self):
        _, _, _, shared, independent = derivation.jointLinearIdentity()
        self.assertEqual(shared, 0)
        self.assertGreater(independent, shared)

    def testFiniteSupportsHaveOneFixedParameterAndAnAttainingNoiseHistory(self):
        radius, inputRadius = derivation.finiteJointSupports()
        self.assertGreater(radius, 0)
        self.assertGreater(inputRadius, 0)

    def testFreshParameterDrawsCanLoseAcrossTimeCancellation(self):
        fixed, fresh = derivation.persistentParameterSupport()
        self.assertEqual(fresh, 3 * fixed)

    def testAValidatedNonlinearReserveCoversAFalseLinearOnlyBound(self):
        linear, reserve, actual = derivation.nonlinearReserve()
        self.assertGreater(actual, linear)
        self.assertLessEqual(actual, linear + reserve)

    def testOldCertificatesSurviveAValidButWiderFreshApproximation(self):
        posterior, carried = derivation.carryCertificates()
        self.assertLessEqual(posterior[0], posterior[1])
        self.assertLessEqual(carried[0], carried[1])

    def testOneBrakeRuleStopsAllAllowedSpeedsAndRatesWithinItsBound(self):
        time, holds, margin = derivation.commonBrakingCertificate()
        self.assertEqual(time, F(85, 38))
        self.assertEqual(holds, 12)
        self.assertGreater(margin, 0)

    def testSpecializationsDoNotClaimAFullVehicleCertificate(self):
        result = derivation.runChecks()
        self.assertFalse(result["fullVehicleNonlinearCertificate"])


if __name__ == "__main__":
    unittest.main()
