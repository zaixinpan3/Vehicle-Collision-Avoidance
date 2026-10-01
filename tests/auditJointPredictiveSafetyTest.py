"""Behavior checks for strict collision geometry with zero added buffer."""

import sys
import math
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from auditJointPredictiveSafety import clearance_passes, distance, rectangle, target_state, feasible_hold, audit, recovery_completed


class CollisionAuditTest(unittest.TestCase):
    def test_affine_result_needs_both_completed_stages_and_its_declared_tolerance(self):
        hold = affine_hold()
        self.assertTrue(feasible_hold(hold))
        hold['solverStages'][1]['exitFlag'] = 0
        self.assertFalse(feasible_hold(hold))
        hold = affine_hold()
        hold['hardResidual'] = 2e-5
        self.assertFalse(feasible_hold(hold))

    def test_affine_tolerance_does_not_relax_independent_collision_geometry(self):
        result = road_departure_result()
        result['roadConstraintsEnforced'] = False
        result['trace'][0].update(affine_hold())
        self.assertTrue(audit(result)['passed'])
        result['targetInitialState'][0] = 0
        result['minimumReplayClearanceMeters'] = 0
        self.assertFalse(audit(result)['passed'])

    def test_positive_optimized_slack_is_not_a_zero_slack_claim(self):
        result = road_departure_result()
        result['roadConstraintsEnforced'] = False
        result['trace'][0].update(affine_hold())
        result['trace'][0]['predictiveBarrierValue'] = .1
        checked = audit(result)
        self.assertTrue(checked['everyHoldFeasible'])
        self.assertFalse(checked['allPredictionsZeroSlack'])
        self.assertFalse(checked['passed'])

    def test_recovery_needs_sustained_small_errors_in_every_coordinate(self):
        result = dict(sampleTimeSeconds=.05, recovery=dict(enabled=True, recovered=True,
                      dwellSeconds=.1, minimumTimeSeconds=0, tolerances=[.1]*5),
                      trace=[dict(time=t, transverseError=[.01]*5) for t in (0, .05, .1)])
        self.assertTrue(recovery_completed(result))
        result['trace'][1]['transverseError'][2] = .2
        self.assertFalse(recovery_completed(result))

    def test_recovery_does_not_count_time_before_the_declared_minimum(self):
        result = dict(sampleTimeSeconds=.05, recovery=dict(enabled=True, recovered=True,
                      dwellSeconds=.1, minimumTimeSeconds=.1, tolerances=[.1]*5),
                      trace=[dict(time=t, transverseError=[.01]*5) for t in (0, .05, .1)])
        self.assertFalse(recovery_completed(result))

    def test_initialization_failure_is_a_failed_audit_without_geometry_claims(self):
        result = dict(scenario='oncoming', targetInitialState=[], trace=[], completed=False,
                      executedFrames=0, requestedFrames=2, minimumReplayClearanceMeters=None,
                      requiredClearanceMeters=0, finalTransverseError=[],
                      configuration=dict(vehicle=dict(length=4.8, width=1.9, rectangleOffset=[0, 0])))
        checked = audit(result)
        self.assertFalse(checked['passed'])
        self.assertFalse(checked['strictlyCollisionFree'])
        self.assertFalse(checked['everyHoldFeasible'])
        self.assertIsNone(checked['minimumClearanceMeters'])
        self.assertIsNone(checked['minimumRoadMarginMeters'])
        self.assertIsNone(checked['finalLateralErrorMeters'])

    def test_retained_witness_does_not_require_a_new_solve(self):
        self.assertTrue(feasible_hold(dict(hardResidual=0, source='retainedContinuation', solverCalls=0)))

    def test_nonlinear_constraint_correction_requires_an_admitted_witness(self):
        hold = dict(hardResidual=0, source='nonlinearConstraintCorrection', solverCalls=0)
        self.assertTrue(feasible_hold(hold))
        hold['hardResidual'] = 1e-12
        self.assertFalse(feasible_hold(hold))

    def test_feasible_witness_can_finish_without_invoking_an_optimizer(self):
        hold = dict(time=0, auditTimes=[0], auditStates=[[0, 0, 0, 8, 0, 0]],
                    predictiveBarrierValue=0, hardResidual=0, source='feasibleInitialization',
                    solverCalls=0, scvxConverged=True)
        result = dict(scenario='recovery', targetInitialState=[], trace=[hold], completed=True,
                      executedFrames=1, requestedFrames=1, minimumReplayClearanceMeters=None,
                      requiredClearanceMeters=.005, finalTransverseError=[0],
                      configuration=dict(vehicle=dict(length=4.8, width=1.9, rectangleOffset=[0, 0])))
        checked = audit(result)
        self.assertTrue(checked['passed'])
        self.assertTrue(checked['everyHoldConverged'])
        self.assertFalse(checked['everyHoldInvokedOptimizer'])

    def test_small_positive_hard_residual_is_not_feasible(self):
        self.assertFalse(feasible_hold(dict(hardResidual=1e-12, source='sequentialConvexification', solverCalls=2)))

    def test_road_departure_is_diagnostic_when_road_constraints_are_absent(self):
        result = road_departure_result()
        result['roadConstraintsEnforced'] = False
        checked = audit(result)
        self.assertTrue(checked['passed'])
        self.assertTrue(checked['strictlyCollisionFree'])
        self.assertLess(checked['minimumRoadMarginMeters'], 0)
        self.assertFalse(checked['roadBoundarySatisfied'])
        self.assertFalse(checked['roadConstraintsEnforced'])

    def test_historical_exports_still_require_their_road_constraint(self):
        checked = audit(road_departure_result())
        self.assertFalse(checked['passed'])
        self.assertTrue(checked['roadConstraintsEnforced'])

    def test_removing_road_constraints_does_not_accept_a_collision(self):
        result = road_departure_result()
        result['roadConstraintsEnforced'] = False
        result['targetInitialState'][0] = 0
        result['minimumReplayClearanceMeters'] = 0
        checked = audit(result)
        self.assertFalse(checked['passed'])
        self.assertFalse(checked['strictlyCollisionFree'])

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


def affine_hold():
    return dict(source='twoStageConvexOptimization', solverCalls=2, hardResidual=1e-9,
                affineFeasibilityTolerance=1e-5, optimizationConverged=True,
                predictiveBarrierValue=1e-7,
                solverStages=[dict(objective='pcbfSlack', exitFlag=1), dict(objective='clfSlack', exitFlag=1)])


def road_departure_result():
    hold = dict(time=0, auditTimes=[0], auditStates=[[0, 5, 0, 8, 0, 0]],
                predictiveBarrierValue=0, hardResidual=0, source='feasibleInitialization',
                solverCalls=0, scvxConverged=True)
    return dict(scenario='oncoming', targetInitialState=[20, 5, 0, 8, 0, 0, 1.6, 2.4, .95, 0, 0],
                trace=[hold], completed=True, executedFrames=1, requestedFrames=1,
                minimumReplayClearanceMeters=15.2, requiredClearanceMeters=.006,
                finalTransverseError=[5],
                configuration=dict(vehicle=dict(length=4.8, width=1.9, rectangleOffset=[0, 0])))


if __name__ == '__main__':
    unittest.main()
