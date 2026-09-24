# Why the admission SOCP is infeasible: constraint-relaxation diagnosis, 2026-09-23

Question: the 118-case declared-plant sweep has 11 cases in which Clarabel reports
the fixed-direction trajectory SOCP primal infeasible (status 2) and the
controller issues no command. Does that mean no physical avoidance path exists?

Answer: no. In none of the 11 cases does the infeasibility come from collision
avoidance being physically impossible for the actual target. Eight cases are
blocked by the curved-road chart boxes, which act as a trust region around the
initializer's seed trajectory. Three cases are blocked by robust requirements
under ten-fold declared uncertainty, where the uncertainty is propagated open
loop along the plan. Measured on the declared held affine plant only; this is a
diagnostic, not a proof, and no controller code was changed.

## Method

Source revision `4f30d429299d4d56bc7c840cb38c33dd98e2c27a` (controller identical
to `0b0855f`), in a scratch worktree. A one-line capture
([modelCapture.patch](ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923/modelCapture.patch))
stores the frame `model` before the solve. Each failing sweep case was replayed
with `runExactStateRecursiveFeasibilityScenario` (same options as
`runDeclaredPlantFailureSweep`). The failing frame was rebuilt with the
controller's own `formulateAvoidanceProblem`, `solveHardCbfClf.fluidInitialize`
and `avoidanceStageQp.fixedDirections`, and solved with the same native Clarabel
bridge. Every replayed baseline reproduces status 2, and a passing reference
case (stationary, curvature 0.01 /m, zero error) reproduces as feasible.

Constraint groups were relaxed by raising their bounds by 1e3 (rows, cones) or
by removing their support records:

| Group | Meaning |
| --- | --- |
| `collision` | ego-target support certificates at every hold node |
| `exit` | at the final node the ego is at least the 16 m confirmation range beyond the target |
| `terminalCone` | robust membership of the final uncertainty box in the modal invariant cruise set |
| `terminalPoseDomain` | chart-validity box at the final node |
| `poseDomain` | chart-validity box at every node: seed pose +- `poseTrustRadius` = [2 m station; 4 m lateral; 0.5 rad heading], plus the uncertainty radius |

Actuator amplitude and slew rows were never relaxed; the CLF is already soft.
`terminalEntry` and `referencePhaseDomain` rows are absent in these frames. All
2^5 subsets were solved per case; a minimal relaxation is a feasible subset with
no feasible proper subset. Relaxation only enlarges the feasible set, so every
superset of a minimal relaxation is feasible. Results:
[minimalRelaxations.csv](ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923/minimalRelaxations.csv).

## Results

| Case | Horizon | Minimal relaxations |
| --- | --- | --- |
| 27 crossing, curvature -0.02 | 5.60 s | `poseDomain`, or `collision` |
| 48 oncoming, 0.01, uncertainty x10 (t = 2.6 s) | 3.20 s | `poseDomain`, or `collision` |
| 53 crossing, 0.01, uncertainty x3 | 4.80 s | `poseDomain`, or `collision` |
| 94 stationary, 0.01, initial lateral 2 m | 4.80 s | `poseDomain`, or `collision` |
| 96 stationary, 0.01, initial heading 0.1 rad | 4.80 s | `poseDomain`, or `collision` |
| 97 stationary, 0.01, initial heading 0.2 rad | 4.80 s | `terminalCone`, or `terminalPoseDomain+poseDomain` |
| 98 stationary, 0.01, initial heading 0.3 rad | 4.80 s | `terminalCone`, or `terminalPoseDomain+poseDomain` |
| 99 stationary, 0.01, initial speed -2 m/s | 4.80 s | `exit`, or `terminalPoseDomain+poseDomain` |
| 39 stationary, straight, uncertainty x10 | 4.80 s | `terminalCone` |
| 42 stationary, 0.01, uncertainty x10 | 4.80 s | three sets, each containing `terminalCone` |
| 54 crossing, 0.01, uncertainty x10 | 4.80 s | `terminalCone+collision` |

### Class A (8 cases): the chart boxes are a trust region around the seed

In cases 27, 48, 53, 94, 96, 97, 98 and 99, removing only the chart boxes
(`poseDomain`, plus the final-node box in 97-99) makes the complete problem
feasible. That includes every collision certificate, the exit requirement, the
terminal invariant set and all actuator and slew limits. A trajectory that
satisfies all of the physical and certificate requirements therefore exists for
the model. It lies outside the boxes centered on the fluid initializer's seed.
On curved roads the affine Frenet-to-Cartesian chart is only certified inside
those boxes. The fixed-direction SOCP is a single convex step, and the boxes are
never re-centered on a better trajectory, so a poor seed makes the step
infeasible.

For case 27, the other passing side of the fluid reference is feasible without
any relaxation, so the initializer picked the wrong side. In cases 48, 53, 94,
96 and 99 the other side is also infeasible. In cases 97 and 98 the initializer
detects no support conflict at the nominal, so it offers no second side. In the
exploratory run ([diagnosis.txt](ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923/diagnosis.txt)),
directions recomputed from a trajectory feasible without the terminal
requirements did not restore feasibility in any case where this was tried
(39, 97, 98, 99). A 1.6 s longer horizon restored feasibility only in case 99,
where the slower ego needs more time to reach the exit distance.

### Class B (3 cases): robust requirements under ten-fold uncertainty

The declared estimation uncertainty is propagated along the plan by
`rho_k = |A_k| rho_{k-1}`. The plan is an open-loop input sequence with no
feedback term, so the box grows over the horizon. The terminal certificate
requires the whole final box to lie inside the fixed-size modal cruise set
([uncertainty.txt](ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923/uncertainty.txt)):

| Case | Final lateral / heading radius | Terminal margin with the center exactly on the reference |
| --- | --- | --- |
| 39 stationary, straight, x10 | 0.516 m / 0.0107 rad | -0.0296 |
| 42 and 54, curvature 0.01, x10 | 0.737 m / 0.0186 rad | -0.1570 |
| 53 crossing, 0.01, x3 | 0.221 m / 0.0056 rad | +0.0745 |
| crossing, 0.01, x1 (passes) | 0.074 m / 0.0019 rad | +0.1406 |

With ten-fold uncertainty the box does not fit the terminal set even in the best
case, so no input sequence can satisfy `terminalCone`, with or without the
obstacle. In case 54 the collision certificates are also infeasible on their
own. The target's declared reachable box (`targetPrediction.finiteFlow`) has a
half-width per axis of `r_p + r_v t + r_a t^2/2 + J t^3/6`. With the ten-fold
bounds (1.0 m, 0.5 m/s, 0.1 m/s^2, jerk 0.2 m/s^3) this reaches 8.24 m per axis
at the 4.8 s horizon: 1.0 m current position error, 2.40 m velocity error,
1.15 m acceleration error and 3.69 m from the jerk bound. No single input
sequence avoids every target motion inside that box. The true target in this
case deviates about 1.2 m per axis from its constant-velocity path over 4.8 s,
far less than the box; the certificate demands robustness against every motion
the inflated bounds admit.

Correction (2026-09-23): an earlier version of this paragraph said the target
uncertainty grows from 2.5 m to 18.2 m radius and that the true target moves on
a straight line. The 2.5/18.2 m figures in
[uncertainty.txt](ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923/uncertainty.txt) are
the sum of all generator lengths of the ego-target relative-position record
(both target box axes plus the then open-loop ego part), not a target radius.
The true target oscillates with the declared jerk amplitude.

Case 53 (x3) belongs to Class A: its terminal box still fits (+0.0745), and
only the chart boxes bind.

## Implications (not implemented)

- Class A would be addressed by re-centering the chart boxes on the latest
  solution and re-solving (a sequential convex trust-region update), by trying
  both passing sides, or by larger `poseTrustRadius` values at the cost of
  larger chart remainders.
- Class B follows from the open-loop robust formulation: the terminal and
  collision certificates must hold for the uncertainty box grown without
  feedback. Remedies are a feedback (tube) parameterization in the prediction,
  smaller declared bounds, or a terminal condition that does not require the
  whole open-loop box.

## Reproduction

Apply `modelCapture.patch` to a scratch worktree of `4f30d42` (never to the
repository), link `solver/`, then from the worktree root run
`diagnoseInfeasibleAdmission(<dir>)`, `diagnoseRelaxationSubsets(<dir>)` and
`checkUncertaintyCases(<dir>)` from
[ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923/](ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923/).
Logs of the recorded runs are in the same folder.
