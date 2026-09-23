# Controller validation rerun after deleting post-solve verification, 2026-09-23

Purpose: re-execute the controller simulation experiments after commit
`0b0855f408a1e1ed0dffa69b954e4bb1f1e4cc40` ("Delete the post-solve verification
code") and check whether the controller still runs as before. That commit issues
a solver success as returned, removes every post-solve check, and on an
inherited frame issues the shifted previous plan without any check when the
solve fails. This record reports measured outcomes only; it establishes no
real-time or physical-safety guarantee.

Environment: MATLAB R2026a, `matlab -batch`, a clean detached worktree of
`0b0855f` (the main working tree also holds uncommitted edits from another
session; they were not part of these runs). Each experiment ran in its own fresh
MATLAB process, one at a time. Unrelated MATLAB desktop sessions of other
projects were running on the machine, so frame timings are not isolated
measurements. Raw outputs are under the git-ignored
`simulation_output/controller_rerun_20260923/`.

Baselines: [CONTROLLER_VALIDATION_RERUN_20260922.md](CONTROLLER_VALIDATION_RERUN_20260922.md)
and [CONTROLLER_FAILURE_MODE_SWEEP_20260922.md](CONTROLLER_FAILURE_MODE_SWEEP_20260922.md),
both recorded before the verification change.

## 1. High-fidelity PassVeh14DOF scenarios: 4 of 4 fail, unchanged

`Plot=false`, `Report=true`, otherwise default options.

| Scenario | Frames attempted | Failure | Max frame |
| --- | --- | --- | --- |
| `runStraightCenterlineCruiseScenario` | 2 | `inconsistentObservation` at t = 0.050 s | 640.5 ms |
| `runCircularCenterlineCruiseScenario` | 2 | `inconsistentObservation` at t = 0.050 s | 597.2 ms |
| `runOncomingVehicleAvoidanceScenario` | 1 | `optimizationFailed` at t = 0 | 363.4 ms |
| `runCircularCenterlineStraightTargetAvoidanceScenario` | 1 | `optimizationFailed` at t = 0 | 349.9 ms |

The causes are those recorded on 2026-09-22 and do not involve the deleted
code. The cruise scenarios stop on "The ego measurement box does not intersect
the published successor box" (zero-residual plant contract against the 14-DOF
plant). The avoidance scenarios stop on "The recursive cruise certificate
requires an unbounded road-free reference domain" (road boundaries). Both are
raised before or outside any solve.

## 2. Estimator-in-the-loop suite: fails at t = 0, unchanged cause

`runEstimatedStateAvoidanceScenarios` with default options returns
`allScenariosPassed = 0`. Both the straight and the circular scenario stop at
t = 0 with `collisionAvoidanceController:nonexactStudyInput` ("This controller
requires the declared zero-residual held affine plant"): the suite's controller
configuration declares a nonzero model-residual bound, which the controller
rejects before any solve. This matches the blocked status recorded in
[ESTIMATOR_IN_THE_LOOP_ADMISSION_20260917.md](ESTIMATOR_IN_THE_LOOP_ADMISSION_20260917.md).

## 3. Declared held affine plant: 5 of 5 cases pass, unchanged

`runRecursiveSafetyValidation` at default scale (600 holds, 0.05 s, seed 20260912,
road boundaries disabled).

| Case | Holds | Min separation margin | Min terminal margin | Max CLF residual | Median / max frame | Deadline misses |
| --- | --- | --- | --- | --- | --- | --- |
| stationary | 600 / 600 | 9.2788e-05 m | 0.1444 | -5.6297e-06 | 7.42 / 878.70 ms | 3 |
| oncoming | 600 / 600 | 0.0381 m | 0.0222 | -5.6348e-06 | 7.43 / 72.56 ms | 1 |
| crossing | 600 / 600 | 9.5197 m | 0.1823 | -5.7592e-06 | 7.29 / 18.02 ms | 0 |
| cruise | 600 / 600 | Inf (no target) | 0.1824 | -6.4780e-06 | 7.13 / 17.12 ms | 0 |
| uncertainCrossing | 600 / 600 | 9.5196 m | 0.1734 | -1.0745e-05 | 7.30 / 27.40 ms | 0 |

Every case passes, collision-free, with the CLF decrease satisfied. Relative to
2026-09-22 the stationary minimum gap moved from 9.2775e-05 to 9.2788e-05 m;
all other listed values agree to the printed precision. The oncoming case still
leaves the declared heading domain (`minimumModelDomainMargin = -0.155`), and
the stationary clearance is still only about 0.09 mm.

Enforced 50 ms deadline (120 holds): stationary 0 / 120 (work deadline expired
before the first native solve), oncoming 52 / 120 (frame deadline exceeded at
t = 2.6 s), crossing 120 / 120. Identical hold counts to 2026-09-22.

## 4. Failure-mode sweep: 118 of 118 cases reproduce the baseline

`runDeclaredPlantFailureSweep` (240 holds per case, deadline disabled). The
per-case table is
[CONTROLLER_FAILURE_MODE_SWEEP_20260923.csv](CONTROLLER_FAILURE_MODE_SWEEP_20260923.csv).

| Outcome | 2026-09-22 | 2026-09-23 |
| --- | --- | --- |
| Passed | 95 | 95 |
| Controller error | 15 | 15 |
| Silent inter-node overlap | 8 | 8 |

Case by case, every case has the same outcome class, executed hold count and
failure identifier, and all 15 failure messages are byte-identical. The largest
difference in any finite node gap, sampled gap, terminal margin or CLF residual
is 1.6e-06 (case 37, terminal margin). The 11 admission failures are Clarabel
itself reporting the fixed-direction SOCP infeasible (`solverRejected`), so the
deleted post-solve code could not have changed them.

## 5. Did the removed checks matter in these runs?

Across all 126 declared-plant runs (5 recursion, 3 deadline, 118 sweep;
27,944 executed holds):

- The new unchecked fallback (retain the shifted previous plan after a failed
  inherited solve) was never triggered: `usedCertifiedIncumbent` is false on
  every frame.
- No AlmostSolved result was issued (`approximateSolveAccepted` false
  throughout).
- The offline joint-separation residual of every issued plan is negative; its
  maximum is -6.34e-05, inside the admission reserve.
- On inherited frames, the shifted previous plan satisfied the physical rows
  (maximum residual -9.58e-06) and the joint residual (maximum -6.34e-05).

So the deleted checks would not have rejected any plan in these runs, which is
why the outcomes are unchanged. This is a finite-sample observation. It does
not show that the checks are never needed: a false solver success, a
reduced-accuracy result, or a failed inherited solve with an infeasible
previous plan would now be issued without any check.

## 6. Conclusion

After deleting the post-solve verification, the controller behaves exactly as
before on every simulation experiment that was run. On its declared plant,
the recursion campaign passes all five cases and the 118-case sweep reproduces
all outcomes. The PassVeh14DOF, estimator-in-the-loop and 50 ms deadline results
keep the same failures, caused by the zero-residual plant contract, road
boundaries and runtime, not by the verification change.
