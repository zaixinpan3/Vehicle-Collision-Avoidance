# Controller scenario rerun, 2026-09-24

Purpose: run every controller scenario driver in the repository on the current
working tree and record whether the controller completes each one. Measured
outcomes only; nothing here is a proof, a worst-case bound or a real-time claim.

**Code state.** `main` at `3073d577b6fe42317c5599e042ee03a41b316dc0` plus the
uncommitted working-tree edits of a concurrent session in seven files
(`config/collisionAvoidanceControllerConfig.m`, `controller/avoidanceSafetyGeometry.m`,
`controller/collisionAvoidanceController.m`, `controller/hardEncounterBarrier.m`,
`controller/solveHardCbfClf.m`, `scripts/runCircularCenterlineStraightTargetAvoidanceScenario.m`,
`scripts/runOncomingVehicleAvoidanceScenario.m`; `git diff` SHA-256
`8748c283f1723d4e381159b107cc5ec8f3839d3547d7fcb5dfa754718879c34d` before this
task's own edit). Those edits (a) admit an ego measurement that leaves the
predicted successor box from the measurement instead of raising
`inconsistentObservation`, (b) add a `RoadBoundaries` option to the two
avoidance drivers, (c) try the shifted previous plan's separation directions
before the fluid initialization on a conflicting fresh frame, and (d) add a
zero-valued `collision.safetyMarginMeters`. They were not written by this task
and are not committed by it. This task's only source change is the harness
correction in Section 4.

**Environment.** MATLAB R2026a, `matlab -batch`, one fresh process per run.
Up to nine MATLAB processes of this task ran concurrently, and MATLAB
sessions of other projects were running on the same machine, so every frame
time below is a loaded measurement, not an isolated one. Raw outputs are under
the git-ignored `simulation_output/controller_scenario_rerun_20260924/` and
the scratch logs are copied (as `.txt`) into
[CONTROLLER_SCENARIO_RERUN_20260924/](CONTROLLER_SCENARIO_RERUN_20260924/).

## 1. Summary

| Driver | Plant | Outcome |
| --- | --- | --- |
| `runStraightCenterlineCruiseScenario` | PassVeh14DOF | **PASS**, 80 / 80 steps |
| `runCircularCenterlineCruiseScenario` | PassVeh14DOF | **PASS**, 80 / 80 steps |
| `runOncomingVehicleAvoidanceScenario` (default, road boundaries) | PassVeh14DOF | FAIL at frame 1: road boundaries rejected |
| `runCircularCenterlineStraightTargetAvoidanceScenario` (default) | PassVeh14DOF | FAIL at frame 1: road boundaries rejected |
| `runOncomingVehicleAvoidanceScenario(RoadBoundaries=false)` | PassVeh14DOF | FAIL at t = 1.55 s: SOCP infeasible, 31 / 200 steps |
| `runCircularCenterlineStraightTargetAvoidanceScenario(RoadBoundaries=false)` | PassVeh14DOF | FAIL at t = 1.60 s: SOCP infeasible, 32 / 200 steps |
| `runEstimatedStateAvoidanceScenarios` (default) | PassVeh14DOF + NRMM estimator | FAIL at frame 1: nonzero residual bound rejected |
| same, zero-residual controller configuration | PassVeh14DOF + NRMM estimator | FAIL at frame 1: road boundaries rejected |
| straight and circular avoidance, estimator on, road-free (Section 4) | PassVeh14DOF + NRMM estimator | FAIL at the first radar frame (t = 3.55 s and 3.75 s): SOCP infeasible |
| `runRecursiveSafetyValidation` | declared held affine | **PASS**, 5 / 5 cases at 600 holds |
| same, enforced 50 ms deadline | declared held affine | 0 / 3 cases issue a first command (loaded machine) |
| `runDeclaredPlantFailureSweep` | declared held affine | 103 / 118 pass; 3 rows differ from the `3073d57` reference |
| `runtests('tests')` | | 810 / 810 pass, 0 failed, 0 incomplete |

The two cruise scenarios on the 14-DOF plant complete for the first time since
the zero-radius successor-box test was introduced (they stopped at the second
sample on 2026-09-22 and 2026-09-23). Every avoidance scenario on the 14-DOF
plant still fails, for three distinct reasons: the road-boundary rejection
(unchanged), an infeasible fixed-direction program at the close encounter with
truth targets (new, because the runs now get that far), and an infeasible
first-detection program with the estimator's first radar estimate (new for the
same reason).

## 2. PassVeh14DOF cruise scenarios: 2 of 2 pass

`Plot=false, Report=true`, default options (4 s, 80 steps, 15 m/s).

| Scenario | Steps | Mean / max solve | Max speed error | Max lateral error | Max heading error |
| --- | --- | --- | --- | --- | --- |
| straight | 80 / 80 | 27.0 / 1092.9 ms | 0.034 m/s | 0.000 m | 0.000 rad |
| circular, R = 100 m | 80 / 80 | 25.8 / 985.2 ms | 0.042 m/s | 0.006 m | 0.007 rad |

The maximum solve time is the cold first frame. On the 14-DOF plant the
measured ego state leaves the zero-radius predicted successor box at every
sample, so `metadata.readmittedAfterInconsistentObservation` is true on every
frame after the first and `inheritedFeasibleFamily` is false throughout: the
controller re-admits from the measurement at every hold and never carries a
certificate across frames. The cruise runs pass because a fresh cruise
admission is feasible at every sample; no recursive-feasibility claim is
exercised on this plant.

## 3. PassVeh14DOF avoidance scenarios: 0 of 4 pass

### 3.1 Default options: road boundaries rejected at frame 1

Both drivers pass `RoadBoundaryOffsets` by default and both fail at t = 0 with
`collisionAvoidanceController:optimizationFailed`, "The recursive cruise
certificate requires an unbounded road-free reference domain." This is the
outcome recorded on 2026-09-22 and 2026-09-23; the current terminal
certificate admits no road boundary.

### 3.2 `RoadBoundaries=false`: infeasible at the close encounter

Default geometry: ego 15 m/s, target 15 m/s, 50 m initial separation, 30 m
perception range, straight target lateral offset 1.5 m, circular R = 100 m
target constructed to collide at t = 1.675 s.

| Scenario | Target first seen | Failure | Steps | Min controlled SAT margin | Min ego speed | Min ego lateral | Mean / max solve | Frame median / max |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| straight oncoming | t = 0.70 s | t = 1.55 s | 31 / 200 | 0.193 m (at 1.55 s) | 14.907 m/s | -0.555 m | 76.4 / 935.7 ms | 37.4 / 943.3 ms |
| circular, straight target | t = 0.70 s | t = 1.60 s | 32 / 200 | 0.209 m (at 1.60 s) | 14.979 m/s | n/a | 76.7 / 972.7 ms | n/a |

Failure message in both: "Fixed-direction trajectory optimization failed:
solverRejected: solver exit flag -2: Clarabel status 2; previous-plan
directions rejected: solver exit flag -2: Clarabel status 2" (primal
infeasible on both the shifted previous plan's directions and the fluid
initialization). Per-frame metadata
([frameDiagnostics.m](CONTROLLER_SCENARIO_RERUN_20260924/frameDiagnostics.m)):
every frame after the first is re-admitted from the measurement
(`readmittedAfterInconsistentObservation = true`), no frame inherits a
certificate, and from the first target frame the nominal source is the shifted
previous solution. The ego reaches -0.445 m lateral offset at t = 1.50 s while
the no-control SAT margin is -0.5 m; the response starts 0.85 s before the
nominal collision and the program becomes infeasible one or two holds before
the closest approach. Both runs are collision-free up to the last issued
command (`collisionFree = 1`) but issue no command afterwards, so the scenario
fails.

## 4. Estimator-in-the-loop scenarios: 0 of 2 pass

### 4.1 The driver as shipped: rejected at frame 1

`runEstimatedStateAvoidanceScenarios` substitutes `finiteSensingValidationConfig`
(nonzero `plantModelResidualRateBound`) when no controller configuration is
given, so both scenarios fail at t = 0 with
`collisionAvoidanceController:nonexactStudyInput`. With a zero-residual
configuration (`horizonSteps = 32`, `optimalityTolerance = 1e-4`) both fail at
t = 0 with the road-boundary rejection of Section 3.1: the driver has no
`RoadBoundaries` option and always passes the offsets.

### 4.2 Harness correction

Run directly with the estimator on and road boundaries off, both scenarios
failed at t = 0.05 s with
`collisionAvoidanceController:executionContractViolation`, "The next timestamp
and issued held input are required." The estimator branch of
`scripts/runCenterlineCruiseScenario.m` took the controller state from the
adapter's ego estimate and never attached `heldActuatorInput`, while the
truth-state branch attaches the previous command. The branch now attaches it
in the same way (three lines; no controller, adapter or configuration file
changed). Logs of the runs before the correction are kept as
`estimator*_beforeHarnessFix.txt`.

### 4.3 After the correction: infeasible at the first radar frame

Parameters as in the driver: 18 s, ego and target 10 m/s, 100 m initial
range, straight target lateral offset 0.8 m, circular R = 60 m with a 300 m
arc after, target width 2 m, seeds 20260728 (straight) and 20260729
(circular), `estimatorControllerIntegrationConfig` with the driver's yaw-rate
domain adjustment
([estimatorStraightRoadFree.m](CONTROLLER_SCENARIO_RERUN_20260924/estimatorStraightRoadFree.m),
[estimatorCircularRoadFree.m](CONTROLLER_SCENARIO_RERUN_20260924/estimatorCircularRoadFree.m)).

| Scenario | Steps before failure | Failure | Range at failure | Re-admitted frames | Frame median / max | Mean / max solve |
| --- | --- | --- | --- | --- | --- | --- |
| straight oncoming | 71 / 360 | t = 3.55 s, first radar frame | 29.11 m | 1 of 71 | 21.9 / 178.4 ms | 11.2 / 32.5 ms |
| circular, straight target | 75 / 360 | t = 3.75 s, first radar frame | 29.14 m | 2 of 75 | n/a | 11.4 / 34.7 ms |

Before the target is seen the estimator's ego box stays inside the predicted
successor box on all but one or two frames, so on this path the controller
mostly conditions on the carried prediction (`nominalSource =
shiftedPreviousSolution`), though it still never inherits a certificate
(`inheritedFeasibleFamily = false` on every frame). The failing frame is the
first frame with a radar track. Its target estimate
([firstDetectionBounds.m](CONTROLLER_SCENARIO_RERUN_20260924/firstDetectionBounds.m))
carries contract `nrmm-motion-v1` with velocity initialization
`radialOncomingConfiguredSpeedPrior` and the following bounds (straight run;
the circular run is the same to two digits):

| Quantity | First-detection bound |
| --- | --- |
| relative position | 0.084 m (ego frame), 1.335 m (inertial) |
| inertial velocity per axis | [6.75, 9.96] m/s |
| speed | 30.04 m/s |
| course, yaw | 3.142 rad (unbounded) |
| curvature interval | [-Inf, Inf] |
| speed rate | 2.59 m/s^2 |

Under the box-only NRMM contract adopted in `3073d57` the target box is the
parameter-Taylor bound alone, so a course interval of the full circle and a
speed interval of ±30 m/s make the first-detection reachable set the
path-length ball, and no fixed-direction program is feasible. The message is
the same as in Section 3.2 (both direction seeds primal infeasible). Earlier
records could not reach this frame because the suite stopped at t = 0 or
t = 0.05 s; it is recorded here as the first measured outcome of the
estimator-in-the-loop path with the NRMM contract.

## 5. Declared held affine plant

### 5.1 Recursion campaign: 5 of 5 pass

`runRecursiveSafetyValidation`, 600 holds per case, 0.05 s, seed 20260912,
road boundaries disabled
([recursiveSafetySummary.json](CONTROLLER_SCENARIO_RERUN_20260924/recursiveSafetySummary.json)).

| Case | Holds | Min body gap | Min domain margin | Min terminal margin | Max CLF residual | Median / max frame |
| --- | --- | --- | --- | --- | --- | --- |
| stationary | 600 / 600 | 9.27e-05 m | 0.048 | 0.144 | -5.63e-06 | 24.3 / 1569.7 ms |
| oncoming | 600 / 600 | 0.0381 m | -0.155 | 0.022 | -5.63e-06 | 25.1 / 152.8 ms |
| crossing | 600 / 600 | 9.520 m | 0.400 | 0.182 | -5.76e-06 | 30.1 / 113.5 ms |
| cruise | 600 / 600 | no target | 0.400 | 0.182 | -6.48e-06 | 35.8 / 139.0 ms |
| uncertainCrossing | 600 / 600 | 9.521 m | 0.400 | 0.178 | -1.03e-05 | 52.7 / 190.8 ms |

Gaps, margins and residuals match the 2026-09-22 and 2026-09-23 records. The
three enforced 50 ms deadline runs (120 holds) all stop at t = 0: the
stationary fluid initialization hits its search limit (185.5 ms), and the
oncoming and crossing native solves miss the work deadline (85.2 and 71.4 ms).
On 2026-09-22 the crossing case completed 120 / 120 with an 18 ms maximum
frame; the difference is the machine load of this run, not a code change, and
no real-time conclusion follows.

### 5.2 Failure-mode sweep: 103 of 118 pass

`runDeclaredPlantFailureSweep`, 240 holds, deadline disabled, 30 s search
limit ([declaredPlantFailureSweep.csv](CONTROLLER_SCENARIO_RERUN_20260924/declaredPlantFailureSweep.csv)).
The 15 non-passing cases are 6 controller errors (cases 41 and 54, solver
primal infeasible; cases 101-104, the four road-boundary rejections) and 9
completed runs with a negative sampled body gap (cases 83, 84, 86, 87, 89, 90,
91, 96 and 115; inter-node overlaps between -0.05 mm and -27 mm that the node
certificate does not cover).

Compared with the `3073d57` reference sweep
(`simulation_output/nrmm_time_taylor_removal_20260924/regression/sweepDefault/`),
115 rows are identical in outcome, holds and gaps and 3 differ:

| Case | Group | Reference | This run |
| --- | --- | --- | --- |
| 41 | B-uncertainty, stationary, k = 0.01, x3 | pass, 240 holds, gap 0.390 m | infeasible at t = 1.9 s, 38 holds, gap 0.226 m |
| 48 | B-uncertainty, oncoming, k = 0.01, x3 | infeasible at hold 60 | pass, 240 holds, gap 0.522 m |
| 56 | C-targetJerk, oncoming, k = 0 | gap 0.241 / 0.208 m (node / sampled) | 0.238 / 0.237 m |

The uncommitted working-tree edits are the only code difference from the
reference, and the previous-plan direction seed is the one that changes which
fresh frames are feasible; the flips of cases 41 and 48 are attributed to
those edits but not analyzed here.

## 6. Tests

`runtests('tests')` on the working tree described above, after the harness
correction of Section 4.2: 810 tests, 810 passed, 0 failed, 0 incomplete (the
two curb-detection tests that earlier records list as assumption-filtered ran
in this session). The scenario tests that expect
`collisionAvoidanceController:nonexactStudyInput` under
`finiteSensingValidationConfig` still pass; the correction only affects runs
that reach the second sample with the estimator enabled.

## 7. Conclusion

On the plant the controller declares, the recursion campaign and the sweep
reproduce the recorded outcomes apart from two three-fold-uncertainty curved
cases that flip under the concurrent session's edits. On the PassVeh14DOF
plant the controller now completes both cruise scenarios by re-admitting from
the measurement at every hold, but no avoidance scenario completes: with road
boundaries it is rejected at the first frame; without them the fixed-direction
program becomes infeasible one or two holds before the closest approach with
truth targets, and at the first radar frame with the estimator's
first-detection target bounds. The last item is the first measurement of the
estimator-in-the-loop path under the NRMM target contract and points at the
first-detection speed and course bounds (30 m/s and the full circle) as the
quantity that makes the program infeasible.
