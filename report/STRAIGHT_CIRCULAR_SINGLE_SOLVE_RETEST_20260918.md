# Straight and circular controller retest

Date: September 18, 2026. Tested commit: `730c765a73195b110c431bb2f625f70bc2221d12`.
Controller format 35; shifted nominal, fixed node normals, one convex solve.
This task changed no algorithm, constraint, solver setting, experiment driver,
or network configuration. It reran the existing scenarios and added a
parameterized initial-error recovery campaign through the same driver.

## Findings

The current controller tracks and recovers to straight and circular cruise in
the tested exact affine model. It does **not** complete general obstacle
avoidance across the tested references.

- Twenty baseline cases: **7 complete, 13 infeasible**; 6 satisfy combined
  completion, sampled-clearance and timing qualification. The completed
  straight stationary case violates the prescribed clearance between nodes.
- Five additional initially perturbed cruise cases: **5 complete and qualify**.
- All 3,730 strict-run issued commands used exactly one solver call, no
  restoration or terminal fallback, and independent hard verification.
- The largest strict measured frame was **91.276 ms**; no strict-run frame exceeded 100 ms.
- Discarded startup attempts reached **739.727 ms** and did not admit the
  circular stationary encounter. Timing below 100 ms after these startup
  attempts is not a cold-start or worst-case guarantee. The prior task also
  recorded a 113.233 ms strict cruise frame on this same controller version.

The thirteen infeasible cases fail in the same way with an independent
5 s diagnostic budget. These failures are not caused by the 100 ms deadline.
The command comparison against the previous final campaign matches all
2,230 comparable issued holds exactly (maximum input difference 0).
For the earlier deadline-failed straight cruise trial, this comparison uses
its completed independent diagnostic run. Timing variation does not indicate
an algorithm change.

## Experimental setup

MATLAB R2026a, one computational thread. The existing
`scripts/runCircularArcControllerValidation.m` calls
`scripts/runExactStateRecursiveFeasibilityScenario.m` with:

- Reference speed 8 m/s; held control period 0.1 s; 300 requested holds (30 s).
- Straight reference and curvatures +0.01, -0.01, +0.02, -0.02 per metre:
  left/right circles of radius 100 and 50 m, respectively.
- Cruise, stationary, oncoming and crossing target scenarios, zero initial
  tracking error in the baseline. Curved-reference targets move on inertial
  straight lines rather than following the curve.
- Exact ego/target sensing, exact held affine study plant, zero residual and
  target jerk; 16 m perception range; requested body clearance 0.25 m.
- No estimator, physical road boundaries, or state/slip box constraints;
  the existing hard curved-pose validity domains remain.
- Seed 20260912. Six discarded two-hold stationary +0.01 startup attempts
  precede the baseline; each aborts on its first infeasible frame.
- Independent 5 s-budget diagnostics repeat failed qualification cases. The
  physical hold period remains 0.1 s and infeasibility always stops execution.

The five additional target-free recovery trials start with error
`[d, ePsi, vx, vy, r] = [0.4 m, 0.03 rad, -0.5 m/s, 0, 0]` relative to the
reference trim. They run after the baseline in the same process with no
additional startup trials. Initial longitudinal speed is 7.5 m/s.
No early-recovery deadline is imposed; terminal tracking is assessed using
the final two seconds of the full 30 s runs.

```matlab
addpath('scripts');
runCircularArcControllerValidation(OutputDirectory=baselineDirectory, ...
    SampleCount=300,Curvatures=[0,.01,-.01,.02,-.02]);
runCircularArcControllerValidation(OutputDirectory=recoveryDirectory, ...
    SampleCount=300,Curvatures=[0,.01,-.01,.02,-.02], ...
    Scenarios="cruise",WarmupCount=0, ...
    InitialTrackingError=[.4;.03;-.5;0;0]);
```

This was executed with `matlab -singleCompThread -batch`. The frame timer
includes measurement construction plus controller execution and excludes
plant integration and offline physical-geometry diagnostics. An unsuccessful
attempt is included in each trial's maximum latency. Profiling was disabled.

## Baseline results

| Curvature / m | Scenario | Executed holds | Outcome | Max frame / ms |
| --- | --- | ---: | --- | ---: |
| +0.00 | cruise | 300 | complete | 91.276 |
| +0.00 | stationary | 300 | complete; clearance shortfall | 40.714 |
| +0.00 | oncoming | 26 | infeasible at 2.6 s | 10.204 |
| +0.00 | crossing | 300 | complete | 8.856 |
| +0.01 | cruise | 300 | complete | 32.091 |
| +0.01 | stationary | 0 | infeasible at 0.0 s | 21.291 |
| +0.01 | oncoming | 26 | infeasible at 2.6 s | 11.408 |
| +0.01 | crossing | 0 | infeasible at 0.0 s | 17.329 |
| -0.01 | cruise | 300 | complete | 16.537 |
| -0.01 | stationary | 0 | infeasible at 0.0 s | 16.262 |
| -0.01 | oncoming | 26 | infeasible at 2.6 s | 11.829 |
| -0.01 | crossing | 0 | infeasible at 0.0 s | 17.747 |
| +0.02 | cruise | 300 | complete | 44.984 |
| +0.02 | stationary | 0 | infeasible at 0.0 s | 15.199 |
| +0.02 | oncoming | 26 | infeasible at 2.6 s | 10.771 |
| +0.02 | crossing | 0 | infeasible at 0.0 s | 16.566 |
| -0.02 | cruise | 300 | complete | 11.886 |
| -0.02 | stationary | 0 | infeasible at 0.0 s | 15.512 |
| -0.02 | oncoming | 26 | infeasible at 2.6 s | 10.847 |
| -0.02 | crossing | 0 | infeasible at 0.0 s | 18.257 |

All five oncoming cases fail at 2.6 s, when the target first becomes active.
Their 26 prior issued holds are pre-admission operation, not successful
avoidance. Circular stationary and crossing targets fail admission at time
zero. Independent relaxed-budget repeats preserve those infeasibility results.
The straight crossing fixture remains a weak avoidance challenge because its
32 m/s lateral target clears the path well before the ego reaches it.

Both completed straight obstacle runs confirm target release and recover to
cruise. The stationary run is still **not safety-qualified** at the prescribed
clearance, as quantified below.

## Recovery with nonzero initial errors

All five cases complete 300 holds and qualify. Values below are maxima of
absolute errors during the final two seconds, measured relative to each
reference's steady-turn trim rather than zero world yaw rate.

| Curvature / m | Max frame / ms | Lateral error / m | Heading error / rad | Speed error / m/s |
| --- | ---: | ---: | ---: | ---: |
| +0.00 | 9.014 | 2.782e-20 | 2.723e-21 | 1.588e-07 |
| +0.01 | 7.296 | 1.031e-05 | 1.002e-09 | 1.135e-07 |
| -0.01 | 7.016 | 1.031e-05 | 1.002e-09 | 1.135e-07 |
| +0.02 | 7.248 | 1.914e-05 | 8.018e-09 | 1.049e-07 |
| -0.02 | 7.044 | 1.914e-05 | 8.018e-09 | 1.049e-07 |

Across these cases, the largest final-window lateral, speed and heading
errors are approximately 1.914e-05 m, 1.588e-07 m/s and 8.018e-09 rad,
respectively. These finite-trial observations support cruise recovery
under the tested model. They do not establish recovery for arbitrary road
curvature, target behavior, estimator error or a nonlinear physical vehicle.

## Independent stationary-clearance audit

The current controller certifies **hold nodes only**. An offline reconstruction
of the strict straight stationary trajectory uses the same declared constant
affine generator and held inputs, and checks exact oriented-rectangle distance.
Every reconstructed endpoint matches the saved successor exactly (maximum
error 0). Reproducing the driver's eleven samples per hold
recovers its minimum margin to within 1e-9 m.

- Minimum hold-node clearance margin: **+0.000005597 m** above 0.25 m.
- Minimum 10 ms sampled clearance margin: **-0.062069524 m**.
- Refining the worst sampled hold and its neighbors at 0.1 ms resolution:
  margin **-0.062233437 m** at **1.4376 s**.
- The body gap at that refined sample is **0.187766563 m**.

Thus the node certificate passes while the inter-node requested clearance
fails. There is no sampled body overlap in this audit. The refined value is
a local sampled minimum, not a proof of the global continuous-time minimum.
This audit does not modify, repair or authorize any controller command.

## Interpretation and limits

The experiment reproduces the known fixed-normal admission limitation. The
previous [implementation and constraint-ablation report](SHIFTED_NOMINAL_SINGLE_SOLVE_20260918.md)
identified nearly opposite adjacent normals on colliding nominal trajectories.
This rerun did not repeat that ablation or change the directions. The failed
convex family is not proof that physical collision is unavoidable.

Predictive continuation and the terminal certificate remain in the controller.
New active normal families are not guaranteed to contain the old shifted
witness; active-encounter recursive feasibility remains unproved under this
strict direction-update policy. Target-free continuation retains its existing
conditional scope. The present run uses the controller alone and provides no
estimator-in-the-loop, Raspberry Pi, ROS or real-vehicle validation.

## Checks and artifacts

MATLAB MCP executed four existing test classes: shifted nominal convexification,
node certificate, curved pose domain and constant-curvature reference.
**44 passed, zero failed/incomplete.** Passing these behavior tests includes
expected infeasibility tests and is not equivalent to passing all avoidance
scenarios. No source files or new tests were needed for this rerun.

All aggregation assertions, single-solve/certification checks, exact prior-input
comparison and `git diff --check` pass. Only this report, its JSON summary and
the report index are committed. External dependencies, binaries, reference
PDFs, unrelated files and raw generated MAT/JSON traces remain excluded.

Raw artifacts and execution logs:
`/home/zai/.cache/collisionAvoidance/straight-circle-retest-20260918-2119/`.
The directory contains `baseline/`, `recovery/`, `tests.log`, `tests.mat`,
`scenarios.log`, `auditClearance.m`, `clearance-audit.json` and the report
aggregation script. The repository JSON records the complete per-case summary,
startup timings, diagnostics, execution checks and audit results.
