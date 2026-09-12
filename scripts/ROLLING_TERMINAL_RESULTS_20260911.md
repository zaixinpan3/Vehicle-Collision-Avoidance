# Rolling MPC terminal-certificate correction

September 11, 2026. Certificate version 18. This is a controller-only,
exact-state straight-road experiment. The executable terminal fallback is
removed. All three final trials complete 30 seconds with checked finite
predictions and no sampled collision/road violation. Oncoming and crossing
recover or maintain cruise; the stationary-obstacle trial still stops behind
the obstacle. Every measured frame exceeds 100 ms. **The controller is not
real-time qualified, and a global recursive-feasibility proof is not claimed.**

## What changed and why

- Every sample verifies the previous first hold, rebuilds a complete rolling
  problem, solves and checks it, and applies only its first optimized input.
  The original endpoint no longer counts down to a terminal-policy handoff.
  The old fixed-prefix machinery and terminal command-dispatch path are deleted.
- The shifted old controls plus one terminal action are a mathematical safety
  candidate checked in the refreshed problem. Performance initialization instead
  repeats the previous final optimized input. Injecting terminal braking into
  the CLF approximation caused a separate artificial slowdown in development.
- Terminal charts cover the verified stopping excursion. Directional pose
  budgets use nonnegative longitudinal velocity, avoiding a fictitious backward
  stopping requirement near the road origin. The stopping policy and hard
  terminal, collision, road, input, slew and model-domain constraints remain.
- The feasibility LP cannot supply an executable fallback. The performance
  SOCP uses a small numerical interior instead of preserving 99 percent of
  maximized surplus margin. Physical acceptance remains strict and independently
  enclosed; solver buffers are distinguished from physical safety boundaries.
- Squared CLF slack and input effort about the CLF/LQR certificate operating
  input are preserved. No desired-acceleration objective is added. The observer
  is unchanged. No new controller source module or alternate controller is added.

## Final experiment results

Each trial starts at 8 m/s with a 100 ms hold and a 16-step horizon. The road
centerline is straight, with boundaries at y = +/-5 m and required footprint
clearance 0.25 m. Target initial position/velocity are (60,0)/(-8,0) oncoming,
(15,0)/(0,0) stationary, and (15,-4)/(0,32) crossing, in meters and m/s.
There is one persistent, exactly observed target, no process/measurement noise,
no random seed and no injected solver failure. The target retains its original
Cartesian constant-acceleration and constant-yaw-rate motion law.

The ego executes the published first-hold affine generator independently with
`expm`. Eleven points per held interval audit rectangle separation and road
containment; these sampled audits supplement the analytic prediction checks.
Each run executes 300 holds and computes 301 commands; the last command is not
executed. All 903 commands are certified, and no terminal or fallback commands
are used. All prediction horizons remain at 16 steps.

| Scene | Final speed (m/s) | Final lateral error (m) | Final heading error (deg) | Minimum additional separation (m) | Minimum additional road margin (m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| oncoming | 8.0000454 | -5.225865e-06 | 1.234689e-05 | 0.3875553 | 0.814965 |
| stationary | 4.6558267e-05 | -9.019102e-08 | -1.976468e-07 | 0.002697137 | 3.79979 |
| crossing | 8 | -4.562967e-08 | -9.509902e-09 | 9.269858 | 3.8 |

Both margin columns have already deducted the required 0.25 m clearance.
Maximum input slew violation is zero in every run. Oncoming ends close to
8 m/s, zero lateral error and zero heading error; its last-five-second speed
range is 8.00004538--8.00004542 m/s. This is finite-run evidence, not a proof
of asymptotic convergence.

The stationary scene is **safe stopping, not successful passing and cruise
recovery**. It ends at approximately 0.000047 m/s while the optimizer continues
to solve every frame. The ego stays essentially on the centerline behind the
obstacle. Thus deleting terminal dispatch does not by itself solve stationary
obstacle maneuver selection. The local convex geometry and finite-horizon
soft-CLF objective remain a limitation; this run does not show that no other
physically safe maneuver exists. The passing/recovery requirement is not marked
as satisfied for this case.

## Timing and the low-speed bottleneck

| Scene | First frame (s) | Median frame (s) | Maximum frame (s) | Frames above 100 ms |
| --- | ---: | ---: | ---: | ---: |
| oncoming | 2.129387 | 0.499429 | 2.129387 | 301/301 |
| stationary | 2.603560 | 6.101306 | 9.360193 | 301/301 |
| crossing | 0.430688 | 0.440700 | 1.057689 | 301/301 |

Whole-frame measurements include input assembly and the controller, including
its checks; they exclude plant integration, independent geometry audits and
report/progress-file writing. Diagnostic simulation continues after missed
deadlines. These are observed timings on a shared workstation, not WCET bounds.
All 903 calls miss the required period, so none of the trials is runtime-qualified.

Mean instrumented phase times in seconds follow; input preparation and other
small orchestration work are included in whole-frame time but not these columns.

| Scene | Prediction | Formulation | Solve | Verification/commit diagnostics |
| --- | ---: | ---: | ---: | ---: |
| oncoming | 0.0693 | 0.1398 | 0.2928 | 0.0115 |
| stationary | 0.2749 | 2.5063 | 3.2568 | 0.1025 |
| crossing | 0.0606 | 0.1417 | 0.2531 | 0.0114 |

The low-speed diagnostic holds the horizon at 16 steps. At 0.73 m/s its last
frame spends about 2.56 s in formulation and 5.75 s in solving. The subdivision
rule is `max(minimumCells, ceil(2*norm(A,inf)*sampleTime))`. Representative
straight, zero-braking-ratio matrices require 7 cells/hold at 8 m/s, 28 at
2 m/s and 56 at 0.73 m/s. The stiff low-speed lateral dynamics therefore create
many more geometry rows, auxiliary states and CLF cones. These are internal
certification cells, not additional control updates. This bottleneck remains
unresolved; lowering validation accuracy or accepting a late controller as
real-time capable was not used to obtain these results.

## Proof scope and development findings

Huang, Wang, Margellos and Goulart (2025),
[Section III, Lemmas III.1 and III.5, equations (10)--(11)](https://arxiv.org/html/2502.08400v2),
use a shifted sequence and terminal action to establish the safe-MPC successor
argument. Their safety slack differs from this controller's CLF relaxation.
The implementation retains a terminal invariant-set derivation, but refreshed
scheduled matrices, separating geometry and terminal regions do not guarantee
that the shifted candidate remains admissible. The zero-speed scheduled terminal
certificate is hypothetical; invariance has not been transferred to the
refreshed cruise schedule or to a nonlinear physical vehicle.

| Scene | Shifted safety candidate feasible in refreshed problem |
| --- | ---: |
| oncoming | 283/300 successor frames |
| stationary | 277/300 successor frames |
| crossing | 300/300 successor frames |

All fresh optimizations succeed despite 40 rejected shifted candidates across
the oncoming and stationary runs. This is evidence against blindly reusing
the old shift argument, not evidence of global infeasibility. Both global
recursive-feasibility claim flags remain false. Full assumptions, directional
terminal budgets and the missing shift-inclusion premise are documented in
[the controller theory](../controller/SINGLE_PATH_RECURSIVE_FEASIBILITY.md).

Two development diagnostics explain the final design: cold rebuilding without
a carried performance initialization failed at 11.1 s after lateral oscillation;
using the terminal braking action as the performance seed slowed the crossing
ego to 5.81 m/s by 2.4 s. Both precede the final implementation and are kept as
separately labeled diagnostics. The final oncoming/crossing runs correct these
observed failures. They do not establish a global theorem or clear the remaining
stationary-maneuver and timing limitations.

## Validation and reproduction

The full repository suite returns **623/623 passed, 0 failed, 0 incomplete**.
The changed behavioral regressions cover rolling horizons, no terminal takeover,
failed/unsafe solve termination, rejection of an LP-only command, the exact
target/first-hold contract, directional stopping budgets, entry/future slew,
cruise behavior, and agreement with an independent performance solver.
Factory Code Analyzer checks 12 changed MATLAB files: no new findings, with
16 existing sparse-indexing suggestions matching the baseline. The core source
count remains 20. `git diff --check` passes.

```matlab
addpath('controller','config','scripts','tests', ...
    'solver/bicycle','solver/clarabel/matlab');
for scenario = ["oncoming","stationary","crossing"]
    report = runExactStateRecursiveFeasibilityScenario(Scenario=scenario, ...
        SampleCount=300,FailAfterAdmission=false,DeadlineSeconds=0.1);
end
results = runtests('tests');
assertSuccess(results);
```

The final run used MATLAB R2026a Update 3 and the installed native kernels.
Raw MAT/JSON outputs, phase measurements, test results/logs, a standalone plot,
and reproduction scripts are original research outputs in:

`/home/zai/.cache/collisionAvoidance/rolling-terminal-20260911/validation`

Archive copies are identified as exports. The previous fixed-terminal baseline
is recorded in [the earlier result document](STRAIGHT_EXACT_STATE_RESULTS_20260911.md)
at commit `4360bc773d3db44d97b7cd102617616974585204`. No estimator-controller joint,
noisy-sensing, curved-road, nonlinear-vehicle or real-time qualification is claimed
by this straight exact-state rerun.
