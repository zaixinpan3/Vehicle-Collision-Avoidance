# Current-controller exact-state rerun

Date: September 28, 2026. Source snapshot captured at 2026-09-28T20:00:13.123009-05:00.

14/14 closed loops completed, totaling
1760 holds and 54,560
dense geometry samples. 11/14 fixtures retain strictly positive sampled rectangle
distance (target-free fixtures included); 11/14 pass the complete audit including
completion, road containment and nominal solver checks. The collision criterion is
**distance strictly greater than zero**, with no required extra clearance buffer.
Contact and overlap both fail. Zero nominal safety slack is reported separately.

## Executed source and experiment scope

This task runs the user's current working source without changing production code.
The executable snapshot exactly matches production commit `b0cc119324b5a8def8af47103970ad00bb8385b0`
(Model constant target acceleration and sideslip). It includes constant-acceleration/constant-sideslip target
model changes and updated fixtures. The snapshot was captured while those changes
were still uncommitted, on top of `ecfbf48a6f6f66864ab247a3b030f7e86ece52f1`. All tracked files under controller, config, estimator, scripts and tests
were copied to an external source snapshot; every copied file was hash-checked
against the working tree before execution. Both speed cohorts run that same snapshot.
Its source hashes and full working-source patch are retained; executable hashes
were subsequently verified against the production commit. This task commits only
experiment reports and compact measurements.

The target state is `[X,Y,psi,V,A,beta,lr,halfLength,halfWidth,offsetX,offsetY]`.
Tangential acceleration A and sideslip beta remain constant; heading rate is
`V*sin(beta)/lr`. Velocity V is signed, so braking through zero reverses the target
instead of clamping it to a stationary pose. All ego and target states are supplied
exactly; there is no observer, noise, latency or uncertainty envelope in the replays.
Observer compatibility tests do not mean an observer was enabled in simulation.

The baseline uses 8 m/s ego reference speed and 8 minimum horizon holds; the default
cohort uses 15 m/s and 16 minimum holds. The initializer extends encounter horizons
as needed. Both use 50 ms holds, zero added collision buffer, friction coefficient
0.85, 4.8 by 1.9 m bodies, an 8 m wide corridor, a 40-degree steering bound and
default unbounded input slew rates. Full configurations and actual horizon lengths
are exported. No random draws are used.

Recovery starts with 0.01 m lateral error; circular cruise starts at its trim with
curvature 0.005/m. Each lasts 40 holds (2 s). The five target cases each last 160
holds (8 s), with target rear-axle distance 1.6 m and the following initial data:

| Scenario | Initial target position (m) | Heading | Speed (m/s) | Acceleration (m/s^2) | Sideslip |
| --- | --- | --- | ---: | ---: | --- |
| oncoming | (24,0) | pi | 8 | 0 | 0 |
| turningTarget | (24,0) | pi | 8 | 0 | atan(-0.16) |
| acceleratingTarget | (24,0) | pi | 8 | 1 | 0 |
| acceleratingTurn | (24,0) | pi | 8 | 1 | atan(-0.16) |
| brakingTarget | (24,6) | pi | 2 | -1 | 0 |

The braking target starts laterally outside the ego corridor; this checks stop/reverse
prediction and continued control, not an unavoidable frontal braking encounter.
The turning fixture differs from the previous zero-sideslip, constant-heading-rate
fixture. It must not be treated as an identical benchmark for a claimed safety or
runtime improvement. The straight oncoming and target-free fixture conditions remain
comparable, but host timings and time-limited optimizer paths are not controlled.

## Results

| Ego speed | Scenario | Holds | Minimum distance (m) | Overlap samples | Minimum road margin (m) | Final lateral error (mm) | Later median / P95 (ms) | First call (s) | Calls over 50 ms |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 8 m/s | recovery | 40/40 | No target | 0 | 3.039233 | 0.708 | 22.12 / 26.32 | 0.192 | 1 |
| 8 m/s | circular | 40/40 | No target | 0 | 3.022088 | 0.000 | 22.30 / 23.66 | 0.032 | 0 |
| 8 m/s | oncoming | 160/160 | 0.008360305 | 0 | 0.437090 | -5.209 | 241.61 / 2269.56 | 4.850 | 160 |
| 8 m/s | turningTarget | 160/160 | 4.264841714 | 0 | 3.046839 | -2.867 | 274.20 / 291.77 | 0.305 | 160 |
| 8 m/s | acceleratingTarget | 160/160 | 0.000000000 | 1 | 0.515037 | -5.235 | 243.40 / 2023.70 | 2.264 | 160 |
| 8 m/s | acceleratingTurn | 160/160 | 2.579049458 | 0 | 3.046439 | -2.959 | 272.54 / 284.47 | 0.302 | 160 |
| 8 m/s | brakingTarget | 160/160 | 4.099999995 | 0 | 3.050000 | 0.000 | 23.50 / 27.28 | 0.026 | 0 |
| 15 m/s | recovery | 40/40 | No target | 0 | 3.039185 | 0.053 | 40.06 / 43.46 | 0.104 | 2 |
| 15 m/s | circular | 40/40 | No target | 0 | 3.033143 | 0.000 | 40.94 / 42.04 | 0.050 | 1 |
| 15 m/s | oncoming | 160/160 | 0.000000000 | 9 | 0.356025 | -1.593 | 215.21 / 1538.59 | 2.289 | 160 |
| 15 m/s | turningTarget | 160/160 | 1.004535017 | 0 | 3.048312 | -1.593 | 216.56 / 221.10 | 0.238 | 160 |
| 15 m/s | acceleratingTarget | 160/160 | 0.000000000 | 3 | 0.332751 | -1.593 | 215.24 / 1528.22 | 2.347 | 160 |
| 15 m/s | acceleratingTurn | 160/160 | 1.262520651 | 0 | 3.048312 | -1.593 | 216.40 / 222.50 | 0.237 | 160 |
| 15 m/s | brakingTarget | 160/160 | 4.100000002 | 0 | 3.050000 | -0.000 | 43.64 / 45.79 | 0.049 | 0 |

Overlap samples use signed separating-axis gap below -1e-10 m to distinguish
penetration from roundoff. The strict audit itself requires distance > 0 and does
not permit contact under that tolerance. Replays continue through geometric overlap
without impact dynamics. Tracking recovery after a collision is not a safe maneuver.

- 8 m/s recovery: converged 40/40 calls; maximum nominal safety-slack sum 0; maximum nominal hard residual 0; termination counts `{'zeroSlack': 40}`.
- 8 m/s circular: converged 40/40 calls; maximum nominal safety-slack sum 0; maximum nominal hard residual 0; termination counts `{'zeroSlack': 40}`.
- 8 m/s oncoming: converged 159/160 calls; maximum nominal safety-slack sum 8.76996327e-06; maximum nominal hard residual 0; termination counts `{'missingDiagnostic': 1, 'zeroSlack': 159}`. First minimum distance at 1.516667 s; minimum signed SAT gap 0.00836030473 m; 0 zero-distance samples. Minimum node/midpoint distance 0.0112868791 m, with signed SAT gap 0.0112868791 m.
- 8 m/s turningTarget: converged 160/160 calls; maximum nominal safety-slack sum 0; maximum nominal hard residual 0; termination counts `{'zeroSlack': 160}`. First minimum distance at 1.426667 s; minimum signed SAT gap 3.61394378 m; 0 zero-distance samples. Minimum node/midpoint distance 4.26498064 m, with signed SAT gap 3.61671271 m.
- 8 m/s acceleratingTarget: converged 160/160 calls; maximum nominal safety-slack sum 7.62657794e-06; maximum nominal hard residual 0; termination counts `{'zeroSlack': 160}`. First minimum distance at 1.425000 s; minimum signed SAT gap -6.33804343e-06 m; 1 zero-distance samples. Minimum node/midpoint distance 0 m, with signed SAT gap -6.33804343e-06 m.
- 8 m/s acceleratingTurn: converged 160/160 calls; maximum nominal safety-slack sum 0; maximum nominal hard residual 0; termination counts `{'zeroSlack': 160}`. First minimum distance at 4.856667 s; minimum signed SAT gap 2.080739 m; 0 zero-distance samples. Minimum node/midpoint distance 2.58257302 m, with signed SAT gap 2.18076863 m.
- 8 m/s brakingTarget: converged 160/160 calls; maximum nominal safety-slack sum 0; maximum nominal hard residual 0; termination counts `{'zeroSlack': 160}`. First minimum distance at 2.788333 s; minimum signed SAT gap 4.1 m; 0 zero-distance samples. Minimum node/midpoint distance 4.1 m, with signed SAT gap 4.1 m.
- 15 m/s recovery: converged 40/40 calls; maximum nominal safety-slack sum 0; maximum nominal hard residual 0; termination counts `{'zeroSlack': 40}`.
- 15 m/s circular: converged 40/40 calls; maximum nominal safety-slack sum 0; maximum nominal hard residual 0; termination counts `{'zeroSlack': 40}`.
- 15 m/s oncoming: converged 157/160 calls; maximum nominal safety-slack sum 6.3763226e-06; maximum nominal hard residual 0; termination counts `{'zeroSlack': 157, 'timeLimit': 3}`. First minimum distance at 1.011667 s; minimum signed SAT gap -0.000632162144 m; 9 zero-distance samples. Minimum node/midpoint distance 0 m, with signed SAT gap -1.17863039e-06 m.
- 15 m/s turningTarget: converged 160/160 calls; maximum nominal safety-slack sum 0; maximum nominal hard residual 0; termination counts `{'zeroSlack': 160}`. First minimum distance at 1.033333 s; minimum signed SAT gap 0.938405912 m; 0 zero-distance samples. Minimum node/midpoint distance 1.02231614 m, with signed SAT gap 0.958190083 m.
- 15 m/s acceleratingTarget: converged 160/160 calls; maximum nominal safety-slack sum 7.18229884e-06; maximum nominal hard residual 0; termination counts `{'zeroSlack': 160}`. First minimum distance at 1.000000 s; minimum signed SAT gap -2.84023454e-05 m; 3 zero-distance samples. Minimum node/midpoint distance 0 m, with signed SAT gap -6.19761213e-07 m.
- 15 m/s acceleratingTurn: converged 160/160 calls; maximum nominal safety-slack sum 0; maximum nominal hard residual 0; termination counts `{'zeroSlack': 160}`. First minimum distance at 1.010000 s; minimum signed SAT gap 1.16305531 m; 0 zero-distance samples. Minimum node/midpoint distance 1.28195461 m, with signed SAT gap 1.19748212 m.
- 15 m/s brakingTarget: converged 160/160 calls; maximum nominal safety-slack sum 0; maximum nominal hard residual 0; termination counts `{'zeroSlack': 160}`. First minimum distance at 1.295000 s; minimum signed SAT gap 4.1 m; 0 zero-distance samples. Minimum node/midpoint distance 4.1 m, with signed SAT gap 4.1 m.

## Validation and limitations

All 220/220 selected MATLAB tests and five Python
audit/target-flow tests pass. The selected MATLAB suites include controller, tire,
configuration, source budget, road-load, input geometry, NRMM model and online NRMM
runtime checks. The entire repository suite was not rerun. Initial snapshot setup
omitted the external YALMIP/SeDuMi directory, producing 43 observer test failures
and preventing that first validation invocation from reaching simulation. The existing
solver dependency directory was then linked into the snapshot and the validation
was rerun successfully. Initial failures are retained under `initial-environment/`;
no production algorithm was changed to resolve them.

The ego plant uses the same nonlinear Fiala equations as prediction, integrated
independently with ode45 at RelTol 1e-11 / AbsTol 1e-12. Each hold has 31 samples.
Python independently reconstructs target motion from the exported initial state and
checks polygon distances. The supplemental audit includes whole-body circular-road
edge extrema. These are sampled nominal-model results, not continuous-time proofs
or an independent high-fidelity vehicle plant. Simulation time advances by 50 ms
regardless of solve duration; latency is measured but not applied to the plant.
The 5 s SCvx budget is checked between iterations; a running LP/QP can overrun it.

## Artifacts and reproduction

Compact results, timing CSVs normalized to LF, tests, audits and technical/source
hashes are in the sibling `CURRENT_CONTROLLER_RERUN_20260928/` directory. The exact
executed source snapshot, full traces, logs and analysis scripts are at:

`/home/zai/.cache/collisionAvoidance/current-controller-rerun-20260928-1959/`

```bash
matlab -batch "run('/home/zai/.cache/collisionAvoidance/current-controller-rerun-20260928-1959/campaign.m');"
python3 /home/zai/.cache/collisionAvoidance/current-controller-rerun-20260928-1959/source/tests/auditJointPredictiveSafetyTest.py
python3 /home/zai/.cache/collisionAvoidance/current-controller-rerun-20260928-1959/source/scripts/auditJointPredictiveSafety.py /home/zai/.cache/collisionAvoidance/current-controller-rerun-20260928-1959/baseline --output /home/zai/.cache/collisionAvoidance/current-controller-rerun-20260928-1959/baseline/strict-audit.json
python3 /home/zai/.cache/collisionAvoidance/current-controller-rerun-20260928-1959/source/scripts/auditJointPredictiveSafety.py /home/zai/.cache/collisionAvoidance/current-controller-rerun-20260928-1959/default --output /home/zai/.cache/collisionAvoidance/current-controller-rerun-20260928-1959/default/strict-audit.json
```

Audit failures are retained as experimental outcomes. The source snapshot uses the
existing external `solver/` dependency directory; dependencies, native binaries,
source snapshots, raw traces and unrelated user changes are not committed by this task.
