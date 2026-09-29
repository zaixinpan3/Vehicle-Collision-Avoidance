# Current terminal-continuation controller: full exact-state rerun

Prepared September 29, 2026. Executed source `4e42f75c0aa64a74e98f369b75261282a09c5125`; snapshot captured at 2026-09-29T01:12:31.281291-05:00.

**9/14 scenarios completed**, with 1194 executed holds and 37014 independent dense geometry samples. 7/14 scenarios both completed and maintained strictly positive sampled rectangle distance (target-free scenarios included). 7/14 pass the complete current audit. Initialization failures are not counted as collision-free results. Of the 1194 returned calls, 1168 exceed the 50 ms control period.

## Scope and method

This reruns the current algorithm without changing production controller code. The algorithm uses a time-indexed hard completion trajectory, a fixed target epoch and a retained nonlinear witness. Primary and secondary subproblems are conic solves. A feasible retained or initialized witness can supply the input even when a new optimization does not converge. Solver convergence, returned-witness feasibility, physical sampled clearance and scenario completion are reported separately. This is one controller; no method selector was introduced.

The same fourteen physical fixtures as the September 28 report are used: reference speeds 8 and 15 m/s, minimum prefix lengths 8 and 16, 50 ms holds, zero added collision buffer, friction 0.85, 4.8 by 1.9 m rectangles, +/-4 m road boundaries and default unlimited actuator slew. Recovery and circular cruise each request 40 holds; five target encounters each request 160 holds. Recovery starts at 0.01 m lateral error; circle curvature is 0.005/m. Targets start at (24,0), heading pi, speed 8 m/s, acceleration 0 or 1 m/s^2 and sideslip 0 or atan(-0.16); braking starts at (24,6), speed 2 m/s and acceleration -1 m/s^2. Target rear-axle distance is 1.6 m. No random draws, observer noise or measurement delay are used.

The next ego state comes from an independent ode45 integration of the same nonlinear vehicle equations (RelTol 1e-11, AbsTol 1e-12), with 31 geometry samples per hold. It does not come from the nominal RK4 successor. The new retained-witness check uses exact state equality, so even integration differences can trigger restoration from a changed ego state despite exact observation. This is distinct from a nominal-RK4 continuation-only experiment. Optimization remains enabled on all frames and uses the unchanged five-second soft budget. That budget is supplied to individual conic solves and checked between outer iterations; it does not preempt the entire frame or endpoint polishing.

The strict geometric requirement is distance > 0; contact and overlap fail, with no invented 0.10 m requirement. The current complete audit additionally requires exact reported zero safety slack and hard residual. The previous version used a numerical safety-slack tolerance, so overall audit pass counts are not a pure like-for-like algorithm comparison. Completion and sampled physical separation use the same physical meaning.

Wall time is measured but not injected as actuation delay: simulation still advances by 50 ms per returned hold. Other MATLAB tasks were observed during tests and early scenarios; later process snapshots show those active batches had ended while multi-second solves persisted. No unrelated process was stopped. Timings and wall-budget failures describe the observed load, not an isolated speed benchmark. No continuous-time safety or real-time guarantee follows from these sampled experiments.

## Scenario results

| Speed | Scenario | Executed holds | Min distance (m) | Overlap samples | Later median / P95 (ms) | First returned call (s) | Failed call (s) | Complete audit |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 8 m/s | recovery | 40/40 | No target | 0 | 2098.390 / 5407.286 | 7.312 | -- | Pass |
| 8 m/s | circular | 40/40 | No target | 0 | 697.895 / 1378.136 | 1.589 | -- | Pass |
| 8 m/s | oncoming | 160/160 | 0.000000000 | 9 | 71.338 / 6556.216 | 6.381 | -- | Fail |
| 8 m/s | turningTarget | 160/160 | 4.262471482 | 0 | 151.878 / 4170.341 | 2.077 | -- | Pass |
| 8 m/s | acceleratingTarget | 160/160 | 0.000000000 | 9 | 72.330 / 6119.868 | 5.543 | -- | Fail |
| 8 m/s | acceleratingTurn | 117/160 | 2.576376711 | 0 | 441.738 / 3769.914 | 2.841 | 0.001 | Fail |
| 8 m/s | brakingTarget | 160/160 | 4.100000000 | 0 | 7243.362 / 8716.905 | 8.164 | -- | Pass |
| 15 m/s | recovery | 40/40 | No target | 0 | 792.944 / 1579.429 | 5.990 | -- | Pass |
| 15 m/s | circular | 40/40 | No target | 0 | 816.436 / 1072.703 | 1.248 | -- | Pass |
| 15 m/s | oncoming | 0/160 | -- | 0 | -- / -- | -- | 7.185 | Fail |
| 15 m/s | turningTarget | 160/160 | 1.004002140 | 0 | 123.420 / 1082.633 | 1.024 | -- | Pass |
| 15 m/s | acceleratingTarget | 0/160 | -- | 0 | -- / -- | -- | 6.578 | Fail |
| 15 m/s | acceleratingTurn | 117/160 | 1.262488813 | 0 | 125.198 / 1540.951 | 2.932 | 0.001 | Fail |
| 15 m/s | brakingTarget | 0/160 | -- | 0 | -- / -- | -- | 0.147 | Fail |

Missing metrics denote unexecuted work, not zero clearance, zero runtime or a safe outcome. Overlap counts use signed SAT gap < -1e-10 m; the actual strict distance audit does not allow contact. A stopped replay only establishes its observed prefix. Failed calls are excluded from the harness's successful-call median/max fields and are reported separately here.

## Failure, continuation and convergence details

- **8 m/s recovery**: No controller-call exception; completed. Failure time -- s. Converged 36/40 returned calls; sources `{'feasibleInitialization': 34, 'sequentialConvexification': 6}`; initialization `{'laneFeedbackRollout': 1, 'restorationFromChangedEgoState': 39}`; valid shifted witness on 0 calls. Maximum safety slack 0, hard residual 0; minimum whole-body road margin 3.039312204 m; final transverse error vector `[0.0004626187073743848, -0.00012291121939518734, -2.528937764978423e-06, 0.00023752585112525005, 0.00020005972727445777]`; 40 returned calls above 50 ms. Terminations `{'timeLimit': 4, 'zeroSlack': 36}`. Horizon range 50--89 holds. Caught search exceptions: 0.
- **8 m/s circular**: No controller-call exception; completed. Failure time -- s. Converged 40/40 returned calls; sources `{'feasibleInitialization': 40}`; initialization `{'laneFeedbackRollout': 1, 'restorationFromChangedEgoState': 39}`; valid shifted witness on 0 calls. Maximum safety slack 0, hard residual 0; minimum whole-body road margin 3.022087949 m; final transverse error vector `[9.341828267521516e-15, -3.2786273695961654e-16, 0, -1.7832957333041577e-15, -1.6375789613221059e-15]`; 40 returned calls above 50 ms. Terminations `{'zeroSlack': 40}`. Horizon range 29--68 holds. Caught search exceptions: 0.
- **8 m/s oncoming**: No controller-call exception; completed. Failure time -- s. Converged 126/160 returned calls; sources `{'sequentialConvexification': 90, 'feasibleInitialization': 70}`; initialization `{'laneFeedbackRollout': 1, 'restorationFromChangedEgoState': 159}`; valid shifted witness on 0 calls. Maximum safety slack 0, hard residual 0; minimum whole-body road margin 0.595574860 m; final transverse error vector `[1.127232354500515e-08, -1.9428644962737584e-09, 7.080518393820512e-08, 1.145799006595569e-09, 1.060867848287251e-09]`; 160 returned calls above 50 ms. Terminations `{'timeLimit': 34, 'zeroSlack': 126}`. Horizon range 8--68 holds. Caught search exceptions: 21.
- **8 m/s turningTarget**: No controller-call exception; completed. Failure time -- s. Converged 160/160 returned calls; sources `{'feasibleInitialization': 156, 'retainedContinuation': 4}`; initialization `{'laneFeedbackRollout': 1, 'shiftedContinuation': 4, 'restorationFromChangedEgoState': 155}`; valid shifted witness on 4 calls. Maximum safety slack 0, hard residual 0; minimum whole-body road margin 3.050000000 m; final transverse error vector `[9.623480011900157e-27, 2.232899295127933e-28, 3.019806626980426e-13, -6.349043871690514e-29, -6.419224729162431e-29]`; 142 returned calls above 50 ms. Terminations `{'zeroSlack': 160}`. Horizon range 8--100 holds. Caught search exceptions: 0.
- **8 m/s acceleratingTarget**: No controller-call exception; completed. Failure time -- s. Converged 126/160 returned calls; sources `{'sequentialConvexification': 86, 'feasibleInitialization': 74}`; initialization `{'laneFeedbackRollout': 1, 'restorationFromChangedEgoState': 159}`; valid shifted witness on 0 calls. Maximum safety slack 0, hard residual 0; minimum whole-body road margin 0.493883592 m; final transverse error vector `[1.9662921818218012e-09, -4.2476364105125177e-10, 4.559952326843586e-07, 4.602073924714489e-10, 4.0337199883550376e-10]`; 160 returned calls above 50 ms. Terminations `{'timeLimit': 34, 'zeroSlack': 126}`. Horizon range 8--68 holds. Caught search exceptions: 62.
- **8 m/s acceleratingTurn**: collisionAvoidanceController:changedTargetTrajectory: The observation changes the fixed target trajectory. Initialize a new problem explicitly. Failure time 5.850 s. Converged 117/117 returned calls; sources `{'feasibleInitialization': 113, 'retainedContinuation': 4}`; initialization `{'laneFeedbackRollout': 1, 'shiftedContinuation': 4, 'restorationFromChangedEgoState': 112}`; valid shifted witness on 4 calls. Maximum safety slack 0, hard residual 0; minimum whole-body road margin 3.050000000 m; final transverse error vector `[3.7864801555508984e-27, 5.863002747961368e-28, 1.7408297026122455e-13, -1.3508038611615817e-28, -1.39427733326981e-28]`; 109 returned calls above 50 ms. Terminations `{'zeroSlack': 117}`. Horizon range 8--100 holds. Caught search exceptions: 0.
- **8 m/s brakingTarget**: No controller-call exception; completed. Failure time -- s. Converged 9/160 returned calls; sources `{'feasibleInitialization': 156, 'retainedContinuation': 4}`; initialization `{'laneFeedbackRollout': 1, 'shiftedContinuation': 4, 'restorationFromChangedEgoState': 155}`; valid shifted witness on 4 calls. Maximum safety slack 0, hard residual 0; minimum whole-body road margin 3.050000000 m; final transverse error vector `[0, 0, 0, 0, 0]`; 160 returned calls above 50 ms. Terminations `{'timeLimit': 18, 'Fiala force requires abs(alpha)<pi/2 and abs(beta)<=1.': 133, 'zeroSlack': 9}`. Horizon range 198--357 holds. Caught search exceptions: 133.
- **15 m/s recovery**: No controller-call exception; completed. Failure time -- s. Converged 39/40 returned calls; sources `{'feasibleInitialization': 34, 'sequentialConvexification': 6}`; initialization `{'laneFeedbackRollout': 1, 'restorationFromChangedEgoState': 39}`; valid shifted witness on 0 calls. Maximum safety slack 0, hard residual 0; minimum whole-body road margin 3.039192160 m; final transverse error vector `[9.622667408175069e-05, -1.567525243213185e-05, 1.1813607869726184e-09, -2.1564942413437353e-05, 4.221633420726956e-05]`; 40 returned calls above 50 ms. Terminations `{'timeLimit': 1, 'zeroSlack': 39}`. Horizon range 37--76 holds. Caught search exceptions: 0.
- **15 m/s circular**: No controller-call exception; completed. Failure time -- s. Converged 40/40 returned calls; sources `{'feasibleInitialization': 39, 'retainedContinuation': 1}`; initialization `{'laneFeedbackRollout': 1, 'restorationFromChangedEgoState': 38, 'shiftedContinuation': 1}`; valid shifted witness on 1 calls. Maximum safety slack 0, hard residual 0; minimum whole-body road margin 3.033142953 m; final transverse error vector `[-5.249291714151025e-15, 5.490399801466594e-16, 0, 1.2455314557513475e-15, 3.1225022567582528e-15]`; 40 returned calls above 50 ms. Terminations `{'zeroSlack': 40}`. Horizon range 37--76 holds. Caught search exceptions: 0.
- **15 m/s oncoming**: collisionAvoidanceController:noFeasibleContinuation: No feasible nonlinear prefix and indefinite continuation were obtained (timeLimit). Failure time 0.000 s. Converged 0/0 returned calls; sources `{}`; initialization `{}`; valid shifted witness on 0 calls. Maximum safety slack None, hard residual None; minimum whole-body road margin -- m; final transverse error vector `[]`; 0 returned calls above 50 ms. Terminations `{}`. Horizon range -- holds. Caught search exceptions: 0.
- **15 m/s turningTarget**: No controller-call exception; completed. Failure time -- s. Converged 160/160 returned calls; sources `{'feasibleInitialization': 156, 'retainedContinuation': 4}`; initialization `{'laneFeedbackRollout': 1, 'shiftedContinuation': 4, 'restorationFromChangedEgoState': 155}`; valid shifted witness on 4 calls. Maximum safety slack 0, hard residual 0; minimum whole-body road margin 3.050000000 m; final transverse error vector `[-5.2269686063541376e-27, -1.029538966108101e-28, -5.684341886080801e-13, 3.1506281565229933e-30, 7.23124138486482e-30]`; 160 returned calls above 50 ms. Terminations `{'zeroSlack': 160}`. Horizon range 16--76 holds. Caught search exceptions: 0.
- **15 m/s acceleratingTarget**: collisionAvoidanceController:noFeasibleContinuation: No feasible nonlinear prefix and indefinite continuation were obtained (timeLimit). Failure time 0.000 s. Converged 0/0 returned calls; sources `{}`; initialization `{}`; valid shifted witness on 0 calls. Maximum safety slack None, hard residual None; minimum whole-body road margin -- m; final transverse error vector `[]`; 0 returned calls above 50 ms. Terminations `{}`. Horizon range -- holds. Caught search exceptions: 0.
- **15 m/s acceleratingTurn**: collisionAvoidanceController:changedTargetTrajectory: The observation changes the fixed target trajectory. Initialize a new problem explicitly. Failure time 5.850 s. Converged 117/117 returned calls; sources `{'feasibleInitialization': 113, 'retainedContinuation': 4}`; initialization `{'laneFeedbackRollout': 1, 'shiftedContinuation': 4, 'restorationFromChangedEgoState': 112}`; valid shifted witness on 4 calls. Maximum safety slack 0, hard residual 0; minimum whole-body road margin 3.050000000 m; final transverse error vector `[-1.9921277140998783e-27, -9.167238183724724e-29, -2.078337502098293e-13, -1.2280572244414618e-30, -2.4279987785436028e-29]`; 117 returned calls above 50 ms. Terminations `{'zeroSlack': 117}`. Horizon range 16--76 holds. Caught search exceptions: 0.
- **15 m/s brakingTarget**: collisionAvoidanceController:noTerminalContinuation: No indefinitely admissible endpoint was reached within maximumHorizonSteps. Failure time 0.000 s. Converged 0/0 returned calls; sources `{}`; initialization `{}`; valid shifted witness on 0 calls. Maximum safety slack None, hard residual None; minimum whole-body road margin -- m; final transverse error vector `[]`; 0 returned calls above 50 ms. Terminations `{}`. Horizon range -- holds. Caught search exceptions: 0.

## Timing in the initial two seconds

The horizon shortens toward its fixed endpoint. Later short-horizon calls can dominate the overall median and conceal slow encounter-phase solves. These initial-window statistics include the first call; partial runs cover only their executed prefix. Failed-call duration remains separately reported above.

| Speed | Scenario | Executed first-2-s median / maximum (s) |
| --- | --- | ---: |
| 8 m/s | recovery | 2.123 / 7.312 |
| 8 m/s | circular | 0.724 / 1.858 |
| 8 m/s | oncoming | 5.797 / 7.717 |
| 8 m/s | turningTarget | 2.417 / 5.475 |
| 8 m/s | acceleratingTarget | 5.762 / 6.832 |
| 8 m/s | acceleratingTurn | 2.330 / 5.044 |
| 8 m/s | brakingTarget | 8.412 / 9.114 |
| 15 m/s | recovery | 0.799 / 5.990 |
| 15 m/s | circular | 0.816 / 1.248 |
| 15 m/s | oncoming | -- / -- |
| 15 m/s | turningTarget | 0.781 / 1.538 |
| 15 m/s | acceleratingTarget | -- / -- |
| 15 m/s | acceleratingTurn | 0.802 / 2.932 |
| 15 m/s | brakingTarget | -- / -- |

## Dense collision findings

- 8 m/s oncoming: 9 zero-distance samples, minimum signed SAT gap -0.000618234727113 m, first zero distance at 1.485000000 s. Minimum node/midpoint signed gap 8.49679798372e-06 m; maximum reported safety slack 0, hard residual 0.
- 8 m/s acceleratingTarget: 9 zero-distance samples, minimum signed SAT gap -0.000665252573152 m, first zero distance at 1.451666667 s. Minimum node/midpoint signed gap 0.000176913963647 m; maximum reported safety slack 0, hard residual 0.

## Comparison with the previous experiment

The previous production source was b0cc119324b5a8def8af47103970ad00bb8385b0. It completed all 14 cases and 1760 holds; 11/14 completed with strictly positive sampled separation. These wall timings were not measured under a controlled common load. Shorter prefixes in the current run must not be interpreted as faster completion.

| Speed | Scenario | Previous / current holds | Previous / current later median (ms) |
| --- | --- | ---: | ---: |
| 8 m/s | recovery | 40 / 40 | 22.125 / 2098.390 |
| 8 m/s | circular | 40 / 40 | 22.297 / 697.895 |
| 8 m/s | oncoming | 160 / 160 | 241.608 / 71.338 |
| 8 m/s | turningTarget | 160 / 160 | 274.205 / 151.878 |
| 8 m/s | acceleratingTarget | 160 / 160 | 243.401 / 72.330 |
| 8 m/s | acceleratingTurn | 160 / 117 | 272.539 / 441.738 |
| 8 m/s | brakingTarget | 160 / 160 | 23.501 / 7243.362 |
| 15 m/s | recovery | 40 / 40 | 40.061 / 792.944 |
| 15 m/s | circular | 40 / 40 | 40.944 / 816.436 |
| 15 m/s | oncoming | 160 / 0 | 215.209 / -- |
| 15 m/s | turningTarget | 160 / 160 | 216.565 / 123.420 |
| 15 m/s | acceleratingTarget | 160 / 0 | 215.237 / -- |
| 15 m/s | acceleratingTurn | 160 / 117 | 216.396 / 125.198 |
| 15 m/s | brakingTarget | 160 / 0 | 43.644 / -- |

## Validation and reproduction

Selected MATLAB regression tests: 234/234 passed, 0 failed, 0 incomplete. Eight Python geometry, target-flow and audit tests passed. MATLAB test results are retained even if a closed-loop scenario fails. The entire repository suite was not run.

All tracked files under controller, config, estimator, scripts and tests were copied and hash-verified into an external snapshot before execution. All 79 executable MATLAB/Python files match the stated source commit. The snapshot also preserves an unrelated uncommitted estimator theory-document edit, which is not executed or included in the experiment commit. The existing solver dependency directory was linked, not copied or committed. A sequential MATLAB campaign ran the selected tests followed by both speed cohorts; each case exported its own trace, including initialization failures. Python independently reconstructed target motion and rectangle distances, signed SAT gaps, and whole-body road extrema. No production algorithm changes were made by this task.

```bash
matlab -batch "run('/home/zai/.cache/collisionAvoidance/current-controller-rerun-20260929-011231/campaign.m');"
python /home/zai/.cache/collisionAvoidance/current-controller-rerun-20260929-011231/source/tests/auditJointPredictiveSafetyTest.py
python /home/zai/.cache/collisionAvoidance/current-controller-rerun-20260929-011231/analyze.py
```

Compact summaries, per-hold timing CSVs, test outcomes and technical/source hashes are in the matching report directory. Raw traces, executed source snapshot, campaign and analysis scripts remain at:

`/home/zai/.cache/collisionAvoidance/current-controller-rerun-20260929-011231/`

## Supplementary read-only diagnostics

The following diagnostics use the executed snapshot and do not modify the controller or rerun a repaired algorithm.

### Target-heading representation

At the accelerating-turn failure, the parser wraps the observed heading to [-pi,pi], while the fixed-epoch predictor keeps an unwrapped heading. The controller compares their raw state vectors. The same physical heading can therefore be rejected after crossing -pi. The parser and target-conversion functions were rerun on the failed observation, with assertions checking that the difference is exactly one revolution and that the remaining state difference is within the existing comparison threshold.

- baseline: at 5.850 s, predicted yaw -3.169263855069 rad and parsed yaw 3.113921452111 rad. Raw difference 6.28318530718; after comparing yaw modulo 2*pi, maximum difference 2.44929359829e-16, below threshold 2.37239358139e-07.
- default: at 5.850 s, predicted yaw -3.169263855069 rad and parsed yaw 3.113921452111 rad. Raw difference 6.28318530718; after comparing yaw modulo 2*pi, maximum difference 2.44929359829e-16, below threshold 2.37239358139e-07.

### Braking-target endpoint sensitivity

The all-future progress bound treats the sign of arbitrarily small acceleration components exactly. With heading pi, floating-point sin(pi) introduces a tiny lateral component. For this centered, zero-sideslip rectangle, changing heading to zero and reversing signed speed/acceleration is a physically equivalent straight-motion representation, up to floating-point error. Evaluating both representations at the same candidate endpoints isolates the sensitivity of the sufficient infinite-horizon separation condition. This is a diagnostic comparison, not an accepted tolerance change or a proof that all small components may be discarded.

| Ego speed | Endpoint stage | Original separation margin | Equivalent-representation margin | Target lateral acceleration |
| --- | ---: | ---: | ---: | ---: |
| 8 | 68 | -7.90006987 | 3.04993013 | -1.2246468e-16 |
| 8 | 357 | 0.0111801259 | 3.04993013 | -1.2246468e-16 |
| 8 | 512 | 3.04993013 | 3.04993013 | -1.2246468e-16 |
| 15 | 76 | -7.90044682 | 3.04955318 | -1.2246468e-16 |
| 15 | 357 | -7.90044682 | 3.04955318 | -1.2246468e-16 |
| 15 | 512 | -7.90044682 | 3.04955318 | -1.2246468e-16 |

### ODE45 versus RK4 successors

`successor-differences.json` reports per-state maximum and median absolute successor differences, in the state order X, Y, psi, vx, vy, r. These differences arise from numerical integration of exact observed states, not from an observer. The recorded initialization counts show when they disable direct shifted-witness reuse. Empty-case metrics refer to unexecuted work.

Supplementary diagnostic command:

```bash
matlab -batch "run('/home/zai/.cache/collisionAvoidance/current-controller-rerun-20260929-011231/successorDifferences.m'); run('/home/zai/.cache/collisionAvoidance/current-controller-rerun-20260929-011231/diagnoseTargetWrap.m'); run('/home/zai/.cache/collisionAvoidance/current-controller-rerun-20260929-011231/diagnoseBrakingEndpoint.m');"
```
