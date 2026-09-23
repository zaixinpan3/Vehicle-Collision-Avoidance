# Feedback prediction: implementation validation, 2026-09-23

Commit `6c244dd3dfea7a8cc074a1872a5e882eb566394b` ("Predict the ego deviation under
per-hold feedback") implements [FEEDBACK_TUBE_PREDICTION.md](../controller/FEEDBACK_TUBE_PREDICTION.md).
The plan becomes a feedback policy: the first held input is exact, and later inputs add
`K (estimate - nominal)`, with `K` the cruise gain without lateral-velocity feedback and a
halved speed gain. The ego deviation is predicted as a zonotope under that feedback
instead of the open-loop interval box. This record reports measured outcomes on
the declared held affine plant only. Timings are not isolated (other MATLAB sessions
were running).

Environment: MATLAB R2026a batch, clean worktrees, one experiment at a time.
Open-loop baseline: commit `6b477ed` (box-free, open-loop prediction). Raw outputs
are under the git-ignored `simulation_output/feedback_prediction_validation_20260923/`.

## 1. Tests

Committed contents, `runtests('tests')`: 780 tests, 778 passed, 0 failed, 2
incomplete (curb-detection tests skipped by assumption when the untracked LiDAR
dataset is absent). New class `tests/feedbackPredictionTest.m` (7 tests):
- 200 sampled trajectories with random and vertex initial and per-hold estimator
  errors stay inside every predicted node set;
- executed-input and slew deviations stay inside their reserved supports;
- inherited frames issue `plan + K (estimate - carried nominal)`, and the next frame
  accepts that stored state;
- a tampered stored correction is rejected;
- a larger estimator bound re-admits the frame.

Three existing assertions were updated to the new contract:
- the retained plan is issued with its correction;
- large target-free uncertainty keeps the configured horizon instead of shortening it;
- node boxes are zonotope hulls, and only node 1 remains the interval image of the
  initial box.

## 2. Declared plant with the estimator's ego bound

`scripts/runEstimatorBoundCampaign.m`: the recorded NRMM estimator bound
`[0.076 m; 0.076 m; 0.048 rad; 0.089 m/s; 0.497 m/s; 0.0015 rad/s]` is the ego
measurement box, and the measurement is truth plus independent uniform noise inside
that box at every frame. 240 holds, deadline disabled, seed 20260912.

| Scenario | Open loop (`6b477ed`) | Feedback (`6c244dd`) | Minimum node gap (feedback) |
| --- | --- | --- | --- |
| stationary, straight | infeasible at t = 0 | pass 240/240 | 0.587 m |
| oncoming, straight | pass | pass | 0.459 m (open loop 0.056 m) |
| crossing, straight | infeasible at t = 0 | pass | 9.527 m |
| cruise, straight | pass | pass | no target |
| stationary, curvature 0.01 | infeasible at t = 0 | pass | 0.808 m |
| oncoming, curvature 0.01 | pass | pass | 0.650 m (open loop 0.239 m) |
| crossing, curvature 0.01 | infeasible at t = 0 | pass | 1.017 m |
| cruise, curvature 0.01 | pass | pass | no target |

With feedback, all eight cases complete with positive node and sampled gaps and
negative CLF residuals. The smallest terminal membership margin is 0.0045
(stationary, curvature 0.01). Per-case tables:
[open loop](FEEDBACK_PREDICTION_VALIDATION_20260923/estimatorBoundCampaign_openLoop_6b477ed.csv),
[feedback](FEEDBACK_PREDICTION_VALIDATION_20260923/estimatorBoundCampaign_feedback_6c244dd.csv).

## 3. Failure-mode sweep (118 cases)

Compared with the box-free open-loop sweep
([CONTROLLER_FAILURE_MODE_SWEEP_BOXFREE_20260923.csv](CONTROLLER_FAILURE_MODE_SWEEP_BOXFREE_20260923.csv)):

| Outcome | Open loop | Feedback |
| --- | --- | --- |
| Passed | 102 | 103 |
| Controller error | 7 | 6 |
| Node or inter-node overlap | 9 | 9 |

Only three cases change their outcome:

| Case | Open loop | Feedback |
| --- | --- | --- |
| 39 stationary, straight, uncertainty x10 | infeasible at t = 0 | pass, node gap 3.393 m |
| 42 stationary, curvature 0.01, x10 | infeasible at t = 0 | pass, node gap 3.469 m |
| 48 oncoming, curvature 0.01, x10 | pass | **infeasible at t = 3.0 s** |

Per-case table: [CONTROLLER_FAILURE_MODE_SWEEP_FEEDBACK_20260923.csv](CONTROLLER_FAILURE_MODE_SWEEP_FEEDBACK_20260923.csv).
Case 54 (crossing, curvature 0.01, x10) remains infeasible because the target's
declared reachable set grows to 18 m radius. Ego feedback does not address that. The
road-boundary rejections, the eight straight-road inter-node overlaps and the case 96
node overlap left by the chart-box removal are unchanged.

### Case 48

The failing frame was rebuilt with a scratch model capture
([case48.txt](FEEDBACK_PREDICTION_VALIDATION_20260923/case48.txt)). With feedback, the
frame is infeasible; relaxing only the actuator amplitude rows makes it feasible,
while relaxing only the slew rows does not. Without feedback the same frame is
feasible. At ten-fold noise the feedback reserves up to 0.092 rad of steering and
0.19 of braking ratio for the correction. The oncoming swerve admitted at t = 3.0 s
needs nearly full steering lock, so the reservation removes the admissible plans.
This is the reserved-authority cost stated in the design. A smaller gain, or a gain
that reserves less steering during the swerve, trades it against the deviation-set
size.

## 4. Recursion campaign (straight roads)

`runRecursiveSafetyValidation`: all five cases pass 600/600 holds. The values match
the open-loop campaign; the uncertain-crossing terminal margin rises from 0.1734 to
0.1782. The 50 ms deadline runs complete 0/120, 52/120 and 120/120 holds.

## 5. Conclusion

Predicting under per-hold feedback removes the open-loop growth of the ego
uncertainty. With the estimator's actual bound and per-frame noise, all eight
declared-plant scenarios now run to completion, including the four that were
infeasible at the first frame. In the sweep it resolves the two ten-fold stationary
cases and costs one ten-fold oncoming case, whose full-lock swerve cannot spare the
steering authority reserved for the correction.
