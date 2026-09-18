# Estimator-in-the-loop avoidance: admission blocked by published bounds

September 17, 2026. Outcome: **failed as a closed loop, diagnosed**. With the
hold-node controller, the declared-plant joint entry
`runDeclaredPlantEstimatorControllerScenario` completes the 30 s oncoming
encounter when the controller receives exact states, but with the NRMM
estimator in the loop it still stops at the first frame that publishes the
target. The stop is not a runtime failure: the estimator's published
certified bounds at that frame (target velocity within 5.0 and 20.1 m/s,
target heading within pi, ego yaw within 0.048 rad) make the safety program
infeasible. Fresh-admission probes and two bound sweeps locate the binding
element in the ego yaw bound propagated open loop over the horizon.

## Setup

Straight road without boundaries, ego reference 10 m/s, oncoming target from
100 m at 10 m/s with 0.8 m lateral offset, radar range 30 m, 0.1 s holds, 16
cruise stages, deadline 0.1 s enforced per frame, seed 20260913, synthetic
bounded-noise sensors (GNSS 0.04 m and 0.05 m/s, IMU 0.03 m/s^2, gyro
0.0015 rad/s, radar 0.04 m), observer at 80 Hz. The ego plant is the declared
affine generator integrated exactly. The 14-DOF plant suite
`runEstimatedStateAvoidanceScenarios` remains blocked by design: its straight
wrapper always supplies road boundaries, which the recursive cruise
certificate rejects, and its validation configuration declares nonzero plant
residual rates, which the controller rejects as `nonexactStudyInput`.

## Results

| Run | Holds | Outcome | Frames (ms) |
| --- | ---: | --- | --- |
| Exact states, 30 m range gate | 300/300 | completed; target published on 30 frames from t = 3.6 s; speed dips to 8.72 m/s; maximum lateral offset 1.61 m | max 46.7, median 6.1; admission 34.1 |
| NRMM estimator | 36/300 | `optimizationFailed` at t = 3.6 s, first published frame; deficit 83.9; search hit the 0.1 s work limit after two families | max 101.9, median 15.9; observer median 8.9 |

Inter-node clearance in the exact-state run: the 11-point sampled clearance
reaches 0.0188 m below the 0.25 m margin (0.231 m physical clearance) at a
closing speed of 20 m/s (2 m relative motion per hold). The node certificate
does not claim inter-node clearance; this is the first observed margin dip
below zero and is recorded as such.

Published bounds at the failing frame (t = 3.6 s, target 28 m ahead): ego
position 0.076 m, yaw 0.048 rad, speed 0.089 m/s, lateral velocity
0.497 m/s, yaw rate 0.0015 rad/s; target position [0.19, 1.45] m, velocity
[5.0, 20.1] m/s, acceleration [2.5, 2.6] m/s^2, yaw pi, jerk bound from the
target domain. The target estimate itself was accurate (velocity error
0.15 m/s); the bounds are the declared-domain prior of a one-sample
acquisition.

## Fresh-admission probes along the exact trajectory

`probeEstimatorAdmission` replays the adapter on the recorded exact-state
ego trajectory and attempts one fresh admission per frame (5 s budget):

| t (s) | Range (m) | Target velocity bound (m/s) | Target yaw bound (rad) | Deficit |
| ---: | ---: | --- | ---: | ---: |
| 3.6 | 28.0 | [5.00, 20.14] | 3.14 | 86.0 |
| 3.8 | 24.0 | [1.42, 10.40] | 1.37 | 6.4 |
| 4.0 | 20.0 | [1.16, 5.98] | 0.59 | 1.9 |
| 4.2 | 16.0 | [1.18, 4.29] | 0.39 | 1.35 |
| 4.5 | 10.1 | [1.14, 3.28] | 0.27 | 4.4 |
| 4.6 to 6.5 | 8.2 to 28.5 | unavailable (Inf) | Inf | not attempted |

No probe was certified. The longitudinal velocity bound settles near 1.15 m/s
(the 0.5 s measurement-history secant against the 2 m/s^2 target
acceleration domain); the lateral bound follows the slowly shrinking heading
bound. From 8.2 m range through the pass and reacquisition the estimator's
enclosure is unavailable, so the controller would reject the target anyway.
Raising `initialization.minimumRadarSamples` to 8 does not help (first frame
velocity bound [30.0, 20.3] m/s).

## Which bound is binding

`probeEgoBoundSensitivity` (exact-like target, ego bound swept) and
`probeTargetBoundSensitivity` (target bound swept) on the acquisition frame:

| Ego bound | Certified | Deficit | Lateral node box after 36 stages |
| --- | --- | ---: | ---: |
| zero | yes | | 0 |
| estimator (all six) | no | 1.116 | 1.93 m |
| yaw 0.048 rad only | no | 1.071 | 1.73 m |
| position only, speed only, lateral velocity only, yaw rate only | yes | | at most 0.12 m |
| estimator without yaw and lateral velocity | yes | | 0.08 m |
| estimator x 0.5 (yaw 0.024 rad) | no | 0.033 | 0.96 m |
| estimator x 0.25 (yaw 0.012 rad) | yes | | 0.48 m |

The ego yaw bound is the binding element: the controller propagates the
initial box open loop through the exact stage flows, so a heading error of
0.048 rad held for 3.6 s at 10 m/s widens the lateral node box to 1.7 m,
which the terminal set and separation rows cannot absorb. A yaw bound near
0.012 rad (0.7 degrees) is admissible in this scene. The actual yaw error of
the estimator is about 0.0005 rad RMSE; its published bound is 25 to 100
times larger.

With a certifiable ego bound (estimator x 0.1), every target bound in the
sweep is certified, including velocity 3 m/s, acceleration 2 m/s^2 and yaw
1 rad. Inspection shows why: without road boundaries the certified plan
leaves the lane by up to 12.2 m to clear a target box that reaches 7 m at
the pass. The target sweep therefore says nothing about lane-bounded
admissibility; lane-bounded runs need road boundaries, which the current
recursive cruise certificate does not admit.

## Assessment

- Controller runtime is no longer the obstacle: the joint pipeline runs at
  a median of 15.9 ms per frame including the observer, and the exact-state
  admission takes 34 ms.
- The joint closed loop fails because the estimator's certified bounds are
  far wider than its errors: the ego yaw bound (0.048 rad) alone blocks
  admission, and the target bounds at acquisition are domain priors that
  shrink over about a second while the enclosure becomes unavailable during
  the pass.
- Options, none implemented here: tighten the certified ego yaw bound
  (estimator design); condition predicted ego boxes by the measurement
  contract at every node instead of propagating them open loop (a controller
  design change consistent with the terminal set's own premise); admit
  targets only after their bounds tighten (a publication-contract change);
  add road boundaries to the recursive cruise certificate so target-bound
  requirements can be stated for lane-bounded motion.

## Reproduction

```matlab
addpath('scripts');
exact = runDeclaredPlantEstimatorControllerScenario(OutputDirectory=out+"/exact",Warmup=true,UseEstimator=false);
joint = runDeclaredPlantEstimatorControllerScenario(OutputDirectory=out+"/estimator",Warmup=true);
probes = probeEstimatorAdmission(ExactRunFile=out+"/exact/joint-declared-plant.mat");
ego = probeEgoBoundSensitivity(ExactRunFile=out+"/exact/joint-declared-plant.mat");
target = probeTargetBoundSensitivity(ExactRunFile=out+"/exact/joint-declared-plant.mat", ...
    EgoErrorBound=0.1*[0.076;0.076;0.048;0.089;0.497;0.0015]);
```

Raw MAT files, logs and the inspection script are in
`/home/zai/.cache/collisionAvoidance/estimator-in-the-loop-20260917/`; the
compact summary is
[ESTIMATOR_IN_THE_LOOP_ADMISSION_20260917.json](ESTIMATOR_IN_THE_LOOP_ADMISSION_20260917.json).
