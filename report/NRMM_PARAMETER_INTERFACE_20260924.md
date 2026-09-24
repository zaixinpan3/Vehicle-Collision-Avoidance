# NRMM parameter error bounds from the estimator, 2026-09-24

[NRMM_TARGET_PREDICTION_20260924.md](NRMM_TARGET_PREDICTION_20260924.md) made the
certificate's target box enclose NRMM paths, but the controller still derived
the NRMM parameters (speed, course, speed-rate, curvature) of the target from
the estimator's inertial error box. That box is not what the estimator knows.
Its tracker certifies the errors of the target's velocity and acceleration as
component balls in the ego frame; the inertial box adds the ego rotation error
to every axis, and the controller then took the box corner as the ball. At the
user's direction the estimator now publishes the NRMM parameter error bounds
itself, and the controller uses them.

## What changed

1. **Estimator** (`nrmmTargetParameterErrorBounds`, `nrmmControllerErrorBounds`).
   With the `nrmm-motion-v1` contract the target record carries
   `targetSpeedErrorBound`, `targetCourseErrorBound`,
   `targetSpeedRateErrorBound` and `targetCurvatureInterval`, computed from the
   tracker's component balls:
   - speed, speed-rate and normal acceleration do not depend on the frame, so
     they use the velocity and acceleration balls directly;
   - the course adds the ego yaw error to the ball's angle `asin(ball/V)`;
   - the curvature interval is the normal acceleration over the speed squared,
     both over their balls.

   The contract adds `speedRateMaximum`, the domain's speed-rate limit.
2. **Controller** (`readPlanningInputs`, `targetPrediction.nrmmParameters`).
   The encounter carries intervals for the four parameters: the intervals of
   every NRMM state in the box, intersected with the published bounds and the
   declared maxima. `localNrmmState` reads them instead of deriving them.
   Inherited frames propagate the intervals over the hold (`A` and `kappa`
   constant, `V = max(0, V + A h)`, the course advanced by `kappa` times the
   arc) and intersect them with the measurement's
   (`targetPrediction.condition`, `advance`). A larger speed-rate maximum
   voids a carried family (`hardEncounterBarrier`).
3. **Harness** (`runExactStateRecursiveFeasibilityScenario`,
   `runNrmmTargetCampaign`). For NRMM truth targets the velocity and
   acceleration noise is drawn uniformly in discs of the declared radii, as the
   estimator's component balls are; the position, yaw and yaw-rate noise stays
   in boxes. `NrmmParameterBounds=false` withholds the published bounds. The
   campaign gains the `nrmm-boxOnly` variant.
4. **Documentation.** `TARGET_PREDICTION_CONTRACT.md` and
   `ESTIMATOR_BOUND_INTERFACE.md` specify the published bounds; the reachable
   box is described as the intersection of a parameter-Taylor and a time-Taylor
   bound on the same NRMM paths (Section 4).

**Environment.**
- MATLAB R2026a, `matlab -batch`; a clean worktree of `d1735f7` plus the
  change.
- Up to five MATLAB processes ran in parallel, so frame times are not isolated
  measurements.
- Raw outputs are under the git-ignored
  `simulation_output/nrmm_parameter_interface_20260924/`.

## 1. Tests

On the final contents, `runtests('tests')` ran 812 tests: 810 passed, 0 failed
and 2 incomplete (the curb-detection tests skipped by assumption when the
untracked LiDAR dataset is absent).

New and extended tests:
- `nrmmTargetParameterErrorBoundsTest` (2 tests): 400 estimates with 50
  sampled truths each, in discs and in frames rotated within the bound, stay
  inside the published speed, course, speed-rate and curvature bounds; a ball
  containing rest leaves the course and curvature open.
- `nrmmControllerErrorBoundsTest`: for all four reconstruction geometries the
  published bounds contain the fixture's true parameters; for the two regular
  geometries the speed bound is below the inertial box corner.
- `nrmmTargetPredictionTest` (15 tests): published disc bounds tighten every
  parameter interval and the box while 300 sampled NRMM truths in the discs
  stay inside; conditioning propagates the carried intervals, intersects them
  with the measurement's, keeps the true parameters and rejects a
  contradictory curvature interval; a larger speed-rate maximum voids the
  carried family.

## 2. Estimator level: the published bounds against the inertial box

[nrmmEstimatorInterface.m](NRMM_PARAMETER_INTERFACE_20260924/nrmmEstimatorInterface.m)
([output](NRMM_PARAMETER_INTERFACE_20260924/nrmmEstimatorInterface.txt),
[table](NRMM_PARAMETER_INTERFACE_20260924/nrmmEstimatorInterface.csv)) rebuilds
the target reconstruction fixture of `nrmmControllerErrorBoundsTest` (a 15 m/s
target, ego yaw error 0.03–0.04 rad, tracker component errors of about 0.2 m,
0.13 m/s and 0.07 m/s^2, the tracking configuration's design) and admits the
published record twice: from the inertial box alone and with the published
bounds. Position half-width per axis of the reachable box (m):

| Geometry | Parameters | V half-width (m/s) | course (rad) | A (m/s^2) | kappa (1/m) | 1 s | 2 s | 3 s | 4 s | 4.8 s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| rotated | box only | 0.812 | 0.055 | 0.178 | 0.0011 | 1.73 | 2.63 | 3.97 | 6.00 | 8.30 |
| | published | 0.128 | 0.039 | 0.079 | 0.0004 | 1.72 | 2.51 | 3.48 | 4.67 | 5.79 |
| angle branch | box only | 1.034 | 0.069 | 0.212 | 0.0014 | 2.12 | 3.26 | 5.00 | 7.66 | 10.74 |
| | published | 0.128 | 0.048 | 0.080 | 0.0004 | 2.10 | 3.07 | 4.23 | 5.63 | 6.87 |

The box-derived speed half-width is the ego rotation error times the speed
(0.45–0.6 m/s) plus the box corner; the published one is the velocity ball.
The course half-width keeps the ego yaw error (0.03–0.04 rad), which is real:
the target's inertial course is known no better than the ego's own heading.
The two peaking geometries of the fixture (a velocity estimate 190 m/s off and
one at rest) stay enormous in both variants, as they must.

## 3. First-frame collision-record supports

[nrmmSupportGrowth.m](NRMM_PARAMETER_INTERFACE_20260924/nrmmSupportGrowth.m)
([table](NRMM_PARAMETER_INTERFACE_20260924/nrmmSupportGrowth.csv)) is the
first-frame diagnostic of the previous report with two NRMM declarations: the
measurement box alone, and the box with the published bounds of a declarer
whose velocity and acceleration errors are discs of the box radii. Largest
collision-record support in metres, nominal directions, no solve. The last
column is the parameter-Taylor bound alone, from a scratch copy of
`targetPrediction.m` whose `localNrmmFlow` returns before the time-Taylor
intersection ([table](NRMM_PARAMETER_INTERFACE_20260924/nrmmSupportGrowth_parameterTaylorOnly.csv)).

| Case | Errors | Variant | 1 s | 2 s | 4 s | 4.8 s | target box 4.8 s | parameter-Taylor alone, 4.8 s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| crossing, curved road, target curvature 0 | x1 | Cartesian, truth-derived jerk | 0.52 | 0.82 | 1.93 | 2.63 | 1.52 | - |
| | | NRMM, box only | 0.51 | 0.73 | 1.23 | 1.42 | 0.46 | 1.70 |
| | | NRMM, published bounds | 0.51 | 0.73 | 1.23 | 1.42 | 0.46 | 1.49 |
| | | NRMM, published, reactive 30 | 0.51 | 0.70 | 1.26 | 1.46 | 0.46 | - |
| | x3 | Cartesian, truth-derived jerk | 0.85 | 1.32 | 2.79 | 3.67 | 2.43 | - |
| | | NRMM, box only | 0.84 | 1.23 | 2.11 | 2.50 | 1.40 | 3.55 |
| | | NRMM, published bounds | 0.84 | 1.23 | 2.10 | 2.48 | 1.38 | 2.79 |
| | | NRMM, published, reactive 30 | 0.85 | 1.20 | 2.29 | 2.73 | 1.38 | - |
| | x10 | Cartesian, truth-derived jerk | 2.01 | 3.07 | 5.81 | 7.31 | 5.61 | - |
| | | NRMM, box only | 2.00 | 3.03 | 5.57 | 6.94 | 5.29 | 14.05 |
| | | NRMM, published bounds | 2.00 | 3.00 | 5.28 | 6.41 | 4.82 | 8.99 |
| | | NRMM, published, reactive 30 | 2.17 | 3.30 | 7.03 | 8.80 | 4.82 | - |
| crossing, curved road, target curvature 0.02 | x1 | Cartesian, truth-derived jerk | 0.52 | 0.92 | 1.92 | 2.61 | 1.52 | - |
| | | NRMM, box only | 0.51 | 0.88 | 1.62 | 2.03 | 1.04 | 2.08 |
| | | NRMM, published bounds | 0.51 | 0.87 | 1.46 | 1.75 | 0.79 | 1.76 |
| | | NRMM, published, reactive 30 | 0.52 | 0.81 | 1.42 | 1.70 | 0.79 | - |
| | x3 | Cartesian, truth-derived jerk | 0.85 | 1.47 | 2.77 | 3.64 | 2.43 | - |
| | | NRMM, box only | 0.85 | 1.48 | 2.87 | 3.83 | 2.60 | 4.84 |
| | | NRMM, published bounds | 0.84 | 1.46 | 2.65 | 3.43 | 2.30 | 3.66 |
| | | NRMM, published, reactive 30 | 0.88 | 1.40 | 2.83 | 3.52 | 2.30 | - |
| | x10 | Cartesian, truth-derived jerk | 2.00 | 3.41 | 5.77 | 7.24 | 5.61 | - |
| | | NRMM, box only | 2.02 | 3.53 | 7.21 | 10.00 | 8.09 | 16.03 |
| | | NRMM, published bounds | 2.01 | 3.50 | 6.67 | 8.96 | 7.14 | 11.41 |
| | | NRMM, published, reactive 30 | 2.25 | 3.73 | 8.55 | 11.09 | 7.14 | - |
| lead, straight road, target curvature 0.02 | x1 | Cartesian, truth-derived jerk | 0.32 | 0.53 | 0.89 | 1.09 | 0.59 | - |
| | | NRMM, box only | 0.32 | 0.53 | 0.89 | 1.09 | 0.59 | 1.27 |
| | | NRMM, published bounds | 0.32 | 0.53 | 0.86 | 1.03 | 0.56 | 1.06 |
| | | NRMM, published, reactive 30 | 0.32 | 0.52 | 0.85 | 1.03 | 0.56 | - |
| | x3 | Cartesian, truth-derived jerk | 0.63 | 1.01 | 1.65 | 2.00 | 1.50 | - |
| | | NRMM, box only | 0.63 | 1.02 | 1.75 | 2.19 | 1.69 | 3.14 |
| | | NRMM, published bounds | 0.63 | 1.01 | 1.71 | 2.12 | 1.62 | 2.37 |
| | | NRMM, published, reactive 30 | 0.66 | 1.05 | 1.86 | 2.31 | 1.62 | - |
| | x10 | Cartesian, truth-derived jerk | 1.72 | 2.67 | 4.31 | 5.19 | 4.68 | - |
| | | NRMM, box only | 1.73 | 2.70 | 4.90 | 6.29 | 5.78 | 14.32 |
| | | NRMM, published bounds | 1.73 | 2.68 | 4.66 | 5.84 | 5.33 | 8.97 |
| | | NRMM, published, reactive 30 | 2.03 | 3.42 | 6.95 | 8.96 | 5.33 | - |

- **Published bounds.** They tighten the parameter-Taylor bound alone by
  12–37 % at 4.8 s (box only against published, last column) and the
  reachable box by up to 14 % (crossing, target curvature 0.02: 2.03 to
  1.75 m at nominal errors, 10.00 to 8.96 m at ten-fold).
- **Time-Taylor bound.** With the published bounds the parameter-Taylor bound
  alone is 1–5 % (nominal errors) to 27–54 % (ten-fold) wider than the
  intersection. It is the bound that binds for slow targets with large
  errors: a course and a curvature the estimate does not resolve cost the
  parameter-Taylor bound a second-order remainder that the time expansion of
  the same paths does not pay.
- **Reaction.** The reactive tube uses the parameter model only; its supports
  again exceed the ego-only certificate's beyond nominal errors.

## 4. Closed loop with NRMM truth targets

`runNrmmTargetCampaign` (30 cases × 4 variants, 240 holds each, deadline
disabled, seed 20260912). The velocity and acceleration noise is now drawn in
discs, so every variant's truth differs from the previous report's.

Per-case table:
[nrmmTargetCampaign.csv](NRMM_PARAMETER_INTERFACE_20260924/nrmmTargetCampaign.csv).

| Variant | Passed | Failed case |
| --- | --- | --- |
| Cartesian, truth-derived jerk, ego-only | 29 / 30 | oncoming curved, target curvature 0, x10 |
| NRMM, box only, ego-only | 29 / 30 | the same |
| NRMM, published bounds, ego-only | 29 / 30 | the same |
| NRMM, published bounds, reactive first | 29 / 30 | the same |

Every passing run has a positive sampled body gap and CLF residuals of at most
1e-8. The one failure is a fresh admission at t = 2.5 s that Clarabel rejects
with status 2, in every variant.

- **Published bounds in closed loop.** They change no outcome. Against the
  box-only declaration the minimum node gaps move by at most 0.28 m (crossing
  curved, x10: 4.03 to 3.76 m), toward the less conservative side, since the
  time-Taylor bound binds for these slow targets.
- **Curved oncoming, straight-driving target.** With the disc noise
  realization this case moves against the previous report: x1 and x3 now pass
  in every variant (the previous run failed both in the ego-only variants at
  t = 3.25–3.3 s), and x10 fails in every variant (the reactive policy passed
  it before). The re-admission on every second frame, forced by the ego
  measurement contract, makes the outcome depend on the noise realization.
- **Reaction.** With the published bounds the reactive admission is accepted
  at the first frame of every passing case; it changed no outcome and widened
  the pass in the stationary cases (up to 1.5 m more node gap).

## 5. Existing validations

The Cartesian paths are untouched. Each campaign was rerun and compared case
by case with the outputs of `d1735f7` (frame times excluded):

- **Estimator-bound campaign** (`runEstimatorBoundCampaign`): 8/8, every
  value identical.
- **118-case failure-mode sweep**, default order and reaction-first order
  (`runDeclaredPlantFailureSweep`): 103 passed in both, every value identical.
- **Maneuvering-target campaign** (`runTargetMotionCampaign`): 36 of 48,
  every value identical.
- **Recursion campaign** (`runRecursiveSafetyValidation`): 5/5 at 600 holds
  and the 50 ms deadline runs 0/120, 52/120 and 120/120. Only frame times
  differ, and the message of the 52/120 stop, which now reports the solver's
  time limit instead of the frame deadline at the same hold (a timing
  difference under parallel load).

## 6. Why two bounds, both NRMM

The reachable box is the intersection of two bounds on the same set of paths:
every NRMM path whose parameters lie in the intervals and whose initial
position lies in the box.
- The **parameter-Taylor bound** expands the path in the parameters. It
  follows the curve, so it has no term that grows with the cube of time.
- The **time-Taylor bound** expands the path in time from the estimate box.
  Its jerk `hypot(kappa^2 V^3, 3 A kappa V)` is the jerk of an NRMM path, not
  a declared allowance; it is small for slow targets.

Neither admits a motion the other excludes. The intersection is never wider
than either, and Section 3 shows each binding in its regime.

## 7. Limits

- The reactive tube does not use the time-Taylor bound.
- The course error keeps the ego yaw error, and the ego's own uncertainty
  carries the same yaw error; the correlation is not used.
- The estimator's published bounds ignore the measurement-history tightening
  of the inertial box; the intersection with the box-derived intervals keeps
  that tightening.
- The estimator-in-the-loop suite still stops at t = 0 for its recorded,
  unrelated cause (zero-residual plant contract); the published bounds were
  exercised by the fixture of Section 2 and the harness, not by that suite.
