# NRMM-consistent target prediction: implementation and validation, 2026-09-24

The controller's target model is NRMM: constant speed-rate and constant
sideslip. Until this change, the certificate enclosed such a target with a
Cartesian constant-acceleration path plus a jerk bound. That bound is the
Cartesian jerk of an NRMM turn, so the `J t^3/6` term of the certificate was an
artifact of the enclosure, not a maneuver of the target (correction of
2026-09-23 in [TARGET_REACTION_VALIDATION_20260923.md](TARGET_REACTION_VALIDATION_20260923.md)).
This change implements the model-consistent prediction described in Section 7 of
[CLOSED_LOOP_TARGET_PREDICTION.md](../controller/CLOSED_LOOP_TARGET_PREDICTION.md).
It also fixes the acceleration cap the estimator published.

## What changed

1. **Contract `nrmm-motion-v1`** (`targetPrediction.admit`). It declares exact
   NRMM motion: a constant speed-rate, constant-curvature path through the
   current state with `|kappa| <= curvatureMaximum`, stopping and holding at
   zero speed. It carries no jerk or yaw-acceleration allowance: a varying
   speed-rate or curvature leaves every NRMM path through a later estimate, so
   a model error keeps the Cartesian contract.
2. **Target box** (`targetPrediction.finiteFlow`). The box encloses every NRMM
   path through the estimate box:
   - the NRMM parameter enclosure: sound intervals for `p0`, `V`, course, `A`
     and `kappa`, their linearization along the path with a Lagrange
     second-order remainder, or a path-length ball where the linearization does
     not apply;
   - intersected with the Cartesian enclosure of the same paths. That
     enclosure uses the jerk `hypot(kappa^2 V^3, 3 A kappa V)` and yaw
     acceleration `|A kappa|` over the parameter intervals, plus the jump of
     the acceleration to zero wherever a path may have stopped.

   There is no `J t^3/6` term; the growth is the extrapolation of the current
   estimate error along the NRMM path.
3. **Target-reactive tube** (`targetPrediction.deviationModel`,
   `ltvBicycleModel.reactionGains`, `ltvBicycleModel.reactiveTube`). The target
   deviation is split into three parts:
   - generators of the error sources fixed at admission (the NRMM parameters,
     or the Cartesian initial box);
   - a remainder box at each record node;
   - a per-hold acceleration part.

   The reaction to the remainder of each hold enters as its own generators. An
   intermediate version propagated it as an interval through `|A + B K|`. Over
   a 6.4 s horizon with ten-fold errors that inflated the steering reserve to
   362; with generators it is 0.257 (Section 3). The Cartesian contract has no
   remainder, and its tube is unchanged.
4. **Estimator publication** (`nrmmControllerErrorBounds`).
   - It publishes `nrmm-motion-v1` when `modelJerkMaximum = 0` (the default),
     with `curvatureMaximum = sin(beta_max)/l_r`, and the Cartesian contract
     otherwise.
   - **Fix to commit 7df7bd5.** `scalarAccelerationMaximum`, which the
     controller reads as a bound on `|a|`, carried the speed-rate maximum
     (2 m/s^2). For a turning target `|a| = hypot(A, V omega)` reaches
     2.36 m/s^2 (estimator-in-the-loop domain) or 4.25 m/s^2
     (`nrmmTrackingConfig` domain). The cap is now `accelerationNormBound`.
5. **Harness** (`runExactStateRecursiveFeasibilityScenario`). It adds NRMM
   truth targets: `TargetMotionModel="nrmm"`, `TargetSpeedRate`,
   `TargetCurvature` and `TargetCurvatureMaximum`. `NrmmContract="cartesian"`
   declares the same truth under a Cartesian contract whose jerk bound
   `hypot(kappaMax^2 V^3, 3 |A| kappaMax V)` uses the truth's speed and
   speed-rate. `scripts/runNrmmTargetCampaign.m` is the new campaign.
6. **Documentation.**
   - `TARGET_PREDICTION_CONTRACT.md` specifies the contract.
   - `CLOSED_LOOP_TARGET_PREDICTION.md` corrects Section 3 and marks
     Section 7 implemented.
   - `CONTROLLER_FILES.md` is updated.

**Environment.**
- MATLAB R2026a, `matlab -batch`.
- A clean worktree of `a62fbf9` plus the change.
- Up to five MATLAB processes ran in parallel, beside other projects'
  sessions, so frame times are not isolated measurements.
- Raw outputs are under the git-ignored
  `simulation_output/nrmm_target_prediction_20260924/`.

## 1. Tests

On the final contents, `runtests('tests')` ran 804 tests: 802 passed, 0 failed
and 2 incomplete. The two incomplete tests are curb-detection tests skipped by
assumption when the untracked LiDAR dataset is absent. The existing target,
reaction and prediction tests pass unchanged.

The new class `tests/nrmmTargetPredictionTest.m` has 13 tests:
- **Containment.** For five NRMM motions (turning, reversed curvature, braking,
  nearly stopped, stopping), 300 sampled NRMM truths per motion stay inside the
  box and inside the linear model plus remainder at 51 times up to 4.8 s. The
  truths include box vertices and the times around a stop.
- **Enclosure comparisons.**
  - A possible stop falls back to the path-length ball.
  - At 10 m/s the box at 4.8 s is below a tenth of the Cartesian turning-jerk
    box.
  - A slow target with ten-fold errors keeps the Cartesian enclosure of its
    turning, and a target estimated at rest keeps the Cartesian box.
- **Contract checks.** A yaw rate that no admissible curvature explains is an
  inconsistent observation. An NRMM contract with a jerk allowance is rejected.
- **Closed-loop tube.** 60 declared-plant trajectories run under the reactive
  policy with NRMM truths and per-hold estimator errors. They stay inside the
  joint tube, the records and the reserved input and slew supports.
- **Carried family.** A change of motion kind, or a larger curvature maximum,
  voids the carried family.

`tests/nrmmControllerErrorBoundsTest.m` adds two tests: the published NRMM
contract with its magnitude cap, and the Cartesian contract when a model error
is declared.

## 2. Target box: Cartesian jerk contract versus `nrmm-motion-v1`

[nrmmBoxComparison.m](NRMM_TARGET_PREDICTION_20260924/nrmmBoxComparison.m)
([output](NRMM_TARGET_PREDICTION_20260924/nrmmBoxComparison.txt)) uses the target
of the 2026-09-23 correction:
- NRMM at 15 m/s, sideslip 0.005 rad, speed-rate 1 m/s^2;
- estimate errors 0.2 m, 0.1 m/s and 0.1 m/s^2 per axis, 0.01 rad and
  0.005 rad/s.

The contracts are those the estimator publishes for its two domains.

Position half-width per axis (m):

| Estimator domain | Contract | 1 s | 2 s | 3 s | 4 s | 4.8 s |
| --- | --- | --- | --- | --- | --- | --- |
| sideslip max 0.005 rad | Cartesian, J = 0.383 m/s^3 | 0.41 | 1.11 | 2.67 | 5.49 | 8.89 |
| | `nrmm-motion-v1` | 0.38 | 0.82 | 1.38 | 2.13 | 2.85 |
| sideslip max 0.015 rad | Cartesian, J = 1.327 m/s^3 | 0.57 | 2.37 | 6.92 | 15.55 | 26.28 |
| | `nrmm-motion-v1` | 0.38 | 0.83 | 1.42 | 2.22 | 3.02 |

The NRMM box is also below the 4.71 m constant-curvature family envelope
reported on 2026-09-23. With the 0.005 rad domain its yaw half-width at 4.8 s
is 0.033 rad.

## 3. First-frame collision-record supports

[nrmmSupportGrowth.m](NRMM_TARGET_PREDICTION_20260924/nrmmSupportGrowth.m)
([table](NRMM_TARGET_PREDICTION_20260924/nrmmSupportGrowth.csv)) builds each
case's first frame with the nominal directions and no solve. It reports the
largest collision-record support `||G' n||_1 + rho` in metres. The cases are:
- a crossing target at 4 m/s on the curved road (curvature 0.01);
- a lead target at 2 m/s, 15 m ahead on the straight road.

Both use `curvatureMaximum = 0.03`, and target estimate errors 1, 3 and 10 times
the nominal `[0.1 m, 0.05 m/s, 0.01 m/s^2, 0.01 rad, 0.01 rad/s]`.

The Cartesian baseline declares the jerk `kappaMax^2 V^3` from the truth's speed
and zero speed-rate. It is the tightest constant jerk bound a declarer who knows
the truth could state. The NRMM contract instead derives `V`, `A` and `kappa`
from the estimate box.

| Case | Errors | Variant | 1 s | 2 s | 4 s | 4.8 s | target box 4.8 s | steer / brake reserve |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| crossing, target curvature 0 | x1 | Cartesian, truth-derived jerk, ego-only | 0.52 | 0.82 | 1.93 | 2.63 | 1.52 | 0.190 / 0.183 |
| | | NRMM, ego-only | 0.51 | 0.73 | 1.23 | 1.42 | 0.46 | 0.190 / 0.183 |
| | | NRMM, reactive 30 | 0.53 | 0.75 | 1.41 | 1.66 | 0.46 | 0.190 / 0.215 |
| | x3 | Cartesian, truth-derived jerk, ego-only | 0.85 | 1.32 | 2.79 | 3.67 | 2.43 | 0.190 / 0.183 |
| | | NRMM, ego-only | 0.84 | 1.23 | 2.11 | 2.50 | 1.40 | 0.190 / 0.183 |
| | | NRMM, reactive 30 | 0.93 | 1.38 | 2.84 | 3.45 | 1.40 | 0.190 / 0.253 |
| | x10 | Cartesian, truth-derived jerk, ego-only | 2.01 | 3.07 | 5.81 | 7.31 | 5.61 | 0.190 / 0.183 |
| | | NRMM, ego-only | 2.00 | 3.03 | 5.57 | 6.94 | 5.29 | 0.190 / 0.183 |
| | | NRMM, reactive 30 | 2.60 | 4.40 | 10.60 | 13.77 | 5.29 | 0.200 / 0.492 |
| crossing, target curvature 0.02 | x1 | Cartesian, truth-derived jerk, ego-only | 0.52 | 0.92 | 1.92 | 2.61 | 1.52 | 0.190 / 0.183 |
| | | NRMM, ego-only | 0.51 | 0.88 | 1.62 | 2.03 | 1.04 | 0.190 / 0.183 |
| | | NRMM, reactive 30 | 0.54 | 0.87 | 1.65 | 2.01 | 1.04 | 0.190 / 0.213 |
| | x3 | Cartesian, truth-derived jerk, ego-only | 0.85 | 1.47 | 2.77 | 3.64 | 2.43 | 0.190 / 0.183 |
| | | NRMM, ego-only | 0.85 | 1.48 | 2.87 | 3.83 | 2.60 | 0.190 / 0.183 |
| | | NRMM, reactive 30 | 0.98 | 1.63 | 3.66 | 4.65 | 2.60 | 0.190 / 0.250 |
| | x10 | Cartesian, truth-derived jerk, ego-only | 2.00 | 3.41 | 5.77 | 7.24 | 5.61 | 0.190 / 0.183 |
| | | NRMM, ego-only | 2.02 | 3.53 | 7.21 | 10.00 | 8.09 | 0.190 / 0.183 |
| | | NRMM, reactive 30 | 2.66 | 4.72 | 11.80 | 15.61 | 8.09 | 0.200 / 0.484 |
| lead, target curvature 0.02 | x1 | Cartesian, truth-derived jerk, ego-only | 0.32 | 0.53 | 0.89 | 1.09 | 0.59 | 0.170 / 0.167 |
| | | NRMM, ego-only | 0.32 | 0.53 | 0.89 | 1.09 | 0.59 | 0.170 / 0.167 |
| | | NRMM, reactive 30 | 0.35 | 0.58 | 1.00 | 1.23 | 0.59 | 0.175 / 0.196 |
| | x3 | Cartesian, truth-derived jerk, ego-only | 0.63 | 1.01 | 1.65 | 2.00 | 1.50 | 0.170 / 0.167 |
| | | NRMM, ego-only | 0.63 | 1.02 | 1.75 | 2.19 | 1.69 | 0.170 / 0.167 |
| | | NRMM, reactive 30 | 0.75 | 1.27 | 2.41 | 3.05 | 1.69 | 0.186 / 0.253 |
| | x10 | Cartesian, truth-derived jerk, ego-only | 1.72 | 2.67 | 4.31 | 5.19 | 4.68 | 0.170 / 0.167 |
| | | NRMM, ego-only | 1.73 | 2.70 | 4.90 | 6.29 | 5.78 | 0.170 / 0.167 |
| | | NRMM, reactive 30 | 2.60 | 4.70 | 11.00 | 14.54 | 5.78 | 0.257 / 0.963 |

- **NRMM, ego-only.**
  - At nominal errors it is as tight as the truth-derived Cartesian bound or
    tighter (crossing at 4.8 s: 1.42 against 2.63 m).
  - With larger errors on these slow turning targets it can be larger, by up to
    2.8 m at 4.8 s (crossing, target curvature 0.02, x10: 10.00 against
    7.24 m). The box does not know the truth's speed and speed-rate. At ten-fold
    errors it only knows `V` within 0.71 m/s, `|A|` up to 0.20 m/s^2 and `kappa`
    up to the 0.03 maximum, so the jerk it can prove is 0.19 m/s^3, against the
    truth's 0.058 m/s^3.
  - By construction the box is never wider than the Cartesian box whose jerk is
    derived from the same estimate.
- **NRMM, reactive.**
  - The reactive tube uses the NRMM parameter model without the Cartesian
    intersection. Its target part at 4.8 s is 11.6 m where the intersected box
    is 5.3 m (crossing, x10).
  - It also pays for reacting to per-hold estimator errors. At nominal errors
    it changes the supports by -0.02 to +0.24 m; at 3 and 10 times it is
    larger, by up to 8.3 m at 4.8 s.
  - In the default order `[Inf, 30, 100]` it is tried only after the ego-only
    certificate fails.
- **Reserve.** Before the remainder fix, the intermediate interval version gave
  a steering reserve of 3.49 (x1) and 362 (x10) for the lead case's 128-hold
  horizon. It now gives 0.175 and 0.257.

## 4. Closed loop with NRMM truth targets

`runNrmmTargetCampaign` covers 30 cases × 3 variants, 240 holds each, with the
deadline disabled and seed 20260912 (per-frame uniform noise).
- **Cases.** Scenarios: stationary, oncoming (8 m/s) and crossing (32 m/s
  straight, 4 m/s curved). Roads: straight and curvature 0.01. Truth target
  curvature 0 or 0.02 (stationary: 0). Speed-rate 0. Target estimate errors 1,
  3 and 10 times nominal.
- **Ego bound.** The recorded estimator bound.
- **Variants.**
  - `cartesian`: truth-derived jerk bound, ego-only;
  - `nrmm-egoOnly`;
  - `nrmm-reactive`: order `[30, 100, Inf]`.

Per-case table:
[nrmmTargetCampaign.csv](NRMM_TARGET_PREDICTION_20260924/nrmmTargetCampaign.csv).

| Variant | Passed | Failed cases |
| --- | --- | --- |
| Cartesian, truth-derived jerk, ego-only | 26 / 30 | oncoming curved, target curvature 0, x1, x3, x10; stationary curved x3 |
| NRMM, ego-only | 27 / 30 | oncoming curved, target curvature 0, x1, x3, x10 |
| NRMM, reactive first | 29 / 30 | oncoming curved, target curvature 0, x1 |

Every passing run has a positive sampled body gap and CLF residuals of at most
1e-8. Every failure is a fresh admission whose SOCP Clarabel rejects with
status 2.

- **Stationary curved, x3.** The Cartesian variant fails its admission at
  t = 1.05 s. Both NRMM variants pass.
- **Oncoming curved road, straight-driving target.** The target drives
  straight while the road curves.
  - During the avoidance maneuver, the ego's Frenet measurement box exceeds
    the bound the carried certificate was built for on every second frame, so
    a new certificate is required each time. A logged replay of the x3 NRMM
    ego-only run shows `measurementContractChanged` at holds 60, 62 and 64,
    exactly the fresh frames. This is the feedback-tube contract of 6c244dd,
    not the target model.
  - One such re-admission is infeasible in every ego-only variant at every
    scale: at t = 3.25–3.3 s, or at 2.6 s for the Cartesian variant at x10.
  - The reactive policy made these re-admissions feasible at x3 (11 reactive
    admissions) and x10 (9), but not at x1.
- **Other cases.** All pass in all variants. Minimum node gaps differ between
  the two ego-only variants by up to 1.01 m (crossing curved, x10: 6.05 m
  Cartesian, 5.05 m NRMM). They differ between the NRMM variants by up to
  1.73 m where the reactive admission chose a wider pass (crossing curved,
  target curvature 0.02, x10: 7.22 against 5.49 m).

## 5. Existing validations

Each result is compared case by case with the 2026-09-23 outputs
(`simulation_output/target_reaction_validation_20260923_final/`), frame times
excluded. None of these campaigns uses an NRMM contract. They cover the
Cartesian paths, including the restructured reactive tube.

- **Estimator-bound campaign** (`runEstimatorBoundCampaign`): 8/8 pass, with
  every recorded value identical.
- **118-case failure-mode sweep, default order**
  (`runDeclaredPlantFailureSweep`): 103 passed, 9 overlaps and 6 errors,
  identical case by case with no changed value.
- **Reaction-first sweep** (`inputWeightScales = [30, 100, Inf]`): the same
  outcome, failure identifier and message in all 118 cases.
  - 23 cases differ numerically, by at most 2.5e-6; node gaps by at most
    7.5e-8 m.
  - This is rounding from the restructured reactive tube.
- **Maneuvering-target campaign** (`runTargetMotionCampaign`): 36 of 48 runs
  pass, as before. Only 17 rows of the two reactive variants differ, by at most
  4.1e-6.
- **Recursion campaign** (`runRecursiveSafetyValidation`):
  - all five cases pass 600/600 holds, with every value other than frame times
    identical;
  - the 50 ms deadline runs complete 0/120, 52/120 and 120/120, as before.

## 6. What the estimator feedback at every hold buys

- **Within one certificate.** The certificate must hold for every target
  motion the model admits from the admission estimate, so it cannot assume
  future estimates it does not have yet. Under `nrmm-motion-v1` its target part
  is the current estimate error carried along the NRMM path: speed error times
  t, course error times V t, speed-rate error times t^2/2 and curvature error
  times V^2 t^2/2. The Cartesian artifact is gone (Section 2).
- **Across frames.** Every fresh admission starts from the current estimate,
  whose error does not grow. The error of the admission estimate does not carry
  over.
- **Through reaction.** Future estimates enter a certificate only through the
  target-reactive policy. On the declared plant it resolved two of the three
  remaining NRMM failures (Section 4), but it did not shrink the first-frame
  supports (Section 3). The reasons are structural:
  1. The estimator errors are worst-case and independent per hold, so the
     reaction also answers sign-switching errors.
  2. The remainder cannot be followed.
  3. Keeping the relative deviation small moves the target's spread into the
     ego's absolute position, where the lane, actuator and terminal cruise
     constraints bind.

## 7. Limits

- **Reactive tube.** It does not use the Cartesian intersection, and at 3 and
  10 times the nominal errors its supports exceed the ego-only certificate's.
- **Carried certificates.** Inherited frames keep the admission-time records.
  Re-admitting from the latest estimate whenever feasible, with the carried
  certificate as fallback, is not implemented.
- **Model error.** An NRMM model error (a nonzero speed-rate or curvature rate)
  keeps the Cartesian contract and its jerk term.
- **Whole-hold cells.** They are not supported for NRMM targets.
- **Not addressed.** The curved oncoming failure at x1 in every variant, and
  the stationary-curved Cartesian failure, are not addressed.
- **Estimator-in-the-loop suite.** `runEstimatedStateAvoidanceScenarios` was
  rerun and still stops at t = 0 in both scenarios, before any target handling,
  with "This controller requires the declared zero-residual held affine plant".
  The PassVeh14DOF scenarios were not rerun.
