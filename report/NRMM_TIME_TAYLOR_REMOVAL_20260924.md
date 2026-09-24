# Removal of the Cartesian bound from the NRMM target box, 2026-09-24

Between commits `d1735f7` and `72a2ff1` the NRMM target box was the
intersection of two bounds on the same NRMM paths: the linearization in the
parameters with its Lagrange remainder (the parameter-Taylor bound), and the
constant-acceleration extrapolation of the estimate box plus the integrated
NRMM jerk (the time-Taylor bound, written in Cartesian coordinates). At the
user's direction the second bound is removed. The NRMM target box is now the
parameter-Taylor bound alone.

## What changed

- `targetPrediction.localNrmmFlow` returns the parameter-Taylor box; the
  Cartesian extrapolation, the NRMM jerk bound `hypot(kappa^2 V^3, 3 A kappa V)`,
  the yaw-acceleration bound and the stop jump are gone, with their fields in
  `localNrmmState`.
- Two tests that asserted the intersection (a slow target keeps the Cartesian
  enclosure; a target at rest keeps the Cartesian box) are removed. The
  contradictory-yaw-rate test now targets `targetPrediction.nrmmParameters`,
  where the contradiction is detected; before, the intersection had raised the
  error.
- `TARGET_PREDICTION_CONTRACT.md` and `CLOSED_LOOP_TARGET_PREDICTION.md`
  describe the single bound and record the removal.
- The `finite-sensing-motion-v1` contract (a Cartesian motion with a declared
  jerk bound) is unchanged. The estimator still publishes it when it declares
  a model error, and the maneuvering-target and failure-mode campaigns use it.

**Environment.** MATLAB R2026a, `matlab -batch`; a clean worktree of
`72a2ff1` plus the change; up to five MATLAB processes in parallel, so frame
times are not isolated measurements. Raw outputs are under the git-ignored
`simulation_output/nrmm_time_taylor_removal_20260924/`.

## 1. Tests

On the final contents, `runtests('tests')` ran 810 tests: 808 passed, 0 failed
and 2 incomplete (the curb-detection tests skipped by assumption when the
untracked LiDAR dataset is absent). The sampled containment tests of
`nrmmTargetPredictionTest` (five NRMM motions, the published-bound case and
the closed-loop reactive tube) pass against the box alone.

## 2. Cost of the removal

### 2.1 Target box

[nrmmBoxComparison.m](NRMM_TIME_TAYLOR_REMOVAL_20260924/nrmmBoxComparison.m)
([output](NRMM_TIME_TAYLOR_REMOVAL_20260924/nrmmBoxComparison.txt)), the 15 m/s
turning target of the first NRMM report with box-derived intervals. Position
half-width per axis (m):

| Estimator domain | Contract | 1 s | 2 s | 3 s | 4 s | 4.8 s |
| --- | --- | --- | --- | --- | --- | --- |
| sideslip max 0.005 rad | Cartesian, J = 0.383 m/s^3 | 0.41 | 1.11 | 2.67 | 5.49 | 8.89 |
| | `nrmm-motion-v1`, with the removed bound | 0.38 | 0.82 | 1.38 | 2.13 | 2.85 |
| | `nrmm-motion-v1`, now | 0.43 | 0.82 | 1.38 | 2.13 | 2.85 |
| sideslip max 0.015 rad | Cartesian, J = 1.327 m/s^3 | 0.57 | 2.37 | 6.92 | 15.55 | 26.28 |
| | `nrmm-motion-v1`, with the removed bound | 0.38 | 0.83 | 1.42 | 2.22 | 3.02 |
| | `nrmm-motion-v1`, now | 0.43 | 0.83 | 1.42 | 2.22 | 3.02 |

For a fast target the removed bound only mattered at 1 s (0.05 m).

[nrmmEstimatorInterface.m](NRMM_TIME_TAYLOR_REMOVAL_20260924/nrmmEstimatorInterface.m)
([output](NRMM_TIME_TAYLOR_REMOVAL_20260924/nrmmEstimatorInterface.txt)), the
estimator's reconstruction fixture (15 m/s target, ego yaw error 0.03–0.04 rad):

| Geometry | Parameters | 4.8 s box with the removed bound (m) | 4.8 s box now (m) |
| --- | --- | --- | --- |
| rotated | box only | 8.30 | 11.43 |
| | published | 5.79 | 5.79 |
| angle branch | box only | 10.74 | 14.78 |
| | published | 6.87 | 6.87 |

With the estimator's published parameter bounds the box is unchanged; the
removed bound had only covered for the inflated box-derived intervals.

### 2.2 First-frame collision-record supports

[nrmmSupportGrowth.m](NRMM_TIME_TAYLOR_REMOVAL_20260924/nrmmSupportGrowth.m)
([table](NRMM_TIME_TAYLOR_REMOVAL_20260924/nrmmSupportGrowth.csv)), the
first-frame diagnostic of the previous reports. It reproduces exactly the
parameter-Taylor-only table measured there with a scratch copy. Largest
collision-record support in metres, nominal directions, no solve; the second
4.8 s column is the previous report's value with the removed bound.

| Case | Errors | Variant | 1 s | 2 s | 4 s | 4.8 s | 4.8 s with the removed bound | steer / brake reserve |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| crossing, curved road, target curvature 0 | x1 | Cartesian, truth-derived jerk | 0.52 | 0.82 | 1.93 | 2.63 | 2.63 | 0.190 / 0.183 |
| | | NRMM, box only | 0.55 | 0.82 | 1.45 | 1.70 | 1.42 | 0.190 / 0.183 |
| | | NRMM, published bounds | 0.52 | 0.75 | 1.28 | 1.49 | 1.42 | 0.190 / 0.183 |
| | | NRMM, published, reactive 30 | 0.51 | 0.70 | 1.26 | 1.46 | 1.46 | 0.190 / 0.212 |
| | x3 | Cartesian, truth-derived jerk | 0.85 | 1.32 | 2.79 | 3.67 | 3.67 | 0.190 / 0.183 |
| | | NRMM, box only | 0.97 | 1.56 | 2.92 | 3.55 | 2.50 | 0.190 / 0.183 |
| | | NRMM, published bounds | 0.88 | 1.32 | 2.34 | 2.79 | 2.48 | 0.190 / 0.183 |
| | | NRMM, published, reactive 30 | 0.85 | 1.20 | 2.29 | 2.73 | 2.73 | 0.190 / 0.242 |
| | x10 | Cartesian, truth-derived jerk | 2.01 | 3.07 | 5.81 | 7.31 | 7.31 | 0.190 / 0.183 |
| | | NRMM, box only | 2.70 | 4.89 | 10.82 | 14.05 | 6.94 | 0.190 / 0.183 |
| | | NRMM, published bounds | 2.24 | 3.66 | 7.18 | 8.99 | 6.41 | 0.190 / 0.183 |
| | | NRMM, published, reactive 30 | 2.17 | 3.30 | 7.01 | 8.78 | 8.80 | 0.200 / 0.424 |
| crossing, curved road, target curvature 0.02 | x1 | Cartesian, truth-derived jerk | 0.52 | 0.92 | 1.92 | 2.61 | 2.61 | 0.190 / 0.183 |
| | | NRMM, box only | 0.56 | 0.99 | 1.71 | 2.08 | 2.03 | 0.190 / 0.183 |
| | | NRMM, published bounds | 0.53 | 0.90 | 1.46 | 1.76 | 1.75 | 0.190 / 0.183 |
| | | NRMM, published, reactive 30 | 0.52 | 0.81 | 1.42 | 1.70 | 1.70 | 0.190 / 0.210 |
| | x3 | Cartesian, truth-derived jerk | 0.85 | 1.47 | 2.77 | 3.64 | 3.64 | 0.190 / 0.183 |
| | | NRMM, box only | 1.02 | 1.97 | 3.80 | 4.84 | 3.83 | 0.190 / 0.183 |
| | | NRMM, published bounds | 0.91 | 1.64 | 2.93 | 3.66 | 3.43 | 0.190 / 0.183 |
| | | NRMM, published, reactive 30 | 0.88 | 1.40 | 2.82 | 3.51 | 3.52 | 0.190 / 0.240 |
| | x10 | Cartesian, truth-derived jerk | 2.00 | 3.41 | 5.77 | 7.24 | 7.24 | 0.190 / 0.183 |
| | | NRMM, box only | 2.77 | 5.68 | 12.11 | 16.03 | 10.00 | 0.190 / 0.183 |
| | | NRMM, published bounds | 2.31 | 4.44 | 8.78 | 11.41 | 8.96 | 0.190 / 0.183 |
| | | NRMM, published, reactive 30 | 2.25 | 3.73 | 8.48 | 11.00 | 11.09 | 0.201 / 0.430 |
| lead, straight road, target curvature 0.02 | x1 | Cartesian, truth-derived jerk | 0.32 | 0.53 | 0.89 | 1.09 | 1.09 | 0.170 / 0.167 |
| | | NRMM, box only | 0.35 | 0.61 | 1.04 | 1.27 | 1.09 | 0.170 / 0.167 |
| | | NRMM, published bounds | 0.32 | 0.54 | 0.88 | 1.06 | 1.03 | 0.170 / 0.167 |
| | | NRMM, published, reactive 30 | 0.32 | 0.52 | 0.85 | 1.03 | 1.03 | 0.175 / 0.192 |
| | x3 | Cartesian, truth-derived jerk | 0.63 | 1.01 | 1.65 | 2.00 | 2.00 | 0.170 / 0.167 |
| | | NRMM, box only | 0.76 | 1.33 | 2.50 | 3.14 | 2.19 | 0.170 / 0.167 |
| | | NRMM, published bounds | 0.66 | 1.10 | 1.92 | 2.37 | 2.12 | 0.170 / 0.167 |
| | | NRMM, published, reactive 30 | 0.66 | 1.05 | 1.86 | 2.31 | 2.31 | 0.182 / 0.236 |
| | x10 | Cartesian, truth-derived jerk | 1.72 | 2.67 | 4.31 | 5.19 | 5.19 | 0.170 / 0.167 |
| | | NRMM, box only | 2.55 | 4.58 | 10.80 | 14.32 | 6.29 | 0.170 / 0.167 |
| | | NRMM, published bounds | 2.01 | 3.35 | 6.96 | 8.97 | 5.84 | 0.170 / 0.167 |
| | | NRMM, published, reactive 30 | 2.04 | 3.36 | 6.97 | 8.98 | 8.96 | 0.220 / 0.905 |

- With the published bounds the supports at 4.8 s grow by 1–5 % at nominal
  errors, 7–13 % at three-fold and 27–54 % at ten-fold errors.
- At ten-fold errors on these slow targets the NRMM box is now 1.2–1.7 times
  the Cartesian box of a truth-derived jerk bound; at nominal errors it stays
  below it.
- The reactive tube never used the removed bound; its supports change only
  through the record weights of the gain design.

## 3. Closed loop with NRMM truth targets

`runNrmmTargetCampaign` (30 cases × 4 variants, 240 holds each, deadline
disabled, seed 20260912, disc-shaped velocity and acceleration noise).

Per-case table:
[nrmmTargetCampaign.csv](NRMM_TIME_TAYLOR_REMOVAL_20260924/nrmmTargetCampaign.csv).
The Cartesian variant is identical to the previous report's run.

| Variant | Passed | Failed cases |
| --- | --- | --- |
| Cartesian, truth-derived jerk, ego-only | 29 / 30 | oncoming curved, target curvature 0, x10 |
| NRMM, box only, ego-only | 26 / 30 | the same; crossing curved x10 (both target curvatures) at t = 0; oncoming straight x10 at t = 2.55 s |
| NRMM, published bounds, ego-only | 29 / 30 | oncoming curved, target curvature 0, x10 |
| NRMM, published bounds, reactive first | 29 / 30 | the same |

Every passing run has a positive sampled body gap and CLF residuals of at most
1e-8; every failure is a fresh admission that Clarabel rejects with status 2.

- **Box-only declarations** lose three ten-fold cases that passed with the
  removed bound: the two curved crossings are infeasible at admission, the
  straight oncoming target at 2.55 s.
- **Published bounds** keep every outcome. Their minimum node gaps grow by up
  to 2.7 m against the previous run (crossing curved, target curvature 0.02,
  x10: 3.98 to 6.71 m): the wider box makes the certified plans pass wider.
- **Reaction** changes no outcome.

## 4. Existing validations

The changed functions serve NRMM encounters only. Each campaign was rerun and
compared case by case with the outputs of `72a2ff1` (frame times excluded):
the estimator-bound campaign (8/8), the default and reaction-first sweeps
(103/118), the maneuvering-target campaign (36/48) and the recursion
campaign (5/5 at 600 holds; deadline runs 0/120, 52/120, 120/120) are
identical, apart from frame times and the timing-dependent message of the
52/120 deadline stop.

## 5. Limits

- The NRMM box is now looser than the removed intersection wherever the
  estimate resolves the course or the curvature poorly: slow targets, large
  errors, and the first second of any horizon.
- The reactive tube uses the parameter model only, as before.
