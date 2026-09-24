# Closed-loop target prediction: validation, 2026-09-23

Commit `7df7bd50fbfc69f4e6a8b54c641ef06063371607` ("Close the loop on the target
in the prediction") implements
[CLOSED_LOOP_TARGET_PREDICTION.md](../controller/CLOSED_LOOP_TARGET_PREDICTION.md).
It makes two changes to the target part of the prediction:

1. **Declared acceleration maximum.** The target's reachable set uses a
   declared scalar acceleration maximum. Per axis, the deviation from the
   nominal acceleration is bounded by `min(r_a + J t, amax + |a0|)`, so position
   uncertainty grows as `t^2` instead of `t^3` at long horizons. The NRMM
   estimator already publishes such a maximum (2 m/s^2); before this change
   the controller ignored it.
2. **Target-reactive feedback policy.** The policy is
   `u = v + K (xhat - z) + L (shat - s0) + N (ahat - a0)`. The ego and target
   deviations share one zonotope tube, and the collision and exit records use
   the relative deviation. A fresh frame with a target tries it only after the
   ego-only certificate fails; the default order is `[Inf, 30, 100]`.

This record reports measured outcomes on the declared held affine plant only.

**Environment.**
- MATLAB R2026a, `matlab -batch`.
- A clean worktree of `4a8d26d` plus the change; code-state patch SHA-256
  `c9ddae3d721123997b9604a6d693458feef38f6b0bad92b502b14e6994ac6584`. Its `.m`
  changes are identical to those of `7df7bd5`.
- One experiment at a time. MATLAB sessions of other projects were running,
  so timings are not isolated measurements.
- Raw outputs are under the git-ignored
  `simulation_output/target_reaction_validation_20260923_final/`.

## 1. Tests

On the committed contents, `runtests('tests')` ran 789 tests: 787 passed,
0 failed and 2 incomplete. The two incomplete tests are curb-detection tests
skipped by assumption when the untracked LiDAR dataset is absent.

The new class `tests/targetReactionTest.m` has 8 tests:
- **Soundness.** 60 sampled declared-plant trajectories (interior and vertex)
  run under the reactive policy. Each has jerk- and acceleration-limited target
  motion and per-hold ego and target estimator errors. At every node the joint
  ego/target deviation stays inside the predicted zonotope (20 directions per
  node), and the relative position stays inside every collision record's
  support. Executed-input and slew deviations stay inside their reserved
  supports.
- **Relative support.** The reactive tube shrinks the last record's relative
  support.
- **Inherited frames** issue `plan + K (xhat - z) + L (shat - s0) + N (ahat - a0)`
  and store it as the applied input.
- **Contract changes.** A larger target measurement bound, or a larger declared
  acceleration maximum, voids the carried family.
- **Release regression.** A target released after the reaction window
  continues without a target correction. This covers the defect in Section 4.
- **Capped reachability.** The capped radii match a numerical integration of
  the capped deviation bound. Sampled capped targets stay inside the reachable
  box.

Two existing assertions changed:
- A failing fresh admission now makes one solve per configured strength
  (`collisionAvoidanceControllerTest`).
- Format-43 stored states are rejected (`certificateContinuationTest`).

## 2. Maneuvering targets

`scripts/runTargetMotionCampaign.m` runs 12 cases × 4 variants, 240 holds each,
with the deadline disabled. Settings:
- **Ego:** the recorded NRMM estimator bound.
- **Target:** nominal sensing bounds; truth jerk `J cos t` with J = 1 or 2 m/s^3.
- **Declared maximum:** 2 m/s^2 for J = 1 and 3 m/s^2 for J = 2. The truth never
  exceeds `sqrt(2) J`.

"Reactive" means the order `[30, 100, Inf]`. The truth jerk `J cos t`
deliberately violates the controller's target model (constant speed-rate and
constant sideslip, NRMM). The campaign therefore measures robustness to a
declared model mismatch, not the model's design case.

| Case | jerk-only, ego-only (before) | jerk-only, reactive | capped, ego-only | capped, reactive |
| --- | --- | --- | --- | --- |
| stationary, straight, J = 1 | pass, gap 1.537 m | pass, gap 1.588 m | pass, gap 1.323 m | pass, gap 1.393 m |
| stationary, straight, J = 2 | infeasible at t = 0 | infeasible at t = 0 | pass, gap 1.975 m | pass, gap 2.026 m |
| stationary, curvature 0.01, J = 1 | pass, gap 2.315 m | pass, gap 2.346 m | pass, gap 2.464 m | pass, gap 2.541 m |
| stationary, curvature 0.01, J = 2 | infeasible at t = 0 | infeasible at t = 0 | pass, gap 2.601 m | pass, gap 2.563 m |
| oncoming, straight, J = 1 | pass, gap 2.312 m | pass, gap 2.313 m | pass, gap 2.312 m | pass, gap 2.313 m |
| oncoming, straight, J = 2 | pass, gap 7.162 m | pass, gap 7.151 m | pass, gap 7.215 m | pass, gap 7.208 m |
| oncoming, curvature 0.01, J = 1 | pass, gap 2.314 m | pass, gap 2.341 m | pass, gap 2.299 m | pass, gap 2.322 m |
| oncoming, curvature 0.01, J = 2 | infeasible at t = 3.1 s | infeasible at t = 3.1 s | pass, gap 4.240 m | pass, gap 4.258 m |
| crossing, straight, J = 1 | pass, gap 9.532 m | pass, gap 9.532 m | pass, gap 9.532 m | pass, gap 9.532 m |
| crossing, straight, J = 2 | pass, gap 9.538 m | pass, gap 9.538 m | pass, gap 9.538 m | pass, gap 9.538 m |
| crossing, curvature 0.01, J = 1 | infeasible at t = 0 | infeasible at t = 0 | pass, gap 4.813 m | pass, gap 4.456 m |
| crossing, curvature 0.01, J = 2 | infeasible at t = 0 | infeasible at t = 0 | infeasible at t = 0 | infeasible at t = 0 |

Gaps are minimum node body gaps. Every passing run also meets the harness pass
criterion: a positive sampled gap and CLF residuals of at most 1e-8. Per-case table:
[targetMotionCampaign.csv](TARGET_REACTION_VALIDATION_20260923/targetMotionCampaign.csv).

- **Acceleration maximum.** With the jerk-only model, 7 of 12 cases pass.
  Declaring the acceleration maximum resolves 4 of the 5 failures.
- **Reaction.** The reactive policy resolves none of them. Where it was
  accepted, it changed the node gap by −0.357 to +0.078 m.
- **Remaining failure.** The curved crossing with J = 2 (a target that may
  accelerate at 3 m/s^2 in any direction) remains infeasible at admission in
  every variant.

## 3. How the target part grows with prediction time

[recordSupportGrowth.m](TARGET_REACTION_VALIDATION_20260923/recordSupportGrowth.m)
builds each case's first frame with the nominal directions and no solve. The
table gives the largest collision-record support `||G' n||_1 + rho` at each
prediction time, in metres.

| Case | Variant | 1 s | 2 s | 3 s | 4 s | 4.8 s | steer / brake reserve |
| --- | --- | --- | --- | --- | --- | --- | --- |
| stationary, straight, J = 2 | jerk-only, ego-only | 0.65 | 3.32 | 9.64 | 22.15 | 37.82 | 0.170 / 0.167 |
| | capped, ego-only | 0.65 | 3.27 | 8.50 | 16.91 | 25.79 | 0.170 / 0.167 |
| | capped, reactive 30 | 0.61 | 3.30 | 7.29 | 14.98 | 23.63 | 0.179 / 0.277 |
| | capped, reactive 100 | 0.65 | 3.35 | 8.21 | 16.42 | 25.27 | 0.171 / 0.204 |
| stationary, curvature 0.01, J = 2 | jerk-only, ego-only | 0.91 | 3.89 | 11.58 | 26.95 | 46.74 | 0.190 / 0.183 |
| | capped, ego-only | 0.91 | 3.84 | 10.23 | 20.63 | 31.94 | 0.190 / 0.183 |
| | capped, reactive 30 | 0.86 | 3.98 | 8.91 | 18.52 | 29.60 | 0.204 / 0.315 |
| | capped, reactive 100 | 0.91 | 3.96 | 9.91 | 20.11 | 31.40 | 0.194 / 0.223 |
| crossing, curvature 0.01, J = 1 | jerk-only, ego-only | 0.69 | 2.25 | 5.50 | 13.33 | 22.46 | 0.190 / 0.183 |
| | capped, ego-only | 0.69 | 2.25 | 5.33 | 11.80 | 18.24 | 0.190 / 0.183 |
| | capped, reactive 30 | 0.67 | 2.08 | 4.80 | 10.99 | 17.42 | 0.210 / 0.263 |
| | capped, reactive 100 | 0.69 | 2.23 | 5.22 | 11.65 | 18.12 | 0.196 / 0.206 |

The ego part of these supports stays bounded (feedback tube). What grows is the
declared jerk term, which in this campaign is the truth target's declared
model mismatch (jerk `J cos t`), not a free maneuver of an NRMM target:
- The acceleration maximum turns the `t^3` growth into `t^2`.
- At the reaction strengths the actuator budget allows, the reactive policy
  trims the late supports by about 1–2 m, at an extra 0.04–0.13 of braking
  reserve.
- To keep the relative support bounded, the ego would have to match
  accelerations of up to 3 m/s^2 in any direction. That needs a reserve the
  maneuvers and the terminal cruise set cannot spare.

## 4. Existing validations, default order

- **Estimator-bound campaign** (`runEstimatorBoundCampaign`): 8/8 pass. Every node
  gap, sampled gap, terminal margin and CLF residual is identical to the 6c244dd
  run.
- **118-case failure-mode sweep**: identical case by case to
  [CONTROLLER_FAILURE_MODE_SWEEP_FEEDBACK_20260923.csv](CONTROLLER_FAILURE_MODE_SWEEP_FEEDBACK_20260923.csv).
  Outcomes are 103 passed, 9 overlaps and 6 errors, with a largest metric
  difference of 0. The six admission failures (including case 54) also tried
  reaction strengths 30 and 100 and remained infeasible.
- **Reaction-first order** (`[30, 100, Inf]`): the same 103/9/6 outcomes. Node
  gaps change by at most 0.051 m, and by more than 0.01 m in 5 of 86 cases
  ([sweepReactiveFirst.csv](TARGET_REACTION_VALIDATION_20260923/sweepReactiveFirst.csv)).
  - An earlier reaction-first run on the pre-fix code errored in cases 94 and 96
    ("Dot indexing is not supported"). The cause: an inherited frame after a
    target release read the released target while its remaining reaction gains
    were zero.
  - The fix skips the target correction once the reaction has ended. It was
    made before the recorded runs and is covered by the regression test.
- **Recursion campaign** (`runRecursiveSafetyValidation`): all five cases pass
  600/600 holds, with every recorded value identical to the feedback-prediction
  run. The 50 ms deadline runs complete 0/120, 52/120 and 120/120, as before.

## 5. Conclusion

With feedback, the ego's prediction tube stays bounded. The target's part
does not stay bounded in these runs, because their truth targets deliberately
violate the constant-acceleration model and the certificate must cover that
declared jerk. Under the controller's NRMM model, the target's future path is
fixed by its current state. Its growth is then the current estimate error,
which later estimates resolve, plus the certificate's Cartesian enclosure of
the turning (Section 7).

In these runs, the effective change was to use the acceleration maximum the
estimator already publishes. It resolved 4 of the 5 maneuvering-target
failures and changed none of the existing results.

The target-reactive policy is sound and carried recursively. At the actuator
reserve available here, it only trims the late supports and changed no
outcome. It is kept as a fallback after the ego-only certificate, so it costs
nothing when unused.

## 6. Remaining limits

- **Terminal and exit structure.** The certificate ends in a target-independent
  cruise set and must confirm, at the final node, that the target is outside
  the confirmation range for every admissible target motion. This is where the
  target's growth binds. A target-aware terminal set would remove the need to
  predict the target that far.
- **Per-frame re-anchoring.** Inherited frames keep the admission-time target
  sets. Re-admitting from the latest estimate when feasible would make later
  frames less conservative. This is not implemented.
- **Remaining failures.** The curved J = 2 crossing, ten-fold case 54, the
  road-boundary rejections, the straight-road inter-node overlaps and the case 96
  chart error remain.
- **Estimator-in-the-loop and PassVeh14DOF suites.** These still stop on their
  earlier contract causes and were not rerun.

## 7. Correction (2026-09-23)

An earlier version of Sections 3 and 5 attributed the target's growth to "the
target's own maneuver freedom". That reading is wrong for the controller's
target model:
- **Model.** The model is NRMM: constant speed-rate and constant sideslip. The
  path is determined by the current state.
- **Jerk bound.** The estimator's published jerk bound,
  `hypot(vmax wmax^2, 3 amax wmax) + modelJerkMaximum` with
  `modelJerkMaximum = 0` by default, is the Cartesian jerk of an NRMM turn. It
  is not driver freedom.
- **Campaign truth.** The jerk in this campaign is a deliberate model mismatch
  of the harness truth.

[nrmmVersusCartesian.m](TARGET_REACTION_VALIDATION_20260923/nrmmVersusCartesian.m)
([output](TARGET_REACTION_VALIDATION_20260923/nrmmVersusCartesian.txt)) compares,
for one turning NRMM target, the certificate box with the constant-curvature
family envelope `targetPrediction.errorEnvelope` for the same estimate errors.

| Estimator domain | Position half-width per axis at 4.8 s | of which `J t^3/6` |
| --- | --- | --- |
| estimator-in-the-loop domain | 8.9 m | 7.1 m |
| `nrmmTrackingConfig` domain | 26.3 m | 24.5 m |
| family envelope, same estimate errors | 4.7 m | none; all current estimate error |

The measured outcomes in Sections 1–4 are unchanged. The model-consistent
certificate is described, not implemented, in Section 7 of
[CLOSED_LOOP_TARGET_PREDICTION.md](../controller/CLOSED_LOOP_TARGET_PREDICTION.md).


## 8. Correction (2026-09-24): the published acceleration maximum

The introduction and Section 5 say the NRMM estimator "already publishes" the
acceleration maximum. The 2 m/s^2 it published under `scalarAccelerationMaximum` is its
speed-rate maximum `|A|`, while the controller reads the field as a bound on
`|a|`. For a turning target `|a| = hypot(A, V omega)` reaches 2.36 m/s^2 in the
estimator-in-the-loop domain and 4.25 m/s^2 in the `nrmmTrackingConfig` domain,
so the published value was not a valid cap there.

Since 2026-09-24 the estimator publishes `accelerationNormBound`
([NRMM_TARGET_PREDICTION_20260924.md](NRMM_TARGET_PREDICTION_20260924.md)). The
campaigns of this record declared their own maxima, with truths inside them, so
the measured outcomes are unaffected.
