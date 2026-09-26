# Failure after trajectory relinearization — September 25, 2026

The three failed frames have a common failure mechanism: newly linearized
vehicle dynamics are combined with the existing cruise-derived feedback gain
in the prediction-error enclosure. Near maximum longitudinal tire utilization,
the combined-slip tire derivative with respect to braking ratio is very large.
The resulting prediction-error maps amplify tiny arithmetic allowances until
the required feedback reserve exceeds actuator authority or the pose enclosure
crosses the reference chart's regularity domain.

This is a concrete missing compatibility check in the one-pass update. The
nominal dynamics were refreshed, but the prediction-side feedback law was not
redesigned or verified against the new sequence of stage matrices. The diagnosis
does not establish failure of the modified Fiala force saturation itself or
infeasibility of the underlying nonlinear avoidance task.

## Exact replay and evidence

Replayed the saved observations, target estimates and road geometry from
`trajectory-linearization-20260925/{straight,circular,sCurve}.mat`, using explicit
controller state and the recorded computational thread count. Non-pausing
conditional breakpoints captured the formulated model, failed program and
S-curve domain immediately before rejection. Final replays reproduced all
three failure identifiers/times and every previously issued input exactly:
the maximum command difference was zero in each case.

An initial replay omitted the preceding held input and stopped on the execution
contract; this was corrected to supply the replay state's issued input. An
earlier straight replay with a pre-existing model cache differed by up to
0.0104 in actuator-input infinity norm and did not reproduce the final failure.
Clearing `ltvBicycleModel` before rebuilding its cache under the recorded thread
setting produced the exact replay reported here. These preliminary runs are not
used as evidence for the original failed frame's constraints.

All three original scenarios are truth-fed. Their failed models have zero
initial ego error bounds, zero estimator error bounds, and zero held process
reserves. The nonzero seed of the growing interval is the arithmetic allowance
retained by `ltvBicycleModel.finitePredict`, not NRMM acquisition uncertainty.

## Mechanism

The nonlinear combined-slip capacity is proportional to

    eta(beta) = sqrt(1 - beta^2).

On the saturated tire branch, the force magnitude decreases with this capacity,
but the magnitude of its derivative with respect to beta is proportional to
`abs(beta)/sqrt(1-beta^2)`. Saturation limits the force; it does not make this
input derivative small. The retained plans contain inputs near beta = +/-0.99999.
Peak axle-force beta derivatives are approximately 1.328e6 N per unit beta.
The straight/circular/S-curve horizons contain 11/30/12 holds with abs(beta)>0.99.

`feedbackContract` still constructs K from the cruise certificate, with the
existing speed-column scaling and lateral-velocity-column setting. From the
second prediction hold onward, the deviation dynamics use

    e_next = (A_k + B_k K) e + B_k K eta_estimator,
    r_next = abs(A_k + B_k K) r + process_reserve + arithmetic_allowance.

The peak spectral radii of the signed error maps are 29.02, 29.09 and 31.31,
respectively. Their absolute-map spectral radii are 29.06, 29.11 and 31.33.
Thus the observed growth cannot be attributed only to interval re-boxing:
individual signed maps themselves have strongly amplifying modes. Per-stage
spectral radii alone are not a general stability proof for a time-varying
system; the actual propagated bounds below establish the growth in these runs.

The maximum per-stage arithmetic increments across coordinates are only
2.227e-9, 1.968e-8 and 3.023e-9, respectively. Repeated propagation nevertheless
produces the following end-to-end enclosure maxima:

| Scenario | Failure time (s) | Horizon holds | Maximum nominal lateral magnitude (m) | Maximum lateral error bound (m) |
| --- | ---: | ---: | ---: | ---: |
| Straight | 1.75 | 64 | 3.9567 | 54.0719 |
| Circular | 1.40 | 72 | 4.3538 | 2.9199e26 |
| S-curve | 1.30 | 72 | 0.5961 | 8.6954e6 |

These error bounds are computed enclosures, not physical lateral displacements
or measured estimator errors. The nominal path is finite and modest while the
enclosure becomes unusable.

## Why each frame stops

### Straight and circular: contradictory actuator bounds

The robust actuator rows reserve room for the feedback correction. For steering
limit d_max and feedback-support radius r_u, the nominal steering must satisfy

    -d_max + r_u <= delta_nominal <= d_max - r_u.

The existing limit is 0.6981317 rad (40 degrees). In the failed straight program,
hold 25, 1.20 s into the predicted future, already requires r_u = 0.7722926 rad
(about 44.25 degrees). Its lower limit is positive and its upper limit negative.
The circular program reaches the same contradiction at hold 14, 0.65 s into
the predicted future, with r_u = 3.6300200 rad.

This was checked directly against the failed programs' physical actuator rows.
Adding each opposing pair gives exactly zero coefficients and a negative
right-hand side: -0.1483219 for straight and -5.8637765 for circular. Therefore
these particular convex programs are infeasible even without considering their
obstacle directions or terminal requirements. This gives stronger evidence than
the Clarabel status alone and rules out a purely objective-weight explanation
of these two rejected programs.

### S-curve: oversized pose enclosure

The failure occurs while constructing geometry, before the SOCP solve. At
prediction node 28, 1.40 s into the future, the local lateral center is only
-0.34659 m, but its lateral radius is 120.28265 m. The maximum curvature over
the complete station interval is approximately 0.01 /m. Consequently,

    curvature_bound * (abs(lateral_center) + lateral_radius) = 1.20629 >= 1.

The complete local domain violates the existing Frenet regularity check.
The physical vehicle at the failing observation has lateral error 0.02659 m;
it has not driven 120 m away from the path. This is not a recurrence of the
earlier S-curve interface defect. The same inflated prediction also would
exhaust steering reserve at hold 27 if formulation proceeded that far.

## Controlled diagnostic comparison

For each captured model, disabled only `feedbackPrediction.enabled` in a local
diagnostic copy and rebuilt prediction. Checked exact equality of the nominal
anchor inputs and every discrete A/B stage matrix against the failed model.
Initial and estimator error bounds remained zero. No changed controller command
was issued and no physical simulation used this diagnostic setting.

| Scenario | Lateral bound with existing feedback (m) | Lateral bound with feedback disabled (m) |
| --- | ---: | ---: |
| Straight | 54.0719 | 1.7235e-7 |
| Circular | 2.9199e26 | 2.0326e-6 |
| S-curve | 8.6954e6 | 2.3581e-7 |

This isolates the feedback-enclosure propagation as the source of the extreme
growth for these fixed trajectories. It does not recommend removing feedback,
removing arithmetic reserves, or claim that a feedback-disabled controller would
complete nonlinear avoidance. No production code or configuration was changed.
The next design issue is compatibility of prediction-error feedback with the
trajectory-dependent dynamics and their near-limit input sensitivity. No new
feedback design is adopted by this diagnostic task.

## Artifacts and checks

`TRAJECTORY_FAILURE_DIAGNOSIS_20260925/` contains the summary, per-hold traces,
failed-row/domain details and SHA-256 manifest. Original replay snapshots and
retained copies of the temporary diagnostic helpers are in
`/home/zai/.cache/collisionAvoidance/trajectory-failure-diagnosis-20260925/`.
The source revision is `19e0c5e72d511df36a5ba645ab34bb5c1a3e18e1`.

MATLAB assertions checked exact replay inputs, equal counterfactual stage
matrices/anchors, zero input uncertainty/process reserves, and both algebraic
actuator contradictions. Python independently checks exported stage maxima,
domain arithmetic and artifact hashes. Breakpoints were cleared after replay.
This investigation does not rerun the full unit suite or execute a modified
closed-loop controller.
