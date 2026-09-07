# Finite encounter prediction and current target visibility

The September 7, 2026 modeling clarification is that target curvature and
tangential acceleration are approximately constant during the short vehicle
encounter. This is a finite prediction assumption, not a promise that the
target never changes its maneuver. Targets need no specified road or corridor.
Actual current radar visibility controls target publication: internal observer
coasting does not keep a hidden target in the controller's constraint set.

## Prediction law and uncertainty

For each prediction, let speed be `v`, course be `theta`, curvature be `k`,
and tangential acceleration be `a`. Until a braking prediction stops,

\[
 \dot p=v[\cos\theta,\sin\theta]^\top,\qquad
 \dot v=a,\qquad \dot\theta=kv,\qquad \dot k=\dot a=0.
\]

Negative acceleration stops at `v=0`; it does not reverse the vehicle. Define
the nonnegative traveled distance `L(t;v,a)` by integrating
`max(v+a*t,0)`. Nominal curvature is reconstructed as yaw rate divided by
speed, and tangential acceleration is the projection of acceleration onto
course. The existing zero-speed direction fallback is retained.

The initial estimation certificate is not zeroed. `targetPrediction.initialSet`
maps its Cartesian velocity box into speed and course bounds, and its
acceleration/yaw-rate errors into fixed parameter intervals. If
`Bv=norm(b_velocity)` and `Ba=norm(b_acceleration)`, then

\[
 v\in[\max(0,\hat v-B_v),\hat v+B_v],\qquad
 b_\theta=\arcsin(B_v/\hat v)\quad(B_v<\hat v).
\]

Otherwise all initial course directions are admitted. The zero-velocity
case obtains direction from acceleration, with its own error radius.
For nonzero acceleration, a conservative tangential acceleration error is
`Ba + 2*norm(a_hat)*sin(b_theta/2)`. The curvature interval encloses all
quotients of the yaw-rate interval and positive speed interval. It is
unbounded if uncertain speed can approach zero with nonzero yaw rate;
this does not make the finite position enclosure infinite. A point-valued
zero yaw rate produces zero curvature.

Each member retains its own reconstructed parameters for this finite
prediction. The construction adds neither future arbitrary maneuver changes
nor repeated disturbances derived from estimator operating-domain maxima.
The interval product discards correlations and can be conservative.

Let `Lmin`, `Lmax` come from the endpoints of the speed/acceleration ranges,
let `Lhat` be nominal traveled distance, `bL=max(abs([Lmin,Lmax]-Lhat))`,
`bk=max(abs(kRange-kHat))`, and `Lc=min(Lhat,Lmax)`. A componentwise position
enclosure is

\[
 b_p(t)=b_p(0)+b_L+
 \min\{2L_c,\ b_\theta L_c+\tfrac12 b_kL_c^2\}.
\]

This follows by separating unequal arc lengths and integrating the difference
between two unit tangents on their common arc. For unbounded `bk`, use `2Lc`.
Heading error is bounded by
`min(pi, b_psi(0)+bk*Lmax+abs(kHat)*bL)`, with zero turn error at zero
traveled distance. Exact initial motion gives zero additional prediction
error, including across the stop. All-stopping families have constant bounds
after their latest possible stopping time.

The numeric `targetAccelerationInertialErrorBound` is accepted without a
certificate; the older `targetPredictionAccelerationInertialErrorBound` adds
to that initial acceleration uncertainty. A current certificate takes
precedence over those aliases. A nonzero independent future yaw-acceleration
disturbance is outside this fixed-curvature contract and is rejected.

## Finite constraints and continuation checks

The currently published target has hard rectangle-separation rows at every
head/tail node, including the last node. There is no infinite ray/orbit
support calculation, support-direction grid, or infinite-time target gate.
In the matched joint experiment, 24 head stages plus 74 continuation stages
at 0.05 s give a 4.9 s imposed target forecast. The stored extra sample is
used for shift comparison; it does not extend the accepted safety interval.

The ego rest or dissipative terminal policy remains. Its remaining-pose
budget can conservatively tighten the finite last target row, but that row
does not establish permanent target separation. The diagnostics distinguish
`terminalPredictionCertified` from `terminalInvariantCertified`; the latter
is false while a target is present. The experiment evaluator checks finite
terminal admission and does not require the withdrawn invariant-target claim.

On consistent shifts, overlapping geometry may be carried. The newly appended
target node is always rebuilt from the fresh finite forecast, including when
the ego uses a dissipative terminal funnel. The old input sequence must pass
the full new acceptance check before fallback is permitted. A rejected target
extension requires a new accepted solve; feasibility at every future sample
is not assumed. Version 8 prevents reuse of the preceding certificate format.

The adapter already removes target publication on current radar exit. The
controller then drops the corresponding rows and performs admission for that
environment. It does not use predicted future range exit to discard a target
that is still currently visible. Reacquisition creates a currently published
target that must be considered again. The reported guarantee covers declared
model prediction nodes for those targets, not unobserved traffic or a
permanent parked-vehicle safety claim.

## Validation scope

`targetPredictionTest` checks initial error retention, exact-motion stopping,
bounded finite forecasts near zero curvature, independence from legacy global
motion limits, and propagation of all 256 corners of two eight-state boxes.
Corner checks validate the implementation on those cases; the enclosure
argument above, not vertex sampling alone, supplies its mathematical basis.
Controller tests check finite oncoming forecasts, fresh continuation checks,
and removal of all target rows after publication ceases. The adapter's existing
radar lifecycle test separately checks visibility, internal coasting and
retirement. None establishes nonlinear-plant robustness or eventual cruise
convergence from a finite simulation.
