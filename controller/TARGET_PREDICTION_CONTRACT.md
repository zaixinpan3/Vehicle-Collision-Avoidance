# Target motion in the controller: the NRMM contract

The online target state is `z=[p_x,p_y,v_x,v_y,a_x,a_y,psi,omega]` with a
componentwise current-state error box. Every target carries one motion
contract, `predictionMotion.kind="nrmm-motion-v1"`; there is no other kind.
It declares exact NRMM motion: the target keeps its speed-rate `A` and its
sideslip, so it follows a path of constant curvature `kappa` through its
current state, with `abs(kappa) <= curvatureMaximum` (1/m, required) and,
when declared, `abs(A) <= speedRateMaximum` (m/s^2) and
`abs(a) <= scalarAccelerationMaximum` (the acceleration magnitude
`hypot(A, V*omega)`, m/s^2). Its yaw is `psi0 + kappa*s(t)` and its yaw rate
`kappa*V(t)`; it stops and holds when its speed reaches zero.

A varying speed-rate or curvature leaves every NRMM path through a later
estimate. No contract describes such motion: a target without a contract, or
with a nonzero `jerkBound` or `yawAccelerationBound` field, is refused at
admission (`missingPredictionMotion`, `invalidEncounterContract`). The
Cartesian jerk contract `finite-sensing-motion-v1` that covered it (constant
acceleration plus `J t^3/6`) was removed on 2026-09-24 at the user's
direction, together with its `exact-motion-v1` alias, the parsed
`targetPredictionYawAccelerationErrorBound` and the per-hold acceleration part
of the reactive tube. `nrmmControllerErrorBounds` publishes the NRMM contract
when the estimator's `modelJerkMaximum` is zero (the default) and no contract
otherwise.

Zero derivative bounds retain `validityScope="whileEncounterActive"`;
numerical equality never implies an all-future modeling commitment.

## Parameter intervals (`targetPrediction.nrmmParameters`)

The encounter carries intervals for `V`, the course, `A` and `kappa` that
contain the true target's values. They are the intersection of:
- the intervals of every NRMM state in the estimate box: `V = |v| +- |r_v|`,
  `course = atan2(v) +- asin(|r_v|/|v|)`,
  `A = v'a/|v| +- (|r_a| + 2|a| sin(r_course/2))`, and `kappa` in both
  `omega/V` and `a_N/V^2` over the box;
- the bounds the declarer publishes about the same estimate
  (`targetSpeedErrorBound`, `targetCourseErrorBound`,
  `targetSpeedRateErrorBound`, `targetCurvatureInterval`; the NRMM estimator
  derives them from its frame-free component balls, see
  [ESTIMATOR_BOUND_INTERFACE.md](ESTIMATOR_BOUND_INTERFACE.md));
- the declared maxima: `curvatureMaximum`, `speedRateMaximum` and the
  acceleration magnitude bound.

At a continuation frame (`targetPrediction.condition`) the carried box is
propagated over the hold and intersected with the measurement box, with
wrapped heading, and the carried intervals are propagated (`A` and `kappa`
constant, `V = max(0, V + A h)`, the course advanced by `kappa` times the
arc, all monotone in the interval ends) and intersected with the
measurement's intervals. An empty intersection is an inconsistent
observation. The measurement's contract replaces the carried one; when its
`curvatureMaximum`, `speedRateMaximum` or `scalarAccelerationMaximum` is
larger than the carried value (an undeclared maximum counts as infinite),
`hardEncounterBarrier` voids the inherited family and admits the encounter
afresh (`metadata.inheritedFeasibleFamily=false`), after the completed hold
has been checked against its original reachable set. The offline
`targetPrediction.advance` applies the same rule and raises
`changedEncounterContract` instead.

## Reachable box (`targetPrediction.finiteFlow`)

The NRMM paths of the parameter intervals through the position box, enclosed
by the linearization of the path in the parameters with a Lagrange
second-order remainder, or by a path-length ball where the linearization
does not apply: a course radius above 0.5 rad, a speed interval touching
zero, or a possible stop. No Cartesian extrapolation and no jerk term enter:
the growth is the extrapolation of the current parameter errors along the
NRMM path. (A Cartesian bound on the same paths was intersected with this box
between commits d1735f7 and 72a2ff1 and removed at the user's direction; it
was tighter for slow targets with large errors.)

`targetPrediction.deviationModel` gives the reactive tube the same
linearization: the parameter errors as generators shared by the target and
the ego's reaction, and the remainder as a fixed box at every node whose
reaction enters as new generators. There is no per-hold acceleration part.
Whole-hold cells extrapolate a node snapshot as a polynomial in time and are
not supported with a target (`unsupportedWholeHoldTarget`); certification is
at hold nodes.

## Identity and completion scope

The target identity and footprint must persist within an encounter. A sole
anonymous target receives `singleTarget:1`. Zero visible targets and verified
departure use the lifecycle in [INFORMATION_STATE_PCBF.md](INFORMATION_STATE_PCBF.md):
the same optimization retains road, model, actuator and CLF constraints.

The finite collision tube and robust exterior membership at the confirmation
time are verified, followed by a target-independent road terminal
controller. No all-future target support is computed. The entire final target
footprint must be outside the region declared in `ego.perception.range`,
with target, ego and chart uncertainty included. No future measurement
shrinkage is assumed. A current valid observation is required to release the
target; missing confirmation at the deadline stops control. The existing
road-safe suffix survives confirmed removal even when fresh optimization fails.

A finite certificate can still fail because of uncertainty, geometry, input
limits, chosen normal or search limits. Such failure does not establish that
finite collision avoidance is impossible. Re-entry is a new encounter and
requires a new verified certificate. Global detection and entry feasibility
and nonlinear plant inclusion remain separate obligations.

`targetPrediction.nominalFlow` supplies the constant-curvature,
constant-tangential-acceleration anchor of the estimate (the NRMM path of the
box center) for objective anchors and the nominal center carried between
frames; it is not a certificate.
