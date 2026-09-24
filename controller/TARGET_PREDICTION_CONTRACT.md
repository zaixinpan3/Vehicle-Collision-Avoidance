# Bounded target motion in the controller

The online target state is `z=[p_x,p_y,v_x,v_y,a_x,a_y,psi,omega]` with a
componentwise current-state error box. While an identified target is active,
its actual motion obeys

```
p_dot = v,  v_dot = a,  abs(a_dot) <= J
psi_dot = omega,       abs(omega_dot) <= H
```

`predictionMotion.kind="finite-sensing-motion-v1"` supplies nonnegative,
finite `jerkBound=J` (2-by-1, m/s^3) and `yawAccelerationBound=H` (rad/s^2).
Nonzero bounds are admitted by `targetPrediction.admitOnline`; they tighten
the reachable sets and constraints instead of causing an input rejection.
The two-stage PCBF safety-value / CLF-and-input optimization is unchanged.

The nominal center uses Cartesian constant acceleration and constant yaw
rate. `finiteFlow` adds `J*t^3/6` to the position radius, `J*t^2/2` to the
velocity radius, `J*t` to the acceleration radius, `H*t^2/2` to the heading
radius and `H*t` to the yaw-rate radius, in addition to propagated current
error. These bounds cover arbitrary time-varying derivatives inside the
declared limits; actual motion need not equal the nominal center.

Without a motion descriptor, jerk defaults to zero and the parsed
`targetPredictionYawAccelerationErrorBound` supplies `H`. The existing
`targetPredictionAccelerationInertialErrorBound` is included by the parser
in the initial acceleration radius; it is not a substitute for a jerk bound.
An `exact-motion-v1` descriptor is normalized through the same validator,
retaining any supplied derivative bounds instead of discarding them. Zero
derivative bounds retain `validityScope="whileEncounterActive"`; numerical
equality to zero never implies an all-future modeling commitment.

At each continuation frame `targetPrediction.condition` intersects the
carried reachable box with the new measurement box, with wrapped heading.
Motion within the bounds is compatible even when acceleration and yaw rate
change. An empty intersection signals inconsistent supplied bounds or data.
The carried contract is retained if a new bound is smaller; larger future
bounds require fresh admission and report `motionBoundsIncreased=true`. The
completed hold is still checked against its original reachable set before
this admission. A new certificate allows control to continue; failed admission
cannot use the smaller-bound witness. There is no value-descent comparison
across these different future-motion contracts.

The target identity and footprint must persist within an encounter. A sole
anonymous target receives `singleTarget:1`. Zero visible targets and verified
departure use the lifecycle in [INFORMATION_STATE_PCBF.md](INFORMATION_STATE_PCBF.md):
the same optimization retains road, model, actuator and CLF constraints.

## Exact NRMM motion (`nrmm-motion-v1`)

`predictionMotion.kind="nrmm-motion-v1"` declares exact NRMM motion. The target
keeps its speed-rate `A` and its sideslip, so it follows a path of constant
curvature `kappa` through its current state, with
`abs(kappa) <= curvatureMaximum` (1/m, required) and, when declared,
`abs(A) <= speedRateMaximum` (m/s^2). Its yaw is `psi0 + kappa*s(t)` and its
yaw rate `kappa*V(t)`; it stops and holds when its speed reaches zero.
`jerkBound` and `yawAccelerationBound` must be zero. A varying speed-rate or
curvature leaves every NRMM path through a later estimate, so a model error
is declared with a `finite-sensing-motion-v1` contract instead
(`nrmmControllerErrorBounds` does this when `modelJerkMaximum > 0`).

**Parameter intervals** (`targetPrediction.nrmmParameters`). The encounter
carries intervals for `V`, the course, `A` and `kappa` that contain the true
target's values. They are the intersection of:
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

At a continuation frame the carried intervals are propagated over the hold
(`A` and `kappa` constant, `V = max(0, V + A h)`, the course advanced by
`kappa` times the arc, all monotone in the interval ends) and intersected with
the measurement's intervals. An empty intersection is an inconsistent
observation.

**Reachable box** (`finiteFlow`). Two bounds on the same NRMM paths, both
from the parameter intervals and the position box, intersected:
- **Parameter-Taylor bound.** The linearization of the path in the
  parameters with a Lagrange second-order remainder, or a path-length ball
  where the linearization does not apply: a course radius above 0.5 rad, a
  speed interval touching zero, or a possible stop. It is tight for fast and
  turning targets and long horizons.
- **Time-Taylor bound.** The constant-acceleration extrapolation of the
  estimate box plus the integrated jerk of the NRMM paths,
  `hypot(kappa^2 V^3, 3 A kappa V)` over the parameter intervals, and their
  yaw acceleration `|A kappa|`. A stop sets the acceleration to zero, so where
  a path may have stopped the per-axis acceleration deviation also covers
  `max(0, |a0| - r_a)`. It is tight for slow targets and short horizons, where
  the parameter-Taylor bound pays for a course and a curvature that the
  estimate cannot resolve.

No `J*t^3/6` term of a declared jerk is added: the growth is the extrapolation
of the current estimate error along the NRMM path.
`targetPrediction.deviationModel` gives the parameter linearization to the
target-reactive tube. Whole-hold cells extrapolate a node snapshot with a
Cartesian jerk bound and are not supported for NRMM targets. A change of the
motion kind or a larger curvature or speed-rate maximum voids a carried family.

`scalarAccelerationMaximum`, when declared for either kind, bounds the
acceleration magnitude `|a|` (for an NRMM target `hypot(A, V*omega)`), not the
speed-rate alone.

## Finite completion scope

Version 21 verifies the complete finite collision tube and robust exterior
membership at its confirmation time, followed by a target-independent road
terminal controller. Both Cartesian jerk bounds may be positive. No
all-future target support is computed. The entire final target footprint
must be outside the region declared in `ego.perception.range`, with target,
ego and chart uncertainty included. No future measurement shrinkage is
assumed. A current valid observation is required to release the target;
missing confirmation at the deadline stops control. The existing road-safe
suffix survives confirmed removal even when fresh optimization fails.

A finite certificate can still fail because of uncertainty, geometry, input
limits, chosen normal or search limits. Such failure does not establish that
finite collision avoidance is impossible. Larger future bounds require
fresh admission; no radius is suppressed to force it. Re-entry is a new
encounter and requires a new verified certificate. Global detection and
entry feasibility and nonlinear plant inclusion remain separate obligations.

The `nominalFlow` utility supplies constant-curvature/tangential-acceleration
anchors. For an actual curved target, `J` must cover its Cartesian jerk,
including turning-induced jerk; zero Cartesian jerk does not describe an
exact circular trajectory. The observer's high-gain framework is unchanged.
