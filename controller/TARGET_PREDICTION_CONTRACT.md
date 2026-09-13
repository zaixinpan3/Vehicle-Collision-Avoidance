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
