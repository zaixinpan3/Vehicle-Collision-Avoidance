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
derivative bounds retain the exact-motion metadata for existing studies.

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

## Terminal scope and remaining limitation

The current invariant stopping terminal set requires a separating halfspace
whose target position support is finite over all future times. For normal
`n`, the bound is a cubic with leading coefficient `abs(n)'*J/6`. Any positive
coefficient makes that support infinite. Thus nonzero jerk orthogonal to a
safe normal can be certified, whereas a strictly positive two-axis jerk box
cannot be certified by this terminal family. Existing velocity/acceleration
boxes can also prevent a finite support. Nonzero `H` is covered by the
target rectangle's circumcircle in the terminal set, including when the
current yaw-rate box is exactly zero.

`unboundedTargetSupport` means this sufficient terminal construction failed;
it does not mean a nonlinear/finite-encounter avoidance problem was solved
and proved infeasible. No target bound is reduced to force admission. A
finite-encounter terminal certificate consistent with perception departure
remains needed for general NRMM motion bounds. Removing this support check
without replacing its proof would not preserve the recursive PCBF claim.

The `nominalFlow` utility supplies constant-curvature/tangential-acceleration
anchors. For an actual curved target, `J` must cover its Cartesian jerk,
including turning-induced jerk; zero Cartesian jerk does not describe an
exact circular trajectory. The observer's high-gain framework is unchanged.
