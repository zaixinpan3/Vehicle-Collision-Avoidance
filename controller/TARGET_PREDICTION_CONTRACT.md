# Exact target motion in the two-vehicle controller

The controller study uses one persistent target with its full current position,
velocity, acceleration, heading and yaw rate known at every sample. Its exact
prediction is the Cartesian constant-acceleration flow

```
p(t+tau)   = p(t) + v(t)*tau + a(t)*tau^2/2
v(t+tau)   = v(t) + a(t)*tau
a(t+tau)   = a(t)
yaw(t+tau) = yaw(t) + yawRate(t)*tau
yawRate(t+tau) = yawRate(t)
```

`targetPrediction.admitExact` accepts this contract without a motion descriptor,
or with `predictionMotion.kind="exact-motion-v1"`. An old
`finite-sensing-motion-v1` descriptor is accepted only with zero future bounds;
it is normalized to the exact all-future contract. Current-state estimation
boxes (position, velocity, acceleration, yaw, yaw rate) are admitted and
carried as sets; future-motion bounds (`predictionAccelerationErrorBound`,
`predictionYawAccelerationErrorBound`, jerk or yaw-acceleration contracts)
are rejected because the law itself is exact. At every continuation frame
`targetPrediction.conditionExact` intersects the carried box propagated by
one period with the new measurement box; the result is a subset of the
propagated box and contains the true state. Standalone uncertain-prediction
utilities remain available for offline research.

With the exact law, a nonzero velocity or acceleration box makes the target's
all-future support unbounded along every direction on which the box is not
strictly receding, so no terminal halfspace exists (`unboundedTargetSupport`).
Position and yaw boxes are covered without restriction; see
[INFORMATION_STATE_PCBF.md](INFORMATION_STATE_PCBF.md), Section 6.

One stable identifier and the same footprint must persist. With only one
otherwise anonymous target the controller assigns `exactTarget:1`. Missing or
additional targets are outside the strict scene. A missing target never means
safe exit. The controller retains the original absolute-time trajectory and
checks every new state against it, with arithmetic allowance. Measurements do
not reset the forecast. The flow satisfies the required time-shift identity.
The terminal certificate uses the target's entire future, not a finite validity
window, a perception range or a nonreturn assertion.

The curved `nominalFlow` utility remains an offline/initialization model and is
not the target truth in this study. In particular, zero jerk bounds do not make
a Cartesian constant-acceleration predictor equal to a constant-curvature
predictor. At 8 m/s and yaw rate 0.2 rad/s their positions differ by 0.05329335 m
after one second in the regression example. A target generator for this study
must call `finiteFlow` or implement the exact equations above, including
possible reversal under constant negative Cartesian acceleration.

See [INFORMATION_STATE_PCBF.md](INFORMATION_STATE_PCBF.md) for the ego
plant premise, the robust terminal set and the all-future box separation.
