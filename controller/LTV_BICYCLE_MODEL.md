# Scheduled Frenet bicycle and continuation model

The prediction and certificate use `ltvBicycleStageMatrices` at every stage.
State is `x = [s; d; ePsi; vx; vy; r]`; input is front steering angle and
longitudinal acceleration `u = [deltaF; a]`. Station and lateral offset use the
selected lane polyline; heading error is relative to its tangent.

For scheduled speed `vBar >= 0`, curvature `kappa`, and
`vTire = max(vBar, model.scheduleSpeedFloor)`, define `rBar = kappa*vBar`.
The scheduled continuous affine equations are

```text
sDot    = vx + kappa*vBar*d
dDot    = vBar*ePsi + vy
ePsiDot = r - kappa*vx - kappa^2*vBar*d
vxDot   = a + rBar*vy + b
vyDot   = -rBar*vx - (Cf+Cr)/(m*vTire)*vy
          - ((lf*Cf-lr*Cr)/(m*vTire)+vBar)*r + Cf/m*deltaF + rBar*vBar
rDot    = -(lf*Cf-lr*Cr)/(Iz*vTire)*vy
          - (lf^2*Cf+lr^2*Cr)/(Iz*vTire)*r + lf*Cf/Iz*deltaF
```

Here `b` is the declared constant longitudinal acceleration bias. One
forward-Euler step gives `Ad = I + Ts*A`, `Bd = Ts*B`, `cd = Ts*c`. This is
the declared prediction model, not the exact held-input flow of a nonlinear
vehicle. In particular the first predicted pose is independent of the new
input because the first three rows of `Bd` vanish.

The tire regularization is distinct from transport speed. At `vBar = 0`,
`[s; d; ePsi; 0; 0; 0]` is invariant under `[0; -b]` even on a curved route.
Using the positive tire floor in the kinematic rows would destroy this rest
property. The low-speed continuation is a regularized research model and
has no independently validated nonlinear-vehicle accuracy claim.

## Schedule and condensation

The admission schedule cruises at measured speed for `N` stages and then
brakes to zero. `brakingSchedule` derives `Nb` from maximum speed and the
configured nominal braking rate, with two additional rest stages. Stations
use the same Euler convention `sNext = s + Ts*vBar`; there is no
`0.5*Ts^2*a` term. Curvature is sampled from the supplied route.

All `M=N+Nb` stages have two control variables. `ltvBicyclePrediction`
condenses them into `x_j = F_j*plan + f_j`, including all lateral dynamics
through rest. A compatible certificate shifts the complete speed, station
and curvature schedules verbatim and appends zero speed. Thus an old
continuation stage and the executable stage it becomes have identical
matrices. Scheduling is part of the augmented controller state, rather than
a warm-start hint. A scheduled speed is an affine-model parameter, not a
claim that the realized nonlinear vehicle equals that speed.

A complete speed profile is exposed as `scheduleSpeedProfile`.
`frictionCirclePolygonRows` uses each stage's `max(vBar, floor)` consistently
with its bicycle matrices. Command force diagnostics use that same first
stage denominator. They do not silently recompute a different tire schedule
from the measured speed after certification.

## Constraints and uncertainty

Acceleration/steering bounds, front/rear inscribed friction polygons with
load transfer, and tire-slip validity constraints apply at all `M` stages.
There is no continuation acceleration-only input block and no kinematic
lateral-band handoff. Nonnegative longitudinal speed, maximum speed, heading,
lateral and affine-frame station limits are hard at every node. Rest requires
zero longitudinal/lateral speed and yaw rate; the final control is `[0;-b]`.

Error boxes are propagated through every stage:

```text
rNext = abs(Ad)*r + Ts*(ltvModelErrorRateBound + plantModelResidualRateBound)
```

This fixes the former propagation that expanded only the first future node.
It does not establish a robust terminal invariant set. The current persistent
trajectory certificate therefore rejects nonzero ego or model error bounds.
The initial Cartesian-position box conversion is useful predictor data, not
a certified nonlinear Frenet-coordinate error transformation on arbitrary
curved routes. Such a transformation and a feedback tube remain prerequisites
for extending the controller's uncertainty domain.

The configuration's Euler stiffness check screens the lateral/yaw diagonal
rates at the tire floor. It is not a proof of every scheduled matrix's
stability or of nonlinear-model validity. Closed-loop empirical behavior and
model-error enclosures require separate experiments.

## Geometry and proof scope

Physical rectangle separation is evaluated in Cartesian space using an
affine lane-segment frame, with hard station-domain bounds. Finite quadratic
road bounds use analytic extrema over the admitted footprint interval.
The proof and finite-precision acceptance protocol are in
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md). They cover prediction
nodes of the declared model, and represented road boundaries where covered.
