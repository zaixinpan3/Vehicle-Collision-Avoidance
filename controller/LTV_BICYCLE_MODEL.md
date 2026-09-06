# Scheduled Frenet bicycle and continuation model

The prediction and certificate use `ltvBicycleModel.stageMatrices` at every stage.
State is `x = [s; d; ePsi; vx; vy; r]`; input is front steering angle and
requested longitudinal acceleration `u = [deltaF; a]`. Station and lateral offset use the
selected lane polyline; heading error is relative to its tangent.

For scheduled speed `vBar >= 0`, curvature `kappa`, and
`vTire = max(vBar, model.scheduleSpeedFloor)`, define `rBar = kappa*vBar`.
The scheduled continuous affine equations are

```text
sDot    = vx + kappa*vBar*d
dDot    = vBar*ePsi + vy
ePsiDot = r - kappa*vx - kappa^2*vBar*d
vxDot   = gamma*a + rBar*vy + b
vyDot   = -rBar*vx - (Cf+Cr)/(m*vTire)*vy
          - ((lf*Cf-lr*Cr)/(m*vTire)+vBar)*r + Cf/m*deltaF + rBar*vBar
rDot    = -(lf*Cf-lr*Cr)/(Iz*vTire)*vy
          - (lf^2*Cf+lr^2*Cr)/(Iz*vTire)*r + lf*Cf/Iz*deltaF
```

Here `b` is the declared constant longitudinal acceleration bias and
`gamma = model.longitudinalInputGain` is the fixed commanded-input gain
(default 1, admissible range `(0,1]`). The block
matrix exponential of `Ts*[A,B,c; zeros(3,9)]` supplies `Ad`, `Bd` and `cd`.
The bias is added as `Bd(:,2)*(b/gamma)`, so it affects pose as well as velocity.
This is the exact held-input flow of each scheduled affine linearization,
not the exact flow of the nonlinear bicycle or the Blockset plant. The first
predicted pose can depend on the new input. On a straight road the longitudinal
update is `sNext = s + Ts*vx + 0.5*Ts^2*(gamma*a+b)` at every stage.

The tire regularization is distinct from transport speed. At `vBar = 0`,
`[s; d; ePsi; 0; 0; 0]` is invariant under `[0; -b/gamma]` even on a curved route.
Using the positive tire floor in the kinematic rows would destroy this rest
property. The low-speed continuation is a regularized research model and
has no independently validated nonlinear-vehicle accuracy claim.

The continuous-time CLF reads this same generator through
`ltvBicycleModel.continuousMatrices`, before held-input integration. It
computes `LfV + LgV*u` at the current scheduled state with a fixed local
cruise reference. Its Lyapunov matrix is obtained from the continuous
Riccati equation at straight reference cruise. This performance change
leaves the discrete prediction and every hard safety constraint unchanged.

## Schedule and condensation

The admission schedule cruises at measured speed for `N` stages and then
brakes to zero. `ltvBicycleModel.brakingSchedule` derives `Nb` from maximum speed and the
configured nominal braking rate, with two additional rest stages. Initial
scheduled station increments use trapezoidal integration of that speed
profile, consistent with constant acceleration within a sample. Curvature is
sampled from the supplied route.

All `M=N+Nb` stages have two control variables. `ltvBicycleModel.predict`
provides the stage matrices and the condensed map `x_j = F_j*plan + f_j`,
including all lateral dynamics through rest. The native QP uses explicit
states with sparse dynamic equalities; acceptance independently reconstructs
states from the condensed map. Identical adjacent scheduling parameters reuse
exactly the same stage matrices; no speed or curvature quantization is used.
Reference and resting commands divide their required model
acceleration by `gamma`; the gain also enters the cruise Riccati design. A compatible certificate shifts the complete speed, station
and curvature schedules verbatim and appends zero speed. Thus an old
continuation stage and the executable stage it becomes have identical
matrices. Scheduling is part of the augmented controller state, rather than
a warm-start hint. A scheduled speed is an affine-model parameter, not a
claim that the realized nonlinear vehicle equals that speed.

A complete speed profile is exposed as `scheduleSpeedProfile`.
`axleFriction.polygonRows` uses each stage's `max(vBar, floor)` consistently
with its bicycle matrices. Command force diagnostics use that same first
stage denominator. They do not silently recompute a different tire schedule
from the measured speed after certification.

## Constraints and uncertainty

Acceleration/steering bounds, front/rear inscribed friction polygons with
load transfer, and tire-slip validity constraints apply at all `M` stages.
There is no continuation acceleration-only input block and no kinematic
lateral-band handoff. Nonnegative longitudinal speed, maximum speed, heading,
lateral and affine-frame station limits are hard at every node. Rest requires
zero longitudinal/lateral speed and yaw rate; the final control is `[0;-b/gamma]`.

Error boxes are propagated through every stage:

```text
rNext = abs(Ad)*r + Ts*(ltvModelErrorRateBound + plantModelResidualRateBound)
```

The rate fields here specify a per-step map residual bounded by `Ts*rate`;
an arbitrary continuous disturbance bound must first be propagated through
the held-input flow to satisfy that contract. This fixes the former
propagation that expanded only the first future node.
It does not establish a robust terminal invariant set. The current persistent
trajectory certificate therefore rejects nonzero ego or model error bounds.
The initial Cartesian-position box conversion is useful predictor data, not
a certified nonlinear Frenet-coordinate error transformation on arbitrary
curved routes. Such a transformation and a feedback tube remain prerequisites
for extending the controller's uncertainty domain.

The former forward-Euler stiffness restriction is removed. Exact integration
does not itself establish closed-loop stability, nonlinear-model validity or
low-speed tire accuracy. Those require separate analysis and experiments.

## Geometry and proof scope

Physical rectangle separation is evaluated in Cartesian space using an
affine lane chart, with hard station-domain bounds and explicit position/yaw
error allowances over every polyline segment intersecting that domain.
Acceptance evaluates the actual polyline pose. Finite quadratic road bounds
use analytic extrema over the admitted footprint interval.
The proof and finite-precision acceptance protocol are in
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md). They cover prediction
nodes of the declared model, and represented road boundaries where covered.
