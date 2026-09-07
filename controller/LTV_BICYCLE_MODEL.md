# Scheduled Frenet bicycle and continuation model

The prediction and certificate use `ltvBicycleModel.stageMatrices` at every stage.
State is `x = [s; d; ePsi; vx; vy; r]`; input is front steering angle and
signed braking ratio `u = [deltaF; beta]`. Station and lateral offset use the
selected lane polyline; heading error is relative to its tangent.

For scheduled speed `vBar >= 0`, curvature `kappa`, and
`vTire = max(vBar, model.scheduleSpeedFloor)`, define `rBar = kappa*vBar`.
The scheduled continuous affine equations, using the Fiala tangent defined below, are

```text
sDot    = vx + kappa*vBar*d
dDot    = vBar*ePsi + vy
ePsiDot = r - kappa*vx - kappa^2*vBar*d
vxDot   = gBeta*beta - (Froad(vBar) + FroadSlope(vBar)*(vx-vBar))/m + rBar*vy + b
vyDot   = -rBar*vx - (Cf+Cr)/(m*vTire)*vy
          - ((lf*Cf-lr*Cr)/(m*vTire)+vBar)*r + Cf/m*deltaF + (bf+br)/m*beta + rBar*vBar + (df+dr)/m
rDot    = -(lf*Cf-lr*Cr)/(Iz*vTire)*vy
          - (lf^2*Cf+lr^2*Cr)/(Iz*vTire)*r + lf*Cf/Iz*deltaF + (lf*bf-lr*br)/Iz*beta + (lf*df-lr*dr)/Iz
```

Here `Cf` and `Cr` are stage-dependent effective cornering stiffnesses, not
the nominal tire parameters. `bf` and `br` are the force derivatives with
respect to beta; `df` and `dr` are the joint tangent intercepts.

Here `b` is the declared constant longitudinal acceleration bias. The paper's
signed ratio lies in `[-1,1]`: negative values brake, zero coasts and positive
values request throttle. Static loads give `Fx_i=beta*mu_i*Fzi` and
`gBeta=sum(mu_i*Fzi)/m`, which is `mu*g` for a common friction coefficient.
There is no independent longitudinal effectiveness gain or front-drive/brake
split in this input definition. The affine generator includes the bias in
its fourth row *before* exponentiation. It must not be represented by adding
`b/gBeta` to the input, because the beta column also acts laterally.
The block matrix exponential of `Ts*[A,B,c; zeros(3,9)]` supplies `Ad`, `Bd`
and `cd`.
This is the exact held-input flow of each scheduled affine linearization,
not the exact flow of the nonlinear bicycle or the Blockset plant. The first
predicted pose can depend on the new input. The straight-road longitudinal
flow includes passive road-load damping and its affine intercept. The former
constant-acceleration position formula applies only when road load is zero.

`longitudinalRoadLoad` supplies the signed aerodynamic and equivalent rolling
forces and their speed derivative. See [LONGITUDINAL_FORCE_BALANCE.md](LONGITUDINAL_FORCE_BALANCE.md)
for the physical input contract, parameter extraction, and approximation limits.

The tire regularization is distinct from transport speed. At `vBar = 0`,
`[s; d; ePsi; 0; 0; 0]` is invariant under `[0; -b/gBeta]` even on a curved route.
Using the positive tire floor in the kinematic rows would destroy this rest
property. The low-speed continuation is a regularized research model and
has no independently validated nonlinear-vehicle accuracy claim.

The continuous-time CLF reads this same generator through
`ltvBicycleModel.continuousMatrices`, before held-input integration. It
computes `LfV + LgV*u` at the current scheduled state with a fixed local
cruise reference. Its Lyapunov matrix is obtained from the continuous
Riccati equation at straight reference cruise. This performance change
uses the same first-stage tire schedule as prediction. The local cruise target
solves the three affine velocity balance equations. If a saturated tangent
makes that system singular, its minimum-residual target is a soft performance
reference, with no claim of attainable steady cornering.

## Modified Fiala force and local linearization

The source is Fahmy, Abd El Ghany and Baumann, *Vehicle Risk Assessment and
Control for Lane-Keeping and Collision Avoidance at Low-Speed and High-Speed
Scenarios*, IEEE TVT 67(6), 4806–4818 (2018), Section II-A, equations (7)–(13),
[DOI: 10.1109/TVT.2018.2807796](https://doi.org/10.1109/TVT.2018.2807796).
The implementation adopts equation (13). The extra `arctan` printed in
(12) is inconsistent with that equation; it is not applied. Equation (7)'s
branch test is interpreted symmetrically in `abs(alpha)`, so the force
opposes either sign of slip.

For each equivalent axle `i`, use the nominal axle cornering stiffness `Ci`,
static normal loads `Fzf=m*g*lr/L`, `Fzr=m*g*lf/L`, and

```text
Fx_i   = beta*mu_i*Fzi
eta    = sqrt(1-beta^2)
Qi     = eta*mu_i*Fzi
alphaSl_i = atan(3*Qi/Ci)
t = tan(alpha)
Fy_i(alpha) = -Ci*t + Ci^2/(3*Qi)*abs(t)*t - Ci^3/(27*Qi^2)*t^3
              if abs(alpha) < alphaSl_i and Qi > 0
            = -Qi*sign(alpha) otherwise.
```

Axle stiffness is the sum for its equivalent wheel pair, and the static axle
load is its total normal load. Axle-specific coefficients are a bicycle
adaptation of the paper's four-wheel model. A common beta applies to both
axles; the force distribution follows `mu_i*Fzi`. Road load remains a separate
vehicle-level force. Tire utilization, load-transfer limits and friction
polygons are not analyzed or constrained. At `abs(beta)=1`, the nonlinear
lateral force is zero. Its joint slip/beta tangent is singular there, so
`modifiedFialaTire.evaluate` permits endpoint force evaluation but rejects
endpoint derivative requests. Inputs remain bounded by the physical interval;
only nominal linearization ratios are clipped to `[-1+sqrt(eps),1-sqrt(eps)]`.
This numerical anchor convention does not certify nonlinear accuracy at the
endpoints.

The operating point is `vyBar=0`, `rBar=kappa*vBar`, and
`deltaBar=atan(L*kappa)*vBar/vTire`. Its slips are
`alphaBar_f=lf*rBar/vTire-deltaBar`, `alphaBar_r=-lr*rBar/vTire`.
At fixed static loads and `vTire`, linearize in both slip and beta. In adhesion,
with `t_i=tan(alphaBar_i)`, `etaBar=sqrt(1-betaBar^2)` and
`Qi=mu_i*Fzi*etaBar`,

```text
q_i = Ci*abs(t_i)/(3*Qi)
k_i = -Ci*(1-q_i)^2*sec(alphaBar_i)^2
b_i = Ci*t_i*q_i*(1-2*q_i/3)*betaBar/etaBar^2
FyLin_i(alpha,beta) = Fy_i(alphaBar_i,betaBar)
                     + k_i*(alpha-alphaBar_i) + b_i*(beta-betaBar)
Ceff_i = -k_i
di = Fy_i(alphaBar_i,betaBar)-k_i*alphaBar_i-b_i*betaBar.
```

On the saturated branch, `k_i=0` and
`b_i=mu_i*Fzi*betaBar/etaBar*sign(alphaBar_i)`. Both derivatives agree
across the adhesion/saturation junction when `abs(betaBar)<1`. At zero slip,
`Ceff_i=Ci`, `b_i=0`, `di=0`. Curved schedules generally change all of them.
The beta derivative supplies the direct braking/steering coupling in each
QP, while the generally nonzero affine intercept preserves the force at
the operating point.

This is a local linearization in `(alpha,beta)` with a frozen tire-speed
denominator, not a full Jacobian of the unregularized nonlinear vehicle.
The tangent can depart from the nonlinear force envelope away from its
anchor. Plant/model residual bounds must cover such departures when a plant
guarantee is sought. The low-speed denominator regularization extends the
paper's strictly nonzero-speed assumption.

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
exactly the same stage matrices, including the scheduled braking ratio; no parameter quantization is used.
The initial schedule input adds `Froad(vBar)/m` to its required net
acceleration before dividing by `gBeta`. Rest uses zero road load; the beta force scale also enters the cruise Riccati design. A compatible certificate shifts the complete speed, station
and curvature schedules verbatim and appends zero speed. Thus an old
continuation stage and the executable stage it becomes have identical
matrices. Scheduling is part of the augmented controller state, rather than
a warm-start hint. A scheduled speed is an affine-model parameter, not a
claim that the realized nonlinear vehicle equals that speed.

The speed and braking-ratio schedules are exposed as `scheduleSpeedProfile`
and `scheduleBrakingRatio`. The latter is derived from adjacent speeds and
road load, so shifting the speed profile also preserves every overlapping
Fiala tangent. `tireSlipRows` uses each stage's `max(vBar, floor)` consistently
with its bicycle matrices. Command force diagnostics use that same first
stage denominator. They do not silently recompute a different tire schedule
from the measured speed after certification.

## Constraints and uncertainty

Braking-ratio/steering bounds and the configured slip-angle domains apply at
all `M` stages. Slip limits are chosen model domains below `pi/2`, not
friction-limit constraints or a certificate of tangent accuracy.
There is no continuation acceleration-only input block and no kinematic
lateral-band handoff. Nonnegative longitudinal speed, maximum speed, heading,
lateral and affine-frame station limits are hard at every node. Exact-state rest
requires zero longitudinal/lateral speed and yaw rate. With velocity uncertainty,
a dissipative terminal set replaces those equalities; both branches use the
final control `[0;-b/gBeta]`.

Error boxes are propagated through every stage as `rNext = abs(Ad)*r+d`.
The two model rate fields now specify continuous derivative-error bounds,
with `d` obtained by transition-weighted integration using a Metzler
comparison. This includes forcing transported into other state channels.
Initial Cartesian boxes cover all possible closest projection segments and
their heading differences. Robust initial admission additionally requires
one invertible interior chart.

The stationary-pose certificate retains exact velocity channels. The dissipative
extension admits nonzero velocity uncertainty when the final zero-speed model
has a Hurwitz Metzler velocity comparison and no persistent forcing. It
reserves the complete remaining pose excursion and enforces invariant speed
and slip domains. See
[DISSIPATIVE_TERMINAL_CERTIFICATE.md](DISSIPATIVE_TERMINAL_CERTIFICATE.md) for
the new inequalities and [MINIMAL_UNCERTAINTY_CERTIFICATE.md](MINIMAL_UNCERTAINTY_CERTIFICATE.md)
for the original stationary-pose/set-membership argument.

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

## Input contract and migration

`command.actuatorInput`, every optimized stage and `heldActuatorInput` use
`[frontWheelSteeringAngle; brakingRatio]`, with units `[rad; 1]`.
`actuation.brakingRatioMinimum/Maximum` default to `[-1,1]`, and
`clf.brakingRatioWeight` weights the normalized dimensionless input.
The old acceleration bounds, acceleration weight, longitudinal gain and
braking force distribution are rejected as unknown configuration fields.
`terminal.backupDeceleration` remains a speed-schedule parameter in m/s^2;
it is checked against the available beta braking range.

`command.longitudinalAcceleration` is a derived gross acceleration
`gBeta*beta`, not the second control input or net body derivative. Passive
road load and the declared bias enter the separately reported body derivative.
The plant adapter receives the two axle force requests `beta*mu_i*Fzi` and
converts them to wheel torque/brake pressure. Archived `[delta,a]` input
arrays must not be replayed as beta arrays; their old configuration and
recorded force allocation also belong to the earlier model.
