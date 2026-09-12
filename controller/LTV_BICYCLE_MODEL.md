# Scheduled Frenet bicycle and continuation model

## Exact scheduled-plant study (September 11, 2026)

The current online experiment executes the first held interval of each newly
published scheduled affine prediction exactly. Future schedules are refreshed
when replanning. The zero-speed terminal schedule and sampled velocity feedback
are hypothetical certificate dynamics; they are never an actual handoff mode.
State and process errors are zero; arithmetic prediction enclosures remain.
This does not transfer terminal invariance to a refreshed model or a nonlinear
Fiala plant. The terminal derivation and outstanding shift-compatibility
condition are in
[SINGLE_PATH_RECURSIVE_FEASIBILITY.md](SINGLE_PATH_RECURSIVE_FEASIBILITY.md).
The model derivations and nonlinear comparison utilities below remain useful,
but statements about the former online finite-perception scope are superseded.

Version 9 uses `ltvBicycleModel.continuousMatrices` with finite swept tubes
from `ltvBicycleModel.finitePredict`. The governing continuation requirement is
[certified encounter discharge](ENCOUNTER_SCOPED_CBF_CLF.md).
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

`ltvBicycleModel.roadLoad` supplies the signed aerodynamic and equivalent rolling
forces and their speed derivative. See [LONGITUDINAL_FORCE_BALANCE.md](LONGITUDINAL_FORCE_BALANCE.md)
for the physical input contract, parameter extraction, and approximation limits.

The tire regularization is distinct from transport speed. At `vBar = 0`,
`[s; d; ePsi; 0; 0; 0]` is invariant under `[0; -b/gBeta]` even on a curved route.
Using the positive tire floor in the kinematic rows would destroy this rest
property. The low-speed continuation is a regularized research model and
has no independently validated nonlinear-vehicle accuracy claim.

The predictive CLF uses this same continuous generator throughout every
held interval. A common positive-definite Riccati metric is synthesized at
straight reference cruise. The tracking reference includes explicit constant
lateral-velocity/yaw-rate offsets and an optional affine rate in absolute time.
Every reference derivative enters the dissipation bound. One nonnegative slack
per interval bounds the residual over all swept control-point boxes; a common
convex majorant per cell supplies the intermediate-time argument. The fixed
cruise metric is a performance construction; curvature and constraints can
require positive slack.

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

At admission, the finite schedule uses the initial measured speed over `N`
held intervals and samples curvature along the corresponding nominal station
profile. Its braking-ratio anchor balances declared road load and longitudinal
bias. Both steering and signed braking ratio remain optimization variables at
every stage; no braking or rest tail is appended.

`ltvBicycleModel.finitePredict` constructs condensed affine control-point and
endpoint maps. The sparse native SOCP uses the held controls and interval CLF
slacks, with linear hard rows and Lorentz cones. A carried witness retains and
truncates its schedule, so an inherited stage keeps the same continuous
matrices and physical residual assumptions. An uncertain observation refines
the stored initial set without silently changing the model. A scheduled speed
is a linearization parameter, not an assertion that the physical vehicle
follows that speed exactly.

## Constraints and uncertainty

Steering, signed braking ratio, slip-angle domains, longitudinal speed,
lateral velocity, yaw rate, heading, lateral position and chart station are
hard constraints throughout every held interval. Slip limits describe the
admitted model domain; they do not establish the nonlinear tangent's accuracy.
A positive minimum speed and weak braking authority can be admitted when a
finite certificate passes. There is no universal exact-rest requirement.

The two model rate fields specify continuous derivative-error bounds.
`stateUncertainty.flowTube` encloses the complete affine flow using a Taylor
polynomial, a remainder and arithmetic allowances, represented in Bernstein
form. Endpoint propagation retains cancellation in the nominal transition.
Initial Cartesian boxes require an invertible projection chart; a cell
spanning a noncollinear polyline reference jump needs a separate jump/reset
certificate and is currently rejected.

Bounded nonzero forcing and velocity uncertainty can grow over a finite
certificate. The terminal invariance limitations are derived in
[TERMINAL_CBF_PROOF.md](TERMINAL_CBF_PROOF.md).
The physical plant must remain inside the declared affine inclusion over its
whole tube. Exact affine integration alone does not establish that premise,
nonlinear stability, or low-speed tire accuracy.

## Geometry and proof scope

Cartesian rectangle and road constraints use one fixed lane chart and one
separating normal per target per cell. Directional support is maximized over
the admitted yaw interval, and every Bernstein control-point box must satisfy
the hard halfspaces. Finite quadratic road boundaries must cover the complete
admitted footprint range. Target constraints remain active through certified
exit, regardless of current publication.

[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md) states the finite-witness
acceptance and fail-stop protocol. The certificate covers held intervals of
the declared inclusion until its guarded exit; physical residual validity,
route contracts and execution timing remain assumptions.

## Input contract and migration

`command.actuatorInput`, every optimized stage and `heldActuatorInput` use
`[frontWheelSteeringAngle; brakingRatio]`, with units `[rad; 1]`.
`actuation.brakingRatioMinimum/Maximum` default to `[-1,1]`, and
`clf.brakingRatioWeight` weights the normalized dimensionless input.
The old acceleration bounds, acceleration weight, longitudinal gain and
braking force distribution are rejected as unknown configuration fields.

`command.longitudinalAcceleration` is a derived gross acceleration
`gBeta*beta`, not the second control input or net body derivative. Passive
road load and the declared bias enter the separately reported body derivative.
The plant adapter receives the two axle force requests `beta*mu_i*Fzi` and
converts them to wheel torque/brake pressure. Archived `[delta,a]` input
arrays must not be replayed as beta arrays; their old configuration and
recorded force allocation also belong to the earlier model.
