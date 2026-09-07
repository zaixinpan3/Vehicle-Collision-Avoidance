# Encounter-scoped predictive CBF–CLF controller

The version-9 runtime implements a finite continuation witness for the
[governing encounter specification](ENCOUNTER_SCOPED_CBF_CLF.md). Each admitted
target must have a finite Cartesian motion inclusion and a supported exit
contract. The controller certifies every held interval, carries its remaining
witness through observations, and discharges a target only
when its uncertain footprint satisfies the exit guard.

Every issued command requires a newly verified solution from the current
optimization call. If no maneuver candidate supplies one, the controller
raises `collisionAvoidanceController:noCertifiedContinuation`. Simulation
drivers record the failed sample and terminate before another plant interval.
Stored inputs are never issued as a backup after an unsuccessful solve.

```matlab
certificate = [];
[command, inputs, problem, certificate] = ...
    collisionAvoidanceController(ego, targets, road, cfg, certificate);
```

Retain the fourth output and pass it back at the next sample. The convenience
four-input interface stores the same state persistently. Resetting that
interface does not erase an explicitly supplied witness. Version-8 certificates
are rejected. Actuator order remains steering angle in radians followed by the
dimensionless signed braking ratio. The command is held for
`cfg.controller.sampleTime`; no command clipping follows verification.

## Supported finite contracts

`targets` is a structure array with unique stable track identifiers. Every
admitted record includes the `encounterContract` fields documented in
[TARGET_PREDICTION_CONTRACT.md](TARGET_PREDICTION_CONTRACT.md). `ego.stateTime`
is required during encounters. A missing observation retains the target and
propagates its existing uncertainty. Valid observations intersect the carried
reachable set, retaining its nominal center. Empty intersections, changed
motion contracts, changed model/road/CLF data, incorrect sample times, and
reported actuator mismatches invalidate inherited authority.

The implemented exit guard is `nonreturningHalfspace`: the whole uncertain
target footprint clears a declared plane, and its route contract keeps it
beyond that plane plus the required clearance thereafter. Admission checks that the complete allowed ego
route, lateral domain, and footprint lie on the opposite side. This is a
conditional route assumption supplied by the caller. Perception absence,
positive instantaneous distance, and forecast expiry are never substitutes.
Discharged records retain their route obligation without extending the motion
forecast. A later conflicting target needs a new identity and joint admission;
reliable early detection remains an external sensing assumption.

Finite holding, arbitrary deadline renewal, route changes after discharge, and
nonreturn guards for other encounter classes are not implemented. Such inputs
fail admission. There is no automatic forecast extension or stationary-ego
shortcut. Separate rest/dissipation utilities are not prerequisites of this
finite construction and are not invoked by the controller.

## Prediction and interval proof

The ego uses the existing scheduled Frenet bicycle with modified Fiala tire
linearization, signed braking ratio, passive road load, and explicit
longitudinal bias. Its physical validity is conditional on the supplied
`ltvModelErrorRateBound` and `plantModelResidualRateBound` enclosing the admitted
plant throughout the tube. Nonzero finite residuals are propagated. This change
does not establish those enclosures experimentally for a nonlinear vehicle.

`ltvBicycleModel.finitePredict` divides each held sample into certification
cells. `stateUncertainty.flowTube` converts a Taylor polynomial into Bernstein
control points, with a remainder and arithmetic allowance added to the radius.
The convex hull property encloses the complete cell. Endpoint propagation
retains cancellation in the transition matrix to avoid compounding the swept
absolute-power bound at every cell. Exact matrix-exponential stage matrices are
also exposed for independent declared-model rollouts.

A cell spanning a noncollinear polyline reference jump is rejected until an
explicit jump/reset certificate is available. Each cell uses one lane chart and one fixed separating normal per active
target. `avoidanceSafetyGeometry` imposes rectangle support, road, chart,
state-domain, and tire-slip rows on every control point. Quadratic road
boundaries must cover the complete cell footprint; perception-limited gaps
are rejected. Directional rectangle support is maximized over yaw intervals.
Normals arise from the maneuver's anchor trajectory; the construction is a
conservative family of convex domains, not a global nonlinear maneuver solve.

All physical and model-domain constraints are hard. Safety margins are in
metres, with common unit normalization. The carried nonnegative margin is
capped by `maximumCarriedMargin` and includes swept separation and exit margins.
The next accepted witness must have margin at least
`(1-barrierFraction)*previousMargin`.

## CLF and maneuver optimization

Each candidate optimizes all held steering/braking inputs and one nonnegative
CLF slack per interval. Candidates are `yield`, `passLeft`, and `passRight`;
these select overlapping lateral corridors and independent geometric anchors.
A candidate keeps its maneuver throughout its finite witness. A new candidate
can switch only if its complete corridor admits the current uncertain state.
The cost includes tracking error, input effort, input changes divided by sample
time, squared CLF slacks, and a maneuver-switch penalty. No hard slew-rate
limit is invented.

A common positive-definite Riccati metric regulates
`[d; ePsi; vx-vDes; vy-vyEq; r-rEq]`. The optional reference is affine in absolute time:
`referenceStart = [0;0;referenceSpeed;0;0] + referenceOffset + referenceRate*(time-referenceEpoch)`.
The last two offset entries supply constant lateral-velocity and yaw-rate
references; they default to zero.
The full reference derivative enters the CLF residual; the metric is constant.
The optimization uses convex quadratic upper bounds of `Vdot+c*V`, including
state uncertainty and bounded process residuals, over every Bernstein cell.
These bounds are represented as Lorentz cones. Thus the actual held interval
satisfies

\[
V(t_i+\tau)\le e^{-c\tau}V(t_i)
 +\frac{1-e^{-c\tau}}{c}\delta_i,\qquad 0\le\tau\le h.
\]

Slacks have no hard cap, affect only tracking, and are recomputed conservatively
for the returned fixed input vector. The numerical backend is the existing
Clarabel native bridge. A solver hook receives `P,q,A,b,cones` and a callable
`defaultSolver`; the old affine-QP-only hook contract is superseded.

## Continuation, acceptance, and diagnostics

`certifyAvoidancePlan` independently evaluates hard rows, swept/exit margins,
and every predictive CLF bound. A positive solver status supplies no authority
by itself. A negative physical margin is never accepted using a solver
feasibility tolerance. Numerical reserves are charged before command issuance.
The optimization tightens nonconstant hard rows by twice the configured
reserve, while acceptance charges one reserve plus its arithmetic allowance.
This preserves verifiable room at an active constraint without relaxing the
physical safety check.

After a valid sample, the first input and first slack are substituted into the
stored affine maps and removed. The remaining dynamics, safety rows, and CLF
bounds are truncated algebraically. There is no appended node. New optimization
uses the retained schedule. The truncated witness supplies prediction and
optimization context, not a backup execution policy. An unsuccessful call
reports control failure even when that stored sequence remains feasible.
New-target admission requires a newly checked joint witness and cannot use an
incumbent that omits that target. With no active encounters, a fresh finite
cruise plan is allowed; it does not claim perpetual road safety.

Certificate outputs include `remainingSteps`, absolute `deadline`, verified
`margin`, `maneuver`, target contracts/status, controls, reachable tubes, CLF
slacks, and acceptance results. Diagnostics report solver calls,
active/discharged target keys, interval safety scope, and actual wall timings.
The compatibility field `fallbackUsed` is always false for returned commands.
Runtime measurements are research observations, not a real-time guarantee.
