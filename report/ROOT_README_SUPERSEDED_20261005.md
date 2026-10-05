# Superseded root README (kept for reference)

This is the root `README.md` as it stood before October 5, 2026. It describes
retired controller generations (scheduled LTV bicycle, terminal Frenet bands,
PassVeh14DOF scenario drivers) and must not be read as the current design. The
current overview is [`../README.md`](../README.md).

---

# Collision Avoidance Research Core

This research-only MATLAB repository contains:

- a predictive-control-barrier-function (PCBF) safe-MPC safety
  formulation, after Huang, Wang, Margellos, and Goulart (ECC 2025),
  under exact ego dynamics and actuation;
- an experimental bounded-error real-time-iteration approximation;
- finite route-conditioned quadratic road-boundary constraints;
- a scheduled LTV dynamic-bicycle controller model with hard
  inscribed-polygon friction-circle coupling rows and a smooth
  one-sided rear-braking torque split at the actuator contract; and
- a synchronized cascaded measured-input ego observer and Sharma-model
  NRMM target tracker in transformed coordinates.

## Controller

The mathematical controller is the state-estimation-robust PCBF
safe-MPC problem defined in
[`controller/PCBF_SAFETY.md`](controller/PCBF_SAFETY.md): Huang et
al.'s slack-relaxed safe MPC on the ego–target product system, whose
value function is the predictive control barrier function. Its
closed-loop descent follows from optimality and the shifted-candidate
argument — no cross-sample budget and no imposed descent constraint
exist anywhere in the formulation.
Its state-set transition is the declared scheduled LTV bicycle map with
its one-step model-error box, its commanded actuator input is applied exactly, its
physical targets are covered by their predicted occupied tubes, successive
target tubes are one-sample shift consistent, complete ego rectangles remain
inside the selected route-conditioned road region, successive road regions
contain the shifted previous region, and a certified terminal controller
accepts the ego only when its complete uncertain rectangle's lateral Frenet
interval lies inside one of the nominal-path offset bands `I_R` or `I_L`,
its robust heading error, lateral velocity, and curvature-consistent yaw-rate
error lie in the configured feedback domain, and the ego remains strictly
separated from every target. There is no separate terminal trajectory or
finite longitudinal terminal section. The terminal set adds no
longitudinal-speed admission bound. The independent local policy tracks the
midpoint of the selected Frenet band and
commands only nonpositive longitudinal input until the robust longitudinal,
lateral, and yaw-motion completion tolerances all hold. It then returns no
actuator input and the finite control task ends. There is no `drive/park`
mode, latched handoff state, post-stop hold command,
actuator-error set, or plant/model-disturbance set in the theorem, and no
target is assumed to follow one nominal predicted trajectory. The
recursive-feasibility result applies to that fixed nonlinear closed loop
under its stated terminal-band and target-occupancy assumptions.

The MATLAB MPC function is an experimental real-time-iteration
approximation of this controller: per control sample and disjunction
branch, one linearization and one EXACT LEXICOGRAPHIC solve (two QPs) —
stage A minimizes the accumulated per-stage violation subject to the
hard selected component of `Z_f=Z_f^R∪Z_f^L`, and its optimum is the
reported branch value function; stage B tie-breaks by the CLF and
regularization tiers
under a solver-scale cap. The post-handoff
[`controller/shoulderTerminalController.m`](controller/shoulderTerminalController.m)
is a separate analytic controller and never solves an MPC or QP; the
avoidance MPC uses it only to define the terminal set and in the
recursive-feasibility argument. The experimental MPC does not inherit
the ideal theorem merely because it linearizes the same vehicle model.

The controller state is

```text
[positionX; positionY; yaw;
 longitudinalVelocity; lateralVelocity; yawRate].
```

Its input is the signed drive/brake torque of the complete front axle
followed by the single equivalent front-wheel steering angle of the
two-axle model. The optimizer uses and outputs that angle directly;
there is no steering ratio or Ackermann mapping. Reported per-wheel
quantities split the resulting axle force evenly between the two wheels
of the axle.

During braking the rear axle brakes together with the front, split by
the front share of the total braking torque
`beta = |frontAxleTorque| / (|frontAxleTorque| + |rearAxleTorque|)`.
The one-sided split is smoothed by `T*logisticSigmoid(-T/w)`, which is
exactly zero at zero commanded torque, so coasting produces no phantom
deceleration.

Tires are linear-cornering: each axle's lateral force is its cornering
stiffness times its slip angle at the frozen schedule speed, valid
inside the hard slip-angle rows. The braking-steering coupling of
emergency maneuvers is carried by the hard friction-circle rows: the
commanded `(ax, ay)` stays inside the regular polygon inscribed in the
`mu*g` circle, so hard braking visibly consumes steering authority,
conservatively. Each prediction stage is one exact zero-order-hold
step of the sample time on the scheduled LTV bicycle (per-stage LTI
flow via the matrix exponential). The combined-slip
Dugoff model of the previous controller generation is gone from the
repository; the closed-loop unit tests roll their own nonlinear-bicycle
surrogate plant, and the plant-in-the-loop scenarios use PassVeh14DOF.

Nominal route-following cruise is encoded by a control Lyapunov function
on the path-frame error `[e_y; e_psi; vx - vRef; vy; r - kappa*vRef]`:
`V(e) = e'Pe` with `(P, K)` from the discrete Riccati equation of the
error linearization at the reference cruise, and relaxed predictive rows
`V(e_1) - V(e_0) <= -e'(Q + K'RK)e + delta`, `delta >= 0`, imposed as
ONE relaxed row on the first predicted transition (the classical
reactive relaxed-CLF form; a horizon-wide cruise demand would price
the mandatory terminal-band merge that the hard terminal set forces,
and was measured to distort the first commanded input). The program
objective's CLF tier is that row's
relaxation (weight `clf.relaxationWeight`), a small exact certificate
value anchor on the first decision node using the same `P`, and a
numerical regularization tier
(curvature-feedforward input reference and input-rate costs, the first
stage anchored to the previously applied input). There is no tracking-error weight family, no terminal
tracking weight, and no collision-interior reward; the hard collision
condition remains exactly `dRect > 0` with no clearance parameter, and
`delta(1)` is the reported per-sample conflict measure between safety
and nominal cruise.

The centerline is the only lateral reference. There is no behavior
layer, committed passing-side reference, maneuver phase, or persistent
avoidance direction. Every sample linearizes about the phase-shifted
preceding prediction (or a state-conditioned terminal-band entry seed on the
first sample) and solves one program per branch; there is no
multi-start beyond the enumerated separation sides, no directional
initialization family, and no fixed cruise first input. At every
constrained prediction node, the collision normal
is the exact signed-distance projection/active-face normal of the
rectangle configuration obstacle, hoisted to one side per target where
the nominal penetrates it; the terminal node carries no collision row -
its safety statement is the hard nominal-path Frenet terminal set together
with the terminal contract (`Z_f` inside the safe set, positively invariant
under the analytic terminal law).

The built-in active-set solver can cycle on the semidefinite long-tail QP at
a particular step tolerance. If it exhausts the iteration budget, the
controller retries the identical scaled QP with configured alternate step
tolerances. The objective, all hard constraints, and the explicit
post-solve affine-feasibility check remain unchanged; this is a numerical
solver retry, not a fallback command.

At each active control sample, the controller:

1. builds the scheduled LTV bicycle condensation from the measured
   state and the route (no per-frame Jacobian rollout, tube, trust
   region, or sequential-convex loop);
2. evaluates the geometric nominal (shifted previous plan or schedule
   reference) through that affine map;
3. freezes each Li-style rectangle-distance dual and rectangle
   orientation, hoisting one separation side per target per branch;
4. constructs conservative whole-rectangle tangent rows for the selected
   finite quadratic road segments and folds every robust tightening into
   the row offsets;
5. solves, per branch, the two-stage exact lexicographic PCBF safe-MPC
   problem: stage A minimizes the accumulated per-stage violation of
   the set-tightened collision and road rows subject to the hard
   speed/heading/friction rows, input bounds, and one HARD component of
   the nominal-path Frenet set `Z_f^R∪Z_f^L` on the split terminal state
   (six equality-coupled decision variables with the rectangle support
   coupled affinely to the terminal heading, and no longitudinal
   terminal-section row); stage B optimizes the linearized
   relaxed-CLF decrease rows, value anchor, and regularization among
   the tolerance-minimizers under the lexicographic cap;
6. commands the first input of the lexicographically best branch
   (stage-A value first, stage-B objective second); and
7. reports the committed value function `pcbfValue`, its stage-0 and
   stage-A components, the per-stage slacks, the lexicographic tie
   residual, the episode and candidate diagnostics, CLF relaxations,
   and certificate quantities in the planning metadata.

Positive slack is quantified predicted violation with a recovery
interpretation, never an error and never a fallback. The safety
certificate is Huang et al.'s optimal-value argument, not a mechanism
of the implementation: the shifted previous solution with the terminal
law appended is feasible at the next sample, so the optimal value
satisfies the descent inequality `V*(k+1) <= V*(k) - xi_0(x_k)` by
optimality alone, the realized stage-0 violations are summable, and
the hard-feasible set is positively invariant — safety is recovered
from initially unsafe states. The executed optimization satisfies the
structural hypotheses of that argument within a prediction episode:
the solve is exactly lexicographic (the reported value is stage A's
optimum), the terminal set is hard in the declared-exact currency, the
episode schedule is stored and shifted so the stage data at shifted
nodes equals last sample's data exactly at EVERY in-episode sample
(there are no drift-based re-anchors, and the left/right terminal
component selected at the episode's anchor frame is inherited verbatim — the
hard terminal set is problem data and stays constant in-episode;
episode restarts are reported as
`metadata.scheduleShifted`), an episode is pinned to an explicit
identity — target count, ordered target identities, route, and
configuration — whose change discards the shifted plan and schedule
and starts a new episode at the frame where that target set was first
perceived (the recursive-feasibility claim is per episode: first frame
feasible implies every in-episode frame feasible; an infeasible first
frame claims nothing; the argument is never continued across a
target-set change), and the
warm-start nominal is the shifted candidate itself with the analytic
terminal law, evaluated at the appended stage's start node (the
previous plan's terminal state under the exact shift), as its
appended tail input — so the executed value
sequence satisfies the descent inequality, up to the declared
solver-scale tie tolerance, under the exactness premises; the
closed-loop test `executedValueFunctionDescendsUnderExactModel`
verifies exactly this. The remaining premises are one-step coverage of
the declared model-error box (`model.ltvModelErrorRateBound +
model.plantModelResidualRateBound`, charged as one open-loop Euler step
at the first-step node only; the declared dynamics contain the
deployment plant exactly when a residual set is validated and
installed - currently none is, so every run carries the declared-model
claim only and nothing is asserted for the PassVeh14DOF plant),
estimator shift consistency, and ROBUST containment of the committed
terminal state at the declared radii — audited every frame as
`terminalRobustContainmentCertified` (it holds by construction at zero
declared radii and can honestly fail at large ones, where the lateral
band is robustly empty), plus the terminal contract stated in full in
`controller/PCBF_QP.md`: `Z_f` inside the safe set and positively
invariant under the analytic terminal law, so the appended stage is
admissible and its node demands zero slack.
The lateral bands do not bound progress along the path; consequently,
Huang et al.'s terminal compactness premise requires a separately declared
compact route/state domain and is not established by the executable rows.
The stage rows start at
the first-step reachable node, whose forced slot `xiBar_1` (tightened
by the rigorous open-loop one-step reachable set - no feedback
assumption) is the shifted candidate's stage-0 slack, covering the
next sample's measured stage-0 violation. The controller carries no
cross-sample safety state: each sample measures the stage-0 violation
`xi_0(x_k)` (`measureStageZeroViolation`) and reports the committed
value `pcbfValue = xi_0 +` the stage-A optimum — the executable
surrogate of `V*`; the PCBF property itself remains a statement about
the exact value function.
A solver failure produces no command and raises
`collisionAvoidanceController:optimizationFailure`.

There is no trust region: the physical amplitude bounds are the only
input bounds, and the hard nondimensional speed- and heading-domain
rows cap the linearization validity instead. Two warm starts carry
across
frames: the predicted input plan is phase-shifted as the geometric
nominal, and the QP reuses the previous working set through an `optimwarmstart` object that is
discarded on any change of decision dimension, row structure,
configuration or control mode.

The collision-avoidance condition is exactly \(d_{\mathrm{rect}}>0\):
positive signed distance means separated rectangles, zero means contact, and
negative distance means overlap. There is no \(d_{\min}\) or configurable
minimum-clearance quantity. State-estimation bounds still tighten the
rectangle-distance lower bound; those set-derived tightenings are not a
geometric clearance parameter. The experimental QP uses the closed affine
approximation of this open condition to generate candidates; its solver
tolerance is not a physical distance or a replacement \(d_{\min}\).

Road contact is separately permitted but road-boundary crossing is not. To
enable road constraints, pass a scalar third argument with `centerline`,
`boundaries`, and the selected `routeBranchId`. Each finite boundary stores
its local `origin`, right-handed orthonormal
`longitudinalDirection`/`lateralDirection`, quadratic `coefficients`,
`parameterRange`, `safeSideSign`, optional
`normalDistanceErrorBound`, branch identifier, and boundary identifier.
The optional `coveragePolicy` is `strict` by default; a finite segment
created from a range-limited sensor may explicitly use
`perceptionLimited`.
Alternative intersection branches are not simultaneously intersected.
Every affine road row includes the complete ego-rectangle support, a
quadratic-curvature correction over the input-box reachable station
range, ego
position/yaw estimation tightening, and any declared normal-distance fitting
bound. These are derived tightenings, not a road-clearance parameter.

A finite curve is inactive when the full reachable rectangle is disjoint
from its parameter range and valid when that rectangle is fully covered.
Partial coverage is rejected instead of clamping or infinitely extending
the curve. The upstream route-conditioned corridor must therefore provide
adequately overlapping segments near joins and intersections. Runtime uses
only the conservative affine rows; the exact
`quadraticRoadBoundaryRectangleMargin` utility is for nonlinear theory
checks and tests, not post-solve rollout acceptance.
For `perceptionLimited` data, a prediction node whose complete reachable
rectangle is not covered by the measured parameter range is inactive. In the
30 m experiments this means exactly that no curb is modeled or constrained
outside the current sensing range. The sensed quadratic is never extended.

Terminal sets are separate from sensed curbs. The nominal path is their sole
geometric reference: scenarios supply the signed lateral-offset bands
`terminalLateralOffsetBands.right = I_R` and
`terminalLateralOffsetBands.left = I_L`. With Frenet lateral coordinate
`e_y`, heading error `e_psi`, ego half-length `l_e/2`, and half-width
`w_e/2`, the complete rectangle occupies the lateral interval

```text
J_perp = [e_y - h_perp(e_psi), e_y + h_perp(e_psi)]
h_perp = l_e/2*abs(sin(e_psi)) + w_e/2*abs(cos(e_psi)).
```

The core definitions are `Z_f^R: J_perp subset I_R` and
`Z_f^L: J_perp subset I_L`, together with the configured heading-error,
lateral-velocity, and curvature-consistent yaw-rate conditions; their union
is `Z_f`. No terminal centerline, boundary polynomial, parameter range, or
longitudinal coverage row is generated. The bands are 2.6 m wide in the
three 5 m by 2 m scenario drivers, and the terminal controller tracks the
selected band midpoint while continuously braking to a stop. Parser
validation requires each band to be ordered, wider than the vehicle, on the
declared side of the path, and disjoint from the other band.

Perception, optimization, and control use a common default period of 0.1 s.
There is ONE prediction horizon, \(N=48\) (4.8 s), exactly as in
Huang et al.'s problem (8): slacked collision and road rows uniformly
cover stages \(0,\ldots,N-1\), the relaxed CLF rows span the same
horizon, and the terminal node carries only one selected hard component
of the Frenet terminal set; the
horizon length is fixed for every
sample. A road description without both `I_R` and `I_L` is rejected.

Row-tightening radii are declared, not propagated: the measured
estimation radii at the current node, the rigorous one-step reachable
set `abs(A_1)*E + Ts*W` at the first-step node (`W` the declared
`model.ltvModelErrorRateBound + model.plantModelResidualRateBound`
box - the only place model error is charged), and the measured radii
held constant beyond it, with no multi-step reachability claim. The
yaw allowance of every frozen stage row is the declared
heading-domain cap plus the measured yaw radius, converted through the
rotation-Hausdorff bound so optimized yaw cannot invalidate a row
built from the frozen ego orientation.
Target position
and yaw boxes use the supplied initial, rate, and
prediction-acceleration-estimation bounds. Missing estimation bounds mean
singleton initial sets.

For nonzero position uncertainty, the frozen separating normal is computed
from the rectangle configuration obstacle expanded by the combined ego and
target position-error box. Orientation uncertainty is added through
rectangle-rotation Hausdorff bounds. Collision and road rows are
evaluated at the prediction nodes only; the terminal node carries the
hard selected Frenet-band set and no collision or road row.

The ego input supplies
`controllerStateErrorBound` in controller-state order. Each target may
supply `targetPositionInertialErrorBound`,
`targetVelocityInertialErrorBound`,
`targetPredictionAccelerationInertialErrorBound`,
`targetYawErrorBound`, `targetYawRateErrorBound`, and
`targetPredictionYawAccelerationErrorBound`. The observer does not invent
these bounds; they must come from the estimator-state error contract.
For the nonlinear theorem, the resulting target tube must contain the
physical target throughout every executed interval, and each newly predicted
tube must be a subset of the previous prediction shifted forward by one
sample. Target containment supplies physical safety; shift inclusion supplies
the recursive-feasibility argument. Exact equality to the deterministic
prediction center is only a singleton special case.

After every solve, the predicted input sequence is retained as numerical
warm-start state. The next sample advances the complete sequence by one
stage, repeats its final input, and repropagates the nominal state
trajectory from the current ego estimate before the single new QP is
built. The warm start is numerical memory only: it does not constrain
the new QP. A configuration change, analytic terminal handoff, or
`collisionAvoidanceController("resetNominalTrajectory")` clears it.
A changed sensed road fit still reuses the preceding input sequence,
while all constraints use the current road geometry. Independent
controller sessions must call the reset action before their first
sample. Both level solves must return a successful solver status;
otherwise the controller raises
`collisionAvoidanceController:optimizationFailure` and produces no
command.

An estimate completes the task only when it belongs to one declared
Frenet terminal component and its robust longitudinal-speed,
lateral-speed, and yaw-rate bounds meet their completion tolerances. The
controller then returns empty command, prediction, and planning-problem
outputs. A stationary state elsewhere remains an MPC problem; stopping
outside `Z_f^R∪Z_f^L` is not a terminal state.

The decision vector stays condensed: the \(N\) control inputs, one
nonnegative safety slack per constrained prediction stage (in metres),
one CLF relaxation per performance stage (stage B only), and the six
split terminal-state variables — 160-ish variables at the default
horizon, depending on the constrained-stage count. Speed-domain rows,
physical input-amplitude bounds, and the selected Frenet terminal set are
hard; every collision and road row is slacked, and the accumulated
stage slack is the quantity stage A minimizes exactly — the branch
value function, with no safety weight anywhere.
The speed domain includes a planned floor
(`model.plannedSpeedMinimum`) that the MPC may not plan below, distinct
from `model.speedMinimum`, which validates a measured state and stays at
zero so a stopped vehicle remains a legitimate input. The floor is
applied as `min(plannedSpeedMinimum, currentSpeed)`, so it never demands
speed the vehicle does not already have, and slowing past it is the
terminal controller's job. Terminal-set rows are divided by their own
physical scales (band width and feedback-domain bounds), act on the split
terminal state, and are HARD; the floor is
additionally frozen per prediction episode so the hard speed rows
cannot rise mid-episode and exclude the shifted candidate. No rows are
pruned: the slacked feasible set is exactly the assembled one.

The terminal constraints require:

- robust complete uncertain-rectangle lateral containment in `I_R` or
  `I_L` in the nominal-path Frenet frame;
- the configured heading-error, lateral-velocity, and
  curvature-consistent yaw-rate bounds;
- no terminal-specific longitudinal-position or longitudinal-speed bound
  beyond the ordinary vehicle-model domain; and
- strict current target separation for direct handoff, with terminal/target
  disjointness retained as the theory-side scenario contract.

At the start of any invocation, `Z_f^R/Z_f^L` membership is evaluated
directly from the current estimate set. It uses the exact maximum rectangle
lateral support over the yaw-error interval and bounds nominal-path heading
and curvature variation across every segment reachable by the position-error
ball. The predictive terminal node freezes the local Frenet frame at the
nominal terminal projection and imposes four affine band-containment rows,
two heading rows, two lateral-velocity rows, and two yaw-rate-error rows on
the split terminal state. Neither path has a longitudinal coverage test.
Current admission still requires the complete speed-error interval to lie in
the ordinary vehicle-model domain. Solver feasibility tolerance is not used
as a geometric penetration allowance. Handoff occurs
whenever the current estimate set satisfies membership, whether or not a
target is present; membership itself requires strict separation from
every current target, which is what distinguishes handing off with a
target in view from ignoring it. The independent controller still has no
target-tube input or one-step separation check, so a target-present
handoff rests on the current membership test alone and its predictive
stage-\(M\) terminal condition remains a noncertifying finite-horizon
waypoint rather than an executable recursive backup. Leaving the domain
returns control to the MPC, so no handoff flag or terminal side is
latched.
Nonlinear invariance of the domain remains an explicit theoretical
certificate premise rather than a consequence of the affine QP. Inside the
domain, the separate analytic controller continues band-midpoint tracking and
uses only nonpositive longitudinal input. Its braking command is tapered over
the final sample to avoid reverse motion. Once the robust longitudinal,
lateral, and yaw-motion completion test holds, no MPC, terminal-control
command, or post-stop hold command is produced.
Road geometries without both terminal lateral-offset bands are rejected.

The controller has no estimator, supervisory route chooser, deadline policy,
fallback controller, or post-solve nonlinear acceptance test. It has no
preceding-actuator-input state or input-rate constraint. Its only
preceding-plan memory is the predicted input sequence used to initialize
the next linearization.
Every numeric path runs the reviewed MATLAB sources; there is no compiled
kernel dispatch, and the MATLAB path provides no worst-case 0.1 s deadline
guarantee.

The executable result is an experimental safe-MPC RTI affine
MPC whose committed value is a linearized surrogate of the ideal
worst-case violation sum — the PCBF is the exact problem's value
function; no affine optimum realizes it, and none is claimed to. Its
value depends on the
current linearization center and branch restriction, and its set
statements are conditional on the supplied state-estimation bounds, the
numerical affine-remainder bound, and the validated plant-model
residual bound containing their stated errors, and on the reported
violation being zero. Physical
actuation and one-step model-error coverage are
assumptions of the ideal theorem. The executable predictor uses exact
zero-order-hold scheduled transitions and prediction-node checks; it does not
provide a validated continuous flowpipe enclosure.
The MATLAB predictor consumes target-error bounds but does not independently
certify physical-target containment or prediction-tube shift inclusion.
The Frenet-band terminal rows do not prove one-step terminal invariance,
continuous-time target separation during the next held-input interval, or
that every newly linearized and newly frozen QP is feasible. The executable
controller does not perform an online terminal flowpipe check, a nonlinear
interval acceptance test, or a deadline-certified supervision step. Any
first-frame or later-frame experimental failure is
intentionally exposed as an optimization failure.

See
[`controller/PCBF_SAFETY.md`](controller/PCBF_SAFETY.md)
for the PCBF safe-MPC theory and
[`controller/PCBF_QP.md`](controller/PCBF_QP.md)
for the experimental QP formulation.

## Estimator

All GPS, IMU, gyroscope, and radar signals must be synchronized externally to
one common sample period before entering the observer. The observer is a
cascaded measured-input design:

1. A certified rotation correspondence converts the GNSS-course vector pair
   into one hard yaw arc. The channel first computes
   `betaHat = asin(lrE*yawRateMeasured/norm(gnssVelocity))` and subtracts it
   from GNSS course. Its arc includes an exact pointwise sideslip-error
   radius derived from GNSS velocity error, gyroscope error, and a declared
   single-track yaw-rate mismatch; the full sideslip operating range is no
   longer inserted as a fixed heading disturbance. The arc centre is the yaw
   measurement `yF` and its radius is `BF`. There are no confidence weights,
   heading-mode selectors, or tuned magnitude thresholds: an uncertainty ball
   containing the origin certifies no direction, which is a violated
   operating-domain assumption rather than an estimator output.
2. A body-velocity observer uses the measured body acceleration directly
   and corrects with the rotated GNSS velocity innovation. Its rotation
   term is skew-symmetric, so `P = I` certifies the decay rate `kv`
   exactly for every measured yaw rate, with no bound on the yaw-rate
   derivative.
3. A GNSS position observer propagates the estimated inertial velocity.
5. A high-gain NRMM target observer runs as an SO(2)-covariant third-order
   chain in the transformed coordinates
   `[rho; q; s]` — radar relative position, absolute target velocity in the
   ego frame, and absolute target acceleration in the ego frame. One LMI in
   the gains and a metric recovers the injection direction: it minimizes a
   bound on the output-noise velocity variance subject to the structured
   Lipschitz dissipation inequality at the transit decay floor, and a
   dimensionless transit-plus-ultimate certified-state minimax solve selects
   its physical bandwidth.

Inertial-sensor biases are neglected: the accelerometer and gyroscope are
treated as unbiased, so no stage carries a bias state, both pseudo-heading
channels read measurements only, and every cascade coupling is
feedforward. The comparison system below is therefore triangular and needs
no small-gain condition.

The transformed target model is an exact one-to-one coordinate change of the
Sharma NRMM target model (constant scalar acceleration, constant sideslip,
kinematic single track); its dynamics contain only the ego body velocity and
the measured yaw rate. No ego jerk, ego angular acceleration, or derivative
of any measured input appears anywhere in the observer core, and no
zero-jerk closure is assumed. The published ego yaw rate is the measured
gyroscope rate. The target nonlinearity is evaluated
through a globally Lipschitz saturation extension that equals the exact map
on the certified operating domain, so high-gain peaking cannot evaluate the
model outside its physical domain and domain exits are audited rather than
rejected.

The complete yaw, velocity, position, and target differential
equations are integrated together using fixed RK4 substeps. The
GNSS-position and radar innovations act through continuous output
predictors: each frame resets the predictor from the measurement and then
integrates it with the estimated output derivative across the interval, so
the innovation does not accumulate the known intersample motion of the
measured quantity. Runtime accepts complete synchronized sensor frames; it
does not implement asynchronous channel events or retrospective
interpolation. A radar row is corrected only when `radarDetectionAvailable`
is true. An unavailable row must contain `NaN` and advances the target
model without inventing a radar observation.
An acquired target may therefore coast inside the estimator for the
configured 0.5 s timeout, but the estimator publishes it to the controller
only while the current radar sample observes it. The track is retired when
that timeout expires and must be reacquired from radar; inactive observer
slots are re-anchored to a finite dormant prior rather than propagated
indefinitely. Under the synthetic sensor contract there are no missed
detections inside the declared range; an out-of-range extrapolation is not
silently converted into a controller collision constraint.

The estimator/controller integration uses
`estimatorControllerIntegrationConfig`. Its 0.1 s synthetic sensor contract
contains GNSS position and velocity, center-of-mass body acceleration,
gyroscope yaw rate, and radar body-frame relative position. The
bounded-uniform noise maxima are 0.04 m, 0.05 m/s, 0.03 m/s², 0.0015 rad/s,
and 0.04 m, respectively; radar detections are generated only at true ranges
not exceeding 30 m. Controller vehicle parameters are copied from the loaded
`PassVeh14DOF` plant rather than duplicated in the estimator configuration.

The straight, circular, and retained S-curve scenario drivers use the same
30 m radial range for target and curb perception. The active automated suite
still contains only the straight and circular cases. Lane edges are fixed
nominal-path offsets of 6 m right and 8 m left. A 2.6 m shoulder, about one
2.0 m vehicle width plus alignment allowance, lies outside each lane edge, so the
sensed right and left curbs are at nominal offsets 8.6 m and 10.6 m. Only
in-range curb points are fitted as finite local quadratics; prediction nodes
outside that finite sensed interval contain no curb constraint. Separately,
the known nominal path supplies the direct Frenet bands
`I_R=[-8.6,-6.0] m` and `I_L=[8.0,10.6] m`, with no generated terminal
curves or lookahead sections. The road evaluator checks a 5 m by 2 m ego
footprint against each applicable finite sensed fit after subtracting its
continuous source-polyline fitting-error bound; terminal admission checks
the footprint directly against the selected offset band. The target starts
100 m away,
outside the shared sensing range. The controller therefore receives an empty
target set while the ego first cruises on the centerline. The first in-range
radar sample occurs after at least 2 s of verified pre-detection cruise. That
first sample publishes a provisional radial-oncoming velocity using the
configured target-speed prior; later radar samples correct the sampled target
observer and output filter. The active straight case uses a 0.8 m
target-center lateral offset. The current circular and retained-S video cases
use 1.2 m with the same equal 2 m-wide target and ego rectangles, so their
no-avoidance baselines still have 0.8 m of lateral overlap without artificial
center-to-center symmetry. The retained S visualization starts at 80 m, so
radar entry occurs after 2.5 s of pre-detection cruise and the nominal
encounter occurs at the zero-curvature transition between its two bends.

The estimator/controller adapter used by these vehicle scenarios runs the
observer at its 80 Hz sensor rate between 10 Hz controller instants and
publishes the observer output directly, smoothed by a separately
pole-placed constant-velocity alpha-beta output filter on the published
target position and velocity. This preserves sample-to-sample translation
consistency instead of forwarding raw radar-position jitter through the
high-gain acceleration channel. The measured GNSS velocity initializes ego
position, speed, and course from a single frame. For a target that was not
visible during initialization, each radar point is first transformed to
inertial position using the synchronized GNSS/yaw estimate. The first point
uses the configured oncoming speed prior to seed the absolute target
velocity `q`; subsequent measurements correct the direction and state, and
the acceleration `s` is seeded at zero. The default video suite configures
both vehicles at 10 m/s.
The dormant internal target slot is not exposed to the controller. No ego
truth velocity/yaw or target truth velocity enters the controller.

The synthetic sensor adapter preserves the same rotation correspondence as
the observer geometry: it backward-differences the inertial velocity used for
GNSS and then rotates that acceleration into the current body frame for the
IMU sample. This avoids an inconsistent approximation that can arise from
differencing body velocity while interpolating yaw and yaw rate independently.

The visualization driver runs 18 s by default and uses a 300 m forward
centerline domain for the circular case so the controlled and no-avoidance
trajectories remain defined through the recovery interval. It renders the
two active cases plus the retained S-curve visualization without adding S
back to `runEstimatedStateAvoidanceScenarios`. Each full-HD video shows the
moving 30 m sensing circle, raw in-range outer-curb samples, the current
finite right/left quadratic fits, the 2.6 m terminal offset bands and their
lane-edge entries, truth and estimated vehicle states, rectangle separation,
road margin, controls, and body-frame accelerations. Per-video acceptance
still requires collision avoidance,
road containment, forward motion, estimator use, delayed target acquisition,
and recovery to centerline cruise. Recovery orientation is the inertial velocity
course relative to the centerline tangent, rather than body yaw alone: a
dynamically cornering vehicle may need nonzero sideslip while its center of
mass follows the curve. The unchanged significant-coupling thresholds are a
suite-coverage criterion: at least one of the three rendered scenarios must
satisfy them.

These closed-loop runs are empirical estimator-in-the-loop tests. The
sampled implementation does not yet produce a certified online
controller-state error box, so the runs use point estimates and do not by
themselves establish the nonlinear recursive-safety theorem with
state-estimation sets.

Configuration declares only physical operating domains, deterministic sensor
bounds, and sample timing. Every gain is solved by one balance principle:
each scalar bandwidth is the projected balance point of persistent-disturbance
leakage against noise or one-sample-innovation amplification on the physical
gain interval — yaw, body velocity, and position take the closed form
`k = proj(sqrt(a/(Ts*b)))`. The target direction is recovered from the
noise-variance LMI under the Lipschitz dissipation inequality, a free metric
certifies each candidate bandwidth, and the certified bound solves the
physical bandwidth as a scalar minimax. Legacy manual decay-rate and gain-shape fields are rejected.

The essential yaw–body-velocity–target error system is a positive lower-
triangular cascade. Backward positive weights satisfy `w'*A = -ones(1,3)`,
giving one explicit linear copositive ISS function rather than a generic
five-state Lyapunov-equation solve. Position is a downstream output-filter
corollary; its ultimate bound follows by scalar forward substitution, so
no auxiliary matrix exists.
The certified bounds remain worst-case disturbance chains and
are orders of magnitude more conservative than measured errors; the
integration configuration therefore publishes measured engineering error
radii, labeled as such. The continuous-time theorem does not automatically
cover the RK4 predictor-reset sampled realization; sampled behavior is
validated empirically by tests and scenario runs. See
[`estimator/OBSERVER_ISS_THEORY.md`](estimator/OBSERVER_ISS_THEORY.md) for details.

## Layout

- `config/` — configuration entry points: `collisionAvoidanceControllerConfig`
  (defaults, override merge, and validation), `nrmmTrackingConfig`,
  `estimatorControllerIntegrationConfig`
- `controller/` — the MPC as one orchestration entry over single-purpose
  modules:
  - `collisionAvoidanceController.m` — public entry, prediction-model
    assembly, terminal handoff, command construction
  - `readPlanningInputs.m` — the ego / road-geometry / target input contract
  - `terminalFrenetSetMembership.m`, `terminalFrenetReference.m` —
    terminal-set admission audits and the nominal-path Frenet reference
  - `formulateAvoidanceProblem.m` — rows, CLF data, and objective of
    one branch's safe-MPC program
  - `solveAvoidancePlan.m` — geometric nominal, branch loop, and the
    safe-MPC QP solve
  - `measureStageZeroViolation.m` — the reported stage-0 violation of
    the committed value function
  - `rectangleConfigurationDistance.m`, `laneProjection.m`,
    `quadraticRoadBoundaryRectangleMargin.m` — geometry
  - `ltvBicycleStageMatrices.m`, `ltvBicyclePrediction.m`,
    `frictionCirclePolygonRows.m` — the scheduled LTV bicycle model, its
    condensation, and the hard friction-coupling rows
  - `shoulderTerminalController.m` — the separate analytic post-handoff
    controller; `qpWarmStartStore.m` — cross-frame numerical memory
- `estimator/` — `onlineNrmmTrackingRuntime.m` (sampled orchestration),
  `certifiedRotationCorrespondence.m` and
  `certifiedKinematicCourseCorrespondence.m` (certified yaw geometry),
  `nrmmObserverVectorField.m` (the continuous mathematical core),
  `nrmmObserverCertificate.m` (the copositive triangular ISS certificate),
  `synthesizeNrmmObserverGains.m` and
  `synthesizeTargetTrackerCertificate.m` (the balance-principle gain solve
  and the target LMI/minimax certificate),
  `nrmmTargetTrackerDerivative.m` (covariant NRMM model with Lipschitz
  extension), and `OBSERVER_ISS_THEORY.md`
- `perception/` — the Python YOLO+LiDAR target-position pipeline and the
  offline MATLAB curb-detection experiment (`curbDetectionExperiment/`,
  algorithms together with their evaluation and export tooling)
- `scripts/` — vehicle scenario drivers, the estimator-controller adapter,
  estimator research scenarios and benchmarks, video rendering
- `report/` — [experiment reports](report/README.md), validation results,
  runtime analyses, and recorded implementation findings
- `tests/` — `matlab.unittest` behavior tests

## Run

Run the synchronized synthetic scenario:

```bash
matlab -batch "addpath('scripts'); runOnlineNrmmComplexManeuverScenario('Plot',false);"
```

Run the Monte-Carlo estimator tracking-error benchmark:

```bash
matlab -batch "addpath('scripts'); runOnlineNrmmTrackingErrorBenchmark;"
```

Run all tests:

```bash
matlab -batch "results=runtests('tests'); assertSuccess(results)"
```

Run only the online estimator tests:

```bash
matlab -batch "results=runtests('tests/onlineNrmmTrackingRuntimeTest.m'); assertSuccess(results)"
```

Run the two active estimator-in-the-loop avoidance scenarios:

```bash
matlab -batch "addpath('scripts'); result=runEstimatedStateAvoidanceScenarios; assert(result.allScenariosPassed)"
```

Generate the three synchronized H.264 videos in the repository root:

```bash
matlab -batch "addpath('scripts'); summary=generateEstimatedStateAvoidanceVideos; assert(summary.allVideosCreated)"
```

Run the four active controller scenarios:

```bash
matlab -batch "addpath('scripts'); runStraightCenterlineCruiseScenario(Plot=false,Report=false);"
matlab -batch "addpath('scripts'); runCircularCenterlineCruiseScenario(Plot=false,Report=false);"
matlab -batch "addpath('scripts'); runOncomingVehicleAvoidanceScenario(Plot=false,Report=false);"
matlab -batch "addpath('scripts'); runCircularCenterlineStraightTargetAvoidanceScenario(Plot=false,Report=false);"
```

The S-curve research drivers remain outside the active automated acceptance
entry point. The video generator invokes one retained S case separately and
requires that exact rendered run to pass before replacing its video.
