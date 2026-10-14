# Real-time-iteration PCBF/CLF control

Each measured sample runs one RTI step: one reference, one affine dynamics and
geometry model, and at most two native convex solves on that same model. The
primary objective minimizes PCBF deficits; the secondary minimizes CLF slack at
the attained primary cap. There is no within-frame nonlinear iteration, elastic
restoration, merit acceptance loop, trust-region backtracking or restart.

The optimized actuator inputs are issued unchanged. A nonlinear rollout predicts
their effect and updates the next frame's trust scale; it does not certify the
nonlinear constraints, accept a different iterate or repair an input. An
infeasible, unfinished or timed-out solve produces no command. No previous-plan
execution or alternate controller substitutes for a solved command.

This is an approximate numerical implementation. Recursive feasibility concerns
the ideal nonlinear problem and its causal information-set policies, independently
of numerical solver success. See [ROBUST_RECURSIVE_FEASIBILITY.md](ROBUST_RECURSIVE_FEASIBILITY.md)
and [OBSERVER_ROBUST_PCBF_THEORY.tex](OBSERVER_ROBUST_PCBF_THEORY.tex). A completed
RTI subproblem is not nonlinear convergence, full nonlinear safety, or a
certificate of those theorem premises.

## Reference and horizon

The state is `x = [px; py; yaw; vx; vy; yawRate]` and the input is
`u = [frontWheelSteeringAngle; signedLongitudinalForceRatio]`. The modified-Fiala
bicycle blends into a kinematic model at low speed. RK4 and its variational
equations use the same mesh. On the default one-step mesh the midpoint is the
endpoint interpolant.

A usable previous input sequence is shifted and rolled out from the current
measured estimate. Missing tail inputs use nominal lane feedback. Without an
inherited sequence, nominal feedback supplies the reference. If prescribed cruise
intersects the target, cold initialization instead uses one mild-braking rollout,
with lane feedback for steering and the terminal braking rate for longitudinal
input, respecting actuator and slew limits. The same selection applies when a
conflict first appears. This is one initialization selected before optimization;
no old-reference solve is attempted first, no avoidance path is planned, and no
reference input can be executed without optimization. A head-on encounter may
still be infeasible: braking alone does not produce a lateral passing reference.

Without a target the horizon is `controller.horizonSteps`. With a target,
`rti.minimumPredictionSeconds` is the minimum duration (1 s). The nearest
predicted cruise interaction plus `rti.settlingSeconds` (0.5 s) can extend it to
`rti.predictionSeconds` (2 s). A longer inherited sequence is retained while a
target is present, within `controller.maximumHorizonSteps` (40 by default). This selects duration,
not an avoidance route. Initial avoidance and terminal membership are not
prerequisites for constructing the one subproblem; failure to satisfy its hard
rows remains an optimization failure.

Every reference is an exact declared-model rollout. The dynamics increments obey

    dx_0 = 0,
    dx_{i+1} = A_i dx_i + B_i du_i.

The dynamics, rectangle multipliers and terminal tangents are rebuilt once per
sample. The nonlinear rollout of the optimized inputs is stored separately from
the affine predicted states. No second model is constructed from that rollout.

## Constraints and objective hierarchy

State and input increments lie in local trust boxes. Actual longitudinal-force
magnitude and optional slew limits remain hard. Steering has no artificial
actuator magnitude or slew limit; the Fiala angle domain is enforced. State
limits, sideslip, optional road-corner rows, rear handling cones and terminal
rows retain their existing meanings. No shared elastic variable relaxes them.

Nominal collision rows in the first `controller.horizonSteps` nodes carry
nonnegative PCBF slacks. With observer uncertainty, every tightened collision
node also carries a nonnegative robustness-deficit slack; the nominal collision
row remains hard after the prefix. These explicit collision deficits are
separate from physical state, road, handling and terminal constraints.

The primary stage minimizes `sum(xi)`. If the zero correction with zero slacks
satisfies every primary constraint, it attains the global lower bound and no
native primary solve is needed. Otherwise the native conic solver runs once.
The secondary stage caps the attained primary objective, with the configured
numerical tie, and minimizes the target-independent soft CLF condition

    V(xhat_next) <= rho * V(xhat_now) + clfSlack,
    rho = exp(-2*h/clf.convergenceTimeConstantSeconds).

`V` is the single precomputed transverse quadratic at prescribed-speed/path
trim. Only the estimate enters it; it does not ask the worst member of an
observer enclosure to contract to zero. A bounded whole-horizon tracking and
input-proximity tie distinguishes passing/recovery from following a slow lead
and stabilizes otherwise free future controls. It introduces no additional
avoidance controller. Both stages must complete; an unfinished secondary stage
does not cause execution of the primary point.

Rear adhesion uses `w = vy-lr*yawRate`,
`k = 3*rearLongitudinalForceScale/rearCorneringStiffness`, and

    norm([w/k; vx*b]) <= vx,
    vx*b approximately vx_bar*b_bar + b_bar*dvx + vx_bar*db.

Both derivatives of this bilinear product are retained. Below the kinematic
threshold the cone reduces to the force-ratio domain. No front-tire
nonsaturation restriction is introduced; Fiala saturation is part of the model.

## Collision geometry and observer information

The target follows constant tangential acceleration and sideslip within each
prediction; braking ends at rest. Each current target observation refreshes its
forecast center and parameters while retaining the input warm start. Complete
current perception with no target clears the encounter; otherwise a missing
observation retains the last forecast. Extrapolation is not a new certificate.

Ordinary rectangle distance supplies feasible separating dual multipliers. Four
fixed-multiplier corner rows include the yaw derivative. At overlap a nonzero
support direction replaces the zero ordinary-distance dual. An encounter-side
choice and smooth longitudinal/lateral support combination supply a consistent
escape derivative. A compatible closest-feature normal replaces it after
separation. Predominantly longitudinal encounters retain the branch across
frames; crossing targets refresh their closest geometry. These are local
convexification choices and do not establish global feasibility.

The observer publishes a relative position ball, course bound, and intervals for
speed, constant acceleration and curvature. Absolute common ego pose cancels
only from pairwise collision geometry; road and state bounds retain ego
uncertainty. Directional supports preserve generator correlation, and rectangle
orientation contributes the chord reserve `2*reach*sin(min(pi,b_yaw)/2)`.
Target support coefficients are calculated in a batch once per frame, then
reused for the separating normals. Their parameter-family formulas are unchanged.

The implementation restarts the present ego enclosure `G0` at each future hold
and propagates it through that hold: `G(t_i+tau)=Phi_i(tau)*G0`. The target set
similarly starts at its predicted state at each hold. These are affine images
and an assumed future posterior width. They do not certify nonlinear remainders,
future observation histories or shift-consistent joint reachable sets. No
uncertainty is removed to obtain a command. The first hold also lacks a full
nonlinear remainder certificate. `robustNonlinearSafetyCertified` remains false.

Terminal rows use the mild braking-continuation set in
[TERMINAL_SAFE_SET.md](TERMINAL_SAFE_SET.md): braking to rest at fixed sideslip
and curvature, road and target clearance along the continuation, and parked
clearance against the remaining target track. Motion at the endpoint is allowed;
the stop need not fall inside the prediction horizon. These conditions are
linearized once at the reference endpoint. Ego position/yaw margins are retained;
the target enters as its estimate. This is not full robust terminal membership.

## Computation budget and diagnostics

Defaults target a 50 ms sample: `solver.timeLimitSeconds=0.05`, at most 40 native
interior-point iterations per stage, and a 20 ms completion reserve. Preprocessing
counts against the same timer as model construction and both solves. Reference
rollout, tangent/constraint construction, CLF construction, terminal geometry and
optimized-input prediction check the timer. A native termination callback includes
MEX validation and problem construction, in addition to Clarabel's iteration
timer. Native infeasibility certificates and unfinished iterates are never inputs.

These checks bound the admitted work, not a hard wall-clock guarantee on a desktop
MATLAB process. A running factorization, allocation, initial JIT compilation or OS
preemption cannot be interrupted by the checks. Warm up before timed execution;
`warmUp` uses a separate startup budget for cruise and bounded low-speed
feedback with curved targets, and discards every probe command.
The integrated scenario driver warms after observer initialization, including
one discarded bounded-input startup probe. It does not reuse that probe's
solution, consume the first observation or advance its random stream. These
startup costs are reported separately; first-measurement-to-actuation latency
is not established by this benchmark. Observer publication also avoids repeatedly
reordering MATLAB's search path. Actual
successful and failed controller call times must both be measured from the
same controller timer. Observer/publication preparation belongs to the
separate pipeline timer; failed pipeline latency is not solver latency. A timeout is an allowed
failure, not a successful real-time control result. Raising the budget for a
functional test does not change the one-step RTI algorithm.

`search.rtiIterations` and `linearizationCount` are at most one; `solverCalls` is
at most two. `convexSolveCompleted` records completion of both convex objectives.
`optimizationConverged` is false because nonlinear convergence is not tested.
`nonlinearValidationPerformed` is false and `nonlinearHardResidual` is unavailable.
The reported PCBF and CLF quantities belong to the affine subproblem;
`predictedState` is a nonlinear prediction of the unchanged optimized inputs.
Independent scenario replay measures actual clearance, road margins and recovery.

Controller-state schema 81 rejects old numerical state schemas. The ideal
recursive-feasibility theorem, its model-mismatch bookkeeping and causal policy
shift are unchanged. Neither RTI success nor its time limit establishes the
ideal theorem's hypotheses for the executable controller.
