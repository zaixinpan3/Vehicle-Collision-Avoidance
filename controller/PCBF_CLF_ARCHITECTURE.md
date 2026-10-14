# Successive-convexification PCBF/CLF control

The controller directly optimizes a nonlinear vehicle input sequence by repeated
convex approximation. There is no independent avoidance-path planner. A nominal
lane-feedback rollout, or the previous solution's shifted inputs rolled out from
the current estimate, supplies a starting reference. That reference may intersect
the target or fail the terminal conditions.

At each iteration the controller updates the Fiala vehicle tangents, rectangle
separation multipliers, observer-margin approximations and braking-continuation
terminal rows. It solves the safety stage followed by the same target-independent
one-step quadratic CLF stage. An internal elastic variable restores feasibility
of the local state/path constraints; it cannot remain above the configured
residual tolerance in a returned solution. Nonlinear rollout determines which
trial steps the optimizer accepts. The returned actuator sequence is exactly one
completed accepted convex optimization's sequence, without clipping or blending.

This is a local numerical method. Iteration or time limits can prevent a feasible
return, and a returned feasible iterate need not be a stationary or global
optimum. The theoretical recursive-feasibility claim concerns the ideal nonlinear
problem and its causal information-set policies, not numerical solver success.
See [ROBUST_RECURSIVE_FEASIBILITY.md](ROBUST_RECURSIVE_FEASIBILITY.md) and
[OBSERVER_ROBUST_PCBF_THEORY.tex](OBSERVER_ROBUST_PCBF_THEORY.tex).

The nonlinear check applies to sampled nominal constraints with the current
affine observer margins. It does not certify nonlinear error-set propagation,
future posterior inclusion or continuous-time collision avoidance. The existing
observer integration remains as described below; no error radius is removed to
obtain a command.

## Current observer integration

`nrmmControllerErrorBounds` publishes a relative position ball, course bound
and intervals for speed, constant tangential acceleration A and constant
curvature `kappa=sin(beta)/lr`. It intersects the reconstructed intervals with
the declared target domain and recenters acceleration error at the clipped
acceleration value actually published. There is no target-model mismatch
allowance or sideslip-rate state. Every possible future target obeys the
same constant-A, constant-beta contract.

Let `s=V0*t+A0*t^2/2` and `b_s=b_V*|t|+b_A*t^2/2`. The analytic target bounds
implemented in `targetErrorTube` are

    b_p(t) = b_p(0) + b_s + |s|*b_course(0) + s^2*b_kappa/2,
    b_course(t) = min(pi, b_course(0) + |s|*b_kappa + (|kappa0|+b_kappa)*b_s),
    b_yaw(t) = min(pi, b_course(t) + b_beta).

They enclose the fixed-parameter family without predicting future measurement
updates or shrinking the reachable set using future observer decay. Common
absolute ego pose cancels from pairwise distance: the aligned relative target
set and ego intrinsic velocity/rate errors are propagated in the frame frozen
at the current sample. Full ego pose uncertainty remains in path and CLF rows.

For each fixed separating normal, position uncertainty uses directional
support of the propagated ego generators plus the target position radius.
Rectangle orientation uncertainty contributes `2*reach*sin(min(pi,b_yaw)/2)`.

**Information-consistent holds (2026-10-05).** The estimator publishes a fresh
enclosure `G0` of its estimate at every sample, and every hold of the plan is
issued from such an estimate. Hold `i` therefore starts from `G0` and
propagates it through that hold only:

    G(t_i + tau) = Phi_i(tau) G0,   tau in [0, h],

with `Phi_i` the affine hold map at the midpoint and endpoint. Node `i+1`
closes hold `i` and uses `A_i G0`. The common-pose cancellation is taken about
the ego at the start of each hold, where the target is again measured relative
to it. The target set restarts at each hold from the target's predicted state
at `t_i`, for the time elapsed in the hold. The first hold is the certified
one. Later holds assume that the estimator's enclosure at their start equals
the present one; neither the family of future estimates nor the target's
forecast error beyond one hold is covered. The earlier open-loop image
`G_next = A_i G_i` grew with the variable horizon (up to 512 holds). Its
reserves on the hard road, state-limit and terminal rows made 3 of 4 inspected
failing noisy frames infeasible. Each of these was solvable without the ego
enclosure.
No ancillary feedback gains are used, and the shifted plan is rolled out with
its own inputs.

Propagating the target set from the present sample over the whole prediction
instead was measured on October 8, 2026 and not adopted: 31 of 84 noisy
encounters recovered against 55, and one collided, because in the prefix the
tightened and the nominal row of a node share one slack
(`report/TARGET_PARAMETER_SET_20261008.tex`).

Every current target estimate, with or without an error certificate, updates
the prediction center and its A and beta. These two parameters remain constant
within that frame's prediction and may change at the next observation. The
physical simulated target still obeys one fixed A and beta. Previous inputs
still initialize the next solve. The primary
stage is solved in every frame; no slack budget is carried between frames.
Missing current target
observations retain the last target forecast. Neither this extrapolation nor a revised point estimate
is relabeled a certified enclosure.

The terminal rows widen the clearances by the ego enclosure's position radius
and yaw chord; the target enters the terminal rows as its estimate (see
Terminal constraints). Uncertainty is not
dropped to force a solve, and these rows do not establish robust continuation.

### Experimental outcomes versus theorem premises

The controller neither rejects revised forecasts because of an unproved
theorem premise nor produces assumption-status diagnostics. A usable finite
uncertainty constraint that actually makes the conic problem
infeasible is still an optimization failure; it is not waived by this policy.

`runNonlinearPredictiveSafetyValidation` propagates physical target truth from
the scenario's initial state, independently of the controller's revised target
forecast. Its optional `TargetEstimateFunction(targetTruth,egoTruth)` supplies
synthetic observations without changing the plant. Rectangle contact/overlap
in the independent replay, or failure to return a finite solved control,
constitutes experimental failure. A positive clearance below the requested
margin, an unverified theory premise, or no recovery yet at the observation
cap is reported separately. Reaching that cap is not a recovery claim. Replay
or harness errors have their own inconclusive `executionError` outcome.

The ideal conditional theorem is unchanged. A successful run with unverified
premises is empirical robustness evidence, not a proof that those premises
hold or that recursive feasibility has been established.

## Starting reference and prediction horizon

The state is `x = [px; py; yaw; vx; vy; yawRate]`; the input is
`u = [frontWheelSteeringAngle; signedLongitudinalForceRatio]`. The declared model
is the modified-Fiala bicycle at higher speed, a kinematic bicycle at low speed,
and a smooth blend between them. RK4 and variational equations share the same
mesh. With the default single-step mesh, the hold midpoint is the endpoint
interpolant, not a separately integrated half hold.

Without a target the requested horizon is `controller.horizonSteps`. With a
target it covers at least `scp.minimumPredictionSeconds` (three seconds), and
extends through the nearest forecast interaction along prescribed cruise plus
`scp.settlingSeconds` (two seconds), up to `scp.predictionSeconds` (six seconds).
Current geometric enclosures widen the interaction trigger. This selects duration
only: the prescribed cruise contains no avoidance maneuver. A late braking-lead
conflict gets a longer horizon than an immediate head-on. The maximum hold count
still bounds the result. A longer inherited sequence is retained while a target
is present; missing tail inputs come from nominal lane feedback. Seed avoidance
and terminal membership are not prerequisites for constructing a subproblem.
Target onset retains the input warm start and refreshes the geometry branch.
A shifted reference receives at most 60 percent of the frame's wall budget.
If it stalls or fails without an eligible result, the same optimizer can restart
once from nominal feedback using the remaining budget. Terminal and escape
geometry are rebuilt. A restart does not execute an unoptimized nominal input.

Every reference is an exact declared-model rollout. Consequently zero dynamics
increments satisfy its linearized dynamics, even if the reference is unsafe.
No virtual dynamic control is needed for this shooting-reference implementation.
The target retains the current constant tangential acceleration and sideslip
forecast, with braking absorbed at rest and parameter estimates refreshed each
frame.

## Convex subproblem and restoration

For reference `(x_bar, u_bar)` the dynamics increments obey

    dx_0 = 0,
    dx_{i+1} = A_i dx_i + B_i du_i.

State and input increments lie in local trust boxes. Actual input magnitude
bounds remain hard. State, road, sideslip, sampled collision and terminal rows
are relaxed by one shared nonnegative elastic variable `e`; the handling cone's
right-hand side receives the same relaxation. Prefix PCBF slacks `xi_i` retain
their existing meaning. With usable nonzero observer enclosures, tightened
collision deficits remain soft over the horizon and nominal collision rows stay
hard after the prefix. The elastic variable is separate from these PCBF deficits.

The first stage minimizes

    J_s = sum_i xi_i + scp.restorationWeight * e.

The second stage caps `J_s` at the computed first-stage optimum plus the declared
numerical tie, then minimizes one-step CLF slack plus a bounded combination of
whole-horizon nominal tracking and input-increment proximity. The CLF remains `V(x) = error(x)' P error(x)` with

    V(x_next) <= exp(-2*h/T) V(x_now) + rho.

Only the estimate enters this performance row. The tie has a minimum
loss scale of one, including at trim. Half its bound goes to mean predicted
nominal CLF value, normalized by its maximum over the local state box, and half
to input-increment proximity. The sum cannot exceed `clfTieBound/clfScale` on
the box. This distinguishes future passing/recovery from following a slow lead
while controlling otherwise free future inputs. It does not generate another avoidance route. The exact optimum of
each convex subproblem is distinct from the nonlinear problem's optimum.

## Collision separation updates

Rectangle distance supplies target-to-ego normals and feasible nonnegative dual
multipliers. Four fixed-multiplier rows account for all ego corners, including
the yaw derivative. Those multipliers are rebuilt after every accepted nonlinear
rollout. At overlap the ordinary distance dual is zero; the least-penetrated
rectangle axis instead supplies a feasible nonzero dual and an escape derivative.
At symmetric lateral overlap, a small signed projection cannot let roundoff pick
opposite sides at adjacent prediction nodes: orientation is tied to the ego
lateral axis. This tie is invariant under a common rotation of the scene and
chooses a local geometry branch, not a precomputed maneuver.

One side is selected for each encounter, away from a target approaching from
one side of the road, or using the wider available side for an aligned target.
At an intersecting reference, longitudinal and that lateral support direction
are smoothly combined using their signed geometric gaps. Near longitudinal
alignment the lateral direction dominates. This avoids both a zero escape
derivative and forcing pure lateral separation too early. Once separated,
a compatible closest-feature normal supplies the exact ordinary-distance dual.

For targets mainly traveling along the road, an active escape branch survives
relinearization and shifted frames until the reference passes the target.
Crossing targets use refreshed closest geometry so that the vehicle can yield
without chasing a target laterally across the road. This changes geometric
linearization, not the control law or objective hierarchy. Target onset and
absence clear the geometry memory. These local branch choices do not establish
global optimality.

The local constraints are affine approximations of rotated-corner support rows.
Positive sampled separation does not establish a continuous-hold or robust
nonlinear safety certificate. Positive PCBF slack explicitly denotes relaxation.

## Complete handling-product derivative

Rear adhesion uses `w = vy - lr*yawRate`,
`k = 3*rearLongitudinalForceScale/rearCorneringStiffness`, and

    norm([w/k; vx*b]) <= vx.

The bilinear product must include both first derivatives:

    vx*b ≈ vx_bar*b_bar + b_bar*dvx + vx_bar*db.

The cone is normalized by `vx_bar`. Omitting `b_bar*dvx` gives an inconsistent
first-order model when braking and speed change together; reducing the trust
region alone cannot repair that missing derivative. Below the declared
kinematic threshold the cone reduces to `abs(b) <= 1`. The node sideslip bound
and sampled physical state limits are rebuilt on each reference.

Front slip remains inside the Fiala model's angle domain:
`abs(atan2(vy + lf*yawRate, vx) - steering) < pi/2`.
Its rows use the angle derivative and the current body's interval enclosure
when forward velocity remains positive over that enclosure. They apply at hold
ends and midpoints; nonlinear trials outside the angle domain are unavailable.
Front-wheel force saturation is allowed: the Fiala model already limits force,
and steering still changes its direction in the body frame. No extra front
nonsaturation or steering actuator cap is imposed. Force-ratio bounds include a
small numerical reserve inside the model domain, rather than clipping a returned
command.

## Nonlinear merit and trust updates

A trial is rolled out with its unchanged optimized inputs. The controller
rebuilds the constraint approximation at that rollout and evaluates the zero-step
hard residual `H`, the PCBF slacks required there, and the true one-step CLF
deficit. Its merit is

    M = restorationWeight*max(0,H-constraintTolerance)
        + sum(required PCBF slacks)
        + performanceWeight*true CLF deficit/max(1,V_now)
        + bounded horizon-tracking and input-proximity terms.

Predicted reduction uses the corresponding convex elastic excess above tolerance,
PCBF slacks, scaled CLF slack and bounded tie terms. Reference and trial tracking
costs use the same local normalization for that ratio. Hard residuals within
the declared numerical tolerance are not penalized as infeasible. The ratio of actual to predicted reduction decides whether
to accept a step and enlarge or shrink its input trust region. A rejected step
keeps the reference and rebuilds the input bounds for a smaller trial. Twelve
iterations without material restoration progress, or a rejected step at the
minimum trust scale, terminate an unsuccessful reference early. The optional
nominal restart shares the original frame deadline.

If a convexly feasible step leaves nonlinear hard residual above tolerance, the
next trust scale is reduced using that measured residual even if total merit
improved. The intra-solve scale may fall below the across-frame trust estimator's
minimum: `scp.minimumTrustScale` avoids the old fixed correction floor preventing
boundary refinement. All accepted references are nonlinear rollouts.

An accepted completed trial is eligible for return only if both its elastic
variable and its reevaluated hard residual are at most
`scp.constraintTolerance` (default `1e-3` in each row's units; handling cones
are normalized). The loop keeps the latest eligible trial while refining.
`scp.feasibleRefinementIterations` limits eligible refinements to three by
default; restoration can use up to 80 iterations per reference within the shared wall budget.
The reported termination distinguishes this finite refinement limit from the
small interior-step convergence criterion. Reaching an iteration/time limit can
return an eligible trial, with
`optimizationConverged = false`. If none exists, it reports no optimization
solution. There is no alternate controller or previous-plan execution fallback.
The first input of the selected completed optimization is issued directly.
Nonlinear merit checks are iterations inside the optimizer, not a separate
post-solve actuator modification.

The across-frame trust estimate still records full-step affine/nonlinear pose
error and distinguishes it from departure of the new estimate from the previous
prediction. This estimate is a numerical heuristic, not a proved remainder bound.

## Braking-continuation terminal conditions

The terminal set remains the mild-state braking-continuation set in
[TERMINAL_SAFE_SET.md](TERMINAL_SAFE_SET.md). It allows motion: the endpoint
must admit the declared kinematic law that brakes to rest with fixed sideslip
and course curvature. Continuation nodes and midpoints stay on the optional
road and clear the target until its exit; the parked pose clears the remaining
target track. The actual stop need not occur inside the prediction horizon.

These terminal rows, including continuation and parked-witness geometry, are
relinearized at each accepted endpoint. Their nominal kinematic modeling
assumption and the observer-set certification limits are unchanged. Numerical
SCP feasibility must not be substituted for the exact-set terminal-invariance
premises of the theoretical proof.

At an unsafe parked pose, an arbitrary first contact on the target track
cannot provide a useful longitudinal escape derivative: the contact point moves
with the pose. Straight-track restoration uses a transverse support direction.
For a circular track, inner-hole corner radii or an outer support of the complete
remaining arc provide the restoration rows, with one local radial branch kept
within the solve. Target rectangle vertex radii and arc support extrema are
computed directly; an arbitrary contact witness cannot switch the gradient.
These are sufficient local restoration conditions. Exact terminal membership
and the declared terminal law are unchanged.

## Diagnostics and validation

`search.iterations` records accepted/rejected steps, elastic slack, reevaluated
hard residual, merit ratio and trust scale. `scpIterations`,
`scpRestorationSlack`, `nonlinearHardResidual` and `realizedSafetyValue` expose
the selected solution's scope. `search.stopReason` distinguishes small interior-step termination, finite
feasible refinement, stalled restoration, iteration limit and time limit.
`search.nominalRestarted` records the one permitted initialization restart. The reported primary/CLF quantities belong to
the selected convex solve; realized quantities belong to its nonlinear rollout.
`robustNonlinearSafetyCertified` remains false.

`twoStagePredictiveControlTest` checks restoration from intersecting nominal
references, infeasible/budget-limited failure, lexicographic priority, unmodified
optimized input return, model-successor continuation, actuator bounds, handling,
road geometry and nonlinear rollout consistency. `ordinaryDistanceDualTest`
checks symmetric-overlap orientation as well as ordinary dual and corner
geometry. Scenario drivers independently replay issued holds using `ode45`,
fixed target truth, optional noisy NRMM feedback and nominal-recovery dwell.
Routine campaign outputs can remain in a temporary directory.

The design uses the trust-region and feasibility-restoration ideas of
[Mao, Szmuk and Açıkmeşe (2016)](https://arxiv.org/abs/1608.05133) and
[Mao et al. (2017)](https://arxiv.org/abs/1701.00558). This implementation uses
dynamically consistent rollout references and path-row elastic relaxation;
it does not claim those papers' convergence theorems apply to its lexicographic
PCBF/CLF formulation or to its approximate observer margins.
