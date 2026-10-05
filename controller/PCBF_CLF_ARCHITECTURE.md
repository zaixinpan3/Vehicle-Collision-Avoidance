# PCBF slack and one-step CLF optimization to a separating terminal set

Every frame uses the same target-independent analytic quadratic CLF, defined in
[NOMINAL_CLF.md](NOMINAL_CLF.md). Every frame minimizes PCBF slack, then CLF
slack, on one affine model. A bounded input-increment tie term regularizes the
future plan. There is no third optimization stage, target-range CLF switch,
direct nominal-feedback command, inherited slack budget or executable backup.
A primary problem infeasible inside the trust region is re-solved on the same
linearization with the trust scale doubled until feasible (at most
`trustMaximumScale = 1`); a shifted plan still infeasible is then solved from
a fresh potential-field rollout in the same way. Any other failure reports no
solution.

The implementation now consumes timestamped NRMM observer enclosures. It
propagates constant target-parameter sets analytically and ego error boxes
through the shared affine variational model. Their supports tighten collision,
path, physical-state, terminal and first-successor CLF constraints.
Usable current enclosures remain optimization data. Unavailable, stale,
incomplete or misaligned proof metadata does not prevent
optimization around the current point estimates. Its unsupported tightening
is omitted explicitly; this does not assert that the corresponding error is
zero. Inputs without uncertainty metadata use point-estimate prediction.
This is an affine uncertainty integration, **not a certified nonlinear
output-feedback PCBF**. The earlier baseline audit is retained in
[the observer-to-PCBF gap analysis](../report/OBSERVER_ROBUST_PCBF_GAP_20261003.tex).

The completed mathematical design is in
[Observer-to-robust-PCBF theory](OBSERVER_ROBUST_PCBF_THEORY.tex). It derives
current joint uncertainty sets, causal prediction tubes, whole-hold robust
rectangle constraints, a per-sample safety statement with a separating terminal set, and
disturbance-dependent recovery with this same CLF. A vanishing proximal
weight preserves dissipation in one convex solve. These are conditional
theoretical results, with finite checks in
`scripts/verifyObserverRobustConnection.m`. The executable interface realizes
part of that design; nonlinear remainder bounds, a causal output-feedback
prediction policy, posterior-set inclusion and
continuous-hold safety remain unimplemented certification premises.

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
The ego generator propagates as `G_next=A_i*G_i`, for fixed candidate inputs.
This is an open-loop affine image, not a nonlinear tube with future feedback:
no ancillary feedback gains are used, and the shifted plan is rolled out with
its own inputs.

Every current target estimate, with or without an error certificate, updates
the prediction center and its A and beta. These two parameters remain constant
within that frame's prediction and may change at the next observation. The
physical simulated target still obeys one fixed A and beta. Previous inputs
still initialize the next solve. The primary
stage is solved in every frame; no slack budget is carried between frames.
Missing current target
observations retain the last target forecast. Neither this extrapolation nor a revised point estimate
is relabeled a certified enclosure.

The terminal rows subtract the ego generator support; the target enters
the terminal rows as its estimate (see Terminal constraints). Uncertainty is not
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

## Prediction and initialization

The Fiala bicycle state is `x = [px; py; yaw; vx; vy; yawRate]` and its input is
`u = [steering; brakingRatio]`. RK4 defines the prediction map and its tangents.
The default uses one 50-ms RK4 step; its midpoint collision node is an endpoint
interpolant, not an independently integrated half hold.

The prediction length is not fixed. The anchor rollout stops at the first node
`M` with `N <= M <= maximumHorizonSteps` whose separating speed is at least
`terminalSeparatingMarginMetersPerSecond` (0.5 m/s); without a target `M = N`.
The terminal row itself requires only zero. A terminal row active at the anchor
had stalled the CLF stage in Clarabel in an earlier version, hence the margin. A rollout that never reaches it uses
`M = maximumHorizonSteps`, and the optimization then decides feasibility.

Previous inputs are shifted and rolled out from the current measurement in
both encounter and recovery frames, then extended by path guidance. Prior
affine states are not reused as the linearization trajectory. Only at startup
is one moving-target potential-field rollout constructed; when no target is
present, nominal path guidance supplies it. A shifted plan that cannot be
rolled out (non-finite inputs, a braking ratio at the limit, or a tire-domain
error) is reported as no solution; no other anchor replaces it. Potential guidance is a search reference, not a safety
certificate. No maneuver bank is used.

The target keeps Sharma et al. (2026), Eq. (17)'s constant tangential
acceleration `A` and constant sideslip `beta`. The prediction uses inertial
coordinates, with `V(t)=V0+A*t` and curvature `sin(beta)/lr`; yaw rate therefore
changes with speed. There is no added sideslip-rate state. The exact arc-length
prediction supplies both initialization and collision constraints at matching
absolute times. The existing signed-velocity continuation is retained; a stop
clamp would change the constant-acceleration model.

The initialization borrows Zhai et al. (2024), IEEE Access,
DOI 10.1109/ACCESS.2024.3355952: direction-dependent obstacle influence,
motion-dependent repulsion, and joint course/speed guidance. It is an adaptation,
not a reproduction of their printed potential equations, tracked-vehicle model,
Bezier path, or eight behavioral priorities. The local-paper assessment in
[Zhai initialization assessment](../report/FLOW_INITIALIZATION_ZHAI_20261002.tex)
records the formula issues and the need for longitudinal planning.

At each seed hold, project the actual nonlinear ego state onto the given path.
Preview station using its measured tangent velocity, and preview lateral return
as `d(tau) = d(0)*exp(-tau/T)`, with `T = lookaheadSeconds`. Compare these positions
with the target at the same absolute times. With rectangle supports `a_j,b_j`
in the local road axes, define

    r_j^2 = (Delta_s_j/a_j)^2 + (Delta_d_j/b_j)^2,
    w_j = exp(-r_j^2/2 - tau_j/(2*T)),  w = max_j w_j.

The supports include both vehicles' dimensions, the target orientation, body
center offsets, and the seed padding. They are a directional guidance envelope,
not an exact collision constraint. A held passing sign is selected once per
seed from the closest predicted encounter and transverse target velocity. A
bounded circulation component resolves symmetric head-on geometry. The field is

    v_d = -d/T + 0.7*v_ref*w*(Delta_d_star/b_star + 2*c),
    v_s = max(1.5, 0.5*v_ref, v_ref*(1-0.35*w*min(2,closing/v_ref))),
    heading_d = heading_path + atan2(v_d,v_s).

Here `c = -side*tangent'*(p_ego-p_target)/max(norm(p_ego-p_target),1)`.
The star denotes the largest predicted influence. This combines attraction,
a normalized lateral repulsion and circulation; it is not claimed to be the
full gradient of one scalar potential. There is no division by closing speed.
Both acceleration and sideslip affect the target preview through the exact prediction.
The Gaussian now measures predicted interaction; no fitted Gaussian displacement
or fixed nominal-speed station clock is used.

Course guidance produces a yaw-rate demand. Inverse Fiala force feedback and a
speed loop produce controls, which are rolled out through the same nonlinear
bicycle RK4 map. Seed-only friction and braking shaping keep this reference
within the useful tire region; they do not add steering magnitude or slew limits
to the optimization. A seed may still violate collision or terminal constraints
and is never executable by itself.

**Normal frames shift the previous input plan.** The field rollout is
constructed at startup, and once more when the primary problem around the
shifted plan is primal infeasible inside the trust region. Each trajectory is
linearized once. There is no SCP iteration on
an optimized nonlinear rollout within the same hold.

For an anchor `(xbar_i, ubar_i)` the shared model is

    x_(i+1) = f_h(xbar_i,ubar_i) + A_i (x_i-xbar_i) + B_i (u_i-ubar_i).

The same anchor supplies dynamics, tire derivatives, collision directions,
the terminal rows and the analytic first-successor CLF approximation. Input/state correction
boxes are numerical iteration bounds. At unit trust scale, the steering
correction is +/-0.075 rad and the braking-ratio correction +/-0.125. The
trust scale is estimated from the plan innovation (below).
Steering has no physical
magnitude or slew bound. Physical braking/model-domain bounds remain. These
numerical boxes are not certified nonlinear remainder bounds.

## Ordinary distance dual and fixed collision multipliers

Collision rows follow the distance-dual / fixed-multiplier sequence of
Li et al. (2023), [Section 3, equations (12)--(13)](https://doi.org/10.1007/s42154-023-00222-7).
Only the collision formulation is adopted; the PCBF/CLF objectives and
Fiala dynamics remain the project model.

For target body inequalities `A (z-pT) <= b` and the four ego vertices
`v_j(x)`, the finite-body meaning of the paper's set `S(x)` is made explicit:

    maximize alpha - b' lambda
    subject to alpha <= lambda' A (v_j(xbar)-pT), j=1,...,4,
               lambda >= 0, norm(A' lambda,2) <= 1.

This is the ordinary nonnegative set distance dual: the minimum over a
rectangle is attained at a vertex, and `alpha` is just its hypograph variable.
For rectangles its optimum is now constructed from the closest-feature
normal `n` supplied by the exact rectangle geometry. If `w = R_T' n`, then
`lambda = [max(w,0); max(-w,0)]`. The normal has unit norm for disjoint
rectangles and provides a supporting hyperplane at a closest feature, so
these nonnegative multipliers attain the same ordinary-distance optimum.
Overlap (and the implemented touching convention) gives `n = 0` and
`lambda = 0`. No signed penetration objective is substituted. The resulting
`lambda` is fixed when assembling all four trajectory inequalities

    lambda' [A (v_j(x)-pT) - b] + s_i >= safetyMarginMeters.

The existing affine RTI model retains the first derivative of the rotated
vertices with respect to ego yaw. Thus the paper's unspecified set notation
is explicitly represented by both full rectangles, rather than replacing
the ego by a point or silently freezing its orientation. This representation
and the yaw tangent are stated here; they are not a claim to reproduce an
uninspected author implementation. At the anchor, the optimized minimum
equals the ordinary rectangle distance up to solver tolerance.

There is no signed-distance objective, preferred passing direction, normal
normalization, or replacement of an overlapping distance dual. Zero
multipliers stay zero: a relaxed row then needs at least the node buffer in
slack, while a hard positive-buffer row is infeasible. Contact duals may be
nonunique. Potential-field initialization does not replace those multipliers or repair
their zero gradients. The retired signed-direction MEX is neither called nor built.
The geometric construction retains the nonnegative-multiplier, unit-normal
and primal-dual identities as numerical assertions. It removes the many
five-variable `coneprog` calls and their near-optimal stopping failures;
it does not relax a trajectory constraint. The distant recovery geometry
that previously aborted with a 1.68-micrometer gap now has a gap of order
1e-14 m. Both the overlap plateau and nonunique touching normals remain
properties of the ordinary-distance formulation.

The trajectory stages share the remaining soft frame budget. Formulation
and a currently running factorization are not interrupted by a hard deadline.
The original conic distance implementation and its historical experiment
results remain recorded in the dated reports; they do not describe the
current geometric implementation.

## PCBF slack and CLF dissipation

One nonnegative collision slack is assigned to each primary-horizon stage;
it relaxes the start and midpoint separation rows. Completion-tail rows are
hard. The node buffer is 0.20 m, whereas the experiment's physical pass criterion
is positive rectangle clearance. The road is the only lateral constraint: every
ego rectangle corner stays within `lateralClearance` at every predicted node,
hold midpoint and the endpoint, hard in both stages (see Terminal constraints).
The former 10-m center corridor around the path is removed.

The first problem minimizes the sum of PCBF slacks. The unbounded CLF slack
and its cone are eliminated from this problem without changing its feasible
projection. If the zero correction with zero slacks satisfies every linear
row, equality, bound and path cone, it attains the global lower bound zero.
The primary stage is then completed analytically; the CLF stage still runs.
`numericalSolve` distinguishes analytic completion from an actual solver call. The primary therefore contains only the
shared dynamics, state/input limits, collision, path and terminal rows.

A finite positive primary value need not be a useful safety result. The fixed
current-state rows supply the unavoidable lower bound

    J_lower = max(0, -min(g(x_current))).

A primary value above this lower bound is issued as positive slack. When the
solver certifies a primary problem primal infeasible, the input trust scale is
doubled on the same linearization, never beyond `trustMaximumScale = 1`, until
the problem is feasible. An earlier cap of 16 produced feasible problems on an
inaccurate linearization: model innovation rose to 1.7 m and issued inputs
reached their physical limits. If the shifted plan is still infeasible, one fresh
potential-field model is solved with the same expansion, starting again from
the estimated scale. The expansion does not change the online trust estimate.
Positive slack, numerical stalls and a CLF stage without a result are reported,
not re-solved. There is no second fresh model. A finite array from a failed
primary solve is not a valid PCBF optimum.

On the selected model the single CLF requirement is

    V(x_next) <= exp(-2 h / T) * V(x) + rho,    rho >= 0,
    V(x) = e(x)' P e(x),

with T = `clf.convergenceTimeConstantSeconds` = 4 s. P is a quadratic CLF
synthesized offline by an LMI for the given-path cruise trim and read from
`config/clfMatrices.json` (see [NOMINAL_CLF.md](NOMINAL_CLF.md)); there is no
LQR design. The analytic
error Jacobian applied to the affine first successor gives a convex quadratic
with Hessian J' P J. One second-order cone represents its epigraph. There are
no cost-to-go rollouts or finite-difference curvature calculations. This
quadratic is locally justified, not a global nonlinear CLF; positive slack can
remain necessary away from cruise even without a target. The secondary minimizes

    (rho + epsilon * lossLower * R(dU)) / max(V(x),1),
    sum(xi) <= achievedPrimary + lexicographicTieTolerance * max(1, achievedPrimary),

with `lexicographicTieTolerance = 1e-4` m. With a 1e-5 cap the CLF stage often
stalled in Clarabel (InsufficientProgress, primal residual 1e-5 to 6e-5) because
the zero-slack set is a slab only micrometres thick; 0.1 mm restored progress.

`R` is the normalized squared input increment about the linearization inputs.
At an exact optimum its effect on minimum physical CLF slack is bounded by
`epsilon * lossLower`, where `epsilon = clfTieTolerance = 1e-4` and
`lossLower` bounds `e'Qe` from below over the current affine error set.
The tie vanishes at nominal behavior. It penalizes neither absolute zero input
nor deviation from the construction feedback. PCBF priority is retained;
CLF minimization has this explicit bounded tie allowance. A zero primary
result still runs the CLF stage. The componentwise input boxes imply `R <= 1`,
so the former epigraph `R <= sigma <= 1` is eliminated exactly. Clarabel solves
the native sparse quadratic objective with the path and CLF cones retained;
these are conic QPs rather than linearly constrained QPs. State variables remain
explicit to preserve dynamic sparsity.

The compiled adapter seeks the configured constraint/optimality precision.
Its reduced-accuracy termination uses the existing `feasibilityTolerance`
(default 1e-5) for primal/dual residuals and absolute/relative objective gap.
Flags 1 and 2 distinguish full and reduced-accuracy convergence. Numerical
acceptance error is additional to the analytic lexicographic tie bounds.
Infeasibility rays, time-limit iterates and unresolved numerical failures return
no point. `solverInfo` records native status, iterations and residuals.

## Issuing the solved plan

The completed PCBF/CLF result is issued directly. Its inputs are not replayed
through the nonlinear model, and no pose, state, CLF or path-deviation agreement
test, damped step or line search follows the solve. The road rows hold for the
affine prediction only.

Each frame builds one trajectory model, or two when the shifted problem is
re-solved from the fresh rollout, with at most two numerical solves per model. The input trust scale follows the plan innovation (next
section). This local numerical step bound is not an actuator constraint. No
global SQP convergence theorem or hard execution deadline follows from this
finite work budget.

A frame whose anchor is unavailable, whose CLF stage returns no result, or
whose fresh re-solve still has no result, reports `collisionAvoidanceController:noOptimizationSolution`.
No enlarged box, inherited budget, previous-frame plan or incomplete numerical
point is used instead.

## Input trust scale from the plan innovation

The correction box `+/-Delta*r*[0.15;0.25]` (`r = trustRadius`) bounds how far
one solve moves the inputs from its anchor. `Delta` is estimated, not set.

For a correction of size `s` (largest input change from the anchor, in box
units), the affine prediction differs from the nonlinear rollout of the same
inputs by a second-order remainder, `e ~ L*s^2`. The coefficient `L` depends
on the operating point (speed, tire slip) and is not known in advance.

Every new posterior measures it. Let `xhat_1..N` be the previous plan's affine
prediction of nodes 2..N+1 and `xbar_1..N` the shifted rollout of the same inputs
from the new posterior (the next anchor). A second nonlinear replay `xtil` of
the same inputs, started at the plan's own prediction `xtil_1 = xhat_1`, splits
the plan innovation exactly at every node:

    xbar - xhat = (xbar - xtil) + (xtil - xhat).

The first term propagates the posterior's departure from the prediction
(estimation error, plant mismatch and the first hold's remainder); it belongs to
the observer. The second contains no posterior: it is the model remainder of the
step that was taken. With the safety body-point metric
`P(dx) = norm(dp) + r_E * abs(dpsi)`, the model innovation is

    e_k = max_j P(xtil_j - xhat_j).

Only `e_k` sets the trust scale; the total and observation parts are recorded.
An earlier variant subtracted twice the observer's worst-case tube radius from
the total innovation instead. On the noisy-estimator campaign that bound
explained every innovation (0.27 m included), so the scale rose to its maximum
and the encounters became infeasible within 10 to 70 holds; the exact split
replaced it.

The coefficient estimate uses fast attack and slow release, and the scale is
the step whose predicted remainder equals the target innovation `tau`:

    Lhat_k  = max( e_k / max(s_(k-1), Delta_(k-1)/2)^2 , gamma * Lhat_(k-1) ),
    Delta_k = clip( sqrt(tau / Lhat_k), Delta_min, Delta_max ).

`Delta` grows by at most `1/sqrt(gamma)` per sample and shrinks at once. A step
far inside the box (`s < Delta/2`) is not taken as evidence about the box
boundary. Startup uses `Delta_0` with `Lhat_0 = tau/Delta_0^2`. A frame without
a shifted rollout keeps the previous estimate. The defaults are `tau = 0.02 m`,
`Delta_0 = 0.125`, `Delta` in `[1/16, 1]` and `gamma = 0.5`.

On the fourteen exact-observation encounters (2026-10-04), the measured
`e/s^2` was about 0.07-0.09 m at 8 m/s and 0.25-0.30 m at 15 m/s, almost
independent of a fixed `Delta` between 0.125 and 0.5. Runs whose innovation
stayed below about 3 cm needed few fresh restarts at both speeds. One `tau`
therefore yields about 0.4-0.5 at 8 m/s and 0.2-0.27 at 15 m/s, without a
speed schedule.

This update is not a solution check: every completed solve is issued, and the
innovation only sets the next step bound. `tau` is an empirical consistency
target, not a certified bound on the nonlinear remainder or on separation.

## Terminal constraints

The terminal set requires only that the ego separates from the target and stays
on the road. At the last node `M`, with ego position `p`, yaw `psi`, body
velocity `v` and predicted target position `q` and velocity `w`:

    (p - q)' (Rot(psi) v - w) >= 0,                 (separating)
    every ego rectangle corner within lateralClearance = [right; left] of the path
    (imposed at every predicted node, not only the last one).

If the relative velocity stays constant after `M`, the squared distance has
derivative `2 (p - q)' v_rel >= 0` and second derivative `2 norm(v_rel)^2 >= 0`,
so the distance never decreases again; the clearance at the endpoint then bounds
the clearance afterwards. Because the endpoint may be close to the target, it
also carries the collision rows of every other node inside `R`. An earlier
version also required the target beyond `R` (50 m); following a braking lead
never reached that set within 512 nodes, and the noisy brakingLead frames were
infeasible only because of it.

The separating row is linearized at the anchor endpoint in position, yaw and
body velocity and divided by the anchor distance. The target enters as its
estimate, and its uncertainty is carried by the collision rows inside `R`; the
ego generator support along the row is subtracted.

The road rows are the same at every node: the ego drives on the road, so every
predicted node after the measured one and every hold midpoint keeps all four
rectangle corners within `lateralClearance`, and the endpoint is one of these
nodes. Each row linearizes one corner's lateral coordinate on the path normal
at the anchor corner, with the ego generator support subtracted. An earlier
version imposed the road rows only at the last node with a separate 10-m center
corridor elsewhere; plans then left the road during avoidance and became
infeasible when the target exit shortened the horizon to 8--29 nodes. Without a
target only the road rows remain; without `lateralClearance` there is no road
constraint. Collision rows apply at nodes inside `R`, including a later re-entry.

The distance argument above holds only for a constant relative velocity. A
braking lead, a target whose constant sideslip turns it back, or the ego's own
return to the path can close the distance after `M`; the next frame's horizon
then extends again. No invariance or recursive-feasibility argument for this
terminal set is claimed.
The terminal quantities of the issued affine endpoint are recorded as
`terminalDistanceMeters`, `terminalSeparatingSpeed` and
`terminalRoadMarginMeters`.

## Issued commands, diagnostics and limits

The selected CLF optimizer result supplies the applied input. Its applied
input is not clipped. A nonpositive solver flag cannot supply an input. Positive
numerical termination is required; it is not a continuous-time safety
certificate. If no completed CLF result exists,
the controller reports `collisionAvoidanceController:noOptimizationSolution`.
No previous trajectory or primary-only point is issued instead.

`primaryOptimum` records the achieved primary value; without positive solver
termination it is not a certified optimum. Attempt logs retain all primary
models and timings; `selectedAttempt` identifies the model supplying the
command. `optimizationConverged` describes the numerical stages of the selected
problem, not optimality of the issued point under nonlinear dynamics.
Continuation state version 73 stores the per-stage slacks and their total
alongside the existing single-CLF trajectory state, and the trust estimate
(`scale`, coefficient `curvature`, last `correction`).

Metadata retains the explicit scope:

- `predictionModel = affineFialaRk4Linearization`;
- `safetyScope = affineSampledConstraints`;
- `affineValidationPerformed = false`;
- `nonlinearValidationPerformed = false` (no complete nonlinear-constraint certificate);
- `nonlinearPredictionEvaluated = false` (the issued plan is not replayed online);
- `trust` (scale, curvature, correction, innovation, observationInnovation, modelInnovation, updated);
- `recursiveFeasibilityScope = notCertifiedForNonlinearPlant`;
- unmeasured constraint residuals and margins are NaN;
- `clfFunction = quadraticTransverseError`, with no tertiary objective;
- required decrease, modeled successor value and bounded CLF tie allowance are recorded.

Positive PCBF slack is relaxation, not collision freedom. Zero modeled CLF
slack alone is not a nonlinear decrease theorem: the local RTI value model is
an approximation. The local quadratic construction and its remaining proof obligations are stated in NOMINAL_CLF.md.
Independent ODE45 replay measures actual clearance and recovery offline;
these dense ODE45 safety/recovery measurements remain offline. Online
nonlinear RK4 expansion evaluates approximation accuracy, not a full
physical-vehicle safety certificate.

This is a bounded-work RTI implementation. It does not automatically inherit
nonlinear stability or recursive-feasibility guarantees. For the general RTI
method see Diehl, Bock and Schloder,
[Real-Time Iterations for Nonlinear Optimal Feedback Control](https://cdn.syscop.de/publications/Diehl2005c.pdf).
Vehicle-specific results and full controller-call timing belong in the dated
experiment report under `report/`.
