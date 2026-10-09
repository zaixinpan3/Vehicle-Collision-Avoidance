# PCBF slack and one-step CLF optimization to a CLF-tube terminal set

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

The horizon ends in the terminal set of
[TERMINAL_SAFE_SET.md](TERMINAL_SAFE_SET.md): the CLF tube of the endpoint, the
region every trajectory of the terminal controller (this problem without PCBF
rows) stays in, misses the target until the encounter ends: the target leaves
the perception range, or the target's forecast motion relative to the tube's
box is outside their collision cone (the two never come within the collision
buffer again). No encounter duration is preset, and no lane or other mode is
used. The issued plan is the nonlinear rollout of an accepted step of the
affine solution (*Issuing the solved plan*). On the declared sampled model with
exact information the previous plan, shifted by one hold, is always an
acceptable step, which makes the problem recursively feasible within an
encounter.

The implementation now consumes timestamped NRMM observer enclosures. Every hold
of the plan starts from the current enclosure and propagates it through that
hold only: the target's constant-parameter set analytically, and the ego error
box through the shared affine variational model (see *Current observer
integration*). Their supports tighten collision, road, physical-state,
stable-handling and terminal constraints. The CLF row acts on the estimate.
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

The prediction length is not fixed. At startup the potential-field rollout
stops at the first node `M` with `N <= M <= maximumHorizonSteps` that lies in
the terminal set; a rollout that never reaches it uses
`M = maximumHorizonSteps`, and the optimization then decides feasibility.

Later frames shift the previous accepted plan by one hold and roll its inputs
out from the current estimate. The plan keeps its endpoint at the same
absolute time, so the horizon shrinks by one hold per frame. Only when the
shifted plan is shorter than `N`, or its endpoint is no longer in the terminal
set (a changed forecast or state), is it extended by holds of the terminal
controller (`terminalSafeSet.terminalInput`). Each such hold is an admissible
input whose successor meets the CLF without slack. Prior affine states are
not reused. When no target is present, nominal path guidance supplies the
startup rollout. A shifted plan that cannot be rolled out (non-finite inputs,
a braking ratio at the limit, or a tire-domain error) is reported as no
solution; no other anchor replaces it. Potential guidance is a search
reference, not a safety certificate. No maneuver bank is used.

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

The supports include both vehicles' dimensions along the path and its normal,
including the ego's own heading relative to the path, the target orientation,
body center offsets, and the seed padding. They are a directional guidance
envelope, not an exact collision constraint. A held passing sign is selected
once per seed from the closest predicted encounter and transverse target
velocity. With a road it also compares the room beside the target: clearing the
target on the left needs `b - Delta_d` of lateral motion and on the right
`b + Delta_d`, so half the difference of the room left and right of the target
is added to the preference, and the side that keeps more road margin after
clearing wins. Before this term a head-on in the lane was decided by the sign
of the estimated target sideslip (noise of +/-0.03 rad); in four noisy runs it
chose the narrower right side and the swerve then ran toward the road edge.
A bounded circulation component resolves symmetric head-on geometry. The field is

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
linearized once; a second linearization happens only when no step of the
solution is acceptable on the nonlinear model (*Issuing the solved plan*).

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
`lambda = 0` for the ordinary-distance optimum. The resulting
`lambda` is fixed when assembling all four trajectory inequalities

    lambda' [A (v_j(x)-pT) - b] + s_i >= safetyMarginMeters.

At overlap the trajectory rows do not use the zero multiplier, whose row has
no gradient in the ego pose. `supportLinearization` selects another feasible
dual instead: the unit normal of the rectangle axis (of the four axes of both
bodies) with the least penetration, oriented from the target to the ego. Any
feasible multiplier gives a lower bound of the distance, so the row remains a
sufficient separation condition; its value is negative at the anchor and its
gradient points along that axis. This is not a signed-distance objective: the
multiplier is fixed, and the row still needs slack until the bodies separate.

The existing affine RTI model retains the first derivative of the rotated
vertices with respect to ego yaw. Thus the paper's unspecified set notation
is explicitly represented by both full rectangles, rather than replacing
the ego by a point or silently freezing its orientation. This representation
and the yaw tangent are stated here; they are not a claim to reproduce an
uninspected author implementation. At the anchor, the optimized minimum
equals the ordinary rectangle distance up to solver tolerance.

There is no signed-distance objective, preferred passing direction or normal
normalization. Contact duals may be nonunique. Potential-field initialization
does not replace the multipliers. The retired signed-direction MEX is neither
called nor built.
The geometric construction retains the nonnegative-multiplier, unit-normal
and primal-dual identities as numerical assertions. It removes the many
five-variable `coneprog` calls and their near-optimal stopping failures;
it does not relax a trajectory constraint. The distant recovery geometry
that previously aborted with a 1.68-micrometer gap now has a gap of order
1e-14 m. Nonunique touching normals remain a property of the
ordinary-distance formulation.

The trajectory stages share the remaining soft frame budget. Formulation
and a currently running factorization are not interrupted by a hard deadline.
The original conic distance implementation and its historical experiment
results remain recorded in the dated reports; they do not describe the
current geometric implementation.

## PCBF slack and CLF dissipation

One nonnegative collision slack is assigned to each primary-horizon stage;
it relaxes the start and midpoint separation rows. Completion-tail rows are
hard. With an observer enclosure every node also carries a slack on its
tightened row, and the untightened row stays hard after the prefix only, so the
formulation reduces to the certainty-equivalent one as the enclosure vanishes.
(Hard untightened rows in the prefix made the problem stricter for a small
reported enclosure than for none. In a single-change ablation on `d1c31d6`
they left 79 of 84 noisy encounters recovering instead of 82, with three new
failures at the closest approach of the 15-m/s accelerating head-on;
`report/TRACKER_NOISE_AND_DESIGN_ABLATIONS_20261005.tex`.) The node buffer is 0.20 m, whereas the experiment's physical pass criterion
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

on the estimate: `x` is the current estimate and `x_next` its affine first
successor. Estimation error enters as an input (ISS); the row carries no
observer enclosure. A worst case over the enclosure (the previous version)
asks for a decrease below the enclosure's own size, which no input achieves
near the path. That turned the CLF stage into a greedy one-step minimizer;
in a noisy 15-m/s head-on the braking ratio then chattered (standard deviation
0.21) and the run did not recover in 100 s. In a single-change ablation on
`d1c31d6`, the worst-case row left 38 of 84 noisy encounters recovering
instead of 82; 37 of the 42 at 15 m/s did not recover within 100 s
(`report/TRACKER_NOISE_AND_DESIGN_ABLATIONS_20261005.tex`).

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

The affine solution is a search direction about the anchor. The issued plan
is the rollout, on the sampled nonlinear model, of `anchor + alpha*correction`
for the largest `alpha` in `terminal.acceptanceSteps = [1, 1/2, ..., 1/32, 0]`
that satisfies all of:

- the rollout meets every hard row (road, state limits, handling envelope,
  and collision clearance after the prefix, at nodes and hold midpoints);
- it ends in the terminal set;
- its prefix collision deficit does not exceed the anchor's.

`alpha = 0` is the anchor itself. When the anchor is the previous plan
shifted (and extended by the terminal controller), it is feasible by
construction under the hypotheses of
[TERMINAL_SAFE_SET.md](TERMINAL_SAFE_SET.md), Section 6.

Without an acceptable step (possible only when the anchor is not a feasible
plan, e.g. a startup rollout), the problem is linearized again at the full
step's rollout and solved again. This happens at most
`terminal.sqpIterations = 2` times, and then the controller reports no
solution (`nonlinearAcceptanceFailed`).

This is a step-size rule of one sequential convex method. It is not a second
controller, and a previous plan is issued only when it satisfies the current
problem. The issued plan is a nonlinear rollout, so the next frame's shift
reproduces it.

A frame fails, with `collisionAvoidanceController:noOptimizationSolution`, in
any of these cases:

- its anchor is unavailable;
- its CLF stage returns no result;
- its fresh re-solve still has no result;
- no step is acceptable.

No enlarged box, inherited budget or incomplete numerical point is used
instead.

## Input trust scale from the plan innovation

The correction box `+/-Delta*r*[0.15;0.25]` (`r = trustRadius`) bounds how far
one solve moves the inputs from its anchor. `Delta` is estimated, not set.

For a correction of size `s` (largest input change from the anchor, in box
units), the affine prediction differs from the nonlinear rollout of the same
inputs by a second-order remainder, `e ~ L*s^2`. The coefficient `L` depends
on the operating point (speed, tire slip) and is not known in advance.

The nonlinear rollout of the full affine step measures it in the frame that
takes the step. With the safety body-point metric
`P(dx) = norm(dp) + r_E * abs(dpsi)`, the model innovation is

    e_k = max_j P(x_nl,j - x_aff,j),

the largest departure of the full step's nonlinear rollout from its affine
prediction. The departure of the next posterior from the plan's second node is
recorded separately as the observation innovation: it belongs to the
estimator and the plant, and it does not set the scale.

Before the issued plan was a nonlinear rollout, the model share was obtained
at the next frame by replaying the shifted inputs from the plan's own
prediction. An earlier variant subtracted twice the observer's worst-case
tube radius from the total innovation instead. On the noisy-estimator
campaign that bound explained every innovation (0.27 m included), so the
scale rose to its maximum and the encounters became infeasible within 10 to
70 holds.

The coefficient estimate uses fast attack and slow release, and the scale is
the step whose predicted remainder equals the target innovation `tau`:

    Lhat_k  = max( e_k / max(s_k, Delta_k/2)^2 , gamma * Lhat_(k-1) ),
    Delta_(k+1) = clip( sqrt(tau / Lhat_k), Delta_min, Delta_max ),

with `s_k` the full step's size in box units. `Delta` grows by at most
`1/sqrt(gamma)` per sample and shrinks at once. A step far inside the box
(`s < Delta/2`) is not taken as evidence about the box boundary. Startup uses
`Delta_0` with `Lhat_0 = tau/Delta_0^2`. The defaults are `tau = 0.02 m`,
`Delta_0 = 0.125`, `Delta` in `[1/16, 1]` and `gamma = 0.5`.

On the fourteen exact-observation encounters (2026-10-04), the measured
`e/s^2` was about 0.07-0.09 m at 8 m/s and 0.25-0.30 m at 15 m/s, almost
independent of a fixed `Delta` between 0.125 and 0.5. Runs whose innovation
stayed below about 3 cm needed few fresh restarts at both speeds. One `tau`
therefore yields about 0.4-0.5 at 8 m/s and 0.2-0.27 at 15 m/s, without a
speed schedule.

The step-size rule, not the trust scale, decides what is issued; the scale
only bounds the next correction. `tau` is an empirical consistency target,
not a certified bound on the nonlinear remainder or on separation.

## Stable-handling envelope and the estimator's domain

At both ends of every hold, under that hold's input, except the measured node:

    ||[ (v_y - lr r)/k ; v_x b ]|| <= v_x,   k = 3 mu_r Fz_r / C_r,    (rear adhesion)
    |v_y| <= tan(model.sideslipMaximum) v_x,                            (sideslip cone)

with `v_x` frozen at the anchor in the product `v_x b`. The first is the
rear Fiala tire's unsaturated region under the braking ratio's capacity
`sqrt(1 - b^2)`, a stable-handling envelope in the sense of Beal and Gerdes
(2013), *IEEE Trans. Control Syst. Technol.* 21(4). A saturated rear axle has
no restoring yaw moment, and its force no longer determines the slip, so the
estimator's force-balance measurement would lose its information
(`estimator/MODEL_AIDED_ESTIMATION.md`). The second is that measurement's cone
premise; it is robust to the hold's ego enclosure. The adhesion cone is a
handling and observability row, not part of the safety certificate (the
estimator's enclosures need only the sideslip cone), and acts on the estimate.
The certified lateral-velocity radius widens exactly as the rear slip nears
saturation. In a noisy 15-m/s turning crossing it reached 0.81 m/s, and its
reserve lowered the admissible normalized rear slip to 0.71 while the current
state was at 0.75 and decreasing along the plan: the problem was infeasible.
The cone is divided by the anchor speed so that its entries are of order one;
unscaled, the CLF stage stalled numerically (Clarabel insufficient progress) in
a noisy 15-m/s head-on.
Before this envelope, a noisy 15-m/s head-on combined steering of -0.40 rad with
a braking ratio of +0.95 (rear lateral capacity 32%); the vehicle spun to a
0.68-rad sideslip.

## Terminal constraints

The terminal set and its proofs are in
[TERMINAL_SAFE_SET.md](TERMINAL_SAFE_SET.md). In short, with the CLF
`V = e'Pe`, an endpoint `y` is in the set when it meets all of:

- `V(y) <= cbar`, the smaller of the CLF's certified region (1) and the
  largest level whose ellipsoid lies inside the state rows (0.403 at 8 m/s on
  the straight road, where rear adhesion binds);
- the tube of every terminal-controller trajectory from `y` (lateral and
  heading errors shrinking as `exp(-t/T)`, station within a bounded drift of
  the trim's) stays on the road;
- that tube does not meet the target's forecast rectangle until the
  encounter ends: the target leaves the perception range, or the target's
  forecast motion relative to that box, widened by the tube's remaining
  drift, is outside their collision cone (straight road: the relative
  parabola under constant `A` never enters the summed box), within the 60 s
  that the check computes.

In the convex problem this is one cone at the last node,
`||F (e(y) + J dx_N)|| <= sqrt(c*)`, with `c*` the largest level whose tube at
the anchor endpoint's station is clear. The endpoint also carries the
collision rows of every other node inside `R` and the road rows. Membership
of the issued plan's endpoint is checked on its nonlinear rollout.

This replaces the earlier separating, road-recoverable terminal set
(separating relative velocity, lateral road recovery
`max(w,0)^2 <= 2 a d`). That set was not invariant: a braking lead, a target
whose sideslip turns it back, or the ego's own return to the path could close
the distance again. No recursive-feasibility argument held for it.

The road rows are the same at every node: every predicted node after the
measured one and every hold midpoint keeps all four rectangle corners within
`lateralClearance`. Each row linearizes one corner's lateral coordinate on the
path normal at the anchor corner, with the ego generator support subtracted.
Collision rows apply at nodes inside `R`, including a later re-entry.

The terminal records are:

- `terminalValue` and `terminalLevel` (the endpoint's `V` and `c*`);
- `terminalExitSeconds`;
- `terminalDistanceMeters` and `terminalRoadMarginMeters`;
- `acceptedStep`, `candidateFeasible`, `anchorCertified`,
  `appendedTerminalSteps` and `linearizationGapMeters`.

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
Continuation state version 77 stores the per-stage slacks and their total,
the issued nonlinear plan (inputs and states) and the trust estimate
(`scale`, coefficient `curvature`, last `correction`).

Metadata retains the explicit scope:

- `predictionModel = nonlinearFialaRk4RolloutOfAffineStep`;
- `safetyScope = nonlinearRolloutSampledConstraints`;
- `affineValidationPerformed = false`;
- `nonlinearValidationPerformed = true` and `nonlinearPredictionEvaluated = true`
  (the issued plan is the RK4 rollout that met the rows; this is the sampled
  model, not the physical vehicle);
- `trust` (scale, curvature, correction, innovation, observationInnovation, modelInnovation, updated);
- `egoUncertaintyModel = currentPosteriorEnclosurePropagatedThroughEachHold`,
  `targetUncertaintyModel = constantParametersRestartedEachHoldFromCurrentObserverSet`;
- `recursiveFeasibilityScope = shiftedPlanOnDeclaredModelWithConsistentForecast`;
- `terminalSet = clfTubeEncounterSafe`;
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

This is a bounded-work RTI implementation with a step-size rule. Its
recursive feasibility is the conditional statement of
[TERMINAL_SAFE_SET.md](TERMINAL_SAFE_SET.md), Section 6; it does not cover
estimation error, model mismatch or forecast changes (Section 9 there). For the general RTI
method see Diehl, Bock and Schloder,
[Real-Time Iterations for Nonlinear Optimal Feedback Control](https://cdn.syscop.de/publications/Diehl2005c.pdf).
Vehicle-specific results and full controller-call timing belong in the dated
experiment report under `report/`.
