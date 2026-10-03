# Inherited PCBF slack budgets and one-step CLF optimization

Every frame uses the same target-independent analytic quadratic CLF, defined in
[NOMINAL_CLF.md](NOMINAL_CLF.md). Initialization and restoration minimize PCBF slack before CLF slack.
A continuation frame first attempts CLF optimization under an inherited slack
budget. A bounded input-increment tie term regularizes the future plan. There is no third optimization stage, target-range
CLF switch, direct nominal-feedback command, or executable backup.

## Prediction and initialization

The Fiala bicycle state is `x = [px; py; yaw; vx; vy; yawRate]` and its input is
`u = [steering; brakingRatio]`. RK4 defines the prediction map and its tangents.
The default uses one 50-ms RK4 step; its midpoint collision node is an endpoint
interpolant, not an independently integrated half hold.

The prediction length is fixed at

    M = min(maximumHorizonSteps, N + ceil(completionSeconds / sampleTime)).

Previous inputs are shifted and rolled out from the current measurement in
both encounter and recovery frames. Prior affine states are not reused as the
linearization trajectory. Without usable previous inputs, one moving-target
potential-field rollout is constructed; when no target is present, nominal path guidance
supplies the initialization. Potential guidance is a search reference, not a safety
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
to the optimization. The last `settlingSeconds` settle intrinsic velocities into
the existing free-pose straight terminal core. The final pose remains free, so
this construction does not require arrival at a fixed path station. A seed may
still violate collision or terminal constraints and is never executable by itself.

**Normal frames shift the previous input plan.** Only startup, an unevaluable
shift, or failure of its optimization/accuracy step constructs this new field
rollout. Each shifted or fresh trajectory is linearized once. There is no SCP
iteration on an optimized nonlinear rollout within the same hold.

For an anchor `(xbar_i, ubar_i)` the shared model is

    x_(i+1) = f_h(xbar_i,ubar_i) + A_i (x_i-xbar_i) + B_i (u_i-ubar_i).

The same anchor supplies dynamics, tire derivatives, collision directions,
terminal geometry and the analytic first-successor CLF approximation. Input/state correction
boxes are numerical iteration bounds. At unit trust scale, the steering
correction is +/-0.075 rad and the braking-ratio correction +/-0.125. The
input scale adapts from measured prediction disagreement and transfers to
the next frame. Steering has no physical
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

## Inherited safety budget

For an accepted affine prediction with prefix slacks `xi[0:N-1]`, retain
`S = sum(xi)` and construct `xi_shift = [xi[1:N-1], 0]`. The CLF problem first
uses the hard constraints

    sum(xi_new) <= sum(xi_shift) = S - xi[0],
    xi_new[0] <= xi_shift[0].

No lexicographic tie is added to the inherited budget. The first-stage cap
prevents transferring the available slack into the executed hold. Its rows
cover the hold's start and midpoint, not continuous-time clearance. The same
budget remains fixed through any model-accuracy refinement in this frame;
`xi[0]` is subtracted once per physical sample, never once per iteration.
The normal objective remains the same CLF slack plus bounded proximal tie.
There is no additional weighted safety objective.

In an exact common prediction model, if the previous full trajectory is
feasible and its appended terminal feedback stays admissible, the shifted
slacks and controls are a feasible candidate. The tail contributes the new
zero slack. Consequently the budget-constrained problem is feasible without
recomputing the optimum of the primary problem. The retained quantity is a
trajectory-dependent upper bound on the optimal safety value, not the optimal
PCBF value itself. Inherited-budget stages therefore record `primaryOptimum`
as NaN and `primaryOptimumComputed` as false.

This exact-shift premise does not automatically hold in the implemented
nonlinear controller: the saved states satisfy the previous affine model;
the new anchor is integrated from the current measurement; collision tangents
and terminal placement are rebuilt. Even small discrepancies can invalidate
a tight cap. The implementation attempts the inherited cap in the current
convex program; `budgetAnchorResidual` records whether the zero-correction
shift itself satisfies its assembled constraints. It is a diagnostic, not a
nonlinear safety certificate. A failed capped CLF solve triggers primary
restoration and the existing bounded potential-field retry. The restoration creates a
new budget and breaks the previous monotonic-budget chain; this event is
explicit in the attempt log. No stored plan supplies an issued command.

Appending the old terminal feedback, rather than merely a trim input, uses
the old terminal reference and augmented input memory. Outside that core the
feedback is only an initializer; terminal membership still belongs to the
new optimization. A target/context reset uses the existing explicit reset
contract, rather than inheriting a budget for a changed problem.

With exact feasibility and no restoration/tolerance errors,
`S[k+1] <= S[k] - xi[0|k]` implies summability of the executed-stage slacks and
`xi[0|k] -> 0`. It does not alone imply `S[k] -> 0`: the sequence `(0,c)` can
postpone a future slack indefinitely while keeping its sum constant.
Thus Huang et al.'s optimal-value convergence theorem cannot simply be
relabelled as a theorem for this suboptimal budget. The reference is
[Huang et al., 2025, Section III](https://arxiv.org/html/2502.08400v1).
Finite solver tolerances and budget restorations further limit any exact
monotonicity claim; closed-loop recovery and collisions are audited separately.

## Primary restoration and secondary dissipation

One nonnegative collision slack is assigned to each primary-horizon stage;
it relaxes the start and midpoint separation rows. Completion-tail rows are
hard. The node buffer is 0.20 m, whereas the experiment's physical pass criterion
is positive rectangle clearance. No road-boundary constraint is imposed.
The independent given-path center-deviation bound is described in
[PATH_DEVIATION_BOUND.md](PATH_DEVIATION_BOUND.md); it defaults to 10 m and is
hard in both optimization stages, including the finite completion tail.

The first problem minimizes the sum of PCBF slacks. The unbounded CLF slack
and its cone are eliminated from this problem without changing its feasible
projection. If the zero correction with zero slacks satisfies every linear
row, equality, bound and endpoint cone, it attains the global lower bound zero.
The primary stage is then completed analytically; the CLF stage still runs.
`numericalSolve` distinguishes analytic completion from an actual solver call. The primary therefore contains only the
shared dynamics, state/input limits, collision rows and endpoint cone.

A finite positive primary value need not be a useful safety result. The fixed
current-state rows supply the unavoidable lower bound

    J_lower = max(0, -min(g(x_current))).

When a shifted problem has no numerical point, or its primary value exceeds
this lower bound, one fresh potential-field model is tried. Primary values are compared only after positive solver termination; a finite
array from a failed primary solve is not a valid PCBF optimum. The smaller
primary value is retained, with ties favoring the shifted model. A fresh problem that reports infeasibility or numerical
stalling permits one bounded enlargement to twice the nominal input box; its better primary result is retained. State boxes, physical bounds,
collision rows and terminal conditions remain. The CLF is constructed only once for each model and reused if the inherited budget needs primary restoration. At most one fresh potential-field retry is used across
the complete within-frame refinement, rather than resetting that allowance
each time the dynamics are rebuilt.

On the selected model the single CLF requirement is

    V(x_next) - V(x) <= -eta * e(x)' Q e(x) + rho,    rho >= 0,
    V(x) = e(x)' P e(x).

P is the local discrete LQR matrix for the given-path cruise trim. The analytic
error Jacobian applied to the affine first successor gives a convex quadratic
with Hessian J' P J. One second-order cone represents its epigraph. There are
no cost-to-go rollouts or finite-difference curvature calculations. This
quadratic is locally justified, not a global nonlinear CLF; positive slack can
remain necessary away from cruise even without a target. The secondary minimizes

    rho / max(V(x),1) + epsilon * R(dU),
    sum(xi) <= inheritedBudget or achievedPrimary + lexicographicTieTolerance.

`R` is the normalized squared input increment about the linearization inputs.
At an exact optimum its effect on minimum scaled CLF slack is bounded by
`epsilon = clfTieTolerance = 1e-4`. It penalizes neither absolute zero input
nor deviation from the construction feedback. PCBF priority is retained;
CLF minimization has this explicit bounded tie allowance. A zero primary
result still runs the CLF stage. The componentwise input boxes imply `R <= 1`,
so the former epigraph `R <= sigma <= 1` is eliminated exactly. Clarabel solves
the native sparse quadratic objective with the endpoint and CLF cones retained;
these are conic QPs rather than linearly constrained QPs. State variables remain
explicit to preserve dynamic sparsity.

The compiled adapter seeks the configured constraint/optimality precision.
Its reduced-accuracy termination uses the existing `feasibilityTolerance`
(default 1e-5) for primal/dual residuals and absolute/relative objective gap.
Flags 1 and 2 distinguish full and reduced-accuracy convergence. Numerical
acceptance error is additional to the analytic lexicographic tie bounds.
Infeasibility rays, time-limit iterates and unresolved numerical failures return
no point. `solverInfo` records native status, iterations and residuals.

## Nonlinear model agreement and damping

Each completed PCBF/CLF pair produces an affine state trajectory and input
sequence. The inputs are propagated from the current measured state through
the same nonlinear RK4 map. Let `x_aff` and `x_nl` denote those predictions.
The agreement measures are

    E_pose = max_i (norm(p_aff_i-p_nl_i) + r_E * abs(psi_aff_i-psi_nl_i)),
    E_state = max_i norm((v_aff_i-v_nl_i) ./ [5;3;1.5], inf),
    E_clf = abs(V_nl_next-V_model_next) / max(1,V_current),

For `E_pose`, the maximum covers nodes supporting active collision rows
(including the endpoint of the last such hold), or the full horizon when
terminal geometry remains active. After the encounter exit, positions do not
restrict this collision-pose discrepancy measure. The separate path-deviation
measure below still covers the full nonlinear prediction. With no collision
pose constraints, only the fixed initial node enters this discrepancy measure. `E_state` still covers all
horizon nodes and `E_clf` always evaluates the actual first successor value.
The full-horizon pose discrepancy is separately reported as
`fullPoseErrorMeters`; `poseConstraintNodeCount` identifies the measured
subset. Here `r_E` is the ego half-diagonal plus offset norm and
`v = [vx;vy;yawRate]`.
The body-point displacement expression avoids accepting a small center
position error with a large heading error. The defaults are 0.01 m for
`E_pose` and 0.01 for each scaled state/CLF error. These are numerical
agreement thresholds, not certified bounds on plant/model uncertainty or
intersample safety.

The nonlinear rollout also projects every state onto the given path. For a
finite lateral limit `L`, it contributes
`max(0, 1 + (max(abs(e_y_nl))-L)/predictionToleranceMeters)` to the agreement
ratio. Thus a trajectory outside the path corridor cannot be accepted merely
because its model errors are small. This reuses the existing rollout and damping;
it adds no trajectory linearization or optimization stage.

A ratio `r` is the maximum of the three normalized errors and this deviation ratio.
If `r <= 1`, the completed CLF candidate becomes an eligible optimization
iterate. A zero primary value never skips the CLF stage. Positive PCBF slack
remains a relaxed result and is not interpreted as collision freedom.

The first completed CLF step satisfying model agreement ends the frame.
Positive CLF slack at a local input boundary does not trigger optional extra
rounds. A failed shift can request a fresh initialization only. This is a computational
stopping rule, not a proof that a local step preserves global nominal recovery;
the closed-loop campaign must test that property. The complete CLF objective
has been solved before this stopping rule is considered. A damped step need
not attain the optimum of that solved CLF problem.

To damp an inaccurate optimization step, retain the primary point `zP` and the
secondary point `zS` of the same assembled conic problem. In an inherited-budget
attempt, `zP` is the zero correction with the shifted slacks; it is not presumed
feasible. Give `zP` the minimum nonnegative CLF epigraph height for its first
input. Damping is available only when its equality, inequality, box and cone
residuals are within `constraintTolerance` in this exact assembled problem.
This is a condition on the segment construction, not a new nonlinear safety
admission layer.

For two feasible endpoints, convexity gives

    z(alpha) = zP + alpha * (zS-zP),  0 <= alpha <= 1,

in the same feasible set. States, controls and stage slacks are interpolated
together. The safety budget, first-stage cap, affine dynamics, collision rows,
physical bounds and terminal cone are preserved up to endpoint solver tolerances.
The CLF epigraph variable is then tightened to the exact value of the existing
convex quadratic at the interpolated input. This does not rebuild or differentiate
the CLF, and cannot invalidate its epigraph inequality. The successor CLF value
must be evaluated quadratically, not interpolated between endpoint values.

Try at most three fractions starting at `min(0.5,0.8/sqrt(r))` and halving after
an inaccurate trial. Each trial uses the existing nonlinear prediction-agreement
evaluation, with no new conic solve. The first accurate trial ends the frame.
When the full secondary solution has zero CLF slack within tolerance, a damped
trial must retain zero slack. If it loses zero, smaller fractions are skipped:
the convex zero-slack sublevel set on the segment is an interval containing
alpha=1. This prevents a runtime shortcut from deliberately giving up a
modeled zero-slack dissipation step. Positive optimal slack may increase after
damping; global recovery and collision performance must still be measured.

The approximate quadratic scaling of linearization error applies when the
segment starts at the nonlinear linearization anchor. A nonzero primary
correction can retain model error even as alpha tends to zero. Neither a small
fraction nor conic feasibility proves nonlinear collision freedom. An infeasible
base or unsuccessful bounded line search does not create another linearization
of the same plan.

There are at most **two actual trajectory model builds** per frame. A normal
frame uses one shifted model and one inherited-budget CLF solve. If the shifted
problem cannot produce an accurate completed CLF result, one fresh potential
rollout may replace the initialization and receive its own model. A fresh seed
that fails reports failure; it is not repeatedly optimized and relinearized.
`maximumLinearizations=1` disables the fresh retry after a built shifted model;
the default of two allows it. Startup has only the one fresh model regardless.
The initial input trust scale is 0.125; the previous accepted scale is inherited
on a normal frame. This local numerical step bound is not an actuator constraint.

A failed inherited-budget attempt restores PCBF on the existing matrices and
reuses the analytic CLF cone. One bounded input-box enlargement also reuses those
matrices, changing only input bounds and the normalized proximal objective.
Initialization and these restoration solves can require more than one numerical
solve; one model does not imply one solve in every exceptional frame.
`linearizationCount` counts actual builds, while attempt logs retain reused-model
solves with `modelBuilt=false`.

A returned scale grows by at most 1.5 for the next frame when agreement is good,
up to its nominal value of one. A damped result also multiplies that scale by the
accepted fraction, with a 1/1024 floor. These are numerical correction limits,
not physical steering limits. No global SQP convergence theorem or hard execution
deadline follows from this finite work budget.

The first accurate result from the current frame is issued. A frame without
an accurate completed CLF result reports
`collisionAvoidanceController:noOptimizationSolution`; inaccurate numerical
points and previous-frame plans are not issued instead.

## Terminal constraints

The completion tail and augmented input memory remain. The endpoint core has
free rigid pose; its Schur-reduced ellipsoid restricts velocity and final input,
not a fixed longitudinal arrival position. It does not redefine the CLF.

The existing encounter-range contract is unchanged. Its exit index is selected
on the anchor. At an endpoint still within range, the existing separation or
departure geometry contributes affine rows. Fitting the stored endpoint pose
after optimization is not an acceptance test. Neither the core's nominal
invariance argument nor the affine tail certifies the realized nonlinear plant.

## Issued commands, diagnostics and limits

The selected CLF optimizer result, or its accepted feasible-segment step,
supplies the applied input. Its applied input is not clipped. A nonpositive solver flag cannot supply an input. Positive numerical
termination and nonlinear prediction agreement are required; neither is a
continuous-time safety certificate. If no sufficiently
accurate completed CLF result exists,
the controller reports `collisionAvoidanceController:noOptimizationSolution`.
No previous trajectory or primary-only point is issued instead.

`primaryOptimum` records the achieved primary value; without positive solver
termination it is not a certified optimum. Attempt logs retain all primary
models and timings; `selectedAttempt` identifies the model supplying the
command. `optimizationConverged` describes the numerical stages of the selected
problem, not discarded restoration attempts, optimality of an inherited cap,
or optimality of an issued damped point. `secondaryOptimumApplied` distinguishes
a full solution from a damped step. Continuation state version 71 stores the per-stage slacks and their total
alongside the existing single-CLF trajectory state. The optional stored
`linearizationTrustScale` carries the learned step size, not an executable
backup policy.

Metadata retains the explicit scope:

- `predictionModel = affineFialaRk4Linearization`;
- `safetyScope = affineSampledConstraints`;
- `affineValidationPerformed = false`;
- `nonlinearValidationPerformed = false` (no complete nonlinear-constraint certificate);
- `nonlinearPredictionEvaluated = true`, with `predictionAgreement`;
- `modelAgreementHistory`, `refinementCount`, and `modelAgreementSatisfied`;
- `lineSearchHistory`, `lineSearchSeconds`, `lineSearchTrials`, and `acceptedStepFraction`;
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
