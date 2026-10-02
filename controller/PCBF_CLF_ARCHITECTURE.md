# Two-stage PCBF / CLF real-time iteration

Every frame uses the same target-independent nominal cost-to-go CLF, defined in
[NOMINAL_CLF.md](NOMINAL_CLF.md). The objectives remain PCBF slack first, CLF
slack second. A bounded input-increment tie term in the second objective makes
the future plan determinate. There is no third optimization stage, target-range
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
flow rollout is constructed; when no target is present, nominal path guidance
supplies the initialization. Flow guidance is a search reference, not a safety
certificate. No maneuver bank is used.

The target keeps Sharma et al. (2026), Eq. (17)'s constant tangential
acceleration `A` and constant sideslip `beta`. The prediction uses inertial
coordinates, with `V(t)=V0+A*t` and curvature `sin(beta)/lr`; yaw rate therefore
changes with speed. There is no added sideslip-rate state. The exact arc-length
flow supplies both initialization and collision constraints at matching
absolute times. The existing signed-velocity continuation is retained; a stop
clamp would change the constant-acceleration model.

The flow seed adds a timed Gaussian reference inspired by Cheng et al.
(2021), Eqs. (34)-(36). On the finite seed horizon it pairs nominal station
`s0+vRef*t` with the target's predicted station and lateral position. A single
Gaussian's signed amplitude covers the sampled circular exclusion envelope
on the less-displaced side. Its center is the closest nominal encounter time;
its temporal width is at least the nominal lookahead and half the longitudinal
interaction span. This finite-time construction never divides by closing
speed. Its lateral displacement and derivative guide the existing moving
cylinder field; that field still includes the target center velocity, including
rotation of an offset body center. The guide previews the entire finite seed
horizon, even when the target initially lies outside the encounter range.

The nominal station clock is approximate. The actual Fiala rollout samples
the moving field and the target at the same absolute time, but need not follow
the Gaussian exactly. Neither the geometric envelope nor its curved-road
Frenet approximation certifies this bicycle rollout. A failed seed remains a
possible search object. This changes initialization only, without changing
the Li distance dual, solver stages, CLF, or command acceptance semantics.
The derivation, paired experiments, and remaining failures are recorded in
[the timed-flow study](../report/SHARMA_TIMED_FLOW_20261002.tex).

For an anchor `(xbar_i, ubar_i)` the shared model is

    x_(i+1) = f_h(xbar_i,ubar_i) + A_i (x_i-xbar_i) + B_i (u_i-ubar_i).

The same anchor supplies dynamics, tire derivatives, collision directions,
terminal geometry and the first-input CLF approximation. Input/state correction
boxes are numerical RTI bounds. Normally the steering correction is +/-0.075
rad and the braking-ratio correction +/-0.125. Steering has no physical
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
It is solved by `coneprog` at each collision anchor. The resulting `lambda`
is fixed when assembling all four trajectory inequalities

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
nonunique. Flow initialization does not replace those multipliers or repair
their zero gradients. The retired signed-direction MEX is neither called nor built.
The exact rectangle-distance routine serves offline geometry, the existing
encounter-range query, and an independent primal-dual check at each anchor.
That check requires nonnegative multipliers and a unit-ball normal within
`1e-8`, and a distance gap within `1e-6 m`. It handles `coneprog` stopping
with a small search direction at an already optimal point; it never replaces
or normalizes the returned multipliers. It is not a nonlinear trajectory
acceptance test.

The reported two trajectory solver calls exclude the small geometric dual
solves. Geometry work contributes to formulation and full-call elapsed time.
The trajectory stages receive the remaining soft time budget; as before,
formulation itself is not interrupted by that budget.

The October 2 replacement is recorded in
[the validation record](../report/ORDINARY_DISTANCE_DUAL_20261002.json).
The current focused suites contain 224 passing checks, combining the broad
run with the final rerun of the changed two-stage suite. The former
crossing-restoration assertion failed and was replaced by an explicit
hard-overlap infeasibility check; that scenario failure remains in the record.
An independent 15 m/s crossing replay executes 17 of 19 requested 50 ms
holds, then stops at 0.85 s with `pcbfNoNumericalResult`. Its 17 issued holds
all exceed 50 ms computation time (subsequent median 0.364 s, maximum
0.768 s). This is not a successful encounter or a real-time qualification.

## Primary restoration and secondary dissipation

One nonnegative collision slack is assigned to each primary-horizon stage;
it relaxes the start and midpoint separation rows. Completion-tail rows are
hard. The node buffer is 0.10 m, whereas the experiment's physical pass criterion
is positive rectangle clearance. No road-boundary constraint is imposed.

The first problem minimizes the sum of PCBF slacks. An unbounded CLF slack and
the redundant increment epigraph can be eliminated from this problem without
changing its feasible projection. The primary therefore contains only the
shared dynamics, state/input limits, collision rows and endpoint cone.

A finite positive primary value need not be a useful safety result. The fixed
current-state rows supply the unavoidable lower bound

    J_lower = max(0, -min(g(x_current))).

When a shifted problem has no numerical point, or its primary value exceeds
this lower bound, one fresh flow model is tried. The smaller primary value is
retained, with ties favoring the shifted model. A fresh problem that returns an
infeasibility exit with no point permits one bounded input-box enlargement by a factor of
two; its better primary result is retained. State boxes, physical bounds,
collision rows and terminal conditions remain. The CLF is evaluated only after
this primary-model selection. There is no repeated nonlinear acceptance loop.

On the selected model the single CLF requirement is

    V_K(F_h(z,u_0)) - V_K(z) <= -eta * ell(z) + rho,    rho >= 0.

Only two first-input variables enter the successor value. A centered stencil
forms a nonnegative Gauss-Newton residual model, augmented by the positive
part of the omitted residual curvature. It is a convex quadratic represented
as a second-order cone, not a certified global upper bound. The secondary
problem minimizes

    rho / max(V_K(z),1) + epsilon * sigma,
    sum(xi) <= achievedPrimary + lexicographicTieTolerance,
    R(dU) <= sigma <= 1.

`R` is the normalized squared input increment about the linearization inputs.
At an exact optimum its effect on minimum scaled CLF slack is bounded by
`epsilon = clfTieTolerance = 1e-4`. It penalizes neither absolute zero input
nor deviation from the construction feedback. PCBF priority is retained;
CLF minimization has this explicit bounded tie allowance. A zero primary
result still runs the CLF stage. All conic calls use `coneprog` because the
endpoint and CLF constraints are quadratic cones, not linear QP constraints.

Normally there are two solves. Restoration can add two primary solves. A
failed secondary numerical solve can use the one fresh initialization if it
has not already been used, with the same bounded enlargement. There is a
shared soft time budget, not a demonstrated execution deadline.

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

Only the selected second-stage optimizer result supplies the applied input.
It is not clipped, replayed or replaced. Finite points with nonpositive solver
exit flags are still executable. If no second-stage numerical result exists,
the controller reports `collisionAvoidanceController:noOptimizationSolution`.
No previous trajectory or primary-only point is issued instead.

`primaryOptimum` records the achieved primary value; without positive solver
termination it is not a certified optimum. Attempt logs retain all primary
models and timings; `selectedAttempt` identifies the model supplying the
command. `optimizationConverged` describes the selected PCBF and CLF stages,
not discarded restoration attempts. Continuation state version 66 distinguishes
this single-CLF implementation from older stored plans.

Metadata retains the explicit scope:

- `predictionModel = affineFialaRk4Linearization`;
- `safetyScope = affineSampledConstraints`;
- `affineValidationPerformed = false`;
- `nonlinearValidationPerformed = false`;
- `recursiveFeasibilityScope = notCertifiedForNonlinearPlant`;
- unmeasured constraint residuals and margins are NaN;
- `clfFunction = nominalCostToGo`, with no tertiary objective;
- required decrease, modeled successor value, rollout length, tail diagnostic
  and bounded CLF tie allowance are recorded.

Positive PCBF slack is relaxation, not collision freedom. Zero modeled CLF
slack alone is not a nonlinear decrease theorem: the local RTI value model is
an approximation. The mathematical regional CLF construction, its local tail
condition and the remaining proof obligations are stated in NOMINAL_CLF.md.
Independent ODE45 replay measures actual clearance and recovery offline;
these measurements never become online admission gates.

This is a bounded-work RTI implementation. It does not automatically inherit
nonlinear stability or recursive-feasibility guarantees. For the general RTI
method see Diehl, Bock and Schloder,
[Real-Time Iterations for Nonlinear Optimal Feedback Control](https://cdn.syscop.de/publications/Diehl2005c.pdf).
Vehicle-specific results and full controller-call timing belong in the dated
experiment report under `report/`.
