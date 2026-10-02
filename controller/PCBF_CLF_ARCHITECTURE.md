# Two-stage PCBF / CLF iteration within one control hold

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
possible search object. The flow constructor supplies only the initial
trajectory; the following within-hold iteration controls prediction accuracy.
The derivation, paired experiments, and remaining failures are recorded in
[the timed-flow study](../report/SHARMA_TIMED_FLOW_20261002.tex).

For an anchor `(xbar_i, ubar_i)` the shared model is

    x_(i+1) = f_h(xbar_i,ubar_i) + A_i (x_i-xbar_i) + B_i (u_i-ubar_i).

The same anchor supplies dynamics, tire derivatives, collision directions,
terminal geometry and the first-input CLF approximation. Input/state correction
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
nonunique. Flow initialization does not replace those multipliers or repair
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
this lower bound, one fresh flow model is tried. Primary values are compared only after positive solver termination; a finite
array from a failed primary solve is not a valid PCBF optimum. The smaller
primary value is retained, with ties favoring the shifted model. A fresh or relinearized problem that reports infeasibility or numerical
stalling permits one bounded enlargement to twice the nominal input box; its better primary result is retained. State boxes, physical bounds,
collision rows and terminal conditions remain. The CLF is evaluated only after
this primary-model selection. At most one fresh-flow retry is used across
the complete within-frame refinement, rather than resetting that allowance
each time the dynamics are rebuilt.

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

## Nonlinear model agreement and relinearization

Each completed PCBF/CLF pair produces an affine state trajectory and input
sequence. The inputs are propagated from the current measured state through
the same nonlinear RK4 map. Let `x_aff` and `x_nl` denote those predictions.
The agreement measures are

    E_pose = max_i (norm(p_aff_i-p_nl_i) + r_E * abs(psi_aff_i-psi_nl_i)),
    E_state = max_i norm((v_aff_i-v_nl_i) ./ [5;3;1.5], inf),
    E_clf = abs(V_nl_next-V_model_next) / max(1,V_current),

For `E_pose`, the maximum covers nodes supporting active collision rows
(including the endpoint of the last such hold), or the full horizon when
terminal geometry remains active. After the encounter exit, free positions
have no pose constraint and do not restrict this accuracy test. With no pose
constraints, only the fixed initial node enters. `E_state` still covers all
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

A ratio `r` is the maximum of the three errors divided by their tolerances.
If `r <= 1`, the complete two-stage candidate becomes an eligible optimization
iterate. A zero primary value never skips the CLF stage. Positive PCBF slack
remains a relaxed result and is not interpreted as collision freedom.

Accuracy is not the only stopping condition. If CLF slack is positive and
the normalized first-input correction reaches the local box boundary (within
1e-3), the controller rebuilds and solves both stages again. Otherwise a
small, accurate correction can repeatedly stop short of a descent input,
while the shifted second input restores the same bad trust center next frame.
This condition depends on the existing CLF and numerical box, not target
distance or a separate recovery mode.

The full nonlinear propagation of the optimized inputs becomes the next
anchor when evaluable. Damping every inaccurate candidate can leave its
endpoint far from the small terminal core; shrinking the next correction
box then creates artificial infeasibility and repeated expansion. Rebasing
the complete trajectory avoids that particular obstruction. Only an invalid
nonlinear rollout uses up to nine halved input steps to recover an evaluable
anchor; otherwise the old anchor remains. These search controls are never
directly issued. When `r > 1`, the next input trust scale contracts by
`max(0.1,min(0.8,0.8/sqrt(r)))`, with a floor of 1/1024. Dynamics, tire
derivatives, collision duals, terminal fit and CLF approximation are rebuilt.

There are at most eight refinement rounds and a shared soft time budget.
There is normally one PCBF/CLF pair once the previous frame supplies a useful
anchor and trust scale. A returned scale can grow by at most 1.5 for the next
frame when agreement is good, up to its nominal value of one. This does not
impose a physical steering magnitude or slew bound. It also does not prove
SQP convergence: no global merit-function descent theorem is asserted.

The best accurate complete pair evaluated in this frame is retained, with
priority to PCBF slack within its tie tolerance, then the actual nonlinear
first-successor CLF slack. A later failed or inaccurate trial cannot delete
it. `selectedRefinement` identifies this iterate, whereas `refinementCount`
counts all evaluated pairs. This is ordinary retention of optimization
iterates; it does not execute a previous frame's trajectory or another policy.
A frame without any accurate completed pair reports
`collisionAvoidanceController:noOptimizationSolution`; finite inaccurate
solver vectors are no longer sufficient for execution. This deliberately
implements the user-selected same-frame relinearization route.

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
Its applied input is not clipped or replaced. A finite point with a
nonpositive solver flag must still satisfy nonlinear prediction agreement;
it is not thereby certified affine-feasible or optimal. If no sufficiently
accurate completed second-stage result exists,
the controller reports `collisionAvoidanceController:noOptimizationSolution`.
No previous trajectory or primary-only point is issued instead.

`primaryOptimum` records the achieved primary value; without positive solver
termination it is not a certified optimum. Attempt logs retain all primary
models and timings; `selectedAttempt` identifies the model supplying the
command. `optimizationConverged` describes the selected PCBF and CLF stages,
not discarded restoration attempts. Continuation state version 66 distinguishes
this single-CLF state schema from older stored plans. The optional stored
`linearizationTrustScale` carries the learned step size, not an executable
backup policy.

Metadata retains the explicit scope:

- `predictionModel = affineFialaRk4Linearization`;
- `safetyScope = affineSampledConstraints`;
- `affineValidationPerformed = false`;
- `nonlinearValidationPerformed = false` (no complete nonlinear-constraint certificate);
- `nonlinearPredictionEvaluated = true`, with `predictionAgreement`;
- `modelAgreementHistory`, `refinementCount`, and `modelAgreementSatisfied`;
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
these dense ODE45 safety/recovery measurements remain offline. Online
nonlinear RK4 expansion evaluates approximation accuracy, not a full
physical-vehicle safety certificate.

This is a bounded-work RTI implementation. It does not automatically inherit
nonlinear stability or recursive-feasibility guarantees. For the general RTI
method see Diehl, Bock and Schloder,
[Real-Time Iterations for Nonlinear Optimal Feedback Control](https://cdn.syscop.de/publications/Diehl2005c.pdf).
Vehicle-specific results and full controller-call timing belong in the dated
experiment report under `report/`.
