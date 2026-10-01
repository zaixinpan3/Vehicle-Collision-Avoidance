# Two-stage PCBF / CLF real-time iteration

The online controller normally builds one affine prediction model and solves two convex
problems in lexicographic order: minimum PCBF slack, then minimum CLF slack
while retaining the first-stage optimum. There is one controller and one
initialization trajectory per attempt. A failed warm-start solve can trigger one
fresh flow initialization and another two-stage attempt within the same soft
budget. There is no nonlinear candidate admission,
convergence loop, restoration solve, correction QP, zero-CLF
trial, predictive-cost third stage, or retained-control fallback.

## Prediction and initialization

Let the six-dimensional ego state be $x=(p_x,p_y,\psi,v_x,v_y,r)$ and the input
be $u=(\delta,b)$. The Fiala bicycle model supplies a discrete RK4 map $f_h$ and
its derivatives. The working tree's 50-ms default integration step is retained.
With a single RK4 step, the midpoint is the endpoint interpolant; it is not a
second, independently integrated map.

The finite prediction length is fixed for a call:

$$M=\min\{M_{\max},N+\lceil T_{\rm completion}/h\rceil\}.$$

It is not increased until a candidate passes a safety test. Previous inputs are
shifted, extended by the nominal terminal input, and rolled out from the new
measured state. Previous affine states are not reused as dynamics references.
If those inputs cannot be evaluated in the bicycle model's domain, or no previous
plan exists,
one moving-target flow rollout is constructed, using braking-ratio magnitude and
slew limits. With no target, initialization uses lane feedback. Flow guidance
is an initialization method, not an executable safety certificate or a second
controller. No seed bank or branch comparison is performed.

At the fixed anchor $(\bar x_i,\bar u_i)$, the optimization uses

$$x_{i+1}=f_h(\bar x_i,\bar u_i)
 + A_i(x_i-\bar x_i)+B_i(u_i-\bar u_i).$$

The anchor satisfies $\bar x_{i+1}=f_h(\bar x_i,\bar u_i)$ by construction;
the formulation still retains the defect explicitly. Initial
state equality uses the new measurement, and braking slew constraints use the
actual previous input. Dynamics, tire derivatives, collision directions and tracking
error derivatives all use this same anchor. State and input correction bounds
remain fixed through both stages. With numerical radius $\Delta$, input
corrections obey $|u_i-\bar u_i|\le\Delta[0.15,0.25]^T$; the default
$\Delta=0.5$ gives a 0.075-rad steering correction per local solve. This is an
iteration bound recentered at every sample, not an actuator steering magnitude
or slew constraint. It is a numerical starting choice, not a certified universal
linearization-error bound. The flow initializer selects a tire-informed steering reference.
The two retired steering-limit configuration fields are rejected as unknown
options. Continuation state version 63 separates this reference update from older
plans.

## Shared constraints and the two objectives

Write the common convex feasible set as $\mathcal F(\bar X,\bar U)$.
It contains affine dynamics, physical state bounds, braking-ratio magnitude/slew
bounds, sampled affine rectangle-separation rows, the terminal constraints,
and a relaxed first-step CLF constraint. Road boundaries are excluded.

There is one nonnegative collision slack $\xi_i$ per primary-horizon stage.
It relaxes that stage's start and midpoint separation rows. Completion-tail
collision rows are hard. The configured 5-cm geometric buffer is retained
in the affine separation rows. A positive slack relaxes that buffer. The
buffer applies at the sampled rows only; it is not a continuous-time
clearance bound between them.

The nominal CLF remains referenced to the given path and desired cruise speed,
independently of terminal placement. With $V_k=e(x_k)^T P e(x_k)$ and the
first-step affine error $\widehat e_1$, its constraint is

$$\widehat e_1^T P\widehat e_1\le(1-\alpha)V_k+\rho,
\qquad \rho\ge0.$$

The two stages are exactly

$$J_B^*=\min_{(X,U,\xi,\rho)\in\mathcal F}\sum_{i=0}^{N-1}\xi_i,$$

$$\min_{(X,U,\xi,\rho)\in\mathcal F}\rho
\quad\text{subject to}\quad
\sum_{i=0}^{N-1}\xi_i\le J_B^*+\varepsilon_{\rm lex}.$$

CLF slack is scaled numerically by $\max(V_k,1)$ without changing its
minimizer. `lexicographicTieTolerance` is the explicit numerical allowance on
the first objective, not a tunable exchange weight. No tracking or input cost
can purchase either slack. Even if the initializer has zero slack, both stages
run. There is no third tie-breaking objective, so multiple minimizing input
sequences may exist.

The quadratic CLF sublevel and ellipsoidal endpoint set are represented as
second-order cones. Thus both calls use `coneprog`; the formulation is not a
linearly constrained QP. This does not require an outer nonlinear iteration.

## Terminal constraints retained in the shared problem

The completion tail and augmented terminal input memory remain in the
formulation. The endpoint core has free rigid pose; its Schur-reduced
ellipsoidal constraint restricts velocity and final input, without requiring a
fixed longitudinal arrival position. It does not redefine the nominal CLF.

The existing encounter-range convention is retained. Its exit index is selected
once on the initialization trajectory. At an endpoint still inside that range,
the existing separation/departure geometry is evaluated at the anchor and its
first-order rows are inserted in both solves. The endpoint pose is fitted to
the final affine result for storage; that fit is not a post-solve safety gate.

Core construction and anchor-based terminal geometry still require computation.
They are not repeated searches over optimized nonlinear candidates. The
existing terminal-core math remains tested, but its invariance proof does not
certify an affine prediction's realized nonlinear endpoint.

## Output, failure and claim scope

Only the second-stage optimizer result supplies the applied first input. It is
not clipped, replayed, polished or replaced after optimization. Returned
`predictedState` values are affine predictions. A usable previous trajectory
only supplies the next linearization. No executable `witness` is stored.

Each finite solver result proceeds directly, regardless of its exit flag.
The first result supplies the PCBF cap; the second supplies the applied input.
There is no post-solve feasibility or optimality admission test, and no online
recomputation of constraint residuals or separation margins. In particular,
a finite result with exit `-7` or an iteration-limit exit is not discarded.
The solver's own convergence and stopping criteria remain part of `coneprog`.

An empty/nonfinite result from either stage of a shifted-input attempt triggers
one fresh flow rollout if time remains. Dynamics, tire tangents, collision normals,
terminal rows and the CLF model are all rebuilt from it. The restarted primary
objective obtains its own optimum and cap; the old cap is not carried over.
No second restart is made, and a cold flow attempt is not repeated identically.
Positive PCBF slack and finite nonconverged results do not trigger a restart.
If that attempt also returns no vector, or the budget is exhausted,
the controller reports `collisionAvoidanceController:noOptimizationSolution`.
No old trajectory or first-stage-only result is substituted. Exit flags,
initialization and assembly times, and each attempt's solver timings are recorded
separately. Normally there are two optimizer calls; a failed first or second
stage followed by a complete restart takes three or four. A first-stage value returned
without solver convergence is the achieved value, not a certified optimum;
the legacy `primaryOptimum` field records that value.

Metadata explicitly records:

- `predictionModel = affineFialaRk4Linearization`;
- `safetyScope = affineSampledConstraints`;
- `nonlinearValidationPerformed = false`;
- `affineValidationPerformed = false`;
- `recursiveFeasibilityScope = notCertifiedForNonlinearPlant`;
- `optimizationReturned` records availability of both numerical results;
  `optimizationConverged` records positive solver exits and never gates output;
- `hardConstraintResidual`, `minimumCollisionMargin` and
  `terminalSeparationMargin` are `NaN` (unmeasured), not zero or a safety claim;
- both stage exit flags, the achieved primary value, and the second-stage cap;
- `zeroSlack` using the declared affine feasibility tolerance.

Positive optimized PCBF slack is returned as a relaxed result; it does not
mean collision-free. Tiny positive slack can also occur within the numerical
lexicographic allowance even when the primary optimum is zero. Removing
nonlinear admission removes the former nonlinear sampled-safety claim.
Likewise, zero affine CLF slack establishes dissipation in this call's affine
error model, not a proof of nonlinear closed-loop asymptotic recovery.

Independent ODE replay and rectangle checks remain available in experiment
scripts. They are offline measurements and do not add online acceptance steps.
Historical audit exports retain their original interpretation. New exports
declare that affine residuals were not measured; the offline audit reports
that affine feasibility was not verified and still measures replay clearance.
Missing residuals serialize to JSON null and do not establish feasibility.

This is an RTI-style lexicographic successive linearization: one local update is
carried across sampling instants instead of converging a nonlinear program within
each frame. It does not inherit classical RTI stability guarantees automatically;
the slack objectives, collision constraints and initialization require their own
analysis. A useful starting point is Diehl, Bock and Schloder,
[Real-Time Iterations for Nonlinear Optimal Feedback Control](https://cdn.syscop.de/publications/Diehl2005c.pdf).

The shared soft time budget remains configurable. A bounded number of solves does not by
themselves establish a 100-ms runtime bound, especially including initialization,
terminal construction and matrix assembly. Measurements for this change belong
in the dated RTI experiment report under `report/`.
