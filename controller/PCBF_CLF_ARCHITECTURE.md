# Two-stage affine PCBF / CLF controller

The online controller builds one affine prediction model and solves two convex
problems in lexicographic order: minimum PCBF slack, then minimum CLF slack
while retaining the first-stage optimum. There is one controller and one
initialization trajectory. There is no nonlinear candidate admission,
sequential-convexification loop, restoration solve, correction QP, zero-CLF
trial, predictive-cost third stage, or retained-control fallback.

## Prediction and initialization

Let the six-dimensional ego state be $x=(p_x,p_y,\psi,v_x,v_y,r)$ and the input
be $u=(\delta,b)$. The Fiala bicycle model supplies a discrete RK4 map $f_h$ and
its derivatives. The working tree's 50-ms default integration step is retained.
With a single RK4 step, the midpoint is the endpoint interpolant; it is not a
second, independently integrated map.

The finite prediction length is fixed for a call:

$$M=\min\{M_{\max},N+\lceil T_{\rm completion}/h\rceil\}.$$

It is not increased until a candidate passes a safety test. A usable previous
affine state/input trajectory is shifted and extended to this length. Otherwise,
one moving-target flow rollout is constructed, using actuator magnitude and
slew limits. With no target, initialization uses lane feedback. Flow guidance
is an initialization method, not an executable safety certificate or a second
controller. No seed bank or branch comparison is performed.

At the fixed anchor $(\bar x_i,\bar u_i)$, the optimization uses

$$x_{i+1}=f_h(\bar x_i,\bar u_i)
 + A_i(x_i-\bar x_i)+B_i(u_i-\bar u_i).$$

The defect $f_h(\bar x_i,\bar u_i)-\bar x_{i+1}$ is retained explicitly.
A shifted affine prediction need not itself be a nonlinear rollout. Initial
state equality uses the new measurement, and slew constraints use the actual
previous input. Dynamics, tire derivatives, collision directions and tracking
error derivatives all use this same anchor. Bounds on increments remain fixed
through both stages; the trust box is never expanded or contracted online.

## Shared constraints and the two objectives

Write the common convex feasible set as $\mathcal F(\bar X,\bar U)$.
It contains affine dynamics, physical state bounds, actuator magnitude/slew
bounds, sampled affine rectangle-separation rows, the terminal constraints,
and a relaxed first-step CLF constraint. Road boundaries are excluded.

There is one nonnegative collision slack $\xi_i$ per primary-horizon stage.
It relaxes that stage's start and midpoint separation rows. Completion-tail
collision rows are hard. The configured 6-mm geometric buffer is retained
in the affine separation rows. A positive slack relaxes that buffer.

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

A missing/unsuccessful stage, or an exhausted shared soft budget, reports
`collisionAvoidanceController:noOptimizationSolution`. The controller does not
execute an old trajectory or a first-stage-only result. Optimizer exit flags,
objective values and timings are recorded separately. The maximum number of
optimizer calls is two, with fewer calls only on an incomplete frame.

Metadata explicitly records:

- `predictionModel = affineFialaRk4Linearization`;
- `safetyScope = affineSampledConstraints`;
- `nonlinearValidationPerformed = false`;
- `recursiveFeasibilityScope = notCertifiedForNonlinearPlant`;
- raw convex `hardConstraintResidual`, both stage exit flags, the primary
  optimum, and the second-stage PCBF cap;
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
check affine residuals under their declared tolerance and still check physical
replay clearance independently.

The shared soft time budget remains configurable. Two solver calls do not by
themselves establish a 100-ms runtime bound, especially including initialization,
terminal construction and matrix assembly. No new scenario campaign or runtime
benchmark is claimed by this implementation change.
