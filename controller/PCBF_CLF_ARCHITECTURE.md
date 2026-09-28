# Joint-state PCBF, lane CLF and sequential convexification

## Implemented design

The controller has one receding-horizon optimization:

\[
\boxed{\text{PCBF safety priority} + \text{lane-following CLF}
       + \text{SCvx with polygon distance duals}.}
\]

`collisionAvoidanceController` parses the current joint state,
`solveNonlinearPredictivePlan` solves the nominal MPC problem, and the first
input of its solution is applied. `predictiveSafetyGeometry` supplies the
constant-parameter target flow, polygon duals and terminal geometry.
`nonlinearBicycleModel` supplies the nominal dynamics, analytic tangents and
lane LQR ingredients. There is no interval integration, MPFR library,
nonlinear certificate, immutable target epoch, or exact-successor admission
contract in this execution path.

The public four-output call remains:

```matlab
[command, inputs, problem, state] = ...
    collisionAvoidanceController(ego, target, road, cfg, previousState);
```

Use `state` as the next call's `previousState`. The optional persistent state
can still be reset with `collisionAvoidanceController("resetNominalTrajectory")`.
`problem.solution` contains nominal states, inputs, stage slacks, CLF values
and optimization residuals. State format 49 replaces format 48; an older state
starts a fresh solve. The old `certificate` output field and certification
metadata have been removed. Use `metadata.planFeasible`, `zeroSlack`,
`predictiveBarrierValue`, `solutionSource` and `search`.

## Joint prediction model

There is at most one target. The ego state and input are

\[
x_e=(X_e,Y_e,\psi_e,v_{x,e},v_{y,e},r_e),\qquad
u=(\delta_f,\beta).
\]

The ego uses the nonlinear combined-slip Fiala bicycle equations and RK4
prediction under a held input. Its analytic force and kinematic derivatives
are integrated with the RK4 variational equations.

For constant tangential speed \(v_t\), heading rate \(\omega_t\), and an optional
known constant course/body-heading offset \(b_t\),

\[
\dot p_t=v_t[\cos(\psi_t+b_t),\sin(\psi_t+b_t)]^T,
\qquad \dot\psi_t=\omega_t.
\]

The offset is zero when velocity follows body heading. Speed and heading rate
are independent parameters. The exact discrete target flow is

\[
p_t^+=p_t+h v_t\operatorname{sinc}(\omega_t h/2)
 \begin{bmatrix}\cos(\psi_t+b_t+\omega_t h/2)\\
 \sin(\psi_t+b_t+\omega_t h/2)\end{bmatrix},\quad
\psi_t^+=\psi_t+h\omega_t,
\]

where \(\operatorname{sinc}(a)=\sin(a)/a\), including its limit at zero.
The joint state is \(z=(x_e,p_t,\psi_t)\in\mathbb R^9\). The target's autonomous
coordinates are eliminated analytically from the optimization; they remain
in `model.jointState` and `predictedJointState`. With finite slew limits, the
previous applied input is an additional memory coordinate for MPC reasoning.

Each new target observation initializes the target part of the current state.
During a missing observation, the previous target is propagated by this flow;
a timestamp supplies elapsed time when available, otherwise one sample is used.
The controller is nominal: uncertainty fields are not propagated into tubes.
Changing measurements can require renewed feasibility restoration.

## Huang-style primary problem

Let \(c(z_i,u_i)\le0\) collect polygon clearance and road containment at the
stage start and its predicted midpoint. A scalar slack covers both samples of
each hold. The primary problem is

\[
V_N(z)=\min_{\mathbf u,\mathbf z,\boldsymbol\xi}
                  \sum_{i=0}^{N-1}\xi_i
\]

subject to

\[
z_0=z,\quad z_{i+1}=F_h(z_i,u_i),\quad
u_i\in U,\quad |u_i-u_{i-1}|\le h\dot u_{\max},
\]
\[
c(z_i,u_i)\le\xi_i,\quad \xi_i\ge0,\quad z_N\in Z_f.
\]

Velocity-domain limits and terminal constraints are hard. The horizon starts
from the configured minimum request and is extended during the initial lane
rollout to cover the encounter and terminal lane set, up to the configured
maximum. Subsequent shifts keep the selected horizon length.

This follows Huang et al., problem (8), with the additional midpoint samples
sharing the stage slack. The additional samples preserve the shift argument.
A zero-slack feasible plan attains the global lower bound of the safety
objective. Positive slack represents safety recovery; it does **not** assert
collision-free execution. Positive-slack plans are returned with their actual
barrier value, instead of requiring a separate zero-slack certificate before
any input can be returned.

## Polygon duals and SCvx

Both vehicles are oriented rectangles, including optional offsets from their
state reference points. For body-frame halfspaces \(A b\le d\), rotations
\(R_e,R_t\), and a target-to-ego unit normal \(n\), the distance dual uses

\[
A^T\mu=-R_e^T n,\quad A^T\lambda=R_t^T n,\quad
\mu,\lambda\ge0,\quad \|n\|_2\le1,
\]
\[
n^T(p_e-p_t)-d_e^T\mu-d_t^T\lambda\ge d_{\mathrm{safe}}-\xi.
\]

For separated rectangles, the closest-point direction supplies an optimal
distance dual. At overlap, signed support gaps supply a nonzero restoration
direction; a lateral tie-break avoids a zero gradient at a symmetric encounter.
The four ego-vertex support rows retain heading dependence. Their pose
Jacobians and the dynamics are linearized at each current SCvx iterate.
This extends Li et al.'s fixed-dual convex trajectory step to two rectangular
footprints. Joint pose/dual optimization remains nonconvex.

Each iteration solves a safety LP and then a CLF QP with the same dynamics,
input, geometry and terminal rows. The QP constrains the slack sum to the LP
optimum plus `solver.lexicographicTieTolerance`. A trust region and the usual
nominal rollout merit/residual update control linearization error. These are
numerical optimizer operations. No formal verifier follows the solver.

The initial center is a clipped lane-feedback rollout. It may intersect the
target, so no preplanned avoidance trajectory is needed. Later calls shift the
previous input sequence and append lane terminal feedback. A feasible shifted
plan is retained if further iterations fail or exhaust their budget. Reuse
starts from the measured state; it has no exact-successor equality predicate.

## Secondary lane-following CLF

The lane error is

\[
e=(e_y,e_\psi,v_x-v_x^r,v_y-v_y^r,r-r^r),\quad W(e)=e^TPe,
\]

where \(P\) and \(K\) come from the discrete LQR design about a realizable
constant-curvature cruise trim. The first predicted hold uses

\[
W(e_1)-(1-\alpha)W(e_0)\le\rho,\qquad \rho\ge0.
\]

Its linearization enters the QP, along with the horizon lane-error cost,
small input-effort cost and `clf.relaxationWeight` times \(\rho^2\). The returned
CLF slack is recomputed from the nominal prediction. A positive CLF slack
allows avoidance to take priority; the same objective encourages lane return
as the collision constraints relax. There is no lane/avoidance mode selector.

## Terminal ingredients and recursive feasibility

The terminal lane controller is \(u_f=u^r+Ke\). A local ellipsoid
\(\|P^{1/2}e\|_2\le r_f\) defines its error domain. Analytic row norms bound
state and input deviations and limit the configured radius using actuator,
velocity and slew limits. The QP uses an inscribed 1-norm polytope for this
ellipsoid. Terminal input memory leaves room for the next lane-feedback input.

Straight and constant-curvature global lane corridors are supported. A
closed-form terminal geometry constraint separates the whole terminal lane
tube from the target's remaining straight ray or circular orbit. On a straight
road this can be a lateral separation or an ego-forward halfspace with
nondecreasing relative progress. On a circular road it uses separated radial
ranges. This is a sufficient terminal set, so it can exclude some feasible
maneuvers; it avoids infinite-horizon numerical search and interval arithmetic.

**Nominal recursive-feasibility statement.** Assume that the selected terminal
set is invariant for the prediction model under \(u_f\), satisfies the hard
constraints and has zero stage slack. Shifting any feasible plan and appending
\(u_f\) gives a feasible successor plan with slack sequence
\((\xi_1,\ldots,\xi_{N-1},0)\). Consequently, an optimal primary solve satisfies

\[
V_N(z^+)\le V_N(z)-\xi_0.
\]

Feasible suboptimal updates that improve upon that shift retain the same plan
cost inequality. In particular, a zero-slack feasible plan preserves the
nominal sampled safe set under the shift. CLF optimization is secondary and
cannot override this safety priority.

The LQR design proves contraction for the local linearized terminal model;
nonlinear invariance of a selected radius remains a model/design assumption.
SCvx is a local numerical solver, so a finite iteration/time limit is not a
proof of the global positive-slack PCBF optimum or Huang's global recovery
result. `predictiveBarrierValue` reports the achieved feasible plan cost.
The nominal sampling and integration model also does not establish continuous
intersample safety or robustness to disturbances and estimation error.
These limits are documented rather than enforced through a formal runtime
certificate layer. The online implementation contains no universal
`recursiveFeasibilityGuaranteed` flag.

## Validation and references

Run the focused implementation checks and independent closed-loop replays:

```matlab
addpath('scripts');
validateNonlinearPredictiveController(outputDirectory);
```

The MATLAB entry requires Optimization Toolbox and Control System Toolbox.
No MEX build or MPFR installation is needed. See
`report/SIMPLIFIED_PCBF_20260928.md` for the executed scope and measured timing.
Offline replay and Python geometry audits produce research evidence, and are
not called when the controller issues an input.

- J. Huang, H. Wang, K. Margellos and P. Goulart, *Predictive Control Barrier
  Functions: Bridging Model Predictive Control and Control Barrier Functions*,
  ECC 2025, problem (8), Lemmas III.1/III.5 and Section III.C.
  [Author preprint](https://arxiv.org/abs/2502.08400).
- G. Li, X. Zhang, H. Guo, B. Lenzo and N. Guo, *Real-Time Optimal Trajectory
  Planning for Autonomous Driving with Collision Avoidance Using Convex
  Optimization*, Automotive Innovation 6, 481–491 (2023), Sections 3.1–3.2.
  [Publisher](https://doi.org/10.1007/s42154-023-00222-7).
