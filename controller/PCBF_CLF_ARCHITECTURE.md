# Joint-state PCBF, lane CLF and sequential convexification

## Implemented design

The controller has one receding-horizon optimization:

\[
\boxed{\text{PCBF safety priority} + \text{lane-following CLF}
       + \text{SCvx with polygon distance duals}.}
\]

`collisionAvoidanceController` parses the current joint state,
`solvePredictiveControl` solves the nominal MPC problem, and the first
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
and optimization residuals. State format 51 stores `inputTrajectory` and
`stateTrajectory` solely for the next numerical warm start. An older state
starts a fresh solve. Each call runs the optimizer and directly applies its
first returned control, for both zero and positive safety slack. If no
optimizer result exists, the call raises `optimizationFailed`.

Use `metadata.optimizationReturned`, `zeroSlack`, `predictiveBarrierValue`,
`hardConstraintResidual`, `scvxConverged`, `controlSource` and `search` for
diagnostics. `zeroSlack` means within `solver.feasibilityTolerance`; it does
not mean an exact real-arithmetic zero. Neither the slack sign nor the reported nonlinear residual
selects an execution mode. The first input always matches `inputs(:,1)`.

## Joint prediction model

There is at most one target. The ego state and input are

\[
x_e=(X_e,Y_e,\psi_e,v_{x,e},v_{y,e},r_e),\qquad
u=(\delta_f,\beta).
\]

The ego uses the nonlinear combined-slip Fiala bicycle equations and RK4
prediction under a held input. Its analytic force and kinematic derivatives
are integrated with the RK4 variational equations. This tire model requires
longitudinal velocity above `model.scheduleSpeedFloor` (1 m/s by default).

The target has constant tangential acceleration \(A_C\) and constant
sideslip \(\beta_C\). Its rear-axle distance \(l_{r,C}>0\) is fixed. The
single-track kinematics agree with the existing NRMM target model on its
positive-speed domain:

\[
\dot p_C=V_C[\cos(\psi_C+\beta_C),\sin(\psi_C+\beta_C)]^T,\qquad
\dot V_C=A_C,\qquad \dot\psi_C=\kappa_C V_C,
\]
\[
\kappa_C=\frac{\sin\beta_C}{l_{r,C}},\qquad
\dot A_C=\dot\beta_C=0.
\]

Thus curvature is constant and heading rate changes with speed. With signed
arc length \(s(h)=V_C h+A_C h^2/2\), the exact flow is

\[
p_C^+=p_C+s(h)\operatorname{sinc}(\kappa_C s(h)/2)
 \begin{bmatrix}\cos(\psi_C+\beta_C+\kappa_C s(h)/2)\\
 \sin(\psi_C+\beta_C+\kappa_C s(h)/2)\end{bmatrix},
\]
\[
\psi_C^+=\psi_C+\kappa_C s(h),\qquad V_C^+=V_C+A_C h,
\]

where \(\operatorname{sinc}(a)=\sin(a)/a\), including its limit at zero.
The joint state is \(z=(x_e,p_C,\psi_C,V_C)\in\mathbb R^{10}\). The target's autonomous
coordinates are eliminated analytically from the optimization; they remain
in `model.jointState` and `predictedJointState`. With finite slew limits, the
previous applied input is an additional memory coordinate for MPC reasoning.

`model.target` stores `[X; Y; psi; V; A; beta; lr; halfLength; halfWidth;
offsetX; offsetY]`. Its last seven entries are `model.targetParameters`.
The target block of the exact joint-state Jacobian includes
\(\partial p_C^+/\partial V_C=h[\cos(\psi_C^++\beta_C),
\sin(\psi_C^++\beta_C)]^T\) and
\(\partial\psi_C^+/\partial V_C=h\kappa_C\).

The input accepts `targetTangentialAcceleration` or the NRMM field
`targetScalarAcceleration`. If neither is supplied, acceleration is the
projection of `targetAccelerationInertial` (or its framed equivalent) along
\([\cos(\psi_C+\beta_C),\sin(\psi_C+\beta_C)]^T\); missing acceleration defaults
to zero. `targetSideslip` may be supplied, or is inferred from velocity and
explicit body heading on the branch \(|\beta_C|<\pi/2\). At zero velocity it
defaults to zero unless supplied. A supplied `targetRearAxleDistance` overrides
`cfg.target.rearAxleDistance` (1.6 m by default). The NRMM estimate publishes its
own rear-axle distance. `targetYawRate` is no longer a prediction parameter;
heading rate is always calculated from \(V_C\sin\beta_C/l_{r,C}\).

The prediction retains constant acceleration through zero using **signed
tangential velocity**. A braking target can stop instantaneously and then
reverse along the same path; no stop-and-hold clamp changes \(A_C\). In reverse,
\(\beta_C\) defines the body's forward tangent and the velocity course differs
by \(\pi\). The NRMM observer's positive-speed operating domain is unchanged.
This continuation is a mathematical modeling assumption, not a prediction
that a physical braking driver must reverse.

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
A zero-slack feasible solution attains the global lower bound of the safety
objective. Positive slack represents safety recovery; it does **not** assert
collision-free execution. Both zero-slack and positive-slack optimizer results are executed directly
and returned with their achieved slack sum.

## Polygon duals and SCvx

The physical collision criterion is strictly positive rectangle distance.
`collision.safetyMarginMeters` defaults to zero: there is no required additional
0.10 m buffer. A nonnegative optional buffer can be configured explicitly.
The optimizer imposes nonnegative signed separation at its constraint samples;
these closed inequalities and their numerical tolerances cannot certify a
strict inequality or exclude contact between samples. Offline collision checks
therefore require distance strictly greater than zero, with no tolerance that
would admit touching or overlapping rectangles. A reported zero safety slack
alone is not a strict collision-free certificate.

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
optimum plus `solver.lexicographicTieTolerance`. A trust region compares actual and predicted reductions of the nominal
rollout merit to control linearization error. These are
numerical optimizer operations. No formal verifier follows the solver.

The initial center is a clipped lane-feedback rollout. It may intersect the
target, so no preplanned avoidance trajectory is needed. Later calls shift the
previous input sequence and append lane terminal feedback to initialize SCvx.
That initialization is never an executable fallback. At least one LP result
from the current call is required. If a QP fails, its successful primary LP
result remains the current iteration's optimizer result. If later SCvx
iterations fail or exhaust their budget, the solve returns the numerical
iterate already obtained during this call. LP/QP exit status determines whether the solver returned a solution.
Its linear constraint residual is reported without an execution veto.

Trial iterates outside the bicycle model domain shrink the trust region.
Cold and warm starts use the same SCvx iteration budget.
Nominal rollout merit chooses SCvx updates; the CLF linearization residual
also enters its stopping condition. A finite iteration budget can leave
nonlinear residuals. They are reported without a separate execution veto.
The soft time budget is checked between iterations and can be overrun by a
running LP/QP. It does not supply a real-time deadline guarantee.

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
tube from the target's remaining straight path or circular orbit. For
\(\kappa_C\ne0\), the fixed orbit radius is \(1/|\kappa_C|\), independent of
acceleration. A target with both \(V_C=A_C=0\) remains a fixed rectangle.
For a straight target path, analytic quadratic extrema bound all future
displacement \(V_C t+A_Ct^2/2\), including any reversal. On a straight road,
separation can be lateral, or the ego rear can remain ahead of the target's
maximum relative forward excursion. This accounts for the ego terminal
progress lower bound and the target's acceleration. An accelerating target
behind the ego cannot qualify merely because its current speed is lower.
On a circular road the target path gives separated radial ranges.
This is a sufficient terminal set, so it can exclude some feasible
maneuvers; it avoids infinite-horizon numerical search and interval arithmetic.

**Nominal recursive-feasibility statement.** Assume that the selected terminal
set is invariant for the prediction model under \(u_f\), satisfies the hard
constraints and has zero stage slack. Shifting any feasible input sequence and appending
\(u_f\) gives a feasible successor sequence with slack values
\((\xi_1,\ldots,\xi_{N-1},0)\). Consequently, an optimal primary solve satisfies

\[
V_N(z^+)\le V_N(z)-\xi_0.
\]

Feasible suboptimal updates that improve upon that shift retain the same
cost inequality. In particular, a zero-slack feasible solution preserves the
nominal sampled safe set under the shift. CLF optimization is secondary and
cannot override this safety priority.

The LQR design proves contraction for the local linearized terminal model;
nonlinear invariance of a selected radius remains a model/design assumption.
SCvx is a local numerical solver, so a finite iteration/time limit is not a
proof of the global positive-slack PCBF optimum or Huang's global recovery
result. The direct execution implementation does not enforce a separate
comparison with the shifted cost or a nonlinear feasibility admission rule.
`predictiveBarrierValue` reports the achieved nominal slack sum; it need not
equal the globally optimal PCBF value. A returned result by itself therefore
does not establish the hypotheses of the recursive-feasibility theorem.
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
`report/DIRECT_PCBF_CLEANUP_20260928.md` for the executed scope and measured timing.
The revised target model and its checks are recorded in
`report/TARGET_ACCELERATION_SIDESLIP_20260928.md`.
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
