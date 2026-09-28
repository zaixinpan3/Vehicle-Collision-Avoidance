# Nonlinear predictive safety controller

The public `collisionAvoidanceController` now uses the nonlinear combined-slip
Fiala bicycle model in inertial coordinates. Its format-48 state contains a
validated finite feedback policy and an invariant cruising continuation. The former
affine, node-only encounter certificate is not the public controller's plant or
safety argument. The September 28, 2026 design specifies Huang-style predictive safety, Li-style
polygon dual certificates, sequential convexification, and a secondary lane CLF.

## Implemented contract

The ego state is `[px; py; psi; vx; vy; r]`, in meters, radians and seconds. Inputs
are held steering angle and signed longitudinal-force ratio `[delta; b]`.
`nonlinearBicycleModel.derivative` retains front-force rotation, nonlinear slip
angles and combined-slip saturation. Numerical RK4 rollouts and integrated
variational equations propose controls. Acceptance uses the independently
validated real ODE flow in `fialaFeedbackSampleMex`, with 128-bit MPFR directed
arithmetic, interval Jacobians, quadratic remainder bounds and enclosed matrix
exponentials. An RK4 endpoint is never substituted for an exact-flow certificate.

The positive-speed model requires `vx > model.scheduleSpeedFloor`; this field
defines an explicit domain boundary and never floors the state in the nonlinear
plant. No stationary equilibrium or braking-to-rest certificate is asserted.
Actuator limits, input slew, longitudinal/lateral velocity and yaw-rate limits
are hard. The configured diagnostic slip-angle envelope is not an extra tire
constraint; the Fiala branch/domain conditions are checked by the verifier.

## Autonomous joint state and one target

The joint state is

\[
 z=[p_x,p_y,\psi,v_x,v_y,r,p_{tx},p_{ty},\psi_t]^\top.
\]

Exactly one active target record is accepted. Its tangential speed `v_t` and
heading rate `omega_t` are independent constant parameters, with

\[
 \dot p_t=v_t[\cos(\psi_t+\beta_t),\sin(\psi_t+\beta_t)]^\top,
 \qquad \dot\psi_t=\omega_t.
\]

The default is zero sideslip when the supplied velocity aligns with body heading.
A known constant course/body offset `beta_t` is also represented. The target
needs an explicit body heading. The supplied inertial velocity sets tangential
speed and the initial course. An explicit heading rate permits acceleration to
be inferred as `omega_t*J*velocity`; any separately supplied acceleration must
agree. No wheelbase/sideslip identity constrains `omega_t`. Optional
`targetRectangleOffset` specifies the footprint reference point. Nonzero ego or
target error bounds, parameter uncertainty, model residuals and acceleration
biases remain outside the exact-model certificate.

The target flow has the stable straight-line limit

\[
 p_t(t+h)=p_t(t)+v_t h\,\operatorname{sinc}(\omega_t h/2)
 [\cos(\psi_t+\beta_t+\omega_t h/2),
  \sin(\psi_t+\beta_t+\omega_t h/2)]^\top,
\]

where `sinc(a)=sin(a)/a`, with value one at zero. The target is unactuated, so its
coordinates can be eliminated exactly from each joint-state optimization step.
`nonlinearBicycleModel.jointSample` exposes the joint transition and its block
Jacobians; the controller returns `model.jointState`, fixed `targetParameters`,
and `predictedJointState`. With no target, those interfaces reduce to the ego
state. The previous applied input augments the state when slew limits apply.

The immutable target epoch and parameters allow interval checks to evaluate
absolute sample times without accumulating numerical propagation error. This
is an implementation of the autonomous joint flow. A missing later observation
retains the target. Observation consistency and sample-clock checks detect
contract changes; their numerical tolerance is not a robust uncertainty tube.
A changed target trajectory requires fresh exact-model admission.

## Geometry and full sampling intervals

For each pair of fixed rectangular poses, the closest vertex/edge construction
returns the actual Euclidean set distance (zero for overlap), a separating
normal, and nonnegative two-body dual multipliers. Body offsets are included.
For every validated integration cell, a fixed normal is used to bound all four
ego vertices below and the target support above over the **complete** cell.
The target's translation and rotation are enclosed with the analytic sinc flow.
Positive clearance is mandatory. Whole-body road containment uses the same
swept state boxes. Normal selection can be approximate; the acceptance inequality
is independently evaluated with directed rounding.

Dual variables certify separation at fixed poses. Joint pose/normal optimization
is nonconvex. No global convexity claim is made.

## Computed invariant backup

The built-in road contract is an infinite straight or constant-curvature corridor,
with global `lateralClearance = [right; left]`. A straight polyline supplies its
line. A circular `referenceCurve` supplies its continued circle. Missing road
edges mean that road containment is unconstrained. Finite quadratic fits and
varying-curvature references are rejected: their continuation needs a different
proof, rather than extrapolation of a local fit.

A nonlinear steady-turn trim uses constant path speed `hypot(vx,vy)`, body
sideslip, steering and longitudinal force balance. The transverse coordinates
are `e = [ey; epsi+betaRef; vx-vxRef; vy-vyRef; r-rRef]`; station is unpenalized.
Let `T` be the computed Cholesky factor and let `u = uRef + K*e` be held for
one sample. The candidate terminal set is

\[
  \|Te\|_2\le R,\qquad |u_{previous}-u_{ref}|\le d_u,
\]

together with the road and infinite-future target-separation conditions below.
The previous-input coordinates are part of the set, including when slew limits
are finite. The synthesis checks the worst change `2*d_u` against each slew limit.

The full initial ellipsoid is enclosed in an outward-rounded coordinate box.
A validated held-feedback sample encloses every trajectory from that box.
On its entire domain, directional interval second derivatives bound the
nonlinear residual about the trim's continuous affine tangent. This includes
trim residual and tangent roundoff. Validated matrix exponentials produce an
upper bound `a` for the linear sampled feedback map in the T norm and an upper
bound `b(R)` for the integrated residual. An interval LDL test proves the
operator-norm bound. The Metzler comparison matrix retains the negative diagonal
of the continuous tangent when bounding its impulse response. Admission requires

\[
  aR+b(R)<R.
\]

This establishes invariance of the full ball; a Riccati calculation alone does
not. The template also certifies physical limits over every appended hold.
Radii are reduced until the proof passes, or synthesis fails without a command.

For a straight road, the target must remain outside the backup's lateral swept
strip, or the ego must already be ahead with a separating forward halfspace
preserved by a lower bound on forward speed. For a target with nonzero constant heading rate, its
entire future rectangular sweep is the annulus about its orbit center, including
the footprint reference-point offset. For a circular ego continuation, disjoint
radial bands give a conservative sufficient condition. Straight target motion
uses its complete forward ray. These conditions are invariant as target time
advances. A finite-horizon encounter deadline is not used to release a target.

The backup can be conservative or empty. A stationary target on a circular
cruising path, for example, cannot be dismissed after one pass.

## Huang-style safety priority

Let `F(z,u)` be the sampled joint dynamics, `c(z)` the road and polygon
clearance violations (positive means unsafe), and `Z_f` the invariant cruise
set, including target separation and input memory. The conceptual safety
problem is

\[
 V_N(z)=\min_{U,Z,\xi}\sum_{i=0}^{N-1}\xi_i,
 \quad z_0=z,\quad z_{i+1}=F(z_i,u_i),\quad u_i\in\mathcal U,
 \quad c(z_i)\le\xi_i\mathbf1,\quad\xi_i\ge0,\quad z_N\in Z_f.
\]

The implementation also requires full sampling-interval safety. Physical state
and actuator limits, terminal membership and its infinite target continuation
remain hard. The safety slacks used during search are measured in meters for
collision and road rows. Sampling at hold boundaries and midpoints supplies a
proposal discretization; the original full-interval constraints govern admission.

This adopts the nonnegative stage-slack objective of
[Huang et al., Section III, (8)](https://arxiv.org/abs/2502.08400)
<!--ref:huang2025--><!--anchor:section:III-->. Its shift construction supplies the
safety argument. Given an admitted zero-slack policy, its successor suffix plus
the invariant controller is another zero-slack policy. Hence the optimum is
exactly zero on this certified domain without requiring a global nonconvex
solve. Input memory shifts with the issued command and the target coordinates
advance through the same autonomous dynamics.

The controller executes only this zero-slack domain. A positive LP slack sum is
a local search result, **not** a certified global PCBF value or an executable
collision allowance. Initial admission from arbitrary states and Huang's
positive-value safety-recovery theorem are not claimed. In particular, global
optimality, compactness/continuity assumptions, and convergence outside the
certified domain have not been established for this nonlinear moving-body model.
The reported `predictiveBarrierValue = 0` is justified by the admitted witness
and nonnegative objective, rather than by rounding a numerical LP result.

## Li-style polygon duals and sequential convexification

For world-frame rectangles `p_e+R_e B_e` and `p_t+R_t B_t`, with body
halfspaces `H b <= h`, choose `||n||_2 <= 1` and nonnegative multipliers satisfying

\[
 H_e^\top\mu=-R_e^\top n,\qquad H_t^\top\lambda=R_t^\top n.
\]

Then `n^T(p_e-p_t)-h_e^T mu-h_t^T lambda` is a lower bound on distance. A
positive bound proves separation. The dual update and frozen-direction
trajectory step follow [Li et al., Sections 3.1-3.2, (9)-(13)](https://doi.org/10.1007/s42154-023-00222-7)
<!--ref:li2023--><!--anchor:section:3-->. The present implementation includes both
vehicle footprints and heading-dependent ego vertices. It recomputes these duals
from each nonlinear iterate. Overlapping nominal rectangles use a signed
support direction with a negative gap; this supplies a restoration derivative
and cannot certify positive separation. A deterministic lateral tie rule breaks
geometric symmetry without a preplanned avoidance trajectory.

Each SCA iteration integrates the nonlinear held dynamics and their variational
maps around the current input sequence, updates the polygon certificates, and
constructs a sparse convex subproblem in state/input increments. State and input
trust bounds limit tangent errors. The terminal ellipsoid is represented by a
conservative polyhedral subset, with hard input-memory and linearized infinite
continuation rows. These tangents are **not** certified nonlinear inner sets.

1. `linprog` minimizes the sum of nonnegative stage safety slacks.
2. `quadprog` minimizes the lane CLF performance cost while capping the same
   safety-slack sum at the LP optimum plus a declared numerical tie tolerance.
3. A fresh nonlinear rollout evaluates safety and hard-constraint residuals.
   Decreasing a constraint merit accepts a new search center; otherwise the
   trust bounds shrink. Signed penetration prevents a flat overlap cost.
4. A candidate with small proposal residuals undergoes independent directed
   full-flow validation, including the invariant terminal set. Only an admitted
   zero-slack witness can replace the executable incumbent.

The proposal uses an extra clearance reserve to compensate for tangent and
sampling errors; admission uses the configured physical clearance directly.
If a complete prefix passes geometry but its terminal enclosure is still too
large, the same lane-feedback continuation is appended for a bounded settling
period and the entire extended candidate is validated again. This can account
for numerical enclosure width that the nominal terminal constraint does not
predict. An unsuccessful extension never authorizes a command.

The first center is a lane-feedback rollout. Horizon selection accounts for the
relative target encounter time and configured recovery duration. This rollout
may cross the target. Optional `initialPlan` is only a numerical warm start.
No passing path, potential field, or lane/avoidance mode variable is required.
`maximumAdmissionIterations` bounds restoration attempts; initial admission can
exceed the improvement time budget, as can initial proof construction.

## Secondary CLF and recursive execution

The same lane-reference function `W=||T e||_2^2` is active throughout avoidance
and recovery. The secondary objective combines its predicted values, a squared
first-hold dissipation slack, and nominal input effort. The SCA row linearizes
`W(next)-(1-gamma)W(current)<=rho`, with `rho>=0`; it can yield during an
avoidance maneuver without relaxing safety priority. The native validator then
computes an outward bound on the actual first-hold dissipation and records its
nonnegative slack. When zero slack holds on successive visited states, the
recorded inequalities imply exponential decrease along that sequence. Global
lane convergence requires compatibility of lane following and persistent target
safety; a soft CLF alone does not establish it.

A stored finite feedback policy and the invariant cruise continuation preserve
recursive feasibility independently of SCA convergence. Future holds can use
stored linear feedback evaluated once at each hold boundary. The verifier carries
state/input correlation and checks actual amplitude and slew over each hold.
Only the first nominal input is immediately executable; the certificate stores
all feedback gains and reference states. Setting `feedbackPrediction.enabled`
false requests an open-loop finite prefix. A failed solver or rejected candidate
leaves the admitted incumbent available. Executing its terminal feedback is the
continuation of this safety policy, with the same lane objective.

There is no lane-following/collision-avoidance mode switch. A custom proposal
hook is also subject to nonlinear admission, but does not implement the default
SCA search or justify claims about its local optimization hierarchy.

## Operational limits and validation

The implementation is a research controller, not a demonstrated 50 ms real-time
controller. Initial admission, interval verification and terminal synthesis can
exceed an optimization time budget. Once admitted, the stored suffix is available
before revalidation or improvement. Its original fixed-reference feedback is
evaluated at the current exact state. Every predicted future command includes a
`1e-12` actuator-unit evaluation allowance; native directed evaluation checks the
issued value against that allowance. The terminal remainder proof also includes
this allowance. On an exhausted improvement deadline the suffix is dispatched
directly. After its finite prefix, the invariant law supplies every future hold.
The prefix length may shrink below the configured nominal horizon: its implicit
infinite backup can always pad the mathematical finite-horizon witness.

Inherited proofs require the unchanged plant/road/target contracts, the actual
previous command and the proper sample clock. Endpoint box containment detects
obvious state inconsistencies; it does **not** prove that an arbitrary measured
state is the exact successor or belongs to a correlated reachable set. Thus this
reuse is conditional on exact execution and exact state data. Disturbances,
changed target parameters and nonzero estimation bounds require a robust backup
and are outside this implementation. A changed road/model requires fresh admission.
No initial witness means no issued command.

Build outside versioned source, then run the current checks from the repository
root:

```matlab
addpath('controller','config','scripts');
buildFialaIntervalVerifier('/tmp/pdca-nonlinear-mex');
addpath('/tmp/pdca-nonlinear-mex');
results = runtests('tests/nonlinearPredictiveSafetyTest.m');
assertSuccess(results);
report = runNonlinearPredictiveSafetyValidation;
```

The native build requires a MATLAB C++ compiler and system MPFR/GMP development
libraries. `prepareCollisionAvoidanceController` builds the kernels in the
excluded `solver/nonlinear/` directory if missing. Format-47 and older prediction
states, affine-plant scenario replay and the older estimator-bound scenario
contracts are incompatible with this entry point. The new validation driver
replays the nonlinear ODE independently and reports failures as failures.

The [September 28 joint-state SCA validation](../report/JOINT_PCBF_SCA_20260928.md)
records the implementation, 139 passing selected tests, complete stored-policy
avoidance/recovery, and the substantial online optimization runtime limitation.

Design context: [Huang et al., predictive barrier values](https://arxiv.org/abs/2502.08400),
[Batkovic et al., safe MPC](https://arxiv.org/abs/2305.03312), and
[Li et al., dual-based convex trajectory steps](https://doi.org/10.1007/s42154-023-00222-7).
