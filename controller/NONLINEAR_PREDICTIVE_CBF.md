# Nonlinear predictive safety controller

The public `collisionAvoidanceController` now uses the nonlinear combined-slip
Fiala bicycle model in inertial coordinates. Its format-47 state contains a
validated finite feedback policy and an invariant cruising continuation. The former
affine, node-only encounter certificate is not the public controller's plant or
safety argument. The supplied redesign in the September 28, 2026 request is the
design source; the separately linked sandbox attachment was unavailable locally.

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

Inputs use the existing ego and target record fields. A target must publish its
body heading separately from velocity. Optional `targetSideslip`,
`targetRearLength` and `targetRectangleOffset` describe the literal NRMM and
footprint reference point. Speed, sideslip and inertial heading rate obey
`omega = speed*sin(sideslip)/rearLength`; acceleration must be `omega*J*velocity`.
Ego and target error bounds must be zero. Nonzero parameter uncertainty,
unmodeled dynamics and acceleration biases are rejected.

The target's initial epoch and parameters are retained. Future checks use the
analytic constant-parameter flow evaluated at absolute sample times with outward
time arithmetic. A missing later observation does not delete the target.
Observation consistency is checked against this immutable model; the configured
comparison tolerance is a diagnostic tolerance, not a robust uncertainty tube.
A changed target trajectory or sample clock requires a new exact-model admission.

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
preserved by a lower bound on forward speed. For a rotating NRMM target, its
entire future rectangular sweep is the annulus about its orbit center, including
the footprint reference-point offset. For a circular ego continuation, disjoint
radial bands give a conservative sufficient condition. Straight target motion
uses its complete forward ray. These conditions are invariant as target time
advances. A finite-horizon encounter deadline is not used to release a target.

The backup can be conservative or empty. A stationary target on a circular
cruising path, for example, cannot be dismissed after one pass.

## Predictive barrier, CLF and optimization

A certified hard plan ending in the backup is a zero-slack witness for the
nonnegative predictive-barrier value. Thus `predictiveBarrierValue = 0` is known
without globally solving the nonconvex positive-slack problem. Under exact
successors, unchanged road/model/target contracts and the invariant backup,
shifting the controls and appending the backup establishes the mathematical
recursive-feasibility argument. No positive collision slack is executable.

The controller retains the inherited suffix, validates an appended plan, and tests a
nominal feedback continuation. It can also test passing-side initialization
sequences. Every such sequence is only a proposal until its complete nonlinear
flow and terminal membership pass validation.

From the second hold, the prediction can apply the stored linear feedback gain
to the difference between the state and its stored nominal state. Each command
is evaluated once at that hold's boundary and then held. The nonlinear interval
verifier carries state/input correlation and bounds both the resulting actuator
values and their slew. This reduces accumulation of numerical enclosure error.
The returned input plan contains nominal controls; its certificate also stores
the feedback gains and reference states. Only the first command is immediately
executable. `feedbackPrediction.enabled = false` requests an open-loop finite
prefix; the terminal backup remains a feedback law in either case.

The numerical improvement uses a convex control box around a certified policy.
The stored feedback gains and reference states remain fixed. Two independent
nominal-input interval generators enter each held sample. Propagating the whole
box, checking every swept rectangle/road cell, actuator/slew bound and terminal
condition certifies **every** control sequence in that box. A failed box is
shrunk, and no optimization occurs unless box admission succeeds. This is a
conservative convex inner set of the nonlinear problem, with no dynamic tangent
substituted for the plant and no claim of a globally convex collision-free set.

The first held flow also supplies interval Jacobians. A Peano--Baker series with
an outward norm tail encloses its time-varying variational map. If the scaled CLF
gradient is in `m + [-r,r]` throughout the certified box, then

\[
 g_{CLF}(\bar U+Dv) \le \overline g_{CLF}(\bar U)
       +m^\top v_0+r^\top |v_0|.
\]

Absolute-value epigraphs make this a convex row. The bound touches the certified
anchor residual at zero increment. Stage one minimizes the nonnegative CLF slack
using `linprog`; stage two minimizes a positive-semidefinite quadratic nominal
surrogate with that slack capped using `quadprog`. Nominal cost derivatives come
from variational integration along the nonlinear policy rollout and an adjoint;
these derivatives affect performance only. Directed box membership and independent
point-policy validation precede replacement, and the actual certified slack/cost
pair must improve lexicographically. A custom proposal hook has only nonlinear
acceptance guarantees; the certified-inner metadata flag is false for that hook.
The local hierarchy does not claim global nonlinear lexicographic optimality.

The native validator evaluates outward bounds on
`W(next) - (1-gamma)*W(current)`, where `W = norm(T*e)^2`. The only performance
slack is its nonnegative upper bound. If the local error chart is unavailable,
the CLF row is omitted and `clfAvailable` is false. This cannot invalidate a
hard safety plan. When zero slack is certified on successive visited states,
the recorded one-step bounds imply exponential decrease on that sequence.
Tiny positive slack at numerical trim is reported honestly, not rounded to zero.
Return from arbitrary avoidance states is not asserted without this compatibility.

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
excluded `solver/nonlinear/` directory if missing. Format-46 affine prediction
states, affine-plant scenario replay and the older estimator-bound scenario
contracts are incompatible with this entry point. The new validation driver
replays the nonlinear ODE independently and reports failures as failures.

Design context: [Huang et al., predictive barrier values](https://arxiv.org/abs/2502.08400),
[Batkovic et al., safe MPC](https://arxiv.org/abs/2305.03312), and
[Li et al., dual-based convex trajectory steps](https://doi.org/10.1007/s42154-023-00222-7).
