# Certificate-preserving predictive CBF-CLF-QP

This is the implemented controller contract as of 2026-09-06. The online
controller uses one performance objective, a maintained safe continuation,
and at most one QP call per sample. It does not optimize over alternative
geometric starts. It is a conservative predictive controller on a selected
convex domain, not an exact convex reformulation of unrestricted trajectory
optimization or a pointwise CBF theorem.

## Controller state and admission

Use the explicit state interface for independent controller instances:

```matlab
certificate = [];
[command, headInputs, problem, certificate] = ...
    collisionAvoidanceController(ego, target, road, cfg, certificate);
```

The certificate contains the complete input sequence, predicted states,
scheduled model, node frames and supporting geometry, target continuation,
configuration/environment identity, committed actuator vector, and acceptance
residuals. The four-input interface stores the same state persistently for
existing scenario drivers. `resetNominalTrajectory` clears that convenience
interface and does not change an explicitly supplied certificate.
Certificates use version 4 for the route-interval geometry, held-input
flow and commanded-input model contract. Earlier certificates must undergo initial admission again.

Initial admission constructs one domain from the schedule reference and
attempts one QP. This reference is not certified in advance. The augmented
state enters the certified domain only after a feasible plan passes acceptance.
An infeasible initial problem issues no command. Admission is not a complete
maneuver planner, and a safe state can lie outside the selected domain.

An applicable certificate supplies its shifted schedule and inputs, followed
by the terminal policy. The formulation checks this witness before solving.
Geometry may change only when the replacement admits that witness; otherwise
the carried geometry is retained. A failed preservation check is an explicit
`invalidStoredCertificate` error, not permission to execute an unchecked plan.

## One model and one optimization

Let `N` be the performance horizon, `Nb` the additional continuation length,
and `M = N + Nb`. All `M` stages use the same scheduled Frenet bicycle:

\[
 x_{j+1}=A_jx_j+B_ju_j+c_j,\qquad
 x=[s,d,e_\psi,v_x,v_y,r]^\top,\quad u=[\delta_f,a]^\top.
\]

Each stage integrates its scheduled affine model exactly under a held input,
using one block matrix exponential. The CLF uses the continuous generator
of that same scheduled model and a continuous-time cruise Riccati design. Acceleration bias propagates through the entire acceleration
input column. The first predicted pose therefore includes the input's
within-sample effect. This replaces Euler integration consistently in both
head and continuation; it does not make the nonlinear plant model exact.

Every stage has both steering and acceleration decision variables. There is
no dynamic-to-kinematic handoff. `ltvBicycleModel.brakingSchedule` only supplies an initial
speed schedule and derives `Nb` from maximum speed and the configured
braking rate, with two rest stages. The optimizer can steer and accelerate
throughout the continuation subject to the same physical limits as the head.
The configured braking rate does not replace those physical rows.

`model.longitudinalInputGain` is a fixed declared gain `gamma` in `(0, 1]`:
`vxDot = gamma*a + rBar*vy + b`. The input `a` remains the requested
longitudinal acceleration used by the actuator adapter. The default is 1;
the Blockset experiment uses 0.80 after observing a step-average braking
response of 0.841 times its command. This is a fixed modeling choice motivated
by the recorded response, not a certified lower bound on every nonlinear-plant response. Gain uncertainty
and actuator dynamics have not been enclosed by a feedback tube.

The gain enters all head/continuation matrices, cruise equilibrium, CLF
Riccati cache, bias propagation and resting input. Axle polygons still check
the full requested longitudinal force `rho*m*a`, while their affine load
transfer uses the model's effective acceleration `gamma*a`. For the declared
gain no larger than one, the model's realized longitudinal force has no larger
magnitude than that request. The same hard force/request envelope applies
at every stage. The unsuccessful future-capacity-reserve variant is not
part of the implemented controller.

The condensed map is `x_j = F_j plan + f_j`. The decision is
`z = [plan; delta]`, with one nonnegative scalar relaxation:

\[
 \begin{aligned}
 \min_{\mathrm{plan},\delta\ge0}\quad&
 J_{\mathrm{input}}(\mathrm{plan})+\rho\delta,\\
 \text{s.t.}\quad& \text{hard actuator, axle-friction, slip and domain rows},\\
 &\underline g_{\ell j}(\mathrm{plan};c_t)\ge0,\\
 &(v_x,v_y,r)_{M}=(0,0,0),\\
 &L_fV(x_0)+L_gV(x_0)u_0\le-\alpha V(x_0)+\delta.
 \end{aligned}
\]

The input cost measures deviation from the performance-reference equilibrium over the head.
A small positive input quadratic in the continuation removes degeneracy.
There is no safety slack. Define the local cruise error by

\[
 e=Sx-r_0,\qquad Sx=[d,e_\psi,v_x,v_y,r]^\top,\qquad V(x)=e^\top Pe.
\]

The current cruise reference `r0` includes reference speed and the existing
curvature-dependent lateral velocity and yaw-rate targets. Both `r0` and
`P` are fixed while evaluating the current derivative. The scheduled
continuous dynamics are `xDot = Ac*x + Bc*u + cc`; the fourth entry of `cc`
includes the declared acceleration bias. Therefore

\[
 L_fV=2e_0^\top PS(A_cx_0+c_c),\qquad
 L_gV=2e_0^\top PSB_c.
\]

Only `u0` and the nonnegative slack appear in this CLF constraint. Its QP row is

\[
 [L_gV,\;0,\ldots,0,\;-1]\,z\le-L_fV-\alpha V(x_0).
\]

`ltvBicycleModel.continuousMatrices` provides the shared continuous generator;
`stageMatrices` integrates it without changing the hard-safety prediction.
The derivative uses the actual first scheduled speed and curvature, including
a carried schedule. It is not a finite difference of future Lyapunov values
or a Taylor approximation of the old quadratic constraint.

At straight reference cruise, let `Ae = Ac(2:6,2:6)` and `Be = Bc(2:6,:)`.
The existing state scales and input weights define `Q` and `R`. The continuous
Riccati solution and feedback gain satisfy

\[
 (A_e-B_eK)^\top P+P(A_e-B_eK)=-(Q+K^\top RK)=-W.
\]

The rate is `alpha = clf.decreaseRateFraction * lambdaMin(W,P) > 0`, in
inverse seconds. `decreaseRateFraction` retains its range `(0,1]`; there is
no discrete-time cap of one on the resulting rate. `P` and the rate are
independent of the sample period. This uses the continuous-matrix syntax of
[MathWorks `lqr`](https://www.mathworks.com/help/control/ref/lti.lqr.html).
The affine performance inequality follows the CLF-QP construction in
[Ames, Xu, Grizzle and Tabuada (2017)](https://arxiv.org/abs/1609.06408).

The native solver retains the quadratic input objective and the existing
linear slack penalty directly. Slack now has units of `V` per second. The
numerical penalty coefficient remains unchanged; its previous discrete-time
tuning and closed-loop performance results do not transfer automatically.
All hard rows, their tolerances, the continuation, and the rest constraints
are unchanged. The resulting optimization is a convex QP with affine
constraints, while its safety certificate retains its predictive node scope.

The numerical decision also includes `x_1,...,x_M`. Sparse equality rows
impose `x_(j+1) = A_j*x_j+B_j*u_j+c_j`, terminal rest and the fixed final
input. Each geometric, physical and model-domain row touches only its own
state and input. `avoidanceStageQp` constructs this sparse lift. Eliminating
those states gives the condensed problem above. Acceptance independently
reconstructs states from the returned input plan.

The default solve has only zero and nonnegative cones in the general native
solver interface; it has no Lorentz cone or quadratic constraint. The
fault-injection hook receives a standard condensed QP with fields
`H`, `f`, `A`, `b`, `Aeq`, `beq`, `lb`, `ub`, plus the objective `constant`
and `defaultSolver`. It returns a decision of exactly `[plan; delta]`, with
no objective epigraph variable. No second optimization is used.

The reference and `P` may be updated at the next sample; their time variation
is not included in this frozen-reference CLF. Curvature, saturation and
scheduled-speed changes can require positive slack. A soft CLF evaluated at
sample instants does not establish continuous-time convergence, sampled
Lyapunov decrease, or a recovery deadline. Later predicted CLF values remain
diagnostics only. The hard CBF certificate and its scope are independent of
these performance claims.

## Crossing-traffic performance reference

`crossingCruiseReference` makes the longitudinal preference explicit. It
projects the target forecast onto the nominal path, includes the two rectangle
supports and clearance margin, and identifies the last occupied path node.
When the target leaves this corridor within the forecast and the stopping
station is ahead, the reference speed is the smaller of requested cruise and
distance to that station divided by clearance time plus
`performance.crossingTimeGap` (default 0.25 s). Clearance time is the node
following the last occupied node. Otherwise, the requested cruise speed is
retained. Both the input objective and continuous-time CLF use this reference.

This is a deterministic preference to yield before a crossing, not a safety
certificate or another trajectory solve. It uses nominal target motion; the
QP still enforces the complete hard continuation constraints. Its time gap
is not a plant-error bound. Targets without a forecast lateral exit, including
the oncoming steering regression, retain the original cruise preference.
Reference changes do not invalidate an otherwise applicable certificate:
the carried plan remains admissible with the sole CLF relaxation. The
controller reports the reference speed, yielding state and clearance time.

## Sound node geometry

`avoidanceSafetyGeometry` constructs physical Cartesian halfspaces. A node's
affine chart approximates the physical polyline map over a station interval:

\[
 p=o+T s+N d+r_p,\qquad \psi=\theta+e_\psi+r_\psi,
 \qquad |r_p|\le b_p,\quad |r_\psi|\le b_\psi.
\]

Hard station bounds keep the solution within `controller.stationTrustRadius`
of its geometry anchor and the finite route extent. The interval may cross
polyline vertices; it is not limited by the centerline sampling distance.
`laneGeometry.frameBounds` computes componentwise position-error bounds at both
ends of each intersecting segment and both extremes of the admitted lateral
offset. The error is affine on each such rectangle, so these corners cover
the full interval. The maximum wrapped segment-heading difference gives the
yaw bound. Both sides of a vertex are included. Hard lateral and heading
bounds apply at every node. This is a bounded inner approximation, not an
unrestricted route-domain reformulation.

For a fixed unit inertial normal `n`, rectangle support at the anchor is
maximized over the published yaw interval. An exact interval Lipschitz bound
`L` gives

\[
 \underline g_j=n^\top(p_{E,j}-p_{T,j})
 -\sigma_{T,j}-\bar\sigma_{E,j}-L|e_{\psi,j}-\bar e_{\psi,j}|-\epsilon_j.
\]

Both signs of the absolute value become hard affine rows. The position charge
includes `abs(n)'*b_p`; the yaw-support interval includes `b_psi` in addition
to the published yaw uncertainty. Thus widening a station interval also
tightens its geometric inequalities by a computed allowance. The independent
acceptance check evaluates the actual polyline pose with `laneGeometry.fromFrenet`,
rather than checking clearance only in the affine chart.

For each supplied quadratic road graph, the implementation computes its
maximum signed height over the whole admitted rectangle-parameter interval.
The only extrema needed are interval endpoints and a quadratic stationary
point. A supporting halfspace with ego rectangle support and fit/clearance
charges is therefore sufficient for every point of the footprint. It does
not interpolate a finite grid of boundary samples to assert safety.

`perceptionLimited` boundaries may omit nodes whose entire admitted rectangle
range is not covered. Diagnostics report which nodes carry rows. Consequently,
road guarantees concern the represented, covered boundaries only; unknown
road geometry is not certified. Strict partial coverage raises an error.

## Rest and complete target continuation

The schedule reaches zero speed, while tire denominators use the positive
regularization floor. At zero schedule speed,

\[
 x_R=[s_R,d_R,e_{\psi,R},0,0,0]^\top,
 \qquad \pi_R=[0,-b/\gamma]^\top
\]

is a fixed point of the same scheduled bicycle; `b` is the declared constant
longitudinal bias and `gamma` the configured input gain. The last input is fixed to this policy and checked against
actuator and axle constraints. The policy, zero-speed schedule and resting
frame can be appended indefinitely. Rest remains valid at nonzero lateral
offset and heading error within the admitted geometry domain.

For targets, the last node has a hard halfspace separating the resting ego
from the target's complete future center-trajectory support. A target
circumradius covers every future yaw; the resting ego retains its directional
rectangle support. `targetPrediction.futureSupport` supplies the nominal
complete-future support. Constant position uncertainty is added directionally.
A direction with persistent velocity or acceleration uncertainty is treated
conservatively as having infinite future support and cannot certify rest.
This can reject cases where nominal motion would dominate the uncertainty.
A finite error radius at the rest time is never substituted for future support.

The halfspace is sufficient, not necessary. A safe resting point inside the
convex hull of a target's complete orbit can remain outside this certificate
class. More optimization starts would not repair that restriction.

## Uncertainty and sampling contract

The implemented persistent certificate is an exact-model trajectory
certificate. Nonzero ego estimation radii or model/disturbance rate bounds
raise `unsupportedCertificateUncertainty`. A robust feedback tube with a
robust terminal invariant set has not been implemented. The predictor itself
now propagates all boxes through `r(j+1) = abs(A(j))*r(j) + Ts*w`; the former
first-future-node-only update has been removed. Propagation alone does not
supply terminal robustness, so it is not used to label such inputs certified.

The estimator adapter now publishes its time-varying state-time enclosure
instead of fixed configured state-error margins. The controller validates
timestamps and availability, converts the relative target bound to absolute
state bounds, and propagates future target uncertainty from declared motion
limits. These interface changes do not remove the terminal restriction above.
See [ESTIMATOR_BOUND_INTERFACE.md](ESTIMATOR_BOUND_INTERFACE.md) for the field
contract, derivation and distinction between current estimation and prediction.

Static target uncertainty is admitted when the directional rows and complete
future support remain feasible. State, actuator execution, target finite-node
overlap, complete target support, route and model assumptions must agree with
the carried certificate before its shift can be treated as applicable.
A changed observation or environment requires admission again. When the
track, route and controller configuration still identify the same operation,
the old shifted inputs supply the single admission proposal. Its model uses
the same deterministic measured-speed cruise/brake template as initial
admission. This prevents an aging braking schedule during repeated cruise
readmission and avoids feeding unexecuted optimized speeds back into the
entire model. The template is a declared local model choice; it does not
certify approximation error for the nonlinear plant. Exact certificate reuse
still shifts the original model verbatim. All geometric rows are rebuilt
from the new environment. A proposal can supply a
fallback only after passing the complete current certificate checks. An
unrelated track does not inherit that proposal. This preserves maneuver
memory without claiming that an expired certificate still applies. Successful
re-admission does not retroactively establish recursive feasibility across
the change. The higher-fidelity plant is outside the exact-model proof.

The public safety scope is `declaredModelPredictionNodes`; collision
diagnostics state `predictionNodesOnly`. There is no intersample collision
claim. The regression suite deliberately includes a fast between-node
crossing that is admitted under this convention, and a collision at the next
node that is rejected. Continuous-motion protection needs additional proven
motion bounds or a sampled-data certificate.

## Acceptance and fallback

`solveHardCbfClf` calls the native Clarabel backend once. Its primal decision
contains inputs, CLF slack and explicit future states. Equality slacks lie in
a zero cone; all inequality slacks, including the affine CLF row, lie in a
nonnegative cone. The objective is quadratic, so this is a standard QP.
The feasibility and optimality targets are each the minimum of 1e-9 and their
respective configured tolerance. Physical acceptance remains a separate
absolute check. There is no retry with another solver or geometric start.

Run `addpath('scripts'); buildAvoidanceSocpSolver()` once before experiments.
The validated Linux recipe builds the pinned Apache-2.0 Clarabel C API and
three small MEX bridges under `solver/clarabel/matlab`, using the committed
Cargo lockfile. No download, compilation or additional optimization occurs
within a sample. QDLDL uses one solver thread. MATLAB path projection and
frame-bound implementations remain available when their optional MEX files
are absent. The native solver backend itself is required; its existing build and MEX names are retained. See
[CONTROLLER_RUNTIME.md](CONTROLLER_RUNTIME.md) for build and timing details.

For its returned input plan, the sole
performance slack is reconstructed as `max(0, LfV+LgV*u0+alpha*V0)`, its analytic
minimum at that fixed plan. This prevents a numerical slack residual from
rejecting an otherwise safe input and reports the actual performance loss.
It changes no actuator or hard-safety variable. `certifyAvoidancePlan` checks
the reconstructed vector for dimensions, finite real values, actuator bounds, all hard
rows, rest equalities, the continuous-time CLF and Cartesian rectangle
clearance at every node. The checked vector is stored and committed without
clipping. The same checks apply to the carried witness.

A finite iterate from iteration-limit or numerical termination may be used
if it passes acceptance. Its solver exit flag remains visible; no optimum is
claimed for that status. A positive exit flag alone cannot authorize an input.
If no returned candidate passes, the already checked carried witness supplies
the command without another optimization. If no such witness exists, the
controller reports failure and issues no command.

Acceptance uses the declared numerical tolerance (ten times solver constraint
tolerance, with a roundoff floor for the CLF). Residuals are reported. These
are numerical checks, not interval-arithmetic proofs of exact equalities or
infinite-time invariance. In particular, a small nonzero terminal velocity
residual must not be interpreted as an exact physical resting state.

## Conditional recursive-feasibility argument

In exact arithmetic, assume initial admission, sound represented geometry,
exact execution of the declared scheduled model, unchanged applicable road
coverage, shift-consistent target prediction with contained complete future
support, and an admissible rest policy. Then:

1. The current stored plan is a feasible witness. A geometric replacement is
   adopted only if it admits the witness.
2. Any accepted feasible plan satisfies the declared node constraints.
3. After applying its first input, all overlapping state transitions use the
   identical shifted matrices, affine terms and two-input sequence. No speed,
   tire or heading constraint becomes stricter at a head/continuation boundary.
4. Appending the resting model/policy preserves the endpoint and terminal
   separation. The unbounded nonnegative CLF slack admits this continuation.
5. The next convex problem therefore has a feasible witness. Solver failure
   does not require another optimization to preserve the certificate.

This is an induction on augmented controller state. It establishes neither
optimality, maneuver completeness, global navigation, nonlinear-plant safety,
nor continuous-time safety. A maintained local domain may brake or wait while
another safe passing maneuver exists. An application requiring a particular
passing side, liveness or recovery deadline must supply and validate that
behavioral requirement; there is no hidden global planner in this controller.

## Relation to literature

Zhang, Liniger and Borrelli's [Optimization-Based Collision Avoidance](https://arxiv.org/abs/1711.03449)
uses duality to obtain smooth nonlinear collision constraints. It does not
make unrestricted trajectory optimization convex. Liu, Lin and Tomizuka's
[Convex Feasible Set algorithm](https://arxiv.org/abs/1709.00627) motivates
feasible inner approximations; convergence for repeated optimization of a
fixed problem is not a recursive-control theorem. Leeman et al.'s
[Predictive safety filter using system level synthesis](https://proceedings.mlr.press/v211/leeman23a.html)
illustrates that disturbance handling needs a feedback/error-containment
construction. Those references do not independently certify this implementation.

## CLF-QP conversion validation (2026-09-06)

The isolated commit tree passes all 120 tests in `continuousTimeClfTest`,
`sparseAvoidanceQpTest`, `collisionAvoidanceControllerTest`,
`collisionAvoidanceControllerConfigTest`, `ltvBicyclePredictionTest`, and
`controllerEstimatorBoundsTest`. Factory Code Analyzer reports zero findings
in all 11 changed MATLAB files; the core controller remains at 18 sources.
The new tests compare continuous Lie derivatives with symmetric held-flow
finite differences on straight and curved schedules with nonzero acceleration
bias, verify the Riccati rate, sample-period independence, zero-error behavior,
slack rejection, and the equivalent public `quadprog` solve.

A before/after fixture with target and road constraints gives bitwise-identical
hard matrices, bounds, terminal equalities and input objective. Four held-flow
cases, including zero speed and both curvature signs, also match bitwise.

The full working-tree suite returns 362 passes and eight failures among
370 tests. All eight failures reproduce on the pre-edit snapshot: six
estimator-scenario tests and two pre-existing untracked stationary-pose
uncertainty tests. Seven of those failures are also incomplete. The initial
MCP call timed out after 300 seconds, but its saved result was recovered;
the isolated commit checks completed separately in a fresh MATLAB process.
These checks do not establish new closed-loop performance or runtime claims.

The measurements below describe earlier SOCP revisions; they have not been
reproduced for the continuous-time CLF-QP.

## Historical SOCP runtime validation before the CLF-QP conversion

The sparse native implementation completes both 10 s crossing experiments
and their own nominal counterfactuals with all functional criteria passing.
Avoidance median times are 37.336/38.693 ms, with 95th percentiles
42.203/42.874 ms. Five straight and one arc call exceed 50 ms, with maxima
119.561/75.958 ms. The strict real-time requirement is still unmet. The
maximum hard-row violation is 4.25e-11, with one SOCP and no fallback at each
sample. Most native terminations are almost-solved or numerical; their
accepted plans do not establish exact optimality. The broad regression and
affected-estimator rerun yield 298 passes and six existing uncertainty-domain
failures among 304 tests. See [CONTROLLER_RUNTIME.md](CONTROLLER_RUNTIME.md)
for the equivalent transcription, benchmark method, build recipe, residual
comparisons, startup/acquisition limits and complete numerical status scope.

## Experiment-driven validation before runtime optimization (commit 856be7c)

The repaired implementation passes 119 focused tests and has zero Code
Analyzer findings across its 24 changed MATLAB files. A frozen full-suite run
on the current estimator base produced 284 passes, six failures and two
additional skips among 292 tests. The six failures are estimator-integrated
scenarios supplying nonzero ego/model uncertainty outside this controller's
declared certificate domain. Two historical curb-data tests skip because
their recorded dataset is absent. The full suite is therefore not green.

Both complete 10 s PassVeh14DOF crossing trials and their nominal
counterfactuals meet the functional experiment requirements. Their minimum
sampled SAT gaps are 1.4500 m (straight) and 1.7902 m (400 m arc). Both recover
the specified cruise tolerances throughout the final second. Every successful
sample uses one SOCP with no fallback. Maximum measured controller times are
0.95233 s and 0.982513 s, so neither meets the 0.05 s deadline. These are offline
plant experiments, not real-time or continuous-motion safety demonstrations.
See [the experiment record](../scripts/CONTROLLER_DESIGN_EXPERIMENTS.md) for
the failed candidates, unchanged acceptance criteria and complete metrics.

The separate declared-model oncoming test with input gain 0.80 solves once,
then forces solver outages through rest. Its minimum rectangle clearance is
0.250000291 m for a 0.25 m requirement, and its final velocity residual is
2.77e-14; the rotated case also passes (2.82e-14). This retains steering-based
continuation coverage alongside the crossing trials' longitudinal yielding.
It does not extend the certificate theorem to the nonlinear plant.

## Initial redesign validation (commit 85f9e34)

MATLAB R2026a Update 3, with Control System Toolbox and Optimization Toolbox:

```matlab
results = runtests({'tests/collisionAvoidanceControllerTest.m', ...
    'tests/ltvBicyclePredictionTest.m', ...
    'tests/collisionAvoidanceControllerConfigTest.m'});
assertSuccess(results)
addpath('scripts', 'controller', 'config');
trial = runCertificateContinuationScenario();
```

All 65 focused tests passed. They cover explicit state, exact-model shift
through rest, re-admission after target changes, unsafe successful solver
outputs, full-horizon error propagation, zero-speed curved-route equilibrium,
node/inter-node distinction, and evasive steering with road constraints.
Static Code Analyzer checks reported zero issues in the changed MATLAB files.

The deterministic continuation scenario uses `Ts = 0.05 s`, `N = 12`,
`Nb = 74`, ego speed 15 m/s, an oncoming target at 30 m longitudinal and 1.5 m
lateral offset moving at 5 m/s, 4.8-by-1.9 m rectangles, and road boundaries
at lateral offsets +/-6 m. It solves once, then injects an empty solver
result at every later attempt through three samples beyond terminal rest.
Over 90 sampled states, minimum physical rectangle clearance was
0.317602565 m for a 0.25 m requirement, minimum certified road margin was
3.944734069 m, maximum lateral displacement was 0.7602 m, maximum shifted
state discrepancy was 9.95e-14, and final velocity-state residual was
9.40e-13. The rotated-frame regression also passed. These results demonstrate
steering continuation on the declared discrete model, not continuous-motion
or high-fidelity plant safety and not a runtime/deadline result.

A broader `runtests('tests')` working-tree snapshot produced 237 passes and
10 failures among 247 tests (nine failures were also incomplete). The
failures were scenario checks requiring 0.1 s while the existing default is
0.05 s. The unchanged scenario guards and baseline configuration were
compared with commit `d14bb9519967093d8dd8ed5510b5a421fc7bfa46` to identify that
pre-existing mismatch. The snapshot preceded the final focused additions and
concurrent estimator edits; it is not a full-suite result for those unrelated
edits. Estimated-state integration also remains outside the exact-ego
certificate domain, even if its scenario timing guards are repaired.
