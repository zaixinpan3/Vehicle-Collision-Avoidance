# PCBF, lane CLF and freely placed terminal continuation

## Scope

The controller uses one nonlinear ego bicycle model and one known target
trajectory. A Huang-style primary objective minimizes the sum of safety
slacks over a fixed MPC prefix. Once zero safety slack is attained, a second
stage optimizes the soft lane CLF slack while preserving zero safety slack.
Li-style polygon support duals and sequential convexification supply
numerical search directions. Positive slack guides feasibility restoration;
only a zero-prefix-slack candidate satisfying all original hard constraints
can authorize an input. An initialization seed need not satisfy these checks.

The terminal condition is an implicit backward-reachable tube, represented
by a hard completion trajectory and an indefinitely admissible endpoint
family. Its construction and numerical scope are described below. The
controller shifts the previous solution as the next numerical initialization,
including its slack budget. The common nonlinear admission test applies to
initializations and solver iterates alike. Finding a safe continuation alone
does not complete the CLF stage.
There is no separate runtime backup controller or nominal-versus-backup selector.

Road boundaries are currently excluded from the controller problem, including
the finite prediction and indefinite endpoint admission. The given path still
defines the nominal lane/CLF reference. The terminal core has a free world pose
and is constructed at straight cruise, independently of the given-path curvature. This is one controller;
there is no road-constraint switch or additional driving mode. Optional road
widths are metadata, and changing them does not invalidate a retained plan.

## Model, clock and input memory

The ego state is `x = [X; Y; psi; vx; vy; r]`, with input
`u = [frontWheelSteeringAngle; brakingRatio]`. Combined-slip modified Fiala
forces, aerodynamic drag and rolling resistance enter the nonlinear bicycle
model. The positive longitudinal-speed model domain is hard.

The nominal transition `f_h` is the global-coordinate held-input RK4 map in
`nonlinearBicycleModel.sample`. One even mesh is used for a full hold and its
two half holds. This is a discrete numerical prediction model, not an exact
continuous-time plant flow. The augmented state and transition are

\[
y_j=(x_j,w_j),\quad w_j=u_{j-1},\qquad F(y_j,u_j)=(f_h(x_j,u_j),u_j).
\]

The default integration step is 0.005 s. At the default 50-ms hold this
defines ten integration intervals, shared by prediction, near-contact checks
and terminal-core construction. A coarser former mesh missed short corner
contacts between its nodes in the CLF-recovery campaign. This consistent mesh
refinement improves numerical resolution; it is not a continuous-time proof.

Terminal membership therefore has eight coordinates. It retains position
and previous input. Magnitude and slew restrictions are imposed in the
optimization and in nonlinear evaluation. The terminal policy is never
clipped after selection.

The target is stored once as
`q = [X; Y; psi; V; A; beta; lr; halfLength; halfWidth; offsetX; offsetY]`.
Its tangential acceleration `A` and sideslip `beta` are constant:

\[
\dot V=A,\quad \dot\psi=V\sin\beta/l_r,\quad
\dot p=V[\cos(\psi+\beta),\sin(\psi+\beta)]^T.
\]

`targetFlow` integrates this model analytically using traveled signed arc
length `V*t + A*t^2/2`. A braking target continues through zero signed speed
and retraces its path. No stopping rule or constant independent yaw rate is
introduced. Eliminating the autonomous target trajectory from the numerical
variables preserves the joint ego/target model.

Target directions use `sinpi(angle/pi)` and `cospi(angle/pi)` so that cardinal
headings have exact zero transverse components. The straight endpoint test
projects motion using relative headings in the road frame, avoiding cancellation
of world-coordinate dot products. No small physical acceleration is thresholded
to zero; nearby non-cardinal headings retain their transverse motion.

Every target prediction uses the original epoch and an absolute time.
Regular mesh nodes use an **absolute integer subsample index**.
Nodes and midpoints retain the same half-sample arithmetic;
additional collision checks use the interior nodes of the half-hold integration
mesh, when present. This preserves repeated absolute-time evaluations
under a shift. Adaptive off-mesh cuts use their actual absolute time without
rounding to a mesh node. Later observations cannot reinitialize the forecast. A changed
target trajectory, reference path, physical constraint set or prediction model requires
an explicit new problem (`previousState=[]`). A timestamp gap is rejected.
Equivalent headings across the +/-pi representation boundary are compared
modulo one turn and do not invalidate or restart the fixed target epoch.
A changed measured ego state or input memory invokes fresh feasibility
restoration and disables the shifted slack guarantee for that transition.

## The nonlinear MPC problem

Let `N = controller.horizonSteps`, let `k` be the absolute sample index and
let `M` be the endpoint selected by the current solution. The moving minimum
prediction length is

\[
H_{\min}=\min\!\left(H_{\max},N+\left\lceil
T_{\rm recovery}/h\right\rceil\right).
\]

The first `N` stages form the PCBF prefix. Stages `N,...,H_k-1` form the
completion tail. An initially longer certificate may shorten down to
`H_min`; afterward terminal-policy inputs extend its endpoint as the window
moves. The configured three-second completion allowance is preserved rather
than consumed as an implicit deadline. This does not fix an endpoint position,
heading, path phase, or nominal-recovery time.

At each stage, collision constraints are evaluated at its start and midpoint.
Near a target, the RK4 integration nodes are also checked with the declared
collision margin. The nominal pose between adjacent integration nodes is a
linear interpolant of position and unwrapped heading. The known target keeps
its exact absolute-time flow. A relative-motion Lipschitz bound certifies
separation throughout each interpolation interval; unresolved intervals are
bisected and their midpoint checks become additional convexification rows.
This is constraint generation within the same controller, not an execution
fallback or a separate verifier of the physical vehicle.

Let the endpoint body distances on an interval of duration `dt` be `d_a,d_b`.
For the ego-pose interpolant, a valid distance Lipschitz constant is

\[
\begin{aligned}
L={}&\|v_E-v_T(t_{\rm mid})\|
+\tfrac12\Delta t\bigl(|A_T|+V_{T,\max}^2|\kappa_T|\bigr)\\
&+|\omega_E|r_E+V_{T,\max}|\kappa_T|r_T .
\end{aligned}
\]

Here the radii include rectangle offsets. Common translation cancels rather
than causing unnecessary refinement. A sufficient lower bound is
`min(d_a,d_b,(d_a+d_b-L*dt)/2)`. With sampled margin `delta`, a leaf is accepted
only when this lower bound exceeds `delta/2`; every newly evaluated midpoint
must still satisfy the full sampled margin. For positive `delta`, refinement
has a finite resolution reserve: if all checked distances are at least
`delta`, subinterval length below `delta/L` suffices. Zero margin still
requires strict positive separation; unresolved floating-point intervals
are rejected. Small roundoff padding is included, without claiming formal
exact-real arithmetic validation.

A whole-hold travel bound from the actual mesh pose increments screens distant
encounters. Prefix stages use one shared nonnegative search slack per stage;
the completion tail has zero slack. Input limits, slew limits and velocity
bounds at nodes and midpoints remain hard. The endpoint has hard safety
constraints and its indefinite certificate. Only a complete original
nonlinear evaluation with every required interval established can authorize
execution. Newly introduced cuts use interpolated variational derivatives;
no off-mesh target time is rounded to a coarser integration node.

The nominal interpolant is distinct from the tightly integrated ODE45 plant.
The interval argument does not bound their difference or observer/model error.
The independent plant replay remains necessary experimental evidence.

The signed collision predicate uses nonnegative support dual variables for
both oriented rectangles and a unit separating normal. During overlap, a
signed support gap supplies a restoration direction. An unsigned rectangle
distance of zero is never used as evidence of separation. The configured
additional physical collision buffer defaults to 0.006 m and can be explicitly
set to zero. It applies at the prediction constraint samples and in the
terminal separation certificate; it is not a continuous-time clearance guarantee.
The terminal interval construction encloses pose deviations at every RK4
integration node. Convex interpolation preserves those endpoint-error bounds;
a circular nominal orbit adds its chord-sag padding, while a straight core
needs no additional padding. Its reference-orbit padding also takes the maximum nominal
position/heading defect over those nodes. This covers the same additional
numerical sample times used by finite-horizon collision refinement without
changing the terminal feedback policy or relaxing membership.

With the union of endpoint families `B_j` defined below, the terminal tube is represented
implicitly by

\[
K_j=\operatorname{Pre}_j(K_{j+1}),\quad K_M=B_M,\qquad
K_j=B_j\quad(j\ge M).
\]

Here `Pre` uses the nonlinear RK4 transition, hard physical constraints and
zero-slack stage safety. The optimized continuation from `y_(N|k)` to
`B_(k+H_k)` is a membership witness for `K_(k+N)`. Its first input is the
terminal selector. The final input is part of endpoint membership; there is
no separate conservative input-radius band.

## Indefinite endpoint construction

`terminalContinuation` constructs one eight-dimensional local straight-cruise
core per model configuration. The original lane LQR, including its curved-road
trim, remains a separate nominal CLF/preference ingredient. It is not an
invariance proof and changing terminal placement never changes that reference.

The core construction linearizes the **actual global RK4 map** at straight
cruise, includes input memory, transforms the successor into a moving reference
frame, and designs a discrete augmented feedback gain `K`. Let `P=R^T R` be
its positive definite metric. A rigid placement `g=(p_g,theta_g)` gives

\[
B_j(g)=\{y:\|R e_j(y;g)\|_2\le r\},\qquad
\kappa_{B,j}(y;g)=u^r+K e_j(y;g).
\]

The reference advances from its stored absolute endpoint epoch using the
straight sampled RK4 pose increment. Its body-state trim need not be
numerically exact: the construction includes its successor defect below.
The pose is fixed for a selected solution and its shifted feasibility witness.
No fixed endpoint position, lateral position or heading is prescribed by the
road. The general reference utility still supports phase and curved trim for
its invariant-core tests; the controller has no phase decision, core-mode
selector or curved terminal orbit.

A cached enclosure of the closed-loop RK4 Jacobian establishes a sufficient
contraction bound `gamma`, including the error in the computed inverse of
`R`. A separate center evaluation bounds the reference defect `d`. The
radius is reduced until all these conditions hold:

\[
\sup_{\|Re\|\le r}\|R D\Phi(e)R^{-1}\|_2\le\gamma<1,
\qquad \|R\Phi(0)\|_2\le d,
\qquad \gamma r+d<r.
\]

Consequently `Phi` maps the whole selected neighborhood into itself. The
same construction bounds input magnitude, `K e - (w-u^r)`, velocity bounds
and the RK4 internal domain. It fails explicitly if it cannot establish a
nonempty admissible neighborhood. It never infers nonlinear invariance from
LQR eigenvalues or a finite collection of sampled states.

The global bicycle RK4 map is equivariant to rigid planar translations and
rotations: applying a fixed pose transform before a step gives the same result
as applying it after that step. Body velocities, tire forces and actuator
constraints are unchanged. Each fixed rigid placement is such a transform of
the original reference, so its moving-frame closed-loop error map `Phi` is the
same. The gain, norm, defect and contraction bounds therefore apply
to every placement without a new enclosure computation. This argument uses the
spatially homogeneous nominal model and absence of road-edge constraints;
it does not translate the target or discard its separation requirement.

The enclosure implementation uses outward-padded double interval operations,
Taylor remainders for sine/cosine and exponential, analytic differentiation
through RK4, and a checked positive-definiteness bound for the proposed
matrix norm. The inverse residual is handled by a Neumann-series bound.
The endpoint is restricted to a positive-speed, unsaturated local Fiala
chart; this restriction does not restrict the prefix to that chart. These
are sufficient local bounds and can yield a small, conservative seed.

### Eliminate endpoint pose analytically

Partition the moving-frame error into pose `a` (three coordinates) and intrinsic
error `z=[vx-vr; vy; r; uPrevious-uReference]` (five coordinates). Completing
the square gives

\[
\begin{aligned}
G&=-P_{aa}^{-1}P_{az},&
S&=P_{zz}-P_{za}P_{aa}^{-1}P_{az},\\
e^TPe&=(a-Gz)^TP_{aa}(a-Gz)+z^TSz.
\end{aligned}
\]

Every pose error can be realized by translating and rotating the reference.
Thus the union of ego-only core memberships has the exact reduced test
`||chol(S)*z|| <= r`. The implementation chooses the minimizing error `a=Gz`
and reconstructs its reference pose analytically:

\[
\theta_g=\psi-a_3,\qquad p_g=p-R(\theta_g)a_{1:2}.
\]

`terminalContinuation.fit` returns this pose and its derivative with respect to
all eight augmented coordinates. The endpoint conic row and endpoint correction
use the five-dimensional norm; nonlinear admission still checks the original
full eight-dimensional membership at the reconstructed pose. No search grid,
pose penalty, separate maneuver mode or additional controller is introduced.
After a new solution is accepted its pose is fixed when shifting and appending
that solution, rather than silently refitted during the invariance argument.

### Indefinite target separation

The chosen core must also satisfy a sufficient all-future target-separation
bound. No assumption says that the target departs after a single encounter.
Straight target motion uses extrema of same-time relative quadratic progress,
including acceleration, signed reversal and arbitrary ego heading. For a
turning target, its whole spatial orbit is enclosed by a disk of center `c_T`
and outer radius `R_T`. The straight ego reference is a ray `p_g+v_g*t`.
Writing `b_E` for the ego footprint's circumscribed radius and `epsilon_p` for
the enclosed integration-node position deviation, the sufficient bound is

\[
\min_{t\ge0}\|p_g+v_gt-c_T\|-R_T-b_E-\epsilon_p-d_{min}\ge0,
\qquad
t_* = \max\{0,-(p_g-c_T)^Tv_g/\|v_g\|^2\}.
\]

The finite completion still checks ego and target at matching absolute times.
The ray only starts at its endpoint. Earlier crossings of the target orbit are
allowed when time-aligned rectangle constraints hold. All-future disk/ray
separation is conservative, but allowing free terminal position **and heading**
lets the ego leave a returning target's orbit. The old phase-only circular
core could not do this: changing phase never changed the intersecting circles.

The active ray or separating-axis branch supplies the analytic derivative with
respect to the reference pose. Composing it with the fit derivative includes
endpoint position, heading, velocity and input-memory sensitivities in the
SCvx separation row. The complete nonlinear certificate is rechecked afterward.
Road edges do not enter this certificate.

For a selected pose with margin `c_j(g)>=0`, membership in `B_j(g)` admits a
safe indefinite continuation under its fixed policy. The abstract union over
all such poses is control invariant when the selected pose is retained. The
implemented analytical fit chooses one pose and checks its separation; unlike
ego-only membership, this need not find every pose that jointly satisfies
target separation. It remains a sufficient admission test, not a complete
viability-kernel computation. The finite completion length also remains bounded.

If no seed endpoint passes admission within `maximumHorizonSteps`, the bounded
rollout remains an uncertified search seed. Search may repair it, but original
terminal membership, target separation and zero collision slack remain
necessary for execution. Failure of the sufficient test is not a proof of
physical unavoidability.

## Moving-target flow initialization

The field is an initialization heuristic, not an extra safety constraint or a
vehicle controller. In an ellipse frame `q = B^-1 (p-c)`, define

\[
w=B^{-1}(v_0-\dot c-\dot Bq),\qquad
M(q)=(1+\|q\|^{-2})I-2qq^T\|q\|^{-4},
\]
\[
v_F=\dot c+\dot Bq+B\left(M(q)w+\gamma Jq/\|q\|^2\right).
\]

The ideal exterior field has zero relative normal velocity on the moving
boundary. This property is not required of an initialization trajectory.
Inside the envelope, denominators are bounded by one so an infeasible seed
remains numerically evaluable. The current rollout uses a circular envelope
containing both vehicle bodies and the existing collision buffer. Its center
velocity includes rotation of a nonzero target rectangle offset. A circle
needs no shape-rotation transport; the general field interface also handles
rotating and deforming ellipses.

At each future hold the target is predicted from the original epoch. A bounded
three-second preview decides whether to use the flow guide or nominal lane
feedback. The flow receives a tangential bias of magnitude
`0.6 * max(relativeGuideSpeed, 0.5 * referenceSpeed) / envelopeRadius`.
The temporary heading and speed guide feed the existing lane-feedback gain.
Speed guidance is bounded between half cruise speed (at least 1.5 m/s) and
cruise speed. A longitudinal-input preference of +/-0.35 is applied before
computing the front tire's available lateral capacity
\(F_{y,\max}=\mu_f F_{z,f}\sqrt{1-\beta^2}\). The steering preference uses
the actual Fiala adhesion branch. Its force fraction satisfies
\(f=1-(1-q)^3\), where \(q=C_f|\tan\alpha_f|/(3F_{y,\max})\).
Inverting this relation at the existing 0.8 force-fraction preference gives

\[
|\alpha_f|\le\arctan\left(
\frac{3F_{y,\max}}{C_f}\left[1-(1-0.8)^{1/3}\right]\right).
\]

The steering interval is centered at the current front-axle zero-slip heading,
not zero steering. The previous kinematic acceleration-to-steering cap could
already lie beyond Fiala saturation at low speed, making the front-tire
steering derivative zero and placing the CLF search on an uninformative branch.
The new cap is still only a seed preference, not an optimizer constraint or a
safety certificate. Final amplitude and slew clipping has priority and may
override the preference. The nonlinear model generates every next seed state.

The actual state/input rollout supplies dynamics, tire and collision expansion
points. The input regularizer in the convex subproblem penalizes the change
from this anchor; the nominal lane/CLF objective remains tied to the original
given path. The retained nonlinear cost has no separate input-amplitude term;
input proximity regularizes the local search step only. After the nominal
recovery prefix, seed construction fits the free core and uses its feedback to
settle body velocities and input memory. Flow guidance remains active while
the preview encounters the target before that completion stage. This is only
initialization of the same optimization problem. A feasible initialization
supplies the zero-safety anchor for CLF optimization. Small measured-successor
differences repair its shifted inputs; fresh flow candidates are considered for
cold initialization or a state discrepancy larger than 1e-4 in the raw state
infinity norm. This threshold is a search-cost heuristic, not a safety tolerance.
Every issued candidate undergoes the same nonlinear admission checks.

## Sequential convexification and acceptance

A lane-feedback rollout supplies the baseline. When it is inadmissible and
there is no usable prior reference, at most two moving-target flow rollouts
provide opposite passing biases within the same controller-call budget.
A seed may collide or fail terminal admission; evaluated nonlinear violation
selects the restoration anchor. A fully admissible seed enters the CLF stage,
unless its actual CLF residual already meets the lower-bound stopping test.
The endpoint index belongs to the selected rollout. Each SCvx step uses
variational RK4 dynamics and linearized support-dual geometry. Its endpoint
is the exact reduced five-dimensional **2-norm cone** obtained by eliminating
free pose. The separate future-separation row differentiates the fitted pose
through the endpoint state and final input. No scalar phase variable remains.
`coneprog` solves the primary safety-slack problem. The CLF stage first tries
zero CLF slack with tracking and input-proximity costs inside that slice. If
the local zero-slack slice is infeasible, a separate linear objective minimizes
CLF slack alone. Tracking costs cannot purchase a positive CLF relaxation.
After a positive-slack solve, the predictive cost also resolves the later-input
nullspace while fixing the minimizing first control. This preserves its actual
first-step CLF value. Positive first-step stationarity does not skip this
predictive cost attempt. Raw and cost-refined nonlinear candidates are retained
under the same admission and comparison rules.
On the zero-slack slice, each tracking/input stage has a small squared-norm
epigraph. There is no squared CLF-slack penalty or associated weight.

A revalidated initial or shifted witness proves that the nonnegative primary
safety objective already has minimum zero. It skips the redundant primary
solve, but still enters the CLF stage when its actual CLF residual is positive.
A nonlinear constraint correction can attain the zero lower bound without
another conic solve. Otherwise the secondary conic programs remain available.
Its nonlinear hard residual and prefix safety-slack sum must both be exactly
zero. Safety admission and secondary-stage completion are distinct records.

When no witness exists, every finite primary conic result is checked against
the nonlinear constraints while time remains. Raw, endpoint-corrected and
secondary candidates are compared using their nonlinear evaluations. Endpoint
correction may follow a working trajectory with decreasing terminal error, but
retains the best complete evaluation independently. A later solver result or
correction cannot overwrite a better evaluated candidate merely because it
exists. An admitted candidate always has priority over an inadmissible one.

The numerical restoration score is

\[
\Theta=\sum_{i<N}\xi_i+
100\left(\sum_{i\ge N}\xi_i+v_T+v_G+v_P\right),
\]

where `v_T` is terminal-norm excess, `v_G` is the maximum endpoint/future
separation deficit, and `v_P` is the maximum physical/input/slew violation.
This heuristic score guides search and matches the completion-slack sum in the
elastic subproblem; it is not the PCBF value or a distance with uniform units.
The original hard residual is still the maximum original violation. Equal
scores prefer smaller terminal excess. Among admitted candidates, smaller
actual nonlinear CLF slack has priority; equal slack is broken by the nonlinear
tracking cost. That cost excludes the input-proximity term, which depends
on the current linearization anchor rather than the trajectory alone.

A feasible primary returns to the outer loop as the new linearization anchor
for the CLF stage. An improving but inadmissible primary is also relinearized
before attempting more restoration. A conic direction is checked by geometric
backtracking, halving its amplitude within the same time budget (at most 20
halvings). Before whole-trajectory backtracking discards a useful first input,
a nonlinear correction fixes that first input and repairs the later trajectory.
This correction is available at feasible anchors as well as during restoration.
A secondary trial is admitted only after the original nonlinear constraints
pass. A slight same-state slack improvement does not complete the CLF stage.
Poor actual versus predicted CLF improvement contracts the trust radius;
admitted progress remains available for relinearization. Positive prefix
collision slack remains a search residual and does not authorize execution.

The soft first-step CLF is

\[
W(e_{1|k})-(1-\alpha)W(e_{0|k})\le\rho,\qquad\rho\ge0.
\]

For the affine first-successor error \(\widehat e_1=e_1+J_ed\), the subproblem
retains the full convex quadratic \(\|L\widehat e_1\|_2^2\), where \(W(e)=\|Le\|_2^2\).
It does not substitute a tangent plane of \(W\). With
\(c=(1-\alpha)W(e_0)\), the exact second-order cone representation is

\[
\left\|\begin{bmatrix}2L\widehat e_1\\\rho+c-1\end{bmatrix}\right\|_2
\le\rho+c+1.
\]

This is convex in the affine error; the actual nonlinear successor is still
checked after the solve. On the zero-slack slice, a direct norm ball avoids
subtraction of nearly equal rotated-cone constants near nominal trim:

\[
\|L\widehat e_1\|_2\le\sqrt{(1-\alpha)W(e_0)}.
\]

Within that slice the objective is

\[
\frac{1}{H_k}\sum_{i=0}^{H_k-1}\left(
\|L\widehat e_{i+1}\|_2^2+0.01\|D_u(u_i-\bar u_i)\|_2^2\right),
\quad D_u=\operatorname{diag}(\sqrt{w_\delta},\sqrt{w_\beta}).
\]

Here `bar u` is the current linearization input, not zero. The given path
continues to define the tracking cost and CLF. On an admitted safety anchor,
all collision slack upper bounds and terminal elastic variables are zero.
If the local zero-CLF slice cannot be solved, the relaxed problem minimizes
`rho` alone, bounded above by the incumbent's actual slack. Nominal cost is
not traded against positive relaxation. A subsequent predictive-cost solve
fixes the attained first input and its modeled relaxation cap (with a roundoff
allowance), so later controls are not left arbitrary by the one-step objective.
These are stages of the same predictive
controller, not selectable driving methods or backup policies.

The change of units is
`s = max(W_initial, rho_anchor, sqrt(eps))`, `rhoHat = rho/s`.
The CLF cone uses squared-norm units divided by `s`; the zero-slack cone uses
the corresponding norm units. The tracking objective is scaled by
`max(1, mean(W_anchor_stages))`. These positive scalings preserve the mathematical
feasible set and objective ordering, while resolving small residuals near trim.
The retired `clf.relaxationWeight` configuration is no longer accepted.

The lower-bound stop tests the actual nonlinear residual against only
`64*eps(max(1,W_initial,W_next))`. It no longer compares a squared penalty with
an optimization tolerance. The raw residual remains reported without being
rounded to zero. This is a floating-point allowance, not a proof of exact-real
arithmetic or a material permitted CLF violation.

A complete admitted zero-slack witness attains the global lower bound of the
nonnegative CLF objective, so further cost refinement is unnecessary. Positive
relaxation can instead terminate at a checked *local* stationarity condition:
either conic and actual reductions are both below the scaled optimality
tolerance after an optimal relaxed solve, or the feasible first control has
small projected gradient for the one-step nonlinear objective with actuator
and slew bounds. Either positive-slack stop also requires attempting the
predictive-cost refinement. The latter is stationary also on the more restricted
predictive feasible set. Neither test proves a global positive minimum or
that a disconnected zero-slack solution does not exist. A finite iteration
or time limit preserves the admitted solution while reporting incomplete CLF
optimization. `clfStageAttempted` includes nonlinear constraint correction;
`solverCalls` counts conic and correction-QP calls, not analytic Newton steps.
`clfLowerBound`, `clfStationary` and `clfStageCompleted` distinguish these outcomes.

If the actual zero-slack condition continues to hold after a time `k0`, then

\[
W_{k+1}\le(1-\alpha)W_k
\quad\Longrightarrow\quad
W_k\le(1-\alpha)^{k-k_0}W_{k_0}\longrightarrow0.
\]

Positive definiteness of `P` implies convergence of all five nominal errors,
with no fixed path phase or prescribed arrival time. The implementation's
roundoff allowance gives a correspondingly tiny practical residual bound.
Temporary positive slack during conflict does not establish this premise;
nor does a finite-budget local solve guarantee discovery of every globally
feasible zero-slack trajectory. The closed-loop regression and campaign test
that the intended regime is actually reached in the evaluated scenarios.

If the hard convex subproblem is infeasible, numerical restoration enables
elastic completion collision rows, terminal membership and endpoint/future
separation. The two terminal elastic variables are fixed to zero in the hard
primary and secondary programs. All elastic variables are search devices:
admission still requires the original hard completion, terminal membership,
future separation and physical constraints. The configured collision buffer
is unchanged.

A cheap lower bound for each affine row over the variable bounds skips a hard
conic solve when that row is already impossible. If the elastic program itself
is infeasible, the outer loop allows bounded trust expansion instead of
repeated shrinking. Unresolved restoration solver failures stop the attempt;
evaluated nonlinear rejection still supports backtracking or shrinking. No
radius update establishes global feasibility. When a newly discovered interior
collision adds rows, the incumbent is reevaluated under the same check set
before comparing scores. Trust is restored to at least its initial radius so
an already shrunken search box does not conceal the new correction requirement.

Nonlinear constraint correction is triangular in time. Analytic Gauss--Newton
directions first restore the one-step CLF sublevel by adjusting `u_0`; the
shorter intersection with the linearized sublevel avoids requesting excessive
one-step reduction. These directions also obey the existing input trust box
about the correction's starting input. Actuator and slew projection further
bounds the proposals. Larger changes require subsequent relinearization;
one nearly singular first-step Jacobian cannot authorize an arbitrarily large
Newton jump to a combined-slip input extreme.
The entire nonlinear rollout is checked before any input can be issued.

An unevaluable transferred or conic rollout is retracted onto the nonlinear
dynamics using finite-horizon linear-quadratic defect feedback. A defined but
inadmissible full conic rollout is offered the same correction when its first
stage is admissible. This also addresses accumulated nonlinear defects before
they leave the tire model's domain; the raw candidate remains eligible.
The gain recursion uses the already evaluable anchor, the existing numerical
state scales, and configured input-deviation weights. Later controls follow
`u_j = uReference_j + K_j (x_j-xReference_j)`, projected onto amplitude and
slew limits; `u_0` stays fixed. Near a regular anchor the affine proposal's
defect is second order, and its propagation becomes
`epsilon_(j+1) = (A_j+B_j K_j) epsilon_j + O(||d||^2)`.
This supplies a nonlinear search candidate when an unstable open-loop tail
would otherwise leave the model domain before it could be evaluated. It
does not itself authorize execution or add a scheduled-position constraint.
The same hard/zero-slack admission and later-input correction still apply.

The later-input correction holds `u_0` fixed, so its actual first successor and
CLF value are preserved exactly. A short endpoint correction first removes the
intrinsic five-dimensional endpoint defect and reconstructs the free core pose.
If other sampled constraints still fail, a minimum-input-deviation QP uses
condensed variational dynamics to correct later inputs against collision,
physical, slew and terminal-separation rows, with the endpoint-center tangent
as an equality. Tight contacts use a numerical interior equal to the configured
solver feasibility tolerance; zero-sensitivity rows are not artificially tightened.
Every QP trial is nonlinearly replayed and its endpoint retracted before merit
comparison. Only the original nonlinear admission test authorizes execution.
The full correction is attempted on the full conic step; subsequent backtracking
retains the cheaper endpoint retraction. Raw and corrected candidates remain
independently eligible, so a failed correction cannot erase an admitted witness.
This is a correction within the same optimization, not an alternate control law.

Evaluation stores per-stage safety, physical residuals and costs. During an
endpoint correction the unchanged prefix is reused and only the modified suffix
is integrated again. An interval flag defaults to unverified for a target-bearing trajectory.
Only a completed interval argument or a valid whole-hold bound sets it true;
endpoint correction cannot turn an unevaluated search prefix into a verified
cache. Independent replanning tests recheck every returned interpolated plan.
Input magnitude/slew violations reject a correction before
integration. The actual terminal-set membership bound is used; an already
admissible candidate is not polished further to obtain half-radius headroom.
Under exact successor and input-memory equality, a shifted witness reuses its
unchanged absolute-time stages; an appended terminal input alone needs a new
rollout. Changed measured states require a fresh full evaluation. Endpoint,
initial-state, input and slew checks are still performed for every returned plan.

All restoration steps share the time remaining since controller-call entry.
Each conic call receives the remaining budget; correction QPs have bounded iterations, and no new candidate processing
or polishing trial is started after expiration. Initial witness construction or
revalidation must still finish before returning any input. An in-flight conic/QP
factorization or rollout is not preemptible, so the budget remains a soft limit,
not a real-time deadline guarantee. A verified feasible witness is returned
even if its final validation crosses the budget. Without one, expired search
reports failure.

Finite suboptimal secondary conic iterates remain eligible even when the
solver stops without an optimal exit flag. Their reported exit flags are
retained. A returned conic point is insufficient for execution. Nonlinear trajectories
are generated by the defining RK4 map. A candidate replaces the retained
witness only if its hard residual is zero and its achieved prefix slack sum
is zero. A positive solver feasibility tolerance never
licenses a positive hard residual. `zeroSlack` means an achieved sum exactly
zero in the numerical evaluation; it no longer means `sum <= 1e-5`.
A shifted, slightly inconsistent iterate is first offered to the same nonlinear
constraint correction before rebuilding a conic problem. It becomes a witness
only after complete admission. A feasible initialization can also supply a witness. If none exists and
search fails, the controller reports `noFeasibleContinuation`.

The endpoint enclosure is cached construction work, not a per-trajectory
interval verifier. There is no MPFR library or native verification pipeline.
Ordinary floating-point evaluation is not a formal proof of exact real
arithmetic, nor a robustness guarantee against plant/integration errors.
The structural theorem concerns the declared nominal map under consistent
state, clock, target and constraint assumptions. Boundary decisions in
floating-point geometry and unmodeled disturbances require a separate error
analysis for a stronger execution guarantee. Solver tolerances are reported
as numerical settings, not treated as theorem hypotheses already proved.

## Shift and slack proof

While `H_k > H_min`, remove the first input from the retained prefix/completion.
The old first completion input enters the new prefix with zero slack. The
remaining completion reaches the same endpoint with the same stored pose.
Once `H_k=H_min`, append `kappa_(B,k+H_min)(.;g)` at the **old** absolute terminal
time and state, retaining that pose. Fixed-pose endpoint invariance puts its
successor in `B_(k+H_min+1)(g)` and supplies zero appended slack. The already
certified all-future separation also covers this later suffix; its future set
is a subset of the one previously checked. The implemented sufficient
straight-progress and ego-ray/target-orbit bounds preserve this suffix property in the
nominal real-arithmetic construction.
Input memory makes the appended slew constraint part of the same result.

The pose is therefore free during a new search and fixed when constructing
the shifted feasibility candidate. A later accepted search may change it only
after rechecking the full nonlinear constraints and future separation. The
existence of the old fixed-pose candidate is sufficient for the recursive
argument; the optimizer need not select the same pose in every new search.
This is a feasibility construction inside one MPC problem. It does not require
a runtime fallback selector or authorize execution after a failed admission.

Every retained start, midpoint, target evaluation and input comparison has
unchanged absolute time and values. Thus the shifted sequence is feasible
for the next nonlinear problem, under the nominal successor assumption.
The completion witness and endpoint policy together realize

\[
y\in K_j\implies\kappa_{f,j}(y)\in U_{\mathrm{safe},j}(y),\quad
F(y,\kappa_{f,j}(y))\in K_{j+1}.
\]

For an exact primary optimum this yields Huang's shifted-value inequality
`V_N(k+1) <= V_N(k) - xi_(0|k)`. SCvx is not a global optimizer of this
nonconvex problem. The implementation therefore retains achieved slacks
and imposes the component-sum budget

\[
S_{k+1}\le\sum_{i=1}^{N-1}\xi_{i|k}=S_k-\xi_{0|k}.
\]

Accepted witnesses have `S_k = 0`; positive restoration candidates do not
enter this execution induction. The shifted witness already meets the budget.
Improvements cannot increase it,
and infeasible search points cannot become executable. A changed measured ego state invalidates the
old shift proof; renewed feasibility does not retroactively repair that
transition's slack inequality.

## Interfaces and limits

State format **59** stores the absolute sample index, target epoch, full
input/state continuation, the freely placed endpoint core, achieved prefix
slacks and problem context. It separates the original-path nominal reference
from the straight terminal trim and carries the Schur-complement construction.
Old state formats are discarded. Predictions contain prefix plus completion;
metadata distinguishes their lengths and reports the chosen `terminalPose`.
No lane/avoidance mode or backup-controller switch is introduced.

The time limit is checked between complete numerical steps and passed to
each conic solve as its soft `MaxTime`. A running factorization or terminal
correction can overrun it. Real-time completion within a
50 ms period is not established. Independent ODE replay is an experiment,
not part of execution admission. Sampled start/midpoint safety does not by
itself establish continuous-intersample collision freedom.

## Literature

The primary slack objective and shift reasoning follow Huang, Wang,
Margellos and Goulart, *Predictive Control Barrier Functions: Bridging model
predictive control and control barrier functions* (2025), problem (8) and
its recursive-feasibility argument. The terminal family above supplies the
zero-slack append condition in place of a fixed invariant terminal set.
Polygon support duals follow the convex collision-avoidance formulation of
Li et al. (2023), with both moving rectangles represented explicitly.
The local source PDFs are in `reference/`.

Optimizing a reference parameter is related to the artificial-reference idea
in Limon, Alvarado, Alamo and Camacho, [*MPC for tracking piecewise constant
references for constrained linear systems*](https://doi.org/10.1016/j.automatica.2008.01.023),
Automatica 44(9), 2382--2387 (2008). Only the publisher's indexed abstract was
consulted here. That linear-system result is not a proof for this nonlinear,
moving-target controller. The rigid-pose equivariance, Schur elimination and
fixed-pose shift argument above are project derivations.

Luque, Chanfreut, Limon and Maestre, [*Model predictive control for tracking
with implicit invariant sets*](https://doi.org/10.1016/j.automatica.2025.112436),
Automatica 179, 112436 (2025), supplies related context for implicitly
represented terminal constraints. Its Theorems 2--3 concern constrained linear
systems; their finite-completion and shift arguments motivate the architecture
but do not certify this nonlinear Fiala/RK4 implementation.
