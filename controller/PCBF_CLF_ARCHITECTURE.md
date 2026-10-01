# PCBF, lane CLF and free-phase terminal continuation

## Scope

The controller uses one nonlinear ego bicycle model and one known target
trajectory. It returns the first admissible nonlinear continuation. While
restoration is needed, a Huang-style primary objective minimizes the sum of
safety slacks over a fixed MPC prefix, and a soft lane CLF guides the secondary
search. Li-style polygon support duals and sequential convexification supply
numerical search directions. Positive slack guides feasibility restoration;
only a zero-prefix-slack candidate satisfying all original hard constraints
can authorize an input. An initialization seed need not satisfy these checks.

The terminal condition is an implicit backward-reachable tube, represented
by a hard completion trajectory and an indefinitely admissible endpoint
family. Its construction and numerical scope are described below. The
controller retains the feasible continuation, including its slack budget,
and executes it immediately whenever revalidation succeeds.

Road boundaries are currently excluded from the controller problem, including
the finite prediction and indefinite endpoint admission. The given path still
defines lane recovery, cruise and terminal references. This is one controller;
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

Every target prediction uses the original epoch and an **absolute integer
subsample index**. Nodes and midpoints retain the same half-sample arithmetic;
additional collision checks use the interior nodes of the half-hold integration
mesh, when present. This preserves repeated absolute-time evaluations
under a shift. Later observations cannot reinitialize the forecast. A changed
target trajectory, reference path, physical constraint set or prediction model requires
an explicit new problem (`previousState=[]`). A timestamp gap is rejected.
Equivalent headings across the +/-pi representation boundary are compared
modulo one turn and do not invalidate or restart the fixed target epoch.
A changed measured ego state or input memory invokes fresh feasibility
restoration and disables the shifted slack guarantee for that transition.

## The nonlinear MPC problem

Let `N = controller.horizonSteps`, let `k` be the absolute sample index and
let `M` be the fixed endpoint selected during initialization. Define
`H_k = max(N,M-k)`. The first `N` stages form the PCBF prefix. Stages
`N,...,H_k-1` are the completion tail.

At each stage, collision constraints are evaluated at its start and midpoint.
Before admitting a candidate, nearby encounters are also checked at the interior
half-hold integration nodes. Proximity uses a one-hold travel estimate from the
configured ego speed/yaw bounds and predicted target motion; it is a screening
rule, not a continuous-time certificate. An interior violation adds the affected
stage's interior linearized collision rows to subsequent convex subproblems.
Thus additional rows are generated when a nominally feasible candidate exposes
a missed collision, rather than inserted throughout every initial restoration.
The prefix uses one shared nonnegative slack for all checked stage samples.
The completion tail uses **zero slack**. Input limits, slew limits and
velocity bounds at nodes and midpoints remain hard everywhere. The endpoint
also has hard sampled safety constraints. Internal RK4 stages must remain
in the tire/model domain.

The signed collision predicate uses nonnegative support dual variables for
both oriented rectangles and a unit separating normal. During overlap, a
signed support gap supplies a restoration direction. An unsigned rectangle
distance of zero is never used as evidence of separation. The configured
additional physical collision buffer defaults to 0.006 m and can be explicitly
set to zero. It applies at the prediction constraint samples and in the
terminal separation certificate; it is not a continuous-time clearance guarantee.
The terminal interval construction encloses pose deviations at every RK4
integration node. Its reference-orbit padding also takes the maximum nominal
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

`terminalContinuation` constructs an eight-dimensional local continuation
family once per model configuration and curvature. The existing five-state
lane LQR remains a CLF/preference ingredient. It is not an invariance proof.

The endpoint construction instead linearizes the **actual global RK4 map**
at the cruise state, includes input memory, transforms the successor into a
moving reference frame, and designs a discrete augmented feedback gain `K`.
Let `R` be its nonsingular norm factor and let `sigma` denote longitudinal
reference phase in meters. For each fixed phase the endpoint family is

\[
B_j(\sigma)=\{y:\|R e_j(y;\sigma)\|_2\le r\},\qquad
\kappa_{B,j}(y;\sigma)=u^r+K e_j(y;\sigma).
\]

The reference pose follows the sampled RK4 pose increment from an absolute
endpoint epoch. Its phase is a free scalar decision variable, with no prescribed
value, phase penalty or explicit phase bound. On a straight path, phase
translates the reference along its cruise direction. On a circular path, phase
rotates it about the same discrete orbit center. If `ell` is the straight
distance or circular arc length per reference increment, the pose group power
uses `j - epochIndex + sigma/ell` increments. On a curved road this gives the
discrete pose orbit, rather than assuming that a Frenet integration equals global RK4.
The body-state trim need not be numerically exact: its successor defect is
included in the bound below.

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
constraints are unchanged. Each fixed-phase reference is such a transform of
the original reference, so its moving-frame closed-loop error map `Phi` is the
same. The existing gain, norm, defect and contraction bounds therefore apply
to every phase without a new enclosure computation. This argument uses the
spatially homogeneous nominal model and absence of road-edge constraints;
it does not translate the target or discard its separation requirement.

The enclosure implementation uses outward-padded double interval operations,
Taylor remainders for sine/cosine and exponential, analytic differentiation
through RK4, and a checked positive-definiteness bound for the proposed
matrix norm. The inverse residual is handled by a Neumann-series bound.
The endpoint is restricted to a positive-speed, unsaturated local Fiala
chart; this restriction does not restrict the prefix to that chart. These
are sufficient local bounds and can yield a small, conservative seed.

The reference footprint plus the enclosed deviations must remain separated
from the target for **all future times**. Straight
motion uses extrema of relative quadratic progress, including both ahead
and behind separation and signed reversal. Circular motion uses sufficient
orbit/ray bounds. Integration-node pose deviations relative to the sampled reference
orbit are included. No road-edge or lane-width condition is imposed. These all-future bounds
apply only to the endpoint seed. The finite completion compares the two
vehicles at matching times, allowing earlier passages through locations
that the target reaches later.

Let `c_j(sigma)` be this sufficient all-future separation margin, evaluated
using the phase-adjusted reference and the target at the **same absolute
endpoint time**. The effective endpoint set is

\[
\mathcal A_j=\{\sigma:c_j(\sigma)\ge0\},\qquad
B_j=\bigcup_{\sigma\in\mathcal A_j}B_j(\sigma).
\]

The controller retains the selected phase as part of the feasibility witness.
It never assumes that an arbitrary longitudinal shift preserves target safety.
Straight-path separation uses a maximum of sufficient separating-axis bounds;
the active branch provides the affine search row, followed by full nonlinear
rechecking. Circular phase preserves the full orbit, so the conservative
all-future orbit separation is independent of phase. Consequently this change
does not resolve rejection of intersecting circular terminal orbits.

This union removes the requirement to recover a preselected longitudinal
position at the endpoint time. It retains a finite completion horizon and a
local cruise-state condition at its end. Initialization still selects endpoint
index `M` from a lane or moving-flow rollout; no free terminal-time optimization or general
viability-kernel computation is implemented. A terminal condition necessarily
restricts admissible trajectories, and this sufficient construction can remain
conservative even with free phase.

If no seed endpoint passes this sufficient continuation within
`maximumHorizonSteps`, the bounded rollout remains an uncertified search seed.
The optimizer still has to satisfy the original terminal conditions before
execution. Failure does not establish that other terminal families are impossible.
There is no finite-only fallback masquerading as an indefinite tail.

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
cruise speed. A steering preference based on `0.8 * mu * g` lateral acceleration
and a longitudinal-input preference of +/-0.35 reduce aggressive seed motion;
these are heuristic preferences, not new optimizer constraints or a certified
friction allocation. Final clipping enforces the actual amplitude and slew
bounds, then the nonlinear Fiala model generates the next seed state.

The actual state/input rollout supplies dynamics, tire and collision expansion
points. The input regularizer in the convex subproblem penalizes the change
from this anchor; the nominal lane/CLF objective remains tied to the original
given path. The retained nonlinear cost has no separate input-amplitude term;
input proximity regularizes the local search step only. A valid prior witness
returns immediately. Small measured-successor
differences repair its shifted inputs; fresh flow candidates are considered for
cold initialization or a state discrepancy larger than 1e-4 in the raw state
infinity norm. This threshold is a search-cost heuristic, not a safety tolerance.
Every issued candidate undergoes the same nonlinear admission checks.

## Sequential convexification and acceptance

A lane-feedback rollout supplies the baseline. When it is inadmissible and
there is no usable prior reference, at most two moving-target flow rollouts
provide opposite passing biases within the same controller-call budget.
A seed may collide or fail terminal admission; evaluated nonlinear violation
selects the restoration anchor. A fully admissible seed returns immediately.
The endpoint index and orbit anchor belong to the selected rollout, with
longitudinal phase subsequently free. Each SCvx
step uses variational RK4 dynamics and linearized support-dual geometry.
The endpoint is represented by its actual **2-norm cone**, with ego state
and final input in the cone. One additional scalar phase increment enters the
cone through the analytic derivative of the full eight-coordinate moving-frame
error. A separate affine row enforces the sufficient future-separation branch.
The previous inscribed 1-norm polytope has been
removed. `coneprog` solves the primary slack problem and the secondary
quadratic lane/CLF objective. Each stage and the CLF penalty has its own
small squared-norm cone epigraph; their epigraph values sum to the total
objective. This is algebraically equivalent to one horizon-wide norm cone,
while avoiding its global factorization coupling. The default internal
linear solver remains unchanged.

A revalidated initial or shifted witness terminates with `feasibleWitness`
without a conic solve. Its nonlinear hard residual must be exactly zero and
its prefix safety-slack sum must be exactly zero. The
secondary cost may be large. Neither objective optimality nor agreement
between affine and nonlinear CLF slack is required for execution.

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
scores prefer smaller terminal excess, with the existing feasible cost tie
rule retained.

A feasible primary returns immediately. An improving but inadmissible primary
returns to the outer loop for relinearization before optional secondary cost
work. Conic steps that fail to improve can be checked at half and quarter
amplitude, within the same time budget. The first admitted nonlinear candidate
ends the iteration; no subsequent cost improvement is required.
`search.converged` and its legacy `scvxConverged` metadata field describe this
feasibility stopping target, not numerical optimality. Positive prefix slack
remains a search residual and does not authorize execution.

The soft first-step CLF is

\[
W(e_{1|k})-(1-\alpha)W(e_{0|k})\le\rho,\qquad\rho\ge0.
\]

The secondary solve is constrained by the primary slack optimum plus the
configured numerical tie tolerance, and by the retained slack budget when
one is available. Neither CLF optimality nor asymptotic lane convergence is
asserted merely because the feasibility proof holds.

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

A short endpoint correction jointly adjusts final inputs and free phase.
A feasible candidate bypasses correction and ends it immediately. Every trial
retains the best complete nonlinear candidate before following a terminal-error
improvement; inputs are never clipped after the solve.

Evaluation stores per-stage safety, physical residuals and costs. During an
endpoint correction the unchanged prefix is reused and only the modified suffix
is integrated again. Input magnitude/slew violations reject a correction before
integration. The actual terminal-set membership bound is used; an already
admissible candidate is not polished further to obtain half-radius headroom.
Under exact successor and input-memory equality, a shifted witness reuses its
unchanged absolute-time stages; an appended terminal input alone needs a new
rollout. Changed measured states require a fresh full evaluation. Endpoint,
initial-state, input and slew checks are still performed for every returned plan.

All restoration steps share the time remaining since controller-call entry.
Each conic call receives the remaining budget, and no new candidate processing
or polishing trial is started after expiration. Initial witness construction or
revalidation must still finish before returning any input. An in-flight conic
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
A feasible initialization can also supply a witness. If none exists and
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

For `k+N < M`, remove the first input from the retained prefix/completion.
The old first completion input enters the new prefix with zero slack. The
remaining completion reaches the same endpoint with the same stored phase.
Once `H_k=N`, append `kappa_(B,k+N)(.;sigma)` at the **old** absolute terminal
time and state, retaining that phase. Fixed-phase endpoint invariance puts its
successor in `B_(k+N+1)(sigma)` and supplies zero appended slack. The already
certified all-future separation also covers this later suffix; its future set
is a subset of the one previously checked. The implemented sufficient
straight-progress and full-orbit bounds preserve this suffix property in the
nominal real-arithmetic construction.
Input memory makes the appended slew constraint part of the same result.

The phase is therefore free during a new search and fixed when constructing
the shifted feasibility candidate. A later accepted search may change it only
after rechecking the full nonlinear constraints and future separation. The
existence of the old fixed-phase candidate is sufficient for the recursive
argument; the optimizer need not select the same phase in every new search.

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
and failed solves retain it. A changed measured ego state invalidates the
old shift proof; renewed feasibility does not retroactively repair that
transition's slack inequality.

## Interfaces and limits

State format **56** stores the absolute sample index, target epoch, endpoint
family and its chosen phase, full input/state continuation, achieved prefix slacks, and problem
context. Format 56 invalidates previous positive-prefix-slack execution
semantics; the earlier interior collision checks are retained. Old state formats are discarded. Predictions include the whole
prefix plus completion; metadata distinguishes their lengths. No lane/
avoidance mode switch is introduced.

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
moving-target controller; the specific fixed-phase equivariance and retained
witness argument above supplies the structural extension used here.
