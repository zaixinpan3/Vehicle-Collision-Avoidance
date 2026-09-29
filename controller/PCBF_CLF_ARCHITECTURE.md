# PCBF, lane CLF and time-indexed terminal continuation

## Scope

The controller uses one nonlinear ego bicycle model and one known target
trajectory. A Huang-style primary objective minimizes the sum of safety
slacks over a fixed MPC prefix. A soft lane CLF supplies the secondary
objective. Li-style polygon support duals and sequential convexification
supply numerical search directions. Both zero-slack and positive-slack
**feasible** prefixes can be executed. Positive slack describes recovery;
it does not imply collision-free motion.

The terminal condition is an implicit backward-reachable tube, represented
by a hard completion trajectory and an indefinitely admissible endpoint
family. Its construction and numerical scope are described below. The
controller retains the feasible continuation, including its slack budget,
so an unsuccessful improvement solve does not discard an available control.

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
half-sample index**. This preserves retained node and midpoint evaluations
under a shift. Later observations cannot reinitialize the forecast. A changed
target trajectory, road, physical constraint set or prediction model requires
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

At each stage, collision and road constraints are evaluated at its start
and midpoint. The prefix uses one shared nonnegative slack for these samples.
The completion tail uses **zero slack**. Input limits, slew limits and
velocity bounds at nodes and midpoints remain hard everywhere. The endpoint
also has hard sampled safety constraints. Internal RK4 stages must remain
in the tire/model domain.

The signed collision predicate uses nonnegative support dual variables for
both oriented rectangles and a unit separating normal. During overlap, a
signed support gap supplies a restoration direction. An unsigned rectangle
distance of zero is never used as evidence of separation. The configured
additional physical collision buffer defaults to 0.005 m and can be explicitly
set to zero. It applies at the prediction constraint samples and in the
terminal separation certificate; it is not a continuous-time clearance guarantee.

With the endpoint family `B_j` defined below, the terminal tube is represented
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
Let `R` be its nonsingular norm factor. The endpoint family is

\[
B_j=\{y:\|R e_j(y)\|_2\le r\},\qquad
\kappa_{B,j}(y)=u^r+K e_j(y).
\]

The reference pose follows the sampled RK4 pose increment from a fixed
absolute endpoint epoch. On a curved road this gives the discrete pose
orbit, rather than assuming that a Frenet integration equals global RK4.
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

The enclosure implementation uses outward-padded double interval operations,
Taylor remainders for sine/cosine and exponential, analytic differentiation
through RK4, and a checked positive-definiteness bound for the proposed
matrix norm. The inverse residual is handled by a Neumann-series bound.
The endpoint is restricted to a positive-speed, unsaturated local Fiala
chart; this restriction does not restrict the prefix to that chart. These
are sufficient local bounds and can yield a small, conservative seed.

The reference footprint plus the enclosed deviations must also fit the road
and remain separated from the target for **all future times**. Straight
motion uses extrema of relative quadratic progress, including both ahead
and behind separation and signed reversal. Circular motion uses sufficient
orbit/ray bounds. Midpoint pose deviations and the discrepancy between the
sampled reference orbit and road circle are included. These all-future bounds
apply only to the endpoint seed. The finite completion compares the two
vehicles at matching times, allowing earlier passages through locations
that the target reaches later.

If no endpoint with this sufficient continuation can be found within
`maximumHorizonSteps`, initialization reports failure. This does not establish
that the original nonlinear problem has no other viable terminal family.
There is no finite-only fallback masquerading as an indefinite tail.

## Sequential convexification and acceptance

A lane-feedback rollout initializes the search; no avoidance trajectory is
required. The fixed endpoint is selected once from that rollout. Each SCvx
step uses variational RK4 dynamics and linearized support-dual geometry.
The endpoint is represented by its actual **2-norm cone**, with ego state
and final input in the cone. The previous inscribed 1-norm polytope has been
removed. `coneprog` solves the primary slack problem and the secondary
quadratic lane/CLF objective. Each stage and the CLF penalty has its own
small squared-norm cone epigraph; their epigraph values sum to the total
objective. This is algebraically equivalent to one horizon-wide norm cone,
while avoiding its global factorization coupling. The default internal
linear solver remains unchanged.

Both objectives have a known lower bound of zero. A nonlinear feasible witness
with exactly zero safety slack and secondary cost no greater than
`solver.optimalityTolerance` terminates with `objectiveLowerBound` without a
conic solve. The tolerance is used as an absolute secondary objective gap here;
it never relaxes safety slack or hard constraints. A zero-slack feasible anchor
also establishes the primary optimum of its affine subproblem, so only the
secondary solve is needed when its cost still warrants improvement. These are
termination rules within the same controller, not separate controller modes.

The soft first-step CLF is

\[
W(e_{1|k})-(1-\alpha)W(e_{0|k})\le\rho,\qquad\rho\ge0.
\]

The secondary solve is constrained by the primary slack optimum plus the
configured numerical tie tolerance, and by the retained slack budget when
one is available. Neither CLF optimality nor asymptotic lane convergence is
asserted merely because the feasibility proof holds.

If hard completion rows make a convex subproblem infeasible, a numerical
restoration step temporarily uses elastic completion rows. This is only a
search direction. It cannot be retained unless a fresh nonlinear rollout
satisfies the original **hard, zero-slack** completion. A short shooting
correction of the final controls removes terminal linearization defects.
All corrected inputs, slew limits, safety samples and endpoint membership
are reevaluated; selected controls are not clipped.

Evaluation stores per-stage safety, physical residuals and costs. During an
endpoint correction the unchanged prefix is reused and only the modified suffix
is integrated again. Input magnitude/slew violations reject a correction before
integration. The original half-radius polishing target retains interior
headroom in the small terminal set.
Under exact successor and input-memory equality, a shifted witness reuses its
unchanged absolute-time stages; an appended terminal input alone needs a new
rollout. Changed measured states require a fresh full evaluation. Endpoint,
initial-state, input and slew checks are still performed for every returned plan.

All improvement steps share the time remaining since controller-call entry.
Each conic call receives the remaining budget, and no new candidate processing
or polishing trial is started after expiration. Initial witness construction or
revalidation must still finish before returning any input. An in-flight conic
factorization or rollout is not preemptible, so the budget remains a soft limit,
not a real-time deadline guarantee. Expiration retains an available feasible
witness; without one, initialization reports failure.

Finite suboptimal secondary conic iterates remain eligible even when the
solver stops without an optimal exit flag. Their reported exit flags are
retained. A returned conic point is insufficient for execution. Nonlinear trajectories
are generated by the defining RK4 map. A candidate replaces the retained
witness only if its hard residual is zero and its achieved prefix slack sum
meets the retained bound. A positive solver feasibility tolerance never
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
remaining completion reaches the same fixed endpoint. Once `H_k=N`, append
`kappa_(B,k+N)` at the **old** absolute terminal time and state. Endpoint
invariance puts its successor in `B_(k+N+1)` and supplies zero appended slack.
Input memory makes the appended slew constraint part of the same result.

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

The shifted witness already meets it. Improvements cannot increase it,
and failed solves retain it. A changed measured ego state invalidates the
old shift proof; renewed feasibility does not retroactively repair that
transition's slack inequality.

## Interfaces and limits

State format **52** stores the absolute sample index, target epoch, endpoint
family, full input/state continuation, achieved prefix slacks, and problem
context. Old state formats are discarded. Predictions include the whole
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
