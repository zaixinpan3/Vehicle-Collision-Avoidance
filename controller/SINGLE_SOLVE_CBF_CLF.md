# Predictive continuation with one soft-CLF solve per sample

The format-26 controller optimizes a complete finite input sequence. The
configured prediction length is used (default 16 holds), and finite encounter
admission may require a longer sequence. Only the first input is executed.
The fourth output retains the accepted input plan, exact affine node boxes,
Bernstein enclosures, stage generators, separating normals, charts, finite
exit deadline, and target-independent terminal set.

The terminal law is exclusively a prediction-side mathematical certificate.
It never directly supplies an actuator command. An unsuccessful or malformed
solve raises `collisionAvoidanceController:optimizationFailed` before control
is issued, including when a carried feasible witness exists. There is one
optimization per sample, no retry, and no separate post-solve acceptance checker.
The former forced one-hold truncation and circumdisk barrier-decay rows are
removed. Current swept collision constraints use oriented rectangles.

## Optimization and model

For horizon $N$, the decision is $[U;\delta]$, where
$U=[u_0^\top,\ldots,u_{N-1}^\top]^\top$ and $\delta\ge0$ is the first-hold
sampled CLF norm slack. The quadratic objective is

\[
\sum_{i=1}^{N}e_i^\top P e_i+
\sum_{i=0}^{N-1}(u_i-u_*)^\top W(u_i-u_*)+w_\delta\delta^2.
\]

The cruise trim $u_*$ and metric $P$ are those used to synthesize the discrete
Riccati CLF certificate. Road load and local curvature enter that trim.
Prediction anchors select convex geometry; they are not desired accelerations
or input references. Future state and input decisions affect collision,
road/chart/model-domain, slip, actuator and slew constraints. The only relaxed
constraint is the first-hold CLF. Its units and dissipation bound below are
unchanged from the soft-CLF design.

The declared plant is the held affine bicycle generator, with zero process
residual. An admitted prediction uses the sampled cruise generator frozen at
its admitted curvature. An inherited prediction retains that exact generator,
trim and metric; it is not replaced by a newly scheduled plant. This scope
does not establish nonlinear physical-vehicle inclusion or a common Lyapunov
proof across curvature/metric changes.

## Complete prediction certificate

Every held interval has a swept Bernstein enclosure. Oriented rectangle
support bounds, target finite-flow uncertainty, and road/model allowances
produce hard affine rows. The finite target model retains Cartesian jerk and
yaw-acceleration bounds; no all-future support bound is imposed.

The final node must certify both:

1. The entire target footprint lies outside the declared complete perception
   region in a certified separating direction. A current valid observation,
   not a timer, discharges the encounter. Its absolute deadline cannot move
   forward during inherited optimization.
2. The ego information box belongs to the target-independent road terminal
   set, including terminal-input entry slew. This is the comparison-system
   stopping-excursion construction, with rows of the form
   $A_f z_N+E_f\rho_N\le b_f$. The hypothetical terminal law brakes the lower
   speed endpoint and admits a nonnegative invariant speed box.

`hardEncounterBarrier.completionRows` constructs both conditions.
`terminalStep`, `terminalFlow`, and `terminalMembership` describe or audit
this mathematical terminal continuation; the online controller never calls
them to obtain its command. Physical road boundaries are optional. Model-domain
and chart constraints remain present when the scenario has no road boundaries.

## Feasible continuation is part of the next optimization

Store the accepted hard affine family $A U\le b$, with its inward numerical
reserves. Partition $A=[A_0\;A_+]$. After executing $u_0^*$, the inherited
family is

\[
A_+ U^+\le b-A_0u_0^*.
\]

The accepted suffix $U^{+*}=(u_1^*,\ldots,u_{N-1}^*)$ satisfies this family
by direct substitution. Rows with no remaining decision dependence are
already fixed by the certified prefix and can be removed. The first-stage
slew row shifts in the same way. No additional tightening or new geometric
convexification is applied to the inherited family.

The cell maps and offsets undergo the same substitution. Their original
radii, normals and charts remain valid; current measurement intersection
only restricts the actual information set. The prediction centers are
propagated with exact held affine transitions, not with truncated polynomial
node approximations. The CLF uses the current conditioned state, and its
unbounded nonnegative slack admits any finite physically feasible suffix.
Therefore it cannot destroy existence of a solution to the inherited hard
family. The solver uses translated coordinates about the carried/trim plan
to improve conditioning; this exact translation does not constrain its optimum.

This proves mathematical successor feasibility **within an admitted active
encounter, with unchanged obligations, valid information-set inclusion,
unchanged execution/model contracts, and a nonempty finite suffix**. Numerical
solver completion is a separate obligation. The terminal construction supplies
a mathematical safe continuation after the certified exit, but that law is
not an executable fallback in this implementation.

After confirmed release, the controller starts a fresh cruise performance
horizon. A new target, enlarged motion bounds, or partial release among multiple
targets also requires fresh formulation/admission. This implementation does
not prove that every such fresh convexification contains the old road witness.
Accordingly, `recursiveFeasibilityGuaranteed` remains false for the complete
online hybrid controller; `inheritedFeasibleFamily` specifically identifies
frames to which the affine shift argument applies. Saving a witness alone
would not justify a stronger claim.

## Observation and execution contracts

Targets must have stable identities and bounded finite motion. An active
encounter requires a current `perception` declaration with `time`, `range`,
and logical `completeWithinRange=true`. An observed exterior target imposes
no current encounter obligation. Missing data cannot release an interior
reachable target. New target admission and future re-entry remain separate
conditions, not consequences of the old certificate.

Every successor reports the actual previous held input and next timestamp.
Measured ego/target boxes are intersected with the carried prediction. An
empty intersection violates the declared execution/measurement/model contract.
The observer implementation and its high-gain mathematical framework are
unchanged by this restoration.

## Soft sampled CLF and its dissipation bound

Let \(q_0<1\) bound the Riccati feedback's nominal squared-norm contraction,
choose \(0<f<1\), and set

\[
c=1-f(1-q_0),\qquad a=(c+q_0)/2<c.
\]

The controller enforces the single SOC

\[
\|R(F\hat e+G(u-u_*)+d)\|
\le \sqrt a\,\|R\hat e\|+\epsilon_{\rm num}+\delta,\qquad\delta\ge0.
\]

For \(e=\hat e+\eta\), \(|\eta|\le r\), define

\[
\sigma=\sqrt a\,\||R|r\|+\||RF|r\|+2\epsilon_{\rm num}.
\]

The triangle inequality gives
\(\|Re^+\|\le\sqrt a\|Re\|+\sigma+\delta\). Young's inequality yields

\[
V(e^+)\le cV(e)+B+S(\delta),\qquad V(e)=e^\top Pe,
\]
\[
B=\frac{c}{c-a}\sigma^2,\qquad
S(\delta)=\frac{c}{c-a}\delta(2\sigma+\delta).
\]

Metadata reports `clfSlack` as the optimizer's final decision,
`clfSlackPenalty` as \(w_\delta\delta^2\), `clfDisturbanceBound` as \(B\), and
`clfSlackDissipationBound` as \(S(\delta)\). A tiny negative numerical slack is
replaced by zero only when forming this outward reporting allowance; neither
the solver decision nor the command is changed or checked. The reported
`clfDissipationCertified` flag means this **slack-dependent sampled bound**,
as specified by `clfDissipationScope`.

At zero slack, exact arithmetic and exact states, \(B=0\) and this is sampled
exponential dissipation. Positive slack allows \(V\) to increase, including
when avoiding an obstacle requires leaving exact cruise. A positive quadratic
penalty does not itself prove that slack vanishes, even on a clear road.
Strict decrease follows whenever \(B+S(\delta)<(1-c)V\). If
\(B_k+S(\delta_k)\le\overline E\) uniformly along an indefinitely feasible
sequence, the bound implies \(\limsup V\le\overline E/(1-c)\).
This conditional statement does not assert such a uniform bound exists.
No claim of
pointwise continuous-time \(\dot V<0\) is made between samples. A common fixed
curvature and trim are required to chain one metric across holds. A changed
curvature/metric or reference needs a separate switching analysis.

For any finite physical input satisfying the hard constraints, a finite
nonnegative slack can satisfy the CLF cone. Thus the CLF no longer excludes a
physically feasible input merely because it would increase tracking error.
Hard obstacle, road, domain and actuator constraints can still conflict. The
controller stops on an unsuccessful solve; slack provides no backup design
or recursive-feasibility guarantee.

The exact-state driver saves the optimized slack, its dissipation allowance,
the residual with that allowance removed, and the unrelaxed residual. Offline
experimental success uses the slack-dependent inequality. A positive unrelaxed
residual remains visible and is never described as unrelaxed CLF dissipation.

## Numerical and runtime scope

The native solver must return strict success with a finite, real decision of
the expected size. Physical reserves are constructed before solving; there is
no independent online residual verifier. A solver hook must satisfy the same
feasibility/status contract. Incorrect success declarations are not caught by
a separate checker. The test suite audits physical rows, sampled rectangles,
CLF dissipation, witness shifting and terminal membership independently.

`frameDeadlineSeconds` limits native solver work. It does not bound complete
MATLAB preparation, prediction construction, loading, or scheduling. Experiments
report complete measured frame times and cold/warm behavior separately. The
restored full prediction is not yet qualified for every-frame 100 ms execution.
See the dated restoration report under `report/` for measured results and
remaining admission/performance failures.
