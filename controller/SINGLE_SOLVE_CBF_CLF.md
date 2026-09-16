# Predictive safety with finite convex admission branches

The format-29 controller retains a complete finite input plan, swept rectangle
collision certificate, finite confirmed exit and a permanent terminal
information-state set. Both the predictor and terminal set use the same
admitted held affine cruise generator. Physical input effort remains centered
on the CLF/LQR trim; the first-hold CLF has a nonnegative squared-penalty slack.
The target high-gain observer is unchanged.

The complete mathematical proof and its precise premises are in
[TERMINAL_CBF_PROOF.md](TERMINAL_CBF_PROOF.md). The guarantee covers admitted
encounters, partial/full confirmed release, prediction exhaustion and
indefinite target-free operation under the declared model, sensing and
admission contracts. `recursiveFeasibilityGuaranteed=true` has this conditional
scope, explicitly published in metadata. It is not a 100 ms runtime guarantee
or a nonlinear physical-vehicle claim.

## Actual next-frame optimization

1. Retain the accepted hard affine family, terminal SOC, generators and swept
   enclosures. Substitute the executed input to obtain its feasible suffix.
2. After confirmed target release, delete only that target's collision and exit
   rows. Retain the permanent certificate and any remaining target obligations.
3. A fresh target-free performance horizon may replace the suffix only when
   a concrete candidate satisfies every row and all cones of that new problem.
4. If the suffix is exhausted and no longer replacement is certified, solve
   a free one-hold constrained optimization inside the invariant terminal
   information-state set. Its feedback candidate proves nonemptiness; the
   optimizer chooses the command.

An admitted active encounter uses one convex optimization of its retained
branch per frame. Fresh admission searches the finite polyhedral family
described in [FINITE_CONVEX_BRANCHES.md](FINITE_CONVEX_BRANCHES.md), using
integer assignment search and complete convex subproblems. No prescribed
lateral trajectory or maneuver side is used. Exhausted or incomplete search
and failed independent hard-safety verification issue no command; a stored
input or terminal law is never executed as a fallback.

## Terminal set and sensing contract

The terminal set is a product of disks in the stable modal coordinates of
the same sampled LQR closed-loop map. Its component radii satisfy robust
contraction, held-input amplitude inequalities and finite slew limits.
State and slip bounds are not imposed. The finite plan's final information box must lie in this set and satisfy
robust transition slew to every possible conditioned terminal feedback input.

The sensing limit is initialized from the admitted ego measurement enclosure
and retained. A curved reference converts the current Cartesian measurement
box to a declared Frenet radius limit without imposing a lateral state domain. Future published measurement bounds must remain
within it. Finite prediction never assumes a favorable future measurement
reset. The local terminal problem retains true-state modal membership as well as
the measured box; it does not require each later rectangular hull to lie
inside the modal set. Only the permanent invariant argument uses the bound
on future actual measurements. Increasing this bound is a changed contract.

The permanent reference is an analytically continued straight line or constant-
curvature curve with no physical road boundaries, matching the current
no-road-boundary experiment specification. Only actuator amplitude/slew and the collision/terminal certificates are hard. Finite
centerline samples and analytic arc length no longer cause projection clipping
at their display endpoints. Curved station measurements unwrap about the
carried prediction. A finite physical road, arbitrary polyline corner, or
varying-curvature path needs its own permanent continuation certificate; such
geometry is not silently admitted under this theorem.

## Objective and certificate storage

For `N` holds the decision is `[U; delta]` and the objective is

\[
\sum_{i=1}^{N}e_i^\top P e_i+
\sum_{i=0}^{N-1}(u_i-u_*)^\top W(u_i-u_*)+w_\delta\delta^2.
\]

The first-hold CLF and five terminal modal inequalities are SOC constraints. All
collision, actuator and slew rows are hard. The absolute target
exit deadline is preserved while any admitted target remains active.

The returned format-29 state contains the plan, verified inherited affine
bounds and terminal cone, exact nominal nodes, uncertainty boxes, whole-hold
geometry, stable target identities, common generator and permanent terminal
certificate. Earlier state formats must be reset.

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
Hard obstacle and actuator constraints can still conflict at new
admission. After admission, the predictive and invariant-set construction
establishes successor feasibility; CLF slack preserves that feasible family.
A failed numerical solve still stops control.

The exact-state driver saves the optimized slack, its dissipation allowance,
the residual with that allowance removed, and the unrelaxed residual. Offline
experimental success uses the slack-dependent inequality. A positive unrelaxed
residual remains visible and is never described as unrelaxed CLF dissipation.

## Numerical verification and runtime

A native positive solve status with a finite real decision is necessary but
not sufficient. A positive approximate-solve status is permitted only after
the same independent certificate checks; it is reported separately.
Infeasibility, iteration limits and timeouts remain failures. `solveHardCbfClf.certify` checks physical hard rows and terminal
SOCs with arithmetic allowances before a command is returned. The inherited
family may consume part of its original inward reserve to contain the actual
accepted numerical solution, but it may never exceed the physical safety
bound. This avoids repeated tightening invalidating a carried witness.
`postSolveCertificationPerformed=true` reports this check.

Fresh-problem selection checks the complete candidate against the proposed
optimization before its single solve. CLF slack is chosen large enough for
that feasibility witness; it remains optimized in the actual solve.

The normalized objective tolerance remains 1e-7. Safety depends on a verified
feasible solution, not exact objective minimization. Physical feasibility uses
its separate tolerance and independent safety reserves.

`frameDeadlineSeconds` is shared by admission preparation and all search
solves. Numerical solver time limits do not preempt MATLAB preparation or
operating-system scheduling; the actual frame time remains independently
measured against the control hold. An offline diagnostic search budget does
not change that real-time requirement.
The controller has no executable fallback when a mathematically feasible
problem fails to solve on time. See the current dated report under `report/`.
