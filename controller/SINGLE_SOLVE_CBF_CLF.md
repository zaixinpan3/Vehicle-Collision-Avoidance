# Predictive safety with one shifted-nominal convexification

This document describes the default `fixedNormal` baseline. The selectable
`jointSupport` research implementation has free separation angles, unexecuted
restoration and complete-certificate continuation; see
[JOINT_SUPPORT_CERTIFICATES.md](JOINT_SUPPORT_CERTIFICATES.md).

> Certificate sampling note (2026-09-17): the online controller now certifies the safety rows at the hold nodes of the exact sampled affine plant only; statements below about whole-hold, swept or Bernstein coverage hold at the nodes and no longer claim inter-node coverage. See [NODE_SAMPLED_CERTIFICATE.md](NODE_SAMPLED_CERTIFICATE.md).

The format-35 controller retains a complete finite input plan, a rectangle
collision certificate at every hold node, finite confirmed exit and a permanent
terminal information-state set. Both the predictor and terminal set use the same
admitted held affine cruise generator. Physical input effort remains centered
on the CLF/LQR trim; the first-hold CLF has a nonnegative squared-penalty slack.
The target high-gain observer is unchanged.

The geometric algorithm and its relation to Li et al. (2023) are specified in
[SUPPORT_CONVEXIFICATION.md](SUPPORT_CONVEXIFICATION.md).

## Actual next-frame optimization

1. Shift the stored prediction by one hold and preserve its generators,
   charts, terminal certificate and absolute exit deadline.
2. Compute one analytic signed-distance direction per nominal node/target.
   Fix these directions for the entire current optimization.
3. Solve the complete hard-safety/soft-CLF SOCP once, then independently
   certify its solution. Exact duplicate-row reduction and sparse lifting
   remain; branch search, restoration and constraint generation are removed.
4. On confirmed release, remove that target's obligations. Target-free horizon
   renewal retains its witness inclusion check. At exhaustion, a free
   invariant one-hold optimization continues; terminal feedback is not issued.

Recomputed active collision rows can exclude the old witness. The reported
`shiftedWitnessContained` tests the current update, while
`recursiveFeasibilityGuaranteed` is false during active encounters. It remains
true for target-free continuation under the declared contracts. Retaining a
finite predictive certificate alone does not prove feasibility of every new
normal family. The active theorem requires the additional premise stated in
[TERMINAL_CBF_PROOF.md](TERMINAL_CBF_PROOF.md).

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
no-road-boundary experiment specification. Actuator amplitude/slew and the
collision/terminal certificates are hard, including the local pose-domain
rows required to validate the circular affine geometry. Those domains are centered
on fresh nominal anchors and are retained by accepted continuations; they
are not physical road or tire-slip constraints. Finite
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

The returned format-35 state contains the plan, verified inherited affine
bounds and terminal cone, exact nominal nodes, uncertainty boxes, hold-node
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
admission. The predictive and invariant-set construction supplies a feasible suffix
in the retained family. Recomputed active collision rows must additionally
contain a feasible candidate; CLF slack cannot repair incompatible hard rows.
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

`frameDeadlineSeconds` is shared by admission preparation and the single
solve. Numerical solver time limits do not preempt MATLAB preparation or
operating-system scheduling; the actual frame time remains independently
measured against the control hold. An offline diagnostic search budget does
not change that real-time requirement.
The controller has no executable fallback when a mathematically feasible
problem fails to solve on time. See the current dated report under `report/`.

## Smooth variable-curvature references

Profiles use a fixed reference-phase sequence of held affine models, backward
Riccati five-error CLF matrices, and a verified six-dimensional terminal family.
The online objective still penalizes path/velocity error, trim-input deviation
and squared CLF slack. Whole-hold phase domains prevent arbitrary mismatch
between the scheduled model and actual station. The reference/terminal setup is
explicitly measured separately from periodic frames. See
[the full construction and limitations](CURVED_CRUISE_CERTIFICATE.md).
