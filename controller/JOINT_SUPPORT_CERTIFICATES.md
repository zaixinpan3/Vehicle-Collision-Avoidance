# Fixed-direction convex trajectory optimization

Format 41 separates initialization from execution. A bounded geometric search
selects a trajectory anchor and one unit separation direction per collision or
exit record. These directions are then fixed while one hard SOCP optimizes the
complete finite control sequence. A scalar candidate is never issued, even if
it already passes the original verifier. Every new admission requires an
independently verified result from the full trajectory solve. Inherited frames
use their stored directions and attempt one full trajectory improvement; only
a previously optimized, verified suffix can be retained after solve failure.

The certificate covers **hold nodes of the declared zero-residual sampled
affine plant**. It establishes neither inter-node separation nor a real-time
deadline. Clear saved certificates from formats earlier than 41. See
[the implementation and validation](../report/FIXED_DIRECTION_CONVEX_OPTIMIZATION_20260921.md).

## Relationship to Li et al. (2023)

Li, Zhang, Guo, Lenzo and Guo's *Real-Time Optimal Trajectory Planning for
Autonomous Driving with Collision Avoidance Using Convex Optimization*
([publisher, DOI](https://doi.org/10.1007/s42154-023-00222-7)), Section 3,
Eq. (12), computes distance-dual variables from an initialization. Section 3.2,
Eq. (13), fixes those variables and optimizes the complete trajectory. The
local reference PDF was inspected directly. Its final QP includes nonnegative
collision slack. This implementation follows the **initialize directions,
fix directions, optimize trajectory** architecture; it does not reproduce
that QP literally. It retains hard collision acceptance, uncertain rectangular
occupied sets, a yaw support majorant, terminal cones and a soft CLF cone.
Those retained requirements give an SOCP. No collision slack is introduced.

The preceding scalar fast path and joint trajectory/normal improvement are
retired. The present geometric initializer is deliberately incomplete; its
restricted family does not restrict the final control sequence to that line.
Fixing directions still selects an inner subset of the original avoidance
problem. Failure of this subset does not establish inevitable collision.

## Occupied sets and physical residual

For each stored node, the certified chart gives an affine center and yaw,

\[
r(x)=c+Px,\qquad \psi(x)=\psi_0+w^\top x.
\]

The ego and target bodies are centered rectangles with half-size vectors
\(a_E,a_O\). Their yaw uncertainties define the convex hulls \(B_E,B_O\)
of the corresponding rotated rectangles. The target yaw center \(\phi\) is
fixed in the certificate. Position uncertainty is the zonotope
\(G[-1,1]^m\), plus a chart-error disk of radius \(\rho_p\).

The implementation deliberately uses independent position and yaw enclosures.
They can overbound a correlated true set, but they are fixed at admission and
retained on shifting; no later enlargement is hidden in the recursion argument.
With \(n(\theta)=(\cos\theta,\sin\theta)^\top\), the physical residual is

\[
f(x,\theta)=d+\rho_p+h_{B_E}(n(\theta-\psi(x)))
 +h_{B_O}(n(\theta-\phi))+\|G^\top n(\theta)\|_1
 -n(\theta)^\top r(x).
\]

For collision records, \(d=0\): no physical clearance offset is imposed.
The tightened solve and independent arithmetic allowance enforce strictly
positive separation of the declared occupied sets. The removed
`collision.clearanceMargin` setting is not replaced by another physical buffer.
Position/yaw uncertainty, chart remainders and numerical reserves remain.

The physical certificate is \(f\le0\). It implies that every pair of points in the
occupied enclosures is separated by at least \(d\). Positive-clearance
separation of compact convex sets has such a unit normal, although the chosen
enclosures themselves may be conservative. Exit records use a point ego body
and \(d=R_{\rm sensing}+\epsilon_{\rm encounter}\); the target body is retained.

`avoidanceSafetyGeometry.supportValue` is positively homogeneous for arbitrary
arguments, including zero. It does not mistake a nonunit affine argument for a
unit direction. `jointResidual` reconstructs states from the input sequence and
evaluates the exact interval-yaw support independently of solver epigraphs.

## Global majorant with a fixed separation direction

Let $n=(\cos\theta,\sin\theta)^\top$ be fixed, $\psi_0=\psi(x_0)$,
$e_0=R(-\psi_0)n$, $t_E=(-e_{0,2},e_{0,1})$, and
$\Delta\psi=w^\top(x-x_0)$. Only the trajectory is optimized. The unit rotation
has the global Taylor remainder bound

\[
\|R(-\psi(x))n-(e_0-t_E\Delta\psi)\|_2
\le\tfrac12(\Delta\psi)^2.
\]

The ego support function is $R_E=\|a_E\|_2$-Lipschitz, including the interval-yaw
hull. Thus the following convex, globally valid majorant touches the exact
fixed-direction residual at the anchor:

\[
\begin{aligned}
\widehat f(x;n)={}&d+\rho_p+h_{B_E}(e_0-t_E\Delta\psi)
+h_{B_O}(R(-\phi)n)+\|G^\top n\|_1\\
&-n^\top(c+Px)+\tfrac12 R_E(\Delta\psi)^2,\\
f(x,\theta)\le{}&\widehat f(x;n),\qquad
\widehat f(x_0;n)=f(x_0,\theta).
\end{aligned}
\]

The builder enforces $\widehat f\le b$ for the stored numerical bound $b\le0$.
Target shape and target position-uncertainty supports are constants. The ego
support has an affine argument, represented by absolute-value epigraphs for
fixed rectangles or the interval-yaw hull dual. One SOC represents the yaw
quadratic. A point ego body, or a state-independent yaw, needs no such
auxiliaries. Physical pose-domain, input, slew and terminal rows remain hard.

`fixedDirectionMajorant` evaluates this expression directly for diagnostics.
`avoidanceStageQp.fixedDirections` has **no angular decision columns** and
returns `fixedCertificateAngles` as data. The former `angleIndex`, direction
normalization cone, position-direction bilinear majorant, and `positionScale`
configuration are removed. Holding the old angular coordinate at zero would
retain an unnecessary position quadratic; deriving the fixed-direction bound
directly avoids that restriction.

## Remove only majorants made redundant by a hard pose domain

For a curved chart the convex base already imposes a hard box
$|q-q_c|\le r_q$ on $q=(s,d,e_\psi)$. When the position map is
$r=c+P_q q$ (no velocity columns), a fixed unit direction $n_0$ has the
following support bound over the **entire** admitted box:

\[
\begin{aligned}
\overline f={}&d+\rho_p+\|a_E\|_2
+h_{B_O}(R(-\phi)n_0)+\|G^\top n_0\|_1\\
&-n_0^\top(c+P_q q_c)+|P_q^\top n_0|^\top r_q.
\end{aligned}
\]

The ego circumradius bounds every possible orientation, including its yaw
uncertainty. If this bound, with an outward arithmetic allowance, is at most
the stored bound $b$, then $f(x,\theta_0)\le b$ for **every** state in the
convex base. The conic builder omits that record's redundant support
epigraphs and majorant. Its direction remains fixed like every other record.
Exit records are never screened. Without a hard pose box, screening is not
used. A merely safe nominal point is insufficient.

This can only enlarge the feasible set projected onto the trajectory
coordinates, relative to retaining those same majorants: any base-feasible
trajectory already satisfies the omitted physical constraint using the fixed
direction. All original occupied-set records and unit-support residuals remain
in the independent acceptance check and carried certificate. No target is
released or safety obligation shortened by this calculation. Hard pose-domain
rows remain hard in both scalar admission and continuation, and are independently
verified. The same test is repeated against the inherited domain on continuation.

The builder precomputes its final sparse width and auxiliary-variable count.
This avoids repeated width expansion; it is a separate, algebraically exact
implementation improvement. Preallocation alone was checked against complete
conic matrices, vectors, cone partitions and metadata for fixed and uncertain
footprints and hard-performance programs.

## Compact numerical enclosures

Very small positive uncertainty widths must not be silently set to zero.
At **fresh record construction only**, a zonotope whose circumscribing-disk
radius $\sum_j\|G_j\|_2$ is at most the configured numerical tolerance is
replaced by that outward disk. A yaw interval can likewise be replaced by a
fixed-yaw rectangle plus a disk of radius
$\|a\|_2\min(2,r_\psi)$ when this charge is below the same tolerance.
An outward arithmetic allowance is added to each nonzero charge.

These are conservative occupied-set inclusions, not deleted uncertainty and
not a new physical clearance offset. The added disks join `positionBall`;
larger uncertainties retain their original supports. The resulting enclosure
is independently verified and stored at admission, then inherited unchanged.
This avoids introducing a complete interval-yaw dual for widths on the order
of $10^{-10}$ rad without losing the touching property during continuation.

## One signed control section and finite interval computation

For direction initialization, retain the nominal input sequence $U_0$ and restrict
candidate inputs to $U(\alpha)=U_0+\alpha v$. Both amplitude signs are available.
The section is deliberately incomplete; an empty section is not evidence of
physical inevitability of collision.

Choose the target with the largest violated nominal support certificate and
the first, middle and last violated stages for that target. The nominal
relative motion defines a transverse displacement direction in the first
pose chart. A stationary relative-motion tie uses the chart lateral axis.
Compute one minimum-energy input deformation that produces prescribed displacements
in that direction at the selected stages, a final correction to the terminal
reference and a compatible last-input correction. At a trim origin the latter
corrections are zero. If nominal geometry is already clear but the base witness
is invalid, the single direction corrects terminal state/input error instead.
The positive quadratic metric combines the
existing actuator effort weights with input differences weighted by
`encounter.inputRateWeight/sampleTime^2`. A scaled small SVD supplies a
least-norm deformation; all actual constraints are subsequently checked even
if these design equalities are rank deficient. The rule chooses no amplitude
or left/right route. It does restrict the control-sequence shape and the
relative steering/braking timing; that loss of freedom is explicit.

On a curved prediction, the displacement at selected stage $k$ is weighted by

\[
w_k=\rho+(1-\rho)\sin\!\left(\pi\frac{k-k_a}{\max(k_b-k_a,1)}\right),
\qquad \rho=\texttt{admission.temporalShoulderFraction}\in(0,1].
\]

Here $k_a,k_b$ are the first and last violated stages. The default $\rho=0.8$
reduces the shoulders while retaining the middle peak; it addresses premature
chart-boundary excursions without adding another search coordinate. Exactly
straight predictions retain $w_k=1$, avoiding an observed oncoming-admission
regression from unnecessary tapering. Setting $\rho=1$ restores the plateau
on all roads. Terminal interpolation rows and all acceptance constraints are
unchanged. This is one deterministic direction, not a catalog of attempted
time shapes. It can miss plans available to other shapes.

Propagate the origin and direction through the declared affine stages. Actuator,
slew, chart and terminal linear rows reduce to scalar half-lines. A terminal
modal cone has a fixed radius and becomes an interval obtained by projecting
its affine center onto the scalar direction. A negative radius is rejected
before squaring. Their intersection is the hard base interval $I_0$.

Partition $I_0$ into `admission.amplitudeCells` uniform closed cells (default
8). This refines geometry on the **same** control line; it does not create new
control modes or solver starts. For each cell, bound the entire affine nominal
yaw range plus its stored uncertainty. Position maps, target yaw/position
uncertainty and pose-chart remainders retain the original certificate scope.
Use `admission.normalCount` unit directions (default 16), uniformly spaced and
rotated by the initial chart heading. The vectorized analytic rectangle support
checks both yaw endpoints and any interior support maximizer.

For each collision or exit record and a fixed amplitude cell, the reserved
support inequalities take the form

\[
\exists \ell:\quad a_{i\ell}\alpha\ge b_{i\ell}.
\]

Their complement is the open interval

\[
I_i=\bigcap_\ell\{\alpha:a_{i\ell}\alpha<b_{i\ell}\}.
\]

Constant rows are handled explicitly. Sort and subtract these open intervals
from the amplitude cell; touching excluded intervals preserve their common
endpoint. Each cell yields at most $R+1$ closed components for $R$ records.
With $C$ amplitude cells and $L$ normals, geometric work is
$O(C(RL+R\log R))$, excluding prediction, basis construction, objective and
verification. Shared cell endpoints may appear twice without changing safety.
Compared with the previous 32 directions and 16 cells, the default evaluates
one quarter as many cell/record/direction combinations. For an unchanged
control line, fewer support directions and wider yaw-enclosure cells can lose
certifiable amplitudes; they do not authorize a point that fails the independent
original verifier. The changed tapered line is not a subset of the old plateau.
The selected numerical candidate is moved inward where possible and must pass
independent original constraints; endpoint arithmetic alone never authorizes
execution.

Restrict the original quadratic performance objective to the line. Eliminate
only the first-hold CLF slack through
$\delta(\alpha)=\max(0,\|a+b\alpha\|-c)$, where $c\ge0$. The resulting objective
is convex. A fixed `admission.performanceIterations` derivative bisection
(default 32) locates an approximate unconstrained minimizer on $I_0$; project
it onto each certified component and choose the best scalar cost. The iteration
limit affects performance accuracy, not the requirement for a hard certificate.
No collision or terminal slack is introduced.

After this initialization, a full fixed-direction convex trajectory solve is
mandatory. The complete control sequence is free; it need not retain the
initializer's steering/braking timing or scalar amplitude. An initially
certified nominal also proceeds through this solve. Inherited successors use
their retained directions and attempt the same full optimization. Empty base,
terminal exclusion, collision exclusion, exit exclusion and verification
failure remain distinct initializer outcomes. A failed section can still
supply the least-violated anchor below; failure to initialize or to obtain a
verified full-plan admission issues no command.

The common full affine sensitivity maps and canonical condensed objective are
still constructed for compatibility with the complete inherited family. The
scalar search propagates its two trajectories and accumulates its objective
directly; its improvement is not a claim that all preparation is linear in the
horizon. Finite loop counts and observed desktop timings are not a WCET proof.

## Initialization after section exclusion

The scalar search can exclude every amplitude even when a hard-certified plan
exists. The circular-crossing diagnosis demonstrated an early chart-boundary
peak coupled to a later collision bottleneck. Since the restriction is in the
control sequence, merely refining its normals or amplitude cells cannot remove it.

After collision or exit exclusion, evaluate the exact dictionary residuals at
the existing amplitude-cell boundaries, nudged inward at the base endpoints.
For each proposal alpha, define the worst record residual

\[
v(\alpha)=\max_i\min_{n\in\mathcal D}r_i(U_0+\alpha v,n).
\]

Select the least-violated proposal that passes the original base/terminal/CLF
checks, completing only its permitted CLF slack. It may still violate collision
or exit certificates. `admitSection` returns it in a separate fourth output;
its admitted `decision` output stays empty. The proposal is neither issued nor
stored as an incumbent. This bounded scan chooses one initialization, not a
library of predetermined trajectories or temporal weights.

Build the fixed-direction SOCP around the selected candidate or proposal.
All input coordinates are free, while all selected directions are constants.
The support majorants are valid at infeasible centers as well as feasible
ones. A feasible solution of this inner problem can therefore satisfy the
original certificates even when its center does not. Touching preserves a
feasible inherited anchor; it does not guarantee fresh admission.

At most one full trajectory solve is attempted, subject to the existing work
timer and full-frame deadline. A positive solver status proceeds through the
independent original verifier. A failed fresh solve, unverified solution or
expired frame issues no command. Metadata reports
`policy = "fixedDirectionTrajectoryOptimization"`, `fixedCertificateAngles`,
`directionSeedSource`, and `fullPlanStatus`. Every successful fresh admission
has `usedFullPlanAdmission = true` and one native solve;
`issuedAdmissionWitness = false` always. `restorationSolverCallCount` is zero:
this is the main trajectory stage, not a recovery-only call.

The generated prepared-frame adapter has status 4 for optimized admission,
2 for optimized continuation, 3 for a retained previously optimized suffix,
and 0 for rejection. There is no direct-scalar status 1 branch. Rebuild
compiled adapters after this change. This remains a prepared-frame benchmark,
not a complete standalone real-time controller.

## Hard acceptance and continuation

A scalar candidate never authorizes execution by itself. Acceptance reconstructs the input
trajectory and independently checks every physical base row, terminal/CLF
cone and exact nonlinear unit-support residual with arithmetic allowances.
Only a full-plan solver result may become a new executable admission after
these checks; passing them does not authorize direct issuance of a scalar seed.
The CLF is the only softened physical optimization condition. The next frame improves the ordinary performance
objective. An already certified continuation can be retained if its attempted
improvement fails, with the actual numerical status reported. The complete
frame deadline still prohibits late command issuance.

The accepted record contains the input plan, angles, occupied-set parameters,
charts, dynamics, terminal witness, absolute exit time and numerical bounds.
On a compatible update, the executed prefix is eliminated and remaining records
are shifted without recomputing their geometry. Conditioning intersects true
sets with inherited enclosures. For the inherited direction, narrowing either
occupied set can only increase its separating gap. The previous suffix
therefore remains physically feasible. Touching at this complete suffix makes
the next hard conic subproblem nonempty in exact arithmetic. Numerical reserve
transfer follows the existing physical-verifier policy, without imposing a
fresh inward margin at every shift.

This is **conditional recursive subproblem feasibility**, not guaranteed solver
completion or indefinite safety against future targets. New targets, larger
motion/sensing bounds, changed execution contracts or an unconfirmed exit
require admission or rejection. Target release does not prove that the target
can never return. Nonempty physical road-boundary sets remain unsupported by
the existing terminal construction. Whole-hold separation would require a
certificate for a complete hold enclosure and is outside this change.

## Implementation and checks

- `avoidanceSafetyGeometry`: occupied-set records, homogeneous support, yaw
  hulls, exact nonlinear residuals, majorant evaluation and verification.
- `avoidanceStageQp`: sparse stage lifting, fixed-direction support epigraphs and SOC
  yaw majorants. The original dynamics and terminal blocks remain sparse.
- `solveHardCbfClf`: scalar direction initialization, full fixed-direction optimization,
  physical acceptance and the certified-incumbent interface.
- `formulateAvoidanceProblem` and `hardEncounterBarrier`: preserve and shift
  the entire certificate; collision and exit angles share the same method.
- `jointSupportCertificateTest`: homogeneous/interval support, 2,000 seeded
  majorant checks, global majorants beyond the former trust domain, hard
  admission, uncertain multiple targets, suffix preservation and deadlines.
- `affinePlanAdmissionTest`: scalar half-lines, modal balls, open interval
  subtraction, complete yaw support, mandatory full-plan admission and hard rejection.

The implementation adds no core source file or third-party dependency. Its
twenty-file source budget remains in force. `certificateContinuationTest`
checks fixed-direction admission, suffix containment, changed sensing contracts,
rejection of older saved states and target-free renewal.
