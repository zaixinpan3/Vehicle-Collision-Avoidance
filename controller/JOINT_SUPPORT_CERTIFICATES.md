# Joint trajectory and separation certificates

Fresh admission first searches one signed affine section using a finite
support dictionary. If the section has a nonempty hard base interval but no
certified amplitude, one hard joint solve may release the complete input
sequence around its least-violated geometric proposal. Inherited performance
improvement optimizes the input sequence and one
separation angle per target/node, including terminal exit angles, in one SOCP.
Every issued plan passes the original hard physical verification. Admission
recovery has no collision slack or executable uncertified incumbent. The certificate covers **hold nodes of
the declared zero-residual sampled affine plant**, not the space between nodes
or a real-time deadline.

```matlab
cfg = collisionAvoidanceControllerConfig();
```

Start with an empty saved state after upgrading to format 40. The public
interface, actuator limits, terminal continuation, sensing contracts and soft
CLF remain. The [affine-section review](../report/AFFINE_SECTION_ADMISSION_REVIEW_20260919.md)
states the mathematical prerequisites;
[the implementation report](../report/AFFINE_SECTION_ADMISSION_20260919.md)
records the measured runtime and admission-capability tradeoff.
The [September 21 admission repair](../report/CIRCULAR_CROSSING_ADMISSION_REPAIR_20260921.md)
supersedes its first-admission search restriction; certificate format 40 remains
unchanged.

## Research assessment

The proposed distinction between search feasibility and executable safety is
valid. Inspection of the retired baseline showed that it installed a new
fixed-normal family even when that family excluded the shifted witness.
That replacement branch has been removed. The controller preserves the admitted occupied
sets, charts, angular directions, terminal set and absolute exit deadline.

Li, Zhang, Guo, Lenzo and Guo's *Real-Time Optimal Trajectory Planning for
Autonomous Driving with Collision Avoidance Using Convex Optimization*
([2023, DOI](https://doi.org/10.1007/s42154-023-00222-7)), page 6, Eq. (13),
does contain nonnegative collision slack. Its two-stage fixed-dual construction
does not establish the hard-admission property required here. The local
reference PDF was inspected directly.

Zhang, Liniger and Borrelli's
[Optimization-Based Collision Avoidance](https://arxiv.org/html/1711.03449),
Section 4 and Theorem 2, supports an exact lifted certificate viewpoint for
convex occupied sets, with the resulting trajectory problem remaining
nonconvex. The present angular majorization is a project implementation and
derivation, not a claim that OBCA supplies this conic algorithm or a novelty
claim relative to the entire literature.

[CFS](https://arxiv.org/html/1709.00627v3) and
[SCvx](https://arxiv.org/html/1804.06539) provide relevant local-convexification
frameworks, with problem-specific assumptions. SCvx's feasible-limit/local
optimality statements do not establish admission of every finite iterate.
[SCvx-fast](https://arxiv.org/html/2112.00108v1), Algorithm 1 and Theorem 3.1,
also cannot be imported as a feasibility proof for this actuator-limited
terminal problem without checking the full intersection of constraints.

[Predictive safety filtering using system level synthesis](https://proceedings.mlr.press/v211/leeman23a.html)
provides context for retaining an admissible continuation under explicit
uncertainty contracts. [Drusvyatskiy and Lewis](https://arxiv.org/abs/1602.06661)
provide error-bound/proximal convergence analysis; the local error bound and
absence of positive-violation stationary points needed for a stronger admission
theorem have **not** been established for this controller.

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

## A homogeneous global majorant

At an anchor $(x_0,\theta_0)$ write $n_0=n(\theta_0)$,
$t_0=(-n_{0,2},n_{0,1})$, $r_0=c+Px_0$, $\Delta r=P(x-x_0)$ and
$\Delta\psi=w^\top(x-x_0)$. Optimize a **tangent coordinate** $a\in\mathbb R$:

\[
\widetilde n=n_0+a t_0,\qquad
\ell=\|\widetilde n\|_2=\sqrt{1+a^2}\ge1,\qquad
\theta=\theta_0+\arctan a.
\]

The vector can never be zero. Its magnitude is immaterial to separation, and
its normalized direction is precisely $n(\theta)$. The physical certificate
still stores unit angles; `angleIndex` identifies tangent-coordinate decision
columns in the conic interface, **not additive angle increments**.

Let $b\le0$ be the inherited or initial numerical bound on the physical
residual. Positive homogeneity gives the equivalent certificate

\[
\begin{aligned}
F=\ell(f-b)={}&(d+\rho_p-b)\ell
+h_{B_E}(R(-\psi)\widetilde n)
+h_{B_O}(R(-\phi)\widetilde n)\\
&+\|G^\top\widetilde n\|_1
-\widetilde n^\top(r_0+\Delta r)\le0.
\end{aligned}
\]

The target and position-uncertainty supports now have **affine arguments**.
They need no unit-circle Taylor remainder depending on target distance.
For the ego, set $n_E=R(-\psi_0)n_0$, $t_E=R(-\psi_0)t_0$, and
$v_E=n_E+t_E(a-\Delta\psi)$. The rotation remainder satisfies globally

\[
\|R(-\psi)\widetilde n-v_E\|_2
\le\tfrac12(\Delta\psi)^2+|a\Delta\psi|
\le\tfrac12a^2+(\Delta\psi)^2.
\]

The support function is $R_E=\|a_E\|_2$-Lipschitz, also for an interval-yaw
hull. For any $\eta>0$, the remaining position-direction bilinear term has
another global, touching convex bound:

\[
-a\,t_0^\top\Delta r
\le\tfrac14\left(\sqrt\eta\,t_0^\top\Delta r-a/\sqrt\eta\right)^2.
\]

Subtracting the left side from the right gives the nonnegative square
$(\sqrt\eta\,t_0^\top\Delta r+a/\sqrt\eta)^2/4$. Therefore the implemented
majorant is

\[
\begin{aligned}
\widehat F={}&(d+\rho_p-b)\sqrt{1+a^2}
+h_{B_E}(v_E)+h_{B_O}(R(-\phi)\widetilde n)
+\|G^\top\widetilde n\|_1\\
&-n_0^\top(r_0+\Delta r)-a\,t_0^\top r_0
+\tfrac12R_E\bigl(a^2+2(\Delta\psi)^2\bigr)\\
&+\tfrac14\left(\sqrt\eta\,t_0^\top\Delta r-a/\sqrt\eta\right)^2.
\end{aligned}
\]

It is convex, $F\le\widehat F$ globally, and equality holds at the anchor.
`positionScale` selects $\eta=1/\texttt{positionScale}$; it does **not** limit
displacement. The former 4 m position and 0.5 rad angle trust bounds are
removed because this derivation does not need them. The certified pose/chart
domains, dynamics, actuator and slew limits and terminal constraints remain
hard. Each subproblem covers the open directional hemisphere about its
anchor; it is still a local inner approximation, not the complete nonconvex set.

`jointMajorant` evaluates this expression at $b=0$ for diagnostics; it bounds
$\ell f$, not the unscaled unit residual. The conic builder includes $-b\ell$.
Support epigraphs use absolute values for fixed rectangles and the exact
interval-yaw hull dual otherwise. The disk norm and the positive quadratic
terms use ordinary second-order cones. All couplings remain stage-local.

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
convex base. The conic builder fixes that record's tangent coordinate to zero
with an equality and omits its redundant support epigraphs and majorant.
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

For the fast admission attempt, retain the nominal input sequence $U_0$ and restrict
candidate inputs to $U(\alpha)=U_0+\alpha v$. Both amplitude signs are available.
The section is deliberately incomplete; an empty section is not evidence of
physical inevitability of collision.

Choose the target with the largest violated nominal support certificate and
the first, middle and last violated stages for that target. The nominal
relative motion defines a transverse displacement direction in the first
pose chart. A stationary relative-motion tie uses the chart lateral axis.
Compute one minimum-energy input deformation that produces unit displacements
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

Propagate the origin and direction through the declared affine stages. Actuator,
slew, chart and terminal linear rows reduce to scalar half-lines. A terminal
modal cone has a fixed radius and becomes an interval obtained by projecting
its affine center onto the scalar direction. A negative radius is rejected
before squaring. Their intersection is the hard base interval $I_0$.

Partition $I_0$ into `admission.amplitudeCells` uniform closed cells (default
16). This refines geometry on the **same** control line; it does not create new
control modes or solver starts. For each cell, bound the entire affine nominal
yaw range plus its stored uncertainty. Position maps, target yaw/position
uncertainty and pose-chart remainders retain the original certificate scope.
Use `admission.normalCount` unit directions (default 32), uniformly spaced and
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

A successful restricted admission needs no conic solve. If the initial nominal
already passes the complete verifier, one ordinary hard performance improvement
is permitted. Inherited successors also attempt one such improvement while
retaining the verified incumbent. Empty base, terminal exclusion, collision
exclusion, exit exclusion and independent-verification failure remain distinct
section outcomes. A failed section can initialize the hard recovery below;
failure of the complete admission search issues no command.

The common full affine sensitivity maps and canonical condensed objective are
still constructed for compatibility with the complete inherited family. The
scalar search propagates its two trajectories and accumulates its objective
directly; its improvement is not a claim that all preparation is linear in the
horizon. Finite loop counts and observed desktop timings are not a WCET proof.

## Release the time shape after section exclusion

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

Build the same global joint-support SOCP used for continuation around this
proposal and its best dictionary directions. All input coordinates and support
angle increments are free; physical rows, chart radii/remainders, terminal
cones, encounter-exit requirements and the performance objective remain hard
or soft exactly as before. The support majorants are valid at infeasible
centers as well as feasible ones. A feasible solution of this inner problem
can therefore satisfy the original certificates even though its center does
not. The touching argument guarantees inclusion of a feasible incumbent only
for continuation; it does not guarantee that admission recovery succeeds.

At most one such solve is attempted, subject to the original work timer and
full-frame deadline. A positive solver status proceeds to the independent
original verifier. Failure, an unverified solution or an expired frame issues
no command. No collision slack is added. Metadata distinguishes the rejected
section, the proposal violation, the full-plan solve and its verification.
`restorationSolverCallCount` counts this one hard admission recovery when used;
the scalar fast path still makes zero conic calls.

The generated replay adapter implements the same branch with status 4 for a
verified full-plan admission and an empty decision on failed admission. Existing
compiled replay binaries must be rebuilt after this source change. It remains
a prepared-frame benchmark rather than a complete real-time controller.

## Hard acceptance and continuation

A scalar candidate never authorizes execution by itself. Acceptance reconstructs the input
trajectory and independently checks every physical base row, terminal/CLF
cone and exact nonlinear unit-support residual with arithmetic allowances.
A candidate may be issued only when these original hard checks pass.
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
- `avoidanceStageQp`: sparse stage lifting, angular/support variables and SOC
  majorants. The original dynamics and terminal blocks remain sparse.
- `solveHardCbfClf`: scalar interval admission, hard continuation improvement,
  physical acceptance and the certified-incumbent interface.
- `formulateAvoidanceProblem` and `hardEncounterBarrier`: preserve and shift
  the entire certificate; collision and exit angles share the same method.
- `jointSupportCertificateTest`: homogeneous/interval support, 2,000 seeded
  majorant checks, global majorants beyond the former trust domain, hard
  admission, uncertain multiple targets, suffix preservation and deadlines.
- `affinePlanAdmissionTest`: scalar half-lines, modal balls, open interval
  subtraction, complete yaw support, zero-conic admission and hard rejection.

The implementation adds no core source file or third-party dependency. Its
twenty-file source budget remains in force. `certificateContinuationTest`
checks default joint admission, suffix containment, changed sensing contracts,
rejection of older saved states and target-free renewal.
