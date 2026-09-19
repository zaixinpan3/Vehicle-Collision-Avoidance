# Joint trajectory and separation certificates

The controller optimizes the input sequence and
one separation angle per target/node, including each target's terminal exit
angle. Fresh admission uses one geometric initialization and at most three
conic solves by default. A fully verified admission witness is issued directly;
subsequent frames improve its performance within the carried certificate. Joint support is the sole
controller algorithm; there is no certificate-method selector. It certifies
**hold nodes of the declared zero-residual sampled affine plant**. It does not
certify the space traversed between nodes or a real-time admission deadline.

```matlab
cfg = collisionAvoidanceControllerConfig();
```

Start with an empty saved state after upgrading. Certificates use stored
state format 39; earlier formats are rejected. The public controller interface,
actuator limits, terminal continuation, sensing contracts and soft CLF remain.
The actual paired results and reproduction commands are in
[the bounded-admission report](../report/BOUNDED_ADMISSION_20260919.md).

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
rows remain enforced during restoration, and are themselves independently
verified. The same test is repeated against the inherited domain on continuation.

The builder precomputes its final sparse width and auxiliary-variable count.
This avoids repeated width expansion; it is a separate, algebraically exact
implementation improvement. Preallocation alone was checked against complete
conic matrices, vectors, cone partitions and metadata for fixed and uncertain
footprints, restoration and hard-performance programs.

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

## One geometric initialization and a bounded search

A single hard restriction about an overlapping cruise prediction can be empty
even when another restriction admits an escape. For example, with relative
center $(0.9,y)$, $0\le y\le2$, square obstacle $[-1,1]^2$, clearance $0.1$ and
initial direction $(1,0)$, both the old angular majorant and the new homogeneous
majorant have a positive minimum at the stationary anchor. The physical point
$(y,\theta)=(1.2,\pi/2)$ is nevertheless safe. Removing road boundaries cannot
remove this local-convexification obstruction.

`admissionAngles` generates **one** coherent initialization per encounter:

1. Retain nominal controls and states exactly. For a target with violated
   nominal certificates, obtain an orientation from the nominal relative
   displacement across the prediction window, transverse to the first chart.
   Initial relative position breaks a near-zero motion tie; exact symmetry
   uses a deterministic reference-coordinate tie rule.
2. Project each nominal relative position into the support half-space for
   that orientation, using the actual footprint/uncertainty support and
   numerical bound. Query the rectangle oracle at these geometric points to
   obtain a coherent schedule of initial directions.
3. Discard the projected points. They are not a vehicle rollout, reference,
   dynamic constraint, prescribed lateral displacement or maneuver timing.
   The controls and directions remain optimization decisions. Exit directions
   retain their own nominal initialization and are optimized jointly.

There is no alternative-start loop, route enumeration or search over a list of
left/right/forward/backward initializations. The orientation rule is a heuristic
for finding a useful convex family; it is not a completeness theorem. Multiple
targets still share one coupled trajectory solve.

The convex base $\mathcal D$ contains hard dynamics, actuator/slew, chart and
terminal constraints plus the soft CLF. If the initial point is outside this
base, one base solve is charged to the same admission call budget. Search then
solves $\widehat F_i\le e$, $e\ge0$, minimizing $e$ plus a small proximal term.
At a base-feasible anchor, the current exact maximum violation provides a
feasible deficit. The exact unit-support violations are recomputed after each
candidate, and only nonincreasing progress is retained.

`maximumAdmissionSolves` defaults to **3**, counting a base solve if needed.
There is one initialization, no horizon retry and no mandatory performance
solve after a candidate passes complete hard verification. Continuation uses
one hard performance solve containing its verified shifted witness. Search
failure means no certificate was found in this bounded local search, not
physical inevitability of collision or a global infeasibility result.

The native iteration limit, shared search clock and full-frame acceptance
deadline still apply. A bound on optimization calls is not a wall-clock WCET
proof; timings and success rates must be measured separately.

## Hard acceptance and continuation

Search deficits never authorize execution. Acceptance reconstructs the input
trajectory and independently checks every physical base row, terminal/CLF
cone and exact nonlinear unit-support residual with arithmetic allowances.
A search iterate may be issued only when these original hard checks pass;
its auxiliary conic deficit is neither a physical safety relaxation nor part
of the stored control plan. The next frame improves the ordinary performance
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
- `solveHardCbfClf`: base feasibility, bounded single-initialization admission, hard continuation improvement,
  physical acceptance and the certified-incumbent interface.
- `formulateAvoidanceProblem` and `hardEncounterBarrier`: preserve and shift
  the entire certificate; collision and exit angles share the same method.
- `jointSupportCertificateTest`: homogeneous/interval support, 2,000 seeded
  majorant checks, global majorants beyond the former trust domain, projected square escape, unsafe base rejection, hard
  admission, uncertain multiple targets, suffix preservation and deadlines.

The implementation adds no core source file or third-party dependency. Its
twenty-file source budget remains in force. `certificateContinuationTest`
checks default joint admission, suffix containment, changed sensing contracts,
rejection of older saved states and target-free renewal.
