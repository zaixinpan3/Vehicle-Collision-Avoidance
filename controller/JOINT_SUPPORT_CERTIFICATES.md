# Fixed-direction convex trajectory optimization

Format 42 separates initialization from execution. An NRMM-timed VFFM reference
initializes a trajectory anchor and one unit separation direction per collision or
exit record. These directions are then fixed while one hard SOCP optimizes the
complete finite control sequence. An initialization seed is never issued.
Every new admission requires a successful full trajectory solve, and a
solver-reported success is issued as returned. No verification code runs after
the solve. Inherited frames use their stored directions and attempt one full
trajectory improvement; if that solve fails, the shifted previous plan is
issued.

The certificate covers **hold nodes of the declared zero-residual sampled
affine plant**. It establishes neither inter-node separation nor a real-time
deadline. Clear saved certificates from formats earlier than 42. See
[the implementation and validation](../report/CHENG_FLUID_INITIALIZATION_ADOPTION_20260921.md).

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
reference shape does not restrict the final optimized control sequence.
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
The tightened solve enforces strictly positive separation of the declared
occupied sets, up to the solver's primal feasibility tolerance. The removed
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
auxiliaries. Physical input, slew and terminal rows remain hard.

`fixedDirectionMajorant` evaluates this expression directly for diagnostics.
`avoidanceStageQp.fixedDirections` has **no angular decision columns** and
returns `fixedCertificateAngles` as data. The former `angleIndex`, direction
normalization cone, position-direction bilinear majorant, and `positionScale`
configuration are removed. Holding the old angular coordinate at zero would
retain an unnecessary position quadratic; deriving the fixed-direction bound
directly avoids that restriction.

## Curved-road charts carry no pose box

On a curved reference the ego Frenet state is mapped to Cartesian position
and yaw by an affine chart around the seed pose of each node, and each record
charges the chart remainder computed for a range of `poseTrustRadius` around
that pose. By project decision (2026-09-23) this range is **not** enforced:
the former pose-domain and terminal pose-domain rows are removed, and so is
the screening that omitted a record whose fixed direction was safe over the
whole box. Every collision and exit record is therefore always a constraint.
Once a plan leaves the chart range, the charged remainder is no longer
guaranteed to bound the chart error, so the curved-road collision and exit
certificates are no longer rigorous there. Roads without a curved reference
chart never had pose-domain rows and are unaffected. The release test evaluates the carried exit direction in
a chart centered on the observed state when the carried chart does not cover
it.

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
is stored at admission, then inherited unchanged.
This avoids introducing a complete interval-yaw dual for widths on the order
of $10^{-10}$ rad without losing the touching property during continuation.

## NRMM-based time-dependent VFFM initialization

The current initializer is specified in [NRMM_VFFM_INITIALIZATION.md](NRMM_VFFM_INITIALIZATION.md).
It propagates the nominal target analytically at the ego arrival times, constructs
normal-coordinate charts from the selected quadratic road boundaries, and
superposes moving Gaussian contributions from every nominally conflicting
target. Conflict-derived passing ordinates and relative-station widths are
design parameters. Two joint passing assignments share one terminal-preserving
affine-model fit. Body-yaw preferences subtract anchor sideslip from reference
course; analytical derivatives retain target acceleration, turning and road
width variation.

The fitted rollout fixes the measured initial state and preserves the nominal
terminal state/input. Prefer a fit satisfying the physical base rows, then
minimize the worst reserved support residual within the same feasibility
class. Analytic rectangle signed-distance normals initialize exactly one full
fixed-normal SOCP. All controls remain free in that solve. The seed cannot be
issued or retained as an executable fallback. Inherited optimized certificates
retain their directions and do not regenerate the reference.

Compute base-row feasibility before candidate normals. If a finite-scored
physical fit is already selected, a later physically infeasible fit cannot
win this ranking and needs no normal/support evaluation. Preserve the original
candidate order and tie rule. Fresh joint geometry also omits projection of
preliminary collision rows that the joint representation would remove; occupied
sets, normals, road/pose rows and final hard support inequalities are retained.

`program.fluidReference` contains numeric reference preparation shared with
the generated prepared-frame adapter. Metadata retains
`directionSeedSource = "chengFluidReference"` for compatibility; its
`initialization` reports the target-active flag, two support scores, two physical
base-row excesses, target amplitude/width/conflict interval,
terminal-fit error and selected seed residuals. `usedFullPlanAdmission` is
true for successful fresh admission and `issuedAdmissionWitness` stays false.
An unevaluated candidate's support score remains `Inf`, including a candidate
screened by the physical ranking; it is not a measured support violation.
Adapter status 4 denotes optimized admission, 2 optimized continuation,
3 retained shifted previous plan, and 0 rejection. Rebuild generated adapters
after changing their prepared input structures.

This extends the 2021 Cheng reference rule; it is not a fluid PDE solution.
The online bounded Cartesian target model remains the hard safety model.
The initializer supports selected route corridors rather than general unions
of road patches. The unchanged node certificate does not establish all-time
separation. Two passing assignments are incomplete, and failure does not prove
that no feasible trajectory exists.

## Hard acceptance and continuation

An initialization seed never authorizes execution by itself. Only a full-plan
solver result may become a new executable admission. Acceptance is the solver
status: Solved and the reduced-accuracy AlmostSolved are issued as returned
(`approximateSolveAccepted` marks the latter). The controller contains no
post-solve verification: physical rows, terminal/CLF cones and the exact
nonlinear unit-support residuals hold only to the solver's feasibility
tolerance, and a false solver success is issued unchecked. `planCertified`
reports acceptance. Infeasible, iteration-limit, timeout and non-finite
results issue no command on a fresh frame.
The CLF is the only softened physical optimization condition. The next frame improves the ordinary performance
objective. If that improvement fails on an inherited frame, the shifted
previous plan is issued without any check, with the actual numerical status
reported. The complete frame deadline still prohibits late command issuance.

The accepted record contains the input plan, angles, occupied-set parameters,
charts, dynamics, terminal witness, absolute exit time and numerical bounds.
On a compatible update, the executed prefix is eliminated and remaining records
are shifted without recomputing their geometry. Conditioning intersects true
sets with inherited enclosures. For the inherited direction, narrowing either
occupied set can only increase its separating gap. The previous suffix
therefore remains physically feasible. Touching at this complete suffix makes
the next hard conic subproblem nonempty in exact arithmetic. The shifted
suffix is not checked before the next solve; it serves as the optimizer center
and as the plan retained on solve failure. The inward reserves of the admission
rows shift unchanged.

This is **conditional recursive subproblem feasibility**, not guaranteed solver
completion or indefinite safety against future targets. New targets, larger
motion/sensing bounds, changed execution contracts or an unconfirmed exit
require admission or rejection. Target release does not prove that the target
can never return. Nonempty physical road-boundary sets remain unsupported by
the existing terminal construction. Whole-hold separation would require a
certificate for a complete hold enclosure and is outside this change.

## Implementation and checks

- `avoidanceSafetyGeometry`: occupied-set records, homogeneous support, yaw
  hulls, exact nonlinear residuals, majorant evaluation and direction storage.
- `avoidanceStageQp`: sparse stage lifting, fixed-direction support epigraphs and SOC
  yaw majorants. The original dynamics and terminal blocks remain sparse.
- `solveHardCbfClf`: fluid-reference direction initialization, full fixed-direction optimization,
  solver-status acceptance and retention of the shifted previous plan.
- `formulateAvoidanceProblem` and `hardEncounterBarrier`: preserve and shift
  the entire certificate; collision and exit angles share the same method.
- `jointSupportCertificateTest`: homogeneous/interval support, 2,000 seeded
  majorant checks, global majorants beyond the former trust domain, hard
  admission, uncertain target geometry, suffix preservation and deadlines.
- `fluidInitializationTest`: mirrored crossing admission, terminal-fit accuracy,
  side selection, retired-option rejection, fixed normals and exported-frame parity.
- `admissionSafetyTest`: mandatory full-plan admission and hard rejection.

The implementation adds no core source file or third-party dependency. Its
twenty-file source budget remains in force. `certificateContinuationTest`
checks fixed-direction admission, suffix containment, changed sensing contracts,
rejection of older saved states and target-free renewal.
