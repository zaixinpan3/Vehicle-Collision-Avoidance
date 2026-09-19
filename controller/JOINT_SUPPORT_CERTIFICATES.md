# Joint trajectory and separation certificates

The controller optimizes the input sequence and
one separation angle per target/node, including each target's terminal exit
angle. It restores feasibility before admission and carries the complete
accepted certificate into subsequent frames. Joint support is the sole
controller algorithm; there is no certificate-method selector. It certifies
**hold nodes of the declared zero-residual sampled affine plant**. It does not
certify the space traversed between nodes or a real-time admission deadline.

```matlab
cfg = collisionAvoidanceControllerConfig();
```

Start with an empty saved state after upgrading. Certificates use stored
state format 37; earlier formats are rejected. The public controller interface,
actuator limits, terminal continuation, sensing contracts and soft CLF remain.
The actual paired results and reproduction commands are in
[the September 19 report](../report/JOINT_SUPPORT_CERTIFICATES_20260919.md).

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

## Touching convex majorant

Let \(t(\theta)=(-\sin\theta,\cos\theta)^\top\). For any real increment,

\[
\|n(\alpha+\delta)-n(\alpha)-t(\alpha)\delta\|_2\le\tfrac12\delta^2.
\]

Support functions of radius-\(R\) sets are \(R\)-Lipschitz. Therefore
\(h_B(n+t\delta)+R\delta^2/2\) is a convex upper bound touching
\(h_B(n(\alpha+\delta))\) at zero. No differentiation through a selected
rectangle corner is needed.

At a center \((x^\ell,\theta^\ell)\), define
\(\Delta r=P\Delta x\), \(\Delta\psi=w^\top\Delta x\), and use the hard
step domain \(\|\Delta r\|_2\le R_{\rm step}\).
Set \(R_r=\|r^\ell\|_2+R_{\rm step}\), \(\eta=1/R_{\rm step}>0\),
\(R_E=\|a_E\|_2\), \(R_O=\|a_O\|_2\), and
\(R_U=\sum_j\|G_j\|_2\). The upper bound used by the conic builder is

\[
\begin{aligned}
\widehat f^\ell={}&d+\rho_p+h_{B_E}(a_e)+h_{B_O}(a_o)+\|G^\top a_u\|_1
 -(n^\ell)^\top r^\ell-(n^\ell)^\top\Delta r
 -(t^\ell)^\top r^\ell\Delta\theta\\
&+\tfrac12\{R_E(\Delta\theta-\Delta\psi)^2
 +(R_O+R_U+R_r+\eta^{-1})(\Delta\theta)^2
 +\eta\|\Delta r\|_2^2\},
\end{aligned}
\]

where \(a_e,a_o,a_u\) are the respective first-order unit-circle arguments.
The relative-position cross term is bounded by Young's inequality; its
remaining angular remainder is charged against \(R_r\). Thus
\(f\le\widehat f^\ell\) on the declared step domain and
\(f(x^\ell,\theta^\ell)=\widehat f^\ell(x^\ell,\theta^\ell)\).
The chart-error disk contributes the exact constant \(\rho_p\), since the
physical normal is always unit length.

The four rectangle vertices lie on a common radius-\(R\) circle. After their
yaw arcs are expanded, the remaining angular gaps have chord inequalities
\(Dy\le b\). Their convex hull is
\(B=\{y:\|y\|_2\le R,\ Dy\le b\}\). For nondegenerate rectangles,
strong duality gives

\[
h_B(v)=\min_{\lambda\ge0}\ b^\top\lambda+R\|v-D^\top\lambda\|_2.
\]

This is an exact support epigraph, including zero yaw width and a full-circle
hull. The implementation uses cheaper absolute-value epigraphs for zero yaw
width and skips point bodies. Quadratic upper bounds use ordinary SOCs via
\(\|(z-1,\sqrt2 L\Delta)\|_2\le z+1\), equivalent to
\(z\ge\|L\Delta\|_2^2/2\). All new couplings are local to a stage.

## Admission and its limits

The convex base \(\mathcal D\) includes dynamics, actuator and slew limits,
pose/reference domains, the soft CLF and the hard terminal cones. Fixed
collision and exit slices are removed. If the nominal is outside this base,
solve the base first; a failed base solve is reported separately.

Certificate residual bounds \(b_i\le0\) initially include inward numerical
reserves. Within one start, restoration solves

\[
\min_{q,s}\ s+\tfrac12\|q-q^\ell\|_M^2,
\quad q\in\mathcal D,\quad
\widehat f_i^\ell(q)\le b_i+s,\quad 0\le s\le s^\ell.
\]

Here \(s^\ell=\max(0,\max_i[f_i(q^\ell)-b_i])\). The zero increment is
feasible because of touching, and comparison with it gives monotone violation
and a proximal-step bound for exact solves. The implementation independently
checks the physical base constraints before adopting an intermediate center,
and re-evaluates the true nonlinear residual. Invalid or violation-increasing
proposals are discarded; insufficient decrease ends that start. It records a separate violation history
for each start. This is not a global monotonicity claim across restarts.

The square example demonstrates a real obstruction. With
\(r=(0.9,y)\), \(0\le y\le2\), \(K=[-1,1]^2\), \(d=0.1\), and
\((y^0,\theta^0)=(0,0)\), this majorant becomes

\[
\widehat f^0=0.2+|\Delta\theta|
 +\tfrac12(\sqrt2+R_r+\eta^{-1})(\Delta\theta)^2
 +\tfrac12\eta y^2.
\]

The monotone cap at its initial violation forces \(y=\Delta\theta=0\),
although the upward safe escape exists. A test reproduces this stalled,
feasible subproblem. The implementation permits at most five deterministic
angle initializations: the nominal directions, then common lateral positive,
lateral negative, forward and backward collision angles in the first chart.
Exit angles remain free decisions initialized from the nominal exit geometry.
Every start uses the same feasible base point and freely variable angles.
These are bounded local searches, not a complete enumeration of joint branches;
they do not establish global admission or finite-time convergence.

Settings live in `cfg.jointCertificate`: `maximumIterations`, `maximumStarts`,
`positionStep` (metres), `angleStep` (radians), `proximalWeight` and
`stallTolerance`. The solver search budget and full-frame deadline still apply.

## Hard acceptance and continuation

No restoration slack appears in an issued command or a stored accepted plan.
Acceptance reconstructs the trajectory and checks all base physical rows,
terminal/CLF cones and exact support residuals with roundoff allowances. Solver
status alone is insufficient. Once accepted, one hard joint SOCP improves the
verified point. If this solve fails, times out or produces an invalid point,
only the independently reverified incumbent may be returned. Metadata reports
`retainedCertifiedIncumbent` and preserves the actual solver failure status.
Exceeding the complete frame deadline still prohibits command issuance.

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
- `solveHardCbfClf`: base feasibility, bounded restoration, hard improvement,
  physical acceptance and the certified-incumbent interface.
- `formulateAvoidanceProblem` and `hardEncounterBarrier`: preserve and shift
  the entire certificate; collision and exit angles share the same method.
- `jointSupportCertificateTest`: homogeneous/interval support, 2,000 seeded
  majorant checks, the stalled square escape, unsafe base rejection, hard
  admission, uncertain multiple targets, suffix preservation and deadlines.

The implementation adds no core source file or third-party dependency. Its
twenty-file source budget remains in force. `certificateContinuationTest`
checks default joint admission, suffix containment, changed sensing contracts,
rejection of older saved states and target-free renewal.
