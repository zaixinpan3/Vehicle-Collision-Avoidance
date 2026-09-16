# Distance-dual convexification with a retained safety witness

Current implementation: controller format 31, September 16, 2026.

## Source and scope

Li, G., Zhang, X., Guo, H., Lenzo, B., and Guo, N. (2023), *Real-Time Optimal
Trajectory Planning for Autonomous Driving with Collision Avoidance Using
Convex Optimization*, Automotive Innovation 6, 481--491,
[DOI: 10.1007/s42154-023-00222-7](https://doi.org/10.1007/s42154-023-00222-7).
Section 3, equations (12)--(13), first optimize distance-dual variables and
then fix them in a convex trajectory problem. Equation (13) includes a
nonnegative collision slack and a quadratic penalty. Its dynamics are
successively linearized and its trajectory subproblem is a QP.

We adopt the two-stage distance-dual mechanism. Our adaptation retains full
ego/target rectangles, bounded target prediction, continuous whole-hold
checking, hard collision constraints, a soft CLF, the same affine plant and
terminal continuation. The trajectory subproblem remains an SOCP. No
collision slack, state-box constraint or prescribed maneuver is introduced.
The target observer is unchanged. This is not a reproduction of the paper's
entire controller or its experimental performance.

## Stage one: a continuous geometric direction

At a fixed anchor pose, let the full configuration obstacle of the two
rectangles be

\[
\mathcal O=\{z:Az\le b\},\qquad
b_i=A_ip_T+\sigma_T(A_i)+\sigma_E(A_i).
\]

Rows of A are the four signed body-axis normals of each rectangle. Their
eight-halfspace intersection is the exact Minkowski configuration polygon
at these fixed orientations; coincident rows are harmless. The ego center
p lies outside this polygon precisely when the two fixed rectangles are
separated. Solve

\[
\max_{\lambda\ge0}\;(Ap-b)^\top\lambda,
\qquad \|A^\top\lambda\|_2\le1. \tag{1}
\]

For positive distance, define the proposed unit normal
\(n=A^\top\lambda^*/\|A^\top\lambda^*\|_2\).
The implementation solves an eight-variable SOCP with the existing native
solver. It compares against independent exact rectangle geometry in tests.
Directions are continuous; no finite direction grid is constructed.

For an interior point, the ordinary distance is zero and lambda=0 is optimal.
There is no usable positive-separation direction from that solution. The
implementation identifies overlap geometrically and returns an unavailable
proposal. It does not normalize numerical noise into a maneuver direction.
The same treatment applies to a boundary point. The optimizer's finite
precision output is only a proposal, never the safety certificate.

An anchor is a computed control sequence or the cruise operating input,
used to evaluate geometry. It creates no path tracking constraint. Each hold
uses its exact Bernstein midpoint evaluation for the anchor query. This
midpoint is not a new execution subdivision or the sole collision check.

## Stage two: complete hard continuous safety

With n fixed, rebuild the existing robust support rows. Schematically,

\[
n^\top(p_E-p_T)\ge
\widehat\sigma_E(n,e_\psi)+\overline\sigma_T(n)
+d_{\min}+\text{position, chart and flow allowances}. \tag{2}
\]

The ego support majorant is affine or piecewise affine over the complete
reachable heading interval; target support covers its complete yaw interval.
Enforce every Bernstein coefficient inequality of the full held interval.
The resulting rows are affine in the input sequence, even though both
rectangles can rotate along the plan. Recomputing their supports is essential:
the dual from one fixed pose alone cannot certify varying orientations.

Actuator, slew, terminal and finite-exit conditions remain hard. Only the
CLF uses a slack, with its existing squared penalty. The exit direction and
absolute encounter-completion deadline are not changed by this update.
The final independent hard-certificate check is unchanged.

## First admission and unavailable initialization

Distance-dual geometry is the sole online collision convexification. At fresh
admission the controller computes the cruise-input anchor, its whole-hold
frames and distance-dual normals before constructing collision rows. Every
midpoint must supply a valid normal. Overlap or contact yields zero ordinary
distance and no usable direction; the controller reports initialization failure
before invoking the hard trajectory solver. A numerical distance-solver failure
also supplies no direction. The error reports the number of overlapping/touching
midpoints and the minimum anchor signed distance; it does not claim physical
infeasibility or infeasibility of a trajectory problem that was never solved.

When all normals are available, solve the complete hard trajectory SOCP and
independently verify it. A proven infeasible trajectory program may trigger the
existing finite horizon extension, using the same distance-dual method. Solver
errors, iteration limits and expired deadlines terminate the call. There is no
integer search, finite directional initializer, penetration-normal substitute,
collision slack or alternate controller. The configured prediction horizon is
not a maneuver template and the CLF/input objective is unchanged.

This removal intentionally narrows first-admission capability. The previous
stationary/oncoming successes used the removed initializer and are not evidence
of successful admission by this implementation. An overlap-capable initialization
within the selected method remains unresolved. Failed initialization emits no
command and terminates the scenario, as required by the execution contract.

The obsolete execution-policy selector and finite direction count are removed,
not accepted as aliases. Stored controller format 31 rejects earlier states so
that a previously initialized constraint family cannot enter through state reuse.

## Recursive-feasibility preservation

Let the shifted accepted program be \(\mathcal P_k\), with a known feasible
suffix \(\bar U_k\), and construct a proposed dual-directed program
\(\mathcal P_k^{\mathrm{dual}}\). Keep the certified stage generators,
reachable enclosures, charts, actuator/road conditions, terminal cones and
absolute exit deadline. Change only collision inequalities.

Complete the known suffix with a sufficiently large CLF slack. Before the
trajectory solve, require

\[
(\bar U_k,\bar\delta_k)\in\mathcal P_k^{\mathrm{dual}}, \tag{3}
\]

checking every linear row and every CLF/terminal cone with the numerical
reserve. If (3) holds, the new program is nonempty and contains a valid
continuation. If it fails, optimize the unchanged inherited program. No
command is selected or executed by this formulation choice.

Any returned plan still passes the complete independent hard check before
it replaces the stored witness. Inductively, each frame therefore starts
from the previous accepted continuous certificate and either keeps that
feasible family or replaces it with a verified family containing its known
suffix. This extends the existing conditional recursive-feasibility proof
without assuming that relinearization or new normals automatically preserve
feasibility. Physical model, sensing, execution and new-target-admission
premises remain those in [TERMINAL_CBF_PROOF.md](TERMINAL_CBF_PROOF.md).

## Runtime interpretation

Distance SOCPs and the trajectory SOCP are separate work. Metadata counts
both. No integer solver is called. A full-frame 100 ms claim
requires measuring their sum with formulation and verification. A faster
distance calculation or one successful frozen-frame solve does not establish
that bound. Experimental results belong under `report/`.
