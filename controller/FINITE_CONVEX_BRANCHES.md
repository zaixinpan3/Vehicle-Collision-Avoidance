# Finite convex branches for predictive collision avoidance

In format 30 this finite family initializes encounters when the continuous
distance-dual proposal is unavailable or its hard trajectory problem is
infeasible. It is no longer constructed for every fresh admission. Later
distance-dual updates preserve the known feasible suffix; see
[DISTANCE_DUAL_CONVEXIFICATION.md](DISTANCE_DUAL_CONVEXIFICATION.md).

## Modeling decision

The user authorized a conservative polyhedral approximation on September 16,
2026. Admission searches its finite convex branches rather than prescribing a
maneuver, a lateral displacement, or a time at which lateral separation must
be achieved. There is no left/right mode selector or cosine lateral template.
Inputs remain free optimization variables under the declared dynamics.

This is a finite union for the **declared approximation**, not an exact finite
convex cover of rotating rectangles at positive Euclidean clearance. Even at
fixed headings, the configuration obstacle is a polygon Minkowski-summed with
a clearance disk. Its rounded corners preclude a general finite convex cover
of the complete safe exterior: two different points on the same circular
boundary arc have a chord midpoint inside the forbidden clearance region.
A safe convex subset cannot contain both points. Infinitely many such boundary
points therefore require infinitely many convex subsets. This geometric
argument does not assert that every particular dynamics-restricted planning
instance has infinitely many necessary branches.

For the two 4.8 by 1.9 m aligned bodies, the uninflated configuration obstacle
has half-extents (4.8, 1.9) m. Two points at clearance 0.25 m and corner angles
5 and 85 degrees have midpoint clearance 0.1915111108 m. The authorized
polyhedral replacement avoids claiming an impossible exact finite cover.

## Finite directional certificate

Choose K uniformly spaced unit directions, with K a multiple of four; the
default is eight. Each cell is now one complete held-input period; there are no temporal
subcells. All K directions are available in every hold for every
target. They do not encode a chosen trajectory or a chosen maneuver side.
Let n_q denote direction q. For a fixed pair of poses, define

\[
 H_q=\sigma_E(n_q)+\sigma_T(-n_q)+d_{\min}.
\]

The intersection n_q^T(p_E-p_T) <= H_q contains the exact forbidden
configuration obstacle. Its safe exterior is a union of K separating
half-spaces. The approximation is conservative near rounded corners and
when directions are coarse. In the implemented uncertain, rotating case,
the same construction uses the existing sound target supports and convex
piecewise-affine majorants of ego footprint support over the computed
actuator-reachable heading interval. A full-rotation interval uses a constant
circumradius bound. Neither choice imposes a heading-domain constraint.
Each majorant piece is enforced. Thus each branch is still convex in the
input sequence, and acceptance implies the original required body clearance.

For cell c and target j, the branch q contains all original swept Bernstein
rows for that direction:

\[
 \mathcal C_{cjq}=\{U:A_{cjq}U\le b_{cjq}\}.
\]

Every point, support-majorant piece, uncertainty allowance and Taylor
remainder belonging to that cell uses the same chosen direction. Safety
continues to cover the entire hold, not only sampled endpoints. Requiring
one common direction over a cell is an additional conservative restriction;
finite direction enumeration does not remove it.

Finite target exit also has K directional alternatives, including the entire
target footprint and the declared sensing range. No guessed exit direction
is mandatory. Let E_jq be these affine final-state branches. With permanent
convex constraints H (dynamics, input/slew, optional road, terminal SOCs and
soft CLF), the finite planning set is

\[
 \mathcal F_K=\mathcal H\cap
 \bigcap_{c,j}\left(\bigcup_{q=1}^{K}\mathcal C_{cjq}\right)
 \cap\bigcap_j\left(\bigcup_{q=1}^{K}\mathcal E_{jq}\right)
 =\bigcup_{\sigma}\mathcal F_{\sigma}.
\]

Each complete assignment sigma defines a convex SOCP. With C cells and J
targets, the raw assignment count is K^{J(C+1)}; empty branches and duplicate
assignments need not be enumerated literally. At the 32-hold oncoming horizon, C=32, J=1 and K=8, giving 8^33
raw assignments. This replaces the former seven-subcell-per-hold family.
These are geometric assignments through time, not two global maneuver modes.

The fixed affine plant and support bounds remain conservative modeling
premises. Charts now cover the actuator-reachable station range when forming
new encounter geometry; they do not enforce a station corridor around a
cruise rollout. A numerical input center is used to translate solver
coordinates and construct valid support majorants, not as a tracking path or
an equality constraint on the optimized trajectory.

## Search and acceptance

1. Solve the common convex relaxation. If its solution satisfies a branch
   in every group, attach those complete rows and verify the resulting witness.
2. Otherwise, construct binary direction choices for violated groups. For a
   branch row a^T U <= b, use a^T U + M z <= b+M, with one selected direction
   per active group. M comes from explicit actuator-box support with an outward
   arithmetic allowance; no guessed large constant is used.
3. Search assignments using MATLAB `intlinprog` branch-and-bound. Initially
   omit unneeded disjunction groups while retaining all common physical rows.
   Check every candidate against the complete original family and add violated
   groups. These are outer relaxations used only for search, never
   executable safety certificates.
   Rank integer search candidates by increasing terminal path station when
   a physical prediction map is available. This affine objective biases the
   initializer toward the requested forward cruise task; it changes no
   feasible constraint, no pruning certificate and no trajectory SOCP cost.
4. Use supporting half-spaces as outer relaxations of the terminal SOCs in
   the integer master. Solve the complete original SOCP for a candidate
   assignment, retaining the actual terminal cones, the original performance
   objective and the squared CLF slack penalty.
5. A reported infeasible complete SOCP assignment may be excluded by a
   no-good cut. Before doing so, activate all groups: excluding a partial
   assignment could otherwise discard feasible combinations of directions
   that had not yet been examined. Add valid terminal supporting cuts when
   available. A timeout or numerical failure cannot justify this exclusion.
6. Accept the first complete feasible branch only after the existing physical
   hard-row, terminal and CLF certificate checks. No global performance
   optimum over all branches is claimed. Failure or search exhaustion issues
   no command and never executes a backup input.

At a fixed horizon, an infeasible outer master excludes every complete branch
it relaxes, subject to the numerical solvers' feasibility judgments. A timeout
means **search incomplete**, not that the finite family or physical problem
is infeasible. For a fresh encounter, reported infeasibility may trigger the
next finite horizon allocation up to four times the configured performance
horizon. The entire search shares the frame budget. Horizon choices are
finite certificate lengths, not prescribed vehicle displacement schedules.

The integer master produces only candidate assignments. An incumbent returned
when its integer search stops is never executed directly; the complete SOCP
and independent certificate remain necessary. Numerical infeasibility reports
are not presented as exact-arithmetic proofs.

## Recursive feasibility and unchanged framework

Once a complete branch is certified, store its actual affine family,
selected cell/exit directions, exact generators, terminal cones, uncertainty
enclosures and absolute confirmation deadline. At the successor, substitute
the executed control and retain the suffix of this **same branch**. Its
feasibility follows from the existing inclusion and terminal argument.
There is no requirement to re-enumerate branches at every active-encounter
sample. Confirmed removal deletes only discharged obligations.

Accordingly, the PCBF predictive continuation, target-independent invariant
terminal certificate, soft CLF and high-gain target observer remain in place.
The change concerns finding an initial member of the safe predictive domain.
It does not establish admission for all physically avoidable encounters,
nonlinear plant inclusion, or guaranteed 100 ms computation. A fixed K, fixed
hold partition and bounded set of horizons can still reject a safe physical
trajectory. Search times and full-frame deadlines must be reported separately
from safety and final cruise recovery.

## Sources and attribution

The finite-family construction above is the repository-specific derivation.
The continuous-hold Bernstein certificate and successor proof are retained
from the existing implementation. Relevant external foundations are:

- Deits and Tedrake, *Computing Large Convex Regions of Obstacle-Free Space
  through Semidefinite Programming*: convex regions can cover part of free
  space without constituting a complete cover. [Author-hosted paper](https://groups.csail.mit.edu/robotics-center/public_papers/Deits14.pdf).
- Marcucci, Petersen, von Wrangel and Tedrake, *Motion Planning around
  Obstacles with Convex Optimization* (2022): mixed-integer selection among
  convex regions combined with continuous-trajectory certificates. Its GCS
  algorithm is background, not an implementation claim for this controller.
  [Primary paper](https://arxiv.org/abs/2205.04422).
- MathWorks, [`intlinprog` documentation](https://www.mathworks.com/help/optim/ug/intlinprog.html):
  integer feasibility status, search time/node budgets and incumbent handling.
