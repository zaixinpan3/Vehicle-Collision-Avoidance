# Overlap-aware support convexification

> Certificate sampling note (2026-09-17): the online controller now certifies the safety rows at the hold nodes of the exact sampled affine plant only; statements below about whole-hold, swept or Bernstein coverage hold at the nodes and no longer claim inter-node coverage. See [NODE_SAMPLED_CERTIFICATE.md](NODE_SAMPLED_CERTIFICATE.md).

Controller format 33, September 17, 2026.

## Geometric contract

For a unit direction `n`, a configuration obstacle `C`, and required clearance
`m`, `n'*p >= support(C,n)+m` is a sufficient separation inequality. Its
construction does not require the numerical anchor to satisfy it. The analytic
rectangle-polygon query returns a signed clearance, a unit support direction,
and tied face alternatives separately. Overlap and contact are admissible
search inputs; neither is a safe executable configuration.

The finite search ranks joint geometric sectors from the current relative
positions, retaining opposite alternatives. Within each sector, midpoint,
tied-face, previous-cell, road-axis and retained directions are candidates.
Restoration updates score the minimum conservative reserved clearance across
all Bernstein/support rows of a whole hold. There is no prescribed vehicle
trajectory, lateral amplitude, acceleration target or passing time. These
sectors and candidate directions are an incomplete inner search, not an exact
finite partition of the original nonconvex safe set.

## Certified circular pose maps

For a constant-curvature reference and a local anchor $(s_0,d_0)$, define
$\Delta s=s-s_0$, $\Delta d=d-d_0$. The Cartesian position approximation is

$$
p_{\rm aff}=c(s_0)+d_0n(s_0)+(1-\kappa d_0)t(s_0)\Delta s+n(s_0)\Delta d.
$$

On $|\Delta s|\le r_s$, $|\Delta d|\le r_d$, write $D=|d_0|+r_d$.
The Euclidean Taylor remainder is bounded by

$$
E_p=\tfrac12|\kappa|(1+|\kappa|D)r_s^2+|\kappa|r_s r_d,
$$

plus an arithmetic reserve. A unit collision/exit direction is charged $E_p$.
The affine heading $\theta(s_0)+\kappa(s-s_0)+e_\psi$ is exact for a circle.
Rectangle support majorants use that same affine heading and its domain;
station and heading are not treated as independent orientation errors.
Geometric unit tangent/normal vectors remain separate from the position
Jacobian. Direction proposals and nominal distances use exact circular poses.

Every Bernstein coefficient, including its propagated radius, must satisfy
six hard inequalities for the local box in $(s,d,e_\psi)$. Convex hull
containment then establishes domain validity over the entire unchanged hold.
These inequalities have no restoration slack. Internal working sets may omit
rows temporarily, but no completed solve or executable certificate can omit
their full check. Terminal exterior membership uses the final cell's map and
six additional hard endpoint-domain rows. Current observation release must
also lie inside the chosen map's domain.

Fresh admission centers each domain on the numerical anchor's enclosed hold,
with configurable extra radii `controller.poseTrustRadius = [2;4;0.5]` in m,
m and rad. Additional bounded families expand those radii. Reanchoring updates
collision geometry, hard domains and terminal exit rows together; it does not
prescribe a trajectory. An inherited family retains its old maps/domains and
absolute completion deadline. Optional normal replacement must preserve the
complete carried witness, including the domain and terminal rows.

This extension covers analytic constant-curvature encounters without physical
road boundaries. It does not add variable-curvature models, correlated-set
conditioning or a new road-constrained terminal set.

## Admission and restoration

Clear seeds attempt the hard problem first. Colliding seeds start with
restoration. Collision deficits are shared by each hold-target pair; exit and
terminal-entry rows and each terminal cone receive search-only deficits.
Physical input/slew limits, affine dynamics and local pose-domain rows remain
hard. The sum of squared normalized deficits is minimized without a competing
cruise or input objective. This makes zero the same hard-feasibility target as
the former linear deficit objective and improves numerical behavior in curved
searches; positive values still have no execution authority. The optional
lexicographic proximity objective is not needed by this implementation.

After restoration, controls update the support geometry. A new hard problem
restores the original cruise/trim/CLF objective and removes every search
deficit. Stagnation changes the joint sector, fresh exit direction or finite
horizon under one shared budget. No relaxed search iterate is an executable
witness. A native success flag is insufficient: the complete original physical
rows, physical terminal cones and reserved soft-CLF cone must pass independent
verification. Failure and timeout issue no command.

## Equivalent sparse solution

Auxiliary stage deviations satisfy the fixed held-model transitions. Collision
rows use each hold's starting state and input; the terminal modal cones use the
last state. Eliminating auxiliaries recovers the condensed program within its
numerical model tolerances. The future-state quadratic cost is block diagonal
in these coordinates; input cost remains relative to the CLF operating point.
Exact duplicate left sides retain only their tightest bound.

For larger geometry programs, numerical constraint generation starts with the
two most violated rows of each hold, separately grouping collision, local-domain
and phase rows, and retains all nongeometric hard rows and cones.
Every returned decision is checked against all omitted inequalities, and up to
sixteen violated rows per group join the working set before the next native
solve. These counts only trade native solves against program size; the
converged decision satisfies the same complete row set. The loop cannot accept
an unfinished working set.
The independent original-program verifier remains unchanged. Restoration uses
the same transcription and checks its omitted deficit-adjusted inequalities.

## Continuation and limits

Admitted encounters retain their original generators, swept enclosures,
terminal cones and absolute exit deadline. Optional replacement geometry must
contain the complete known suffix witness. Confirmed target removal only
removes that target's obligations. The terminal law remains a mathematical
continuation, never an executable fallback.

For target-free initialization, a shorter horizon within configured limits
can retain a verified terminal entry when a longer open-loop uncertainty tube
has a negative terminal-cone radius. Enlarged measurement bounds trigger a new
complete admission and new terminal sensing contract. They do not inherit
recursive feasibility under the old bound; the old prediction only conditions
the actual successor. Empty intersections still indicate inconsistent data.

The high-gain target observer, predictive CBF, soft squared-slack CLF and
zero-residual held affine model are retained. Safety after successful admission
is conditional on the documented motion, observation and execution contracts.
Neither global admission completeness, nonlinear physical-vehicle safety,
asymptotic convergence with persistent CLF slack, nor a worst-case 100 ms
execution bound follows from this search algorithm.

Native support and geometry kernels are built by
`scripts/buildAvoidanceGeometryKernel.m`; generated artifacts stay in the
excluded `solver/` tree. The old ordinary-distance query and obsolete
initialization diagnostic driver have been removed.
