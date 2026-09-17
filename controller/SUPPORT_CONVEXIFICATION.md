# Overlap-aware support convexification

Controller format 32, September 16, 2026.

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

## Admission and restoration

Clear seeds attempt the hard problem first. Colliding seeds start with
restoration. Collision deficits are shared by each hold-target pair; exit and
terminal-entry rows and each terminal cone receive search-only deficits.
Physical input/slew limits and affine dynamics remain hard. Positive normalized
deficits are minimized without a competing cruise objective. The optional
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
most violated rows of each hold and retains all permanent hard rows and cones.
Every returned decision is checked against all omitted inequalities, and
violated rows are added. The loop cannot accept an unfinished working set.
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
