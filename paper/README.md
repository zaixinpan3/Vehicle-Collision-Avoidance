# Draft manuscript

`manuscript.tex` is the IEEE Transactions-style draft titled
"Integrated Lane-Keeping and Cruise Control System With Vehicle Collision Avoidance". A lane-keeping and cruise control system
maintains travel within a designated lane at a prescribed speed. The
October 14 terminology revision presents this as a single nominal system
throughout the manuscript. The October 13, 2026
revision synchronizes the joint
observer--controller mathematical design with the current theory documents.
The abstract, introduction, safety argument, robust derivation, discussion
and conclusion use the same guarantee scope.

Compile from this folder:

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error manuscript.tex
```

The derived `manuscript.pdf` is the readable draft. Remove build
intermediates with `latexmk -c manuscript.tex`.

## Joint mathematical design

The robust subsection defines a nonlinear joint state containing the
physical ego and target, uncertain parameters, observer state and bounds,
measurements, and control memory. Future measurements and causal feedback
are propagated with the plant; their model-mismatch sources are shared.
The published current error enclosure initializes the physical set rather
than supplying an extra independent disturbance at every predicted step.

Exact reachable images, or outer images with proved nonlinear reserves,
contain all admitted trajectories. The next posterior intersects the old
first successor with new sound information. A shifted causal policy
retains the old tube and row certificates and appends a common admissible
zero-deficit terminal braking rule. The terminal family contains mild
moving states and their continuation to a parked pose set with nonzero
uncertainty. A soft CLF stage preserves mathematical feasibility.

These constructions complete the conditional recursive-feasibility proof
for the defined mathematical design. Initial feasibility and compatible
model, sensor, observer and terminal contracts are explicit premises.
The derivation temporarily omits road boundaries and proves collision
rows at the stated constraint instants. It requires neither a fixed tube
width nor numerical solver success. Optimal-relaxation descent additionally
requires applying an attained primary minimum; positive collision slack
remains relaxation.

The detailed derivations are maintained in
[`CLOSED_LOOP_PREDICTION_THEORY.md`](../controller/CLOSED_LOOP_PREDICTION_THEORY.md),
[`ROBUST_RECURSIVE_FEASIBILITY.md`](../controller/ROBUST_RECURSIVE_FEASIBILITY.md)
and [`TERMINAL_SAFE_SET.md`](../controller/TERMINAL_SAFE_SET.md).
Exact-arithmetic specializations are checked by
`../scripts/checkJointMismatchDerivations.py` and
`../tests/jointMismatchDerivationsTest.py`.

## Numerical evidence

The reported October 11 campaign is unchanged: fourteen exact-state
encounters recovered cruise, and 74 of 84 noisy estimated-state runs
recovered; none collided or left the road. The reported controller uses
per-hold affine tightening and center-based terminal rows and retains road
constraints. Those trials do not evaluate the full joint future-feedback
mathematical problem or the subsequent direct-execution revision. No new
vehicle simulation was run for this manuscript synchronization.

## Figures and submission fields

`figures/` is not tracked. Compilation uses
`driving_environment_curved_v08.pdf`, `ego_obstacle_dynamics_v02.pdf`,
`system_architecture_v11.pdf`, `yolo_lidar_projection_v01.png`,
`obstacle_rectangle_fit_topdown_v01.png` and `closed_loop_results_v03.pdf`.
The generated manuscript PDF and these figure dependencies are excluded
from the source commit.

Before submission, complete the author, affiliation, funding,
competing-interest, contribution and repository-release declarations.
Authors should review the proofs, references and AI-assistance disclosure.
