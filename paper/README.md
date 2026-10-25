# Draft manuscript

`manuscript.tex` is the IEEE Transactions-style draft titled
"Safety-Guaranteed Output-Feedback Collision Avoidance Using NRMM-Based Vehicle
State Estimation". The October 25, 2026 revision aligns the paper with the
unified observer-to-safety formulation in the controller theory. The title's
safety guarantee refers to the conditional mathematical design; the production
Fiala controller does not yet implement the complete certificate construction.

Compile from this folder:

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error manuscript.tex
```

The derived `manuscript.pdf` is the readable draft. Remove build
intermediates with `latexmk -c manuscript.tex`.

## Unified mathematical design

The observer supplies a current pointwise physical enclosure, including the
initial transient and applicable sensor, model and digital-realization bounds.
Candidate held inputs propagate that set. The next observation conditions the
retained first successor, preserving truth containment and the inclusion needed
by the shifted-plan proof. Raw observer balls need not be nested. Dropouts keep
the physical prediction; an empty intersection invalidates the contract.

A signed geometric margin distinguishes overlap from zero-distance contact.
Whole-hold reachable images, compatible hard constraints and one common
admissible terminal continuation establish conditional recursive feasibility.
Executing a primary minimizer gives optimal-relaxation descent and an invariant
safe zero-value set. Positive slack remains relaxation. An unbounded soft CLF
slack preserves feasibility but does not by itself prove recovery. Road
boundaries are omitted from this robust theorem.

The core is output feedback through measurement-conditioned reoptimization;
it does not forecast future observer convergence. A predicted policy that uses
future observations requires the optional joint observer/measurement lift.
Both constructions use the same inclusion and shift argument. Initial
feasibility, sound nonlinear images and terminal closure are explicit premises.

Tezuka and Nakamura (2020) and Quan et al. (2025) motivate error compensation
and containing-set interfaces. Their hypotheses are distinguished from this
vehicle construction; an energy-gain inequality alone is not treated as a
pointwise Euclidean error bound.

The detailed derivation is maintained in
[controller theory, Section 7](../controller/CONTROLLER_THEORY.md#recursive-feasibility),
with the optional joint extension in Section 6. The executable longitudinal
reference is [verifyUnifiedObserverSafety.m](../scripts/verifyUnifiedObserverSafety.m);
settings, test results and limitations are in
[the reference verification report](../report/UNIFIED_OBSERVER_SAFETY_20261025.md).

## Numerical evidence and implementation scope

The October 11 vehicle results are unchanged: fourteen exact-state encounters
recovered cruise; 74 of 84 noisy estimated-state runs recovered; none collided
or left the road. Those trials use per-hold affine tightening and center-based
terminal rows, with road constraints. They do not evaluate the complete
information-set algorithm or later runtime revisions.

The paper separately reports the October 25 longitudinal reference, including
moving observation boxes, dropouts, common braking, parked uncertainty and a
between-sample collision counterexample. Eleven reference behavior tests and
64 targeted MATLAB tests in total passed in the preceding algorithm task.
These are floating-point specializations, not full-vehicle certification.
No vehicle campaign was rerun for this manuscript revision.

The campaign-era discrete CLF formulas are labeled as historical. The current
runtime uses a continuous derivative CLF row, a primary LP and secondary QP,
and nonlinear sampled checks. It still lacks validated whole-hold images,
retained posterior sets and robust terminal witnesses. Neither solver success
nor sampled replay closes those gaps.

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
