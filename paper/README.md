# Draft manuscript

`manuscript.tex` is the IEEE Transactions-style draft titled
"Safety-Guaranteed Output-Feedback Collision Avoidance Using NRMM-Based Vehicle
State Estimation". The current revision integrates the exact nonlinear
joint observer--feedback construction and its conditional shift proof. The title's
safety guarantee refers to the conditional mathematical design; the production
Fiala controller does not yet implement the complete certificate construction.

Compile from this folder:

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error manuscript.tex
```

The derived `manuscript.pdf` is the readable draft. Remove build
intermediates with `latexmk -c manuscript.tex`.

## Algorithm specification

The current continuous-time derivation for the observer and controller is
maintained in [ALGORITHM.md](../ALGORITHM.md). The manuscript records the
formulation and experiments from its own revision.

## Numerical evidence and implementation scope

The October 11 vehicle results are preserved as historical baseline
evidence and remain pending rerun for the revised compact nonlinear
controller: fourteen exact-state encounters recovered cruise; 74 of 84
noisy estimated-state runs recovered; none collided or left the road.
Those historical trials used the superseded affine-RTI/CLF/braking-terminal
formulation with per-hold affine tightening and center-based terminal rows.
They do not evaluate the adopted compact nonlinear controller or the
complete information-set algorithm.

The paper separately reports the October 25 longitudinal reference, including
moving observation boxes, dropouts, common braking, parked uncertainty and a
between-sample collision counterexample. Eleven reference behavior tests and
64 targeted MATLAB tests in total passed in the preceding algorithm task.
These are floating-point specializations, not full-vehicle certification.
No vehicle campaign was rerun for this manuscript revision.

## Figures and submission fields

The generated figure outputs are not tracked. Compilation uses
`driving_environment_junction_v20.pdf`, `ego_obstacle_dynamics_v04.pdf`,
`system_architecture_v12.pdf`, and `closed_loop_results_v03.pdf`.
The generated manuscript PDF and these figure dependencies are excluded
from the source commit.

The current system architecture source is
`figures/system_architecture_v12.tex`. Compile it with `pdflatex` in the
`figures/` directory before building the manuscript when its PDF is absent.
The driving-environment figure is generated in the same way from
`figures/driving_environment_junction_v20.tex`. Both vehicle figures share
`figures/vehicle_top_view_v01.tex`, which draws the contoured sedan body,
glazing, mirrors and four tires with independently steered front wheels.
The kinematics figure source is `figures/ego_obstacle_dynamics_v04.tex`.

Before submission, complete the author, affiliation, funding,
competing-interest, contribution and repository-release declarations.
Authors should review the proofs, references and AI-assistance disclosure.
