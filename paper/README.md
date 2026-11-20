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

## Unified mathematical design

The observer supplies a valid current enclosure, including the initial
transient and applicable sensor and model uncertainty. Prediction carries
both physical vehicles, both observers, held sensor records, persistent tire
parameters and observable histories under causal control policies. Future
target observations can therefore change future ego inputs. Each physical
parameter and sensor realization is shared by all affected equations.

Exact sensor-period flows and resets define the nonlinear successor. At the
next decision, conditioning on the entire received sensor block retains a
subset of the old first successor. Raw observer balls need not be nested.
The predictor-innovation decay identity does not claim contraction of the
full estimation error. Neither constant tube width nor a uniformly small
tube is assumed.

The theoretical encounter controller minimizes PCBF slack and executes the
first common command of an attained primary minimizer. Restricting the old
tail to surviving histories and appending a compatible terminal rule proves
recursive feasibility and nonincreasing optimal slack sum. Zero remains zero
and excludes interior collision at decision instants. Contact is allowed at
equality; positive slack is relaxation, and finite-time arrival at zero is
not proved. Road boundaries and intersample collision are outside this theorem.

Initial feasibility, truth containment, well-posed joint flows, primary
attainment, policy/certificate closure and a nonempty downward-closed terminal
family remain explicit premises. The terminal family must admit a common
causal zero-slack continuation under the same model and uncertainty contract.
Its existence is assumed; the simplified braking candidate is an
implementation choice, not a terminal construction supplied by the theorem.

No performance optimization is required by the theoretical control law.
A feasible secondary refinement under the exact attained primary cap is
still a primary minimizer. An affine RTI cap alone does not establish this
nonlinear property. An unbounded soft CLF slack preserves feasibility for
finite residuals but does not prove recovery.

The detailed compact nonlinear controller derivation is maintained in
[controller theory](../controller/CONTROLLER_THEORY.md).
Section II-B of `manuscript.tex` now presents the adopted
compact nonlinear controller (fixed measurement-consistency ensemble
inference, condensed nonlinear PCBF SQP over coarse control knots, exact
dual rectangle separation at control-boundary nodes, moving straight-cruise
terminal set with $B$-hold bridge under an explicit physical handoff
assumption, and zero-deficit tire-loading/rear-slip preference $J_{\rm tire}$
with damped Gauss--Newton refinement) alongside the idealized joint
output-feedback safety theory.

Tezuka and Nakamura (2020) and Quan et al. (2025) motivate error compensation
and containing-set interfaces. Their hypotheses are distinguished from this
vehicle construction; an energy-gain inequality alone is not treated as a
pointwise Euclidean error bound.

The longitudinal reference
[verifyUnifiedObserverSafety.m](../scripts/verifyUnifiedObserverSafety.m)
checks selected containment and shift mechanisms of the simpler held-input
specialization. It does not implement the full joint observer/policy
construction.

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
