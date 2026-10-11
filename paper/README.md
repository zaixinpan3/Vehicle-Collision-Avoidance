# Draft manuscript

`manuscript.tex` is the IEEE Transactions-style research draft, titled
"Simultaneous Lane Following and Vehicle Collision Avoidance System with
Safety Guarantees". The title names the goal of the work; Section II-A of the
manuscript states which links of the safety argument are proved at present and
which are open. The October 11, 2026 revision synchronizes the draft with the
implementation at commit `ea88059` (the controller of that date: the
braking-continuation terminal set, the step-size rule, the low-speed regime of
the vehicle model, the cruise law certified on the sampled nonlinear model,
and the faster assembly). The revision that followed on the same day, commit
`e207dde`, floors the weight of the CLF stage's tie term on the shifted plan
(cruise exact at 8 and 15 m/s on both paths; 14 of 14 and 73 of 84
encounters recovered, `report/SOLVER_TOLERANCE_20261011.tex`); the
manuscript's discussion mentions it, and its results predate it. The
control-design section, the closed-loop
protocol, outcomes, in-loop estimation, failure analysis, computation,
discussion, conclusion, abstract and the end of the introduction are
rewritten; the system overview, the safety-argument subsection and the
model-aided measurement subsection are revised where the design changed. The
literature discussion of the introduction, the perception section, the
estimator lemmas and theorem, and the estimator-alone results (Sections VI-A
and VI-B, measured October 1, 2026) are unchanged.

Compile from this folder with:

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error manuscript.tex
```

Clean generated LaTeX intermediates with:

```bash
latexmk -c manuscript.tex
```

## What the manuscript now describes

The estimator keeps direct body-velocity observation, the parallel yaw
observer, the covariant target tracker and the two-stage ISS comparison, with
target gains from the noise-variance semidefinite program inside the Lipschitz
certificate; in closed loop the lateral velocity is measured through the
lateral force balance of the controller's Fiala model and held input, and the
estimator publishes an ego zonotope and a target parameter set.

The controller is the two-stage PCBF/CLF real-time iteration of
`controller/PCBF_CLF_ARCHITECTURE.md` with the terminal set of
`controller/TERMINAL_SAFE_SET.md`: one nonlinear rollout per sample (the
shifted issued plan, extended by holds of the declared model that keep its
endpoint terminal, or a potential-field rollout at startup and at a target's
onset); collision rows with fixed distance-dual multipliers; road rows, a
rear-adhesion cone and the estimator's sideslip cone; a variable horizon that
ends in the braking-continuation terminal set (mild endpoints; past the
endpoint the obstacle keeps its forecast, which brakes to rest, and the ego
brakes to rest with its steering held; the joint continuation is closed-form
and must stay on the road and clear the obstacle until it leaves the range,
after the stop over the obstacle's remaining track); one quadratic CLF whose
matrix and cruise gain are certified offline on the sampled nonlinear model by
an LMI over a sampled enclosure of ray-averaged Jacobians; per-hold tightening
by the estimator's current enclosure; the issued plan as the nonlinear rollout
of the largest acceptable step of the convex solution; and an input trust
scale estimated from the full step's remainder. The section states one lemma
(fixed-multiplier rows), one assumption (mild states follow the kinematic
vehicle), two propositions (forward invariance of the terminal set; recursive
feasibility on the declared model with exact information) and an explicit
scope.

The closed-loop results are the campaign of
`report/CONTROLLER_SPEED_20261011.tex`, rerun on the same code: fourteen
encounters with exact states (14 of 14 recovered, the shifted plan feasible at
every sample) and the same encounters with the estimator and bounded sensor
noise for six seeds (74 of 84 recovered, ten without a solved command, no
collision, no road departure), an in-loop estimation audit, the analysis of
the ten failures, and computation times. The plant is the prediction model
with nominal parameters; perception is not in the loop.

## Figures

`figures/` is not tracked. This revision adds `closed_loop_results_v03.pdf`,
drawn from the recorded campaign traces by
`../report/MANUSCRIPT_ALGORITHM_SYNC_20261011/figure-sources/closed_loop_results_v03.py`;
it runs no simulation. The architecture figure is still
`system_architecture_v11.pdf`
(`../report/MANUSCRIPT_ALGORITHM_SYNC_20261002/figure-sources/`); its caption,
not the drawing, carries the changed information flow. The curb-feature
figure of Section III-B (`curb_ring_features_v01.pdf`) was produced by the
offline curb-detection experiment that was removed from the repository on
October 6, 2026; it can no longer be regenerated from the repository.

## Record of this revision

Sources, derived statistics and open review items are recorded in
[`MANUSCRIPT_ALGORITHM_SYNC_20261011.tex`](../report/MANUSCRIPT_ALGORITHM_SYNC_20261011.tex).
The revision ran no controller simulation and no unit test; the campaign it
quotes is a rerun of the committed campaign on the same code, with identical
outcomes in all 98 runs, and the estimator audit replays the estimator on
those recorded traces. Every number is taken from committed reports or derived
from the recorded traces by
`../report/MANUSCRIPT_ALGORITHM_SYNC_20261011/deriveClosedLoopStatistics.py`.
The preceding revisions are recorded in
[`MANUSCRIPT_ALGORITHM_SYNC_20261005.tex`](../report/MANUSCRIPT_ALGORITHM_SYNC_20261005.tex),
[`MANUSCRIPT_ALGORITHM_SYNC_20261002.tex`](../report/MANUSCRIPT_ALGORITHM_SYNC_20261002.tex)
and
[`MANUSCRIPT_ALGORITHM_SYNC_20260919.md`](../report/MANUSCRIPT_ALGORITHM_SYNC_20260919.md).

Before submission, complete the author, affiliation, funding, competing-interest,
contribution and repository-release declarations, and review the revised
derivations and experimental scope.
