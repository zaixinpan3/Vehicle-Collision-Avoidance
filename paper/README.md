# Draft manuscript

`manuscript.tex` is the IEEE Transactions-style research draft, titled
"Simultaneous Lane Following and Vehicle Collision Avoidance System with
Safety Guarantees". The October 11, 2026 rewrite (the second manuscript
revision of that day) restructures the paper around one narrative: lane
keeping and collision avoidance are one system; existing designs assume
known states and offer no assurance that a safe action exists a moment
later; Sharma et al.'s NRMM observer and Huang et al.'s predictive control
barrier function each supply part of what is needed and each stops short;
the estimator and the controller here are designed for each other; and the
link that was missing, the estimator's certificate inside the controller's
guarantee, is supplied by a new theorem (Section IV-J, Theorem 2). The prose
of every section was rewritten in that voice. The equations, lemmas,
propositions, parameters and every number of the results are those of the
previous revision (synchronized with commit `ea88059`, campaign of
`report/CONTROLLER_SPEED_20261011.tex` rerun on the same code); no
simulation, test or estimator run was made for the rewrite.

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
observer, the covariant target tracker and the two-stage ISS comparison,
with target gains from the noise-variance semidefinite program inside the
Lipschitz certificate; in closed loop the lateral velocity is measured
through the lateral force balance of the controller's Fiala model and held
input, and the estimator publishes an ego zonotope and a target parameter
set. The enclosure corollary now also states the uniform bound of the
comparison solution, which the new theorem uses.

The controller is the two-stage PCBF/CLF real-time iteration of
`controller/PCBF_CLF_ARCHITECTURE.md` with the terminal set of
`controller/TERMINAL_SAFE_SET.md`, unchanged from the previous revision:
one nonlinear rollout per sample, fixed-multiplier collision rows, road
rows, the rear-adhesion and sideslip cones, the braking-continuation
terminal set with its invariance proposition and the mild-state assumption,
one quadratic CLF certified for the cruise law on the sampled nonlinear
model, per-hold tightening by the current enclosure, the two cone programs,
the step-size rule, and recursive feasibility on the declared model with
exact information (Proposition 2).

New in this revision is Section IV-J, "Guarantees under the estimator's
certified uncertainty and revised forecasts": the obstacle's information set
(the published sets intersected along the flow of the constant-parameter
model), Lemma 6 (the forecast families consistent with it are nested and
contain the truth), the innovation set fixed by the ISS certificate's uniform
bound, a tube hypothesis (T1)-(T2) in the manner of Chisci et al. and Mayne
et al., the robust problem with soft obstacle rows against the whole family,
and Theorem 2: feasibility is kept, the minimal relaxation is nonincreasing
by the applied hold's deficit, its zero set is invariant, and the applied
hold separates the true vehicles. A paragraph states what the implementation
realizes ((T1) only; forecast centers; no intersection), and quantifies the
width of the forecast family under the published sets
(`../report/MANUSCRIPT_NARRATIVE_REWRITE_20261011/forecastFamilyWidth.py`).

The closed-loop results are unchanged: fourteen encounters with exact
states (14 of 14 recovered, the shifted plan feasible at every sample) and
the same encounters with the estimator and bounded sensor noise for six
seeds (74 of 84 recovered, ten without a solved command, no collision, no
road departure), the in-loop estimation audit, the ten failures, and the
computation times. The plant is the prediction model with nominal
parameters; perception is not in the loop.

Removed in this revision: the road-curb extraction subsection of the former
Section III, whose code was removed from the repository on October 6, 2026
and whose figure cannot be regenerated; the obstacle perception pipeline is
now a short part of the system overview (Section II-C). Seven references
were added (Xu et al. 2018; Voros et al. 2026; Alan et al. 2023; Jeon et al.
2019; Mayne et al. 2006; Schweppe 1968; Milanese and Vicino 1991).

## Figures

`figures/` is not tracked. The manuscript uses `driving_environment_curved_v08.pdf`,
`ego_obstacle_dynamics_v02.pdf`, `system_architecture_v11.pdf`,
`yolo_lidar_projection_v01.png`, `obstacle_rectangle_fit_topdown_v01.png`
and `closed_loop_results_v03.pdf` (drawn from the recorded campaign traces by
`../report/MANUSCRIPT_ALGORITHM_SYNC_20261011/figure-sources/closed_loop_results_v03.py`).
`curb_ring_features_v01.pdf` is no longer used.

## Record of this revision

The rewrite is recorded in
[`MANUSCRIPT_NARRATIVE_REWRITE_20261011.tex`](../report/MANUSCRIPT_NARRATIVE_REWRITE_20261011.tex).
The preceding revisions are recorded in
[`MANUSCRIPT_ALGORITHM_SYNC_20261011.tex`](../report/MANUSCRIPT_ALGORITHM_SYNC_20261011.tex),
[`MANUSCRIPT_ALGORITHM_SYNC_20261005.tex`](../report/MANUSCRIPT_ALGORITHM_SYNC_20261005.tex),
[`MANUSCRIPT_ALGORITHM_SYNC_20261002.tex`](../report/MANUSCRIPT_ALGORITHM_SYNC_20261002.tex)
and
[`MANUSCRIPT_ALGORITHM_SYNC_20260919.md`](../report/MANUSCRIPT_ALGORITHM_SYNC_20260919.md).

Before submission, complete the author, affiliation, funding,
competing-interest, contribution and repository-release declarations, have
the proof of Theorem 2 and the bibliographic data of the added references
reviewed, and decide whether the estimator-alone results of Sections V-A and
V-B (measured October 1, 2026) are rerun.
