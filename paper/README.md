# Draft manuscript

`manuscript.tex` is the IEEE Transactions-style research draft. The October 5,
2026 revision synchronizes it with the implementation at commit `d1c31d6`: the
controller section, the closed-loop results, the discussion, the conclusion,
the abstract and the end of the introduction are rewritten; the estimator
section gains the model-aided lateral-velocity measurement and the published
enclosures; the obstacle-perception subsection describes the pose fit that is
initialized by the estimator's prediction. The literature discussion of the
introduction, the curb-perception subsection, the estimator lemmas and theorem,
and the estimator-alone results (Sections VI-A and VI-B, measured October 1,
2026) are unchanged.

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
certificate. New in this revision: in closed loop the lateral velocity is
measured through the lateral force balance of the controller's Fiala model and
held input, with a certified interval over sensor bounds and a parameter box,
and the estimator publishes an ego zonotope and a target parameter set.

The controller is the two-stage PCBF/CLF real-time iteration of
`controller/PCBF_CLF_ARCHITECTURE.md`: one nonlinear rollout per sample,
collision rows with fixed distance-dual multipliers (a least-penetration axis
at overlap), road rows, a rear-adhesion cone and the estimator's sideslip
cone, a separating and road-recoverable terminal set with a variable horizon,
one quadratic CLF synthesized offline by an LMI, per-hold tightening by the
estimator's current enclosure, and an input trust scale estimated from the
plan innovation. The section states one lemma (fixed-multiplier rows), one
proposition (distance after a separating endpoint under constant relative
velocity) and an explicit scope: no certificate for the nonlinear vehicle and
no recursive feasibility.

The closed-loop results are the campaign of
`report/ESTIMATOR_DIRECT_TARGET_20261005.tex`: fourteen encounters with exact
states (14 of 14 recovered) and the same encounters with the estimator and
bounded sensor noise for six seeds (82 of 84 recovered, two without a solved
command, no collision, no road departure), an in-loop estimation audit, the
analysis of the two failures, a window-fit ablation, and computation times.
The plant is the prediction model with nominal parameters; perception is not
in the loop.

## Figures

`figures/` is not tracked. This revision adds `closed_loop_results_v02.pdf`,
drawn from the recorded campaign traces by
`../report/MANUSCRIPT_ALGORITHM_SYNC_20261005/figure-sources/closed_loop_results_v02.py`;
it runs no simulation. The architecture figure is still
`system_architecture_v11.pdf`
(`../report/MANUSCRIPT_ALGORITHM_SYNC_20261002/figure-sources/`); its caption,
not the drawing, carries the changed information flow.

## Record of this revision

Sources, derived statistics and open review items are recorded in
[`MANUSCRIPT_ALGORITHM_SYNC_20261005.tex`](../report/MANUSCRIPT_ALGORITHM_SYNC_20261005.tex).
The revision ran no simulation, estimator trial or unit test; every number is
taken from committed reports or derived from the recorded traces by
`../report/MANUSCRIPT_ALGORITHM_SYNC_20261005/deriveClosedLoopStatistics.py`.
The preceding revisions are recorded in
[`MANUSCRIPT_ALGORITHM_SYNC_20261002.tex`](../report/MANUSCRIPT_ALGORITHM_SYNC_20261002.tex)
and
[`MANUSCRIPT_ALGORITHM_SYNC_20260919.md`](../report/MANUSCRIPT_ALGORITHM_SYNC_20260919.md).

Before submission, complete the author, affiliation, funding, competing-interest,
contribution and repository-release declarations, and review the revised
derivations and experimental scope.
