# Draft manuscript

`manuscript.tex` is the IEEE Transactions-style research draft. The October 2,
2026 update keeps its section order and leaves the introduction's literature
discussion and the perception section unchanged. The estimator derivations
receive small clarifications of notation and wording only. The gain selection,
the controller, the results, the discussion and the conclusion are synchronized
with the committed implementation at `5692e15`, and the second half of the
title now reads "Covariant Ego--Target Estimation and Two-Stage Predictive
Control". The reference paper is a structural and rhetorical exemplar; the
manuscript text and derivations are specific to this project.

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
observer and the two-stage ISS comparison. The target injection gains are the
solution of one semidefinite program that minimizes a bound on the
noise-induced velocity-error variance inside the Lipschitz certificate,
followed by the free-metric bandwidth search. The estimator results are the
current design values (MnCAV stock rear-axle distance) and the held-out
comparison with a reproduced reference observer tuned per scenario. The
manuscript states that the estimator gains of that comparison are synthesized
anew from the operating domain declared for each scenario, and it reports the
metrics that favor the reference.

The controller is the two-stage PCBF/CLF real-time iteration: an RK4
linearization of the combined-slip Fiala bicycle along a nonlinear rollout,
collision rows with fixed distance-dual multipliers, a freely placed terminal
core, one nominal cost-to-go CLF and a moving-target flow initialization. The
section states three formal results (fixed-multiplier rows, free-pose core
reduction, cost-to-go decrease) and an explicit scope: no certificate for the
nonlinear vehicle and no recursive feasibility.

The closed-loop results are the fourteen-scenario campaign of controller
commit `a29ccb6` with exact states and an ODE45 replay of the prediction
model: twelve runs end collision-free with cruise recovered, one collides
(15 m/s turning crossing) and one stops on a distance-dual numerical check
(8 m/s braking lead). Every controller call exceeds the 50 ms hold, the
maneuvers leave the road corridor, and the estimator and controller are not
evaluated together. The failure mechanisms are part of the results.

The manuscript follows the committed controller. When this revision was
written the working tree held a concurrent, uncommitted within-hold
relinearization change and a closed-form distance-dual construction; the
manuscript names both only as next steps and reports no result for them.

## Figures

`figures/` is not tracked. This revision uses two new assets there,
`system_architecture_v11.pdf` and `closed_loop_results_v01.pdf`. Their sources
are tracked in `../report/MANUSCRIPT_ALGORITHM_SYNC_20261002/figure-sources/`;
the closed-loop figure is drawn from the recorded campaign traces and runs no
simulation.

## Record of this revision

Source versions, the numerical audit, the review passes, the review items left
open and the deliberate exclusions are recorded in
[`MANUSCRIPT_ALGORITHM_SYNC_20261002.tex`](../report/MANUSCRIPT_ALGORITHM_SYNC_20261002.tex).
The revision ran the committed estimator gain synthesis and evaluated
controller constants; it ran no closed-loop simulation, no estimator trial and
no unit-test suite. Check the quoted numbers against their records with:

```bash
python3 report/MANUSCRIPT_ALGORITHM_SYNC_20261002/auditManuscript.py . /tmp/audit.json 5692e1551241b568bcfb300093814373c674a412
```

(run from the repository root; the third argument is the preceding revision).
The preceding revision is recorded in
[`MANUSCRIPT_ALGORITHM_SYNC_20260919.md`](../report/MANUSCRIPT_ALGORITHM_SYNC_20260919.md).

Before submission, complete the author, affiliation, funding, competing-interest,
contribution and repository-release declarations, and review the revised
derivations and experimental scope.
