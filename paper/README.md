# Draft manuscript

`manuscript.tex` is the IEEE Transactions-style research draft. The September
19, 2026 update preserves its section order, mathematical presentation,
introduction literature discussion and perception section while synchronizing
the estimator and controller with the inspected implementation. The reference
paper is a structural and rhetorical exemplar; the manuscript text and
derivations are specific to this project.

Compile from this folder with:

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error manuscript.tex
```

Clean generated LaTeX intermediates with:

```bash
latexmk -c manuscript.tex
```

The estimator now uses direct body-velocity observation, a parallel yaw
observer, a two-stage ISS comparison and free-metric target gain synthesis.
The controller uses one joint trajectory/support convex restriction, a soft
sampled CLF, a robust modal cruise terminal set and a fixed encounter-exit
deadline. Its recursive feasibility statement preserves the complete admitted
certificate under explicit model, sensing and execution assumptions.

The manuscript distinguishes recorded controller admission-search results
from the current single-restriction policy. Positive node gaps do not imply
separation inside a held command; two recorded straight-road trials collide
between nodes. Physical road boundaries, nonlinear-plant inclusion, universal
timely admission and complete estimator-in-the-loop avoidance remain outside
the demonstrated results. The estimator's continuous theorem is separate from
its sampled implementation.

Source versions, numerical checks and deliberate exclusions are recorded in
[`MANUSCRIPT_ALGORITHM_SYNC_20260919.md`](../report/MANUSCRIPT_ALGORITHM_SYNC_20260919.md).
No new MATLAB simulation or unit-test execution is claimed for this manuscript
revision. Existing figure assets are used to compile the local PDF.

Before submission, complete the author, affiliation, funding, competing-interest,
contribution and repository-release declarations, and review the revised
derivations and experimental scope.
