# Simplify the joint PCBF/CLF controller

## Implemented decision

The public controller now solves the nominal joint-state PCBF/CLF problem by
sequential convexification and applies its first input directly. Removed the
nonlinear interval/MPFR execution layer, terminal remainder synthesis,
immutable target epoch, exact-successor predicates, certificate extensions,
custom proposal hook and certificate-specific configuration. Removed
`nonlinearSafetyCertificate.m` and `nonlinearSafetyMex.cpp`.

The active geometry module is `predictiveSafetyGeometry.m`. It provides the
single constant-speed/constant-heading-rate target flow, two-rectangle dual
rows, a local LQR terminal ellipsoid, and analytic separation of its lane tube
from the target's remaining ray or orbit. The target is part of the joint state
and is initialized from each new observation. Dropouts use model propagation.

The primary LP minimizes the stage safety-slack sum. The secondary QP
minimizes lane error and CLF slack under that safety optimum. An intersecting
lane-feedback rollout is a valid SCvx initializer. The optimizer retains a
feasible shifted plan when further iterations fail or exhaust their budget.
Positive PCBF slack is exposed as recovery cost rather than being rejected by
a separate certificate gate. There is no lane/avoidance mode selector.

Analytic Fiala/body derivatives replace finite differences. Full and half holds
share one RK4 integration grid. The shift reuses the previous terminal point
and performs one nominal rollout from the current measured state. Constraint
block containers are preallocated; an already negligible nonnegative objective
can terminate improvement. No native compilation occurs in controller setup.
The core source count decreased from 24 to 23, including shared research
utilities and configuration. The remaining interval kernels are explicit
independent model-study tools, outside the public execution path.

## Mathematical scope

[PCBF_CLF_ARCHITECTURE.md](../controller/PCBF_CLF_ARCHITECTURE.md) gives the
joint model, primary and secondary problems, Li-style dual rows, and Huang's
shift argument. Recursive feasibility and zero-slack collision safety are
nominal MPC properties under terminal invariance, prediction-model consistency,
and feasible optimization. The default LQR construction establishes local
linearized contraction; nonlinear terminal invariance remains a design
assumption. A finite SCvx solve is not a proof of global positive-slack recovery.

Collision/road constraints use stage starts and midpoints. Ordinary numerical
rollout residuals belong to the optimizer; no interval bounds or continuous
intersample certificate is claimed. Offline sampled replay is validation
evidence, not an online execution requirement. Straight and constant-curvature
global corridors are supported. The sufficient terminal ray/orbit separation
can still exclude feasible maneuvers.

## Reproduction

From the repository root:

```bash
matlab -batch "addpath('scripts'); validateNonlinearPredictiveController(fullfile(pwd,'report','SIMPLIFIED_PCBF_20260928'));"
python3 scripts/auditJointPredictiveSafety.py report/SIMPLIFIED_PCBF_20260928 --output report/SIMPLIFIED_PCBF_20260928/independent-audit.json
```

The selected regression suites cover current controller behavior, configuration,
source budget and modified Fiala forces. Older affine public-entry scenarios
and certificate-format tests are outside this validation scope; the complete
repository suite was not run. No claim is made that those older interfaces have
been migrated. Factory Code Analyzer reports only sparse-index performance
notices in the SCvx assembly; the other checked current-path files are clear.

All experiments use R2026a Update 3, 0.05 s holds, deterministic inputs with no
random seed, 8 m/s reference speed, 4.8 m by 1.9 m bodies, 4 m corridor clearance
on each side and 0.10 m required collision clearance. Ego rollout uses the
configured RK4 mesh; independent held-input replay uses `ode45` with relative
and absolute tolerances of 1e-11 and 1e-12 and 31 samples per hold. The Python
standard-library audit independently computes polygon distances and corner
road margins, including the analytic turning-target pose.

Recovery starts at 0.01 m lateral error. Circular cruise uses curvature
0.005 /m. The solver-failure case starts at 0.01 m lateral error and restricts
the LP/QP solver to one iteration. The oncoming target starts at (24,0) m,
heading pi, speed 8 m/s and zero heading rate. The turning target uses the same
initial pose/speed and heading rate -0.8 rad/s. The stored-plan case exhausts the
SCvx time budget after its first solve; it continues ordinary nominal shifted
plan evaluation and terminal feedback append operations.

## Executed results

The machine-readable test, replay, timing and audit results are in
[SIMPLIFIED_PCBF_20260928/](SIMPLIFIED_PCBF_20260928/). The final result table is
assembled from those exports below.

All **102 tests passed**, with no failures or incomplete tests. All six
replays completed: **600 holds and 18,600 independently audited samples**.
Every returned replay plan had zero achieved safety slack.

| Scenario | Holds | Minimum clearance (m) | Final lateral error (m) | First call (s) | Later median / P95 (ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| recovery | 40 | No target | 0.000697289 | 0.118 | 22.05 / 25.98 |
| circular | 40 | No target | 9.56316e-15 | 0.013 | 3.95 / 5.62 |
| solverFailure | 40 | No target | 0.000734195 | 0.024 | 14.57 / 17.21 |
| storedPolicy | 160 | 0.121004 | 8.69977e-05 | 4.314 | 40.60 / 41.96 |
| oncoming | 160 | 0.121004 | 0.000159971 | 4.250 | 379.44 / 426.24 |
| turningTarget | 160 | 2.615954 | 0 | 0.078 | 43.20 / 47.22 |

Both oncoming replays passed the target and returned within 0.2 mm of the lane
center after eight seconds. Their minimum sampled clearance was 0.121004 m
against 0.10 m required, and minimum sampled road margin was 0.345634 m.
The turning target also passed its sampled geometry audit. These are simulated
nominal-model results, not vehicle measurements or intersample proofs.

Avoidance initialization still takes roughly four seconds and normal oncoming
replanning exceeds the 50 ms hold period. Reusing the feasible shifted plan
is substantially cheaper. The implementation is simpler and faster than the
previous measured certificate pipeline, but a 50 ms online SCvx optimizer is
**not established**. Timing is descriptive: cold calls are retained, host
activity was not isolated, and a separate preparation smoke check overlapped
the end of the final replay campaign. No controlled speedup ratio is claimed.

Additional checks: controller/pipeline preparation ran successfully without a
native build; factory analysis of preparation, the optional offline builder,
and configuration tests had no findings. `git diff --check` passed. Earlier
development replays were replaced by the final-source exports; no earlier
metrics are presented as final results.

## Deliberate exclusions

Pre-existing target-prediction, observer, scenario and documentation changes
remain outside this task commit. Reference PDFs, external solver trees, native
binaries and unrelated research outputs are excluded. Historical validation
reports are retained as dated records and are not rewritten to describe the
new implementation. The source/artifact manifest excludes weekly and monthly
archive documents.
