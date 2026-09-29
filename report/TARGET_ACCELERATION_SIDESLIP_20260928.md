# Constant target acceleration and sideslip

Prepared September 28, 2026. This implementation replaces the controller's independent constant-speed/constant-heading-rate target prediction.

## Model and implementation

The single target uses the same single-track relation as the existing NRMM model on its positive-speed domain:

\[
\dot p_C=V_C[\cos(\psi_C+\beta_C),\sin(\psi_C+\beta_C)]^T,
\quad \dot V_C=A_C,\quad
\dot\psi_C=V_C\sin\beta_C/l_{r,C},\quad \dot A_C=\dot\beta_C=0.
\]

The exact arc displacement is \(s(t)=V_Ct+A_Ct^2/2\). Heading changes by
\(s(t)\sin\beta_C/l_{r,C}\). Position uses the circular-arc sinc formula,
including its straight-line limit. Target speed is part of the 10-coordinate
joint state; acceleration, sideslip, rear-axle distance, and rectangle shape
are fixed parameters during a prediction. The autonomous target block is
eliminated analytically from the SCvx subproblems. The analytic joint
Jacobian includes speed-dependent heading and position.

`readControllerInputs` accepts `targetTangentialAcceleration` or NRMM's
`targetScalarAcceleration`, otherwise projects the supplied inertial
acceleration onto the target tangent; absent acceleration defaults to zero.
Sideslip is supplied explicitly or inferred from velocity and body heading.
`targetRearAxleDistance` overrides the 1.6 m configuration default. NRMM now
publishes its own rear-axle distance. The former independent heading-rate
parser and acceleration reconstruction from constant heading rate are deleted.
State version 51 prevents a previous version's target vector from being reused.
New observations update the joint state; dropouts use elapsed-time propagation.

Terminal geometry uses the fixed curvature radius for turns and analytic
quadratic displacement extrema for straight paths. It includes future
reversal and the maximum target advance relative to the ego terminal progress
bound. Current lower target speed alone no longer admits an accelerating
follower into the terminal separation condition. A stationary target remains
a fixed rectangle even with nonzero sideslip.

**Zero-speed convention:** acceleration stays constant literally. Signed
velocity crosses zero and may reverse along the same path. For reverse motion,
constant sideslip specifies the forward body tangent; actual velocity points
oppositely. This is an explicit mathematical continuation, not a stop-and-hold
driver model. The NRMM observer retains its existing positive-speed domain.

The core still consists of eight MATLAB files. The first returned PCBF/CLF/SCvx
input is executed for zero or positive safety slack. The terminal constraints
remain part of the optimizer, and offline replay is separate from execution.
The zero default clearance buffer is preserved. See
[the architecture](../controller/PCBF_CLF_ARCHITECTURE.md) for the complete
state layout, flow equations, terminal assumptions, and nominal theorem scope.

## Validation

Executed from the repository root:

```bash
matlab -batch "addpath('scripts'); validateNonlinearPredictiveController('report/TARGET_ACCELERATION_SIDESLIP_20260928');"
python3 tests/auditJointPredictiveSafetyTest.py
python3 scripts/auditJointPredictiveSafety.py report/TARGET_ACCELERATION_SIDESLIP_20260928 --output report/TARGET_ACCELERATION_SIDESLIP_20260928/independent-audit.json
```

220/220 selected MATLAB tests and 5/5 Python tests pass. The MATLAB suites cover exact-flow composition, independent ODE agreement, zero and signed acceleration, both turn directions, near-zero sideslip, reversal, analytic joint tangents, input mapping, dropout propagation, terminal acceleration geometry, direct zero/positive-slack execution, Fiala forces, road load, configuration, source size, and the NRMM model/runtime. The initial run had 128/129 passes; the new estimator-interface fixture omitted `yawRateMaximum` and other derived operating-domain setup. Corrected fixture values are used in the passing run. Initial results and log are retained.

Factory Code Analyzer inspected 15 files: the 12 existing `SPRIX` sparse-indexing performance notices remain in `solvePredictiveControl.m`; the other files have no findings. Python AST parsing and `git diff --check` pass. The tests are selected controller/NRMM regressions, not the entire repository suite.

## Closed-loop replay

All fixtures use exact ego/target observations, no noise or delay, ego reference speed 8 m/s, 50 ms holds, minimum requested horizon 8 (extended by the terminal rollout), maximum horizon 512, 24 SCvx iterations, and a 5 s soft solve budget. There is no random seed because the fixtures are deterministic. Each issued input is replayed by `ode45` with relative tolerance 1e-11 and absolute tolerance 1e-12 at 31 samples per hold. The Python audit reconstructs target motion from the exported initial state and parameters. It requires strictly positive rectangle distance with zero extra clearance buffer.

The accelerating straight and turning targets start at (24, 0) m, heading pi, speed 8 m/s, and A=1 m/s². The turn uses beta=atan(-0.1*lr), with lr=1.6 m. The braking target starts at (24, 6) m, heading pi, speed 2 m/s, A=-1 m/s², and beta=0; it crosses zero velocity at 2 s and remains laterally separated. The A=0 oncoming and turning fixtures provide regression coverage. Every export includes its complete configuration and target vector.

| Fixture | Holds | Minimum distance (m) | Minimum road margin (m) | Later median solve (s) | 50 ms misses | Audit |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| recovery | 40/40 | — | 3.03923 | 0.022404 | 1 | pass |
| circular | 40/40 | — | 3.02209 | 0.02226 | 0 | pass |
| oncoming | 160/160 | 0.0083603 | 0.43709 | 0.241804 | 160 | pass |
| turningTarget | 160/160 | 4.26484 | 3.04684 | 0.272384 | 160 | pass |
| acceleratingTarget | 160/160 | 0 | 0.515037 | 0.240865 | 160 | FAIL |
| acceleratingTurn | 160/160 | 2.57905 | 3.04644 | 0.271071 | 160 | pass |
| brakingTarget | 160/160 | 4.1 | 3.05 | 0.02359 | 0 | pass |

880 holds and 27,280 independent geometry samples were evaluated. The acceleratingTarget fixture fails the strict collision audit; its failed result is retained.

The accelerating oncoming fixture has one sampled overlap at 1.425 s, the midpoint of hold 29. Independent polygon SAT gives a 6.338043 micrometre penetration. The achieved nominal safety slack is 6.337029e-6 m, within `solver.feasibilityTolerance=1e-5`, so metadata reports `zeroSlack=true` and `scvxConverged=true`; its hard residual is zero. The hold starts with 0.006285135 m separation and ends with 0.047889425 m separation. At this constrained sample, the achieved positive slack accounts for the observed penetration within about 1 nanometre of integration difference. The strict physical collision audit still fails. See [contact diagnostics](TARGET_ACCELERATION_SIDESLIP_20260928/accelerating-contact.json).

These measurements do not establish a real-time deadline or continuous collision safety. The finite SCvx solve and sampled constraints retain the limitations already documented in the architecture. Positive slack indicates recovery, and nonlinear terminal invariance remains a design assumption. This study does not repeat the historical 15 m/s default-speed failure experiments or claim to resolve them.

## Artifacts and scope

[Summary](TARGET_ACCELERATION_SIDESLIP_20260928/summary.json), [tests](TARGET_ACCELERATION_SIDESLIP_20260928/tests.json), [code analysis](TARGET_ACCELERATION_SIDESLIP_20260928/code-analysis.json), [independent audit](TARGET_ACCELERATION_SIDESLIP_20260928/independent-audit.json), and [technical hashes](TARGET_ACCELERATION_SIDESLIP_20260928/artifact-hashes.json) accompany the two replay exports and initial failed-fixture record. Unrelated observer-theory/report edits, observer comparison work, reference PDFs, nested solver dependencies, and generated binaries are excluded from this task commit.
