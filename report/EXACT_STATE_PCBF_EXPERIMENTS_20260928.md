# Exact-state experiments for the simplified PCBF controller

Date: September 28, 2026. Production source commit: `c0ce5a479868ef94504f3bf6171c0eefbfeeb281`.
MATLAB: 26.1.0.3276743 (R2026a) Update 3.

All eight replays completed without sampled rectangle overlap or road crossing.
Five of eight passed the strict dense clearance audit. Both oncoming cases and the
15 m/s turning-target case missed the configured 0.10 m margin; the latter missed
it by 26.885 mm. Every target-case solve exceeded the 50 ms control period.

## Scope and configuration

Executed fresh closed loops for the current single PCBF/CLF sequential-convexification
controller. Ego and target states are supplied exactly, with no observer, noise,
measurement delay or uncertainty envelope. Production controller and configuration
sources were unchanged. The experiment driver now exports iteration diagnostics,
configuration and explicit observation assumptions, and accepts configuration overrides.
Previous trajectories are numerical warm starts; every hold runs fresh optimization.

Two cohorts use the same four fixtures. The baseline retains the existing driver's
8 m/s reference and minimum horizon of 8 holds. The default cohort passes an empty
configuration override, using the production defaults of 15 m/s and 16 minimum holds.
Encounter initialization extends the horizon automatically; actual horizons are in the
JSON summaries. Both cohorts use 50 ms holds, 4.8 by 1.9 m rectangles, an 8 m wide
corridor, 0.10 m collision clearance, friction coefficient 0.85, 40-degree steering
limit and the default unbounded input slew rates. Full configurations are exported;
JSON nulls in rate/time fields represent MATLAB infinities where applicable.

Recovery starts 0.01 m off a straight lane, so this is a small-error recovery test.
Circular cruise starts at its trim with curvature 0.005/m. These cases run 40 holds
(2 s). Both target cases run 160 holds (8 s), starting the target at (24, 0) m,
heading pi, tangential speed 8 m/s; target yaw rate is zero for oncoming and -0.8 rad/s
for turningTarget. The target speed stays 8 m/s in the default cohort. No random draws
are used. Variable-curvature/S-bend roads are outside the current model's supported
straight/constant-curvature scope and were not tested.

The controller uses its nonlinear combined-slip Fiala RK4 prediction. Held inputs
are replayed with the same physical equations but an independent `ode45` integration
(RelTol 1e-11, AbsTol 1e-12), with 31 samples per hold. Python independently checks
rectangle distances; an additional whole-body circular-road check includes radial
extrema along rectangle edges. This is nominal-model, sampled validation, not an
independent high-fidelity vehicle plant or a continuous-time safety proof.
Simulation time advances by 50 ms regardless of solve duration: measured overruns are
reported and are not injected as actuation latency.

## Executed results

8/8 replays completed; 800 holds and 24,800 dense samples were audited.
All 136/136 selected controller/configuration/model regression tests passed.
The complete estimator/perception test suite was not rerun for this experiment-only change.

| Ego reference | Scenario | Holds | Minimum clearance (m) | Minimum road margin (m) | Final lateral error (mm) | Later median / P95 (ms) | First call (s) | Calls over 50 ms |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 8 m/s | recovery | 40/40 | None | 3.039233 | 0.708 | 21.18 / 23.37 | 0.127 | 1 |
| 8 m/s | circular | 40/40 | None | 3.022088 | 0.000 | 21.66 / 22.60 | 0.029 | 0 |
| 8 m/s | oncoming | 160/160 | 0.099049 | 0.337452 | -5.195 | 267.34 / 2989.01 | 5.031 | 160 |
| 8 m/s | turningTarget | 160/160 | 2.618563 | 3.046506 | -3.110 | 287.02 / 309.46 | 0.288 | 160 |
| 15 m/s | recovery | 40/40 | None | 3.039185 | 0.053 | 40.99 / 78.51 | 0.920 | 6 |
| 15 m/s | circular | 40/40 | None | 3.033143 | 0.000 | 40.71 / 44.64 | 0.067 | 1 |
| 15 m/s | oncoming | 160/160 | 0.099879 | 0.236834 | -1.593 | 218.32 / 1433.33 | 3.324 | 160 |
| 15 m/s | turningTarget | 160/160 | 0.073115 | 2.739007 | 1.152 | 211.45 / 1993.80 | 2.217 | 160 |

`None` denotes a fixture without a target. Timing percentiles exclude the first call.

## Safety and solver findings

- 8 m/s recovery: 40/40 calls satisfy the implemented SCvx convergence test; termination counts `{'zeroSlack': 40}`. Maximum nominal safety-slack sum 0; maximum nominal hard residual 0.
- 8 m/s circular: 40/40 calls satisfy the implemented SCvx convergence test; termination counts `{'zeroSlack': 40}`. Maximum nominal safety-slack sum 0; maximum nominal hard residual 0.
- 8 m/s oncoming: 157/160 calls satisfy the implemented SCvx convergence test; termination counts `{'timeLimit': 3, 'zeroSlack': 157}`. Maximum nominal safety-slack sum 7.88785802e-06; maximum nominal hard residual 0. Closest approach 0.099049399 m at 1.535000 s; shortfall against the configured clearance 0.950601 mm. Dense overlap samples: 0; samples below clearance: 8.
- 8 m/s turningTarget: 160/160 calls satisfy the implemented SCvx convergence test; termination counts `{'zeroSlack': 160}`. Maximum nominal safety-slack sum 0; maximum nominal hard residual 0. Closest approach 2.618562850 m at 1.426667 s; shortfall against the configured clearance 0.000000 mm. Dense overlap samples: 0; samples below clearance: 0.
- 15 m/s recovery: 40/40 calls satisfy the implemented SCvx convergence test; termination counts `{'zeroSlack': 40}`. Maximum nominal safety-slack sum 0; maximum nominal hard residual 0.
- 15 m/s circular: 40/40 calls satisfy the implemented SCvx convergence test; termination counts `{'zeroSlack': 40}`. Maximum nominal safety-slack sum 0; maximum nominal hard residual 0.
- 15 m/s oncoming: 158/160 calls satisfy the implemented SCvx convergence test; termination counts `{'zeroSlack': 158, 'timeLimit': 2}`. Maximum nominal safety-slack sum 3.31903439e-06; maximum nominal hard residual 0. Closest approach 0.099878972 m at 1.021667 s; shortfall against the configured clearance 0.121028 mm. Dense overlap samples: 0; samples below clearance: 4.
- 15 m/s turningTarget: 159/160 calls satisfy the implemented SCvx convergence test; termination counts `{'zeroSlack': 159, 'missingDiagnostic': 1}`. Maximum nominal safety-slack sum 9.87853796e-06; maximum nominal hard residual 0. Closest approach 0.073114741 m at 1.018333 s; shortfall against the configured clearance 26.885259 mm. Dense overlap samples: 0; samples below clearance: 6.

In the default turning-target case, the critical 1.00--1.05 s hold starts at
0.287772 m clearance, reaches 0.073115 m at 1.018333 s, is back to 0.100546 m
at the constrained midpoint (1.025 s), and ends at 0.207203 m. Its minimum
across all actual nodes and midpoints is 0.100546 m. Thus the dense violation
is specifically between constraint samples, despite exact state observations.
The oncoming node/midpoint minima are 0.099999100 m at 8 m/s and 0.099998130 m
at 15 m/s, within numerical tolerance of the 0.10 m requirement.

One default turning-target call at 0.65 s executes all 24 iterations without
meeting the convergence test. Its exported termination string is missing
(JSON null); summaries label this `missingDiagnostic`. There were no caught
solver exceptions in that call. The final subproblem succeeded, but earlier
failed subproblems left the diagnostic unset. This reporting defect is retained
as observed and was not repaired in this experiment task.

Nominal zero-slack status means at or below the configured 1e-5 tolerance, not exact
zero. Direct execution can also apply a result from a call that reaches its time or
iteration limit. Per-iteration LP/QP exit flags, including unsuccessful subproblems,
are retained in the raw traces; the compact summaries count them. The primary and
secondary optimization stages are not separate controller modes.

The strict audit is intentionally allowed to fail when dense clearance drops below
0.10 m. An experiment completing without rectangle overlap does not establish that
the configured safety margin was respected. The sample/midpoint constraints leave
intersample clearance as an outstanding limitation. No safety retuning or controller
fix was made during this task.

Target encounters remain slower than the 50 ms control period. Timing comes from
ordinary workstation execution; other MATLAB processes were observed, with no CPU
isolation. These measurements do not establish a controlled speedup over older runs.
The 5 s SCvx budget is checked between iterations and is not an interruptible LP/QP
wall-clock guarantee.

## Artifacts and reproduction

Compact per-cohort summaries, independent/strict audit outcomes, per-frame timing
CSVs (exported with normalized LF endings), tests, analyzer results and source/artifact manifests are in the sibling
`EXACT_STATE_PCBF_EXPERIMENTS_20260928/` directory. Full traces, solver iterations,
logs, analysis scripts and driver snapshots are retained outside the repository at:

`/home/zai/.cache/collisionAvoidance/direct-pcbf-experiments-20260928/`

The baseline was executed before adding the configuration override argument; its
exact instrumented driver snapshot is retained and checked against its source hash.
The final driver has identical baseline fixture defaults. The default cohort uses
the final driver. The first attempt to launch the default cohort failed before any
simulation because MATLAB `run` changed directories; the external driver was corrected
to use an explicit repository working directory and rerun successfully. Its original
launch log remains available.

```matlab
addpath('scripts');
validateNonlinearPredictiveController('/absolute/output/baseline');
runNonlinearPredictiveSafetyValidation(Scenarios=["recovery","circular"], ...
    Frames=40, ControllerConfiguration=struct(), ...
    OutputFile='/absolute/output/default/short-replays.json');
runNonlinearPredictiveSafetyValidation(Scenarios=["oncoming","turningTarget"], ...
    Frames=160, ControllerConfiguration=struct(), ...
    OutputFile='/absolute/output/default/oncoming.json');
```

Create output directories first. For each output directory, run:

```bash
python3 scripts/auditJointPredictiveSafety.py /absolute/output/baseline --output /absolute/output/baseline/strict-audit.json
```

The strict audit's nonzero exit is a recorded safety-margin result. It should not be
suppressed or interpreted as a MATLAB execution failure. The final experiment-driver
Code Analyzer result is exported separately; the controller analysis retains its
existing sparse-indexing performance notices. Generated figures, raw traces,
dependencies and unrelated working-tree changes are excluded from the project commit.
