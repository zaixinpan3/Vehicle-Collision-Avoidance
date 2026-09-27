# Online flow-reference model reconstruction

Date: September 27, 2026. The online gap identified in the preceding anchor study is now implemented. A selected flow reference rebuilds the prediction model and its dependent optimization problem before any control input can be accepted. Six closed-loop trials and exact-input replays validate the integration; complete avoidance/recovery and real-time execution remain unachieved.

## Implemented behavior

`solveHardCbfClf.prepare` attempts the previous trajectory model. Previous input amplitude/slew violations and recognized tire/Frenet operating-domain failures invalidate that anchor and produce a cruise bootstrap for geometric fitting only. Errors outside the recognized operating-domain failures still propagate. A valid previous model receives the first hard solve; rejection also enters flow reconstruction.

For a fresh trajectory frame, `fixedDirections` fits the flow reference on the bootstrap, projects the fitted inputs onto amplitude and causal slew bounds from the actual prior input, and calls `formulateAvoidanceProblem` with the bounded reference. The nonlinear rollout starts at the current measurement. Stage dynamics, tire tangents, feedback gains/tubes, geometry, hard rows, terminal/CLF cones and the input-deviation objective center are rebuilt together. Changing the passing side repeats that complete reconstruction. The existing target-reaction and direction searches retain the selected operating trajectory.

Projection changes the geometric path and affine endpoint fit; it does not certify the reference. Both ordinary and alternate references require the existing solver-status, lifted-cone and original-coordinate acceptance checks. Failed references never issue a command. No controller method, nonlinear shooting implementation, softened safety row or new configuration selector is added. The default input-deviation weight remains zero.

The controller now returns the selected model reference and takes its prediction and CLF metadata from the final accepted program. Prepared trajectory exports disable a second geometric refit because the standalone frame kernel cannot rebuild dynamics. Its replay remains a solver adapter, not a standalone closed-loop controller. Explicit replay of an already prepared flow model also retains its operating trajectory; the earlier isolated-anchor diagnostic uses that path. Inherited unchanged-model certificates and target-free continuation retain their existing behavior.

## Experiment

MATLAB R2026a, independent PassVeh14DOF physical plant, exact controller observations, 50 ms hold, 10 m/s ego and target, requested 18 s duration and final 2 s recovery window. The straight, radius-100-m circular and S-bend cases retain the preceding experiment geometry, road boundaries, sensing range, horizons and 5 s search budget. Configuration and scene geometry equality against the prior weight-matched experiment are asserted. No random perception or estimated-state campaign was introduced.

Commands:

```matlab
addpath('scripts');
maxNumCompThreads(1);
runInputDeviationPenaltyExperiment(outputDirectory,[0,10],18);
auditInputDeviationPenaltyExperiment(outputDirectory);
```

The actual batch driver and logs are retained externally. It performed one discarded S-bend preparation call before timing the scenarios. Trials ran serially on the shared workstation; elapsed figures are observational rather than isolated real-time benchmarks. Final integration refinements were checked by exact public-input replay of all six recorded physical trials. Runtime numbers refer to the original physical execution, before removal of an unnecessary no-conflict direction-restoration attempt. Initial harness/test issues were resolved: the probe needed an explicit repository working directory; a tampered stored certificate was correctly rejected, so anchor-domain tests use the model-preparation interface; the alternate-side test disables intermediate reaction attempts to select the intended branch. The export test now replays the accepted model and declares its native solver path fixture. Expanded regression also exposed an optional model-metadata assumption in the synthetic fixed-program interface, which was fixed. The continuation test now explicitly uses the unchanged-model setting required by its witness-transfer claim, paired with a new trajectory-model rejection test; the initial-source expectation now records the actual flow source.

## Closed-loop results

| Scenario | Deviation weight | Prior failure (s) | New failure (s) | Minimum sampled SAT gap (m) | New median / p95 call (ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| straight | 0 | 2.05 | 2.05 | 4.019774 | 32.7 / 190.2 |
| circular | 0 | 1.50 | 1.80 | 9.533296 | 9.8 / 386.5 |
| sCurve | 0 | 1.40 | 1.75 | 10.525062 | 2239.3 / 2368.2 |
| straight | 10 | 2.40 | 2.40 | 0.113203 | 31.9 / 76.2 |
| circular | 10 | 2.45 | 2.45 | 0.209581 | 33.5 / 108.6 |
| sCurve | 10 | 2.35 | 2.35 | 0.261758 | 185.0 / 2407.9 |

All six runs stop on optimization rejection and none completes avoidance/recovery. The prior zero-weight circular case stopped during trajectory assembly at 1.50 s; the new online fallback passes that failure point. Positive gaps refer only to the executed sampled trajectory up to rejection and do not establish safety after stopping or between samples. Improvements in stopping time are not full-task success.

## Reference and model audit

| Scenario / weight | Accepted flow frames | Accepted previous-plan frames | Recoveries after invalid prior anchor | Maximum reference rollout difference |
| --- | ---: | ---: | ---: | ---: |
| straight_w0_t18 | 0 | 20 | 0 | not applicable |
| circular_w0_t18 | 5 | 10 | 0 | 0 |
| sCurve_w0_t18 | 6 | 8 | 0 | 0 |
| straight_w10_t18 | 0 | 27 | 0 | not applicable |
| circular_w10_t18 | 0 | 28 | 0 | not applicable |
| sCurve_w10_t18 | 0 | 26 | 0 | not applicable |

The 11 accepted flow frames followed rejected previous-model solves; model-domain recovery is separately exercised by regression tests. Every replayed issued command agrees within 1e-7 with the recorded command; failure timestamps and identifiers reproduce. On every accepted flow frame, the model input sequence equals the selected reference, its recorded linearization states equal a fresh nonlinear reference rollout within 1e-10, and the input-deviation center exactly equals the actual stage tire operating inputs. Counts above exclude rejected frames; `reference-audit.csv` also records reconstruction counts on accepted frames.

| Scenario / weight | Maximum normalized candidate departure | Maximum first-hold force error (kN) | Maximum whole-plan force error (kN) |
| --- | ---: | ---: | ---: |
| straight_w0_t18 | 1.999971 | 44.282 | 64.072 |
| circular_w0_t18 | 1.999972 | 27.828 | 134.424 |
| sCurve_w0_t18 | 1.999986 | 65.788 | 65.788 |
| straight_w10_t18 | 0.464191 | 3.352 | 19.559 |
| circular_w10_t18 | 1.004802 | 7.954 | 12.380 |
| sCurve_w10_t18 | 0.672866 | 12.670 | 30.346 |

Force errors compare affine and nonlinear Fiala forces at the same candidate affine-predicted state/input; they are not measured plant forces. Invalid slip stages are counted separately and omitted from the force maximum. A bounded, valid reference does not guarantee that the optimized trajectory stays near its tangents. Rebuilding fixes the initialization/model mismatch but does not remove the remaining model-extrapolation, optimization-feasibility or runtime issues.

## Validation and scope

- 246 MATLAB regressions passed, zero failed/incomplete, covering flow/previous-reference branches, trajectory/feedback/CLF consistency, input-deviation objective, hard certificates, target reaction, continuation, native frame replay, deadlines and the 20-file core budget.
- Eight new behavior tests cover first flow reconstruction, prior-input rejection, prior tire-domain failure, prior solver rejection, alternate-side reconstruction, prepared-reference replay, rejection without an old-plan fallback and input/slew projection with optimizer rejection.
- Independent Python rectangle-SAT calculations agree with all six minimum gaps within 1e-8 m; completion/recovery predicates are independently checked.
- Factory Code Analyzer is clean on eight of nine changed MATLAB files. The solver retains one pre-existing FNDSB advisory on its unchanged find-based row indexing. Scoped whitespace checks pass. No generated standalone library or native dependency is committed.
- The previous feasibility report remains a historical diagnostic. This change installs the online policy; it does not claim that all previously diagnosed failures are repaired.

Raw MAT results, source copies, test/probe logs and batch drivers:
`/home/zai/.cache/collisionAvoidance/flow-relinearization-20260927`.
Compact CSV/JSON exports and source/artifact manifests are adjacent to this report. Full physical traces remain external. The native manifest inventories available MEX binaries, rather than tracing per-call loading. Existing target/observer/scenario/document workspace changes used by the run are deliberately excluded from the implementation commit.
