# Small collision buffer and slowest-frame analysis

Prepared September 29, 2026. The production change sets `collision.safetyMarginMeters` to **0.005 m** by default; an explicit zero override remains supported. The matching configuration test and current architecture description are updated. The solver algorithm is unchanged. This is a small sampled-constraint buffer, not a new 0.10 m requirement or a continuous-time clearance guarantee.

## Focused collision revalidation

Two exact-observation scenarios at 8 m/s were rerun for 60 holds (3 s) each, covering the previously overlapping encounters. The prefix is 8 steps, sample time 50 ms, straight-road half-width 4 m, both rectangles 4.8 by 1.9 m, friction 0.85 and solver soft budget 5 s. Targets start at (24,0), heading pi, speed 8 m/s, acceleration 0 or 1 m/s squared, zero sideslip and rear-axle distance 1.6 m. There is no observer, noise, delay or random draw. Default actuator slew limits remain unlimited. The ego successor uses independent ode45 integration with relative/absolute tolerances 1e-11/1e-12, and 31 geometry samples per hold.

Comparison uses the first 60 holds of the previous zero-buffer run. Wall-time-limited optimizer trajectories can vary with host timing; this is an observed focused revalidation, not a statistically controlled timing comparison. This does not rerun all fourteen earlier cases or their full eight-second windows.

| Scenario | Holds | Previous minimum signed gap (mm) | New minimum distance (mm) | New node/midpoint minimum (mm) | New overlap samples | Dense samples below 5 mm |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| oncoming | 60/60 | -0.618235 | 4.607377 | 5.076380 | 0 | 8 |
| acceleratingTarget | 60/60 | -0.665253 | 4.062625 | 5.025709 | 0 | 9 |

Strict physical separation and achievement of the complete configured buffer are separate results. The independent full audit enforces the configured 5 mm at every replay sample, so a buffer shortfall can fail that audit despite positive physical separation. Sampled positive distance does not establish a continuous-time bound. All geometry results and full audit findings are retained in `margin-comparison.json`.

| Scenario | Later median (s) | Maximum (s) | Calls over 50 ms | Passed target | Full configured-buffer audit |
| --- | ---: | ---: | ---: | --- | --- |
| oncoming | 5.249902 | 7.901716 | 60/60 | True | False |
| acceleratingTarget | 5.332506 | 6.936761 | 60/60 | True | False |

Latency is measured but not injected into vehicle motion. These runs do not establish real-time operation.

## Recorded slowest frame and reconstruction

The previous campaign contains 1194 returned controller calls. The largest recorded call is **8 m/s brakingTarget, frame 12, t = 0.55 s, 9.113704 s**. Failed-call times in that campaign are also smaller. That original frame used zero extra buffer. Profiling deliberately retains its original zero-buffer configuration; it is separate from the new 5 mm scenario checks.

The frame contains 346 stages: 8 prefix stages and 338 hard-completion stages (17.3 s of prediction for a 50 ms command). Its original trace records one outer sequential-convexification iteration and two conic calls, primary exit flag 1 and secondary flag 0. The candidate linear residual is 0.0855543885 and its nonlinear hard violation after polishing is 311.7788126. It is rejected. The issued command comes from the already feasible initialization, whose reported safety and hard residuals are zero. The original trace does not store the solver termination message or its inner iteration count, so flag 0 alone does not identify its stopping cause.

Original source: `4e42f75c0aa64a74e98f369b75261282a09c5125`. The preceding eleven frames never retained an optimized candidate. Their stored measured states were used to reconstruct the deterministic seed/shifted continuation with improvement attempts disabled. Every applied command matches the original exactly; prediction length, absolute sample index and endpoint index are asserted. This skips unretained optimization work, rather than claiming an identical replay of the whole original prefix. The complete pre-frame ego, target, configuration and controller state is saved externally in `captured-frame.mat`.

One warmup is followed by three independently timed replays of the same captured frame. Instrumentation is confined to an external copy of `solvePredictiveControl.m`; its zero-context diff is exported (`git apply --unidiff-zero`). A separate MATLAB profiler replay localizes functions. The same five-second solver limits remain enabled: time-limited partial candidates and profiler execution paths can differ from the original. These are measured replay phase times, not recovered internal timestamps from the original 9.113704 s frame. This task ran its own MATLAB jobs sequentially. Two other project MATLAB jobs were observed during the frame replays; their process counts and CPU usage are recorded in `host-load.json`. Timings are under uncontrolled host load.

## Measured phases

Three warmed replays have total durations 9.298113, 7.742739, 11.993648 s; median **9.298113 s**. Runs 3 and 4 encounter a caught Fiala-domain error while examining their time-limited candidates; the feasible baseline is still returned. The table below uses replay 2 (9.298113 s), which completes the same one-iteration candidate-evaluation/polishing path as the original frame. It does not average missing postprocessing timers as zero.

| Nonoverlapping phase | Replay 2 seconds |
| --- | ---: |
| Initial retained-trajectory evaluation | 0.192830 |
| Linearization and conic problem assembly | 0.625372 |
| Primary safety coneprog | 0.383354 |
| Secondary cost-cone setup | 0.001535 |
| Secondary performance coneprog | 6.699505 |
| Candidate nonlinear rollout | 0.227598 |
| Endpoint correction including trial rollouts | 1.157444 |

Per-run timers and exact solver outputs (including stopping messages and iteration counts) appear in `frame-replays.json`; `phase-timing.csv` is the compact phase table.

| Replay | Primary iterations | Secondary iterations | Polishing trials | Returned source | Applied-input difference |
| --- | ---: | ---: | ---: | --- | ---: |
| 2 | 7 | 4 | 6 | feasibleInitialization | 0 |
| 3 | 7 | 2 | 0 | feasibleInitialization | 0 |
| 4 | 7 | 3 | 6 | feasibleInitialization | 0 |

## Why the frame is expensive

**The retained trajectory already attains the objective lower bound.** In every replay, initial safety slack, hard residual, CLF slack and cost are exactly zero. Both lexicographic objectives are nonnegative, so this feasible plan already attains their zero lower bounds in the evaluated nominal problem. Nevertheless, the code unconditionally enters improvement iterations while time remains. An exact lower-bound early exit would avoid the two solves and rejected postprocessing on this frame. This is a code-level opportunity, not an implemented or benchmarked fix. Initial full-tail evaluation itself still costs about 0.19 s, above the 50 ms period.

1. **The hard completion expands the problem far beyond the 8-step prefix.** At 346 stages the conic problem has 3122 variables, 2082 equalities and 10393 primary linear inequalities, before the extra secondary safety-cap row. The endpoint cone has 8 norm components; the quadratic performance epigraph has 2424. The secondary cone couples costs across the entire completion as well as the CLF relaxation.
2. **The long horizon originates upstream in terminal admission.** The previous [endpoint diagnostic](TERMINAL_CONTINUATION_RERUN_20260929/braking-endpoint-diagnostic.json) records a lateral floating-point acceleration of about -1.22465e-16 m/s squared from sin(pi). The sufficient all-future separation condition treats this as persistent acceleration. At stage 68 the original representation gives margin -7.90007 m, whereas the geometrically equivalent zero-heading/reversed-signed-speed representation gives +3.04993 m. The original seed therefore extends to stage 357; frame 12 retains 346 stages. This is numerical representation sensitivity in the infinite-future certificate, not evidence that arbitrary small physical acceleration may safely be discarded.
3. **The five-second setting is not a whole-frame deadline.** Each conic call receives the full `MaxTime`; assembly and initial rollout precede the calls, and candidate rollout plus endpoint correction follow them. The outer timer is checked only at the beginning of the next iteration. Primary success therefore does not reserve time for the secondary solve or nonlinear checks. Inspect the saved output messages to distinguish an inner iteration limit from a wall-time limit.
4. **An unsuccessful secondary candidate can still trigger expensive full-horizon work.** A finite partial conic point is rolled out and then polished before witness admission. Each polishing line-search trial calls a nonlinear evaluation of the entire 346-stage trajectory, even though only the last 20 input stages are adjusted. A rejected candidate contributes no new applied plan. Polishing counts and before/after terminal and hard residuals are recorded, so this cost is distinguishable from repeated outer solves.

Road constraints contribute 8 corner-boundary rows at each node/midpoint and endpoint: 5544 of the 10393 primary inequalities. Collision rows contribute 2772; midpoint physical-state rows 2076; the first-step CLF contributes one. Road rows are therefore numerous, but their direct construction time is not the same as their effect on conic factorization. Nested geometry timers below include all initial, assembly, candidate and polishing evaluations and must not be added to the nonoverlapping phase table.

Replay 2 direct road-row computation: 0.215296 s. Collision/target-row preparation: 0.366206 s. A road-removal counterfactual was not executed, so the net solver speedup from removing road constraints is not identified. The measured phase decomposition supports addressing the long tail and budgeting before attributing all latency to road geometry.

The separate profiler run takes 11.878270 s. Inclusive coneprog time is 7.880 s and compiled interiorPointMethod accounts for about 7.879 s. Local nominal evaluation is called eight times, with 6014 bicycle sample calls and 72343 derivative calls. These inclusive times overlap. Native interior-point internals are not decomposed further, so factorization alone is not assigned a measured share. The primary solve takes seven iterations; secondary runs stop after two to four iterations with the explicit message Maximum time limit is reached, taking 6.27--8.84 s in warmed runs. The soft solver limit can be overrun within an iteration. The endpoint radius is 0.0001220703125 in weighted coordinates; replay 2 candidate terminal violation is 826.2302764 before and after six polishing trials.

Potential follow-up changes, not implemented here: skip improvement when a feasible witness exactly attains both objective lower bounds; make straight-target terminal geometry invariant to equivalent heading/signed-speed representations without discarding physical acceleration; enforce a shared remaining frame budget across solves and postprocessing; reuse the unchanged trajectory prefix during terminal-only correction; benchmark any constraint reduction separately while retaining appropriate physical road behavior.

## Validation and artifacts

The first diagnostic-script attempt stopped on empty-struct array assignment before any timed frame replay; that script initialization was corrected and the successful replay was rerun. This was a diagnostic harness error, not a controller-call failure. 234/234 selected MATLAB tests pass, including the default-buffer/zero-override behavior. Eight independent Python audit behavior tests pass. These test results do not override the scenario-level configured-buffer audit failures. No production solver change or controller method was introduced.

Raw outputs, snapshots, logs and binary diagnostic states remain at `/home/zai/.cache/collisionAvoidance/margin-profile-20260929-043632`. The new-margin snapshot is based on `1b12f50d4c0c75ccd2c8afc0a56324b11b776643` plus the scoped configuration/test edits, with per-file hashes. The current architecture text was updated after snapshot capture; executable code did not subsequently change. `campaign.m.txt`, `profile_frame.m.txt` and `analyze_margin.py.txt` are exports of the executed diagnostic scripts, with their original external paths. The instrumented solver diff targets the prior source snapshot. No external dependency, binary snapshot or unrelated user change is part of the project commit.

`artifact-hashes.json` identifies this report and compact technical exports; `raw-artifact-hashes.json` identifies the external raw evidence. `frame-profile.json` includes inclusive/self function timings and executed-line records. Profiling overhead changes timed stopping conditions; overlapping inclusive times must not be summed.
