# Adopt first-admission assembly shortcuts and validate runtime

Date: September 22, 2026. Baseline revision:
`ff2a901369623b9c0fef1de1956d08a3dbc00f44`.

The production controller now avoids projecting preliminary collision rows
that the joint formulation replaces, and skips normal/support evaluation for
physically dominated VFFM candidates. Alternating warmed measurements reduce
circular-crossing median first-admission time from **50.4685 to 46.377 ms**,
a **4.0915 ms / 8.1%** improvement. The full 799-test regression passes.
All 12 diagnostic closed loops preserve their baseline controls and states
exactly. Strict 50 ms completion is 10/12, with two first-frame timeouts
remaining. This is an improvement, not a hard real-time guarantee.

The [machine-readable results](NRMM_VFFM_ADMISSION_OPTIMIZATION_20260922.json)
retain every measured population, failed attempt, equivalence comparison,
independent footprint audit and technical artifact hash.

## Production changes

- `avoidanceSafetyGeometry.build` accepts an optional `includeCollisionRows`
  argument, defaulting to true for existing callers. Fresh joint admission
  passes false. For node cells, filter collision rows before projecting into
  the full control space. Retain target occupied sets, uncertainty, separating
  normals, road boundaries and pose/reference-domain rows. The final joint
  SOCP still contains the required hard collision inequalities. The geometric
  row kernel itself remains unchanged; this optimization avoids its discarded
  rows' projection and downstream assembly, not all initial geometry work.
- `solveHardCbfClf` evaluates the existing physical-base feasibility before
  candidate normals. If a finite-scored physically feasible fit already exists,
  a later physically infeasible fit cannot win the existing ranking and is
  skipped. Preserve candidate order and tie rules. A skipped support score
  remains `Inf` to mean unevaluated, rather than a measured support violation.
- Retain both fitted references, 96-step conflict horizons, all 192 control
  variables, one full SOCP and the independent final verifier. Solver settings,
  configuration, model, control period and safety margins are unchanged.
  Certification-result reuse is not part of this change.

`benchmarkAdmissionOptimization.m` alternates the preserved baseline and the
current implementation. The earlier prototype builder now directs current
users to this production comparison; historical prototype reconstruction
requires the matching pre-adoption checkout.

## Regression and numerical behavior

**799/799 MATLAB tests pass**, zero failed/incomplete, aggregate duration
513.843304 s. The seven added parameterized cases cover road/pose row retention
with zero, one and two uncertain targets, selected-candidate preservation on
mirrored curves, and successful selection when the physically feasible
candidate is second. Existing coverage exercises continuation, uncertainty,
multiple targets, actuator/slew limits and the prepared-frame native adapter's
MATLAB implementation.

In the added quadratic-road geometry test, changing the projected matrix row
count changes BLAS rounding at approximately machine precision. Matrix
coefficients are checked within 1e-13 and evaluated residuals within 1e-12;
footprint data, normals and physical bounds remain exact. An initial test
helper swapped the wrong reference dimension; the finalized helper swaps
candidate rows. These test-authoring corrections precede the complete passing
regression. No physical-road closed-loop claim follows from this assembly test:
the terminal certificate's previously documented road-boundary limitation
remains.

The MATLAB MCP request hit its 300 s response timeout, while MATLAB continued
the regression. Saved results were later loaded and `assertSuccess` verified;
the suite was not run twice. Timing began only after the persisted complete
passing result. Factory Code Analyzer reports no issues in the new test,
benchmark, geometry or formulation files. One pre-existing `FNDSB` advisory
remains in `solveHardCbfClf.m`. Python compilation, manifest checks and whitespace
checks pass. No generated native binary was rebuilt in this task.

## Alternating same-input timing

Use a fresh MATLAB R2026a Update 3 `-singleCompThread` process with profiling off.
No other task-controlled MATLAB workload runs concurrently. Keep original
source copies outside the repository and verify their SHA-256 hashes against
the baseline Git revision. Reuse the three exact first-admission fixtures from
the preceding profiling study. The diagnostic deadline is 30 s, so overruns
remain observable instead of being excluded as failed samples.

For each fixture, run four rounds, alternating baseline/optimized block order.
After each source switch, exclude five warmup calls and measure 15 calls.
This yields **60 calls per version per fixture, 360 measured calls and 120
excluded warmup calls**. Golden-result preparation and post-call equivalence
assertions are outside timing. All 360 complete decisions, control plans,
predicted states and final `A`, `b`, `P`, `q`, cone, physical-bound, label,
joint-certificate, completion and geometry data equal the baseline exactly.
Each call remains independently certified with one native solve.

| First admission | Baseline median | Optimized median | Decrease | Baseline maximum | Optimized maximum | Over 50 ms, before/after |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Straight stationary | 45.703 ms | 44.041 ms | 1.662 ms / 3.6% | 55.148 ms | 52.009 ms | 6/60 to 1/60 |
| Circular stationary | 49.047 ms | 47.7615 ms | 1.2855 ms / 2.6% | 56.529 ms | 52.473 ms | 18/60 to 8/60 |
| Circular crossing | 50.4685 ms | 46.377 ms | 4.0915 ms / 8.1% | 54.566 ms | 52.100 ms | 37/60 to 2/60 |

Every round's median improves in each scene. Circular-crossing decreases are
3.570, 3.987, 4.237 and 4.263 ms across rounds. Straight-stationary decreases
are 1.623, 0.842, 3.352 and 1.189 ms; circular-stationary decreases are
1.706, 0.492, 1.720 and 1.194 ms. Finite host scheduling variation remains;
none of these maxima is a worst-case execution-time bound.

| Circular-crossing component median | Baseline | Optimized |
| --- | ---: | ---: |
| Input preparation | 0.8305 ms | 0.7880 ms |
| Formulation, initialization and conic assembly | 33.3455 ms | 29.2505 ms |
| Native solver timing category | 9.6315 ms | 9.6390 ms |

The decrease comes from preparation before the numerical solve. The native
solver's own time is essentially unchanged. Component medians are not additive
partitions of the median total. The separate legacy 44-call replay is also
retained in the JSON; it is not mixed into this paired population.

## Closed-loop verification

After paired timing, run the existing six straight/circular cases with two
repetitions under each deadline: **24 trials**, each requesting 600 holds
(30 simulated seconds). Ego reference speed is 8 m/s; hold time is 50 ms,
nominal performance horizon 1.6 s and curvature 0 or 0.01/m. Use exact sensing,
zero declared plant residual, disabled physical road boundaries, seed 20260912,
30 s diagnostic versus 50 ms strict frame deadlines, and 30 s search limit.
Exclude 1,680 warmup holds and the separate replay's 20 warmup calls.

| Population | Completed trials | Executed holds | Rejected attempts | Median frame | Maximum frame | Above 50 ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Diagnostic | 12/12 | 7,200 | 0 | 7.114 ms | 49.299 ms | 0 |
| Strict 50 ms | 10/12 | 6,000 | 2 | 7.105 ms | 52.709 ms | 2 |

Strict statistics retain both rejected attempts, totaling 6,002 timings.
One straight-stationary first frame takes **52.709 ms**, and one
circular-stationary first frame **50.819 ms**. Both reject without applying
any command. Their second repetitions complete. Both circular-crossing strict
trials complete, with maximum frames **47.959 and 48.552 ms**. Diagnostic
continuation median/max is 11.9415/27.572 ms; diagnostic admission median/max
is 39.897/49.299 ms across 12 scene admissions.

The preceding baseline campaign completed 6/12 strict trials, versus 10/12
now. Those campaigns were collected at different times; the alternating replay
above provides the more controlled speed comparison. No timed sample or failed
attempt was removed to produce this result.

All 12 diagnostic trajectories have **zero state difference and zero control
input difference** from the corresponding saved baseline runs over all 600
holds. All 30 existing variation fixtures admit, including 28 with active
targets. These are previously used development fixtures, not held-out evidence.

Independent footprint reconstruction at 5 ms spacing and 0.1 ms refinement
near closest approaches gives the same minimum body gaps as before:

| Scene | Refined minimum body gap |
| --- | ---: |
| Straight stationary | 0.0000927376 m |
| Straight oncoming | 0.0380147 m |
| Straight crossing, non-conflict control | 9.51948 m |
| Circular stationary | 0.0994193 m |
| Circular oncoming | 0.1341962 m |
| Circular crossing | 0.1550021 m |

Independent SAT and rectangle-distance overlap signs agree, and reconstructed
endpoints match exactly. Straight crossing has no cruise collision threat and
is retained as a non-conflict control. The very small straight-stationary gap,
optional heading-envelope excesses, finite sampling and declared affine plant
remain limitations; this performance change establishes no additional physical
robustness or continuous-time safety guarantee.

## Reproduction

Raw results and original source copies are under
`/home/zai/.cache/collisionAvoidance/nrmm-vffm-optimization-20260922/`.
Fixture inputs are under `nrmm-vffm-profile-20260922/`; prior closed-loop
results are under `nrmm-vffm-validation-20260922/` beside that directory.

1. Run the MATLAB regression from the repository root and persist `TestResult`
   plus count/duration summaries before asserting success. The recorded MCP
   invocation runs external `runRegression.m`; its saved suite contains all
   799 repository tests.
2. In a fresh `matlab -singleCompThread` process, call
   `benchmarkAdmissionOptimization(RAW, BASELINE_SOURCE_DIRECTORY, FIXTURE_DIRECTORY)`.
   Preserve all three original files: `avoidanceSafetyGeometry.m`,
   `formulateAvoidanceProblem.m`, `solveHardCbfClf.m`.
3. Run `runFixedDirectionValidation` modes `replay`, `campaign`, `sensitivity`
   serially, followed by `auditFixedDirectionCampaign`. The external
   `runTiming.m` records these exact calls.
4. `python scripts/analyzeAdmissionOptimization.py RAW BASELINE_CAMPAIGN OUTPUT`
   summarizes every population, checks baseline source provenance and compares
   all diagnostic state/control sequences. Factory analyzer output is retained
   separately in `code-analysis.json`.

Further reductions can target redundant certificate evaluation and batched
normal/record assembly. They require separate implementation and validation;
this change adopts only the two measured, selection-preserving shortcuts.
