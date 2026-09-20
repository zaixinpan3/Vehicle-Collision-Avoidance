# MATLAB joint-support assembly optimization — September 20, 2026

The sustained avoidance bottleneck is substantially reduced. In paired warm
replays of the same straight stationary-target continuation, controller-only
median/max falls from **56.656/59.164 ms to 30.428/32.494 ms**. In two complete
closed-loop campaigns, all 544 active continuation frames are below 50 ms;
their median/max is **13.9275/42.613 ms**. First admission remains a separate
unresolved timing bottleneck, reaching **97.773 ms**.

## Implemented changes

`controller/avoidanceStageQp.m` now assembles each support record in local
coordinates: six stage-state components, one direction increment, and that
record's auxiliary variables. A fixed index map places its triplets in the
complete horizon matrix. Previously, small intermediate sparse matrices used
the complete program width (1,545 columns in the slow straight fixture).

The implementation reserves triplet, bound and cone-size buffers once and
fills them using cursors. It removes the repeated sparse padding helpers;
only the base equalities need additional global columns. Variable and row
ordering, coefficients, scaling, objective and cone definitions are preserved.
The global problem is not reduced or approximated further.

`controller/avoidanceSafetyGeometry.m` skips slicing and copying physical or
geometric fields when no collision/exit rows remain to remove. This matters
for the already-filtered inherited family. Fresh admission still builds its
geometric data and performs the necessary filtering.

No safety constraint, independent verification, terminal continuation,
uncertainty enclosure, CLF slack, horizon, solver setting or input limit is
changed. No C regeneration or Raspberry Pi/network action is performed.

## Measurement scope

- Baseline source: `c9c8df43e126acb2182f5df9ebba52e805efdaf1`. A frozen copy
  remains only in the external benchmark cache, not in the repository.
- AMD Ryzen 7 7800X3D, MATLAB R2026a; sequential fresh
  `matlab -singleCompThread -batch` processes. The existing native Clarabel and
  geometry MEX dependencies remain active; this is the MATLAB pipeline, not
  the generated standalone controller adapter.
- Straight and analytic radius-100-m circle, each with stationary, oncoming
  and crossing targets. Controller only; exact sensing and declared affine
  plant; seed 20260912; speed 8 m/s; sensing radius 16 m; 50 ms hold/node/update;
  nominal 1.6 s horizon; no road boundary or fixed clearance buffer.
- Paired comparison: the same 13 previously captured input/carried-state
  fixtures, five discarded warmups and 21 measured calls per fixture for
  each version. Full successful decision vectors must equal the saved
  baseline exactly. Expected failure must remain failure.
- Complete campaigns: one excluded 120-hold warmup per case, then two measured
  campaigns requesting 600 holds per case. All first admissions, including
  rejected attempts, remain in the 6,002 measured frames. Frame timing covers
  synthetic measurement construction and the full controller, excluding plant
  integration, file output and offline collision audit. Diagnostic computation
  budgets remain 30 s, with misses counted against 50 ms.
- Function profiles and seven-repeat coarse instrumentation run in separate
  processes. These times identify components; they are not deadline samples.

## Full frames after warmup

The baseline columns use the preceding identically configured
[MATLAB profile campaign](MATLAB_RUNTIME_PROFILE_20260920.md). They are separate
campaigns, not synchronized per-frame timing pairs.

| Category | Frames | Baseline median / max ms | Improved median / max ms | Above 50 ms, before / after |
| --- | ---: | ---: | ---: | ---: |
| Active-target continuation | 544 | 18.2655 / 65.469 | **13.9275 / 42.613** | 13 / **0** |
| No active target | 5,446 | 7.064 / 10.377 | 7.125 / 10.765 | 0 / 0 |
| First admission, including rejection | 12 | 29.543 / 96.576 | 30.783 / **97.773** | 3 / **3** |
| All attempted frames | 6,002 | 7.075 / 96.576 | 7.137 / 97.773 | 16 / 3 |

The pooled median is dominated by target-free cruising. This change improves
active avoidance; it does not establish a reduction in cruise or admission
latency. Small changes in those medians must not be attributed to the joint
assembly optimization. These are measured maxima, not worst-case execution
bounds or a complete 50 ms real-time guarantee.

| Scenario | First admission, run 1 / run 2 ms | Continuation median / max ms |
| --- | ---: | ---: |
| Straight stationary | 97.773 / 76.489 | 16.9675 / 42.613 |
| Straight oncoming | 25.610 / 28.279 | 12.099 / 20.026 |
| Straight crossing | 17.704 / 16.444 | 7.296 / 9.276 |
| Circular stationary | 52.544 / 49.849 | 16.122 / 36.063 |
| Circular oncoming | 26.553 / 26.144 | 11.805 / 20.428 |
| Circular crossing, rejected | 33.287 / 33.336 | No accepted continuation |

Oncoming admission is frame 53 at 2.6 s, after target-free operation. The other
cases have an initially visible target. Their first frame is measured after
the separate warmup and is not discarded.

The slowest continuation remains straight stationary frame 2: preparation
2.684 ms, combined formulation 19.493 ms, solve 13.220 ms and remainder
7.216 ms, totaling **42.613 ms**. The corresponding previous campaign maximum
was 65.469 ms, of which formulation consumed 42.463 ms.

The slowest admission is straight stationary frame 1: preparation 7.534 ms,
combined formulation/admission 77.954 ms, zero solve and remainder 12.285 ms,
totaling **97.773 ms**. Fresh admission uses the scalar-section method and
never calls the optimized joint SOCP assembler. It still requires separate
prediction/geometry/admission optimization.

## Paired controller-only replays

Each row has 21 clean measurements of identical saved inputs per version.

| Fixture | Baseline median / max ms | Improved median / max ms |
| --- | ---: | ---: |
| Straight stationary admission | 37.551 / 41.911 | 34.549 / 38.754 |
| Straight stationary continuation | **56.656 / 59.164** | **30.428 / 32.494** |
| Straight stationary cruise | 7.547 / 8.681 | 7.002 / 7.848 |
| Straight oncoming admission | 25.129 / 25.935 | 23.478 / 35.860 |
| Straight oncoming continuation | 29.498 / 30.788 | 19.198 / 19.617 |
| Straight crossing admission | 18.499 / 19.243 | 14.459 / 15.356 |
| Straight crossing continuation | 11.672 / 12.204 | 8.676 / 9.408 |
| Circular stationary admission | 37.981 / 40.506 | 34.782 / 37.211 |
| Circular stationary continuation | 40.571 / 43.940 | 31.338 / 33.161 |
| Circular stationary cruise | 8.167 / 8.622 | 7.619 / 8.056 |
| Circular oncoming admission | 24.552 / 26.109 | 21.751 / 23.156 |
| Circular oncoming continuation | 23.670 / 29.640 | 19.800 / 20.342 |
| Circular crossing rejected admission | 34.796 / 36.571 | 30.755 / 33.289 |

The straight stationary continuation median decreases by **46.3%**. Repeated
same-input admission is much faster than admission during the mixed closed-loop
campaign, both before and after this revision. Its different execution context
and timing boundary must not replace the actual 97.773 ms first-frame result.

## Current bottleneck after optimization

A representative instrumented straight stationary continuation takes
**31.205 ms**. Its disjoint buckets are:

| Component | Previous profile ms | Improved profile ms |
| --- | ---: | ---: |
| Joint conic assembly | 30.565 | **7.053** |
| Solver wrapper | 11.993 | 11.978 |
| Three complete certificate checks | 5.384 | 5.525 |
| Inherited base formulation | 5.203 | 3.865 |
| Remaining work | 2.763 | 2.784 |
| Total | 55.908 | 31.205 |

Joint assembly decreases by approximately **76.9%** in these coarse profiles.
The solver is now the largest bucket. Its native call takes 11.737 ms with
17 iterations, still solving 1,545 variables and 2,828 rows. The program
retains 198 second-order cones and 95 remaining stages. Inherited joint-record
handling takes 0.447 ms inside base formulation, compared with 1.696 ms in the
previous profile. These nested times are not added again to the total.

For circular stationary continuation, the representative total is 31.977 ms:
4.646 ms joint assembly, 14.870 ms solver wrapper, 5.624 ms certificate checks,
4.291 ms base formulation and 2.546 ms remainder. Its native solve still uses
19 iterations, 1,095 variables and 2,280 rows.

Warm straight admission remains dominated by base formulation: a 37.745 ms
coarse invocation contains 24.070 ms base work, 6.371 ms scalar admission,
4.814 ms verification and 2.490 ms other work. Geometry takes 9.104 ms inside
the base. These replay measurements do not subdivide the actual 97.773 ms
campaign admission beyond its recorded phase timers.

## Correctness and retained limitations

- All **278 captured conic programs** match the frozen assembler exactly,
  including complete output structures, matrices, objectives, cones and indices.
- All 273 paired successful/expected-failure calls per version preserve the
  baseline decision or rejection. Additional clean replays, coarse probes and
  function profiles also preserve complete decisions.
- All **12 closed-loop trials** exactly match the baseline input arrays,
  state arrays, completion status, executed hold count, minimum sampled body
  gap and sampled collision-free status.
- **737/737 MATLAB tests pass**, including six new combinations checking that
  fixed-plan conic feasibility agrees with independent support geometry for
  fixed, interval and full-angle yaw uncertainty, with positive/negative
  certificate margins and two distinct support records.
- Code Analyzer reports no issues in the geometry, test or benchmark files.
  The assembler retains three informational allocation/indexing advisories,
  with no errors. The long suite exceeded the MCP response timeout but
  continued in MATLAB; its saved results confirm all 737 tests passed.

This is an equivalent implementation improvement. The previous **node-only**
safety scope remains: offline inter-node sampling still finds small overlaps
in straight stationary (minimum -0.000551365 m) and straight oncoming
(-0.00538249 m). Circular crossing still fails fresh admission. Runtime
improvement does not resolve those independent algorithm limitations.

## Reproduction and artifacts

Raw results, frozen source, fixtures, profiles, test results and logs are under
`/home/zai/.cache/collisionAvoidance/matlab-assembly-optimization-20260920/`.
The [JSON summary](MATLAB_ASSEMBLY_OPTIMIZATION_20260920.json) includes full-frame
categories, maxima, paired comparisons, coarse breakdowns, profile records,
validation counts and technical-artifact hashes.

Run each timing command in a separate fresh process, sequentially:

```bash
matlab -singleCompThread -batch "addpath('scripts'); benchmarkMatlabControllerFixtures('/absolute/baseline-profile','/absolute/output/baseline-fixtures.json',ControllerDirectory='/absolute/frozen/controller');"
matlab -singleCompThread -batch "addpath('scripts'); benchmarkMatlabControllerFixtures('/absolute/baseline-profile','/absolute/output/current-fixtures.json');"
matlab -singleCompThread -batch "addpath('scripts'); profileMatlabControllerRuntime('/absolute/output',Mode='campaign');"
matlab -singleCompThread -batch "addpath('scripts'); profileMatlabControllerRuntime('/absolute/output',Mode='replay');"
matlab -singleCompThread -batch "addpath('scripts'); profileMatlabControllerRuntime('/absolute/output',Mode='profile');"
python3 scripts/buildMatlabControllerProbes.py /absolute/output
matlab -singleCompThread -batch "addpath('scripts'); profileMatlabControllerRuntime('/absolute/output',Mode='probes',Repetitions=7);"
python3 scripts/analyzeMatlabControllerProfile.py /absolute/output /absolute/output/profile-summary.json
matlab -batch "results = runtests('tests'); assertSuccess(results)"
```

The frozen copy must precede the optimization and remain external. The paired
fixture driver validates saved decisions automatically. The report JSON adds
the paired comparisons and complete-campaign equivalence audit to the profile
aggregator output; the corresponding analysis script is retained in the raw
artifact directory as `finalizeComparison.py`.
