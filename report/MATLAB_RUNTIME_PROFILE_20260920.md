# Current MATLAB controller profiling — September 20, 2026

The principal sustained bottleneck is **assembling the joint trajectory/support
conic program**, especially on the straight stationary-target continuation.
The solver is significant but is not the largest component of that frame.
Fresh admission has a different bottleneck: prediction/geometry/base construction
plus scalar-section admission. The previous multi-round restoration analysis
does not describe the present algorithm.

This study uses the original MATLAB controller at source commit
`ecf782e7af33668f90a3e793ebc0ed3631279a03`, with its existing Clarabel and geometry
MEX dependencies. The generated standalone controller adapter is not used.
Production controller/configuration/solver code is unchanged. Profiling drivers
and temporary diagnostic copies are the only executable changes.

## Measurement method

- AMD Ryzen 7 7800X3D desktop, MATLAB R2026a, fresh sequential
  `matlab -singleCompThread -batch` processes; one Clarabel thread.
- Straight and analytic radius-100-m left-circle references; stationary,
  oncoming and crossing targets; seed 20260912; exact sensing and declared
  zero-residual affine plant. Cruise 8 m/s, sensing radius 16 m, 50 ms input
  hold/prediction node/update, nominal 1.6 s horizon. No road boundaries or
  fixed physical clearance buffer. This is controller-only, without estimator.
- A separate 120-hold campaign warms all six cases. Two subsequent campaigns
  request 600 holds per case, yielding 6,002 measured attempts. **First target
  admission is retained**, including each rejected circular crossing attempt.
  Diagnostic computation budgets are 30 s; misses against the actual 50 ms
  period are counted, not truncated. These are observations, not strict
  real-time or physical-vehicle certification.
- Closed-loop frame timing includes synthetic measurement construction and the
  full controller call, excluding plant integration, file output and offline
  collision audits. The clean timing process never enables the profiler.
- Thirteen exact input/carried-state fixtures cover six admissions, the
  slowest continuation in each of the five admitted cases, and straight/circular
  target-free cruise. Prefix replay matches the original commands. A second
  clean process performs five discarded warmups and eleven controller-only
  measurements per fixture, with observation construction outside the timer.
- Separate processes collect MATLAB function/line profiles and seven coarse
  timed replays per fixture. The complete decision vectors remain identical;
  the rejected admission remains rejected. Coarse tables below use the single
  median-total replay so components belong to the same invocation. Inclusive
  parent/child timers are never added together.

## Warm full-frame measurements, admission included

| Frame category | Calls | Median ms | Maximum ms | Above 50 ms |
| --- | ---: | ---: | ---: | ---: |
| All measured attempts | 6,002 | 7.075 | 96.576 | 16 |
| New-target admission, including two rejected attempts | 12 | 29.543 | 96.576 | 3 |
| Active-target inherited continuation | 544 | 18.266 | 65.469 | 13 |
| No active target | 5,446 | 7.064 | 10.377 | 0 |

The pooled median is dominated by target-free cruising and must not represent
active avoidance. All thirteen continuation misses occur in the straight
stationary-target case.

| Scenario | First admission, run 1 / run 2 ms | Continuation median / max ms |
| --- | ---: | ---: |
| Straight stationary | 96.576 / 78.239 | 26.750 / 65.469 |
| Straight oncoming | 24.703 / 24.625 | 16.497 / 30.006 |
| Straight crossing | 19.903 / 19.199 | 9.667 / 12.350 |
| Circular stationary | 51.837 / 44.263 | 18.528 / 45.327 |
| Circular oncoming | 26.649 / 24.631 | 12.992 / 23.980 |
| Circular crossing, rejected | 34.340 / 32.437 | No accepted continuation |

Oncoming admission occurs at frame 53, time 2.6 s, after target-free operation.
Stationary/crossing targets are initially visible. Their admission is the first
scenario frame, but follows the excluded program warmup and is still retained.

The actual slowest continuation is straight stationary frame 2, time 0.05 s:
2.668 ms input preparation, 42.463 ms combined formulation, 13.044 ms solve,
and 7.294 ms remaining work, totaling **65.469 ms**. The formulation timer
includes both inherited base construction and the joint conic construction.

The actual slowest admission is straight stationary frame 1: 7.195 ms
preparation, 77.207 ms combined formulation/admission, zero solve, and
12.174 ms remaining work, totaling **96.576 ms**. This is a new MATLAB
measurement, not the historical 92.155 ms checked-MEX hybrid result.

## A sustained continuation hotspot

The exact straight stationary frame-2 fixture has clean controller-only
median/max **57.745/58.620 ms** after five same-input warmups. It remains above
50 ms; startup overhead alone cannot explain the runtime problem.

The representative coarse replay takes **55.908 ms**. The following buckets
partition that instrumented invocation without double-counting:

| Component | Time ms | Fraction |
| --- | ---: | ---: |
| `avoidanceStageQp.joint`: construct joint conic program | 30.565 | 54.7% |
| Solver wrapper, including native Clarabel | 11.993 | 21.5% |
| Three complete certificate checks | 5.384 | 9.6% |
| Inherited base formulation | 5.203 | 9.3% |
| Input preparation, outputs and other work | 2.763 | 4.9% |
| **Total instrumented controller invocation** | **55.908** | **100%** |

The native solver call itself takes 11.750 ms, reporting 17 iterations and
`Solved`; wrapper work outside the call is about 0.243 ms. The call includes
native setup and solver execution; this study does not separately measure
factorization, cone scaling or setup inside Clarabel.

There is **one** conic solve, not repeated feasibility restoration. The prediction
has 95 remaining stages: although only 190 control coordinates plus one CLF
slack are primary variables, lifting the states and support expressions gives
**1,545 solver variables, 2,828 rows and 198 second-order cones**. The rows
include 570 dynamic equalities and 1,469 linear inequalities. The long horizon
is inherited from the 96-stage, 4.8 s finite encounter-completion certificate;
the configured nominal 32-stage horizon is not the size of this avoidance frame.

### Why assembly costs more than solving

The MATLAB profile records these counts in one straight continuation assembly:

| Operation | Calls |
| --- | ---: |
| `pad`: form sparse matrices at final column width | 1,803 |
| `absoluteSupport`: absolute-value support epigraphs | 287 |
| `shapeSupport`: vehicle rectangle support | 192 |
| `addLinear`: extract and append row triplets | 325 |
| `addCone`: extract and append conic triplets | 192 |

`avoidanceStageQp.m` constructs small sparse selector matrices and support maps
for each collision/exit record, repeatedly concatenates padded matrices, calls
`find`, and grows triplet/value/bound arrays. Precomputing the final variable
count and using triplets removed some earlier overhead, but did not eliminate
these per-record allocations and helper calls. The profile points to
`localJointConic`, `absoluteSupport` and sparse padding, rather than repeated
control solves, as the remaining assembly mechanism.

Function profile times overlap and are heavily perturbed: the same straight
continuation takes 227.889 ms with full profiling, compared with 57.745 ms in
clean replay. Profiled inclusive times are therefore used for call structure
and line attribution, not substituted into the clean timing table.

A separate count-only audit finds that **1,802 of 1,803** straight padding calls
already receive the final column width; only one actually needs extra columns.
The circle similarly has 718 of 719 calls already at full width. Both audited
complete decisions remain identical. This identifies a concrete redundant
operation; the audit measures call conditions, not a speedup from removing them.

### Why this straight case assembles more slowly than the circle

The circular stationary continuation has clean median/max **40.731/43.424 ms**.
Its representative coarse invocation is **41.216 ms**:

| Component | Straight ms | Circle ms |
| --- | ---: | ---: |
| Joint conic construction | 30.565 | 11.843 |
| Solver wrapper | 11.993 | 14.958 |
| Complete certificate checks | 5.384 | 5.568 |
| Base formulation | 5.203 | 6.368 |
| Remaining work | 2.763 | 2.479 |

Both have 95 stages and 96 collision/exit records. The circle's hard local
pose domains allow `localDomainCertificate` to prove 56 records already safe
over their entire admitted domain. Only 40 records require the expensive
support-cone expansion. The straight chart lacks the `domainCenter` field
required by that shortcut, so all 96 records are expanded. Correspondingly,
the circle makes 719 padding calls instead of 1,803 and solves a problem with
1,095 variables and 86 second-order cones. Its native solve takes 14.725 ms
and 19 iterations; fewer variables do not automatically imply a faster solve.

This does not mean curves are intrinsically easier. It identifies a specific
implementation asymmetry in the current redundant-constraint certificate.
Any analogous straight-path reduction must be proved over the actual feasible
family, for example from existing actuator/slew reachability. Arbitrarily
deleting rows or adding restrictive pose limits would change the problem.

## Admission and target-free work

For the exact straight stationary admission fixture, clean median/max is
**35.919/37.975 ms**. The representative coarse replay is 37.922 ms:

| Disjoint component | Time ms |
| --- | ---: |
| Initial base formulation | 23.674 |
| Scalar affine-section admission | 7.085 |
| Complete certificate checks, three calls | 4.837 |
| Remaining work | 2.326 |
| SOCP solve | 0 |

Inside the 23.674 ms base formulation, geometry construction takes 8.974 ms,
joint-certificate record construction 4.301 ms, prediction 3.768 ms, support
direction proposals 2.255 ms, frame construction 1.180 ms, and completion rows
0.776 ms. These nested figures explain the base bucket and must not be added
again to the total above. The circle's admission is comparable: clean
36.149/37.972 ms, coarse geometry 9.939 ms and scalar admission 7.030 ms.

There is concrete duplicated representation work: `geometry.build` assembles
fixed-direction collision rows; `jointProgram` creates the joint support
records, removes those collision/exit rows, and filters their per-cell fields.
Inherited continuation repeats the field filtering even though the previous
accepted joint program already removed those rows. Shared target/pose data
remain necessary; avoiding discarded row construction and unnecessary copying
is a narrower improvement than removing geometry or certification.

Fixed-input warmed admission is materially faster than first admission during
the mixed-scenario closed loop. Both measurements are retained. Observation
assembly, execution context and warm/cache state differ; the current evidence
does not isolate one cause for the entire discrepancy. The fine/coarse replay
cannot retroactively assign the original 96.576 ms to individual functions.

Target-free cruise has clean fixture medians 6.909 ms straight and 7.045 ms
circular. Coarse geometry construction is 3.075/3.165 ms and prediction
1.261/1.149 ms, while native solves are 0.905/0.996 ms. A proved no-target
construction path may help this baseline cost, but it does not address the
dominant straight avoidance continuation above 50 ms.

## Priorities supported by these measurements

1. Assemble joint support rows/cones in preallocated numeric buffers, with
   fixed index maps and fewer temporary sparse objects. Avoid padding inputs
   already at their final width. Preserve coefficients, scaling, cones and
   independent verification; test program/decision equivalence.
2. Generalize the existing whole-family redundant-record proof to straight
   references using sound existing reachability bounds. Reuse the inherited
   certificate and verify any replacement family. The observed 96-versus-40
   expansion count motivates this; no speedup for a new proof is yet measured.
3. Separate geometry-data construction from fixed-direction row assembly so
   joint admission does not build rows it immediately discards. Avoid repeated
   filtering/copying of already stripped inherited geometry.
4. Audit the three certificate calls for safe reuse of an unchanged verified
   result across the internal interface. Retain independent post-solve
   verification. Their approximately 5 ms cost is secondary to assembly.
5. Then investigate solver sparsity/workspace reuse and, separately, a simpler
   target-free preparation path. The current data do not support treating
   solver iterations as the sole bottleneck or assuming that compilation alone
   resolves the complete MATLAB frame deadline.

No proposed controller optimization is implemented or claimed as a measured
speedup in this profiling task. The existing node-only safety scope, diagnostic
inter-node overlaps in two straight cases, and circular-crossing admission
failure remain unchanged.

## Reproduction and validation

Run sequentially in **separate** MATLAB processes:

```bash
matlab -singleCompThread -batch "addpath('scripts'); profileMatlabControllerRuntime('/absolute/external/profile-directory',Mode='campaign');"
matlab -singleCompThread -batch "addpath('scripts'); profileMatlabControllerRuntime('/absolute/external/profile-directory',Mode='replay');"
matlab -singleCompThread -batch "addpath('scripts'); profileMatlabControllerRuntime('/absolute/external/profile-directory',Mode='profile');"
python3 scripts/buildMatlabControllerProbes.py /absolute/external/profile-directory
matlab -singleCompThread -batch "addpath('scripts'); profileMatlabControllerRuntime('/absolute/external/profile-directory',Mode='probes',Repetitions=7);"
python3 scripts/analyzeMatlabControllerProfile.py /absolute/external/profile-directory report/MATLAB_RUNTIME_PROFILE_20260920.json
```

The executed study uses
`/home/zai/.cache/collisionAvoidance/matlab-profile-20260920/`. Full traces,
fixtures, MATLAB profiles, diagnostic source copies and logs remain there.
The [JSON summary](MATLAB_RUNTIME_PROFILE_20260920.json) includes full-frame
groups, selected maxima, clean replay statistics, coarse partitions, profile
counts/hottest lines, solver sizes and technical-artifact hashes.

Validation checks 143 clean calls, 91 coarse calls and 13 profiled calls across
13 fixtures. Successful complete decision vectors are unchanged; failed
admission remains failure. Prefix commands match within the asserted `1e-7`
bound. Aggregation checks category counts, nonoverlapping coarse partitions,
native-call counts and unchanged instrumented production-source hashes. Code
Analyzer reports three growth advisories in untimed fixture-list construction,
with no executable-code errors. No new full unit-test suite is claimed for
this diagnostic-only change. The first replay helper required the straight
road origin mapping; profile export required explicit string conversion and
scalar cell wrapping. These tooling issues were corrected before the final
successful runs; their initial logs are preserved.
The additional two padding-count replays also preserve complete decisions;
their diagnostic sources and results are retained in `padding-audit/` and
`padding-audit.json` under the same external directory.
