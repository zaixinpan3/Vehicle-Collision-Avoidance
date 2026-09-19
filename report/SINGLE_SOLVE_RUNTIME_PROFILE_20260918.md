# Single-convexification controller runtime profile

September 18, 2026. Analyzed repository commit `33659dab6f2b298007598da01c7756099113452e`;
controller source remains `730c765a73195b110c431bb2f625f70bc2221d12` (format 35).
No production code, algorithm, weights, tolerances, driver, network or native
binary is changed. This report supersedes the runtime conclusions for the
previous multi-family/restoration implementation, whose measurements remain
in [the earlier report](CONTROLLER_RUNTIME_HOTSPOTS_20260918.md).

## Main findings

The current controller's warmed ordinary frames are approximately **4–14 ms**
on this desktop in the diagnostic repetitions below. The previous 739.727 ms
startup observation is not the cost of its one SOCP solve. This task measures
another first-call latency of **917.795 ms**, followed by a 13.757 ms median on
that same circular stationary fixture. Startup remains unsuitable for an
unprepared 100 ms execution loop.

**Formulation is the largest aggregate warm component.** Native conic execution
is another substantial component, especially for curved collision programs.
Computing the fixed separation directions is now less than 1 ms per measured
warm encounter frame. Active frames use one normal-generation pass and one native solve,
with no branch enumeration, restoration or solve retry.

Fast termination is not successful avoidance: four profiled admission fixtures
remain infeasible. These numbers do not upgrade the closed-loop safety or
feasibility results in the [retest report](STRAIGHT_CIRCULAR_SINGLE_SOLVE_RETEST_20260918.md).

## Measurement method

Hardware: AMD Ryzen 7 7800X3D desktop, x86_64. MATLAB R2026a with
`-singleCompThread`; the native solver also sets one thread. Eight fixtures
come from the saved straight/+0.01-per-metre circular retest. The necessary
closed-loop prefixes are replayed to reconstruct the actual stored prediction;
every prefix input agrees with the saved trace to machine equality. Fixtures
preserve the exact observed state, target, nominal plan and certificate.

Each clean fixture has five discarded warmups and 21 measured repetitions.
These measure the controller with observations already constructed, excluding
measurement assembly, plant propagation and offline audits. The deadline and
search budget are both 5 s for diagnostics; the physical hold remains 0.1 s.
This permits measuring unsuccessful attempts without deadline truncation.

A separate process runs temporary copies with coarse timing probes: five
warmups and 11 measured repetitions per fixture. Every successful input is
identical to the recorded input; every unsuccessful fixture still reports
Clarabel PrimalInfeasible. There are **168 clean and 88 coarse measured calls**.
Probes add overhead, so their component times are **not** substituted for clean
latencies. Another separate process runs MATLAB's fine profiler to inspect
call counts and lines; its heavily perturbed times are not runtime evidence.

| Fixture | Clean median / ms | Clean min–max / ms | Coarse-probe median / ms |
| --- | ---: | ---: | ---: |
| Arc stationary admission (fails) | 13.757 | 13.546–17.251 | 14.969 |
| Arc crossing admission (fails) | 13.334 | 13.145–13.778 | 14.184 |
| Straight stationary admission | 11.373 | 11.295–12.294 | 12.335 |
| Straight stationary, frame 14 | 7.876 | 7.776–9.306 | 8.465 |
| Straight oncoming admission (fails) | 8.072 | 7.965–8.335 | 8.841 |
| Arc oncoming admission (fails) | 9.746 | 9.689–9.949 | 10.610 |
| Straight cruise, frame 100 | 4.235 | 4.182–5.084 | 4.842 |
| Arc cruise, frame 100 | 4.285 | 4.212–4.467 | 4.888 |

The clean table is a repeatability measurement of eight selected frames, not a
new whole-scenario real-time qualification or a worst-case execution-time bound.
No estimator or Raspberry Pi performance is inferred.

## Where the time goes

For circular stationary admission, the instrumented total has median
**14.969 ms**. Its phase medians are **0.724 ms preparation, 8.322 ms formulation,
and 5.470 ms solve**. The remaining fraction includes error reporting and
orchestration. Medians of parts need not sum exactly to the median total.

Inside those phases:

| Component | Instrumented median / ms |
| --- | ---: |
| `avoidanceSafetyGeometry.build` | 3.583 |
| `hardEncounterBarrier.predict` | 1.758 |
| `laneGeometry.sweptCellFrames` | 0.756 |
| `avoidanceSafetyGeometry.supportNormals` | 0.626 |
| `hardEncounterBarrier.completionRows` | 0.489 |
| `avoidanceStageQp.build` | 0.886 |
| Native SOCP call, including its C/MEX wrapper | 4.344 |

The prediction row includes `ltvBicycleModel.finitePredict`; these are nested
measurements and must not be added together. Sparse transcription and native
execution are both inside the 5.470 ms solve phase. The geometry build and
other listed construction operations are inside the 8.322 ms formulation phase.

The geometry work maps predicted Frenet states into local Cartesian pose
constraints, propagates target uncertainty, constructs collision and pose-domain
rows, projects them to decision coordinates, and packages matrices and labels.
The MATLAB loops build per-node structures and concatenate them even though
rectangle support, geometric rows and row projection already use MEX kernels.
This is a larger warm construction cost than selecting the support normals.
There are no physical road boundaries in these fixtures.

The predictor constructs the multi-stage affine state maps and node enclosures.
`formulateAvoidanceProblem` then creates the condensed hard rows, objective,
soft CLF cone and terminal cones. `avoidanceStageQp.build` introduces sparse
stage states and reconstructs the stage objective for the native solve. Thus
one convex solve still entails substantial model and matrix construction.

For straight stationary admission, the warm native call is **2.369 ms**,
geometry **3.371 ms**, prediction **1.814 ms**, and sparse transcription
**0.742 ms**. For straight active continuation at frame 14, the stored
prediction avoids a fresh finite prediction call: geometry is **2.534 ms**
and native execution **2.007 ms**. These facts explain the lower continuation
latency without invoking fewer than one solve.

The final hard verifier is **0.053–0.077 ms** across the successful measured
fixtures. It is not a meaningful latency bottleneck. Terminal completion rows
are **0.196–0.489 ms**; removing the terminal certificate would target a small
component while changing the mathematical framework.

## Size and cost of the single solve

| Fixture | Native variables | Native rows | Iterations | Native wall median / ms |
| --- | ---: | ---: | ---: | ---: |
| Arc stationary admission (fails) | 385 | 907 | 8 | 4.344 |
| Arc crossing admission (fails) | 385 | 893 | 8 | 3.859 |
| Straight stationary admission | 385 | 596 | 11 | 2.369 |
| Straight stationary, frame 14 | 273 | 447 | 13 | 2.007 |
| Straight oncoming admission (fails) | 257 | 418 | 8 | 1.446 |
| Arc oncoming admission (fails) | 257 | 613 | 8 | 2.912 |
| Straight cruise, frame 100 | 129 | 182 | 9 | 0.540 |
| Arc cruise, frame 100 | 129 | 182 | 9 | 0.584 |

The 48-hold admissions have 96 input coordinates, one CLF slack and 288 lifted
state coordinates: **385 variables**. The circular stationary program contains
**907 rows**, including 288 dynamics equalities, 598 nonnegative-cone rows, a
six-dimensional CLF cone and five three-dimensional terminal cones. The
straight counterpart has fewer geometry/domain rows. This is not the former
tens-of-thousands-of-rows implementation. The native solver needs 8 iterations
to report infeasibility in the four failed fixtures and 9–13 iterations in the
successful fixtures.

`solveAvoidanceSocpMex.cpp` creates and frees the native solver workspace on
every call. Its setup, sparse factorization and iterations are included in the
native wall column; this task does not separately time those internal phases.
The native solver and geometric kernels are already compiled.

## Why startup can still take hundreds of milliseconds

A separately instrumented first circular stationary call takes **804.906 ms**:

| First-use region | Time / ms |
| --- | ---: |
| Input/configuration/model preparation | 177.674 |
| Complete formulation | 587.044 |
| Complete solve phase | 34.384 |
| Within formulation: sampled cruise certificate | 208.483 |
| Within formulation: support-direction routine | 254.577 |
| Within formulation: geometry build | 50.881 |
| Within solve: native-call wall time | 11.701 |
| Native solver's reported time | 4.868 |

The indented conceptual categories are shown as separate table rows for
readability but are nested; they must not all be summed. The first uninstrumented
call is 917.795 ms, while the previous retest's first startup attempt was
739.727 ms. Those different observations show substantial first-use variation.

The sampled cruise routine constructs the nonlinear trim, exact sampled
transition and discrete LQR certificate on initial use and caches the
compatible result. Terminal ingredients are constructed along the admission
path. The first support-direction call also traverses
previously unused MATLAB/MEX code. The first-use/warm difference is consistent
with function loading, JIT and cache initialization; the probes do not split
all of these mechanisms. It is incorrect to attribute the entire startup
latency to interior-point iterations.

The preceding closed-loop run also recorded a 91.276 ms first straight-cruise
frame: **43.966 ms preparation, 27.860 ms formulation, 6.470 ms solve**,
with the remaining 12.980 ms in measurement/output/other work. At stationary
frame 14 the same run recorded **8.309 ms total**, including **4.274 ms
formulation and 2.759 ms solve**. The recorded maximum is not the typical
steady-frame cost.

## Measurement-environment artifact

The initial analysis harness placed generated fixture/summary files in the
same directory it added to MATLAB's search path. Subsequent measurements
showed anomalous 40–74 ms warm medians. Fine profiles located roughly 19–27 ms
in individual `exist(...,'file')` checks for the support/geometry MEX functions,
not in their numeric kernels. The anomaly was also observed without profiling.

The final harness keeps code in `runner/`, temporary instrumented code in
`instrumented/`, and all outputs in their parent directory outside the search
path. Replaying the identical saved fixtures in fresh processes then produces
the 4–14 ms table above; the corresponding fine profiles no longer show those
large availability-check costs. Preserve the initial clean/path-output logs
rather than presenting them as production numeric computation. This comparison
supports a path/cache interaction but does not identify every MATLAB-internal
mechanism. It is not evidence that all prior slow frames had this cause.

## Engineering priorities

1. Prepare the configured model, cruise/CLF certificate, terminal ingredients
   and native functions before enabling the timed control loop. Warm every
   operational path that may first execute on target detection. This can
   address startup work, but a release condition must confirm readiness;
   merely excluding first frames from statistics does not meet a deadline.
2. Reduce geometry packing and matrix rebuilding: reuse invariant dynamics,
   sparsity patterns and objective blocks; batch per-node data where practical.
   Audit the current condensed-objective construction followed by sparse-stage
   objective reconstruction for avoidable duplicate work. Preserve all hard
   rows and exact program equivalence.
3. Consider compatible native-workspace reuse after measuring setup separately.
   The current wrapper allocates a solver each call. Potential savings are
   limited by the measured 0.5–4.3 ms warm native cost on these fixtures;
   no unmeasured speedup or embedded timing is promised.
4. Keep hard verification and predictive continuation. Neither their deletion
   nor changes to fixed-normal/one-solve policy are justified by these timings.
   Cache native-availability checks carefully if runtime path mutation is part
   of deployment; the measured lookup anomaly is separate from numeric work.

These are identified optimization opportunities, not implemented changes.
The fixed-family admission failures and inter-node clearance shortfall still
require separate algorithm/safety work.

## Artifacts and validation

Raw fixtures, measurement logs, timing copies, manifests and full profiles:
`/home/zai/.cache/collisionAvoidance/single-solve-profile-20260918/`.
Run `profileSingleSolve('replay')`, `profileSingleSolve('probe')`, and
`profileSingleSolve('fine')` in **separate fresh** single-thread MATLAB
processes, adding only `runner/` for the harness. Probe mode adds its isolated
instrumented directory. Raw copies preserve source hashes in
`instrumentation-manifest.json`; the companion repository JSON includes the
complete clean/coarse observations and relevant artifact hashes.

All replay outcome, exact input, prefix and single-native-call assertions pass.
Report aggregation assertions and `git diff --check` pass. No production code
changed, so no new behavioral unit tests were added or claimed; the prior
44-test result remains in the retest report. Only this report, its JSON and
the report index are in scope for this task's commit.
