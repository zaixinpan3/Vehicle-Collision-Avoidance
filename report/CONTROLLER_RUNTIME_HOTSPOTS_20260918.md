# Current controller runtime bottlenecks

September 18, 2026. Analyzed commit `55a0f2a3a814c2732f00d7e500af63bff6900af4`; controller source
is unchanged from the straight/circular validation of the same day. This task
does not change the controller, solver, settings or network.

**Fresh encounter admission dominates the tail latency. Repeated geometric
convexification and multiple restoration/hard solves cost far more than normal
cruise, carried-witness continuation, or the final independent verifier.**
The slow +0.01/m crossing fixture remains above 100 ms even after repeated
warm execution, so first-use compilation alone does not explain its failure.

## Full-frame evidence from the closed loop

The original failing circular crossing trial's independent diagnostic run has
a 161.035 ms admission frame:

| Recorded component | Time (ms) | Fraction of frame |
| --- | ---: | ---: |
| Input preparation and contract conditioning | 7.920 | 4.9% |
| Initial formulation and subsequent reformulations | 73.253 | 45.5% |
| Restoration and hard solves, including sparse transcription | 74.118 | 46.0% |
| Measurement assembly, final verification/output and other work | 5.744 | 3.6% |

These components partition that observed frame. They are not estimates based
on source line counts. The separate strict run stopped at 106.291 ms without
a command. Its truncated runtime must not be substituted for the cost of
completing the search.

In the same completed crossing trace, the 38 active-encounter continuation
frames have median 9.193 ms and maximum 19.524 ms. Its 261 target-free frames
have median 4.613 ms and maximum 8.034 ms. Those are ordinary complete-frame
observations, including measurement construction. The high latency is a
fresh-admission event rather than a permanent 161 ms cost on every hold.

## Controlled repetition with profiling disabled

Seven exact-state fixtures were reconstructed from the saved MAT trials,
including replaying the necessary prefix for carried-witness and late-admission
frames. All replayed prefix and measured actuator inputs match the original
trace exactly (maximum observed difference zero). The work/search budget is
5 s to observe complete searches; the plant period remains 0.1 s. No tolerances,
weights, support-family rules or optimization constraints change.

Each fixture has five discarded warmups followed by eleven measured calls in
a fresh single-thread MATLAB R2026a process that never enables the profiler.
Measurements below are controller-only with preconstructed observations;
they exclude measurement assembly, plant integration and offline auditing.
They are diagnostic repetitions, not new strict closed-loop qualification.

| Fixture | Median (ms) | Min--max (ms) | Formulation median (ms) | Solve median (ms) | Native calls |
| --- | ---: | ---: | ---: | ---: | ---: |
| arc_crossing_admission | 120.923 | 119.091--128.333 | 52.406 | 67.183 | 8 |
| arc_crossing_successor | 14.176 | 13.843--18.381 | 7.274 | 5.450 | 1 |
| arc_crossing_frame10 | 11.703 | 11.510--12.462 | 5.858 | 4.529 | 1 |
| arc_stationary_admission | 46.480 | 45.130--49.215 | 22.596 | 22.094 | 3 |
| arc_oncoming_admission | 27.265 | 26.984--28.032 | 14.664 | 11.306 | 2 |
| straight_stationary_admission | 26.427 | 25.959--28.677 | 16.282 | 8.799 | 2 |
| arc_cruise_frame100 | 4.799 | 4.436--5.015 | 2.846 | 1.060 | 1 |

Column medians need not sum exactly to the median total. The 120.923 ms crossing
median comprises approximately 52.406 ms of formulation and 67.183 ms of solve
work, leaving about 1--2 ms for preparation and output. The 119.091--128.333 ms
range shows that warming the same input does not bring this fixture below
100 ms. Historical 161.035 ms and warmed 120.923 ms are different measurement
contexts and are retained separately.

## What happens inside admission

The crossing frame has 48 prediction holds (4.8 s). Its final hard program has
96 input coordinates plus one CLF slack; the sparse state formulation adds
288 state coordinates, for 385 variables. There are 606 physical rows, of
which 407 are geometry/domain rows. The lifted final program has 916 rows,
including 288 dynamic equalities, the linear cone and six second-order cones.
This current node-based program is no longer the tens-of-thousands-of-rows
whole-hold program discussed in older reports.

The search visits two support families. It solves four search-only restoration
problems and three hard problems; constraint generation expands the final hard
solve once, giving eight native invocations. The first candidate family produces
two hard infeasibility results and a restoration stall before the other family
provides an accepted plan. These are internal search outcomes. Only the final
hard-certified command is issued.

Coarse timing probes were inserted only in temporary copies under the cache.
Their measured commands are identical to the original commands, and their
native call counts agree. The instrumented crossing median is 149.356 ms,
versus 120.923 ms without instrumentation: **these subcomponent times are
instrumented diagnostics, not a replacement frame benchmark**. Seven
instrumented repetitions provide these per-frame component medians:

| Component | Calls per crossing admission | Instrumented total (ms) |
| --- | ---: | ---: |
| `avoidanceSafetyGeometry.supportNormals` | 9 | 29.559 |
| `avoidanceSafetyGeometry.build` | 5 | 22.182 |
| `hardEncounterBarrier.predict` | 1 | 2.308 |
| `hardEncounterBarrier.completionRows` | 5 | 2.994 |
| `avoidanceStageQp.build` | 7 | 6.694 |
| `solveHardCbfClf.certify` | 1 | 0.147 |

The direction proposal scans 48 predicted nodes each time, obtains rectangle
support directions, compares candidate directions against the node constraints,
and enforces the selected support sector. Geometry construction then maps the
Frenet state to certified Cartesian pose constraints, applies uncertainty and
numerical reserves, and assembles the collision/domain rows. Both repeat as
the internal search anchor and direction family change.

The native solver calls alone have median cumulative wall time **63.667 ms**:
**44.689 ms for restoration** and **18.780 ms for hard problems**. Medians of
the parts need not sum exactly. Restoration penalizes search deficits but has
zero objective curvature in the plan coordinates, so its optimum can be
nonunique. In this trace its solves need 28--30 interior-point iterations,
compared with 13 for the final successful hard solve. This is measured
iteration behavior; no claim is made that degeneracy alone explains all of it.

The representative median-total instrumented frame contains:

| Native call | Kind | Variables | Rows | Iterations | Native status | Wall time (ms) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | restoration | 438 | 940 | 29 | 4 | 11.202 |
| 2 | hard | 385 | 893 | 9 | 2 | 5.264 |
| 3 | restoration | 438 | 940 | 30 | 4 | 13.296 |
| 4 | hard | 385 | 896 | 10 | 2 | 5.255 |
| 5 | restoration | 438 | 943 | 28 | 4 | 9.946 |
| 6 | restoration | 438 | 940 | 29 | 4 | 10.245 |
| 7 | hard | 385 | 701 | 13 | 1 | 4.112 |
| 8 | hard | 385 | 730 | 13 | 1 | 4.347 |

Status 4 means AlmostSolved (an approximate restoration result), 2 means
PrimalInfeasible, and 1 means Solved, as defined by the pinned Clarabel headers.
A restoration result, even with positive solver status, is never issued as a
command. The final hard solve still has to satisfy the complete independent
physical and CLF checks. The C API wrapper constructs and frees a native solver
on each call; this experiment does not isolate initialization, factorization
and iteration costs inside Clarabel.

## Concrete redundant work

`collisionAvoidanceController.m:368` computes replacement normals before
calling the formulation with those normals. On curved fresh admission,
`formulateAvoidanceProblem.m:266` re-enters `localFormulate` to reconstruct the
local chart. That function computes a new normal schedule at line 106 and
then immediately replaces it with the supplied schedule at line 107.
The normal computation also produces diagnostics that the implementation uses;
an optimization must retain their meaning, rather than deleting the call and
silently losing diagnostics.

The observed sequence is nine direction-proposal calls and five geometry builds
for one admission. Four inner direction results are overwritten. This is a
specific engineering duplication worth removing by separating direction search
from required geometric diagnostics. It does not justify changing the accepted
constraint family, the local-domain bounds or the hard verifier.

Curved reanchoring also rebuilds the full program five times even though the
finite dynamics prediction is constructed only once. Geometry/domain bounds
and terminal exit rows that depend on the new chart must be recomputed. Dynamics,
input bounds and objective blocks that do not depend on the new chart or
support directions are candidates for reuse. Any such optimization still needs
numerical/program equivalence checks. No speedup from these proposed changes
has been measured in this task.

## Priorities implied by the evidence

1. Remove duplicate direction scoring while preserving its required diagnostics,
   and reuse formulation blocks that are unchanged across an admission iteration.
   This targets the measured repeated geometry/formulation cost.
2. Reduce unproductive restoration/hard-solve cycles and improve finite-family
   ordering using verified search diagnostics. This targets the 8-call outlier;
   it must preserve full hard certification and the carried witness. A constant
   small time budget cannot guarantee that every new encounter becomes feasible.
3. Then examine native workspace/sparsity reuse and the 7 sparse transcriptions.
   Their structure changes between restoration, hard and generated-row programs,
   so reuse needs explicit compatibility checks.

The final independent certificate is approximately 0.147 ms in the instrumented
crossing trial. The finite predictor is called once (2.308 ms), and terminal
completion rows total approximately 2.994 ms across five calls. Deleting these
small certificate components would not address the dominant costs. The native
SOCP and several geometry kernels are already compiled; compiling the remaining
MATLAB orchestration cannot be assumed to eliminate the repeated native solves
or to guarantee a Raspberry Pi deadline. No Raspberry Pi timing is inferred.

## Profiling limitations and validation

The initial diagnostic attempt interleaved a fine-grained `profile on/off`
session after each fixture. Later nominally unprofiled samples were distorted
by that process's JIT/profiler state, including large timing on native-kernel
availability checks. Those samples remain in `mixed-profile-summary.json` and
are excluded from the timing table. Fine profiles are used only for call counts
and control-flow inspection. The clean timing process never enables profiling.
Coarse probes also add overhead, disclosed above; nested inclusive times such
as `restore`, `constrained` and native execution must not be added together.

Validation: 77 clean measured command replays and 49 coarse-probe measured
replays match the recorded inputs exactly, as do the replayed prefixes. Native
call counts are preserved. The original independent acceptance checks remain
active for every call. No new unit suite was required for report-only analysis.
`git diff --check` and report aggregation assertions were run. No production
source, generated binary, network configuration or solver setting changed.

Raw scripts, fixtures, profiles, instrumentation-source hashes, observations
and logs are retained at
`/home/zai/.cache/collisionAvoidance/runtime-hotspots-20260918/`.
Run `profileCurrentController` in a fresh `matlab -singleCompThread -batch`
process for the clean measurements. `buildProbes.py` prepares temporary timing
copies; `runCoarseProbes` runs them in a separate fresh process. The committed
[JSON summary](CONTROLLER_RUNTIME_HOTSPOTS_20260918.json) includes raw-artifact
hashes, all clean samples, per-component probe statistics and native call data.
The originating experiment is
[STRAIGHT_CIRCULAR_RERUN_20260918.md](STRAIGHT_CIRCULAR_RERUN_20260918.md).
