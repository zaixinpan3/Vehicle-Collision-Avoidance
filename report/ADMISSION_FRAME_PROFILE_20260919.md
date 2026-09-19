# Slowest target-admission frame: measured cost and search trace

September 19, 2026. Controller commit
`0bac5eecb7fcdcd83c3347e5cea1e11a396aded3`, state format 38, joint trajectory/support
optimization, no fixed physical-clearance buffer. This is a profiling investigation;
production algorithms, configuration, native binaries and network are unchanged.

## Finding

The slowest frame in the previous 9,000-frame experiment is the **left-radius-50 m
circular crossing, measured repetition 2, time 0 s**. It takes **4,653.899 ms**.
Its 48-hold prediction spans 4.8 s. Fresh admission performs **123 feasibility
restoration solves and one hard performance solve**, all within this single frame.
There is one horizon attempt, no convex-base initialization solve, and three
support-angle initializations. The principal issue is repeated local feasibility
search with poor progress, multiplied by rebuilding and solving each conic program.

The first two initializations consume 60 restoration iterations each without
obtaining a certificate. The third obtains one in three iterations. A further
hard performance solve improves that verified solution. These are internal search
iterates, not 124 commands or 124 physical control periods.

## Original recorded frame

These are the original unprofiled closed-loop measurements, not reconstructed
profiler times. The frame includes synthetic measurement assembly and the complete
controller; plant integration and offline audits are excluded.

| Recorded phase | Time / ms | Frame share |
| --- | ---: | ---: |
| Accumulated conic solves, including MATLAB solver wrappers | 2,643.075 | 56.79% |
| Initial formulation and repeated convex-subproblem construction | 1,809.439 | 38.88% |
| Input preparation and model/contract conditioning | 0.919 | 0.02% |
| Remaining frame work, including search verification and output | 200.466 | 4.31% |
| Total | 4,653.899 | 100% |

The last row before the total is a subtraction residual, not a timer measuring
only final verification. Internal search repeatedly checks certificates outside
the formulation and solve timers.

## Exact replay and timing method

Reconstruct the initial observation from the saved report's initial Frenet state,
analytic arc, target state and original configuration. Use the exact curved
Frenet-to-Cartesian pose map; pass an explicitly empty previous controller state.
There is no previous encounter witness at this frame. The scene uses reference
speed 8 m/s, hold 0.1 s, sensing range 16 m, exact measurements and declared affine
plant, zero motion residuals, no road boundaries and no estimator. Both diagnostic
frame/search budgets remain 30 s so admission can finish; this is not a successful
100 ms periodic execution test.

Use the previous experiment's frozen controller/configuration/scripts and existing
native dependencies. All 89 frozen source hashes and 103 production native-binary
hashes are rechecked. Run MATLAB R2026a on the AMD Ryzen 7 7800X3D desktop with
`-singleCompThread`; Clarabel uses one thread. No simultaneous benchmark runs.

- Clean process, profiler off: two excluded warmups, then five measured calls.
  Controller-only times are **4,717.721, 4,680.097, 4,755.900, 4,707.558 and
  4,686.704 ms**; median **4,707.558 ms**. Input assembly is outside replay timing.
- Separate process: a temporary source copy adds coarse timers and a temporary
  MEX copy measures native input handling, setup, iteration call and output/free.
  Two warmups precede five measurements. Their median is **4,782.495 ms**, about
  1.59% above the clean median. The detailed tables below use this actual fourth
  replay, which is the median-total sample, so components come from one execution.
- Separate process: two warmups and one MATLAB function/line profile of the
  unmodified frozen code. Its measured controller time is **5,311.814 ms**.
  This trace identifies functions and call counts; its perturbed times are not
  real-time qualification results.

All eleven measured calls reproduce the **entire decision vector exactly**, the
same violation histories and the same 124 solver calls. The original saved first
command, horizon and violation history also match. Instrumentation does not change
the accepted trajectory in this fixture. The host's known MEX post-link inspection
warning occurred; the fresh instrumented binary subsequently executed successfully
and passed the exact-decision comparisons. Production binaries were not replaced.

## Which searches consume the time

The measured values below include subproblem construction and its solve wrapper;
search checks and frame setup/output are outside these sums.

| Initialization | Restoration solves | Hard solves | Build / ms | Solve / ms | Sum / ms | Certificate deficit, initial to final |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Original per-node support angles | 60 | 0 | 882.143 | 1,084.040 | 1,966.183 | 3.366418 to 2.863721 |
| Reference-heading + pi/2 angle seed | 60 | 0 | 879.237 | 1,537.528 | 2,416.765 | 5.814465 to 0.096391 |
| Reference-heading - pi/2 angle seed | 3 | 1 | 61.522 | 69.824 | 131.346 | 8.589583 to 0 |

Only non-exit angles are reset for alternative initializations; the exit angle
retains its initial seed. Angles remain optimization variables. These are current
local-search initializations, not fixed passing trajectories or a complete
partition of the nonconvex feasible set. No path is prescribed by this diagnostic.

The first two searches account for **4,382.948 ms**, or about **97.1% of accumulated
construction-plus-solve time** and **91.6% of the instrumented complete frame**.
A larger starting certificate deficit is not a reliable predictor of difficulty:
the third seed starts worst by that scalar but succeeds rapidly.

The first two searches hit the configured 60-iteration limit. They do not hit a
native infeasibility result. Across 124 native calls there are 122 `Solved` and two
`AlmostSolved` statuses; independent acceptance checks still apply. Native
iteration medians are 22 for initialization 1 and 33 for initialization 2. Thus
initialization 2 also costs more per solve, despite the same program dimensions.

The stall guard compares successive deficit improvement against
`1e-7 * (1 + previousDeficit)`. On the last iteration of initialization 1,
improvement is approximately 0.0002385 while the threshold is 0.0000003864.
On initialization 2, improvement is approximately 0.0007137 against
0.0000001097. Both make enough numerical progress to evade the guard while still
being far from certification. These deficits measure violation of the complete
support certificate, including exit; they are not measured physical penetration.

The original nominal prediction overlaps at eight nodes. Support directions are
nevertheless generated and restoration starts successfully. This is no longer
the earlier missing-normal initialization abort.

## Inside the native solve and repeated formulation

The following native components are disjoint within the representative replay.

| Native component, accumulated over 124 calls | Time / ms |
| --- | ---: |
| Solver iteration-call execution | 2,525.657 |
| Solver construction/setup | 125.945 |
| Input validation and cone/settings conversion | 0.725 |
| Output extraction and solver destruction | 2.404 |
| MEX boundary and instrumentation remainder | 2.239 |
| Complete measured native calls | 2,656.971 |

MATLAB work outside the native call adds about 35.829 ms to the accumulated
2,692.800 ms solve phase. Native-call median is **19.250 ms**, maximum **28.499 ms**.
The iteration-call bucket includes Clarabel's internal numerical work; this
investigation does not separately time factorization, line search or cone scaling.
The bridge and repeated solver setup are measurable but are not the leading costs.

A restoration problem sent to the native solver has **1,163 variables, 2,453 rows,
201 second-order cones and 9,079 matrix nonzeros**. The final hard problem has
1,162 variables, 2,451 rows, 201 second-order cones and 9,028 nonzeros. These are
expanded conic dimensions, including lifted state, angle and support variables;
they should not be confused with the 97 primary input/CLF decision variables.

Repeated `avoidanceStageQp.localJointConic` calls consume **1,822.902 ms**.
Their nested sparse stage transcription totals **111.988 ms**, including
16.645 ms of duplicate-row detection; most remaining time is support-majorant
construction and matrix assembly. Initial `formulateAvoidanceProblem` body time
is only 14.499 ms in this replay. Outer formulation timers additionally include
call/return and probe overhead, so the inner measurements are not a complete
partition of the reported 1,852.476 ms formulation phase.

The separate MATLAB profile identifies the assembly mechanism:

| Function or source operation | Calls | Profiled inclusive time / ms |
| --- | ---: | ---: |
| Joint conic construction | 124 | 2,198.621 |
| `pad`: repeatedly extend sparse matrices to current variable count | 170,374 | 449.939 |
| `shapeSupport`: ego and target footprint support epigraphs | 12,152 | 426.408 |
| Final constraint-matrix concatenation at `avoidanceStageQp.m:148` | 124 | 363.633 |
| `absoluteSupport`: uncertainty-support epigraphs | 6,076 | 189.640 |
| Sparse stage transcription | 124 | 130.686 |

These values overlap: for example, `shapeSupport` calls support/assembly helpers.
Do not add them. The prediction has 49 joint support records (48 collision nodes
and one exit condition), visited 6,076 times across the 124 constructions. Dynamic
column padding, small sparse allocations and repeated concatenation are concrete
implementation targets; actual savings require measurement after a change.

The representative coarse trace also records 248 joint-point verification calls,
123 base-point checks and 372 total `certify` calls, including final acceptance.
Total time inside `certify` is 166.053 ms, nested within the check wrappers and
other frame work. Certification is necessary; deleting it would change the
acceptance contract and would not address the principal repeated-search cost.

## Implications

1. Admission search strategy is the first bottleneck. The current progress rule
   permits two expensive sequences with very slow remaining-deficit reduction.
   Progress-aware stopping or better initialization deserves investigation; this
   trace does not justify always selecting a particular direction or promise
   global feasibility from one seed.
2. Repeated conic construction is the second major target. Precomputed sparsity,
   batched assembly and reuse of unchanged blocks could preserve the mathematical
   problem while reducing allocation and concatenation. No such optimization is
   implemented or claimed here.
3. Native iteration work remains substantial after assembly. A solver setup cache
   alone cannot remove the 2.526 s spent inside 124 iteration calls.
4. Removing just the first 120 solves would still leave **131.346 ms of measured
   construction and solve work**, before other frame overhead. This arithmetic is
   not an executed alternative controller or a speedup guarantee. Search-count
   reduction alone is insufficient evidence for a 100 ms first admission.

This investigation isolates runtime. It does not repair the separately documented
inter-node collision defect or expand node certification into whole-hold safety.

## Artifacts and validation

Source report: `CONTROLLER_RUNTIME_RETEST_20260919.md` and corresponding JSON/CSV.
The original frame is in
`~/.cache/collisionAvoidance/runtime-retest-20260919/measured-2/crossing-0.02/crossing-exact-state.mat`.

Raw extracted fixture, full saved decisions, histories, coarse traces, MATLAB
profile, temporary instrumented source/MEX, process logs and replay/analysis
scripts are under `~/.cache/collisionAvoidance/admission-hotspot-20260919/`.
The paired `ADMISSION_FRAME_PROFILE_20260919.json` records hashes, dimensions,
counts and numerical summaries. These are local research artifacts, not committed
binaries or new production implementations.

Reproduction uses `runner/replayAdmission.m` from that directory, the retained
frozen source and original report. Run separate MATLAB CLI processes with
`-singleCompThread` and modes `"clean"`, `"timed"`, `"profile"`; the timed mode
first requires `instrument.py` and `runner/buildProbe.m`. `analyze.py` verifies
all decisions/histories, reselects the largest recorded frame and checks source
and production-binary hashes. All three MATLAB runs and analysis complete;
`git diff --check` passes. No unit-test suite is claimed for this report-only task.
