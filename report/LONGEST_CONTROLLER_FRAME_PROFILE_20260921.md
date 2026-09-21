# Longest controller frames: original measurements and component attribution

Prepared September 21, 2026. Controller revision:
`cce7dc307a6ab086f95c96db78fa5101425a5eee`.

Assessment scope clarified September 21, 2026: exclude warmup/startup overhead
from the real-time assessment. Use uninstrumented measurements after warming
the same execution path. Fresh admission after detecting an obstacle remains
online work, even when it occurs at simulation time zero; warming executable
code does not supply an inherited feasible plan.

Exact-input, uninstrumented warm replay takes **34.865/37.236 ms median/maximum**
for straight stationary and **60.888/64.033 ms** for circular crossing. All
eleven circular replays exceed 50 ms. Circular admission therefore retains a
50 ms deadline problem after warmup is excluded. The largest clean time among
these two replay fixtures is 64.033 ms; this is not an exhaustive warmed
maximum over every scenario and controller state.

Its main costs are fresh prediction/geometry construction, scalar admission
and proposal selection, joint-program construction, and one 19-iteration
Clarabel solve. Production controller and configuration files were unchanged.

## Original experimental frames retained for traceability

The original 104.576 ms straight-frame observation and 906.718 ms startup
observation remain historical records, rather than the headline warmed-runtime
results. No estimated warmup duration is subtracted from an individual frame.
The original measurements below are unchanged; the matched warm replays above
provide the applicable timing evidence for the clarified assessment scope.

The source is the campaign accompanying
[the admission repair](CIRCULAR_CROSSING_ADMISSION_REPAIR_20260921.md): three
140-hold circular warmups followed by two repetitions of six 600-hold cases.
The measured population has 7,200 issued frames. Warmups and strict-deadline
rejected attempts are separate populations. The diagnostic frame budget was
30 s, while the actual sample/hold period was 50 ms. These observations do
not certify a worst-case execution time.

| Original recorded bucket, ms | Measured maximum: straight stationary | Circular-crossing maximum | Startup warmup maximum |
| --- | ---: | ---: | ---: |
| Input preparation | 23.370 | 0.997 | 133.339 |
| Base formulation plus admission/search formulation | 73.982 | 39.456 | 647.605 |
| Conic solve wrapper | 0.000 | 15.924 | 80.323 |
| Other frame work, by subtraction | 7.224 | 8.168 | 45.451 |
| **Full experimental frame** | **104.576** | **64.545** | **906.718** |
| Native conic calls | 0 | 1 | 1 |

Source paths, relative to the campaign directory:

- `diagnostic/repeat-1-kappa-0-stationary/stationary-exact-state.mat`, frame 1.
- `diagnostic/repeat-2-kappa-0.01-crossing/crossing-exact-state.mat`, frame 1.
- `warmup-1/crossing-exact-state.mat`, frame 1.

The experiment starts its timer before synthetic ego/target input assembly
and stops after the controller returns. Plant integration and offline safety
audits are excluded. The controller's preparation bucket includes configuration,
input parsing, Frenet model creation and encounter preparation. Its formulation
bucket adds base prediction/constraint construction, scalar admission, and
any full-plan conic construction. Its solve bucket includes solver preparation,
native execution and result normalization, not just numerical iterations.

The residual includes initial/candidate/final certification, reporting and
carried-state assembly, orchestration, the experiment's input construction and
return overhead. The saved trace cannot split this original residual further.
Nor can it separate JIT compilation, class initialization, allocation or OS
scheduling inside the original 73.982 ms formulation. Later timings below
must not be presented as exact subdivisions of the original 104.576 ms.

The startup case uses a shorter trial/reference path than the measured cases;
its cold costs are retained as their own observation, not a matched warm/cold
ablation. Only circular cases were exercised in the campaign warmups, so the
first measured straight admission was not itself warmed by an identical case.

## Matched warm replay and timing isolation

Both measured maxima use the original saved initial state, road, target motion,
configuration and empty explicit carried state. Each starts a 96-hold, 4.8 s
encounter plan with 192 actuator coordinates and 97 joint certificate records.
The circular case has curvature 0.01/m, ego reference speed 8 m/s and target
crossing speed 4 m/s. Exact sensing and the declared held affine plant remain
unchanged; seed 20260912 belongs to the original experiment, and replay draws
no new random samples.

Three separate single-thread MATLAB processes perform clean timing, line
profiling and temporary-source probe timing, in that order. Each mode warms
each fixture five times. Clean and probe modes then measure eleven invocations
per fixture; line profiling records one. The machine is an AMD Ryzen 7 7800X3D,
MATLAB R2026a Update 3, `26.1.0.3276743`. This is desktop elapsed-time evidence,
without CPU isolation or hard-real-time scheduling guarantees.

| Controller-only replay, ms | Straight stationary | Circular crossing |
| --- | ---: | ---: |
| Clean median, 11 calls | 34.865 | 60.888 |
| Clean maximum | 37.236 | 64.033 |
| Clean calls over 50 ms | 0/11 | 11/11 |
| Nested-probe median | 37.112 | 64.124 |
| Nested-probe maximum | 41.964 | 66.044 |
| MATLAB line-profiler invocation | 229.081 | 310.554 |

Clean replay times the controller only; original campaign timing also includes
input construction. The straight-frame reduction is consistent with substantial
initial-use/cache and runtime-state effects. It does not identify their exact
shares or establish that startup latency has been fixed. Circular admission
requires at least 10.888 ms reduction from this clean median to reach 50 ms,
before allowing any execution margin.

Line profiling strongly perturbs MATLAB execution. Its times identify code
locations and call counts, not operational latency. Nested probes also add
overhead. The detailed tables use the **single invocation with median total
probe time** for each fixture; they do not sum independent per-component
medians. Nested rows below are partitioned to prevent double counting.

## Complete component partition of the representative probe replays

| Disjoint component, ms | Straight stationary | Circular crossing |
| --- | ---: | ---: |
| Configuration lookup | 0.063 | 0.070 |
| Input parsing/validation | 0.280 | 0.359 |
| Frenet/finite model creation | 0.067 | 0.099 |
| Encounter preparation | 0.221 | 0.328 |
| Base prediction, geometry and problem formulation | 23.420 | 25.499 |
| Scalar admission, including proposal when needed | 6.592 | 9.502 |
| Full-plan joint SOCP assembly | 0.000 | 5.118 |
| Conic solve wrapper | 0.000 | 15.846 |
| Initial witness check | 1.136 | 1.377 |
| Accepted candidate certification | 1.862 | 2.128 |
| Final controller certification | 1.860 | 1.982 |
| Command, metadata and carried-state output | 1.242 | 1.318 |
| Other orchestration and timing overhead | 0.369 | 0.498 |
| **Total** | **37.112** | **64.124** |

The initial witness check rejects the nominal overlapping plan. Straight
admission then finds a certified scalar candidate, while circular admission
excludes its whole scalar interval and constructs a separate least-violated
proposal for one full-plan solve. The two subsequent checks certify the
accepted candidate and the controller's final output, respectively.

Base formulation is the largest grouped component: 63.1% of the straight
probe frame and 39.8% of the circular probe frame. Its full internal partition
is:

| Base formulation component, ms | Straight stationary | Circular crossing |
| --- | ---: | ---: |
| Sampled cruise/reference retrieval | 0.076 | 0.084 |
| Finite prediction and anchor plan | 4.054 | 3.326 |
| Lane chart frames | 1.104 | 1.493 |
| Initial separation normals | 2.124 | 2.124 |
| Geometry/pose-domain rows and projection | 8.727 | 9.917 |
| Exit and terminal constraints | 0.767 | 0.837 |
| CLF cone and common sparse constraint assembly | 0.208 | 0.498 |
| Condensed tracking/input objective | 1.408 | 1.409 |
| Joint-support record construction | 4.174 | 4.513 |
| Remaining actuator rows, reserves and base bookkeeping | 0.778 | 1.298 |
| **Base formulation total** | **23.420** | **25.499** |

Finite bicycle prediction itself accounts for 3.988/3.250 ms inside the
4.054/3.326 ms prediction row. Terminal-set retrieval accounts for
0.085/0.089 ms inside the 0.767/0.837 ms terminal row. These are inclusive
subdivisions and must not be added again to the table total.

| Scalar admission component, ms | Straight stationary | Circular crossing |
| --- | ---: | ---: |
| Direction construction, including nominal joint residual | 0.904 | 0.899 |
| Scalar objective construction | 0.284 | 0.000 |
| Separate full-plan initialization proposal | 0.000 | 3.759 |
| Interval/dictionary processing and remaining scalar search | 5.404 | 4.844 |
| **Scalar admission total** | **6.592** | **9.502** |

The scalar search uses 32 dictionary normals and 16 amplitude cells. Straight
stationary starts with 24 overlapping nodes and returns amplitude
-2.037735995. Circular crossing starts with 16 overlapping nodes, exhausts
the scalar section, and scans its 17 boundaries for a proposal. The proposal
row includes a 0.325 ms hard-base inspection. Line profiling records 34 calls
to the vectorized rectangle-support evaluator in the circular admission;
these calls are part of scalar search/proposal work, not 34 conic solves.

| Circular full-plan work, ms | Time |
| --- | ---: |
| Sparse stage-state lifting and row compaction | 1.767 |
| Domain screening of joint records | 0.666 |
| Support-cone construction and remaining joint assembly | 2.685 |
| **Joint assembly subtotal** | **5.118** |
| Inactive column/trivial row reduction | 0.126 |
| Native Clarabel call, including interface/argument overhead | 15.584 |
| Coordinate/objective scaling and other solve-wrapper work | 0.136 |
| **Solve-wrapper subtotal** | **15.846** |

Clarabel reports status 1, 19 iterations and 15.542761 ms internally for the
representative call. Its reduced problem has 1,098 variables, 2,281 rows and
11,607 nonzeros in A. Of 97 support records, 39 require movable support-cone
construction in the line-profiled execution. The available interface does
not expose separate KKT factorization, linear-solve or line-search timers;
no such internal allocation is inferred. The 15.542761 ms is included within
15.584 ms, which is included within 15.846 ms.

| Output assembly component, ms | Straight stationary | Circular crossing |
| --- | ---: | ---: |
| Actuator/vehicle command derivation | 0.087 | 0.102 |
| Joint residual for reporting | 0.560 | 0.583 |
| Carried prediction/certificate data | 0.312 | 0.335 |
| Predicted-state propagation, metadata and remaining output | 0.283 | 0.298 |
| **Output total** | **1.242** | **1.318** |

## Interpretation and subsequent optimization targets

1. **The overall measured maximum is not caused by native optimization.**
   That frame has no conic call. Its saved formulation/search bucket is 70.7%
   of the frame and input preparation is 22.3%. Warm replay establishes a much
   lower recurring cost for this particular input, but cannot decompose its
   historical one-time costs.
2. **Circular admission retains a real warm bottleneck.** Fresh base formulation
   plus scalar search plus joint assembly consumes 40.119 ms, 62.6% of the
   representative probe frame. The native call consumes 24.3%. Treating the
   whole 64.545 ms original frame as solver time would misdirect optimization.
3. **Geometry preparation is the largest identifiable construction target.**
   Geometry rows/projection plus joint-record construction consumes 14.430 ms
   in the circular probe. Line profiling points to 96 per-stage data/label
   conversions, projection, and 97 joint-record constructions. Existing MEX
   geometry kernels are already active; MATLAB-side preparation and assembling
   their results still matter. The 1.409 ms condensed objective alone is too
   small to close the clean 10.888 ms deadline deficit.
4. **Admission selection and repeated verification merit separate study.**
   The failed scalar section and proposal cost 9.502 ms before the useful
   full-plan solve. A cheaper way to choose a full-plan center could help,
   but must retain the original hard admission checks. Candidate and final
   verification cost 2.128+1.982 ms; reuse would require proving that the
   certified program/decision remain unchanged. This analysis removes no checks.
5. **First admission dominates deadline misses in this campaign.** All six
   measured misses occur among twelve admissions. The 708 active continuation
   frames have median/max 14.103/40.644 ms and no 50 ms misses; the 6,480 cruise
   frames have maximum 16.247 ms. These are finite empirical results, not
   controller-wide timing guarantees.

## Reproduction and verification

Raw campaign directory:
`/home/zai/.cache/collisionAvoidance/circular-crossing-repair-20260921/campaign`.
Raw profiling directory:
`/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260921`.
The [JSON companion](LONGEST_CONTROLLER_FRAME_PROFILE_20260921.json) records
original maxima, clean statistics, disjoint partitions, complete nested call
trees, line hotspots and technical source/artifact hashes.

Run each MATLAB command in its own process, without overlapping timing runs:

```bash
matlab -singleCompThread -batch "addpath('scripts'); profileMatlabControllerRuntime('/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260921',Mode='replay',Selection='longest',CampaignDirectory='/home/zai/.cache/collisionAvoidance/circular-crossing-repair-20260921/campaign');"
matlab -singleCompThread -batch "addpath('scripts'); profileMatlabControllerRuntime('/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260921',Mode='profile');"
python3 scripts/buildMatlabControllerProbes.py /home/zai/.cache/collisionAvoidance/longest-frame-profile-20260921
matlab -singleCompThread -batch "addpath('scripts'); profileMatlabControllerRuntime('/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260921',Mode='probes');"
python3 scripts/analyzeLongestControllerFrames.py /home/zai/.cache/collisionAvoidance/circular-crossing-repair-20260921/campaign /home/zai/.cache/collisionAvoidance/longest-frame-profile-20260921 report/LONGEST_CONTROLLER_FRAME_PROFILE_20260921.json
```

The probe builder requires a fresh `instrumented/` destination; retain the
original artifacts and use a new output directory for a later rerun. In this
execution the probe copies were prepared before line profiling, but were only
added to MATLAB's path in the separate probe process.

All 22 measured clean calls reproduce the recorded actuator command within
1e-7, and repeated complete decisions are exactly equal. All 22 measured probe
calls and both line-profiled calls preserve those complete decisions exactly.
All issued replay decisions remain certified. The summarizer independently
selects original maxima, verifies non-overlapping timer partitions and exact
total reconciliation, and checks production/instrumented source hashes.

MATLAB Code Analyzer reports three pre-existing growth suggestions in the
standard fixture-list builder; no new diagnostic is associated with the
longest-frame selection. The user Code Analyzer settings file is absent, so
default settings were used. Python compilation and Git whitespace checks pass.
This profiling-only task did not rerun the controller's complete unit suite or
claim a new closed-loop safety experiment. No production algorithm, limits,
solver settings or certificates were modified.
