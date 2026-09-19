# Current controller runtime retest

September 19, 2026. Tested commit `0bac5eecb7fcdcd83c3347e5cea1e11a396aded3` (controller-state
format 38, without the fixed physical-clearance configuration). No controller,
configuration, solver, native binary, estimator or network changes in this task.

## Result

**Stable active-encounter frames: median 15.816 ms,
maximum 48.790 ms** for the complete measured frame.
Their internal solve phase has median 6.164 ms and
maximum 22.768 ms. All 986 active continuation
frames use one conic solve and 0 exceed 100 ms.

Target-free continuation has frame median
4.480 ms and maximum
6.470 ms. These categories answer
the steady-runtime question without treating first admission as steady operation.

Admission remains much slower: median 655.210 ms,
maximum 4653.899 ms. Across all 9,000
measured frames, median is 4.489 ms
and maximum is 4653.899 ms. The pooled
median is dominated by target-free frames and is not the typical avoidance cost.

| Frame category | Count | Frame median / ms | Frame maximum / ms | Solve median / ms | Solve maximum / ms | Frames > 100 ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Active encounter with carried certificate | 986 | 15.816 | 48.790 | 6.164 | 22.768 | 0 |
| Target-free continuation | 7940 | 4.480 | 6.470 | 0.997 | 1.553 | 0 |
| New/returning target admission | 32 | 655.210 | 4653.899 | 377.502 | 2653.240 | 27 |
| First target-free frame of a trial | 10 | 6.077 | 7.612 | 1.131 | 1.166 | 0 |
| Release/target-set transition | 32 | 4.971 | 9.234 | 1.040 | 1.111 | 0 |
| All measured frames | 9000 | 4.489 | 4653.899 | 1.000 | 2653.240 | 27 |

## Measurement boundaries and reproducibility

- AMD Ryzen 7 7800X3D desktop, x86_64, MATLAB R2026a Update 3 CLI with
  `-singleCompThread`; profiler disabled. This is not a Raspberry Pi measurement.
- Freeze the committed controller, configuration and script tree outside the
  repository. Reuse the existing native solver/geometry binaries through a
  symlink. Verify source and all 103 native-binary hashes after timing.
- Run one separate warmup campaign: 15 scenarios, 60 holds each (900 holds).
  Then run two measured campaigns of 15 scenarios, 300 holds each (9,000 holds).
  Retain both measured repetitions; do not select the faster one or delete outliers.
- Each campaign covers curvature 0 and +/-0.01, +/-0.02 per metre, with stationary,
  oncoming and crossing targets. Reference 8 m/s, hold 0.1 s, sensing 16 m,
  seed 20260912, exact declared affine plant and exact measurements, no road
  boundaries or estimator. Inputs and measurements evolve through actual
  closed-loop simulation, rather than repeated calls to one selected fixture.
- Frame timer: synthetic input construction plus the entire controller,
  including parsing, prediction/formulation, all solves, independent
  verification, command/state packaging and return. Plant propagation and the
  offline physical audits are outside the timer.
- `runtimeBreakdown.solveSeconds` is the controller's existing solve-phase
  timer. It includes sparse transcription/bridge overhead and conic execution;
  it is not a native solver iteration-only timer. During admission it accumulates
  all subproblem solves in the frame. It excludes the separate formulation,
  input-preparation and final verification work.
- Diagnostic search/frame budgets are 30 s, while the physical hold remains
  0.1 s. The larger budget lets slow admissions finish instead of censoring
  their latency at 100 ms. This is not a new strict-deadline execution campaign.
- Category predicates are recorded per frame: active target plus inherited
  family is active continuation; active target without inherited family is
  admission; target-free frame with target-free predecessor is stable cruise.
  A trial's initial target-free frame and release transitions are separate.
  Every one of the 9,000 measured frames belongs to exactly one category.

## Straight and circular active continuations

| Reference | Count | Frame median / ms | Frame maximum / ms | Solve median / ms | Solve maximum / ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| Straight | 132 | 12.168 | 33.598 | 3.675 | 9.749 |
| Circular (+/-100 m and +/-50 m radii) | 854 | 16.934 | 48.790 | 6.856 | 22.768 |

| Measured repetition | Active frames | Frame median / ms | Frame maximum / ms |
| --- | ---: | ---: | ---: |
| 1 | 493 | 15.749 | 48.731 |
| 2 | 493 | 15.883 | 48.790 |

The longest active continuation occurs in repetition 2,
crossing target, curvature -0.02 per metre,
simulation time 0.1 s, horizon 55 holds,
with 1 conic call. The longest admission is repetition
2, crossing target, curvature
+0.02 per metre at 0.0 s,
with 124 conic calls. Admissions after time zero are online
encounter events, even though code has already been warmed.

These are observed maxima on this desktop and workload, not a proven
worst-case execution-time bound. The timing result does not repair the known
inter-node collision defect: both measured rounds reproduce the straight
stationary/oncoming physical-audit failures (26/30 trials pass the sampled
collision criterion). All 30 trials numerically complete; numerical completion
and node certification are not whole-hold collision avoidance.

## Artifacts and checks

Existing executable driver: `scripts/runJointSupportCertificateValidation.m`.
The temporary outer runner resets each scenario through that public driver and
writes per-frame timing/classification records. It changes no production code.
Equivalent campaign calls use `SampleCount=60` for warmup, then `SampleCount=300`
for each of two repetitions, always `Curvatures=[0,.01,-.01,.02,-.02]` and
separate output directories.

Artifacts are at `/home/zai/.cache/collisionAvoidance/runtime-retest-20260919/`:
source/native manifest, isolated runner, warmup traces, two measured campaigns,
`frames.csv`, `timing.mat`, `trials.json`, process log and aggregation script.
The companion report JSON includes category/road/repetition summaries and
exact locations of maxima, along with artifact hashes. Aggregation checks
9,000 classified/certified frames, 30 completed trials, one conic call for every
active continuation, timer nesting and unchanged source/native hashes.
Only report artifacts are committed; no new unit suite or implementation
change is claimed for this timing-only task.
