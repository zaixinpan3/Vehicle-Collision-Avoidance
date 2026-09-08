# Estimator and controller pipeline runtime attribution

Analysis date: September 8, 2026. Algorithm baseline:
`d0c08bd4dc7826d691528e138de272fffc561ab5`.

## Findings

The dominant recurring cost is constructing the controller optimization
problem, including swept geometry, CLF inequalities and the sparse conic
transcription. Numerical solving becomes significant at target acquisition
and repeated nonlinear refinement, but is a smaller part of average frame
time. The observer is the second largest component. Its MATLAB enclosure,
measurement and domain-audit work dominates the profiled call structure;
the high-gain RK4 point update already uses a native kernel.

The 238.432 ms delayed physical frame is a separate, unreproduced timing
spike. Its state and command reproduce exactly outside the physical
simulation harness, with much shorter computation. That does not erase
the failed deadline or establish its cause.

No controller, observer, gain, constraint or runtime configuration is changed
by this analysis. The new script reads results and runs component replays;
it applies no actuator commands and starts no new joint closed loop.

## Historical joint measurements

The last complete joint record contains 600 frames over 30 s, with a
50 ms update period, 32 prediction stages and one certified interval. Its
instantaneous simulated actuation and 100 ms threshold are not a valid
50 ms scheduling certificate. The timing correction in
[the delayed-execution study](STRAIGHT_DELAYED_EXECUTION_RESULTS.md) remains
in force. The record is useful for attributing measured work.

| Component | Mean per frame (ms) | Share of summed complete-frame time |
| --- | ---: | ---: |
| Controller, total | 25.055 | 75.88% |
| Observer/synthetic sensor adapter | 6.031 | 18.27% |
| Road fitting | 1.882 | 5.70% |
| Other frame work | 0.051 | 0.16% |
| Complete frame | 33.020 | 100% |

The following rows split the controller total; they must not be added to
the controller-total row above.

| Controller phase | Mean per frame (ms) | Share of complete-frame time |
| --- | ---: | ---: |
| Problem formulation and planning-window work | **14.214** | **43.05%** |
| Initial model prediction | 4.761 | 14.42% |
| Numerical solve stage | 2.132 | 6.46% |
| Input preparation and prior-certificate admission | 1.809 | 5.48% |
| Acceptance and result construction | 1.645 | 4.98% |
| Other controller work | 0.494 | 1.50% |

The solve-stage timer includes solver wrappers and CLF-slack reconstruction,
not just native factorization. The initial-prediction timer excludes some
candidate-specific prediction work; the residual category retains time
not allocated by the existing internal timers. Every frame has a
nonnegative residual category, within arithmetic tolerance. Shares use
summed times, not ratios of independently selected maxima or medians.

The slowest joint frame occurs at 3.55 s, when the target is acquired:
94.244 ms total, 12.981 ms observer, 1.977 ms road fitting and 79.236 ms
controller. Its formulation stage takes 30.243 ms and solving takes
23.974 ms, with four numerical calls and one nonlinear refinement. Thus
low average solver cost does not imply that repeated solves are irrelevant
to the deadline.

## Current delayed controller

The current strict profile uses a 100 ms period, 16 prediction stages, one
scheduled period of input delay and two certified held intervals. The
recorded pure physical experiment stops at 1.7 s on frame 18. Its 238.432 ms
frame includes 236.360 ms controller and 1.984 ms road fitting. The target
is absent, and the controller performs **one solve and zero refinements**.
Therefore repeated obstacle solves cannot explain this particular spike.

The recorded controller phase times are 99.779 ms prediction, 70.500 ms
formulation, 58.213 ms acceptance and 5.091 ms solving. These identify where
elapsed time accumulated, not why it accumulated there.

Current code replays all 18 recorded input frames, including the expired
candidate, with **zero actuator-input difference**. Twenty additional
unprofiled evaluations of the same peak-frame input and prior certificate
produce:

| Measurement | Controller time (ms) |
| --- | ---: |
| Original physical-harness frame | 236.360 |
| Identical-input replay minimum | 28.496 |
| Identical-input replay median | 30.419 |
| Identical-input replay maximum | 41.863 |

The replay component medians are 16.332 ms formulation, 5.200 ms solving,
4.411 ms initial prediction, 1.925 ms acceptance and 1.812 ms input
preparation. Component medians need not sum to the frame median. The original
failure remains a failure. The replay excludes the surrounding Simulink
plant execution and does not isolate JIT, allocation, cache or operating-system
effects sufficiently to assign the spike to any of them.

Current function profiles point to
`avoidanceStageQp/localLiftedProgram`, `formulateAvoidanceProblem`,
`avoidanceSafetyGeometry`, and `ltvBicycleModel.finitePredict`.
The examined target-free frame constructs:

| Representation | Size |
| --- | ---: |
| Physical input plan | 32 scalar input coordinates |
| Prediction cells | 28, including 14 robust cells |
| Original CLF quadratic inequalities | 112 |
| Original hard inequalities | **6,192** |
| Hard inequalities retained for numerical solving | **288** |
| CLF cones retained for numerical solving | 57 |
| Variables before / after inactive-slack and committed-input elimination | 222 / 206 |

Row screening makes the numerical problem much smaller, but the front end
has already constructed the full expressions. Independent certification
continues to evaluate the original constraints. Sparse transcription also
builds auxiliary cell-state dynamics and cone representations. This explains
why further acceleration must address formulation and representation work,
not only the external conic solver.

The already committed first input is eliminated near the native solve.
Its fixed value is known before prediction/formulation. Moving applicable
constant propagation earlier is an identifiable optimization candidate,
provided both held-interval certificates and independent checks are retained.
No speedup from that candidate is claimed here.

## Current observer at a 100 ms output cadence

An observer-only replay uses the historical physical ego states as prescribed
inputs, linearly sampled every 100 ms from 0 through 8 s. It retains the
same synthetic target trajectory, seed 20260907, sensor/noise configuration,
observer gains and 12.5 ms integration step used by the strict profile.
Ordinary frames process eight 80 Hz sensor intervals. These prescribed
states are not trajectories generated by the delayed controller.

| Observer measurement | Time (ms) |
| --- | ---: |
| Median across 81 output frames | 9.915 |
| Median while a target is published | 10.597 |
| Median without target publication | 9.596 |
| First output at 0 s | 45.073 |
| First target publication at 3.6 s | 39.845 |
| Largest later target-tracking frame, at 3.9 s | 16.247 |

The exact sensor random-stream state is copied for each profiled sample;
all profiled estimates, target publications and audits match the unprofiled
outputs exactly. Hotspots include `nrmmPositionErrorBound`, `nrmmYawSet`,
`nrmmControllerErrorBounds`, `nrmmTargetHistory`, operating-domain audits
and `certifiedKinematicCourseCorrespondence`. The profiled tracking frame
makes 16 operating-domain audit calls, 40 yaw-set helper calls and eight
native RK4 calls. Those helper counts are internal operations, not extra
sensor frames.

The native RK4 and target-history kernels fall below the profiler's displayed
time resolution in these individual samples. A displayed zero is not zero
execution cost. Reducing repeated enclosure/audit data handling is a more
direct candidate than redesigning the high-gain observer dynamics.

Do not add this observer replay to the pure-controller replay and call the
sum a current joint result: estimator uncertainty changes controller
constraints and can change its maneuver/refinement workload. The current
delayed joint closed loop has not passed the pure-controller gate.

## Measurement and optimization boundaries

The profiler expands the examined controller call to approximately 281 ms,
versus a 30.419 ms unprofiled replay median. Profiler values identify functions
and call patterns; they are not deadline measurements and must not be summed
across parent and child functions. Analysis-wrapper rows are excluded when
ranking pipeline hotspots, but retained in raw exports.

The evidence supports this order of investigation:

1. Reduce repeated constraint/cone construction, intermediate arrays and
   fixed-prefix work while preserving the full mathematical checks.
2. Reduce unnecessary rebuild/refinement work at target acquisition without
   accepting nonlinear violations or substituting stored controls after failure.
3. Consolidate repeated observer enclosure/audit operations while preserving
   the high-gain state equations, measurement set intersections and error bounds.
4. Instrument the full physical-harness execution to determine the cause of
   unreproduced spikes. Component replays do not establish a platform WCET.

Road fitting is a lower average-cost component in the recorded synthetic
pipeline. Actual YOLO/LiDAR inference and actuator/network transport are
absent from these measurements. The ranking does not extend to those
unmeasured components.

## Reproduction

```matlab
addpath('scripts');
report = analyzePipelineRuntime(historicalJointResultPath, ...
    delayedControllerResultPath, outputDirectory);
```

Both inputs are `straight-realtime-validation.mat` files containing `report`.
Original analysis outputs are in
`~/.cache/collisionAvoidance/pipeline-profile-20260908/`: frame-level CSVs,
complete JSON summaries, original profiler tables, saved replay contexts,
exact-output checks and the captured controller problem. The historical
inputs remain in the earlier EV-0078 and EV-0079 experiment bundles.
The executed analysis completes successfully. Factory Code Analyzer reports
one unused intermediate replay-output warning; no algorithm test suite is
rerun because the estimator/controller implementation is unchanged.
