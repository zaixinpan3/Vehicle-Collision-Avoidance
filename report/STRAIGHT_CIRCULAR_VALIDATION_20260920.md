# Straight and circular controller simulation — September 20, 2026

The current controller does **not** satisfy strict collision avoidance and the
50 ms deadline across all tested encounters. Circular stationary and oncoming
cases avoid sampled body overlap and recover cruising. Straight stationary and
oncoming cases exhibit small inter-node overlaps; circular crossing fails initial
admission. All 544 measured active continuation frames fit within 50 ms, but
four first-admission frames exceed that period, with a maximum of **108.710 ms**.

This is a fresh run of project commit
`5104a8247cad7da796f2d3831c1325ddef6d1df6`. Production source and configuration
are unchanged. The [JSON summary](STRAIGHT_CIRCULAR_VALIDATION_20260920.json)
contains per-trial results, timing categories, independent audits and artifact
hashes. The experiment completed on September 20 in America/Chicago.

## Experiment conditions

- MATLAB R2026a on an AMD Ryzen 7 7800X3D desktop, one computation thread.
  The MATLAB controller uses its existing native geometry and Clarabel MEX
  dependencies; these are complete controller-frame measurements, not isolated
  standalone C solver timings.
- Straight road and analytic left circular reference of radius 100 m; each
  tested with stationary, oncoming and crossing targets. Reference speed is
  8 m/s, confirmation range 16 m, vehicle footprint 4.8 by 1.9 m, seed 20260912.
- Exact state sensing, zero declared state-error boxes, the declared held affine
  bicycle plant, no enforced road edges, and zero fixed clearance buffer.
  Hold, prediction-node and update periods are all 50 ms. The nominal horizon
  is 1.6 s; encounter completion can extend it, reaching 96 stages in the
  first straight stationary plan.
- Each case receives an excluded 120-hold warmup, followed by two measured
  trials requesting 600 holds (30 simulated seconds). Five cases execute all
  600 holds in each trial; circular crossing stops before issuing a command.
  All 6,002 attempted measured frames, including rejected admission, are counted.
- Computational budgets are 30 s to allow diagnostic execution. Deadline misses
  are independently counted against the actual 50 ms control period. Timings
  include synthetic observation construction and the entire controller call;
  plant integration, offline geometry checks and file output are excluded.
  Computation delays are measured but are not injected as actuator delays.

## Avoidance results

The two repetitions have exactly identical states, controls, completion outcomes
and sampled minimum gaps. The table uses the finer offline signed body gap:
positive means separated footprints; negative means overlap. Finite sampling
does not prove continuous-time collision freedom.

| Road / target | Executed holds per trial | Refined minimum body gap, m | Avoidance outcome | Maximum full frame, ms | Frames above 50 ms / attempts |
| --- | ---: | ---: | --- | ---: | ---: |
| Straight / stationary | 600 | -0.000551406 | Inter-node overlap | 108.710 | 2 / 1200 |
| Straight / oncoming | 600 | -0.005442167 | Inter-node overlap | 28.394 | 0 / 1200 |
| Straight / crossing | 600 | 9.519501 | Clear, but no nominal collision threat | 17.473 | 0 / 1200 |
| Circular / stationary | 600 | 0.082579984 | Sampled avoidance and recovery pass | 52.966 | 2 / 1200 |
| Circular / oncoming | 600 | 0.089778364 | Sampled avoidance and recovery pass; see model-domain caveat | 27.358 | 0 / 1200 |
| Circular / crossing | 0 | Not evaluated | Initial admission rejected | 36.861 | 0 / 2 |

Straight stationary overlap reaches approximately **0.5514 mm at 1.6849 s**;
straight oncoming reaches **5.4422 mm at 3.7319 s**. Their node-only minimum
gaps remain positive, respectively 0.07281 mm and 0.06160 mm. Thus every issued
command can satisfy its node certificate while the actual interpolated held
trajectory overlaps between nodes. Neither successful optimization nor completion
of 600 holds is sufficient evidence of collision avoidance.

Circular crossing fails at time zero with
`collisionAvoidanceController:optimizationFailed` and the diagnostic
`Scalar admission found no hard-certified plan: collisionExcluded`. It issues
no command. This is failure of the restricted admission family, not a proof
that physical avoidance is impossible. Gap and recovery fields for this case
are not applicable; the driver's empty-trajectory collision-free flag is not
interpreted as success.

The straight crossing target moves at 32 m/s laterally from (15, -4) m. A separate
unchanged-cruise counterfactual also stays clear, with minimum separating-axis
margin 7.85 m. This case therefore does **not** establish meaningful evasive
capability. The circular crossing target instead moves at 4 m/s from an initial
7.5 m normal offset at road station 15 m, and its cruise counterfactual overlaps.
Cruise counterfactuals also overlap in both stationary and both oncoming cases.
Across the five distinct cases that actually require avoidance, only the two
circular stationary/oncoming cases maintain sampled separation; this is a
deterministic scenario finding, not an estimated statistical success rate.

All five completed runs release the encounter and recover cruise. At 30 s,
the maximum absolute lateral tracking error is 0.00001090 m and the maximum
speed error is 0.000000225 m/s. Recorded relaxed CLF residuals are negative,
and actuator slew violations are zero. Recovery does not undo earlier overlap.

Model-domain caution: the optional heading-error diagnostic limit is 0.4 rad.
The saved oncoming node trajectories reach absolute errors of 0.4590 rad
(straight) and 0.4439 rad (circular), already outside that limit. The existing
controller does not enforce this state-domain diagnostic. The circular oncoming
result is therefore a sampled avoidance result for the declared affine plant,
without evidence that the nonlinear vehicle remains accurately represented.

## Runtime

| Frame category | Attempts | Median, ms | P95, ms | Maximum, ms | Above 50 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| First target admission, including rejection | 12 | 31.134 | 91.454 | 108.710 | 4 |
| Active-target continuation | 544 | 14.3545 | 30.467 | 42.173 | 0 |
| No active target | 5446 | 7.2525 | 8.128 | 12.759 | 0 |
| All attempted frames | 6002 | 7.270 | 12.751 | 108.710 | 4 |

The overall median is dominated by target-free cruising. The four misses all
occur on initial stationary-target admission: straight 108.710 / 77.335 ms,
circular 52.966 / 50.272 ms across the two runs. These first-admission frames
remain included after warmup. Oncoming admission occurs at 2.6 s, after initial
target-free motion. The first straight stationary invocation in the excluded
warmup takes 915.469 ms; cold initialization must also be considered in deployment.

The worst measured frame is straight stationary frame 1: input preparation
8.346 ms, combined formulation/admission 87.051 ms, zero conic-solver time,
and 13.313 ms remaining frame work. These are the saved phase timers and their
residual, not a newly instrumented attribution. The worst continuation is its
frame 2: preparation 2.875 ms, formulation 18.997 ms, solve 13.084 ms and
remaining work 7.217 ms, totaling 42.173 ms.

Sustained continuation fits 50 ms in this experiment. Complete 20 Hz operation
still fails the measured deadline requirement at admission. Observed maxima
on a desktop do not establish a worst-case execution bound or performance on
a Raspberry Pi. No standalone C benchmark is claimed in this task.

## Independent validation and artifacts

The offline audit reconstructs all executed affine endpoints with zero maximum
discrepancy. It recomputes the original 5 ms body-gap samples and checks every
overlap sign against the independent `scripts/rectangleSeparationMargin.m`
separating-axis implementation. Holds with SAT margin below 0.25 m, the worst
SAT/body-gap holds, and neighboring holds are resampled every 0.1 ms. The two
geometry implementations agree on every sampled collision/noncollision sign.
Positive SAT margins and Euclidean body gaps can differ; for example the refined
circular stationary SAT margin is 0.0660365 m, while its minimum body gap is
0.0825800 m. These quantities are not substituted for one another.

All 6,000 issued commands retain their hard certificate; the other two attempts
reject admission. Source/dependency hashes are unchanged. Aggregation validates
frame counts, deadline counts, exact repetition equivalence, refined/coarse
minimum consistency and applicable versus unexecuted outcomes. No production
code is changed, and no new unit-test-suite result is claimed. Python analysis,
numerical assertions, artifact integrity and `git diff --check` pass.

Raw MAT/JSON traces, logs, provenance, the independent audit, aggregation code,
and the inspected plot/PDF are retained outside the repository:
`/home/zai/.cache/collisionAvoidance/straight-circular-validation-20260920-2229/`.
`safety-runtime.png` and `safety-runtime.pdf` show close-passing geometry and
the first six seconds of both timing runs. Generated figures, raw traces,
external solver trees and unrelated user files are excluded from the commit.

Reproduce the campaign in a new external output directory:

```bash
matlab -singleCompThread -batch "addpath('scripts'); profileMatlabControllerRuntime('/absolute/new/output',Mode='campaign');"
```

The retained `auditCampaign.m` accepts that directory and runs after timing.
The retained `summarizeCampaign.py` records the exact source manifest, aggregation
and plot method used for this report. Only this report, its compact JSON and
the report index are committed. No estimator-in-the-loop, nonlinear plant,
road-edge, sensor-delay, target-hardware or delayed-actuation validation is
claimed. The immediate technical priorities remain inter-node separation,
circular-crossing admission capability and first-admission latency.
