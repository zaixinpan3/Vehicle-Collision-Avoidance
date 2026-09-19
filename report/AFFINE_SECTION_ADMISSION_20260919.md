# Affine-section admission: implementation and measured tradeoff

September 19, 2026. Baseline executable behavior: commit
`5f0a8267c7b5d0a09bb8d7e01da2681268ed8369`; task parent
`4b41456d3de23a83519060c6117d430111212d1f` adds the preceding design review.

**Implemented:** fresh admission can now return an independently verified plan
from one signed affine control section without a conic solve. The admitted
nominal and inherited continuation still use at most one hard performance SOCP.
The restoration solver and its iteration settings are removed.

**Measured outcome:** admission is faster on the eleven scenario types both
versions admit, but the new restriction rejects all four curved crossing types.
Continuation latency increases in this campaign. Strict 50 ms qualification
improves from 5/15 to **7/15**, while diagnostic completion falls from 15/15 to
**11/15**. Two straight-road inter-node collisions remain. This is a bounded
admission implementation with a documented capability tradeoff, **not a completed
real-time collision-avoidance solution**.

## Algorithm and retained guarantees

The derivation is in [Joint trajectory and separation certificates](../controller/JOINT_SUPPORT_CERTIFICATES.md).
The [preceding review](AFFINE_SECTION_ADMISSION_REVIEW_20260919.md) states the
fixed-geometry and certificate-inheritance requirements.

1. Retain the cruise or shifted nominal sequence `U0`. Select the target with
   the largest violated nominal collision certificate. Use its first, middle
   and last violating stages and nominal relative motion to construct one
   minimum-energy input deformation `v`, including terminal state/input
   correction. With clear nominal geometry but an invalid terminal witness,
   construct only the terminal correction. The input metric uses existing
   actuator weights and input differences; dynamics are not replaced by
   independent geometric point motions.
2. Restrict fresh search to `U0 + alpha*v`, allowing both amplitude signs.
   Intersect all hard actuator, slew, chart and terminal constraints with this
   line. Fixed-radius modal cones reduce analytically to scalar intervals;
   negative radii are rejected before squaring.
3. Split the permitted amplitude interval into **16** uniform geometry cells.
   Each cell has a fixed yaw enclosure valid over its whole amplitude range,
   including the original uncertainty. Evaluate **32** uniform support
   directions. Each node/target or terminal-exit condition contributes an open
   forbidden scalar interval. Sort and subtract these intervals; preserve
   singleton safe endpoints and verify floating-point candidates independently.
4. Restrict the original quadratic objective to the line. Eliminate only the
   soft CLF slack analytically and use **32** derivative-bisection iterations.
   Project that minimizer into every remaining component and select its lowest
   cost. This searches scalar components, not a catalog of left/right routes or
   combinations of nodewise normal assignments.
5. Re-evaluate every original hard row, terminal cone, CLF cone and exact
   occupied-set support residual. Only a complete accepted certificate can
   issue a command. Inherited stages, occupied sets, charts, exit deadline and
   terminal witness remain available next frame; one ordinary joint-support
   improvement may replace the verified incumbent. Search failure or a missed
   complete-frame deadline issues no command.

The one-dimensional control shape is a deliberate restriction of steering,
braking and timing freedom. The support dictionary and yaw envelopes introduce
additional conservatism. `collisionExcluded` means no certificate was found in
that declared family; it does not prove physical collision is unavoidable or
that the unrestricted problem is infeasible.

The prediction/CLF/terminal framework, input operating point, actuator bounds,
conditional carried-witness feasibility and observer remain unchanged. Physical
road boundaries and separate state/slip limits remain disabled in these trials;
certified pose-chart domains remain hard. No physical collision clearance
buffer or executable backup controller is added. Stored certificate format is
**40**; reset older saved states. The core remains within **20 source files**.

This first implementation still builds complete affine sensitivity maps and the
canonical condensed objective for compatibility with the inherited family.
Scalar rollouts and their objective are accumulated directly, but total
preparation is not linear-time and is not yet a hardware WCET bound. Domain,
node, target and configured dictionary sizes also govern work.

## Validation protocol

- Same 50 ms hold/node/update period, 1.6 s performance horizon (32 stages),
  30 s intended duration (600 holds), and scenario definitions as the
  [preceding 50 ms study](CONTROL_PERIOD_50MS_20260919.md).
- AMD Ryzen 7 7800X3D desktop, MATLAB R2026a CLI with one computational/native
  solver thread. No Raspberry Pi, ROS or estimator-in-the-loop qualification.
- Exact declared affine plant and observations; reference speed 8 m/s; sensing
  range 16 m; zero process residual, target jerk and yaw acceleration; seed
  20260912. Curvatures `0, ±0.01, ±0.02` per metre, each with stationary,
  oncoming and crossing targets. Existing straight crossing is 32 m/s and
  curved crossing is 4 m/s; these are different physical scenario definitions.
- Warm up all fifteen types for up to 120 holds, then the saved positive-0.02
  crossing fixture twenty times. Measure thirty fixture replays, two complete
  fifteen-scenario campaigns, then fifteen independently reset strict 50 ms
  trials. Failed admission ends the scenario immediately.
- Diagnostic search/frame budgets are 30 s, so overruns can be measured. Strict
  trials impose 50 ms. Frame measurements include synthetic observations and
  the whole controller call; plant integration and the offline collision audit
  are excluded. No profiler or solver hook is active in the timed campaign.
- Audit rectangle gaps at eleven points per hold, spaced 5 ms apart. A strictly
  positive signed body gap is required. Positive audit samples do not prove
  continuous-time safety; a negative sample does establish a collision in the
  simulated trajectory.
- Freeze and verify 90 MATLAB source files and 103 unchanged native binary
  hashes. Independently inspect every issued node certificate, 50 ms timestamps
  and the one-conic-call cap. Source/binary inventories and raw traces remain
  outside the repository; no solver dependency or generated binary is committed.

## Runtime comparison

The matched comparison below includes the eleven scenario types both versions
admit, with two runs each. Continued trajectories and encounter durations differ,
so continuation timing is a scenario-matched measurement, not an identical-frame
microbenchmark.

| Category | Previous median (ms) | Previous max (ms) | New median (ms) | New max (ms) | Frames >50 ms, previous → new |
| --- | ---: | ---: | ---: | ---: | ---: |
| Accepted admission | 44.564 | 86.727 | 31.276 | 55.189 | 10/22 → 3/22 |
| Active continuation | 16.816 | 57.127 | 19.253 | 72.416 | 12/1308 → 39/1320 |

Accepted-admission median drops by **29.8%**, and its maximum by
**36.4%**. Twenty stationary/oncoming admissions use zero conic calls. The two
straight-crossing admissions already have a certified nominal and use one
performance solve. All 1,320 active continuations use one conic call; every
attempted improvement is independently accepted in this campaign.

For the whole new diagnostic campaign:

| Category | Attempted frames | Median (ms) | Maximum (ms) | Frames >50 ms |
| --- | ---: | ---: | ---: | ---: |
| All attempted frames | 13208 | 7.968 | 72.416 | 42 |
| Accepted new-target admission | 22 | 31.276 | 55.189 | 3 |
| Active continuation | 1320 | 19.253 | 72.416 | 39 |
| Rejected new admission | 8 | 36.288 | 46.712 | 0 |
| Target-free continuation | 11826 | 7.912 | 14.808 | 0 |

The previous all-frame median/maximum were 7.005/157.056 ms over 18,000 frames.
The new 7.968/72.416 ms figures include only 13,200 executed holds and eight
rejected attempts because four scenario types terminate immediately in both
rounds. The smaller maximum must not be presented as an unchanged-capability
speedup; the overall median and continuation overruns do not improve.

The new largest frame is straight stationary continuation at **0.15 s**, first
round: **93** remaining stages, one conic call, **50.902 ms** formulation,
**11.624 ms** solving, and **72.416 ms** total. The largest accepted admission is
straight stationary at 0 s: 96 stages, zero conic calls, 42.538 ms formulation,
55.189 ms total. Removing the admission solver exposes the remaining preparation
and continuation costs; it does not remove them.

The old positive-0.02 crossing fixture is now rejected in all **30/30** replays
with `collisionExcluded`. Rejection median/maximum is **36.032/51.162 ms**;
one replay exceeds 50 ms. Previously all thirty copies were accepted at
93.674/106.390 ms. A fast rejection is explicitly not counted as a faster
successful admission. The negative-0.02, 112-stage former worst scenario is also
rejected in both complete campaigns.

## Avoidance, convergence and strict deadlines

Both diagnostic rounds have the same qualitative outcome: **11/15** complete,
**9/15** pass sampled collision checks. Every issued plan passes the independent
node certificate. All completed trials return near the cruise reference by
30 s: maximum absolute final lateral error is
1.43362717e-05 m and speed error is
1.13265176e-06 m/s.
This convergence does not excuse an earlier collision.

| Target | Curvature (1/m) | Diagnostic completion | Minimum sampled body gap (m) | Strict 50 ms qualified |
| --- | ---: | --- | ---: | --- |
| stationary | +0.00 | Yes | -0.000551365 | No |
| oncoming | +0.00 | Yes | -0.005382492 | No |
| crossing | +0.00 | Yes | 9.519695532 | Yes |
| stationary | +0.01 | Yes | 0.082600557 | No |
| oncoming | +0.01 | Yes | 0.089781627 | Yes |
| crossing | +0.01 | No | Not executed | No |
| stationary | -0.01 | Yes | 0.082600557 | No |
| oncoming | -0.01 | Yes | 0.089781627 | Yes |
| crossing | -0.01 | No | Not executed | No |
| stationary | +0.02 | Yes | 0.163251009 | Yes |
| oncoming | +0.02 | Yes | 0.193389662 | Yes |
| crossing | +0.02 | No | Not executed | No |
| stationary | -0.02 | Yes | 0.163251009 | Yes |
| oncoming | -0.02 | Yes | 0.193389662 | Yes |
| crossing | -0.02 | No | Not executed | No |

All four curved crossing failures report `collisionExcluded` at time zero.
The former controller found node-certified plans for all four with its broader
search. Therefore the restriction loses admission capability in these particular
fixtures. An exploratory feedback-tail section did not recover them and was not
added as another search branch. No claim is made that every possible scalar
section would fail.

The straight stationary and oncoming sampled gaps are respectively
**−0.000551365 m** and **−0.005382492 m**. These are collisions even though their
magnitudes are small. Positive gaps at the certified nodes do not imply positive
gaps throughout the hold. The admission redesign preserves, rather than repairs,
this limitation of the node-only safety specification.

Strict trials qualify 7/15 types. Four curved crossings fail admission; straight
stationary/oncoming and the two radius-100 m stationary cases fail the frame
budget. Some diagnostic trajectories meet the budget while independently reset
strict runs do not. This is measured runtime variability, not proof of physical
infeasibility. No late or uncertified command is substituted at rejection.

## Regression checks and artifacts

The final full repository suite passes **722/722** cases, with 0 failures and 0 incomplete cases.
The initial full run found two regressions: the added class exceeded the source
budget, and a clear-target perturbed cruise seed needed terminal-error admission.
Both were repaired by merging scalar logic into `solveHardCbfClf` and adding the
terminal-correction direction. The affected six test files passed 80/80 cases,
and the newly added deadline case passed before the final complete suite. Interval tests cover impossible constant
rows, negative cone radii, tangent singletons, open-interval endpoints, complete
exclusion, yaw support, zero-conic admission and failure without command issuance.

Factory-configured static analysis covers the controller MATLAB files, changed
configuration/drivers and affected test classes: 20 MATLAB files. Eight existing
advisories remain (three in `avoidanceStageQp`, five in `hardEncounterBarrier`);
the scalar solver has none. The JSON result lists each advisory. Diff checks pass. No native
binary rebuild, network change or unrelated manuscript/perception edit is included.

- [Machine-readable report](AFFINE_SECTION_ADMISSION_20260919.json)
- Main code: [solveHardCbfClf](../controller/solveHardCbfClf.m),
  [configuration](../config/collisionAvoidanceControllerConfig.m),
  [scalar-admission tests](../tests/affinePlanAdmissionTest.m)
- Reusable driver: [runBoundedAdmissionBenchmark](../scripts/runBoundedAdmissionBenchmark.m)
- Raw results: `/home/zai/.cache/collisionAvoidance/affine-section-20260919/`;
  `campaign/frames.csv`, `campaign/benchmark.json`, frozen sources, manifests,
  exploratory studies, full-suite logs and matched comparisons.

The next unresolved work is separate: reduce the full continuation preparation
cost, improve a single section's ability to represent curved crossings without
unbounded search, and establish interval safety if noncollision is required
between nodes. This change does not claim those tasks are solved.
