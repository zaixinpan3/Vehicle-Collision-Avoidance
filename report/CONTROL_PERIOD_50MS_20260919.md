# Unified 50 ms prediction and control period

September 19, 2026. The prediction step, safety-node interval and actual held
input/control-update period now default to **0.05 s** in the current experiment
drivers. Strict periodic runs use a **0.05 s complete-frame deadline**.

The period change is implemented, but **50 ms real-time avoidance is not
qualified**. With a diagnostic computation budget, all 30/30
trials finish at the node-certificate level and 26/30 pass the
sampled-clearance checks. Under the strict deadline, only 5/15 finish,
and 5/15 qualify both runtime and the scenario checks. The maximum
warmed diagnostic frame is **157.056 ms**.
The two straight-road inter-node collision cases remain.

## Implementation

`collisionAvoidanceControllerConfig` already declared `sampleTime = 0.05`.
The preceding experiments overrode it with `0.1` in the exact-state and joint
drivers. Replace those overrides with an explicit `SampleTime` option defaulting
to 0.05. The same value reaches the model discretization, prediction cells,
target node times, executed input holds, measurement timestamps and stored
certificate shift. The discrete CLF and terminal certificate are recomputed for
that period. Start with an empty controller state when changing the clock.

Preserve the experiment's **1.6 s** performance horizon using
`ceil(HorizonSeconds/SampleTime)`: 16 nodes at 100 ms become **32 nodes at 50 ms**.
Preserve **30 s** simulation runs using **600 holds**, instead of 300. The
finite encounter-exit deadline is still determined in physical time and can
require a longer horizon. The generic configuration's 16-node horizon is not
silently changed; these experiment drivers explicitly select 1.6 s.

Propagate timing through the bounded-admission, circular-arc, recursive-safety,
overlap, declared-plant joint, and smooth-reference drivers. Record deadline
misses against the selected period instead of fixed 100 ms labels. The exact
and joint single-scenario default computation deadline is also 50 ms. A caller
selecting another period should pass the corresponding `DeadlineSeconds` for
strict operation; diagnostic overrides remain explicit. The generic solver's
unlimited diagnostic default is distinct from a periodic acceptance deadline.

The controller algorithm, three-call admission budget, independent physical
node verification, CLF slack, finite terminal continuation, and actuator limits
are unchanged. No collision buffer, fallback controller, road boundary, route
enumeration, solver rebuild or estimator-mathematics change is introduced.

## Validation protocol

- Baseline: commit `9935d49ef7302f0a029487eb089259b8c1d38197`,
  [the preceding 100 ms bounded-admission study](BOUNDED_ADMISSION_20260919.md).
- AMD Ryzen 7 7800X3D desktop, MATLAB R2026a CLI, one computational thread and
  one native solver thread. This is not Raspberry Pi or ROS timing.
- Controller-only exact affine plant and exact observations, 8 m/s reference,
  16 m sensing range, zero residual/jerk/yaw acceleration, seed 20260912, no road
  boundaries or state/slip bounds. Certified chart domains remain hard.
- Curvatures `0, +0.01, -0.01, +0.02, -0.02` per metre, each with stationary,
  oncoming and crossing targets. Preserve the original physical scenario inputs,
  including 32 m/s straight crossing and 4 m/s curved crossing. These two
  crossing definitions have different speeds.
- Exclude 15 warmup trials of 120 holds (6 s each), plus 20 adapted-fixture
  warmups. Measure 30 fixture replays, two rounds of 15 trials with 600 holds,
  then 15 independently reset strict 50 ms trials. Diagnostic budgets are 30 s;
  diagnostic overruns are measured, not accepted as real-time success.
- Frame times include synthetic observations and the complete controller call;
  offline plant integration and collision audits are outside the timer. Replay
  times cover the complete controller call. No profiler or hook is used during
  the timed campaigns. Cold startup is excluded, fresh-target admission is not.
- Collision audits use 11 samples per held interval, now 5 ms apart. Negative
  signed rectangle distance is a collision; the pass criterion is strictly
  positive distance with no physical margin. Positive samples alone are not a
  continuous-time certificate.
- Verify all 90 frozen/current MATLAB source hashes and 103 unchanged native
  binary hashes, every issued hard node certificate, actual sample timestamps,
  the three-call admission cap and one-call active continuation.

## Complete-frame runtime

The two periods cover the same physical duration, so the sample counts are
9,000 at 100 ms and 18,000 at 50 ms. The last
column uses the new 50 ms requirement. These are empirical maxima, not WCET.

| Category | 100 ms median (ms) | 100 ms maximum (ms) | 50 ms median (ms) | 50 ms maximum (ms) | New frames >50 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| All frames | 4.468 | 69.822 | 7.005 | 157.056 | 30 / 18000 |
| New target admission | 32.761 | 69.822 | 66.462 | 157.056 | 18 / 32 |
| Active encounter continuation | 8.362 | 21.170 | 16.762 | 57.127 | 12 / 2004 |
| Target-free continuation | 4.459 | 6.243 | 6.995 | 11.949 | 0 / 15922 |

The new largest frame is repetition 2, crossing,
curvature -0.02 per metre, at 0.00 s.
It requires **112 holds** and **3 conic
calls**. Its formulation and solver totals are respectively
67.420 and 70.854 ms.
The complete frame also includes preparation and independent verification.

Adapting the previous +0.02 crossing fixture to 50 ms produces a
96-hold problem. All 30/30 diagnostic
replays are accepted; median/maximum complete-call times are
**93.674/106.390 ms**.
The old 100 ms result for that physical initial state was a 48-hold problem,
43.134 ms median, with two conic calls. This is a comparison of different
discretizations, not a replay of the identical optimization matrix.

An untimed conic-dimension probe records:

| Call | Variables | Rows | SOCs | Domain-screened records |
| --- | ---: | ---: | ---: | ---: |
| 1 | 1111 | 2315 | 88 | 56 |
| 2 | 1111 | 2315 | 88 | 56 |

The old 100 ms probe had 559 variables, 1179 rows, 48 SOCs and 28 screened
records per call. Keeping the terminal duration and halving the node interval
increases the optimization size while halving available wall time. The existing
three-call bound does not guarantee completion within 50 ms.

## Collision and deadline outcomes

Both diagnostic repetitions complete all 600 holds for each scenario. The
table uses the smaller body gap and larger runtime of the two repetitions.
Strict deadline failures terminate before issuing the failed frame's command.
The diagnostic result does not supply a witness to the independent strict run.

| Curvature (1/m) | Target | Old 100 ms body gap (m) | New 50 ms body gap (m) | New maximum frame (ms) | Strict 50 ms outcome |
| --- | --- | ---: | ---: | ---: | --- |
| +0.00 | stationary | -0.110561 | -0.091335 | 86.727 | Deadline failure |
| +0.00 | oncoming | -0.035467 | -0.010437 | 44.656 | Deadline failure |
| +0.00 | crossing | 9.519705 | 9.519696 | 19.056 | Pass |
| +0.01 | stationary | 0.068748 | 0.062209 | 66.382 | Deadline failure |
| +0.01 | oncoming | 0.075915 | 0.099463 | 37.922 | Pass |
| +0.01 | crossing | 0.061342 | 0.100701 | 95.530 | Deadline failure |
| -0.01 | stationary | 0.068748 | 0.062209 | 67.817 | Deadline failure |
| -0.01 | oncoming | 0.075915 | 0.099463 | 37.614 | Pass |
| -0.01 | crossing | -0.024611 | 0.073971 | 127.519 | Deadline failure |
| +0.02 | stationary | 0.199738 | 0.168234 | 69.395 | Deadline failure |
| +0.02 | oncoming | 0.187732 | 0.202312 | 38.180 | Pass |
| +0.02 | crossing | 0.196089 | 0.209323 | 96.447 | Deadline failure |
| -0.02 | stationary | 0.199738 | 0.168234 | 70.537 | Deadline failure |
| -0.02 | oncoming | 0.187732 | 0.202312 | 37.979 | Pass |
| -0.02 | crossing | -0.027341 | 0.128861 | 157.056 | Deadline failure |

Remaining sampled collisions:

- Curvature +0.00, stationary: minimum signed body gap **-0.091335487 m**.
- Curvature +0.00, oncoming: minimum signed body gap **-0.010436860 m**.

The two negative-curvature crossing cases that collided at 100 ms are clear
at the audit samples at 50 ms. Thus this campaign improves sampled clearance
from 11/15 to 13/15 distinct scenario types, but does not
establish whole-hold safety. The straight stationary and oncoming overlaps
prevent a claim of complete avoidance. Strict runs additionally expose the
new admission deadline limitation: 10 of 15 stop before finishing.
These deadline-limited failures do not show that the corresponding finite
trajectory problem is mathematically infeasible; diagnostic runs find plans.
The straight-oncoming diagnostic maximum is 44.656 ms, but its independent
strict admission at 2.60 s takes 51.714 ms and is rejected. The two runs are
separate wall-time observations; a diagnostic maximum below the deadline does
not establish that a subsequent deadline-limited solve will complete in time.

## Checks, scope and reproduction

The 67 relevant regression cases pass after updating one stale test expectation
from 120 to 240 calls for the unchanged 12-second recovery test. The original
run had 66 passes and that one expectation mismatch; only the corrected test
was rerun. Test logs and MAT results preserve both outcomes. Coverage includes
50/100 ms node flows, physical input-hold duration, target prediction times,
one-period witness shifting, curved trim, no-road behavior, failure termination,
and the joint driver's exact-state clock. Tests run through MATLAB MCP.

After timing, smoke-test circular, smooth, recursive and overlap wrappers at
50 ms and inspect their metadata. These short interface checks are not separate
runtime qualifications or full estimator avoidance tests. Factory-configured
`checkcode` examines 13 changed MATLAB files and reports 0
advisories; exact messages are in the JSON. `git diff --check` passes. No full
repository-suite rerun is claimed for this experiment-driver change.

```matlab
addpath('scripts');
runBoundedAdmissionBenchmark( ...
    OutputDirectory='/absolute/path/to/50ms-results', ...
    SampleTime=0.05, HorizonSeconds=1.6, SampleCount=600);
```

Use `ReplayFixture='/home/zai/.cache/collisionAvoidance/admission-hotspot-20260919/fixture.mat'`
to reproduce the adapted physical-initial-state comparison. For the old 100 ms
clock with the same physical durations, select `SampleTime=0.1`,
`HorizonSeconds=1.6`, `SampleCount=300`.

Raw results, frozen sources, manifests and logs are under
`/home/zai/.cache/collisionAvoidance/control-period-50ms-20260919`. The accompanying
[JSON report](CONTROL_PERIOD_50MS_20260919.json) includes every scenario,
runtime groups, strict failures, tests, probe and source/binary hashes. Reports
are in `report/`; executable drivers remain in `scripts/`. Unrelated files,
the manuscript, native dependencies and generated binaries are excluded.
