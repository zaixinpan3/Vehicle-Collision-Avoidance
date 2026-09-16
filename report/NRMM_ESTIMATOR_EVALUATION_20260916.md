# NRMM estimator simulation evaluation — September 16, 2026

The current estimator tracks relative position accurately with uninterrupted
synthetic measurements, but target derivative noise, startup peaking and radar
reacquisition remain material limitations. Forty-one 12 s trials complete with
finite states; all 165 estimator-related tests pass. This evaluation changes no
observer code, gains, defaults or controller behavior.

Tested source revision: `f03d5dcd96b2d0d4bb717375d5266a28ce6c71a6`.
MATLAB R2026a Update 3 (`26.1.0.3276743`), Linux x86-64, AMD Ryzen 7 7800X3D.
The existing observer RK4 and target-history MEX kernels were available and
used; these generated dependencies are excluded from this report commit.

## Experiment and metric definitions

The existing `runOnlineNrmmTrackingErrorBenchmark` executes one noise-free trial
and ten trials for each of four noisy cases, using seeds 7–16. Each trial lasts
12 s with one tracked target. The default sample period is 0.02 s (50 Hz), the
RK4 step limit is 0.005 s, and the target-history duration is 2 s. The 25 Hz case
uses a 0.04 s sample period with the same integration-step limit.

GNSS position, GNSS velocity, IMU acceleration, gyro and radar have deterministic
error bounds of 0.12 m, 0.02 m/s, 0.05 m/s², 0.002 rad/s and 0.12 m,
respectively. Noise is independent bounded uniform noise; each planar component
is scaled by `1/sqrt(2)` so the vector error respects the configured norm bound.
No additional bias, delay, asynchronous sampling, association error or physical
sensor pipeline is simulated.

The ego speed is `12 + 1.2*sin(0.25*t)` m/s and its yaw rate is
`0.12*sin(0.35*t)` rad/s, with a consistent kinematic single-track truth. The
nominal target starts at `[25, 4]` m with speed 12.5 m/s, heading 0.10 rad,
sideslip 0.0064 rad, scalar acceleration 0.08 m/s² and rear-axle distance 1.6 m.
It follows the retained constant-scalar-acceleration/constant-curvature model.

In the changing-motion case, a smooth transition centered at 6 s increases
scalar acceleration from 0.08 to 0.68 m/s² and reduces curvature by 0.007 m⁻¹.
That case declares acceleration-rate and curvature-rate bounds of 1.5 m/s³ and
0.0175 m⁻¹s⁻¹ and increases the relative-position domain from 50 to 55 m.
The synthesis therefore selects a different bandwidth (6.1147 versus
6.4431 s⁻¹); its lower errors do not establish that harder motion is intrinsically
easier to estimate. The dropout case omits radar frames on `[4, 5)` s while
retaining the other sensors.

Initial offsets are ego position `[1, -0.8]` m, ego yaw 0.03 rad, initial
ego-body-velocity offset `[0.5, -0.3]` m/s relative to `[V_E(0), 0]`, and target
transformed-state offset `[1, -0.5, 0.5, 0.5, 0.2, -0.2]` in position,
velocity and acceleration units. These are the existing scenario's offsets.

At time `t`, a measurement is assimilated and the observer propagates to
`t + Ts`; the estimate is compared to truth at that advanced time. RMSE is
`sqrt(mean(sum(error.^2, 2)))` on samples with `t >= 2 s`. Target position is
relative position in the true ego body frame; velocity is inertial target
velocity; acceleration is the transformed target acceleration in the true ego
body frame. Heading errors are wrapped. Startup peaks separately cover 0–2 s.
Reported case means average the individual seed RMSEs. Seed standard deviations
and ranges describe this small synthetic sample, not a population guarantee.

## Tracking results

| Case | Runs | Relative-position RMSE (m) | Target velocity RMSE (m/s) | Target acceleration RMSE (m/s²) | Worst position error after 2 s (m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Noise-free, 50 Hz | 1 | 0.00000792 | 0.01506 | 0.005706 | 0.0000531 |
| Bounded noise, 50 Hz | 10 | 0.04770 ± 0.00162 | 0.4843 ± 0.0153 | 1.8352 ± 0.0582 | 0.1220 |
| Changing motion, 50 Hz | 10 | 0.04636 ± 0.00163 | 0.4472 ± 0.0145 | 1.6143 ± 0.0515 | 0.1210 |
| One-second radar outage, 50 Hz | 10 | 0.2400 ± 0.0990 | 1.4633 ± 0.5742 | 5.1572 ± 1.9903 | 2.6329 |
| Bounded noise, 25 Hz | 10 | 0.07193 ± 0.00278 | 0.7221 ± 0.0299 | 2.7005 ± 0.1097 | 0.1594 |

Here `±` is the sample standard deviation across seeds. The last column is
the largest individual sample error across all seeds after the startup window.
The near-zero noise-free relative-position error is a same-model synthetic
result, not measured vehicle accuracy.

For the nominal noisy 50 Hz case, ego-position, ego-velocity and ego-yaw RMSE
average 0.01103 m, 0.01179 m/s and 0.0502 degrees. Target-heading RMSE averages
1.029 degrees; it rises to 4.789 degrees for dropout and 1.701 degrees at 25 Hz.
The changing-motion curvature fit reports a mean lag of 0.248 s (seed range
0.10–0.40 s), using the driver's delayed-truth least-squares lag metric.

## Peaks, domain excursions and uncertainty

The nominal noisy trials have initial target body-velocity error peaks of
9.84–10.68 m/s and acceleration-error peaks of 32.09–36.50 m/s², despite small
initial derivative offsets. Even the noise-free startup peaks reach 10.31 m/s
and 34.82 m/s². Thus the post-2-s RMSE must not be used to characterize startup.

During the one-second radar outage, position error peaks range from 0.541 to
2.633 m across seeds. After radar returns, acceleration-error peaks range from
17.06 to 84.37 m/s². Relative-position error returns below 0.15 m and stays below
that threshold through 12 s within 0.04–0.42 s after radar recovery (mean
0.244 s). This empirical position recovery does not imply that the derivative
states recover equally quickly. Outage robustness is the clearest weakness in
this campaign.

All 41 true trajectories satisfy the benchmark's declared physical domain and
model-rate check. Estimated transformed states satisfy the physical domain at
100%, 62.40%, 72.95%, 58.14% and 37.77% of evaluated samples for the five cases
in table order. For nominal noisy tracking, scalar-acceleration and curvature
limits are exceeded at 12.46% and 28.52% of samples, respectively; violations
overlap. Neither speed nor range violates its limit after 2 s in that case.
These are estimated-state excursions, not true-trajectory violations or
numerical divergence. The extended vector field continues to produce finite
estimates, but physical admissibility cannot be assumed from finite values.

The separately published online relative-position enclosure is finite and
available, and contains the measured error at all 21,641 exported samples,
including startup and dropout. It is conservative: the nominal noisy mean
radius after 2 s is 1.693 m versus 0.0477 m RMSE. The dropout mean radius is
5.929 m, and the maximum across the dropout runs reaches 89.173 m. Default
ego intersample rate envelopes remain unspecified (`Inf`); this evaluation
does not tune them to the known synthetic truth.

This observed containment is distinct from a sampled exponential-stability
certificate. The benchmark's `digitalErrorBoundCertified` flag remains false.
The runtime enclosure is conditional on its true-motion/sensor assumptions
and real-arithmetic derivation; it is not a verified floating-point interval
proof. The nominal continuous position ultimate bound is 5.760 m and is not
used as a digital sample-by-sample guarantee.

## Runtime and validation

| Case | Update median (ms) | Update P95 (ms) | Update maximum (ms) | Timed updates |
| --- | ---: | ---: | ---: | ---: |
| Noise-free, 50 Hz | 2.164 | 2.345 | 10.010 | 600 |
| Bounded noise, 50 Hz | 2.132 | 2.285 | 6.069 | 6,000 |
| Changing motion, 50 Hz | 2.159 | 2.467 | 11.644 | 6,000 |
| Radar outage, 50 Hz | 2.147 | 2.556 | 6.060 | 6,000 |
| Bounded noise, 25 Hz | 2.951 | 3.314 | 6.917 | 3,000 |

These 21,600 updates follow an initial exploratory noisy seed-7 run, so the
campaign timing describes a warmed MATLAB session. No timed campaign update
exceeds its sample period. The initial probe had a 396.040 ms maximum and
3.096 ms mean; its spike cause was not profiled. Timing includes the runtime
`step` and output publication for one target, including its online bounds;
it excludes gain synthesis, runtime initialization, simulated measurement
generation, controller computation and file export. It does not establish
hard real-time operation, cold-start latency or multiple-target scaling.

All **165 tests across 15 estimator-related classes pass**, with zero failed or
incomplete tests. Coverage includes model coordinates, gain synthesis,
continuous certificates, direct velocity, yaw observer/sets, course
correspondence, online runtime, position/controller bounds, target history,
truth-enclosure auditing, and native/reference kernel parity. The complete
controller suite was not rerun for this estimator-only evaluation.

An independent Python audit recomputes 328 RMSE/domain-fraction values from
the exported sample arrays. Maximum MATLAB/Python discrepancy is
`1.76e-14`. It also computes peaks, per-step timing percentiles, domain
violation fractions, position recovery and empirical bound containment.
Both exported figures were visually inspected. No observer modifications or
gain tuning were made after seeing the results.

## Reproduction and artifacts

From the repository root, in MATLAB:

```matlab
addpath('scripts');
probe = runOnlineNrmmComplexManeuverScenario( ...
    'Plot',false,'Report',true,'NoiseModel','boundedUniform');
benchmark = runOnlineNrmmTrackingErrorBenchmark( ...
    'Report',true,'Duration',12,'MonteCarloRuns',10,'Seed',7);
files = [dir('tests/nrmm*Test.m'); ...
    dir('tests/onlineNrmmTrackingRuntimeTest.m'); ...
    dir('tests/observerGainSynthesisTest.m'); ...
    dir('tests/certifiedKinematicCourseCorrespondenceTest.m')];
paths = arrayfun(@(f) fullfile(f.folder,f.name),files,'UniformOutput',false);
results = runtests(paths);
assertSuccess(results);
```

Actual execution used MATLAB MCP `evaluate_matlab_code` from the repository
root. The existing drivers and exact command above reproduce the scenario
conditions. Numerical trial results are committed in
[NRMM_ESTIMATOR_TRIALS_20260916.csv](NRMM_ESTIMATOR_TRIALS_20260916.csv).

Full local experiment artifacts are outside Git at
`/home/zai/.cache/collisionAvoidance/estimator-evaluation-20260916/`:

- `benchmark.mat`: all 41 results plus the initial exploratory run.
- `tests.mat`, `tests.csv`, `validation.json`: actual regression outcomes.
- `*-seed*.csv`, `exported-metrics.json`: raw samples and original metrics/configs.
- `independent-analysis.json`, `analyze.py`, `exportEvaluation.m`: independent
  analysis and the exact export/analysis helpers used for this run.
- `performance-summary.png/.pdf`, `error-traces.png/.pdf`: standalone figures.
- `artifact-manifest.json`: source/dependency and local result hashes.

The experiment supports prioritizing reduced derivative peaking/noise and
radar-reacquisition behavior in subsequent research. It does not validate
real sensor data, physical model mismatch outside these cases, arbitrary
outages, or closed-loop collision avoidance.
