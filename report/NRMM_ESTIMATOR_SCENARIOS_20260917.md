# NRMM estimator multi-scenario campaign — September 17, 2026

The current estimator (free-metric gains of `a6fd787`) was exercised in eleven
open-loop synthetic scenarios, 101 twelve-second (oncoming: eight-second)
trials with fresh seeds 101–110, plus one partial estimator-in-the-loop run on
the declared affine ego plant. Relative-position RMSE after the 2 s startup
window stays between 0.027 and 0.043 m in every scenario that respects the
declared sensor bounds, rises to 0.060 m when every sensor noise is doubled
beyond its declared bound, and the online relative-position enclosure
contains the true error at all 55,701 exported samples. The clearest
remaining weaknesses are the 1 s radar outage (position peak up to 0.33 m,
enclosure radius up to 87.7 m) and the 0.6 s curvature response lag. No
observer code, gain, default or controller was changed; the two scenario
drivers gained opt-in options whose defaults reproduce the committed
seed-71 trial to the last digit.

Tested source revision: `37a652e9ba2fe4d05bdff6dce89f33c25b28c232` plus the
driver changes committed with this report. MATLAB R2026a Update 3, Linux
x86-64, AMD Ryzen 7 7800X3D. The working tree also held unrelated,
uncommitted controller edits; they do not enter the open-loop estimator
path and are excluded from this commit.

## Scenario definitions

All scenarios share the synthetic sensor contract of
[the September 16 evaluation](NRMM_ESTIMATOR_EVALUATION_20260916.md):
50 Hz synchronized GNSS position/velocity, IMU body acceleration, gyro yaw
rate and radar relative position, bounded uniform noise inside the declared
maxima 0.12 m, 0.02 m/s, 0.05 m/s², 0.002 rad/s and 0.12 m, RK4 step limit
0.005 s, and the documented initial offsets. Metrics use samples at
`t >= 2 s`; startup peaks cover 0–2 s. The retained ego drives
`12 + 1.2 sin(0.25 t)` m/s with yaw rate `0.12 sin(0.35 t)` rad/s; the
retained target starts at `[25, 4]` m, 12.5 m/s, heading 0.10 rad, with
constant scalar acceleration 0.08 m/s² and curvature 0.004 m⁻¹.

| Case | Change from the retained scenario | Declared-domain change |
| --- | --- | --- |
| `retained-noise-free` | No noise, one trial | none |
| `retained-noise` | none | none |
| `varying-noise` | 0.2 s tanh step at 6 s: +0.6 m/s² acceleration, −0.007 m⁻¹ curvature | range 55 m; rate bounds 1.5 m/s³, 0.0175 m⁻¹s⁻¹ |
| `dropout-noise` | radar absent on [4, 5) s | none |
| `retained-noise-25Hz` | 0.04 s sample period | none |
| `aggressive-ego-noise` | ego speed `12 + 3 sin(0.5 t)`, yaw rate `0.12 sin(0.35 t) + 0.16 sin(1.2 t)` (peak 0.28 rad/s, peak sideslip 0.031 rad) | range 80 m |
| `oncoming-noise` | straight ego at 12 m/s; target at `[95, 3.5]` m heading π, 11 m/s, closing at about 23 m/s, passing at 4.12 s; 8 s duration | range 100 m |
| `lane-change-noise` | one sine period of curvature, amplitude 0.005 m⁻¹ over 4–8 s (peak curvature 0.009 m⁻¹, lateral shift about 2 m) | range 55 m; curvature rate 0.008 m⁻¹s⁻¹ |
| `noise-2x` | every noise draw doubled, so all sensors exceed their declared bounds by up to 2× | none |
| `large-offset-noise` | all initial estimate offsets tripled (target velocity offset 2.1 m/s, acceleration offset 0.85 m/s²) | none |
| `intermittent-dropout-noise` | five 0.2 s radar gaps starting at 2.5, 4.5, 6.5, 8.5, 10.5 s | none |

A declared-domain change alters the synthesized target bandwidth
(2.591 s⁻¹ nominal; 2.323 at 80 m, 2.233 at 100 m, 2.554 for the lane change,
2.592 for the varying case), so cross-case comparisons include that design
difference. Every true trajectory satisfies its declared domain and
model-rate bounds; the benchmark asserts this before recording a trial.

The new driver options are `EgoManeuver`, `TargetInitialPosition`,
`TargetInitialHeading`, `TargetInitialSpeed`, `NoiseScale`,
`InitialOffsetScale` and the `laneChange` target motion in
`scripts/runOnlineNrmmComplexManeuverScenario.m`, and six case names in
`scripts/runOnlineNrmmTrackingErrorBenchmark.m`. With default options the
seed-71 nominal trial reproduces the committed
`NRMM_NOISE_REDUCTION_TRIALS_20260916.csv` values exactly (position
0.0288376456587 m, velocity 0.1192879119 m/s, acceleration 0.183148147047
m/s²).

## Open-loop results

Entries are means of the individual seed RMSEs with `±` the sample standard
deviation across the ten seeds; the noise-free row is a single trial. The
last column is the largest single-sample relative-position error after 2 s
across all seeds of the case.

| Case | Relative-position RMSE (m) | Target velocity RMSE (m/s) | Target acceleration RMSE (m/s²) | Target heading RMSE (deg) | Worst position error after 2 s (m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Noise-free, 50 Hz | 0.000123 | 0.01531 | 0.00645 | 0.063 | 0.0010 |
| Bounded noise, 50 Hz | 0.02980 ± 0.00183 | 0.1231 ± 0.0070 | 0.1873 ± 0.0102 | 0.321 | 0.0785 |
| Changing motion | 0.03029 ± 0.00186 | 0.1347 ± 0.0061 | 0.3123 ± 0.0069 | 0.323 | 0.0799 |
| 1 s radar outage | 0.03961 ± 0.01146 | 0.1440 ± 0.0296 | 0.2153 ± 0.0418 | 0.386 | 0.3315 |
| Bounded noise, 25 Hz | 0.04258 ± 0.00211 | 0.1766 ± 0.0086 | 0.2656 ± 0.0134 | 0.462 | 0.1120 |
| Aggressive ego weave | 0.02820 ± 0.00182 | 0.1157 ± 0.0054 | 0.1515 ± 0.0076 | 0.316 | 0.0767 |
| Oncoming target, 8 s | 0.02724 ± 0.00179 | 0.0982 ± 0.0069 | 0.1295 ± 0.0088 | 0.312 | 0.0795 |
| Target lane change | 0.03067 ± 0.00191 | 0.1401 ± 0.0064 | 0.3608 ± 0.0076 | 0.352 | 0.0811 |
| Noise 2× declared bounds | 0.05960 ± 0.00367 | 0.2448 ± 0.0138 | 0.3744 ± 0.0204 | 0.632 | 0.1571 |
| Initial offsets 3× | 0.02983 ± 0.00185 | 0.1233 ± 0.0070 | 0.1884 ± 0.0099 | 0.322 | 0.0785 |
| Five 0.2 s radar gaps | 0.03293 ± 0.00241 | 0.1327 ± 0.0095 | 0.2009 ± 0.0138 | 0.340 | 0.1250 |

Ego-state accuracy is stable: ego position, velocity and yaw RMSE are
0.0122 m, 0.0115 m/s and 0.050° in the nominal case, 0.0175 m, 0.0233 m/s
and 0.099° at 25 Hz, 0.0202 m, 0.0183 m/s and 0.072° for the aggressive
weave, and 0.0320 m, 0.0124 m/s and 0.042° for the straight oncoming
scenario. The larger straight-road ego position figure comes from a slower
initial position transient (0.39 m at 1 s versus 0.11 m for the retained
profile, identical initial offsets), which the shorter 8 s window then
weights more heavily; the settled error at 8 s is 0.007 m in both. The
cause of the profile-dependent transient was not analyzed.

Observations by scenario:

- **Ego maneuvering and encounter geometry barely matter for target
  accuracy.** The aggressive weave and the oncoming pass give the smallest
  target errors of all noisy cases, partly because their larger declared
  ranges lower the synthesized bandwidth. Around the oncoming pass (±1 s of
  4.12 s) the position error peaks at 0.043–0.080 m and the acceleration
  error at 0.23–0.33 m/s², so the change of relative-position sign does not
  disturb the tracker.
- **Target maneuvers cost derivative accuracy and lag, not position.** The
  lane change and the acceleration/curvature step keep position RMSE within
  3% of nominal but raise acceleration RMSE to 0.36 and 0.31 m/s². The
  least-squares curvature lag is 0.58–0.64 s for the lane change (mean
  0.616 s) and 0.58–0.68 s for the step (mean 0.61 s), consistent with the
  lag reported for the lower-bandwidth design on September 16. During the
  lane change the estimated curvature exceeds the 0.009375 m⁻¹ domain
  limit at 0.9% of samples because the true peak (0.009 m⁻¹) is already
  within noise of that limit.
- **Doubling every noise beyond its declared bound scales the errors
  linearly** (position 2.00×, velocity 1.99×, acceleration 2.00×) with no
  loss of finiteness or domain validity, and the enclosure still contains
  the error at every sample because its nominal radius (1.76 m) is far above
  the realized error. This is degradation, not a certified property: the
  bound's premises are violated in this case.
- **Tripled initial offsets triple the startup peaks and double the
  settling time**, then leave the settled errors unchanged. The initial
  acceleration-error peak rises from 5.07 to 15.2 m/s² and the body-velocity
  peak from 3.8 to 11.2 m/s; the time after which position error stays below
  0.15 m grows from 0.48–0.54 s to 1.06–1.26 s. After 2 s all metrics match
  the nominal case to three digits.
- **Short radar gaps are absorbed; a full second is not.** Five 0.2 s gaps
  raise the position peak during a gap to at most 0.125 m, the
  post-reacquisition acceleration peak to at most 0.55 m/s², and the
  enclosure radius to 17.6 m at the end of a gap; recovery below 0.15 m is
  immediate in every seed. The 1 s outage produces position peaks of
  0.036–0.331 m across seeds, reacquisition acceleration peaks of
  0.40–1.69 m/s², enclosure radii growing to 87.7 m, and brief domain
  excursions in some seeds (mean valid fraction 99.72%). Compared with the
  previous gains, the 1 s outage position peak has fallen from 0.54–2.63 m
  to 0.04–0.33 m and the reacquisition acceleration peak from 17–84 m/s² to
  0.4–1.7 m/s², measured on different seeds.
- **25 Hz sensing remains the costliest sensor-side change** among the
  declared-bound cases (position +43%, acceleration +42% versus 50 Hz), with
  the enclosure radius almost doubling to 3.23 m.

The estimated states satisfy the physical domain at 100% of post-2 s
samples in nine cases, 99.72% for the 1 s outage and 99.08% for the lane
change. All 101 trials are finite. Every published online relative-position
enclosure contains the measured error (0 violations in 55,701 samples,
including startup, gaps and the pass), with mean post-2 s radii of
1.65–1.76 m for the continuous-radar cases, 2.56 m with intermittent gaps,
3.23 m at 25 Hz and 5.93 m with the 1 s outage. Observed containment is not
a sampled digital certificate; `digitalErrorBoundCertified` remains false.

Per-update runtime (runtime `step` plus output publication for one target)
has a pooled median of 2.10–2.31 ms and a 95th percentile of 2.30–2.57 ms
at 50 Hz, and 2.88/3.24 ms at 25 Hz. The only deadline misses are three
updates in the first trial of the session (maximum 326 ms, cold JIT and
kernel load); the remaining 55,000 timed updates stay below their sample
period, with a maximum of 10.1 ms. This describes a warmed MATLAB session,
not hard real-time behaviour.

## Estimator-in-the-loop attempts

The active estimator-in-the-loop suite `runEstimatedStateAvoidanceScenarios`
(PassVeh14DOF plant, straight and circular roads) cannot run with the
current controller: both scenarios stop at the first control step with
`collisionAvoidanceController:nonexactStudyInput` ("This controller
requires the declared zero-residual held affine plant"), an error that is
part of the committed controller, not of the estimator. No estimator
metrics exist for that suite.

`runDeclaredPlantEstimatorControllerScenario` (declared affine ego plant,
straight road, oncoming target from 100 m at 10 m/s, 30 m radar range,
seed 20260917, 5 s frame deadline recorded but not enforced) ran 36 holds
with the NRMM output driving the controller and then stopped at t = 3.6 s
with `collisionAvoidanceController:optimizationFailed` (bounded
support-family search without a hard certificate, Clarabel status 2) at the
first frame after target acquisition (3.5125 s). A first attempt with road
boundaries enabled stopped at the first sample because the recursive cruise
certificate requires a road-free reference domain. Over the 37 recorded
frames the ego position error has RMSE 0.0053 m and maximum 0.0139 m
against published position bounds of 0.040–0.078 m, the ego yaw error has
RMSE 0.00049 rad and maximum 0.00179 rad against bounds of 0.047–0.051
rad, and both bounds contain the truth at all 37 frames. The single
published target frame has 0.0135 m position and 0.179 m/s velocity error
with a 0.0586 m relative-position enclosure. Observer time per frame has
median 9.4 ms (maximum 293 ms on the first frame). These 37 frames are a
partial closed-loop record; they do not validate closed-loop avoidance.
The working tree's uncommitted controller edits were active during this
run, so the controller stop is not attributed to a committed revision.

## Validation and reproduction

The three test classes exercising the modified drivers pass (65/65:
`nrmmStructuredHighGainTest`, `nrmmPositionErrorBoundTest`,
`onlineNrmmTrackingRuntimeTest`); the full suite was not rerun for this
estimator-only campaign. Code Analyzer reports no findings on the two
modified scripts. An independent Python audit recomputes 808 RMSE and
domain-fraction values from the exported sample arrays with maximum
discrepancy `1.79e-14`, and computes peaks, settling and recovery times,
domain-violation fractions, bound containment and timing percentiles.
Both figures were inspected. No observer modification followed the
results.

From the repository root:

```matlab
addpath('scripts');
campaign = runOnlineNrmmTrackingErrorBenchmark( ...
    'Report',true,'Seed',101,'MonteCarloRuns',10,'Duration',12);
results = runtests({'tests/nrmmStructuredHighGainTest.m', ...
    'tests/nrmmPositionErrorBoundTest.m','tests/onlineNrmmTrackingRuntimeTest.m'});
assertSuccess(results);
```

Per-trial numerical results are committed in
[NRMM_ESTIMATOR_SCENARIOS_20260917.csv](NRMM_ESTIMATOR_SCENARIOS_20260917.csv).
Full local artifacts are outside Git at
`/home/zai/.cache/collisionAvoidance/estimator-scenarios-20260917/`:
`campaign.mat` and the campaign table/summary CSVs, 101 raw sample CSVs,
`exported-metrics.json`, `analyze.py` with `independent-analysis.json` and
`audited-trials.csv`, `campaign-summary.png/.pdf`, `campaign-traces.png/.pdf`,
the exact drivers `runCampaign.m`, `runClosedLoop.m`, `runDeclaredPlant.m`,
`extractDeclaredPlant.m` and `probeCases.m`, the closed-loop records
`closed-loop.mat`, `declared-plant/joint-declared-plant.mat` and
`declared-plant/estimator-extract.txt`, `tests.csv`, all logs, and
`artifact-manifest.json` with source and result hashes.

The campaign supports prioritizing outage handling and maneuver lag; it
does not validate real sensors, model mismatch beyond these synthetic
cases, multiple targets, or closed-loop collision avoidance.
