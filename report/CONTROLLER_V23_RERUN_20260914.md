# Independent version-23 straight-scene rerun

September 14, 2026. Tested revision:
`f410e595784f4051d22581aeabd626c3d5f1cc2d`
(`Add sampled backup CBF control with hard cruise dissipation`).
Controller, observer, configuration, native binaries and experiment drivers
were unchanged during this rerun.

## Result

The exact-state stationary, oncoming and crossing trials complete avoidance
and recover constant-speed path cruising. The independent 90 s exact-state
trial with a current 30 m visibility gate also recovers: during 80--90 s,
speed remains 10 m/s and lateral/heading errors are at numerical precision.
The sustained oscillation in the [version-22 rerun](CONTROLLER_V22_RERUN_20260914.md)
is absent in this experiment.

The complete pipeline is still not qualified. Bounded-error stationary and
oncoming admissions fail; actual NRMM fails when its first target is
published at 3.5 s. Several trials exceed the 100 ms complete-frame limit,
including successor frames. A fresh MATLAB process takes 917.248 ms for
its first stationary admission. These failures are retained in the results.

## Policy and model being tested

The existing drivers select `ExecutionPolicy="auto"`. With their positive
constant-speed references and finite 100 ms frame budget, version 23 selects
`"backup"`: bounded fixed-continuation proposals, full swept verification,
carried encounter witnesses, and hard sampled cruise CLF checks. The 900
executed holds in the long trial record this policy and zero online solver
calls. The SOCP search remains available through `"predictive"` and is
covered by the selected regression tests.

This execution-policy change is material to interpreting the results. The
new policy uses its declared cruise-trim affine generator. The previous
predictive branch used its own declared stage generators. Both drivers
integrate the generator issued with the command, so this is not a comparison
on one common, independently validated nonlinear vehicle plant. See the
[implementation report](SAMPLED_BACKUP_CBF_CLF_RESULTS_20260914.md) and
[sampled CBF/CLF derivation](../controller/SAMPLED_BACKUP_CBF_CLF.md).

## Sequential campaign and complete-frame timing

All ordinary trials request 300 held controls at 0.1 s each. The eight-case
campaign ran after regression tests in the same MATLAB session. The 90 s
trial then restarted the exact visibility-gated scene from its initial state.
A separate fresh MATLAB process subsequently ran the stationary trial.
This task launched no concurrent MATLAB test/simulation work during timing;
unrelated workstation load was present and not controlled.

Timing includes first admission, failed decisions and the final unexecuted
command. For NRMM it includes online observer/adapter and controller work.
Offline observer synthesis/initialization, true-plant integration, audit,
plotting and saving are outside the timed online frame.

| Trial | Executed / requested holds | Median frame (ms) | Maximum frame (ms) | Frames above 100 ms | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| Exact stationary | 300 / 300 | 4.332 | 365.527 | 3 / 301 | Safety/truth checks pass; returns to 8 m/s cruise |
| Exact oncoming | 300 / 300 | 4.109 | 103.303 | 1 / 301 | Safety/truth checks pass; returns to 8 m/s cruise |
| Exact crossing | 300 / 300 | 4.059 | 28.321 | 0 / 301 | Safety/truth checks pass; returns to 8 m/s cruise |
| Bounded stationary | 0 / 300 | 104.905 | 104.905 | 1 / 1 | Two proposals rejected; no control issued |
| Bounded oncoming | 0 / 300 | 127.341 | 127.341 | 1 / 1 | Two proposals rejected; no control issued |
| Bounded crossing | 300 / 300 | 4.633 | 46.967 | 0 / 301 | Safety/truth checks pass; practical cruise recovery |
| Exact oncoming, 30 m visibility | 300 / 300 | 3.510 | 52.580 | 0 / 301 | Avoidance and 10 m/s path cruise complete |
| Actual NRMM, 30 m visibility | 35 / 300 | 16.070 | 155.952 | 2 / 36 | First target admission fails at 3.5 s |
| Additional exact visibility trial, 90 s | 900 / 900 | 4.799 | 63.790 | 0 / 901 | Avoidance and stable 10 m/s path cruise |
| Fresh-process exact stationary | 300 / 300 | 4.373 | 917.248 | 3 / 301 | Safety/truth checks pass; returns to cruise |

The ordinary stationary overruns occur at 0, 4.7 and 5.7 s, with frame
times 365.527, 123.418 and 136.731 ms. Its maximum successor time is
136.731 ms. The fresh-process successor maximum is 142.758 ms. Thus moving
initialization outside the loop alone does not establish the required
100 ms bound. Exact oncoming overruns only at initial admission in this run.

The NRMM initial frame takes 152.201 ms. Its failed 3.5 s frame takes
155.952 ms, including 24.517 ms observer/adapter and 131.015 ms controller
work; the small remainder is frame orchestration. The faster routine frames
do not erase these misses. These are observed timings, not a worst-case
execution-time proof.

## Safety and eventual cruise

The following sampled geometric margins are additional to the configured
0.25 m clearance. Independent audits sample eleven points per executed
hold; the controller's continuous-time certificate uses swept Bernstein
enclosures. Sampled audit success alone is not a continuous-time proof.

| Completed ordinary trial | Minimum extra separation (m) | Minimum extra road margin (m) | Confirmed release (s) |
| --- | ---: | ---: | ---: |
| Exact stationary | 0.153430 | 0.559733 | 4.2 |
| Exact oncoming | 1.039595 | 0.562679 | 5.0 |
| Exact crossing | 9.269716 | 3.800000 | 0.7 |
| Bounded crossing | 9.286414 | 3.790549 | 0.7 |
| Exact 30 m visibility, both durations | 0.683082 | 0.545434 | 6.5 |

The completed exact-driver trials pass their model-domain and truth-box
containment checks and retain zero safety value. No terminal-law command
occurs in these completed trials or in the long visibility-gated run.
The bounded crossing ends at 8.000069447 m/s, lateral error 0.004125842 m
and heading error 0.000075509 rad. Its large separation makes it a mild
collision test; it does not resolve the difficult uncertain passing cases.

For both exact visibility-gated runs, independent reconstruction of every
executed affine hold matches saved successors within `4.45e-16`; the
minimum sampled model-domain margin is 0.163891. All issued commands have
zero safety value. The range driver does not retain a complete carried-set
history for a second independent full witness-containment audit.

During 80--90 s in the long run:

- Speed is 10 m/s to reported precision.
- Maximum absolute lateral error is below `2.4e-52` m.
- Maximum absolute heading error is below `9.1e-53` rad.
- No terminal-law command or deadline miss occurs in the full 90 s run.

These extremely small pose values indicate ideal-model numerical convergence,
not physically achievable positioning accuracy. No early-recovery deadline
was imposed.

For each executed hold marked as CLF-certified, the audit independently
evaluates, from consecutive true states and the recorded metric/reference,

```text
V(x[k+1]) - (1 - decay[k]) * V(x[k]) - disturbanceBound[k].
```

All 837 such holds satisfy the inequality. The largest residual is
`-1.1627196410549331e-24`, the minimum decay fraction is
`0.0736260316405486` per 0.1 s hold, and the largest disturbance allowance
is `1.1627196410549331e-24`. This checks sampled cruise dissipation; it
does not assert dissipation on the other 63 executed holds or pointwise
decrease between samples.

## Remaining admission failures

The bounded stationary/oncoming trials each reject two complete fixed
continuations before issuing control. The last proposals have safety values
9.85679 and 14.9234, respectively, with zero hard-row violation. Their
sufficient road/collision safety inequalities are violated. This establishes
failure of this candidate family, not global infeasibility or unavoidable
collision.

Actual NRMM completes 35 target-free holds before first publication at 3.5 s.
Both encounter proposals are rejected; the last records safety value
2254.13 and hard-row violation 127.343. The run stops immediately and does
not execute a road-only witness as if it covered the new target.

All 36 sampled ego containment and premise checks pass; all available
target components checked at first publication contain truth. Nevertheless,
the initial target information is broad: position radii are approximately
`[0.173241; 1.509781]` m, velocity radii
`[29.996358; 20.269849]` m/s, acceleration radii
`[2.358493; 2.358493]` m/s^2, and yaw radius is pi. The history enclosure
contains only one measurement. Finite prediction also uses jerk bounds
`[0.383050; 0.383050]` m/s^3. The target velocity point estimate near
`[-9.996358; -0.269849]` m/s is much more informative than the guaranteed
set available on that first frame.

The evidence therefore identifies an unresolved initial information-set
and robust-admission problem. It does not establish observer divergence,
nor prove that every robust control law fails. The target-free prefix's
positive separation is not evidence of completed joint avoidance.

## Regression outcome

The fresh selected run contains 95 cases: **94 pass and one fails/incompletes**.
All 20 sampled-backup tests pass, including uncertain box vertices, cached
swept enclosures, curved trims, finite input slew and optimizer-independent
avoidance. The selected online NRMM runtime tests also pass.

The failure is
`controllerRepairTest/aPositiveValueAttemptCanTryAnotherPassingSide`.
Its retained predictive search exhausts its 15 s budget after four attempts
(last solver report: Clarabel status 5). The test then attempts to access
`check.safetyCertified` when no check structure was returned, producing a
secondary dot-indexing error. A single isolated repeat, without modifying
code or parameters, passes in 12.490 s. The latest result for all 95 distinct
selected cases is therefore passing, but this is not one clean 95-case run.
The original failure is preserved as evidence of time-sensitive predictive
search behavior.

## Reproduction and retained artifacts

MATLAB R2026a Update 3, GLNXA64, eight computational threads; seed 20260914.
Initial horizon 16, sample time 0.1 s, road boundaries +/-5 m, clearance
0.25 m. The first six scenes use reference 8 m/s and confirmation range
16 m. Exact initial targets are stationary at `[15;0]`, oncoming at
`[60;0]` with velocity `[-8;0]`, and crossing at `[15;-4]` with velocity
`[0;32]`, in SI units. Ego starts at `[0;0;0;8;0;0]`.

Bounded cases use ego radii `[.05;.05;.005;.05;.02;.005]` in
`[px;py;psi;vx;vy;r]`, target radii
`[.1;.1;.1;.1;.05;.05;.01;.01]` in
`[px;py;vx;vy;ax;ay;psi;omega]`, jerk amplitudes `[.1;.1]` m/s^3,
yaw-acceleration amplitude .05 rad/s^2 and frequency 1 rad/s.
The exact/NRMM visibility pair uses reference 10 m/s, physical range 30 m,
target initial position `[100;.8]` m and velocity `[-10;0]` m/s. Actual NRMM
retains the configured 80 Hz observer/sensor and 0.0125 s maximum
integration step. The first six scenes and visibility pair are different
initial encounters.

From the repository root:

```matlab
addpath('scripts');
out = '/home/zai/.cache/collisionAvoidance/controller-v23-rerun-20260914';
campaign = runStraightControllerRerun( ...
    SampleCount=300, Seed=20260914, OutputDirectory=fullfile(out,'campaign'));
longRange = runDeclaredPlantEstimatorControllerScenario( ...
    SampleCount=900, UseEstimator=false, Seed=20260914, ...
    OutputDirectory=fullfile(out,'range-exact-90s'));
```

Use a new output directory when repeating to preserve these originals.
For a fresh-process test, launch MATLAB `-batch` and call
`runExactStateRecursiveFeasibilityScenario` with `Scenario="stationary"`,
`SampleCount=300`, `Seed=20260914` and a separate output directory.

The selected regression files are `sampledBackupControllerTest`,
`controllerRepairTest`, `boundedTargetMotionTest`, `bicycleNativeKernelTest`,
`avoidanceNativeGeometryTest`, `freeCompletionTimeTest`,
`encounterCertificateScenarioTest` and `onlineNrmmTrackingRuntimeTest`.
Run their `tests/*.m` files with `runtests`; the recorded initial
`assertSuccess` fails and the isolated repeat is retained separately.

Original MAT/CSV/JSON results, initial/retry test records, failure diagnostic,
environment metadata, audit helpers and the visually checked 90 s figure
remain under the cache directory above. Key files are
`campaign/summary.json`, `range-audits.json`, `clf-audit.json`,
`timing-detail.json`, `cold-summary.json`, `regression.csv`,
`regression-retry.csv` and `range-exact-90s.png`. Generated artifacts and
solver dependencies are excluded from the project commit. The unchanged
native Clarabel bridge has SHA-256
`30f2cca72ef8622733f6bfd32f6567be0dafede7a6dfe94d050ca33c23344e5a`.

The experiment measures computation while retaining an ideal hold clock;
overruns are not simulated as actuation delay. The successful long trial
supports declared-model recovery, while robust first admission and an
every-frame real-time requirement remain open.
