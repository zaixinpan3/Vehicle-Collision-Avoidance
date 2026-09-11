# Straight-road rerun of the exact two-vehicle controller

Test date: September 11, 2026. Controller baseline:
`388500be9f3af917930902ccaa0bd87e9021b847`.

All three 30-second mathematical-model trials complete with certified commands,
nonnegative sampled collision/road margins and no input-slew violation.
**None meets the 100 ms complete-frame deadline. None restores 8 m/s cruise.**
The controller's current terminal law dissipates velocity to rest.

## Scope and reproduction

The current version-17 contract requires exact ego/target states, one persistent
target and an exactly executed retained scheduled affine plant. It rejects the
former finite-perception/uncertain-estimator inputs. Accordingly these runs use
`runExactStateRecursiveFeasibilityScenario`, not the superseded physical/joint
runner. They do not validate a nonlinear Fiala vehicle, PassVeh14DOF plant,
state estimator, sensing pipeline or physical recursive safety.

```matlab
addpath('scripts');
report = runExactStateRecursiveFeasibilityScenario( ...
    Scenario="oncoming", SampleCount=300, FailAfterAdmission=false, ...
    DeadlineSeconds=0.1, OutputDirectory="/absolute/output/path");
```

Repeat with `Scenario="stationary"` and `Scenario="crossing"`. Execution order
was oncoming, stationary, crossing, without an explicit warm-up campaign or
failure injection. Each run has 300 executed 100 ms holds and 301 controller
calls, including the final unexecuted command. The road is straight with edges
at lateral positions -5 and +5 m. Ego starts at world position [0,0], heading
zero and speed 8 m/s. The initial horizon is 16 stages; admission may extend it.
The certificate-search budget is 30 s. Target states are:

| Scenario | Initial position (m) | Constant velocity (m/s) | Heading |
|---|---|---|---|
| Oncoming | [60,0] | [-8,0] | pi |
| Stationary | [15,0] | [0,0] | 0 |
| Crossing | [15,-4] | [0,32] | pi/2 |

All target accelerations/yaw rates are zero. There is no random noise or seed.
Complete configurations and terminal certificates are saved in each MAT/JSON
result. Original outputs are in
`/home/zai/.cache/collisionAvoidance/straight-current-20260911`.

## Results

| Scenario | Certified calls | Terminal entry (s) | Minimum separation margin (m) | Minimum road margin (m) |
|---|---:|---:|---:|---:|
| Oncoming | 301/301 | 2.1 | 0.056741 | 0.274144 |
| Stationary | 301/301 | 1.6 | 2.815658 | 3.800000 |
| Crossing | 301/301 | 1.6 | 9.461575 | 3.800000 |

Margins already subtract the configured 0.25 m clearance. Geometry is audited
at 11 points per executed hold using independent matrix-exponential propagation.
These sampled checks supplement the declared-model certificate; sampling alone
is not a continuous-time proof. Maximum input-slew violation is zero in all runs.

| Scenario | First frame (ms) | Maximum later prefix frame (ms) | Maximum terminal frame (ms) | Frames over 100 ms |
|---|---:|---:|---:|---:|
| Oncoming | 9004.179 | 2821.003 | 24.319 | 21/301 |
| Stationary | 1691.816 | 1131.361 | 18.148 | 16/301 |
| Crossing | 1348.827 | 987.458 | 14.045 | 16/301 |

Timing includes input assembly and the controller call, excluding exact plant
propagation, dense geometry auditing and report I/O. Initial scene-structure
assembly is included in the first frame. This is diagnostic execution: it
continues after missed deadlines, so completed geometry trials do not imply
realtime executability. `report.passed` retains its certificate/geometry meaning;
`report.runtimeQualified` additionally requires every frame to meet the deadline.
No measurement is a platform worst-case execution-time bound.

All 53 deadline misses occur before terminal feedback. This locates the observed
problem in admission/prefix optimization rather than the analytic terminal phase;
it is phase attribution, not an internal solver profile. Even later prefix frames
alone exceed the deadline, so removing startup overhead would not suffice.

Despite no injected solver failures, the oncoming and crossing runs each retain
the existing certified witness on 14 calls; stationary retains it on zero calls.
The current algorithm permits this when an attempted replacement is not accepted.
This does not establish that those mathematical programs were infeasible.

Final forward speeds are approximately 1.67e-15, 7.27e-16 and 7.27e-16 m/s,
respectively. The oncoming trial ends at lateral offset 2.823008 m and heading
error 0.275122 rad (15.76 degrees). This is the admitted stopping behavior,
not convergence to centered constant-speed cruise. Extending the simulation
under the same terminal law will not restore the cruise reference.

## Regression and implementation scope

The following current-contract suites pass, 34 tests total:
`hardEncounterBarrierTest`, `targetObservationConditioningTest`,
`encounterCertificateScenarioTest`, and `straightControllerRecoveryTest`.
The last suite confirms rejection of inputs outside the exact-study contract;
its pass is not evidence of successful estimator/controller recovery.
Factory Code Analyzer reports zero findings for the instrumented scenario.

Only the experiment driver gains per-frame timing, deadline diagnostics and a
separate runtime qualification flag. Controller/observer algorithms, optimizer
settings and terminal policy are unchanged. The measured result is: successful
exact scheduled-model avoidance/continuation, failed realtime qualification,
and stopping rather than cruise recovery. No joint-estimator experiment is
claimed under the currently unsupported input contract.
