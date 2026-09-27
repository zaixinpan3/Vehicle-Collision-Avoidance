# Vehicle controller method comparison

> Historical record: the experimental nonlinear shooting controller and its dedicated execution/audit scripts were removed on 2026-09-26 at the user's request. Commands and implementation descriptions below refer to the recorded experiment, not the current controller. See [removal decision](NONLINEAR_SHOOTING_REMOVAL_20260926.md).

Prepared 2026-09-26T19:59:04-05:00.

The default `affineSocp` controller completes both no-target cruise controls but
none of the three exact-state avoidance scenarios. With the explicitly selected
development optimization profile, `nonlinearShooting` completes the full
avoidance-and-recovery criteria in 3/3 matched PassVeh14DOF scenarios.
This is a fresh independent nonlinear-vehicle experiment, distinct from the
previous declared Fiala bicycle study. The experiment changes no production source.

## Matched vehicle results

| Scenario | Affine executed/requested (s) | Affine minimum prefix SAT gap (m) | Affine recovery | Nonlinear executed/requested (s) | Nonlinear minimum SAT gap (m) | Nonlinear recovery |
| --- | ---: | ---: | --- | ---: | ---: | --- |
| Straight oncoming | 2.05 / 18 | 4.019774 | No | 18.00 / 18 | 0.140368 | Yes |
| Circular oncoming | 1.50 / 18 | 15.360714 | No | 18.00 / 18 | 0.107608 | Yes |
| S bend oncoming | 0.00 / 18 | N/A | No | 18.00 / 18 | 0.161762 | Yes |

SAT gap is the largest separating-axis gap between the oriented vehicle
rectangles; it is not Euclidean closest-point distance. Positive stopped-prefix
values establish sampled non-overlap only through the executed prefix. They do
not establish safe continuation. A zero-hold run has no measured plant gap.

Affine straight stops at 2.05 s after all three fixed-direction searches are
rejected (Clarabel status 2). Affine circular stops at 1.50 s with
`invalidTireOperatingPoint`, whose domain is `abs(alpha)<pi/2`, `abs(beta)<=1`.
The first S-bend attempt expires its internal work budget before the native
solve at time zero. A separate new-process cold/warm pair reproduces the same
cold stop, followed by a warmed 1.40 s stop with solver rejection and a
17.181475 m prefix SAT gap. The warm repeat remains a failure and does not
replace the original result.

## Completion and recovery audit

Recovery requires the whole requested 18 s episode, positive sampled vehicle
separation, complete-body target passing, sampled perceived-road containment,
and every recorded plant sample in the final 2 s within 0.5 m/s longitudinal
speed error, 0.2 m lateral error, and 0.02 rad velocity-course error. Course
includes sideslip. Geometry, passing, path errors, quadratic-road rectangle
minima and the final-window verdict are independently recomputed in Python.

| Nonlinear scenario | Full-body passing time (s) | Final-window max speed / lateral / course error (m/s, m, rad) | Sampled road safe |
| --- | ---: | --- | --- |
| `straight_oncoming` | 2.735 | 0.091277 / 0.083825 / 0.018493 | Yes |
| `circular_oncoming` | 2.755 | 0.086658 / 0.031524 / 0.009101 | Yes |
| `varying_curvature_oncoming` | 2.745 | 0.182737 / 0.046519 / 0.018556 | Yes |

Both affine no-target controls complete 18 s and satisfy recovery. The affine
NRMM-estimated straight and circular cases stop at first target visibility,
1.05 s; their prefix SAT gaps are 23.994603 and 24.048799 m. The nominal
nonlinear controller does not support nonzero state uncertainty, so those two
estimator cases are not presented as matched nonlinear trials.

## Conditions and interpretation

- MATLAB R2026a Update 3, PassVeh14DOF from Vehicle Dynamics Blockset;
  one computational thread per MATLAB process. Requested duration 18 s,
  ego/target speed 10 m/s, control period 0.05 s, initial target distance 50 m,
  visibility 30 m, circular radius 100 m, existing S-bend geometry and perceived
  road-boundary defaults. No-target controls retain their no-boundary defaults.
  Ego and target rectangles are both 5.0 by 2.0 m; target lateral conflict
  offset is 1.5 m. Road offsets are 6 m right and 8 m left, with 2.6 m shoulders.
- Exact-state cases have no stochastic measurement input. Estimated cases use
  seed 20260925 and 10 m/s target-speed priors. That seed is a reproducibility
  parameter, not the activity date.
- Affine uses the current defaults, including its 5 s certificate search budget.
  Nonlinear uses the existing reported development profile: 5 s horizon,
  four-hold blocking, maximum 12 SQP iterations per seed, 20 s nominal search
  budget, and four refinement iterations after first feasibility. Other limits
  remain the current configuration defaults. These are different optimization
  configurations, not a controlled solver-only ablation. Callback budgets are
  not hard real-time deadlines on indivisible evaluations.
- The three paired runs are checked for matching initial control state,
  duration, speed, period, centerline, vehicle parameters, road-load parameters,
  actuator limits, target dimensions/motion and road-boundary offsets. Results and individual checks are
  in `paired-scenario-checks.json`.
- Actual scheduled actuation is immediate; deadline enforcement is disabled.
  Prediction-to-plant mismatch, numerical integration and finite sampled geometry
  remain. No continuous-time or robust recursive-safety proof follows from
  these trials. No production algorithm repair or unit-test suite run is claimed.

## Observed runtime

| Scenario | Method | Median / p95 / max attempted frame time (s) | Attempts exceeding 50 ms |
| --- | --- | ---: | ---: |
| `straight_oncoming` | `affineSocp` | 0.036 / 0.301 / 0.410 | 10 / 42 |
| `straight_oncoming` | `nonlinearShooting` | 2.408 / 3.565 / 7.895 | 360 / 360 |
| `circular_oncoming` | `affineSocp` | 0.014 / 0.131 / 0.275 | 10 / 31 |
| `circular_oncoming` | `nonlinearShooting` | 2.382 / 2.985 / 7.099 | 360 / 360 |
| `varying_curvature_oncoming` | `affineSocp` | 5.288 / 5.288 / 5.288 | 1 / 1 |
| `varying_curvature_oncoming` | `nonlinearShooting` | 6.531 / 8.054 / 14.224 | 360 / 360 |

Separate single-threaded processes overlap on a shared workstation. These
times are descriptive and not a controlled speed benchmark. A controller that
completes an offline avoidance trial can still be unsuitable for a 50 ms deadline.

## Execution, validation and reproduction

There are 12 saved scenario results: seven primary affine trials, three
nonlinear trials, and the separate affine S-bend cold/warm pair. The affine
batch and repeat batch exit cleanly. The nonlinear straight result is saved and
independently audited before its original multi-case process is intentionally
stopped to avoid repeating circular/S-bend cases already running in dedicated
processes. SIGINT and SIGTERM did not stop that queue; SIGKILL ends it with
status 137. Its partial redundant circular attempt is excluded, and this
intentional stop is not reported as a numerical controller failure or clean exit.
Dedicated process outcomes are retained in `execution-status.json`.

The seven affine primary MAT files pass the existing campaign's independently
recomputed completion/recovery/SAT accounting. All ten nonempty saved results
also pass the stronger independent physical-state, rectangle, road and recovery
audit. The two zero-hold affine S-bend attempts have no physical trace to audit;
their failure, finite initial state and zero applied-command count are independently
verified in `zero-hold-audit.json`. Matching scenario checks
also pass. These checks validate result accounting, not a general safety guarantee.

```matlab
addpath('scripts');
runControllerRecoveryValidation(outputDirectory, 'vehicle');

settings = struct('solver',struct('method',"nonlinearShooting"), ...
    'nonlinear',struct('horizonSeconds',5,'blockSteps',4, ...
    'maxIterations',12,'timeLimitSeconds',20,'feasibleRefinementIterations',4));
result = runOncomingVehicleAvoidanceScenario(Duration=18,ReferenceSpeed=10, ...
    TargetSpeed=10,RecoveryWindow=2,ControllerConfiguration=settings, ...
    Plot=false,Report=false,Progress=true);
% Replace the driver with runCircularCenterlineStraightTargetAvoidanceScenario
% or runVaryingCurvatureStraightTargetAvoidanceScenario for the other cases.
save(outputFile,'result');
```

The exact launched wrappers, console logs, source snapshot, original MAT files,
audit code copies and generated comparison PNG/PDF remain in:

`/home/zai/.cache/collisionAvoidance/controller-simulation-20260926-185456`

```bash
uv run --with scipy python scripts/auditNonlinearVehicleRecovery.py \
  /path/to/result.mat --output-dir /path/to/audit
```

The vehicle-only primary audit uses the existing
`scripts/auditControllerRecoveryResults.py` imports and vehicle loop, omitting
only its unrelated 15-case declared-model loop; the exact extracted script is
preserved in the run directory. `verifyComparison.py` and `plotComparison.py`
are retained there as well. Independent figures are generated outside the
repository, as required by repository instructions.

The run starts at base commit `299a9eca1257d70ed13cceb04c65f7664e820121` with pre-existing
working-tree modifications, including experimental nonlinear implementation.
`source-manifest.json` records 145 MATLAB files from the controller, config,
estimator and scripts directories, including the executed sources; an external
source archive preserves their exact bytes. Python audit and native artifact
hashes are also recorded. The report commit contains only this report and its
compact result exports; it deliberately excludes pre-existing algorithm,
configuration, observer, scenario, test and documentation changes, solver trees,
generated binaries, large raw results and figures. The source commit alone is
not represented as containing all experimental implementation files.
