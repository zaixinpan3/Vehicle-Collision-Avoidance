# Independent straight-road rerun of the information-state controller

September 12, 2026. Source commit `e83df709c736c143b8bb2ce19241d8cd797282b7` (certificate version 19).
This rerun executes the current controller without changing its algorithm,
configuration defaults, native kernels or existing tests. It is distinct from
the earlier implementation campaign in
[INFORMATION_STATE_PCBF_RESULTS_20260912.md](INFORMATION_STATE_PCBF_RESULTS_20260912.md).

## Scope and changes under test

The new version conditions carried ego and target information boxes by their
measurement boxes, verifies the shifted previous plan using its carried stage
models, charts, normals and terminal set, and accepts a fresh plan only within
the carried safety-value budget. Fresh search compares shifted and equilibrium
initializations by safety value and CLF/input objective.

The implementation also restores execution of the verified carried witness
when a fresh solve fails or exceeds its safety-value budget. After all optimized
stages have been consumed, the terminal law can become the actual command.
That is a material change from version 18 and differs from the earlier requested
fail-fast policy. This rerun records the behavior as implemented; it does not
silently delete it. The metadata field `fallbackUsed` alone is insufficient to
identify that behavior because it remains false even for carried commands.
The independent summary counts `candidateExecuted`, `certificateSource` and
`terminalActive` instead. Existing regression tests separately exercise forced
failure and terminal-law execution.

## Experiment definition

Three sequential runs use oncoming, crossing and stationary targets, 300 holds
(30 s), a 100 ms hold, 16-step horizon, reference speed 8 m/s, straight road
boundaries at y = +/-5 m and 0.25 m rectangle clearance. Ego and target
footprints are 4.8 by 1.9 m. Target position/velocity are (60,0)/(-8,0),
(15,-4)/(0,32) and (15,0)/(0,0), in meters and m/s. Exactly one target persists.
Ego and target measurement error boxes are zero; the explicit RNG seed is
20260912 and no nonzero noise is applied. No solver failure is injected in
these three experiments.

The true ego state follows the accepted first-stage affine generator with an
independent `expm` propagation. Eleven samples per hold audit footprint
separation and road containment. The plant waits for every solver call, so
missed wall-clock deadlines are measured without simulating delayed actuation.
This is an exact declared-model controller study, not an estimator-in-the-loop
or nonlinear-vehicle validation. Zero-box runs do not validate noisy box
conditioning by themselves.

## Outcomes

| Scene | Completed holds | Certified calls | Final speed (m/s) | Final lateral error (m) | Final heading error (deg) | Extra separation (m) | Extra road margin (m) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| oncoming | 300 | 301/301 | 8.00000005 | 1.09228e-15 | -1.24099e-14 | 0.01481039 | 1.463511 |
| stationary | 300 | 301/301 | 4.13094208e-05 | 2.85714e-12 | 8.0606e-11 | 0.002395642 | 3.8 |
| crossing | 300 | 301/301 | 8.00000004 | -3.06077e-08 | 2.70561e-08 | 9.269858 | 3.8 |

Both geometric margins have already deducted 0.25 m. The last computed command
in each 301-call trial is not executed. Sampled positive margins supplement the
per-frame certificate; they are not an independent continuous-time proof.

| Scene | Verified successor witnesses | Maximum PCBF value | Maximum descent residual | Carried commands | Terminal-law commands | Maximum slew violation |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| oncoming | 300/300 | 0 | 0 | 0 | 0 | 0 |
| stationary | 300/300 | 0 | 0 | 0 | 0 | 0 |
| crossing | 300/300 | 0 | 0 | 0 | 0 | 0 |

The descent residual is the accepted current value minus the carried successor
value, as computed by the controller. A zero value and nonpositive residual
check the executed instances, not every state in the claimed invariant domain.
The new design note claims recursion under its declared model/measurement
premises; this rerun is numerical evidence, not a new audit or proof of that
claim.

| Scene | Maximum speed error in last 5 s (m/s) | Maximum lateral error in last 5 s (m) | Maximum heading error in last 5 s (deg) |
| --- | ---: | ---: | ---: |
| oncoming | 5.346263e-08 | 1.367467e-12 | 8.182506e-12 |
| stationary | 7.999962 | 7.470873e-11 | 2.734498e-09 |
| crossing | 4.345942e-08 | 6.223882e-08 | 2.516755e-07 |

The oncoming and crossing cases recover or maintain the 8 m/s cruise state.
The stationary-obstacle case still ends stopped behind the obstacle; successful
passing and subsequent cruise recovery are not achieved. With no carried or
terminal commands in normal operation, that stop comes from the repeated fresh
optimization, not terminal takeover. No conclusion that an alternative physical
maneuver is impossible follows from this experiment.

## Runtime

| Scene | First frame (ms) | Median frame (ms) | Maximum frame (ms) | Calls above 100 ms | Median successor witness check (ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| oncoming | 1317.701 | 852.250 | 2210.527 | 301/301 | 100.907 |
| stationary | 1905.848 | 13874.207 | 17857.042 | 301/301 | 2448.508 |
| crossing | 326.512 | 950.653 | 2078.219 | 301/301 | 135.296 |

All three trials fail the 100 ms deadline and are not real-time qualified.
Frame timing includes input assembly and the controller, and excludes independent
plant/geometry audits and file writing. Trials run sequentially; regression
tests run afterward so this task does not add concurrent MATLAB test load.
These measurements are workstation observations, not WCET bounds.

Mean instrumented times (ms) expose the cost of certificate verification and
the fresh problem. Other preparation/orchestration is included in whole-frame
time, so the following columns need not sum to it.

| Scene | Prediction | Formulation | Solve | Acceptance/commit | Carried witness verification |
| --- | ---: | ---: | ---: | ---: | ---: |
| oncoming | 57.419 | 218.584 | 488.091 | 7.017 | 139.605 |
| stationary | 381.119 | 4600.425 | 6123.379 | 78.176 | 2394.667 |
| crossing | 80.233 | 248.978 | 466.330 | 7.475 | 165.442 |

The stationary low-speed case remains particularly expensive. Version 19
adds a verified carried candidate and generally two fresh initializations per
frame. The previously identified low-speed tube-cell growth is still present;
this rerun does not alter that algorithm or establish a new optimization result.

## Regression and reproduction

Existing focused regression: **71/71 passed, 0 failed, 0 incomplete**.
The five classes are `collisionAvoidanceControllerTest`,
`hardEncounterBarrierTest`, `cruiseRecoveryTest`, `freeCompletionTimeTest` and
`encounterCertificateScenarioTest`, executed through MATLAB MCP after the
scenario batch. They cover public control behavior, information-set and terminal
checks, CLF/input costs, rolling cruise and the current carried-witness policy.
No full-suite result from the earlier implementation campaign is presented as
having been rerun here. `git diff --check` validates the new result document;
no controller or test source is changed.

```matlab
addpath('controller','config','scripts','tests', ...
    'solver/bicycle','solver/clarabel/matlab');
outputDirectory = fullfile(tempdir,'straight-controller-rerun');
for scenario = ["oncoming","crossing","stationary"]
    report = runExactStateRecursiveFeasibilityScenario(Scenario=scenario, ...
        SampleCount=300,FailAfterAdmission=false,DeadlineSeconds=0.1, ...
        EgoErrorBound=zeros(6,1),TargetErrorBound=zeros(8,1),Seed=20260912, ...
        OutputDirectory=outputDirectory);
end
```

MATLAB R2026a Update 3 and the already installed native kernels are used.
Original MAT/JSON results, progress checkpoints, batch/regression logs,
`summary.json`, `straight-rerun.pdf`/PNG, source provenance and reproduction
scripts are retained outside the repository:

`/home/zai/.cache/collisionAvoidance/straight-rerun-20260912`

Archive copies are identified as exports; source/model scope and the remaining
stationary-maneuver, runtime and physical-model limitations are retained.
