# Removing the additional tire-force constraints

Experiment date: September 10, 2026.

Version 14 removes the additional combined-tire-force polygon from the sole
online optimization. The original target-free first-frame contradiction is
eliminated: the same saved physical-scene state and model allowances now
produce an independently certified first control. The complete straight
oncoming experiment still fails subsequent integration gates.

## Implementation and safety scope

The geometry assembler and its MATLAB/native projection kernels no longer
generate axle-force polygon rows. The unused polygon constructor and
`model.frictionPolygonSides` setting are removed. No option restores them.
Certificates use version 14 and reject previous versions. The old diagnostic
that reconstructs the removed polygon is retained in its original Git/archive
record rather than as an alternative current implementation.

The complete nonlinear modified Fiala force law is unchanged. With
`Fx=beta*mu*Fz` and `abs(Fy)<=mu*Fz*sqrt(1-beta^2)`, its combined-force limit
is intrinsic for admitted inputs. Tire-slip domains, model-state domains,
collision and road geometry, actuator limits, input slew, terminal target
exit, and independent certificate verification remain in force. The affine
predictor does not automatically inherit nonlinear saturation; valid model
residual bounds remain a premise. This removal does not prove physical
vehicle enclosure or fix the previously diagnosed road/timing/lifecycle issues.

## Direct comparison with the original failure

The input is the actual September 9 failed first frame from the 100 m
oncoming scenario, with no visible target, 10 m/s ego speed, 0.1 s period,
16 prediction intervals and residual rate bound
`[0.2;0.06;0.02;2.5;5;4]`. Only the obsolete polygon setting is removed when
reading that archived configuration. The uncertainty bounds are unchanged.

The old first frame returned `noCertifiedContinuation`. Version 14 returns
a certified command with positive margin **0.0412952878857**, and zero
`combinedTireForce` rows. Hard row count falls from 43,152 to 21,648: exactly
21,504 force rows are removed. Retained right-hand sides are unchanged; the
largest retained matrix-coefficient difference is 8.8817841970e-16 after
recompiling the smaller projection kernel. Independent `linprog` checks give
exit flags -2 before and 1 after removal. This comparison can be reproduced
with `verifyStraightForceConstraintRemoval`.

## Physical scene reruns

The three runs retain the September 9 scene settings: straight road, ego and
target speeds 10 m/s, target lateral offset 0.8 m, target dimensions 5 by 2 m,
30 m circular perception, 0.1 s control period, 16-stage prediction and the
unchanged empirical residual allowance. The physical plant is PassVeh14DOF
with its MF62 tire model and mapped vehicle parameters. Exact state observations
isolate the controller; the sequential strict gate did not start its joint
NRMM stage.

| Run | Applied intervals | Stopping event | Result |
| --- | ---: | --- | --- |
| Original 100 m scene, 30 s requested, 100 ms deadline enforced | 0 | First computed frame takes 420.540 ms | `runtimeDeadlineExceeded` |
| Original 100 m scene, 30 s requested, deadline disabled | 1 | Road identity changes at 0.1 s | `changedExecutionContract` |
| Direct first detection at 29.9 m longitudinal distance, 10 s requested, deadline disabled | 0 | First complete encounter problem still uncertified | `noCertifiedContinuation` |

The functional original-scene run really advances the plant for one interval;
it is no longer the previous zero-input admission failure. Its two attempted
frames have maximum 412.159 ms and median 210.610 ms. The sampled rectangle
separation margin over that short trace is 92.999912 m, while the target is
still outside perception. This is not an avoidance or target-exit success.
The direct-acquisition failed frame takes 216.404 ms. These are observed
attempt timings, not a platform worst-case execution-time proof.

An additional offline check of the current direct-acquisition problem retains
the full empirical uncertainty. Core constraints without collision or exit
are feasible. Core plus robust collision constraints is infeasible, and core
plus terminal exit constraints is independently infeasible. Thus removing the
force polygon does not resolve the uncertain collision tube or the exit
requirement. This differs from the previous zero-residual diagnostic in which
collision without exit was feasible; the new check retains the full residual
allowance. The fresh road fit also changes the representation of the same
physical road, while the stored witness requires identical road input. That
interface was left unchanged for this experiment.

## Regression validation

Both native geometry kernels rebuilt successfully. Initial focused checks
passed 66/66. Four stale migration expectations were updated to require the
newly feasible short admissions and the precise remaining road-identity stop.
The full repository run then passed 593/594 tests; its sole failure was an
obsolete expectation that the strict physical gate would fail admission.
That gate now computes a command and rejects its missed runtime deadline.
After updating this expectation, all three pipeline deadline tests passed.
Combining the full run with this targeted rerun verifies all 594 tests, with
zero remaining failures or incomplete tests. Original results are preserved
separately from the consolidated result. Factory Code Analyzer reported zero
findings across 13 changed or new MATLAB files.

## Reproduction

After rebuilding the geometry kernels once with
`buildAvoidanceGeometryKernel`, run:

```matlab
addpath('scripts', 'controller', 'config');
comparison = verifyStraightForceConstraintRemoval( ...
    '/path/to/September-9-rerun/exports', '/path/to/output');
strict = runStraightRealtimeValidation(Duration=30, Progress=true);

cfg = finiteSensingValidationConfig();
cfg.controller.sampleTime = 0.1;
cfg.controller.horizonSteps = 16;
functional = runOncomingVehicleAvoidanceScenario( ...
    Duration=30, CenterlineLengthAfter=1000, ...
    ReferenceSpeed=10, TargetSpeed=10, ...
    TargetInitialLongitudinalDistance=100, TargetLateralOffset=0.8, ...
    TargetWidth=2, UseStateEstimator=false, ControllerConfiguration=cfg, ...
    EnforceRuntimeDeadline=false, Plot=false, Report=true);
acquisition = runOncomingVehicleAvoidanceScenario( ...
    Duration=10, CenterlineLengthAfter=1000, ...
    ReferenceSpeed=10, TargetSpeed=10, ...
    TargetInitialLongitudinalDistance=29.9, TargetLateralOffset=0.8, ...
    TargetWidth=2, UseStateEstimator=false, ControllerConfiguration=cfg, ...
    EnforceRuntimeDeadline=false, Plot=false, Report=true);
```

Generated binaries and raw experiment outputs remain outside version control.
The original results and the new outputs are preserved as identified archive
exports; no earlier failure record is rewritten as a success.
