# S-curve scenario interface repair — September 25, 2026

The S-curve scenario now supplies the explicit varying-curvature reference and
the target motion contract required by the current controller. The original
`unsupportedReferenceJump` at t=0 and the independently reproduced
`missingPredictionMotion` are resolved. A fresh nonlinear run executes 44 holds
through 2.20 s, including 23 target-visible holds, before a subsequent
`optimizationFailed`. This repair establishes a runnable interface; it does
not establish completed nonlinear avoidance or cruise recovery.

## Implementation

`scripts/runVaryingCurvatureStraightTargetAvoidanceScenario.m` constructs
`geometry.referenceCurve` with a sampled sinusoidal `curvatureProfile`, an
origin and heading, and explicit `constantCurvature` continuation. The bend's
quarter-wave stations are included when inside the configured reference
length. In particular, the bend endpoint is included even when it falls
between regular samples; its curvature and the following straight segment's
curvature are exactly zero.

The reference uses the controller's existing PCHIP curvature interpolation and
integrated pose implementation. Road centerline samples, nominal collision
construction, and baseline evaluation now derive from that same reference.
This replaces independently trapezoidal-integrated road samples and linear
pose interpolation, so the controller and scenario no longer describe slightly
different paths. Existing centerline sampling and road/shoulder dimensions are
preserved. If a configured reference ends inside the bend, its explicit
continuation uses that endpoint's curvature, as required by the interface.

The straight target now has stable identity `synthetic-target-1` and an
`nrmm-motion-v1` contract with curvature maximum 0.05 /m and zero scalar
acceleration maximum, matching the existing straight/circular scenario
convention. Its exact velocity, acceleration, and yaw-rate observations recover
the constructed constant-velocity trajectory. No controller, estimator, solver,
physical plant, or safety constraint was changed.

## Validation

All **23/23** selected MATLAB tests pass, with no failed or incomplete tests:

- Four S-curve interface/geometry tests cover successful initial holds with
  existing shoulder dimensions, collision construction on the controller
  reference, target admission and exact two-second straight propagation, and a
  wavelength of 80.03 m with 0.3 m sample spacing whose endpoint is off grid.
- Eight scheduled-reference controller tests and eleven smooth-reference
  geometry tests continue to pass.

The existing S-curve test previously asserted the interface failure. It now
requires two successful 50 ms holds at the default 15 m/s. A separate 10 m/s,
0.1 s smoke run also completes both holds. Code Analyzer reports no source
issues in the two changed MATLAB files; the only notice is an unreadable old
R2025b settings file, with default analyzer settings used instead.

The original 18 s truth-fed PassVeh14DOF experiment was rerun at 10 m/s ego and
target speeds, 50 m initial target distance, 30 m detection range, 50 ms control
updates, 0.01 /m peak curvature, 80 m wavelength, and a 2 s recovery window.
This exact-sensing run has no stochastic sensor seed. Default road offsets
remain 6/8 m with 2.6 m shoulders; default solver settings are retained.

| Result | Observed value |
| --- | ---: |
| Executed / requested duration | 2.20 / 18 s |
| Completed holds | 44 |
| First target-visible attempt | 1.05 s |
| Accepted target-visible holds | 23 |
| Minimum executed-prefix SAT margin | 1.1046 m |
| Minimum executed-prefix road function margin | 6.7218 |
| Constructed nominal collision time | 2.5086 s |
| SAT margin at constructed nominal collision | -0.5000 m |
| Collision construction position error | 4.44e-16 m |
| Passed target / recovered cruise | No / No |
| Subsequent failure | `collisionAvoidanceController:optimizationFailed` |

The prefix ends before the constructed collision. A positive prefix margin is
not a completed avoidance success, and the baseline's prefix-only collision
flag is false because it also ends early. The collision geometry itself has a
negative SAT margin at the intended collision time. The later optimization
failure needs separate controller/plant investigation; this task does not
claim its cause was isolated or corrected. Successful controller calls before
that failure demonstrate that both repaired interfaces are exercised.

MATLAB R2026a Update 3 was used. Compact test results, simulation summary and
source/result hashes are in `S_CURVE_INTERFACE_FIX_20260925/`. Raw results remain
outside Git in `/home/zai/.cache/collisionAvoidance/s-curve-interface-fix-20260925/`.
The base revision is `be53cd086adcfc653a52fdcf2c3d470574776b0f`; unrelated
pre-existing changes, dependencies, and generated binaries are excluded.

```matlab
addpath('scripts');
results = runtests({'tests/varyingCurvatureShoulderGeometryTest.m', ...
    'tests/scheduledReferenceControllerTest.m', ...
    'tests/smoothReferenceGeometryTest.m'});
assertSuccess(results);
result = runVaryingCurvatureStraightTargetAvoidanceScenario( ...
    Duration=18, ReferenceSpeed=10, TargetSpeed=10, RecoveryWindow=2, ...
    Plot=false, Report=false, Progress=true);
```
