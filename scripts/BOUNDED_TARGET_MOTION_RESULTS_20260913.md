# Bounded target motion: admission fix and terminal limitations

September 13, 2026. The previous `nonexactStudyInput` failure was an online
admission rule, not a solver proof of infeasibility. Nonzero jerk and yaw
acceleration are now admitted and propagated in the existing PCBF/CLF
controller. This is a partial compatibility improvement: the actual NRMM
joint encounter still cannot obtain the current all-future terminal
certificate. The observer algorithm is unchanged.

## Implemented behavior

`targetPrediction.admitOnline` replaces `admitExact`. It preserves supplied
finite derivative bounds, normalizes supported motion descriptors through
one validator, and retains the zero-bound case. `condition` intersects each
measurement with the carried bounded reachable box. Acceleration and yaw
rate may change inside the bounds. Inconsistent observations and increased
future bounds cannot silently validate an old witness. Larger future bounds
trigger fresh admission after the original hold and measurement checks,
rather than a direct rejection. Successful readmission continues control
with the enlarged contract and no cross-contract descent comparison. Failed
readmission cannot execute the old smaller-bound witness.

The existing finite swept prediction already propagates the current-state
box and the integrated disturbance radii:

```
r_p(t) = r_p + t*r_v + t^2*r_a/2 + t^3*J/6
r_v(t) = r_v + t*r_a + t^2*J/2
r_a(t) = r_a + t*J
r_psi(t) = r_psi + t*r_omega + t^2*H/2
r_omega(t) = r_omega + t*H
```

The terminal calculation now checks projected jerk as well. For separating
normal `n`, its target support has cubic coefficient `abs(n)'*J/6`. A positive
coefficient makes the all-future support unbounded, regardless of receding
velocity or acceleration. Cartesian axis candidates are included for nonzero
jerk, so a direction unaffected by that jerk is not omitted on a rotated
road. Nonzero yaw acceleration uses the target's circumcircle at the terminal
set, including a zero current yaw-rate box. No small positive bound is rounded
to zero. Metadata reports the actual contracts and does not call a nonzero
motion-bound certificate exact.

The information-set monotonicity argument extends through the nonnegative
Cartesian transition matrix and integrated-disturbance semigroup. The
PCBF safety-value LP, CLF/input objective, ego model, observer gains and
terminal braking feedback are unchanged. The terminal invariant-set premise
remains necessary for the shifted-candidate argument used in
[Huang et al., Section III](https://arxiv.org/html/2502.08400v2#S3).
The bounded information-set extension and directional support derivation
here are project-specific; that paper alone does not prove this uncertain
driving implementation safe.

## Independently generated changing target motion

`runExactStateRecursiveFeasibilityScenario` now accepts optional sinusoidal
jerk/yaw-acceleration amplitudes and frequency. Its target truth is integrated
analytically, independently of the controller's reachable-set code:
`j(t)=[0.1,0]'*cos(t)` m/s^3 and `yawAcceleration(t)=0.05*cos(t)` rad/s^2.
The supplied bounds are `J=[0.1,0]'` and `H=0.05`, with zero current-state
measurement errors. Thus actual acceleration and yaw rate vary throughout
the runs. Zero amplitudes retain the original exact-motion scenes.

Both trials execute 300 holds (30 s), h=0.1 s, requested cruise 8 m/s,
configured horizon 16/minimum 2, straight road boundaries at y=+/-5 m,
clearance 0.25 m, seed 20260912. The crossing target starts at (15,-4) m with
velocity (0,32) m/s; the oncoming target starts at (60,0) m with velocity
(-8,0) m/s. These controller-only drivers retain the target throughout the
run; they do not simulate finite sensor release. Ego truth is independently
integrated by `expm` using the accepted first-hold affine generator. Eleven
geometry samples per hold provide an independent numerical audit.

| Trial | Completed holds | Safety value, initial / maximum | Minimum vehicle margin (m) | Minimum road margin (m) | Final speed (m/s) | Safety audit |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Crossing | 300 | 0 / 0 | 9.275004 | 3.800000 | 8.00000003 | Pass |
| Oncoming | 300 | 3.462904 / 3.462904 | 1.291070 | -0.039760 | 8.60592959 | Fail |

Both produce 301 certified program decisions and verify all 300 successor
candidates with maximum descent residual zero. Crossing uses no carried
commands; oncoming uses 10. Program feasibility and value descent are not
synonyms for collision/road safety from a positive initial value. The
oncoming trial starts with positive safety slack, and its road clearance
audit fails. Its final lateral error is 3.644872 m; it does not return to
along-path cruise. It must not be reported as a successful avoidance-and-
recovery trial. This persistent-target experiment also does not test recovery
after confirmed sensing departure. The crossing final lateral/heading errors
are approximately 3.03e-7 m / 6.49e-9 rad.

| Trial | Median / maximum frame (ms) | Frames over 100 ms |
| --- | ---: | ---: |
| Crossing | 70.078 / 101.884 | 1/301 |
| Oncoming | 84.106 / 1650.133 | 24/301 |

Timing includes input assembly and controller, and excludes plant integration
and independent audits. These trials have no estimator or road fitting.
Simulation waits for computation rather than applying a delayed command.
Neither trial satisfies the user's every-frame 100 ms requirement.

## Actual estimator plus controller rerun

The unchanged `runDeclaredPlantEstimatorControllerScenario` is rerun with
60 requested holds, default 100 m initial target distance, 30 m radar,
10 m/s ego and target speeds, 0.1 s control period, horizon 16/minimum 2,
seed 20260913, configured bounded sensor noise and actual NRMM output boxes.
The observer runs at 80 Hz with one RK4 step per sensor sample and its
integration-defect accounting. The road is fixed; ego truth follows the
independently integrated declared affine plant.

It executes 36 no-target holds. At the first target publication (3.6 s),
the nonzero motion contract is admitted (`J=[0.3830499211,0.3830499211]`
m/s^3, `H=0.02499989583` rad/s^2), but terminal construction reports
`unboundedTargetSupport`. No old no-target command is used after that failed
admission. This is not completed joint avoidance. The error now identifies
the sufficient terminal construction that cannot certify the input; it does
not claim a finite-encounter optimizer proved every control infeasible.

For any strictly positive two-axis jerk box, every nonzero direction has a
positive cubic support coefficient. Even an arbitrarily small such box
therefore defeats this terminal family. Current-state acceleration boxes can
independently have the same effect through the quadratic term. This explains
why nonzero disturbances need not prevent ordinary finite-horizon control,
while this implementation can still fail to obtain its recursive certificate.
It does not establish observer divergence or incorrect observed states.
Joint median/maximum frame times are 67.859/151.326 ms with one of 37 calls
over 100 ms, including the failed admission in the measurements. These
figures are not successful avoidance-pipeline timing.

A terminal construction tied to a certified finite encounter and perception
departure, or other justified motion structure that bounds the relevant
support, remains necessary for the general joint case. A nominal prediction
leaving range is not by itself proof that the entire target information set
has left. Such an extension is not implemented or claimed in this change.
The independent nonlinear-plant residual restriction also remains.

## Reproducibility and validation

```matlab
addpath('scripts');
runExactStateRecursiveFeasibilityScenario(Scenario="crossing",SampleCount=300, ...
    TargetJerkAmplitude=[0.1;0],TargetYawAccelerationAmplitude=0.05, ...
    OutputDirectory=fullfile(tempdir,'bounded-crossing'));
runExactStateRecursiveFeasibilityScenario(Scenario="oncoming",SampleCount=300, ...
    TargetJerkAmplitude=[0.1;0],TargetYawAccelerationAmplitude=0.05, ...
    OutputDirectory=fullfile(tempdir,'bounded-oncoming'));
runDeclaredPlantEstimatorControllerScenario(SampleCount=60, ...
    OutputDirectory=fullfile(tempdir,'bounded-joint'));
```

Original MAT/JSON traces and validation output:
`/home/zai/.cache/collisionAvoidance/bounded-target-20260913`.

The final ten-case `boundedTargetMotionTest` passes. It covers bounded
admission, actual changing motion, outside-bound observation rejection,
successful contract enlargement, failed readmission without old-witness
reuse, preservation of the past-bound check, smaller bounds, the tiny
positive two-axis terminal limitation, independent trajectory containment,
and a 22-hold forced-fresh-failure run through the terminal tail. That run
verifies all candidates, has zero value/descent residual and positive road
and vehicle margins. An initial dimension mismatch in the new analytic truth
helper was corrected before the successful test and scenario runs.

Eight relevant test files initially run through MATLAB MCP: 95 of 97 cases
pass. Two existing cases still expect the removed exact-motion rejection.
They are updated to require fresh admission for enlarged jerk/yaw bounds.
After implementing that behavior, the ten bounded-motion cases, eight
visibility-lifecycle cases and those two controller cases all pass (20/20).
This is a focused regression campaign, not a complete repository test run.
The first MCP request times out after 300 s, but MATLAB completes and saves
the results, which are retrieved without launching a duplicate batch.

Code Analyzer checks the seven changed/new MATLAB sources. Its only messages
concern a pre-existing missing R2025b settings file; it explicitly uses
default settings and reports no source findings. `git diff --check` passes.
Core source count remains 20; no observer, default configuration, native
kernel, objective weight or third-party solver files are changed.
