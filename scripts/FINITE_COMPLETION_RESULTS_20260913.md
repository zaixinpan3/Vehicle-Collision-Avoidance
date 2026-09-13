# Finite encounter completion implementation and validation

September 13, 2026. Certificate version 21 implements `N=L=M`: a finite
swept collision/road witness, robust whole-target exterior membership at its
confirmation node, and a target-independent invariant road terminal law.
The ego need not stop at exit. This replaces the mandatory infinite-future
target halfspace. The mathematical and interface scope is specified in
[INFORMATION_STATE_PCBF.md](../controller/INFORMATION_STATE_PCBF.md).

## Behavior and interface changes

`hardEncounterBarrier.completionRows` appends separate finite exit rows to
road-only terminal membership and the terminal input transition. The exit
uses the declared `ego.perception.range`, both propagated information boxes,
chart error and the target rectangle support over its final yaw interval.
The final anchor proposes its direction. Every finite cell remains subject
to collision verification. No finite timeout replaces the required geometric
exit, and no future observer reset is assumed.

Admission requires a current complete observation with positive finite range
and matching time. The range exceeds both body circumradii plus clearance.
A current complete absence, or a conditioned current observation certifying
exterior membership, releases the encounter. Contradictory absence, changed
active range, stale confirmation and a missing deadline confirmation are
rejected. The final observation uses the carried exit direction and chart;
a newly chosen direction can be worse for an anisotropic box.

The absolute active exit deadline cannot be postponed by fresh optimization.
A confirmed removal retains the road-safe suffix even if fresh solving fails.
The terminal law uses the carried nominal box, preserving its certified
first input and slew transition. Both verification modes use analytic
terminal membership when no optimized hold remains; rebuilding a stiff
rest-model tube is unnecessary for this terminal certificate. Stored inputs
and completion data are checked against the verified decision before reuse.

`candidateAccepted` reports numerical candidate acceptance with hard
non-safety rows and soft CLF constraints. `safetyCertified` and `accepted`
require exactly zero accumulated physical collision/road violation.
Positive-value LP/SOCP results remain diagnostics and cannot issue commands.
Zero jerk does not change target model validity to infinity. Metadata now
separates finite target validity, confirmed release and road-tail validity.
The existing zero-residual affine plant and road terminal assumptions remain.
Core source count stays at 20; no default controller/observer tuning, native
kernel, third-party solver, or estimator algorithm is changed.

## MATLAB regression results

MATLAB R2026a Update 3 passes **174/174** cases across these 15 test classes:

- `finiteEncounterCompletionTest` (20 cases), `boundedTargetMotionTest`,
  `visibleTargetLifecycleTest`, `hardEncounterBarrierTest`,
  `freeCompletionTimeTest`, `sparseControllerIntegrationTest`,
  `controllerSourceBudgetTest`;
- `targetObservationConditioningTest`, `targetPredictionTest`,
  `sweptFlowCertificateTest`, `avoidanceSafetyGeometryTest`,
  `continuousTimeClfTest`, `sparseAvoidanceQpTest`,
  `liftedAvoidanceSocpTest`, `collisionAvoidanceControllerConfigTest`.

The final 20-case completion suite passes again through MATLAB MCP after the
explicit direction-norm support scaling and comment/unused-argument cleanup.
It covers two-axis jerk, non-postponed fresh deadlines, both carried-verifier
modes, early and deadline release, absent/stale/changed-range observations,
contradictory absence, an anisotropic exit direction, altered unexecuted
inputs, non-shrinking future uncertainty, and positive safety violations down
to `1e-10`, below solver tolerance. The lifted/condensed and working-set/full
program comparisons pass. This is a focused regression run, not the entire
repository suite.

Initial runs exposed a shadowed MATLAB `error` function and legacy tests
that discarded the now-required scan metadata; those were corrected. An MCP
request timed out after 300 seconds. A subsequent trace showed the old
forced-failure equivalence test spending its allowed 20-second search budget
on each of 18 frames with the frame deadline disabled. Interrupted runs are
not counted as passes. That behavior test now supplies a 0.1-second frame
search deadline, with the same solver-failure premise and command assertions.
The completed broad run initially had 171 passes and two scan-fixture errors;
after correction and the added input-integrity case, all 174 pass.

Factory Code Analyzer has **zero findings in all 13 changed/new MATLAB
files**. `git diff --check` passes. The existing exact Python derivation
check also passes 16 Cartesian semigroup pairs and all 498 finite scalar
policy/disturbance paths; these remain mathematical fixtures, not vehicle
validation.

## Noisy crossing with every fresh solve forced to fail

The reproducible trial uses a 0.1 s sample, 16 initial stages, reference
speed 8 m/s, a 16 m confirmation region and 0.25 m collision clearance.
The road boundaries are at lateral positions -5 and 5 m. Target initial
state is `[15;-4;0;32;0;0;pi/2;0]`. Actual jerk is
`[0.1;0.1]*cos(t)` m/s^3 and actual yaw acceleration is `0.05*cos(t)` rad/s^2,
integrated analytically independently of target prediction. The safety
contract retains their componentwise amplitudes.

Ego measurement bounds in `[px,py,psi,vx,vy,r]` are
`[0.05;0.05;0.005;0.05;0.02;0.005]`; target bounds in
`[px,py,vx,vy,ax,ay,psi,omega]` are
`[0.1;0.1;0.1;0.1;0.05;0.05;0.01;0.01]`, with SI units and angles in radians.
Uniform bounded measurement noise uses seed **20260913**. The declared ego
plant is integrated with its issued held affine exponential. Geometry is
independently sampled at 11 points per hold as an audit of the continuous
certificate, not a substitute for it.

| Result | Measured value |
| --- | ---: |
| Executed holds | 40 |
| Certified commands, including the final unexecuted command | 41 / 41 |
| Forced-failure continuation commands | 40 |
| Admitted exit deadline | 1.6 s |
| Current-observation release | 0.7 s |
| Road terminal law begins | 1.6 s |
| Maximum safety value / descent residual / slew violation | 0 / 0 / 0 |
| Minimum sampled separation margin | 9.275482 m |
| Minimum sampled road margin | 3.785794 m |
| Final longitudinal speed | 0.445648 m/s |

The decreasing final speed is the retained road terminal policy under forced
solver failure; this run does not demonstrate cruise recovery. A separate
24-hold exact crossing regression with fresh optimization completes at
8 m/s and continues beyond the original horizon after release.

```matlab
addpath('scripts');
report = runExactStateRecursiveFeasibilityScenario( ...
    Scenario="crossing", SampleCount=40, FailAfterAdmission=true, ...
    EgoErrorBound=[.05;.05;.005;.05;.02;.005], ...
    TargetErrorBound=[.1;.1;.1;.1;.05;.05;.01;.01], ...
    TargetJerkAmplitude=[.1;.1], TargetYawAccelerationAmplitude=.05, ...
    Seed=20260913, OutputDirectory=fullfile(tempdir,"finite-noisy-crossing"));
assert(report.passed);
```

## Actual NRMM integration remains unresolved

`runDeclaredPlantEstimatorControllerScenario(SampleCount=60,Seed=20260913)`
executes 36 certified target-free holds before the first target publication
at **3.6 s**. It then ends with `certificateSearchLimit` after **15 attempts**
and its three-second admission-search budget. No active-encounter command
is issued and no finite completion is certified. The old
`unboundedTargetSupport` structural gate is absent, but this run does not
establish feasibility of the new finite completion for the actual NRMM boxes.

The diagnostic uses reference speed 10 m/s, target initial distance 100 m,
actual NRMM outputs and the declared affine ego plant. Before rejection its
sampled road/separation margins are 3.787718 m / 22.990563 m. Those values
cover only the executed target-free prefix; they are not evidence of a
completed avoidance maneuver. The last internal conic solve reports
infeasibility for its chosen finite candidate. The limited horizon/direction
search does not prove global or physical avoidance infeasibility.

Global detection/re-entry guarantees, general `L<M` tails, validated nonlinear
plant inclusion, and a worst-case execution-time bound remain outside this
implementation's demonstrated scope. Nonzero confirmation delay is not
supported and cannot be supplied implicitly through a stale scan.

Original logs, MATLAB test objects/CSV, analyzer output, and MAT/JSON trial
outputs are retained at
`/home/zai/.cache/collisionAvoidance/finite-completion-20260913`.
The source and this report are the committed project artifacts; generated
binaries and unrelated working-tree material are excluded.
