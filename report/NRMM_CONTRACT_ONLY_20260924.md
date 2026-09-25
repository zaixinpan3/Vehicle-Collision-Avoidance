# Removal of the Cartesian jerk contract, 2026-09-24

Until this change a target could be declared under two motion contracts:
`nrmm-motion-v1` (exact NRMM motion: constant speed-rate and constant
sideslip, so a constant-curvature path through the estimate) and
`finite-sensing-motion-v1` (Cartesian motion with a declared jerk bound `J`
and yaw-acceleration bound `H`, enclosed by constant-acceleration
extrapolation plus `J t^3/6`). The user chose to remove the second
("那么我们选择3."). `nrmm-motion-v1` is now the only target motion contract.

## What changed

**Controller** (`controller/`).
- `targetPrediction`: `admitOnline`/`admit` accept `nrmm-motion-v1` only.
  `curvatureMaximum` is required; `speedRateMaximum` and
  `scalarAccelerationMaximum` are optional; a `jerkBound` or
  `yawAccelerationBound` field must be zero if present. A target without a
  contract raises `missingPredictionMotion` (before, it received the Cartesian
  contract with zero jerk); the `exact-motion-v1` alias is gone. Removed:
  `isFiniteSensing`, the constant-acceleration branch of `finiteFlow` with its
  jerk terms and the declared-maximum cap (`localCappedDeviation`),
  `accelerationDeviationBound`, the initial-box branch of `deviationModel`
  with the per-hold fields `holdBound`/`holdJerk`, the finite-sensing seeding
  of the nominal center in `localEncounter`, and the NRMM guards
  (`localIsNrmm`; conditioning of the parameter intervals is unconditional).
  The offline `advance` adopts a tighter or equal contract and raises
  `changedEncounterContract` when a declared maximum grows (an undeclared
  maximum counts as infinite), the rule `hardEncounterBarrier` applies
  online; `admitOnline` only renames an anonymous target and delegates the
  validation to `admit`.
- `ltvBicycleModel.reactionGains`/`reactiveTube`: the per-hold acceleration
  part of the target deviation (mean deviation, hold remainder, `jerk*h/2` in
  the acceleration noise) is removed; the target deviation is the NRMM
  parameter generators plus the node remainder.
- `hardEncounterBarrier`: the re-admission test drops the jerk,
  yaw-acceleration and kind terms; a larger curvature, speed-rate or
  acceleration maximum still voids the carried family.
- `avoidanceSafetyGeometry` (`localCellRows`, shared by MATLAB and the
  MATLAB Coder kernel): the target is evaluated at the node's own box; the
  Cartesian whole-hold target block (constant-acceleration Bernstein
  polynomial with `jerkBound/6`) and its `finiteFlow` calls are gone, a
  whole-hold cell with a target raises `unsupportedWholeHoldTarget` in both
  implementations, and the kernel's target struct lost its `contract`
  placeholder (`buildAvoidanceGeometryKernel` updated, kernels rebuilt).
- `readPlanningInputs`: `targetPredictionYawAccelerationErrorBound` and
  `predictionYawAccelerationErrorBound` are no longer parsed.

**Estimator.** `nrmmControllerErrorBounds` publishes the NRMM contract
(`curvatureMaximum`, `speedRateMaximum`, `scalarAccelerationMaximum`) and the
parameter error bounds when `modelJerkMaximum == 0`, and an empty
`predictionMotion` otherwise: motion outside every NRMM path has no contract,
and the controller refuses the target at admission.

**Scenario harness and campaigns** (`scripts/`).
- `runExactStateRecursiveFeasibilityScenario`: the truth target is always an
  exact NRMM motion (`nrmmTargetTruth`), measured with box noise for position,
  yaw and yaw rate and disc noise for velocity and acceleration, and declared
  under `nrmm-motion-v1` (`nrmmTargetMeasurement`, both new shared
  functions). The options `TargetJerkAmplitude`,
  `TargetYawAccelerationAmplitude`, `TargetMotionFrequency`,
  `TargetMotionModel` and `NrmmContract` are removed; `TargetSpeedRate`,
  `TargetCurvature`, `TargetCurvatureMaximum` (0.05 1/m),
  `TargetAccelerationMaximum` and `NrmmParameterBounds` remain.
- `runDeclaredPlantFailureSweep`: group B (uncertainty scales) no longer adds
  a truth jerk; the groups `C-targetJerk` (6 cases) and `C-targetYawAcc`
  (4 cases) are replaced by `C-targetCurvature` (oncoming and crossing at
  0.005, 0.01, 0.02 1/m) and `C-targetSpeedRate` (at -0.5, 0.5, 1.0 m/s^2);
  120 cases. Columns `targetCurvature`/`targetSpeedRate` replace
  `targetJerk`/`targetYawAcc`.
- `runRecursiveSafetyValidation`: the uncertain crossing case has no truth
  jerk. `runNrmmTargetCampaign`: the `cartesian` variant is gone (three
  variants). `runTargetMotionCampaign` (truths that violated the NRMM model
  by construction) is deleted.
- `analyzeSingleHoldInfeasibility` replays the recorded measurements with
  `nrmmTargetMeasurement` and collapses the parameter intervals for its exact
  counterfactual; `profileMatlabControllerRuntime` rebuilds the recorded
  scene with the same function; the diagnostic and probe scripts declare the
  NRMM contract (`probeTargetBoundSensitivity` option `CurvatureMaximum`
  replaces `JerkBound`; `probeEstimatorAdmission` records `curvatureMaximum`);
  the two PassVeh14DOF avoidance scripts declare
  `nrmm-motion-v1` with `curvatureMaximum 0.05` for their truth targets.

**Tests.** Fixtures declare `nrmm-motion-v1` (`encounterTestFixture`, with
the shared NRMM lead encounter `nrmmLead`; `nrmmTruthFixture` holds the
sampled NRMM truths, which stay inside the encounter's parameter intervals
and whose states come from the harness's `nrmmTargetTruth`, and the
closed-loop sampled-excess check used by `nrmmTargetPredictionTest` and
`targetReactionTest`). New tests: a target without a contract (empty or
absent) and a foreign contract kind are refused; a nonzero `jerkBound` or
`yawAccelerationBound` is refused while zero fields are accepted; a declared
`speedRateMaximum` clips the parameter interval and shrinks the box while
capped truths stay inside; a tighter continuation contract is adopted and a
larger one refused; the enclosure tests assert finite boxes and remainders.
Removed: the Cartesian
bounded-flow tests (`boundedTargetMotionTest`, two tests of
`sweptFlowCertificateTest`, one of `targetPredictionTest`), the two capped
Cartesian reachability tests and the jerk simulator of `targetReactionTest`,
the Cartesian comparison and the Cartesian-switch check of
`nrmmTargetPredictionTest`. `nrmmControllerErrorBoundsTest` checks that a
model error publishes no contract.

**Documents.** `TARGET_PREDICTION_CONTRACT.md` describes the single contract;
`CLOSED_LOOP_TARGET_PREDICTION.md` (sections 1 to 6 marked historical,
section 8 added), `CONTROLLER_FILES.md`, `ESTIMATOR_BOUND_INTERFACE.md`,
`NODE_SAMPLED_CERTIFICATE.md`, `TERMINAL_CBF_PROOF.md` and
`JOINT_SUPPORT_CERTIFICATES.md` are updated. An independent static review
of the change by six reviewers (one per dimension: admission, reactive tube,
geometry kernel, estimator consumers, harness, tests and documents) produced
28 findings, none of them a defect of the executed certificate: the stale
document sentences above, the help texts of `reactiveTube` and
`localCellRows`, the missing tests listed above, the offline diagnostics
`analyzeForceFreeStraightFeasibility` and `probeEstimatorAdmission` and the
recorded rerun helper `firstDetectionBounds.m` still reading removed fields,
and the inconsistency between the offline `advance` (any change refused) and
the online re-admission rule (only an increase). All were corrected before
the commit.

**Environment.** MATLAB R2026a, `matlab -batch`; a clean worktree of
`daab2e7` plus the change; up to seven MATLAB processes in parallel (another
session's runs included), so frame times are not isolated measurements. Raw
outputs are under the git-ignored `simulation_output/nrmm_contract_only_20260924/`.

## 1. Tests

On the final contents (rebuilt kernels included), `runtests('tests')` ran
814 tests: 812 passed, 0 failed and 2 incomplete (the curb-detection tests
skipped by assumption when the untracked LiDAR dataset is absent). A first
full run before the last two fixes had 20 failures, all from two causes that
the fixes address: the remote stationary fixture `encounterTestFixture.stationaryTarget`
had no contract (11 tests, now refused by design and given the NRMM contract),
and the interpreted geometry kernel still called `targetPrediction.finiteFlow`
on the kernel's target data (9 tests). The sampled containment tests of
`nrmmTargetPredictionTest` and the reactive-tube tests of `targetReactionTest`
(now on the NRMM lead encounter, reaction strength 30 accepted at admission)
pass; the new tests cover a missing contract, a nonzero jerk field, a
whole-hold cell with a target in both kernel implementations, and an
estimator model error publishing no contract.

## 2. Declared-plant campaigns

All campaigns ran on this worktree with the deadline disabled; the reference
records are the 2026-09-24 reports of `3073d57` (NRMM campaign, recursion,
estimator bound) and the other session's `daab2e7` sweep
([declaredPlantFailureSweep_coverage.csv](TERMINAL_LATERAL_CLEARANCE_20260924/declaredPlantFailureSweep_coverage.csv)).

### 2.1 NRMM closed-loop campaign (`runNrmmTargetCampaign`, 30 cases x 3 variants)

| variant | passed | failures |
| --- | --- | --- |
| nrmm-boxOnly | 26 / 30 | crossing k=0.01 x10 at t=0 (kT=0 and kT=0.02); oncoming k=0 x10 at t=2.55 s; oncoming k=0.01 x10 at t=2.5 s |
| nrmm-egoOnly | 29 / 30 | oncoming k=0.01 x10 at t=2.5 s |
| nrmm-reactive | 29 / 30 | oncoming k=0.01 x10 at t=2.5 s |

The same counts and the same failing cases as the `3073d57` record; the
`cartesian` variant no longer exists.

### 2.2 Recursion and estimator-bound campaigns

`runRecursiveSafetyValidation`: 5 of 5 cases complete 600 holds (stationary
gap 9.3e-5 m, oncoming 0.038 m, crossing 9.52 m, cruise, uncertain crossing
9.52 m); the 50 ms deadline runs end at 0, 52 and 120 of 120 holds, as
recorded. `runEstimatorBoundCampaign`: 8 of 8 (the recorded estimator ego
bound on every scenario and curvature).

### 2.3 Failure sweep (`runDeclaredPlantFailureSweep`, 120 cases)

108 of 120 pass: 3 controller errors and 9 silent inter-node overlaps, all
in the groups that existed before. The new groups pass completely:
`C-targetCurvature` 6 of 6 (oncoming and crossing NRMM truths at 0.005,
0.01, 0.02 1/m) and `C-targetSpeedRate` 6 of 6 (-0.5, 0.5, 1.0 m/s^2).

Against the `daab2e7` record (107 of 118), 91 of the 108 rows outside the C
groups agree in outcome, executed holds and body gaps to within 1e-6
relative; the 17 differences are all in group B, whose inputs changed (the
truth jerk `0.02 x scale` is gone and the contract is NRMM):

| B case (scenario, road k, scale) | `daab2e7`, truth jerk 0.02x (record) | `daab2e7`, constant velocity, Cartesian zero-jerk contract | this change, NRMM contract |
| --- | --- | --- | --- |
| stationary, 0.01, x3 | pass, node gap 0.39 m | fails at hold 21 | fails at hold 14 |
| oncoming, 0.01, x10 | fails at hold 60 | pass, 0.62 m | fails at hold 50 |
| crossing, 0.01, x10 | fails at t=0 | fails at t=0 | fails at t=0 |
| the other 15 | pass | pass | pass (gaps differ by up to 1.4 m) |

The middle column is a run of the pre-change code on the same
constant-velocity truths (`simulation_output/.../baseGroupB/`): the two
outcome flips are sensitive to the exact trajectory (the curved stationary
case with three-fold errors fails under the old contract as well once the
truth jerk is removed, and the ten-fold curved oncoming case passes there),
so neither is attributable to the contract alone.

### 2.4 PassVeh14DOF and estimator-in-the-loop scenarios

On this worktree and on a pristine `daab2e7` worktree alike:
`runOncomingVehicleAvoidanceScenario` with truth targets ends at t=1.4 s
(`optimizationFailed`, Clarabel status 2), and both scenarios of
`runEstimatedStateAvoidanceScenarios` stop at the first control step with
`nonexactStudyInput` before any target is admitted. Neither outcome changes
with this commit; the 2026-09-24 scenario rerun report measured these
scenarios on the other session's uncommitted controller edits, which are not
in `daab2e7`.

### 2.5 Kernel rebuild

The geometry kernels were regenerated with `buildAvoidanceGeometryKernel`
(the MEX under the excluded `solver/bicycle` must be rebuilt wherever the
tree is checked out). The rebuilt kernel reproduces the recorded
`crossing k=0 kT=0 x1 nrmm-egoOnly` run of section 2.1 bitwise (inputs and
states of all 240 holds).

## 3. Limits

- The removal is a scope decision, not a measured improvement: a target
  whose speed-rate or curvature varies now has no contract at all. The
  estimator's `modelJerkMaximum` is zero in the shipped configuration, so
  nothing published changes for it; a nonzero model error would stop the
  controller at admission instead of enlarging the box.
- The failure sweep's new C groups are new cases; their outcomes are
  recorded, not compared against the jerk groups they replace.
- Recorded runs from before this change (reports of 2026-09-22 to
  2026-09-24) that used `TargetJerkAmplitude` or the `cartesian` variant can
  no longer be reproduced with the current scripts.
