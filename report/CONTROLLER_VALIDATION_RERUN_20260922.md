# Controller validation rerun, 2026-09-22

Purpose: re-execute the controller simulation experiments to confirm whether the
controller runs as designed in the current working tree. This record reports
measured outcomes only; it establishes no real-time or physical-safety guarantee.

Environment: MATLAB R2026a, `matlab -batch`, repository `main` at
`4f95a68977e052d5dd484b6e33b91be836cd4b46` plus the harness correction described
below. Each scenario ran in its own fresh MATLAB process. Two unrelated MATLAB
desktop sessions belonging to other projects were running on the same machine and
were left untouched, so the frame timings below are not isolated measurements.

## 1. Harness correction required before any scenario could run

`scripts/runCenterlineCruiseScenario.m` read `planningProblem.qp.geometry.label`
in `localControllerRoadAudit`. `collisionAvoidanceController` publishes that
struct under the field `program` (`collisionAvoidanceController.m:165`), so every
scenario routed through this driver aborted with `Unrecognized field name "qp"`
before producing any result. The two references were corrected to
`planningProblem.program.geometry.label`. No controller, configuration or
algorithm file was modified.

The same stale field name survives in `scripts/analyzeClfRecoveryMechanisms.m`,
`scripts/verifyStraightForceConstraintRemoval.m` and
`scripts/evaluateCertifiedPrefixCost.m`. Those are one-off analysis scripts
outside the automated set and were left unchanged; they also reference further
fields (`stageProgram`, `inequalityMatrix`, `barrier.baseBound`) that the current
`program` struct does not define, so a field rename alone would not repair them.

## 2. High-fidelity PassVeh14DOF scenarios: 4 of 4 fail

The four scenarios listed in `CLAUDE.md` as "the active automated set" were run
with `Plot=false`, `Report=true` and otherwise default options.

| Scenario | Completed steps | Failure | First-frame solve |
| --- | --- | --- | --- |
| `runStraightCenterlineCruiseScenario` | 1 / 80 | `inconsistentObservation` at t = 0.050 s | 622.6 ms |
| `runCircularCenterlineCruiseScenario` | 1 / 80 | `inconsistentObservation` at t = 0.050 s | 582.2 ms |
| `runOncomingVehicleAvoidanceScenario` | 0 | `optimizationFailed` at control step 1 | 382.1 ms |
| `runCircularCenterlineStraightTargetAvoidanceScenario` | 0 | `optimizationFailed` at control step 1 | n/a |

All four return `passed = 0`. Note that the MATLAB process still exits with code
0 in these runs, because the scenario functions report the failure in the result
struct rather than rethrowing; exit status is not a pass signal here.

Tracking quality over the one completed cruise step was accurate: maximum
absolute speed error 0.000980 m/s (straight) and 0.001062 m/s (circular),
maximum lateral tracking error 0.000000 m and 0.001551 m respectively.

### 2.1 Why the cruise scenarios stop at the second sample

`collisionAvoidanceController.m:36-38` rejects any nonzero
`cfg.model.ltvModelErrorRateBound` or `cfg.model.plantModelResidualRateBound`:
the controller accepts only the declared zero-residual held affine plant.
Consistently with that contract, the published successor box is propagated by
`prediction.initialErrorBound(:,stage+1) = abs(A)*prediction.initialErrorBound(:,stage)`
(`ltvBicycleModel.m:589`) with no process-reserve term, even though a
`processReserve` term is added to `domainErrorBound` and `executionReserve` on the
adjacent lines.

`runCenterlineCruiseScenario` supplies no `controllerStateErrorBound` when the
state estimator is disabled, so `readPlanningInputs` defaults the measurement
radius to zeros. Both radii entering `localConditionBox`
(`hardEncounterBarrier.m:453-467`) are therefore identically zero, and the
intersection test reduces to requiring the measured ego state to equal the
one-step LTV-bicycle prediction within a `256*eps` allowance. The plant is the
14-DOF `PassVeh14DOF` Simulink model, whose one-sample deviation from the affine
prediction exceeds that allowance by many orders of magnitude, so
`collisionAvoidanceController:inconsistentObservation` is raised at the first
sample that has a stored prediction to condition against. This is a consequence
of the declared-exact-plant contract, not an isolated defect.

### 2.2 Why the avoidance scenarios stop at the first frame

Both avoidance scenarios fail with
`collisionAvoidanceController:optimizationFailed` and the message "The recursive
cruise certificate requires an unbounded road-free reference domain"
(`hardEncounterBarrier.m:492-494`; the scheduled terminal certificate carries the
same rejection at `hardEncounterBarrier.m:571-573`). Both scenarios pass
`RoadBoundaryOffsets`, and the current terminal certificate admits no road
boundary at all. This matches the limitation already recorded in
`report/README.md`.

The repository's own `tests/straightCenterlineCruiseScenarioTest.m` asserts
`result.failure.occurred` for this scenario rather than a successful run, so the
`CLAUDE.md` command list describes an expectation the current tree does not hold.

## 3. Declared held affine plant: 5 of 5 cases pass

`runRecursiveSafetyValidation` is the validation entry point matching the
controller's declared contract. Executed at default scale (600 holds per case,
0.05 s sample time, seed 20260912, road boundaries disabled).

| Case | Holds | Min separation margin | Min terminal margin | Max CLF residual | Median / max frame | Deadline misses |
| --- | --- | --- | --- | --- | --- | --- |
| stationary | 600 / 600 | 9.2775e-05 m | 0.1444 | -5.6297e-06 | 7.57 / 934.33 ms | 3 |
| oncoming | 600 / 600 | 0.0381 m | 0.0222 | -5.6348e-06 | 7.65 / 81.03 ms | 1 |
| crossing | 600 / 600 | 9.5197 m | 0.1823 | -5.7592e-06 | 7.45 / 22.34 ms | 0 |
| cruise | 600 / 600 | Inf (no target) | 0.1824 | -6.4780e-06 | 7.16 / 18.26 ms | 0 |
| uncertainCrossing | 600 / 600 | 9.5196 m | 0.1734 | -1.0745e-05 | 7.34 / 32.15 ms | 0 |

Every case reports `passed = 1`, `completed = 1` and
`allExecutedHoldsVerified = 1`. All separation margins are strictly positive, so
no case collides. All CLF residuals are negative, so the demanded decrease holds
at every executed hold. The `uncertainCrossing` case carries nonzero declared
bounds (ego `[.01;.01;.001;.01;.01;.001]`, target
`[.1;.1;.05;.05;.01;.01;.01;.01]`, jerk `[.02;.02]`) and still completes.

Two qualifications:

- `oncoming` reports `minimumModelDomainMargin = -0.1550`, the only negative
  value among the five cases. The scenario header states that state-domain
  margins are offline diagnostics that neither accept nor reject a command, but
  the negative value means the declared model domain was left during that run.
- `stationary` retains a 9.2775e-05 m (0.0928 mm) minimum body gap. It is
  positive, so the case passes, but the clearance is at the scale at which
  numerical conditioning matters.

## 4. Enforced 50 ms deadline: 1 of 3 cases completes

The same campaign's deadline runs (120 holds, `DeadlineSeconds = 0.05`):

| Case | Holds | Max frame |
| --- | --- | --- |
| stationary | 0 / 120 | 55.60 ms |
| oncoming | 52 / 120 | 51.36 ms |
| crossing | 120 / 120 | 17.99 ms |

The stationary case rejects its first frame outright. Cold first frames reached
934.33 ms in the unconstrained runs, and the warmed maximum for that case was
still 108.96 ms against a 50 ms period. No real-time claim follows from these
measurements, and the machine was shared with unrelated MATLAB sessions.

## 5. Conclusion

The controller runs correctly on the plant it contractually declares: the
declared-plant recursion campaign completes every hold in all five cases,
collision-free, with the CLF decrease satisfied. It does not run against the
high-fidelity PassVeh14DOF plant, and it rejects road boundaries outright.
The four PassVeh14DOF commands listed in `CLAUDE.md` therefore do not constitute
a usable verification of the current controller; `runRecursiveSafetyValidation`
does. Real-time execution under a strict 50 ms deadline remains unmet in two of
three deadline cases.
