# Visible-target lifecycle and no-target joint cruise

September 13, 2026. The controller now supports zero visible vehicles using
its existing PCBF/CLF optimization. No virtual target, second controller or
new desired-acceleration objective is introduced. Vehicle collision rows and
vehicle terminal halfspaces are absent when the target array is empty; road,
pose/model, actuator, CLF and road/pose terminal conditions remain.

## Execution and certificate changes

`targetPrediction.admitExact` accepts an empty target array. The controller
reports `hasTarget=false`, empty active target keys and an 8-by-0 target error
matrix for that state. Repeated no-target frames retain the existing ego-box
conditioning and shifted-plan check, just as a fixed single-target encounter.

When a target appears or leaves, execution time, held input, ego-box
consistency and model/road identity are checked first. A complete fresh plan
for the new target set is then required; the previous set's certificate cannot
authorize a command if fresh admission fails. The switch reports new/discharged
keys, `targetSetChanged=true`, no verified candidate and no numerical descent
comparison between the two different value functions. The fixed-set recursive
argument is unchanged, including its terminal policy; it does not prove
unconditional feasibility or unseen-object safety at an arrival event.

Departure requires a current, finite-range, complete-within-range perception
declaration. An absent or stale sample is not sufficient. This relies on the
sensor's no-missed-detection premise rather than proving it. Initial explicit
no-target scenes can still run without perception metadata. Multiple targets
and changing the identity of an active target remain unsupported.

## Actual estimator plus declared-plant trials

The new driver `runDeclaredPlantEstimatorControllerScenario` sends synthetic
GNSS/IMU/gyro/radar measurements through the actual NRMM adapter and supplies
its estimated states and unchanged online error boxes to the controller.
Only issued-input memory is added by the driver. Plant truth is used for
sensor synthesis, independent `expm` integration of the accepted first-hold
affine generator and geometry/error-bound audits. It is not passed as an
estimated state. The road is fixed, straight, with boundaries at y = +/-5 m.

Both trials request 300 holds (30 s): cruise 10 m/s, control period 0.1 s,
configured horizon 16/minimum 2, seed 20260913, 30 m radar, target velocity
(-10,0) m/s and lateral position 0.8 m. The NRMM uses the integration
configuration's bounded sensor noise, 80 Hz samples and one RK4 step per
sample with integration-defect accounting. Its target prior is 10 m/s and
rear-axle distance matches the declared vehicle. The first target starts
1000 m ahead and stays outside range; the second starts 100 m ahead and
enters range during the trial. The target's published motion uncertainty is
not changed to force acceptance.

| Initial target distance | Executed holds | Outcome | Final speed (m/s) | Median / maximum frame (ms) | Over 100 ms |
| --- | ---: | --- | ---: | ---: | ---: |
| 1000 m | 300 | Completed no-target cruise | 9.99406953 | 65.590 / 813.403 | 4/301 |
| 100 m | 36 | Stopped at first target publication, 3.6 s | 9.99429543 | 65.607 / 117.085 | 1/37 |

The completed no-target trial has 301 certified commands, all fresh, zero
predictive safety value, no terminal commands and no carried commands. The
final lateral displacement is -0.001542 m and heading is -0.000334 rad.
Minimum audited road margin after the 0.25 m clearance is 3.787718 m. Every
evaluated ego state lies inside its published error box and all sampled ego
premises pass. There are no published target boxes to test in this trial.
These noisy tracking results do not assert exact asymptotic convergence.

The second trial executes 36 certified no-target commands before the first
target publication at 3.6 s. It then stops with
`collisionAvoidanceController:nonexactStudyInput`: the existing controller
still rejects the NRMM target's nonzero jerk/yaw-acceleration contract. The
new obstacle is not ignored and the prior no-target plan is not executed
past that failed admission. The one published target's checked Cartesian
components and all ego boxes contain truth at the audited samples. This
trial is not a completed avoidance encounter.

Timing includes the sensor/observer adapter, input assembly and controller,
including failed attempts, but excludes plant integration and independent
truth audits; the road is fixed, so no online road-fit cost is included.
The simulator waits for computation rather than applying delayed commands.
The medians meet 100 ms, but both trials have overruns, including four in the
completed trial: no every-frame real-time or physical zero-latency guarantee
is established. The final computed command in a completed trial is unexecuted.

## Remaining joint compatibility limitations

The controller's zero-process-residual, exact Cartesian target law and
all-future terminal support premises remain. The independent PassVeh14DOF
configuration with nonzero empirical residual rates must still be rejected,
even with no target. Supporting no-target cruise is not permission to erase
physical model mismatch. The uncertain-target terminal-support problem from
`JOINT_RERUN_RESULTS_20260913.md` also remains after the motion-contract guard.
These are separate follow-up tasks from the implemented visibility lifecycle.

## Validation and reproducibility

The new eight-case lifecycle suite passes: no-target CLF cruise, no-target
continuation, arrival admission, failed arrival without reuse of the old
witness, confirmed departure, stale departure rejection, execution-contract
rejection on arrival and preservation of model-residual rejection. Existing
tests that formerly stopped on empty targets are updated to expect their
actual unsupported physical-residual condition; missing active-target data
requires the new explicit departure declaration. The complete suite ran through MATLAB MCP: 647/649 passed initially. The
remaining two tests expected the removed empty-target rejection. Their observed
errors were nonzero model residuals and unsupported reference jumps; the
expectations were corrected and both cases passed a targeted rerun. Thus all
649 cases are covered by passing results across the full run and retest.
The MCP request timed out at 300 s but MATLAB saved completed results. A
fallback batch run started while those results were unavailable was interrupted
once the completed suite evidence arrived; it is not counted as another full run.

Code Analyzer checks the four changed controller/input/target modules, new
driver and lifecycle test. It falls back to default analyzer settings because
a pre-existing R2025b preference file is unavailable; after removing one unused
driver initialization there are no source findings. `git diff --check` passes.
No observer, native kernel, objective-weight or default configuration changes
are included. The mathematical scope is documented in
`controller/INFORMATION_STATE_PCBF.md`.

```matlab
addpath('scripts');
runDeclaredPlantEstimatorControllerScenario(TargetInitialDistance=1000, ...
    OutputDirectory=fullfile(tempdir,'no-target-joint'));
runDeclaredPlantEstimatorControllerScenario(TargetInitialDistance=100, ...
    OutputDirectory=fullfile(tempdir,'arrival-joint'));
```

Original MAT traces, MATLAB batch scripts, summary JSON and validation records:

`/home/zai/.cache/collisionAvoidance/visible-target-20260913`
