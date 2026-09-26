# Current controller avoidance and cruise recovery — September 26, 2026

The current controller does **not** complete avoidance followed by sustained
cruise recovery in any of the genuinely conflicting scenarios tested here.
The nonlinear PassVeh14DOF vehicle completes both 18 s no-target cruise
controls, but all five primary avoidance runs stop early. The declared-model
campaign completes four cruise controls and one non-conflicting crossing;
all ten genuinely conflicting cases stop before recovery.

These are fresh executions at base revision `0f54988b0c3c3c7584d15f15971404c77ff72493`, with
existing working-tree scenario edits preserved and recorded by source hashes.
The controller, estimator, physical model, safety constraints and tuning were
not changed. The new Python audit independently checks persisted results.

## Conditions and acceptance

- Vehicle: Vehicle Dynamics Blockset PassVeh14DOF, requested duration 18 s,
  ego and target speed 10 m/s, control period 0.05 s, initial target distance
  50 m and visibility 30 m. Circular radius 100 m; the S bend uses the current
  sinusoidal-curvature driver. Avoidance retains default perceived road
  boundaries and shoulders; the two no-target controls have no road boundaries.
- Estimator variants use seed 20260925 and matching 10 m/s target-speed priors.
  The historical seed is deliberately retained for comparison; it is not an
  activity timestamp. Truth-fed vehicle runs have no stochastic sensor seed.
- Declared model: 30 s, 600 holds at 0.05 s, 8 m/s reference, friction
  coefficients [0.85, 0.85], 1.6 s nominal horizon, seed 20260925 and 30 s offline
  search/frame budgets. The current default linearization policy is
  `trajectory`. Its executed plant is the declared held affine generator,
  not an independent nonlinear vehicle. Rectangle geometry is audited every
  5 ms. Curved declared-model cases have no physical road boundaries; bounded
  straight cases use physical edges at lateral +/-5 m and strict coverage.
- Success requires completing the requested duration, positive sampled body
  separation, passing the target, and satisfying the final 2 s
  recovery window at recorded samples: speed error <=0.5 m/s, path lateral error <=0.2 m, and
  velocity-course error <=0.02 rad. Course includes sideslip.
- These are offline simulations with immediate scheduled actuation. Runtime
  deadline enforcement is disabled for the vehicle campaign; the solver's
  finite internal work budget still applies. The two campaign processes
  overlap briefly, so timing is not a controlled performance benchmark.
- Positive margins in stopped runs apply only to their executed prefixes.
  No collision, safe continuation, or recovery is inferred after termination.
  Finite sampled geometry is not a continuous-time safety proof.

## Primary nonlinear vehicle results

SAT margin is the largest separating-axis gap between the oriented vehicle
rectangles, not Euclidean closest-point distance. A positive value certifies
non-overlap only at that evaluated sample.

| Scenario | Executed / requested (s) | Minimum prefix SAT margin (m) | Sustained final cruise | Result |
| --- | ---: | ---: | --- | --- |
| `straight_cruise` | 18.00 / 18 | — | Yes | Completed; cruise criteria met |
| `circular_cruise` | 18.00 / 18 | — | Yes | Completed; cruise criteria met |
| `straight_oncoming` | 2.05 / 18 | 4.019774 | No | optimizationFailed |
| `circular_oncoming` | 1.50 / 18 | 15.360714 | No | invalidTireOperatingPoint |
| `varying_curvature_oncoming` | 0.00 / 18 | — | No | optimizationFailed |
| `estimated_straight_oncoming` | 1.05 / 18 | 23.994603 | No | optimizationFailed |
| `estimated_circular_oncoming` | 1.05 / 18 | 24.048799 | No | optimizationFailed |

The straight and circular no-target controls have maximum full-run speed
errors 0.001820 and 0.001910 m/s; circular maximum lateral error is 0.002754 m.
This establishes stable basic cruise in these two cases, not avoidance recovery.

Straight avoidance fails at 2.05 s after 41 holds; circular avoidance fails
at 1.50 s after 30 holds. Both estimator-fed cases stop on the first
visible-target attempt at 1.05 s. Their finite executed-prefix estimates and
large positive clearance do not establish successful encounter admission.

The primary S-curve run executes no hold: its first program reports
"The program work deadline expired before the native solve." Although the
public identifier is `optimizationFailed`, this instance is a work-budget
expiration, not a solver infeasibility result. It is retained separately
from any warmed repetition.

## Warmed S-curve repetition

After the primary campaign, repeated the same S-curve settings in the same
MATLAB session without changing controller parameters or budgets. This run
executes 28 holds and stops at 1.40 s with Clarabel status 2 after the stored,
previous-plan and alternate-direction searches are rejected. Minimum prefix
SAT margin is 17.181475 m; minimum speed is 8.871464 m/s. Passing and recovery
both remain false. The constructed nominal encounter at 2.508625 s has a
-0.5 m rectangle SAT margin. The baseline evaluated only through the stopped
prefix has not yet reached that collision; its `baselineCollides=false` field
must not be interpreted as a non-conflicting full scenario.

The warmed repeat separates the first-run work-budget expiration from a later
planning rejection. It reproduces the previous 1.40 s failure to reported time
precision. S-curve pre-detection solve times are about 2.25--2.82 s per frame,
well above a 50 ms period; this offline experiment is not a real-time result.
The original failed primary result remains unchanged. The extra run brings
the total to 23 saved simulations (22 primary plus one repeat).

## Current declared-model results

The undisturbed-cruise baseline collides in every failed target case below.
The completed straight crossing is non-conflicting (baseline minimum gap
about 9.52 m) and is excluded from active avoidance successes.

| Scenario | Executed / requested (s) | Minimum prefix body gap (m) | Final cruise | Result |
| --- | ---: | ---: | --- | --- |
| `cruise_k+0.00` | 30.00 / 30 | — | Yes | Completed; cruise criteria met |
| `stationary_k+0.00` | 0.30 / 30 | 7.889977 | No | invalidTireOperatingPoint |
| `oncoming_k+0.00` | 2.85 / 30 | 9.604207 | No | invalidTireOperatingPoint |
| `crossing_k+0.00` | 30.00 / 30 | 9.519556 | Yes | Completed; cruise criteria met |
| `cruise_k+0.01` | 30.00 / 30 | — | Yes | Completed; cruise criteria met |
| `stationary_k+0.01` | 0.25 / 30 | 8.134614 | No | invalidTireOperatingPoint |
| `oncoming_k+0.01` | 2.80 / 30 | 10.339197 | No | invalidTireOperatingPoint |
| `crossing_k+0.01` | 0.20 / 30 | 10.997268 | No | invalidTireOperatingPoint |
| `cruise_k-0.01` | 30.00 / 30 | — | Yes | Completed; cruise criteria met |
| `stationary_k-0.01` | 0.25 / 30 | 8.134614 | No | invalidTireOperatingPoint |
| `oncoming_k-0.01` | 2.80 / 30 | 10.339197 | No | invalidTireOperatingPoint |
| `crossing_k-0.01` | 0.10 / 30 | 11.144858 | No | optimizationFailed |
| `cruise_straight_boundaries` | 30.00 / 30 | — | Yes | Completed; cruise criteria met |
| `stationary_straight_boundaries` | 0.15 / 30 | 8.988755 | No | roadBoundaryCoverageGap |
| `oncoming_straight_boundaries` | 2.70 / 30 | 11.989748 | No | roadBoundaryCoverageGap |

Seven conflicting cases report `invalidTireOperatingPoint`, one reports
optimization rejection, and two reject strict road-boundary coverage of a
certified cell. A coverage rejection is not a measured road departure.
The tire error requires `abs(alpha)<pi/2` and `abs(beta)<=1`; the error string
alone does not identify which future axle/state/input violated that domain.

These current results supersede the earlier 15/15 completion result for the
older revision in [the September 25 campaign](CONTROLLER_RECOVERY_VALIDATION_20260925.md).
The controller has since adopted trajectory linearization and corresponding
feedback. No claim is made that the two revisions implement the same algorithm.
The [preceding solve-failure audit](SOLVE_FAILURE_AUDIT_20260925.md) and
[tire prediction design](TIRE_PREDICTION_DESIGN_20260925.md) discuss related
prediction limitations; this rerun does not independently prove every prior
causal explanation or implement the proposed repair.

## Reproduction and preserved evidence

MATLAB R2026a Update 3; each driver and the exact batch use one computational
thread. Raw MAT files and console outputs are original results in
`/home/zai/.cache/collisionAvoidance/recovery-validation-20260926/`.
Compact export copies, source hashes and independent audit outputs are in
`CONTROLLER_RECOVERY_VALIDATION_20260926/` beside this report. Existing unrelated
working-tree changes, solver trees, native binaries and large MAT files are
excluded from this task's project commit.

```matlab
addpath('scripts');
outputDirectory = '/home/zai/.cache/collisionAvoidance/recovery-validation-20260926';
runControllerRecoveryValidation(outputDirectory, 'vehicle');
% Separate batch process, with one computational thread:
maxNumCompThreads(1);
runControllerRecoveryValidation(outputDirectory, 'exact');
```

```bash
uv run --with scipy python scripts/auditControllerRecoveryResults.py \
  /home/zai/.cache/collisionAvoidance/recovery-validation-20260926
```

The independent audit reloads all 22 primary MAT results, checks finite states,
completion/time consistency and final-window recovery, and recomputes vehicle
rectangle SAT margins using NumPy projections independent of the MATLAB
geometry function. All checks pass. A separate copy with a deliberately false
straight-avoidance recovery claim is rejected; original results are unchanged.
Passing the audit verifies truthful result accounting, not controller success.

The exact batch saves all 15 results before native teardown reports
`free(): chunks in smallbin corrupted` and the shell exits with status 137.
It is not a clean process exit. Independent reload and numerical checks pass
for all saved files. The vehicle MCP call exceeds the 300 s transport wait;
MATLAB continues and ultimately saves all seven primary results. The transport
timeout is distinct from the controller's S-curve work-budget expiration.

## Focused regression checks

All 9 MATLAB unit tests in `trajectoryLinearizationTest` and
`trajectoryFeedbackTest` pass through MATLAB MCP. They check trajectory-stage
linearization and feedback enclosure behavior, not successful complete
vehicle avoidance. The independent Python audit also passes its real-result
checks and false-recovery negative check. Python byte-compilation succeeds.
No full MATLAB unit-test suite is claimed to have run in this task.
