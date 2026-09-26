# One-pass trajectory linearization — September 25, 2026

The controller now implements the selected option 1: active encounters use
stage Jacobians evaluated along a nonlinear rollout of the shifted previous
input plan, starting from the current observation. The implementation passes
251 selected MATLAB tests. However, all three nonlinear vehicle validation
scenarios still stop before passing the target or recovering cruise. This is
an implemented model update, not a successful avoidance repair.

## Implemented behavior and scope

`ltvBicycleModel.trajectoryStages` rolls out the existing nonlinear modified
Fiala model once for the anchor input sequence. At each hold it evaluates
the continuous state/input Jacobians and affine offset at that hold's own
rollout state and input, then computes the held affine transition with a
matrix exponential. Varying references include the spatial curvature
derivative in station and heading dynamics. The first encounter frame uses
cruise inputs; subsequent frames shift the preceding solution and use cruise
inputs for any additional horizon nodes.

`hardEncounterBarrier` now honors the existing default
`model.linearizationPolicy="trajectory"` in the online encounter path instead
of always supplying cruise generators. Operating points stay fixed during
the existing separation-direction search and trajectory solve. There is no
iteration that relinearizes dynamics around the newly optimized trajectory.
No new trust region, tire-force constraint, nonlinear acceptance check, or
terminal/cruise-controller redesign was added.

The first-hold CLF cone, CLF diagnostic, executed-generator metadata, and
reported affine axle forces now use the actual prediction stage. The cruise
reference and Lyapunov metric remain the tracking objective. Force reporting
does not clip or modify the steering/braking command.

Changing generators invalidates the previous affine witness. Every refreshed
encounter frame starts from the measured state and requires fresh admission;
the shifted inputs supply an anchor, not an automatically executable fallback.
Metadata explicitly sets `recursiveFeasibilityGuaranteed=false` with scope
`freshTrajectoryModelRequiresNewAdmission`. The existing terminal continuation
is not described as the same online generator. Stored state version 45
requires resetting older states. Explicit `linearizationPolicy="cruise"`
retains the unchanged-generator study mode. Target-free cruise is unchanged.

## Nonlinear vehicle validation

The three truth-fed PassVeh14DOF experiments request 18 s at 10 m/s ego and
target speeds, with a 2 s cruise recovery window. They retain the existing
50 ms control period, 50 m initial target distance, 30 m perception range,
road boundaries and shoulders, tire parameters and solver settings. The
S-curve uses peak curvature 0.01 /m and wavelength 80 m. Full effective
configuration and failure messages are exported in `scenario-details.json`.
These exact-sensing scenarios use no stochastic sensor seed; no NRMM estimator
campaign was rerun in this task.

| Scenario | Executed / requested (s) | Trajectory-linearized holds | Stop reason | Passed target / recovered cruise |
| --- | ---: | ---: | --- | --- |
| Straight oncoming | 1.75 / 18 | 14 | `optimizationFailed` | No / No |
| Circular oncoming | 1.40 / 18 | 7 | `optimizationFailed` | No / No |
| S-curve oncoming | 1.30 / 18 | 5 | `singularReferenceDomain` | No / No |

The target first becomes visible at 1.05 s in each experiment. Total executed
holds are 35, 28 and 26. Minimum executed-prefix SAT margins are respectively
9.9928, 17.2386 and 19.0879 m, and road-function margins are 7.5737, 7.5284 and
6.9626. These positive prefix margins do not demonstrate completed avoidance.
No command is issued at the failing attempt.

Straight and circular runs reject the searched fixed-direction problems with
Clarabel status 2, including previous-plan and alternate directions. This
does not prove that the underlying nonlinear avoidance task is infeasible.
The S-curve rejects a local pose domain because its curvature bound times
lateral extent is at least one. This is a prediction-domain rejection, not
the earlier repaired S-curve input-interface error, and does not establish
that the physical vehicle reached a coordinate singularity. This task does
not isolate which anchor, uncertainty, or pose-bound contribution dominates
that domain rejection.

The preceding cruise-linearized runs reached 2.20 s before failing in all
three cases (S-curve after its interface repair). The new runs therefore stop
earlier; they do not establish improved closed-loop performance.

### Tire-force diagnostic and remaining limitation

Only the nonlinear anchor rollout respects the modified Fiala saturation
globally. Each local tangent remains an affine approximation: the optimizer
can select a different steering angle or braking ratio within the existing
actuator constraints. Updating the operating point alone does not impose
the nonlinear tire limit on every candidate input.

| Scenario | Peak reported front affine force (kN, magnitude) | Time (s) | Nonlinear Fiala at that state/input (kN, magnitude) |
| --- | ---: | ---: | ---: |
| Straight | 6.9934 | 1.55 | 6.1431 |
| Circular | 7.5411 | 1.35 | 0.6923 |
| S-curve | 5.2422 | 1.10 | 5.9870 |

The circular discrepancy illustrates remaining extrapolation, including the
combined-slip capacity change with the optimized braking ratio. These values
recompute the controller's Fiala law at the issued command; they are not axle
force measurements from the Blockset plant. They also cover different,
shorter prefixes than the preceding 58–77 kN audit, so the smaller peak alone
is not a controlled measure of prediction improvement.

## Verification and reproducibility

All **251/251 selected tests** pass, with no failed or incomplete final
results. Five new behavior tests cover per-node state/input operating points,
the zero steering derivative of a saturated Fiala anchor, spatial curvature
Jacobian agreement, shifted-plan refresh and consistent CLF/force reporting,
and unchanged target-free cruise. Four parameterized solver-failure checks
reject reuse of an old plan after a trajectory-model refresh. Existing tests
whose subject is a carried immutable witness now explicitly select cruise
linearization instead of assuming the online default still carries it.

The selection includes controller/configuration, Fiala, bicycle prediction,
scheduled references and terminals, feedback, continuation, safety closure,
actuation, recovery, NRMM target prediction and curved geometry tests. Five
of the six `targetReactionTest` methods were run; its long release scenario
was not part of this selection. The complete repository suite was not run.
`tests.csv` records every selected test by name and its latest result; earlier
failures caused by stale inheritance expectations were resolved by explicitly
selecting the fixed-model test mode, without relaxing their assertions.

Code Analyzer with factory settings reports no findings in eleven of twelve
modified/new MATLAB files. Seven existing performance suggestions remain in
unchanged portions of `hardEncounterBarrier.m` (five sparse-indexing and two
logical-indexing suggestions). No production change was made to address them.

```matlab
addpath('scripts');
summary = runTrajectoryLinearizationValidation( ...
    '/home/zai/.cache/collisionAvoidance/trajectory-linearization-20260925');
```

MATLAB R2026a Update 3 was used. The orchestration tool timed out after 300 s;
MATLAB continued and saved all three MAT results and the completed summary,
which were subsequently inspected. A transport timeout was not counted as
a scenario outcome. The simulation is an offline experiment, not a verified
50 ms real-time run; per-frame timing diagnostics are in `force-diagnostics.csv`.

Compact exports and a SHA-256 manifest reside in
`report/TRAJECTORY_LINEARIZATION_20260925/`. Raw results remain outside Git in
the cache directory above. Base commit:
`a5ac4dc3c49f8a4b5dd8c546bd93c6b543392e39`. Unrelated pre-existing scenario-driver
edits, observer work, solver dependencies, reference material and generated
binaries are excluded from this task's commit; the manifest identifies the
effective scenario sources used by the validation.
