# Previous-plan and flow-based linearization anchors

Date: September 27, 2026. Scope: source inspection and three saved-frame
diagnostics. The online initialization policy is not changed by this study.

The proposed ordering is appropriate: choose a dynamically realizable reference
trajectory first, construct the affine model along it, and then solve the hard
optimization problem. Prefer a usable shifted previous plan; otherwise construct
a bounded flow-based reference. These are reference sources for one controller,
not additional controllers or execution methods.

## Current behavior and the missing step

`hardEncounterBarrier.predict` already uses shifted previous inputs for active
trajectory linearization. It starts from the current measured ego state and calls
`ltvBicycleModel.trajectoryStages`, which rolls out the nonlinear vehicle model
before forming stage Jacobians. It does not blindly reuse the previous predicted
state sequence. Any additional horizon inputs come from the configured reference.

The flow branch runs later, inside `solveHardCbfClf.fixedDirections`, after the
optimization model has been constructed. Its fitted inputs change the search
point and separating directions, but do not rebuild the dynamics or tire-force
tangents. Moving `program.anchorPlan` for numerical centering is not model
relinearization. The previous-plan branch and the flow branch therefore do not
currently implement equivalent trajectory-based linearization.

The existing flow fit is a regularized least-squares geometric fit with an affine
terminal nullspace projection. It has no actuator bounds in the fitting solve.
Consequently, its output cannot be assumed to be a bounded vehicle reference.

## Reproducible diagnostic

Run from the repository root:

```matlab
addpath('scripts');
maxNumCompThreads(1);
diagnoseFluidLinearizationAnchors( ...
    '/home/zai/.cache/collisionAvoidance/failure-analysis-20260927-034213', ...
    '/home/zai/.cache/collisionAvoidance/trajectory-anchor-selection-20260927');
```

The inputs are the three exact-state failed-frame snapshots from the preceding
failure diagnosis. Their measured states, road/target inputs and original horizons
are retained. There is no random sampling or new plant simulation. Each case:

1. Builds a cruise-linearized bootstrap without the previous input seed and calls
   the existing flow initializer. This bootstraps the geometric fit; it is an
   offline experiment, not a newly exposed controller mode.
2. Projects the fitted steering/braking inputs componentwise onto the existing
   actuator bounds, then rolls out the nonlinear model from the observed state.
   Clipping does not preserve the exact flow path or terminal fit and does not
   certify road, collision or terminal feasibility.
3. Rebuilds the complete trajectory-linearized optimization problem. Assertions
   verify that the model operating inputs and states exactly equal the selected
   inputs and nonlinear rollout.
4. Disables a second flow refit in this diagnostic, so it cannot silently change
   the seed while retaining the new tangents. Runs the existing hard solver with
   a 30 s diagnostic work budget and input-deviation weight zero. No production
   hard constraint family is removed or softened.
5. Independently checks accepted physical rows, joint residuals against their
   reserved bounds, and conic residuals at tolerance 1e-8. Measures departure from
   the selected operating inputs and same-state/input affine versus nonlinear
   Fiala force differences. Candidate nonlinear rollout is checked for model
   domain validity only; no plant avoidance or recovery is inferred from it.

## Results

The steering bound is 0.698132 rad (40 degrees). All raw flow fits exceed it:

| Failed frame | Raw maximum steering | Maximum steering after projection | Rebuilt hard solve |
| --- | ---: | ---: | --- |
| Straight, 2.05 s | 1.528754 rad (87.59 deg) | 0.698132 rad | Rejected, Clarabel status 2 |
| Circular, 1.50 s | 0.706481 rad (40.48 deg) | 0.698132 rad | Accepted |
| S bend, 1.40 s | 1.051980 rad (60.27 deg) | 0.698132 rad | Accepted |

All three projected anchors pass the nonlinear model-domain rollout. Both
accepted problems pass the explicit residual checks. Circular physical/joint
excesses are -9.619336e-6 and -4.638273e-9; S-bend excesses are -9.589176e-6 and
-5.184769e-4. Conic excess is zero for both. These rows have different physical
units and are not a distance-based safety metric. No new independent infeasibility
proof was computed for the rejected straight case.

| Accepted case | Maximum normalized input departure | First-stage force mismatch | Whole-plan force mismatch |
| --- | ---: | ---: | ---: |
| Circular | 1.953012 | 0.511076 kN | 13.002691 kN |
| S bend | 1.999986 | 1.412703 kN | 29.951056 kN |

Input departure uses steering-angle and braking-ratio actuator scales. Force
differences compare affine and nonlinear Fiala forces at the same candidate
affine-predicted state and input; they are not measured plant forces. Invalid
slip-domain stages are omitted from this force statistic, so it is not a
domain-validity certificate. Both accepted candidates separately admit a
nonlinear model-domain rollout. Diagnostic elapsed times (including bootstrap,
rollout, formulation, solve and checks) were 1.081488 / 0.273565 / 2.253573 s for
straight / circular / S bend. These are single observations on a shared machine,
not isolated controller timing benchmarks or evidence of real-time execution.

## Design conclusion and remaining work

Changing the operating trajectory repairs two of these three individual failed
frames. This supports the proposed initialization order, but does not establish a
complete closed-loop repair. The accepted plans can still depart almost two full
actuator scales from the new reference and exploit inaccurate tire tangents.

A production implementation should distinguish **usable as a model reference**
from **certified for execution**. The former requires time/horizon alignment,
compatible model data, finite bounded inputs and a valid rollout from the current
observation. Current actuator slew restrictions, if present, must also be
respected. Changed obstacles and road geometry must enter the new problem; an old
certificate cannot simply be transferred across a refreshed model. A reference
does not have to be a feasible solution to the new optimization problem.

If that previous reference is unavailable or cannot produce an acceptable new
model/solve, construct a bounded dynamically realizable flow reference before
building the final model. Recompute stage dynamics, tire-force tangents, feedback
tubes and dependent hard rows together. Center the recently added input-deviation
penalty at these same fixed operating inputs. The earlier penalty experiments
show that a soft cost helps locally but does not guarantee tangent validity;
candidate/model consistency still needs explicit validation. Neither initializer
may issue an actuator command without the required hard-solve acceptance.

This study does not install that online selection policy or choose an automatic
production usability threshold. It does not introduce nonlinear shooting,
additional controller methods, safety relaxation, or a closed-loop success claim.

## Validation and artifacts

- `trajectoryLinearizationTest` and `fluidInitializationTest`: 38 passed, zero
  failed or incomplete. Final Code Analyzer check of the diagnostic script:
  zero findings.
- Compact numerical export: `TRAJECTORY_LINEARIZATION_ANCHORS_20260927/summary.csv`.
- Input, executed-source and raw-output hashes are in the adjacent manifests.
  Source hashes include existing workspace changes used by the experiment;
  those unrelated changes are excluded from this study's commit.
- Original raw MAT results and logs are external at
  `/home/zai/.cache/collisionAvoidance/trajectory-anchor-selection-20260927`.
  The regression-test log is an identified copy from the initial diagnostic
  workspace. Initial analyzer advisories about redundant message/feasibility
  initializations were removed before the final clean analyzer check.
