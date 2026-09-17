# Smooth-reference controller implementation and validation

September 17, 2026. Outcome: **partial**. Variable-curvature functional trials complete under the diagnostic budget; strict encounter real-time qualification remains open.

## Implemented behavior

- Added spatial PCHIP curvature profiles, explicit constant-curvature continuation, bounded position integration, and local position/yaw certificates. Constant lines/arcs retain their interfaces.
- Compiled immutable phase-indexed affine models, including the spatial curvature derivative; backward five-error Riccati CLF uses the next matrix and reference. Input effort is centered at the phase-specific force-balanced trim, with squared soft CLF slack.
- Added a six-dimensional terminal family whose phase, modal successors, moving-reference defects, actuator bounds and finite slew are verified. Hypothetical terminal feedback is never executed after solver failure. The complete predictive CBF witness is retained and shifted.
- Enforced hard whole-hold phase and Frenet regularity domains. Phase rows share the stage-local sparse geometry transcription; full original rows remain in independent verification. Fixed exact-endpoint continuation and carried projection-branch edge cases.
- Updated diagnostic nonlinear rollout to evaluate curvature at each evolving station and retain its station sensitivity; this diagnostic is not used as a safety proof.

## Material limitation of this implementation

**The default 2 m clock-relative phase band imposes an additional progress requirement.** It prevents the declared scheduled model from drifting arbitrarily far from actual spatial curvature, but also excludes sufficiently long braking or waiting. It is not behaviorally neutral, and it does not satisfy a stronger requirement for unrestricted path following independent of timing. No lateral avoidance curve is prescribed; nevertheless, the station progression is restricted. Removing that restriction needs a spatially parameterized model/terminal family with a valid witness transfer argument, rather than deleting the bound.

The model is the explicit zero-residual held affine study plant. No nonlinear physical-vehicle guarantee, estimator-controller qualification, arbitrary road-boundary guarantee or worst-case execution-time guarantee is claimed. Nonzero Cartesian ego position uncertainty on a profile currently fails the inverse-projection contract. All executed safety rows are hard; a solver failure stops simulation without a fallback.

## Reproduction

```matlab
addpath('scripts');
runSmoothReferenceControllerValidation( ...
    OutputDirectory='/home/zai/.cache/collisionAvoidance/general-path-20260917/final', ...
    SampleCount=300, RunStrictTiming=true);
```

MATLAB R2026a, one computational thread, 8 m/s body longitudinal reference speed, 0.1 s holds, 300 intended holds (30 s), exact state and target observations, no process residual or physical road boundaries. The region is 16 m and required rectangle clearance is 0.25 m. Main trials retain default unbounded input slew; separate terminal tests use 0.5 rad/s steering and 1/s signed-force-ratio rates. Profiles and targets are deterministic; there is no random input.

The 120 m references are: PCHIP samples every 10 m of `0.015*sin(2*pi*s/120)` (S bend); `0.01*(1-cos(pi*s/120))` (transition to 0.02/m); and stations `[0,20,45,75,100,120]` with curvature `[0,.005,.018,.010,0,0]` (asymmetric bend). The stationary target is at station 15 m. The oncoming target starts 30 m ahead of the station-30 tangent point and moves along that inertial tangent at -8 m/s. Target motion is not forced to follow the curved ego reference.

Cold reference preparation and discarded startup probes are recorded separately. Frame timing includes online observation construction and the controller, excluding truth integration and offline audits. Diagnostic 5 s budgets assess functionality only. Independent strict 100 ms trials stop when no timely certified command is available.

## Final results

| Path / scene | Diagnostic holds | Clearance above required 0.25 m | Largest frame (ms) | Strict 100 ms holds | Strict result |
| --- | ---: | ---: | ---: | ---: | --- |
| sBend / cruise | 300/300 | — | 74.717 | 300/300 | Pass |
| transition / cruise | 300/300 | — | 16.479 | 300/300 | Pass |
| asymmetric / cruise | 300/300 | — | 20.156 | 300/300 | Pass |
| sBend / stationary | 300/300 | 0.244100 m | 268.157 | 0/300 | Deadline failure |
| sBend / oncoming | 300/300 | 0.350157 m | 151.500 | 26/300 | Deadline failure |

All issued diagnostic commands pass the independent hard certificate. Separation in the table is a sampled physical diagnostic in addition to the swept certificate, not its replacement.

| Path / scene | Final lateral trim error (m) | Final heading trim error (rad) | Final longitudinal speed error (m/s) | Largest phase error (m) |
| --- | ---: | ---: | ---: | ---: |
| sBend / cruise | -5.14639e-07 | -1.25488e-10 | 0.000328753 | 0.0105161 |
| transition / cruise | 4.35867e-05 | 1.57587e-07 | 6.26648e-05 | 0.00699723 |
| asymmetric / cruise | -1.07653e-07 | 1.065e-12 | -0.000173275 | 0.00681432 |
| sBend / stationary | 1.28629e-06 | 3.21033e-09 | 2.4406e-05 | 0.885481 |
| sBend / oncoming | -6.03708e-07 | 2.25672e-11 | 0.000426387 | 0.212149 |

Errors are relative to the steady-turn trim at the **actual spatial station**; nominal body heading offset, lateral velocity and yaw rate need not be zero on a bend. Final small errors demonstrate recovery in these finite trials, not a proof of zero-slack tracking during curvature transitions.

## Runtime diagnosis

The initial condensed phase constraints produced static/oncoming admission maxima of 2192.774/849.472 ms. Sharing the existing stage-local swept projection removed long-range dense coupling in the lifted SOCP. The first sparse repeat reduced these to 252.024/153.974 ms. These are development measurements from distinct runs, not hardware speedup guarantees. The final table reflects subsequent full-hold regularity rows and separate phase working-set groups.

Encounter admission still includes support-family formulation, restoration, and several native conic working-set solves. Strict real-time failures are retained rather than counted as completed avoidance. The full slow-frame decomposition, setup times and search counts are in the JSON; raw traces remain in the cache.

## Validation

- Full working-tree suite: **657 passed, 0 failed, 0 incomplete**. This began before the final exact-tail matrix identity and two additional focused checks; its results are not relabeled as a rerun of the final snapshot.
- Final geometry/schedule/terminal suite: **24 passed, zero failed/incomplete**. Includes sparse/full objective and row equivalence, nonlinear station sensitivity, exact endpoint continuation, finite slew and phase/regularity boundary checks.
- Native/interpreted bicycle kernel parity: 9 passed; core source-budget test: 1 passed (20 files). Geometry MEX rebuilt. Code Analyzer ran on changed MATLAB sources; local-settings fallback and array/sparse performance advisories remain. `git diff --check` passes.
- Independent auxiliary audits traverse all 152 reference transitions, including the repeated tail, and 250 hypothetical terminal holds per path. A separate 640-boundary-state audit checks 31 exact-flow points per hold. These sampled checks supplement, rather than replace, the implemented Bernstein and modal inclusion inequalities.
- Initial integration attempts and initial slower implementations are retained in the cache. Concurrent unrelated observer scenario/benchmark work is excluded from this task's commit scope.

## Artifacts and theory

- [Machine-readable summary](SMOOTH_REFERENCE_VALIDATION_20260917.json).
- [Scenario driver](../scripts/runSmoothReferenceControllerValidation.m).
- [Reference and terminal derivation](../controller/CURVED_CRUISE_CERTIFICATE.md), [recursive proof extension](../controller/TERMINAL_CBF_PROOF.md).
- Raw MAT/JSON, tests, independent audits and failed development attempts: `/home/zai/.cache/collisionAvoidance/general-path-20260917/`.
