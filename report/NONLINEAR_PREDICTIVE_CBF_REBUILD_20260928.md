# Nonlinear predictive-CBF controller rebuild — September 28, 2026

Status: implementation and development checks completed; final validation is
blocked by MATLAB batch startup. The final-source scenario campaign has not run.
The selected regression runner is ready to reproduce the remaining checks.
This is a research implementation with explicit exact-model assumptions, not a
real-time or physical-vehicle safety validation.

## Behavior changed

The public `collisionAvoidanceController` uses one nonlinear control path and a
format-47 stored policy. Its authoritative plant is the combined-slip Fiala ODE
with front-wheel force rotation, inertial poses and held steering/force ratio.
RK4 and variational integration supply proposals; directed MPFR flow enclosures
supply acceptance. The previous affine, node-only plant is no longer the public
controller's guarantee.

The implementation follows the derivation pasted in the September 28 request.
The separately linked sandbox Markdown attachment was unavailable locally.
The full implemented contract and proof conditions are in
[`NONLINEAR_PREDICTIVE_CBF.md`](../controller/NONLINEAR_PREDICTIVE_CBF.md).

| Component | Implemented behavior |
| --- | --- |
| Nonlinear prediction | Positive-speed Fiala flow; no hidden velocity floor; interval certification covers every complete applied hold. |
| Target | Immutable constant-parameter NRMM epoch; distinct body heading and velocity course; analytic sinc flow and explicit clock/parameter consistency. |
| Rectangle geometry | Actual offset body rectangles, fixed-pose two-body dual witnesses, and outward support separation over complete integration cells. |
| Backup | Computed invariant cruise ball with input memory, nonlinear trim residuals and domain-wide remainder bounds; target separation persists for its whole future ray or annulus. |
| Predictive CBF | A hard feasible policy into the invariant backup proves the nonnegative barrier minimum is zero. Collision constraints have no executable slack. |
| CLF | Transverse path/speed error, no timetable penalty, outward first-step dissipation bounds and lexicographic performance slack. |
| Convex improvement | Fixed future feedback/reference states; a whole control box is certified before an LP/QP hierarchy. Interval sampled-flow derivatives majorize the CLF row. Point validation precedes replacement. |
| Failure handling | The stored certified feedback suffix is available before new numerical verification. On expiry it executes directly; the invariant cruise law follows after the finite prefix. |

The native substep schedule caps growth after subdivision so subtraction of
successive binary64 time nodes is exact by Sterbenz's lemma. This keeps the
validated subflows on precisely the declared hold interval. Future policy
commands include a checked `1e-12` actuator-unit evaluation allowance, also
accounted for in terminal synthesis.

## Findings during implementation

Open-loop interval accumulation was too wide to admit the oncoming prefix into
the small terminal ball. Future sampled feedback reduced this accumulation.
Re-centering that feedback independently on each numerical interval center then
introduced drift from the seeded trajectory. Retaining the seed's fixed reference
states and gains removed that source of drift and admitted the 32-frame replay.
These are numerical-certification findings, not evidence about an uncertain plant.

A convex tangent alone was insufficient to justify the requested inner-step
claim. The final implementation instead certifies the entire control box with
independent interval input generators. The first-step CLF uses an interval-gradient
absolute-value majorizer. This can be conservative and does not make the jointly
optimized rectangular avoidance problem globally convex.

A stored policy must remain usable without successful re-integration. The
implementation now inherits its original suffix under exact execution and an
unchanged model/road/target contract, and dispatches the terminal law after the
prefix. Endpoint box containment is only an inconsistency check; it is not a
proof that a disturbed measured state is a correlated reachable successor.

## Checks actually completed

Compact development results are in
[`development-checks.json`](NONLINEAR_PREDICTIVE_CBF_REBUILD_20260928/development-checks.json)
and [`development-replays.json`](NONLINEAR_PREDICTIVE_CBF_REBUILD_20260928/development-replays.json).
Their revision stages are recorded explicitly; they do not certify the final
source revision.

- An early six-suite run passed 120/120 tests: the then 27-case nonlinear suite,
  Fiala interval/feedback kernels, modified tires, controller configuration and
  source budget.
- The certified-control-box revision passed all 33 then-current nonlinear tests.
- The inherited-policy revision passed all 39 then-current nonlinear tests,
  including four control-box perturbations, sampled CLF bounds, target clock,
  exhausted-verifier suffix execution and terminal dispatch.
- Final native source syntax passed `g++ -std=c++17 -DMATLAB_MEX_FILE
  -I/opt/MATLAB/R2026a/extern/include -fsyntax-only` on
  `nonlinearSafetyMex.cpp`, `fialaFeedbackSampleMex.cpp` and `fialaIntervalMex.cpp`.
- `git diff --check` passed.

The final three added test cases cover explicit body heading, default cruise
speed and world yaw beyond the old local chart. They and the final adaptive-time
change await the final MATLAB run. No final-source Code Analyzer result is claimed.

| Development replay | Actual outcome |
| --- | --- |
| Small tracking-error recovery | 8/8 holds; certified W fell from 0.0076018924 to 0.0036603654; zero CLF slack; no replay enclosure violation. |
| Constant-curvature cruise | 8/8 holds; no replay enclosure violation; tiny positive numerical CLF slack is retained. |
| Forced improvement failure | 8/8 holds; a certified continuation remained available. |
| Early oncoming admission | Rejected before issuing a command; recorded as failed admission. |
| Oncoming after fixed-reference repair | 32/32 holds; minimum replay clearance 0.174162893 m versus required 0.100 m; no enclosure violation; maximum frame 12.11364 s; maximum horizon 189 holds. |

Each replay used a separate tight `ode45` integration with 31 audit points per
50 ms hold, `RelTol=1e-11`, `AbsTol=1e-12`, 8 m/s reference speed and deterministic
inputs without a random seed. The oncoming target started 24 m ahead at 8 m/s
in the opposite direction; both rectangles were 4.8 by 1.9 m in an 8 m corridor.
The turning reference curvature was 0.005 per meter. Replay sampling checks the
implementation independently; the continuous guarantee comes from interval
certificates, not those audit points. The 32-frame oncoming replay ends during
the maneuver and does not demonstrate complete avoidance recovery.

## Final validation blocker and reproduction

Several MATLAB batch invocations stalled before their first script statement.
Serial, display-free and JVM-free attempts also stalled. They produced no test
or scenario result and were terminated. An unrelated Vehicle Localization MATLAB
visualization remained open. Approval to restart the shared MathWorks Service
Host was requested because doing so may interrupt that session; the shared
service was not restarted without that approval. The cause of the startup stall
has not been established.

Run from the repository root once MATLAB startup works:

```bash
matlab -batch "addpath('scripts'); validateNonlinearPredictiveController('report/NONLINEAR_PREDICTIVE_CBF_REBUILD_20260928');"
```

The runner builds all three native verifiers into temporary storage, executes
the six selected suites, exports factory Code Analyzer findings, and requests
8-frame recovery/circular/solver-failure replays, a 210-frame oncoming stored-policy
replay with improvement disabled after initial admission, and a 32-frame online
oncoming replay. Those final outputs are not present yet and are not counted as
completed experiments.

## Scope and integration limits

Only exact state/model data and a single constant-parameter target are supported.
The invariant backup supports global straight or constant-curvature corridors.
Varying-curvature paths, finite fitted road boundaries, nonzero estimator bounds,
changing target parameters and braking-to-rest require additional certificates
and are rejected. Existing affine public-controller tests and scenario adapters
have not been migrated. The complete repository suite was not run; no all-tests
claim is made. Shared affine utilities remain callable for their research uses.

Initial admission and interval synthesis can exceed the timing budget. The
recorded 12.1-second frame is far above a 50 ms command period. Stored-policy
availability does not establish a measured real-time bound. The CLF gives
verified decrease only while zero-slack dissipation is compatible with the safe
continuation; arbitrary post-avoidance return is not asserted.

Unrelated target/observer edits, existing scenario edits, untracked manuscripts,
reference material, agent instructions, external dependencies and generated
native binaries are outside this change. No government-compliance or external
validation claim is made.
