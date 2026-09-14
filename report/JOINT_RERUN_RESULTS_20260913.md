# Estimator-controller straight experiment: admission failure

September 13, 2026. Tested source `defd45943bd2217a014e99061faf510548355148`. The existing
estimator-controller entry was actually run with a requested duration of 30 s.
It failed at t = 0 before any closed-loop control hold executed. No avoidance,
cruise recovery or complete-pipeline real-time pass is claimed. Controller and
observer code and configuration defaults are unchanged by this experiment.

## Full existing joint entry

`runOncomingVehicleAvoidanceScenario` was called directly with
`UseStateEstimator=true`; the controller-only gate in
`runStraightRealtimeValidation` was not used because this request specifically
asks for the joint experiment. Settings retain the existing straight joint
validation definition: ego/target speeds 10 m/s, initial target position
(100,0.8) m, target width 2 m, radar range 30 m, 0.1 s control period,
16-stage horizon, and `finiteSensingValidationConfig` for the PassVeh14DOF
plant. Seed is 20260913. The NRMM samples every 0.0125 s; this runtime profile
uses one RK4 step per sensor sample, including the runtime's integration-defect
accounting. The main entry retains the integration configuration's default
provisional target-speed prior (15 m/s); no target is acquired before failure.

The independent PassVeh14DOF template loads and initializes. Offline preparation
runs and is excluded from online timing. All 24 pipeline preparation probes
fail admission (empty-target or nonexact-input conditions). The first actual
online call receives no target because the target is outside the 30 m radar
range. The current controller requires exactly one persistent target and returns:

`collisionAvoidanceController:invalidExactScene`

The runner stops with zero executed control holds. The failed frame takes
21.798 ms: observer 9.873 ms, road fitting 7.852 ms and controller 3.478 ms
(the precise fields are retained in `joint-summary.json`). This is a quick
rejection, not the runtime of an accepted MPC solve or a successful pipeline.
No physical avoidance segment executes and its distance/recovery metrics are
unavailable. `joint.mat` retains the failure context, estimator output,
configuration, preparation and timing evidence.

## Admission probes using actual estimator outputs

To separate the first failure from later restrictions, initialize the actual
NRMM adapter on a straight target at (25,0.8) m moving at (-10,0) m/s, with
0.5 s synthetic measurement history and the same bounded sensor noise. The
provisional speed prior is explicitly 10 m/s for these supplemental probes.
Use a fixed straight road, y = +/-5 m, and the current default declared-plant
controller at 10 m/s with a 0.1 s hold and 16 stages. No true state replaces
an estimate, and no current-state error radius is reduced.

| Probe | Difference from the actual in-range output | Observed result |
| --- | --- | --- |
| No published target | Empty target argument and empty bundled target list | `invalidExactScene` |
| Actual in-range output | None | `nonexactStudyInput`: nonzero jerk/yaw-acceleration bounds |
| Physical plant allowance | Add existing empirical plant residual rates | `nonexactStudyInput`: process residuals excluded |
| Exact scene motion | Declare zero jerk and yaw acceleration for this truly constant-velocity target; retain all estimated states and radii | `unboundedTargetSupport` |

Each is a separate first-call diagnostic, with explicit empty carried state;
none executes a control hold. A 3 s certificate search budget bounds diagnostics,
but each failure occurs before a complete optimization search. They are not
four completed closed-loop scenes. `interface.mat/json` and the individual
MAT files retain exact inputs and results. The observer acquisition call takes
13.893 ms; the four rejection calls take 7.566, 6.619, 2.626 and 98.745 ms.
These are individual diagnostic timings, not a pipeline benchmark.

The restrictions occur in three distinct places:

- `hardEncounterBarrier.validateAdmission` requires one persistent target and
  zero plant/model residual rates. This is incompatible with the existing
  finite-range lifecycle and PassVeh14DOF empirical-residual configuration.
- `nrmmControllerErrorBounds` publishes bounded future motion under its target
  domain, whereas `targetPrediction.admitExact` rejects nonzero jerk and yaw
  acceleration. Constant curvature/tangential acceleration and exact Cartesian
  constant acceleration are different contracts in general. Only the truly
  straight constant-velocity diagnostic justifies the zero future derivatives.
- The current terminal set requests separation from every member of the target
  box for all future time. Even with zero future derivative uncertainty,
  current acceleration uncertainty remains. In the saved probe, the acceleration
  center is (0,0) m/s^2 with radii (2.35849,2.35849) m/s^2. For any nonzero
  normal n, the quadratic coefficient of the directional position support is
  0.5 times (|n_x| + |n_y|) times 2.35849, which is positive. Its all-future
  supremum is therefore infinite. `localFutureSupport` and `localAdmissibleNormal`
  reject the terminal halfspace. This is a consequence of this robust
  all-future construction, not a measured collision or proof that finite-time
  avoidance of the actual target is impossible.

The exact-motion probe changes only scenario knowledge about future motion;
it does not erase estimator uncertainty or claim the nonlinear ego executes
the declared affine generator. Removing the earlier guards alone cannot make
these different model and terminal premises compatible.

## Separate observer lifecycle check

To check the observer independently of the failed control admission, prescribe
ego motion (10t,0), target motion (100-10t,0.8), and their constant velocities
for 30 s. This is an observer-only synthetic test: prescribed trajectories
cross and no avoidance claim is made. The same NRMM adapter, noise seed and
80 Hz integration are used; output is evaluated at 301 controller instants.
The target-speed prior is explicitly 10 m/s.

- All 301 ego state boxes contain truth at the evaluated times, and all sampled
  ego domain/premise checks pass.
- The target is published at 29 evaluated instants, from 3.6 through 6.4 s.
  All eight target state components, including wrapped body heading and yaw
  rate, lie within their published bounds at these times.
- At 6.5 s the target is coasting internally and is no longer published. At
  7.0 s it returns to dormant status after the configured coast timeout.
- The observer/adapter call median is 8.100 ms and maximum
  14.928 ms. These exclude controller and road fitting, and do not
  establish joint real-time feasibility or closed-loop maneuver performance.

`observer-lifecycle.mat/json` retain frame measurements, estimates' errors,
bounds and track states. This supplements the failed joint attempt; it does
not convert that failure into an estimator-controller success.

## Validation, reproduction and next work

Executed: the existing joint entry in MATLAB R2026a Update 3 with
Vehicle Dynamics Blockset; four isolated admission probes through MATLAB MCP;
and the 301-output observer lifecycle through MATLAB MCP. The temporary
probe initially used incompatible empty-struct accumulation, and the first
observer diagnostic used the wrong output field name for heading; these
harness errors were corrected before complete saved diagnostic results were
produced. No algorithm changes or unit-test reruns are claimed. Tracked source
comparison and `git diff --check` verify the result-document-only scope.

Original outputs and the complete runnable MATLAB scripts are retained at:

`/home/zai/.cache/collisionAvoidance/joint-rerun-20260913`

Reproduce with `runJoint.m`, `probeInterface.m`, and `runObserverLifecycle.m`
from that directory. The first two are scripts; the third is a function called
after adding that directory to the MATLAB path. The native solver and observer
paths are explicit in each file. Source commit and native hashes are recorded
in `source-provenance.json`. Archive copies are identified as exports.

Before a successful joint avoidance test can be expected, the controller must
support the finite-range target lifecycle and a target-motion/terminal contract
that admits the observer's actual uncertainty. Independent nonlinear-plant
validation additionally needs a supported model-residual contract. These are
identified follow-up requirements, not changes implemented by this rerun.
