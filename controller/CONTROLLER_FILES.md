# Controller source map

The controller and configuration contain nine MATLAB source files and one data table. See
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md) for the two stages and the
prediction scope, and [TERMINAL_SAFE_SET.md](TERMINAL_SAFE_SET.md) for the
terminal set and recursive feasibility.

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Target prediction updates (and target onset), input memory, solver orchestration, uncertainty scope and first input; the `resetNominalTrajectory` and `warmUp` actions |
| `readControllerInputs.m` | Ego, one target, timestamped error enclosures and given-path normalization |
| `solvePredictiveControl.m` | One RTI model from a shifted, nominal or bounded planned avoidance rollout; PCBF/CLF subproblems, shared computation budget, across-frame trust update and unchanged optimizer inputs |
| `terminalSafeSet.m` | Joint encounter terminal set and explicit terminal controller (`jointState`, `jointSuccessor`, `jointMember`, `control`): the closed-form joint continuation under the terminal law (the ego a kinematic vehicle braking at `a_b` with held steering and a forward-only stop, retaining first-exit memory; the target on its forecast), membership (mildness, road, clearance until the exit, the parked ego against the target's remaining track in closed form), its linearized rows, the anchor's holds (the cruise hold `u = u* + K e` and a bounded cruise-to-braking seed search distinct from the terminal law), nonlinear hard-row checks, and the offline certificate's helpers (Jacobians of the sampled error map, their ray averages `rayJacobians`, the sampled check, the state-row level) |
| `nonlinearBicycleModel.m` | The declared model (Fiala bicycle above the kinematic speed range, kinematic bicycle below it, smooth blend between; braking ends at rest) with RK4, variational tangents, road load, trim, and the single analytic quadratic CLF, whose matrix it reads from the precomputed table ([NOMINAL_CLF.md](NOMINAL_CLF.md)) |
| `modifiedFialaTire.m` | Combined-slip tire forces and derivatives |
| `predictiveSafetyGeometry.m` | Constant-acceleration/sideslip target prediction and analytic parameter-set enclosure, common-pose cancellation, a single conflict-interval-aware analytic avoidance guide, ordinary-distance dual multipliers and fixed-multiplier rows; offline interval geometry |
| `laneGeometry.m` | Straight and circular given-path coordinates |
| `../config/collisionAvoidanceControllerConfig.m` | Defaults, merging and validation |
| `../config/clfMatrices.json` | CLF matrices per operating point, written before experiments by `../scripts/synthesizeClfMatrices.m` (LMI, YALMIP/SeDuMi); the controller only reads it |

Each sample builds one affine model from one declared-model reference rollout
and runs at most two native solves for the PCBF/CLF objectives. A cold cruise
conflict invokes a single-path analytic reference planner. Its base curve keeps
the given path's curvature from the measured position and course. NRMM prediction and
rectangle checks identify the entire conflict interval. Ray exits from
rectangle support strips give a displacement demand at each time, allowing
different separating axes during approach, passing and departure. One
quintic rise-and-hold uses these deadlines; it may finish after the first
conflict and does not return within the horizon. Moderate braking shares an
acceleration budget with the deformation. One heading-aware geometry correction
precedes exactly one nonlinear vehicle rollout, with feedforward and feedback
tracking, curvature preview and a speed-dependent mild endpoint envelope. This rollout supplies
the dynamics and collision linearizations. Its geometry, node-clearance and
terminal diagnostics do not certify robust or whole-hold safety. Unmet guide
demands remain visible and do not trigger another rollout or RTI. The optimized inputs are never
replaced by a planner candidate. Other cold references use nominal feedback.
Warm references shift the previous inputs from the current estimate, retain
their absolute endpoint until the minimum prefix is reached, and extend a
terminal endpoint with a checked cruise or braking hold when available.
Those extension checks concern the reference and the sampled declared model;
they do not certify robust terminal invariance or veto an optimized input.
There is no nonlinear restoration loop, same-frame solve restart or
previous-plan execution fallback. Failed or unfinished optimization returns
no command. The first optimized input is issued unchanged. The ideal
mathematical recursive-feasibility theorem is independent of numerical RTI
success.

The terminal set is the set of states whose joint continuation under the
terminal law (the ego braking to a stop with its sideslip and course
curvature held, the target on its forecast) stays on the road and clears the
target until it leaves the perception range, after the stop over the target's
remaining track (`terminalSafeSet`). In the convex problem its conditions are
linearized at the anchor endpoint: collision rows at the continuation's nodes,
midpoints and parked pose, and road rows along it. The endpoint also carries
the collision and road rows. The road `lateralClearance = [right; left]` is the
only lateral constraint: every ego rectangle corner stays inside it at every
predicted node and hold midpoint, including the endpoint. Both ends of every
hold also keep the rear tire below Fiala saturation under the hold's braking
ratio (one second-order cone) and the sideslip inside the estimator's cone
`model.sideslipMaximum`. A cold no-target horizon is `horizonSteps`; a shifted longer horizon shrinks
one hold per frame until that minimum. The
given path defines the single CLF; there is no separate path corridor.

Observer inputs tighten sampled collision, road, physical-state, sideslip,
front model-domain and terminal constraints. Rear handling cones act on the
nominal state. Each hold starts from the current enclosure
and propagates it through that hold only; the CLF row acts on the estimate. No
posterior-inclusion algorithm has been implemented, and terminal invariance
is established for the closed-form terminal law, not the complete Fiala
information-set dynamics (TERMINAL_SAFE_SET.md, Section 9); the
radii are not discarded to obtain a command. `observerPredictiveControlTest`
and `../scripts/verifyObserverControllerIntegration.m` exercise this interface
and expose that limitation. Complete nonlinear robust safety remains conditional
on the additional premises in `OBSERVER_ROBUST_PCBF_THEORY.tex`.

`CLOSED_LOOP_PREDICTION_THEORY.md` develops the proposed feedback-aware policy
prediction, varying finite-horizon tubes and policy-shift proof. Fixed-width
invariant tubes are optional diagnostics. `../scripts/verifyJointNrmmPrediction.m`
propagates the actual NRMM joint local maps with shared future sensor noise;
the current input is common and future inputs include feedback. The research verifier
`../scripts/verifyFeedbackAwarePrediction.m` tests local linear invariance,
nonlinear feedback probes and correlated target response; it does not change
the runtime or certify nonlinear avoidance/parking.

The estimator shares the controller's vehicle model, input and domain; see
`../estimator/MODEL_AIDED_ESTIMATION.md`. The latest point estimate always refreshes target A and beta, held constant
only within the current prediction. Unavailable or stale certificates do not
block execution; unsupported uncertainty padding is omitted without declaring
zero estimation error. No assumption-status diagnostics are produced. Usable finite
enclosures still tighten the same optimization. Experimental collision checks
use independent fixed-parameter target truth, and failure means collision or
no solved control output. Terminal membership is a predicted-endpoint safety
certificate, not an execution mode, shutdown trigger or nominal-task success
criterion. Scenario runs default to no frame limit and continue until failure
or sustained nominal recovery. Finite frame limits are explicit diagnostics.
Campaigns measure recovery to lane keeping and cruise control after the
encounter. Recovery, margin shortfalls and unverified theory
premises are separate observations. `nominalRecoveryValidationTest` and
`../scripts/verifyExperimentalAssumptionPolicy.m` exercise these distinctions.

Initialization metadata uses `nominalFeedbackRollout`, `plannedAvoidanceRollout`
and `shiftedInputRollout`.
The legacy experiment entry names `runPotentialFieldCampaign` and
`extendPotentialFieldRecovery` remain for existing campaign callers; the
controller uses a single analytic avoidance guide and one vehicle rollout. Historical experiment
records describe the algorithm version that actually produced them.

MATLAB Control System Toolbox and the compiled Clarabel 0.11.1 adapter are required.
Optimization Toolbox is used by independent comparison tests.
Optional MATLAB Coder kernels are built outside the core by
`../scripts/buildControllerKernels.m`; interpreted MATLAB remains available.
Generated native binaries and `solver/` dependencies are not source artifacts
of this change. Kernel equivalence has dedicated tests. The retired signed
collision-direction kernel is not used. Ordinary-distance dual multipliers
come from the exact closest-feature normal. Trajectory problems use one sparse
conic quadratic solver through `../scripts/native/predictiveConicSolverMex.cpp`.
Build its external Clarabel dependency under `solver/clarabel`, then run
`../scripts/buildPredictiveConicSolver.m`. The adapter contains numerical
interface code, not another controller or alternate objective.
`predictiveConicSolverTest` checks quadratic/epigraph equivalence, second-order
cones, equalities and the exclusion of infeasibility certificates.
Rebuild kernels after changing the configuration structure passed to the
nominal-value MEX.

`ordinaryDistanceDualTest` compares the optimized ordinary dual with geometric
rectangle distance, checks full ego size, fixed-multiplier yaw derivatives,
zero multipliers and translation invariance. At overlap the trajectory rows
use the feasible dual of the least-penetrated rectangle axis
(`supportLinearization`), which the test checks as an escape direction.

`twoStagePredictiveControlTest` checks one-model RTI, both objective stages,
unchanged optimized inputs, measurement updates, limits and explicit failure
without retries or fallback. `observerPredictiveControlTest` retains enclosure
integration and directional-family containment tests, including batch/scalar
support equivalence. `ordinaryDistanceDualTest`, `terminalSafeSetTest`,
`nominalClfTest`, `trustInnovationTest` and `clfNominalRecoveryTest` retain their
geometry, terminal, CLF, trust and recovery checks. Functional tests can use an
explicit generous wall budget; scenario timing uses the real-time defaults.
Routine numerical outputs can remain in temporary directories.
