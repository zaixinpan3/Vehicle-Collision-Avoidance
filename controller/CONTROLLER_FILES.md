# Controller source map

The controller and configuration contain nine MATLAB source files and one data table. See
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md) for the two stages and the
prediction scope, and [TERMINAL_SAFE_SET.md](TERMINAL_SAFE_SET.md) for the
terminal set and recursive feasibility.

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Target prediction updates (and target onset), input memory, solver orchestration, uncertainty scope and first input; the `resetNominalTrajectory` and `warmUp` actions |
| `readControllerInputs.m` | Ego, one target, timestamped error enclosures and given-path normalization |
| `solvePredictiveControl.m` | Successive convexification from nominal or shifted-input rollout references; repeated dynamics, collision-dual and terminal updates; internal elastic restoration, PCBF/CLF subproblems, nonlinear merit and trust decisions, unmodified selected optimizer inputs |
| `terminalSafeSet.m` | Mild braking-continuation terminal set: the closed-form joint continuation under the terminal law (the ego a kinematic vehicle braking at `a_b` with its steering, hence sideslip and course curvature, held; the target on its forecast), membership (mildness, road, clearance until the exit, the parked ego against the target's remaining track in closed form), its linearized rows, the anchor's holds (the cruise hold `u = u* + K e` and the hold toward the held steering), nonlinear hard-row checks, and the offline certificate's helpers (Jacobians of the sampled error map, their ray averages `rayJacobians`, the sampled check, the state-row level) |
| `nonlinearBicycleModel.m` | The declared model (Fiala bicycle above the kinematic speed range, kinematic bicycle below it, smooth blend between; braking ends at rest) with RK4, variational tangents, road load, trim, and the single analytic quadratic CLF, whose matrix it reads from the precomputed table ([NOMINAL_CLF.md](NOMINAL_CLF.md)) |
| `modifiedFialaTire.m` | Combined-slip tire forces and derivatives |
| `predictiveSafetyGeometry.m` | Constant-acceleration/sideslip target prediction and analytic parameter-set enclosure, common-pose cancellation, ordinary-distance dual multipliers and fixed-multiplier rows; offline interval geometry |
| `laneGeometry.m` | Straight and circular given-path coordinates |
| `../config/collisionAvoidanceControllerConfig.m` | Defaults, merging and validation |
| `../config/clfMatrices.json` | CLF matrices per operating point, written before experiments by `../scripts/synthesizeClfMatrices.m` (LMI, YALMIP/SeDuMi); the controller only reads it |

Each sample iterates convex subproblems around exact declared-model rollouts.
A starting reference may be unsafe or nonterminal; temporary elastic relaxation
allows local restoration. Accepted nonlinear rollouts update the dynamics,
collision multipliers and terminal rows. Only a completed accepted optimization
with small elastic and nonlinear hard residual is eligible for return. The first
input is issued unchanged. There is no separate avoidance-path generator,
inherited slack budget or alternate controller. Numerical failures remain
separate from the ideal mathematical recursive-feasibility claim.

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
`model.sideslipMaximum`. Without a target the horizon is `horizonSteps`. The
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
no solved control output. Recovery, margin shortfalls and unverified theory
premises are separate observations. `nominalRecoveryValidationTest` and
`../scripts/verifyExperimentalAssumptionPolicy.m` exercise these distinctions.

Initialization metadata uses `nominalFeedbackRollout` and `shiftedInputRollout`.
The legacy experiment entry names `runPotentialFieldCampaign` and
`extendPotentialFieldRecovery` remain for existing campaign callers; the
controller itself no longer uses potential-field guidance. Historical experiment
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

`twoStagePredictiveControlTest` checks elastic restoration, repeated nonlinear
relinearization, selected unmodified inputs, lexicographic priority, terminal
membership, road and handling limits, actuator bounds and failure without an
elastic fallback. `ordinaryDistanceDualTest` also checks consistent lateral
orientation at symmetric overlap. `terminalSafeSetTest`, `nominalClfTest`,
`trustInnovationTest` and `clfNominalRecoveryTest` retain their model, terminal,
CLF, across-frame trust and recovery checks. Scenario campaigns use independent
nonlinear replay and can keep routine results in temporary directories.
