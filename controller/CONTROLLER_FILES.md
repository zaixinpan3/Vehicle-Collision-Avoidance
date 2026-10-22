# Controller source map

The controller and configuration contain nine MATLAB source files and one data table. See
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md) for the two stages and the
prediction scope, and [TERMINAL_SAFE_SET.md](TERMINAL_SAFE_SET.md) for the
terminal set and recursive feasibility.

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Target prediction updates (and target onset), input memory, solver orchestration, uncertainty scope and first input; the `resetNominalTrajectory` and `warmUp` actions |
| `readControllerInputs.m` | Ego, one target, timestamped error enclosures, given-path normalization and perceived boundary chains |
| `solvePredictiveControl.m` | RTI models from shifted, nominal or planned avoidance rollouts with terminal-constrained adaptive horizon search; PCBF/CLF subproblems, shared computation budget, affine/nonlinear discrepancy diagnostics and unchanged optimizer inputs |
| `terminalSafeSet.m` | Joint encounter terminal set and explicit terminal controller (`jointState`, `jointSuccessor`, `jointMember`, `control`): the closed-form joint continuation under the terminal law (the ego a kinematic vehicle braking at `a_b` with held steering and a forward-only stop, retaining first-exit memory; the target on its forecast), membership (mildness, road, clearance until the exit, the parked ego against the target's remaining track in closed form), its linearized rows, the anchor's holds (the cruise hold `u = u* + K e` and a bounded cruise-to-braking seed search distinct from the terminal law), nonlinear hard-row checks, and the offline certificate's helpers (Jacobians of the sampled error map, their ray averages `rayJacobians`, the sampled check, the state-row level) |
| `nonlinearBicycleModel.m` | The declared model (Fiala bicycle above the kinematic speed range, kinematic bicycle below it, smooth blend between; braking ends at rest) with RK4, variational tangents, road load, trim, and the single analytic quadratic CLF, whose matrix it reads from the precomputed table ([NOMINAL_CLF.md](NOMINAL_CLF.md)) |
| `modifiedFialaTire.m` | Combined-slip tire forces and derivatives |
| `predictiveSafetyGeometry.m` | Constant-acceleration/sideslip target prediction and analytic parameter-set enclosure, common-pose cancellation, one path-attracted circulation rollout with virtual wall guidance, ordinary-distance dual multipliers and fixed-multiplier rows; offline interval geometry |
| `laneGeometry.m` | Straight and circular given-path coordinates |
| `../config/collisionAvoidanceControllerConfig.m` | Defaults, merging and validation |
| `../config/clfMatrices.json` | CLF matrices per operating point, written before experiments by `../scripts/synthesizeClfMatrices.m` (LMI, YALMIP/SeDuMi); the controller only reads it |

Each reference rollout supplies one affine model and at most two native solves.
The first minimizes PCBF slack. With that slack capped, the second minimizes
CLF slack plus a soft, unit-scaled state/input deviation from this reference.
`rti.referenceDeviationWeight` controls that tradeoff without adding correction
bounds; cruise-path recovery remains in the CLF and planner attraction.
Local primal infeasibility permits fresh references and successively longer
horizons until the terminal-constrained solve completes or the common sample
deadline / an explicit user horizon cap stops search. Every fresh
reference with a target invokes the circulation planner, including infeasibility
retries, without a preceding risk gate. A newly observed encounter also requests
fresh guidance instead of shifting the previous unobstructed cruise plan.
Other warm references first attempt RTI on the shifted plan. The planner
generates one path-attracted circulation rollout using the predicted target
pose at each step. The route attractor stays on the nominal path; target
clearance, relative velocity and perceived boundaries determine its correction.
There is no prescribed offset or entry, hold or return schedule. The circulation
sense persists within the encounter to avoid left/right chattering. There is no candidate bank,
scoring or seed admission layer. The path attraction damps outward velocity;
the obstacle adds outward normal acceleration and a passing-direction
correction only when needed. Directional footprint and observer-error supports
replace disk-width clearance. Perceived boundary chains anticipate outward
momentum in arbitrary road geometry. Only range-cut ends extend as tangent
rays, and a scan remains frozen throughout its prediction. The cruise-speed loop, rather than target translation, determines
the preferred acceleration along the ego's current velocity. Actual-yaw gaps and state checks remain diagnostics. The
existing inverse Fiala guidance produces inputs and the nonlinear model
produces every state over exactly the requested number of holds, without
checking terminal membership or waiting for recovery. Only RTI's optimized command is executed; the field is
neither a robust safety certificate nor a minimum-deviation optimizer.
Cold references without a target use nominal feedback with boundary-field
guidance. The same guidance updates shifted seed inputs; optimized execution
inputs remain unchanged.
Warm references shift the previous inputs from the current estimate, retain
their absolute endpoint until the minimum prefix is reached, and lengthen
the horizon when the current geometric encounter preview requires it. Missing
tail holds use a checked cruise or braking hold from a terminal endpoint when
available.
A nonterminal seed endpoint is passed directly to the optimizer's terminal
constraints; it does not trigger an outer feedback-extension loop. The
initializer is not required to solve the terminal problem before optimization.
A retry discards the failed reference, old separation normals and terminal
context. It regenerates geometry from the same observation and error sets and
rebuilds dynamics, collision duals and terminal rows. The retry adds one minimum prefix to the failed horizon,
subject to an explicit horizon cap, including when the failed reference was
shifted. No nonlinear restoration loop or previous-plan execution
fallback runs. A failed retry, unfinished solve or exhausted budget returns
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
the collision and optional road rows. Road boundaries are disabled by default:
the input reader clears legacy lateral-clearance metadata unless
`controller.enforceRoadBoundaries` is explicitly enabled. Perceived boundary
chains remain available to the guide independently of that flag. With the legacy opt-in,
`lateralClearance = [right; left]` is the only lateral road constraint: every
ego rectangle corner stays inside it at every predicted node and hold midpoint, including the endpoint. Both ends of every
hold keep the sideslip inside the estimator's cone
`model.sideslipMaximum`. Horizon construction starts at `horizonSteps`; a shifted longer horizon shrinks
one hold per frame until that minimum. Infeasibility can request longer
horizons throughout the search; seed terminal membership does not select its length. The
given path defines the single CLF. `road.boundaryObservation` supplies oriented
world-coordinate chains with drivable space on the left and `extendStart` /
`extendEnd` flags marking perception cuts. An explicit empty scan means no
observed boundary. This interface always clears legacy hard lateral rows.
Without a boundary scan, `guideHalfWidthMeters=9` retains the legacy virtual
parallel-wall field (`Inf` disables that fallback). These fields are planner
guidance, not optimizer or terminal constraints. The
simulation option `LateralDeviationLimitMeters=10` independently stops and
reports a truth-replay violation. It measures the ego position reference,
without subtracting a vehicle footprint or tightening by estimated error.
`lateralDeviationLimitTest` covers this separation.

Observer inputs tighten sampled collision, road, physical-state, sideslip,
front model-domain and terminal constraints. Fiala saturation is part of the
vehicle model, without an additional rear handling cone. Each hold starts from the current enclosure
and propagates it through that hold only; the CLF row acts on the estimate. No
posterior-inclusion algorithm has been implemented, and terminal invariance
is established for the closed-form terminal law, not the complete Fiala
information-set dynamics (TERMINAL_SAFE_SET.md, Section 9); the
radii are not discarded to obtain a command. `observerPredictiveControlTest`
and `../scripts/verifyObserverControllerIntegration.m` exercise this interface
and expose that limitation. Complete nonlinear robust safety remains conditional
on the additional premises in `OBSERVER_ROBUST_PCBF_THEORY.tex`.

For the horizon collision rows, target yaw uncertainty is evaluated by the
exact directional support of the four rotated, offset vertices over the
allowed yaw interval. Each vertex maximum occurs at an interval endpoint or
its angular maximum. Subtracting the nominal support gives the required
inflation in the existing fixed dual normal. This is no larger than the old
rotation-chord enclosure for the same uncertainty set, including fully unknown
yaw. Directional position support and common-pose cancellation are retained.
The ego chord remains in the RTI rows because it is uniform over changes of
the nominal ego yaw; the planner can evaluate the full ego yaw support at
each fixed rollout pose. Terminal continuation bounds are unchanged. These
geometric improvements do not close the nonlinear information-set proof gaps.

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
use independent fixed-parameter target truth, and also count exceeding the
configured lateral limit as a failure, alongside collision or no solved
control output. Primary campaigns use the actual NRMM observer and
synthetic noisy sensors by default. Observer initialization failures remain
failed cases; exact-state diagnostics are excluded from the joint campaign
score. Terminal membership is a predicted-endpoint safety
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
controller uses a single path-attracted circulation field and a vehicle rollout. Historical experiment
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

`twoStagePredictiveControlTest` checks RTI per reference, adaptive horizon search, both objective stages,
unchanged optimized inputs, measurement updates, limits and explicit failure
after the allowed retry, without an execution fallback. `observerPredictiveControlTest` retains enclosure
integration and directional-family containment tests, including batch/scalar
support equivalence. `ordinaryDistanceDualTest`, `terminalSafeSetTest`,
`nominalClfTest`, `unrestrictedPredictiveControlTest` and `clfNominalRecoveryTest` retain their
geometry, terminal, CLF, unrestricted-search and recovery checks. Functional tests can use an
explicit generous wall budget; scenario timing uses the real-time defaults.
Routine numerical outputs can remain in temporary directories.
