# Controller source map

The controller and configuration contain nine MATLAB source files and one data table. See
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md) for the two stages and the
prediction scope, and [TERMINAL_SAFE_SET.md](TERMINAL_SAFE_SET.md) for the
terminal set and recursive feasibility.

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Target prediction updates, input memory, solver orchestration, uncertainty scope and first input |
| `readControllerInputs.m` | Ego, one target, timestamped error enclosures and given-path normalization |
| `solvePredictiveControl.m` | One anchor per sample (startup potential-field rollout, or the shifted accepted plan extended by the terminal controller) over a horizon that reaches the terminal set; PCBF slack stage, CLF stage with the terminal CLF-level cone, step-size rule on the nonlinear rollout, full-step remainder trust |
| `terminalSafeSet.m` | CLF-tube terminal set: tube of the terminal controller, encounter end (exit or a relative motion outside the collision cone), level bisection, state-row level, the terminal controller's hold (the force-level feedback `w = K e` through the inverse Fiala curve, `inputOf`), nonlinear hard-row checks, the Jacobians of the sampled error map and their ray averages (`rayJacobians`) and the sampled check of the offline certificate |
| `nonlinearBicycleModel.m` | Fiala bicycle RK4, variational tangents, road load, trim, and the single analytic quadratic CLF, whose matrix it reads from the precomputed table ([NOMINAL_CLF.md](NOMINAL_CLF.md)) |
| `modifiedFialaTire.m` | Combined-slip tire forces and derivatives |
| `predictiveSafetyGeometry.m` | Constant-acceleration/sideslip target prediction and analytic parameter-set enclosure, common-pose cancellation, Zhai-inspired artificial potential guidance, ordinary-distance dual multipliers and fixed-multiplier rows; offline interval geometry |
| `laneGeometry.m` | Straight and circular given-path coordinates |
| `../config/collisionAvoidanceControllerConfig.m` | Defaults, merging and validation |
| `../config/clfMatrices.json` | CLF matrices per operating point, written before experiments by `../scripts/synthesizeClfMatrices.m` (LMI, YALMIP/SeDuMi); the controller only reads it |

Each sample builds one nonlinear anchor: a potential-field rollout at startup,
stopped at its first node in the terminal set; otherwise the shifted previous
plan, extended by terminal-controller holds when it is shorter than
`horizonSteps` or its endpoint has left the set. The anchor is linearized once;
the PCBF stage minimizes prefix safety slack and the CLF stage follows. A
primary problem that is primal infeasible inside the trust region is re-solved
with the trust scale doubled until feasible (at most `trustMaximumScale`); a
shifted plan still infeasible is solved once more from a fresh potential-field
rollout in the same way. The issued plan is the nonlinear rollout of the
largest acceptable step of the solution, where `alpha = 0`, the shifted plan,
is acceptable by construction on the declared model. Without an acceptable
step the problem is linearized again at most twice. An unusable shift, or a
frame still without an accepted plan, reports `noOptimizationSolution`. There
is no inherited slack budget or alternate controller. The input trust scale is
estimated from the full step's second-order remainder. Positive PCBF slack
still denotes relaxation.

The terminal set is the set of states whose CLF tube misses the target until
the encounter ends: the target leaves the perception range or its forecast
motion relative to the tube's box is outside their collision cone
(`terminalSafeSet`). In the convex
problem it is one cone on the endpoint's CLF level. The endpoint also carries
the collision and road rows. The road `lateralClearance = [right; left]` is the
only lateral constraint: every ego rectangle corner stays inside it at every
predicted node and hold midpoint, including the endpoint. Both ends of every
hold also keep the rear tire below Fiala saturation under the hold's braking
ratio (one second-order cone) and the sideslip inside the estimator's cone
`model.sideslipMaximum`. Without a target the horizon is `horizonSteps`. The
given path defines the single CLF; there is no separate path corridor.

Observer inputs tighten affine sampled collision, road, physical-state,
handling and terminal constraints. Each hold starts from the current enclosure
and propagates it through that hold only; the CLF row acts on the estimate. No
posterior-inclusion proof has been implemented, and the terminal set is
invariant only under exact information (TERMINAL_SAFE_SET.md, Section 9); the
radii are not discarded to obtain a command. `observerPredictiveControlTest`
and `../scripts/verifyObserverControllerIntegration.m` exercise this interface
and expose that limitation. Complete nonlinear robust safety remains conditional
on the additional premises in `OBSERVER_ROBUST_PCBF_THEORY.tex`.

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

Initialization metadata uses `movingTargetPotentialField`, `laneFeedbackRollout`
and `shiftedInputRollout`; `predictTarget` advances the prescribed target
motion. The experiment entry points are `runPotentialFieldCampaign` and
`extendPotentialFieldRecovery` in `scripts/`. Saved validation continuations
must use the current trace schema. Historical reports and recorded artifacts
retain the names used when those experiments were performed.

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

`twoStagePredictiveControlTest` checks both objectives, the same CLF at all
target ranges, primary priority, affine dynamics with consistent anchors,
braking bounds, input increments distinct from physical steering limits,
positive-slack reporting, terminal-set membership of the endpoint, that the
issued plan is the nonlinear rollout of its inputs, that the shifted plan is a
feasible candidate at the next samples, the shortest target-free horizon, the
single fresh re-solve of a failed shift, and that an unusable shift is
reported. `terminalSafeSetTest` checks invariance under the terminal
controller, nesting of levels, the state-row level, the end of an encounter
(exit or outside the collision cone) and the grid; `nominalClfTest` checks
the stored certificate of P and K (input bound, box, fill, contraction,
hold factor) and repeats its sampled check with another seed. `trustInnovationTest` checks the full-step remainder trust law:
the next scale, bounded growth, the clamped law, and attribution of a
posterior departure to the observer part. `clfNominalRecoveryTest` checks target-free
nonlinear value decrease without requiring the optimizer to issue the nominal
construction feedback. `nominalClfTest` checks the trim, strict local nonlinear
Lyapunov decrease, path-phase invariance, and rejection of retired cost-to-go
settings. `movingTargetPotentialFieldTest` checks attraction, motion-aware
repulsion, passing-side retention, and frame invariance.
`../scripts/validateNominalClf.m` supplies sampled nominal-feedback diagnostics, not a regional proof. Model, geometry and
configuration tests retain their mathematical checks. Scenario campaigns and
independent nonlinear replay belong in `scripts/`, with results in `report/`.
