# Controller source map

The controller and configuration contain eight MATLAB source files. See
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md) for the two stages, the
terminal set and the affine prediction scope.

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Target prediction updates, input memory, solver orchestration, uncertainty scope and first input |
| `readControllerInputs.m` | Ego, one target, timestamped error enclosures and given-path normalization |
| `solvePredictiveControl.m` | One anchor per sample (startup potential-field rollout or shifted plan) over a horizon that reaches the terminal set; PCBF slack stage, CLF stage, plan-innovation trust |
| `nonlinearBicycleModel.m` | Fiala bicycle RK4, variational tangents, road load, trim, and the single analytic quadratic CLF ([NOMINAL_CLF.md](NOMINAL_CLF.md)) |
| `modifiedFialaTire.m` | Combined-slip tire forces and derivatives |
| `predictiveSafetyGeometry.m` | Constant-acceleration/sideslip target prediction and analytic parameter-set enclosure, common-pose cancellation, Zhai-inspired artificial potential guidance, ordinary-distance dual multipliers and fixed-multiplier rows; offline interval geometry |
| `laneGeometry.m` | Straight and circular given-path coordinates |
| `../config/collisionAvoidanceControllerConfig.m` | Defaults, merging and validation |

Each sample builds one nonlinear anchor: a potential-field rollout at startup,
otherwise the shifted previous plan extended by path guidance. The rollout stops
at the first node of the terminal set, between `horizonSteps` and
`maximumHorizonSteps`. The anchor is linearized once; the PCBF stage minimizes
prefix safety slack and the CLF stage follows. A primary problem that is primal
infeasible inside the trust region is re-solved with the trust scale doubled
until feasible (at most `trustMaximumScale`); a shifted plan still infeasible is solved once more
from a fresh potential-field rollout in the same way. The completed CLF result is
issued directly, without a nonlinear replay or agreement test. An unusable shift,
or a frame still without a result, reports `noOptimizationSolution`. There is no
inherited slack budget or alternate controller. The input trust scale is estimated
from the next posterior's plan innovation. Positive PCBF slack still denotes
relaxation.

The terminal set is the perception-radius exit: at the last node the target
is beyond `encounterRangeMeters` (50 m) and separating. The road
`lateralClearance = [right; left]` is the only lateral constraint: every ego
rectangle corner stays inside it at every predicted node and hold midpoint,
including the endpoint. Without a target the horizon is `horizonSteps`. The
given path defines the single CLF; there is no separate path corridor.

Observer inputs tighten affine sampled collision, path, physical-state,
terminal and CLF constraints. No posterior-inclusion or robust terminal
invariance proof has been implemented; the radii are not discarded to obtain a
command. `observerPredictiveControlTest`
and `../scripts/verifyObserverControllerIntegration.m` exercise this interface
and expose that limitation. Complete nonlinear robust safety remains conditional
on the additional premises in `OBSERVER_ROBUST_PCBF_THEORY.tex`.

The latest point estimate always refreshes target A and beta, held constant
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
motion. The experiment entry points are `runPotentialFieldCampaign`,
`extendPotentialFieldRecovery` and `analyzePotentialFieldStudy.py` in `scripts/`. Saved validation continuations
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
zero multipliers and translation invariance. Overlap has no imposed direction.

`twoStagePredictiveControlTest` checks both objectives, the same CLF at all
target ranges, primary priority, affine dynamics with consistent anchors,
braking bounds, input increments distinct from physical steering limits,
positive-slack reporting, the terminal set (perception-radius exit, separating
speed, road rectangle), the shortest target-free horizon, the single fresh
re-solve of a failed shift, and that an unusable shift is reported.
`trustInnovationTest` checks the plan-innovation trust law: startup scale,
bounded growth, square-root shrinkage, attribution of a posterior departure to
the observer part, and the minimum scale. `clfNominalRecoveryTest` checks target-free
nonlinear value decrease without requiring the optimizer to issue the nominal
construction feedback. `nominalClfTest` checks the trim, strict local nonlinear
Lyapunov decrease, path-phase invariance, and rejection of retired cost-to-go
settings. `movingTargetPotentialFieldTest` checks attraction, motion-aware
repulsion, passing-side retention, and frame invariance.
`../scripts/validateNominalClf.m` supplies sampled nominal-feedback diagnostics, not a regional proof. Model, geometry and
configuration tests retain their mathematical checks. Scenario campaigns and
independent nonlinear replay belong in `scripts/`, with results in `report/`.
