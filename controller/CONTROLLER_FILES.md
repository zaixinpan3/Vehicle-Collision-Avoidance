# Controller source map

The controller and configuration contain nine MATLAB source files. See
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md) for budget inheritance and restoration
and its affine prediction scope.

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Fixed target epoch, input memory, solver orchestration, affine-result metadata and first input |
| `readControllerInputs.m` | Ego, one target and given-path normalization |
| `solvePredictiveControl.m` | Optimize CLF under inherited slack caps, restore PCBF when needed, and relinearize inaccurate predictions |
| `terminalContinuation.m` | Augmented free-pose endpoint core, construction bounds and anchor-based terminal geometry |
| `nonlinearBicycleModel.m` | Fiala bicycle RK4, variational tangents, road load, trim, and the single nominal cost-to-go CLF ([NOMINAL_CLF.md](NOMINAL_CLF.md)) |
| `modifiedFialaTire.m` | Combined-slip tire forces and derivatives |
| `predictiveSafetyGeometry.m` | Constant-acceleration/sideslip target flow, timed Gaussian and transported-flow guidance, exact ordinary-distance dual multipliers and fixed-multiplier rows; offline interval geometry |
| `laneGeometry.m` | Straight and circular given-path coordinates |
| `../config/collisionAvoidanceControllerConfig.m` | Defaults, merging and validation |

Each completed CLF result is rolled through the nonlinear model to measure
prediction agreement. Its full evaluable rollout becomes the next search
anchor; the entire local model is rebuilt before another accuracy refinement.
An accurate completed CLF result ends the current frame. The inherited prefix
slack sum and first-stage cap replace a primary solve when feasible; a failed
capped solve restores the primary problem. Extra rounds repair model accuracy.
There is no stored executable witness or alternate controller. Flow is an initializer,
not a safety certificate. Positive PCBF slack still denotes relaxation.
Road boundaries remain excluded, and the original given path defines the
single CLF. Terminal feedback is used only for construction.

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
positive-slack reporting, same-frame rebuilding of both objectives, nonlinear
prediction accuracy, first-accurate-iterate stopping, inherited budgets, failed-budget restoration,
and rejection when no accurate pair exists within the budget. `clfNominalRecoveryTest` checks target-free
nonlinear value decrease without requiring the optimizer to issue the nominal
construction feedback. `nominalClfTest` checks the trim, strict local Lyapunov
tail, fixed evaluation horizon, input memory, sampled large-state convergence
and a target-free closed loop. `../scripts/validateNominalClf.m` supplies sampled nominal-feedback diagnostics, not a regional proof. Model, geometry, terminal-core and
configuration tests retain their mathematical checks. Scenario campaigns and
independent nonlinear replay belong in `scripts/`, with results in `report/`.
