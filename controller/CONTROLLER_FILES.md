# Controller source map

The controller and configuration contain nine MATLAB source files. See
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md) for the two-stage problem
and its affine prediction scope.

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Fixed target epoch, input memory, solver orchestration, affine-result metadata and first input |
| `readControllerInputs.m` | Ego, one target and given-path normalization |
| `solvePredictiveControl.m` | Roll out shifted inputs, restore PCBF feasibility with one flow refresh and bounded numerical expansion, then optimize the single nominal CLF |
| `terminalContinuation.m` | Augmented free-pose endpoint core, construction bounds and anchor-based terminal geometry |
| `nonlinearBicycleModel.m` | Fiala bicycle RK4, variational tangents, road load, trim, and the single nominal cost-to-go CLF ([NOMINAL_CLF.md](NOMINAL_CLF.md)) |
| `modifiedFialaTire.m` | Combined-slip tire forces and derivatives |
| `predictiveSafetyGeometry.m` | Known target motion, transported flow guidance and rectangle duals; offline interval geometry |
| `laneGeometry.m` | Straight and circular given-path coordinates |
| `../config/collisionAvoidanceControllerConfig.m` | Defaults, merging and validation |

There is no nonlinear candidate replay or correction in the online solve and
no stored executable witness. Previous inputs are rolled out from the measured
state to initialize the next pair of optimizations. The flow retry rebuilds the
entire local model and shares the frame budget. Positive PCBF slack is reported as relaxation.
Road boundaries are excluded, while the original given path remains the CLF
reference. Terminal feedback constructs the initialization/core geometry; it
is not a separate runtime fallback controller.

MATLAB Optimization Toolbox and Control System Toolbox are required.
Optional MATLAB Coder kernels are built outside the core by
`../scripts/buildControllerKernels.m`; interpreted MATLAB remains available.
Generated native binaries and `solver/` dependencies are not source artifacts
of this change. Kernel equivalence has dedicated tests.

`twoStagePredictiveControlTest` checks both objectives, the same CLF at all
target ranges, primary priority, affine dynamics with consistent anchors,
braking bounds, input increments distinct from physical steering limits,
positive-slack reporting, direct execution of finite solver results, and bounded
restoration without a backup. `clfNominalRecoveryTest` checks target-free
nonlinear value decrease without requiring the optimizer to issue the nominal
construction feedback. `nominalClfTest` checks the trim, strict local Lyapunov
tail, fixed evaluation horizon, input memory, sampled large-state convergence
and a target-free closed loop. `../scripts/validateNominalClf.m` supplies sampled nominal-feedback diagnostics, not a regional proof. Model, geometry, terminal-core and
configuration tests retain their mathematical checks. Scenario campaigns and
independent nonlinear replay belong in `scripts/`, with results in `report/`.
