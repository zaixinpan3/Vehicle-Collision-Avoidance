# Controller source map

The controller and configuration contain nine MATLAB source files. See
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md) for the two-stage problem
and its affine prediction scope.

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Fixed target epoch, input memory, solver orchestration, affine-result metadata and first input |
| `readControllerInputs.m` | Ego, one target and given-path normalization |
| `solvePredictiveControl.m` | Roll out shifted inputs during an encounter or the recovery feedback without one, take one PCBF/CLF RTI step (plus the anchor-deviation stage without collision rows), and reinitialize flow once if a warm or recovery-anchored solve returns no vector |
| `terminalContinuation.m` | Augmented free-pose endpoint core, construction bounds and anchor-based terminal geometry |
| `nonlinearBicycleModel.m` | Fiala bicycle RK4, variational tangents, road load, trim, lane CLF, and the recovery feedback, its terminal quadratic and cost-to-go CLF ([RECOVERY_CLF.md](RECOVERY_CLF.md)) |
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

`twoStagePredictiveControlTest` checks the two objectives during an encounter,
the third stage without one, primary priority,
affine dynamics with dynamically consistent anchors, braking bounds, numerical
iteration limits independent of actuator steering magnitude/slew, positive-slack reporting,
direct use of finite iteration-limit results, and missing-result failure
without fallback, and a single flow retry after warm-start failure. Online constraint audits are absent; unmeasured residuals
and margins remain NaN. `clfNominalRecoveryTest` checks that a target-free
frame issues the recovery feedback with zero CLF slack and decreases the
recovery CLF in the nonlinear hold model. `recoveryClfTest` checks the trim
equilibrium, the terminal Lyapunov equation, the cost-to-go identity,
convergence from distant and reversed states, and a target-free closed loop.
`../scripts/certifyRecoveryClf.m` is the sampled region certificate. Model, geometry, terminal-core and
configuration tests retain their mathematical checks. Scenario campaigns and
independent nonlinear replay belong in `scripts/`, with results in `report/`.
