# Controller source map

The controller and its configuration contain **eight MATLAB source files**.
The current algorithm is described in [PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md).

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Current joint state, optimizer call, direct first control output and warm start state |
| `readControllerInputs.m` | Ego, single target and global road corridor input normalization |
| `solvePredictiveControl.m` | SCvx, primary safety LP, secondary lane CLF QP and numerical trust region |
| `nonlinearBicycleModel.m` | Nonlinear Fiala dynamics, RK4, analytic tangents, road load, cruise trim and lane CLF |
| `modifiedFialaTire.m` | Combined-slip tire forces and derivatives |
| `predictiveSafetyGeometry.m` | Constant-acceleration/constant-sideslip target flow, rectangle distance duals and MPC terminal geometry |
| `laneGeometry.m` | Straight and circular lane coordinates and projection |
| `../config/collisionAvoidanceControllerConfig.m` | Active defaults, strict merging and validation |

`controllerState.inputTrajectory` and `stateTrajectory` initialize the next
solve. Each call applies its newly returned first control regardless of
whether its safety slack is zero or positive. Failed optimization with no
result raises an error. See the architecture for nominal model assumptions.

The old affine solvers, interval/MPFR verifiers, support/tube algorithms,
trajectory admission contracts, native bridges and dependent studies have
been deleted. Git history retains their source. The retired implementations
are no longer kept as executable research utilities.

MATLAB Optimization Toolbox and Control System Toolbox are required. The
controller has no native build dependency. Estimator and perception modules
remain separate research components. Their own certificates and numerical
kernels do not authorize controller execution.

Current behavior tests are `nonlinearPredictiveSafetyTest`,
`controllerInputGeometryTest`, `longitudinalRoadLoadTest`,
`modifiedFialaTireTest` and `collisionAvoidanceControllerConfigTest`.
`controllerSourceBudgetTest` limits the core to eight source files.
