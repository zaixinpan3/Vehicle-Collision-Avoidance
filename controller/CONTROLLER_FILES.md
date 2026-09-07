# Core controller source budget

The core control algorithm has an upper limit of **20 source files**. Keep
related operations together; fewer files are welcome when their responsibilities
remain clear. The current implementation contains **20 source files**:
16 MATLAB modules, three native C++ bridges, and one controller configuration.

The count includes every source file under `controller/`, including any future
subdirectories, plus `config/collisionAvoidanceControllerConfig.m`. Tests,
scenario drivers, estimator/perception algorithms, theory documents, generated
binaries and third-party solver implementations have separate responsibilities.
Do not move controller helpers into those directories to evade the limit.

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Encounter lifecycle, carried finite certificate and diagnostics |
| `readPlanningInputs.m` | Input normalization and lane/target model construction |
| `formulateAvoidanceProblem.m` | Maneuver objective, hard swept constraints and predictive CLF cones |
| `avoidanceStageQp.m` | Sparse quadratic/Lorentz-cone transcription and truncation |
| `solveHardCbfClf.m` | Numerical solve and independent constraint acceptance |
| `certifyAvoidancePlan.m` | Independent geometric verification of a plan |
| `avoidanceSafetyGeometry.m` | Swept lane and obstacle geometry rows |
| `rectangleConfigurationDistance.m` | Oriented rectangle configuration distance |
| `terminalDissipation.m` | Separately validated optional rest-funnel construction |
| `laneGeometry.m` | Polyline and analytic-arc projection, Frenet poses and chart bounds |
| `ltvBicycleModel.m` | Held-input stage matrices, full prediction and initial braking schedule |
| `longitudinalRoadLoad.m` | Signed aerodynamic and equivalent rolling forces with their speed derivative |
| `modifiedFialaTire.m` | Modified Fiala forces, local tire tangents and nominal combined-force rows |
| `tireSlipRows.m` | Robust slip-angle model-domain rows |
| `stateUncertainty.m` | Estimator certificate validation, Frenet boxes, disturbance propagation, intersection and terminal rest |
| `targetPrediction.m` | Finite target inclusion, nominal motion, footprint support and encounter lifecycle |
| `projectLanePolylineMex.cpp` | Native batched polyline projection |
| `laneFrameBoundsMex.cpp` | Native affine chart bounds |
| `solveAvoidanceSocpMex.cpp` | Native conic solver bridge |
| `../config/collisionAvoidanceControllerConfig.m` | Controller defaults, merging and validation |

The five grouped MATLAB modules expose named static methods, such as
`laneGeometry.project`, `ltvBicycleModel.finitePredict`, `modifiedFialaTire.evaluate`,
`stateUncertainty.flowTube` and `targetPrediction.admit` / `targetPrediction.finiteFlow`.
Their local helpers stay in the owning file. Version 10 retains the controller
entry signature and supports finite-sensing and optional strong encounter contracts and conic solver-hook
interface documented in `PCBF_CLF_ARCHITECTURE.md`. Callers of the former standalone helpers must use the
corresponding module methods; repository scripts and tests use these interfaces.

When changing native geometry paths in a running MATLAB session, use
`clear laneGeometry` to release both retained geometry backend handles.
The build recipe and native/reference comparison test perform this reset.

From the repository root, verify the source budget with:

```matlab
sources = [dir(fullfile('controller', '**', '*.m')); ...
    dir(fullfile('controller', '**', '*.cpp')); ...
    dir(fullfile('controller', '**', '*.c')); ...
    dir(fullfile('controller', '**', '*.h')); ...
    dir(fullfile('controller', '**', '*.hpp')); ...
    dir(fullfile('config', 'collisionAvoidanceControllerConfig.m'))];
assert(numel(sources) <= 20, 'Core controller exceeds its source-file budget.');
```
