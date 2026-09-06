# Core controller source budget

The core control algorithm has an upper limit of **20 source files**. Keep
related operations together; fewer files are welcome when their responsibilities
remain clear. The current implementation contains **18 source files**:
14 MATLAB modules, three native C++ bridges, and one controller configuration.

The count includes every source file under `controller/`, including any future
subdirectories, plus `config/collisionAvoidanceControllerConfig.m`. Tests,
scenario drivers, estimator/perception algorithms, theory documents, generated
binaries and third-party solver implementations have separate responsibilities.
Do not move controller helpers into those directories to evade the limit.

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Controller entry, carried certificate, terminal admission and diagnostics |
| `readPlanningInputs.m` | Input normalization and lane/target model construction |
| `formulateAvoidanceProblem.m` | Objective, hard constraints and CLF construction |
| `avoidanceStageSocp.m` | Sparse stage-state SOCP transcription |
| `solveHardCbfClf.m` | Numerical solve and independent constraint acceptance |
| `certifyAvoidancePlan.m` | Independent geometric verification of a plan |
| `avoidanceSafetyGeometry.m` | Robust lane and obstacle geometry rows |
| `rectangleConfigurationDistance.m` | Oriented rectangle configuration distance |
| `crossingCruiseReference.m` | Crossing-aware cruise reference |
| `laneGeometry.m` | Projection, Frenet pose, curvature and affine chart bounds |
| `ltvBicycleModel.m` | Held-input stage matrices, full prediction and initial braking schedule |
| `axleFriction.m` | Front/rear tire parameters and hard friction polygon rows |
| `stateUncertainty.m` | Estimator certificate validation, Frenet boxes, disturbance propagation, intersection and terminal rest |
| `targetPrediction.m` | Future target error envelope and analytic continuation support |
| `projectLanePolylineMex.cpp` | Native batched polyline projection |
| `laneFrameBoundsMex.cpp` | Native affine chart bounds |
| `solveAvoidanceSocpMex.cpp` | Native conic solver bridge |
| `../config/collisionAvoidanceControllerConfig.m` | Controller defaults, merging and validation |

The five grouped MATLAB modules expose named static methods, such as
`laneGeometry.project`, `ltvBicycleModel.predict`, `axleFriction.polygonRows`,
`stateUncertainty.readCertificate` and `targetPrediction.futureSupport`.
Their local helpers stay in the owning file. The controller entry signature,
model equations, constraints and solver acceptance criteria are unchanged by
this consolidation. Callers of the former standalone helpers must use the
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
