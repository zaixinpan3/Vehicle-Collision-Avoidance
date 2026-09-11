# Core controller source budget

The core control algorithm has an upper limit of **20 source files**. The
current implementation contains **20**: 13 MATLAB modules, five native C++
translation units, one native header, and one controller configuration.
Related operations stay in the module that owns their responsibility.

The count includes every source file under `controller/`, including future
subdirectories, plus `config/collisionAvoidanceControllerConfig.m`. Tests,
scenario drivers, estimator/perception algorithms, theory documents, generated
binaries and third-party solver implementations have separate responsibilities.
Do not move controller helpers into those directories to evade the limit.

| Source | Responsibility and principal interfaces |
| --- | --- |
| `collisionAvoidanceController.m` | Public controller entry, encounter lifecycle and diagnostics |
| `readPlanningInputs.m` | Input normalization and lane/target model construction |
| `hardEncounterBarrier.m` | Complete-encounter planning (`plan`), retained deadline, exit obligations and certified execution |
| `formulateAvoidanceProblem.m` | Objective, hard swept constraints and predictive CLF cones |
| `avoidanceStageQp.m` | Sparse transcription (`build`) and updates using explicit row maps (`updateBounds`) |
| `solveHardCbfClf.m` | Lexicographic solve (`solve`) and independent plan verification (`certify`) |
| `avoidanceSafetyGeometry.m` | Swept separation (`build`), continuous normal proposals (`optimizeNormals`), rectangle distance (`rectangleDistance`) and shared numeric kernels (`cellRows`, `projectRows`) |
| `laneGeometry.m` | Polyline/arc projection, Frenet poses and chart bounds |
| `ltvBicycleModel.m` | Held-input prediction, nonlinear dynamics, signed road forces (`roadLoad`) and slip-domain rows (`slipRows`) |
| `modifiedFialaTire.m` | Modified Fiala forces, tangents and tire parameters |
| `stateUncertainty.m` | Estimator bounds, held-interval enclosures, intersection and sampled-feedback transition (`sampledFeedbackTransition`) |
| `targetPrediction.m` | Target inclusion, nominal motion, footprint support and encounter lifecycle |
| `fialaCertificate.m` | Validated nonlinear residuals (`residual`), held-feedback samples (`sample`), prescribed sequences (`sequence`) and shared constants (`parameters`) |
| `projectLanePolylineMex.cpp` | Native batched polyline projection |
| `laneFrameBoundsMex.cpp` | Native affine chart bounds |
| `solveAvoidanceSocpMex.cpp` | Native conic solver bridge |
| `fialaIntervalCore.hpp` | Directed interval arithmetic and authoritative nonlinear Fiala equations |
| `fialaIntervalMex.cpp` | Native domain-wide nonlinear residual verifier |
| `fialaFeedbackSampleMex.cpp` | Native correlated sampled-feedback flow verifier |
| `../config/collisionAvoidanceControllerConfig.m` | Controller defaults, merging and validation |

## Consolidated interfaces

The public `collisionAvoidanceController` signature is unchanged. Supporting
operations use named static methods; their local helpers remain in the owning
file. Repository callers and native build adapters use the interfaces below.
The removed standalone files have no compatibility wrappers.

| Former interface | Current interface |
| --- | --- |
| `planCompleteEncounter` | `hardEncounterBarrier.plan` |
| `avoidanceStageQp(...)` | `avoidanceStageQp.build(...)` |
| `updateAvoidanceStageBounds` | `avoidanceStageQp.updateBounds` |
| `solveHardCbfClf(...)` | `solveHardCbfClf.solve(...)` |
| `certifyAvoidancePlan` | `solveHardCbfClf.certify` |
| `avoidanceSafetyGeometry(model,prediction)` | `avoidanceSafetyGeometry.build(model,prediction)` |
| `avoidanceSafetyGeometry('cellRows',data)` | `avoidanceSafetyGeometry.cellRows(data)` |
| `avoidanceSafetyGeometry('projectRows',data)` | `avoidanceSafetyGeometry.projectRows(data)` |
| `optimizeSeparationNormals` | `avoidanceSafetyGeometry.optimizeNormals` |
| `rectangleConfigurationDistance` | `avoidanceSafetyGeometry.rectangleDistance` |
| `longitudinalRoadLoad` | `ltvBicycleModel.roadLoad` |
| `tireSlipRows` | `ltvBicycleModel.slipRows` |
| `sampledFeedbackTransition` | `stateUncertainty.sampledFeedbackTransition` |
| `fialaResidualCertificate` | `fialaCertificate.residual` |
| `certifyFialaFeedbackSample` | `fialaCertificate.sample` |
| `certifyFialaFeedbackSequence` | `fialaCertificate.sequence` |
| `fialaIntervalParameters` | `fialaCertificate.parameters` |

This consolidation changes source organization and call names. The hard
constraints, independent acceptance checks, explicit affine residual bounds,
retained deadlines and numerical equations are preserved. In particular,
`fialaCertificate.sample` and `.sequence` retain their documented limited
scope: nonlinear ego-flow certification does not yet establish an integrated
nonlinear collision/road/target-exit certificate. See
[Fiala feedback tightening](FIALA_FEEDBACK_TIGHTENING.md).

After updating an existing MATLAB session, clear the changed modules or restart
the session before running. Regenerate the MATLAB Coder kernels with
`buildAvoidanceGeometryKernel` and `buildBicycleNominalKernel`; generated adapters
and binaries remain under `solver/` and are excluded from source control.
When changing native lane geometry paths, `clear laneGeometry` releases both
retained geometry backend handles.

## Verification

`tests/controllerSourceBudgetTest.m` enforces the recursive source count. The
2026-09-11 consolidation passed all 650 repository tests with no failed or
incomplete cases. All five geometry/prediction MEX kernels regenerated
successfully. Factory Code Analyzer checked 42 changed MATLAB files and
introduced no new findings; 23 existing suggestions matched the original
sources. Source comparison also verified the relocated bodies and migrated
callers after the declared interface substitutions.

Run all behavior regressions from the repository root:

```matlab
results = runtests('tests');
assertSuccess(results);
```

The source inventory can also be checked directly:

```matlab
sources = [dir(fullfile('controller', '**', '*.m')); ...
    dir(fullfile('controller', '**', '*.cpp')); ...
    dir(fullfile('controller', '**', '*.c')); ...
    dir(fullfile('controller', '**', '*.h')); ...
    dir(fullfile('controller', '**', '*.hpp')); ...
    dir(fullfile('config', 'collisionAvoidanceControllerConfig.m'))];
assert(numel(sources) <= 20, 'Core controller exceeds its source-file budget.');
```
