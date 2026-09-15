# Core controller source budget

The core control algorithm has an upper limit of **20 source files**. The
current implementation contains **20**: 13 MATLAB modules, five native C++
translation units, one native header, and one controller configuration.
The controller uses one SOCP with two actuator inputs and a penalized CLF slack.
Its format-25 state stores only the previous input and timestamp.
See [the algorithm and guarantees](SINGLE_SOLVE_CBF_CLF.md).
Related operations stay in the module that owns their responsibility.

The count includes every source file under `controller/`, including future
subdirectories, plus `config/collisionAvoidanceControllerConfig.m`. Tests,
scenario drivers, estimator/perception algorithms, theory documents, generated
binaries and third-party solver implementations have separate responsibilities.
Do not move controller helpers into those directories to evade the limit.

| Source | Responsibility and principal interfaces |
| --- | --- |
| `collisionAvoidanceController.m` | Public target-array entry; one solve per hold, applied-input memory, error on any failed solve |
| `readPlanningInputs.m` | Input normalization, target-departure sensor declaration and lane/target model construction |
| `hardEncounterBarrier.m` | Sampled obstacle barrier and swept intersample rows (`rows`) |
| `formulateAvoidanceProblem.m` | Three-variable objective, hard swept safety and obstacle rows, and one sampled CLF cone with optimized nonnegative slack |
| `avoidanceStageQp.m` | Sparse transcription with per-stage violation columns (`build`) and bound updates using explicit row maps (`updateBounds`) |
| `solveHardCbfClf.m` | Single hard-safety, soft-CLF SOCP (`constrained`) and strict solver-status handling; legacy two-coordinate row compaction does not apply to this program |
| `avoidanceSafetyGeometry.m` | Swept separation (`build`), passing-side proposals (`passingNormals`), continuous normal proposals (`optimizeNormals`), rectangle distance (`rectangleDistance`) and shared numeric kernels (`cellRows`, `projectRows`) |
| `laneGeometry.m` | Polyline/arc projection, Frenet poses and chart bounds |
| `ltvBicycleModel.m` | Held-input prediction, affine input-family swept prediction (`fixedPredict`), sampled cruise synthesis (`sampledCruise`), nonlinear dynamics, signed road forces (`roadLoad`) and slip-domain rows (`slipRows`) |
| `modifiedFialaTire.m` | Modified Fiala forces, tangents and tire parameters |
| `stateUncertainty.m` | Estimator bounds, held-interval enclosures, intersection and sampled-feedback transition (`sampledFeedbackTransition`) |
| `targetPrediction.m` | Bounded target admission (`admitOnline`), reachable-box conditioning (`condition`), absolute-time flow, offline uncertainty studies and footprint support |
| `fialaCertificate.m` | Validated nonlinear residuals (`residual`), held-feedback samples (`sample`), prescribed sequences (`sequence`) and shared constants (`parameters`) |
| `projectLanePolylineMex.cpp` | Native batched polyline projection |
| `laneFrameBoundsMex.cpp` | Native affine chart bounds |
| `solveAvoidanceSocpMex.cpp` | Native conic solver bridge |
| `fialaIntervalCore.hpp` | Directed interval arithmetic and authoritative nonlinear Fiala equations |
| `fialaIntervalMex.cpp` | Native domain-wide nonlinear residual verifier |
| `fialaFeedbackSampleMex.cpp` | Native correlated sampled-feedback flow verifier |
| `../config/collisionAvoidanceControllerConfig.m` | Controller defaults, merging and validation |

## Current interfaces and scope

The public controller signature is unchanged. Its fourth output is applied-input
memory, format 25, and contains no plan or terminal controller. The online path
calls `formulateAvoidanceProblem(model)` and `solveHardCbfClf.constrained` exactly
once. Removed terminal, checker and carried-witness methods have no wrappers.

`avoidanceStageQp` and the general model/geometry kernels remain standalone
research transcription and model utilities; they are not alternate execution
paths. Nonlinear Fiala enclosure tools retain their separate model-study scope.
The current online guarantee is the declared zero-residual affine plant only.

Clear changed MATLAB functions/classes after updating a live session. Native
kernels remain under the excluded `solver/` tree and do not need regeneration
for this MATLAB orchestration change. `controllerSourceBudgetTest` enforces the
20-source upper limit.
