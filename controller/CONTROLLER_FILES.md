# Core controller source budget

The core control algorithm has an upper limit of **20 source files**. The
current implementation contains **20**: 13 MATLAB modules, five native C++
translation units, one native header, and one controller configuration.
The controller initializes separation directions and then fixes them while
optimizing the complete trajectory in one hard SOCP. The
accepted complete certificate supplies the next frame's feasible incumbent.
Safety is certified at the hold nodes only
([NODE_SAMPLED_CERTIFICATE.md](NODE_SAMPLED_CERTIFICATE.md)).
Its format-44 state retains directions, occupied sets, prediction, terminal
continuation, the feedback correction of the issued input and, for a
target-reactive admission, the reaction gains and nominal target flow
([CLOSED_LOOP_TARGET_PREDICTION.md](CLOSED_LOOP_TARGET_PREDICTION.md)). See [the algorithm and guarantees](JOINT_SUPPORT_CERTIFICATES.md).
Related operations stay in the module that owns their responsibility.

The count includes every source file under `controller/`, including future
subdirectories, plus `config/collisionAvoidanceControllerConfig.m`. Tests,
scenario drivers, estimator/perception algorithms, theory documents, generated
binaries and third-party solver implementations have separate responsibilities.
Do not move controller helpers into those directories to evade the limit.

| Source | Responsibility and principal interfaces |
| --- | --- |
| `collisionAvoidanceController.m` | Public target-array entry; fixed-direction admission and solver-accepted improvement, complete witness storage and full-frame deadline enforcement |
| `readPlanningInputs.m` | Input normalization, target-departure sensor declaration, optional NRMM parameter error bounds and lane/target model construction |
| `hardEncounterBarrier.m` | Finite encounter admission/conditioning, same-model invariant cruise certificate and carried-witness data |
| `formulateAvoidanceProblem.m` | Full-plan objective and hard node/terminal rows, affine elimination of the executed prefix, verified fresh-problem inclusion, and soft CLF / hard terminal cones |
| `avoidanceStageQp.m` | Sparse base transcription (`build`) and fixed-direction support majorants (`fixedDirections`) |
| `solveHardCbfClf.m` | Convex base solving (`constrained`) and fixed-direction admission/continuation (`fixedDirections`), including the ordered ego-only / target-reactive admission attempts; solver success is accepted with no post-solve verification |
| `avoidanceSafetyGeometry.m` | Joint occupied-set records (relative ego-target zonotope for a target-reactive tube), support functions, exact residuals and direction storage (`setDirections`); chart construction, signed-distance initialization and offline geometry kernels |
| `laneGeometry.m` | Polyline, arc and smooth-profile projection, Frenet poses and local chart bounds (chart range not enforced) |
| `ltvBicycleModel.m` | Held-input node prediction with feedback deviation sets (`finitePredict`, `feedbackContract`), target-reactive gain design and joint ego-target deviation sets (`reactionGains`, `reactiveTube`), affine input-family swept prediction for offline audits (`fixedPredict`), sampled cruise and immutable phase scheduling (`sampledCruise`, `referenceSchedule`, `referenceAt`), nonlinear dynamics and signed road forces (`roadLoad`) |
| `modifiedFialaTire.m` | Modified Fiala forces, tangents and tire parameters |
| `stateUncertainty.m` | Estimator bounds, held-interval enclosures for offline synthesis and audits, held process reserves, intersection and sampled-feedback transition (`sampledFeedbackTransition`) |
| `targetPrediction.m` | Bounded target admission (`admitOnline`) under a Cartesian-jerk (`finite-sensing-motion-v1`) or exact-NRMM (`nrmm-motion-v1`) contract, NRMM parameter intervals from the estimate box and the published bounds (`nrmmParameters`), reachable-box and parameter conditioning (`condition`), absolute-time flow with a declared acceleration maximum or over every NRMM path of the parameter intervals (`finiteFlow`, `accelerationDeviationBound`), the target deviation model of the reactive tube (`deviationModel`), offline uncertainty studies and footprint support |
| `fialaCertificate.m` | Validated nonlinear residuals (`residual`), held-feedback samples (`sample`), prescribed sequences (`sequence`) and shared constants (`parameters`) |
| `projectLanePolylineMex.cpp` | Native batched polyline projection |
| `laneFrameBoundsMex.cpp` | Native affine chart bounds |
| `solveAvoidanceSocpMex.cpp` | Native conic solver bridge |
| `fialaIntervalCore.hpp` | Directed interval arithmetic and authoritative nonlinear Fiala equations |
| `fialaIntervalMex.cpp` | Native domain-wide nonlinear residual verifier |
| `fialaFeedbackSampleMex.cpp` | Native correlated sampled-feedback flow verifier |
| `../config/collisionAvoidanceControllerConfig.m` | Controller defaults, merging and validation |

## Current interfaces and scope

The public controller signature is unchanged. Its fourth output is a
format-44 predictive certificate. `formulateAvoidanceProblem(model)` retains
the convex dynamics/terminal base and attaches collision and exit certificates.
Accepted successors preserve their angles, occupied sets, charts, prediction,
terminal set and absolute exit deadline. Touching majorants contain that full
shifted witness under unchanged contracts. The terminal law is never dispatched
after solver failure.

`avoidanceStageQp.build` supplies the common sparse base. Its `fixedDirections`
method adds stage-local support epigraphs and a global yaw majorant without
angle variables. `solveHardCbfClf.fixedDirections` owns analytical NRMM/VFFM reference preparation and fitting,
mandatory full-plan admission, and full continuation improvement. Target-free
frames solve only the common convex base with a single `constrained` call.
Nonlinear Fiala tools retain their separate study scope. The online guarantee
remains the declared zero-residual held affine plant.

Clear changed MATLAB functions/classes after updating a live session. Native
kernels remain under the excluded `solver/` tree and must be regenerated with `scripts/buildAvoidanceGeometryKernel.m`
after changing the shared geometry kernel; the current kernels include
the numeric affine pose map and hard local-domain payload. `controllerSourceBudgetTest` enforces the
20-source upper limit.
