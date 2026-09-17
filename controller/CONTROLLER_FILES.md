# Core controller source budget

The core control algorithm has an upper limit of **20 source files**. The
current implementation contains **20**: 13 MATLAB modules, five native C++
translation units, one native header, and one controller configuration.
Fresh admission uses analytic support families and search-only restoration, followed by a hard
trajectory SOCP with a full input sequence and penalized first-hold CLF slack. Since
2026-09-17 the safety rows are certified at the hold nodes only
([NODE_SAMPLED_CERTIFICATE.md](NODE_SAMPLED_CERTIFICATE.md)).
Its format-34 state retains the complete prediction and terminal continuation.
See [the algorithm and guarantees](SINGLE_SOLVE_CBF_CLF.md).
Related operations stay in the module that owns their responsibility.

The count includes every source file under `controller/`, including future
subdirectories, plus `config/collisionAvoidanceControllerConfig.m`. Tests,
scenario drivers, estimator/perception algorithms, theory documents, generated
binaries and third-party solver implementations have separate responsibilities.
Do not move controller helpers into those directories to evade the limit.

| Source | Responsibility and principal interfaces |
| --- | --- |
| `collisionAvoidanceController.m` | Public target-array entry; support-family admission and restoration, one inherited-family solve per successor, complete witness storage, no command after failed search |
| `readPlanningInputs.m` | Input normalization, target-departure sensor declaration and lane/target model construction |
| `hardEncounterBarrier.m` | Finite encounter admission/conditioning, same-model invariant cruise certificate and carried-witness data |
| `formulateAvoidanceProblem.m` | Full-plan objective and hard node/terminal rows, affine elimination of the executed prefix, verified fresh-problem inclusion, and soft CLF / hard terminal cones |
| `avoidanceStageQp.m` | Equivalent sparse stage transcription and exact duplicate-row reduction (`build`) |
| `solveHardCbfClf.m` | Hard and search-only conic solving (`constrained`, `restore`), constraint generation and independent verification (`certify`) |
| `avoidanceSafetyGeometry.m` | Node separation rows (`build`; multi-point cells keep the whole-hold Bernstein path for offline audits), analytic support proposals (`supportDirection`, `supportNormals`), rectangle distance and shared numeric kernels |
| `laneGeometry.m` | Polyline, arc and smooth-profile projection, Frenet poses and certified local chart bounds |
| `ltvBicycleModel.m` | Held-input node prediction (`finitePredict`), affine input-family swept prediction for offline audits (`fixedPredict`), sampled cruise and immutable phase scheduling (`sampledCruise`, `referenceSchedule`, `referenceAt`), nonlinear dynamics and signed road forces (`roadLoad`) |
| `modifiedFialaTire.m` | Modified Fiala forces, tangents and tire parameters |
| `stateUncertainty.m` | Estimator bounds, held-interval enclosures for offline synthesis and audits, held process reserves, intersection and sampled-feedback transition (`sampledFeedbackTransition`) |
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

The public controller signature is unchanged. Its fourth output is a
format-34 predictive certificate. The online path calls
`formulateAvoidanceProblem(model)` and `solveHardCbfClf.constrained`. Fresh
admission uses bounded support sectors, node scoring, grouped-deficit
restoration and final hard verification. Accepted successors retain the
certified prediction and absolute exit deadline. New geometry must contain
the complete inherited witness. The terminal law proves nonemptiness; it is
never dispatched after solver failure. The conditional proof is in
TERMINAL_CBF_PROOF.md.

`avoidanceStageQp.build` is now the sole online sparse realization, including
restoration. Its obsolete slack/budget transcription has been removed.
Nonlinear Fiala tools retain their separate study scope. The online guarantee
remains the declared zero-residual held affine plant.

Clear changed MATLAB functions/classes after updating a live session. Native
kernels remain under the excluded `solver/` tree and must be regenerated with `scripts/buildAvoidanceGeometryKernel.m`
after changing the shared geometry kernel; the current kernels include
the numeric affine pose map and hard local-domain payload. `controllerSourceBudgetTest` enforces the
20-source upper limit.
