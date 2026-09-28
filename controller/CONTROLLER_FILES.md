# Core controller source budget

The core has **23 source files**, including the controller configuration. The
public path is the nominal joint-state PCBF/CLF/SCvx controller described in
[the current architecture](PCBF_CLF_ARCHITECTURE.md). The affine formulation,
uncertainty and interval kernels are independent research utilities and are
not called by the public optimizer. The active path requires no native build.

The count includes every MATLAB/C/C++ source or header under `controller/`, plus
`config/collisionAvoidanceControllerConfig.m`. Tests, scenario drivers, theory
documents, generated binaries and third-party solvers have separate roles.

| Source | Responsibility and principal interfaces |
| --- | --- |
| `collisionAvoidanceController.m` | Current joint-state initialization, nominal MPC dispatch, format-49 plan state and metadata |
| `nonlinearBicycleModel.m` | Nonlinear dynamics, RK4 prediction, analytic variational flow, cruise trims and lane CLF |
| `predictiveSafetyGeometry.m` | Constant-speed/heading-rate target flow, polygon distance duals, lane terminal set and analytic terminal separation |
| `solveNonlinearPredictivePlan.m` | Shifted-plan and lane-rollout initialization, safety LP, secondary CLF QP and SCvx trust-region updates |
| `readPlanningInputs.m` | Shared ego, road and target input normalization |
| `hardEncounterBarrier.m` | Finite encounter admission/conditioning, same-model invariant cruise certificate (shrunk by the declared lateral clearance), road-refit readmission and carried-witness data |
| `formulateAvoidanceProblem.m` | Full-plan objective and hard node/terminal rows, affine elimination of the executed prefix, verified fresh-problem inclusion, and soft CLF / hard terminal cones |
| `avoidanceStageQp.m` | Sparse base transcription (`build`) and fixed-direction support majorants (`fixedDirections`) |
| `solveHardCbfClf.m` | Previous-reference validation and flow bootstrap (`prepare`), bounded flow-trajectory model reconstruction, convex base solving (`constrained`), fixed-direction admission/continuation (`fixedDirections`), alternate-side and bounded phase-I direction recovery; independent lifted and original-coordinate hard-constraint checks before accepting solver candidates |
| `avoidanceSafetyGeometry.m` | Joint occupied-set records (relative ego-target zonotope for a target-reactive tube), support functions, exact residuals and direction storage (`setDirections`); chart construction, signed-distance initialization and offline geometry kernels |
| `laneGeometry.m` | Polyline, arc and smooth-profile projection, Frenet poses and local chart bounds (chart range not enforced) |
| `ltvBicycleModel.m` | Held-input node prediction with feedback deviation sets (`finitePredict`, `feedbackContract`), target-reactive gain design and joint ego-target deviation sets (`reactionGains`, `reactiveTube`), affine input-family swept prediction for offline audits (`fixedPredict`), sampled cruise and immutable phase scheduling (`sampledCruise`, `referenceSchedule`, `referenceAt`), nonlinear dynamics and signed road forces (`roadLoad`) |
| `modifiedFialaTire.m` | Modified Fiala forces, tangents and tire parameters |
| `stateUncertainty.m` | Estimator bounds, held-interval enclosures for offline synthesis and audits, held process reserves, intersection and sampled-feedback transition (`sampledFeedbackTransition`) |
| `targetPrediction.m` | Target admission (`admitOnline`, `admit`) under the exact-NRMM `nrmm-motion-v1` contract (the only motion contract), NRMM parameter intervals from the estimate box and the published bounds (`nrmmParameters`), reachable-box and parameter conditioning (`condition`), the reachable box over every NRMM path of the parameter intervals (`finiteFlow`), the target deviation model of the reactive tube (`deviationModel`), the nominal NRMM anchor (`nominalFlow`), offline uncertainty studies and footprint support |
| `fialaCertificate.m` | Validated nonlinear residuals (`residual`), held-feedback samples (`sample`), prescribed sequences (`sequence`) and shared constants (`parameters`) |
| `projectLanePolylineMex.cpp` | Native batched polyline projection |
| `laneFrameBoundsMex.cpp` | Native affine chart bounds |
| `solveAvoidanceSocpMex.cpp` | Native conic solver bridge |
| `fialaIntervalCore.hpp` | Directed interval arithmetic and authoritative nonlinear Fiala equations |
| `fialaIntervalMex.cpp` | Native domain-wide nonlinear residual verifier |
| `fialaFeedbackSampleMex.cpp` | Native correlated sampled-feedback flow verifier |
| `../config/collisionAvoidanceControllerConfig.m` | Controller defaults, merging and validation |

## Current interfaces and scope

The public four-output signature is unchanged. Controls and predicted states
are nominal; the fourth output retains the next warm start. The target is
reinitialized from each observation, with constant-parameter propagation during
a dropout. See the architecture for positive-speed and global-road limits.

`prepareCollisionAvoidanceController` warms the MATLAB prediction and optimizer
without building any native libraries. `buildFialaIntervalVerifier` remains an
explicit offline utility for the two independent Fiala model-study kernels;
it is not part of controller preparation or execution.

The focused current-controller suite is `nonlinearPredictiveSafetyTest`.
Older public-entry tests and scenario drivers for affine certificate formats
are not regression coverage of format 49; those interfaces require migration
before use. The source-budget test retains the existing ceiling of 24 files.
