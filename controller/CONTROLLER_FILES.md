# Core controller source budget

The core has **24 source files**, including the controller configuration. The
public entry uses nonlinear inertial Fiala dynamics, full-interval rectangular
certificates and an invariant cruise backup. See
[the current contract and proof](NONLINEAR_PREDICTIVE_CBF.md). The remaining affine
formulation, geometry and prediction modules are shared research utilities;
they do not define the public controller's safety guarantee.

The count includes every MATLAB/C/C++ source or header under `controller/`, plus
`config/collisionAvoidanceControllerConfig.m`. Tests, scenario drivers, theory
documents, generated binaries and third-party solvers have separate roles.

| Source | Responsibility and principal interfaces |
| --- | --- |
| `collisionAvoidanceController.m` | Public nonlinear controller, immutable target epoch, format-48 joint witness and metadata |
| `nonlinearBicycleModel.m` | Nonlinear inertial/Frenet derivatives, RK4 proposal and variational flow, realizable trims and transverse errors |
| `nonlinearSafetyCertificate.m` | Constant-speed/heading-rate target parsing and exact flow, rectangle dual witnesses, swept interval admission, invariant-ball synthesis and infinite target continuation |
| `solveNonlinearPredictivePlan.m` | Stored-policy execution and terminal handoff, lane-rollout initialization, Huang safety-slack LP and Li dual SCA, secondary CLF QP and nonlinear admission |
| `nonlinearSafetyMex.cpp` | Directed target/rectangle/road geometry, Frenet norm and tail certificates, interval CLF derivatives and terminal remainder proof |
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

The public four-output signature is unchanged. Returned controls beyond the first
are nominal controls of the stored policy; the certificate carries their fixed
reference states and feedback gains. Use the fourth output as the next call's
state. The target model, global corridor and exact plant must remain consistent.
A stored suffix is executable on solver or verification failure, and the invariant
cruise law is dispatched when that prefix ends. See the main theory document for
clock, uncertainty, road and positive-speed limits.

Affine formulation functions remain callable for model studies. Their format-46
metadata, node-only certificates and affine-plant scenario drivers are not the
new public interface. Existing tests of those public-controller contracts require
migration; the nonlinear regression suite is `nonlinearPredictiveSafetyTest`.

Build the MPFR kernels with `scripts/buildFialaIntervalVerifier.m` outside tracked
source. `prepareCollisionAvoidanceController` can build missing kernels under
excluded `solver/nonlinear/`. Clear loaded MATLAB functions/native kernels after
source changes. `controllerSourceBudgetTest` enforces the 24-source limit.
