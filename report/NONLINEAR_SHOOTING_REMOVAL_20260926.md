# Remove the nonlinear shooting controller

Decision and source removal: September 26, 2026, at 23:59 CDT. Validation and documentation completed September 27, 2026 (America/Chicago).

The user rejected the experimental nonlinear shooting method because its observed computation time is incompatible with the 50 ms control period and explicitly requested complete removal. The controller now has one optimization implementation. There is no `solver.method` selector and no `cfg.nonlinear` configuration. Both former method-name overrides and nonlinear-only settings are rejected as unknown configuration fields.

The [runtime diagnosis](NONLINEAR_RUNTIME_DIAGNOSIS_20260926.md) records second-scale solve times even after removing road constraints, and an interrupted circular-road experiment. Those measurements motivated this removal. This operation does not establish that nonlinear optimization in general can never run in real time, or that the retained controller now satisfies every physical avoidance or timing requirement.

## Removed implementation and experiment tooling

- `controller/nonlinearAvoidanceController.m`
- `controller/nonlinearAvoidanceRollout.m`
- `controller/nonlinearAvoidanceSeed.m`
- `controller/solveNonlinearAvoidancePlan.m`
- `tests/nonlinearAvoidanceControllerTest.m`
- `tests/nonlinearAvoidancePlanTest.m`
- `tests/nonlinearAvoidanceSeedTest.m`
- `scripts/runNonlinearRecoveryStudy.m`
- `scripts/auditNonlinearRecoveryStudy.py`
- `scripts/auditNonlinearVehicleRecovery.py`
- `scripts/exportNonlinearVehicleRecovery.m`
- `scripts/profileNonlinearControllerRuntime.m`
- `scripts/compareRoadConstraintRuntime.m`
- `scripts/runRoadConstraintAblation.m`
- `scripts/auditRoadConstraintAblation.py`

The four implementation files, three dedicated test files and four original study/export/audit scripts were untracked workspace additions. The four runtime/road-ablation scripts were tracked and their deletions are recorded in this commit. The manifest preserves all 15 removed paths and their pre-removal source hashes, including the untracked files whose deletions cannot appear in a Git diff.

`collisionAvoidanceController.m` and `collisionAvoidanceControllerConfig.m` are restored byte-for-byte to the parent commit's single-implementation versions. Their only pending edits were the nonlinear method branch, configuration, validation, comments and conditional reference-bank construction. These restorations remove local modifications and therefore do not produce additional committed changes in those two files. No compatibility alias, fallback to shooting, or alternate shooting controller copy remains in the executable controller/configuration sources.

Shared nonlinear Fiala/RK4 vehicle dynamics, trajectory linearization and independent plant diagnostics remain because the retained controller also uses them. General scenario configuration overrides and road-boundary inputs remain available. Unrelated observer, target-prediction and scenario changes, dependencies under `solver/`, generated binaries and other pending research work are excluded from this commit.

Historical reports and numeric experimental results are preserved; the four relevant reports carry an explicit removal notice. Their original commands describe the recorded experiment and are no longer current executable instructions. Original source snapshots and raw experiment evidence remain outside the repository.

## Verification

- 132 tests pass across controller configuration, controller execution, certificate continuation, trajectory linearization and immediate actuation.
- 20 additional tests pass across scheduled-curvature references and LTV bicycle prediction. Total: **152 passed, 0 failed, 0 incomplete**.
- Three new parameter cases verify rejection of the former `nonlinearShooting` and `affineSocp` method selectors and the removed nonlinear configuration section.
- MATLAB function resolution confirms the removed controller and solver are unavailable after clearing loaded functions and refreshing the path. Default configuration contains neither removed field.
- Source search finds no removed method/solver references in executable `controller/`, `config/` or `scripts/` MATLAB/Python sources; only negative configuration tests retain the old names.
- Code Analyzer reports no source-line findings for the restored controller/configuration and edited test. It emits a settings-file warning for each invocation and uses default settings; the raw warnings are retained.
- No full repository test suite, new closed-loop avoidance campaign or real-time performance improvement is claimed.

Compact results: [removal manifest](NONLINEAR_SHOOTING_REMOVAL_20260926/removal-manifest.json), [test results](NONLINEAR_SHOOTING_REMOVAL_20260926/test-results.csv), and [Code Analyzer output](NONLINEAR_SHOOTING_REMOVAL_20260926/code-analysis.json).
