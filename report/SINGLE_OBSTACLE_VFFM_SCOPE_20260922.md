# Single-obstacle NRMM/VFFM study scope

Date: September 22, 2026.

The current study considers one ego vehicle and at most one obstacle vehicle.
This narrows the multi-target implementation introduced in the original
NRMM/VFFM reconstruction; it does not change that historical report's results.

## Behavior

- `readPlanningInputs` rejects target structure arrays with more than one
  record, including estimates bundled on the ego input. The controller never
  silently selects or discards a target to satisfy the restriction.
- `solveHardCbfClf.vffmReference` and `prepareFluidReference` reject multiple
  encounters with `collisionAvoidanceController:unsupportedTargetCount`.
- The reference uses one moving Gaussian. The two fitted candidates pass on
  opposite sides of that same obstacle; there is no Gaussian superposition,
  multi-target conflict ranking or joint passing-side assignment.
- With no target or no nominal conflict, preparation retains the nominal seed.
  A present nonconflicting target still contributes its hard safety constraints.
- NRMM propagation, road-chart derivatives, terminal-preserving fitting and
  the final fixed-direction optimization retain their single-target formulas.
  Only the optimized, independently verified plan can authorize a command.

The estimator and general geometric support kernels are outside this scope
change. Their array interfaces and component tests are retained. Existing
node-only certification, affine-plant assumptions, unsupported physical road
boundaries in the terminal certificates and measured deadline limitations are
not resolved by restricting the number of targets.

## Validation method

MATLAB `matlab.unittest` tests cover direct and bundled multi-target rejection,
rejection before a trajectory solve, analytical single-obstacle references,
left/right crossing candidates, target-free control and certified continuation
after the single target leaves the observation region.

An initial focused run exposed an incorrect assertion in the revised release
test: after the last target is released, the controller can renew a certified
target-free plan instead of inheriting the previous finite prediction family.
The test now checks release, absence of obsolete target rows, a certified plan
and the witness's cone margins. No production continuation logic was changed
to satisfy this test.

Full regression command from the repository root:

```matlab
results = runtests('tests');
assertSuccess(results);
```

The corresponding current derivation is
[NRMM/VFFM initialization](../controller/NRMM_VFFM_INITIALIZATION.md).

## Executed results

MATLAB R2026a Update 3, version 26.1.0.3276743.

- The first focused run passed 118/119 tests; the release assertion described
  above was then corrected.
- The full repository run passed 800/801 tests in 532.864017 aggregate test
  seconds. The only failure/incomplete result was the old
  `conditioningTwoUncertainTargetsPreservesTheirCertificates` fixture,
  correctly rejected by the new input contract.
- That fixture was changed to condition one uncertain target while retaining
  identity, direction inheritance, shifted-witness and support-residual checks.
  Final focused regression passed 159/159 tests with no incomplete results in
  9.636247 aggregate test seconds. It covered
  `nrmmVffmReferenceTest`, `fluidInitializationTest`,
  `collisionAvoidanceControllerTest`, `recursiveSafetyClosureTest`,
  `jointSupportCertificateTest`, `standaloneControllerFrameTest` and
  `standaloneBenchmarkWorkflowTest`.
- A name-level check against a newly discovered current 801-test suite confirmed
  that every current test has a passing result from the full run or final
  focused run. This is combined regression coverage, not a second full run.
- Factory-configuration Code Analyzer checked all seven changed MATLAB files.
  The sole finding is the pre-existing `FNDSB` suggestion at
  `solveHardCbfClf.m:280`; no new finding was introduced. The initial analysis
  also reported a missing user-settings file and fell back to defaults, so
  the factory configuration was selected explicitly for the final check.
- `git diff --check` passed. No new native build or timing benchmark was run.
  The full MATLAB call exceeded the tool's 300-second response limit; its
  saved results were subsequently loaded and checked after completion.

Machine-readable results: [validation summary](SINGLE_OBSTACLE_VFFM_SCOPE_20260922.json).
Raw MAT results are local diagnostic artifacts under
`~/.cache/collisionAvoidance/single-obstacle-20260922/` and are not committed.
