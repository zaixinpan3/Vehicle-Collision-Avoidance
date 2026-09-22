# End-to-end single-target pipeline reconstruction

Date: September 22, 2026.

The observer, sampled error bounds, estimator/controller adapter, initial
trajectory generator, controller and geometry now carry one target. This
supersedes the earlier input-count restriction: target collections and their
algorithms have been removed from the implementation itself.

## Implemented changes

- The observer integrates a fixed 15-vector: ego velocity, ego position, GNSS
  output predictor, six-state target chain, radar output predictor and yaw.
  Target-count configuration, target-state matrices, track slots, radar row
  association/permutation and batch publication are removed. Reset accepts the
  six-state target directly.
- Sampled containment carries one three-component target error bound, one
  Lyapunov radius, one radar predictor radius and one sensor-history record.
  Radar dropout, sensor inconsistency, reacquisition and separate ego validity
  retain their existing meanings.
- The adapter and scenario tools use `targetEstimate` and `targetState`.
  The controller stores one optional `encounter`, one exit direction and one
  release flag. Identity comparison concerns successive observations of the
  same encounter; it no longer searches an active-target collection.
- Native cell geometry contains one scalar target and a `hasTarget` flag.
  The remaining record axis is prediction time. Target-key indexing in
  completion certificates and prepared native exports is removed.
- VFFM uses one moving Gaussian, one longitudinal width and two opposite
  passing candidates. There is no obstacle superposition, joint side assignment
  or dominant-target selection. The complete constrained solve and independent
  verifier still decide whether the initial trajectory can produce a command.
- Multi-target fixtures, row-permutation checks and count-rejection tests have
  been removed. Single-target tests retain absent/present geometry, identifier
  continuity, dropout, reacquisition, bounded motion and recursive control.

The derivation and new interface are in
[`SINGLE_TARGET_PIPELINE.md`](../controller/SINGLE_TARGET_PIPELINE.md).
Current theory, manuscript wording, runtime documentation and estimator API
examples have been updated. Earlier dated validation reports retain their
historical meaning.

## Validation evidence

A baseline captured before this refactor used
`encounterTestFixture.circularCrossing` at curvatures -0.01, 0 and 0.01 1/m,
with both solver limits set to 60 s. The new implementation reproduced every
complete decision vector and input sequence exactly: maximum absolute
comparison error was zero in all three cases. This verifies those fixtures,
not general equivalence for every possible input.

`buildAvoidanceGeometryKernel` and `buildNrmmObserverKernel` regenerated and
compiled the geometry/projection/support and fixed-state observer/history
kernels successfully. Generated sources and binaries remain outside version
control.

An intermediate focused run completed 223 checks: 216 passed and seven failed.
One failure used the obsolete `normals` fixture field, and six scenario failures
came from a renamed scalar estimate shadowing a synthetic target-motion
function. Both causes were corrected before final regression. An initial test
load also exposed a missing MATLAB line continuation after removal of a target
count; it was corrected. A prepared-kernel build needed its output directory
created before code generation. None of those failed attempts is counted as a
successful final validation. The full run subsequently identified one test-only
name collision between a raw target input and its parsed result; renaming the
parsed variable restored all 12 checks in `controllerEstimatorBoundsTest`.

The full `runtests('tests')` execution exercised 789 tests: 786 passed and
three failed solely in fixtures being migrated to the scalar interface. Besides
the parsed-variable name collision above, two ego-equation tests supplied
zero-column target states to the formerly variable-size core. They now use the
fixed six-state target with radar correction disabled, preserving their original
ego-equation assertions. All 28 tests in the three affected classes passed on
rerun. No production behavior changed between the full run and these reruns.
Across the full run and corrected-class reruns, all 789 distinct tests have
passing results with no outstanding failed or incomplete checks.

Factory Code Analyzer checked all changed MATLAB files. Its only findings were
six existing performance suggestions in sparse indexing and `find` usage;
there were no syntax errors. `git diff --check` passed. The source scan found
no remaining target-count options, plural target output fields, target-slot
logic, target-array association or indexed completion keys in the MATLAB chain.

Reproduction uses the repository-root command
`matlab -batch "results = runtests('tests'); assertSuccess(results)"`.
The final targeted rerun used `runtests` on
`controllerEstimatorBoundsTest.m`, `nrmmDirectVelocityTest.m` and
`nrmmObserverVectorFieldTest.m`. Baseline comparison used the three fixture
curvatures listed above; generated-kernel checks used
`standaloneControllerBenchmark.buildMex` and the same prepared circular
admission frame for interpreted/native evaluation. Local run logs and MAT
results are under
`/home/zai/.cache/collisionAvoidance/scalar-pipeline-20260922/`; they are generated
validation artifacts, not additional tracked research source or admitted
Evidence Index items.

## Scope and migration

Saved controller states use format 42. Reinitialize a saved format-41
controller and rebuild generated native adapters when adopting the scalar
schema. Plural aliases and multi-target compatibility wrappers are absent.

The mathematical guarantee remains conditional and node-based for the
existing declared sampled affine plant. Refactoring does not certify
inter-node collision avoidance, nonlinear-vehicle behavior or a runtime
worst-case bound. A missed observation remains distinct from confirmed target
release. Road boundaries and time samples remain collections because they
represent geometry and time, respectively.

Solver dependencies, generated binaries, reference PDFs, unrelated untracked
files and external archive documents are excluded from the project commit.
