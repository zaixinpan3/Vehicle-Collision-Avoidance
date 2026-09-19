# Remove finite-branch initialization and retain distance-dual control only

Date: September 16, 2026. Controller certificate format: 31.
Baseline: `8a9344d3c796eb98d02eb2b747c90014638e661c`.

## Implementation and explicit limitation

The requested cleanup removes the hybrid solver. The only online collision
convexification is now the distance-dual method documented in
[current convexification documentation](https://github.com/zaixinpan3/Vehicle-Collision-Avoidance/blob/8529584774ffd17b2cae7d2fb18aac219b2fde13/controller/SUPPORT_CONVEXIFICATION.md).

Removed executable components:

- Finite direction-grid construction, lazy integer assignment search,
  big-M rows, no-good cuts, terminal-cone outer cuts and progress ranking from
  `solveHardCbfClf`. The public controller calls `constrained` directly.
- The offline `optimizeNormals` maximum-margin method and its
  `evaluateContinuousSeparation` driver.
- `profileFiniteBranchRuntime`, `benchmarkFiniteBranchRowReduction` and the
  duplicate `evaluateDualDistanceConvexification` comparison driver, which
  depended on successful admission by the removed initializer.
- `executionPolicy` and `separationDirectionCount`, including their aliases,
  scenario options and integer/branch telemetry. Unknown configuration fields
  are rejected by the existing strict configuration merge.
- Branch-search-specific tests and the retired finite-branch design document.
  Dated experiment reports remain evidence of their original revisions, not
  current executable implementations or current success claims.

Fresh admission computes distance-dual directions before constructing collision
rows. An overlapping or touching midpoint of the initial cruise-input anchor
has ordinary distance zero and supplies no usable separating direction.
Initialization then fails before any hard trajectory solve, with an explicit
message and no command. When directions are available but the hard trajectory
SOCP is infeasible, existing horizon retries use only the same method.
No alternate initializer or collision relaxation has been introduced.

This is an intentional loss of the old initializer's admission capability.
It does not establish that an encounter is physically unavoidable. The previous
stationary/oncoming successes cannot be carried forward as validation of this
revision. Resolving their initialization within the selected method remains
future controller work.

The predictive continuation, finite confirmed exit, invariant terminal set,
whole-hold collision checking, actuator bounds, independently verified hard
safety, soft CLF and its squared penalty remain. The input objective and target
high-gain observer are unchanged. New collision directions must still contain
the known feasible suffix before replacing inherited rows. Format 31 rejects
older saved states; a format-30 witness cannot import an old initialization.
The [recursive-feasibility proof](../controller/TERMINAL_CBF_PROOF.md) still
starts from successful admission and retains its model/sensing assumptions.

## Verification method

The removal audit scans all 80 MATLAB files under `controller/`, `config/`
and `scripts/`. It finds no `buildBranches`, `solveHardCbfClf.branches`,
`branchFamily`, `intlinprog`, `optimizeNormals`, `admissionGeometry` or
`completionDirections`. Both removed configuration fields are absent from
controller defaults. Six tracked files are deleted; no archived source copy
is introduced elsewhere in the repository.

Behavior tests cover clear-anchor distance accuracy, one trajectory solve,
complete hard-row verification, suffix inclusion and fixed exit deadlines,
22 successive whole-hold clocks, failure before any trajectory call for
stationary/oncoming overlap, hard trajectory infeasibility, recorded simulation
failure, rejected obsolete settings and rejected old stored certificates.
The former stationary-success test is replaced by explicit failure coverage;
a passing test suite must not be interpreted as successful stationary avoidance.
The soft-CLF/hard-collision test uses a separately feasible clear-target tracking
fixture, without claiming obstacle-forced departure from cruise.

The first targeted run exposed an incorrect expected exception identifier and
an infeasible replacement test fixture. Both are recorded in its log. The
version assertion now expects `invalidControllerState`; the tight crossing
fixture has its own explicit hard-infeasibility test. An injected-solver-failure
test uses an unlimited diagnostic budget so a cold-start timeout cannot satisfy
it before the intended injected failure.

The MATLAB MCP call returned no verifiable output. Validation therefore uses
independent `matlab -batch` processes with saved result objects and logs.
Original test/campaign artifacts are under
`/home/zai/.cache/collisionAvoidance/remove-legacy-20260916/`.

## Straight-scene protocol

Driver: `scripts/runExactStateRecursiveFeasibilityScenario.m`, 300 requested
holds per scene, `h = 0.1 s`, cruise speed 8 m/s, confirmation range 16 m,
seed 20260912, no road boundaries, exact held-affine plant, zero ego/target
error boxes and zero target jerk/yaw acceleration. Steering is bounded by
40 degrees and normalized longitudinal input by [-1, 1]; no finite slew
limit is imposed. The observer is not part of this campaign.

Stationary target: initial [15, 0] m. Oncoming target: initial [60, 0] m,
velocity [-8, 0] m/s. Crossing target: initial [15, -4] m, velocity [0, 32] m/s.
A target-free cruise run checks the permanent continuation independently.

Each scene is run with a 30 s diagnostic work budget and then a 100 ms work
budget. A separate one-hold target-free warmup precedes the measured campaign.
Long-budget simulation does not inject computation delay into plant motion.
The frame metric includes input assembly and controller work and excludes
plant integration and offline geometry audits. Neither a safe partial prefix
nor rapid failure counts as successful real-time avoidance.

## Executed results

MATLAB R2026a Update 3. Separate cold warmup: 1698.785 ms.

| Work budget | Scene | Executed holds | Outcome | Median / maximum frame (ms) | Frames above 100 ms |
| --- | --- | ---: | --- | ---: | ---: |
| 30 s diagnostic | Stationary | 0 / 300 | Initialization fails at 0 s | 107.595 / 107.595 | 1 |
| 30 s diagnostic | Oncoming | 26 / 300 | Initialization fails at 2.6 s | 10.134 / 71.471 | 0 |
| 30 s diagnostic | Crossing | 300 / 300 | Completes and recovers cruise | 9.816 / 101.723 | 1 |
| 30 s diagnostic | Cruise | 300 / 300 | Completes cruise | 9.480 / 18.404 | 0 |
| 100 ms | Stationary | 0 / 300 | Initialization fails at 0 s | 41.277 / 41.277 | 0 |
| 100 ms | Oncoming | 26 / 300 | Initialization fails at 2.6 s | 10.069 / 26.198 | 0 |
| 100 ms | Crossing | 300 / 300 | Completes and recovers cruise | 9.674 / 45.391 | 0 |
| 100 ms | Cruise | 219 / 300 | Work deadline expires at 21.9 s | 10.219 / 122.448 | 1 |

The stationary anchor contains 12 overlapping/touching midpoints; the oncoming
anchor contains six. Both have minimum signed rectangle distance -1.9 m.
These are **predicted anchor overlaps**, not collisions of an executed plan.
No hard trajectory solve is attempted for those admissions. The 26 accepted
oncoming holds precede first encounter admission; their positive physical gap
is not evidence that the encounter was completed.

Both completed crossing runs have minimum sampled body gap 9.519711 m
(required clearance 0.25 m), maximum lateral error 0.000027561 m, one confirmed
release, and final tracking-error norm approximately 3.24e-7. Each uses 300
trajectory solves and 91 distance solves over accepted frames. Completed cruise
has final tracking-error norm approximately 3.24e-7. The strict cruise failure
message reports that the work deadline expired **before the native trajectory
solve**; the run stops with no command for that attempted frame. This campaign
does not isolate the source of that preparation/formulation time excursion.
No claim of every-frame 100 ms operation is made.

An independent Python audit checks all 1171 accepted holds across the eight
runs: physical input bounds, stored hard-certificate flags, terminal cone
margins, inherited suffix margins, slack-dependent CLF residuals, no terminal
commands and solver-call accounting. Counts describe accepted holds; the
controller does not return solver telemetry for failed frames. In particular,
a zero saved distance-call count for a failed first admission does not mean
no distance queries were evaluated during the failed attempt.

The failure tests pass because the expected failure is correctly identified and
terminates execution. They do not restore the removed stationary/oncoming
avoidance capability. Cleanup is complete; general encounter initialization
and every-frame real-time completion are unresolved.

## Checks and reproducibility

- Full final repository suite: **615 / 615 passed**, zero failed or incomplete.
  This is one complete `runtests('tests')` invocation after final source edits.
- Factory Code Analyzer: **12 changed MATLAB files, zero findings**.
- Native geometry/flow parity and the **20-source controller budget** pass in
  the full suite. No observer algorithm or native solver dependency was changed.
- `git diff --check` and the 80-file source-removal scan pass.
- Original raw artifacts in the cache directory: `full-tests.mat`,
  `full-tests.log`, `final-validation.json`, `source-removal-audit.json`,
  `campaign-summary.json`, `independent-audit.json`, and per-scene JSON/MAT files
  under `diagnostic/` and `strict/`. The initial targeted failure log is retained.

Repository test command:

```bash
matlab -batch "results = runtests('tests'); assertSuccess(results)"
```

The executed harness saves those result objects before `assertSuccess` and
runs `validateRemoval.m` and `runRemovalCampaign.m` from the cache directory.
The scenario itself is reproduced from the repository root, for example:

```matlab
addpath('scripts');
runExactStateRecursiveFeasibilityScenario(Scenario="crossing", ...
    SampleCount=300, DeadlineSeconds=0.1, OutputDirectory="/tmp/dual-only-crossing");
```

Use `Scenario="stationary"` or `"oncoming"` to reproduce the initialization
failures. Results are saved before the exception is rethrown. Do not pass the
removed `ExecutionPolicy` option or reuse a format-30 controller state.
