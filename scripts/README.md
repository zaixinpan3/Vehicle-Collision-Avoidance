# Research experiment entry points

## Native controller performance after source changes

Use this entry for controller performance measurements:

```bash
python3 scripts/runNativeControllerBenchmark.py
```

Every invocation captures the current straight and circular scenarios, generates
fresh C with MATLAB Coder, compiles and links a new executable, runs that
executable, and independently verifies its returned decisions with MATLAB. It
does not reuse an earlier build or previously captured scenario programs.
MATLAB, MATLAB Coder, GCC, and the existing built Clarabel dependency are required.

The default experiment uses 600 holds per scenario, curvatures 0 and 0.01/m,
stationary/oncoming/crossing targets, and two warmups followed by five measured
native calls per captured active-target frame. The existing scenario driver
defines the remaining settings: seed 20260912, 50 ms holds, 1.6 s nominal horizon,
8 m/s cruise, exact sensing and declared affine plant. Failed admission frames
are retained. Correctness outcomes are recorded separately from timing.

Optional arguments include `--output /absolute/new/external/directory`,
`--sample-count`, `--curvatures`, `--scenarios`, `--warmups`, and `--repetitions`.
An output directory must not already exist and must be outside the repository.
By default, a unique directory is created in `~/.cache/collisionAvoidance/`.

The output contains:

- `manifest.json`: base Git commit, exact worktree source hashes, solver-library
  hashes, generated C/library/executable/configuration/fixture hashes, commands,
  toolchain, settings, and final validation status. The worktree hashes identify
  uncommitted source changes; a commit name alone is not a clean-tree claim.
- `summary.json`: **only independent executable timings** and verification
  outcomes, including a breakdown of the slowest native call.
- `native-replay.jsonl`: individual native timings, statuses, and the first
  returned decision per prepared frame.
- `build.log`, `validation.log`, and the original capture/verification artifacts.

The driver checks source/dependency hashes after building and after validation,
and checks compiled artifacts and fixtures before/after replay and validation.
Changed source or artifacts, incomplete replay output, build failure, or failed
independent certification prevent a successful summary. Generated artifacts and
raw traces remain outside Git; compact experiment reports belong in `report/`.

**Current native scope is the prepared active-target numerical kernel.** Input
parsing, prediction and upstream formulation, reference/terminal synthesis,
target-free/terminal-optimization modes, and carried-state handling are not a
standalone C pipeline yet. Fixture loading is excluded from the measured call.
The summary therefore explicitly sets `fullPipelineMeasured=false`. This
benchmark must not be reported as complete-frame or Raspberry Pi performance.

Capture timings, the one MATLAB reference call used for correctness, old MATLAB
runtime replays, and hybrid checked-MEX measurements are diagnostic only. They
do not substitute for executable performance. Ordinary unit tests can still run
without a C rebuild; the performance entry always regenerates and recompiles.

The historical 92.155 ms complete hybrid frame is explained in
[`../report/NATIVE_BENCHMARK_WORKFLOW_20260920.md`](../report/NATIVE_BENCHMARK_WORKFLOW_20260920.md).
