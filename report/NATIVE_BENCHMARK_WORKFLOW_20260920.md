# Native benchmark automation and the 92.155 ms historical frame

## Timing scope

The earlier 92.155 ms maximum is a complete **MATLAB plus checked-MEX hybrid**
frame. It is not a full standalone C controller measurement. The previous
standalone executable maximum, 28.015 ms, covers a prepared numerical problem
only. These maxima belong to different frames and different timed boundaries;
they must not be substituted for one another.

Future performance experiments use `scripts/runNativeControllerBenchmark.py`.
It recaptures current scenario programs, regenerates C, links a new executable,
measures that executable, and independently validates its decisions. It always
uses a new external directory. Hash checks reject changed source, solver
dependencies, generated artifacts, configuration, and fixtures. Reports retain
the exact worktree hashes as well as the base Git commit and toolchain.

This task changes benchmark orchestration and reporting, not the controller's
optimization, mathematical certificate, or physical model. Complete native
upstream formulation and carried-state handling remain unimplemented.

## Exact historical frame breakdown

The source is the preserved
`~/.cache/collisionAvoidance/standalone-c-20260919/hybrid/measured/stationary-0/stationary-exact-state.mat`.
It is straight stationary-target frame 1, with 96 prediction stages, 24
overlapping nominal nodes, one successful scalar-section admission attempt,
and zero conic solver calls. No restored or approximate historical timings
are used below.

| Nonoverlapping measured or residual bucket | Milliseconds | Interpretation |
| --- | ---: | --- |
| Controller input preparation | 7.628000 | Configuration, parsing, model construction and encounter preparation |
| Upstream formulation and adapter overhead, not separately timed | 67.623909 | Prediction, geometry, terminal/CLF program construction, numerical packing, checked-MEX conversion and untimed native-call overhead |
| Native scalar-section admission | 5.336133 | Restricted control section and feasible-interval search |
| Native independent verification | 0.687958 | Initial/candidate checks inside the generated numerical entry |
| Native conic solver | 0.000000 | No SOCP solve was called on this admission frame |
| Remaining full-frame work, not separately timed | 10.879000 | Driver measurement construction, extra MATLAB certification, output/diagnostic construction and stored-witness handling |
| **Full hybrid frame** | **92.155000** | Driver wall timer around measurement construction and controller call |

The saved controller timer reports `formulationSeconds = 73.648 ms`. This is
an inclusive aggregate, not an additional row: subtracting the two native
subtimers gives `73.648 - 5.336133 - 0.687958 = 67.623909 ms`.
The remaining full-frame bucket is
`92.155 - 7.628 - 73.648 - 0 = 10.879 ms`.

The formulation aggregate includes the upstream `formulateAvoidanceProblem`
timer plus the checked-MEX adapter's elapsed time excluding the solver. The
adapter's additional MATLAB certification occurs after its timer stops; the
controller also certifies, generates outputs and stores the carried witness.
Those operations are included in the remaining full-frame bucket. They have
not each been assigned invented durations.

Thus about 73.4% of this historical frame lies in the unresolved upstream and
adapter bucket. Its 92.155 ms maximum cannot be explained by a slow SOCP solve:
that frame performs no conic solve. The existing trace cannot distinguish
prediction, terminal/geometry construction and MEX marshalling individually.
A later instrumented replay could measure them, but would be a new observation,
not a further exact decomposition of this original timing.

For comparison, the **previous** slowest standalone C call is straight
stationary frame 2, with 95 stages: 15.519079 ms assembly/solver preparation,
12.018040 ms solver, 0.474160 ms verification, and approximately 0.003790 ms
remaining overhead, totaling 28.015069 ms. It is an inherited continuation,
not the first-frame admission described above.

## Reproduction

```bash
python3 scripts/runNativeControllerBenchmark.py
```

See [`../scripts/README.md`](../scripts/README.md) for options, generated
artifacts, current native scope, and the distinction between correctness tests
and executable performance measurements.

## Fresh automatic-run validation

Executed the new entry with its six default scenarios and 600 requested holds,
using `/home/zai/.cache/collisionAvoidance/native-auto-20260920` as a fresh
external output directory. C generation, native compilation/linking and the
complete automatic workflow succeeded. All source, dependency and sealed
artifact hashes still match after execution. The worktree is based on commit
`d65d04bd9d1380311728d5be61961a117d5b61bb`; the new driver itself is included in
the manifest's source hashes. The complete native pipeline remains out of scope.

The newly compiled executable replayed 278 active-target frames, two warmups and
five measured calls each, giving 1,390 timing observations. Preparation,
serialization and MATLAB validation are excluded from these native timings.

| Native call group | Calls | Median ms | Maximum ms |
| --- | ---: | ---: | ---: |
| All | 1,390 | 8.490 | 27.547 |
| Inherited continuation | 1,360 | 8.864 | 27.547 |
| Fresh admission, including the rejected circular crossing | 30 | 3.993 | 4.767 |

No measured native call exceeds 50 ms. This is an observed desktop maximum,
not a WCET bound or complete-frame deadline guarantee. The slowest new call is
straight stationary frame 3, with 94 remaining stages: 15.271254 ms assembly and
solver preparation, 11.811278 ms solver, 0.460818 ms verification, and 0.003780 ms
remaining overhead. It is a new measurement and does not revise the historical
92.155 ms hybrid frame or the earlier 28.015 ms native observation.

Independent MATLAB validation of the first output for every prepared frame
certifies all 277 accepted plans and agrees with the one rejection. Maximum
decision difference is `8.9378e-8`; maximum wrapped direction difference is
`4.2590e-5 rad`. Every repeated native call also uses its built-in hard verifier.
Scenario capture reproduces the earlier outcomes: five complete 600-hold runs;
circular crossing fails admission; straight stationary/oncoming retain the
previous diagnostic inter-node overlaps. Faster execution is not reported as
an improvement in those existing safety/capability limitations.

Nine focused MATLAB tests pass: five new workflow tests reject changed/added
source, replaced executable and duplicate observations, and verify that the
summary ignores MATLAB timing; four existing adapter tests check hard
certification, invalid inputs and target identity. Python syntax checks and
`git diff --check` pass. `ldd` confirms ordinary system-library dependencies
without MATLAB Runtime. The preceding 726-test full-suite result belongs to
the earlier implementation task and was not repeated for this tooling-only
change.

Machine-readable results, the extracted historical breakdown, test outcomes
and technical-artifact hashes are in
[`NATIVE_BENCHMARK_WORKFLOW_20260920.json`](NATIVE_BENCHMARK_WORKFLOW_20260920.json).
