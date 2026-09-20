# Generated C numerical controller benchmark — September 20, 2026

This experiment generates C from the current controller's numerical routines,
links the pinned Clarabel native solver, and runs a Linux executable without
MATLAB Runtime. It also tests the same generated kernel inside the existing
closed-loop MATLAB driver. **The complete perception-to-command controller has
not been converted to a standalone executable.** In particular, parsing,
prediction construction, reference/terminal synthesis, encounter conditioning,
and carried-certificate transfer remain in MATLAB.

The native executable accepts an already prepared numerical program. Its timed
call includes admission or sparse conic construction, solver preparation,
Clarabel, and independent physical/terminal/CLF/geometric verification. Loading
and allocating the fixture, reference synthesis, and the preceding formulation
are excluded. This is a numerical-frame benchmark, not a full-pipeline or
Raspberry Pi timing claim.

## Implementation

- `avoidanceStageQp` uses numerical sparse triplets instead of heterogeneous cell
  concatenations. The original coefficients, objective and cones are retained.
- `solveHardCbfClf.inspect` exposes the same hard-row, terminal and soft-CLF
  verifier used by the MATLAB entry. It explicitly rejects nonfinite CLF data.
  The geometric arithmetic reserve is shared through
  `avoidanceSafetyGeometry.jointAllowance`.
- Scalar admission retains the signed affine section and interval subtraction.
  Its interval buffer is preallocated; sparse indexing uses numerical indices.
  Global reductions state their dimension explicitly. Arithmetic-scale objective
  ties use interval order, preventing different SVD/BLAS rounding from selecting
  opposite symmetric minima. The tie threshold is `64*eps*(1+abs(bestCost))`;
  no hard constraint is softened.
- `standaloneControllerFrame` calls these shared numerical algorithms, retains
  the verified incumbent if an improvement is rejected, and uses the same
  Clarabel tolerances and 400-iteration limit. Benchmark calls have no imposed
  wall-clock timeout. The native bridge is C; Clarabel itself remains its
  existing compiled Rust implementation.
- Packing removes labels and diagnostics, preserving target identity through a
  bijective numeric mapping. Empty target sets and the distinct
  `terminalOptimization=true` mode are outside this replay entry's tested scope.
  All captured frames use the ordinary finite prediction family.
- Build and capture tooling is in `scripts/`. Generated C, libraries, MEX,
  executable, binary fixtures and full traces remain outside Git in the task
  cache. The default MATLAB controller has not been switched to this experimental
  adapter. No ROS, Raspberry Pi, network, estimator or vehicle-model changes
  were made.

Direct code generation of `collisionAvoidanceController` stopped at dynamic
configuration-cache typing and string-array input parsing. The explicit numeric
boundary is the implemented partial port; there is no claim that these entry
restrictions have been resolved.

## Code-generation validation findings

The initial unchecked C probe differed substantially from MATLAB despite
returning a certified plan. Inspection found that implicit `min/max` on the
variable-size list of violated stages selected a different operating dimension.
After explicit all-element reductions, only symmetric objective ties remained;
consistent numerical tie handling resolved those. A checked MEX build also
identified an implicit `any` reduction on a variable-size interval, which was
made explicit before the final builds.

This behavior is documented by MathWorks in [default-dimension incompatibilities
for variable-size code generation](https://www.mathworks.com/help/coder/ug/limitations-with-variable-size-support-for-code-generation.html).
The investigation used both checked MEX execution and independent MATLAB
verification of native results; compilation alone was not treated as validation.

## Experiment configuration

- Host: AMD Ryzen 7 7800X3D, Linux x86-64; MATLAB R2026a Update 3, MATLAB Coder,
  GCC; one numerical thread and one Clarabel thread. Ordinary desktop scheduling
  and frequency scaling remain active; these are observed maxima, not WCET bounds.
- Straight reference and analytic left circular reference, curvature
  `0.01 m^-1` (radius 100 m), each with stationary, oncoming and crossing targets.
- Existing research scenarios, seed 20260912, cruise 8 m/s, sensing range 16 m,
  50 ms input hold/prediction node/control update, nominal horizon 1.6 s,
  600 requested holds (30 s). No road boundaries, physical clearance buffer or
  new trajectory branches were added.
- Exact sensing and declared zero-residual affine plant. The established safety
  claim remains at hold nodes. Straight and circular crossing scenarios retain
  their existing different target speeds (32 and 4 m/s respectively).
- Capture runs allow 30 s computation for diagnosis. Their instrumentation and
  file I/O invalidate their timings as performance measurements.
- 278 prepared active-target frames are replayed, including six fresh encounter
  frames and 272 inherited continuations. One circular crossing frame is a
  rejection. Each numerical frame has two warmups and five measured calls.
  This is 1,390 measured calls per implementation.

## Measured results

The final prepared-program replay has the following timings in milliseconds.
Both implementations use identical prepared inputs and the same numerical
entry. The MATLAB column includes the already-native Clarabel MEX backend.

| Scenario | Frames | MATLAB median / max | Standalone C median / max |
| --- | ---: | ---: | ---: |
| Straight stationary | 84 | 20.365 / 66.169 | 12.414 / 28.015 |
| Straight oncoming | 48 | 11.731 / 24.242 | 6.825 / 15.487 |
| Straight crossing | 13 | 6.525 / 9.094 | 3.139 / 4.095 |
| Circular stationary | 84 | 12.072 / 30.108 | 10.361 / 24.134 |
| Circular oncoming | 48 | 7.798 / 16.717 | 6.278 / 13.711 |
| Circular crossing, rejected | 1 | 6.704 / 7.153 | 3.925 / 3.989 |
| **All 1,390 measured calls** | **278** | **12.406 / 66.169** | **8.523 / 28.015** |
| Inherited continuation only, 1,360 calls | 272 | 12.755 / 66.169 | 8.895 / 28.015 |

The pooled numerical median improves by about **1.46 times**. Zero native calls
exceed 50 ms; one MATLAB numerical call does. This does not include the upstream
preparation and is not an every-frame deadline guarantee.

The slowest native call is the second straight stationary frame, with 95
remaining prediction stages: 15.519 ms assembly and solver preparation,
12.018 ms native solve, and 0.474 ms verification. Their sum is 28.011 ms;
the enclosing call is 28.015 ms. Sparse problem construction still costs more
than the solver on this particular frame.

Independent MATLAB checks accept all 277 returned native plans and agree with
the one rejection. Maximum decision difference from the MATLAB replay is
`8.9378e-8`; maximum wrapped separation-angle difference is `4.2590e-5 rad`.
Every returned plan passes the original physical, terminal, soft-CLF and
nonlinear support checks. The checked MEX version also executes all 278 frames,
with maximum decision difference `1.9811e-15` and zero angle difference. On 28
sampled programs, the refactored `P`, `q`, `A`, `b` and cone sizes exactly match
the pre-task implementation.

### Complete hybrid closed-loop measurements

The generated kernel is then run through a **checked MEX adapter**, with two
curvatures and three target scenarios. After a separate 120-hold warmup per
scenario, five trials execute 600 holds; circular crossing rejects admission.
The complete frame includes input preparation, upstream formulation, structure
packing/MEX conversion, the native kernel, and final MATLAB certification.
This MEX build retains Coder's default development checks. MathWorks documents
that [MEX runtime checks can add execution cost](https://www.mathworks.com/help/coder/ug/controlling-run-time-checks.html);
these measurements must not be interpreted as the timing of a full standalone C
pipeline or as an optimized release MEX benchmark.

| Complete-frame group | Calls | Median ms | Maximum ms | Above 50 ms |
| --- | ---: | ---: | ---: | ---: |
| All attempts, including target-free cruising and one rejection | 3,001 | 7.624 | 92.155 | 9 |
| Accepted frames with an active target | 277 | 19.583 | 92.155 | 9 |
| **Inherited active-target continuation** | **272** | **19.494** | **57.804** | **7** |
| Accepted fresh target frames | 5 | 28.805 | 92.155 | 2 |

The small all-frame median is dominated by target-free MATLAB cruising. It
should not be used as the median cost of active avoidance. **Even warmed
continuation still misses 50 ms in this hybrid integration.** The direct native
replay demonstrates a faster numerical kernel, while the complete pipeline
remains only partially ported and is not yet qualified against every 50 ms hold.

Compilation preserves the existing experimental outcomes: straight stationary
and oncoming complete but have sampled inter-node footprint gaps of
`-0.000551365 m` and `-0.00538249 m`; straight crossing and circular
stationary/oncoming are collision-free at the diagnostic samples; circular
crossing still finds no admitted plan. No improvement in admission capability
or whole-hold collision safety is claimed.

## Verification and remaining work

The full MATLAB suite passes **726/726**, including four new behavior tests.
After the final explicit-dimension changes, the 22 native-adapter and
scalar-admission tests pass again. C generation and GCC linking succeed; `ldd`
shows only ordinary system libraries, with no MATLAB Runtime dependency. Python
AST checks and `git diff --check` pass. Code Analyzer reports four performance
advisories plus ten notices about an unavailable user analyzer-settings file;
no executable-code error is reported. Numerical sparse indexing is intentional
because the corresponding logical sparse indexing is not code-generation
compatible.

Remaining implementation is the native upstream preparation and carried-state
interface, including target-free and terminal-optimization modes, followed by a
true standalone closed-loop simulation. The current executable is a useful and
validated numerical benchmark, **not a deployable complete controller**. No
Raspberry Pi, ROS, physical-vehicle or hardware WCET result is inferred.

## Reproduction

From the repository root, with the existing pinned native solver built:

```matlab
addpath('scripts');
work = '/absolute/external/benchmark-directory';
captureStandaloneControllerFrames(fullfile(work,'capture'), ...
    Curvatures=[0,.01], SampleCount=600);
data = load(fullfile(work,'capture','stationary-0','frames','frame-0001.mat'));
standaloneControllerBenchmark.build(data.p,data.c,fullfile(work,'native'));
standaloneControllerBenchmark.buildMex(data.p,data.c,work);
prepareStandaloneControllerReplay(fullfile(work,'capture'),fullfile(work,'native'));
```

The generated `native/controller-replay` takes `repetitions warmups fixture...`.
Pass the binary files listed in `capture/matlab-replay.json` in that order and
save stdout as `native-replay.jsonl`. Fixture files are trusted local benchmark
artifacts, not a supported external input protocol. Compile-time numerical
configuration must match every replayed frame.

```matlab
validateStandaloneControllerReplay(fullfile(work,'capture'), ...
    fullfile(work,'native-replay.jsonl'));
runStandaloneKernelClosedLoop(work,fullfile(work,'hybrid'));
```

`runStandaloneKernelClosedLoop` changes only a disposable source copy. It uses
the generated kernel through MEX and retains the MATLAB orchestration and final
verifier. Its measured complete-frame times include preparation and conversion;
target-free frames continue to use the original MATLAB path.

Machine-readable numerical results and technical artifact hashes are in
[`STANDALONE_C_BENCHMARK_20260920.json`](STANDALONE_C_BENCHMARK_20260920.json).
The local raw root is
`/home/zai/.cache/collisionAvoidance/standalone-c-20260919/`; work began September
19 and compilation/validation continued after midnight September 20.
