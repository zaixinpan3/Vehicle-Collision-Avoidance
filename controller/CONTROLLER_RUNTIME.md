# Controller runtime and sparse QP implementation

Current runtime: version 10 uses a sparse SOCP, with bounded geometric
relinearization and independently checked physical decisions. See
[FINITE_SENSING_CONTROLLER.md](FINITE_SENSING_CONTROLLER.md) for current scope and
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md) for the interface. The dated
measurements and earlier formulations below are historical engineering records;
they do not state the current controller's timing or certification guarantees.

## Quadratic input and relaxation objective (2026-09-06)

The current formulation minimizes normalized head input effort and squared
continuous-time CLF relaxation. It introduces no desired acceleration or
steering target. The sparse lift, native solver, hard constraints and explicit
pre-sampling initialization remain. See
[QUADRATIC_CLF_INPUT_OBJECTIVE.md](QUADRATIC_CLF_INPUT_OBJECTIVE.md) for the
current validation. The earlier relaxation-only measurements in
[CLF_RELAXATION_ONLY.md](CLF_RELAXATION_ONLY.md) and timings below describe
preceding objectives and do not validate the current controller.

## Historical continuous-time CLF-QP update (2026-09-06)

The CLF-QP conversion used `avoidanceStageQp` with the unchanged sparse
hard-safety and dynamic rows, the same quadratic input objective, and one
affine continuous-time CLF constraint. The native Clarabel bridge receives
only the zero and nonnegative cone dimensions. It requires no Lorentz cone,
objective epigraph, solver rebuild, or additional solve.

The optional joint solver hook now exposes a standard condensed QP
(`H`, `f`, `A`, `b`, `Aeq`, `beq`, `lb`, `ub`) and returns `[plan; delta]`.
A regression solves this interface with MATLAB `quadprog` and compares its
objective with the native sparse result. `clfDerivativeResidual` replaces
`clfExactResidual`; it measures `LfV + LgV*u0 + alpha*V0 - delta`.
`clfDerivative` and `clfDecayRate` report the derivative and rate separately.
The rate is in inverse seconds and the slack is in `V` per second.

The existing `solveAvoidanceSocpMex` and `buildAvoidanceSocpSolver` names are
retained for the general solver bridge and build entry. The online problem
is a QP. See [PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md) for the
formulation and its frozen-reference and sampled-control limitations.

## Historical runtime study before the CLF-QP conversion

The following measurements and equivalent-SOCP discussion describe the
previous exact discrete-time CLF implementation. They are preserved as
historical observations and do not establish the new QP's timing or
closed-loop performance.

The controller still solves one certificate-preserving convex problem per
sample. The performance objective, 24-stage experiment head, 74-stage
continuation, exact first-step CLF, physical limits, geometric certificates
and independent acceptance checks are retained. The numerical transcription
and implementation are changed to reduce runtime.

## Diagnosis

The baseline is commit `856be7ce55c85b71b569781d4ed1954ab5d74bd1`.
Replaying its recorded straight and arc avoidance inputs gave median complete
controller times of 178.8 and 207.8 ms. A separate MATLAB profile of 16
representative calls attributed 2.590 of 4.325 s to SeDuMi, 1.217 s to problem
formulation, and 0.787 s within formulation to geometric certificates.
Profiler times include instrumentation and are not deadline measurements.

Replacing SeDuMi with a native solver on the same condensed transcription did
not solve the problem: that prototype took approximately 0.55--1.44 s per
solve and often ended with limited numerical status. The decisive change was
to retain the chain of stage states instead of condensing it into controls.

## Equivalent sparse formulation

`avoidanceStageSocp` augments the physical decision `[u_0,...,u_(M-1),delta]`
with `x_1,...,x_M`. It assembles the held-input affine dynamics as sparse
equalities. Geometry, model-domain and axle-force rows touch only the relevant
stage. The original terminal velocity equalities and final fixed input remain
equalities; duplicate bounds on that fixed input are omitted from the native
nonnegative cone.

Eliminating the added states recovers the original condensed constraints.
One set of local coefficients generates both native rows and condensed
acceptance rows. Acceptance reconstructs states from the returned input plan;
it does not accept the solver's auxiliary states as a substitute for those
predictions. Behavioral tests compare arbitrary input plans, including
infeasible ones, to verify equality of hard-row, dynamics and rest residuals.

Clarabel accepts the quadratic objective directly. Consequently the large
input-cost epigraph cone is unnecessary. The only Lorentz cone has dimension
seven and represents the same exact first-step quadratic CLF. The solver-hook
interface retains the equivalent two-cone condensed format for existing fault
injection. It is not built on the default sample path.

The native prototype solved the 16 captured problems in approximately
12.5--17.3 ms, including its MEX call. Input-plan hard residuals were at most
7.7e-12; terminal reconstruction residuals were at most 8.1e-12. First-command
differences from SeDuMi were at most 0.000769 in the unscaled two-component
infinity norm. Weakly weighted continuation controls need not coincide across
solvers. Limited solver statuses are reported, not relabeled as optimal.
Independent physical acceptance remains mandatory.

Other implementation changes are:

- Batched slip-domain and geometric-row assembly, with preallocated storage.
- Batched exhaustive polyline projection in C++, preserving first-minimum
  ties, endpoints and every segment; no nearest-neighbor approximation.
- Batched frame-bound corner enumeration in C++, with binary station searches
  that include both sides of vertices. MATLAB implementations remain available
  for direct comparison and use without the optional geometry binaries.
- Reuse of identical adjacent bicycle matrices, without rounding scheduled
  speed or curvature, and vectorized initial curvature lookup.
- Reuse of the empty geometry-node template and direct quadratic-polynomial
  evaluation.
- Retained native function handles after first load, avoiding repeated MEX
  file searches. The build script clears these handles before replacing a
  binary. Clear `laneGeometry` and `solveHardCbfClf`
  when manually changing backend paths in an existing MATLAB session.
- Phase wall times in `planningProblem.metadata.runtime`, separating input
  preparation, prediction, formulation/witness checking, solving, acceptance
  and diagnostics. An outer `tic`/`toc` remains the complete-call measurement.

## Build and dependencies

Run from the repository root before controller experiments:

```matlab
addpath('scripts');
information = buildAvoidanceSocpSolver();
```

The recipe is validated for Linux x86-64, MATLAB R2026a Update 3, a configured
C++ MEX compiler, Git and Rust/Cargo. Other platforms need an adapted build
recipe. It clones the Apache-2.0
[Clarabel C API](https://github.com/oxfordcontrol/Clarabel.cpp) at
`0de6259a3edfd5cc041ec42b2148599ce63e73cb`, with Clarabel.rs 0.11.1 at
`25540f559592068d0c8a80e46ded1b21760212a1`.
`config/clarabelCargo.lock` fixes transitive dependencies. Release compilation
uses the QDLDL backend; the solver uses one thread. No separate BLAS backend is
required by this native solver. MATLAB still uses its own numerical libraries
for prediction and CLF construction.

Dependency sources and generated MEX files stay under `solver/clarabel` and
are excluded from the project commit. The checked-in files are the small
bridges, build recipe and lockfile. The controller loads the built backend
when necessary; it never downloads or compiles during a control sample.
Missing solver binaries cause an explicit failure, not a hidden solver retry.
Control System Toolbox remains required for the CLF; Optimization Toolbox's
`secondordercone` is used by the compatibility hook.

On this host MATLAB's post-link inspection reports a valid GCC-built binary
as "not a MEX file". The build recipe handles only that specific diagnostic,
removes any stale binary before compiling, and requires MATLAB to load and
execute independent SOCP, projection and frame smoke tests. Other compiler
errors are propagated. The diagnostic is retained in the returned build
information. Child build tools run without MATLAB's bundled library search
path because the system Rust compiler requires a newer `libstdc++`.

## Reproduce measurements

Use a dedicated MATLAB process to reduce contention, and record its thread
setting. The evaluated configuration uses `matlab -singleCompThread`.
For recorded-input replay:

```matlab
addpath('scripts');
data = load('/path/to/straight_avoidance.mat');
report = benchmarkControllerRuntime(data.result, Repetitions=3, ...
    OutputDirectory='/tmp/controller-replay');
```

Each pass resets certificate state, then maintains its own continuation over
the original input sequence. It includes target acquisition and readmission.
The first pass is separate from later passes, and every sample and deadline
miss is retained. The measurement excludes perception and plant simulation;
it is not a new closed-loop experiment. Steering and acceleration differences
from the recorded commands are reported separately in their physical units.

For the independent paired vehicle experiments:

```matlab
study = runControllerDesignExperiments( ...
    OutputDirectory='/tmp/controller-plant-study', Plot=false);
```

The evaluator retains the original all-calls 50 ms criterion. MATLAB loading
and JIT initialization can be much slower than steady calls, and an ordinary
desktop OS can introduce scheduling outliers. Warm measurements establish
observed throughput, not a worst-case execution-time guarantee. The offline
plant harness does not model control computation latency. The controller's
exact-model, prediction-node safety scope is unchanged.

## Completed validation on 2026-09-05

The final four 10 s PassVeh14DOF trials completed 200 intervals each. Both
target-free nominal runs reached the intended counterfactual contact, and
both range-triggered avoidance trials passed every functional criterion.
No physical, collision, recovery, sampling or horizon requirement was relaxed.

| Final avoidance trial | Straight | 400 m left arc |
| --- | ---: | ---: |
| Median complete controller call [ms] | 37.336 | 38.693 |
| 95th-percentile call [ms] | 42.203 | 42.874 |
| Maximum call [ms] | 119.561 | 75.958 |
| Calls exceeding 50 ms | 5 / 200 | 1 / 200 |
| Minimum sampled SAT separation [m] | 1.450020 | 1.790228 |
| Maximum hard-row violation | 2.32e-11 | 4.25e-11 |
| Maximum terminal velocity residual | 5.57e-11 | 5.16e-11 |
| SOCP calls per sample | 1 | 1 |
| Fallback commands | 0 | 0 |
| All functional criteria | Pass | Pass |
| Every call within 50 ms | Fail | Fail |

The largest avoidance calls occur at first target acquisition (3.10/3.15 s).
They spend 18.27/20.24 ms in solving, with 23/24 iterations; preparation and
formulation account for most of the remaining time. The initial target-free
call in the fresh process takes 585.82 ms. Thus neither steady throughput nor
the solver's own time establishes a hard deadline. The final implementation
retains native function handles, avoiding per-call backend file searches.
An earlier four-trial run before that cache had the same physical results
but larger timing outliers; those observations do not isolate cache effects
from host load. The shared desktop also exhibited substantial I/O wait.

A separate same-process comparison used the same 200 recorded inputs per
scene, the same original configuration, one MATLAB compute thread and three
passes per implementation. Passes two and three gave:

| Recorded-input replay | Baseline median [ms] | Optimized median [ms] | Optimized p95 [ms] | Optimized deadline misses |
| --- | ---: | ---: | ---: | ---: |
| Straight, pass 2 | 262.235 | 37.463 | 41.633 | 0 / 200 |
| Straight, pass 3 | 249.563 | 37.119 | 41.195 | 0 / 200 |
| Arc, pass 2 | 272.080 | 38.359 | 43.067 | 1 / 200 |
| Arc, pass 3 | 273.560 | 38.121 | 43.781 | 1 / 200 |

Every baseline replay call missed 50 ms. The observed median improvement is
approximately sevenfold. First passes remain recorded separately; optimized
straight's first pass has eight misses and a 459.28 ms maximum. These are
sequential measurements on a shared host, not randomized isolated hardware
benchmarks or worst-case execution-time bounds.

Requested native accuracy is unchanged at 1e-9, but most solves terminate
with almost-solved or numerical status: 187/200 straight and 194/200 arc
avoidance calls have such limited statuses, including 42/29 numerical
terminations. All pass the independent physical acceptance checks. The timing
improvement therefore concerns accepted feasible candidates; exact optimality
is not claimed. The flags and numerical outcomes remain visible.

Sixteen captured baseline formulations were compared against the new
assembly on 12 decisions each. The maximum hard-row residual difference is
7.47e-14; dynamic maps and CLF matrices agree exactly in this comparison.
The independent tests also verify the sparse lift, objective, exact CLF,
projection ties/endpoints and frame bounds against the MATLAB reference.

The broad regression run and a rerun of its affected estimator tests produce
298 passes and six failed/incomplete tests among 304. The initial run had 92
estimator setup failures because a pre-existing dependency check mistook
MATLAB's `vrnode/optimize` for YALMIP. Explicitly adding the installed YALMIP
and SeDuMi paths resolves 86; the remaining six estimated-state scenarios
still request nonzero ego/model uncertainty outside this controller's domain.
The 50 tests affected by the final function-handle cache also pass. Code
Analyzer reports zero findings in all 15 changed MATLAB files; the three C++
bridges pass `-Wall -Wextra -Werror` syntax checks. Native build smoke tests
pass. A separate oncoming steering continuation with gain 0.80 survives forced
solver outages through rest: minimum rectangle clearance is 0.250000145 m
(0.250034647 m after rotation) for 0.25 m, with terminal residuals
6.13e-15 / 2.69e-13.

For full-suite reproduction in an existing MATLAB session, explicitly load
the estimator's separate dependencies before `runtests('tests')`:

```matlab
addpath(genpath('solver/YALMIP'), genpath('solver/sedumi'));
results = runtests('tests');
```

The strict real-time requirement remains open. Its next validation must
include startup/acquisition paths, bounded execution on the deployment target
and the effect of computation delay. This work does not extend the safety
claim to continuous motion, the nonlinear plant, or uncertain ego dynamics.
