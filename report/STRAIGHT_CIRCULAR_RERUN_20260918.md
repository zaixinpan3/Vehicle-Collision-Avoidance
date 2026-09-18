# Straight and circular controller validation after node-certificate adoption

September 18, 2026. Tested commit: `c37f4706c2ed595cc17c26b8fb1729a9872ab298`. This task changed no
controller, solver, configuration, scenario driver or network settings.

**17 of 20 cases pass the complete 30 s strict trial, the 100 ms frame
criterion and the sampled 0.25 m clearance check.** Two straight cases run
within the deadline and recover cruise but pass closer than 0.25 m between
certified nodes. One circular crossing case cannot admit a plan within
100 ms; its independent 5 s diagnostic trial completes and recovers. None of
the sampled completed trajectories has overlapping vehicle bodies.

## Current certificate and experiment scope

The intervening implementation commit `d9d37a4` adopted safety at hold nodes.
`metadata.certificateSampling` is `holdNodes` and
`metadata.wholeHoldCertificate` is false. This is a different guarantee from
the earlier [whole-hold circular rerun](CIRCULAR_RUNTIME_RERUN_20260917.md),
not simply a faster implementation of the same continuous-time constraint.
See [NODE_SAMPLED_CERTIFICATE.md](../controller/NODE_SAMPLED_CERTIFICATE.md).
The Predictive CBF continuation, terminal witness and soft CLF remain. The
final independent verifier checks the implemented node constraints.

Reused `scripts/runCircularArcControllerValidation.m` with
`Curvatures=[0,.01,-.01,.02,-.02]`, exercising its existing zero-curvature
branch as well as 100 m and 50 m circles in both directions. Each path has
cruise, stationary, oncoming and crossing trials. MATLAB R2026a Update 3,
one computational thread, 8 m/s reference, 0.1 s holds, 300 holds (30 s),
zero initial tracking error, 16 m confirmation range, 0.25 m rectangle
clearance and seed 20260912. Ego execution is exactly the declared held
affine plant with zero sensing/process errors. No estimator, road boundaries,
Raspberry Pi executable or ROS runtime is part of this experiment.

The straight crossing fixture retains its existing fast target: initially
(15,-4) m moving at 32 m/s perpendicular to the path. Its large minimum
clearance makes it a weak collision-avoidance challenge. Circular crossing
targets start 7.5 m along the negative normal at reference station 15 m and
move at 4 m/s. Circular oncoming targets follow the inertial tangent at
station 30 m. Targets do not follow the reference circle. These are the
existing declared fixtures, not a general test of arbitrary traffic.

The focused tests ran through the MATLAB MCP before a separate single-thread
MATLAB timing process. The timing driver performs six discarded two-hold
stationary warmups at curvature +0.01/m before the 20-case matrix. Full frame
time includes measurement construction and controller work; plant integration
and offline clearance audits are excluded. No tests or other batch MATLAB
simulation were launched concurrently with the timing matrix. These are
measured development timings after startup, not hardware WCET guarantees.

## Complete matrix

Minimum body separation below is the refined diagnostic minimum. All cases
use the strict trace except the +0.01/m crossing row, where separation comes
from its independent diagnostic completion. A positive body separation means
no overlap at that sampled point; the configured clearance requires at least
0.25 m. Strict result includes both timing and the driver's physical audit.

| Path curvature (1/m) | Scene | Strict holds | Strict maximum frame (ms) | Minimum sampled body separation (m) | Strict result |
| --- | --- | ---: | ---: | ---: | --- |
| straight | cruise | 300/300 | 79.236 | n/a | pass |
| straight | stationary | 300/300 | 49.216 | 0.222494 | inter-node margin |
| straight | oncoming | 300/300 | 22.078 | 0.239573 | inter-node margin |
| straight | crossing | 300/300 | 10.848 | 9.551550 | pass |
| +0.01 | cruise | 300/300 | 24.992 | n/a | pass |
| +0.01 | stationary | 300/300 | 50.074 | 0.293467 | pass |
| +0.01 | oncoming | 300/300 | 28.888 | 0.321163 | pass |
| +0.01 | crossing | 0/300 | 106.291 | 0.347574 | admission deadline |
| -0.01 | cruise | 300/300 | 23.323 | n/a | pass |
| -0.01 | stationary | 300/300 | 47.367 | 0.293466 | pass |
| -0.01 | oncoming | 300/300 | 28.660 | 0.321162 | pass |
| -0.01 | crossing | 300/300 | 53.571 | 0.288936 | pass |
| +0.02 | cruise | 300/300 | 44.422 | n/a | pass |
| +0.02 | stationary | 300/300 | 46.813 | 0.426271 | pass |
| +0.02 | oncoming | 300/300 | 29.530 | 0.427315 | pass |
| +0.02 | crossing | 300/300 | 75.409 | 0.387656 | pass |
| -0.02 | cruise | 300/300 | 11.355 | n/a | pass |
| -0.02 | stationary | 300/300 | 46.938 | 0.426386 | pass |
| -0.02 | oncoming | 300/300 | 28.565 | 0.427304 | pass |
| -0.02 | crossing | 300/300 | 61.818 | 0.397589 | pass |

The strict +0.01/m crossing run stops at frame 0 after 106.291 ms and issues
no command. Its diagnostic run executes 300/300 holds, maximum 161.035 ms,
and confirms departure. Its admission uses two support families, four
restoration and three hard solves, with eight native solver calls. Search
formulation and solve telemetry is retained in the machine-readable report.
Subsequent frames do not require this full fresh-admission search.

All other strict runs finish 300/300 holds with zero deadline misses. The
driver also repeats the two straight margin failures with a 5 s budget because
its `runtimeQualified` flag requires the safety audit to pass; those runs
reproduce the same clearance violations. They are not deadline failures.

Every completed trajectory has 300 independently checked issued commands,
no terminal/fallback command, and a final state back at its own straight or
curved 8 m/s trim. All fifteen obstacle scenarios confirm departure.
Across all completed cases, the final and last-two-second absolute lateral
error is below 1.92e-5 m, heading error below 8.1e-9 rad and longitudinal
speed error below 1.59e-7 m/s. There is no early-recovery requirement. These
are finite-run observations, not a general convergence theorem.

## Verified inter-node clearance losses

The straight stationary case has minimum node margin **+0.000013581 m**
above the required 0.25 m, yet the refined held-interval audit finds body
separation **0.222494031 m at 1.3341 s**, a **2.751 cm shortfall**.
The straight oncoming case has node margin **+0.000006955 m**, yet reaches
**0.239572671 m at 3.7718 s**, a **1.043 cm shortfall**.

Thus both satisfy the implemented node condition while violating the desired
clearance between nodes. Neither is an observed body collision, and neither
is caused by a missed 100 ms deadline. The evidence specifically exposes the
gap between the node predicate and full-time clearance. Additional iterations
of the same node-constrained solve do not establish the missing guarantee.

The independent audit reconstructs the constant-curvature held affine generator
from each saved configuration and trim, reproduces every stored successor
with zero observed numerical difference, and reproduces all original 11-point
minimum margins within 1e-9 m. It checks every node, then refines the worst
sampled hold and its two neighbors at 0.1 ms resolution. The table reports
separation minus 0.25 m. Refinement is local to those holds and is not a global
continuous-time minimum proof; its negative values are concrete violations.

| Curvature (1/m) | Scene | Node margin (m) | 10 ms sample margin (m) | Refined margin (m) | Refined minimum time (s) |
| --- | --- | ---: | ---: | ---: | ---: |
| straight | stationary | 0.000013581 | -0.027225322 | -0.027505969 | 1.3341 |
| straight | oncoming | 0.000006955 | -0.010387547 | -0.010427329 | 3.7718 |
| straight | crossing | 9.301618701 | 9.301618701 | 9.301550313 | 0.3011 |
| +0.01 | stationary | 0.088131588 | 0.044809079 | 0.043466938 | 1.2427 |
| +0.01 | oncoming | 0.097720551 | 0.071480655 | 0.071162723 | 3.9354 |
| +0.01 | crossing | 0.101603582 | 0.097662268 | 0.097573522 | 1.6739 |
| -0.01 | stationary | 0.088131830 | 0.044807893 | 0.043465756 | 1.2427 |
| -0.01 | oncoming | 0.097720075 | 0.071480082 | 0.071161854 | 3.9354 |
| -0.01 | crossing | 0.099953499 | 0.038947714 | 0.038935917 | 2.0697 |
| +0.02 | stationary | 0.176509213 | 0.176273893 | 0.176270878 | 2.3090 |
| +0.02 | oncoming | 0.204762017 | 0.177342131 | 0.177315116 | 3.7413 |
| +0.02 | crossing | 0.206680756 | 0.139191491 | 0.137655542 | 2.1656 |
| -0.02 | stationary | 0.176623214 | 0.176389247 | 0.176386065 | 2.3090 |
| -0.02 | oncoming | 0.204753219 | 0.177325835 | 0.177303618 | 3.7412 |
| -0.02 | crossing | 0.203379438 | 0.149430147 | 0.147588530 | 2.0653 |

All circular completed trajectories retain positive sampled clearance above
0.25 m, including the locally refined samples. This does not expand the online
certificate beyond its node-only scope. Returning a full-time 0.25 m guarantee
would require an inter-node motion bound or continuous-hold certificate; merely
reporting positive node margins is insufficient. No such redesign was made in
this validation task.

## Checks and reproduction

**48 focused tests passed, zero failed/incomplete:** node certificate (5),
circular exact-state scenarios (2), constant-curvature geometry (7), local pose
domains (12), native geometry (5), and recursive safety closure (17). The
prior full-suite results are not presented as a new full-suite run.

```matlab
results = runtests({'tests/nodeCertificateTest.m', ...
    'tests/circularArcExactStateScenarioTest.m', ...
    'tests/constantCurvatureReferenceTest.m', 'tests/curvedPoseDomainTest.m', ...
    'tests/avoidanceNativeGeometryTest.m', 'tests/recursiveSafetyClosureTest.m'});
assertSuccess(results);
```

The timing process was launched with `matlab -singleCompThread -batch`:

```matlab
addpath('scripts');
runCircularArcControllerValidation( ...
    OutputDirectory='/home/zai/.cache/collisionAvoidance/straight-circle-20260918/validation', ...
    SampleCount=300, Curvatures=[0,.01,-.01,.02,-.02]);
```

Raw JSON/MAT traces, test and experiment logs, `auditClearance.m`, its audit
results and the aggregation script remain at
`/home/zai/.cache/collisionAvoidance/straight-circle-20260918/`.
The committed [JSON summary](STRAIGHT_CIRCULAR_RERUN_20260918.json) preserves
all strict and diagnostic outcomes, per-frame search summaries, clearance
audits, source commit and raw-trace hashes. The English report and JSON are
the only new project artifacts; no algorithm behavior changed.
