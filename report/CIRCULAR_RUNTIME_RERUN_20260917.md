# Circular controller rerun after runtime optimization

September 17, 2026. Tested controller commit: `8f4fa087b91c1a231c615fe1d2114bea0b45cecd`
(`Reduce controller frame time without algorithm changes`). No controller,
configuration, solver or experiment-driver changes were made for this rerun.

**Functional completion is observed in all 16 circular cases, using independent
5 s diagnostic trials where the strict trial fails. Only 8 of 16 cases satisfy
the required 100 ms complete-frame limit.** All four cruise and four oncoming
strict trials complete 300 holds. All four stationary and four crossing strict
trials stop at their first encounter-admission frame and issue no command.
Their diagnostic runs complete avoidance and return to curved cruise, but do
not qualify as real-time operation.

## Protocol and scope

Reused `scripts/runCircularArcControllerValidation.m` and
`scripts/runExactStateRecursiveFeasibilityScenario.m` unchanged. MATLAB
26.1.0.3276743 (R2026a) Update 3, one computational thread, 8 m/s reference,
0.1 s holds, 300 holds (30 s), curvatures +/-0.01/m and +/-0.02/m
(100 m and 50 m radii), 16 m confirmation range, 0.25 m required clearance,
seed 20260912, zero initial tracking error and exact ego/target measurements.
There are no physical road boundaries or estimator calls. The ego executes
the exact declared held affine generator. Each arc uses its own curved trim,
including lateral velocity, heading offset and yaw rate.

Targets are stationary or follow inertial straight lines: the oncoming target
follows the tangent at reference station 30 m and the crossing target follows
the normal at station 15 m. This is not a test of targets following the circle.
The plant and sensing assumptions do not establish nonlinear real-vehicle or
estimator-controller performance.

The five focused test classes ran first in the same MATLAB process, followed
by the driver's six discarded two-hold stationary startup trials. Then each
strict case ran with a 100 ms controller work budget. Every strict failure was
repeated independently with a 5 s diagnostic budget. The latter changes only
the search time budget, not the control hold period or the runtime criterion.
No MATLAB tests or other batch simulations were launched concurrently with
this timing matrix. Startup was excluded explicitly; encounter admission was
included. These observations are not a worst-case execution-time bound.

Measured complete-frame time includes measurement/input construction and the
controller, excluding offline truth integration and geometric audits. The
failure timer is checked at search/solver boundaries, so an exhausted 100 ms
budget may return an error later than 100 ms; those frames remain failures.

## Results

The completed trial in this table is the strict trial when it succeeds and the
independent diagnostic trial otherwise. Minimum margin is sampled rectangle
separation **minus** the required 0.25 m. The whole-hold safety argument comes
from the independent hard certificate, not the offline samples.

| Curvature (1/m) | Scene | Strict holds | Strict max (ms) | Completed trial | Completed max (ms) | Min margin (m) | Peak lateral error (m) |
| --- | --- | ---: | ---: | --- | ---: | ---: | ---: |
| +0.01 | cruise | 300/300 | 45.296 | strict | 45.296 | n/a | 0.000 |
| +0.01 | stationary | 0/300 | 128.573 | diagnostic | 174.428 | 0.175471 | 2.495 |
| +0.01 | oncoming | 300/300 | 97.856 | strict | 97.856 | 0.253852 | 2.904 |
| +0.01 | crossing | 0/300 | 111.524 | diagnostic | 221.843 | 0.212392 | 4.793 |
| -0.01 | cruise | 300/300 | 31.201 | strict | 31.201 | n/a | 0.000 |
| -0.01 | stationary | 0/300 | 126.281 | diagnostic | 144.586 | 0.175477 | 2.495 |
| -0.01 | oncoming | 300/300 | 88.249 | strict | 88.249 | 0.254036 | 2.904 |
| -0.01 | crossing | 0/300 | 102.645 | diagnostic | 165.979 | 0.192102 | 4.804 |
| +0.02 | cruise | 300/300 | 11.960 | strict | 11.960 | n/a | 0.000 |
| +0.02 | stationary | 0/300 | 102.434 | diagnostic | 168.021 | 0.317661 | 2.899 |
| +0.02 | oncoming | 300/300 | 89.063 | strict | 89.063 | 0.445769 | 3.054 |
| +0.02 | crossing | 0/300 | 102.463 | diagnostic | 243.402 | 0.353400 | 4.886 |
| -0.02 | cruise | 300/300 | 11.675 | strict | 11.675 | n/a | 0.000 |
| -0.02 | stationary | 0/300 | 102.965 | diagnostic | 165.727 | 0.316985 | 2.905 |
| -0.02 | oncoming | 300/300 | 84.830 | strict | 84.830 | 0.445795 | 3.054 |
| -0.02 | crossing | 0/300 | 102.763 | diagnostic | 418.456 | 0.558387 | 4.331 |

Every completed run has 300/300 independently certified issued commands and
zero terminal/fallback commands. All twelve obstacle cases confirm release.
Across those cases, the lowest sampled rectangle clearance is 0.425471 m,
including the 0.25 m requirement. The lowest longitudinal speed is 5.557 m/s;
none of the completed obstacle runs stops the vehicle. Lateral excursions up
to 4.886 m are permitted by this experiment's absence of road boundaries.

At 30 s, and throughout the last 2 s, every completed obstacle trial has
absolute lateral error below 1.92e-5 m, heading error below 8.1e-9 rad and
longitudinal speed error below 1.14e-7 m/s relative to the curved trim.
These are finite-run recovery measurements, not a general convergence theorem
for a controller with unrestricted CLF slack.

## Remaining bottleneck and a regression

All eight diagnostic runs exceed 100 ms only at frame 0, their fresh encounter
admission. The maximum non-admission frame across completed obstacle cases is
56.047 ms. Oncoming admission occurs at frame 26 (2.6 s) and now takes
84.830--97.856 ms; all four strict trials complete. The slowest oncoming trial
has only 2.144 ms measured headroom, so the batch does not establish robust
timing margin for additional estimator/perception work.

The admission telemetry separates cumulative formulation/reformulation time
from cumulative solve time. It is not a function-level profiler. The remaining
frame time includes input preparation, certification/output and orchestration.

| Curvature (1/m) | Scene | Horizon holds | Formulation (ms) | Solve (ms) | Support families | Restoration / hard / native solves | Max later frame (ms) |
| --- | --- | ---: | ---: | ---: | ---: | --- | ---: |
| +0.01 | stationary | 48 | 74.266 | 85.046 | 1 | 1 / 1 / 5 | 53.230 |
| +0.01 | oncoming | 32 | 37.820 | 54.539 | 1 | 1 / 1 / 6 | 30.765 |
| +0.01 | crossing | 48 | 86.973 | 128.725 | 1 | 2 / 2 / 9 | 42.141 |
| -0.01 | stationary | 48 | 61.112 | 79.687 | 1 | 1 / 1 / 5 | 47.513 |
| -0.01 | oncoming | 32 | 33.243 | 53.040 | 1 | 1 / 1 / 6 | 28.743 |
| -0.01 | crossing | 48 | 59.388 | 103.071 | 1 | 1 / 1 / 6 | 43.798 |
| +0.02 | stationary | 48 | 60.405 | 104.099 | 1 | 1 / 1 / 6 | 47.801 |
| +0.02 | oncoming | 32 | 35.961 | 51.039 | 1 | 1 / 1 / 6 | 27.312 |
| +0.02 | crossing | 48 | 85.739 | 153.477 | 1 | 2 / 2 / 10 | 41.814 |
| -0.02 | stationary | 48 | 60.861 | 101.354 | 1 | 1 / 1 / 6 | 47.573 |
| -0.02 | oncoming | 32 | 32.912 | 49.886 | 1 | 1 / 1 / 6 | 27.053 |
| -0.02 | crossing | 56 | 142.992 | 269.288 | 2 | 3 / 2 / 14 | 56.047 |

The 50 m right-turn crossing case is a material regression: 418.456 ms versus
240.573 ms in the previous local-pose report. Its search visits two support
families, performs three restoration and two hard solves, and invokes the
native solver fourteen times. The prior trace used one family, one restoration,
one hard solve and nine native calls. The new run remains hard-certified and
recovers, but runtime optimization did not improve every admission case.
Current cumulative formulation and solve times are 142.992 and 269.288 ms.

The latest implementation changes how many violated rows enter each numerical
working set at once. Its own implementation report documents that restoration
minimizers are nonunique and can change the subsequently selected support
directions. The observed changed family/solve counts are consistent with this
mechanism; this rerun does not isolate causation with a numerical-path ablation.
Historical comparisons are not paired same-process timing benchmarks.

The previous circle report had 4/16 strict successes (cruise only); the current
run has 8/16. Stationary diagnostic maxima are now 144.586--174.428 ms, compared
with 177.518--251.746 ms previously. The remaining failures are finite-time
admission-search failures under the strict work budget, not evidence that the
physical avoidance task or the final hard program is infeasible: independent
longer-budget trials find certified complete plans for the same initial scenes.

## Validation and reproduction

Focused checks: **33 passed, 0 failed, 0 incomplete**, covering circular trim,
constant-curvature parsing, local pose-domain enclosure, hard domain rejection,
carried-witness preservation, native/interpreted geometry and bicycle parity.
The full 659-test result in the implementation report belongs to that earlier
task and was not rerun or relabelled as a new full-suite check here.

```matlab
results = runtests({'tests/circularArcExactStateScenarioTest.m', ...
    'tests/constantCurvatureReferenceTest.m', 'tests/curvedPoseDomainTest.m', ...
    'tests/avoidanceNativeGeometryTest.m', 'tests/bicycleNativeKernelTest.m'});
disp(table(results)); assertSuccess(results);
addpath('scripts');
runCircularArcControllerValidation( ...
    OutputDirectory='/home/zai/.cache/collisionAvoidance/circular-rerun-20260917/validation', ...
    SampleCount=300);
```

Invoked with `matlab -singleCompThread -batch`. Raw JSON/MAT traces, the full
test/experiment log and the aggregation script are retained under
`/home/zai/.cache/collisionAvoidance/circular-rerun-20260917/`.
The committed [machine-readable summary](CIRCULAR_RUNTIME_RERUN_20260917.json)
contains all strict failures, diagnostic outcomes, raw-trace hashes, the tested
commit and per-case admission telemetry. Earlier results are retained in
[the local-pose report](CIRCULAR_LOCAL_POSE_VALIDATION_20260917.md) and the
[runtime implementation report](CONTROLLER_RUNTIME_OPTIMIZATION_20260917.md).
