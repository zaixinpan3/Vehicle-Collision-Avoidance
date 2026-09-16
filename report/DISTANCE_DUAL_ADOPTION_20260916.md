# Distance-dual convexification: adoption and straight-scene validation

Date: September 16, 2026. Baseline: commit
`4171da3938b028cf7571a1fffec763fdaaca24a6`. Final controller certificate format: 30.

## Outcome

The controller now uses Li et al.'s two-stage mechanism: optimize geometric
distance duals, fix the resulting continuous separation directions, then
optimize the control trajectory. All three final exact-state straight trials
complete 300 holds (30 s), preserve collision safety, and recover along-path
cruise at approximately 8 m/s. Stationary and oncoming avoidance do not reverse
in these final runs; their maximum lateral excursions are 2.507 m and 3.142 m.

**The complete-frame 100 ms requirement remains unmet.** Strict stationary
and oncoming trials stop at first admission. The first active stationary
successor also takes a median 149.089 ms in repeated frozen-frame measurement.
The method improves the tested trajectory behavior but is not a completed
real-time solution. These are controller-only declared-affine-plant trials;
neither nonlinear physical-vehicle validity nor a joint observer-controller
timing guarantee is established.

## What is adopted, and what is retained

Source: Li, G., Zhang, X., Guo, H., Lenzo, B., and Guo, N. (2023), *Real-Time
Optimal Trajectory Planning for Autonomous Driving with Collision Avoidance
Using Convex Optimization*, Automotive Innovation 6, 481--491,
[DOI 10.1007/s42154-023-00222-7](https://doi.org/10.1007/s42154-023-00222-7).
The local reference PDF and the publisher's Section 3, equations (12)--(13),
were checked. The paper first solves a nonnegative distance dual and then
fixes its multipliers in a convex trajectory program. Its trajectory QP
permits a penalized collision slack.

Our implementation adopts the distance-dual direction calculation while
retaining hard collision safety, the existing soft CLF, input cost relative
to the certificate operating point, whole-hold Bernstein checking, finite
encounter completion and the invariant terminal continuation. It remains an
SOCP and retains full ego/target rectangles and uncertainty support. State
boxes, tire-slip bounds, road boundaries in these scenarios, internal time
subdivision and fallback commands are not added. The high-gain observer is
unchanged. The complete derivation and recursion argument are in
[DISTANCE_DUAL_CONVEXIFICATION.md](../controller/DISTANCE_DUAL_CONVEXIFICATION.md).

`avoidanceSafetyGeometry.distanceDual` solves an eight-variable SOCP for the
exact fixed-orientation Minkowski configuration polygon. Its normal is
continuous rather than restricted to the eight initialization directions.
`distanceDualNormals` queries the computed anchor at each hold midpoint.
The full trajectory problem subsequently recomputes support bounds and
enforces every coefficient of the complete continuous hold; midpoint safety
alone never authorizes a command.

For an admitted encounter, the original input suffix is retained first.
New collision rows replace old ones only if that same suffix, with a finite
CLF slack, satisfies every proposed linear row and cone. Otherwise the
unchanged inherited family is optimized. The terminal cone, finite-exit
direction and absolute deadline are preserved. A final independent check
still precedes execution. This maintains the existing conditional recursive
feasibility construction without assuming that arbitrary new normals retain
feasibility.

## Why finite initialization remains

Ordinary distance is zero inside a configuration obstacle; the zero dual
vector is optimal there and supplies no separation direction. On the two
saved baseline admission fixtures, a pure cruise anchor overlaps the target
at 6 oncoming and 12 stationary hold midpoints. Direct numerical dual solves
yield 5 and 12 directions below norm 1e-6. The remaining oncoming overlap is
near a boundary and does not supply a positive-clearance certificate either.
The largest primal/dual distance discrepancies in those probes are below
1.3e-7 m. Positive-distance queries from the known certified plans produce
fully hard-certified trajectory candidates in both cases. These initialized
comparisons are not independent first-admission successes.

The online algorithm therefore tries the dual-directed hard problem first
when a usable proposal exists, and constructs the finite branch initializer
only when the proposal is unavailable or the complete hard problem is
infeasible. An incomplete numerical solve or deadline expiration is not
treated as a proof of infeasibility. Every accepted branch still receives
the same hard SOCP and independent verification. A direction initializer is
not an alternative executable controller.

## Integration findings and corrections

The first long oncoming trial stopped after 41 holds because repeated clock
subtraction made a nominally zero tube start slightly negative. This had not
affected inherited rows, but rebuilding geometry queried target propagation
at that value. Tube start times now follow the integer stage index; a
22-frame regression covers repeated shifting. The trial was rerun, not
counted as completed.

The first complete integration retained the initializer's zero objective.
Its stationary trial was collision-free but repeatedly reversed out of
range, recovered forward motion and encountered the same target again. It
completed 300 holds but ended at -2.293 m/s, with five integer initialization
calls and five release events. Calling that a cruise-recovery success would
be incorrect. The terminal certificate proved safety of those finite
episodes; it did not optimize progress across repeated episodes.

The final initializer ranks feasible search candidates by terminal path
station. This affine search objective changes no safety constraint, removes
no feasible branch, imposes no passing side or trajectory shape, and leaves
the final trajectory SOCP's existing objective unchanged. A separate
120-hold probe recovered at 8 m/s with one release and 2.507 m maximum lateral
excursion. The final three-scene campaign below confirms this behavior.
This is an initialization improvement, not a general proof of cruise
convergence for every encounter. Both the dual update and initialization
ranking differ from the baseline, so the behavioral change is not attributed
solely to the distance-dual calculation.

## Final 30-second controller-only results

Driver: `scripts/runExactStateRecursiveFeasibilityScenario.m`. Each scene uses
300 holds, h=0.1 s, reference speed 8 m/s, a complete 16 m perception region,
seed 20260912, zero ego/target measurement radii, zero target jerk/yaw
acceleration and no road boundaries. Steering is limited to 40 degrees,
normalized longitudinal input to [-1,1]; finite slew limits are not enabled
in these baseline trials. The diagnostic work budget is 30 s, while all
frame overruns are judged against 100 ms. A target-free warmup call precedes
the campaign and is excluded from the table; it took 1060.167 ms.

| Scene | Holds | Minimum sampled body gap (m; required 0.25) | Maximum lateral excursion (m) | Minimum / final speed (m/s) | Median / maximum frame (ms) | Frames >100 ms |
| --- | ---: | ---: | ---: | --- | --- | ---: |
| Stationary | 300 | 0.345066 | 2.507379 | 8.000000 / 8.000000 | 9.758 / 880.645 | 10 |
| Oncoming | 300 | 0.587276 | 3.141941 | 8.000000 / 8.000000 | 9.781 / 302.761 | 1 |
| Crossing | 300 | 9.519711 | 0.000028 | 7.999996 / 8.000000 | 9.822 / 57.777 | 0 |

All final tracking-error norms are 3.24e-7 in the script's state coordinates.
Maximum speeds are 9.9480, 9.2133 and 8.000052 m/s, respectively. Stationary
and oncoming steering reaches the physical limit. Confirmed release occurs
once in each trial, at 3.8, 4.9 and 0.7 s. No terminal/fallback command is
executed. The raw JSON separation margin excludes the required 0.25 m;
the table adds it back to report the actual sampled body gap.

Distance-dual rows are used in 27, 14 and 7 frames. The corresponding
witness-preserving updates number 27, 14 and 6; crossing's first admission
uses the dual problem directly. In 10 stationary and 8 oncoming frames,
available new directions fail the complete-witness inclusion test, so the
inherited program is retained. This explicitly exercises the recursive
feasibility guard rather than assuming all re-convexifications are safe.

Strict 100 ms reruns, in the same warmed process, stop stationary admission
at t=0 after 205.815 ms (zero commands) and oncoming admission at t=2.6 s
after 152.035 ms (26 prior target-free holds). Crossing completes 300 holds
with maximum 53.775 ms. The stopped frames issue no command. Longer-budget
diagnostic runs do not inject computation delay into plant motion; their
collision results cannot be interpreted as deadline-compliant execution.

## Repeated frame timings

`profileFiniteBranchRuntime` uses three admission repetitions and five
regular repetitions. The following complete-call medians exclude MATLAB
profiling overhead. Matrix counts are the full exposed program, including
constant rows where present.

| Frame | Variables | Matrix rows | Median full call (ms) |
| --- | ---: | ---: | ---: |
| Fresh cruise | 33 | 86 | 14.986 |
| Cruise successor | 33 | 86 | 14.922 |
| Oncoming admission | 65 | 2387 | 299.492 |
| Stationary admission | 97 | 2763 | 352.591 |
| Oncoming active successor | 63 | 2253 | 83.791 |
| Stationary active successor | 95 | 2577 | 149.089 |

The prior version measured 48.139 and 78.104 ms for its first active oncoming
and stationary successors. The added distance queries, geometry rebuilding
and changed convex programs do not produce an across-the-board runtime
improvement. Different trajectories and matrix conditioning prevent a
solver-only causal interpretation. All six profiled frame decisions repeat
within 1e-7 across timed, profiled and instrumented calls. Solver metadata
now includes distance queries in the total rather than counting only the
trajectory and integer calls.

## Validation and reproduction

The complete suite passes **609/609** before the final initialization ranking
change. The final eight affected classes pass **86/86**, including the new
stationary recovery case. Combined current coverage is **610/610**, formed
from the complete-suite results and final affected-class reruns; this is not
a second full-suite invocation. Tests cover distance-dual/primal agreement,
overlap degeneracy, direct admission without integers, hard collision rows
without slack, suffix inclusion, absolute exit deadlines and nonnegative
shifted times. Native geometry and flow parity pass in the full suite.
All 13 changed/new MATLAB files have zero factory Code Analyzer findings
after replacing one scalar-length comparison with `isscalar` in the offline
probe. `git diff --check` passes; the core controller remains at 20 sources.

An independent Python audit verifies all 900 accepted diagnostic holds:
hard certificates, nonpositive inherited-witness violations, nonnegative
terminal margins, CLF residual bounds, no fallback commands, final cruise
recovery and total solver-call accounting. The test harness initially used
one incorrect test filename; it ran no tests and was corrected. A final
post-test Git file-discovery command ran from the cache directory and failed;
the passing test results were already saved and were reused for the static
analysis step. These harness failures are not credited as successful checks.

Run from the repository root with the existing native solver installed:

```matlab
addpath('scripts');
output = '/path/to/experiment-output';
for scene = ["stationary","oncoming","crossing"]
    runExactStateRecursiveFeasibilityScenario(Scenario=scene,SampleCount=300, ...
        DeadlineSeconds=30,OutputDirectory=fullfile(output,"diagnostic-"+scene));
end
```

Repeat with `DeadlineSeconds=0.1` for strict execution; failed runs save their
report and throw an error, so an outer harness must catch it to proceed to
the next scene. `evaluateDualDistanceConvexification` provides the separate
frozen-anchor study and explicitly re-admits saved measured states instead
of interpreting an old-format witness as a new one.

Original MAT/JSON outputs, profiles, prototype findings, validation results
and the captured commit record are under
`/home/zai/.cache/collisionAvoidance/li-dual-20260916`. Only current source,
tests, theory and reports are committed. Dependencies, generated binaries,
reference PDFs and unrelated user files are excluded.
