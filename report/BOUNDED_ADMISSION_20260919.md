# Bounded admission with homogeneous support certificates

Experiment and implementation date: September 19, 2026.

The admission bottleneck was a poorly positioned, overly conservative sequence
of convex restrictions, not evidence that these unbounded-road encounters were
physically unavoidable. A trial that merely retained one hard convex restriction
and removed restoration completed **1 of 15** scenarios; it was rejected before
commit. The implemented replacement uses one geometric initialization, global
homogeneous support majorants, and at most three admission solves. It completes
**30/30** measured trials at the node-certificate level.

The old worst frame falls from 124 native calls to 2.
Thirty warmed replays of that same input have median
**43.134 ms** and maximum
**44.225 ms**, versus the preceding clean
replay median 4707.558 ms. Across two complete 15-scenario campaigns, the maximum
measured frame is **69.822 ms**.

**This is not a successful physical-avoidance qualification.** Only
**22/30** measured trials pass the positive sampled body-gap
criterion. Four scenario types still collide between certified control nodes;
the earlier baseline had two such failures. The speed improvement and improved
admission feasibility do not repair that safety defect. No physical clearance
buffer is added and no collision is counted as acceptable.

## Why the selected subset mattered

Removing road boundaries enlarges the original trajectory problem. It does not
make every convex inner approximation nonempty. Fixed or locally varying support
directions can still exclude feasible escapes, and artificial step bounds can
prevent a useful update even when actuator-feasible trajectories exist.

A behavior regression demonstrates the obstruction: relative center `(0.9, y)`,
`0 <= y <= 2`, square obstacle `[-1,1]^2`, and an initial normal `(1,0)` can have
a positive local minimum of certificate deficit. The point `y = 1.2` with normal
`(0,1)` is safe in that test. A single bad restriction therefore cannot justify a
global-infeasibility claim. The square test's 0.1 m illustrative clearance is
local test data; the vehicle controller and all reported driving trials use no
physical clearance offset.

The broad baseline completed all 15 node-feasible trials, and sampled collision
audits passed in 13. The rejected single-restriction prototype completed only
one. This is direct evidence that deleting search without improving its convex
family discards useful solutions. It is not a proof of continuous-time or
nonlinear-vehicle feasibility for every scenario.

## Implemented algorithm

1. Preserve the shifted nominal controls and states. Generate one coherent
   schedule of initial support directions using projected geometric query
   points. These points never become a tracking reference, prescribed lateral
   path, timing schedule, or executable rollout. There is one deterministic
   orientation heuristic, with a symmetry tie rule; no alternative-route list
   or multi-start loop is searched.
2. Use a nonunit normal `n + a*t`, whose norm is `sqrt(1+a^2)` and whose unit
   angle is `theta + atan(a)`. Positive homogeneity removes the old
   distance-dependent unit-circle remainder. A global touching convex bound
   handles the remaining position/direction bilinear term and ego-yaw term.
   The former 4 m position-step and 0.5 rad angle-step restrictions are removed.
   Existing certified chart domains remain hard. `positionScale = 4 m` selects
   majorant curvature; it is not a displacement bound.
3. Charge tiny numerical position/yaw uncertainty to an outward disk at fresh
   certificate construction. Do not discard it. The enlarged occupied set is
   stored and inherited, preserving the touching argument. Larger uncertainty
   keeps its original support representation.
4. When a hard pose domain proves a fixed-direction collision record safe for
   every possible state in that domain, fix that record's direction and omit
   its redundant majorant. A circumscribed ego disk gives an orientation-wide
   support bound. Exit records are never screened. The original physical
   records remain in every independent acceptance check. This removes a
   conservative numerical restriction without removing a physical obligation.
5. Bound admission to `maximumAdmissionSolves = 3`, including any base-feasibility
   solve. Keep dynamics, actuator/slew, chart, terminal and CLF constraints hard
   apart from the existing CLF slack. Geometric restoration deficits are
   internal search variables. Stop if the exact deficit fails to improve.
6. Issue an admission candidate only after independently checking the original
   support residuals and every hard base/terminal constraint. A verified
   admission witness does not need another performance solve in the same frame.
   Subsequent frames attempt one hard performance solve containing the complete
   carried witness. Failure cannot authorize an unverified command.

The controller still uses predictive safety, its finite terminal continuation,
soft CLF, and inherited occupied-set/model/exit-deadline contracts. The terminal
law is not introduced as an executable fallback. The angular search remains
local and is not complete; admission failure means no verified plan was found
within the selected family and budget. Three solver calls are a work bound,
not a worst-case execution-time proof.

The conic builder also preallocates its final sparse width. A separate comparison
found identical complete conic matrices/vectors/cone partitions before and after
preallocation alone for three fixed/uncertain, restoration/performance cases.
Its isolated speed benefit was small; the main improvement comes from avoiding
unsuccessful starts and reducing majorant conservatism and conic work.

The derivation and conditional recursive-feasibility scope are in
[JOINT_SUPPORT_CERTIFICATES.md](../controller/JOINT_SUPPORT_CERTIFICATES.md).
Saved state format is now 39; reset earlier controller states. Removed
`maximumIterations`, `maximumStarts`, `positionStep` and `angleStep` settings are
rejected rather than silently accepted. No method selector or legacy executable
search path is retained.

## Experiment protocol

- Desktop: AMD Ryzen 7 7800X3D; MATLAB R2026a CLI `-singleCompThread`, with one
  native solver thread. No estimator, Raspberry Pi, ROS, nonlinear vehicle, or
  network reconfiguration is part of this experiment.
- Reference speed 8 m/s; held input period 0.1 s; sensing range 16 m; exact
  declared affine plant and exact observations; zero process residual and target
  jerk/yaw acceleration. Seed 20260912. Road boundaries and separate state/slip
  bounds are disabled; certified pose/chart domains and actuator/slew bounds
  remain.
- Curvatures `0, +0.01, -0.01, +0.02, -0.02` per metre, each with stationary,
  oncoming and crossing targets. Each measured run has 300 holds (30 seconds).
  The crossing scenario uses 32 m/s Cartesian crossing on the straight and
  4 m/s along the local normal on arcs; these are the existing scenario inputs,
  not equivalent crossing speeds. Stationary targets start at reference station
  15 m; exact definitions are in the scenario driver.
- Exclude 15 warmup runs of 60 holds and 20 replays of the old worst input.
  Measure 30 independent replays, then two complete 15-scenario campaigns.
  Finally run a separate warmed campaign with both search and frame deadlines
  set to 100 ms. No profiler or solver hook runs inside timed campaigns.
- Diagnostic campaigns use 30 s search/frame budgets so a slow attempt is
  observed rather than censored; every measured time is still compared against
  100 ms. Complete scenario-frame timing includes synthetic observation
  construction; the same-input replay timer covers the complete controller call.
- Freeze and verify SHA-256 values for 90 MATLAB source files and 103 existing
  native binaries. The source manifest and raw traces are outside the repository
  at `/home/zai/.cache/collisionAvoidance/bounded-admission-20260919/final`. The production native binaries were not modified.

These are empirical warmed desktop times, not hard real-time guarantees or
Raspberry Pi performance. Excluding cold MATLAB/MEX initialization does not
exclude fresh target-admission frames: those are explicitly included below.

## Complete-frame runtime comparison

Each table cell with a slash is **baseline / new implementation**. Counts can
differ because optimized encounters last for different numbers of holds.

| Frame category | Count | Median (ms) | Maximum (ms) | Frames >100 ms |
| --- | ---: | ---: | ---: | ---: |
| All measured frames | 9000 / 9000 | 4.489 / 4.468 | 4653.899 / 69.822 | 27 / 0 |
| Target admission | 32 / 32 | 655.210 / 32.761 | 4653.899 / 69.822 | 27 / 0 |
| Active encounter continuation | 986 / 996 | 15.816 / 8.362 | 48.790 / 21.170 | 0 / 0 |
| Target-free continuation | 7940 / 7930 | 4.480 / 4.459 | 6.470 / 6.243 | 0 / 0 |

The new campaign maximum occurs in repetition 2,
`crossing`, curvature -0.02 per metre, at
0.0 s. It uses 3 conic calls and
56 prediction holds. Its accumulated formulation and solve
times are 27.979 and
31.769 ms; the complete-frame value also includes other
controller and observation work.

The old worst input is the initial +0.02 per metre crossing admission with 48
holds. All 30/30 new replays pass the independent certificate.
They use 2 solves each. Median accumulated formulation and
solver times are 19.447 and
16.954 ms. The original case required
123 restoration calls across three starts, followed by one performance call.
The first two starts alone spent 4382.948 ms in formulation/solving in the earlier
instrumented replay.

An additional **untimed** solver-hook probe of the new same-input replay records:

| Call | Variables | Rows | SOCs | Records proved safe by the hard pose domain |
| --- | ---: | ---: | ---: | ---: |
| 1 | 559 | 1179 | 48 | 28 |
| 2 | 559 | 1179 | 48 | 28 |

The old restoration problem had 1163 variables, 2453 rows, and 201 SOCs.
Probe timing is not used as a real-time measurement. All original collision and
exit records remain in the final physical verifier.

The separate strict 100 ms campaign completes **15/15** runs at
the node-certificate level and passes **11/15** full scenario checks.
Its largest recorded complete frame is **69.635 ms**. No trial stopped for a controller deadline or admission failure.

## Avoidance and recovery results

The distance audit evaluates exact oriented rectangles at 11 equally spaced
points per held interval (10 ms spacing for this 100 ms hold). Positive sampled
distance is necessary, but does not prove all-time separation. Negative sampled
distance establishes an actual overlap in the simulated affine trajectory.
No 0.25 m margin or other physical clearance threshold is required: the
acceptance criterion is strictly greater than zero.

Results below combine both measured repetitions. Completion means all 300
commands were issued and their **node** certificates verified; it is not a
collision-free label.

| Curvature (1/m) | Target | Node-complete runs | Minimum sampled body gap (m) | Maximum calls/frame | Maximum frame (ms) | Scenario result |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| +0.00 | stationary | 2/2 | -0.110560588 | 1 | 39.306 | **Fail** |
| +0.00 | oncoming | 2/2 | -0.035466548 | 1 | 22.177 | **Fail** |
| +0.00 | crossing | 2/2 | 9.519704958 | 1 | 11.639 | Pass |
| +0.01 | stationary | 2/2 | 0.068748085 | 1 | 33.636 | Pass |
| +0.01 | oncoming | 2/2 | 0.075914732 | 1 | 19.586 | Pass |
| +0.01 | crossing | 2/2 | 0.061342234 | 2 | 43.851 | Pass |
| -0.01 | stationary | 2/2 | 0.068747711 | 1 | 32.902 | Pass |
| -0.01 | oncoming | 2/2 | 0.075914731 | 1 | 19.640 | Pass |
| -0.01 | crossing | 2/2 | -0.024610552 | 3 | 57.087 | **Fail** |
| +0.02 | stationary | 2/2 | 0.199738097 | 1 | 33.067 | Pass |
| +0.02 | oncoming | 2/2 | 0.187731679 | 1 | 20.316 | Pass |
| +0.02 | crossing | 2/2 | 0.196088777 | 2 | 45.593 | Pass |
| -0.02 | stationary | 2/2 | 0.199738096 | 1 | 32.965 | Pass |
| -0.02 | oncoming | 2/2 | 0.187731683 | 1 | 20.444 | Pass |
| -0.02 | crossing | 2/2 | -0.027340898 | 3 | 69.822 | **Fail** |

Failed sampled-clearance cases: stationary at curvature +0.00 m^-1 (-0.110560588 m); oncoming at curvature +0.00 m^-1 (-0.035466548 m); crossing at curvature -0.01 m^-1 (-0.024610552 m); crossing at curvature -0.02 m^-1 (-0.027340898 m).
The baseline passed 13/15 distinct sampled-clearance cases; this implementation
passes 11/15 in both measured repetitions. In particular,
the two negative-curvature crossing collisions are a closed-loop regression
relative to that baseline. They must not be hidden behind successful admission
or passing unit tests. The new optimizer changes the selected trajectories;
the node-only certificate permits inter-node contact in both algorithms.

The measured runs eventually release the target and return to the cruise trim.
Maximum absolute final errors across the 30 runs are lateral position
1.914e-05 m, heading 8.18524e-09 rad, longitudinal speed
1.58843e-07 m/s, lateral speed 6.67915e-08 m/s, and yaw rate
6.36353e-08 rad/s. Recovery after an overlap does not make that run a
successful avoidance.
Restoring a continuous held-interval separation certificate is a distinct
remaining requirement; reducing the control period or silently adding a fixed
clearance buffer is not claimed as its solution here.

## Validation and reproduction

Full repository suite: **703/703 passed**,
0 failed, 0 incomplete. The final suite includes
2,000 seeded global-majorant/touching comparisons beyond the old trust region,
uncertain supports, the blocked-square and large-escape examples, rejection of
positive physical deficits and invalid actuator solutions, domain-wide screening
and unsafe-normal checks, inherited witnesses, deadlines, and earlier-state
rejection. The tests validate their stated behaviors; they do not override the
failed collision-audit results above.

The final tests ran through the MATLAB MCP session, with a saved TestResult MAT
file, diary, and JSON summary. The preceding focused screening suite passed
68/68 checks. Factory-configured `checkcode` on the ten changed MATLAB files
reports no syntax errors and eight performance advisories on preexisting sparse
indexing/concatenation statements. Their exact IDs and locations are retained in
the JSON. `git diff --check` passes.

From the repository root, with the existing native solver installed:

```matlab
results = runtests('tests');
assertSuccess(results);
addpath('scripts');
runBoundedAdmissionBenchmark(OutputDirectory='/absolute/path/to/results');
```

For the exact old-worst-input replay used here, add
`ReplayFixture='/home/zai/.cache/collisionAvoidance/admission-hotspot-20260919/fixture.mat'`.
The committed [benchmark script](../scripts/runBoundedAdmissionBenchmark.m)
performs warmups, measured campaigns and strict deadline runs. Raw experiment
outputs are deliberately outside `scripts/`. The machine-readable
[result](BOUNDED_ADMISSION_20260919.json) includes source/native manifests,
per-scenario outcomes, grouped times, tests, and artifact hashes.

Only controller/configuration, behavior tests, the benchmark driver, and their
research documentation are changed. Native dependencies, generated binaries,
perception code and unrelated files are excluded. A concurrent manuscript commit
`e64d5dd9bf6d476470f7c3395d61fa0821fd60d8` described an earlier transient
single-solve prototype. The paper was preserved and is not claimed to be
synchronized with this final bounded-admission implementation.
