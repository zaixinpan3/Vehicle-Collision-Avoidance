# Shifted-nominal single-convexification controller

Date: September 18, 2026. Base source: `926ff9a368ce1406d0f4577f5fcdd446bca3fcbc`.
Controller state format: 35. This report accompanies the implementation commit.

## Result

Implemented the requested policy: shift the previous optimized nominal,
compute one support direction per predicted node and target, fix the directions,
and solve one complete convex trajectory problem. Removed sector enumeration,
normal-candidate scoring, restoration, retry/expansion loops, alternate exit
families, and native constraint-generation solves. The terminal continuation,
CLF slack, physical hard verification and failure-without-command behavior remain.

The policy is implemented and 163 focused tests pass. It is **not qualified as
an all-scenario avoidance controller**. In the final 20-case campaign, six
strict trials complete 30 s, thirteen fail admission as infeasible, and one
fails its 100 ms deadline. Only five satisfy the driver's combined completion,
sampled physical-clearance and runtime criterion. The independent relaxed-budget
cruise run completes, giving seven functionally completed scenario configurations.

## Paper interpretation and retained mathematics

Li, Zhang, Guo, Lenzo and Guo, *Real-Time Optimal Trajectory Planning for
Autonomous Driving with Collision Avoidance Using Convex Optimization*,
Automotive Innovation 6, 481--491 (2023),
[Sections 3.1--3.2, equations (9)--(13)](https://doi.org/10.1007/s42154-023-00222-7),
first compute distance-dual geometry and then fix it in a convex trajectory
problem. The local PDF under `reference/` was read alongside the publisher record.

For the rectangle configuration obstacle, the analytic closest-point normal
supplies the outside-distance supporting hyperplane without a geometric SOCP.
The signed-distance normal at overlap/contact is an explicit extension: an
ordinary unsigned distance dual can instead return zero. The paper's equation
(13) includes collision slack. This implementation retains hard collision
constraints and the existing soft CLF, affine bicycle plant and predictive
terminal certificate. It does not reproduce every paper assumption or claim
that the paper proves feasibility of this implementation.

The first frame uses cruise continuation. An unchanged active encounter shifts
its complete accepted prediction and counts down to its absolute exit deadline.
On new-target admission the previous controls are a seed only, extended with
the last stored input once and then trim inputs when the new horizon is longer.
Target-free horizon renewal keeps its existing verified continuation rule.
No avoidance trajectory or common passing side is prescribed.

Each online support query depends only on the nominal rectangles at that node.
Polygon tie-breaking is deterministic but does not enforce temporal coherence.
Exact curved-reference nominal poses are used; the convex rows retain their
certified affine pose maps, uncertainty/remainder charges and hard domains.
The optimizer is called once with all rows. Only exact duplicate-row reduction
and equivalent sparse lifting precede that solve.

## Validation and timing method

- MATLAB R2026a, one computational thread, 0.1 s holds, 8 m/s cruise.
- Curvatures 0, +0.01, -0.01, +0.02 and -0.02 per metre; cruise, stationary,
  oncoming and crossing targets; 300 holds (30 s) requested for each case.
- Exact sensing/held affine plant, zero process residual, no estimator and no
  physical road boundaries; 16 m perception region and 0.25 m requested clearance.
- Seed 20260912; the original scenario driver and target definitions are retained.
  Targets on circular-reference trials move on inertial straight lines.
- Six discarded two-hold stationary +0.01 startup trials precede the matrix.
  Each failed qualification receives an independent 5 s-budget diagnostic run.
  Failed admission still stops immediately; no command is executed from an
  infeasible solution.
- Frame time includes measurement construction and controller execution;
  plant integration and truth-geometry auditing are excluded. No profiler is used.

Command:

```matlab
addpath('scripts');
runCircularArcControllerValidation(OutputDirectory=outputDirectory, ...
    SampleCount=300,Curvatures=[0,.01,-.01,.02,-.02]);
```

Final strict results (maximum includes a failed attempted frame):

| Curvature / m | Scenario | Executed holds | Outcome | Max frame / ms |
| --- | --- | ---: | --- | ---: |
| +0.00 | cruise | 0 | deadline | 113.233 |
| +0.00 | stationary | 300 | complete | 53.344 |
| +0.00 | oncoming | 26 | infeasible | 15.847 |
| +0.00 | crossing | 300 | complete | 10.268 |
| +0.01 | cruise | 300 | complete | 34.265 |
| +0.01 | stationary | 0 | infeasible | 23.948 |
| +0.01 | oncoming | 26 | infeasible | 13.107 |
| +0.01 | crossing | 0 | infeasible | 27.918 |
| -0.01 | cruise | 300 | complete | 19.758 |
| -0.01 | stationary | 0 | infeasible | 25.032 |
| -0.01 | oncoming | 26 | infeasible | 14.324 |
| -0.01 | crossing | 0 | infeasible | 20.222 |
| +0.02 | cruise | 300 | complete | 56.224 |
| +0.02 | stationary | 0 | infeasible | 22.083 |
| +0.02 | oncoming | 26 | infeasible | 14.474 |
| +0.02 | crossing | 0 | infeasible | 30.072 |
| -0.02 | cruise | 300 | complete | 12.964 |
| -0.02 | stationary | 0 | infeasible | 20.054 |
| -0.02 | oncoming | 26 | infeasible | 14.104 |
| -0.02 | crossing | 0 | infeasible | 29.647 |

All 1,930 commands issued in the final strict matrix used one solver call,
zero restoration calls, and passed independent hard verification. This count
includes pre-detection cruise holds of the failed oncoming trials; it does
not count those trials as completed avoidance.

An earlier campaign with the same control law completed seven strict trials
and had a largest observed frame of 91.621 ms. The final campaign observed
113.233 ms on the first straight cruise frame; its independent diagnostic
completed with maximum 23.401 ms. Both campaigns are retained in the JSON.
The startup trials do not fully warm every control path. These observations
therefore do not establish a 100 ms worst-case bound. Comparing a fast failed
admission with the old successful multi-solve search is not a speedup claim
for equivalent delivered behavior. No Raspberry Pi timing was performed.

Completed cruise and straight obstacle runs recover near the path/trim at
30 s. This does not resolve the admission failures. The straight crossing
fixture passes far ahead of the ego and remains a weak avoidance challenge.

## What makes the single convex family fail

`diagnoseShiftedNominalConvexification` constructs four fresh-admission
fixtures and removes constraint groups **offline only**. The oncoming fixture
starts at 18.4 m relative distance and is a representative fresh admission,
not an exact replay of a stored state at 2.6 s. These condensed solves diagnose
feasibility, not online timing. No ablated solution is executed.

| Fixture | Full program | Remove collision | Remove exit | Remove terminal/exit | Remove pose domains |
| --- | --- | --- | --- | --- | --- |
| Straight stationary | solved | solved | solved | solved | solved |
| +0.01 stationary | infeasible | solved | infeasible | infeasible | infeasible |
| +0.01 crossing | infeasible | solved | infeasible | infeasible | infeasible |
| Straight oncoming at 18.4 m | infeasible | solved | infeasible | infeasible | infeasible |

Successful ablations pass a separate check of their remaining conic residuals.
“Infeasible” denotes the native solver's primal-infeasibility status, not an
independent exact Farkas certificate. The evidence locates the conflict in the
fixed collision family together with dynamics/actuator restrictions. It does
not establish unavoidable physical collision.

Concrete nominal normal reversals:

- +0.01 stationary: at prediction 1.8 and 1.9 s the normal dot product is
  -0.999968. The shortest escape changes to nearly the opposite side.
- +0.01 crossing: the 1.8 and 1.9 s normals have dot product -1.
- Straight oncoming: the 1.2 and 1.3 s normals have dot product -1.

A direction is individually valid as a support half-space while the sequence
of directions can be dynamically incompatible. Fixing these directions once
provides no subsequent opportunity to change the family. Straight stationary
happens to produce a consistent sequence and obtains a solution; the policy
is not universally infeasible.

## Safety and recursive-feasibility limits

The implementation retains the existing **node-only** certificate. The straight
stationary run completes, but its independent 10 ms truth audit observes a
minimum clearance margin of -0.062070 m: about 0.187930 m between the bodies,
below the required 0.25 m. No sampled body overlap occurs. This is an inter-node
clearance failure and is counted as failed physical qualification. The sampled
minimum is not a proof of a global continuous-time minimum.

The stored generators, charts, terminal cones and absolute deadline remain.
However, always accepting newly computed nominal directions can exclude the
old shifted feasible witness. `shiftedWitnessContained` reports the current
complete-program check. `inheritedPredictionFamily` and
`inheritedFeasibleFamily` distinguish retained prediction from retained
feasibility. During active encounters `recursiveFeasibilityGuaranteed=false`:
the additional witness-inclusion premise is not proved for all future updates.
Target-free continuation keeps its existing conditional guarantee. The theory
and architecture documents state this scope explicitly.

## Tests, code cleanup and artifacts

163 tests passed, zero failed/incomplete, through the MATLAB MCP interface:
controller/configuration, recursive continuation, node certificates, curved
pose domains, shifted nominal convexification, native geometry, source budget,
constant-curvature references, scheduled references and encounter scenarios.
Tests check exact nominal shift, oracle-derived fixed normals, one solver call
on success and failure, new-target initialization, hard acceptance, persistent
terminal/exit structure and sparse transcription equivalence.

The old admission-search tests were renamed/reworked for the new policy.
Previously solvable overlapping curved and oncoming fixtures now explicitly
exercise failed single-family admission. Geometry-inclusion tests use a clear
outward-offset target so they still test the geometric bounds independently;
this is not counted as successful blocked-path avoidance. The unchanged
20-case scenario matrix above preserves that performance regression visibly.

Code Analyzer found no new syntax errors; existing sparse-indexing and small
block-concatenation advisories remain. The local missing analyzer-settings
warning falls back to defaults. `git diff --check` passes. The 20-core-source
budget remains satisfied. No generated native binary or solver dependency is
committed; the shared generated geometry kernel itself was unchanged.

Reports: this Markdown and `SHIFTED_NOMINAL_SINGLE_SOLVE_20260918.json`.
Drivers: `scripts/runCircularArcControllerValidation.m` and
`scripts/diagnoseShiftedNominalConvexification.m`.
Raw MAT/JSON trajectories, initial/final campaign logs, test logs and diagnostics:
`/home/zai/.cache/collisionAvoidance/shifted-nominal-20260918/`.

The requested algorithm change is complete. Robust active-encounter recursive
feasibility, general admission success, inter-node clearance and guaranteed
100 ms execution remain unresolved under this strict single-family policy.
