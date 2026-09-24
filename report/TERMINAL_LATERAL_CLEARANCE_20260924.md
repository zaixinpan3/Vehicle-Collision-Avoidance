# Road boundaries in the terminal set: declared lateral clearance, 2026-09-24

Until this change the controller rejected every road geometry with finite
boundaries at the first frame ("The recursive cruise certificate requires an
unbounded road-free reference domain"), because the modal terminal set was
sized by actuator limits and measurement noise alone and could extend beyond
the road. A terminal set that is only a recursive-feasibility device still has
to lie inside the state constraints, otherwise the shifted plan's final
feedback step is not feasible for the next frame's node road rows. This
change shrinks the terminal set by a declared lateral clearance and keeps the
finite fitted boundaries as node-row sensor products. Measured outcomes only.

## 1. What changed

- **Input contract** (`controller/readPlanningInputs.m`). A road geometry may
  declare `lateralClearance = [right; left]` (metres from the nominal path to
  the physical road edges, valid along the whole continued reference) and an
  optional `lateralClearanceErrorBound`. Finite `boundaries` without the
  declaration are rejected with `missingLateralClearance`.
- **Terminal set** (`controller/hardEncounterBarrier.m`,
  `localLateralClearanceRows`). Four rows on the true tracking error,
  `±y ± L psi <= d − W − delta_k − epsilon_d`, join the actuator rows of both
  terminal families (the constant-curvature modal set and the scheduled
  phase family). `L`, `W` are the half length and half width; `delta_k` is
  the exact corner allowance on a reference of curvature `k`,
  `|k|(L^2+W^2)/(2(1−|k| max d))`, derived from the corner's Frenet lateral
  lying in `[v − u^2|k|/(2(1−|k|v)), v]`. The rows bound the true error, so
  they carry no measurement-noise term. The derivation is in
  `controller/TERMINAL_CBF_PROOF.md`, "Declared lateral clearance".
- **Refitted boundaries** (`hardEncounterBarrier.prepare`). Perception-fitted
  boundaries change every frame, which changed the execution identity and
  raised `changedExecutionContract`. A frame whose boundaries differ while
  the declared clearance and every other identity component are equal is now
  admitted afresh, seeded by the shifted previous plan
  (`metadata.readmittedAfterRoadRefit`), because the inherited family is
  never relinearized against new road rows. A changed clearance still
  rejects the state.
- **Harness.** `scripts/runExactStateRecursiveFeasibilityScenario.m` declares
  `[5; 5]` with its ±5 m boundaries; `scripts/fitPerceivedRoadBoundaries.m`
  declares the outer curb offsets (lane offset plus shoulder) alongside the
  fitted curbs.
- **Tests.** `tests/terminalLateralClearanceTest.m` (8 tests): the declared
  clearance shrinks the straight set and every sampled boundary point of the
  set keeps all four footprint corners inside the clearance (exact corner
  geometry, straight and 0.01 /m references); the error bound tightens the
  rows; a road narrower than the footprint, boundaries without a declaration
  and a nonpositive clearance are rejected with their identifiers; refitted
  boundaries are re-admitted while a changed clearance is rejected; a bounded
  straight cruise from 2.5 m lateral error completes 12 holds without leaving
  the road. `tests/straightRoadBoundaryConfigurationTest.m` now expects a
  3.9 m lateral start on the 5 m road to be certified (road margin 0.089 m)
  and a 4.2 m start (footprint over the edge) to be rejected.

**Environment.** MATLAB R2026a, `matlab -batch`, a clean detached worktree of
`fd85ec9` with `solver/` linked; two or three MATLAB processes at a time plus
other projects' sessions, so frame times are loaded measurements.

## 2. Declared plant with ±5 m road boundaries: 4 of 4 complete

`runExactStateRecursiveFeasibilityScenario(Scenario=..., SampleCount=240,
DeadlineSeconds=Inf, SearchTimeLimitSeconds=30, UseRoadBoundaries=true)`, the
four cases that the failure-mode sweep lists as group E-road (cases 101-104),
all rejected at frame 0 in every earlier record.

| Case | Holds | Min body gap | Min road margin | Min terminal margin | Max CLF residual | Inherited frames | Max frame |
| --- | --- | --- | --- | --- | --- | --- | --- |
| stationary | 240 / 240 | 9.22e-05 m | 1.669 m | 0.0746 | -8.87e-06 | 83 | 968.7 ms |
| oncoming | 240 / 240 | 0.0381 m | 0.764 m | 1.6e-05 | -8.88e-06 | 46 | 90.5 ms |
| crossing | 240 / 240 | 9.520 m | 4.050 m | 0.1126 | -4.29e-06 | 12 | 24.1 ms |
| cruise | 240 / 240 | no target | 4.050 m | 0.1126 | -1.01e-05 | 0 | 25.1 ms |

Every case passes, collision-free, with the sampled footprint inside the road
at every hold and the CLF decrease satisfied. The stationary and oncoming
cases inherit certificates across frames with the fixed boundaries, so the
recursion with road rows is exercised, not only fresh admissions. The
oncoming terminal margin of 1.6e-05 shows the terminal-entry row binding at
the swerve's end.

## 3. Regressions

### 3.1 Recursion campaign (no road): identical

`runRecursiveSafetyValidation` at 600 holds: body gaps 9.273e-05, 0.03806,
9.5197, no target, 9.5208 m; terminal margins 0.1444, 0.0222, 0.1823, 0.1824,
0.1782; CLF residuals -5.63e-06, -5.63e-06, -5.76e-06, -6.48e-06, -1.03e-05.
All values equal the `3073d57` record; without a declared clearance the
terminal synthesis receives no extra row. The 50 ms deadline runs complete
0 / 120, 52 / 120 and 120 / 120 holds, as on 2026-09-22.

### 3.2 Failure-mode sweep

`runDeclaredPlantFailureSweep` (240 holds, deadline disabled), compared row by
row with the `3073d57` reference sweep
([declaredPlantFailureSweep.csv](TERMINAL_LATERAL_CLEARANCE_20260924/declaredPlantFailureSweep.csv)):

| | Reference (`3073d57`) | This change |
| --- | --- | --- |
| passing cases | 103 / 118 | 107 / 118 |
| controller errors | 6 (48 and 54 solver-infeasible; 101-104 road rejected) | 2 (48 and 54) |
| rows that differ | | 4: cases 101-104 only |

The four E-road cases move from "rejected at frame 0" to "completed, passed";
the other 114 rows are identical in outcome, holds and gaps (the two
solver-infeasible cases 48 and 54 and the nine inter-node overlaps 83, 84,
86, 87, 89, 90, 91, 96 and 115 are unchanged; case 41 passes in both, as in
the reference, since this worktree carries no uncommitted controller edits).

### 3.3 Tests

`runtests('tests')` on the worktree: 819 tests, 817 passed, 0 failed,
2 incomplete (the two curb-detection tests filtered by assumption because the
untracked LiDAR dataset is absent from the worktree). The count rises from
810 by the 8 new tests and the split of one road-configuration test into two.

## 4. PassVeh14DOF scenarios with road boundaries

Run in the shared working tree after merging `e81d56e`, i.e. with the
concurrent session's uncommitted edits (which the 14-DOF runs need to pass
the second sample; `report/CONTROLLER_SCENARIO_RERUN_20260924.md`). The
harness gained a `RoadPerceptionRange` option (curb-fit range, default equal
to `PerceptionRange`) so that a map-known curb can be declared farther than
the target sensor sees; `runCircularCenterlineCruiseScenario` gained the road
options it lacked. Logs: `passveh_*.txt` in the artifact folder.

| Scenario | Curb range | Outcome |
| --- | --- | --- |
| straight cruise, curbs at 8.6 / 10.6 m | 30 m | **PASS**, 80 / 80; min road margin 7.52 m; 79 of 79 later frames re-admitted after the curb refit |
| circular cruise R = 100 m, curbs | 100 m | **PASS**, 80 / 80; max lateral error 0.0057 m |
| straight cruise, no road (regression) | - | **PASS**, 80 / 80 |
| oncoming avoidance (default) | 30 m | `roadBoundaryCoverageGap` at t = 0.70 s, the first target frame (14 frames) |
| oncoming avoidance | 60 m | `roadBoundaryCoverageGap` at t = 0 (the target is visible at 50 m) |
| oncoming avoidance | 150 m | infeasible SOCP at t = 1.45 s (29 frames; horizon 16, then 56, 48, 40 holds after detection; min lateral -0.355 m; SAT margin 1.449 m at failure) |
| circular straight-target avoidance | 30 m | `roadBoundaryCoverageGap` at t = 0.70 s |
| circular straight-target avoidance | 100 m | `roadBoundaryCoverageGap` at t = 0.70 s; the single-chart quadratic fits cover [-53, 91] and [-43, 80] m with fit error bounds 3.5 and 4.4 m |
| estimator-in-the-loop, straight | 150 m | 71 steps, then `roadBoundaryCoverageGap` at the first radar frame (the admission cell needs [-77, 151] m of the ±150 m fit) |
| estimator-in-the-loop, circular R = 60 m | 150 m | `invalidRoadBoundary` at t = 0: the 150 m quadratic fit of a 60 m arc crosses the centerline |

Both cruise scenarios on the 14-DOF plant now complete with physical curbs;
these are the first road-bounded PassVeh14DOF runs to complete. The
avoidance scenarios are no longer rejected by the terminal set; what stops
them is elsewhere:

- **Coverage of the finite curb fit.** When a target is admitted, the
  certified cells extend over the whole encounter (up to four horizons), so
  a perception-limited curb fit shorter than that raises the node-row
  coverage gap. On the straight road a 150 m map-known fit suffices for the
  truth-target run; with the estimator's first-detection bounds (speed
  ±30 m/s, course unbounded) the cells span about ±150 m and no finite fit
  covers them. This is the first-detection-bound problem of
  `CONTROLLER_SCENARIO_RERUN_20260924.md`, Section 4.3, seen through the road
  rows.
- **Single-chart quadratic curbs on curved roads.** `fitPerceivedRoadBoundaries`
  fits one quadratic per curb in one frame; over 100 m of a 100 m radius arc
  the fit error bound is 3.5-4.4 m and the covered range is short, and over
  150 m of a 60 m arc the fit is invalid. Covering a curved encounter needs
  piecewise charts, which is a perception-model change outside this task.
- **The close-encounter infeasibility** of the straight truth-target run
  (t = 1.45 s with curbs, 1.55 s without) is unchanged by this task.

## 5. Perception-limited boundaries constrain only the cells they cover

Follow-up decision by the user: beyond the perceived range the road is
unknown and therefore unconstrained; the controller must not reject the frame.
`avoidanceSafetyGeometry` now reads the boundary's `coveragePolicy`: a
`strict` boundary (the input default) must still cover every certified cell
and raises `roadBoundaryCoverageGap` otherwise; a `perceptionLimited` or
`knownNominalPathOffset` boundary adds no row for a cell whose station range
leaves its declared parameter range. The declared clearance keeps bounding
the terminal set. The generated geometry kernel carries a `coverageRequired`
flag and was rebuilt (`buildAvoidanceGeometryKernel`); the native/MATLAB
equivalence tests pass. `runExactStateRecursiveFeasibilityScenario` gained
`RoadCoveragePolicy` and `RoadBoundaryParameterRange`;
`straightRoadBoundaryConfigurationTest` checks that a strict 12 m boundary is
rejected while a perception-limited one of the same length completes and
issues fewer road rows than a full-length boundary.

PassVeh14DOF, default 30 m curb range, shared working tree:

| Scenario | Outcome |
| --- | --- |
| straight cruise, curbs | **PASS** 80 / 80 |
| circular cruise R = 100 m, curbs | **PASS** 80 / 80 (previously needed a 100 m fit) |
| oncoming avoidance (default) | 29 frames; infeasible SOCP at t = 1.45 s; min road margin 7.02 m; horizon 16 to 48 to 40 holds |
| circular straight-target avoidance (default) | 32 frames; infeasible SOCP at t = 1.60 s; min road margin 6.34 m |
| estimator-in-the-loop, straight | 71 steps; infeasible SOCP at the first radar frame (t = 3.55 s); min road margin 7.56 m |
| estimator-in-the-loop, circular R = 60 m | 69 steps; infeasible SOCP at the first radar frame (t = 3.45 s); min road margin 6.63 m |

No run raises a coverage gap or an invalid-boundary error any more. Every
road-bounded run now ends exactly where its road-free counterpart ends (the
close-encounter infeasibility at 1.45-1.60 s with truth targets, the
first-detection infeasibility with the estimator), with the ego inside the
road throughout. Road boundaries are therefore no longer a blocker for any
scenario in `CLAUDE.md`.

Declared-plant regression: the sweep is identical to Section 3.2 in every row
(107 / 118, [declaredPlantFailureSweep_coverage.csv](TERMINAL_LATERAL_CLEARANCE_20260924/declaredPlantFailureSweep_coverage.csv), 0 of 118 rows differ); the strict boundaries of the E-road cases cover their
cells. Test suite: 821 tests, 819 passed, 0 failed, 2 incomplete (the curb-detection data skips).

## 6. Scope

- The clearance is a declaration about the road along the continued
  reference, like the reference itself. It is not derived from the finite
  fitted boundaries, which are never extrapolated.
- The rows are node-sampled, as the node road rows are; the road between
  hold nodes is not certified.
- The curvature allowance is charged to both sides; the inner side would not
  need it.
- Re-admission after a boundary refit is a fresh admission and therefore
  subject to the same infeasibility as any fresh frame; with per-frame
  refits no certificate is inherited.
- A perception-limited boundary leaves the cells beyond its range without a
  road row. A plan may therefore end outside the perceived road; the next
  frame's refit constrains those cells once they are seen. The declared
  clearance bounds only the terminal set.
