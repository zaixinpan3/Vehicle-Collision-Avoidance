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

Completed in the follow-up commit after merging this change into the shared
working tree, whose concurrent-session edits the PassVeh14DOF runs need to
pass the second sample (see the section of the same name below if present).

## 5. Scope

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
