# Controller runtime optimization without algorithm changes

September 17, 2026. Outcome: **partial**. Implementation-only changes make every
cruise frame and the complete S-bend oncoming encounter, including its admission
frame, finish inside the 100 ms strict trial. The S-bend stationary admission
frame drops from 268 ms to 176 ms and still misses the deadline; its residual
cost is the degenerate feasibility-restoration solve.

## Scope

The predictive hard-safety, soft-CLF controller is unchanged as an algorithm:
the same swept held-interval rows, support-family admission search with
search-only restoration, hard trajectory SOCP, terminal cones, soft CLF and
independent certificate. No tolerance, horizon, Taylor order, row, cone,
weight or solver setting changed. Every edit below either reproduces the
previous numbers bitwise or changes only the numerical path through which the
same convex programs are solved.

## Implementation changes

Exact edits (bitwise identical commands and states on the 60-hold S-bend
cruise, stationary and oncoming trials):

- `avoidanceStageQp.build`: duplicate-row detection by two fixed projections
  confirmed with an exact row comparison instead of a dense signature sort; the
  lifted dynamics, restoration charge columns and stage Hessian blocks are
  assembled from triplets in single `sparse` calls instead of `blkdiag` and
  indexed sparse assignment.
- `avoidanceSafetyGeometry.supportNormals`: whole-hold midpoint poses and
  target centers of all cells are evaluated in one batch; the rounded-direction
  deduplication replaces `unique(...,'rows','stable')` on tiny matrices; the
  actuator reach support and kernel-availability checks leave the inner loops.
- `avoidanceSafetyGeometry.build`: target flows are evaluated once per
  encounter over all cell starts; per-cell constants are hoisted.
- `laneGeometry.localPoseFrames`: the certified local pose charts of all cells
  are evaluated in one batch; `localPoseFrame` is the single-column case.
- `ltvBicycleModel`: the exact held flow `expm(h*[A,B,c;0])` is stored with each
  scheduled stage and reused by `finitePredict`; the kernel-availability check
  leaves the stage loop; the reference schedule is looked up once per frame and
  attached to the model (`model.referenceBank`), and the scheduled terminal
  family cache is keyed by the bank stamp.
- `formulateAvoidanceProblem`: the complete constraint matrix is assembled
  sparse from the dense physical rows and the small slack, CLF-cone and
  terminal-cone blocks instead of four dense concatenations; the discharged-key
  string scans in the shift run only when a target was discharged.
- `hardEncounterBarrier`: departure proposals computed in `prepare` are reused
  by the completion rows; `observedExterior` computes only the chart it uses.
- `solveHardCbfClf.restore`: the restoration program reuses the sparse physical
  rows already stored in the program matrix.
- `stateUncertainty.heldDisturbance`: a zero rate returns zero before the
  generator validation.
- Native geometry kernel: `localProjectedRows` no longer multiplies the zero
  input and stage-start blocks and multiplies only plan columns the held cell
  can depend on; `avoidanceProjectedRowsKernelMex` was regenerated with
  `buildAvoidanceGeometryKernel` and passes the native/interpreted parity test.

Numerical-path edit (same converged programs, different interior-point path):

- `solveHardCbfClf` constraint generation starts from the two most binding
  rows of every held-cell family and adds up to sixteen violated rows per
  family per iteration instead of one. The hard trajectory program has a unique
  optimum, so its decision changes only within solver tolerance. The
  search-only restoration program has zero cost on the plan, so its minimizer
  is not unique and the returned restoration plan, hence the selected support
  directions of the first admission, can differ. Over 60 S-bend holds the
  maximum command difference against the previous rule is 1.19e-2 (stationary)
  and 1.0e-5 (oncoming); every issued command passes the unchanged hard
  certificate. Variants with one, four, eight, sixteen or unlimited rows per
  family, and with two or four initial rows, were measured; sixteen with two
  initial rows was fastest on both encounters. Seeding the working set with
  every row violated at the anchor was slower (stationary admission 215 ms
  against 175 ms) and was not adopted.

Startup preparation (scripts only, no controller change):

- `prepareCollisionAvoidanceController` follows each fresh target probe with
  two chained calls on the declared affine successor of its own accepted plan,
  so the carried-witness code paths are compiled before periodic sampling.
- `runSmoothReferenceControllerValidation` probes with the scenario's declared
  target at its first sample inside the initial perception range and chains
  successor probes after its target-free cruise probes. Probe commands are
  discarded and their times are recorded in `preparation`.

## Same-conditions comparison (60 holds, one MATLAB process, one thread)

| Path / scene | Frame | Before (ms) | After (ms) | Native solves |
| --- | --- | ---: | ---: | --- |
| sBend / cruise | first successor (frame 1) | 102.6 | 12.0 | 1 / 1 |
| sBend / stationary | admission (frame 0) | 258.9 | 174.8 | 9 / 5 |
| sBend / stationary | frame 1 | 150.5 | 62.8 | 2 / 2 |
| sBend / stationary | frame 2 | 103.5 | 54.9 | 1 / 1 |
| sBend / stationary | frames 3 to 6 | 62.8 to 74.9 | 48.3 to 52.9 | 1 / 1 |
| sBend / oncoming | admission (frame 26) | 157.0 | 101.5 | 8 / 4 |
| sBend / oncoming | frame 27 | 43.2 | 36.9 | 2 / 2 |

The "after" column of this table predates the in-range probe target; with it the
oncoming admission measured 91.1 ms (diagnostic) and 82.7 ms (strict) in the
300-hold run below. Medians fell from 15.6/26.7/16.0 ms to 10.8/21.9/11.5 ms
for cruise/stationary/oncoming.

## Final 300-hold validation

Same protocol as `SMOOTH_REFERENCE_VALIDATION_20260917.md`: MATLAB R2026a Update
3, one computational thread, 8 m/s reference, 0.1 s holds, exact ego and target
states, no road boundaries, 16 m perception range, 0.25 m clearance, diagnostic
5 s budget and independent strict 100 ms trials.

| Path / scene | Before max frame (ms) | After max frame (ms) | After median (ms) | Before strict | After strict |
| --- | ---: | ---: | ---: | --- | --- |
| sBend / cruise | 74.717 | 18.790 | 10.601 | 300/300 | 300/300, max 12.318 |
| transition / cruise | 16.479 | 11.943 | 10.704 | 300/300 | 300/300, max 13.648 |
| asymmetric / cruise | 20.156 | 11.463 | 10.347 | 300/300 | 300/300, max 11.352 |
| sBend / stationary | 268.157 | 176.055 | 11.119 | 0/300 | 0/300, deadline at frame 0 |
| sBend / oncoming | 151.500 | 91.140 | 11.115 | 26/300 | 300/300, max 82.680 |

All diagnostic trials complete 300/300 holds with every command certified.
Sampled clearance above the 0.25 m margin is 0.244167 m (stationary) and
0.350159 m (oncoming); largest station phase errors are 0.924 m and 0.212 m.
The stationary encounter's closed loop differs slightly from the previous
report because of the restoration tie-break described above; the previous
values were 0.244100 m and 0.885 m.

Stationary diagnostic frames above 50 ms: 5 (176.1, 61.7, 53.8, 51.1,
50.8 ms). Oncoming: 1 (91.1 ms). The strict stationary trial stops at frame 0
when the 100 ms work limit expires inside the admission search; no command is
issued, as the failure policy requires.

## Where the stationary admission time remains

Unprofiled medians on the admission fixture after the changes (48 stages, 96
plan coordinates, 15.2k physical rows):

| Component | Median (ms) |
| --- | ---: |
| Fresh formulation (prediction 12.2, pose charts 2.8, support normals 2.4, geometry build 13.0, terminal rows 0.8, objective and sparse assembly) | 40.9 |
| Reformulation with restored support normals | 31.2 |
| Restoration solve on the initial program (3 native calls, about 46 interior-point iterations each, 19 to 20 ms of Clarabel time per call) | 70.6 |
| Hard solve after restoration (2 native calls, 17 iterations, 5.5 ms Clarabel each) | 20.7 |
| Lifted transcription per program | 8.1 |

The restoration program has zero cost on the plan coordinates, so the
interior-point method needs roughly three times the iterations of the hard
program, and all working-set variants needed three restoration solves. Reducing
it further requires changing the restoration formulation or the declared solver
tolerances, which this task did not do. The oncoming admission (32 stages) has
the same structure at about half the size and fits the deadline once its code
paths are warm; 20 ms of its previous 101.5 ms was first-use compilation.

## Validation

- Focused controller tests during development: 113 passed, 0 failed
  (`admissionSupportSearchTest`, `scheduledReferenceControllerTest`,
  `curvedPoseDomainTest`, `collisionAvoidanceControllerTest`,
  `recursiveSafetyClosureTest`, `bicycleNativeKernelTest`,
  `avoidanceNativeGeometryTest`, `controllerSourceBudgetTest`); native
  geometry parity and geometry tests after kernel regeneration: passed.
- Full working-tree suite after all changes: 659 passed, 0 failed,
  0 incomplete (`runtests('tests')`, MATLAB R2026a Update 3).
- Code Analyzer (`checkcode -id -config=factory`) on the changed sources reports
  only the pre-existing array-growth and sparse-indexing advisories; no finding
  originates in the new code. `git diff --check` passes.
- Bitwise equality of all 60-hold commands and states was verified for the
  exact edits before the working-set rule changed.

## Reproduction

```matlab
addpath('scripts');
runSmoothReferenceControllerValidation( ...
    OutputDirectory='/home/zai/.cache/collisionAvoidance/runtime-optimization-20260917/final', ...
    SampleCount=300, RunStrictTiming=true);
```

Raw MAT/JSON traces, the 60-hold before/after pairs, profiler tables,
component-timing scripts and the working-set variant logs are in
`/home/zai/.cache/collisionAvoidance/runtime-optimization-20260917/`. The
machine-readable summary of the final run is
[CONTROLLER_RUNTIME_OPTIMIZATION_20260917.json](CONTROLLER_RUNTIME_OPTIMIZATION_20260917.json).
Measurements are development observations on a shared workstation, not a
worst-case execution-time bound.
