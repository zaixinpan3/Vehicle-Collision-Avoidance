# NRMM/VFFM first-admission profile and optimization experiments

Date: September 22, 2026. Production controller revision:
`7ec600053b2da40c77f3060a964b9ad3ea211410`; experiment starting revision:
`bd6096a8dae037e55175b37211944150aa433d4d`.

Geometry, prediction and certificate assembly dominate Gaussian evaluation.
Two external prototypes preserve the complete decisions and final safety
programs on three saved admission fixtures. Together they reduce the
circular-crossing median by 2.449--3.495 ms against bracketing baseline runs.
Production controller/configuration remain unchanged. This is local timing
and equivalence evidence, not general validation or a hard real-time result.
The [measurement and hash manifest](NRMM_VFFM_ADMISSION_PROFILE_20260922.json)
retains clean timing, nested probes, line profiles and all prototype outcomes.

## Protocol and clean timing

Select the slowest saved fresh admission for straight stationary, circular
stationary and circular crossing from the preceding diagnostic campaign.
Those scenario types missed the strict 50 ms deadline; their original
campaign maxima were 51.498, 55.566 and 53.314 ms respectively.

Replay identical prepared inputs and explicit controller state in separate,
serial MATLAB R2026a Update 3 `-singleCompThread` processes. Exclude five
warmup calls per fixture in every mode. No task-controlled MATLAB timing
workloads run concurrently; external host scheduling is uncontrolled.
Inherit the saved configuration: 8 m/s reference speed, exact sensing, zero
declared plant residual, 30 s diagnostic deadline, physical road boundaries
disabled, circular curvature 0.01/m. The circular-crossing target starts at
route station 15 m, normal offset -7.5 m and normal speed 4 m/s. The source
campaign uses seed 20260912; replay introduces no randomness.

Every fixture retains 96 holds of 50 ms, 192 controls, 97 separation records
including the exit record, two fitted references and one final SOCP. Normals
stay fixed inside that optimization; all controls remain optimization variables.
There is no former direction/amplitude grid search or seed-only output.

Measure 63 clean calls, 63 coarse probe calls, 63 additional detailed probe
calls and three line-profiled calls. All complete decisions equal their saved
clean baseline exactly. Five clean prototype/baseline experiments add 315
measured calls, excluding 75 warmup calls and untimed baseline comparisons.
An earlier nine-call row-filter smoke check passed; its timing output was
superseded by the full experiment.

| Clean production replay | Calls | Median | Maximum | Above 50 ms |
| --- | ---: | ---: | ---: | ---: |
| Straight stationary | 21 | 45.532 ms | 61.352 ms | 2 |
| Circular stationary | 21 | 50.618 ms | 56.199 ms | 11 |
| Circular crossing | 21 | 51.829 ms | 54.866 ms | 21 |

These timings cover the controller with prepared inputs. The campaign also
includes measurement construction. Later replays do not reproduce its exact
wall times. Line profiling inflates complete calls to approximately 299--316 ms;
use its call counts and hotspots, not its times for deadline assessment.
Lightweight nested probes also add overhead and are reported separately.

## Detailed circular-crossing attribution

Select the median-total-time call from 21 detailed probe repetitions.
The disjoint rows below sum to **55.965 ms**. This is a new instrumented call,
not a retroactive subdivision of an older 55.566, 59.589 or 64.033 ms sample.

| Region | Time | Work |
| --- | ---: | --- |
| Base formulation | 27.531 ms | Prediction, nominal normals, geometry, terminal/CLF rows, objective, joint records and reference preparation |
| VFFM candidate initialization | 6.146 ms | Shared fit, candidate normals, residuals and ranking |
| Fixed-direction SOCP assembly | 3.823 ms | Lifted dynamics, hard rows, support epigraphs and sparse cones |
| Solver wrapper | 10.079 ms | Reduction, native solve and conversion |
| Three certification calls | 5.531 ms | Nominal witness, optimizer result and final controller check |
| Remaining work | 2.855 ms | Input preparation, orchestration, output/carry construction and reporting |
| **Total** | **55.965 ms** | **Instrumented attribution only** |

The 27.531 ms base formulation has this disjoint decomposition:

| Base subregion | Time |
| --- | ---: |
| 96-step affine prediction | 3.389 ms |
| Road/pose frames | 1.575 ms |
| Nominal separation directions | 2.350 ms |
| Initial geometry construction | 10.056 ms |
| Completion and terminal rows | 0.848 ms |
| CLF rows | 0.474 ms |
| Dense condensed objective | 1.396 ms |
| Joint records and replacement of initial collision rows | 4.556 ms |
| VFFM reference preparation | 1.768 ms |
| Other base assembly | 1.119 ms |

The 10.056 ms geometry construction divides further:

| Geometry subregion | Time |
| --- | ---: |
| Assemble node, target, footprint and pose data | 2.099 ms |
| Geometric-row kernel and input concatenation | 2.525 ms |
| Assemble projection inputs | 0.961 ms |
| Project into control coordinates, including kernel input concatenation | 2.702 ms |
| Labels, local records and final concatenation | 1.721 ms |
| Timer/dispatch remainder | 0.048 ms |

Both geometric kernels already use installed MEX binaries. These measured
regions include their inputs/outputs, not exclusively MATLAB arithmetic.
The subsequent 4.556 ms joint stage contains 2.360 ms creating records,
0.287 ms for arithmetic reserves, 1.834 ms deleting superseded rows and
0.075 ms other work. For 96 cells, deletion touches eight local fields per
cell: 768 dynamic-field slices, plus global matrix and label slices.

Reference preparation's 1.768 ms includes nominal collision evaluation,
conflict selection, target propagation and chart preparation. The actual
`vffmReference` evaluation costs **0.419 ms**, already inside that total.
Two nominal target propagations total 0.121 ms and two road-chart evaluations
0.142 ms; these nested measurements must not be added as independent costs.

Candidate initialization's 6.146 ms contains:

- Shared fit of both references: **1.301 ms**, comprising 0.899 ms assembly,
  0.320 ms linear solution and 0.082 ms terminal correction/other work.
  The 192-by-192 system shares the two candidate right-hand sides and terminal
  correction directions. Removing one candidate would not halve this cost.
- Candidate normals: **3.082 ms** for two passes over 97 records, or 194
  analytic configuration-obstacle evaluations. These are geometric operations,
  not additional trajectory optimization solves.
- Residual comparison, physical ranking, state reconstruction and remaining
  initializer work: **1.763 ms**.

Reference preparation plus candidate initialization is 7.914 ms, about 14%
of this instrumented frame. Gaussian-function optimization alone cannot
remove the larger formulation expense.

The native solver takes 9.792 ms wall time and reports 9.746087 ms internally:
13 iterations, 868 lifted variables, 1,829 rows, 10,588 matrix nonzeros,
576 equalities and 1,133 linear cone rows. Other cones encode the CLF and
convex support bounds. No internal KKT/line-search timers are exposed;
this study does not invent a finer solver breakdown.

## Repeated work

`avoidanceSafetyGeometry.build` initially projects fixed-normal collision
rows into the 192-control space. `jointProgram` then deletes collision/exit
slices while keeping actuator, chart, terminal and other constraints. The
final SOCP rebuilds the required support inequalities from retained joint
records. Projecting representations that will be discarded is avoidable;
the joint records and final hard collision inequalities remain necessary.

The nominal trajectory supplies initial normals. Both fitted trajectories
then receive a 97-direction pass. The selected normals define a complete
constrained optimization; the fitted curve itself is never issued. This
explains the extra first-admission work compared with continuation using
carried feasible trajectories and directions.

Three `certify` calls cost 1.399, 2.131 and 2.001 ms: nominal witness,
optimizer result and final controller result respectively. The nominal
witness check rejects this unsafe witness. Seven `jointResidual` evaluations
across preparation, ranking, verification and reporting total 4.222 ms.
That inclusive total overlaps the tables above and must not be added again.
The final reporting residual alone costs 0.605 ms.

Successful verification repeatedly searches stage/target indices and rewrites
normal/completion metadata. The line profiler confirms 192 stage lookups in
the two successful verification calls. Indexed updates and forwarding a
valid existing certificate may avoid repeated work, provided the decision,
normals, bounds and arithmetic allowances have not changed. Removing the
independent verifier wholesale would change the safety contract.

## Bounded optimization experiments

Two hypotheses are tested using source copies outside the repository:

1. **Early row filtering:** retain node/target/normal data and all joint
   records, but remove collision rows after the geometric kernel and before
   control-coordinate projection/local-label assembly. The final joint SOCP
   retains exactly the existing collision constraints. This still pays for
   the initial kernel, testing only part of the possible saving. The prototype
   asserts node-only cells; it is not a general geometry replacement.
2. **Dominated-candidate screening:** evaluate existing physical base-row
   feasibility before computing its 97 normals. Once a finite-scored physical
   candidate exists, a later physically infeasible candidate cannot win the
   current lexicographic ranking and can be skipped. Candidate order and tie
   rules remain unchanged. Circular-crossing physical excesses are
   -0.0183552561 and +0.0186079701 in the existing mixed-row residual units,
   not distances in meters. Both stationary candidates are physically
   feasible, so neither is eliminated there.

Every measured call retains one native solve and independent certification.
Require exact equality of complete decisions, all planned controls, predicted
states and final program fields: `A`, `b`, `P`, `q`, cones, physical matrix,
physical/safety bounds, labels, joint certificate, completion and geometry.
All 315 measured calls pass. Skipped diagnostic score entries may remain
unevaluated; the winning candidate and final safety program are unchanged.

Baseline runs bracket the variants to expose process/host variation.
Each cell gives median / maximum in milliseconds over 21 calls.

| Execution order | Straight stationary | Circular stationary | Circular crossing |
| --- | ---: | ---: | ---: |
| Baseline before | 48.677 / 53.353 | 49.997 / 61.743 | 48.992 / 52.745 |
| Early row filtering | 46.797 / 49.607 | 48.099 / 50.196 | 47.804 / 50.364 |
| Dominated-candidate screening | 45.585 / 55.851 | 48.224 / 51.156 | 47.493 / 50.013 |
| Combined | 46.258 / 50.436 | 47.426 / 51.717 | 45.497 / 48.095 |
| Baseline after | 47.036 / 54.241 | 48.251 / 52.296 | 47.946 / 51.725 |

Combined circular-crossing median improvement is **2.449--3.495 ms (5.1--7.1%)**.
Its 21 calls remain below 50 ms; combined straight stationary has one overrun
and circular stationary two. Individual variant differences cannot be summed.
In particular, stationary changes under candidate screening do not demonstrate
saved normal evaluations: both candidates remain. Baseline drift prevents a
precise causal estimate from these serial batches. No prototype closed-loop
campaign or full regression is claimed.

## Implementation priorities

| Priority | Opportunity | Evidence and limit |
| --- | --- | --- |
| First | Avoid construction/projection of replaced collision rows; specialize initial node-data assembly | Geometry plus reorganization costs 14.612 ms. Partial filtering preserves final programs on these fixtures; general callers and uncertainty need regression. |
| First | Screen physically dominated candidates before normal calculation | Existing selection rule permits avoiding one 97-normal pass and one residual pass in crossing; no candidate-elimination saving in the two stationary fixtures. |
| Next | Reuse certified outputs/residuals and precompute indices | Final verification is 2.001 ms and reporting residual 0.605 ms here. These costs are candidate regions, not guaranteed additive savings. Invalidate reuse on all relevant changes. |
| Next | Batch rectangle normals/supports and replace per-record struct/string operations with numeric arrays | Nominal normals cost 2.350 ms, candidate normals 3.082 ms and joint-record creation 2.360 ms. Preserve overlap, tie and uncertainty behavior. Not benchmarked. |
| Later | Reuse structural prediction/sparse assembly patterns; avoid redundant condensed objectives | Prediction 3.389 ms, condensed objective 1.396 ms and lifted assembly 3.823 ms; refresh all state-dependent coefficients. Not benchmarked. |
| Lower | Gaussian-function or shared-fit optimization | Only 0.419 and 1.301 ms. Even hypothetical complete elimination is small relative to geometry expense. |

One-side initialization, shorter horizons, coarser safety nodes and blocked
controls would change available maneuvers or certificate scope. Neither tested
improvement needs those concessions. The current two-side fit has little
search breadth compared with the removed direction/amplitude grid. These
observations do not support a promise of 30 ms or uniform 50 ms admission.

## Reproduction and validation

Raw artifacts: `/home/zai/.cache/collisionAvoidance/nrmm-vffm-profile-20260922/`.
Source campaign: neighboring `nrmm-vffm-validation-20260922/`.

- `buildMatlabControllerProbes.py RAW` creates external diagnostic copies.
  Run `profileMatlabControllerRuntime` in separate fresh MATLAB processes
  with `Selection='initialization'`, saved `CampaignDirectory`, `Warmups=5`,
  `Repetitions=21` and modes `replay`, `profile`, `probes`. A second `RAW/deep`
  probe run adds geometry timers, reusing saved fixtures. Manifests pin each
  generation; the final builder includes the later detailed timers.
- `buildAdmissionProfilePrototypes.py RAW/prototypes` creates variants.
  Run `benchmarkAdmissionProfilePrototype` in fresh single-thread processes
  for base, early-row-filter, dominated-candidate, combined and base, with
  distinct output names for the bracketing baselines.
- `analyzeMatlabControllerProfile.py RAW OUTPUT` separates timing populations,
  verifies original/instrumented/prototype hashes and retains all outcomes.
- MATLAB Code Analyzer reports no issues in the new benchmark runner. Three
  pre-existing dynamic-growth informational messages remain in the profiling
  driver's standard fixture-selection path. Python compilation, manifest
  verification, exact replay checks and whitespace checks pass. No production
  source/configuration or native binary changes occur. The preceding
  792-test regression is baseline evidence and was not rerun for this task.

These three fixtures do not validate turning/accelerating targets, multiple
obstacles, uncertainty or finite physical road boundaries. Earlier simulation
review limitations remain applicable. Optimization adoption requires broader
behavior tests and closed-loop validation.
