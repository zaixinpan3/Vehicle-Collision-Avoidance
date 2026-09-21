# Full trajectory optimization after fixing separation directions

Date: September 21, 2026. Parent implementation:
`8c67a7b4d73576b63a4ebcbafbc2edbe9e4076e7`.
Machine-readable results and technical artifact hashes:
[validation JSON](FIXED_DIRECTION_CONVEX_OPTIMIZATION_20260921.json).

## Implemented behavior

Direction search now supplies only an initialization. Every new active
admission fixes the selected collision and exit normals, solves one convex
problem for the **complete control sequence**, and independently verifies the
result before issuing its first command. A safe scalar candidate cannot bypass
the full solve; failed fresh optimization issues no command. Inherited frames
fix their retained normals and attempt full trajectory optimization. After a
failed improvement, only a previously optimized, independently verified suffix
can be retained. All measured campaign solves succeed without such retention.

The scalar initializer still uses 16 directions, 8 amplitude cells, 32 objective
iterations and a curved temporal shoulder fraction of 0.8. These settings limit
initialization exploration. The final optimization has no scalar-line equality
and can alter steering/braking timing across the whole horizon. Its fixed
normals still select an inner subset of the nonconvex avoidance problem;
admission completeness is not established.

The conic builder removes normal-angle decision columns, normalization cones,
target support epigraphs and the position-direction bilinear majorant. It
computes target shape and position-uncertainty supports as constants. Ego yaw
remains a trajectory variable. Its support is bounded by a global touching
majorant using the rotation remainder

\[
\|R(-\Delta\psi)e-(e-Je\Delta\psi)\|_2
\le \tfrac12(\Delta\psi)^2,\qquad \|e\|_2=1.
\]

Multiplying this remainder by the ego circumradius bounds the support error.
The resulting convex yaw term needs one SOC. The full derivation, uncertain
rectangle support and hard pose-domain screening are documented in
[the formulation](../controller/JOINT_SUPPORT_CERTIFICATES.md).
`jointCertificate.positionScale` is removed. Certificate format 41 rejects
older saved certificates. The public controller interface is unchanged.

## Relationship to the paper

Li, Zhang, Guo, Lenzo and Guo, *Real-Time Optimal Trajectory Planning for
Autonomous Driving with Collision Avoidance Using Convex Optimization*,
Automotive Innovation 6, 481–491 (2023),
[publisher](https://link.springer.com/article/10.1007/s42154-023-00222-7):
Section 3 Eq. (12) computes distance-dual variables; Section 3.2 Eq. (13) fixes
them for the full trajectory stage. The original local PDF was read directly.
The implemented separation of stages follows that principle. The project
retains uncertain rectangular footprints, the yaw majorant, terminal/CLF cones
and hard collision acceptance. It therefore solves an SOCP rather than
literally reproducing the paper's QP with collision slack.

The preceding direct-scalar branch and joint trajectory/normal branch no
longer define current behavior. Historical reports retain their original
measurements and algorithm descriptions.

## Evidence that the final plan is fully optimized

Two 96-hold admission fixtures were repeated with exactly unchanged inputs.
Every accepted call makes one native solve. The output normal arrays exactly
match the selected arrays. The optimized control vector leaves the scalar
initializer line by a nonzero orthogonal projection norm:

| Fixture | Control-line departure norm | Seed objective | Optimized objective | Conic variables / rows |
| --- | ---: | ---: | ---: | ---: |
| Straight stationary | 0.644651 | 12.460243 | 9.473596 | 1057 / 1751 |
| Circular crossing | 1.180034 | -36.378482 | -43.942425 | 874 / 1845 |

These norms use the unscaled vector of mixed actuator coordinates and are
structural diagnostics, not distances in meters. Objective values use the
canonical condensed quadratic without its additive constant. Repeated full
decisions are identical within each measurement block. Original physical and
support checks pass independently of solver epigraphs.

## Warmed timing, including outliers

MATLAB R2026a Update 3 on the existing Ryzen 7 7800X3D workstation, using
`matlab -singleCompThread`. No other task-generated MATLAB workload ran
concurrently with the timing campaign. All offline audits are outside timing.
Two replay blocks per fixture each exclude five warmups and retain eleven
measured full calls: 44 measured calls and 20 excluded warmups in total.
These are diagnostic-budget calls, so an over-50-ms sample remains observable.

| Admission replay | Calls | Median (ms) | Maximum (ms) | Over 50 ms |
| --- | ---: | ---: | ---: | ---: |
| Straight stationary | 22 | 42.2365 | 51.142 | 1 |
| Circular crossing | 22 | 46.9885 | 55.598 | 2 |

The 55.598 ms circular maximum contains 0.905 ms input preparation,
34.352 ms formulation/direction initialization, 11.711 ms solver-stage work,
and 8.630 ms remaining verification/output/orchestration work. The solver
stage includes wrapper overhead; it is not a pure native solver-core time.
These disjoint metadata regions sum to the measured full call. They do not
provide a new instrumented breakdown inside initialization.

The earlier circular maximum of 64.033 ms and the direct-scalar variant's
33.421 ms came from different implementations and measurement sessions.
They are historical context, not a new paired speedup experiment. Mandatory
full optimization adds work relative to issuing the initializer directly.
**Three warmed replay outliers remain; a hard 50 ms guarantee is not established.**

## Closed-loop and footprint validation

Twenty-four 600-hold trials combine six cases, two repetitions, and diagnostic
30 s / strict 50 ms frame budgets. Each case first has two 140-hold warmups:
1680 warmup frames are excluded. All **14,400 measured frames** complete and
pass hard verification; there are no rejected attempts or retained-incumbent
frames. Each budget population contains 12 admissions, 694 active
continuations and 6494 cruise frames.

| Frame population | Diagnostic maximum (ms) | Strict maximum (ms) |
| --- | ---: | ---: |
| Admission | 49.198 | 48.965 |
| Active continuation | 28.575 | 26.067 |
| Cruise | 14.295 | 10.501 |

Parameters: 50 ms holds, 1.6 s performance window, finite encounter extension
(96 holds at initial circular crossing), reference speed 8 m/s, seed 20260912,
exact sensing and the declared held affine bicycle plant. Curvature is 0 or
0.01 per meter (radius 100 m); the crossing target speed is 4 m/s. Raw reports
retain all target geometry, motion, uncertainty and configuration fields.

Independent audit reconstructs saved affine endpoints exactly and matches
5 ms body-gap minima within 1e-9. Independent separating-axis overlap signs
agree. Near-contact holds, minimum-gap holds and their neighbors are refined
at 0.1 ms. Refined Euclidean body gaps are:

| Road | Target | Minimum refined body gap (m) | Maximum closed-loop frame across four trials (ms) |
| --- | --- | ---: | ---: |
| Straight | Stationary | 0.009805352 | 45.719 |
| Straight | Oncoming | 0.480284442 | 29.121 |
| Straight | Crossing | 9.519482818 | 19.839 |
| Circular | Stationary | 0.184172469 | 48.569 |
| Circular | Oncoming | 0.229180166 | 30.878 |
| Circular | Crossing | 0.323076195 | 49.198 |

Cruise counterfactuals collide in five cases; straight crossing has no nominal
collision threat and is not evidence of an avoidance maneuver. The previously
observed straight stationary/oncoming inter-node overlaps are absent in this
finite rerun. The online guarantee still covers hold nodes only, and finite
refinement does not prove continuous-time safety.

A separate untimed sensitivity check varies mirrored curvature, ego speed,
target speed, station and lateral offset over 30 initial conditions. All
28 active-target cases and two inactive cases certify. Every active case
performs full trajectory optimization with unchanged selected normals.

Physical road boundaries and optional state/slip limits are disabled in these
scenarios. Optional model-domain diagnostic minima are negative in five cases,
including -0.113860482 in circular crossing; enforced pose-domain and original
hard rows pass. Thus these results concern the declared affine plant and do
not validate nonlinear tire dynamics or unconstrained physical road use.

## Regression and generated code

The generated prepared-frame MEX was rebuilt. Optimized admission status 4
matches the MATLAB decision within 5.6145199600621254e-11; optimized
continuation status 2 matches exactly. Both pass the original verifier. This
checks the numerical adapter, not a complete standalone sensing/estimation/
controller pipeline. Status 1 direct-scalar issuance is removed.

Focused validation first passed 105 tests. The complete suite then passed
769 of 773: four parameterizations of an older test still expected a fresh,
physically safe initializer to be issued after solver rejection. That test now
first obtains an optimized plan and checks retention of its verified suffix.
Four added cases separately require rejection of fresh safe seeds when the
mandatory solver fails. The corrected controller class passes 51/51 tests;
the complete suite plus this affected-class rerun has **777 unique current
passing tests**, with no failed or incomplete cases. The initial failure and
rerun artifacts are both preserved. No production algorithm changed after
timing measurements or to resolve these expectation changes.

Behavioral checks cover fixed output directions, full input-sequence freedom,
unsafe positive solver results, failed/expired admission, old-format rejection,
uncertain supports, touching majorants and inherited feasibility. Factory
Code Analyzer on 18 touched MATLAB files reports nine performance advisories
(two array-growth, six sparse-indexing and one logical-indexing suggestion),
with no other messages. They are retained in the JSON. Python syntax checks,
analysis assertions, diagnostic probe construction, native capture insertion
points, local report links and `git diff --check` pass.

## Reproduction and artifacts

Use the repository root. The replay driver imports the two original fixtures
from `/home/zai/.cache/collisionAvoidance/longest-frame-profile-20260921`.
It removes only the retired `positionScale` setting and applies the current
initializer defaults. Run timing workloads sequentially:

```matlab
addpath('scripts');
directory = '/home/zai/.cache/collisionAvoidance/fixed-direction-20260921';
runFixedDirectionValidation(directory, Mode='replay');
runFixedDirectionValidation(directory, Mode='sensitivity');
runFixedDirectionValidation(directory, Mode='campaign');
```

Full-suite invocation: `results = runtests('tests'); assertSuccess(results)`.
The external `verifyFixedDirections.m`, `auditFixedDirectionCampaign.m`,
`runFullRegression.m` and `finalizeFixedDirectionTests.m` preserve the generated
check, independent footprint method and original/rerun test results. Raw MAT,
JSON, logs and generated binaries remain in the external cache; the committed
JSON lists their hashes. The core source budget remains 20, and no solver
dependency, generated binary or unrelated user file is included in the commit.

```bash
python scripts/analyzeFixedDirectionValidation.py \
  /home/zai/.cache/collisionAvoidance/fixed-direction-20260921 \
  report/FIXED_DIRECTION_CONVEX_OPTIMIZATION_20260921.json
```
