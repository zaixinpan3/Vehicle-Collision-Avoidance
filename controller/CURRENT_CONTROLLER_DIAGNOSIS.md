# Current uncertainty, collision geometry, road updates and runtime

September 10, 2026. Controller baseline: commit
`665c98b7f99bd4c262cc7a937e64dd8fd959f715`, certificate version 15.
This follow-up explains mechanisms and measures runtime; it does not change
controller policy, residual bounds, solver settings or the physical plant.

## Uncertainty propagation

For the retained scheduled affine inclusion, the error obeys
`eDot = A(t)*e + w`, with bounded residual `w`. Future velocity error integrates
into position; yaw-rate error integrates into heading and affects lateral
position. Even an exact initial measurement does not identify all future
residuals. A bounded residual is a set of allowed disturbances, not a Gaussian
standard deviation, and the bound does not say that every actual error grows.
For example, a persistent acceleration error of magnitude a permits position
error a*T^2/2 in a simple double integrator.

There are also avoidable enclosure effects. `stateUncertainty.flowTube`
propagates endpoint radii by `abs(transition)*radius + processBound`.
`ltvBicycleModel.finitePredict` copies each cell's `endRadius`, overwrites
`domainErrorBound`, and resets `domainGenerators=diag(radius)`. Signed
correlations are therefore discarded even though an earlier intermediate
calculation retains generators. A rotated narrow set becomes a larger
axis-aligned box, and repeated enclosing compounds this expansion. Taylor
remainders and arithmetic reserves are additional terms, not all physical
model uncertainty.

The retained witness currently certifies a fixed control sequence across all
allowed disturbances. Online observations validate that witness; they do not
replace its tube with a newly contracted one. Feedback replanning occurs, but
the proof does not encode a future feedback policy such as `u=v+K*e` with
closed-loop error matrix `A+B*K`. Fresh measurements cannot simply erase future
model errors or reset a stored proof. Feedback-tube design would require
compatible input/slew limits, robust interval constraints and successor proof.

The previous same-generator directional diagnostic reduced the 1.6 s lateral
radius from 3.39960 to 2.22897 m and heading radius from 0.357238 to 0.238788 rad.
Those are earlier numerical endpoint comparisons, not new certified tubes.
At 3 s even directional heading support was 0.451533 rad. Avoiding reboxing is
useful but is not alone a solution for the full physical residual allowance.

## Continuous collision-constraint alternatives

The following source-based comparison concerns geometric formulations;
none automatically establishes this project's recursive feasibility or finite
exit theorem under uncertain nonlinear dynamics.

- Sequential convexification / convex feasible sets iteratively rebuild
  supporting constraints around updated trajectories. This can improve the
  present fixed-anchor restriction without discrete maneuver selection.
  Initialization, local minima, inner feasibility and iteration cost remain
  important. A collision-crossing initial anchor is not guaranteed to yield a
  feasible convex subproblem. See Changliu Liu, Chung-Yen Lin and Masayoshi
  Tomizuka, *The Convex Feasible Set Algorithm for Real Time Optimization in
  Motion Planning*, abstract and method scope:
  <https://arxiv.org/abs/1709.00627>.
- Optimization-based collision avoidance (OBCA) uses strong duality to express
  convex-body distance constraints as smooth nonlinear constraints with
  additional variables. It avoids fixing one separating direction in advance
  for the represented geometry. It remains nonlinear/nonconvex and local
  numerical success is not global feasibility. See Xiaojing Zhang, Alexander
  Liniger and Francesco Borrelli, *Optimization-Based Collision Avoidance*,
  abstract and reformulation scope: <https://arxiv.org/abs/1711.03449>.
- Direct signed-distance constraints in a nonlinear program avoid a fixed
  halfspace but introduce closest-feature switches and derivative issues.
  This is a project-level formulation option, not an implemented algorithm or
  a claimed faster solution. Robust body uncertainty and continuous intersample
  safety still need certification.
- Collision-cone CBFs constrain relative velocity to avoid collision-cone
  directions. See *A Collision Cone Approach for Control Barrier Functions*,
  abstract: <https://arxiv.org/abs/2403.07043>. Transferring this idea requires
  accommodating rectangles, dynamic bicycle inputs, road/input constraints,
  uncertainty, feasibility and eventual exit. Adding a nominal CBF inequality
  alone does not settle those obligations.

Project judgment: continuous geometric refinement and OBCA are candidates for
comparison; no replacement has been selected or implemented. The currently
certified incumbent must remain available until any replacement complete
witness passes independent verification. Geometry refinement across samples
cannot silently discard retained obligations. Mixed-integer left/right maneuver
selection is outside the user's no-mode-branch requirement.

## Why road updates interrupt execution

A road point at world longitudinal coordinate 10 m has local coordinate 10 m
when the origin is zero, and 9 m after the origin moves forward by 1 m. That is
the same physical point. The current controller stores the entire road packet
inside its certificate identity and `hardEncounterBarrier.localValidateStored`
requires `isequaln(identity,stored.identity)` before continuation.

The previously captured second sample changes local origin, fitted coefficients
and coverage bounds. In common world coordinates the fitted boundaries differ
by only 1.12786e-14 m, yet exact packet comparison rejects at 0.1 s. This is a
representation-compatibility failure in that sample, not evidence of a changed
physical road. Independently, finite sensor coverage can be too short for the
complete witness; extending a polynomial beyond observed coverage is not a
certificate.

A repair must retain valid world-coordinate certified road information, or
verify updated information against the remaining witness and its uncertainty.
It must distinguish coordinate changes from changed geometry and missing
coverage. Merely disabling identity checks or using an arbitrary tolerance
would bypass required compatibility checks. No road repair was made here.

## Runtime measured on the current version

`profileForceFreeAdmission` replayed the saved original target-free first frame
at 10 m/s, 0.1 s period, 16 stages, with original physical residual allowance
`[0.2;0.06;0.02;2.5;5;4]`. All three admissions were certified. Total observed
calls were 1954.378, 616.974 and 565.438 ms. The first includes substantial
warmup (1269.303 ms in prediction) and is not a steady-state estimate. These
are current machine observations, not WCET or a physical closed-loop rerun.
The final two native-wrapper times were 124.454/209.973 and 120.498/204.255 ms
for LP/SOCP respectively. An additional MATLAB-instrumented call spent
322.053 ms in the native MEX, out of 610.691 ms inside the controller; that
profile is recorded separately because profiling changes overhead.

The last unprofiled complete call has the following phase measurements:

| Phase | Time [ms] |
| --- | ---: |
| Input preparation | 8.121 |
| Prediction | 15.004 |
| Formulation / geometric construction | 156.223 |
| Two-stage solve, assembly and internal verification | 379.263 |
| Final acceptance and commit | 4.541 |
| Total outer call | 565.438 |

Phase totals differ slightly from the outer timer because of orchestration.
This frame has no target, so target tracking/collision prediction cannot explain
the dominant solver cost. The 16 held intervals expand into 112 certified cells
and eight Bernstein points per cell. The LP contains 21,650 rows and 33 active
variables after inactive CLF-slack elimination. The performance SOCP contains
30,608 rows, 48 variables and 896 ten-dimensional Lorentz cones. Both take 15
iterations. Small control dimension does not imply a small conic system.

`profileNativeAvoidanceSolver` builds a separately named, instrumented COPY of
the production C++ bridge under the output directory, using the same pinned
Clarabel static library, single-thread QDLDL settings and exact captured
matrices. Production sources and binaries are unchanged. It measures data
checks/cone construction, solver creation, the solve call, and output/free.
Each phase is replayed six times. All 12 return the same status and decisions
within 1e-8 of the production solve. Medians below use the last five samples:

| Native phase | Hard-margin LP [ms] | Performance SOCP [ms] |
| --- | ---: | ---: |
| Data checks and cone construction | 0.075 | 0.174 |
| Solver creation | 11.592 | 16.809 |
| Actual solve call | 105.538 | 193.521 |
| Output and workspace release | 0.027 | 0.297 |
| Measured complete native call | 117.346 | 209.749 |

Each median is independent, so column medians need not sum exactly. The solve
call includes initialization, iterations and solver postprocessing. KKT
factorization, triangular solves, cone scaling and residual work have not been
separately measured; it would be premature to attribute all solve time to one
of them. The existing `solveTime` field is also saved but not treated as a
pure iteration timer. Source inspection shows a fresh solver workspace on every
production call, but measured setup is much smaller than actual solving.

The next performance experiments should test mathematically valid redundant-row
and cone reduction, alternative sparse transcription, and native linear-algebra
profiling before claiming a speedup. Sampling less often, deleting hard rows or
relaxing acceptance to obtain a faster number would change the guarantee.
Caching setup alone cannot eliminate the observed roughly 299 ms of solve-call
work across the two stages. No optimization or speedup is claimed in this task.

The new diagnostic driver passes factory Code Analyzer with zero findings.
A preliminary compiler-argument string was corrected. The host's known MEX
post-link inspection warning was handled only after rebuilding a fresh binary;
12 successful executions and decision comparisons verify loadability and
functional equivalence. No full unit suite was rerun because controller code
was unchanged. Results are saved as MAT/JSON with the instrumented source copy.
