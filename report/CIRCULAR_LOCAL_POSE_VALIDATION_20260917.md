# Circular avoidance with certified local pose domains

Date: September 17, 2026. Controller-only implementation and validation.

The twelve circular obstacle cases that previously failed admission now complete
30 s with the complete hard certificate when given a 5 s diagnostic search
budget. All confirm departure and recover to curved cruise. They remain
**unqualified for the required 100 ms complete-frame deadline**: each diagnostic
run has one overdue initial encounter-admission frame. Strict periodic runs stop
at that frame and issue no command from it. Offline completion is not reported
as real-time success.

## Implemented change

The controller now uses a local first-order Frenet-to-Cartesian map, including
the station derivative `(1-kappa*d0)*t`, and the exact affine circular heading
`theta0+kappa*(s-s0)+ePsi`. The position Taylor remainder is certified on an
explicit local box. Every uncertain Bernstein coefficient has six hard domain
rows, and the final exit condition has hard endpoint-domain rows. The map,
domain and finite exit geometry are reanchored together during fresh search;
accepted suffixes retain the original domains and absolute exit deadline.
Nominal distance/direction queries use exact curved poses. The geometric unit
tangent is stored separately from the position Jacobian.

Default extra radii are `[2;4;0.5]` in station metres, lateral metres and heading
radians. These define a searched convex approximation domain, not a prescribed
avoidance path or a physical road boundary. The domain restrictions are required
to justify the smaller remainder; they have no restoration slack.

Pure linear search-deficit minimization produced unstable restoration candidates
and asymmetric left/right admission failures during development. Restoration now
minimizes the sum of squared normalized deficits, with no competing control or
tracking objective. Zero still means all relaxed certificate obligations are
satisfied, but a positive-deficit search plan is never executable. The final
hard problem and independent verifier retain their original acceptance role.

Sparse stage rows are assembled directly from coordinate triplets. Constraint
generation groups domain rows separately from collision rows, checks the complete
omitted-row set and retains all nongeometric constraints. This avoids repeatedly
editing large condensed sparse matrices or solving every redundant Bernstein
row at every internal iteration. Domain rows remain hard in the completed
program. The predictor, Predictive CBF, squared soft CLF, curved operating input,
actuator/slew limits and high-gain observer framework remain in place. The
terminal law remains a prediction certificate, never an executable fallback.

Controller state format is 33; old stored states must be reset. Rebuild the
changed native geometry payload with
`scripts/buildAvoidanceGeometryKernel.m`; generated solver artifacts remain
outside the commit. The mathematical argument is updated in
[SUPPORT_CONVEXIFICATION.md](../controller/SUPPORT_CONVEXIFICATION.md) and
[TERMINAL_CBF_PROOF.md](../controller/TERMINAL_CBF_PROOF.md).

## Experiments and limitations

Reused `scripts/runCircularArcControllerValidation.m`: MATLAB R2026a, one
computational thread, six independent discarded startup attempts, 8 m/s cruise,
0.1 s hold, 300 intended holds, radii 100 and 50 m in both turn directions,
16 m complete-observation region, 0.25 m required clearance and seed 20260912.
There are no physical road boundaries, estimator errors or process residuals.
The plant is exactly the declared held affine generator. Targets are stationary,
oncoming along an inertial tangent, or crossing along an inertial normal.
The straight-target law is not replaced by a road-following target law.

Strict 100 ms trials and independent 5 s diagnostic trials are separate runs.
Measured frame time includes input assembly and controller work, excluding
independent plant integration and offline geometric audits. Timing experiments
finished before the full test suite was launched. These are observed timings,
not worst-case execution proofs.

All four nominal-cruise periodic runs complete within 100 ms. The twelve
obstacle periodic runs stop at admission (0 s for stationary/crossing, 2.6 s for
oncoming). Every corresponding diagnostic run completes 300 holds and all
issued commands pass the hard verifier. No terminal/fallback command is used.

The following are diagnostic results. Clearance margin is rectangle separation
**minus** the required 0.25 m; it is sampled independently within executed holds.
The whole-hold guarantee comes from the hard Bernstein certificate, not those
samples. Lateral excursions are permitted because road boundaries are absent.

| Curvature (1/m) | Scenario | Minimum margin (m) | Peak lateral error (m) | Maximum frame (ms) | Maximum non-admission frame (ms) |
| --- | --- | ---: | ---: | ---: | ---: |
| +0.01 | stationary | 0.223565 | 2.473 | 251.746 | 78.792 |
| +0.01 | oncoming | 0.248790 | 2.901 | 134.137 | 56.116 |
| +0.01 | crossing | 0.212298 | 4.793 | 249.399 | 51.435 |
| -0.01 | stationary | 0.223833 | 2.471 | 221.412 | 58.311 |
| -0.01 | oncoming | 0.248854 | 2.901 | 112.019 | 44.230 |
| -0.01 | crossing | 0.191733 | 4.804 | 188.757 | 54.870 |
| +0.02 | stationary | 0.305053 | 2.869 | 188.695 | 58.701 |
| +0.02 | oncoming | 0.445741 | 3.054 | 120.542 | 31.148 |
| +0.02 | crossing | 0.356680 | 4.879 | 233.874 | 48.781 |
| -0.02 | stationary | 0.304721 | 2.873 | 177.518 | 56.300 |
| -0.02 | oncoming | 0.445741 | 3.054 | 128.427 | 30.589 |
| -0.02 | crossing | 0.284702 | 4.913 | 240.573 | 76.619 |

All twelve diagnostic cases end with absolute lateral error below 0.000020 m,
heading error below 1e-8 rad and longitudinal speed error below 1.2e-7 m/s.
These are finite-run recovery observations, not a general asymptotic theorem
with unrestricted CLF slack. The +0.02 crossing target is detected again at
22.5 s on the continuing circular route; that new admission also succeeds.
Sensor release is not assumed to mean permanent non-reencounter.

The unresolved bottleneck is fresh encounter admission: formulation/reanchoring
and multiple restoration/working-set solves. In this batch, primary admission
uses one support family, one or two restoration solves, one or two hard solves,
and seven to eleven native working-set solves. Maximum non-admission frame time
is at most 78.792 ms across these diagnostic cases. The strict per-frame requirement is
still not met because admission is part of the pipeline. Moving admission
outside the timing measurement or executing a relaxed iterate is not a remedy.

## Certificate and implementation checks

The same stationary 48-hold initial-geometry diagnostic formerly reported
271.083 m coordinate-map error at curvature +0.01/m. The local-domain map now
reports 0.125954 m, and +0.02/m reports 0.254221 m. Both have zero individually
impossible collision rows over the actuator outer box. Positive outer-box
margins alone do not establish joint feasibility; the completed experiments
provide separate certified-plan evidence. Initial terminal SOC radii remain
positive. The bound reduction is justified by the newly enforced domain, not
by discarding uncertainty or reducing an unexplained constant.

New behavior tests cover both curvature signs, sampled Taylor enclosure and
exact yaw, native/interpreted geometric parity, physical rectangle residual
majorization, hard rejection of an impossible domain during restoration,
sparse-row/objective equivalence (including quadratic restoration), and
carried-witness/domain/deadline preservation. Existing admission, recursive
closure, geometry and observer tests remain part of the complete suite.

Initial complete MATLAB suite: **633 passed, 1 failed, 1 incomplete**.

The only initial failure was in the new sparse-equivalence test fixture: the
hard-program anchor omits its CLF slack coordinate. The fixture now pads that
coordinate before perturbing a complete decision. A subsequent check found a
1.57e-8 absolute (1.61e-10 scaled) row residual from the long-horizon coordinate
transcription. The test now compares row residuals at the configured scaled
feasibility tolerance, with the independent physical verifier unchanged. The
circular and recursive-closure test classes were rerun in isolation:
**29 passed, 0 failed, 0 incomplete**.
One intermediate concurrent rerun exceeded its default 5 s search budget before
search began; the behavior fixture now consistently grants 30 s for both search
and frame budgets. The separate 100 ms experiment criterion is unchanged.
The final controller adjustment removes a 1e-9 outward allowance from the
observation-release domain check; an explicit outside-domain test, the curved
test class, recursive-closure tests and the complete circular experiment batch
were rerun afterward.

Code Analyzer ran on all changed MATLAB files: no syntax diagnostics; it reports
an unavailable local settings file (defaults used), bounded candidate/block
concatenation and pre-existing sparse-indexing performance advisories. Native
geometry, projection and support kernels were regenerated successfully.
`git diff --check` was also run. The core source count remains 20.

Straight-reference 30 s diagnostic regressions, after independent startup:

| Scenario | Completed | Maximum frame (ms) | Minimum margin (m) |
| --- | --- | ---: | ---: |
| cruise | True | 13.534 | n/a |
| stationary | True | 80.587 | 0.055981 |
| oncoming | True | 54.827 | 0.134923 |
| crossing | True | 15.089 | 9.301601 |

Full compact metrics are in
[CIRCULAR_LOCAL_POSE_VALIDATION_20260917.json](CIRCULAR_LOCAL_POSE_VALIDATION_20260917.json).
Raw MAT/JSON runs, profiling, intermediate failed attempts, geometry audits and
test logs are retained at
`/home/zai/.cache/collisionAvoidance/local-chart-20260917/`.
The earlier linear-restoration and full-row timings are development diagnostics,
not silently replaced successful benchmarks. Changes to the observer, variable
curvature, nonlinear physical-model inclusion, correlated uncertainty sets and
physical road-terminal containment are outside this implementation.
