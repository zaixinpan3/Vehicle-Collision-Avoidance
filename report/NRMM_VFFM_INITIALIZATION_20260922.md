# Reconstruct NRMM/VFFM initial trajectories

Date: September 22, 2026. Baseline source commit:
`4b678e2aa418184baff1a962ae56d609de02080b`.

The production initializer now uses analytical moving-target poses at ego
arrival times, normal sections of the selected quadratic road corridor, and
multi-target Gaussian superposition. It fits two passing assignments to the
existing affine vehicle prediction and initializes the full fixed-normal SOCP.
The complete mathematical specification, source attribution and implementation
interfaces are in
[NRMM_VFFM_INITIALIZATION.md](../controller/NRMM_VFFM_INITIALIZATION.md).

## Implemented behavior

- `targetPrediction.nrmmFlow` provides the fixed-frame NRMM pose, Cartesian
  velocity, acceleration and jerk for constant acceleration and sideslip. It
  rejects extrapolation through a braking stop unless the caller explicitly
  selects the stopped-pose branch. A shared stable sinc evaluation also fixes
  cancellation and premature straight-line approximation in `nominalFlow`.
- `laneGeometry.normalRoadChart` solves finite quadratic intersections with
  route normals and differentiates the resulting midpoint and width twice.
  Safe-side signs and parameter ranges select the component containing the
  route. The circular inverse now uses an available station hint to choose
  the correct revolution. An unbounded section reports its unit normalization
  explicitly; that normalization is not a physical road boundary.
- `solveHardCbfClf.vffmReference` evaluates the preferred path and its first
  two Cartesian time derivatives. It retains target acceleration, turning,
  curved-route geometry and width variation. Its heading is a course angle;
  the model-fit preference subtracts the anchor sideslip to obtain body yaw.
- `prepareFluidReference` selects every nominally conflicting target, chooses
  conflict-derived passing ordinates and relative-station widths, and prepares
  two joint side assignments. The fitted initial state is exact by rollout,
  and the final state/input equalities are retained. Physical base-row
  feasibility precedes worst-support-residual comparison in side selection.
- `standaloneControllerBenchmark.pack` exports the same numeric reference
  preparation to the native frame solver. Controller source count remains
  within the existing 20-file limit, including configuration and native sources.

All targets retain their original hard constraints, including targets that
do not shape the preference. The safety predictor remains the bounded
Cartesian `finiteFlow`. The exact NRMM assumptions characterize reference
generation and the explicit analytical API; they do not silently remove
measurement uncertainty or replace the controller's admitted motion contract.

## Design finding from the first implementation

The supplied example passing ordinate follows target lateral position at
each instant. Applying it directly to the crossing fixtures made the
reference sweep laterally with the obstacle. The first focused run passed
30 of 46 tests and failed 16 crossing admission tests; no command was issued
by those failed solves. Fixing the passing ordinate at the predicted conflict
while retaining the moving Gaussian center resolved most failures.

The remaining faster-ego fixture exposed a selection issue. For its two
fits, the maximum support residual was approximately 1.1756 versus 1.2820,
but the first fit violated physical base rows by 0.1781 while the second had
maximum excess -0.0831. Offline diagnostic solves rejected the first set of
directions and accepted the second. Therefore, base-row feasibility now
precedes the support score. The production policy still runs one trajectory
solve; it does not retry the two candidates with separate optimizations.
These observations motivate a bounded heuristic, not a completeness result.

## Validation

The complete repository suite passes **792/792 tests**, with zero failed or
incomplete tests and 600.884 s aggregate test duration in MATLAB R2026a Update 3.
The MATLAB MCP call exceeded its 300 s response limit; MATLAB continued to
completion and saved `full-tests.mat`. An independent subsequent call loaded
the saved results, ran `assertSuccess`, and exported `full-tests.json`.

The subsequent native rebuild found a prepared-input type issue: diagnostic
target arrays were scalar on admission and empty on continuation. The exporter
now retains only fields consumed by the solver. After this export-only fix,
the final focused suite passes **61/61** tests (standalone frame/workflow,
fluid initialization and the new mathematical reference tests).

Generated C/MEX admission returns status 4 and differs from MATLAB's complete
decision by at most **2.7462054852378515e-9**. Continuation returns status 2
and matches exactly. Both results pass the original independent hard verifier.
The native experiment is the mirrored-suite left circular crossing fixture:
ego reference speed 8 m/s, curvature 0.01/m, target crossing speed 4 m/s,
50 ms holds, 96-hold initial horizon, zero sensing-error bounds and a diagnostic
30 s work/deadline budget. It is a prepared-frame parity check, not a full
standalone runtime or timing campaign. The first failed native type probe is
retained in `native.log`; no source binary is committed.

Factory Code Analyzer reports no new findings across eight changed MATLAB
files. One pre-existing `FNDSB` performance suggestion remains in the unchanged
solver reduction. `git diff --check` passes. The compact machine-readable
[validation record](NRMM_VFFM_INITIALIZATION_20260922.json) contains actual
suite counts, native differences and source hashes.

The focused pre-existing suite passed **56/56** tests covering fluid admission,
full-plan optimization, rejection after failed optimization, target prediction
and continuation. The new `nrmmVffmReferenceTest` passed **16/16** tests:

- Independent `ode45` integration of accelerating, turning NRMM motion;
  finite-difference jerk and consistent Cartesian-interface equivalence.
- Small curvature with substantial accumulated lateral displacement;
  explicit braking-stop policy and known-sideslip acceleration from rest.
- Exact quadratic midpoint/width derivatives, rotated boundary frames,
  curved normal intersections and rejection of an irregular chart.
- Recovery of the stationary straight-road Cheng Gaussian, constant reference
  under equal longitudinal speeds, multiple-target contributions, and invalid
  parameter rejection.
- Independent centered differences of Cartesian reference position on
  straight and mirrored curved roads, with target acceleration/turning and
  quadratic boundary variation. Velocity tolerance is 2e-7 and acceleration
  tolerance is 5e-6 in their respective SI units; normal acceleration is also
  checked independently from Cartesian velocity and acceleration.

The deterministic checks use explicit fixtures and no random sampling. The
focused full-plan suite also completed its existing 140-hold circular crossing
scenario with sampled body gap approximately 0.155046 m. That diagnostic run
is not a strict-deadline or inter-sample proof.

## Limits

The complete source changes concern initialization and its numerical support.
The existing six-state Frenet affine Fiala model, hard SOCP, verifier and
continuation logic remain the execution architecture. General union-domain
intersection containment and the supplied seven-state nonlinear planning
problem are not implemented by this initializer. The current node certificate
does not prove continuous-time separation. Two joint passing assignments can
miss feasible combinations. No hard real-time, physical nonlinear-vehicle or
universal collision-avoidance claim follows from these results.

Raw checks and diagnostic artifacts are retained under
`/home/zai/.cache/collisionAvoidance/nrmm-vffm-20260922/`. Generated binaries,
temporary diagnostics and literature PDFs are excluded from the source commit.
