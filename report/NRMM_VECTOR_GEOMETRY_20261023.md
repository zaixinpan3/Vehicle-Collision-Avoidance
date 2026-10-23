# Vector Jacobian geometry in the retained NRMM observer

The revised certificate preserves the eight-state body-velocity/target cascade,
the parallel yaw observer, the NRMM saturation extension, the scalar block
innovation gains and the sampled runtime. It exploits nonlinear derivative
couplings before taking norms and the physical correlation of the target's
shared gyro disturbance. At the default domain, the global velocity-Jacobian
bound falls by 41.23%; resynthesis reduces the continuous ultimate component
bounds by 4.53%, 9.35% and 14.70% at the same 0.8/s decay floor.

## Paper and mathematical specialization

Source: Gaurav Sharma, Ali Zemouche and Rajesh Rajamani, *Observer Design for
Vector Nonlinear Systems With Applications to Autonomous Vehicles and Robots*,
International Journal of Robust and Nonlinear Control, first published
September 23, 2026, [DOI 10.1002/rnc.70732](https://doi.org/10.1002/rnc.70732).
The supplied local PDF is under `reference/`. The relevant material is
Lemma 1 and Theorem 1, Eqs. (9)--(19), the disturbance formulation (26)--(32),
and the state scaling in (33)--(35).

The paper motivates incremental vector-Jacobian constraints and scaled quadratic
certificates. The project-specific derivations are in
[OBSERVER_ISS_THEORY.md](../estimator/OBSERVER_ISS_THEORY.md), Sections 2.1, 5 and 6:

- In the orthonormal frame aligned with target velocity, retain the repeated
  clipped acceleration/yaw factors in the complete 2-by-2 velocity Jacobian.
  Three analytic envelopes cover the inverse floor, ordinary speed, and speed
  saturation regions, with both scalar clips and the acceleration projection.
  The maximum spectral norm, intersected with the earlier product bound,
  yields a global incremental bound even when the connecting segment crosses
  zero or leaves the physical domain.
- Keep the rotation-invariant metric `P kron I2`, so common rotating-frame
  transport contributes exactly zero to the Lyapunov derivative. The existing
  two-multiplier dissipation LMI uses the improved bound, with unchanged
  physical state scaling and decay preference.
- Bound the single target gyro column using `q'*s = V*A`, maximizing over
  relative-position direction first. Separate convexity reduces the remaining
  speed/acceleration box maximum to four corners. The same certificate enters
  offline gain selection and online sampled propagation. Initial error and
  integration-defect vectors retain their generic bounds.

The Jacobian envelopes and four-corner gyro bound are derived for this observer;
they are not the paper's stated theorems. This implementation uses norm
incremental quadratic constraints after retaining vector derivative structure.
It does not implement the paper's general sector LMI or gain switching.
The triangular comparison still separates ego and target stage norms; the
exact joint gyro support is documented without claiming that unimplemented
cross-stage cancellation has been certified.

## Default design comparison

Baseline: project commit `b7c2f696736f0eb8accba2fae986b7ba329ebd8d`, synthesized
before the changes. Both designs use `nrmmTrackingConfig`: target speed
10--20 m/s, scalar acceleration at most 2 m/s^2, sideslip at most 0.015 rad,
rear-axle distance 1.6 m, range at most 50 m; gyro bound 0.002 rad/s and
radar bound 0.12 m. The full configuration and numerical metrics are in
[comparison.json](NRMM_VECTOR_GEOMETRY_20261023/comparison.json).

| Quantity | Baseline | Revised |
|---|---:|---:|
| Global velocity-Jacobian bound | 0.801053 | 0.470760 |
| Global acceleration-Jacobian bound | 0.870822 | 0.870822 |
| Target bandwidth, 1/s | 2.104213 | 1.958305 |
| Position innovation gain | 7.297652 | 6.812159 |
| Velocity innovation gain | 26.254050 | 23.048664 |
| Acceleration innovation gain | 40.942346 | 33.900888 |
| Certified decay floor, 1/s | 0.800000 | 0.800000 |
| Continuous position ultimate bound, m | 1.714400 | 1.636667 |
| Continuous velocity ultimate bound, m/s | 7.553679 | 6.847181 |
| Continuous acceleration ultimate bound, m/s^2 | 16.518027 | 14.090095 |

For the revised metric held fixed, the gyro coefficient drops from 384.225927
to 383.578627, a 0.1685% improvement. The larger improvement comes from the
nonlinear Jacobian refinement and the resulting gain/metric resynthesis.
Gyro coefficients across different metrics cannot be compared in isolation.
The independently recovered dissipation residual has minimum eigenvalue
approximately `1e-6` for both designs. These are floating-point checks, not
directed-rounding certificates or globally optimal synthesis claims.

## Paired sampled trajectories

`verifyNrmmGeometricDesign(baselineDesign)` runs both sets of gains through the
same current runtime, truth generator and bounded-uniform random draws. Each
run lasts 12 s; RMSE excludes the initial 2 s. The retained NRMM maneuver uses
seeds 7, 17 and 23 at 20 ms sensing. The additional seed-23 cases use 40 ms
sensing or radar dropout from 4.0 to 4.4 s. RK4 substeps remain at most 5 ms.

| Case | Position RMSE, baseline / revised, m | Velocity RMSE, baseline / revised, m/s | Acceleration RMSE, baseline / revised, m/s^2 |
|---|---:|---:|---:|
| Seed 7 | 0.02385 / 0.02344 | 0.08305 / 0.07754 | 0.12427 / 0.11143 |
| Seed 17 | 0.02492 / 0.02402 | 0.08986 / 0.08227 | 0.13559 / 0.11960 |
| Seed 23 | 0.02429 / 0.02406 | 0.08561 / 0.08021 | 0.13077 / 0.11748 |
| Slower sensing | 0.03169 / 0.03079 | 0.11038 / 0.10157 | 0.16578 / 0.14626 |
| Radar dropout | 0.02451 / 0.02398 | 0.08588 / 0.07980 | 0.13078 / 0.11661 |

All ten runs remain finite, satisfy the sampled truth-domain checks and have
positive position-enclosure slack. The revised minimum slack across these
runs is 1.39016 m. Mean sampled position radii change by less than 0.001 m
and increase slightly in these cases: approximately 1.71 m for ordinary
sensing, 3.24 m for slower sensing and 2.38 m with dropout. Smaller continuous
certificates do not imply a reduction in the complete sampled enclosure,
which also includes held-input uncertainty, predictors and integration defects.
The paired test isolates the gain change; it is not a comparison against an
independently rerun historical runtime, and it establishes no uniform accuracy
or real-vehicle advantage.

## Validation and reproduction

All **821 tests pass**, with zero failed or incomplete tests, using:

```matlab
results = runtests('tests');
assertSuccess(results)
```

[tests.csv](NRMM_VECTOR_GEOMETRY_20261023/tests.csv) records the individual
outcomes (230.16 s summed test duration on this execution). The new 11 cases
check regional vector Jacobians, global finite increments, gyro containment
including zero speed, an attained rotated maximizer, metric scaling,
independent acceleration caps and invalid metrics. Existing tests cover
nonlinear Lyapunov decay, varying maneuvers, model-aided ego estimation,
dropout, sampled defects, native integration and controller interfaces.
Numerical probes supplement the analytic derivations; they are not proofs.

An initial 235-test focused run had one failure because the cascade test still
expected the old separated gyro formula. That test now checks the metric norm
of the actual maximizing gyro column and its improvement over the separated
bound. The final full suite includes the corrected test. MATLAB Code Analyzer
reported no source issues in the six edited/new MATLAB files; a missing local
settings file caused it to use default rules. `git diff --check` passed.

To reproduce the paired comparison, synthesize and save `baselineDesign` in a
checkout of the baseline commit, then use the revised checkout:

```matlab
addpath('scripts');
comparison = verifyNrmmGeometricDesign(baselineDesign);
```

Calling the driver without an argument audits the revised design alone.
The comparison uses a saved baseline design only as an experimental input;
the operational observer continues to synthesize its gains normally.
