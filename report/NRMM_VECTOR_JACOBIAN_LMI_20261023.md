# Full vector-Jacobian LMI in the retained NRMM observer

Date: October 23, 2026. Baseline: `51fc76b509cebec5a93d038e182366d566cfe8ef`,
the immediately preceding geometric scalar-bound implementation.

The previous change retained nonlinear coupling while deriving two scalar
Lipschitz constants, then used those scalars in the SDP. This implementation
carries the full signed 2-by-4 Jacobian into both gain selection and bandwidth
certification. The body-velocity/NRMM cascade, parallel yaw observer, target
vector field, saturation extension, and shared gyro disturbance bound remain
in use.

## Implemented mathematics

[Theory, Sections 2.2, 5 and 7](../estimator/OBSERVER_ISS_THEORY.md) derives the
global matrix enclosure and finite-error proof. In position-normalized error
coordinates, with \(M=P\otimes I_2\), the actual constraints are
\[
F_j^TM+MF_j+2\lambda M\preceq0,\qquad
F_j=\omega(A_\ell\otimes I_2)+(e_3\otimes I_2)
 [0,D_q\Phi_j/\omega,D_s\Phi_j].
\]
The Jacobian entries retain their signs, directional dependence, repeated
nonlinear factors and acceleration-projection derivatives. Orthogonal
invariance removes heading from the SDP and cancels common measured rotation
exactly. A common metric and constant gain cover all saturation branches.
Finite-error contraction follows by integrating Jacobians along the complete
segment, including crossings of zero speed and saturation surfaces.

`nrmmTargetJacobian.m` covers the unbounded extended state space using three
compact radial parameterizations. It converts branch polynomials, and the
middle region's rational polynomial with positive denominator, to common
Bernstein coefficients. Their convex weights are shared by all matrix entries.
This is an analytic enclosure, not a maximum over sampled Jacobians. Eight
subdivisions per compact axis give 1,670 retained branch boxes and 51,209 unique
matrix vertices for the default domain. Acceleration projection, both scalar
clips and all three radial regions are included.

Constraint generation keeps the SDP small, while a complete eigenvalue check
of every enclosure vertex is mandatory before acceptance. The default shape
solve uses 40 constraints and the final metric solve uses 31; both converge
in two iterations. Maximum residual eigenvalues are approximately
\(-10^{-7}\) and \(-10^{-6}\), respectively. The shape is solved through
\(Y=PL\); no hand-tuned gain is introduced. Pagewise eigenvalue evaluation
reduces fresh default design time from a measured 13.74 s loop implementation
to 7.13 s on this machine. These are single-run development timings, not a
benchmark or personal work hours. The new calculation is offline gain design;
the runtime observer state derivative is unchanged.

This specializes the vector incremental idea of Sharma, Zemouche and Rajamani,
*Observer Design for Vector Nonlinear Systems With Applications to Autonomous
Vehicles and Robots*, Lemma 1 and Theorem 1 (13), with disturbance and scaling
context in (26)–(35), [DOI](https://doi.org/10.1002/rnc.70732). It is a matrix
polytope formulation derived for this extension, not an implementation of all
the paper's sector multipliers. The enclosure is sufficient and can still be
conservative. The numerical residual checks use double precision; no directed
rounding or globally optimal bandwidth claim is made.

## Default design result

The domain and sensor bounds are unchanged: speed 10–20 m/s, scalar
acceleration at most 2 m/s², sideslip 0.015 rad, rear-axle distance 1.6 m,
range 50 m, gyro error 0.002 rad/s and radar error 0.12 m. The target decay
floor remains 0.8/s. The two auxiliary scalar bounds are also unchanged:
\(L_q=0.4707599283\), \(L_s=0.8708222481\). They support integration-defect
and dropout-growth bounds but no longer enter the gain SDPs.

| Continuous ultimate bound | Previous scalar LMI | Full Jacobian LMI | Change |
|---|---:|---:|---:|
| Position (m) | 1.636667 | 1.625006 | -0.71% |
| Velocity (m/s) | 6.847181 | 6.446161 | -5.86% |
| Acceleration (m/s²) | 14.090095 | 12.480716 | -11.42% |

Physical innovation gains change from
`[6.812159; 23.048664; 33.900888]` to
`[6.511933; 20.802440; 27.177565]`.
The normalized shape changes as well, so the slightly increased bandwidth
(1.958305/s to 1.988463/s) does not mean larger physical gains.

There is a direct numerical witness that the new metric uses information
lost by norm reduction. For the final \(P\), let
\(B_0=\omega(A_\ell^TP+PA_\ell)+2\lambda P\), \(d=Pe_3\), and take
\(v=(0.10056550,0.45619032,0.88418153)^T\). The independent norm-ball model
admits the derivative excess
\[
v^TB_0v+2|d^Tv|[(L_q/\omega)|v_2|+L_s|v_3|]=1.96001924>0.
\]
Thus this metric does not certify the independent norm-ball uncertainty set,
although it certifies every full Jacobian vertex. A separate fixed-metric SDP
minimizing the old norm LMI's slack gives the same positive value numerically.
This does not show infeasibility of every possible metric for the old gains
or universal dominance of the new enclosure.

## Paired synthetic trajectories

`verifyNrmmGeometricDesign` runs both designs through the same current sampled
runtime, with identical truth and bounded-uniform noise. Each case is 12 s;
RMSE omits the first 2 s. Seeds are 7, 17 and 23, sensor periods are 20 ms
and 40 ms, RK4 substeps are at most 5 ms, and the dropout case removes radar
from 4.0 to 4.4 s. The baseline is the previous commit's synthesized design;
the new design is freshly synthesized from current code.

| Case | Velocity RMSE (m/s), old → new | Acceleration RMSE (m/s²), old → new | Mean position-radius change (m) |
|---|---:|---:|---:|
| retained7 | 0.077536 → 0.070754 | 0.111435 → 0.089009 | +0.000191 |
| retained17 | 0.082271 → 0.072615 | 0.119599 → 0.091852 | +0.000287 |
| retained23 | 0.080214 → 0.072760 | 0.117481 → 0.092067 | +0.000336 |
| slower23 | 0.101570 → 0.091512 | 0.146259 → 0.114623 | +0.000554 |
| dropout23 | 0.079797 → 0.071976 | 0.116607 → 0.090862 | +0.000505 |

All ten runs are finite, sampled truth-domain checks pass, position-bound
availability and observed containment are 100%, and minimum position slack
is positive. Velocity RMSE improves 8.75–11.74%, and acceleration RMSE improves
20.12–23.20%. Position RMSE also improves in these cases, but its maximum error
is slightly worse in some seed-23 cases. Mean sampled radii increase by less
than 0.0006 m; the dropout case's maximum radius increases about 0.010 m.
The cumulative `operatingDomainValid` flag remains false because initial
estimates can activate scalar clips; the truth-domain flag remains true.
`digitalErrorBoundCertified` remains false: these numerical containment results
do not establish an unconditional sampled-data proof or closed-loop safety.

## Validation and reproduction

The initial complete run passed 832 of 833 tests. The stationary/braking
integration case exposed SeDuMi numerical-termination status 4, despite a
recovered shape satisfying the full enclosure and Gramian inequalities.
Acceptance now permits that status only after the same strict independent
checks used for status 0. The affected integration test then passed; the
returned solver status remains visible in certificate metadata. The scenario
controller can still terminate without an optimization solution; passing this
test verifies observer integration, not successful collision avoidance.

The targeted suite passed all 31 tests, including 12 new cases for finite
differences across every radial/projection regime, rotation and reflection,
full nonlinear Jacobian dissipation, finite errors crossing saturation
surfaces, zero velocity and support-function enclosure diagnostics.
The existing shape and free-metric tests independently reconstruct the full
six-state dynamics at every enclosure vertex. Sampled diagnostics supplement
the analytic coverage proof; they are not its basis.

The final complete repository suite passed **all 833 tests**, with zero
failures or incomplete cases. Summed test duration was
684.862846 s. This duration is not a measure of personal work hours.

MATLAB R2026a Update 3 Code Analyzer uses its default rules because the local
legacy settings file is absent. Two test-local variable-name warnings were
removed; final analysis has no source diagnostics. Git whitespace checks pass.

Run the complete suite from the repository root:

```matlab
results = runtests('tests');
assertSuccess(results);
```

For a paired comparison, synthesize `baselineDesign` at baseline commit
`51fc76b509cebec5a93d038e182366d566cfe8ef` in a separate checkout with the
same solver dependencies, then run on the current checkout:

```matlab
addpath('scripts');
comparison = verifyNrmmGeometricDesign(baselineDesign);
```

The comparison helper accepts both legacy scalar certificates and the new
matrix certificate. Full metrics and matrix-enclosure metadata are in
[comparison.json](NRMM_VECTOR_JACOBIAN_LMI_20261023/comparison.json), the explicit
norm-set witness in [norm-comparison.json](NRMM_VECTOR_JACOBIAN_LMI_20261023/norm-comparison.json),
and the complete test results in [tests.csv](NRMM_VECTOR_JACOBIAN_LMI_20261023/tests.csv).
