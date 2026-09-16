# Reduce NRMM derivative noise with a free Lyapunov metric

Date: September 16, 2026. Scope: continuous radar observation, as requested.
The revised automatic gain synthesis reduces nominal noisy velocity RMSE by
74.82% and acceleration RMSE by 89.90% in ten paired validation seeds. It
retains the NRMM model, normalized high-gain shape, physical/sensor bounds and
required continuous decay rate. Its lower bandwidth increases maneuver lag.

The comparator is the gain design at
`e87af46662334a39256cb950406510ebc940a737` (the estimator sources are unchanged from the preceding evaluation).
Only function names and bootstrap paths are adapted in the external baseline
source export; both designs use the same current runtime and native kernels.
No historical implementation is added to the active repository.

## Diagnosis and change

Noise-isolation trials use seed 7, 12 s, 50 Hz sensing, a maximum RK4 step of
0.005 s and the original gains. The design continues to assume the full
configured error bounds while individual simulated noise sources are removed.
The truth and initial conditions remain identical.

| Simulated measurement noise | Position RMSE (m) | Velocity RMSE (m/s) | Acceleration RMSE (m/s²) |
| --- | ---: | ---: | ---: |
| All configured sensors | 0.045884 | 0.470140 | 1.786139 |
| Radar only | 0.045875 | 0.469460 | 1.784276 |
| All except radar | 0.000853 | 0.021916 | 0.046977 |
| None | 0.00000792 | 0.015064 | 0.005706 |
| All sensors, RK4 step halved | 0.045884 | 0.470141 | 1.786143 |

These controls identify amplification of sampled radar position noise as the
dominant source of derivative error in this scenario. Integration refinement
does not materially improve the result. They do not imply that other sensor
errors or physical-model errors are negligible on a real vehicle.

The earlier synthesis fixed the target metric by `A_l'*P + P*A_l = -I` before
searching the bandwidth. That restriction made its sufficient nonlinear
decay test conservative. At each candidate bandwidth, the new synthesis
jointly solves for `P` and the two positive Lipschitz multipliers using the
Schur inequality in [the theory, Section 5](../estimator/OBSERVER_ISS_THEORY.md).
It minimizes `trace(P)` subject to `P >= I` and checked dissipation at
`lambda = 1 / domainTransitTime`. A bracketed scalar search then minimizes the
largest domain-normalized ultimate component bound. Numerical infeasibility
or an invalid recovered matrix rejects a candidate.

The metric normalization fixes homogeneous scaling. This is a feasible local
search over the bandwidth with a deterministic metric-selection rule, not a
global joint optimum or a directed-rounding proof. The seed-7 exploratory
bandwidths 2.7, 2.8, 3, 3.5, 4 and 5 confirmed the expected noise tradeoff;
the final bandwidth is selected by the certificate objective, without using
simulation RMSE, validation seeds or the sensor sample period.

| Default design quantity | Previous | Revised |
| --- | ---: | ---: |
| Physical bandwidth (s⁻¹) | 6.443096 | 2.591046 |
| Required continuous decay (s⁻¹) | 0.8 | 0.8 |
| Position innovation gain | 29.4808 | 11.8555 |
| Velocity innovation gain | 327.997 | 53.0434 |
| Acceleration innovation gain | 1278.66 | 83.1565 |
| Continuous relative-position ultimate bound (m) | 5.7599 | 2.2072 |

The existing radar-predictor equation also explains the noise reduction. At
zero ego rotation, integrating just the acceleration innovation over one hold
gives `(K3/K1)*(1-exp(-K1*Ts))*initialRadarResidual`. At `Ts=0.02 s`, this
coefficient decreases from 19.321 to 1.481 s⁻². This is the innovation term's
response, not the full nonlinear acceleration error or an RMSE prediction.

The actual equations, output transformations, sensor-noise bounds, RK4
implementation and online enclosures are unchanged. Scalar ego gains are
unchanged. No smoothing window, future measurements, simulated truth or
manual observer gain enters the revised runtime. Constant gains are designed
before simulation; the SDP is not solved per sensor frame.

## Paired validation without radar outages

There are 31 identical-input pairs (62 executions): one noise-free pair and
ten pairs each for nominal bounded noise, changing target motion and 25 Hz
sensing. Validation seeds are 71–80, separate from the exploratory seed 7.
The model, initial offsets and sensor bounds are those in the
[preceding evaluation](NRMM_ESTIMATOR_EVALUATION_20260916.md). Each trial lasts
12 s; metrics use samples at `t >= 2 s`, comparing the propagated estimate to
truth at the same advanced time. Both methods receive identical truth,
measurements and initialization. The new benchmark `Cases` option excludes
the dropout scenario from this campaign.

Entries below are means of the individual seed RMSEs, not fitted simulation
objectives. The committed CSV retains every run.

| Scenario | Position RMSE, old → new (m) | Velocity RMSE, old → new (m/s) | Acceleration RMSE, old → new (m/s²) |
| --- | ---: | ---: | ---: |
| Bounded noise, 50 Hz | 0.047617 → 0.029455 | 0.481970 → 0.121377 | 1.8276 → 0.1846 |
| Changing acceleration/curvature, 50 Hz | 0.046245 → 0.029940 | 0.44433 → 0.13253 | 1.6025 → 0.30951 |
| Bounded noise, 25 Hz | 0.071523 → 0.042847 | 0.71450 → 0.17719 | 2.6721 → 0.26652 |
| Noise-free, 50 Hz | 0.00000792 → 0.00012256 | 0.015064 → 0.015312 | 0.005706 → 0.006451 |

Nominal noisy position, velocity and acceleration RMSE improve by 38.14%,
74.82% and 89.90%. The changing-motion acceleration improvement is 80.69%,
and the 25 Hz acceleration improvement is 90.03%. The noise-free comparison
has slightly larger residual errors, consistent with the slower transient.
It remains a same-model synthetic comparison, not a real-world accuracy claim.

The mean initial acceleration-error peak falls from 34.60 to 5.06 m/s² for
nominal noise and from 36.13 to 5.25 m/s² at 25 Hz. Startup peaking is reduced,
not eliminated. After 2 s the revised estimated physical domain is valid at
100% of samples in all 31 executions, versus nominal noisy 62.44%, changing
motion 73.73% and 25 Hz 38.69% for the previous gains. All true-target domain
checks pass and all estimate samples remain finite.

The material tradeoff is response lag: the changing-motion curvature fit
increases from 0.292 to 0.604 s on average. The maneuver transitions at 6 s
using the existing smooth 0.2 s transition scale; scalar acceleration changes
by 0.6 m/s² and curvature by -0.007 m⁻¹. The corresponding model-rate bounds
remain 1.5 m/s³ and 0.0175 m⁻¹s⁻¹. The varying-motion synthesis uses the same
required decay as its comparator (0.72727 s⁻¹). A better noisy RMSE does not
remove this additional lag or establish performance for faster maneuvers.

The benchmark retains `digitalErrorBoundCertified=false`. The new matrix
condition is a continuous-time certificate under the existing global
Lipschitz and bounded-input assumptions. The sampled position enclosure is
a separate conditional calculation. No closed-loop collision-avoidance
guarantee or improved controller admission result follows from this campaign.

## Validation and reproduction

The final full repository regression passes **618/618 tests**, with zero failed
or incomplete cases. The initial focused gain/certificate regression passed
all 25 cases. Updated regressions independently check nonlinear dissipation,
tighter nominal noisy errors/startup peaks, 25 Hz errors and changing-motion
accuracy/lag. Five
changed MATLAB files have zero factory Code Analyzer findings. Independent
Python analysis recomputes 186 RMSE values from 31,262 exported samples with
maximum discrepancy `4.14e-15`, verifies identical paired truth, and checks
positive metrics, dissipation inequalities and component conversions for all
62 certificates. Revised minimum metric and dissipation eigenvalues are about
1.0 and `1.00e-6`. Every exported position error is inside its available finite
online enclosure. The revised nominal mean radius remains 1.704 m, so the
smaller point-estimate error does not imply equally small controller margins.
Both result figures were inspected.

An initial JSON export rejected complex-valued pole metadata; the corrected
export omits that unnecessary field and preserves the real matrices needed
for the independent check. The full-suite MCP call exceeded its 300 s tool wait limit. A separate logged
`matlab -batch` invocation of `runtests('tests')` followed by `assertSuccess`
completed with 618/618 passing. The original MCP call's saved results later
appeared and also show 618/618 passing. Both MAT/CSV results and logs are
preserved; this is 618 distinct tests run twice, not 1,236 distinct tests.

From the repository root, the current implementation can be evaluated with:

```matlab
addpath('scripts');
comparison = runOnlineNrmmTrackingErrorBenchmark( ...
    'Cases',["retained-noise-free","retained-noise", ...
        "varying-noise","retained-noise-25Hz"], ...
    'Seed',71,'MonteCarloRuns',10,'Duration',12);
results = runtests('tests');
assertSuccess(results);
```

To reproduce both sides, export the two synthesis functions from the prior
revision outside the repository, rename their entry points and adapt their
bootstrap paths, then pass `BaselineRuntime=@onlineNrmmTrackingRuntime` and
`BaselineDesign=@baselineNrmmObserverGains`. The actual export and exact
`runComparison.m` driver are preserved locally at
`/home/zai/.cache/collisionAvoidance/estimator-noise-20260916/`.
The current `Cases` option retains the old full case set by default.

Local artifacts include the noise-isolation and metric probes, complete
`comparison.mat`, 62 per-trial sample CSVs, original/config/design JSON,
`independent-audit.json`, test results and logs, `validation.json`, source and
native-kernel hashes, and standalone `noise-reduction.png/.pdf` and
`response-traces.png/.pdf`. Generated plots, binaries and baseline source
exports remain outside Git. Numerical results are committed in
[NRMM_NOISE_REDUCTION_TRIALS_20260916.csv](NRMM_NOISE_REDUCTION_TRIALS_20260916.csv).
