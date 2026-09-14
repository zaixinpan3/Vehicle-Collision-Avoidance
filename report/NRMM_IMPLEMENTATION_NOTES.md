# NRMM implementation notes and recorded experiments

This document contains the implementation analysis and recorded experiments
previously included in the observer theory document. The main derivation is in
[OBSERVER_ISS_THEORY.md](../estimator/OBSERVER_ISS_THEORY.md) and treats the
observer as a continuous-time system. The digital enclosure and experimental
results below are separate from that continuous-time ISS theorem. Historical numerical tables retain their recorded versions; they do not validate
the direct common-gyro redesign. Current implementation descriptions below use
the redesigned core and position-normalized target metric.

## 1. Continuous design followed by digital realization

`synthesizeNrmmObserverGains.m` determines the continuous gains from physical
domains and sensor/model bounds. Its scalar minimum-bandwidth rule and target
Lyapunov/Lipschitz optimization are specified in Section 7 of the theory
document. The design contains no sample period or sample-product constraint.

`onlineNrmmTrackingRuntime.m` subsequently consumes `cfg.runtime.samplePeriod`
and `cfg.runtime.integrationStepMaximum` to construct its integration steps.
The same continuous design can initialize multiple runtime configurations.
Runtime timing must be positive and finite. This separation does not establish
numerical stability for an arbitrary integration step.

The previous scalar objective `max(a/k, Ts*b*k)` and ceiling `1/Ts` were
removed from synthesis. The following recorded experiments predate that
change and retain their original gains and measurements; their numerical
results do not validate the revised scalar gain-selection rule.

The current eight-state body-frame core has no yaw input or yaw chart condition.
`runtime.yawEstimate` is the state of the parallel continuous yaw observer,
integrated by RK4 with `psiHatDot=u+chi*kPsi*wrap(hm-psiHat)`. The correction is
active only for an informative certified course heading; otherwise the observer
propagates the gyro alone. This adds one scalar observer state and never enters
the velocity or target innovation. `output.orientationSet` publishes the separate
union of circular intervals and its validity. `output.egoYaw` is the integrated
observer estimate, and `egoYawErrorBound` encloses the set about that estimate.
Uninformative course data contribute the whole circle; an empty intersection
remains invalid without resetting either the yaw observer or body states.

## 2. Sampling, radar prediction, and scope of the theorem

`onlineNrmmTrackingRuntime.m` receives a synchronized frame at `t`, resets its
output predictors, and advances the continuous observers to `t+Ts` with RK4.
The independent GNSS position predictor follows measured inertial GNSS velocity. The radar
predictor uses its own moving-frame dynamics,

\[
\dot y_P=\hat q-\hat v-u_3Jy_P.
\]

While radar is available its innovation obeys exactly

\[
\dot r=-(K_1I+u_3J)r,\qquad r=y_P-\hat\rho.
\]

Using `-u3*J*rhoHat` in both predictor and observer would omit the rotation of
this residual. The corrected equation is tested against its matrix exponential.
For zero ego rotation, the acceleration injection integrated over one interval
is `(K3/K1)*(1-exp(-K1*Ts))*r(0)`. This explains why excessive bandwidth can
strongly amplify fresh radar noise even with a motion predictor.

The continuous theorem in the theory document assumes bounded-error measurements throughout time.
A noise bound at one sample instant does not bound the error of a held signal
over an interval without additional intersample assumptions. Predictor resets,
intersample errors, dropouts, and RK4 remainders require a separate digital
argument. Section 5 supplies a finite-time position enclosure for this runtime,
including these effects. During radar dropout the correction is removed and the
nominal model coasts; the positive continuous correction decay cannot be claimed
for that interval. Both design and runtime explicitly publish
`sampledImplementationCertified=false`.

Scenario controller margins are separately labeled engineering assumptions.
They are not derived from this continuous observer certificate, and neither the
benchmark nor this document proves closed-loop collision avoidance. The online
position enclosure below is published separately from the configured controller
margins and from a sampled exponential-stability claim.

## 3. Executable verification and reproduction

The behavior tests check the NRMM coordinate correspondence, exact equality of
`Phi_e` and `Phi` on the operating domain, global bounds across saturation
regions, the nonlinear Lyapunov derivative, component ellipsoid factors,
common gyro cancellation, forward-cone floor crossings, standstill operation,
circular set intersections, radar residual rotation, and track reset isolation.
A noisy seed outside the benchmark seed range checks acceleration performance.

From the repository root:

```matlab
addpath('estimator','config','scripts');
results = runtests({'tests/nrmmDirectVelocityTest.m','tests/nrmmYawSetTest.m', ...
    'tests/nrmmStructuredHighGainTest.m', ...
    'tests/observerGainSynthesisTest.m','tests/nrmmCascadeCertificateTest.m', ...
    'tests/nrmmModelFormulationTest.m','tests/nrmmObserverVectorFieldTest.m', ...
    'tests/certifiedKinematicCourseCorrespondenceTest.m', ...
    'tests/onlineNrmmTrackingRuntimeTest.m'});
assertSuccess(results);
runOnlineNrmmComplexManeuverScenario('Plot',false,'NoiseModel','boundedUniform');
benchmark = runOnlineNrmmTrackingErrorBenchmark();
```

A paired historical comparison accepts `BaselineRuntime` and `BaselineDesign`
function handles to independently versioned source exports. Trials share truth,
noise draws, initialization, and sample timing. Continuous ultimate bounds,
empirical RMSE, initial peaks, curvature lag, domain exits, and computation time
are reported separately. Generated results and source exports belong outside
the estimator directory.

## 4. Paired synthetic benchmark

The reference is the previous repository high-gain implementation at commit
`ac20bc4317c373e388b846bcf0f42fc8eb077d61`, not a reproduction of the paper's
full-scale vehicle experiments. Both variants retain the NRMM high-gain cascade.
There are 42 trials: 21 per variant, 12 s duration, 2 s burn-in, bounded-uniform
noise seeds 7--11, one noise-free trial, 50/25 Hz sensing, a changing-motion case,
and a radar dropout from 4 to 5 s. Initial states and sensor draws are identical
within each pair. Noise-free trials retain the same designed gains.

Five-seed mean results:

| Case | Design | Position RMSE (m) | Velocity RMSE (m/s) | Acceleration RMSE (m/s^2) |
|---|---|---:|---:|---:|
| Retained model, 50 Hz | Previous high gain | 0.093528 | 2.7231 | 29.5604 |
| Retained model, 50 Hz | Structured high gain | 0.046948 | 0.48023 | 1.8236 |
| Retained model, 25 Hz | Previous high gain | 0.17934 | 4.8262 | 48.511 |
| Retained model, 25 Hz | Structured high gain | 0.071183 | 0.71372 | 2.6685 |
| Changing motion | Previous high gain | 0.094063 | 2.7576 | 30.1445 |
| Changing motion | Structured high gain | 0.045581 | 0.44295 | 1.6020 |
| One-second dropout | Previous high gain | 2.8549 | 39.6818 | 404.986 |
| One-second dropout | Structured high gain | 0.24014 | 1.4605 | 5.1457 |

Position and acceleration errors use the ego frame; velocity RMSE uses inertial
reconstruction. The default bandwidth changes from 19.011 to 6.4431 /s and the
three gains from approximately `[86.99,2856,32846]` to `[29.48,328.0,1278.7]`.
The sufficient continuous target decay changes from 1.5594 to 0.8000 /s, while
the continuous position ultimate bound decreases from 312.03 to 6.2185 m.
This is a smaller sufficient bound, not a tight error radius for the sampled run.

The mean peak acceleration error during the first 2 s falls from 346.06 to
34.37 m/s^2. Startup peaking remains. Post-burn-in estimates satisfy the full
physical reconstruction domain at about 62.4% of samples in the noisy retained
case; the global extension handles the remaining samples, and the proof does
not require the estimated state to stay inside that physical domain. Noise-free
post-burn-in domain membership is 100%. Mean step time is approximately 2.44 ms
on this host, without a worst-case execution claim.

The changing-motion case declares `|Adot|<=1.5 m/s^3` and
`|kappaDot|<=0.0175 /(m*s)`, with a 55 m range domain. Its best-shift curvature
lag estimate is 0.272 s for the revised observer and 0.644 s for the previous
one; this noisy-signal metric is not an exact phase-delay measurement. The
previous certificate does not include the additional model-jerk forcing.
`DeclaredModelJerkCovered`, `HasRadarDropout`, and `DigitalCertified` in the
trial table make these qualifications explicit. Dropout results are empirical;
the continuous correction theorem is not applied to missing-measurement intervals.

## 5. Recursively updated deterministic position enclosure

`nrmmPositionErrorBound.m` augments the existing observer with uncertainty state;
it does not change the point estimates, gains, NRMM model, or cascade. At the
published state timestamp it supplies

\[
\boxed{\|\rho(t)-\hat\rho(t)\|\le B_\rho(t).}
\]

This is a conditional deterministic norm enclosure, not a covariance, confidence
interval, or empirical RMSE multiplier. Its containment theorem below is in real
arithmetic and includes input holds, predictor resets, radar dropout, and the
defect of the accepted numerical trajectory. The MATLAB implementation uses
ordinary matrix exponentials and explicit roundoff guards; it is not an
interval-arithmetic verification of floating-point operations or the gain LMI.
`floatingPointVerified=false` states this distinction. Nor does finite-time
containment prove uniform exponential stability of the hybrid observer.

The intersample predictor approach has an established sampled-observer context:
Karafyllis and Kravaris study continuous observers coupled to output predictors
and conditions on sampling for inherited robustness
([author preprint](https://arxiv.org/abs/0801.4824)). The specific enclosure here
follows from the inequalities below; their stability theorem is not assumed to
apply automatically to this implementation.

### 5.1 Comparison initialization

The continuous core comparison has two states `[bv,WT]`, with the matrix and
forcing in Section 6 of the theory document. Its affine flow, including initial
error, is evaluated by an augmented matrix exponential; the steady-state bound
alone cannot enclose an arbitrary initial condition. Use
`D=diag(1,1/wT,1/wT^2)` for all target metric conversions.

The default true prior is the whole yaw circle, `bv=VEmax+norm(vHat)`, and
`bT=[R0+norm(rhoHat);VCmax+norm(qHat);S+norm(sHat)]`.
Then `WT <= sqrt((D*bT)'*abs(P)*(D*bT))`. Smaller known initial errors can be
supplied as `options.initialErrorBounds.yaw`, `bodyVelocity`, and
`targetComponents` (3-by-target-count). The yaw prior is centered at the optional
`egoInitialYaw`; if omitted, the whole circle needs no initial heading.
These are premises on truth, not deductions from the estimates.

An independent range enclosure advances by
`Rplus=R+(VEmax+VCmax)*h` and is tightened at detections by `norm(yR)+nR`.
Thus the sampled bound does not keep imposing an initial range on a moving
unobserved target. True target speed, acceleration, curvature and model-jerk
bounds, the ego forward cone and mismatch, and the sensor bounds remain premises.
Estimated-domain exits do not invalidate the global extension.

### 5.2 The additional predictor-error state

The evolving radar reference is not a measurement with fixed error `nR`.
Introduce `bP >= norm(rho-yP)`. Its derivative bound, including numerical defect,
is `bPdot <= bq+bv+R*epsilonOmega+etaP`.
During a detection interval the comparison state is `zA=[bv,WT,bP]'`, with

\[
G_A=\begin{bmatrix}-k_v&0&0\\c_{Tv}&-\lambda_T&c_{T\rho}\\1&c_2&0\end{bmatrix},
\quad d_A=\begin{bmatrix}
d_v^h+\eta_v\\c_{T\omega}(R)\epsilon_\omega+c_{Tj}\varepsilon_j+\eta_T\\
R\epsilon_\omega+\eta_P
\end{bmatrix}.
\]

Here `c2=wT*sqrt(inv(P)(2,2))`,
`etaT=sqrt((D*etaTarget)'*abs(P)*(D*etaTarget))`, and the target gyro coefficient
uses the propagated range. For a constant upper forcing over the substep,

\[
\begin{bmatrix}z_A^+\\1\end{bmatrix}
=\exp\left(h\begin{bmatrix}G_A&d_A\\0&0\end{bmatrix}\right)
\begin{bmatrix}z_A\\1\end{bmatrix}.
\]

All off-diagonal entries are nonnegative. The predictor feedback loop means
`GA` need not be Hurwitz. Its third state is reset at detections. Finite-time
containment does not establish uniform contraction of the hybrid reset/flow
products or uniform sampled exponential stability.

### 5.3 Held measurements and separate orientation propagation

Let `age` denote the largest measurement age on a substep. Optional physical
envelopes under `cfg.ego.domain` are `Ea=accelerationNormMaximum`,
`Ja=bodyAccelerationRateMaximum` and `Jomega=yawAccelerationMaximum`.
The second bounds the derivative of the body-frame acceleration vector. They
all default to `Inf`, meaning unspecified. Valid effective error bounds are

\[
\begin{aligned}
\epsilon_\omega&=\min(\bar\omega_E+|u_k|,\varepsilon_\omega+J_\omega\,\mathrm{age}),\\
E_a^h&=\min(E_a,\|a_{m,k}\|+\varepsilon_a+J_a\,\mathrm{age}),\\
\epsilon_a&=\min(E_a+\|a_{m,k}\|,\varepsilon_a+J_a\,\mathrm{age}),\\
\epsilon_V&=\min(\bar V_E+\|m_{I,k}\|,\varepsilon_G+E_a^h\,\mathrm{age}).
\end{aligned}
\]

An unspecified envelope removes its candidate. The magnitude error is at most
the GNSS vector hold error. With held `M_k` and `u_k`, the exact common-error
identity still holds using `nOmegaEffective=u_k-omegaE(t)` and
`nuEffective=M_k-norm(v(t))`. Consequently

\[
d_v^h=\epsilon_a+k_v\sec b\,\epsilon_V
 +C_\omega(k_v)\epsilon_\omega+k_vl_E\sec b\,\delta_{\rm st}.
\]

The same effective gyro error is combined before bounding; it is never charged
as two unrelated inputs. If no finite acceleration envelope exists, the runtime
uses the uniform velocity cap
`VEmax+max(norm(vHatBefore),norm(vHatAfter))`, sets the velocity comparison row
to zero, and initializes that row to the cap for the substep. It never treats
a held acceleration as exact or integrates an infinite forcing.

With a finite envelope, a velocity-path cap is the minimum of the domain cap
and `max(bv,(dvHold+etaV)/kv)`. Together with the target-velocity path cap this
bounds `bP+h*(bqPath+bvPath+R*epsilonOmega+etaP)`; an independently valid cap can
only tighten the comparison endpoint.

The true orientation set is propagated outside that comparison by a circular Minkowski sum with
`u_k*h` and radius `epsilonOmega*h`. This encloses the integral error even when
only the true rate domain is available. The sensor radius alone would require
continuous gyro measurements or an additional hold-error premise. At a sample,
intersect with the certified course outer arc, using the whole circle when that
arc is uninformative. The interval representation retains disconnected pieces.
A failed intersection stays empty; only orientation-dependent outputs lose their
certificate. The parallel yaw observer is integrated with RK4 using the held
course heading and gyro. Neither a measurement intersection nor an output query
resets its state. At every published time the yaw error radius is the maximum
circular distance from the actual observer estimate to the orientation set,
computed by `nrmmYawSet("radiusAbout",...)`. A change in the yaw estimate never
rotates or resets body-frame states.

### 5.4 Numerical trajectory defects

For an accepted RK4 step, use the linear reconstruction between actual numerical
endpoints. Its defect is `(x1-x0)/h-F(x0+theta*(x1-x0),u_k)`, `0<=theta<=1`.
An affine row with endpoint field increment `DeltaF` has the uniform bound

\[
\eta=\|(x_1-x_0)/h-F(x_0)-\tfrac12\Delta F\|+\tfrac12\|\Delta F\|.
\]

This now applies directly to body velocity, target position/velocity, and the
radar predictor. For body velocity,
`DeltaF=(-kv*I-u_k*J)*(vHatAfter-vHatBefore)`; there is no yaw-dependent remainder.
For the final target row apply the expression to its linear part and add
`Lq*norm(deltaQ)+Ls*norm(deltaS)`. The global extension covers saturation crossings.
No yaw numerical defect enters this comparison: the independent orientation
set is enclosed about the actual numerical yaw endpoint. Its radius includes
any discrepancy of that endpoint, including integration error. The set itself
is propagated with the declared gyro integral-error bound.

These derivative defects enter the comparison forcing. No assumed fifth
measurement derivative, RK4 remainder constant, or comparison between two
numerical trajectories is needed. Ordinary floating-point arithmetic has a
`256*eps` engineering guard; it is not a directed-rounding verification of matrix
exponentials, trigonometric functions, or gain synthesis.

### 5.5 Measurement tightening, components and dropout

At a detection, `yP=yR` resets `bP` to the radar error bound and permits
`bRho=min(bRho,norm(yR-rhoHat)+nR)`. For the body-velocity measurement, the
finite-error identity gives the sample-only bound

\[
E_y=\sec b\,\varepsilon_G+l_E\sec b(\varepsilon_\omega+\delta_{\rm st}),
\qquad b_v^+\le\min(b_v^-,\|y_v-\hat v\|+E_y).
\]

This scalar is used for measurement/prior containment only. It is never fed into
the propagation forcing as an independent gyro disturbance or obtained through
a geometric enclosing-ball construction.

Target components can be intersected with their norm-domain caps. Their metric
bound is `sqrt((D*bT)'*abs(P)*(D*bT))`. At a radar-active endpoint the position
radius is `min(c1*Wcomp,bP+norm(yP-rhoHat),R+norm(rhoHat))`, where
`c1=sqrt(inv(P)(1,1))`. The capped components then recertify the metric for the
next substep; no fixed-point iteration is used.

During dropout use `[bv,bRho,bq,bs,bP]'` and retain the applicable velocity row.
The remaining positive comparison rows are

\[
\begin{aligned}
\dot b_\rho&=b_q+b_v+R\epsilon_\omega+\eta_\rho,\\
\dot b_q&=b_s+\bar V_T\epsilon_\omega+\eta_q,\\
\dot b_s&=L_qb_q+L_sb_s+\bar a_T\epsilon_\omega+\varepsilon_j+\eta_s,\\
\dot b_P&=b_q+b_v+R\epsilon_\omega+\eta_P.
\end{aligned}
\]

There is no target decay claim with correction absent. The same augmented
exponential propagates these bounds; component-to-metric conversion preserves
containment on reacquisition.

**Containment proposition.** Valid initial sets, true sensor/model/hold envelopes
and numerical-defect bounds imply the positive comparison dominates the actual
error on each substep. Independent measurement/domain caps also contain truth,
so taking their minima preserves domination. Radar resets are enclosed by their
sample noise balls. Induction over substeps, detections and dropouts gives
finite-time position containment. It does not assert useful radii for every
signal or uniform hybrid exponential stability.

An empty `C_m` or another detected ego measurement/prior inconsistency invalidates
the body bound and all dependent target bounds. A radar inconsistency affects
only its track. A yaw-prior intersection failure affects orientation outputs
without invalidating an otherwise consistent body-frame cascade. Invalid sets
are never silently reset; a target reset cannot repair an invalid ego bound.
These necessary checks do not verify every premise on the unknown true motion.

### 5.6 Interface for subsequent collision-avoidance control

`output.targetEstimates(i).relativePositionErrorBound` is a scalar Euclidean
radius in the ego body frame at `stateTime`. The accompanying `positionErrorBound`
structure carries `time`, `frame`, `available`, `reason`, and scope flags.
The runtime's `step` consumes the sample at `t` and publishes the radius for the
predicted state at `t+Ts`; it must not be relabeled as a bound at the sample time.
The read-only `output` action may tighten a bound with the supplied current
sample without modifying the stored runtime. Stale bound/measurement times are
rejected. The adapter passes these fields through with the raw target estimate.

```matlab
[runtime, output] = onlineNrmmTrackingRuntime('step', runtime, frame);
track = output.targetEstimates(1);
radiusM = track.relativePositionErrorBound;
boundTime = track.positionErrorBound.time; % equals track.stateTime
usableUnderDeclaredAssumptions = track.positionErrorBound.available;
```

For an inertial *relative vector*, use the separately published bound

\[
B_{\Delta p}^I=B_\rho+2\min(R,\|\hat\rho\|)\sin(\min(b_\psi,\pi)/2),
\quad \|p_C-p_E-\hat R\hat\rho\|\le B_{\Delta p}^I.
\]

An absolute target-position bound additionally needs an ego-position bound.
For center-distance constraints, `norm(rhoHat)-Brho` is a lower bound on actual
center distance without any absolute-yaw reconstruction. Rectangle footprint,
orientation, road, ego tracking, and future target-prediction uncertainty remain
separate controller obligations. In particular, today's `Brho` cannot be copied
unchanged across the controller prediction horizon. The runtime states
`futurePredictionIncluded=false`, and the existing configured controller margins
are not relabeled as this certificate.

The synthetic adapter's two-dimensional noise was corrected to use the same
inscribed-square construction as the research scenario: each independent
uniform component is scaled by `1/sqrt(2)`. Thus its declared vector maxima are
actually Euclidean radii. Its interpolated ego-state/sampled-acceleration harness
is not a validation against a continuous high-fidelity physical sensor stream.

### 5.7 Historical containment experiment

The recorded table below predates the direct common-gyro redesign and retains
its original numerical results. It is not a validation of the current code.

`scripts/runNrmmPositionBoundBenchmark.m` executes 32 trials: three seeds
(71--73), five noisy cases and two envelope choices, plus one noise-free
realization for each choice. Duration is 12 s; the mean and maximum radii below
exclude the first 2 s, while containment is checked at **every** recorded state,
including initialization. The paired truth, sensor arrays, ego estimates and
target estimates are verified bit-identical across envelope choices.

The default domain-only variant leaves all optional rates unspecified. The
experiment's finite variant declares ego acceleration norm at most 5 m/s^2,
body-acceleration rate at most 5 m/s^3, and yaw acceleration at most 0.1 rad/s^2.
These are synthetic trajectory premises, not calibrated vehicle limits. For the
analytic ego maneuver, `V>=10.8`, `|Vdot|<=0.3`, `|Vddot|<=0.075`,
`|omega|<=0.12`, `|omegaDot|<=0.042`, and `|omegaDdot|<=0.0147` give conservative
all-time acceleration and body-acceleration-rate envelopes below 2 and 1.3,
respectively. The script also checks the sampled analytic acceleration, inertial
jerk transformed to body-acceleration rate, yaw acceleration, and target domain.

| Case | Mean radius: domain only (m) | Mean radius: finite ego envelopes (m) | Mean of maximum radii: finite envelopes (m) |
|---|---:|---:|---:|
| Retained NRMM, 50 Hz | 1.6942 | 0.8323 | 0.8951 |
| Changing A and curvature | 1.7461 | 0.8549 | 0.9566 |
| Radar dropout, 4--5 s | 5.9299 | 2.5283 | 34.172 |
| One 0.02 s RK4 step per sample | 1.7016 | 0.8411 | 0.9104 |
| Retained NRMM, 25 Hz | 3.2148 | 1.4892 | 1.5553 |
| Noise-free realization, declared noise bounds retained | 1.6493 | 0.7875 | 0.7962 |

Every tested timestamp had an available bound and satisfied containment in all
32 trials. This experiment is validation evidence for the implementation; the
deterministic claim derives from the stated premises and comparison argument.
The 50 Hz noisy point-estimate RMSE is 0.04785 m, far smaller than the radius.
The domain-only dropout maximum averages 87.69 m; even the finite-envelope
dropout bound is too large for many road geometries. These losses of usefulness
must be visible to the controller. Noise-free measurements do not justify
removing the declared sensor uncertainty from the bound. MATLAB timing is
observational, without a worst-case real-time claim.

The final focused validation contains 50 passing unique MATLAB tests: 15 for
the new bound, 23 existing runtime tests, eight structured high-gain tests and
four existing adapter contracts. Coverage includes large initial errors with
high-gain peaking, the yaw chart boundary, stale timestamps, isolated track
invalidation/reset, the entire numerical reconstruction, and vector-noise norms.
All eight changed MATLAB files have zero Code Analyzer issues under factory
settings. No whole-repository or closed-loop safety pass is claimed.
