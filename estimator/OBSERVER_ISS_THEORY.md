# Multistage high-gain NRMM observer: model, design, and stability

The estimator retains the framework of Sharma, Alai, and Rajamani,
*Simultaneous ego-vehicle state estimation and vehicle trajectory tracking using
a multistage high gain observer*, Transportation Research Part C 182 (2026),
105411, [DOI](https://doi.org/10.1016/j.trc.2025.105411). The relevant parts are
Section 2.3, Section 3.3, Eq. (65), and Theorem 1, Eqs. (30)--(33).

The online architecture is the ego yaw observer, the ego body-velocity observer,
and the third-order target observer. Inertial position is a downstream output
stage. The target state remains `[rho; q; s]`; its gains remain
`[l1*w; l2*w^2; l3*w^3]`. The improvements are analytic global Lipschitz bounds,
a structured Lyapunov inequality, and a noise-aware choice of bandwidth.

## 1. Retained model and covariant chain

Let `J = [0,-1;1,0]`, `R(psi)` denote a planar rotation, and

\[
\nabla_u z=\dot z+uJz,\qquad
\rho=R_E^T(p_C-p_E),\quad q=R_E^Tv_C,\quad s=R_E^Ta_C.
\]

The Sharma target model has constant scalar acceleration `A` and constant
sideslip `betaC`, with positive speed. Equivalently its curvature
`kappa = sin(betaC)/lrC` is constant. This observation is used to derive the
vector field; `A` and `kappa` are not replacement online estimation states.
The exact transformed equations are

\[
\nabla_{\omega_E}\rho=q-v_E,\qquad
\nabla_{\omega_E}q=s,\qquad \nabla_{\omega_E}s=\Phi(q,s),
\]
\[
V=\|q\|,\quad A=\frac{q^Ts}{V},\quad
\Omega=\frac{(Jq)^Ts}{V^2},\qquad
\Phi=-\Omega^2q+3A\Omega J\frac qV.
\]

These are the same NRMM dynamics in physical, covariant companion coordinates.
For example, `q = rhoDot + vE + omegaE*J*rho`. Using this chain avoids
repeated differentiation of measured ego inputs. Ego jerk and angular
acceleration need not vanish. The dynamics and their saturation extension in
`nrmmTargetTrackerDerivative.m` have not been replaced by another motion model.

Truth must remain in the declared positive-speed operating domain:
`a <= |q| <= b`, `|A| <= Amax`, `|Omega| <= Omax`, `|s| <= S`, and
`|rho| <= rhoMax`, where

\[
O_{\max}=b\sin(\beta_{\max})/l_r,\qquad
S=\sqrt{A_{\max}^2+(bO_{\max})^2}.
\]

The physical heading reconstruction is not observable at zero target speed.
Estimated states may peak outside this domain: the globally Lipschitz extension
below keeps the observer defined. Estimated-domain flags report such exits;
they are not substitutes for checking the assumptions on truth.

## 2. Global Lipschitz bounds for the existing extension

Write `r = |q|`, `d = max(r,a)`, `Q = projection_ball(b,q)`,
`Svec = projection_ball(S,s)`, and `e = q/d`. The implemented extension is

\[
\alpha=\operatorname{clip}_{A_{\max}}(Q^TS_{\rm vec}/d),\quad
\nu=\operatorname{clip}_{O_{\max}}((JQ)^TS_{\rm vec}/d^2),
\qquad \Phi_e=-\nu^2Q+3\alpha\nu Je.
\]

It agrees with `Phi` on the true operating domain and is defined on all of
`R^4`. Ball projections and scalar clips are nonexpansive. Their almost-everywhere
Jacobians suffice because the maps are continuous and piecewise differentiable.
Bounds are integrated along the whole line segment in the extended space;
no convexity of the positive-speed annulus is presumed.

For fixed `s`, differentiation in the radial/tangential basis gives:

| Radial region | Bound on `|Dq alpha|` | Bound on `|Dq nu|` | Bound on `|Q| |Dq nu|` |
|---|---:|---:|---:|
| `r <= a` | `S/a` | `S/a^2` | `S/a` |
| `a <= r <= b` | `S/r` | `S/r^2` | `S/r` |
| `r >= b` | `b*S/r^2` | `2*b*S/r^3` | `2*b^2*S/r^3` |

Clipping only decreases these derivative bounds. Consequently, with

\[
c_A=S/a,\quad c_\Omega=S\max(a^{-2},2b^{-2}),\quad
c_{Q\Omega}=S\max(a^{-1},2b^{-1}),
\]
\[
L_q=O_{\max}^2+2O_{\max}c_{Q\Omega}
 +3A_{\max}O_{\max}/a+3O_{\max}c_A+3A_{\max}c_\Omega.
\]

For fixed `q`, retain the acceleration Jacobian's directional structure.
In the basis aligned with `q`, its entrywise absolute values before the
nonexpansive acceleration projection are bounded by

\[
B_s=\begin{bmatrix}0&2O_{\max}\\3O_{\max}&3A_{\max}/a\end{bmatrix},
\qquad L_s=\|B_s\|_2.
\]

At `q=0` the same bound follows by continuity. Thus, globally,

\[
\boxed{\|\Phi_e(q,s)-\Phi_e(\hat q,\hat s)\|
 \le L_q\|q-\hat q\|+L_s\|s-\hat s\|.}
\]

`nrmmTargetLipschitzCertificate.m` evaluates these analytic expressions.
A sampled Jacobian maximum is not used as the proof. The reconstructed
`Omega(q,s)` is differentiated, rather than frozen as a constant in this bound.
The legacy scalar `phi = hypot(Lq,Ls)` is only a diagnostic in numerical SI
coordinates; the physical certificate below uses the separate channels.

## 3. Observer equations and the yaw chart condition

The continuous measurement model is
`u3 = omegaE + dOmega`, `am = aE + na`,
`vm = R(psiE)*vE + nv`, and `yR = rho + nR`, with the declared uniform norm
bounds. The corrected GNSS-course correspondence supplies `yF` with a certified
circular error radius `BF`. Its construction uses the declared single-track
mismatch bound; it is not an additional zero-sideslip assumption.

The retained ego observers are

\[
\dot{\hat\psi}=u_3+k_\psi\operatorname{wrap}(y_F-\hat\psi),
\qquad \nabla_{u_3}\hat v=a_m+k_v(R(\hat\psi)^Tv_m-\hat v).
\]

Let `ePsi = psiE - psiHat` be a compatible lift and `dF = yF - psiE`.
The linear comparison for yaw requires `|ePsi| + BF < pi`. A sufficient invariant
condition is

\[
B_\psi=\max\{|e_\psi(0)|,B_F+\bar d_\omega/k_\psi\},
\qquad B_\psi+B_F<\pi.
\]

Inside this chart, `wrap(ePsi+dF)=ePsi+dF`, so the scalar comparison and a
first-exit argument preserve the condition. A proper measurement arc alone is
insufficient: `ePsi=3`, `dF=0.2`, `kPsi=1` yields a positive derivative of
`|ePsi|`, contradicting an unrestricted linear-decay claim. The runtime's
`yawInnovationChartCompatible` flag is only a pointwise diagnostic; it does not
establish chart invariance or an initial true-error bound.

The target observer is exactly the third-order high-gain chain

\[
\begin{aligned}
\nabla_{u_3}\hat\rho&=\hat q-\hat v+l_1\omega(y_R-\hat\rho),\\
\nabla_{u_3}\hat q&=\hat s+l_2\omega^2(y_R-\hat\rho),\\
\nabla_{u_3}\hat s&=\Phi_e(\hat q,\hat s)+l_3\omega^3(y_R-\hat\rho).
\end{aligned}
\]

There is no extra filter between these estimates and the published acceleration.

## 4. Structured target Lyapunov inequality

Let `Ac` be the three-state integrator-chain matrix, `C=e1^T`, and
`Al=Ac-l*C`. The normalized Sharma-type observer LMI selects `l`. Its existing
normalized pole region and bounded-real noise objective are retained. The
identity `Al^T*P + P*Al = -I` fixes a positive definite, dimensionless metric.

For true-minus-estimated errors use

\[
\epsilon=\operatorname{diag}(\omega^2I_2,\omega I_2,I_2)e_T,
\qquad W_T=\sqrt{\epsilon^T(P\otimes I_2)\epsilon}.
\]

All components of `epsilon` have acceleration units. The common rotation
`-u3*(I3 tensor J)` contributes exactly zero to the quadratic derivative.
Split `DeltaPhi = DeltaPhiq + DeltaPhis` by changing `q` first and then `s`.
The two global bounds become

\[
\|\Delta\Phi_q\|\le(L_q/\omega)\|\epsilon_2\|,\qquad
\|\Delta\Phi_s\|\le L_s\|\epsilon_3\|.
\]

Set `bP=P*e3`. For positive Young multipliers `tq,ts`, define

\[
M=\omega I-t_q(L_q/\omega)^2e_2e_2^T-t_sL_s^2e_3e_3^T
 -(t_q^{-1}+t_s^{-1})b_Pb_P^T.
\]

Twice applying `2 z^T d <= |z|^2/t + t |d|^2` gives

\[
\boxed{M\succeq 2\lambda_TP\quad\Longrightarrow\quad
 \dot W_T\le-\lambda_TW_T\quad\text{without exogenous error inputs}.}
\]

This is a Lyapunov/Lipschitz high-gain proof. It preserves the absence of
position from `Phi` and the `1/omega` scaling of its velocity sensitivity.
`synthesizeTargetTrackerCertificate.m` numerically searches the two positive
multipliers, recovers the generalized-eigenvalue decay rate, subtracts a small
numerical margin, and checks the final dissipation matrix. Feasibility of that
matrix is the certificate; optimizer termination does not prove global
optimality. Ordinary floating-point checks are not a directed-rounding proof.

## 5. Disturbance directions and cascade ISS

If the nominal constant-`A,kappa` model is relaxed, declare rate bounds. The
additional physical jerk is `Adot*e + kappaDot*V^2*J*e`, so

\[
\bar\nu_\Phi=\sqrt{\bar{\dot A}^{\,2}+b^4\bar{\dot\kappa}^{\,2}}.
\]

Both rates default to zero. They enter as uncertainty in the last chain
equation and do not change the nominal observer dynamics.

Define `h = [omega^2*rhoMax, omega*b, S]^T`. Cauchy--Schwarz in the actual metric
gives the directional coefficients

\[
g_{Tv}=\omega^2\sqrt{P_{11}},\quad
g_R=\omega^3\sqrt{l^TPl},\quad
g_\omega=\sqrt{h^T|P|h},\quad g_\Phi=\sqrt{P_{33}}.
\]

Here `|P|` is entrywise absolute value. The target comparison is

\[
D^+W_T\le-\lambda_TW_T+g_{Tv}\|e_v\|
 +g_\omega\bar d_\omega+g_R\bar n_R+g_\Phi\bar\nu_\Phi.
\]

Under the yaw chart condition, the full retained cascade satisfies

\[
D^+\begin{bmatrix}|e_\psi|\\\|e_v\|\\W_T\end{bmatrix}
\le H\begin{bmatrix}|e_\psi|\\\|e_v\|\\W_T\end{bmatrix}+d,
\quad H=\begin{bmatrix}
-k_\psi&0&0\\k_v\bar V_E&-k_v&0\\0&g_{Tv}&-\lambda_T
\end{bmatrix},
\]
\[
d=\begin{bmatrix}
\bar d_\omega+k_\psi B_F\\
\bar n_a+\bar V_E\bar d_\omega+k_v\bar n_v\\
g_\omega\bar d_\omega+g_R\bar n_R+g_\Phi\bar\nu_\Phi
\end{bmatrix}.
\]

`H` is Hurwitz and Metzler. The positive vector `c=-H^{-T}*ones(3,1)` gives
`c^T H=-ones(1,3)`. Therefore `Vc=c^T[|ePsi|,|ev|,WT]^T` is a copositive
cascade Lyapunov function with an explicit ISS comparison. This is implemented
in `nrmmObserverCertificate.m`. The ultimate comparison vector is `-H^{-1}d`.

Component extraction uses the tight ellipsoid factors

\[
\|e_{T,i}\|\le c_iW_T,\qquad
(c_1,c_2,c_3)^T=\operatorname{diag}(\omega^{-2},\omega^{-1},1)
 \sqrt{\operatorname{diag}(P^{-1})}.
\]

The square root is componentwise. This avoids using the worst eigenvalue for
every component and the worst metric direction for every disturbance.
Inertial position remains downstream:
`|ep|_infinity <= np + (|ev|_infinity + VEmax*|ePsi|_infinity)/kp`.
The ego-frame acceleration-difference error is bounded by the target
acceleration error plus accelerometer error, with no velocity term added to
an acceleration quantity.

## 6. Explicit gain-selection preference

The normalized gain shape is unchanged. The physical bandwidth minimizes the
largest ultimate error bound divided by its corresponding declared physical
scale, subject to `lambdaT >= 1/Tdomain` and `omega > 1/s`, where
`Tdomain=rhoMax/(VEmax+VCmax)`. This is an engineering preference for disturbance
rejection with a minimum decay rate. The ISS theorem does not force this
objective or the transit-time convention. Numerical minimization is not claimed
to be globally optimal. The conservative transient formula
`WT(t) <= exp(-lambdaT*t)*WT(0) + WT_infinity` remains available; a full-domain
initial-error bound is not mixed into the persistent-noise objective.

The yaw, velocity, and position gains retain their scalar objective
`max(a/k, Ts*b*k)` on `[1/Tdomain,1/Ts]`. This objective balances process-error
leakage and innovation displacement. It is likewise a stated design preference,
not an optimum implied by the continuous ultimate bound `b+a/k`.

## 7. Sampling, radar prediction, and scope of the theorem

`onlineNrmmTrackingRuntime.m` receives a synchronized frame at `t`, resets its
output predictors, and advances the continuous observers to `t+Ts` with RK4.
The GNSS position predictor follows estimated inertial velocity. The radar
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

The continuous theorem assumes bounded-error measurements throughout time.
A noise bound at one sample instant does not bound the error of a held signal
over an interval without additional intersample assumptions. Predictor resets,
intersample errors, dropouts, and RK4 remainders require a separate digital
argument. Section 10 supplies a finite-time position enclosure for this runtime,
including these effects. During radar dropout the correction is removed and the
nominal model coasts; the positive continuous correction decay cannot be claimed
for that interval. Both design and runtime explicitly publish
`sampledImplementationCertified=false`.

Scenario controller margins are separately labeled engineering assumptions.
They are not derived from this continuous observer certificate, and neither the
benchmark nor this document proves closed-loop collision avoidance. The online
position enclosure below is published separately from the configured controller
margins and from a sampled exponential-stability claim.

## 8. Executable verification and reproduction

The behavior tests check the NRMM coordinate correspondence, exact equality of
`Phi_e` and `Phi` on the operating domain, global bounds across saturation
regions, the nonlinear Lyapunov derivative, component ellipsoid factors,
wrapped-yaw limitations, radar residual rotation, and track reset isolation.
A noisy seed outside the benchmark seed range checks acceleration performance.

From the repository root:

```matlab
addpath('estimator','config','scripts');
results = runtests({'tests/nrmmStructuredHighGainTest.m', ...
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

## 9. Paired synthetic benchmark

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

## 10. Recursively updated deterministic position enclosure

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

### 10.1 Continuous comparison recursion and initialization

For the continuous measurement model of Section 5, initialize a nonnegative
vector `z(0) >= [|ePsi(0)|, |ev(0)|, WT(0)]^T`. On an interval of length `h`
with constant disturbance upper bound `d`, positivity gives

\[
z^+=e^{Hh}z+\left(\int_0^h e^{H\tau}\,d\tau\right)d,
\qquad B_\rho=c_1z_3,
\quad c_1=\omega^{-2}\sqrt{(P^{-1})_{11}}.
\]

The integral must be retained: the steady-state number `-H^{-1}d` by itself
does not cover an arbitrary initialization. An augmented matrix exponential
evaluates this affine update without inverting `H`, including zero-decay modes.

The runtime defaults to the declared physical prior, not to zero error:

\[
b_\psi=\pi,\quad b_v=\bar V_E+\|\hat v\|,\quad
b_T=\begin{bmatrix}R_0+\|\hat\rho\|\\b+\|\hat q\|\\S+\|\hat s\|\end{bmatrix},
\qquad W_T\le\sqrt{(Db_T)^T|P|(Db_T)},
\quad D=\operatorname{diag}(\omega^2,\omega,1).
\]

Here `R0` is the declared initial relative-position maximum. A caller may supply
smaller *known initial error bounds* through `options.initialErrorBounds`, with
fields `yaw`, `bodyVelocity`, and `targetComponents` (3-by-target-count). That is
an additional initial-set premise, not a deduction from an estimated state.

An independent true-range enclosure evolves as
`R^+ = R + (VEmax+VCmax)*h`, since the rotation term vanishes from the derivative
of `|rho|`. At detection it can be intersected with `|yR|+nR`. Thus the online
argument does not keep imposing the initial 50 m range on a coasting target.
Positive target speed, target speed/acceleration/curvature bounds, the declared
model-jerk envelope, ego speed/yaw-rate bounds, sensor calibration and noise
bounds remain premises on truth. Estimated-domain exits do not invalidate this
global-extension argument.

### 10.2 The additional predictor-error state

The continuous theorem cannot use `nR` as the error of the evolving radar
predictor. Introduce `bP >= |rho-yP|`. With a numerical predictor defect `etaP`,

\[
D^+\|\rho-y_P\|\le\|e_q\|+\|e_v\|+R\epsilon_\omega+\eta_P.
\]

During a detection interval use `zA=[bPsi,bv,WT,bP]^T`. On each integration
substep, choose uniform input-error and numerical-defect bounds. In a compatible
yaw chart the augmented comparison matrix and forcing are

\[
G_A=\begin{bmatrix}
-k_\psi&0&0&0\\
k_v\bar V_E&-k_v&0&0\\
0&g_{Tv}&-\lambda_T&g_R\\
0&1&c_2&0
\end{bmatrix},
\qquad
d_A=\begin{bmatrix}
\epsilon_\omega+k_\psi B_F^h+\eta_\psi\\
\epsilon_a+\bar V_E\epsilon_\omega+k_v\epsilon_V+\eta_v\\
g_\omega(R)\epsilon_\omega+g_\Phi\bar\nu_\Phi+\eta_T\\
R\epsilon_\omega+\eta_P
\end{bmatrix},
\]

where `c2` is the velocity component factor from Section 5,
`etaT=sqrt((D*etaTarget)^T*abs(P)*(D*etaTarget))`, and `gOmega(R)` uses the
propagated range upper bound rather than a fixed range. All off-diagonal entries
are nonnegative. Therefore the actual implemented comparison update is

\[
\boxed{\begin{bmatrix}z_A^+\\1\end{bmatrix}
=\exp\!\left(h\begin{bmatrix}G_A&d_A\\0&0\end{bmatrix}\right)
\begin{bmatrix}z_A\\1\end{bmatrix}.}
\]

The predictor feedback loop means `GA` is not assumed Hurwitz. Its fourth state
is reset at detections. Establishing a uniform contraction of the resulting
reset/flow products would be a further sampled-stability result.

### 10.3 Held measurements and yaw charts

Let `a` now denote measurement age (only in this subsection). Optional physical
envelopes are `Ea=accelerationNormMaximum`,
`Ja=bodyAccelerationRateMaximum`, and `Jomega=yawAccelerationMaximum` under
`cfg.ego.domain`. The acceleration rate is the derivative of the *body-frame
acceleration vector*. All three default to `Inf`, meaning unspecified.

At a substep's maximum age, valid hold-error envelopes are

\[
\begin{aligned}
\epsilon_\omega&=\min\{\bar\omega_E+|u_k|,\bar n_g+J_\omega a\},\\
E_a^h&=\min\{E_a,\|a_{m,k}\|+\bar n_a+J_a a\},\\
\epsilon_a&=\min\{E_a+\|a_{m,k}\|,\bar n_a+J_a a\},\\
\epsilon_V&=\min\{\bar V_E+\|v_{m,k}\|,\bar n_v+E_a^h a\},\\
B_F^h&=\min\{\pi,B_{F,k}+\bar\omega_E a\}.
\end{aligned}
\]

An unspecified rate removes its candidate from the minimum. If no finite
acceleration envelope is available, the implementation does not integrate an
infinite velocity forcing or assume a held accelerometer is exact. It replaces
the second comparison state over that substep by the uniform bound
`VEmax + max(norm(vHatBefore),norm(vHatAfter))`; its comparison row is zero.
This remains finite using the existing ego speed premise alone.

For the line segment joining accepted yaw states, a uniform circular-error
bound is `bPsi + omegaEmax*h + abs(deltaPsiHat)`. If this plus `BFh` is below
`pi`, the stable yaw row above is valid throughout the substep. Otherwise use
the first comparison row zero and forcing
`omegaEmax + abs(deltaPsiHat)/h`, and cap its endpoint radius at `pi`. This
fallback uses circular distance and does not assert an invalid linear-decay law
near the antipodal chart boundary.

With a finite acceleration envelope, a tighter uniform velocity bound is

\[
\bar b_v^h=\min\left\{\bar V_E+\max(\|\hat v_0\|,\|\hat v_1\|),\,
\max\left(b_v,\bar V_E\min(\pi,b_\psi^h)+(d_A)_2/k_v\right)\right\}.
\]

Together with `bqPath = VCmax + max(norm(qHatBefore),norm(qHatAfter))`, it gives
an independent predictor endpoint cap
`bP + h*(bqPath+bvPath+R*epsilonOmega+etaP)`. Taking the smaller of two proven
upper bounds remains valid. No estimator gain is changed by these envelopes.

### 10.4 Numerical trajectory defects

For an accepted numerical step from `x0` to `x1`, define its continuous
reconstruction `xHat(t)=x0+theta*(x1-x0)`, `theta in [0,1]`. It joins the
actual values returned by RK4. Define

\[
\delta_{\rm num}(\theta)=(x_1-x_0)/h-F(x_0+\theta(x_1-x_0),u_k).
\]

The bound update encloses this defect over the *entire* line, rather than
estimating integration error from the difference of two numerical runs.
For an affine row with endpoint field increment `DeltaF`, its norm is bounded by

\[
\eta=\|(x_1-x_0)/h-F(x_0)-\tfrac12\Delta F\|+\tfrac12\|\Delta F\|.
\]

This applies to target position, velocity and radar prediction. For the final
target row use the same expression for its linear part and add
`Lq*norm(deltaQ)+Ls*norm(deltaS)`. The existing global extension bounds justify
that remainder even when the line crosses saturation boundaries. For body
velocity, use its initial residual plus
`(|u|+kv)*norm(deltaV)+kv*norm(vm)*min(2,abs(deltaPsi))`.
For yaw use its initial residual plus `kPsi*abs(deltaPsi)` if the innovation
does not cross a branch boundary, or `2*pi*kPsi` otherwise.

These are derivative-defect bounds. Their effects are integrated through the
comparison system as `etaPsi`, `etav`, `etaTarget`, and `etaP`. Thus no
unprovided fifth-derivative bound, assumed RK4 remainder constant, or convergence
test is required. A finer reconstruction can reduce conservatism in later work.
Ordinary floating-point evaluation is padded by a scale-dependent `256*eps`
guard. This guard is explicitly an engineering allowance, not a proof of every
rounding error in `expm`, trigonometric functions or gain synthesis.

### 10.5 Measurement reset, component extraction, and dropout

At a detection, `yP^+=yR` gives `bP^+=nR`. The unchanged position estimate also
admits `bRho^+=min(bRho^-,norm(yR-rhoHat)+nR)`. The circular yaw radius is
intersected with `abs(wrap(yF-psiHat))+BF`. The GNSS sample gives

\[
b_v^+\le\min\{b_v^-,\|\hat R^Tv_m-\hat v\|+\bar n_v
 +2\bar V_E\sin(\min(b_\psi^+,\pi)/2)\}.
\]

Individual target component bounds can additionally be intersected with
`[R+norm(rhoHat), VCmax+norm(qHat), S+norm(sHat)]^T`. After a component update,
`sqrt((D*bT)^T*abs(P)*(D*bT))` is a valid new upper bound on `WT`. It may be
intersected with the prior Lyapunov upper bound; no sign assumption on the
off-diagonal entries of `P` is made.

At a radar-active accepted endpoint, let `Wcomp` denote the Lyapunov bound
returned by the comparison flow, before recertification from component caps.
The reported position radius is

\[
\boxed{B_\rho=\min\{c_1W_{\rm comp},\ b_P+\|y_P-\hat\rho\|,\ R+\|\hat\rho\|\}.}
\]

The capped components then recertify `WT` for the next substep; the algorithm
does not iterate this tightening to a fixed point. During dropout, replace the
first candidate with the directly propagated position component bound.
Specifically use `zC=[bPsi,bv,bRho,bq,bs,bP]^T`, with the same applicable ego rows
and the target/predictor inequalities

\[
\begin{aligned}
\dot b_\rho&=b_q+b_v+R\epsilon_\omega+\eta_\rho,\\
\dot b_q&=b_s+b\epsilon_\omega+\eta_q,\\
\dot b_s&=L_qb_q+L_sb_s+S\epsilon_\omega+\bar\nu_\Phi+\eta_s,\\
\dot b_P&=b_q+b_v+R\epsilon_\omega+\eta_P.
\end{aligned}
\]

Its affine Metzler flow is integrated by the same augmented exponential. There
is no negative target-decay row during dropout. Reconstructing `WT` from its
component bounds on return to detection preserves containment.

**Containment proposition.** Suppose the initial norm/range bounds contain
truth, all declared model and sensor/hold envelopes hold on the interval, and
the chosen defect bounds enclose the numerical reconstruction. On each substep,
the derived error inequalities and Metzler comparison imply componentwise
domination by the corresponding comparison solution. The independent domain
and measurement caps also contain truth, so their minima preserve domination.
Predictor resets are enclosed by the sample noise ball. Induction over substeps,
detections and dropouts proves the claimed position containment. This proves
finite-time containment, with no assertion that the radius necessarily shrinks
or reaches a useful size for every permitted measurement sequence.

Necessary measurement/prior consistency failures mark the affected bound
unavailable and publish `Inf`, rather than an invented finite radius. A radar
inconsistency affects its track; an ego inconsistency affects all tracks and
requires runtime reinitialization. Resetting a target cannot repair an invalid
ego bound. A fresh target reset reinitializes only that target's uncertainty.
These are necessary checks, not an online verification of all premises on truth.

### 10.6 Interface for subsequent collision-avoidance control

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
B_{\Delta p}^I=B_\rho+2R\sin(\min(b_\psi,\pi)/2),
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

### 10.7 Containment experiment

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
