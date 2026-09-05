# Exact-flow finite-window ego–target estimation

The online estimator now separates three objects: yaw-independent body-velocity
information, a nominal constant-acceleration/curvature trajectory fit, and a
conservative outer enclosure of the physical target state. Absolute yaw is a
union of circular arcs used for inertial reconstruction. There is no online
third-order radar injection, yaw correction gain, or downstream acceleration
filter. The retained continuous observer and gain-design functions remain
independently testable research algorithms; their ISS constants do not certify
this sampled estimator.

## 1. Geometry and the retained model

Let \(J=[0,-1;1,0]\), \(R(\psi)=\exp(\psi J)\), and
\(\nabla_\omega z=\dot z+\omega Jz\). The physical target coordinates are
\(\rho=R_E^\top(p_C-p_E)\), \(q=R_E^\top v_C\), and
\(s=R_E^\top a_C\). On the positive-speed, unsaturated operating domain,

\[
\nabla_{\omega_E}\rho=q-b,\qquad
\nabla_{\omega_E}q=s,\qquad
\nabla_{\omega_E}s=-\Omega^2q+3A\Omega Jq/V,
\]

where \(V=\|q\|\), \(A=q^\top s/V\), and
\(\Omega=(Jq)^\top s/V^2\). Rotation terms cancel in scalar products.
Differentiating gives

\[
\dot V=A,\quad \frac{d}{dt}(q^\top s)=A^2,\quad
\dot A=0,\quad\dot\Omega=A\Omega/V,\quad
\kappa=\Omega/V,\quad\dot\kappa=0.
\]

Thus this particular NRMM nonlinearity is constant scalar acceleration and
constant geometric curvature. With relative course \(\chi\), use
\(x=(\rho_x,\rho_y,\chi,V,A,\kappa)\). Its physical reconstruction is

\[
q=Ve(\chi),\qquad s=Ae(\chi)+\kappa V^2Je(\chi),\qquad
\dot\chi=\kappa V-\omega_E.
\]

These identities are a coordinate change of the retained model, **not** an
identity for its globally saturated extension outside the domain.
\(V>0\) must hold throughout the flow interval. Constant curvature does not
mean constant yaw rate when speed changes. Under the constant-sideslip model,
\(\beta_C=\arcsin(l_{r,C}\kappa)\) and relative body heading is
\(\chi-\beta_C\). With varying geometric curvature, that reconstruction is
only the nominal constant-sideslip interpretation; no additional body-heading
dynamics are asserted.

The model class itself is established: Schubert, Richter, and Wanielik,
*Comparison and evaluation of advanced motion models for vehicle tracking*
(2008), [DOI 10.1109/ICIF.2008.4632283](https://doi.org/10.1109/ICIF.2008.4632283),
discuss the constant-curvature-and-acceleration model. The reduction above is
derived directly from this repository's equations.

## 2. Exact nominal flow

For duration \(h\), ego yaw increment \(\Delta\psi_E\), and ego translation
\(d_E\) expressed in the **start** ego frame, put

\[
L=Vh+\tfrac12Ah^2,\quad\delta=\kappa L,\quad
 d_C=L\operatorname{sinc}(\delta/2)e(\chi+\delta/2),
\qquad\operatorname{sinc}(z)=\sin(z)/z.
\]

Then

\[
\rho^+=R(-\Delta\psi_E)(\rho+d_C-d_E),\quad
\chi^+=\chi+\delta-\Delta\psi_E,\quad V^+=V+Ah,
\quad A^+=A,\quad\kappa^+=\kappa.
\]

`nrmmExactFlow` evaluates the removable zero-curvature singularity with a
small-angle series and rejects intervals crossing zero speed. This is the
exact nonlinear nominal target flow in real arithmetic. It does not make
uncertain ego increments exact.

## 3. Yaw-independent body information and circular yaw

The declared single-track residual gives
\(b_y=l_{r,E}(\omega_E-d_{st})\). For measured gyro \(u_3\), define
\(\epsilon_\perp=l_{r,E}(\bar d_\omega+\bar d_{st})\). The body set is

\[
\mathcal B_k=\{b:\ V_{E,\min}\le\|b\|\le\bar V_E,
\ |\operatorname{atan2}(b_y,b_x)|\le\bar\beta_E<\pi/2,
\ |\|b\|-\|v_m^I\||\le\bar n_v,
\ |b_y-l_{r,E}u_3|\le\epsilon_\perp\}.
\]

`nrmmBodyVelocitySet` intersects the speed annulus and lateral strip with the
forward sideslip domain. It returns a feasible representative, an enclosing
rectangle, and a covering radius. If the nominal inverse is inadmissible,
the representative is selected from the feasible speed/lateral intersection;
it is not obtained by clipping a negative radicand. Empty information stays
explicitly empty. The runtime uses this sampled representative directly,
without introducing another continuous smoothing gain.

The GNSS direction cone and the enclosing sideslip interval give an outer
yaw arc. This is a conservative projection of the joint GNSS/gyro/body
information, not an exact nonlinear yaw-feasibility solver. Circular arcs are
stored as unions of closed intervals on \([-\pi,\pi]\), preserving components
and identifying the endpoints. The recursion is

\[
\Theta_k^+=\Theta_k^-\cap\mathcal Y_k,\qquad
\Theta_{k+1}^-=\Theta_k^+\oplus\Delta\psi_m\oplus[-E_\Delta,E_\Delta].
\]

No lifted-error decay theorem is used. Empty intersections remain empty until
explicit initialization; they do not trigger a silent reset or reliability
weight. `egoYawRadius` covers the entire reported arc union about the nominal
yaw, even if it has multiple components. Neither nominal absolute yaw nor
its set enters the internal relative target estimator.

## 4. The sample and increment contract

`step` consumes a synchronized frame at \(t\), assimilates its measurements,
and returns a prediction at \(t+h\). `stateTime`, `inputSampleTime`, and
`lastRadarTime` retain these distinct meanings. `output` is read-only.
Radar slots require stable identifiers for multiple targets. `resetTarget`
is an explicit acquisition/retirement operation that also clears that
track's measurement history and prior enclosure.

A frame can supply `egoMotion.duration`, `yawIncrement`, `translation`,
`yawErrorMaximum`, and `translationErrorMaximum`. The bounds must enclose
the complete interval, and translation must use the start ego frame.
Otherwise, the nominal ego flow integrates held body acceleration and yaw
rate. Its enclosure uses

\[
E_\Delta\le\min\{h(\bar\omega_E+|u_3|),
 h\bar d_\omega+\tfrac12h^2\bar\alpha_E\},
\]

\[
E_d\le\min\{h\bar V_E+\|\tilde d_E\|,
 hB_v+\tfrac12h^2(\bar a_E+\|a_m^E\|)\}.
\]

Here \(\bar\alpha_E\) and \(\bar a_E\) are optional bounds valid throughout
the interval. Defaults are `Inf`, preserving unrestricted intersample
yaw acceleration and acceleration. Domain-only terms then remain valid but
can be loose. A point gyro error bound is never multiplied by \(h\) and
mistaken for a complete hold-error bound. Accelerometer sample noise does not
by itself enclose intersample acceleration variation either.

Ego poses are accumulated from relative increments in a frame anchored at
the beginning of the current window. For pose error radii \(B_\psi,E_p\),
composition encloses the additional translation rotation by
\(2\|\tilde d\|\sin(\min(B_\psi,\pi)/2)\). Re-anchoring the finite window
avoids accumulating uncertainty from the entire runtime history.

## 5. Three-parameter nominal inference

Transform each available radar position into the fixed window frame:

\[
z_j=\tilde d_{E,j}+R(\Delta\tilde\psi_{E,j})y_{R,j},\qquad
p_j=p_0+R(\theta_0)f_j(V_0,A,\kappa).
\]

Here \(f_j=L_j\operatorname{sinc}(\kappa L_j/2)e(\kappa L_j/2)\) and
\(L_j=V_0\tau_j+A\tau_j^2/2\). Centering both point sets eliminates
translation; planar Procrustes alignment eliminates rotation:

\[
\theta_0=\operatorname{atan2}\left(\sum_j\operatorname{cross}(\tilde f_j,\tilde z_j),
\sum_j\tilde f_j^\top\tilde z_j\right),\quad
p_0=\bar z-R(\theta_0)\bar f.
\]

`fitNrmmTrajectoryWindow` minimizes the remaining isotropic squared residual
in the three scaled parameters \((V_0,A,\kappa)\) using `fmincon` (Optimization
Toolbox). Analytic objective gradients use the envelope theorem. Bounds
constrain acceleration, curvature, and speed at both endpoints, including
the one-sample prediction endpoint; constant acceleration makes these speed
constraints sufficient throughout the fitted interval. No robust weights or
hidden filtering are applied.

At straight positive-speed motion, the three-time Jacobian at \(0,T,2T\)
has longitudinal and transverse determinants \(T^3\) and \(V^3T^3\).
Nonzero curvature is unnecessary for local identification. This is not a
global uniqueness claim. The returned dimensionless Jacobian condition number
is a diagnostic, not an observability certificate.

Startup, short windows, coincident measurements, and optimizer failure use an
explicit nominal prediction fallback. A fresh radar sample can anchor its
position without differentiating it. Physical nominal boundary constraints
are reported through the fit/domain diagnostics and do not prune the hard
set. During radar dropout, the last estimated model coasts: an eroding window
is not repeatedly refitted without new information.

## 6. Independent deterministic outer enclosures

A transformed radar radius is

\[
\epsilon_j=E_{d,j}+\bar n_R+
2\bar\rho\sin(\min(B_{\psi,j},\pi)/2).
\]

The nominal fit residual is never substituted for this radius. The exact
trajectory feasibility problem consists of the physical domain, a propagated
prior, and all measurement-consistency inequalities. The implementation
maintains a conservative **outer relaxation** of that problem in physical
\((p,q,s)\) coordinates; it does not enumerate sampled trajectories or use
a local optimizer to eliminate feasible states.

The physical acceleration and fixed-frame jerk bounds are

\[
S_{\max}=\sqrt{\bar A^2+\bar\kappa^2\bar V^4},\qquad
J_{\max}=\sqrt{(\bar j_A+\bar\kappa^2\bar V^3)^2+
 (3\bar A\bar\kappa\bar V+\bar j_\kappa\bar V^2)^2}.
\]

The default model has \(\bar j_A=\bar j_\kappa=0\). Nonzero values explicitly
admit variation of geometric acceleration and curvature. The nominal window
still fits constant values; its mismatch is then enclosed in the hard jerk
bound. This does not assert that the changing target satisfies the original
constant-sideslip retained model.

For a query time after every sample, \(d_j=t-\tau_j\), Taylor's integral
remainder gives, componentwise,

\[
p_j=p_t-d_jq_t+\tfrac12d_j^2s_t+r_j,\qquad
|r_{j,i}|\le J_{\max}d_j^3/6.
\]

For three distinct observation times define rows
\(M_j=[1,-d_j,d_j^2/2]\). With \(W=M^{-1}\), the center and radius for each
coordinate of \((p_t,q_t,s_t)\) are
\(Wz\) and \(|W|(\epsilon+J_{\max}d^3/6)\). Several nested triples are
intersected, together with velocity secant bounds using \(S_{\max}\), every
position reachability ball using \(\bar V\), and the physical component
bounds. Ill-conditioned triples are skipped, which only weakens information.

Between frames, `nrmmTargetSet` propagates the previous box with the Taylor
chain and radii \(J_{\max}(h^3/6,h^2/2,h)\). It encloses uncertain ego
translation and rotation, then intersects the result with measurement/window
information and the physical domain. Rotation error terms use the appropriate
physical norm bounds \((\bar\rho,\bar V,S_{\max})\). Initialization uses the
entire declared domain, never the unverified nominal initialization offset.

**Containment statement (real arithmetic).** If the initial domain contains
the truth, all sensor, model-rate, and intersample bounds hold throughout their
stated intervals, and every numerical approximation is enclosed, prediction
and every intersection above retain the truth. Taking the farthest corner of
each two-coordinate output rectangle about the nominal estimate gives its
reported Euclidean covering radius. This is conditional containment, not
convergence or an accuracy forecast.

`outerNonempty` means only that the retained relaxation is nonempty. It does
not prove that an exact nonlinear feasible trajectory exists. `inconsistent`
means an outer intersection was empty or the instantaneous body/gyro domain
information was contradictory. Such outputs have infinite reported radii.
Estimated domain audits do not establish that the true trajectory stayed in
the physical domain.

**Numerical boundary.** The MATLAB implementation uses explicit Taylor
remainders and a configured floating-point guard (`numericalAllowance`,
default \(10^{-9}\)). It does not implement directed-rounding interval
arithmetic or verify the complete floating-point/libm error budget. Accordingly
`machineVerified=false` is published with every enclosure. The analytic
containment argument is not a machine-verified digital certificate; upgrading
that claim requires validated arithmetic for rotations, alignment-independent
bounds, linear solves, and constant evaluation. Passing sampled containment
tests is not a substitute for that work.

## 7. Continuous-comparator qualifications

For the continuous high-gain radar predictor, its residual obeys
\(\dot r=-K_1r\) between resets. Its acceleration injection integrates to
\((K_3/K_1)(1-e^{-K_1h})r(0)\), about \(311r(0)\ \mathrm{s}^{-2}\) for the
previous default design at 50 Hz. This identifies a noise-amplification
mechanism; it is not a full transfer-function analysis.

The previous wrapped yaw correction does **not** support unrestricted lifted
linear-error decay. For \(e=3\), measurement error \(d_F=0.2\), and unit gain,
\(-\operatorname{wrap}(e+d_F)=3.083185\), while \(-|e|+|d_F|=-2.8\).
Its ISS yaw row requires a compatible chart, such as \(|e|+B_F<\pi\), and an
invariance argument. A proper measurement arc alone is insufficient. The
independently retained gain and vector-field algorithms must be read with that
local qualification. Their innovation-displacement minimax is an engineering
design objective, not an optimum uniquely forced by the continuous ISS bound.

Schiller et al., *A Lyapunov function for robust stability of moving horizon
estimation*, [arXiv:2202.12744](https://arxiv.org/abs/2202.12744), provide an
exponential detectability/Lyapunov route to robust MHE stability with appropriate
objectives and horizon conditions. Those conditions have not been established
for this estimator. Barrau and Bonnabel, *The invariant extended Kalman filter
as a stable observer*, [arXiv:1410.1465](https://arxiv.org/abs/1410.1465), analyze
a characterized class of invariant systems under stability conditions; mere
rotation equivariance of NRMM does not establish membership in that class.
These references identify possible further theory, not inherited guarantees.

## 8. Integration and reproduction

`nrmmEstimatorControllerAdapter` publishes the unfiltered window estimates,
including physical acceleration. It no longer overwrites them with a
constant-velocity alpha-beta filter and zero acceleration. Existing controller
tightening values remain separately labeled `configured-engineering-assumption`;
the relative enclosure is not silently promoted to an inertial-frame controller
certificate. Certified closed-loop tightening needs the corresponding ego
position/yaw uncertainty and controller analysis.

Run the geometry/runtime tests and the scenario from the repository root:

```matlab
results = runtests({'tests/nrmmWindowEstimatorTest.m', ...
    'tests/onlineNrmmTrackingRuntimeTest.m'});
assertSuccess(results);
runOnlineNrmmComplexManeuverScenario('Plot',false,'NoiseModel','boundedUniform');
benchmark = runOnlineNrmmTrackingErrorBenchmark('MonteCarloRuns',5);
```

The benchmark uses 12 s trials, 2 s burn-in, seeds 7--11, 0.4/0.8/1.2 s
windows, 50 Hz and 25 Hz sampling, a one-second radar dropout, and a smooth
change of acceleration and curvature. Its explicit scenario increment bounds
are \(\bar a_E=3\ \mathrm{m/s^2}\) and
\(\bar\alpha_E=0.05\ \mathrm{rad/s^2}\), enclosing the analytic ego profiles.
For varying motion it declares \(\bar j_A=1.5\ \mathrm{m/s^3}\) and
\(\bar j_\kappa=0.0175\ \mathrm{m^{-1}s^{-1}}\). These tighter bounds change
the reported enclosures and yaw set, not the nominal relative input history.

`RuntimeFunction` and `DesignFunction` on the scenario, and `BaselineRuntime`
and `BaselineDesign` on the benchmark, allow paired replay of an independently
versioned comparator. The 2026-09-05 comparison uses an external source export
of the original runtime at `ac20bc4317c373e388b846bcf0f42fc8eb077d61`, with only
its function name and path setup changed for side-by-side invocation. The
standard paired truth and noise draws are unchanged. The retained scenario
positions use the same trapezoidal truth quadrature as the original benchmark;
exact-flow equivalence is separately tested against tight-tolerance ODE
integration. Changing-motion truth uses `ode113` at relative tolerance
\(10^{-11}\) and absolute tolerance \(10^{-12}\).

Curvature lag is the minimum-MSE shift on a 20 ms grid over a declared
transition neighborhood (40 ms at 25 Hz), searching -1 to +2 seconds. Timing
measures estimator steps on this host and does not establish a real-time
worst-case execution bound. Detailed trial outputs and test evidence are
stored in the external research archive, not under `estimator/`.

## 9. Measured comparison, 2026-09-05

The completed paired study contains 84 final trials: 21 sampled high-gain
baseline trials and 21 trials for each window. Values below average the five
seeds after the 2 s burn-in. Velocity RMSE uses inertial reconstruction;
position/acceleration RMSE and all physical-state enclosure checks use the
ego-frame coordinates. The smooth changing-motion trajectory reaches
50.7588 m separation; its explicit domain is therefore 55 m. An initial run
incorrectly retained the 50 m domain. Its outputs are preserved as diagnostic
evidence, and all 20 changing-motion comparisons were rerun with the corrected
domain. The final benchmark checks the truth domain/model-rate assumptions
before accepting a trial.

| Case / estimator | Position RMSE (m) | Velocity RMSE (m/s) | Acceleration RMSE (m/s²) | Curvature lag (s) |
|---|---:|---:|---:|---:|
| retained / sampled high gain | 0.093528 | 2.7231 | 29.560 | — |
| retained / 0.4 s | 0.042472 | 0.40039 | 1.7368 | — |
| retained / 0.8 s | 0.032623 | 0.17856 | 0.41250 | — |
| retained / 1.2 s | 0.026362 | 0.098852 | 0.15462 | — |
| changing / 0.4 s | 0.043013 | 0.41077 | 1.7941 | 0.244 |
| changing / 0.8 s | 0.032794 | 0.18239 | 0.45724 | 0.424 |
| changing / 1.2 s | 0.027411 | 0.11875 | 0.30154 | 0.612 |
| dropout / sampled high gain | 2.8549 | 39.682 | 404.99 | — |
| dropout / 0.8 s | 0.059115 | 0.21100 | 0.57007 | — |
| 25 Hz / sampled high gain | 0.17934 | 4.8262 | 48.511 | — |
| 25 Hz / 0.8 s | 0.049724 | 0.26304 | 0.58069 | — |

All evaluated samples of the final 63 window-estimator trials satisfied the
estimated physical domain and lay within the reported analytic radii. This
is empirical support, with the mathematical and numerical qualifications in
Section 6. The baseline's noisy physical acceleration frequently left its
unsaturated domain; the comparison does not claim that its continuous ISS
certificate applied throughout those realizations. No sampled radius or
coverage percentage is assigned to that baseline.

For retained noisy motion with the explicit intersample bounds, the 0.8 s
window's mean relative-position radius was 0.26238 m and mean step time was
3.81 ms. Under the unrestricted default intersample model, the separate paired
seed-7 probe gave the same nominal relative-position/acceleration errors but
much looser mean physical radii: approximately (2.42 m, 39.22 m/s, 6.66 m/s²).
The tight-enclosure seed-7 probe gave approximately (0.263 m, 3.33 m/s,
6.65 m/s²). Acceleration bounds remain substantially more conservative than
measured acceleration error; no smallest feasible-set radius is claimed.

The default remains 0.8 s as an explicit response/noise compromise. A 1.2 s
window reduces stationary-model acceleration error further while increasing
measured curvature lag. These open-loop results and adapter interface checks
do not establish certified collision avoidance in closed loop.

Validation included the estimator/geometry and adapter contract tests, the
84-trial final benchmark, and the repository-wide test suite. The first full
run exposed a new case-normalization bug in `resetTarget`; this was corrected
and covered by an explicit reset regression. The final combined test record
has 211 passes and 12 remaining controller/scenario failures out of 223 tests.
All 12 remaining failures were independently reproduced in an exported checkout
of the unchanged baseline commit. They include 0.05 s versus 0.1 s sample-time
contracts, a removed `frontTrackWidth` field, and terminal-row feasibility
expectations. They are not represented as passing closed-loop validation.
