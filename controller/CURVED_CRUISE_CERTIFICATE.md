# Curved-road cruise working point and CLF certificate

The working point is a steady turn of the same nonlinear bicycle used by the
trajectory predictor. At a fixed road curvature `kappa`, body longitudinal
speed `vStar`, and declared longitudinal bias `bStar`, solve for lateral
velocity `vyStar`, front steering `deltaStar` and signed force ratio `betaStar`.
The Frenet center offset is zero. Exact kinematics give

\[
 e_{\psi\star}=-\operatorname{atan2}(v_{y\star},v_\star),\qquad
 \dot s_\star=\sqrt{v_\star^2+v_{y\star}^2},\qquad
 r_\star=\kappa\dot s_\star.
\]

In particular, body longitudinal speed and path speed differ when sideslip is
nonzero. Setting `rStar = kappa*vStar` and `ePsiStar = -vyStar/vStar` gives
only the small-angle approximation. The remaining equations are

\[
\begin{aligned}
0&=(F_{xf}^{b}+F_{xr}-F_{\rm road})/m+v_{y\star}r_\star+b_\star,\\
0&=(F_{yf}^{b}+F_{yr})/m-v_\star r_\star,\\
0&=(l_f F_{yf}^{b}-l_r F_{yr})/I_z.
\end{aligned}
\]

The front forces are rotated by `deltaStar` from the wheel frame into the
body frame. Axle lateral forces use the nonlinear modified Fiala law; signed
longitudinal forces use the existing static axle-load scale. Air and rolling
resistance remain explicit. A requested turn with `abs(kappa)*vStar^2` greater than the total tire
acceleration capacity is rejected by a necessary lateral-force check. The
old frozen linear trim is used only as an
initial guess for a three-variable damped Newton solve. Its Jacobian follows
from the existing nonlinear operating-point Jacobians and the exact
kinematic relations above. The solve is bounded to 20 Newton iterations and
12 backtracking evaluations per iteration. Failure raises
`invalidCruiseOperatingPoint`; no approximate trim is silently substituted.
The zero-curvature force balance remains analytic.

At the resulting point, use `continuousMatrices(..., operatingPoint)` and its
five error-state rows to synthesize the continuous Riccati metric. The station
coordinate is cyclic for fixed curvature. With `AStar`, `BStar`, `Q`, `R` and
the certificate gain `K`, the local identity is

\[
(A_\star-B_\star K)^T P+P(A_\star-B_\star K)
  =-(Q+K^TRK).
\]

The gain certifies the local unconstrained model; the online controller still
selects inputs through its hard Predictive CBF and soft CLF optimization.
Both objective transcriptions use the recorded `operatingInput` as the center
of `(u-uStar)'*Ru*(u-uStar)`. No feedback-gain input target is added. The state
reference uses the nonlinear trim at `referenceSpeed`. Synthesis retains
`max(referenceSpeed, certificateSpeedFloor)`; when the floor is active those
two speeds differ and exact equilibrium tracking is not asserted.

## Changing curvature

Each formulated horizon freezes the selected metric and reference working
point; its predicted dynamics still follow the road's scheduled curvature.
Curvature and longitudinal bias are included in the certificate cache key.
Across frames, let `Vold` include the previous reference's declared time
evolution. The diagnostic `clfReferenceSwitchValue` records

\[
\Delta V_k=V_k(x_k)-V_{k-1}(x_k),
\]

using the same current planning state on both sides. This separates changes
of metric/reference from physical dissipation. `clfMetricChanged` identifies
metric changes. A positive jump is not hidden by restarting the value trace.
The implementation imposes local held-interval CLF majorants and does not
impose a bound on this jump. A global convergence argument for varying
curvature would additionally need a common metric, an explicit jump bound,
or a smooth scheduled metric with its derivative term. None is claimed here.

## Validation boundary

Tests verify nonlinear force balance and held-flow invariance on both turn
directions, bias compensation, exact point reuse and the Riccati identity.
The reproducible controller-only benchmark is
`scripts/runCurvedControllerValidation.m`. It uses a nonlinear bicycle,
100 ms sampling, one sample of scheduled actuation delay, two certified
intervals and a 20-stage prediction horizon. Its residual allowances are
empirical diagnostic settings and are checked at independent ODE trace
samples. Sampled agreement is not a global continuous-time residual proof.
Results do not qualify the high-fidelity vehicle, joint observer pipeline,
arbitrary road transitions or hardware worst-case execution time.
