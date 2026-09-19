# Soft sampled CLF and its dissipation bound

The controller optimizes a scalar restriction at fresh admission and a full
finite input plan during inherited performance improvement, together with a
nonnegative first-hold CLF norm slack. Physical input effort is centered on
the CLF/LQR operating input. For N holds its objective is

\[
\sum_{i=1}^{N}e_i^\top P_i e_i+
\sum_{i=0}^{N-1}(u_i-u_{*,i})^\top W(u_i-u_{*,i})+w_\delta\delta^2.
\]

The CLF is the only softened constraint in
[scalar admission and joint continuation](JOINT_SUPPORT_CERTIFICATES.md). Collision, actuator,
slew, chart and terminal constraints remain hard at command acceptance.
[TERMINAL_CBF_PROOF.md](TERMINAL_CBF_PROOF.md) specifies continuation and
sensing assumptions. [NODE_SAMPLED_CERTIFICATE.md](NODE_SAMPLED_CERTIFICATE.md)
specifies the hold-node scope.

## Dissipation bound

Let \(q_0<1\) bound the Riccati feedback's nominal squared-norm contraction,
choose \(0<f<1\), and set

\[
c=1-f(1-q_0),\qquad a=(c+q_0)/2<c.
\]

The controller enforces the single SOC

\[
\|R(F\hat e+G(u-u_*)+d)\|
\le \sqrt a\,\|R\hat e\|+\epsilon_{\rm num}+\delta,\qquad\delta\ge0.
\]

For \(e=\hat e+\eta\), \(|\eta|\le r\), define

\[
\sigma=\sqrt a\,\||R|r\|+\||RF|r\|+2\epsilon_{\rm num}.
\]

The triangle inequality gives
\(\|Re^+\|\le\sqrt a\|Re\|+\sigma+\delta\). Young's inequality yields

\[
V(e^+)\le cV(e)+B+S(\delta),\qquad V(e)=e^\top Pe,
\]
\[
B=\frac{c}{c-a}\sigma^2,\qquad
S(\delta)=\frac{c}{c-a}\delta(2\sigma+\delta).
\]

Metadata reports `clfSlack` as the optimizer's final decision,
`clfSlackPenalty` as \(w_\delta\delta^2\), `clfDisturbanceBound` as \(B\), and
`clfSlackDissipationBound` as \(S(\delta)\). A tiny negative numerical slack is
replaced by zero only when forming this outward reporting allowance. This
reporting step does not modify the decision or command, or replace their
independent verification. The reported
`clfDissipationCertified` flag means this **slack-dependent sampled bound**,
as specified by `clfDissipationScope`.

At zero slack, exact arithmetic and exact states, \(B=0\) and this is sampled
exponential dissipation. Positive slack allows \(V\) to increase, including
when avoiding an obstacle requires leaving exact cruise. A positive quadratic
penalty does not itself prove that slack vanishes, even on a clear road.
Strict decrease follows whenever \(B+S(\delta)<(1-c)V\). If
\(B_k+S(\delta_k)\le\overline E\) uniformly along an indefinitely feasible
sequence, the bound implies \(\limsup V\le\overline E/(1-c)\).
This conditional statement does not assert such a uniform bound exists.
No claim of
pointwise continuous-time \(\dot V<0\) is made between samples. A common fixed
curvature and trim are required to chain one metric across holds. A changed
curvature/metric or reference needs a separate switching analysis.

For any finite physical input satisfying the hard constraints, a finite
nonnegative slack can satisfy the CLF cone. Thus the CLF no longer excludes a
physically feasible input merely because it would increase tracking error.
Hard obstacle and actuator constraints can still conflict at new
admission. The complete-certificate construction retains a feasible suffix and its
separation angles after admission. CLF slack cannot repair incompatible hard
safety constraints. Admission failure issues no command; failed improvement
can return only a separately verified incumbent within the frame deadline.

The exact-state driver saves the optimized slack, its dissipation allowance,
the residual with that allowance removed, and the unrelaxed residual. Offline
experimental success uses the slack-dependent inequality. A positive unrelaxed
residual remains visible and is never described as unrelaxed CLF dissipation.
