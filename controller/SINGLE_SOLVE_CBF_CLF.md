# Single-solve sampled CBF–CLF controller

The current controller solves one three-variable SOCP at every sample. The
variables are the steering angle and signed braking ratio held until the next
sample, and one nonnegative cruise CLF slack. Every physical constraint is hard;
only the CLF is soft. Targets
add obstacle constraints; the objective, CLF, road constraints, actuator limits
and slew limits are identical with and without targets. There are no terminal
sets, continuation policies, fallback commands, alternate normal searches,
solver retries, physical safety slacks, or post-solve plan checkers.

This design supersedes the online mechanisms described in
`SAMPLED_BACKUP_CBF_CLF.md` and `INFORMATION_STATE_PCBF.md`. Those documents
record earlier analyses. The fourth controller output, format 25, contains
only the applied input and timestamp. It cannot supply a future command.

## Scope of the guarantees

The plant is the declared held affine bicycle generator with zero process
residual. The physical state must be inside the current published error box,
and target Cartesian jerk must obey its declared bound during the hold. The
road chart and footprint bounds must cover the entire hold. Inputs execute
immediately and remain held for the declared sample period. A current empty
target list asserts that there is no target obligation to include; detection,
missed detections and re-entry guarantees belong to the sensing interface.
No all-future target model is used.

A successful solve certifies the next hold under these assumptions and the
solver feasibility contract. Chaining successful solves establishes safety of
the executed prefix. This implementation does **not** establish recursive
feasibility, a controlled invariant domain for the joint constraints, or a
physical-vehicle guarantee. Stopping the simulation after failure is not a
physical safe-stop maneuver. There is no safety claim beyond the last completed
certified hold when a successor problem has no solution.

## One optimization

Let the exact sampled error dynamics about the constant-speed path trim be

\[
e^+=F e+G(u-u_*)+d,\qquad e=x_{2:6}-x_{*,2:6}.
\]

`ltvBicycleModel.sampledCruise` synthesizes a positive definite matrix
\(P=R^\top R\) using the discrete Riccati equation. This is cached matrix
synthesis; its feedback gain is not executed and does not generate a backup
trajectory. The trim includes road load, the declared acceleration bias and the steady
body-heading offset on a curved path. `referenceSpeed` specifies longitudinal
body speed; the curved-path trim may also have steady lateral body velocity.
The same frozen-curvature generator builds the swept tube and executes in the
exact-state experiment.

The objective is

\[
\min_{u,\delta\ge0}\;\|R(F\hat e+G(u-u_*)+d)\|^2
 +(u-u_*)^\top W(u-u_*)+w_\delta\delta^2,\quad W\succ0,\ w_\delta>0.
\]

The feasible set consists of the hard input/slew polytope, swept road,
chart, state-domain and tire-slip inequalities, the optional obstacle rows
below, and one soft CLF cone. The decision order is
`[frontWheelSteeringAngle; brakingRatio; clfSlack]`; only the first two
coordinates are issued to the actuators. `cfg.clf.relaxationWeight` sets
\(w_\delta\), default 100. The slack relaxes the **norm** CLF constraint below;
it is in units of \(\sqrt V\), not of \(V\) or \(\dot V\). It has no upper
bound, and its coefficient in every physical safety row is exactly zero.
There is no lexicographic or preliminary optimization. Previous
inputs enter only the slew bounds. Legacy horizon and execution-policy options
are accepted for input compatibility but cannot select a different algorithm.

## Obstacle barrier

For each currently supplied target, choose once the unit direction \(n\) from
its measured center toward the ego center. Let

\[
D=\sqrt{\ell_E^2+w_E^2}+\sqrt{\ell_T^2+w_T^2}+d_{\min},\qquad
b(t)=n^\top(p_E(t)-p_T(t))-D.
\]

The two circumdisks enclose the oriented rectangular footprints at every yaw.
Thus \(b\ge0\) suffices for separation. This approximation is conservative,
particularly for lateral passing. Direction selection is deterministic geometry,
not another optimization. A coincident pair uses an arbitrary unit direction
and an infeasible initial-membership row.

The program enforces robust initial membership and sampled decrease:

\[
\underline b(0)\ge0,\qquad
\underline b(h)\ge e^{-\lambda h}\overline b(0),\quad\lambda>0.
\]

The upper and lower bounds include both published state boxes and chart error.
Using an upper initial bound is conservative but proves the sampled inequality
for every possible true initial barrier, without assuming future measurements
reduce uncertainty. This is a sampled barrier condition; existence of a feasible
input is not asserted on the entire geometric safe set.

The sampled inequality alone is insufficient to exclude intersample collision.
For every swept Bernstein cell, the program additionally imposes
\(b(t)\ge0\) throughout the cell. The held affine ego flow uses the existing
Taylor remainder enclosure. Target position and its radius use the exact
finite polynomials

\[
\bar p_T(t)=\bar p_0+t\bar v_0+\tfrac12t^2\bar a_0,\quad
r_p(t)=r_{p,0}+t r_{v,0}+\tfrac12t^2r_{a,0}+\tfrac16t^3J.
\]

Every Bernstein coefficient of the lower barrier is constrained nonnegative.
Nonnegative basis functions summing to one then imply intersample separation.
The final Bernstein endpoint also supplies the sampled decrease row. Different
holds may choose different normals: each newly solved hold independently
establishes initial membership and complete intersample separation.

The distinction between sampled barrier conditions and intersample safety is
also treated by Tan, Das, Ames and Burdick,
[Zero-Order Control Barrier Functions for Sampled-Data Systems with State and
Input Dependent Safety Constraints](https://arxiv.org/abs/2411.17079).
The specific circumdisk/Bernstein construction above is this implementation's
sufficient condition, not a claim that the paper proves this complete controller.

## Soft sampled CLF and its dissipation bound

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

Metadata reports `clfSlack` as the optimizer's third decision,
`clfSlackPenalty` as \(w_\delta\delta^2\), `clfDisturbanceBound` as \(B\), and
`clfSlackDissipationBound` as \(S(\delta)\). A tiny negative numerical slack is
replaced by zero only when forming this outward reporting allowance; neither
the solver decision nor the command is changed or checked. The reported
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
Hard obstacle, road, domain and actuator constraints can still conflict. The
controller stops on an unsuccessful solve; slack provides no backup design
or recursive-feasibility guarantee.

The exact-state driver saves the optimized slack, its dissipation allowance,
the residual with that allowance removed, and the unrelaxed residual. Offline
experimental success uses the slack-dependent inequality. A positive unrelaxed
residual remains visible and is never described as unrelaxed CLF dissipation.

## Solver and failure behavior

`solveHardCbfClf.constrained` passes the single conic program to Clarabel. The
three-variable program retains all physical rows; the older two-coordinate
row-compaction helper is not used for this program. Physical row reserves
are formed before solving. Execution requires a strict successful solver
status and a finite, real, correctly sized command. These are solver result
handling, not a separate residual checker. A custom solver hook must obey the
same status/feasibility contract. A false success declaration is not detected
by an independent verifier in this design.

An infeasible, timed-out, incomplete or malformed solver result raises
`collisionAvoidanceController:optimizationFailed`. The exact-state and
estimator/declared-plant drivers save their diagnostic prefix when an output
directory is supplied, then rethrow the error. Neither advances the plant with
an old command after a failed solve. Offline tests and experimental geometry
measurements remain independent of command execution.

`frameDeadlineSeconds` limits the native solve, including solver dispatch
preparation counted by its work timer. It does not establish a worst-case bound
on MATLAB input parsing, Riccati synthesis, geometry construction, JIT loading
or scheduling. End-to-end timing is reported separately. Real-time suitability
must therefore distinguish warm measurements, cold startup and a proved WCET.
