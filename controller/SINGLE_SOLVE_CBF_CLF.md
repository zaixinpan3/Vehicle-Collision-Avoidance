# Single-solve sampled CBF–CLF controller

The current controller solves one two-variable SOCP at every sample. The
variables are the steering angle and signed braking ratio held until the next
sample. Every physical constraint and the cruise CLF constraint is hard. Targets
add obstacle constraints; the objective, CLF, road constraints, actuator limits
and slew limits are identical with and without targets. There are no terminal
sets, continuation policies, fallback commands, alternate normal searches,
solver retries, safety slacks, or post-solve plan checkers.

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
\min_u\;\|R(F\hat e+G(u-u_*)+d)\|^2
 +(u-u_*)^\top W(u-u_*),\quad W\succ0.
\]

The feasible set consists of the hard input/slew polytope, swept road,
chart, state-domain and tire-slip inequalities, the optional obstacle rows
below, and one hard CLF cone. There is no slack decision variable. Previous
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

## Hard sampled CLF dissipation

Let \(q_0<1\) bound the Riccati feedback's nominal squared-norm contraction,
choose \(0<f<1\), and set

\[
c=1-f(1-q_0),\qquad a=(c+q_0)/2<c.
\]

The controller enforces the single SOC

\[
\|R(F\hat e+G(u-u_*)+d)\|
\le \sqrt a\,\|R\hat e\|+\epsilon_{\rm num}.
\]

For \(e=\hat e+\eta\), \(|\eta|\le r\), define

\[
\sigma=\sqrt a\,\||R|r\|+\||RF|r\|+2\epsilon_{\rm num}.
\]

The triangle inequality gives
\(\|Re^+\|\le\sqrt a\|Re\|+\sigma\). Young's inequality yields

\[
V(e^+)\le cV(e)+B,\qquad
B=\frac{c}{c-a}\sigma^2,\qquad V(e)=e^\top Pe.
\]

The reported `clfDisturbanceBound` is \(B\), not an optimized relaxation. For
exact arithmetic and exact states, \(B=0\) and this is strict sampled
exponential dissipation. With numerical allowance or measurement uncertainty,
it is practical sampled dissipation; uniform \(B\) implies
\(\limsup V\le B/(1-c)\) along an indefinitely feasible sequence. No claim of
pointwise continuous-time \(\dot V<0\) is made between samples. A common fixed
curvature and trim are required to chain one metric across holds. A changed
curvature/metric or reference needs a separate switching analysis.

At exact cruising equilibrium, hard zero-error dissipation prevents braking or
steering away from that equilibrium. An obstacle can therefore make the hard
CBF and hard cruise CLF incompatible. The numerical allowance only permits a
tiny neighborhood; it does not provide meaningful avoidance freedom. The code
reports infeasibility rather than silently relaxing either requirement.

## Solver and failure behavior

`solveHardCbfClf.constrained` passes one hard conic program to Clarabel. Its
pre-solve row compaction preserves coupled constraints; tiny cross coefficients
are bounded over explicit input limits before removal. Physical row reserves
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
