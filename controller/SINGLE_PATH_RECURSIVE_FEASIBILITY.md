# Rolling MPC and the role of the terminal certificate

Certificate version 18, September 11, 2026, with the version-19 resolution of
September 12, 2026 noted where it applies. The unresolved shift-compatibility
condition recorded below is closed by the carried-witness construction of
[INFORMATION_STATE_PCBF.md](INFORMATION_STATE_PCBF.md): the shifted plan is
verified with its own carried stage generators, charts, normals and terminal
rows on conditioned information sets, so refreshed fresh problems no longer
enter the recursive argument. The terminal derivation in this document is the
nominal half of that construction; the estimation-error half is Section 3 of
the new document.

## Execution contract

Every call starts a complete prediction from the current measured state,
solves the hard-safety and CLF problem, verifies the returned input sequence,
and issues **only its first held input**. At the next sample the previous
first-hold dynamics, input and target law are checked before a new problem is
built. A finite horizon never counts down to an actual terminal-controller
handoff. Numerical failure raises an error; no retained input or terminal
feedback is silently executed. Failure does not establish global infeasibility.

The study still has exactly one persistent, fully observed target following
its original constant Cartesian acceleration and constant yaw rate. Ego and
target uncertainties are zero. The simulation executes each published affine
first-hold generator exactly. The original target law and clock remain
immutable; refreshing its current prediction origin does not change its motion.
This is an exact **per-frame scheduled-model experiment**, not a fixed
nonlinear vehicle validation.

## Why the former execution stopped

The removed `advance` implementation fixed the initial horizon endpoint,
locked executed inputs in the initial QP and optimized only its shrinking
suffix. When that suffix ended, it dispatched terminal feedback indefinitely.
That architecture constructed a safe stopping execution, rather than rolling
MPC with a stopping certificate. Deleting only its last branch was insufficient:

1. An always-decreasing speed seed placed local station constraints around a
   stopping trajectory even when cruise was feasible.
2. The terminal station chart had the ordinary local modeling radius, without
   room for the stopping excursion already present in the invariant-set proof.
3. Using the appended terminal braking input as the CLF convexification anchor
   can propagate an artificial braking preference backward through successive
   horizons. The safety candidate and performance initialization are separate.
4. Maximizing surplus safety margin and retaining 99 percent of it subordinated
   cruise to an additional clearance objective. A failed performance solve
   could expose the arbitrary LP feasibility point as the actual command.

Initial admission starts with a constant-speed prediction seed. Successor
calls initialize performance with the previous controls shifted by one hold
and the last optimized input repeated. Separately, the shifted controls plus
one terminal braking action form a **safety proof candidate**. Its physical
feasibility is checked in the refreshed prediction and recorded; when the
fresh solve fails or does not improve on it, this verified candidate is the
command (see `INFORMATION_STATE_PCBF.md`). The prediction length is not
shortened. The shifted and the constant-speed seeds are both solved at each
horizon and the verified plan with the smaller `(value, objective)` pair is
kept, because the CLF majorant is exact only at its own seed; a slowing seed
is tried only when neither verifies, before extending the horizon. Every
selected plan must be solved and checked anew. These are optimization
initializations, not input targets.
Terminal chart radius is `stationTrustRadius + M(1,:)*q`, including the verified
longitudinal stopping budget. Geometry is recomputed and bounded over that
larger chart; road and target constraints are not removed.

The feasibility LP searches for a hard-safe interior. The performance SOCP
uses only a small numerical interior, capped at ten configured numerical
margins and half the available LP margin, rather than preserving 99 percent
of the maximized surplus. Independent acceptance still requires all physical
rows and the zero hard-safety level. The numerical buffer is used only for solving; acceptance uses independently
enclosed physical margins, so a negative buffer residual cannot be confused
with a physical violation. No negative physical margin is accepted.
CLF slacks remain nonnegative and squared;
input effort remains centered on the CLF/LQR certificate operating input.
The LP point cannot replace a failed or uncertified performance solution.

## What the shift proof requires

For a fixed prediction law, hard held-interval constraints and terminal family,
let an accepted plan at sample k be

\[
(x_{0|k},\ldots,x_{N|k}),\qquad
(u_{0|k},\ldots,u_{N-1|k}),\qquad x_{N|k}\in X_f(t_k+Nh).
\]

After applying only `u_(0|k)`, the **proof candidate** is

\[
\hat{\mathbf u}_{k+1}
=(u_{1|k},\ldots,u_{N-1|k},\kappa_f(x_{N|k})).
\]

It establishes recursive feasibility if (i) the successor state equals the
predicted first successor, (ii) every retained stage has the same dynamics
and physical constraints, (iii) terminal invariance and entry slew hold for
the appended hold, and (iv) the next optimization admits that same candidate,
including its terminal region and separating geometry. Finite CLF slacks can
be assigned to the candidate; they cannot relax safety constraints. The new
optimizer may choose a different first input with better cruise performance.
There is no requirement to execute `kappa_f` when the original horizon ends.

Huang, Wang, Margellos and Goulart, *Predictive Control Barrier Functions:
Bridging model predictive control and control barrier functions*, 2025,
[Section III, Lemmas III.1 and III.5, equations (10)–(11)](https://arxiv.org/html/2502.08400v2),
use such a shifted candidate to establish the safe-MPC feasibility/value
argument. Their nonnegative safety-slack value is distinct from this code's
CLF relaxation and from its diagnostic negative feasible margin. On an
already feasible hard-safe trajectory, zero safety relaxation is enough;
maximizing extra clearance is not a condition of that shift argument.

**Applicability condition (resolved in version 19).** Fresh replanning
refreshes scheduled bicycle matrices, local station charts, separating planes
and the terminal region, and the old shifted plan need not belong to that
refreshed convex problem. Version 19 therefore does not ask it to: the shifted
plan is verified against the *carried* data of the plan it came from, whose
rows are monotone under box inclusion, and the fresh problem may only replace
that verified witness with a verified plan of no larger safety value. Both
recursive-feasibility claim flags are now true under the declared premises
(declared affine stage plant, exact target law, bounded estimation boxes,
conditioned information sets); see the theorem in
[INFORMATION_STATE_PCBF.md](INFORMATION_STATE_PCBF.md). If a fresh solve fails
at a continuation frame, the carried witness is executed; only a premise
failure ends control.

## Terminal dynamics and invariant set

Write `x=(p,v)` with `p=(s,d,ePsi)` and `v=(vx,vy,r)`. The declared terminal
schedule has no pose-dependent drift:

\[
 \dot p=Gv,\qquad
 \dot v_x=-a v_x+g_\beta\beta,\qquad
 \begin{bmatrix}\dot v_y\\\dot r\end{bmatrix}
 =F_\ell\begin{bmatrix}v_y\\r\end{bmatrix},\qquad a\ge0.      \tag{2}
\]

Steering is zero. At each sample of length `h`, hold

\[
 \beta_k=-k_b v_{x,k}/g_\beta,\quad
 \phi_h=\begin{cases}(1-e^{-ah})/a&a>0,\\h&a=0,\end{cases}
 \quad k_b=\frac{e^{-ah}(1-e^{-h})}{\phi_h}.                 \tag{3}
\]

The additional unit decay rate in `exp(-h)` has units 1/s. This gives
`vx_(k+1)=rho vx_k`, `rho=exp(-(a+1)h)` in `(0,1)`. Within the hold,
`vx(t)=(exp(-at)-k_b phi_t) vx_k` stays nonnegative and nonincreasing, and
`vxDot <= -(a+k_b) vx`. Thus active terminal braking also works when passive
road-load damping is zero. No velocity is rounded or clipped to rest.

Let `C` be the Metzler comparison matrix for the velocity subsystem: its
longitudinal diagonal is `-(a+k_b)` and its lateral/yaw block is the diagonal
of `F_l` with absolute off-diagonal entries. The constructor verifies a
positive vector `q` with `Cq<0` and a nonnegative excursion matrix `M` with

\[
 MC+|G|\le0.                                               \tag{4}
\]

`M` is obtained from `|G|(-C)^(-1)` with a conservative arithmetic reserve.
The numerical residual inequalities are checked. Scale `q` to satisfy speed,
lateral-velocity, yaw-rate, terminal slip, braking-ratio and subsequent
braking-rate limits. The finite program separately constrains the slew from
the last prefix input to the first terminal input, including endpoint
arithmetic uncertainty.

The implementation tightens this symmetric comparison using `vx>=0`. For each
pose row, set `g_j = [max((A G)_(j,1),0), abs((A G)_(j,2:3))]` and construct a
nonnegative row budget `R_j` satisfying `R_j C + g_j <= 0`, with a checked
arithmetic reserve. A lower station row consequently has no fictitious
backward longitudinal excursion. The symmetric `M` remains a valid bound
used to size the terminal chart.

For fixed pose halfspaces `Ap<=b`, the terminal set is

\[
 X_f=\{(p,v):v_x\ge0,\ |v|\le q,\ Ap+R|v|\le b\}.       \tag{5}
\]

The pose rows include the station chart, lateral and heading domains, full
vehicle road containment and all-future target separation. Enumerating the
eight velocity signs makes (5) a finite set of linear hard constraints on the
predicted terminal state. The initial endpoint enclosure must lie in this set.

**Invariance.** Comparison gives `D+|v|<=C|v|`, so `|v|<=q` persists. For every
pose row, the upper Dini derivative of `A_j p+R_j|v|` is at most
`(g_j+R_j C)|v|<=0`. The scalar held-flow formula preserves `vx>=0`.
Terminal input/slip bounds follow from the chosen velocity box. Between
successive terminal commands the braking change is at most
`(k_b/g_beta)(1-rho)q_x`, already included in the box construction. Thus (5)
and every physical terminal obligation persist through every held interval.
In version 19 the feedback acts on the predicted nominal velocity; the
estimation error then evolves open loop and receives its own symmetric budget
`R_err` from the unbraked comparison matrix, which requires passive road-load
damping whenever the terminal velocity box is not a point. The two budgets
add by linearity, and `R_nom <= R_err` makes membership monotone under box
inclusion (INFORMATION_STATE_PCBF.md, Propositions 1 and 2).

This proves invariance for the **hypothetical zero-speed scheduled terminal
model**. It does not prove invariance for the refreshed cruise-scheduled model
or for the nonlinear physical vehicle. The policy is used to establish
existence of a continuation; it is never dispatched by the public controller.

## Target safety for all future time

For each terminal separating direction `n`, define the target position after
terminal entry as `p_t(t_N+tau)=p_N+v_N tau+a_t tau^2/2`. Compute

\[
 S_n=\sup_{\tau\ge0}n^T p_t(t_N+\tau).                      \tag{6}
\]

For the scalar quadratic `c+b tau+a tau^2`, this is `c` if `a=0,b<=0` or
`a<0,b<=0`; it is `c-b^2/(4a)` if `a<0,b>0`; otherwise it is infinite.
An infinite support does not admit that terminal halfspace. The implementation
forms upper coefficients directly from the original absolute-time trajectory,
charges arithmetic, and does not round a small positive derivative to zero.
The proposed direction is chosen on the side where future acceleration (or
velocity for zero acceleration) has nonpositive projection. This is a
conservative continuous supporting geometry, not a complete maneuver oracle.

The ego support majorants, physical clearance and target footprint support
are added to (6). A target with nonzero constant yaw rate uses its full rotating
rectangle's circumradius; fixed heading uses its directional rectangle support.
Thus terminal safety covers the target forever, including later approach or
reversal under the exact acceleration law. It does not depend on sensing range
or an assertion that a stopped ego alone is safe. Road rows cover the entire
terminal station/lateral/heading chart, including the full ego footprint.

## Diagnostics and validation

`terminalPolicyRole` is `predictionWitnessOnly`; `terminalActive` and
`fallbackUsed` are always false. `remainingSteps` is the fresh prediction length
and `deadline` is its moving endpoint, not a terminal handoff time. The full
original-prefix bookkeeping and analytic command-dispatch branch are removed.
`certifiedDuration=Inf` refers only to the hypothetical prefix-plus-tail
witness. `commandCertifiedDuration` is one sample. A valid execution transition
sets `certificateCompatible`. `candidateVerified` (formerly `carriedWitnessFeasible`) reports that the
shifted plan passed verification with its carried data; by the version-19
theorem this is true at every continuation frame under the declared premises.

`runExactStateRecursiveFeasibilityScenario` audits each exact first hold at
11 independent geometry samples, logs the moving horizon, separates completed
runs from failed calls, and saves partial results before returning on failure.
Timing covers input assembly and the entire controller; diagnostic runs
continue after 100 ms misses and therefore do not establish real-time control.
See [the rolling experiment report](../scripts/ROLLING_TERMINAL_RESULTS_20260911.md)
for actual results and remaining cruise, feasibility and timing limitations.

Research checks were performed inline: scope review rejected deleting the
terminal constraint; synthesis review identified model/geometry refresh as a
missing shift premise; final review separates finite-run evidence, terminal
invariance, infinite closed-loop claims and deadline qualification.
