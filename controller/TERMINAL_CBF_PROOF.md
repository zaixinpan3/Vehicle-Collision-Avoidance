# Recursive feasibility and encounter safety of the implemented controller

> Certificate sampling note (2026-09-17): the online controller now certifies the safety rows at the hold nodes of the exact sampled affine plant only; statements below about whole-hold, swept or Bernstein coverage hold at the nodes and no longer claim inter-node coverage. See [NODE_SAMPLED_CERTIFICATE.md](NODE_SAMPLED_CERTIFICATE.md).

This is the format-37 conditional continuation proof. The implemented terminal certificate uses the **same held affine
plant** as the online predictor. Its feedback is a feasible prediction candidate;
it is never an actuator fallback. `hardEncounterBarrier`,
`formulateAvoidanceProblem`, and `solveHardCbfClf.certify` implement the objects
below. Numerical tests validate the implementation; the induction below, not
simulation duration, establishes the recursive statement when all premises hold.

**Active encounters:** the complete shifted certificate includes its occupied
sets and separation angles. Touching support majorants establish premise 7
under unchanged contracts, as derived in
[JOINT_SUPPORT_CERTIFICATES.md](JOINT_SUPPORT_CERTIFICATES.md). This proves
subproblem nonemptiness after admission, not global admission or timely
numerical completion. Target-free continuation uses the same terminal proof.

## 1. Statement and declared scope

Starting from an accepted feasible certificate, every subsequent optimization
has a feasible candidate, and every accepted executed hold satisfies the active
collision and permanent actuator-amplitude/slew constraints, provided that:

1. The plant is the declared zero-residual, held affine bicycle model. Its
   admitted generator sequence, sample time, configuration and reference do not change.
   A constant generator or the compiled phase-indexed sequence is retained across
   target release and reoptimization.
2. Ego measurements contain the true state. Their Frenet component radii never
   exceed the stored `measurementRadiusLimit`. Measurements and actual held
   input/time satisfy the execution and set-intersection contracts.
3. While a target remains active, its true motion belongs to its admitted
   finite-flow model. Current observations are sound. A complete observation
   confirms absence from the declared sensing region, or a current conditioned
   target enclosure certifies exterior membership. Observation semantics must
   cover the footprint used by the exit certificate; missed detections cannot
   count as release.
4. A newly acquired target, increased motion bounds, and other enlarged obligations must
   admit a new feasible certificate. Arbitrary newly appearing obstacles are
   not covered by the previous encounter's theorem.
5. The permanent reference is an analytically continued straight line or
   constant-curvature reference, or an explicitly continued smooth profile with
   the scheduled terminal family described below. Physical road edges enter the
   terminal set only through a declared lateral clearance `[d_R; d_L]` from the
   nominal path, valid along the whole continued reference (Section 3,
   "Declared lateral clearance"); finite fitted boundaries are node-row sensor
   products and are not extrapolated by the terminal set.
   No additional state or tire-slip bounds are imposed. The forward reference
   map covers the actuator-reachable envelope; each measured inverse chart
   must remain well defined and meet the declared Frenet sensing contract.
   Centerline samples / analytic arc length describe the reference;
   they are not physical end-of-road barriers. Smooth profiles additionally require
   the hard reference-phase domain and the verified six-dimensional terminal family.
6. A returned candidate passes independent physical-row, nonlinear support,
   CLF and terminal-cone verification. This may be an improved solution or a
   reverified incumbent. Command acceptance must finish before each actuation deadline. Mathematical
   nonemptiness does not imply numerical completion within 100 ms.

7. Every active-encounter update retains the shifted occupied sets, angles,
   charts and numerical bounds, and constructs a touching majorant at that
   complete witness. This inclusion is implemented, rather than imposed as
   an extra assumption on newly chosen fixed normals.

These are explicit model/sensing/admission conditions, not claims about arbitrary
unseen traffic or a nonlinear physical vehicle. The target high-gain observer
is unchanged. The new ego sensing premise bounds its published measurement
set; it does not replace the target observer with a different estimator.

## 2. Common plant and information sets

Let `x = [s; d; ePsi; vx; vy; r]`, `u = [deltaF; beta]`, and

\[
 x^+=F x+G u+g,\qquad
 [F\ G\ g]=[I_6\ 0]\exp\!\left(h
 \begin{bmatrix}A&B&c\\0&0&0\end{bmatrix}\right).
\]

Every future hold in an admitted program retains this exact generator. The
sampled cruise certificate provides a reference `x_*`, trim `u_*`, gain `K`,
and `P = R^T R > 0`. Since `A(2:6,1)=0`, tracking error
`e = x(2:6)-x_*(2:6)` satisfies

\[
 e^+=F_e e+G_e(u-u_*)+d_*,\qquad
 F_c=F_e-G_eK,\qquad \|R F_c R^{-1}\|_2\le q<1.
\]

Here `d_* = F(2:6,:)*x_* + G(2:6,:)*u_* + g(2:6) - x_*(2:6)`
is the actual sampled trim residual. Its absolute support is charged below;
the code does not assume a numerically synthesized trim is exactly balanced.
The implemented `q` includes the synthesis arithmetic allowance. The input
`u_*` includes aerodynamic and rolling loads and the curvature-specific trim.

An ego information box is `X = z + [-rho,rho]`. Finite open-loop prediction
uses exact affine nominal nodes and the interval recursion

\[
 z_{i+1}=Fz_i+Gu_i+g,\qquad \rho_{i+1}=|F|\rho_i.
\]

Swept Bernstein enclosures additionally cover all times inside every hold,
including polynomial remainder and arithmetic allowances. No hypothetical
future measurement is used to shrink these finite predicted boxes.

Update (2026-09-23): the online prediction now uses a feedback policy and the
deviation sets of [FEEDBACK_TUBE_PREDICTION.md](FEEDBACK_TUBE_PREDICTION.md)
instead of the interval recursion above. In (7) and (8) below, `|W_j| rho_N`
and `|K| rho_N` become zonotope supports: `sum_i |W_j g_i|` over the columns of
`E_N`, and the support of `K_term (e_N + eta_N) - Delta u_{N-1}` in the common
source basis of the prediction.

### Whole-hold enclosure without imposed state bounds

There is exactly one certified interval per held command. For
$\dot x=Ax+Bu+c$, let $d=Ax_0+Bu+c$ and truncate its power series at order $p$.
Writing $\gamma=\|A\|_\infty h$ and choosing $p+2>\gamma$ gives, for $0\le t\le h$,

\[
 |R_p(t)|\le
 \frac{|A|^p\mathbf 1\,\overline d}{(p+1)!}
 \frac{t^{p+1}}{1-\gamma/(p+2)},\qquad
 \overline d\ge\|d\|_\infty.
\]

To see this, bound the first omitted term componentwise, then bound every
successive factorial-series ratio by $\gamma/(p+2)$. Unlike the former
$1/(1-\gamma)$ bound, this does not require subdivision when $\gamma\ge1$.
For a condensed initial state $x_0=MU+o$, compute
$|x_0|\le |M|\bar U+|o|$ from the actuator box. This is a consequence of
reachability, not a state constraint. Apply the same argument to initial
error, numerical error and bounded disturbance terms. Increase $p$ until
the truncation support at $h$ is at most $10^{-11}$; order above 64 fails
explicitly. Arithmetic allowances are added separately. Bernstein degree is
$p+1$ and may therefore exceed the configured minimum Taylor order.

On a straight reference the geometry uses the actuator-reachable tube to
bound station, lateral displacement and heading. Active circular encounters
instead use the certified local pose map and exact affine yaw described in
[JOINT_SUPPORT_CERTIFICATES.md](JOINT_SUPPORT_CERTIFICATES.md). Six hard box rows on
every uncertain Bernstein coefficient establish whole-hold validity of the
Taylor remainder and footprint majorant. They constrain only the selected
convex approximation domain, not road width or tire slip. The terminal exit
row uses the same final domain, with hard endpoint containment. These rows
cannot be relaxed by search deficits. The conditional shift proof retains
their coefficients, frame data and uncertainty enclosures unchanged. A fresh
local domain can replace them only together with a complete feasible witness.
The whole-hold safety implication therefore includes domain membership as an
explicit premise. Each hold still uses one separating direction and no time
subdivision. A reachable heading interval spanning a full rotation in the
unrestricted straight branch uses a global rectangle circumradius bound.
The declared globally affine plant scope is essential: removing the old
model-domain rows does not establish nonlinear physical-model validity.

At the next actual observation, conditioning produces a box inside the carried
successor box and the current measurement box. Thus it preserves the old
trajectory enclosure and satisfies `rho <= r_bar`, where `r_bar` is the
admitted sensing limit. On a circular reference a station is unwrapped about
the carried successor, rather than jumping by one revolution.

## 3. Robust modal terminal set and its information-state interpretation

One common ellipsoid with the slowest contraction factor can substantially
overestimate the effect of measurement noise in faster longitudinal modes.
The implementation instead uses a product of disks in stable modal coordinates.
The CLF metric and performance objective remain unchanged.

Let `V` be a nonsingular modal basis of `F_c`, `W=V^{-1}`, and let `r` be the
five tracking-coordinate measurement bounds. Complex conjugate modes are
retained; every physical error is real. Define

\[
 \mathcal Z=\{e\in\mathbb R^5: |W e|\le a\},\qquad a>0.
\]

Each complex magnitude inequality is an ordinary three-dimensional SOC.
Let `C >= |W F_c V|` include the numerical synthesis allowance, and

\[
 d=|W G_e K|r+|Wd_*|.
\]

Synthesis checks

\[
 Ca+d\le a-2\epsilon_f\mathbf 1. \tag{1}
\]

This is a componentwise contraction/forcing condition. The positive radius
and strict reserve also certify a stable nonnegative comparison matrix.
It is not a numerical trajectory sampling argument.

The hypothetical feedback uses the current conditioned estimate:

\[
 \kappa(\hat z)=u_*-K\hat e,\qquad
 \hat e=e+\nu,\quad |\nu|\le r.
\]

For this input the **true** tracking error obeys

\[
 e^+=F_c e-G_eK\nu+d_*,\qquad |We^+|\le Ca+d\le a-2\epsilon_f.
 \tag{2}
\]

The invariant object is the true state set intersected with the current
information box. A rectangular outer hull need not itself remain inside the
modal set after every measurement. The implementation retains the terminal
set certificate as well as the box; it does not infer loss of true-state
membership from outer-box overapproximation. The estimate's error is bounded
because the conditioned box contains the truth and is a subset of the current
bounded measurement box.

### Declared lateral clearance

Road edges are state constraints, so recursive feasibility needs the terminal
set inside them: the shifted plan's final feedback step must satisfy the node
road rows of the next frame. A finite fitted boundary cannot bound a timeless
set (and a perception-limited boundary constrains only the cells inside its
range), so the road geometry may declare a lateral clearance `[d_R; d_L]` (metres
from the nominal path to the physical edges, valid along the continued
reference) with an error bound `epsilon_d`. Supplying finite boundaries without
this declaration is rejected (`missingLateralClearance`).

With Frenet lateral `y`, heading error `psi`, half length `L` and half width `W`,
every footprint corner has linear lateral coordinate `v` with
`|v| <= |y| + L|psi| + W`. On a reference of curvature `k` the exact Frenet
lateral of a corner with tangential offset `|u| <= hypot(L, W)` lies in
`[v - u^2|k| / (2(1 - |k| v)), v]`, so the outer side receives the allowance
`delta_k = |k| (L^2 + W^2) / (2 (1 - |k| max(d_R, d_L)))`, charged to both sides
and requiring `|k| max(d_R, d_L) < 1/2`. The synthesis adds four rows on the
true error, without a measurement-noise term because the modal set bounds the
true error directly:

\[
 \pm y \pm L\,\psi \le d_{R/L} - W - \delta_k - \epsilon_d - |y_\star| - L|\psi_\star|,
\]

with `(y_*, psi_*)` the reference lateral and heading (zero on the analytic
references). Their supports `|c V| a` join the actuator rows in the radius
synthesis, so the set is scaled until either an actuator row or a clearance row
binds. The scheduled family adds the same rows to every phase with the maximum
curvature of the profile. The rows are node-sampled, like the node road rows;
they do not certify the road between nodes.

Refitted finite boundaries change the execution identity. Since the inherited
family is never relinearized, a frame whose boundaries differ from the stored
ones while the declared clearance and every other identity component are equal
is admitted afresh, seeded by the shifted previous plan
(`readmittedAfterRoadRefit`); a changed clearance still rejects the state.

### Continuous hold and input constraints

The input is held constant for the entire period. Synthesis uses actuator
amplitude rows with `H_e=0` and `H_u=[I;-I]`, parameterized by the **true**
initial tracking error and held input deviation:

\[
 H_e e+H_u(u-u_*)\le b_h.
\]

The bound `b_h` is the actuator range shifted by trim and reduced by the
numerical reserve. Because the input is held, these four rows apply for the
whole period without interval subdivision or state/slip constraints. The
sampled trim residual remains in the modal disturbance support above. For
feedback with bounded measurement error, a sufficient whole-set support
condition is

\[
 |(H_e-H_uK)V|a+|H_uK|r\le b_h. \tag{3}
\]

For a free optimized input, define the input deviation from the hypothetical
feedback `v = u - kappa(hat z)`. The terminal optimization imposes

\[
 H_u v\le b_h-|(H_e-H_uK)V|a-|H_uK|r. \tag{4}
\]

It therefore covers the same continuous hold for every true state in
`mathcal Z` consistent with the estimate. The feedback candidate is `v=0`;
its command is not imposed as an equality.

### Finite input rate and the previous-input state

For a current input `u=kappa(hat z)+v` and a hypothetical next feedback input,
let `J=I+K G_e`. Direct substitution gives

\[
 u^+_{\kappa}-u=K(I-F_c)e+J K\nu-Jv-K\nu^+-Kd_*.
\]

Define

\[
 S=|K(I-F_c)V|a+(|JK|+|K|)r+|Kd_*|.
\]

Synthesis requires `S < h u_dot_max` for each finite rate limit. Terminal
optimization additionally imposes

\[
 \pm Jv\le h\dot u_{\max}-S. \tag{5}
\]

Thus the *next* feedback candidate satisfies slew for every allowed next
measurement. The actual current slew relative to `u^-` is enforced directly.
The previous input and the condition that the feedback candidate is currently
slew-admissible belong to the terminal information state.

### Successor membership under a free optimized input

The true successor under this input is

\[
 e^+=F_c e-G_eK\nu+G_e v+d_*.
\]

The terminal optimization uses five SOCs

\[
 |(W G_e)_j v|\le a_j-(Ca)_j-d_j-\epsilon_f. \tag{6}
\]

Equations (4)-(6), current slew, and a soft CLF form the actual one-hold
optimization. At `v=0`, (1), (3) and (5) establish all its hard inequalities.
For *any* accepted optimized `v`, (6) puts the true successor back in
`mathcal Z`, and (5) permits the next feedback candidate. Measurement
conditioning preserves truth inclusion and the bounded sensing contract.
This establishes invariant information-state feasibility.

### Entry from the finite open-loop plan

The finite plan's terminal box must satisfy, for every mode,

\[
 |W_j(z_{N,2:6}-x_{*,2:6})|+|W_j|\rho_{N,2:6}
 \le a_j-\epsilon_f, \tag{7}
\]

and the robust first feedback transition

\[
 |u_*-K(z_{N,2:6}-x_{*,2:6})-u_{N-1}|+
 |K|\rho_{N,2:6}\le h\dot u_{\max}. \tag{8}
\]

Equation (7) covers the entire prior terminal box. A conditioned terminal
center lies in that box, so (8) guarantees that its feedback candidate is
slew-admissible. The true state lies in `mathcal Z`. These are exactly the
premises of the local invariant optimization above. No future measurement
shrinkage is used when certifying the finite plan.

## 4. Complete finite encounter witness

An accepted witness stores its input sequence, exact affine node maps,
uncertainty enclosures, occupied sets and angles, charts, physical row labels,
terminal cones, finite exit directions/deadline, and common cruise generator.
The hard affine base is `M U <= b`. It includes actuator bounds, slew
and robust terminal-entry rows; curved-road chart boxes are not enforced. Nonlinear support certificates
enforce collision separation and sensing-range exit; they are stored together
with the terminal SOCs. Executable safety constraints have no slack. The
first-hold CLF has an unbounded nonnegative norm slack with a squared cost.

Target finite-flow tubes are the NRMM parameter enclosure of the
`nrmm-motion-v1` contract ([TARGET_PREDICTION_CONTRACT.md](TARGET_PREDICTION_CONTRACT.md));
there is no jerk or yaw-acceleration allowance. The final directional exterior inequality covers the
complete target footprint plus the sensing radius. It establishes that a
sound current scan must confirm release by the stored absolute deadline.
The deadline never moves forward during inherited active-encounter planning.
Prediction alone never releases a target.

## 5. Successor construction for every controller mode

### Nonempty suffix and confirmed target release

Partition `M = [M_0 M_+]`. After applying the first accepted input `u_0`,
substitute it in every stored affine row and terminal cones:

\[
 M_+ U^+\le b-M_0u_0. \tag{9}
\]

Do the identical substitution in the exact node maps and uncertainty enclosures.
The accepted suffix is feasible by substitution. Rows concerning only the
already executed prefix may be removed. There is no additional tightening,
relinearization, chart change or normal reselection in this inherited family.
Conditioning only restricts the covered physical states.

Confirmed release removes the single target's collision and exit certificate
records. The remaining road, input, slew and terminal obligations stay intact.
The completion certificate carries one direction; temporal target identity is
checked before conditioning. Removing the target's obligations cannot invalidate
the old suffix.

The first-hold CLF cannot remove this candidate: for any finite hard-feasible
input its cone can be satisfied by a finite nonnegative slack.

### Complete-certificate convexification

The inherited occupied sets cover the conditioned successor sets. For the
same inherited angle, their support residual cannot increase. Thus the
accepted suffix and angles remain feasible. A convex majorant that equals
the nonlinear residual at that full witness contains the witness, establishing
premise 7 and nonemptiness of the next hard SOCP. Both collision and exit
angles remain free in the improvement solve. Numerical acceptance verifies
the true support residuals and the base constraints independently, and can
retain the certified incumbent if improvement fails. Absolute deadlines and
terminal conditions remain inherited. See
[JOINT_SUPPORT_CERTIFICATES.md](JOINT_SUPPORT_CERTIFICATES.md).

### Fresh no-target performance horizon

The controller may try to restore the configured performance horizon. It
constructs a concrete candidate from the carried suffix followed by the same
terminal feedback applied to nominal predicted nodes. It does not assume
this candidate passes a new convexification or that open-loop uncertainty
remains small for that longer horizon.

Before invoking the optimizer, it checks the candidate against **every row
and all CLF and terminal cones of the proposed new program**. Only a program containing this
candidate may replace the inherited one. Otherwise the inherited program is
used. This is pre-solve selection of an optimization problem, not selection
of an actuator command after solver failure. An admitted retained family uses
one hard performance problem and one native solve per sample. Independent
verification checks every physical row and terminal cone. There are no
support-family, restoration, horizon-retry or constraint-generation loops.
The theorem starts from a hard-certified plan and, for active encounters,
requires premise 7 at every update. It proves neither universal admission nor
numerical completion before a deadline. Format 35 rejects earlier states.

### Empty suffix after confirmed encounter completion

If no fresh longer horizon containing a feasible candidate is obtained, the
controller formulates a one-hold optimization. Both actuator coordinates and
CLF slack remain free. The hard hold rows are those used in terminal synthesis;
there are also current-input slew, terminal SOCs and terminal-entry slew rows.

Its current information state has true-state membership in the robust modal
set and an admissible first feedback transition by (7)-(8). Equations (1),
(3), (5), and (6) show that the hypothetical input `kappa(hat z)` is feasible
in the actual one-hold problem. A finite CLF slack completes its decision.
Any accepted optimizer output preserves true-state membership and next-input
admissibility by (5)-(6), while the sensing contract bounds the next estimate.
Thus **this actual optimization and every successor are nonempty**.

This one-hold terminal optimization is a local invariant MPC problem, not
mandatory execution of `kappa`, a saved input, or a stopping controller.
The configured multi-hold performance horizon is attempted again next frame.

## 6. Induction and safety theorem

The initial accepted witness certifies its first continuous hold. At each
subsequent sample exactly one of the preceding constructions applies:

- a nonempty suffix supplies a feasible candidate, with only confirmed
  obligations removed;
- a replacement program explicitly contains a feasible candidate;
- terminal-set invariance supplies a feasible candidate to the one-hold
  optimization after the finite suffix is exhausted.

These cases cover the active encounter, confirmed target release, and
indefinite no-target operation. Their assumptions include the previous held
input, bounded measurement sets, the common plant and stable target contracts.
By induction every successor problem is feasible, and each accepted hold is
safe for every obligation still active during that hold. Under the observation
contract, finite target obligations discharge by their certified deadlines;
permanent actuator constraints remain satisfied thereafter.

A new target can invalidate the initial-feasibility premise of a *new* task.
An optimization timeout can prevent execution of the mathematically existing
candidate. Neither event is silently labeled successful recursive execution.

This proves invariance of the admitted predictive safety domain in the
augmented information/witness state. It does not by itself assert a globally
continuous scalar CBF for arbitrary changing obstacles, or that soft CLF slack
vanishes and guarantees asymptotic tracking under every disturbance sequence.

## 7. Numerical certificate transfer

`solveHardCbfClf.certify` independently checks hard physical rows and terminal
SOC before any command is returned. A positive approximate status may also be certified; exact optimality is not
needed for the theorem. Infeasible, timed-out or iteration-limited solves are
not accepted. A positive solver status alone is insufficient.
Pre-solve reserves separate tightened solver rows from physical constraints.
If numerical residuals consume part of a reserve, the stored inherited bound
is enlarged just enough to contain the accepted decision with an arithmetic
allowance, **and only while remaining inside the physical bound**. The same
rule applies to the terminal cones radii. A failed check ends control.

Subsequent shifting uses these stored certified bounds, rather than imposing
a fresh numerical tightening at every step. The proof is the real-arithmetic
argument above; floating-point residual/enclosure checks support its numerical
implementation, not a formally verified interval implementation of `expm`,
Riccati synthesis or every machine operation. No nonlinear-plant inclusion or
unconditional real-time guarantee is implied.

## Reference and distinction from fixed-horizon MPC

Huang, Wang, Margellos and Goulart, *Predictive Control Barrier Functions:
Bridging model predictive control and control barrier functions* (2025),
Section III and Lemma III.1, use a shifted sequence and invariant terminal
extension to establish feasibility of the successor problem:
https://arxiv.org/html/2502.08400v2.

Here the active finite encounter uses a shrinking stored witness, followed by
an invariant **information-state optimization**. Verified fresh horizons may
replace either construction. This is the explicit hybrid counterpart of the
terminal-extension argument; it does not assume that a stored plan belongs to
an independently rebuilt problem merely because it is used as an initial guess.

## Changed measurement contracts

Format 33 treats an enlarged ego measurement enclosure limit as a new admission
obligation. It conditions the actual successor against the carried prediction,
but discards the old terminal certificate as authority for the larger future
sensing bound. It rebuilds and independently verifies the complete problem.
No successful continuation claim crosses that contract change automatically.
Metadata records `measurementContractChanged` and the absence of an inherited
feasible family. Failed new admission still issues no command. The unchanged-
contract theorem above is not a guarantee against arbitrarily growing bounds.

For target-free initial admission, the configured nominal performance horizon
may be shortened, within `minimumHorizonSteps`, if only a shorter finite witness
can reach the permanent terminal set. This changes the selected certificate
horizon; it removes neither terminal membership nor predictive continuation.

## Scheduled smooth-reference extension

For curvature profiles, replace the fixed F,G,g and terminal set by the immutable
phase-indexed F_j,G_j,g_j and E_j. The complete construction and assumptions are in
[the smooth-reference certificate](CURVED_CRUISE_CERTIFICATE.md). In particular:

1. All finite held generators and the constant-curvature tail are explicitly
   specified, with the reference clock fixed by the execution contract.
2. The six-dimensional modal inclusion charges the moving reference defect,
   future measurement support, cross-stage gains and input slew. Its last phase
   is invariant under station translation at constant trim path speed.
3. Both finite prediction and every terminal hold enforce the same reference
   phase domain; the within-hold terminal flow uses sampled, held feedback.
4. Conditioning does not change the inherited model family. Eliminating the
   first input preserves the remaining phase domains and terminal deadline.
5. Once the finite witness is exhausted, the current terminal family provides
   a feasible next-hold candidate into its successor, with optimized deviations
   independently checked. Its feedback is never an executed fallback.

Inducting on this indexed inclusion gives the same conditional recursive
feasibility and swept safety statement. Five-dimensional CLF slack affects
performance only. This extension does not establish nonlinear vehicle inclusion
or permit unbounded delay relative to the reference clock.
