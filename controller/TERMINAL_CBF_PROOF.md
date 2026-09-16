# Recursive feasibility and encounter safety of the implemented controller

This is the current format-30 proof. It replaces the previous stopping-tail
argument. The implemented terminal certificate uses the **same held affine
plant** as the online predictor. Its feedback is a feasible prediction candidate;
it is never an actuator fallback. `hardEncounterBarrier`,
`formulateAvoidanceProblem`, and `solveHardCbfClf.certify` implement the objects
below. Numerical tests validate the implementation; the induction below, not
simulation duration, establishes the recursive statement.

## 1. Statement and declared scope

Starting from an accepted feasible certificate, every subsequent optimization
has a feasible candidate, and every accepted executed hold satisfies the active
collision and permanent actuator-amplitude/slew constraints, provided that:

1. The plant is the declared zero-residual, held affine bicycle model. Its
   admitted generator, sample time, configuration and reference do not change.
   The frozen generator is retained across target release and reoptimization.
2. Ego measurements contain the true state. Their Frenet component radii never
   exceed the stored `measurementRadiusLimit`. Measurements and actual held
   input/time satisfy the execution and set-intersection contracts.
3. While a target remains active, its true motion belongs to its admitted
   finite-flow model. Current observations are sound. A complete observation
   confirms absence from the declared sensing region, or a current conditioned
   target enclosure certifies exterior membership. Observation semantics must
   cover the footprint used by the exit certificate; missed detections cannot
   count as release.
4. New targets, increased motion bounds, and other enlarged obligations must
   admit a new feasible certificate. Arbitrary newly appearing obstacles are
   not covered by the previous encounter's theorem.
5. The permanent reference is an analytically continued straight line or
   constant-curvature reference, with no physical road-boundary constraints.
   No additional state or tire-slip bounds are imposed. The forward reference
   map covers the actuator-reachable envelope; each measured inverse chart
   must remain well defined and meet the declared Frenet sensing contract.
   Centerline samples / analytic arc length describe the reference;
   they are not physical end-of-road barriers. A finite road or a changing
   curvature requires a different permanent continuation certificate.
6. A returned numerical solution passes the independent hard-row and terminal
   cone verification. To execute an indefinitely safe physical sequence, a
   valid solve must also finish before each actuation deadline. Mathematical
   nonemptiness does not imply numerical completion within 100 ms.

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

The geometry uses this reachable tube to bound station, lateral displacement
and heading. No station corridor, lateral/heading box or tire-slip row is
added. A reachable heading interval spanning a full rotation uses the
rectangle circumradius as a global support upper bound. All directions in
the finite grid remain available, with one direction per complete hold.
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
uncertainty enclosures, whole-hold Bernstein tubes, geometry, physical row labels,
terminal cones, finite exit directions/deadline, and common cruise generator.
The hard affine family is `M U <= b`. It includes collision separation,
actuator bounds, slew and robust terminal-entry/exit rows.
The terminal SOCs are also stored. Safety constraints have no slack. The
first-hold CLF has an unbounded nonnegative norm slack with a squared cost.

Target finite-flow tubes include the admitted Cartesian jerk and yaw-
acceleration bounds. The final directional exterior inequality covers the
complete target footprint plus the sensing radius. It establishes that a
sound current scan must confirm release by the stored absolute deadline.
The deadline never moves forward during inherited active-encounter planning.
Prediction alone never releases a target.

## 5. Successor construction for every controller mode

### Nonempty suffix, including partial or full confirmed release

Partition `M = [M_0 M_+]`. After applying the first accepted input `u_0`,
substitute it in every stored affine row and terminal cones:

\[
 M_+ U^+\le b-M_0u_0. \tag{9}
\]

Do the identical substitution in the exact node maps and swept enclosures.
The accepted suffix is feasible by substitution. Rows concerning only the
already executed prefix may be removed. There is no additional tightening,
relinearization, chart change or normal reselection in this inherited family.
Conditioning only restricts the covered physical states.

Confirmed release removes `collision:<key>` and `exit:<key>` rows for that
key. The remaining input/slew/terminal obligations stay intact.
Completion directions are indexed by stable target keys. Removing an obligation
cannot invalidate the old suffix. In particular, **partial release no longer
forces an unproved fresh admission of all remaining obligations**.

The first-hold CLF cannot remove this candidate: for any finite hard-feasible
input its cone can be satisfied by a finite nonnegative slack.

### Distance-dual collision-row replacement

The controller may solve Li et al.'s distance dual at the carried anchor and
propose new continuous separation normals. The resulting whole-hold rows
retain the complete footprint, uncertainty and truncation allowances.
The same inherited suffix, completed with CLF slack, must satisfy every row
and every cone of the proposed program before it is selected. Thus replacing
the collision rows preserves a concrete feasible continuation. When this
check fails, the unchanged inherited program is optimized. Exit directions,
absolute deadlines and terminal conditions are preserved. No normal-selection
heuristic alone is used as a recursive-feasibility argument. See
[DISTANCE_DUAL_CONVEXIFICATION.md](DISTANCE_DUAL_CONVEXIFICATION.md).

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
of an actuator command after solver failure. An admitted retained branch uses
one trajectory optimization per sample, preceded by distance-dual geometry
queries when targets are active. Fresh admission may search multiple convex branches, as specified in
[FINITE_CONVEX_BRANCHES.md](FINITE_CONVEX_BRANCHES.md). The theorem starts
only after one complete branch has passed the original hard certificate;
its affine family and terminal cones are inherited before any optional
witness-preserving collision-row replacement.

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

These cases cover active encounters, partial release, full release, and
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
