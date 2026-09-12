# Information-state predictive control barrier function for the two-vehicle study

Certificate version 19, September 12, 2026. This document defines the executed
safe-MPC problem, its carried recursive-feasibility witness, and the value
function that the controller reports as its predictive control barrier
function (PCBF). It replaces the "shift-compatibility gap" of version 18 in
[SINGLE_PATH_RECURSIVE_FEASIBILITY.md](SINGLE_PATH_RECURSIVE_FEASIBILITY.md)
with a construction in which the shifted plan is verified, at every frame,
with the very data it was accepted with, on information sets that can only
shrink. The argument is the shifted-candidate/value-function argument of
Huang, Wang, Margellos and Goulart (ECC 2025, Section III, Lemmas III.1 and
III.5, equations (10)–(11), Theorem 4), combined with the two structural
assumptions Batkovic, Gupta, Zanon and Falcone (arXiv:2305.03312, Assumptions
4 and 5, Proposition 1) isolate for driving among other road users:
predictions that are *consistent* over time, and a *safe terminal set* whose
members are safe against the other road user for all future time.

## 1. Premises

The theorem below holds under exactly these premises. None is inferred from
the others, and each is checked or declared at the interface named.

- **P1 (two vehicles).** One ego vehicle and one persistent target with a
  stable identity and a fixed rectangular footprint. Checked by
  `hardEncounterBarrier.validateAdmission` and `validateTransition`.
- **P2 (bounded estimation error).** At each frame `k` the controller
  receives an ego estimate with a componentwise error box (`controllerStateErrorBound`
  or an `ego-state-v1` certificate) and a target estimate with a componentwise
  box on position, velocity, acceleration, yaw and yaw rate. The true states
  lie in those boxes. Only *current-state* boxes are admitted; future-motion
  bounds (`predictionAccelerationErrorBound`, `predictionYawAccelerationErrorBound`,
  jerk or yaw-acceleration contracts) are rejected by `targetPrediction.admitExact`.
- **P3 (exact target law).** The target's true motion is the Cartesian
  constant-acceleration/constant-yaw-rate flow
  `p(t+τ)=p+vτ+aτ²/2, v(t+τ)=v+aτ, a, ψ(t+τ)=ψ+ωτ, ω` from its true current
  state. Under P2 the current state is uncertain inside a box; the *law* is
  not. (Section 7 explains why a nonzero velocity or acceleration box makes
  the terminal set empty for a moving target, and what the demonstrations
  therefore use.)
- **P4 (declared ego plant).** Over each held interval `[t_k, t_k+h)` the true
  ego Frenet state obeys `ẋ = A x + B u_k + c` with `(A,B,c)` the first-stage
  continuous generator of the plan accepted at frame `k`
  (`metadata.executedContinuousGenerator`) and `u_k` the issued input, with
  zero process residual (`ltvModelErrorRateBound`, `plantModelResidualRateBound`
  both zero; checked). This is the same declared scheduled-affine plant the
  version-17/18 exact study executed; the difference is that it is now applied
  to a set of states.
- **P5 (execution contract).** Frames arrive every `h` seconds with the
  previously issued input held and reported; road, route, lane and
  configuration are unchanged (`identity` equality). Arithmetic is exact
  except where a reserve is explicitly charged (Section 6).

Nothing here is a claim about a nonlinear Fiala vehicle, a perception
pipeline, or a real-time budget.

## 2. Objects

**Boxes.** `⟨x̄, r⟩ := {x : |x − x̄| ≤ r}` (componentwise). `⟨x̄', r'⟩ ⊆ ⟨x̄, r⟩`
holds iff `|x̄' − x̄| ≤ r − r'` componentwise.

**Ego information set.** At admission, `X_0 = toFrenet(⟨x̂_0, r̂_0⟩)`
(`stateUncertainty.toFrenet`, which requires one invertible projection chart).
At a continuation frame, with the previous plan's predicted successor box
`⟨x̄_{1|k}, r_{1|k}⟩` and the new measurement box `⟨x̂_{k+1}, r̂_{k+1}⟩`,

    X_{k+1} = hull( ⟨x̄_{1|k}, r_{1|k}⟩ ∩ toFrenet(⟨x̂_{k+1}, r̂_{k+1}⟩) ),

computed by `localConditionBox` in `hardEncounterBarrier.validateTransition`.
An empty intersection is reported as `inconsistentObservation` (a premise
failed); otherwise `x(t_{k+1}) ∈ X_{k+1} ⊆ ⟨x̄_{1|k}, r_{1|k}⟩`.

**Target information set.** `Z_0` is the admitted measurement box. At a
continuation frame, `Z_{k+1} = hull( Φ_h(Z_k) ∩ ⟨ẑ_{k+1}, ρ̂_{k+1}⟩ )`
(`targetPrediction.conditionExact`), where `Φ_τ` is the exact flow of P3 and
`Φ_h(Z_k)` is its interval hull (`targetPrediction.finiteFlow`: the flow is
affine in `(p,v,a)` and in `(ψ,ω)`, so the hull is `⟨Φ_h(z̄), |DΦ_h| ρ⟩` plus a
rounding allowance charged only to components that were actually summed).

**Plan and data.** A plan is an open-loop input sequence `u_0,…,u_{m−1}`
together with its *data* `D`: per stage `j` the generator `(A_j,B_j,c_j)`, the
schedule scalars used by the slip rows, and per certification cell the lane
chart (`frame`) and the separating normal per target; plus the terminal data
`T` of Section 3. `hardEncounterBarrier.carriedData` extracts `D` from an
accepted plan; the certificate carries it.

**Enclosure.** Given `X = ⟨x̄, r⟩`, a plan and its data, `ltvBicycleModel.finitePredict`
(with `model.prescribedStages` when the data are carried) produces, for every
cell `ℓ` and Bernstein control point `q`, a box `⟨p_{ℓq}(x̄,u), ρ_{ℓq}(r)⟩`
that contains the held-input affine flow of every `x ∈ X`
(`stateUncertainty.flowTube`). `p_{ℓq}` is affine in `x̄` and `u`; `ρ_{ℓq}`
is `R_{ℓq} r + ρ⁰_{ℓq}` with `R_{ℓq} ≥ 0` (Taylor remainder and tail terms are
nondecreasing in `r`). Separately, the *node* boxes are the exact held-input
stage flows: centers `x̄_{j+1} = Φ_j x̄_j + Γ_j u_j + γ_j` with
`(Φ_j,Γ_j,γ_j) = expm(h[A_j B_j c_j; 0])` and radii `r_{j+1} = |Φ_j| r_j`
(`prediction.initialErrorBound`, the interval-hull chain). The published
successor box, the conditioning of Section 2 and the terminal node of
Section 3 all use these exact nodes, never the tube endpoints. A
signed-generator chain would be tighter at the terminal node but is not
monotone once a later frame re-boxes an intermediate node, which is why the
hull chain is the published one.

**Rows.** Every constraint is a halfspace on a control point:
`a_ρ·p_{ℓq} + |a_ρ|·ρ_{ℓq} ≤ b_ρ`. Hard rows: input bounds, slew, model
domain (station chart, lateral, heading, velocity box, slip), terminal rows.
Safety rows (relaxable): oriented-rectangle separation along the cell normal
with the target box at the cell time, and road containment. `formulateAvoidanceProblem`
labels rows by `safetyRows` and `rowStage`.

**Value.** For a decision, `solveHardCbfClf.certify` computes the per-stage
violation `ξ_j = max(0, max_{safety rows of stage j}(a·p + |a|ρ − b))` on the
physical rows and reports `V = Σ_j ξ_j`, the exact-arithmetic evaluation of
Huang et al.'s objective with an `ℓ∞`-per-stage slack (their footnote 2).
A decision is *verified* iff every hard row holds (with the rounding
allowance), every CLF cone holds with its slack, and `V` is finite. Verified
does not mean safe; `V = 0` means no predicted violation.

## 3. Robust terminal set

Fix the terminal curvature `κ_T` (lane curvature at the plan's terminal
anchor station). `localTerminalDynamics` builds the zero-speed scheduled
generator `ẋ = A_T x + B_T u + c_T` with `A_T(:,1:3)=0`, longitudinal row
`v̇_x = −a v_x + g β` (`a ≥ 0` the linearized road-load damping), lateral block
`F_ℓ`, and the rest input `u_f = (0, β_0)` with `c_T + B_T u_f = 0`.

**Terminal law (sampled, on the predicted nominal).** With period `h`,
`k_b = e^{−ah}(1−e^{−h})/φ_h` (`φ_h = (1−e^{−ah})/a`, or `h` if `a=0`),

    u_k = u_f + K x̄_k,   K = −(k_b/g) e_{v_x}^⊤,

where `x̄_k` is the *predicted nominal* at that sample, never the estimate.
Under P4 the true state is `x = x̄ + e`; the nominal obeys the sampled closed
loop (`v̄_{x,k+1} = ρ v̄_{x,k}`, `ρ = e^{−(a+1)h} ∈ (0,1)`, `v̄_x(t) ≥ 0` inside
each hold) and, by linearity, the error obeys the *open-loop* generator
`ė = A_T e` with no input.

**Comparison systems.** Let `C_err` be the Metzler comparison matrix of the
velocity block of `A_T` (its diagonal kept, off-diagonal entries in absolute
value) and `C_nom = C_err − k_b e_1 e_1^⊤`. `C_nom ≤ C_err` componentwise.
`localComparisonDirection` certifies a Hurwitz Metzler matrix by a positive
`q` with `C q < 0`; `q` is taken from `C_err` when it is Hurwitz (this needs
`a > 0`), otherwise from `C_nom` and then only a zero velocity box may be
carried (`errorBudgetFinite=false`, checked in `completionRows`). The
velocity box is `Q = {|v| ≤ q}` after `q` is scaled to the speed, lateral
velocity, yaw-rate, slip, braking-ratio and braking-slew limits.

**Pose rows and budgets.** The pose rows `A p ≤ b` (`localTerminalSet`) are
the terminal station chart, the lateral and heading domains, road containment
of the full footprint over the chart, and, per target, the separating row
`−n^⊤(origin + [t,l] p_{1:2}) + slope·e_ψ ≤ n^⊤ origin − S_n − h_T − h_E − …`
with `S_n` the all-future box support of Section 4, `h_T` the target
rectangle support over its yaw box (its circumradius if the yaw-rate box is
not `{0}`) and `h_E` the concave ego support majorant anchored at the terminal
heading. With `G = A_T(1:3,4:6)`, the directional growth `g_j = [max((A_jG)_1,0), |(A_jG)_{2:3}|]`
(valid because the nominal `v̄_x` never becomes negative) and the symmetric
growth `|A_j G|`, `localBudget` computes nonnegative row budgets

    R_nom,j C_nom + g_j ≤ 0,      R_err,j C_err + |A_j G| ≤ 0,      R_nom,j ≤ R_err,j,

with checked floating-point reserves; the last inequality is verified and
required (Proposition 2 uses it; it follows mathematically because `−C_nom ≥ −C_err`
are M-matrices).

**Membership.** A node box `⟨x̄, r⟩` with `x̄ = (p̄, v̄)`, `r = (r_p, r_v)` is in
`X_f` iff, for all rows `j` and all sign vectors `σ ∈ {−1,1}^3`,

    A_j p̄ + R_nom,j (σ ⊙ v̄) + |A_j| r_p + R_err,j r_v ≤ b_j,          (T1)
    ±v̄_i + r_{v,i} ≤ q_i  (i = 1,2,3),      v̄_x ≥ 0.                  (T2)

`terminal.stateRows/stateBound` hold the enumerated `(T1)–(T2)` rows on the
nominal and `terminal.errorRows` the coefficients of `r`; `completionRows`
maps them onto the plan's final node, and adds the entry-slew rows of the
first terminal input (a deterministic function of the nominal, so no box
enters them).

**Proposition 1 (robust invariance and all-time safety).** Let `⟨x̄_0, r_0⟩ ∈ X_f`
and let the ego obey P4 with the terminal generator and the sampled law
above. Then for every `x_0 ∈ ⟨x̄_0, r_0⟩` and every `t ≥ 0`: `A p(t) ≤ b`,
`|v(t)| ≤ q`, and at every later sample the pair (nominal, comparison-propagated
radius) is again in `X_f`.

*Proof.* Write `x(t) = x̄(t) + e(t)`. For the nominal, `N_j(t) := A_j p̄(t) + R_nom,j |v̄(t)|`
has `D⁺N_j ≤ (g_j + R_nom,j C_nom)|v̄| ≤ 0` (the argument of
SINGLE_PATH_RECURSIVE_FEASIBILITY.md, using `D⁺|v̄| ≤ C_nom|v̄|` inside every
hold and `v̄_x ≥ 0`). For the error, `E_j(t) := A_j e_p(t) + R_err,j |e_v(t)|`
has `D⁺E_j ≤ |A_j G||e_v| + R_err,j C_err |e_v| ≤ 0` because `D⁺|e_v| ≤ C_err|e_v|`
for the open-loop generator. Hence `A_j p(t) = A_j p̄(t) + A_j e_p(t) ≤ N_j(t) + E_j(t)
≤ N_j(0) + E_j(0) ≤ A_j p̄_0 + R_nom,j |v̄_0| + |A_j| r_{p,0} + R_err,j r_{v,0} ≤ b_j`
by (T1). The scaled level `max_i |v_i|/q_i` of `v̄` and of `e_v` are each
nonincreasing (`C_nom q < 0` and `C_err q < 0`), so `|v(t)| ≤ |v̄(t)| + |e_v(t)|`
stays inside `Q` by (T2). Nonincrease of `N_j`, `E_j` and of both levels gives
membership at later samples. ∎

**Proposition 2 (monotonicity under box inclusion).** If `⟨x̄, r⟩ ∈ X_f` and
`⟨x̄', r'⟩ ⊆ ⟨x̄, r⟩`, then `⟨x̄', r'⟩ ∈ X_f`.

*Proof.* From `|x̄' − x̄| ≤ r − r'`: `A_j p̄' ≤ A_j p̄ + |A_j|(r_p − r'_p)` and
`|v̄'| ≤ |v̄| + (r_v − r'_v)`. Then the left side of (T1) for the primed box is
at most `A_j p̄ + |A_j| r_p + R_nom,j |v̄| + R_err,j r_v − (R_err,j − R_nom,j)(r_v − r'_v) ≤ b_j`,
using `R_nom,j ≤ R_err,j` and `r_v ≥ r'_v`. (T2) follows the same way. ∎

## 4. Two monotonicity lemmas

**Lemma 1 (interval hulls of affine maps are monotone).** For any matrix `M`,
vector `m`, and boxes `⟨x̄', r'⟩ ⊆ ⟨x̄, r⟩`: `⟨M x̄' + m, |M| r'⟩ ⊆ ⟨M x̄ + m, |M| r⟩`.
*Proof.* `|M x̄' − M x̄| ≤ |M||x̄' − x̄| ≤ |M|(r − r')`. ∎

Consequences used below: (i) for fixed cell data, every control-point box of
the enclosure computed from `X_{k+1}` is contained in the corresponding box
computed from `⟨x̄_{1|k}, r_{1|k}⟩`, because `p_{ℓq}` is affine in `x̄` and
`ρ_{ℓq}` is nondecreasing and affine-dominated in `r` (Section 2); (ii) for
the target, `Φ_τ(Z_{k+1}) ⊆ Φ_{τ+h}(Z_k)` as boxes, because `Z_{k+1} ⊆ Φ_h(Z_k)`
and the flow is affine; (iii) every halfspace row satisfied by a box is
satisfied by any sub-box, and the violation `max(0, a·p̄ + |a|ρ − b)` is
nonincreasing under inclusion.

**Lemma 2 (all-future support).** For a unit normal `n` and a target box
`Z = ⟨z̄, ρ⟩`, define upper coefficients `P = n·p̄ + |n|·ρ_p`, `V = n·v̄ + |n|·ρ_v`,
`A = ½(n·ā + |n|·ρ_a)`. Then `S_n(Z) := sup_{τ ≥ 0, z ∈ Z} n·p_z(τ) ≤ sup_{τ≥0}(P + Vτ + Aτ²)`,
which equals `P` if `A ≤ 0, V ≤ 0`; `P − V²/(4A)` if `A < 0 < V`; and `+∞`
otherwise (`localFutureSupport`). Moreover `S_n(Φ_h(Z_{k+1})) ≤ S_n(Z_k)` for
`Z_{k+1} ⊆ Φ_h(Z_k)` (a supremum over a subset of trajectories and times).
`localAdmissibleNormal` selects, among the displacement-based proposal and the
chart axes, a normal with finite support and the largest current clearance;
if none exists the frame reports `unboundedTargetSupport`.

## 5. The theorem

Per frame the controller (in `collisionAvoidanceController`) does the following.

1. Conditions `X_k` and `Z_k` (Section 2).
2. If a certificate is carried: forms the **candidate** — the previous
   accepted plan without its first input, `(u_{1|k−1},…,u_{m−1|k−1})`, with the
   carried data of stages `2..m` and the carried terminal data `T_{k−1}` — and
   verifies it on `(X_k, Z_k)` (`hardEncounterBarrier.verifyCandidate`).
   With `m − 1 = 0` optimized stages left, the candidate is the terminal law
   itself and verification is the node-zero membership test
   `hardEncounterBarrier.terminalMembership(T_{k−1}, x̄_k, r_k)`.
3. Solves a **fresh** problem (fresh linearization along the shifted plan,
   fresh charts and normals, fresh terminal data): value LP, then CLF SOCP
   within the value optimum (`solveHardCbfClf.solve`), and verifies the
   result with its own data.
4. **Accepts** the fresh plan iff it is verified and its value does not exceed
   the candidate's value (exactly when the candidate's value is zero; within
   `solver.lexicographicTieTolerance` otherwise). Otherwise the candidate is
   the accepted plan. The first input of the accepted plan is issued; the
   accepted plan and its data are carried.

**Theorem (recursive feasibility and PCBF descent).** Under P1–P5, if the
plan accepted at frame `k` is verified (with value `V_k` and first-stage
violation `ξ_{0,k}`), then at frame `k+1` the candidate is verified with
value at most `V_k − ξ_{0,k}`. Consequently a verified plan exists at every
subsequent frame, the accepted values satisfy

    V_{k+1} ≤ V_k − ξ_{0,k} ≤ V_k          (exactly whenever V_k = 0),

the set `{V = 0}` is invariant, and on it every executed hold is free of
predicted collision and road violation over the whole held interval.

*Proof.* (a) *Successor containment.* By P4 and P5 the true state at `t_{k+1}`
is the affine flow of a member of `X_k` under `u_{0|k}`, hence lies in the
verified successor box `⟨x̄_{1|k}, r_{1|k}⟩` of the accepted enclosure; by P2 it
lies in the measurement box; so `X_{k+1}` contains it and
`X_{k+1} ⊆ ⟨x̄_{1|k}, r_{1|k}⟩`. Likewise `Z_{k+1} ⊆ Φ_h(Z_k)` by P3 and P2.
(b) *Carried stages.* The candidate's stage `j` is the accepted plan's stage
`j+1` with the same generator, chart, normal and input. Its enclosure from
`X_{k+1}` is contained cell by cell in the accepted enclosure from
`⟨x̄_{1|k}, r_{1|k}⟩` (Lemma 1(i)), and its target boxes are contained in the
accepted ones at the same absolute times (Lemma 1(ii)). Every hard row of
those stages therefore holds, and each stage violation is at most the
accepted one (Lemma 1(iii)): `ξ_{j|k+1} ≤ ξ_{j+1|k}`. The input, slew and
CLF-slack rows involve only the carried inputs and the previously issued
input, which are unchanged (the CLF slacks are recomputed by `certifyInputs`
and never relax a hard row). (c) *Terminal node.* The candidate's final node
`m−1` is the accepted plan's node `m`, whose box was in `X_f(T_{k−1})`; the
new node box is a sub-box of it (Lemma 1(i)), hence in `X_f` (Proposition 2).
The carried terminal rows use `S_n` computed from the accepted terminal time;
by Lemma 2 they dominate the rows the new terminal time would require. Thus
the candidate is verified with `V ≤ Σ_{j≥1} ξ_{j|k} = V_k − ξ_{0,k}`.
(d) *Terminal-only candidate.* With no optimized stage left, membership of
`X_k` in `X_f(T_{k−1})` holds by Proposition 1 applied at the previous frame
and Proposition 2 applied to the conditioned box; the issued input is the
terminal law on the nominal `x̄_k`, and Proposition 1 gives safety for all
future time, in particular over the next hold, and membership again at
`t_{k+1}`. (e) *Acceptance.* The accepted plan is either the candidate or a
verified fresh plan with no larger value, so the inequality holds for the
accepted values; by induction a verified plan exists at every frame. When
`V_k = 0` the acceptance rule is exact, so `V_{k+1} = 0`; a zero value means
every safety row holds on every Bernstein control point of every cell, and
the control-point boxes enclose the complete held-input flow of every member
of the information set, which gives the continuous-time statement. ∎

**Remarks.** (i) The theorem is the executable form of Huang et al.'s Lemma
III.1 and inequality (11) with the information set as the state and the
verified value of the accepted plan as the barrier: `h(X_k) := V_k` satisfies
`h(X_{k+1}) − h(X_k) ≤ −ξ_{0,k} ≤ 0` along the closed loop, with zero
sublevel set the verified-safe boxes. It is not a claim to have computed the
exact optimum over all plans; the value is an attained upper bound that
decreases as required. (ii) Batkovic et al.'s Assumption 4 is Lemma 1(ii)
and (iii) with conditioning; their Assumption 5 is Proposition 1 with the
all-future support of Lemma 2 playing the role of "the a-priori unknown
constraint can never be active in the safe set". (iii) The recovery statement
of Huang et al. (Lemma III.5, `ξ_{0,k} → 0`) follows with an exact tie-break;
with the positive `lexicographicTieTolerance` `τ`, the descent in the
positive-value regime is `V_{k+1} ≤ V_k − ξ_{0,k} + τ`, which is what the
controller reports as `pcbfDescentResidual`.

## 6. What is numerical, and what the reserves cover

- **Rounding.** Every verified row is evaluated with the allowance
  `γ(|b| + |A||x|)`, `γ = n·eps/(1 − n·eps)`. Fresh solves tighten every hard
  row at solve time by `2·max(numericalMargin·rowScale, constraintTolerance·programScale)`,
  where `rowScale = 1 + |b| + |a|·(input reach)` and `programScale` is its
  maximum over the program: the native solver's feasibility tolerance is
  relative to the program's dominant magnitude, so its absolute error is
  shared by every row, including small ones such as the terminal velocity
  box. The performance stage keeps a further interior of at most
  `10·numericalMargin`, never more than half the margin LP point's margin.
  Accepted fresh plans therefore satisfy the physical rows with margin; the
  carried re-evaluation can lose only rounding-level amounts (the `flowTube`
  arithmetic term depends on the magnitude of the nominal, which is not
  monotone at the `eps` level). A carried witness that nevertheless fails
  verification is reported as `carriedWitnessRejected`; it is a premise or
  arithmetic failure, not a controller decision.
- **Solver tolerance and the value decision.** Stage A first solves the
  margin LP with every violation fixed at zero; it is feasible exactly when
  a zero-violation plan exists on the tightened rows, and its point supplies
  the performance interior. Only when it is infeasible does the value LP
  minimize `Σξ`; Stage B then receives that optimum plus `τ` as its budget.
  With a zero optimum the violation columns stay fixed at zero.
- **Rest measured with error.** A stopped ego measured with speed error
  `±ρ_v` publishes a box straddling `v = 0`. The input contract admits any
  box that meets the model domain; on continuation frames the conditioned
  box is the intersection of the successor box, the measurement box and the
  domain `[v_min, v_max]` (the plant lies in the domain by premise, and
  intersecting with a fixed set is monotone under inclusion), so the centre
  never leaves the domain. Admission, which has no carried box, requires
  the measured centre itself to lie in the domain. Before this rule the
  noisy stationary trial ended at hold 41 with an `invalidInput` report
  when the true speed had fallen below the measurement radius.
- **Seeds are anchors, not policies.** The CLF rows of the performance
  SOCP are a convex majorant of an indefinite quadratic: the concave part is
  linearised at the seed plan, so the majorant is exact at that seed and
  grows with `curvature·|Δ|²` away from it. A verified plan obtained from one
  seed is therefore not the tie-break optimum obtainable from another, and a
  braking tail inherited through the shifted seed would reproduce itself
  frame after frame (observed in the noisy oncoming trial before this rule:
  the ego settled at 2.7 m/s with every frame certified at value zero). The
  fresh search now solves both the shifted seed and the constant-speed
  equilibrium seed at each horizon and keeps the verified plan with the
  lexicographically smaller `(value, objective)` pair; the slowing seed is a
  rescue tried only when neither verifies. Every seed's plan is solved and
  verified in full; none is executed unverified.
- **Normals and charts** are choices; carrying them is what makes the
  candidate's rows identical functions of the box. Fresh solves may choose
  differently, which is why their plan must be verified independently.
- **Target law with boxes.** With `a = 0` exactly and `ρ_v ≠ 0`, or with
  `ρ_a ≠ 0`, Lemma 2 gives `S_n = +∞` for every normal on which the box is not
  strictly receding: an infinitesimal admitted drift integrated over infinite
  time is unbounded, so no stopped ego position is safe forever. This is not
  a weakness of the arithmetic; it is what "the target may move at any
  velocity inside the box, forever" means. The demonstrations therefore use
  boxes on the target's position and yaw (and on all six ego states) and
  treat the target's velocity, acceleration and yaw rate as exactly known,
  which is the reading of P3 under which the encounter has a safe terminal
  set at all. Position-only target uncertainty is fully covered.
- **Not proven.** Physical (nonlinear Fiala) plant containment; any
  real-time property; behaviour after a `carriedWitnessRejected` or
  `inconsistentObservation` report, which ends control.

## 7. Implementation map

| Object | Location |
| --- | --- |
| Ego box conditioning | `hardEncounterBarrier.validateTransition` → `localConditionBox` |
| Target box conditioning | `targetPrediction.conditionExact` |
| Carried data | `hardEncounterBarrier.carriedData`; certificate fields `stages`, `cellStage`, `cellFrames`, `cellNormals`, `terminal` |
| Candidate verification | `hardEncounterBarrier.verifyCandidate` (prescribed stages via `ltvBicycleModel.finitePredict`, prescribed charts/normals via `avoidanceSafetyGeometry.build`, prescribed terminal via `completionRows`) |
| Terminal set | `localTerminalSet`, `localTerminalDynamics`, `localBudget`, `localAdmissibleNormal`, `localFutureSupport`; membership `hardEncounterBarrier.terminalMembership` |
| Value LP / CLF SOCP | `solveHardCbfClf.solve`; violation columns in `avoidanceStageQp` (`violationIndex`, `rowMap.violation`, `rowMap.budget`) |
| Verification and value | `solveHardCbfClf.certify` (`value`, `stageViolation`, `hardRowViolation`) |
| Acceptance rule | `collisionAvoidanceController` (`certificateSource`, `pcbfValue`, `candidateValue`, `pcbfDescentResidual`, `lexicographicTieResidual`, `freshSolveFailure`) |

Reported flags: `recursiveFeasibilityClaimed = true` and
`indefiniteRecursiveFeasibilityClaimed = true` with
`recursiveFeasibilityScope = "declaredAffineStagePlant;exactTargetLaw;boundedEstimationError;conditionedInformationSets"`;
`physicalVehicleGuaranteeEstablished = false`; `terminalActive` is true only
when the terminal law is the issued command (no optimized stage remains);
`fallbackUsed` is always false — the carried witness is the program's own
feasible solution, not an external default.

## 8. Validation

See [the results record](../scripts/INFORMATION_STATE_PCBF_RESULTS_20260912.md)
for the scenario runs (exact and noisy estimates, forced fresh-solve
failure) and the regression suite outcome.
