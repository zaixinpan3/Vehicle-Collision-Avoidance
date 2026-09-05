# A terminal set for the hard separation rows: how the literature constructs control barrier functions, and what fits here

> **Implementation update (2026-09-04).** The controller now imposes every
> collision and road CBF row as a hard constraint with no safety slack. One
> conic solve jointly minimizes a linear-only exact first-step CLF
> relaxation penalty and input intervention. A newly solved feasible plan
> is accepted directly. The terminal continuation is a hard augmented-state
> row, and a stored hard-feasible fallback plan is shifted,
> consistency-checked, and re-certified before fallback execution. The
> terminal diagnostic is `terminalInvariantCertified`. See
> `PCBF_CLF_ARCHITECTURE.md` for the executable contract. The current
> terminal row uses the support of the controller's complete predicted
> target continuation and imposes no stationary, straight, monotone, or
> curvature assumption on that trajectory.

**Last updated:** 2026-09-04 — the research survey was written on
2026-09-02, its terminal target contract was corrected on 2026-09-03, and
the executable optimization hierarchy was simplified on 2026-09-04.
Sections 1–6 preserve the survey context; the construction of Sect. 3–4
is implemented in
`PCBF_CLF_ARCHITECTURE.md` and `LTV_BICYCLE_MODEL.md`, with these decisions on
the items Sect. 4 left open: the tail is sampled at the stage period
with its accelerations as decision variables (no quadratic
stopping-distance row, no fixed law — the optimizer supplies the
witness); rest is the only terminal ego behaviour; its fixed position is
separated from the entire predicted future target swept set by an exact
support oracle; the lateral band is the closed-form shift-consistent one,
`γ_d = 1/(2ω)`; and no LMI is solved because none is needed for a
two-state linear system. The gap list items 5 and 9 this note was written
against are closed under the declared premises.

## 1. The question, stated precisely

The controller imposes, at every node `k = 1..N` of a horizon, a hard
separation row on the true configuration obstacle `O_k(e_ψ)` (the
Minkowski sum of the two oriented rectangles), linearized by freezing
the supporting direction at each prescribed start. Each row is sufficient for `dRect ≥ m` at its
node (Lemma 2) and exact at the linearization (Lemma 4). What the rows
do not say is anything about nodes beyond `N`: a plan may end at node
`N` a metre behind a stopped target at 12 m/s and satisfy every row
(gap item 5). The feasible set of the program is therefore not
control invariant, so nothing is proved from one sample to the next
(gap item 9), and the rows are constraints, not a barrier.

A **control barrier function** (CBF) for the ego–target product
system `z = (x, x_T)`, `z⁺ = F(z, u)`, `u ∈ U`, with the target
autonomous under its declared prediction model, is a function `h`
with

    {h ≥ 0} ⊆ Z   (the collision-free, in-road states),
    ∀ z ∈ {h ≥ 0}  ∃ u ∈ U :  h(F(z, u)) ≥ 0                       (CBF)

(the class-K relaxation `h(F(z,u)) ≥ (1−α) h(z)` is a special case).
Its zero superlevel set is a **control invariant subset of the safe
set** under the input box, and that is the whole content of the
phrase "true CBF": a distance function alone is not one, because from
many states with `dRect > 0` every admissible future collides.

Two facts frame everything that follows.

1. **A horizon plus a terminal set IS a CBF.** Let `X_f ⊆ Z` be
   control invariant under some admissible law `κ_f` (`z ∈ X_f ⇒
   F(z, κ_f(z)) ∈ X_f`). Then the set `X_N` of states from which an
   admissible input sequence satisfies the rows at nodes `1..N−1` and
   lands in `X_f` at node `N` is itself control invariant and inside
   `Z` (Mayne et al. 2000 is the MPC statement; Chen et al. 2021,
   Theorem 1 the CBF statement; Wabersich et al. 2023, eq. (28) with
   Assumption 1 the safety-filter statement; Batkovic et al. 2023,
   Proposition 1 the vehicle statement). Its indicator — or, with the
   margins kept, `h_N(z) = max_{u admissible} min_k g_k(z, u)` — is a
   CBF. The controller already has the horizon; the missing object is
   `X_f`, and `X_f` is itself required to be nothing more than a small
   CBF superlevel set with an explicit law.
2. **The horizon does the enlarging; the terminal set only needs to
   be exact.** `X_N` grows monotonically with `N` (Chen et al. 2021,
   Lemma 1; Huang et al. 2025, who describe their MPC as one "that
   increasingly enlarges a local conservative invariant set"), and
   with an adequate backup law it approaches the maximal control
   invariant set (Chen et al. 2021, Fig. 4 against the Hamilton–Jacobi
   set). So "elegant" here does not mean "the largest terminal set";
   it means **the smallest set whose invariance and safety are
   certified exactly, by construction, in the currency the program
   already speaks — affine rows in the plan.** That criterion decides
   the survey.

## 2. The constructions

### 2.1 Analytic barrier chains on the distance (retired here)

Exponential / higher-order CBFs put `h = dRect` (relative degree 2 in
`[δ_f; a]`) through a chain `ψ_1 = ḣ + k_0 h`, `ψ_2 = ψ̇_1 + k_1 ψ_1 ≥ 0`
(Nguyen and Sreenath 2016; Xiao and Belta 2019), or the discrete-time
form `h(x⁺) − h(x) ≥ −γ h(x)` chained along an MPC horizon (Agrawal
and Sreenath 2017; Zeng, Zhang and Sreenath 2021 — the retired
"discrete CBF chain" of this folder's previous design). The certificate
assumes the input needed by the chain is available; under an input
box it is not a CBF at all, and the QP can be infeasible (Breeden and
Panagou 2021, Sect. I; Agrawal and Panagou 2021, Sect. I). Every
input-aware repair below is, in one way or another, a backup
construction.

### 2.2 Input-constrained CBFs (a backup construction in disguise)

- **Agrawal and Panagou 2021 (ICCBF).** Iterate
  `b_{i+1}(x) = inf_{u∈U} [L_f b_i + L_g b_i u + α_i(b_i)]`, starting
  at `b_0 = h`, and take `C* = ∩_i {b_i ≥ 0}`. Each iteration removes
  the states from which safety would need an input outside `U`. The
  paper's own Remark 1 identifies higher-order CBFs as the special
  case with the infimum dropped. Applied to lane keeping and
  obstacle avoidance on a highway bicycle model by Brüggemann,
  Steeves and Krstić 2022 (Sect. III-E, eqs. (27)–(29): "the
  manipulation effectively treats the control term as a disturbance
  and adds a margin of safety ... equal to the disturbance's upper
  bound").
- **Breeden and Panagou 2021.** For a constraint of relative degree
  `r`, fix a law `u*` that dissipates the "generalized inertia" and
  define `H(x) = sup_t ψ_h(t; x, u*)`, the worst value of `h` along
  the flow under `u*` (their eq. (6)); Theorem 1: `H` is a zeroing CBF
  on `{H ≤ 0} ⊆ S`. Their Sect. III-B special case regulates
  `h^{(r)} = −a_max` (constant authority), for which the worst time is
  a polynomial root and `H` is closed form — for `r = 2` this is the
  stopping-distance function `h + ḣ²/(2 a_max)`-type object. That is
  the RSS safe-distance rule of Sect. 2.5 derived as a CBF.

So the input-constrained analytic route ends at the same place as the
next family: a fixed braking-type law whose finite-time flow is
checked against `h`.

### 2.3 Backup-set (implicit) CBFs — the canonical construction

Gurriet, Singletary and Ames (2018, 2019, 2020) replaced the search
for a large control invariant set by two tractable tasks — finding a
controller that stabilizes the system to a backup set, and verifying
that the backup set is invariant under that controller (Gurriet et
al. 2019, abstract). Chen, Jankovic, Santillo and Ames 2021 give the
tutorial form. With safe region `C = {h_C ≥ 0}`, a small control
invariant `S_0 = {h_S ≥ 0} ⊆ C`, a backup law `π`, closed-loop flow
`Φ_π`, and a horizon `T`,

    S = R(S_0, C, f_π, T) = { x : Φ_π(x, T) ∈ S_0  ∧  Φ_π(x, t) ∈ C ∀ t ∈ [0, T] },   (their (9))
    h(x) = min{ min_{t∈[0,T]} h_C(Φ_π(x, t)),  h_S(Φ_π(x, T)) },                    (their (10))

and Theorem 1: `S` is control invariant with `S_0 ⊆ S ⊆ C`, for every
`T > 0`; Lemma 1: `S` is monotone in `T`; Lemma 2: `S = {h ≥ 0}`;
Theorem 2: the CBF-QP with the condition imposed at every sampled
`τ` (their (12)) is always feasible on `S`, because `π` itself is a
feasible point; Theorem 3: `h` has relative degree one under weak
local controllability. The derivative is the sensitivity of the flow,
`Q̇ = (∂f_π/∂x) Q` (their (7)). Their comparative study (Table I,
Fig. 3–4) is the key quantitative fact for the recommendation: on a
3-state Dubins car the Hamilton–Jacobi set is the largest, the backup
set smaller and the sum-of-squares set the smallest; **with a more
aggressive backup law the backup set is "almost identical to the one
from Hamilton Jacobi"**; and the method runs on a 16-state quadrotor,
"way beyond the limit of HJ (maximum dimension 4-5) and SOS (maximum
dimension around 8-10)".

Refinements that matter here:

- **Multiple backup laws.** Rabiee and Hoagg (Automatica 2025;
  arXiv 2305.10620) take the barrier as a soft-minimum over the
  sampled prediction, `h(x) = softmin(h_s(φ(x,0)), …, h_s(φ(x,NT_s)),
  h_b(φ(x,NT_s)))` (their (12)), and combine several backup laws by a
  soft-maximum, with a single affine constraint in a QP/LP that is
  always feasible; the forward-invariant set is the union of the
  individual backup sets. The union is exactly the structure of a
  terminal set with several admissible resting behaviours.
- **Vehicle instances (2025–2026).** Gacsi, Kiss and Molnar 2025
  build braking backup-set/backup-controller pairs for an
  input-constrained four-wheel vehicle "based on feedback
  linearization and continuous-time Lyapunov equations"; van Wijk,
  Daş, Molnar, Ames and Burdick 2025 give a closed-form interpolation
  between a nominal and a backup controller that "is guaranteed to be
  safe while obeying input bounds"; Daş et al. 2026 add parametric
  robustness. The construction is current, and the vehicle-braking
  case is its home ground.

### 2.4 Predictive CBFs and safe-MPC value functions

- **Wabersich and Zeilinger 2022 (TAC; arXiv 2105.10241).** The
  predictive safety filter with per-stage slack; the value function
  `h_PB` is a discrete-time CBF provided the horizon ends in a
  terminal set `S_f = {h_f ≤ 0}` whose `h_f` is itself a CBF under a
  local law (Sect. III-B, Definition III.1, Assumption 1). Their
  Sect. IV constructs `h_f` **by an LMI on the linearization**
  (their (20): maximize the volume of an ellipsoid `xᵀPx ≤ γ_f` under
  a linear law `u = Kx`, with the decrease and the input box as LMI
  rows) followed by two nonlinear verification problems ((21), (22))
  that shrink `γ_x` until invariance holds for the nonlinear model —
  applied in their Sect. V to a kinematic car with steering-rate and
  acceleration limits.
- **Huang, Wang, Margellos and Goulart 2025 (ECC; arXiv 2502.08400).**
  The slack-sum value function `V*` is a CBF (their Theorem 4) with no
  tightening and a non-increasing rather than decreasing value. Their
  terminal sets are deliberately minimal: the LQR ellipsoid
  `xᵀPx ≤ 0.6` in the linear example and **the origin** in the
  nonlinear example (Sect. IV-B: "the terminal set `X_f` is chosen as
  `(x_1, x_2) = (0, 0)`"), the horizon (`N = 10`) doing the
  enlarging. Their introduction contrasts with backup-policy
  enlargement (their [18]): "the size of the enlarged invariant set
  depends heavily on the chosen backup policy, while our formulation
  does not require such a policy" — true of the optimized head, but
  the tail still needs `X_f` and `κ_f`, which is the same object.
- **Breeden 2022 (arXiv 2204.00208).** A "predictive CBF" built by
  propagating the *nominal* trajectory and encoding the worst future
  value of `h` along it (their (11)–(13)); needs no backup set, but
  by its own account (Sect. I) "unlike Backup CBFs, PCBFs do not
  guarantee input constraint satisfaction". Not applicable to a
  hard-input-box design.

Relation to this folder's hard rows: with slack, the same terminal
set yields Huang's PCBF value function; **with the rows hard (the
ruling of 2026-08-30) the CBF is the implicit backup CBF of Sect. 2.3,
whose superlevel set is the feasible set of the hard program with the
terminal rows appended.** Either way the object to construct is
identical.

### 2.5 Terminal sets in vehicle safe-MPC and fail-safe planning

- **Batkovic, Gupta, Zanon and Falcone 2023 (arXiv 2305.03312).**
  Assumption 5: a robust invariant set `X_safe` with a safe input set
  `U_safe` on which the known and the a-priori unknown (obstacle)
  constraints can never be active; Proposition 1: recursive
  feasibility from it. Their instance: **the vehicle at a full stop**
  — "a vehicle is generally considered safe if it has come to a full
  stop", justified by responsibility ("a parked vehicle is not
  responsible for collisions that might occur") and by refining the
  obstacle model so that others avoid a stopped vehicle. The
  stabilizing terminal ingredients are computed **decoupled**: the
  longitudinal states `[v, a]` by an LQR gain, the LMI (30) for the
  terminal cost, and backward-reachable-set iteration for the
  polytopic terminal set (18); the lateral states `[e_y, e_ψ, δ, α]`
  as a velocity-scheduled LPV polytope with the LMI (47) (their
  (19)). Validated on a test vehicle.
- **Pek and Althoff 2018 (IROS), "invariably safe sets".** A state is
  safe if a collision-free trajectory to another safe state exists;
  the recursion yields sets that are safe for an infinite time
  horizon, and "lift safety verification from finite to infinite time
  horizons"; a tight under-approximation is computed in linear time
  in the number of traffic participants. Fail-safe planning (Pek and
  Althoff, T-RO 2021) executes a fail-safe trajectory — typically a
  brake to standstill, or pulling onto the shoulder and stopping —
  whenever the intended motion cannot be verified. (Read through the
  secondary sources listed in Sect. 6; the primary PDF was not
  retrievable.)
- **Responsibility-Sensitive Safety** (Shalev-Shwartz, Shammah and
  Shashua 2017, arXiv 1708.06374), Definition 1 and Lemma 2: the
  same-direction safe longitudinal distance

      d_min = [ v_r ρ + ½ a_max,accel ρ² + (v_r + ρ a_max,accel)² / (2 a_min,brake) − v_f² / (2 a_max,brake) ]_+ ,

  Lemma 3 the opposite-direction analogue, Definition 4 the "proper
  response" (brake at least `a_min,brake` once the distance is unsafe),
  and the induction argument that a proper response never causes a
  collision. This is a closed-form invariably safe set in the
  `(station, speed)` plane, valid under a worst-case braking
  assumption on the other car, and it is precisely the Breeden–Panagou
  constant-authority CBF of Sect. 2.2 evaluated for a following pair.

### 2.6 Hamilton–Jacobi reachability and sum-of-squares

- **HJ reachability** computes the maximal control invariant set and
  the optimal safety policy (Bansal et al.; Choi et al. 2021 CBVF;
  Tonkens and Herbert 2022 refine a candidate CBF, a backup CBF
  included, toward the maximal set with every iteration provably no
  less safe). Wabersich et al. 2023, Fig. 5: pros "maximal safe set,
  safety certification"; cons "curse of dimensionality, indirect
  synthesis of filter". Chen et al. put the practical limit at 4–5
  states. For a single target the *relative* Frenet state
  (`Δs, Δd, e_ψ, v_x`, with the target's course fixed by its model) is
  4-dimensional, so an offline HJ computation is feasible **as a
  yardstick** for the conservatism of any terminal set — not as a
  row in the QP.
- **Sum-of-squares synthesis** (Korda, Henrion and Jones 2014;
  Clark 2021; Dai and Permenter 2023) needs polynomial dynamics and
  constraints; the support function `ℓ|cos| + w|sin|`, the Frenet
  factor `1/(1 − κd)` and the friction polygon are not polynomial, and
  Chen's comparison found SOS the most conservative of the three.
  Not recommended.
- **Learned CBFs** carry no certificate and are outside this
  repository's standards.

### 2.7 Where the surveys converge

Hsu, Hu and Fisac 2024 (Annual Review of Control, Robotics, and
Autonomous Systems): "every safety filter can be viewed as relying on
a fallback policy" (their footnote 6); model predictive shielding
(Sect. 3.3.1) monitors whether "the fallback policy would safely
drive the system state into `Ω` [a terminal safe set, typically small,
e.g., at-rest states] within the prediction horizon", and "if given
the maximal safe set `Ω*` and an optimal safety policy `π*`, MPS
recovers the least-restrictive filter". Wabersich, Taylor, Choi,
Sreenath, Tomlin, Ames and Zeilinger 2023 (IEEE CSM): the predictive
safety filter (28) requires only a terminal control invariant set
(Assumption 1), "a trivial choice for `S_trm` is any equilibrium
point", control-Lyapunov ellipsoids are "less restrictive", and the
backup-set CBF literature "shares conceptual elements with predictive
safety filters by using a backup set that can be kept forward
invariant with a backup controller to implicitly define a larger
control invariant set". Three communities, one object: **a fixed,
analytic fallback law; a resting set that is invariant under it and
safe by an explicit assumption on the other agents; and a finite
rollout that connects the two.**

## 3. Verdict

The most elegant construction available, by the criterion of
Sect. 1, is the **backup-set construction in its discrete-time form**:

    X_f := { x_N :  the rollout of x_N under the analytic backup law κ_b
                    satisfies every separation and road row at tail nodes
                    N+1 .. N+N_b, and reaches a resting set X_0 at N+N_b },

with `X_0` a set that is invariant under `κ_b` trivially (standstill
under zero input; or the cruise equilibrium under the Riccati law
inside its certified ellipsoid) and safe by a declared, checkable
condition on the target's predicted path. Reasons, in order of
weight:

1. **Certified by construction, in the program's own currency.** On
   the affine stage map with an affine backup law the tail states are
   affine in `x_N`, hence affine in the plan `u`; the tail rows are
   the same frozen-support rows (Lemma 2 makes each
   sufficient at any linearization), and the terminal CBF
   `h_f(x_N) = min_j g_{N+j}(x_N)` is concave piecewise-affine. No
   ellipsoid has to be fitted around a non-convex obstacle, no
   verification NLP (Wabersich's (21)–(22)) has to be solved, and no
   offline table is looked up. The invariance proof is Chen's Theorem
   1 provides the backup argument used in `PCBF_CLF_ARCHITECTURE.md`;
   the argument extends to node `N` because the shifted
   candidate's new last input is `κ_b(x_N)`, which is exactly Huang's
   candidate (10) and Wabersich's (28) induction.
2. **It closes gap items 5 and 9 with a theorem rather than a
   horizon length.** With the tail rows the feasible set of the hard
   program is control invariant under (E), (T): recursive feasibility
   holds per episode, and the rows become a barrier.
3. **The conservatism is the horizon's to remove, and it does.** The
   backup set is only the seed; `X_N` is its `N`-step backward
   reachable set (Chen, Lemma 1), and with a braking law at the
   admissible deceleration Chen's Fig. 4 shows the seed already
   nearly maximal. HJ on the 4-state relative system is the honest
   yardstick if this is ever to be measured.
4. **It respects the folder's rules.** The backup law is the same
   everywhere (no situation test); a union of resting behaviours is
   one more level of the existing branch and bound, not a case in the
   code; safety carries nothing between samples; nothing is weighted.
5. **It is the construction the vehicle literature actually deploys**
   (Batkovic 2023 on a real vehicle; Pek and Althoff's fail-safe
   planner; RSS as an industry rule), and the construction the CBF
   literature has spent 2021–2026 hardening for input-constrained
   vehicles (Sect. 2.3).

What is *not* the most elegant here: an LMI-fitted terminal ellipsoid
for the whole product state (the obstacle side of `X_f` is not
convex, and the ellipsoid must then live inside one facet's
half-plane, which is a branch decision the LMI cannot see); HJ tables
(dimension and the QP's need for affine rows); SOS (non-polynomial
data); any per-node class-K decay row (it re-introduces a rate
parameter and certifies nothing under the input box).

## 4. Implemented construction for this vehicle

The dynamic Frenet bicycle predicts the free head. A kinematic braking tail
avoids its low-speed cornering singularity and reaches rest. Tail
accelerations are independent decision variables, sampled at the controller
period. Collision and road constraints cover every head and tail node;
collision freedom is defined at those discrete nodes.

`terminalLateralCertificate` supplies the closed-form arclength lateral band,
with shift-consistent drift factor `1/(2 omega)`. Steering, lateral
acceleration and axle braking budgets bound the terminal heading, lateral
velocity and yaw-rate errors. This two-state construction requires no LMI.

The resting ego is separated from the support of the target predictor's
complete continuation. The support oracle evaluates the actual continuation;
it does not reject a prediction by a stationary, straight or turning class.
A missing separating witness makes the hard problem infeasible. The fixed
support-direction grid controls conservatism of the resting set.

The shift protocol carries the target continuation, LTV schedule and terminal
lateral reference with the stored plan. Reusing it requires consistent fresh
measurements and a recheck of every hard row, exact node clearance and
terminal condition. `PCBF_CLF_ARCHITECTURE.md`, Sections 3--8, defines these
conditions and the remaining exact-model and handoff premises. The survey's
alternative constructions above are research comparisons; the executable
contains this single resting-backup construction.

## 5. What the construction does and does not prove

Proved, once built: control invariance of the hard program's
feasible set under the declared model (E) and the complete,
shift-consistent target prediction (T), per episode; hence
recursive feasibility and the CBF property of `h_N`. Not proved by
it: anything under plant mismatch beyond the one-step boxes (the
facts of Proposition 2 remain facts), anything about a target that
departs from its published prediction beyond its radii, and any
optimality — the terminal set is a seed, and the measured distance
between `X_N` and the HJ set is the only honest statement of its
conservatism.

## 6. Sources examined

| Source | What was read | Used for |
|---|---|---|
| Chen, Jankovic, Santillo, Ames, *Backup Control Barrier Functions: Formulation and Comparative Study*, CDC 2021 (arXiv 2104.11332) | Sect. II–IV: (9), (10), (12), Theorems 1–3, Lemmas 1–2, Table I, Figs. 3–4 | The construction, its certificate, the HJ/SOS/backup comparison |
| Gurriet, Singletary, Ames, *A Scalable Controlled Set Invariance Framework with Practical Safety Guarantees*, CDC 2019 (Caltech PDF) | Abstract, Sect. I–II | Origin of the backup-set idea |
| Rabiee, Hoagg, *Soft-Minimum and Soft-Maximum Barrier Functions for Safety with Actuation Constraints*, Automatica 2025 (arXiv 2305.10620) | Abstract, Sect. 3–5, eq. (12), Theorem 1 | Multiple backups as a union |
| van Wijk, Daş, Molnar, Ames, Burdick, *Safety-Critical Control with Bounded Inputs: A Closed-Form Solution for Backup CBFs*, 2025 (arXiv 2510.05436) | Abstract | Current vehicle-relevant backup work |
| Gacsi, Kiss, Molnar, *Braking within Barriers: Constructive Safety-Critical Control for Input-Constrained Vehicles via the Backup Set Method*, 2025 (arXiv 2510.15797) | Abstract | Braking backup pairs for vehicles |
| Daş, van Wijk, Molnar, Ames, Burdick, *Robust Adaptive Backup Control Barrier Functions*, 2026 (arXiv 2607.20842) | Abstract | Robustness extensions exist |
| Breeden, Panagou, *High Relative Degree Control Barrier Functions Under Input Constraints*, CDC 2021 (arXiv 2106.10345) | Sect. II–III: (6)–(9), Theorem 1, Sect. III-B (16)–(24) | Input-constrained CBF = backup flow; closed-form constant-authority case |
| Agrawal, Panagou, *Safe Control Synthesis via Input Constrained Control Barrier Functions*, CDC 2021 (arXiv 2104.01704) | Sect. II–III: (7), (11)–(14), Theorem 1, Remark 1 | ICCBF iteration and its relation to HOCBF |
| Brüggemann, Steeves, Krstić, *Simultaneous Lane-Keeping and Obstacle Avoidance by Combining MPC and CBFs*, 2022 (arXiv 2204.06136; local `docs/`) | Sect. III-E (27)–(29), Sect. IV | ICCBF on a highway vehicle |
| Wabersich, Zeilinger, *Predictive control barrier functions: Enhanced safety mechanisms for learning-based control*, TAC 2022 (arXiv 2105.10241; local `docs/`) | Sect. III-B (Def. III.1), Sect. IV (20)–(22), Sect. V | LMI terminal CBF and its verification steps |
| Huang, Wang, Margellos, Goulart, *Predictive Control Barrier Functions: Bridging MPC and CBFs*, ECC 2025 (arXiv 2502.08400; local `docs/`) | Sect. I, III, IV-A/B | Minimal terminal sets; contrast with backup policies |
| Breeden, *Predictive Control Barrier Functions for Online Safety Critical Control*, 2022 (arXiv 2204.00208; local `docs/`) | Sect. I, III: (11)–(13) | Nominal-propagation PCBF; no input-box guarantee |
| Batkovic, Gupta, Zanon, Falcone, *Experimental Validation of Safe MPC for Autonomous Driving in Uncertain Environments*, 2023 (arXiv 2305.03312; local `docs/`) | Sect. II (Assumption 5, Proposition 1), Sect. III-C (18)–(20) | Full-stop safe set; decoupled LMI/LPV terminal ingredients |
| Pek, Althoff, *Efficient Computation of Invariably Safe States for Motion Planning of Self-driving Vehicles*, IROS 2018 | Secondary only: abstract via search, and the restatement in arXiv 2510.06717 (definition, safe-distance rule, shoulder-stop fallback) | Invariably safe sets; primary PDF not retrievable |
| Shalev-Shwartz, Shammah, Shashua, *On a Formal Model of Safe and Scalable Self-driving Cars*, 2017 (arXiv 1708.06374) | Sect. 3.1–3.5: Definitions 1–8, Lemmas 2–4, the induction argument | Closed-form safe distances and proper response |
| Wabersich, Taylor, Choi, Sreenath, Tomlin, Ames, Zeilinger, *Data-Driven Safety Filters: HJ Reachability, CBFs, and Predictive Methods for Uncertain Systems*, IEEE CSM 2023 | PSF section (28), Assumption 1, terminal-set remarks; Fig. 5; the HJ↔backup and backup↔PSF paragraphs | Cross-family comparison |
| Hsu, Hu, Fisac, *The Safety Filter: A Unified View of Safety-Critical Control in Autonomous Systems*, Annu. Rev. Control Robot. Auton. Syst. 2024 (arXiv 2309.05837) | Sect. 2.2, 3.3.1–3.3.3, footnote 6 | The fallback-policy view |
| Tonkens, Herbert, *Refining Control Barrier Functions through Hamilton-Jacobi Reachability*, IROS 2022 (arXiv 2204.12507) | Abstract | HJ as the yardstick |
| Yang, Zheng, Ge, Ma, *Safe and Nonconservative Contingency Planning for AVs via Online Learning-Based Reachable Set Barriers*, 2025 (arXiv 2509.07464) | Abstract | Reachable-set barriers for contingency planning (not adopted) |
| Li, Zhang, Guo, Lenzo, Guo 2023; Ge et al. 2022; Zeng, Zhang, Sreenath 2021; Mayne et al. 2000 | As already cited in `PCBF_CLF_ARCHITECTURE.md` | Context |
| Nguyen and Sreenath 2016; Xiao and Belta 2019; Agrawal and Sreenath 2017; Boyd et al. 1994; Korda, Henrion and Jones 2014; Clark 2021; Dai and Permenter 2023; Choi et al. 2021; Bansal et al. 2017 | Not examined in this pass; cited from prior knowledge as the standard references for the family named | Context only |
