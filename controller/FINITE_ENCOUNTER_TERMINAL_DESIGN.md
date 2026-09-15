# Finite-encounter terminal certificates with recursive feasibility

> Historical design: the current online algorithm is [the single-solve sampled CBF–CLF controller](SINGLE_SOLVE_CBF_CLF.md). Its one-hold guarantee does not use the continuation architecture analyzed below.

Updated September 14, 2026. The finite open-loop witness is implemented in
certificate version 22 with `N=L=M`, current observation confirmation,
non-postponable active deadlines, zero physical safety violation, and a
road-only invariant terminal controller using the lower speed endpoint and
coupled nominal/error excursion budgets. The general policy-predecessor
construction and separate `L<M` tail below remain design extensions.
[INFORMATION_STATE_PCBF.md](INFORMATION_STATE_PCBF.md) specifies the executed
algorithm; the results record identifies the validations actually run.

## Design decision and scope

Replace the requirement to remain in one target-safe parking halfspace
forever with a finite, robustly safe continuation to a **certified handover
to the target-free problem**. The terminal object is a sequence of joint
information sets, with a remaining-step counter, rather than one fixed ego
position set. Terminal states may have nonzero speed; they need a feasible
continuation, not an imposed stop or exact attainment of cruise.

This construction can prove recursive feasibility and collision/road safety
for an admitted encounter until its guarded discharge. It does not by itself
prove safety or admission feasibility when another target appears, or when
the discharged target returns. For that stronger closed-loop statement, an
entry-compatible invariant domain is additionally necessary. In particular,
“outside sensing range” is not a physical invariant set.

There are two independent sources of conservatism: the chosen terminal
family, and the size/structure of the supplied target reachable sets. A
finite terminal continuation removes the all-future fixed-halfspace
restriction but can still be infeasible for broad uncertainty or an
unavoidable encounter. It is not automatically a superset of the old
terminal domain. No disturbance bound may be suppressed to manufacture a
certificate.

## What the literature establishes

Huang, Wang, Margellos and Goulart, *Predictive Control Barrier Functions:
Bridging model predictive control and control barrier functions* (2025),
Section III, equation (8), Lemma III.1 and equations (9)-(11), use a feasible
shifted sequence and a safe terminal continuation to establish invariance
and safety-value descent. Their terminal control need not be a parking
policy. Their result does not establish this project's uncertain hybrid
extension automatically. [Primary source](https://arxiv.org/html/2502.08400v2#S3).

Batkovic, Gupta, Zanon and Falcone, *Experimental Validation of Safe MPC for
Autonomous Driving in Uncertain Environments* (2023 preprint), Assumptions
4-5 and Proposition 1, distinguish consistent obstacle predictions from
terminal safety. Section III-B explicitly considers objects outside sensor
range or occluded. This supports keeping prediction consistency and unseen
entry assumptions visible in the design. It does not justify dropping an
object and assuming it can never matter again.
[Primary source](https://arxiv.org/pdf/2305.03312).

The construction and proof below are project-specific sufficient conditions.
No maximality, continuous-CBF regularity or real-time result is imported from
those papers.

## 1. Information state and robust successor

Write the information state as

    I = (X, Z, u_previous, activeMode, t, D).

`X` is the ego state box, `Z` the target state box, and `D` includes the
retained affine generators, road/chart data, motion bounds and certificate
choices. Input memory is necessary for slew constraints. A remaining-step
counter is added below. The paired boxes are outer enclosures of the joint
state; statistical independence is not required.

Let `Succ(I,u)` contain every possible next information state after holding
one common input `u` for `h`, allowing every declared physical uncertainty
and every consistent measurement update. `Sweep(I,u)` denotes all states
reachable during that whole hold. The safe set includes rectangle collision
clearance, road clearance and model/actuator domains.

The control quantifier is **one input chosen from current information,
before the next disturbance or measurement**:

    exists u, for every I_next in Succ(I,u).

It must not be replaced by “for every disturbance there exists a different
input”, nor by assuming a future measurement reveals the exact target.

The existing bounded Cartesian box operator has the required consistency
property when its motion contract is retained:

    Z_(k+1) subset Phi_h(Z_k)
      implies Phi_tau(Z_(k+1)) subset Phi_(tau+h)(Z_k).

Its nonnegative transition matrix and integrated jerk term satisfy the
semigroup identity. Measurement intersections preserve inclusion; they do
not reset the target to an unrelated forecast. Changed larger future bounds
need new admission and are outside the fixed-contract recursion theorem.
Ego generators and geometry must likewise be carried or rigorously covered,
as in the current implementation.

## 2. A guarded terminal handover

Let `D_free` be an information-state domain with a **verified zero-safety-value
certificate for the existing target-free PCBF problem**. This avoids
inventing another controller or a desired acceleration. A conservative
choice uses the current terminal rows with vehicle rows absent; a larger
choice carries an explicit finite target-free plan and its road-only
terminal tail. Membership must be certified, not inferred from a nominal
LQR matrix alone.

A proposed handover at time `T` requires:

1. The ego information state, including its previous input, belongs to
   `D_free`, with the actual target-free witness retained.
2. Every possible target/ego position pair is outside the sensing range at
   `T`, with numerical/sensing-reference margins accounted for.
3. The publication rule and a current complete observation confirm release.
   In the present synthetic adapter, publication requires current radar
   detection; internal track coasting does not publish an invisible target.
4. All holds before handover are certified against the target. No prediction
   timeout or nominal range crossing releases it early.

A sufficient affine exit inequality for a fixed unit direction `n` is

    n' (pTargetCenter(T) - pEgoCenter(T))
      - supportRelativePositionError(n,T) >= R_exit.

For Cartesian boxes the support term is `abs(n)'*(rTargetPosition+rEgoPosition)`;
Frenet chart and reference-point errors must also be included in this project.
Choose `R_exit` above the sensing threshold with its roundoff margin and above
the sum of the body circumradii plus collision clearance. This certifies the
whole position set outside the range ball. It does not prescribe the target's
lane or limit where it may travel. A center outside the ball alone is
insufficient.

The handover goal `G_exit` includes this guard and the retained free-mode
certificate. Completion is absorbing **only in the specification of this
single encounter**; it is not an assertion about the object's later motion.
If confirmed departure occurs early, remove target rows from the still
verified finite suffix and retain its ego-only terminal continuation. Dropping
constraints relaxes that suffix, providing a target-free feasibility witness
without relying on a new solver call succeeding.

## 3. Construct a finite terminal tube by robust predecessors

On the encounter's information state, define the robust safe predecessor

    Pre_safe(C) = { I : exists admissible u such that
        Sweep(I,u) is safe, and
        every I_next in Succ(I,u) belongs to C }.

Define a sequence

    K_0 = G_exit,
    K_(m+1) = G_exit union Pre_safe(K_m).

The goal includes the completed-mode handover described above. Outside that
goal, each `K_m` has an admissible causal policy whose successor belongs to
`K_(m-1)` for every allowed disturbance and measurement. These sets are
hereditary under information-set inclusion when the witness data and input
memory agree: a controller safe for a larger box also covers a smaller box.

In an MPC with `N` optimized stages and a terminal continuation of at most
`m` holds, require the terminal information enclosure to lie in `K_m`.
The invariant object is the augmented tube

    union over m of (K_m x {m}),

with the counter decreasing at each terminal-policy step and with a verified
transition to the free-mode domain. Individual slices do not have to be
invariant parking sets. Target uncertainty only needs to be propagated over
the finite remaining encounter, rather than maximized over infinite time.

This predecessor definition gives an exact conceptual condition; computing
the full high-dimensional set is not proposed as an online algorithm.

## 4. A realizable implementation within the existing framework

A practical inner approximation is a **carried finite control witness**:

- Choose a feasible absolute exit time `T`, optimize/verify the finite input
  sequence up to it, and attach a verified target-free witness there.
- Use the existing swept rectangle/road rows, derivative-bound propagation,
  actuator and slew rows throughout that sequence. The finite safety tail and
  handover conditions are hard. The existing PCBF safety value and CLF/input
  objective remain distinct.
- Enforce the robust exit inequality at `T`. Retain its direction, the stage
  generators, cell charts/normals, node boxes and target-free terminal data.
- At the next sample, intersect the measurements with the carried prediction,
  discard the executed input, and verify the suffix on the smaller sets.
  Replanning can replace it only with a separately verified certificate.
- Do not renew an unproved completion deadline at every frame. With
  `r_k=(T-t_k)/h`, one implementation chooses
  `N_k=min(N_configured,r_k)` and terminal-tail length `m_k=r_k-N_k`.
  Both planned stages and finite tail count toward the retained deadline.
  One remaining hold and the zero-hold handover need explicit handling.

`T` is selected from a feasible witness, not a user-imposed early recovery
threshold or a prescribed 9-second settling target. Its purpose is to keep
finite completion meaningful. Merely proposing completion two seconds ahead
at every frame can postpone it forever. If delays or a changed contract
require a different deadline, the old proof cannot silently be reused.

A fixed open-loop tail is a conservative inner approximation of the causal
feedback predecessor sets. If it is too restrictive, a small verified family
of information-dependent tails can enlarge it; the branch must depend only
on information available when its input is issued. Future observer accuracy
cannot be assumed to improve unless its worst-case update bound is proved.

The terminal requirement is “able to continue safely in the no-target
problem”, not `speed=0` or `speed=referenceSpeed` at a chosen early time.
Restoring cruise remains the CLF/performance task. A softened CLF or bounded
persistent measurement noise does not alone establish exact asymptotic cruise.

## 5. Conditional recursive-feasibility proof

Assume:

- Initial admission provides a verified finite witness and free-mode tail.
- Actual ego/target motion and measurements satisfy the declared enclosures;
  the fixed or smaller target-motion contract and carried-data consistency
  hold at every common absolute prediction time.
- Inputs are executed with the timing and previous-input contract used in
  the certificate. All finite tail/handover rows are hard; safety-value zero
  is required for a collision-free initial admission.
- The exit publication guard is valid, the free-mode certificate transfers
  under ego-box inclusion, and no unmodelled entry occurs before this
  encounter's guarded handover.

**Proposition.** The finite-encounter problem is recursively feasible until
handover; every admitted zero-safety-value trajectory satisfies the swept
collision/road constraints and hands over by its retained finite deadline.
The target-free problem is feasible at handover.

**Proof.** Apply the first verified input. The actual successor belongs to
its reachable set. Every consistent measured successor box is a subset of
the carried successor enclosure. On that smaller information set, the
remaining inputs with their own generators and geometry satisfy the same
hard inequalities and require no greater safety violation. At the end of
the optimized prefix, the terminal continuation either selects a causal
predecessor-policy input or consumes the next input of the verified finite
witness; its successor is in the next tube slice. Thus a feasible suffix
exists at the next frame. The count of holds to `T` decreases by one and
cannot be reset by a performance replan. Induction reaches `K_0=G_exit` at
or before `T`. The exit guard discharges this encounter, and ego-box
inclusion preserves the attached free-mode witness. For an early confirmed
release, deleting target constraints from the safe suffix gives the same
handover conclusion. Every executed hold is covered by its swept constraints,
not just by safe endpoints. This proves the stated scope. QED.

If the safety value is the sum of stage violations and the terminal tail
has zero safety violation, the same suffix argument gives

    V_(k+1) <= V_k - xi_(0,k)

for a feasible optimum, or for an accepted certificate value bounded by the
suffix value. Positive diagnostic values never authorize execution in version 22;
zero-value safety cannot admit a positive tolerance leak. At guarded completion the supplied free-mode
witness has zero safety value. This is a value-descent/zero-sublevel invariance
argument on the augmented information/certificate state. Calling the result
a continuous classical CBF additionally requires the appropriate value
regularity assumptions, including treatment of discrete mode changes.
It is not an unconditional transplantation of Huang et al.'s theorem.

## 6. What is needed for recursion across all encounters

For a global hybrid guarantee, define a free-mode readiness domain `D_ready`.
It must be invariant under the actual free-mode law while no target is
published, satisfy road/actuator constraints, and have this entry property:

    every admissible first-detection information state generated from
    D_ready belongs to the feasible domain of an active-encounter certificate.

Handover must land in `D_ready`, not merely an arbitrary target-free feasible
state. At first detection, an already justified active-mode witness must be
available. The proof then composes free-mode invariance, the entry property,
finite active-mode recursion and guarded handover. Fresh optimization alone
is not a proof of that entry property.

Sensing range, sampling, acquisition time, observer initialization error,
computation/actuation delay and ego/target maneuver limits all affect it.
For example, a bound of the form

    R_sensor - R_collision > V_close_max*tau
                              + A_close_max*tau^2/2 + positionReserve

can establish clearance until a response starts under those scalar closing
bounds. It is **not sufficient by itself** for a complete avoidance maneuver
or recursive feasibility. It cannot be asserted with `tau=100 ms` while the
pipeline still overruns that budget unless the delay is separately bounded
and modelled.

Without entry feasibility, an appropriate nonreturn premise, or a retained
model of potentially returning/unseen objects, global physical safety does
not follow from sensor disappearance. Bounded jerk alone permits a target
to leave and return. Arbitrary admissible target motions can also include
unavoidable collisions when ego actuation or road space is insufficient.
No choice of terminal-set notation removes this limitation.

The implemented scope is the finite witness and guarded handover for a
single admitted encounter. New-obligation admission is explicitly checked. A general `D_ready`, nonemptiness for
actual NRMM boxes, nonlinear-plant containment and 100 ms execution remain
open obligations, not completed results.

## 7. Adversarial checks and reproducibility

`python scripts/verifyFiniteEncounterTerminalDesign.py` uses exact rational
arithmetic and finite exhaustive checks; it does not run a vehicle:

- Sixteen duration pairs verify the Cartesian matrix/disturbance semigroup.
- A scalar separation example constructs five predecessor slices and checks
  all 498 admissible causal-policy/disturbance paths from their member states.
- A quantifier counterexample distinguishes `exists u for all w` from
  `for all w exists u`.
- A two-step forecast without a terminal continuation loses feasibility at
  the next frame. A separate 20-step example shows completion procrastination
  when the deadline is reset.
- The jerk-bounded separation `d(t)=10+t-t^3/60` exits range 11 by `t=2`,
  re-enters before `t=8`, and crosses collision threshold 1 between `t=10`
  and `t=11`. Finite exit is not nonreturn.
- An estimate centered at distance 6 with radius 2 is not robustly outside
  sensing range 5.

These checks support the algebra and expose omitted premises. They neither
compute a terminal set for the bicycle model nor prove the new construction
nonempty for the saved joint experiment. Those exact checks remain design-level fixtures. The later MATLAB implementation
is described in INFORMATION_STATE_PCBF.md and its own results record.
