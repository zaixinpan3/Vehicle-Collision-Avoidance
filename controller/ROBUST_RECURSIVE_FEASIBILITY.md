# Recursive feasibility with bounded estimation error and direct execution

This note proves feasibility of an ideal mathematical control problem. It does
not promise that a numerical solver returns a command. The input is the first
input of the completed optimization, without post-solve scaling or a
feasibility veto. Estimation error is retained throughout the proof.

The proof uses joint information sets and a zero-slack robust terminal
continuation. It replaces the earlier argument that assumed a per-hold tube
growth condition without constructing a shift-consistent terminal tube. The
current executable controller issues its convex-program inputs directly but
does not implement the joint set recursion below. Consequently the theorem
must not be attributed to its point-estimate, one-linearization problem.

The practical design objective is to include future observer updates and
feedback in prediction, avoiding excessive tails from frozen-input open-loop
propagation. It does not require a constant-width invariant tube. The
complementary `CLOSED_LOOP_PREDICTION_THEORY.md` describes varying finite-horizon
joint tubes and shifts causal control policies rather than only held-input
sequences. Its target prediction retains correlations between future
observations and ego response. Fixed-width constructions are optional
diagnostics. That proposed formulation is also not yet the runtime algorithm.

## 1. Separate feasibility from optimality

Recursive feasibility means that a feasible problem at sample k implies a
feasible problem at sample k+1 after applying the first input of a feasible
plan. A claim about the decrease of the *optimal* slack sum additionally
requires applying a safety-optimal plan. A second performance stage may choose
among safety minimizers; in the exact problem it must keep the same primary
optimum. Solver time, numerical precision and the gap between a local convex
problem and the exact problem are outside this existence statement.

This is the distinction used in Huang, Wang, Margellos and Goulart (2025),
*Predictive Control Barrier Functions: Bridging Model Predictive Control and
Control Barrier Functions*, Lemma III.5 and equations (9)--(11),
[primary source](https://arxiv.org/html/2502.08400v1).

## 2. The information set contains the true state

Let z include the ego state, target state and motion parameters, and the
previous held input. The input memory makes slew bounds state-dependent hard
constraints rather than an unmodeled boundary condition. A finite encounter
may include an absorbing ended flag; re-acquisition is a new initial-feasibility
problem. Alternatively impose collision constraints for the whole forecast,
including re-entry. Do not change this convention between successive problems.

Let T(J,u) be the exact one-hold reachable set from a joint set J under a
common held input u. It includes any *declared* plant-defect set. T_s(J,u)
is the corresponding image at every constrained fraction s of the hold. The
held-input maps and their defect bounds must be the same at consecutive
samples and must cover the declared nonlinear model, not only its tangent.
These maps are monotone:

    J' subset J  =>  T(J',u) subset T(J,u),
                     T_s(J',u) subset T_s(J,u).

The estimator supplies a joint enclosure Q_k containing the truth. Its ego
part may be xhat_k + E_k with E_k obtained from the ISS certificate. Relative
measurements and common ego-pose error can be retained as joint correlations;
independent outer products are permitted if they still contain the truth.
Missing measurements are represented by the full declared domain.

Use the information recursion

    I_0 = Q_0,
    I_(k+1) = T(I_k,u_k) intersect Q_(k+1).                 (1)

Induction proves that the truth belongs to I_k and I_k is nonempty whenever
the model/defect contract and estimator enclosures are sound. It also proves

    I_(k+1) subset T(I_k,u_k).                             (2)

Equation (2) is the cross-sample inclusion needed by the shift proof. The new
estimate need not equal the old predicted center. There is no assumed decay of
future uncertainty and no assumption that successive raw observer balls are
nested. A smaller ball with a shifted center is not necessarily a subset.

The target projection S_k of I_k obeys
S_(k+1) subset phi_h(S_k). The target flow is a semigroup under its rest rule,
so phi_(t-t_(k+1))(S_(k+1)) subset phi_(t-t_k)(S_k) at the same absolute time.
At rest the published acceleration state is reset to zero; propagate before
intersecting rather than intersecting pre-stop and post-stop acceleration
intervals as if that state were an unchanged latent parameter.

## 3. Stage conditions are monotone in the information set

H(J,u) denotes the road, physical-state, handling, actuator and slew conditions
for every joint state in J and every declared defect, at the declared nodes
and hold midpoints. C(J,u) is the largest nonnegative collision-margin deficit
at those instants. One slack covers a whole hold, including its endpoint; this
fixes the indexing and prevents dropping an endpoint condition during a shift.
For a smaller initial set, the same input has no harder conditions:

    H(J,u) and J' subset J  =>  H(J',u),
    J' subset J  =>  C(J',u) <= C(J,u).                    (3)

These are universal constraints on exact reachable sets. A convex restriction
may represent them only with sound nonlinear and integration remainder bounds.
A fixed-multiplier separating-plane row before yaw linearization is a valid
lower bound. Its first-order yaw tangent without a remainder allowance is not
a general certificate of the requested exact distance. A nonlinear replay of
one estimated center is not verification of the whole set.

## 4. A settled terminal set includes the braking capability

Small lateral acceleration alone does not rule out a large lateral drift. The
runtime terminal set now uses a separate sideslip limit (default 0.05 rad,
about 2.9 degrees), tighter than the estimator operating cone of 0.25 rad, and
checks rear adhesion under the terminal braking ratio. This is a design
setting, not a measured bound on model mismatch.

For the declared kinematic terminal law, beta and curvature kappa are held,
V decreases, and therefore beta stays small and |kappa| V^2 decreases. Also
(v_y-l_r r)/v_x stays constant until rest, so rear adhesion under a constant
braking ratio is preserved while that model applies. The ratio and stored
steering must be admissible, and any slew limit from the last optimized input
must be included. The friction-budget inequality alone proves none of these
input conditions. The executable Fiala/blended model is not identically this
closed-form law; equality must be an explicit modeling hypothesis or the
terminal certificate must be built on the actual declared model.

Define a terminal set B of *joint information sets*, not centers. For each
J in B there is one common admissible braking continuation b=(b_0,b_1,...) whose
reachable states remain settled, satisfy every hard condition, and have zero
collision deficit. The allowed backup family is closed under deleting its
first hold. For example, steering and an admissible brake command held through
and after rest have this property on a forward-only model that stays at rest.
The continuation is a witness for the proof, not an emergency command issued
when a solver fails.

This definition gives two properties, without discarding estimation error:

    J' subset J and J in B  =>  J' in B;                   (4)

    J in B => exists b_0:
       H(J,b_0), C(J,b_0)=0, T(J,b_0) in B.                (5)

Proof of (4): the same continuation controls a subset of the old reachable
states. Proof of (5): the tail (b_1,b_2,...) controls T(J,b_0); every reachable
state and constraint there was already included in the old continuation.
Input memory is propagated in T, so the first appended slew condition is not
silently omitted. Conditions (4)--(5) include the parked phase and all admitted
plant defects, not just a nominal braking arc.

A finite certificate of this definition is possible when all reachable ego
states stop by a common finite bound: verify the hard and zero-deficit rows on
the full set until that bound, then verify invariance of the parked ego set and
its clearance against every remaining target forecast. With an exact
forward-only model and no plant defect, a parked pose stays fixed; its position
and heading uncertainty do not vanish, and its rectangle set is checked
against the target family. With persistent defects capable of moving a parked
vehicle, parked-set invariance requires an additional control/defect condition.
A generic bounded defect is not enough. An infinite safe-backup definition is
not, by itself, a completed finite numerical certificate for the Fiala model.

## 5. The ideal robust problem and the shift proof

For N prefix holds and any allowed M with N <= M <= M_max, choose held inputs
u_0,...,u_(M-1) and nonnegative prefix slacks xi_0,...,xi_(N-1). Define

    R_(0|k) = I_k,
    R_(i+1|k) = T(R_(i|k),u_(i|k)).                       (6)

Impose H at every hold, C(R_i,u_i) <= xi_i in the prefix, C(R_i,u_i)=0
thereafter, and R_M in B. The primary objective is

    V*(I_k) = min sum_(i=0)^(N-1) xi_(i|k).              (7)

The terminal continuation has no slack and contributes no additional cost.
A performance stage may select any minimizer of (7), and the controller
executes its first optimized input exactly. Assume a feasible plan and a
terminal witness exist; assume an optimizer is attained only for conclusions
about V*. Initial feasibility is not asserted for every encounter or every
uncertainty size.

**Theorem.** Under sound current enclosures, the unchanged transition/defect
contract, update (1), stage monotonicity (3), and terminal properties (4)--(5):

1. Executing the first input of any feasible plan preserves mathematical
   feasibility at the next sample.
2. If a safety-optimal plan is executed, then
   V*(I_(k+1)) <= V*(I_k) - xi*_(0|k).
3. The applied hold satisfies the hard rows for the true state, and its
   sampled collision clearance is at least d_min - xi_(0|k). If V*(I_k)=0
   and safety-optimal plans continue to be executed, the zero set is invariant.

**Proof.** After applying u_(0|k), (2) gives I_(k+1) subset R_(1|k).
Take u_(1|k),...,u_(M-1|k) as the next plan's first M-1 inputs. Induction
using monotonicity of T gives

    R'_(i|k+1) subset R_(i+1|k),  i=0,...,M-1.            (8)

All shifted hard rows therefore hold by (3). The old terminal set contains
R_(M|k), so R'_(M-1|k+1) belongs to B by (4). Append its common witness input
b_0. Property (5) supplies all last-hold constraints with zero deficit and a
new terminal set in B. The new plan has the same M, so it respects both the
minimum and maximum horizon lengths.

For i=0,...,N-2 take xi'_i=xi_(i+1|k). For i=N-1 take xi'_(N-1)=0: if
M>N this hold was an old hard-tail hold; if M=N it is the appended terminal
hold. Thus the candidate cost is sum_i xi_(i|k)-xi_(0|k). This proves
feasibility for a feasible executed plan. If it was a primary minimizer,
comparison with the next minimum proves conclusion 2. Truth containment and
H,C prove conclusion 3. Nonnegativity and telescoping give
sum_(j=k)^infinity xi*_(0|j) <= V*(I_k), until the declared encounter ends.
No perturbed-center stopping schedule is identified with a nominal one; every
set, input and terminal condition appears in (8), (4) and (5).

## 6. Conservative outer sets and convex problems

An implemented reachable-set operator T# may replace T if it contains T and
is monotone under set inclusion. Update I_(k+1)=T#(I_k,u_k) intersect Q_(k+1)
and propagate every prediction with the same operator. The proof above still
works on those outer sets if H,C and B are certified against them. This is a
constructive alternative to merely postulating a tube-growth inequality.

An arbitrary per-frame linearization is not necessarily such an operator. Its
remainder must cover the complete decision and uncertainty domain. Nor does a
convex inner approximation inherit feasibility simply because the exact
problem is feasible. To obtain a theorem for successive convex problems,
show that the shifted witness is also feasible in the next convex restriction
(e.g. retain its valid separating certificates and reserves) and define the
value function for that formulation. Local convex optimality does not prove
global optimality of (7).

For an executed plan with cost J_k, the argument proves only

    V*(I_(k+1)) <= J_k - xi_(0|k).

If J_k <= V*(I_k)+eta_k has actually been certified, add eta_k to the
optimal-value decrease inequality. Summability needs sum eta_k < infinity;
a fixed numerical tolerance does not imply that. The code's lexicographic cap
is a tolerance relative to its convex primary problem, not a certified eta_k
relative to the exact robust problem.

## 7. Implementation status

Implemented in this revision: direct CLF-stage inputs; no line search,
acceptance relinearization or post-solve constraint veto; optimized affine
slacks retained; an independent nonlinear prediction for state/trust data;
a separate settled-terminal sideslip bound and terminal-braking rear rows.
Uncomputed nonlinear margins are NaN and nonlinear validation is false.

Not implemented: joint posterior intersection (1), monotone full-horizon outer
reachable sets, and a setwise finite terminal certificate satisfying (4)--(5)
on the declared Fiala/blended model. These are mathematical data of the robust
formulation, not solver reliability assumptions. The supplied observer bounds
can initialize Q_k but cannot replace these operations. Numerical failures in
experiments are permitted by the theoretical scope; a theorem is not claimed
for a different optimization problem merely because such failures are allowed.
