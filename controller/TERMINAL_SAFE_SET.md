# Joint encounter terminal set and terminal controller

## State and task boundary

The terminal condition belongs to the **joint encounter**, not to an ego speed
or a parking location. Its state is

    z = (x_E, q_C, a, u_previous),
    x_E = (p_E, psi_E, v_x, v_y, r),
    q_C = (p_C, psi_C, V_C, A_C, beta_C, l_r,C, rectangle_C).

The flag `a` is one while this encounter is active and zero after its first
exit from the ego perception disk, whose configured radius is 50 m. At each
constrained node or midpoint it evolves as

    a_next = a AND (norm(p_C_next - p_E_next) <= R).

During the parked phase the first exit is computed in continuous time. An
ended flag cannot be cleared by predicting a later re-entry. A fresh actual
observation after an actual exit creates a new encounter, with a new
initial-feasibility question. A missing measurement alone is not evidence
of an exit: only a complete current empty scan or a supplied observation
proved outside the disk clears the runtime forecast.

This boundary imposes no arbitrary time limit. If the target never leaves
the disk, it remains part of the encounter. A curved target that stays in
range cannot be dismissed just because it turns or the ego stops.

`jointState`, `jointSuccessor`, and `jointMember` expose this state and its
transition. `member(context,x,node)` and `rows` are the RTI adapters: the
context supplies the target at the same absolute time and the ended flag.
The controller's `encounterId` advances only on actual reacquisition; a
predicted exit does not erase a currently observed target.

Terminal membership certifies an available continuation. It neither commands
parking nor switches off MPC. Moving cruise states may belong to the set.
Normal operation still aims to avoid the target and recover the prescribed
path and speed. This use of a terminal candidate follows Huang et al. (2025),
Section III, equations (8) and (10),
[primary source](https://arxiv.org/abs/2502.08400).

## One terminal controller

In the accepted mild kinematic model, retain the steering and apply a fixed
braking command, with a forward-only absorbing stop:

    b_f = -a_b / longitudinalAccelerationGain,
    delta_f = latched steering,
    dV/dt = -a_b while V > 0; V(t) = 0 after stopping.

`control(context,joint)` returns this command and its modeled acceleration.
The command does not depend on an unknown true speed to choose the last
hold. There is no final weak-braking hold and no reacceleration after rest.
The default `a_b` is 4 m/s^2. With initial speed V and elapsed time t,

    tau = min(t, V/a_b),
    s(t) = V*tau - a_b*tau^2/2,
    t_stop = V/a_b,       s_stop = V^2/(2*a_b).

The stop may occur inside a sampling hold. `continuation` evaluates this
flow at nodes and midpoints; the parked-track check starts at the actual
stop instant, not at the following sample boundary. `successor` is its ego
projection and `jointSuccessor` also advances the target and exit memory.

The mild-model assumption keeps beta_E = atan2(v_y,v_x) and kappa_E = r/V
constant while moving. Consequently

    p_E(t) = p_E + s*sinc(kappa_E*s/2)
                   * direction(psi_E + beta_E + kappa_E*s/2),
    psi_E(t) = psi_E + kappa_E*s.

At rest the pose is fixed. The target uses the same constant tangential
acceleration and sideslip prediction as the main controller, with its own
forward-only stop. There is no backward extrapolation through a stop.

This is the declared terminal model, not an identity with the Fiala plant.
In particular, a held actuator ratio and road loads do not independently
prove exact constant deceleration or curvature in the full plant. A robust
claim needs the compatible terminal uncertainty contract described below.

## Terminal set

Let Phi_f be the joint flow including the absorbing encounter flag. Let C
require the declared physical/input conditions and any road constraints,
and require rectangle separation by d_margin only on active branches.
Then the terminal set is the fixed-policy safe set

    X_f = { z : x_E is mild and Phi_f(z,t) satisfies C
                 at all declared constraint instants }.

Here mildness requires |v_y| <= tan(beta_f)*v_x and |r|*V <= a_m, with
beta_f = 0.05 rad and a_m = 2 m/s^2 by default. The braking ratio must satisfy
actuator and rear-adhesion limits. Since |kappa_E|*V(t)^2 decreases and beta_E
is constant, the mild conditions persist under the declared flow. Input
memory is part of the joint state; finite slew limits require an admissible
entry command, not just a geometric certificate.

The executable membership test checks:

1. Mildness and terminal braking admissibility.
2. Ego road occupancy at nodes, midpoints, and its parked pose.
3. Joint rectangle clearance during braking until the first exit.
4. If the ego stops before exit, its parked rectangle against the target's
   remaining in-range track, beginning at the exact stop time.

The fourth check is necessary: a target can collide with the ego shortly
after it stops while both remain in range. It excludes all motion after the
first exit. The check is analytic: target vertices sweep line segments or
circular arcs against ego edges, and vice versa. A target that stops inside
the disk contributes its stationary endpoint. If a circle stays wholly
inside the disk, its swept track still belongs to this encounter.

No integer terminal index is interpreted as an execution deadline. The
prediction endpoint can be a moving mild state with an available braking
continuation; the main controller need not choose that continuation.

## Shift closure and the role of uncertainty

For exact information under the declared terminal contract, take z in X_f.
The continuation of Phi_f(z,h) is the suffix of the original continuation:
the clipped-speed flow has the semigroup property, the target is advanced
by the same elapsed time, and input/encounter memory are retained. Every
active successor row is therefore an old certified row. Once the encounter
ends, no future prediction can reactivate it. Hence Phi_f(z,h) is in X_f.
Appending this rule to a shifted feasible plan supplies the usual
recursive-feasibility witness. A soft performance objective does not remove
that candidate. This is an existence statement for the ideal problem;
local RTI solution accuracy and deadlines are separate.

For the joint observer-controller design, replace a point by a nonempty
joint information set J containing physical ego and target states, shared
model parameters, observer state, input memory, and the encounter flag.
The terminal family is

    B_f = { J : one common causal terminal policy satisfies all hard rows
                on every admissible branch until that branch's first exit }.

The flag is branch-specific: a center outside range does not certify that
all possible targets have exited. A computed outer set may discard the
target only after every remaining active branch has exited. An unknown
parameter remains the same along each branch. Model mismatch already
represented in the current observer enclosure is not added again as an
independent initial error; future physical model defects must still be
propagated jointly with the observer and feedback.

For a common held braking rule with guaranteed realized deceleration
at least a_min > 0, every branch with V <= V_bar stops by V_bar/a_min and
travels at most V_bar^2/(2*a_min). Pose uncertainty remains after stopping.
All reachable parked poses and all still-active target branches must be
checked. A disturbance that can move a parked vehicle needs a compatible
parked invariant set. Holding an invalid low-speed observer bound constant
is not a substitute for this physical continuation certificate.

The terminal family is downward closed. Its exact joint image remains in
B_f because the same policy tail covers every successor branch, including
its input and encounter memory. A certified outer successor must likewise
belong to B_f. This is the terminal-closure step used in
[ROBUST_RECURSIVE_FEASIBILITY.md](ROBUST_RECURSIVE_FEASIBILITY.md) and
[CLOSED_LOOP_PREDICTION_THEORY.md](CLOSED_LOOP_PREDICTION_THEORY.md).

The runtime RTI is still an approximation to this information-set problem.
Its terminal geometry uses the target center and the published ego pose
padding; it does not implement a universally certified full nonlinear
terminal reachable set. This redesign does not promote those margins into
a robust certificate. `robustNonlinearSafetyCertified` remains false.

## RTI and implementation

The prediction prefix, terminal adapter, and joint transition use the same
first-exit convention. A prefix exit is latched only when the separation
lower bound remains outside R after subtracting both published position
enclosures and the position correction allowed by the RTI trust box. Thus
an affine correction cannot undo that exit. All later collision rows for
this encounter are absent, including terminal target rows. Physical and
road rows remain. This is an affine exit certificate, not a guarantee about
the nonlinear optimized rollout.

`rows` builds the local terminal approximation once. At circular-track
intersection, a nonconvex safe set can admit an inner or an outer local
separation chart. The implementation selects the smaller radial/support
escape using the actual joint geometry, independently of road-side
preferences. These are two local descriptions of the same set under the
same terminal policy; they are not two driving controllers or extra RTI
iterations. Neither chart nor its linearization is a global certificate.

An inherited RTI tail whose endpoint is terminal first tries the certified
cruise-feedback hold. The existing `brakingInput` helper then uses a bounded
set of declared-model holds and five scalar halvings to find a less
restrictive terminal-preserving seed. This numerical initializer is distinct
from the unique `control` law used to define the set. It adds no optimizer
iterations. If no certified hold is found, a moving endpoint uses the
nominal-feedback seed; an already stopped endpoint keeps a nonpositive
longitudinal command. This prevents an uncertified restart from creating a
positive-input trust box while preserving ordinary cruise recovery. The
optimizer still chooses and issues the final input unchanged.

Tests cover subsample stopping without reversal, exact-flow agreement,
joint target dependence, first-exit memory through predicted re-entry,
post-stop collision before exit, fresh observed encounter epochs, prefix
exit propagation to terminal rows, and continued cruise from a state that
already belongs to X_f. These are implementation checks under their stated
models, not empirical proofs of robust recursive feasibility.
