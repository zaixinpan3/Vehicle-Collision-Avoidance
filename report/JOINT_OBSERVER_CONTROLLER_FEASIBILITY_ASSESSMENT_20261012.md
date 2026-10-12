# Joint observer-controller recursive feasibility: achievable goal and missing design

Date: October 12, 2026. Code inspected at
`815cb32565ec6ea4c9c6ffddfae81a854f050855`.

## Finding and status

Nonzero bounded estimation error is compatible with robust recursive
feasibility. No fundamental impossibility has been established for this
project. The project's central result should connect its observer certificate
to an executable robust predictive controller. The current implementation
uses observer information to tighten affine constraints but does not yet
realize the complete joint theorem. The exact-information shift argument also
depends on an admissible terminal continuation matching the declared model;
removing estimation error alone does not close the existing terminal-model gap.

The conditional joint-set proof in
`controller/ROBUST_RECURSIVE_FEASIBILITY.md` already permits nonzero error.
Its outstanding conditions must be constructed and connected to the code,
rather than described as completed properties of the present controller.
Numerical solver failure is outside the requested existence theorem.

## What the existing observer provides

`estimator/nrmmControllerErrorBounds.m` publishes timestamped current ego and
target enclosures, availability, a correlated ego generator when possible,
and a target prediction-parameter set under the zero-model-jerk contract.
`estimator/OBSERVER_ISS_THEORY.md` supplies the conditional error dynamics and
comparison bounds. These are the starting data for the joint proof. They
should not be replaced by zero-error assumptions or an unvalidated generic
padding chosen by the controller.

For an observer-centered prediction, let the plant satisfy

    x_next = F(x,u) + w,     ||w|| <= W,
    ||x - xhat|| <= B,       ||x_next - xhat_next|| <= B_next.

If the same-input plant map has a certified difference bound

    ||F(x,u) - F(xhat,u)|| <= omega_F(B),

then the triangle inequality gives the useful interface bound

    ||xhat_next - F(xhat,u)|| <= omega_F(B) + W + B_next.

On a regular Lipschitz domain omega_F(B) can be L*B. A global L must not be
asserted across the code's discontinuous numerical stop reset: certified
piecewise images or a reset allowance are needed there. This derivation
shows that a center innovation is not an unconstrained new disturbance when
the observer bounds and plant contract are sound. It also shows why a
predictable next-error bound, derived from the observer's comparison dynamics
or a valid worst-case envelope, is needed in this design route. No future
measurement realization or disappearance of sensor noise is assumed.

Alternatively the existing proof uses the information set

    I_next = T_outer(I,u) intersect Q_next,

where Q_next is the observer enclosure and T_outer is a sound monotone
reachable-set outer map. This route retains the past prediction rather than
resetting every nominal initial state to the fresh observer center. It does
not require an exact reachable set. The same certified representation/map
must be used in the constraints and in the posterior update.

## Primary-source comparison

Köhler, Müller and Allgöwer (2021), *Robust output feedback model predictive
control using online estimation bounds*, Section III lists current error,
future error and observer-to-nominal dynamics discrepancy bounds. Section
IV-B, Assumption 7 and Theorem 4 combine them with robust terminal ingredients
and initial feasibility to prove recursive feasibility. This establishes an
available nonlinear design framework; it does not establish its assumptions
for our Fiala/stop model or moving obstacles. Its feedback architecture must
not be silently substituted for direct issuance of our optimized inputs.
[Full primary source](https://arxiv.org/html/2105.03427v1).

Köhler, Kötting, Soloperto, Allgöwer and Müller, *A robust adaptive model
predictive control framework for nonlinear uncertain systems* (2020 extended
version), explains how monotone estimator/model updates and tube properties
preserve feasibility. This supports using consistent enclosing updates rather
than an arbitrary new per-frame tube. It is a parameter-adaptation result,
not our completed output-feedback collision theorem.
[Primary source](https://arxiv.org/abs/1911.02899v2).

## The unavoidable qualification: a bound does not create control authority

Consider the exact scalar plant x_next=2*x+u, |x|<=1, |u|<=1, and a valid
observation xhat=0 with error bound 0.75. Both x=0.75 and x=-0.75 are possible
and currently satisfy the hard constraint. The first requires u<=-0.5 to
remain safe, while the second requires u>=0.5. No common input satisfies both,
although either exact state individually admits an input.

This is an analytical impossibility for that initial information set, not
a counterexample to recursive feasibility starting from a robustly feasible
set. A project theorem should therefore say: if the initial joint information
problem is feasible and the declared contracts persist, all subsequent
problems remain feasible for every admitted observation/disturbance. It cannot
promise a robustly feasible initial plan for arbitrary error widths, road
geometry, encounter onset or target motion. Positive collision slacks do not
remove hard road, actuator or terminal restrictions.

## A concrete terminal certificate is already possible in a restricted family

For the current declared numerical vehicle model, define

    J_park = { [p_x,p_y,psi,0,0,0] : (p_x,p_y,psi) in P }.

P may have nonzero position and heading width. With the common input u=[0;0],
the low-speed model has zero longitudinal force, zero road load at rest,
zero kinematic yaw-rate command and zero body derivatives. Its pose derivative
is also zero. Every RK4 stage is therefore unchanged, and the stop reset
leaves the zero body state unchanged. Hence F(J_park,0)=J_park for the
declared model, independent of the unknown member of P.

If all rectangles in P satisfy the road rows and every admitted target
trajectory stays clear of every rectangle in P, this is a robust invariant
parked terminal information set. Include the previous-input memory: entering
the set must permit the zero input under the configured slew bounds; at
subsequent steps the input memory is zero. The default infinite braking slew
and absence of a steering slew bound permit this input transition. Under a
finite braking slew, restrict the entry memory accordingly or certify a
different nonpositive held braking input at rest.

The family is demonstrably nonempty for a target-free wide straight road
and a sufficiently small nonzero pose box. An uncertain stationary target
sufficiently far away is another simple case. This is a restricted certificate,
not a proof of feasibility for every moving-target family, nor proof that
the current prediction reaches J_park. Certifying a common braking sequence
from the moving endpoint set to J_park is the remaining reachability task.
A known numerical reset can establish zero true velocities for the entire
propagated set even if a fresh noisy observer reports a nonzero velocity
radius: posterior intersection retains the stronger history information.

Measurement error does not physically move a parked car. It must not be
modeled as an arbitrary persistent plant drift solely because the observer
has a nonzero noise bound. If the declared plant includes a defect that can
move a parked car, this particular invariant set needs a different backup
or a rest-compatible defect condition. This certificate concerns the declared
numerical model, not unmodeled hardware behavior.

## Closing the observer-controller assumption loop

The observer certificate has operating-domain and sensor/model premises.
They must hold initially and be maintained by robust controller constraints,
yielding an induction: valid enclosure -> robust admissible input -> plant
remains in the domain -> valid next enclosure. Estimated-domain checks alone
do not establish this induction.

There is a specific stopping-mode interface to address:
`config/estimatorConfigurationFromController.m` sets the ego lower speed to
max(speedMinimum,scheduleSpeedFloor), 1 m/s under defaults.
`estimator/MODEL_AIDED_ESTIMATION.md` limits its Fiala force-balance measurement
model to the supported speed/model regime, while the controller blends below
4 m/s and stops at zero. A joint theorem must not silently apply that
measurement inversion throughout the stopping phase. It can use a certified
low-speed measurement model, or propagate the certified prior/backup set when
a channel is unavailable and intersect only valid measurements. An
unavailable channel then contributes no new restriction, rather than a zero
error radius. The core ISS theorem's admissible zero ego lower speed alone
does not prove the force-balance replacement valid in the blended regime.

The observer's default 5% parameter box also needs a clear shared contract:
if the plant is declared exactly nominal, prediction defect W=0 is legitimate.
If the joint theorem covers every parameter in that box, the predictor must
cover the corresponding plant variation. A wider observation error certificate
does not itself make the plant uncertain, and sensor noise must not be counted
again as a physical process force.

## Project conclusion

The attainable core claim is robust recursive feasibility from a robustly
feasible initial information state, using the observer's certified errors and
preserving their premises. The current exact-information/conditional proofs
are intermediate results. The unresolved technical deliverables are sound
full-hold prediction and update, a nonempty reachable terminal family on the
actual declared model, and shifted-witness preservation in consecutive convex
problems. These are constructive design tasks, not established impossibility
results. Exact enumeration of all possible trajectories is unnecessary.

Direct issuance of optimized inputs can be retained by incorporating the
uncertainty and backup constraints into the optimization. This assessment
does not introduce post-solve correction or a new runtime algorithm. No
complete joint theorem is claimed for the current executable controller.

## Subsequent scope clarification: omit roads and establish uniform bounds

The user clarified that the current theoretical derivation omits road
boundaries entirely. Remove the road part of H in this scoped theorem; this
does not change the runtime road constraints. No road-width or remaining-road
margin condition is an obstacle to the scoped derivation. The joint design
should establish uniform bounds on the current information uncertainty and
the uncertainty within each finite prediction, as well as a certified terminal
extension. Known finite error at each isolated time is weaker than a single
finite bound valid for all time.

The existing continuous observer core already has the relevant structure:

    D+ B_v <= -k_v B_v + d_v,
    D+ B_T <= -lambda_T B_T + c_Tv B_v + d_T.

For fixed positive gains, bounded nonnegative forcing d_v<=d_v_max,
d_T<=d_T_max, and valid finite initial bounds, define

    B_v_max = max(B_v(0), d_v_max/k_v),
    B_T_max = max(B_T(0), (c_Tv*B_v_max+d_T_max)/lambda_T).

At either upper boundary the outward derivative is nonpositive, so this
rectangle is invariant for the positive comparison dynamics. Both bounds are
uniform in absolute time. The target metric bound maps to its physical
component bounds using the positive definite certificate matrix. Initial
transients are retained; the equilibrium radius alone is insufficient.
This is a direct consequence of `estimator/OBSERVER_ISS_THEORY.md`, Section 6,
under that certificate's premises. Absolute pose, heading and reconstructed
parameter channels must retain their own valid uniform bounds; the core
certificate alone is not their proof. A valid gyro-only orientation set can
grow to the full circle, whose angular radius is pi, without an angular
convergence claim. Availability and model/sensing assumptions still matter.

There are two different time variables. The current estimate error is
B(k), measured over absolute control cycles k. The prediction radius is
r(i|k), measured i steps ahead of the information available at cycle k.
Bounded B(k) does not justify restarting r(i|k) from B(k) at every future
hold before those future measurements exist.

Nevertheless, an infinitely contractive open-loop prediction is unnecessary
for a uniformly bounded *finite-horizon* formulation. Suppose the current
observer information Q_k has a uniform size bound B_max, and use the existing
sound posterior recursion I_k=T_outer(I_(k-1),u_(k-1)) intersect Q_k. Then
I_k is contained in Q_k. In a fixed weighted coordinate norm, its diameter
is bounded by twice Q_k's radius. Translation of Q_k's center does not
invalidate this diameter bound or the cross-sample set inclusion.

Suppose a certified, time-uniform, nondecreasing one-hold growth bound Gamma
maps finite uncertainty sizes to finite sizes over the admitted model/input
domain. For the fixed maximum horizon M_max,

    r(0|k) <= B_max,
    r(i+1|k) <= Gamma(r(i|k)),
    r(i|k) <= max_{0<=j<=M_max} Gamma^j(B_max) = R_max < infinity.

This bound is independent of k, even if some predictions expand with i.
For the illustrative linear bound Gamma(r)=L*r+d, its j-step bound is

    L^j B_max + d * sum_{ell=0}^{j-1} L^ell.

L<1 is sufficient for an infinite-horizon bound, but is not necessary for
a fixed finite maximum horizon. The current default maximum is 512 holds
of 0.05 s, or 25.6 s. This corollary assumes a certified uniform growth map;
the current per-hold tangent reset is not its implementation. Across the
discontinuous stop reset, use a certified piecewise/outer map rather than
postulating a global Lipschitz constant. A finite but excessively conservative
R_max does not itself prove collision/handling/terminal feasibility.

For a feedback tube design, a contracting radius recursion
r_next<=a*r+b*B_max+d with a<1 yields the invariant upper bound
max(r(0),(b*B_max+d)/(1-a)). This is an alternative way to keep deviations
bounded. It requires actual incremental control properties and must be
designed consistently with direct optimized-input execution; it is not a
property supplied by observer ISS alone. The finite-horizon information-set
route does not require inserting such an auxiliary feedback after a solve.

An infinite frozen target forecast has a different limitation. Consider two
admitted straight constant-speed targets starting at the same point, with
speeds V-delta and V+delta, delta>0. Their positions separate by 2*delta*tau.
Any enclosure containing both has diameter at least 2*delta*tau, so no finite
uniform position diameter exists as tau tends to infinity. This is compatible
with bounded online observer error, because later observations refine the
current information. A fixed finite horizon bounds this growth; alternatively
the terminal proof can check an entire swept target family directly. A safe
parked ego set does not require an infinitely long target position-error ball
of uniformly bounded radius, but it does require universal target clearance
under the declared encounter convention. Finite encounter termination must
not be silently changed into an unrestricted infinite-time claim.

Thus the scoped joint result should establish: uniform valid current observer
bounds; uniform finite-horizon uncertainty bounds; a road-free robust terminal
extension; and shifted-witness feasibility under the same forecast/update
contract. Observer boundedness, finite prediction boundedness and recursive
feasibility are distinct propositions that must be connected. There is no
need to introduce a road-space premise into this scoped derivation.
