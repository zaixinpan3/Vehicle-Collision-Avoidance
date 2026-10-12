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
