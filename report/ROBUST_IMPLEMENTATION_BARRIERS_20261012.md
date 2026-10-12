# Implementation barriers to robust recursive feasibility

Date: October 12, 2026. Controller baseline: commit
`099e8fe149de92e15a05f857a8ad66057edf96c9`.

The conditional proof in `controller/ROBUST_RECURSIVE_FEASIBILITY.md` is a
complete shift argument for its stated mathematical problem. Implementing its
premises in the present affine controller requires an algorithm change, not
only attaching a set-intersection expression. This audit identifies the
concrete missing operations and supplies reproducible witnesses. Numerical
solver failure is outside the requested mathematical claim.

## What is easy, and what has not been implemented

Interval intersection and interval propagation for the straight-motion
specialization are already executable in
`scripts/verifyInformationSetRecursiveFeasibility.m`. The target's existing
`predictiveSafetyGeometry.targetErrorTube` can also compute a full-horizon
constant-parameter enclosure without a new prediction formula. These are not
intrinsically difficult computations.

The production controller retains a center trajectory, optimized inputs and
the latest observer error enclosure. Its formulation propagates the current
ego generator through each individual hold's tangent, and restarts the target
enclosure at each predicted hold start. It does not retain and update the
propagated joint information set. The expression

    nextInformation = reachable(previousInformation, appliedInput) intersect observation

therefore lacks its first operand in the current continuation state. Taking
componentwise minima of error radii would not supply it: centers move, sets
are expressed in different ego-relative frames, and common-pose correlations
affect the collision geometry. An axis-aligned box representation is possible
but discards correlations; a consistent box outer map must be used in the
optimization and the posterior update. More accurate representations require
explicit intersection/support operations. No exact representation is required
by the proof if a sound monotone outer operator is implemented consistently.

## Full-horizon target uncertainty: a small change with a large consequence

The deterministic audit uses a straight target at 8 m/s, with initial position
radius 0.03 m, course radius 0.01 rad, speed radius 0.1 m/s, acceleration radius
0.2 m/s^2 and curvature radius 0.001 1/m. These are illustrative declared
inputs, not measured campaign statistics. The true example differs only by
an admitted constant acceleration of 0.2 m/s^2.

| Time (s) | Exact position error (m) | Full-horizon position radius (m) | Repeated one-hold radius (m) |
| --- | --- | --- | --- |
| 1 | 0.10 | 0.342 | 0.03933 |
| 3 | 0.90 | 1.758 | 0.03933 |
| 5 | 2.50 | 4.230 | 0.03933 |

The existing analytic full-horizon formula encloses this example. A repeated
one-hold envelope does not enclose the family propagated from the current
information without future observations. It can instead describe conditional
future holds if suitable future estimator bounds are available; those future
measurements cannot be assumed for an open-loop shifted-candidate proof.

Using the full-horizon formula in the current constraints is a straightforward
code edit. It is not the complete requested implementation: robust road,
handling and terminal constraints need corresponding ego images; the target
forecast must be consistent across measurement updates; and zero-slack terminal
clearance must cover the entire family. Larger enclosures can exclude a
previously accepted center trajectory. This audit does not measure optimizer
runtime or assert that the enlarged problem is infeasible in every scenario.
Set growth and representation conservatism must be assessed separately from
computational cost.

## Nonlinear reachability: tangents are not set certificates

At ego state `[0;0;0;15;0;0]` with zero input and a yaw perturbation of 0.1 rad,
the current one-hold tangent gives zero longitudinal error radius. The
declared nonlinear sampled model gives a longitudinal position difference of
-0.0037459193 m. This is a witness outside the tangent image, even during the
first 0.05 s hold. Multiplying Jacobians over the whole horizon cannot repair
an omitted nonlinear remainder.

A sound implementation must enclose the numerical sampled map over the
admitted state set and the optimization input domain, including the Fiala
force branches, combined-slip square root, low-speed blend and stop reset.
The held-input midpoint map needs its own enclosure. Validated interval maps,
Taylor models with bounded remainders, or certified domain partitions are
possible approaches. An ODE-level guarantee additionally needs integration
error bounds; the present theorem may instead declare the numerical RK4/stop
map as its model. Sampling points or replaying one center trajectory does not
prove universal containment. Floating-point outward enclosure also matters
for a literal numerical certificate.

An outer map must remain sound and monotone under subsets, or the program must
explicitly retain a containing old witness. Unrelated new linearizations do
not automatically have this inclusion property. This is a mathematical
representation issue as well as an implementation task; no benchmark here
establishes that certified reachability is too slow.

## Terminal certificate: settledness helps but does not identify the models

The updated runtime correctly rejects large terminal lateral drift and checks
rear adhesion at the terminal braking ratio. The audit additionally tests two
accepted terminal states on a wide straight road with no target:

| Quantity | Straight state | Mild lateral state |
| --- | --- | --- |
| Initial forward speed (m/s) | 15 | 15 |
| Initial lateral speed (m/s) | 0 | 0.3 |
| Initial yaw rate (rad/s) | 0 | 0.03 |
| Sideslip (rad) | 0 | 0.0199973 |
| Geometric lateral acceleration (m/s^2) | 0 | 0.45009 |
| One-hold position difference (m) | 0.00019090 | 0.00275417 |
| One-hold lateral-speed difference (m/s) | 0 | -0.1009564 |

Differences compare `terminalSafeSet.successor` with
`nonlinearBicycleModel.sample` at steering zero and terminal braking ratio
-0.4797025844. The closed-form terminal law holds sideslip and course curvature
constant while speed falls at 4 m/s^2. The Fiala/blended vehicle responds to
actual tire forces, road load and low-speed relaxation. Both test states are
settled under the updated terminal limits; the mismatch does not rely on
aggressive lateral motion. These witnesses show inequality of these specific
maps, not unavoidable collision or impossibility of a different backup law.

A usable terminal certificate needs an admissible backup input sequence under
the declared plant map for every member of the admitted endpoint information
set. It must cover speed, lateral motion, yaw rate, road/rectangle clearance,
handling, input slew if finite, and the parked phase against every admitted
target motion. A single nominal stopping arc with static pose inflation does
not cover uncertain stopping distances or tire-state evolution. For example,
even straight kinematics give stop distance V^2/(2a), so velocity uncertainty
must propagate into parked-position uncertainty. If an admitted target path
later enters a proposed parked rectangle, that particular terminal witness is
invalid; a solver cannot remove an admitted possibility by assertion.

Two practical designs remain available: construct a common held-input backup
and certify all its reachable sets, or construct a feedback backup whose
available measurements and information-set update are part of the proof.
The latter cannot be silently substituted for the former's shifted sequence.
An exact parked endpoint can simplify invariance under the declared stop map,
but increases the required optimization horizon and still needs uncertain
target-track clearance. A persistent defect capable of moving a parked vehicle
requires an additional invariant backup policy or a narrower defect contract.

## Consecutive convex problems need their own shift property

Even with a sound joint posterior and terminal witness, a newly built
fixed-separating-plane/trust-region convex restriction can exclude an old
feasible nonlinear plan. Numerical solution success cannot repair this
mathematical discrepancy. A theorem about the implemented convex problems
must preserve the shifted certified witness in the next restriction, including
the braking extension. Alternatively the theorem must explicitly remain about
the ideal robust problem. This issue is independent of post-solve input
modification: the optimized inputs can still be issued directly.

## Concrete implementation order and validation

1. Choose and state the actual backup policy and model/defect contract; establish
   at least one nonempty certified terminal information-set family.
2. Implement a sound set representation and full-hold nonlinear outer map;
   retain it in the continuation state and propagate before intersecting new
   observer information, with common-frame and stop-reset handling.
3. Use the same map in every horizon constraint and the terminal continuation;
   preserve the shifted witness in consecutive convex restrictions.
4. Verify soundness/inclusion and terminal closure before measuring closed-loop
   conservatism and runtime. Statistical success alone is not a set certificate.

This order prevents inserting a cheap set intersection over an uncertified
prediction and mistaking it for completion. No production propagation or
terminal algorithm was changed by this audit. Misleading comments calling the
first tangent hold certified were corrected. The controller still issues the
optimized inputs directly, with no post-solve modification or constraint veto.

Reproduction:

```matlab
addpath('scripts');
auditRobustControllerImplementation( ...
    'report/ROBUST_IMPLEMENTATION_BARRIERS_20261012/diagnostics.json');
results = runtests('tests/robustControllerImplementationAuditTest.m');
assertSuccess(results);
```

The three behavior tests pass. MATLAB Code Analyzer on the new script and
test reports only unavailable user settings (default settings used), with no
source warning. No random sampling, closed-loop campaign, runtime benchmark,
or complete nonlinear robust certificate is claimed.
