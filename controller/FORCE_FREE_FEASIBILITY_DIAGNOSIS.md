# Why the force-free straight controller still fails

Analysis date: September 10, 2026. Source: certificate version 14, commit
`3c3f7cfa4366e6bfd475b6416da7f0c17b388cfb`. Inputs are the actual outputs of
[the force-constraint removal experiment](../scripts/FORCE_CONSTRAINT_REMOVAL_RESULTS.md),
preserved in EV-0099. This investigation changes no online controller,
configuration, plant, safety acceptance rule or physical uncertainty bound.

## Finding and required scope

The principal problem is a mismatch between the complete-encounter horizon,
the uncertainty enclosure that can survive that horizon, and the separating
geometry selected from a straight cruising anchor. Independent road-interface
and runtime problems prevent the physical driver from getting that far.
Deleting the axle-force polygon solved one incompatibility, but these other
restrictions remain.

The required guarantee is conditional: if the research assumptions hold and
first joint encounter admission is feasible, maintain feasible safe control
until finite target exit. It does not require indefinite post-exit safety.
An infeasible first admission is outside that implication, so the failed
29.9 m trial is not a counterexample to the conditional theorem. However,
a controller that cannot admit the intended operating scene does not meet
the practical experimental objective. Feasibility of the current conservative
convex restriction is also different from physical avoidability.

## 1. A fixed 1.6 s exit is stronger than eventual finite exit

`collisionAvoidanceController.localExitSchedule` assigns every new encounter
the entire configured horizon. `hardEncounterBarrier.completionRows` demands
that the terminal center separation exceed the sensor radius plus 0.1 m in
one supporting direction. This is a sufficient convex condition for circular
exit, with uncertainty support deducted. There is no separate reachability
calculation that establishes whether the chosen completion time is achievable.

In the saved acquisition frame, the ego and oncoming target each travel at
10 m/s, their longitudinal separation is 29.9 m, their lateral offset is
0.8 m, and sensing radius is 30 m. The horizon is 16 times 0.1 s = 1.6 s.
At unchanged cruising speed, the target is near the ego at this endpoint;
passing from the forward sensing boundary to the rear boundary takes about
3 s at the nominal 20 m/s closing speed. This is an illustrative nominal
calculation, not a lower bound over all controlled trajectories.

A stronger numerical check optimizes both endpoint position coordinates over
all retained core rows, removing only collision and exit rows. Even with zero
plant residual, every feasible endpoint lies in a box whose farthest corner
is only **7.04908 m** from the target center. With the original full residual,
the corresponding upper bound is **5.01924 m**. These are outer-box bounds,
not reachable-distance estimates; either bound rules out a 30.1 m terminal
center distance under those retained rows. The terminal supporting-halfspace
condition is therefore not the sole cause of this short-horizon failure.

The core includes the station chart trust region, not just physical actuator
limits. `laneGeometry.sweptCellFrames` centers each chart on the reference
plan with `stationTrustRadius` = 2 m. At 1.6 s this leaves the nominal world
longitudinal endpoint approximately 13.857--18 m. Therefore these numerical
bounds establish impossibility for the implemented restricted problem, not
for every physically admissible maneuver.

## 2. Simply increasing the horizon creates a new algebraic contradiction

The baseline full-residual hard LP has feasible core constraints, but both
core plus collision and core plus exit are independently infeasible.
Off-line minimization of a uniform relaxation of the selected distance rows
requires approximately **3.73318 m** for collision, or **29.72634 m** for the
exit supporting row. These are diagnosis values, not proposed safety slacks;
the largest relaxed LP residual is below 8.4e-8.

There is already a direct collision/road contradiction at 1.6 s. Hard rows
23622 and 23625 have exactly opposite coefficients and summed bound
**-2.69627967180 m**. One demands separation below the target; the other
requires staying above the right curb. At the final Bernstein point,
`r_y=3.401382 m` and `r_psi=0.358702 rad`. The relevant rectangle support
rows require effective transverse width
`2*(1 + r_y + 2.5*r_psi) = 10.596276 m`. The target lower edge is at
-0.2 m and the right curb at -8.6 m; subtracting 0.25 m clearance at both
boundaries leaves only **7.90 m**. No nominal lateral placement can fit this
particular robust envelope. This is a limitation of the declared boxed
inclusion and its fixed lateral separation, not proof that the actual tire
model or vehicle physically occupies that entire envelope.

The heading model domain requires

\[
 |\bar\psi(t)|+r_\psi(t)\le 0.4.
\]

At 1.6 s, the maximum swept heading radius is **0.358702 rad**, leaving
only about **0.041298 rad** for the nominal heading. At 1.8 s it becomes
**0.404279 rad**. The core is then infeasible even with every collision and
exit constraint removed. In the 1.8 s hard matrix, rows 26617 and 26623 have
exactly opposite coefficient vectors and a summed right-hand side of
**-0.00856178716**. Summing them gives the contradiction

\[
 0\le -0.00856178716.
\]

This is direct algebraic evidence, independent of solver tuning or objective
weights. The CLF performance slack cannot repair this hard model-domain pair.

| Horizon [s] | Full-residual core | Full-residual core + collision | Zero-residual core + exit |
| ---: | --- | --- | --- |
| 1.2 | Feasible | Feasible | Infeasible |
| 1.6 | Feasible | Infeasible | Infeasible |
| 1.8 | Infeasible | Infeasible | Infeasible |
| 2.0 | Infeasible | Infeasible | Infeasible |
| 2.4 | Infeasible | Infeasible | Infeasible |
| 3.0 | Infeasible | Infeasible | Feasible |
| 3.2 | Infeasible | Infeasible | Feasible |

Longer-horizon rows in this table are explicit counterfactuals. The target
constant-velocity contract is extended with the horizon. Original sensed
road coverage fails at 2.0 s and above for full residuals, and at 2.4 s and
above for zero residuals among the tested points. Those failures are recorded
before extending the same straight curb polynomials to [-200, 200] m for the
remaining algebraic diagnosis. They are not valid longer-horizon certificates
from the original finite road-perception packet.

## 3. The uncertainty growth has both structural and numerical causes

The actual target has exact constant velocity and zero initial error, jerk
and yaw-acceleration bounds. The present infeasibility is not caused by NRMM
tracking error. The dominant uncertain object is the ego model inclusion,
with empirical residual rates `[0.2;0.06;0.02;2.5;5;4]`.

`stateUncertainty.flowTube` uses absolute Taylor-power bounds for swept
support and propagates an axis-aligned box between cells.
`ltvBicycleModel.finitePredict` repeatedly assigns `radius=tube.endRadius` and
overwrites its earlier generator-based radius with that box. The operative
path therefore does not preserve all signed correlations, despite preceding
comments about generator propagation. The admitted original tube is retained
across observations; this is not a closed-loop contracting error tube.

At 1.6 s, endpoint radii are:

| Component | Implemented endpoint box | Numerically integrated directional LTV support |
| --- | ---: | ---: |
| Station [m] | 3.50803 | 3.50770 |
| Lateral [m] | 3.39960 | 2.22897 |
| Heading [rad] | 0.357238 | 0.238788 |
| Lateral velocity [m/s] | 0.347438 | 0.233515 |
| Yaw rate [rad/s] | 0.213420 | 0.131971 |

The comparison integrates

\[
 r_i(T)=\sum_j|\Phi_{ij}(T,0)|r_j(0)
 +\int_0^T\sum_j|\Phi_{ij}(T,\tau)|\,\bar w_j(\tau)\,d\tau
\]

through the same scheduled affine generators and independent disturbance box.
It is a numerical endpoint diagnostic, not a verified continuous swept-tube
replacement or a nonlinear residual proof. It shows substantial avoidable
reboxing inflation, while longitudinal error barely changes. Even without
reboxing, persistent uncertainty accumulates through position/heading
integrators. At 3.0 s the numerically integrated heading support is already
**0.451533 rad**, exceeding the 0.4 rad domain even without cell reboxing.
Tightening this enclosure alone does not establish a useful
complete-encounter feasible domain for the physical residual allowance.

Reducing the diagnostic residual multiplier from 1 to 0.1 makes core plus
collision feasible at 1.6 s; at multiplier 0.25 it is already infeasible.
Exit remains infeasible at every tested multiplier, including zero. Arbitrarily
reducing physical residual bounds would hide one failure without solving the
completion-time mismatch, and would invalidate the intended physical premise.

## 4. Straight-anchor separation excludes feasible nominal avoidance

`avoidanceSafetyGeometry.localCellRows` selects one separating normal per cell
from the closest configuration-obstacle boundary at the straight anchor.
The chosen normals are frozen into the convex program. In the 1.6 s problem,
they change from negative longitudinal to negative lateral at 1.3 s. Over a
longer horizon, the straight anchor passes through the obstacle and selects
new directions. No outer geometric refinement explores whether those choices
exclude an alternative continuous trajectory.

A controlled zero-residual 3.2 s comparison keeps the same prediction maps,
core rows, exit rows, inputs, slew bounds, model domains, rectangle support,
clearance and arithmetic allowances. It changes only the collision probe:
from 1.3 s onward, its lateral coordinate is -6 m when selecting normals.
This probe is not an executable reference or a new online mode; selected
normals can be diagonal. Original sensed-road coverage is extended as stated
above in both members of this comparison.

- Original straight-anchor full problem: infeasible.
- Alternative probe full problem: feasible; maximum hard-row residual
  **4.44e-16**.
- The same feasible plan violates the original fixed-normal program by
  **2.93295 m**.

This isolates a limitation of the selected convex restriction. It does not
establish nonlinear or physical vehicle feasibility: the comparison explicitly
uses zero plant residual and extended road coverage. It does show why a
longer horizon alone remains insufficient even in the nominal model.

## 5. Road identity checks representation, not compatible retained geometry

At the actual second physical sample, lane identity and acceleration bias
are unchanged. Only `road.boundaries` changes. Its local origin moves from
[0, 0] m to [1.00008691446, 0] m, fitted polynomial coefficients change at
roundoff scale, and the sensed coverage interval translates with the ego.
After expressing both polynomials in world coordinates, their maximum graph
difference over their common coverage is **1.12786e-14 m**.

`hardEncounterBarrier.localValidateStored` nevertheless requires
`isequaln(identity,stored.identity)` before checking the retained witness.
This explains the observed `changedExecutionContract` stop at 0.1 s. Replacing
that test with an arbitrary tolerance would not prove compatibility: the
coverage domains also change, and a real refit may change the certified road.
A valid repair must retain previously certified geometric information or
verify that the updated road admits the remaining old witness, with correct
coordinate transforms and uncertainty accounting.

There is a separate lifecycle issue before acquisition. The target starts
100 m away and nominally enters 30 m sensing after about 3.5 s, but the
no-target certificate consumes its 1.6 s horizon and returns no further input.
Repairing the road comparison alone therefore cannot complete that scenario.
An encounter deadline should not accidentally be consumed before its first
target exists. This is a pre-acquisition orchestration issue, not a request
for indefinite post-exit safety or justification for a second controller mode.

## 6. Runtime cost is primarily the two full numerical solves

Three replays of the saved target-free first frame all produce certified
commands. Measured total times are **590.247, 413.369 and 422.430 ms**. The
first sample includes fresh diagnostic-hook/JIT overhead; these are neither
physical closed-loop reruns nor deployment WCET measurements.

The hard-margin LP takes **115.446--121.855 ms**, with 21,650 rows and 49
program variables before inactive slack elimination. The subordinate conic
performance solve takes **200.904--202.851 ms**, with 30,608 rows and 48
variables. Both report 15 iterations in these samples. The two-stage solve
phase, including assembly and acceptance work, takes **340.890--363.977 ms**.
On the latter two replays it occupies about 83% of the whole call. Formulation
is about 53--55 ms and prediction about 12 ms. One additional instrumented
profile confirms concentration in the native solver; its times are kept
separate from the three unprofiled samples.

The saved acquired-target hard LP has 23,777 rows. Sixteen control intervals
expand to 112 certified cells and eight Bernstein points per cell, followed
by multiple model, slip, road and collision support rows. This explains why
few input variables do not imply a small conic problem. The hard-margin
stage uses a condensed matrix; simply assuming all stages exploit the sparse
lift would be incorrect. Row reduction or alternate numerical transcription
must preserve the same support constraints and independently checked margin.

## Design priorities implied by the evidence

1. Define the complete encounter duration and sensing/road information needed
   to certify it, independently of a cruise frame's original timestamp.
   Demonstrate a nonempty feasible domain for the intended acquisition scene.
2. Construct a compatible uncertainty/feedback policy for that duration.
   Preserve directional correlations and certify swept support; assess whether
   feedback contraction or safe reconditioning is needed. Prove enclosure for
   the nonlinear modified Fiala/physical model over the admitted domain.
3. Improve the continuous geometric candidate construction or refinement
   within the same controller, while admitting only independently certified
   complete witnesses. The offline probe is diagnostic evidence, not a
   proposed hard-coded lane-change branch.
4. Make sensed-road updates compatible with the retained proof, and handle
   target-free execution without prematurely exhausting future obligations.
5. Optimize the measured solver/row bottleneck after the formulation admits
   meaningful complete encounters. Keep the deadline check truthful.

None of these steps requires a separate maneuver mode, an old controller
branch, or an infinite post-exit guarantee. No single parameter change resolves
all the independently demonstrated failures.

## Reproduction and verification

```matlab
addpath('scripts');
analysis = analyzeForceFreeStraightFeasibility( ...
    '/path/to/force-removal/exports', '/path/to/diagnosis');
runtime = profileForceFreeAdmission( ...
    '/path/to/force-removal/exports', '/path/to/diagnosis');
```

The drivers preserve raw MAT outputs, JSON summaries, LP subset outcomes,
relaxation dual rows, counterfactual input plans, opposing-row sums, road
representations, phase timings and the instrumented profile. Behavioral
assertions check the baseline failure, alternative nominal feasibility,
heading contradiction, world-coordinate road equivalence and valid nested
timing. Both drivers have zero factory Code Analyzer findings. No controller
or estimator code changed, and no new full unit-suite or physical avoidance
run was performed during this investigation.
