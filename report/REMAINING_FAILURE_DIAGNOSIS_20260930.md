# Why the seven remaining collision-threat initializations fail

Date: September 30, 2026. Production revision:
`395a8cbdc937945f6c1aef925997f731f6c9c9f8`.
Production controller, configuration and tests are unchanged by this analysis.
All seven failures occur before an input is issued; they are initialization
failures, not collisions during executed closed-loop control.

The failures have different mechanisms. Two high-speed straight-road problems
are solvable with additional search time. Turning crossing has actuator-limited
endpoint correction and a newly identified inconsistency when extra collision
checks become active. The curved cases reject before optimization because the
chosen permanent cruise continuation never passes terminal admission. In curved
head-on encounters, permanent cruising on the same circle would actually cause
repeated collisions; this is not solely a conservative geometric false alarm.

## Current failures, with fresh diagnostic evidence

Seven fixtures were rerun with the original 5 s soft budget; the three 15 m/s
straight-road fixtures were also run at 30 s. Copies of current source log
nonlinear residuals, conic returns and candidate handling. There are ten calls,
799 nonlinear evaluations and 69 outer iterations. Instrumentation changes
elapsed time and can move a budget boundary by an iteration; these are fresh
diagnostic measurements, not an exact reconstruction of an unlogged earlier
call and not running-frame latency measurements.

| Fixture | Best logged restoration candidate within 5 s | Extended diagnostic outcome |
| --- | --- | --- |
| Braking lead, 15 m/s | Collision/physical checks pass; weighted terminal-norm excess 9.351544 remains | Nominally feasible after 9.566163 s and 18 conic calls; dense replay gap 260.494403 mm |
| Crossing, 15 m/s | Terminal membership passes; collision-row deficit 473.496740 mm remains | Nominally feasible after 7.629647 s and 16 calls; dense replay gap 60.151795 mm |
| Turning crossing, 15 m/s | Terminal-norm excess 3.197051 plus collision-row deficit 199.130239 mm | Stops at 24 outer iterations after 15.979399 s, with 32 calls; best logged terminal-norm excess 0.025184 remains and interior collision is still hidden |
| Curved head-on, 8 and 15 m/s | No terminal continuation admitted | Zero conic calls; all tested endpoint states satisfy membership, but permanent separation fails |
| Curved crossing, 8 and 15 m/s | No terminal continuation admitted | Same pre-optimization rejection; free phase does not change the orbit-separation bound |

Terminal-norm excess is a weighted numerical quantity, **not meters**. A
collision-row deficit includes the 6 mm buffer and is not identical to measured
rectangle penetration. All fixtures retain 50 ms holds, exact observations,
8/16 prefix holds, a 512-hold maximum horizon, no road constraints and no random
draws. Extended runs change only the diagnostic time budget unless a control
below expressly states otherwise. The 24-iteration limit remains unchanged.
The complete closed-loop campaign remains 7/14; these open-loop diagnostics do
not revise it.

## 1. Braking lead: the first local problem is poor, then endpoint repair is slow

The initializer rolls out lane feedback at cruise speed, through the target's
future path. It checks eventual terminal admission, but does not construct an
avoidance maneuver. Its selected completion has 134 holds (6.7 s). The first
hard conic problem is infeasible. Removing finite collision rows makes it
feasible; removing just the endpoint cone also makes this particular problem
feasible. Removing future separation or doubling trust alone does not.

The elastic restoration is numerically solvable. Several returns carry flag
`-7`: the solver reports a small search direction before all requested
optimality/constraint tolerances are met. These are not empty failed returns:
they contain points with very small primal residuals and are already evaluated
by the controller. For example, the first restoration has primal feasibility
`6.09e-12`, dual feasibility `9.75e-7`, and a finite point. Treating every such
flag as physical infeasibility would be incorrect.

By the best five-second evaluation the sampled collision and physical
violations are zero. Independent dense replay of that unaccepted plan also
stays separated, with a 210.861478 mm minimum gap. Its terminal state is still
far from its freely selected cruise reference: approximately 0.814 m
longitudinal error, 0.655 m lateral error and 0.320 m/s speed error. Therefore
this is not merely a rounding-level terminal rejection at five seconds.

The 30 s diagnostic converges in ten outer iterations. Conic calls take
5.036297 s and instrumented nonlinear evaluation takes 1.556453 s of the
9.566163 s call; the remainder includes seed construction, linearization,
correction sensitivities and diagnostic overhead. The successful final plan
has zero original hard residual and prefix slack. Its ODE endpoint membership
also passes. The problem has a demonstrated feasible nominal solution, but
repairing a colliding cruise seed and returning to the tight terminal
neighborhood consumes the original budget.

## 2. Crossing: remaining collision repair, not terminal membership, blocks the five-second run

The first hard conic problem is infeasible. Removing collision rows makes it
feasible; removing the endpoint cone, removing future separation, or doubling
the trust radius does not. This distinguishes it from braking lead.

At the best five-second evaluation, terminal membership already passes and
future-separation margin is 27.842576 m. Physical/input residuals are zero.
The remaining 0.473497 m collision-row deficit is in the completion tail.
Dense replay confirms actual overlap at 66 samples between 1.483333 and
1.588333 s, with maximum separating-axis penetration 0.519026 m. This plan
cannot be issued as a collision-free continuation.

The 30 s run reaches feasibility after twelve outer iterations, with no
terminal-set or physical-limit changes. Its full optimized sequence has
60.151795 mm dense ODE clearance and an admissible ODE endpoint. Conic calls
consume 1.667918 s and nonlinear evaluations 2.894947 s of the 7.629647 s
instrumented initialization. Substantial work therefore occurs outside the
conic solver, including repeated nonlinear correction and validation.

Unlike the older revision's crossing diagnosis, the current five-second
bottleneck is collision repair. Historical residual labels should not be
carried forward after an algorithm change without rerunning the fixture.

## 3. Turning crossing: tight terminal entry, input-bound rejection and late collision discovery

At 30 s, the controller ends at its **24-iteration cap** after 15.979399 s;
it does not consume the full time budget. The best logged score corresponds
to a terminal weighted norm of 0.026161 versus an allowed radius of
0.0009765625. Errors include 1.120245 mm longitudinal and 3.016224 mm lateral
displacement relative to the freely selected phase. The certified coordinate
bounds are only 0.108536 mm longitudinal and 0.165359 mm lateral. These are
necessary coordinate bounds of the coupled ellipsoid, not independent box
constraints. No externally prescribed longitudinal destination was restored.

### Why endpoint polishing does not remove the final error

The current correction uses an unconstrained minimum-norm pseudoinverse over
the final twenty controls (one second) and the phase. It subsequently rejects
input-limit violations by trying step scales 1 through 1/32.

For this candidate the minimum tail steering input is -0.698130798904 rad,
only about `9.02e-7` rad inside the -0.698131700798 rad limit. The unconstrained
correction requests up to 0.050472 rad steering change and 0.083127 braking-ratio
change. Every one of the six tested scales is rejected by the input-bound
check before nonlinear evaluation. The tangent matrix has full row rank;
its largest/smallest singular values are 21.142768 and 0.017395, and its linear
endpoint equation residual is `5.56e-16`. A rank-deficient terminal equation is
not the observed cause. The direction ignores an active actuator boundary.
Increasing the correction limit from five to twenty repeats the same first
iteration rejection and yields no improvement.

Diagnostic bounded least-squares and bounded minimum-norm corrections can make
progress, but neither supplies an accepted solution by itself. Bounded
least-squares reduces the best hard residual to 0.022822. The bounded
minimum-norm control reduces it to 0.021867; a subsequent trial reaches terminal
membership but exposes a 0.059935 collision residual and is rejected. Thus
fixing endpoint correction alone is insufficient. These controls are cache-only
experiments, not production implementations.

### The candidate also has a collision between the earlier sample checks

The best unaccepted 30 s candidate has 28.503097 mm ODE clearance at nodes and
midpoints, but **eleven interior samples overlap** between 1.530000 and
1.546667 s, with maximum penetration 100.146035 mm. The controller checks all
nearby interior points when a candidate otherwise passes admission, or at
stages already marked for refinement. Since this candidate still fails
terminal membership, its logged sparse collision score does not expose every
interior violation. It must not be called a safe plan with only a small
terminal error.

### Newly generated constraints leave the incumbent score stale

Restarting from that best candidate, with the same horizon and constraints,
allows the terminal error to become very small. When an interior violation is
finally discovered, the outer loop adds the corresponding collision rows.
It does not reevaluate the incumbent baseline under the newly activated checks
before comparing restoration scores.

At diagnostic restart iteration 19:

| Quantity | Value |
| --- | ---: |
| Stored incumbent restoration score | 0.001936883 |
| Same incumbent reevaluated with the new interior check | 10.099594047 |
| New trial under that check | 10.098502661 |
| Actual incumbent interior violation | 0.100976572 m |
| Newly refined stage | 31, spanning 1.50--1.55 s |
| Current trust radius | 0.000465661 |
| Corresponding position box half-width | 0.002328306 m |

The new trial is an improvement under the same checker. Comparing it against
the obsolete 0.001937 score labels it worse and halves trust. The original
restart rejects all last six candidates and shrinks to `1.45519e-5`. Its
38 conic calls end at the iteration cap without a witness.

A diagnostic control changes only incumbent reevaluation when the collision
check set grows. Iteration 19 is then accepted, as are the following five
iterations; the score decreases from 10.098503 to 8.397370 and trust grows to
0.001421085. It still does not finish within the remaining iteration allowance.
This isolates an erroneous comparison, without claiming that correcting it
alone solves the entire fixture. Much earlier shrinking has already left too
little movement available for a roughly 0.10 m collision repair.

This stale-score defect is observed in the **continued/restarted diagnostic**,
not claimed as the event that exhausted the original five-second call. It
explains why simply requesting more iterations can expose another stall after
the terminal error becomes small.

## 4. Curved encounters: the terminal policy excludes the needed future behavior

All 445 endpoint attempts at 8 m/s and all 437 attempts at 15 m/s pass local
terminal membership, up to the 25.6 s maximum horizon. Every attempt fails
future separation; no conic program is built. These are not time-limit or road
boundary failures.

The ego terminal reference is a circle of radius 200 m. For curved head-on,
the target travels around the same center with body-sweep annulus radii
199.043600--200.983408 m. The resulting sufficient separation margin is about
-3.5436 m. For curved crossing, the target annulus is
39.028664--41.123662 m and the centers are 203.962873 m apart; the ego circle
crosses that annulus, giving about -39.748 m. These are certificate margins,
not measured body penetration.

A quarter-turn phase scan gives the same negative margin, within `2e-13` m,
and zero phase derivative for all four cases. Choosing another endpoint time
or changing longitudinal phase cannot separate the same complete circular
sweeps. Removing road boundaries does not remove the terminal requirement to
return near the given path and subsequently cruise there indefinitely.

There is an important qualification to the earlier conservative-certificate
explanation. In curved head-on, the ego and target circle in opposite directions.
Their nominal relative angle advances at `(vE+8)/200`; hence their centers
meet once every

\[
T=\frac{2\pi\,200}{v_E+8},
\]

or 78.539816 s at 8 m/s and 54.636394 s at 15 m/s. Phase changes when the next
meeting occurs, not whether it occurs. Direct nominal-reference/target geometry
checks at four phases confirm zero body distance at each predicted meeting,
with center differences below `4e-12` m. A time-synchronized certificate alone
cannot make permanent same-circle cruising safe in these head-on fixtures.

Curved crossing also has recurring future conflicts for the inspected nominal
phases. Starting from the 25.6 s endpoint, a 5 ms geometric scan over two ego
laps finds a future overlap for all four tested phases at 8 m/s, and for the
zero phase at 15 m/s. The other three 15 m/s phases have no sampled overlap
within that finite scan; this neither proves infinite safety nor justifies
rejecting every phase as physically impossible. The whole-sweep certificate
cannot make that distinction.

The fixture's displayed curve length is 200 m, but the analytic reference
continues the circle beyond it; there is no modeled straight exit. The curved
crossing target also keeps its constant acceleration/sideslip forever. These
infinite continuation assumptions matter even though the visible simulation
lasts only eight seconds. The future phase scans inspect analytic nominal
reference geometry, not a closed-loop nonlinear-policy simulation or a proof
for all phases.

## Consequences for the next implementation

1. Improve the avoidance initialization and terminal repair used by the same
   optimizer. Terminal correction should account for active actuator bounds
   when choosing its direction; increasing its iteration count cannot fix a
   direction rejected at every tested scale.
2. Compare incumbent and trial using the same collision check set. Retain new
   constraint discoveries independently of whether a trial becomes the next
   anchor. Reevaluate the incumbent when that set grows and reconsider an
   already tiny trust region when the new violation needs a larger movement.
3. Keep checking collision while repairing terminal entry; the turning-crossing
   counterexample shows why sparse collision success is insufficient evidence
   that an unaccepted plan can be executed.
4. Revisit the permitted terminal continuation in curved conflicts. The current
   cruise family genuinely cannot safely continue the head-on circle forever.
   Enlarging the allowed certified continuation, or explicitly modeling a real
   straight exit if that is the intended path, is a different task from merely
   speeding up the numerical solver or dropping the terminal check.

## Evidence and reproduction

The [evidence directory](REMAINING_FAILURE_DIAGNOSIS_20260930/) contains compact
results, all base-run evaluations and iterations, independent replay audits,
eighteen affine controls, endpoint/refinement controls, circle timing checks,
source/raw hashes, instrumentation patches and reproduction drivers. Full MAT
snapshots, dense poses and diagnostic source copies remain in
`/home/zai/.cache/collisionAvoidance/remaining-failure-analysis-20260930/`.
No prototype controller copy, media or generated binary is added to the repo.

Reproduction order: run `prepare.py`, then `prepareProbes.py`; run `diagnose.m`,
`controls.m`, `endpointProbe.m`, `boundedProbe.m`, `curves.m`,
`refinementProbe.m`, and `replayReturned.m` with MATLAB; then run
`curveTiming.py` and `analyze.py`. The driver text and five prototype patches
reproduce the diagnostic helpers. Two harness issues, a variable shadowing
`which` and an uninitialized logging struct in a new MATLAB process, were
corrected; original failed logs are preserved and are not attributed to the
production controller.

The analyzer requires ten base diagnostic calls and six dense replays, verifies
all replay distances independently, and checks unchanged production source
hashes. All eighteen first-program ablations and the prototype patch
reconstruction checks completed. Controller unit tests were not rerun for this
report-only analysis. Additional conic points, unaccepted candidates and
extended-budget successes are never counted as original-budget closed-loop
avoidance successes.
