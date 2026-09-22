# Controller failure-mode sweep on the declared plant, 2026-09-22

Purpose: enumerate every way the controller fails other than a missed frame
deadline. Every run below disables the deadline (`DeadlineSeconds = Inf`,
certificate search limit 30 s), so a run ends only when the controller raises an
error or the sample count is exhausted. Measured outcomes only; nothing here is a
proof, a worst-case bound or a real-time claim.

Environment: MATLAB R2026a Update 3, `matlab -batch`, repository `main` at
`0affcf08c8f4b6cd999cbddd5b6eb2f1d666f831` plus the sweep script added with this
report. Plant: the declared held affine flow of
`runExactStateRecursiveFeasibilityScenario` (the plant is the controller's own
model; ego 8 m/s, 0.05 s holds, 1.6 s nominal horizon, 240 holds, seed 20260912,
`lateralDomainRadius = 4`). Two unrelated MATLAB desktop sessions belonging to
other projects were running on the machine and were left untouched.

Reproduction: `runDeclaredPlantFailureSweep(OutputDirectory=<dir>)`. The
per-case table is `CONTROLLER_FAILURE_MODE_SWEEP_20260922.csv`. An exploratory
run with the same cases preceded the recorded run and agreed case for case on
pass/fail and failure identifier.

## 1. Case set (118 cases)

| Group | Axis | Cases |
| --- | --- | --- |
| A-geometry | scenario {stationary, oncoming, crossing, cruise} x curvature {0, +-0.005, +-0.01, +-0.015, +-0.02} /m | 36 |
| B-uncertainty | declared ego/target bounds and target jerk scaled x1, x3, x10 from the `uncertainCrossing` base; straight and 0.01 /m | 18 |
| C-target motion | declared-and-true target jerk 0.1, 0.5, 1.0 m/s^3; yaw acceleration 0.1, 0.5 rad/s^2 | 10 |
| D-initial error | initial lateral 0.5-3 m, heading 0.1-0.3 rad, speed +-2 m/s; cruise and stationary; straight and 0.01 /m | 36 |
| E-road | straight +-5 m road boundaries, all four scenarios | 4 |
| F-horizon | 0.8, 1.2, 2.4, 3.2 s | 8 |
| G-confirmation | confirmation range 10, 24, 32 m | 6 |

## 2. Outcome

| Group | Cases | Passed | Controller error | Silent overlap |
| --- | --- | --- | --- | --- |
| A-geometry | 36 | 35 | 1 | 0 |
| B-uncertainty | 18 | 13 | 5 | 0 |
| C-targetJerk | 6 | 6 | 0 | 0 |
| C-targetYawAcc | 4 | 4 | 0 | 0 |
| D-initLateral | 16 | 12 | 1 | 3 |
| D-initHeading | 12 | 7 | 3 | 2 |
| D-initSpeed | 8 | 5 | 1 | 2 |
| E-road | 4 | 0 | 4 | 0 |
| F-horizon | 8 | 8 | 0 | 0 |
| G-confirm | 6 | 5 | 0 | 1 |
| Total | 118 | 95 | 15 | 8 |

"Controller error" is a raised `collisionAvoidanceController:*` exception; no
command is issued and the run ends. "Silent overlap" is a run that completes
all 240 holds with every reported certificate satisfied but whose sampled body
gap is negative. No case produced a positive CLF residual, a slew violation or a
timeout.

## 3. Failure class 1: structural rejection (4 cases here, 4 more in the PassVeh14DOF set)

Any road boundary is rejected at the first frame by
`hardEncounterBarrier.m:492-494` ("The recursive cruise certificate requires an
unbounded road-free reference domain"); the scheduled terminal certificate
carries the same rejection at `:571-573`. All four E-road cases fail this way,
as do both PassVeh14DOF avoidance scenarios recorded in
`CONTROLLER_VALIDATION_RERUN_20260922.md`. The PassVeh14DOF cruise scenarios fail
on the companion contract, the zero-residual held affine plant
(`collisionAvoidanceController.m:36-38`), through the zero-radius successor-box
test. These are declared design limits, not numerical failures.

## 4. Failure class 2: admission infeasibility (11 cases)

The fixed-direction trajectory SOCP returns Clarabel status 2 (primal
infeasible) and the controller raises `optimizationFailed` ("The hard-safety,
soft-CLF search failed at t=... s"). Ten of eleven cases fail at t = 0; the
eleventh cruises for 52 holds and fails at t = 2.6 s, when the oncoming target
enters the 16 m confirmation range and the encounter must be admitted.

| Case | Straight | Curvature 0.01 /m | Note |
| --- | --- | --- | --- |
| crossing, curvature -0.02 /m | - | - | fails at t = 0; +0.02 passes |
| uncertainty x3 | passes | crossing fails | stationary, oncoming pass |
| uncertainty x10 | stationary fails | stationary, crossing fail at t = 0; oncoming fails at t = 2.6 s | |
| stationary, initial lateral 2 m | passes | fails | 0.5, 1 and 3 m pass on the curve: not monotone |
| stationary, initial heading 0.1 / 0.2 / 0.3 rad | pass | all fail | |
| stationary, initial speed -2 m/s | passes | fails | +2 m/s passes |

Every curved failure involves the stationary or crossing target placed 15 m
ahead on the arc, 1.9 s from contact at 8 m/s against a 1.6 s horizon. The
zero-error curved stationary case passes with a terminal membership margin of
only 0.0068 (0.0031 at 0.02 /m), so any initial error or bound inflation removes
the remaining admission room. The separation directions are fixed by the fluid
initializer before the SOCP, so these runs cannot distinguish true
infeasibility from an unfavorable direction choice; that distinction is an open
item already recorded in `report/README.md`.

## 5. Failure class 3: silent inter-node overlap (8 cases)

Eight runs complete all 240 holds with `recursiveFeasibilityGuaranteed` true,
every CLF residual negative and every terminal margin positive, yet the sampled
body gap (11 sub-samples per hold along the exact affine flow) is negative. The
node gap, evaluated at the hold boundaries where the certificate applies, is
positive in every one of them:

| Case (straight road) | Node gap [m] | Sampled gap [m] |
| --- | --- | --- |
| stationary, initial lateral 0.5 m | +9.40e-05 | -4.51e-05 |
| stationary, initial lateral 1 m | +7.64e-05 | -2.72e-02 |
| stationary, initial lateral 3 m | +8.64e-05 | -3.99e-03 |
| stationary, initial heading 0.1 rad | +7.75e-05 | -7.42e-05 |
| stationary, initial heading 0.3 rad | +2.69e-02 | -1.04e-03 |
| stationary, initial speed -2 m/s | +9.75e-05 | -1.71e-02 |
| stationary, initial speed +2 m/s | +9.00e-05 | -2.22e-03 |
| oncoming, confirmation range 32 m | +3.65e-03 | -6.53e-04 |

The certificate is not violated: it holds exactly at the sampling nodes, and the
overlap occurs between them. Because the collision condition is exactly
`dRect > 0` with no clearance parameter, the optimum grazes the target at the
node (the unperturbed stationary case passes with a node gap of 9.28e-05 m), and
the exact flow inside a 50 ms hold then penetrates the target by up to 2.7 cm.
A re-run of thirteen stationary/oncoming variants with node-by-node
reconstruction confirmed the sign pattern in every case; the linear inter-node
interpolation used there underestimates the exact-flow penetration, so the
report values above are authoritative. The confirmation-range result is not a
trend: node gaps are driven to zero, so the sign of the inter-node gap at 16, 24
and 32 m (+0.038, +0.008, -0.0007 m) is geometric noise around zero.

## 6. Passed cases that leave the declared model domain (38 cases)

Every oncoming and every curved crossing case, plus several uncertainty and
target-motion cases, completes with a negative sampled model-domain margin
(-0.02 to -0.85). In each inspected run the exceeded component is the heading
error against `headingDomainRadius = 0.40` rad: the base oncoming case reaches
-0.555 rad at t = 3.50 s with the front-wheel steering angle saturated at
0.698 rad. Lateral position (min -2.46 m against +-4 m), speed, lateral velocity
and yaw rate stay inside their bounds. The cause is the encounter geometry: the
oncoming target is confirmed at 16 m with a 16 m/s closing speed, 1.0 s before
contact, so the admitted avoidance is a full-lock swerve. On the declared plant
this is harmless because the plant is the model; on any other plant it is where
the LTV linearization is declared invalid. The domain margin is an offline
diagnostic and neither accepts nor rejects a command.

## 7. Axes that did not fail

Target jerk up to 1.0 m/s^3 and yaw acceleration up to 0.5 rad/s^2 (declared
and true), horizons from 0.8 to 3.2 s, curvature up to +-0.02 /m for stationary,
oncoming and cruise, and all straight-road initial errors for cruise completed
and passed. Straight-road uncertainty x10 passed for oncoming and crossing.

## 8. Conclusion

Apart from missed deadlines, the controller fails in three ways: it rejects road
boundaries and non-affine plants by contract; it finds the fixed-direction
admission SOCP infeasible on curved roads when the encounter is close and the
initial error or declared uncertainty is nonzero; and it passes its own node
certificate while the exact flow between nodes overlaps the target by up to
2.7 cm, because the certificate has no clearance and is enforced only at hold
boundaries. The oncoming maneuver additionally leaves the declared heading
domain in every variant tested.
