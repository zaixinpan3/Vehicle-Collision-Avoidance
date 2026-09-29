# Failure analysis of collision-only controller scenarios

Date: September 29, 2026. Diagnostic source: commit
`17c5a756bcfa8abb183510abf08e972d48a57bb3`.
Controller implementation and configuration are unchanged in this investigation.

The [collision-threat campaign](COLLISION_THREAT_VALIDATION_20260929.md)
contained fourteen genuine uncontrolled-cruise collisions. Four oncoming cases
completed; ten cases rejected initialization before issuing a command. The
failures are attributable to specific initialization, local constraint, and
terminal-admission mechanisms. Increasing the search budget alone does not
resolve them. An additional open-loop audit exposes an intersample collision
in the one newly returned plan under an extended budget.

## Scope and methods

The ten rejected fixtures were replayed through the public controller in an
instrumented cache copy with the original five-second budget. The six straight
fixtures were also replayed with a thirty-second budget. All other configuration
values were retained: exact observations, 8 or 15 m/s reference speed, 50 ms
control interval, 8 or 16 prefix holds, 6 mm collision margin, 4 m road bounds
on each side, 4.8 by 1.9 m vehicle rectangles, and a 512-hold maximum horizon.
Scenarios are deterministic; no random seed or random draws were used.

Instrumentation records seed attempts, nonlinear constraint residuals, trust
radii, and conic solver results. These sixteen calls produced 487 nominal
evaluations and 171 outer-iteration records. Logged elapsed times include
instrumentation and initialization; they are **not running-frame performance
measurements**. No full closed-loop campaign was rerun in this investigation.

Further controls comprised eighteen single-node linear programs, thirty-six
first-affine-program feasibility solves, sixty-eight offline trajectory
construction attempts, and seven independent nonlinear trajectory replays.
The replay uses `ode45`, relative/absolute tolerances `1e-11`/`1e-12`, and 31
samples per 50 ms hold. Python independently recomputes target motion,
rectangle distance, road clearance, and strict separating-axis overlap.
Positive clearance in this dense audit is sampled evidence, not a continuous
safety proof. Production MATLAB source hashes were checked unchanged.

## 1. Four crossing failures: a frozen lateral direction contradicts the road

Both perpendicular crossing and turning crossing fail at both speeds. The
initial seed follows the lane through the target's future position. At the
1.6 s rendezvous, the overlapping rectangles receive the lane-normal separation
direction `[0; 1]`. The resulting affine constraints require lateral escape,
although reducing speed and letting the target pass is another physical option.

For the perpendicular crossing, that direction requires the ego center to reach
at least `0.95 + 2.40 + 0.006 = 3.356 m` laterally. The road permits only
`4.00 - 0.95 = 3.05 m` for a lane-aligned vehicle. An LP including all four
linearized ego vertices confirms that permitting a yaw perturbation does not
remove the conflict.

| Fixture, both speeds | Required collision relaxation, unrestricted pose trust (m) | Trust radius 0.5 (m) | Trust radius 0.25 (m) |
| --- | ---: | ---: | ---: |
| Crossing | 0.306000 | 0.856000 | 2.106000 |
| Turning crossing | 0.350481 | 0.900481 | 2.150481 |
| Braking lead, comparison | 0 | 0 | 0.656000 |

These are necessary relaxations of this local affine collision constraint,
not physical overlap measurements. The turning target's orientation increases
the required lateral displacement to about 3.40048 m. Repeatedly shrinking the
trust region around the colliding seed can make escape harder.

All six straight cases have infeasible first affine programs. Removing their
collision rows makes each program feasible. For the four crossing cases,
removing only road rows, removing only the endpoint cone, widening trust to
1.0, or combining wider trust with removal of road rows still leaves the full
program infeasible. Thus the single-node road conflict is one concrete defect;
it is not the sole source of full-horizon infeasibility. Frozen directions at
other times and the coupled dynamics/endpoint conditions also matter.

Code: [`localSafetyRows`](../controller/solvePredictiveControl.m) and
[`dualLinearization`](../controller/predictiveSafetyGeometry.m).

## 2. Straight cases also require recovery to a fixed position at a fixed time

The current `localSeed` builds bounded lane-feedback inputs and integrates the
nonlinear vehicle. It checks target separation when admitting the eventual
terminal continuation; it does not construct an obstacle-avoiding finite
initial trajectory. There is no flow-field avoidance initializer on this path.

The seed selects one terminal index and one lane-projected reference position.
Subsequent optimization must return to this reference at that same index.
The endpoint norm includes longitudinal position: recovering lane, heading,
and speed at a different path station does not suffice. The weighted terminal
radii are approximately `0.00012207` at 8 m/s and `0.00097656` at 15 m/s;
these are weighted norm radii, **not distances in meters**.

This makes natural yielding difficult: after slowing for a crossing vehicle,
ego must regain its original cruise progress before a predetermined deadline.
It also constrains a braking-lead overtake to recover in the seed's 6.70 s.
For both braking-lead cases, removing the endpoint cone alone makes the first
affine program feasible, while removing the road rows does not. This directly
identifies endpoint coupling in their initial convex subproblems. It does not
prove that the original nonlinear problem is globally infeasible.

To distinguish physical avoidance from the fixed endpoint subproblem, offline
inputs were constructed using the same vehicle dynamics, actuator constraints,
road, collision margin, and terminal membership/separation functions. Braking
lead uses a 2.2 m lateral reference excursion. Crossing cases stay on the lane
and temporarily request 4 m/s from an 8 m/s cruise, or 9 m/s from a 15 m/s
cruise, until 2.1 s, then recover cruise. Terminal position and time are admitted
along each resulting trajectory, within the same maximum horizon.

| Speed (m/s) | Fixture | Original endpoint time (s) | Alternative endpoint time (s) | Dense minimum body distance (m) | Original fixed-endpoint residual |
| ---: | --- | ---: | ---: | ---: | ---: |
| 8 | Braking lead | 6.70 | 16.35 | 0.299478 | 19.5206 |
| 15 | Braking lead | 6.70 | 13.25 | 0.294421 | 16.8110 |
| 8 | Crossing | 3.40 | 4.25 | 0.376916 | 76.9953 |
| 15 | Crossing | 3.80 | 4.25 | 0.232786 | 115.7642 |
| 8 | Turning crossing | 10.05 | 11.15 | 0.795186 | 76.9953 |
| 15 | Turning crossing | 6.15 | 7.00 | 0.963812 | 115.7642 |

All six alternatives have zero nominal hard residual and prefix safety slack,
positive road clearance, no sampled state-limit violation, and passing endpoint
membership after tight integration. Minimum road clearance is about 0.453 m.
Their prefixes also pass collision and road checks against the original fixed
model; the positive hard residual is entirely its terminal norm residual.
The final column is in the weighted endpoint norm, not meters.

These are **offline feasibility witnesses with a different admitted endpoint**.
They are not plans returned by the unchanged controller and do not establish
feasibility of its original fixed-position, fixed-time nonlinear problem. They
do show that these six physical encounters are avoidable within the modeled
vehicle and road restrictions. Sixty-two initial lateral/speed probes found
the two lead witnesses; four crossing grids failed. Six subsequent yielding
probes found the four crossing witnesses. Failed searches remain recorded.

The unchanged constant-acceleration target model continues beyond the original
eight-second scenario. The 8 m/s lead witness ends at 16.35 s, shortly after
that target's signed speed crosses zero at 16 s. This model assumption is
retained, not a claim about realistic driver behavior after stopping.

## 3. Four curved cases reject the terminal policy before optimization

`localSeed` examines 445 possible endpoints at 8 m/s and 437 at 15 m/s, out
to 25.6 s. Every tested endpoint passes membership and has a positive road-only
continuation margin of about 1.419 m. Every endpoint fails the target
all-future separation condition. No conic solve or nominal trajectory
optimization is attempted.

The curved terminal check encloses ego's entire future circular motion in a
circular tube and the turning target's entire future motion in an annulus.
It requires these spatial regions to be disjoint, without accounting for the
relative timing of passage through their intersection.

| Fixture | Separation bound at 8 m/s (m) | At 15 m/s (m) | Geometry |
| --- | ---: | ---: | --- |
| Curved head-on | -3.543615 | -3.543719 | Ego and target share the 200 m orbit center |
| Curved crossing | -39.748003 | -39.748108 | Target annulus intersects ego's 200 m circular tube |

These negative values are certificate bounds, not measured penetration depths.
Extending the seed horizon does not change these circular regions. In the
head-on fixture, indefinite opposite-direction motion on the same circle also
causes repeated future encounters: demanding permanent lane cruise without
another avoidance maneuver is substantively restrictive. For curved crossing,
annulus overlap alone does not prove that every time-phased future policy is
unsafe. The code rejects its sufficient certificate, not every physical
avoidance strategy.

The lane and target models keep turning indefinitely; the displayed finite
road segment does not truncate their future motion. This is a mismatch between
the desired encounter test and the currently admitted terminal behavior, not
a conic solver speed problem. See
[`terminalContinuation.separation`](../controller/terminalContinuation.m).

## 4. Longer computation fails to fix the underlying construction

| Speed (m/s) | Fixture | Extended-budget instrumented initialization (s) | Conic calls | Result |
| ---: | --- | ---: | ---: | --- |
| 8 | Braking lead | 30.017 | 47 | Time limit; no feasible plan |
| 15 | Braking lead | 30.028 | 46 | Time limit; no feasible plan |
| 8 | Crossing | 9.035 | 48 | Safety solve failed; no feasible plan |
| 15 | Crossing | 11.152 | 48 | Outer iteration limit; no feasible plan |
| 8 | Turning crossing | 17.784 | 23 | Nominally feasible plan; dense replay collides |
| 15 | Turning crossing | 19.710 | 48 | Outer iteration limit; no feasible plan |

The current solver already returns once it obtains an accepted feasible
witness. These unsuccessful iterations are attempts to repair infeasibility,
not optional cost improvement after an accepted feasible plan. Simply allowing
more iterations would leave the direction and terminal restrictions in place.

## 5. Additional finding: a returned plan can collide between constraint points

The 8 m/s turning-crossing plan returned under the longer budget has zero
nominal hard residual and safety slack. Its nominal sampled collision margin,
after subtracting the configured 6 mm buffer, is 23.462 mm. Nevertheless, tight
open-loop integration of those returned inputs has eight strict rectangle
overlaps between 1.660000 and 1.671667 s. Maximum separating-axis penetration
is 37.025 mm.

The same tight replay evaluated only at control nodes and midpoints has a
minimum body distance of 29.460 mm. Thus the collision occurs **between** the
25 ms-spaced constraint checks. It is not explained by permitting a 6 mm
constraint violation at those checks. The originally passing sampled plan is
not a successful physical avoidance result in this replay.

The initial dense-verification assertion failed and was retained in the logs.
The follow-up exports its actual failing measurements. The endpoint also has
a very small positive tight-replay membership value, `3.94e-9`, despite passing
the nominal endpoint check. The intersample collision is the material safety
finding. This audit replays a full returned plan open loop; no claim is made
that a receding-horizon experiment with fresh replanning was executed here.

Consequently, the earlier oncoming results do not justify a general claim
that a 6 mm sampled buffer prevents collision in fast crossing encounters.

## Implications for a subsequent controller change

1. Construct an obstacle-aware initialization, including longitudinal yielding,
   and choose separation faces from that trajectory. Relinearize dynamics and
   collision geometry together when changing the reference.
2. Revisit the fixed longitudinal terminal phase and recovery deadline. Admit
   physically suitable positions/times while retaining an explicitly justified
   terminal condition; do not silently remove its safety requirement.
3. Define the intended future behavior for persistent curved encounters before
   changing terminal admission. Finite-encounter safety and indefinite cruise
   on intersecting orbits are different requirements.
4. Check intersample motion before accepting a plan, using a justified bound
   or adaptive refinement. Any larger buffer needs validation against these
   measured crossing speeds and geometries.

These are findings and proposed directions within the single controller.
No production algorithm, controller mode, or configuration was added or changed.

## Artifacts and reproduction

Compact evidence is in [`THREAT_FAILURE_ANALYSIS_20260929/`](THREAT_FAILURE_ANALYSIS_20260929/):
`summary.json`, per-evaluation and per-iteration CSVs, raw/source hash manifests,
probe outcomes, instrumentation patches, and diagnostic scripts. Full MAT
records and dense replay poses remain at
`/home/zai/.cache/collisionAvoidance/threat-failure-analysis-20260929/`.
Exported text logs normalize trailing whitespace and terminal control characters;
the original cache logs are retained. No generated binaries, copied controller
implementation, or media are committed.

The `.m.txt` files preserve diagnostic drivers as experiment records. To
reproduce against the source commit, copy the drivers into that cache directory
with the `.txt` suffix removed, and copy `prepare.py`/`analyze.py` there. From
the repository root run:

```bash
python /home/zai/.cache/collisionAvoidance/threat-failure-analysis-20260929/prepare.py
matlab -batch "run('/home/zai/.cache/collisionAvoidance/threat-failure-analysis-20260929/diagnose.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/threat-failure-analysis-20260929/controls.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/threat-failure-analysis-20260929/yieldProbe.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/threat-failure-analysis-20260929/affineControls.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/threat-failure-analysis-20260929/verifyWitnesses.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/threat-failure-analysis-20260929/verifyOriginalWitness.m')"
python /home/zai/.cache/collisionAvoidance/threat-failure-analysis-20260929/analyze.py
```

Recorded execution was against the stated source identity. Solver-budget
boundary behavior and exact timing can vary with machine load. The final
analyzer checks the specific recorded extended-budget collision as well as the
six separated alternatives. It intentionally treats that seventh replay as a
failing plan, not as an avoided encounter. Production tests were not rerun for
this report-only change; the diagnostic controls and independent replay audits
are the validation performed here.
