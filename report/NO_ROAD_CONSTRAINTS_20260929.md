# Controller validation without road-boundary constraints

Date: September 29, 2026. Base revision:
`b87600c2786a3b2cde4d365c7e31cd6c3383b264`.

Road boundaries have been removed from the controller's finite prediction,
nonlinear feasibility evaluation and indefinite terminal admission. The given
path remains the cruise, lane-recovery and terminal reference. There is one
controller and no new configuration switch or driving mode. The 6 mm collision
buffer, vehicle dynamics, actuator limits, fixed endpoint construction and
target prediction are unchanged.

## Implementation

- `solvePredictiveControl.m` now forms only collision geometry rows. It removes
  the eight road inequalities at each start/midpoint/endpoint, their vertex
  projections and derivatives, and road residuals from cached trajectory
  evaluations. Road departure cannot contribute to prefix slack or reject
  hard completion.
- `terminalContinuation.separation` checks only indefinite target separation.
  It no longer tests road width or whether the terminal orbit fits a corridor.
  A target-free continuation has no geometric separation restriction.
- The controller reference frame contains only origin, heading and curvature.
  Optional road widths do not enter the continuation context, so changing
  them does not invalidate a retained witness. Input normalization still
  validates the shape and positivity of supplied width metadata; it does not
  require the vehicle to fit within it.
- State version 53 prevents reuse of the old road-constrained witness format.
  `minimumRoadMargin` and cached `stageRoad` values are removed from controller
  outputs. Metadata instead explicitly states `roadConstraintsEnforced=false`.
- Experiment exports record this scope. Independent audits still measure road
  clearance, but road departure does not fail a new avoidance result.
  Historical exports without this field retain their road-constrained audit.

The reference path still matters: changing it requires a new problem, and its
lane-recovery costs and terminal position/time constraints remain active.

## Validation

All **240 MATLAB tests** in eight controller/scenario suites pass:
`terminalContinuationTest`, `nonlinearPredictiveSafetyTest`,
`modifiedFialaTireTest`, `collisionAvoidanceControllerConfigTest`,
`controllerSourceBudgetTest`, `longitudinalRoadLoadTest`,
`controllerInputGeometryTest`, and `collisionThreatScenarioTest`.

New or updated behavior checks cover oncoming avoidance with a corridor
narrower than the vehicle, target-free terminal admission at both curvature
signs under the same narrow metadata, target separation independent of road
width, retained-plan reuse after changing widths, and continued rejection of
a changed reference path. Collision, input and terminal restrictions retain
their existing regression coverage. The nine-core-source limit still passes.

All **12 Python audit tests** pass, including road departure accepted as a
diagnostic for new exports, historical road enforcement preserved, and
collision still rejected when road constraints are absent. MATLAB Code
Analyzer checks eight changed MATLAB files; seven have no findings and the
solver has thirteen sparse-indexing advisories in unchanged assembly code.
Python compilation and scoped Git whitespace checks also pass. This is the
selected controller/scenario suite, not the entire estimator/perception suite.

## Fourteen actual collision threats

The same seven encounter types at 8 and 15 m/s were rerun for 160 holds
(eight seconds), with exact observations, no injected observation or timing
error, 50 ms control intervals, 8/16 prefix holds, a 512-hold maximum horizon
and a five-second soft search budget. No random draws were used. Each fixture
starts separated and has a strict collision under constant-speed given-path
cruise, independently verified at 600 Hz.

Every issued hold is replayed using `ode45`, relative/absolute tolerances
`1e-11`/`1e-12`, with 31 output samples per hold. Python independently
reconstructs target motion and rectangle geometry. Road-clearance numbers
below refer to the original displayed corridor, 4 m on each side.

| Speed (m/s) | Scenario | Outcome | Minimum replay body distance (mm) | Maximum running controller call (ms) | Successful initialization (s) | Minimum road margin (m), diagnostic only |
| ---: | --- | --- | ---: | ---: | ---: | ---: |
| 8 | Head-on | 160/160, no sampled collision | 44.026246 | 26.989 | 1.196536 | 0.759043 |
| 8 | Accelerating head-on | 160/160, no sampled collision | 5.773407 | 24.647 | 0.804017 | 0.736540 |
| 15 | Head-on | 160/160, no sampled collision | 81.684963 | 27.594 | 1.668894 | -0.004774 |
| 15 | Accelerating head-on | 160/160, no sampled collision | 26.026604 | 26.626 | 1.322373 | -0.113155 |

The two 15 m/s runs depart the original road boundary by about 4.8 mm and
113.2 mm. This is permitted under the requested problem formulation, and
demonstrates that the boundary is no longer enforced. Every issued plan has
zero hard residual and prefix collision slack.

The other ten scenarios still fail before issuing any command:

| Speed (m/s) | Scenario | Failed initialization duration (s) | Failure |
| ---: | --- | ---: | --- |
| 8 | Braking lead | 5.012352 | No feasible continuation; time limit |
| 8 | Crossing | 5.003617 | No feasible continuation; time limit |
| 8 | Turning crossing | 5.012295 | No feasible continuation; time limit |
| 8 | Curved head-on | 0.466499 | No admissible terminal continuation |
| 8 | Curved crossing | 0.148146 | No admissible terminal continuation |
| 15 | Braking lead | 5.001617 | No feasible continuation; time limit |
| 15 | Crossing | 5.009012 | No feasible continuation; time limit |
| 15 | Turning crossing | 5.010749 | No feasible continuation; time limit |
| 15 | Curved head-on | 0.419700 | No admissible terminal continuation |
| 15 | Curved crossing | 0.139780 | No admissible terminal continuation |

Overall: **4/14 completed with positive sampled separation, 10 initialization
failures**, unchanged in count from the road-constrained campaign. Failed
cases have no executed trajectory or avoidance-clearance measurement. Removing
road constraints has not resolved the other mechanisms identified in the
[failure analysis](THREAT_FAILURE_ANALYSIS_20260929.md): colliding initial
rollouts and frozen separating faces, fixed endpoint phase/time, and
all-future intersecting-orbit admission.

## Timing and safety interpretation

The largest measured **running controller call is 27.594 ms**, excluding
initialization. None of the 636 running calls in the four completed cases
exceeds 50 ms. These calls make no new conic solves: they accept a feasible
continuation. The measurement covers the public controller call, not just
`coneprog`. It does not establish a runtime bound for frames that require
new feasibility restoration or for the ten uninitialized cases.

Successful initialization takes 0.804017--1.668894 s. The six failed straight
initializations exhaust about five seconds. These are initialization results,
not running-frame measurements. Timing is from one unprofiled campaign; no
statistical speedup claim is made against the previous run.

The smallest dense-replay distance is 5.773407 mm. The 6 mm constraint still
applies at prediction nodes/midpoints and in terminal separation; dense replay
can be closer between those checks. This change does not address the
intersample collision found in the previous diagnostic turning-crossing plan.
Positive clearance in this sampled campaign is not a general continuous-time
safety guarantee.

## Reproduction and artifacts

Compact results, tests, analyzer findings, comparison CSV, and source/raw
hash manifests are in
[`NO_ROAD_CONSTRAINTS_20260929/`](NO_ROAD_CONSTRAINTS_20260929/).
Full trajectory exports and original logs remain at
`/home/zai/.cache/collisionAvoidance/no-road-constraints-20260929/`.
No media or generated binary is saved in the repository or admitted into the
external archive.

The recorded driver is `runChecks.m.txt`; copy it to the cache directory as
`runChecks.m` to rerun the selected tests, analyzer and campaign. The main
commands executed were:

```bash
matlab -batch "run('/home/zai/.cache/collisionAvoidance/no-road-constraints-20260929/runChecks.m')"
python tests/auditJointPredictiveSafetyTest.py
python scripts/auditCollisionThreats.py /home/zai/.cache/collisionAvoidance/no-road-constraints-20260929/campaign.json --output /home/zai/.cache/collisionAvoidance/no-road-constraints-20260929/independent-audit.json
python /home/zai/.cache/collisionAvoidance/no-road-constraints-20260929/analyze.py
```

`analyze.py` compares the campaign against the preceding road-constrained
report, verifies source hashes and retains the failed cases. The experiment
configuration and every raw trajectory export have independent hashes.
