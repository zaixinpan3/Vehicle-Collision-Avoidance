# Strict collision criterion with no required extra buffer

Date: September 28, 2026.

## Correction of the previous interpretation

The user's requirement is strictly positive distance between the two rectangular
vehicle footprints, not a minimum 0.10 m separation. The earlier 0.10 m threshold
came from the controller configuration. It was incorrect to present that optional
buffer as the user's requirement.

Reassessment of the **unchanged previous eight trajectories** from
`EXACT_STATE_PCBF_EXPERIMENTS_20260928` passes all eight under the strict positive-distance
criterion, with no sampled road crossing. Their minimum target distances were
0.099049399/2.618562850 m at 8 m/s (oncoming/turningTarget) and
0.099878972/0.073114741 m at 15 m/s. The earlier buffer shortfalls do not constitute
collision failures under the user's criterion. The previous timing conclusions
remain unchanged. Original measurements and reports are preserved; this report
supersedes only the interpretation of the collision requirement. The separate
`previous-reassessment.json` records the original 0.10 m optimization buffer and
the zero-extra-buffer evaluation criterion explicitly.

## Implemented change

`collision.safetyMarginMeters` now defaults to zero and permits finite nonnegative
values. The public controller no longer rejects zero additional clearance. The
existing signed rectangle separation rows remain; no controller method or execution
mode was added. An extra positive buffer is only an explicit configuration choice.
The offline audit now requires distance **strictly greater than zero**, even when
the configured extra buffer is zero. Its tolerance can never admit contact or overlap.
Python tests exercise overlap, exact contact, and small positive distances; MATLAB
regressions cover zero-buffer configuration, invalid values and a separated target.

The numerical optimizer uses closed nonnegative signed-separation constraints and
feasibility tolerances. These do not themselves prove strictly positive distance or
exclude intersample contact. Strict distance is therefore checked independently on
the dense replay. No hidden positive physical buffer is substituted for 0.10 m.

## Fresh zero-buffer experiments

These are new optimizations after changing the default, not relabeled old trajectories.
8/8 replays completed, with 800 holds and
24,800 dense samples. 6/8 replays have strictly positive
sampled target distance (target-free fixtures included); 6/8
pass the complete replay audit including completion, nominal solver and road checks.

| Reference speed | Scenario | Holds | Minimum distance (m) | Zero-distance samples | Strict-overlap samples | Minimum road margin (m) | Later median solve (ms) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 8 m/s | recovery | 40/40 | No target | 0 | 0 | 3.039233 | 21.41 |
| 8 m/s | circular | 40/40 | No target | 0 | 0 | 3.022088 | 22.20 |
| 8 m/s | oncoming | 160/160 | 0.008360305 | 0 | 0 | 0.437090 | 242.69 |
| 8 m/s | turningTarget | 160/160 | 2.618361683 | 0 | 0 | 3.046541 | 258.04 |
| 15 m/s | recovery | 40/40 | No target | 0 | 0 | 3.039185 | 40.31 |
| 15 m/s | circular | 40/40 | No target | 0 | 0 | 3.033143 | 40.79 |
| 15 m/s | oncoming | 160/160 | 0.000000000 | 9 | 9 | 0.355577 | 211.94 |
| 15 m/s | turningTarget | 160/160 | 0.000000000 | 4 | 4 | 2.856097 | 211.92 |

Zero rectangle distance includes contact and overlap; either violates the strict criterion.
Strict-overlap counts use a negative separating-axis support gap below -1e-10 m to
distinguish penetration from roundoff at contact; that tolerance does not relax the
strict collision audit. Replays continue after geometric overlap; no impact dynamics are modeled.

- 8 m/s oncoming: first minimum at 1.516667 s; minimum signed separating-axis gap 0.00836030473 m; nominal safety-slack maximum 8.76996327e-06; hard residual maximum 0; converged 159/160 calls; 160 calls exceed 50 ms.
- 8 m/s turningTarget: first minimum at 1.426667 s; minimum signed separating-axis gap 2.2192216 m; nominal safety-slack maximum 0; hard residual maximum 0; converged 160/160 calls; 160 calls exceed 50 ms.
- 15 m/s oncoming: first minimum at 1.011667 s; minimum signed separating-axis gap -0.000628487832 m; nominal safety-slack maximum 6.3763226e-06; hard residual maximum 0; converged 159/160 calls; 160 calls exceed 50 ms.
- 15 m/s turningTarget: first minimum at 1.018333 s; minimum signed separating-axis gap -0.0218859314 m; nominal safety-slack maximum 2.00347441e-06; hard residual maximum 0; converged 160/160 calls; 160 calls exceed 50 ms.

The default oncoming case has nine overlapping samples from 1.011667 to
1.025000 s. Its most negative signed separating-axis gap is -0.000628488 m;
the constrained midpoint is also slightly overlapping (-0.000000999 m),
consistent with a numerical feasibility tolerance that does not establish
strict separation. The default turning-target case has four overlapping samples
from 1.018333 to 1.023333 s, with minimum signed gap -0.021885931 m. Its minimum
distance at nodes and midpoints is positive (0.000993059 m): this is an
intersample collision. Both default cases fail the user's collision criterion,
not an extra clearance-buffer requirement. Geometry and solver residuals must
be reported separately; the returned nominal safety-slack sums remain within 1e-5.

All 139/139 selected MATLAB tests and all three Python
strict-collision audit tests passed. Factory analysis of current paths retains the
existing sparse-indexing performance notices. The full estimator/perception suite
was not rerun for this collision-criterion change.

## Conditions and limitations

All states are exact: no observer, noise, measurement latency or uncertainty set.
The baseline uses 8 m/s reference speed and 8 minimum horizon holds. The default
cohort uses the production 15 m/s reference and 16 minimum holds. Both use 50 ms
holds, friction 0.85, 4.8 by 1.9 m rectangles, 40-degree steering limit, unbounded
input slew rates and an 8 m wide corridor. Full configurations are exported.
Recovery starts 0.01 m off the straight lane; circular cruise starts at its trim
with curvature 0.005/m. These fixtures run 40 holds each. Each target fixture runs
160 holds; the target starts at (24,0) m, heading pi, speed 8 m/s, with heading rate
zero or -0.8 rad/s. No random draws are used.

The same Fiala physical equations are integrated independently with ode45
(RelTol 1e-11, AbsTol 1e-12), with 31 samples per hold. Python independently audits
rectangle geometry and whole-body road containment. This remains sampled nominal-model
validation; simulation advances by 50 ms regardless of solve duration. Runtime overruns
are measured, not injected as actuator delay. Ordinary workstation activity is not
isolated, so timing does not establish a controlled speedup or real-time guarantee.
A strict sampled pass does not prove separation at every continuous-time instant.

## Reproduction and provenance

```matlab
addpath('scripts');
validateNonlinearPredictiveController('/absolute/output/baseline');
runNonlinearPredictiveSafetyValidation(Scenarios=["recovery","circular"], ...
    Frames=40, ControllerConfiguration=struct(), ...
    OutputFile='/absolute/output/default/short-replays.json');
runNonlinearPredictiveSafetyValidation(Scenarios=["oncoming","turningTarget"], ...
    Frames=160, ControllerConfiguration=struct(), ...
    OutputFile='/absolute/output/default/oncoming.json');
```

Create the output directories first, then audit each cohort:

```bash
python3 tests/auditJointPredictiveSafetyTest.py
python3 scripts/auditJointPredictiveSafety.py /absolute/output/baseline --output /absolute/output/baseline/strict-audit.json
```

A nonzero audit exit denotes a recorded failure of the strict criterion and must not
be suppressed. Compact JSON reports, source/artifact manifests and LF-normalized timing
CSV exports live beside this report. Original full traces, iteration diagnostics,
logs and analysis methods remain at
`/home/zai/.cache/collisionAvoidance/zero-buffer-experiments-20260928/`.
No old trace was overwritten or rerun result substituted into the prior report.
Unrelated observer/report edits, untracked research, dependencies and binaries are
excluded from this change.
