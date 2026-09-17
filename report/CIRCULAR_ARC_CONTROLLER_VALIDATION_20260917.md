# Circular-arc controller validation

Date: September 17, 2026. Controller baseline: `59b08ee07c50d11decbb7af68c0068efbe82e525`.
Only scenario drivers, diagnostics, tests and reports changed in this experiment;
the controller and estimator algorithms were not modified.

## Outcome

Circular cruise works in the tested declared affine plant. Circular collision
avoidance does not pass: all twelve obstacle trials fail fresh encounter
admission, including independent offline trials with a 5 s search allowance.
The failure is not explained by the 100 ms deadline alone. A diagnostic of the
existing convex certificate identifies excessive circular-coordinate remainder
bounds that can make individual collision and exit inequalities impossible.
This is not evidence that physical collision avoidance is impossible.

## Experimental scope

- Controller only, exact state observations and zero process residuals. The plant
  independently integrates the accepted held affine generator using `expm`.
- Analytic circular reference, curvatures +0.01, -0.01, +0.02 and -0.02 1/m
  (left/right radii 100 and 50 m). The polyline is a display/input carrier;
  `referenceCurve` supplies the actual analytic geometry.
- 300 holds of 0.1 s (30 s), reference body longitudinal speed 8 m/s,
  confirmation region 16 m, clearance 0.25 m, seed 20260912.
- No road boundaries or artificial state/slip limits. Model-domain margins
  remain diagnostic. The exact declared affine plant scope is unchanged.
- Existing physical actuator limits, hard safety certification, soft squared
  CLF slack, curved cruise trim and carried prediction remain unchanged.
- MATLAB R2026a, one computational thread. Complete frame time includes input
  assembly, controller formulation/search/solves and certification. Offline
  plant integration and 11-point-per-hold clearance audits are excluded.
- Six separately saved startup attempts precede each validation batch. These
  stationary-arc preparations also fail admission; they execute no holds and
  contribute no state, command or witness to the measured trials. Startup costs
  are retained, not represented as qualified periodic execution.
- Every failed command attempt terminates its trial. No uncertified search
  iterate or terminal/fallback input is executed. Sampled audit margins do not
  replace the controller's continuous whole-hold certificate.

Let p(s), t(s), n(s) be the arc position, tangent and normal. The target fixtures
are stationary at p(15); oncoming at p(30)+30 t(30), velocity -8 t(30) m/s;
and crossing at p(15)-7.5 n(15), velocity 4 n(15) m/s. Target heading aligns
with its motion, or the tangent for the stationary target. Moving targets follow
inertial straight lines with zero acceleration, jerk and yaw rate. They do not
follow the ego arc. The crossing fixture is deliberately synchronized near
s=15 m; it differs from the earlier fast, early-clearing straight crossing case.

## Periodic results

| Curvature (1/m) | Cruise holds | Cruise max frame (ms) | Stationary failure | Oncoming failure | Crossing failure |
| ---: | ---: | ---: | --- | --- | --- |
| +0.01 | 300/300 | 55.688 | 0 s, 0 holds | 2.6 s, 26 holds | 0 s, 0 holds |
| -0.01 | 300/300 | 40.762 | 0 s, 0 holds | 2.6 s, 26 holds | 0 s, 0 holds |
| +0.02 | 300/300 | 23.354 | 0 s, 0 holds | 2.6 s, 26 holds | 0 s, 0 holds |
| -0.02 | 300/300 | 20.045 | 0 s, 0 holds | 2.6 s, 26 holds | 0 s, 0 holds |

All four nominal cruise trials pass the 100 ms gate. Maximum final lateral error
is 1.914e-5 m, heading error relative to curved trim is 8.02e-9 rad, and body
longitudinal speed error is 1.14e-7 m/s. These start at trim and primarily test
maintenance of cruise. They do not demonstrate recovery after avoidance.

All twelve periodic obstacle runs fail, with failing frame times 101.233--108.974
ms. The oncoming runs are certified until first admission at 2.6 s; they do not
complete an encounter. The initial stationary/crossing failures execute no
holds, so an infinite/JSON-null sampled clearance is not an avoidance result.
The independent 5 s diagnostic allowance also fails all twelve admissions:
stationary/crossing searches usually exhaust bounded families, while some
oncoming searches exhaust the work budget. Actual diagnostic maxima are
2.084--5.036 s. No relaxed search plan is accepted.

### Recovery from initial errors

A separate batch begins with deviations [0.5 m, 0.02 rad, -1 m/s, 0, 0] from the
curved trim in [d, e_psi, v_x, v_y, r]. There is no obstacle in this batch.

| Curvature (1/m) | Periodic executed holds | Maximum frame (ms) | Outcome |
| ---: | ---: | ---: | --- |
| +0.01 | 163/300 | 322.423 | Deadline failure at 16.3 s |
| -0.01 | 300/300 | 38.927 | Completed within deadline |
| +0.02 | 300/300 | 22.709 | Completed within deadline |
| -0.02 | 300/300 | 22.985 | Completed within deadline |

The independent +0.01 diagnostic run completes 30 s and recovers, but reaches
423.943 ms; its early expensive frames are dominated by formulation. Thus all
four curves show functional recovery, while the first recovery batch does not
qualify uniformly for periodic execution. All completed runs end at essentially
the same small errors as nominal cruise, also sustained over the last two
seconds. No artificial early-recovery requirement is imposed. A soft CLF alone
is not a general theorem of asymptotic convergence with persistent slack.

An independent repeat of the +0.01 recovery trial completes 300/300 holds
with a maximum frame of 55.586 ms. It does not reproduce the earlier spike.
Both attempts are retained; the repeat does not erase the measured deadline
failure or establish a worst-case bound. The transient latency cause remains
unresolved by this experiment.

The heading error is measured against the curve-specific operating point.
For +0.01 1/m, trim [e_psi,v_x,v_y,r] is approximately
[-0.0113161 rad,8 m/s,0.0905323 m/s,0.0800051 rad/s]; steering is 0.0313892 rad.
Nonzero body sideslip, heading offset and yaw rate are consistent with curved
steady travel and must not be mistaken for failure to recover.

## Certificate diagnosis

`laneGeometry.sweptCellFrames` bounds station and lateral excursion over the
entire actuator box. It uses the maximum lateral extent of the horizon when
building cell charts. `hardEncounterBarrier.completionRows` independently uses
the full terminal actuator-box image to construct its final chart. Neither
calculation uses a tight, verified local search region around a candidate.

For station radius S, lateral magnitude D and curvature k, the current
`laneGeometry.referenceFrame` assigns each Cartesian component the bound

```
E = abs(k)*S^2/2 + D*min(2,abs(k)*S).
```

This is a conservative geometric enclosure, not sensor noise or measured plant
error. At k=0 it vanishes. On an arc it becomes very large as the horizon and
unrestricted actuator-box image expand. The same charge enters collision
separation and finite encounter exit. A single horizon-wide lateral extent
also inflates earlier cell bounds.

The persistent diagnostic constructs the original stationary-target programs
and audits every row A U <= b. With symmetric actuator outer box |U| <= r,
the greatest attainable margin for that row is b+|A|r. A negative value proves
that even this row cannot hold over the outer box. Positive values do not prove
joint feasibility; terminal, dynamics-coupled and other geometric rows still
apply. The diagnostic uses one proposed support family and default exit
normal, not an exhaustive direction search.

| Radius | Holds | Max cell Cartesian remainder per component (m) | Individually impossible collision rows | Best exit-row box margin (m) |
| --- | ---: | ---: | ---: | ---: |
| Straight | 48 | 0 | 0 | +99.157 |
| 100 m | 24 | 15.700 | 0 | -0.706 |
| 100 m | 32 | 50.745 | 303 | -4.419 |
| 100 m | 48 | 271.083 | 1280 | -136.203 |
| 50 m | 48 | 534.957 | 1893 | -383.039 |

For the 100 m stationary fixture, automatic admission proposes 48 holds. At that
horizon, the terminal chart spans 230.376 m of station and charges 271.082 m in
each Cartesian component. The modal terminal SOC radius remains positive
(minimum 0.350245); this differs from the earlier uncertain-estimator negative
SOC-radius failure. The complete sweep includes horizons 16,24,32,40,48,56,64
and curvatures 0,0.0025,0.01,0.02. The raw audit is included in the JSON report.

Increasing solver time or merely extending the horizon cannot repair an
individually impossible fixed row. A suitable next algorithm investigation is
to construct and verify local chart regions for each searched convex family,
recompute valid geometric remainder bounds on those regions, include their
membership in the optimization, and preserve the complete carried witness when
changing regions. This would require a new soundness/continuation check; simply
reducing the remainder constants would invalidate the current certificate.
This experiment does not implement that controller redesign.

## Reproduction and artifacts

```matlab
addpath('scripts');
runCircularArcControllerValidation(OutputDirectory='/absolute/path/main');
runCircularArcControllerValidation(OutputDirectory='/absolute/path/recovery', ...
    Scenarios="cruise",InitialTrackingError=[.5;.02;-1;0;0]);
diagnoseCircularArcCertificate(OutputFile='/absolute/path/geometry-audit.json');
```

Detailed MAT/JSON traces, startup attempts and actual failure messages are in
`/home/zai/.cache/collisionAvoidance/arc-controller-20260917/` under `validation/`,
`recovery/` and `recovery-repeat/`. The compact committed machine-readable record
is [CIRCULAR_ARC_CONTROLLER_VALIDATION_20260917.json](CIRCULAR_ARC_CONTROLLER_VALIDATION_20260917.json).

Validation: 19 focused MATLAB tests passed (new circular scenario behavior,
existing road-boundary configuration and curved cruise certificates), with zero
failures or incomplete tests. `checkcode` was run on the three edited/new drivers
and the new test; its saved diagnostics are in `code-check.mat`. This turn did
not rerun the entire suite; the unchanged controller baseline had 621 passing
regressions in the previous implementation validation. Diff whitespace and
committed file scope were checked. Timing and failure evidence are preserved;
these experiments establish neither a nonlinear physical-vehicle guarantee nor
a worst-case execution-time bound.
