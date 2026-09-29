# Increase sampled collision clearance to 6 mm — September 29, 2026

The default `collision.safetyMarginMeters` is now 0.006 m, increased from 0.005 m. The extra 1 mm gives the tested trajectory room for the body-distance decrease between constraint samples. This is a parameter change in the existing controller. Its first-feasible policy, primary endpoint correction, dynamics, geometry, constraint-sampling schedule and input limits are unchanged.

The previously limiting 8 m/s accelerating-target scenario improves its independently replayed minimum body clearance from 0.004808675146 m to 0.005773051944 m. All fourteen scenarios complete without sampled overlap and meet the previous 5 mm replay clearance level. The new 6 mm value applies at prediction constraint samples and in the terminal separation certificate; it does not claim that the continuous trajectory always retains 6 mm.

## Change scope

- `config/collisionAvoidanceControllerConfig.m`: increase the default collision margin from 0.005 to 0.006 m.
- `tests/collisionAvoidanceControllerConfigTest.m`: update the existing default/explicit-disable contract to 0.006 m; the explicit zero override remains available.
- `controller/PCBF_CLF_ARCHITECTURE.md`: state the new default and retain its sampled-constraint guarantee scope.

The 1 mm increase is supported by the previous dense replay shortfall of approximately 0.191 mm below a 5 mm planning margin. A targeted 160-hold probe with the 6 mm override produced a 5.773052 mm minimum before the default was changed. The full campaign then reproduced that same minimum using the new default. The observed new shortfall from the 6 mm planning level is approximately 0.227 mm, leaving approximately 0.773 mm above the previous 5 mm level. These comparisons are empirical results for the tested model and fixtures, not an analytical intersample bound.

## Validation

204 controller-related MATLAB tests pass across seven suites: terminal continuation, nonlinear predictive safety, tires, configuration, source budget, road load, and input geometry. This includes the 15 m/s oncoming primary-correction regression. Nine independent Python geometry/audit tests pass. Estimator-only suites were not rerun for this scalar controller-default change.

The fourteen fixtures are unchanged: reference speeds 8 and 15 m/s; prefix lengths 8/16 holds; 50 ms sample; 3-second initial recovery; maximum 512 holds; 5-second soft controller-call search budget; 400 inner and 24 outer iterations; friction 0.85; 4.8 by 1.9 m rectangles; default unlimited input slew limits; +/-4 m full-body road bounds; circular curvature 0.005 1/m. Recovery and circular fixtures request 40 holds; side braking, turning, accelerating turn, constant-speed oncoming and accelerating oncoming request 160. There are no random draws, observer errors, measurement noise or delay. Latency is measured but not injected into plant motion.

Independent plant replay uses ode45 with relative/absolute tolerances 1e-11/1e-12 and 31 geometry samples per hold. Python independently checks target motion, body rectangles and whole-body road bounds, including edge extrema for circular-road radial clearance.

| Metric | Result |
|---|---:|
| Completed fixtures without sampled overlap | 14 / 14 |
| Fixtures meeting previous 5 mm replay level and road bounds | 14 / 14 |
| Fixtures meeting full new 6 mm replay level and road bounds | 13 / 14 |
| Executed holds / audited geometry samples | 1760 / 54560 |
| Strict overlap samples | 0 |
| Global minimum replay body clearance | 5.773052 mm |
| Maximum running controller-call time, excluding initialization | 36.566 ms |
| Subsequent running holds above 50 ms | 0 / 1746 |
| Largest initialization call | 3.845960 s |
| Returned maximum hard residual / prefix safety slack | 0 / 0 |

| Speed (m/s) | Fixture | Completed holds | Minimum body clearance (m) | Maximum running call (ms) | Samples below new 6 mm planning level |
|---|---|---:|---:|---:|---:|
| 8 | recovery | 40 | N/A | 28.687 | 0 |
| 8 | circular | 40 | N/A | 21.253 | 0 |
| 8 | brakingTarget | 160 | 4.100000000 | 25.030 | 0 |
| 8 | turningTarget | 160 | 4.262471482 | 35.013 | 0 |
| 8 | acceleratingTurn | 160 | 2.576376711 | 36.566 | 0 |
| 8 | oncoming | 160 | 0.044026302 | 25.931 | 0 |
| 8 | acceleratingTarget | 160 | 0.005773052 | 25.901 | 6 |
| 15 | recovery | 40 | N/A | 20.991 | 0 |
| 15 | circular | 40 | N/A | 22.019 | 0 |
| 15 | brakingTarget | 160 | 4.100000000 | 26.601 | 0 |
| 15 | turningTarget | 160 | 1.004002140 | 26.480 | 0 |
| 15 | acceleratingTurn | 160 | 1.262488813 | 26.727 | 0 |
| 15 | oncoming | 160 | 0.019539670 | 27.646 | 0 |
| 15 | acceleratingTarget | 160 | 0.045339770 | 27.568 | 0 |

Only the 8 m/s accelerating-target fixture has replay samples below the **new** 6 mm value: six samples, minimum 0.005773051944 m. Its minimum distance at nodes and midpoints is 0.006595908061 m. All sampled distances in that fixture exceed 5 mm. Thus increasing the planning margin resolves the prior sub-5-mm replay result; it does not remove the distinction between constraint samples and denser plant replay.

The reported maximum running time excludes each scenario's initialization and includes the entire controller call. All 1746 subsequent calls are below the 50 ms sample time. Initialization still takes seconds. The campaign is nominal sampled evidence, not a continuous-time or universal real-time guarantee.

## Reproduction and artifacts

```bash
matlab -batch "run('/home/zai/.cache/collisionAvoidance/collision-buffer-20260929-145415/probe.m')"
matlab -batch "run('/home/zai/.cache/collisionAvoidance/collision-buffer-20260929-145415/campaign.m')"
python tests/auditJointPredictiveSafetyTest.py
python /home/zai/.cache/collisionAvoidance/collision-buffer-20260929-145415/analyze.py
python /home/zai/.cache/collisionAvoidance/collision-buffer-20260929-145415/collect.py
```

The targeted probe uses an explicit 6 mm override before the default change; the full campaign uses the final default. The probe source snapshot was updated only after that probe completed. Full-campaign source hashes identify the final version. Collector assertions verify unchanged captured source, all 204 passing tests, zero returned hard residual/slack, first-feasible termination, no secondary replacement after primary admission, fourteen completed fixtures and the previous 5 mm replay level in every fixture.

[COLLISION_BUFFER_INCREASE_20260929/](COLLISION_BUFFER_INCREASE_20260929/) retains compact metrics, the full **6 mm** audit, source/raw hashes, test results, a targeted-probe summary and timing CSVs. Original trace exports retain `requiredClearanceMeters=0.006`; the separate previous-5-mm comparison is derived from the same independently computed distances and is explicitly labeled. No audit threshold was silently changed. Raw logs/traces/source snapshots are retained at `/home/zai/.cache/collisionAvoidance/collision-buffer-20260929-145415`. Baseline commit: `11af2e4b12f1b118a3dcc4cff68bf5bbf5cd2952`. Existing estimator/report edits, agent instructions, external dependencies and generated binaries are excluded from the scoped configuration/docs/test/report commit.
