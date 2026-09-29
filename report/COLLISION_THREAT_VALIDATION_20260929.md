# Collision-threat admission and controller validation

Prepared September 29, 2026. An avoidance experiment now requires an initially separated target whose rectangle strictly overlaps the ego rectangle if the ego continues at constant speed along the given path. Proximity, a turning target that moves away, and target-free tracking do not qualify. No controller algorithm or configuration was changed.

## Why the previous campaign was inadequate

An independent 600 Hz counterfactual over eight seconds found actual baseline collision in **4 of the previous 14 fixtures**, the constant-speed and accelerating oncoming cases at 8 and 15 m/s. The four no-target fixtures and six side-braking/turning fixtures had no baseline collision. Their completed replays were useful regression checks, but counting them as avoidance successes overstated the breadth of demonstrated obstacle avoidance. Original results remain intact; their narrower meaning is corrected here.

## New experiment

`scripts/runCollisionThreatValidation.m` is the danger-only campaign entry. Seven deterministic threats are run at both 8 and 15 m/s: constant-speed oncoming, accelerating oncoming, braking lead, perpendicular crossing, accelerating turning crossing, curved-road oncoming and curved-road crossing. Every fixture requests 160 holds (8 s). Road bounds are +/-4 m; curved references have 200 m radius. Rectangles measure 4.8 by 1.9 m. The sampled planning margin remains 6 mm, with existing 50 ms control sampling, 8/16-hold prefixes, 3 s initial recovery, 512-hold maximum horizon and 5 s soft solve budget. Observations are exact; no sensor error, delay or random draws are injected, and measured computation latency is not applied to the plant.

`collisionThreatScenario.m` constructs target initial conditions by reversing the existing constant-acceleration/constant-sideslip target flow from a future position shared with the cruising ego. The braking lead starts at the same speed, 6.25 m ahead (1.45 m body gap), and decelerates at 0.5 m/s². It stays forward-moving throughout the eight-second experiment. Crossing targets are aligned to meet the cruising ego at 1.6 s. The two original oncoming threats retain their 24 m initial center separation. Constant target turning is retained over the entire prediction, including future repeated arcs; no finite-time target disappearance is introduced to make terminal admission easier.

`givenPathCollisionBaseline.m` computes ideal kinematic centerline cruise at the reference speed and tangent heading, independently of avoidance. For a straight road beginning at station -100 m, the initial station is explicitly projected so the ego starts at world (0,0). The four separating axes of both rectangles define strict penetration; touching alone does not count. A penetration greater than 1e-9 m at a dense sample admits danger. Baseline poses, initial separation, overlap count and first/last collision times are exported. `RequireCollisionThreat=true` rejects benign or initially overlapping fixtures before attempting control. The legacy replay helper retains benign fixtures for separate regression use; the danger-only entry always enables admission.

## Actual outcome

**14/14 baseline collisions confirmed; 4/14 avoidance runs completed; 10/14 initializations rejected before any control was issued.** All failures remain in the denominator. A rejected initialization is not a measured collision under the controller and supplies no successful avoidance trajectory.

| Threat | Baseline first overlap, 8 / 15 m/s (s) | Controller outcome at both speeds |
|---|---:|---|
| Constant-speed oncoming | 1.202 / 0.835 | Completed; positive replay separation |
| Accelerating oncoming | 1.158 / 0.822 | Completed; positive replay separation |
| Braking lead | 2.408 / 2.408 | `noFeasibleContinuation`, soft time limit |
| Perpendicular crossing | 1.182 / 1.378 | `noFeasibleContinuation`, soft time limit |
| Accelerating turning crossing | 1.263 / 1.363 | `noFeasibleContinuation`, soft time limit |
| Curved-road oncoming | 1.298 / 1.392 | `noTerminalContinuation` |
| Curved-road crossing | 1.267 / 1.367 | `noTerminalContinuation` |

The six time-limit failures took 5.005--5.017 s before rejecting initialization. The four curved cases rejected terminal continuation in 0.143--0.466 s. These results distinguish lack of a returned feasible witness from the terminal admission rejection; they do not establish physical impossibility of avoiding those encounters.

The four completed cases reproduce the previous minima: 44.026302 mm (8 m/s oncoming), 5.773052 mm (8 m/s accelerating oncoming), 19.539670 mm (15 m/s oncoming) and 45.339770 mm (15 m/s accelerating oncoming). Maximum running controller-call time among those four is 29.032 ms, excluding initialization; their largest initialization is 4.140901 s. This is a new timing measurement of unchanged controller code, not a speed improvement. The minimum remains positive at dense replay samples; it is below the 6 mm planning value between constraint samples, as previously documented. No continuous-time safety claim is made.

## Validation and visualization

31 new MATLAB behavior tests cover all fourteen fixtures, target/ego rendezvous, initial separation, strict baseline overlap, benign-scene rejection, touching-only rejection and target-free rejection. Combined with 76 existing nonlinear predictive-safety tests, **107 MATLAB tests passed**. Nine existing Python audit tests passed. Python compilation and scoped Git whitespace checks passed. MATLAB Code Analyzer output is retained separately: each inspected file reports that the local settings file cannot be read and default settings were used; there are no additional source findings.

An independent Python audit recomputes every baseline path and target pose, all strict rectangle overlap counts and first collision times from the original parameters. All fourteen agree with the MATLAB admission result. For the four executed runs it independently reconstructs replay geometry, checks positive separation and zero returned hard residual/prefix slack, and retains the full configured-margin diagnostic separately. For the ten initialization failures it preserves empty traces and null execution metrics. Compact machine-readable outcomes, input identities and test results are beside this report.

The renderer now accepts the campaign's scenario order. Successful scenes overlay a dashed cruise ghost that turns red during the baseline collision interval. Failed scenes use a gray baseline vehicle, explicitly state `COUNTERFACTUAL ONLY / INITIALIZATION FAILED`, show `No command`, and display no claimed executed control. Their motion is the uncontrolled counterfactual, not a fabricated avoidance rollout. The final card counts completed and rejected cases separately. Generated playback media is temporary under `/tmp/collision-threat-video-20260929/` and is not committed or admitted as an archive evidence bundle. The 1600x900/30 fps MP4 contains 5252 frames, 175.067 seconds and fourteen chapters. All input/source identities, chapter boundaries and complete decode passed; encoded successful and failed scenes were visually inspected. The video was opened directly in the desktop player after validation.

An initial new rendezvous test exposed a test station-origin error on straight roads: the expected position omitted the initial 100 m station. The baseline already projected the actual starting position correctly. Exporting the initial station and using it in the expectation resolved the test; all final tests pass. The failed first attempt remains in local diagnostic logs. No collision threat was adjusted based on controller success.

## Reproduction

```matlab
addpath('scripts','controller','config');
results = runtests({'tests/collisionThreatScenarioTest.m','tests/nonlinearPredictiveSafetyTest.m'});
assertSuccess(results);
runCollisionThreatValidation(OutputDirectory='/home/zai/.cache/collisionAvoidance/collision-threat-20260929');
```

```bash
python scripts/auditCollisionThreats.py \
    /home/zai/.cache/collisionAvoidance/collision-threat-20260929/campaign.json \
    --output /home/zai/.cache/collisionAvoidance/collision-threat-20260929/independent-audit.json
python scripts/renderCollisionAvoidanceVideo.py \
    --data-directory /home/zai/.cache/collisionAvoidance/collision-threat-20260929 \
    --campaign /home/zai/.cache/collisionAvoidance/collision-threat-20260929/campaign.json \
    --output /tmp/collision-threat-video-20260929/collision-threats.mp4
```

Raw JSON exports, logs and static analysis remain in the indicated cache directory. The scoped source, tests and compact report are committed. Production controller code, unrelated estimator/manuscript/report work, instruction files, nested solver dependencies and generated media are deliberately excluded.
