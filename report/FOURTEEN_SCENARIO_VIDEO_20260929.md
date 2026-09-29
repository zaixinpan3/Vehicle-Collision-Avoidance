# Fourteen-scenario controller video

Prepared on September 29, 2026. The video visualizes the existing exact-observation campaign with the 6 mm sampled collision planning margin. It makes each complete closed-loop motion, obstacle pass and subsequent lane return visible. No controller setting or simulation result was changed for rendering.

## Deliverable

- MP4: 1600 by 900 pixels, 30 frames/s, H.264, yuv420p, 4504 frames, 150.133 seconds, silent.
- Original output: `/home/zai/.cache/collisionAvoidance/fourteen-scenario-video-20260929-150838/collision-avoidance-fourteen-scenarios.mp4`.
- Preserved export copy: `../Pan_Dynamics_OPT_Archive_Kit/05_Project_B_Collision_Avoidance/2026-09-29_Fourteen_Scenario_Video/video_output_export/collision-avoidance-fourteen-scenarios.mp4`.
- MP4 SHA-256: `758ac80271c23d251cd89ac852b1c9479b3434c48901f910a23bff785fca3ce1`.
- Renderer: `scripts/renderCollisionAvoidanceVideo.py`; dependencies are Python, NumPy, Pillow, ffmpeg and ffprobe. Inter is used when locally available; DejaVu Sans is the fallback.

The dark dashboard uses cyan for the ego and coral for the target. A top-down camera follows the ego at equal world-axis scale, with true rectangle body dimensions, a 5 m scale bar, held steering, current speed and current geometric body distance. An overview shows the full **recorded** paths. The lower charts show lateral motion, held steering and body clearance (or longitudinal speed when no target exists). These are recorded trajectories, rather than claimed online predictions.

Obstacle scenes play at 1x, slowing to 0.5x within 0.65 simulation seconds of the closest recorded approach and pausing at that exact dense-sample instant for 0.9 video seconds. A local equal-axis-scale inset appears when clearance is below 0.6 m. The two-second no-target fixtures play at 0.5x. Opening, scenario labels, short fades and a final campaign overview provide continuity. The MP4 contains fourteen named chapters; the first chapter includes the 2.2-second opening cover because the MP4 muxer extends its start to zero.

## Scenario order

Times below are scenario content starts; the first embedded chapter starts at 00:00. Entries 1--7 use 8 m/s and entries 8--14 use 15 m/s reference speed.

| Scene | Reference speed | Scenario | Content start |
|---|---:|---|---:|
| 01 | 8 m/s | Lane recovery | 00:02.2 |
| 02 | 8 m/s | Curved-road tracking | 00:08.1 |
| 03 | 8 m/s | Braking target beside the road | 00:14.0 |
| 04 | 8 m/s | Turning target | 00:26.1 |
| 05 | 8 m/s | Accelerating turn | 00:38.3 |
| 06 | 8 m/s | Constant-speed head-on encounter | 00:50.4 |
| 07 | 8 m/s | Accelerating head-on encounter | 01:02.5 |
| 08 | 15 m/s | Lane recovery | 01:14.7 |
| 09 | 15 m/s | Curved-road tracking | 01:20.6 |
| 10 | 15 m/s | Braking target beside the road | 01:26.5 |
| 11 | 15 m/s | Turning target | 01:38.6 |
| 12 | 15 m/s | Accelerating turn | 01:50.7 |
| 13 | 15 m/s | Constant-speed head-on encounter | 02:02.9 |
| 14 | 15 m/s | Accelerating head-on encounter | 02:15.0 |

## Source and interpretation

Input JSON files are the fourteen `speed8`/`speed15` exports under `/home/zai/.cache/collisionAvoidance/collision-buffer-20260929-145415/`, previously analyzed in `report/COLLISION_BUFFER_INCREASE_20260929.md`. The producing controller revision is `86939fee8915337f4662b53342104cde288a5ef7`. Every input hash and exact chapter timing is recorded in `FOURTEEN_SCENARIO_VIDEO_20260929/render-manifest.json`.

The renderer interpolates ego states from the recorded ode45 hold replay and computes targets using their exported initial state and analytic motion. Held controls are selected by the actual hold interval. Nearest rectangle-boundary pairs define the inset. The four no-target scenes remain in the sequence; their 10 mm recovery offset and small curved-road error are best seen in the millimeter-scale lower chart. The braking-target model eventually reverses because its constant negative acceleration is retained; the video describes this recorded behavior.

Independent Python geometry reproduced every recorded minimum to within 1e-9 m and confirmed positive dense-sample separation. The campaign's smallest recorded body clearance is 5.773052 mm in scene 07. The 6 mm line refers to planning constraint samples and is not presented as a continuous 6 mm clearance. The final timing card reports the original maximum running controller call, 36.566 ms, excluding initialization. The inputs use exact observations and do not inject sensing or computation delay into motion. This video is a visualization of the previous experiment, not a new robustness or continuous-time safety result.

## Reproduction and validation

From the repository root:

```bash
python scripts/renderCollisionAvoidanceVideo.py \
    --data-directory /home/zai/.cache/collisionAvoidance/collision-buffer-20260929-145415 \
    --output /home/zai/.cache/collisionAvoidance/fourteen-scenario-video-20260929-150838/collision-avoidance-fourteen-scenarios.mp4
python report/FOURTEEN_SCENARIO_VIDEO_20260929/verifyVideo.py \
    --manifest report/FOURTEEN_SCENARIO_VIDEO_20260929/render-manifest.json \
    --output report/FOURTEEN_SCENARIO_VIDEO_20260929/verification.json
```

Python compilation, all fourteen input identities, renderer identity, independently reconstructed minimum distances, MP4 codec/pixel format/frame rate/frame count/resolution/duration and fourteen chapter boundaries passed. Complete ffmpeg decode returned zero with no errors. Encoded stills from every chapter, the closest approach in scene 07, and renderer previews were visually inspected. The initial chapter-start check exposed MP4's extension of chapter 01 over the cover; the verification now checks that actual container behavior while retaining the content start separately. No controller tests were rerun because no controller code or configuration changed.

Compact manifests, metadata and verification are committed beside this report. Generated MP4 and PNG media remain outside the repository; unrelated estimator changes, manuscripts, instruction files and nested solver dependencies are excluded from this task's commit.
