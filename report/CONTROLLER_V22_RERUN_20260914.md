# Independent version-22 straight-scene rerun

September 14, 2026. Tested controller/observer source:
`e232514d03814c0cc3d1934642861d353d8543a7`.
This rerun changes no controller, observer, configuration, or native solver
algorithm. It adds the reusable sequential campaign entry
[`runStraightControllerRerun.m`](../scripts/runStraightControllerRerun.m).

## Findings

The three exact-state 8 m/s scenes now complete avoidance and return to
cruise. Exact-state oncoming avoidance with a physical 30 m visibility gate
also completes, but its 10 m/s cruise recovery remains oscillatory even in
an additional 90 s trial. Bounded-noise stationary/oncoming admissions and
the actual NRMM joint encounter still fail. Every-frame 100 ms usability is
not established. All 59 selected regression cases pass.

| Trial | Executed / requested holds | Safety and recovery observation | Maximum frame (ms) | Frames over 100 ms |
| --- | ---: | --- | ---: | ---: |
| Exact stationary, 8 m/s | 300 / 300 | Safety/truth audits pass; returns to cruise | 7144.506 | 1 / 301 |
| Exact oncoming, 8 m/s | 300 / 300 | Safety/truth audits pass; returns to cruise | 1478.106 | 1 / 301 |
| Exact crossing, 8 m/s | 300 / 300 | Safety/truth audits pass; returns to cruise | 71.288 | 0 / 301 |
| Bounded stationary | 0 / 300 | No admitted certificate | 30127.155 | 1 / 1 |
| Bounded oncoming | 0 / 300 | No admitted certificate | 34257.104 | 1 / 1 |
| Bounded crossing | 300 / 300 | Safety/truth audits pass; small final cruise errors | 65.759 | 0 / 301 |
| Exact oncoming, 30 m visibility, 10 m/s | 300 / 300 | Avoidance completes; cruise not settled | 423.246 | 5 / 301 |
| NRMM oncoming, 30 m visibility, 10 m/s | 35 / 300 | Fails at first publication, 3.5 s | 3215.662 | 2 / 36 |
| Additional exact 30 m visibility trial, 90 s | 900 / 900 | Avoidance completes; sustained cruise oscillation | 436.688 | 12 / 901 |

First admission, failed decisions, and the final unexecuted command are
included in timing. The first two maxima occur at admission; their maximum
successor frames are 88.361 and 71.490 ms. The static 8 m/s scenarios and the
30 m visibility comparisons use different declared reference speeds,
encounter initialization and confirmation ranges; their results are not
interchangeable.

## Safety, terminal execution, and cruise

All completed runs issue only zero-safety-value certified commands. The
four completed exact/bounded 8 m/s trials pass the driver's independent
sampled state-domain and truth-containment checks. The three exact trials
finish at 7.999999998 m/s, with negligible lateral/heading error. The bounded
crossing finishes at 8.000073896 m/s, lateral error 0.004126 m and heading
error 0.00007564 rad. None of these four trials executes the terminal law.
The exact stationary/oncoming trials use 32/39 carried-witness commands;
those commands still belong to the optimized finite witness.

| Trial | Minimum separation beyond required clearance (m) | Minimum road margin beyond required clearance (m) | Confirmed release (s) |
| --- | ---: | ---: | ---: |
| Exact stationary | 0.001107285 | 1.118265 | 4.2 |
| Exact oncoming | 0.005100649 | 1.302365 | 5.0 |
| Exact crossing | 9.269858213 | 3.800000 | 0.7 |
| Bounded crossing | 9.276598066 | 3.790400 | 0.7 |
| Exact 30 m visibility, 30 s | 0.000855755 | 0.000163807 | 6.6 |
| Exact 30 m visibility, 90 s | 0.000850427 | 0.001234030 | 6.6 |

These margins are additional to the configured 0.25 m clearance. The two
crossing trials have large physical separation and are mild regressions.

The 30 s visibility-gated exact run finishes at speed 9.980063 m/s, but
lateral error 0.184764 m and heading error -0.163516 rad. During 20--30 s,
lateral position ranges from -1.465485 to 3.787568 m. Two commands execute
the terminal law at 17.2 and 17.3 s. A near-reference final speed alone does
not establish along-path cruise recovery.

An independent repeat from the initial state requests 900 holds. The target
releases at 6.6 s; during 80--90 s, with no active target:

- Speed ranges from **6.896396 to 9.750761 m/s** against a 10 m/s reference.
- Lateral position ranges from **-3.710911 to 3.572470 m**.
- Heading error ranges from **-0.261099 to 0.239762 rad**.
- Final state has speed 9.470892 m/s, lateral error 3.572470 m and heading
  error 0.091583 rad.

There are 22 terminal-law commands over the 90 s trial. Every one records
`frameDeadline` after the performance stage exhausts its work budget before
the native solve. Consequently, the accepted finite plan runs out and its
terminal continuation is executed. This identifies a concrete interaction
between performance-stage work time and actual terminal braking. It does
not establish that those 22 commands alone explain the complete lateral
oscillation; the intervening fresh plans also require analysis. This is a
90 s observation, not a proof of a mathematical limit cycle or eternal
nonconvergence. No early recovery deadline is imposed.

The 90 s run is a fresh repeat, not a continuation of the original saved
30 s trajectory. Measured work time affects which fresh plans replace the
carried witness, so the trajectories differ even with the same random seed.
Independent reconstruction of every executed affine hold matches saved
successors within 4.45e-16 in state coordinates. Eleven checks per hold find
positive minimum state-domain margins of 0.132599/0.123922 for the 30/90 s
range-gated runs. The range driver does not expose the complete carried
information-set history, so its report does not claim the exact driver's
full independent set-containment audit.

## Remaining admission failures

Bounded stationary and oncoming trials each try four candidate problems and
exhaust the existing 30 s admission budget without a zero-violation witness.
They apply no control. The last outcomes concern expired work/native budgets;
this rerun does not prove global infeasibility or inevitable collision.

Actual NRMM executes 35 target-free holds, then fails at its first target
publication at 3.5 s after seven admission attempts. The old target-free
witness does not authorize an encounter command. The failed frame spends
19.807 ms in the observer/adapter and 3195.342 ms in control. All 36 sampled
ego enclosure/premise audits pass; available target-component checks pass at
publication. This is not evidence of observer divergence, nor a complete
target-motion proof. Pre-failure separation/road margins of 24.935294 and
3.788048 m cover only the executed target-free prefix.

## Configuration, execution, and reproducibility

MATLAB R2026a Update 3, eight computational threads. All campaign trials use
seed 20260914 and 0.1 s holds. Tests finish before the campaign begins; all
trials run sequentially in the same session. Code paths are already exercised
by tests. Timings are single-workstation measurements without a worst-case
or cold-start guarantee. No additional concurrent test job is launched by
this task; unrelated workstation load is not controlled.

The first six trials retain the exact-state driver's configuration: initial
16-stage performance horizon, 16 m confirmation region, reference 8 m/s,
road boundaries at +/-5 m, and 0.25 m clearance. Targets start at `[15;0]`
stationary, `[60;0]` moving `[-8;0]` m/s, or `[15;-4]` moving `[0;32]` m/s.
That driver continues publishing target observations outside the region;
version 22 retains an approaching exterior target until its confirmed exit.

Bounded cases use ego measurement radii
`[.05;.05;.005;.05;.02;.005]` in `[px,py,psi,vx,vy,r]`, target radii
`[.1;.1;.1;.1;.05;.05;.01;.01]` in `[px,py,vx,vy,ax,ay,psi,omega]`, target jerk
amplitudes `[.1;.1]` m/s^3, yaw-acceleration amplitude .05 rad/s^2 and angular
frequency 1 rad/s. They use synthetic bounded measurements, not NRMM output.

The range-gated pair uses ego reference 10 m/s, target initial position
`[100;.8]` m and velocity `[-10;0]` m/s, physical 30 m visibility, and its
existing 3 s admission budget. Actual NRMM runs at 80 Hz with maximum
integration step .0125 s and the existing sensor noise/prior configuration.
Exact mode bypasses the observer and supplies zero-uncertainty true states.

All plant holds independently integrate the issued affine generator using
`expm`. Safety geometry is also checked at eleven points per hold. Frame time
includes input assembly, control, and actual NRMM/adapter work when enabled;
it excludes offline synthesis, plant integration, geometry audits and result
serialization. Overruns are measured but not applied as actuation delay.
The initial search budget and native work limits remain distinct from a hard
100 ms end-to-end deadline. No physical nonlinear-vehicle validation is
claimed by these declared-affine-plant experiments.

Run from the repository root:

```matlab
addpath('scripts');
out = '/home/zai/.cache/collisionAvoidance/controller-v22-rerun-20260914';
campaign = runStraightControllerRerun( ...
    SampleCount=300,Seed=20260914,OutputDirectory=fullfile(out,'campaign'));
longRange = runDeclaredPlantEstimatorControllerScenario( ...
    SampleCount=900,Seed=20260914,UseEstimator=false, ...
    OutputDirectory=fullfile(out,'range-exact-90s'));
```

All **59/59** selected cases pass in `controllerRepairTest` (14),
`finiteEncounterCompletionTest` (20), `visibleTargetLifecycleTest` (8),
`boundedTargetMotionTest` (10), `targetObservationConditioningTest` (4), and
`pipelineDeadlineTest` (3), through MATLAB MCP. This is a focused rerun, not
a new whole-repository test claim. Native ordinary/expired budget tests pass
against the existing compiled bridge. Its SHA-256 is
`30f2cca72ef8622733f6bfd32f6567be0dafede7a6dfe94d050ca33c23344e5a`.
Factory Code Analyzer reports zero findings in the new campaign driver.

Original per-trial MAT files, campaign MAT/CSV/JSON, regression CSV/MAT,
environment and native-binary records, `range-audits.json`, and
`late-command-diagnostics.json` remain under `out`. The visually checked
`range-exact-90s.png` plots the sustained speed/lateral/heading behavior and
frame times; its plotting/audit helpers are retained there. A display-only
MATLAB table initially had mismatched vector orientations and was corrected;
no simulation was rerun or result altered for that display correction.
