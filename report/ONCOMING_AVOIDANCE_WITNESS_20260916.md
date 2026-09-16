# Feasible avoidance exists at the failed oncoming admission state

## Finding and scope

An independently checked avoidance trajectory exists from the same state at
which the online controller rejects first admission at simulation time 2.6 s.
The strongest new result is a four-second held-input trajectory under the
repository's nonlinear Fiala vehicle equations. Validated continuous flow
enclosures and oriented-rectangle support bounds establish at least
**0.28812246 m body separation**, exceeding the required 0.25 m clearance.
The retained input, model-domain and tire-slip limits are satisfied.

Thus, collision is not inevitable under this experiment's nonlinear model.
The existing initial-certificate construction excludes useful maneuvers. The
previous infeasibility finding concerns that particular sufficient constraint
family, rather than all possible safe trajectories. It is not merely a solver
iteration failure: the original fixed affine family has an independent LP
infeasibility certificate.

This is an offline existence and failure-attribution experiment. Production
controller, estimator and configuration sources are unchanged. The online
admission failure and every-frame 100 ms requirement remain unresolved. The
result does not establish real-hardware feasibility, uncertain-estimator
admission, nonlinear closed-loop recursive feasibility, or asymptotic cruise
convergence. The four-second witness also does not satisfy the old 3.2-second
terminal schedule by construction; the previously isolated early collision
conflict already exists without terminal or CLF constraints.

Source revision before these diagnostic additions:
`447bf64ab609a8e95d324f3f90752b75fe7db9fc`.
See [the preceding admission diagnosis](ONCOMING_ADMISSION_DIAGNOSIS_20260916.md)
for the native failure replay, row ablations and dual certificate.

## Initial conditions and retained limits

The search starts from the saved failed frame, not an earlier detection state.

| Quantity | Value |
| --- | --- |
| Ego Cartesian position | (20.8009561083, approximately 0) m |
| Ego longitudinal speed | 8.0004089251 m/s |
| Target position and velocity | (39.2, 0) m; (-8, 0) m/s |
| Initial center separation | 18.3990438917 m |
| Target heading, acceleration, yaw rate | pi rad, zero, zero |
| Held-control period | 0.1 s |
| Vehicle footprints | 4.8 m by 1.9 m, both vehicles |
| Required body clearance | 0.25 m |
| Perception radius | 16 m, with whole-target-footprint exterior guard |
| Physical road boundaries | None |
| Lateral and heading domains | absolute lateral position <= 4 m; absolute heading <= 0.4 rad |
| Speed, lateral speed and yaw-rate domains | 0 <= vx <= 18 m/s; absolute vy <= 12 m/s; absolute r <= 5 rad/s |
| Physical front/rear tire-slip limits | 10 degrees, using atan2 slip angles |
| Steering and normalized longitudinal input | absolute steering <= 0.69813170 rad; beta in [-1, 1] |
| Actuator-rate bounds | Infinite in this original experiment |

The initial ego state and target motion are exact. The Fiala equations include
the configured combined-slip forces, air drag and rolling resistance. Their
parameters remain the repository's declared parameters, without a new hardware
calibration. Infinite actuator-rate bounds are a material limitation: this
experiment does not prove feasibility for any particular finite steering or
powertrain slew limit.

## Nonlinear search and independent verification

`scripts/findOncomingAvoidanceWitness.m` searches 40 held commands, with 80
input decision variables and a 20 ms nominal checking grid. Its search margins
are stricter than the actual limits: 0.30 m rectangle separation, 3.9 m lateral
domain, 0.395 rad heading, 0.17 rad slip, 0.68 rad steering and 0.98 input
magnitude. The existing native nonlinear rollout supplies local state/input
Jacobians, which are chained through the horizon. A final-state cruise-error
objective and small input/difference costs select among candidates; this
research objective does not modify the online controller's CLF or input cost.

The deterministic interior-point/L-BFGS search uses successive 150- and
1200-iteration budgets. The final run takes 47.536031 s and returns exit flag
0 (iteration budget), with maximum nonlinear constraint residual
-0.000340471157726. No optimality claim is made. Its output is a candidate,
accepted for this existence result only after the independent checks below.
Earlier SQP attempts stalled and were interrupted; they are not successful
experiments or proofs of infeasibility.

`scripts/verifyOncomingAvoidanceWitness.m` applies the saved commands to
`fialaCertificate.sequence`, with zero feedback gain, exact initial state,
0.1 s holds, maximum cell duration 0.0025 s and maximum 128 generators.
All 40 holds pass, producing 1,600 swept cells. MPFR with directed rounding
encloses the continuous nonlinear flow; the held-command memory also checks
the configured input and slew conditions.

For every swept state box, the verifier chooses a separating direction and
subtracts the complete position-box support, moving-target support, both
oriented-rectangle supports over the complete yaw interval, and the required
clearance. The target's exact linear motion is enclosed throughout the cell.
The same boxes check the state domains and both physical tire-slip angles,
including each held steering command. These are continuous-cell checks, not
only trajectory samples. Rectangle/support arithmetic uses double precision
with an explicit conservative arithmetic allowance; a complete formal proof
of every floating-point geometry operation is not claimed.

An additional RK4 replay at 1 ms and 0.5 ms checks numerical consistency and
sampled Euclidean rectangle distances. It supplements, rather than replaces,
the continuous enclosures.

| Check or measurement | Result |
| --- | --- |
| Continuous separation lower bound | 0.2881224602 m |
| Continuous separation beyond required clearance | 0.0381224602 m |
| Minimum sampled rectangle separation at 0.5 ms | 0.3004223927 m |
| Minimum continuous physical tire-slip reserve | 0.0049099393 rad |
| Maximum nominal absolute lateral position | 3.2904814525 m |
| Maximum nominal absolute heading | 0.3947442861 rad |
| Nominal speed range | 7.9999723144 to 12.3195241644 m/s |
| Steering range | -0.2232674955 to 0.3509064040 rad |
| Longitudinal input range | -0.3555604497 to 0.9733297934 |
| Final nominal lateral position / heading | 0.0000031292 m / -0.0001222860 rad |
| Final nominal speed | 7.9999723144 m/s |
| Final validated lateral interval | [-0.0079644137, 0.0075549840] m |
| Final validated speed interval | [7.9995088821, 8.0004197325] m/s |
| Final whole-target exterior reserve beyond the 16 m region | 36.7986088607 m |
| Maximum difference between 1 ms and 0.5 ms replays | 1.1446e-8, maximum across six state coordinates |
| Validated flow-enclosure runtime | 6.772739 s, excluding subsequent geometry/replay work |

The maneuver initially accelerates and steers around the oncoming vehicle,
then decelerates and recenters. Its peak speed is below the existing 18 m/s
domain limit, and by four seconds its state is close to 8 m/s path cruise.
This is finite-time recovery evidence, not a proof of asymptotic convergence.

As a negative control, 40 unchanged straight-cruise commands are passed through
the same verifier from the same initial state. All nonlinear flow holds are
enclosed, but collision verification fails: minimum continuous separation
residual is -2.1685014428 m and the sampled residual is -2.15 m. This confirms
that accepting a flow enclosure alone does not label a colliding plan safe.

## Why the current collision rows miss this maneuver

The old construction fixes a pure lateral separating normal at relative time
0.842857142857143 s, imposing

\[
 d-2.4e_\psi\ge 2.15041260706785\ \mathrm m.
\]

Its linear-program diagnostic proves this row incompatible with the retained
heading/slip limits within that fixed affine family, by a gap of
0.8307610962 m. The selected row has no braking-input coefficient: changing
longitudinal timing cannot relax that particular lateral-only requirement.

At the same time, the continuously verified nonlinear witness has

\[
 d-2.4e_\psi\le 1.13135428440745\ \mathrm m
\]

over the enclosing cell. It would fail the old row, while its actual oriented
rectangles remain separated throughout the maneuver. Useful separation
directions depend on both longitudinal progress and lateral/heading motion;
requiring a preset lateral separation by a preset time is unnecessarily
restrictive. Removing all safety checks is not the remedy. The evidence
supports constructing dynamically feasible maneuver proposals and matching
the cell normals and finite continuation timing to those proposals, followed
by the existing independent hard verification and successor construction.

## Separate witness for the declared affine plant

An auxiliary offline search also finds a different input sequence for the
same frozen declared affine generator. A finite collision-feasible prefix is
extended with a reference-recovery suffix to 49 holds (4.9 s), then checked
with `ltvBicycleModel.fixedPredict`, `avoidanceSafetyGeometry.build`, current
terminal membership and the propagated-target exterior guard.

The complete fixed-input Bernstein collision rows have minimum margin
0.3726446135 m, combined domain/slip rows have minimum margin
0.0008726646, terminal membership margin is 0.1011972644, and target exterior
membership is true. The geometry uses trajectory-consistent cell normals.
This witness uses an extended horizon rather than the rejected 32-hold
schedule. Its saved controls, model and verification data are retained in
`affine-certified.mat` in the experiment directory.

The affine and Fiala witnesses have different inputs. Replaying an earlier
affine candidate in the nonlinear model did not preserve its behavior;
therefore no transfer of affine safety to the nonlinear plant is assumed.
The independently verified nonlinear witness is the primary answer to the
model-level physical-feasibility question.

## Reproduction and retained artifacts

First obtain the preceding diagnosis artifact using the reproduction command
in the linked admission report. Then run from the repository root:

```matlab
addpath('scripts');
diagnosis = "/home/zai/.cache/collisionAvoidance/oncoming-admission-20260916/admission-diagnosis.mat";
output = "/home/zai/.cache/collisionAvoidance/oncoming-witness-20260916/final";
result = findOncomingAvoidanceWitness( ...
    DiagnosisFile=diagnosis, OutputDirectory=output);
summary = verifyOncomingAvoidanceWitness( ...
    WitnessFile=fullfile(output,"nonlinear-witness.mat"), ...
    DiagnosisFile=diagnosis, OutputDirectory=output);
assert(summary.safetyVerified);
```

The saved positive artifacts are `nonlinear-witness.mat`,
`witness-verification.mat`, `verification.json` and `code-checks.mat` under
`output`. The nonlinear verifier builds its MPFR native helpers in
`output/native`. The adjacent `negative-control` directory holds the
straight-cruise negative witness and its verification; the parent experiment
directory holds the auxiliary affine witness and exploratory diagnostics.
Generated binaries and numerical traces remain outside the source repository.

Validation completed: deterministic nonlinear search, independent continuous
positive verification, finer replay comparison, straight-cruise negative
control, auxiliary affine geometry/terminal verification, zero factory Code
Analyzer findings for both new scripts, and `git diff --check`. Scenario
preconditions explicitly restrict the scripts to the diagnosed head-on case.
The prior 593-test suite was not rerun: no production algorithm changed in
this diagnostic-only task. Neither the 47.5 s offline search nor the 6.8 s
flow verification establishes 100 ms online execution.
