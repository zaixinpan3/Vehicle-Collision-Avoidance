# Why the oncoming encounter fails at first admission

The simulation successfully executes 26 target-free cruise holds. At 2.6 s,
the first optimization containing this target's collision constraints is
infeasible. This is the 27th attempted control sample, not the first sample
of the simulation. No initial safe witness for this encounter is obtained.

The recursive theorem starts from a feasible witness and preserves feasibility
under its stated obligations and contracts. A new target adds obligations;
the old target-free witness does not establish membership in the new predictive
safe domain. Consequently the theorem cannot start the safety chain for this
particular oncoming encounter. The September 15 completion claim concerns
conditional continuation after admission, not guaranteed admission of every
newly detected obstacle or an end-to-end guarantee for this failed scenario.

## Reproduction and state at failure

Source revision: `a4fe4db852ef3a461ef5c593c5ac3f335765784d`.
The diagnostic replays the exact controller and frozen affine plant with a
solver hook that records the failed program while preserving the native solve.
It reproduces 26 successful holds followed by native infeasible status 2
(normalized exit flag -2), at the same position and speed as the final campaign.
No controller or estimator algorithm is changed in this investigation.

```matlab
addpath('scripts');
summary = diagnoseOncomingAdmission( ...
    ReplayReport="/home/zai/.cache/collisionAvoidance/recursive-closure-20260915/final-campaign/oncoming/oncoming-exact-state.mat", ...
    OutputDirectory="/home/zai/.cache/collisionAvoidance/oncoming-admission-20260916");
```

The sample time is 0.1 s, reference speed 8 m/s, perception radius 16 m,
vehicle rectangles 4.8 by 1.9 m and clearance 0.25 m. There are no physical
road boundaries; the lateral/heading domains remain 4 m / 0.4 rad and the
tire-slip bounds are 10 degrees. Ego and target uncertainty are zero. Target
motion is exactly constant velocity, with zero jerk/yaw acceleration.

At admission, the ego speed is 8.000408925087978 m/s, the oncoming speed is
8 m/s, and center separation is 18.399043891663183 m. The current guard uses
the **entire target footprint**, so a target center can still be outside the
16 m circle when target-specific constraints first become active. Before
2.6 s the observed target is certified entirely outside and omitted from
active collision rows; the scenario driver actually supplies its record
throughout, rather than simulating an unseen-track detection process.

At unchanged collinear cruise, the available time before exhausting the
5.05 m center separation required by the aligned bodies and clearance is
about 0.834 s. This is context, not a proof that avoidance is impossible.
The admitted optimization proposes 32 holds (3.2 s) of continuation.

## Isolation of the conflict

Remove the CLF and all terminal SOCs, and solve the remaining physical affine
inequalities as an LP in the 64 input coordinates. They are already infeasible.
The table below uses untightened physical bounds, so additional numerical
tightening is not the cause.

| Retained affine program | Result |
| --- | --- |
| All physical affine rows | Infeasible |
| Remove collision rows | Feasible; maximum residual 4.85e-10 |
| Remove finite-exit rows | Infeasible |
| Remove exit and terminal-entry slew rows | Infeasible |
| Collision and actuator rows only | Feasible |
| Remove model-domain rows, retaining tire slip | Infeasible |
| Remove tire-slip rows, retaining model domain | Infeasible |

The baseline actuator-rate limits are infinite, so there are no finite
terminal-entry slew rows in this instance. All terminal membership cones are
absent from every LP above. These ablations never authorize control execution.
They show that neither CLF slack nor terminal membership is required for this
particular infeasibility. An independent LP reproduces the native solver's
hard-constraint conflict.

The first infeasible prefix ends at stage 9 (0.9 s into the prediction).
One collision row at the first Bernstein point of cell 60, starting at
0.842857142857143 s, uses the pure lateral separating normal `[0;1]`:

\[
  -d+2.4 e_\psi\le-2.15041260706785,
  \qquad d-2.4e_\psi\ge2.15041260706785\ \mathrm m.
\]

Under the other physical affine constraints, maximizing this particular
lateral/heading combination reaches only 1.31965151086721 m. The minimum
violation of this one collision row is therefore **0.830761096200636 m**.
Every coefficient of braking in its condensed input row is exactly zero.
Braking cannot relax this selected lateral-only inequality, even though it
can change the actual time of closest approach.

The active dual certificate uses ten rows: that collision row, a 0.4 rad
heading-domain row, and eight tire-slip rows. With nonnegative weights `y`,
the physical system `A U <= b` has

\[
 \min y=0,\quad \|A^Ty\|_\infty=4.42354486374086\times10^{-16},
 \quad b^Ty=-0.830761096200636.
\]

Charging the small stationarity residual over the bounded actuator box still
leaves a negative contradiction of the same magnitude. This is a numerical
linear infeasibility certificate with a large gap, not a formal exact-arithmetic
verification of all floating-point operations. The saved weights and rows allow
independent reproduction.

## Why the construction creates this conflict

`avoidanceSafetyGeometry.passingNormals` proposes separating directions using
a geometric lateral transition of about 3 m, completed after 0.8 s. That
proposal is not itself a dynamically verified vehicle trajectory. The normals
are then fixed while optimizing inputs. In this encounter they demand full
lateral separation at 0.843 s, faster than the retained heading/slip constraints
allow. A different braking trajectory does not move that preselected normal's
switching time. These constraints are sufficient inner approximations of
collision avoidance, so their infeasibility does not prove that every physical
avoidance trajectory is impossible.

This diagnosis locates the failure in **initial encounter-certificate
construction and its maneuver timing**. It does not establish an estimator
defect (this trial uses exact target states), a hard-CLF conflict (CLF is soft),
or a loss of a previously admitted active witness. A subsequent repair should
construct dynamically admissible passing proposals and allow separation
geometry to match longitudinal braking, while retaining hard verification and
the proved successor structure. Guaranteeing admission across perception
transitions additionally needs an explicit detection/safe-admission condition.
No such repair or new globally feasible controller is claimed here.

## Validation and artifacts

The final diagnostic completes its replay and eight LP ablations. Its
assertions verify the failure time/status, positive minimum collision-row
violation, nonnegative dual weights, bounded-input contradiction, and the
feasible/infeasible prefix search. Factory Code Analyzer reports zero findings
for the new script; `git diff --check` passes. No controller code changed, and
the 593-case suite from the prior implementation was not rerun for this
diagnostic-only addition.

`admission-diagnosis.mat`, `summary.json`, and `code-checks.mat` are retained in
the output directory above. An intermediate instrumentation revision retained
a nested callback in the controller's configuration cache and produced a
cleanup warning during source reload; the final static capture helper removes
that lifetime issue, and the final replay/check completes without the warning.
