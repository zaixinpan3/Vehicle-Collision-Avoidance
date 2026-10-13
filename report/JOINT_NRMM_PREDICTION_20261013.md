# Actual NRMM feedback and prediction-tube feasibility

## Material Passport

Date: October 13, 2026. Mode: implemented research probes and executed
simulation/validation. Inputs: the current controller vehicle, integration
configuration, synthesized NRMM gains, actual runtime update and published
current enclosures. Outputs: two reusable scripts, behavior tests, and the
JSON results in `JOINT_NRMM_PREDICTION_20261013/`. No participant data or
external model service was used. Production control and the manuscript were
not changed. This is local numerical evidence and a linear invariant-set
construction, not a nonlinear recursive-feasibility theorem.

## Goal clarification and finite-horizon propagation

Later on October 13 the user clarified that a fixed-width nonlinear invariant
tube is not the objective. Future observer updates and feedback must enter the
prediction so that the tail does not acquire the artificial growth of a
frozen-input open-loop rollout. Tube widths may vary. The earlier fixed-width
experiment below is retained as an optional diagnostic and historical result.

`verifyJointNrmmPrediction` now primarily propagates a varying finite-horizon
joint tube. `ComputeInvariantTube` defaults to false; the finite recursion
does not call a Lyapunov equation or require a contraction/invariance test.
For the constructed linear surrogate, propagate the actual known software
offset in the center, retain current physical uncertainty only in the root
generator, use the common fixed current input in the first map, and include
future observer/feedback noise in later maps. The finite center and generator
maps are

    m_next = J_i*m+c_i,       Z_next = [J_i*Z,D_i].

Coordinate/input supports are computed after the full matrix products,
preserving each shared noise column. The efficient constant-cruise recursion
keeps separate propagated initial/root-noise images and accumulated future
noise-image supports, rather than multiplying a coordinate box by abs(J).
The initial uncertainty is not reset at each predicted hold, and future
measurements are not assumed known or noiseless. Each terminal coordinate
support is independently attained by an explicit adjoint-selected noise
history in the conservative linear sensor-box model. Those box corners need
not belong to the smaller Euclidean sensor balls; the boxes enclose the actual
noise contract and their attainment is a check of the linear calculation.

Retained new data: `JOINT_NRMM_PREDICTION_20261013/finite-horizon.json`.
At 15 m/s, the physical position half widths evolve as follows (meters):

| Prediction time | Joint closed-loop, longitudinal/lateral | Frozen-input initial-error propagation, longitudinal/lateral |
| --- | --- | --- |
| 0 s | 0.0400 / 0.0400 | 0.0400 / 0.0400 |
| 1 s | 0.0893 / 0.0827 | 0.0899 / 0.0961 |
| 3 s | 0.1292 / 0.0913 | 0.1887 / 0.2101 |
| 5 s | 0.1353 / 0.0914 | 0.2860 / 0.3240 |
| 25.6 s | 0.1363 / 0.0914 | 1.2093 / 1.4977 |

The width grows from the current enclosure as future noisy observations
affect control, then approaches its operating noise envelope. It is neither
fixed nor artificially capped. The 8 m/s endpoint widths are 0.1363 / 0.0965 m.
The initial-error contribution at the endpoint is approximately 1.6e-12 m
longitudinally and less than 4e-16 m laterally; the future noise contribution
dominates. Maximum absolute braking ratios over these finite tubes are
0.0612 and 0.0659, respectively. This establishes the finite calculation for
the local linear maps; it is not a full nonlinear safety/feasibility certificate.

The new 512-step data run executes the actual updater to build the local maps
but sets `Replay=false`; the earlier nonlinear 512-step replays remain in the
original JSON. Updated tests run new 80-step nonlinear replays with the actual
NRMM and compare them with the finite-horizon supports. They retain the
target-relative/common-current-input checks and verify terminal linear
support attainment. The nonlinear work now required is a sound finite-horizon
outer image of the closed-loop joint map and its terminal/shift connection,
not proof of a constant-radius nonlinear invariant tube.

## Earlier fixed-width diagnostic

Future observation and feedback prevent the secular ego-error growth in the
tested straight-cruise cases. For the local joint linear surrogate, an
invariant set with fixed physical position half widths of approximately
0.244 m longitudinally and 0.20 m laterally can be constructed using the
actual current NRMM enclosure and the integration configuration's sensor
noise bounds. Its width does not depend on prediction duration. The nonlinear
vehicle with the actual NRMM updater completes 512 holds, or 25.6 s, with
sampled position errors below 0.081 m in these tests.

This does not mean that an arbitrarily small current error remains that
small. Persistent observation noise creates a nonzero operating error floor.
The fixed invariant tube reserves that floor from the start. The experiment
does not cap, clip or truncate a growing radius recursion.

## Actual updater and joint model

`verifyJointNrmmPrediction` calls `onlineNrmmTrackingRuntime` at 80 Hz with
four sensor samples per 50 ms control hold. The runtime's predictor resets,
RK4 point update, force-balance velocity measurement, yaw correspondence,
history, domain audits and bound updates are executed. The plant uses the
project's sampled nonlinear Fiala model above its blending range, with a
12.5 ms vehicle step at each sensor interval. Equivalence to a single 50 ms
numerical plant step is not assumed. Synthetic
GNSS position/velocity, body IMU acceleration, gyro and radar measurements
come from each physical branch; no true state is supplied to the controller.
The yaw-rate channel is the permitted noisy gyro, not exact yaw rate.

The research policy is

    u = u_nominal + K*(publishedEgoState - nominalEgoState).

The six-state LQR gain includes longitudinal position tracking. This is a
research policy; the production PCBF/CLF optimizer has not been replaced by
it. No input clipping or modification after evaluating the policy is used.

The ego-only joint error state has 11 coordinates: six physical deviations
and five persistent observer-point deviations (position, yaw, body velocity).
The GNSS predictor resets every sensor sample. Target point states and radar
predictor are executed but do not drive this particular cruise policy;
their inclusion cannot change its ego point map. Bound/history memory is
still updated on every branch. It is not asserted to have an 11-state linear
certificate. The local point model retains 28 shared primitive noise columns:
seven ego sensor coordinates at each of four samples. For the linear tube,
coordinate boxes conservatively contain the declared vector noise balls.

The initial physical generator is read from the published NRMM certificate.
The software state is known at the current sample, rather than independently
uncertain. Its small deterministic nominal offset is retained. All initial
branches have the same current input. Their first-hold image under that fixed
input, with allowed sensor disturbances, is included before applying future
feedback images in the invariant-set construction.

For the constructed affine linear surrogate, write

    q_next = A_joint*q + D*xi + c,      ||xi||_infinity <= 1.

The same sensor column influences observer and control through the actual
update. It is not added again as an independent observer-error disturbance.
The nominal defect is enclosed as an additional symmetric noise column.
The finite disturbance-image sum, a certified algebraic geometric tail in
a discrete Lyapunov norm, and the convex hull of the initial generator's
orbit give an RPI set. Coordinate and input supports are its displayed outer
bounds. The coordinate box itself is not claimed to be invariant.

The local matrices are numerical secants of the actual sampled map, not
analytically or interval-certified Jacobians. The force-balance inverse
bisects to 1e-5 m/s; taking numerical differences below that scale produces
spurious slopes. The retained differences use 0.1 and 0.05 of the declared
scales, with relative Frobenius changes 0.000696 and 0.001121 at 8 and 15 m/s.
This checks numerical sensitivity; it does not bound every nonlinear or
quantization residual over the invariant set. Floating-point calculations
are not outward rounded.

## Results

All position values below are half widths or absolute errors in meters.
The open-loop column propagates only the current correlated initial
generator with the frozen cruise plant tangent; it adds no plant defect.

| Speed | Open-loop initial-error radius at 25.6 s, longitudinal/lateral | Fixed joint linear radius, longitudinal/lateral | Nonlinear closed-loop sampled peak, longitudinal/lateral |
| --- | --- | --- | --- |
| 8 m/s | 1.262 / 1.441 | 0.244 / 0.200 | 0.0365 / 0.0802 |
| 15 m/s | 1.209 / 1.498 | 0.244 / 0.195 | 0.0364 / 0.0749 |

Both joint matrices have spectral radius approximately 0.95188. Their
weighted norm contraction factors are 0.99895 and 0.99945, respectively;
the spectral radius is not a claim of coordinate-wise 4.8% contraction.
The linear input supports, including nominal input and fresh gyro
feedthrough, give maximum absolute braking ratios 0.0981 and 0.1027 and
steering angles 0.0361 and 0.0209 rad. This checks the braking input domain
for the constructed linear tube; it does not certify all nonlinear handling
constraints or a stopping continuation.

There are three deterministic sensor histories per speed: one persistent
admissible bias and two rotating bounded noise histories. A paired open-loop
replay uses the same initial speed offset of 0.02 m/s. Its longitudinal error
reaches 0.507 m at 8 m/s and 0.487 m at 15 m/s; feedback holds it below
0.037 m. The open-loop replay does not include steering disturbances, so
its lateral error stays at its initial 0.02 m. Feedback can introduce a
larger lateral noise floor while preventing subsequent secular growth.

Across 3,072 audited closed-loop controller samples the current ego
enclosures are always available and contain all six measured physical
estimation errors. Their largest position bounds are about 0.106 m. This
is validation of these histories, not exhaustive certification of the
sampled observer. No Monte Carlo confidence level is claimed.

Including the updater is not automatically a tighter approximation. With
the same current generator, the separate six-state model treating each
future observer error independently yields smaller fixed position support
bounds (approximately 0.139 / 0.11 m) in this particular construction.
That model assumes its current observer envelope remains valid forever;
the actual replay's published bounds vary. The initial first-hold image and
the chosen outer approximations also affect the comparison. Neither model
has a uniform nonlinear certificate. The useful result is the availability
of a practical fixed joint tube, not universal superiority of one enclosure.

## Relative target feedback with the actual tracker

`verifyNrmmRelativeFeedback` additionally uses the actual published NRMM
target position and speed to form an 8 m following reference. Two physical
targets have constant speeds 14.9 and 15.1 m/s, the same position at the
current sample, the same initial observer software state and the same
current input. The common 0.25 s initialization history is consistent with
both branches under the declared radar noise. Future sensor errors are
bounded rotating histories. The target's true velocity is not supplied to
the feedback policy.

After 25.6 s both target and ego longitudinal positions differ by 5.12 m
between branches. Their relative separation differs by only 1.62e-6 m at
the endpoint; the largest relative-position deviation over either branch
is 0.0268 m. The minimum sampled rectangle clearance is 3.182 m. Boxing
the ego and target positions independently instead would give a spurious
longitudinal gap of -1.921 m, by combining different measurement branches.
All 1,024 audited current target position/speed bounds are available and
contain the tested truths. No intersample collision certificate is inferred.

This is not removal of physical target uncertainty. Its absolute-position
spread still grows. The safety calculation should preserve correlation in
relative pose, or retain the joint branches, when future ego feedback responds
to the target. A fixed ego tube alone cannot bound an uncontrolled target's
absolute position forever.

The root target enclosure is much wider than the two replayed speeds:
speed [12.339,17.661] m/s, course radius 0.182 rad, acceleration [-1.1,1.1]
m/s^2 and curvature approximately [-0.03436,0.03436] 1/m. These fields were
not narrowed or overridden. The two-world result cannot be advertised as a
robust guarantee over this entire published set. No joint target RPI set is
constructed here.

## Design decision and remaining proof

Continue with a causal output-feedback tube formulation. Propagate the
joint actual update offline to design gains and sound enclosure coefficients;
online, compact sets/radii can suffice. Preserve known current software
state, shared sensor columns and ego/target relative correlation. Account for
persistent sensor noise in the finite joint recursion and avoid repeatedly
adding independent absolute state-error boxes. Current input must remain the explicit optimized decision;
future feedback belongs inside the optimized policy and its constraints.

The next substantive proof is sound nonlinear finite-horizon joint images over
the admitted nominal trajectories, sensor histories and physical/parameter
uncertainties, including quantization and hold/midpoint effects. These must
also admit feedback input/handling limits, turn/straight transitions, the
low-speed model and a robust parked terminal policy. A joint moving-target
certificate must cover the whole published parameter family and preserve
posterior consistency. Road boundaries remain omitted in this theoretical
scope. Initial robust feasibility is still a required premise. Solver
failure remains outside the mathematical existence theorem.
The observer's declared 5% vehicle-parameter uncertainty was retained; the
replayed physical plant uses nominal parameters. Extremal uncertain vehicle
parameters were not tested or included in the local linear model certificate.

The general observer/tracking-envelope construction and robust MPC terminal
conditions are consistent with Koehler, Mueller and Allgoewer (2021),
*Robust output feedback model predictive control using online estimation
bounds*, Sections III and IV, Proposition 2 and Theorem 4:
[primary source](https://arxiv.org/html/2105.03427v1). Their assumptions must
be verified for this model; the paper is not a certificate for these scripts.

## Execution and validation

The following commands generated the retained data:

```matlab
addpath('scripts');
verifyJointNrmmPrediction('report/JOINT_NRMM_PREDICTION_20261013/verification.json');
verifyNrmmRelativeFeedback('report/JOINT_NRMM_PREDICTION_20261013/relative-feedback.json');
```

Those original fixed-tube/replay artifacts were generated at commit
`7abaee4451ea3cbf557b0b5c4e40de9f61510d67`. With the revised script, fixed-width
diagnostics can be requested explicitly using `ComputeInvariantTube=true`.
The new varying-tube artifact is generated with

```matlab
addpath('scripts');
verifyJointNrmmPrediction('report/JOINT_NRMM_PREDICTION_20261013/finite-horizon.json',Replay=false);
```

The new behavior tests exercise the actual observer, bounded-noise feedback,
common current input, target-relative tracking and certificate scope. The
existing feedback-aware verifier is checked alongside it. No production
solver campaign or complete repository test suite is claimed for this
research-only change.

The original combined MATLAB run passed all 11 tests, with zero failures or incomplete
tests. Code Analyzer uses default settings because this installation lacks
`MLintDefaultSettings.txt`; after routine cleanup it reports only that
environmental settings message, with no source-code findings.

After the scope correction the combined MATLAB run passes all 12 tests, with
zero failures or incomplete tests. The updated probe and test class again
produce only the missing-settings Code Analyzer message. JSON consistency
checks confirm varying widths, no requested/computed invariant tube, linear
terminal support attainment, finite input admission, and endpoint position
supports below one fifth of the corresponding open-loop initial-error
supports for both speeds. The 512-step new data run did not repeat the original
nonlinear scenario replay or execute the production MPC solver.
