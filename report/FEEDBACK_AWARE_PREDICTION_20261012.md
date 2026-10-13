# Feedback-aware prediction: fixed tubes and correlated target response

Date: October 12, 2026. Runtime baseline:
`e3babd341e141429546fd7061b6723158c5f3ed0`.
The theoretical scope omits road boundaries. This work implements a research
verifier and derives a causal-policy shift proof; it does not replace the
production controller or execute the NRMM observer.

## Design result

Finite-horizon boundedness alone does not meet the practical requirement:
an open-loop enclosure may be too large at the endpoint even while finite.
Use feedback-aware prediction, optimizing future rules that respond to the
observer information available at each future sample. A certified invariant
tube can keep the same width at prediction nodes. Noise creates a nonzero
width floor; strict shrinkage from an arbitrarily smaller width is not promised.

The derivation and policy-shift proof are in
`controller/CLOSED_LOOP_PREDICTION_THEORY.md`. Current inputs remain direct
optimization outputs by incorporating the feedback relation inside the
program. The existing open-loop shift proof is a different formulation.

## Reproducible probe

`scripts/verifyFeedbackAwarePrediction.m` uses default straight 15 m/s sampled
Fiala cruise, 0.05 s holds and 512 holds (25.6 s). Illustrative measurement
half-widths are [0.05 m; 0.05 m; 0.005 rad; 0.02 m/s; 0.02 m/s; 0.005 rad/s].
These are explicit probe inputs, not measured NRMM campaign bounds. A six-state
discrete LQR uses state weights [1,4,20,1,1,2] and input weights [20,1].
The lifted existing five-state path-CLF gain retains eigenvalue 1 because
longitudinal position is omitted. New full-tracking spectral radius is
0.9508682199, metric contraction 0.9965517145. These local tangent properties
do not certify arbitrary avoidance, braking or parked modes.

A constant 0.02 m/s velocity-estimation bias is admitted by the probe bound.
In the exact tangent model the existing gain accumulates position error while
the full gain reaches a fixed offset:

| Time (s) | Existing gain position error (m) | Full gain position error (m) |
| --- | --- | --- |
| 5 | -0.0903 | -0.0226 |
| 25.6 | -0.5008 | -0.0228 |
| 100 | -1.9834 | -0.0228 |

This identifies a missing full-tracking certificate, not inevitable divergence
of the current receding-horizon controller. Bounded observer error and an
available gain do not alone imply bounded deviation from a retained trajectory.

## Fixed linear tube

The verifier constructs an invariant set for every sequence of admitted
measurement errors in the tangent model. It sums 256 disturbance-image
zonotopes, adds a matrix-bound ellipsoid tail, and includes an invariant convex
hull of initial-box images. Algebra proves A_c*R+D*U subset R and initial-box
containment. Its coordinate supports are:

| Quantity | Fixed closed-loop half-width | Open-loop half-width at 5 s | Open-loop half-width at 25.6 s |
| --- | --- | --- | --- |
| Longitudinal position (m) | 0.1232 | 0.1482 | 0.5167 |
| Lateral position (m) | 0.1458 | 0.4552 | 2.1189 |

The independent coordinate box is a display, not itself asserted invariant.
Full-set supports give input-correction bounds 0.0815 rad and 0.1832 braking
ratio units. These are tangent-model results; outward floating-point enclosure
and nonlinear remainder certification are not claimed.

A linear stress sequence greedily selects one of 64 measurement-error corners
each hold and stays within the displayed supports; it is not a global
worst-case solver. Eight initial-corner worlds replay the feedback on the
nonlinear sampled vehicle with bounded sinusoidal measurement errors at every
future sample. All worlds share the same first estimate/input. Observed peak
longitudinal/lateral errors are 0.0521/0.0604 m; maximum steering is 0.0421 rad
and absolute braking ratio 0.1065. These replays support the mechanism but do
not prove universal nonlinear safety.

## Target physical uncertainty and joint response

Two true target speeds 14.9 and 15.1 m/s produce positions 5.12 m apart at
25.6 s, even if later measurements reveal their states. A future observer
bound about each estimate cannot become a small physical reachable set about
one frozen target trajectory.

The probe uses a target-relative following reference with desired separation
6 m. Worlds share the initial observation and first input. From the second
sample, the declared target position/speed error bounds are 0.02 m/0.1 m/s;
the injected speed-error sinusoid has amplitude 0.02 m/s, within the unchanged
0.1 m/s bound. No reduction of the declared speed bound is assumed. Ego
errors are also bounded. The same causal rule reacts to each world's
observations. At 25.6 s target and ego position diameters are each about
5.12 m, while relative-position diameter is about 2.28e-6 m. Peak relative
tracking error is 0.0151 m. The terminal diameter is a two-world observation,
not a worst-case bound over all admitted noise sequences. Combining the two
final marginal position ranges independently reports overlapping longitudinal
rectangles, whereas the actual coupled following worlds retain positive
sampled rectangle clearance. These are two-world diagnostics, not universal
nonlinear collision certificates.

This demonstrates correlation: independently combining ego/target marginal
unions can add artificial collision uncertainty. Use coupled observation
branches or certified relative coordinates. This following probe is not the
head-on avoidance scenario and does not certify all target motions or parking.

## Joint relative bound

The companion joint linear construction covers a continuous unknown constant
target speed interval [14.9,15.1] m/s and every admitted ego/target measurement
sequence. Linear trim feedforward cancels the unknown target speed in relative
dynamics. Applying the invariant-set construction to nine measurement-error
coordinates gives fixed relative longitudinal/lateral half-widths
0.2827/0.1861 m and heading half-width 0.03613 rad. Exact rectangle projection
then gives a longitudinal clearance lower bound 0.8830 m, above the required
0.2 m. This is a universal bound for the declared relative tangent model,
stronger than the two-world replay but still not a nonlinear certificate.

## Production work remaining


1. Derive future observer envelopes from the existing ISS comparison, retaining
   useful correlations and valid model/sensing contracts.
2. Certify incremental full-state/relative feedback over actual nominal
   trajectories, with nonlinear reserves and input margins. The transverse
   CLF cannot simply be relabeled as this certificate.
3. Propagate the coupled feedback model, preserving target-reference branches
   and common-pose/gyro correlation; avoid assumed future observer resets.
4. Certify holds/midpoints and braking/parked modes, and retain the shifted
   policy in each new convex restriction. A parked set may remain fixed even
   though positive-speed strict contraction no longer applies.

Reproduction:

```matlab
addpath('scripts');
verifyFeedbackAwarePrediction( ...
    'report/FEEDBACK_AWARE_PREDICTION_20261012/verification.json');
results = runtests('tests/feedbackAwarePredictionTest.m');
assertSuccess(results);
```

All six behavior tests pass. Code Analyzer reports only unavailable user
settings (defaults used). No collision campaign, complete NRMM integration,
nonlinear robust tube certificate or new production recursive guarantee is
claimed.
