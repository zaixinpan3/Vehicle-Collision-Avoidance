# Closed-loop prediction of the target: reaction and acceleration limits

Status: implemented 2026-09-23; the NRMM-consistent target prediction of
Section 7 implemented 2026-09-24. Companion to
[FEEDBACK_TUBE_PREDICTION.md](FEEDBACK_TUBE_PREDICTION.md), which closes the
loop for the ego. This document covers the target part of the prediction.
Configuration: `cfg.feedbackPrediction.targetReaction`; the acceleration cap
is active whenever the target contract declares `scalarAccelerationMaximum`.

## 1. Why the target part of the prediction grows

Per axis the target's reachable box (`targetPrediction.finiteFlow`) has the
half-width

    r_p + r_v t + r_a t^2/2 + J t^3/6,

where `r_p, r_v, r_a` are the current estimate's position, velocity and
acceleration errors and `J` the declared jerk bound. Two different things grow
here:

1. **Today's estimate errors, extrapolated** (`r_v t`, `r_a t^2/2`). The
   estimator re-anchors the target estimate at every hold, so in execution this
   error never accumulates.
2. **The jerk term** (`J t^3/6`). The controller's target model is NRMM:
   constant scalar acceleration and constant sideslip, so the target's path is
   fixed by its current state. The NRMM estimator publishes
   `J = hypot(vmax wmax^2, 3 amax wmax) + modelJerkMaximum`: the largest
   Cartesian jerk of an NRMM turn, plus a model-error allowance that is 0 by
   default (`nrmmTrackingConfig`: zero acceleration-rate and curvature-rate
   maxima). `finiteFlow` extrapolates a constant Cartesian acceleration, so it
   treats the deterministic turning of the estimated NRMM path as unknown jerk.
   Under the NRMM model this term is an artifact of the Cartesian enclosure,
   plus the declared allowance; it is not a free maneuver of the target. In the
   declared-plant harness, `J` is instead the amplitude of a truth jerk
   `J cos t`, which deliberately violates constant acceleration.

   Correction (2026-09-23): an earlier version of this item called the term
   "the target's own future maneuver ... the target driver's freedom". That is
   wrong for the controller's NRMM target model; see Section 7.

A frame that inherits a certificate reuses the admission frame's collision and
exit records for the rest of the encounter. So every record must hold for
every target motion the model admits from the admission time on, including
the exit record at the final node, where the box is largest.

A jerk-only model also lets the target's acceleration deviation grow as
`r_a + J t` without limit. With `J = 2 m/s^3` over 4.8 s the box reaches about
37 m per axis. A vehicle cannot do that, and no bounded-authority reaction can
follow it.

## 2. What feedback can and cannot bound

- **Ego.** The ego is controlled. Planning `u = v + K (xhat - z)` keeps its
  deviation set bounded by the per-hold estimator error (FEEDBACK_TUBE_PREDICTION.md).
- **Target.** The ego cannot push the target back. It can react to target
  measurements, and then the *relative* deviation along each separation
  direction can stay bounded. This requires both:
  1. a bounded target acceleration, so that the target's maneuver per unit time
     is bounded; and
  2. enough ego authority to match that acceleration along the separation
     direction.

  Without (1), no reaction with bounded authority keeps the relative deviation
  bounded. Without (2), the reaction's reserve crowds out the nominal maneuver.

Under the NRMM model, the spread of the target's absolute future position is
the extrapolation of its current estimate error, which each new estimate
resolves. The reaction keeps that spread out of the collision and exit records.

## 3. Declared acceleration maximum (reachability)

If the contract declares `|a| <= amax`, then for every axis

    |a_i(t) - a0_i| <= b_i(t) = min(r_a + J t, amax + |a0_i|),

because the true acceleration starts within `r_a` of the nominal `a0`, changes at
most at rate `J`, and never leaves the `amax` disk. The velocity and position
half-widths integrate `b` (`r_v + int b`, `r_p + r_v t + int int b`). For long
horizons they grow as `t^2` instead of `t^3`. `finiteFlow`,
`condition`/`advance` and every record use these radii
(`targetPrediction.accelerationDeviationBound`, `localCappedDeviation`).

The NRMM estimator already publishes this bound (`nrmmControllerErrorBounds`:
`scalarAccelerationMaximum = observer.target.domain.scalarAccelerationMaximum`,
2 m/s^2 by default). Before this change the controller used it only to clip the
nominal flow.

Correction (2026-09-24): the value the estimator published is its speed-rate
maximum `|A| <= 2 m/s^2`, not a bound on `|a|`. For a turning target
`|a| = hypot(A, V omega)` reaches `hypot(2, 1.25) = 2.36 m/s^2` in the
estimator-in-the-loop domain and `hypot(2, 3.75) = 4.25 m/s^2` in the
`nrmmTrackingConfig` domain, so the published cap was not a valid bound
there. `nrmmControllerErrorBounds` now publishes `domain.accelerationNormBound`.
The declared-plant campaigns of Section 5 declared their own maxima, with truths
inside them, and are unaffected.

A carried family is void if the declared maximum increases or disappears
(`hardEncounterBarrier.prepare`).

## 4. Target-reactive policy

### Policy

From the second hold on, the executed input is

    u_k = v_k + K_k (xhat_k - z_k) + L_k (shat_k - s0_k) + N_k (ahat_k - a0_k),

where:
- `shat` is the target position/velocity estimate and `ahat` its acceleration
  estimate;
- `s0, a0` are the nominal flow of the admitted target
  (`targetPrediction.deviationModel` center): constant acceleration under the
  Cartesian contract, the NRMM path of the estimate under `nrmm-motion-v1`;
- every estimate errs by at most its published bound at every hold. The target
  bound at admission is the contract (`prediction.targetEstimatorBound`).

The first held input is exact.

### Joint deviation dynamics

Let `e` be the ego deviation (Frenet) and `d` the target position/velocity
deviation. Over hold k:

    d+ = F d + G abar_k + r_k
    e+ = (A + B K) e + B L d + B N (abar_k + delta_k)
         + B K eta + B L zeta + B N zeta_a

- `abar_k` is the mean target acceleration deviation of the hold, bounded by
  `b(t_k)`.
- `r_k` is its zero-mean position remainder, at most `min(b, J h) h^2/4`.
- `delta_k` (at most `J h/2`) is the difference between the acceleration at the
  start of the hold, which the estimator measures, and `abar_k`.

Holds are treated independently: a sound over-approximation of every
jerk-limited, acceleration-bounded motion.

### Tube

`ltvBicycleModel.reactiveTube` carries `[e; d]` as one zonotope in one
append-only basis. `abar_k` is a single generator shared by the target's motion
and the ego's feedforward, so their effects cancel exactly. Each collision and
exit record uses the generators `P e - d_p` (`avoidanceSafetyGeometry`,
`localTargetPart`) instead of an independent target box. Actuator amplitude
and slew rows, the terminal cone, the terminal slew row and the road rows use
the ego part, which includes the reaction.

### Gains

`ltvBicycleModel.reactionGains` designs the gains by a finite-horizon LQ
recursion on `[e; d]`, with the base ego gain `K` in the loop. The design uses:
- a squared relative position along each record's fixed direction, weighted by
  its relevance: full weight within `relevanceMeters` of binding at the
  optimizer center, decaying beyond;
- the exit record, scaled by `exitWeight`;
- an optional terminal ego cost (`terminalWeight` times the cruise CLF matrix);
- the CLF input weight scaled by the reaction strength;
- a measured-disturbance feedforward `N` for `abar`.

The target gains are shrunk where the per-hold estimator error dominates the
target deviation (`signal / (signal + noise)`). The lateral-velocity column is
zero unless that feedback is enabled. Any gains give a valid tube; the design
only selects them.

### Admission

A fresh frame with a target tries `inputWeightScales` in order and accepts the
first solver success. `Inf` is the ego-only feedback tube. The default
`[Inf, 30, 100]` keeps the ego-only certificate whenever it is feasible and
tries the reactive policy only when it is not. The directions come from the
ego-only program (nominal or fluid initialization); the reactive program is
rebuilt for those directions. Inherited frames never switch.

### Recursion

An inherited frame issues

    plan(:,1) + K_1 (xhat - z_1) + L_1 (shat - s0_1) + N_1 (ahat - a0_1)

about the carried nominals (`formulateAvoidanceProblem`). Here `shat, ahat`
are the conditioned target estimate, whose radius is at most the measured
bound. The stored state (format 44) carries the gain sequences, the target
nominal and the contract bound.

While the carried policy still reacts, the family is inherited only if the
same target is measured within its declared bound. Otherwise
`hardEncounterBarrier.prepare` requires a new certificate: this covers a
missing target, a different target, an observed exterior (release), a larger
bound, or a changed motion contract.

## 5. Evidence

[TARGET_REACTION_VALIDATION_20260923.md](../report/TARGET_REACTION_VALIDATION_20260923.md)
reports the measured outcomes on the declared plant. Main results:

- **Soundness.** Sampled reactive trajectories stay inside the joint tube, the
  records and the reserved input and slew supports (`targetReactionTest`).
- **Maneuvering targets.** With target jerk 1–2 m/s^3 and the recorded estimator
  ego bound, the jerk-only model passes 7 of 12 cases. The declared acceleration
  maximum brings this to 11 of 12. The reactive policy changed no outcome.
- **Growth of the late record supports.** At 4.8 s they fall from 22–47 m
  (jerk-only) to 18–32 m (capped). The reactive policy at strength 30 trims
  another 1–2 m, at an extra 0.04–0.13 of braking reserve.
- **Existing validations.** With the default order, the sweep, the
  estimator-bound campaign and the recursion campaign are unchanged. A
  reaction-first sweep gives the same outcomes, with node gaps within 0.051 m.

Reading: the ego tube is bounded. The campaign's truth targets deliberately
violate the NRMM model (jerk `J cos t`), and the declared `J` covers that
mismatch. Following such a target would need a reserve comparable to its
acceleration, which the maneuvers and the terminal cruise requirement do not
leave free. For a target that does follow the NRMM model, most of the
certificate's target growth is the Cartesian enclosure of its deterministic
turning (Section 7).

## 6. Limits

- **Authority.** The reaction reserves steering and braking for the correction.
  A maneuver that already needs full authority cannot spare it; example:
  ten-fold case 54, whose admissible plans use full lock and full braking.
- **Terminal and exit structure.** The certificate ends with a target-independent
  cruise set and an exit record that must confirm the target outside the
  confirmation range at the final node, for every admissible target motion.
  Following the target's velocity changes conflicts with returning to the
  reference speed, so the reaction can shrink the late records only partially.
  A target-aware terminal set would remove this structural source of growth;
  it is not implemented.
- **Worst-case sensing.** Estimator errors are independent per hold in the
  worst case. Reaction gains therefore also amplify sign-switching measurement
  errors, which limits how strongly the policy can react.
- **Target yaw** is still propagated open loop; its footprint support saturates
  at the circumradius.
- **Curved charts** are built from the reactive ego radius; the chart remainder
  is not enforced (unchanged).

## 7. NRMM-consistent target prediction (implemented 2026-09-24)

Section 1 identified the `J t^3/6` term as the Cartesian enclosure of the
deterministic turning of an NRMM target. The `nrmm-motion-v1` contract
removes it; [TARGET_PREDICTION_CONTRACT.md](TARGET_PREDICTION_CONTRACT.md)
states it in full.

**Contract.** Exact NRMM motion: constant speed-rate and constant sideslip, so
a path of constant curvature `|kappa| <= curvatureMaximum` through the current
state, stopping and holding at zero speed. There is no jerk or yaw-acceleration
allowance. A varying speed-rate or curvature leaves every NRMM path through a
later estimate, so a model error is declared with the Cartesian contract
instead. `nrmmControllerErrorBounds` publishes `nrmm-motion-v1` when
`modelJerkMaximum = 0` (the default), with `curvatureMaximum = sin(beta_max)/l_r`
and the acceleration magnitude bound, and the Cartesian contract otherwise.

**Parameter intervals** (`targetPrediction.nrmmParameters`). The encounter
carries intervals for `V`, course, `A` and `kappa`: the intervals of every NRMM
state in the estimate box, intersected with the bounds the estimator publishes
about the same estimate (2026-09-24, `nrmmTargetParameterErrorBounds`: the
frame-free speed, speed-rate and normal-acceleration balls of the tracker, the
course adding the ego yaw error) and with the declared maxima. The inertial box
carries the ego rotation error into every axis and its corners into every
norm; the published bounds do not. Inherited frames propagate the intervals
over the hold and intersect them with the measurement's.

**Reachable box** (`targetPrediction.finiteFlow`), for every NRMM path of the
intervals through the position box, as the intersection of two bounds on the
same paths:
1. **Parameter-Taylor bound.** Analytic sensitivities with a Lagrange
   second-order remainder, or a path-length ball where the linearization does
   not apply. Tight for fast and turning targets and long horizons.
2. **Time-Taylor bound.** The constant-acceleration extrapolation of the box
   plus the integrated jerk `hypot(kappa^2 V^3, 3 A kappa V)` and yaw
   acceleration `|A kappa|` of the paths over the intervals, plus the jump of
   the acceleration to zero wherever a path may have stopped. Tight for slow
   targets and short horizons.

All of the remaining growth is the extrapolation of the current estimate error
along the NRMM path.

**Reactive tube** (`targetPrediction.deviationModel`). The target deviation is
`S(t) theta + rho(t)`:
- `S(t) theta`: the parameter errors `theta` are generators shared by the
  target and the ego's reaction.
- `rho(t)`: the remainder is an interval part of the target at each record
  node. The reaction to it enters as new generators at every hold, so that
  `|A + B K|` does not wrap it.

The tube does not use the Cartesian intersection. The Cartesian contract keeps
its model: initial position and velocity generators plus a per-hold
acceleration part.

**Scope.**
- Certification is at hold nodes only. Whole-hold cells extrapolate a node
  snapshot with the Cartesian jerk bound and are rejected for NRMM targets.
- A change of the motion kind, or a larger curvature or speed-rate maximum,
  voids a carried family.

**Measured effect.** See
[NRMM_TARGET_PREDICTION_20260924.md](../report/NRMM_TARGET_PREDICTION_20260924.md)
and, for the published parameter bounds,
[NRMM_PARAMETER_INTERFACE_20260924.md](../report/NRMM_PARAMETER_INTERFACE_20260924.md).
- **Target box.** The target of the table above, at 4.8 s (position half-width
  per axis):
  - sideslip maximum 0.005 rad: 2.85 m, against 8.89 m under the Cartesian
    contract;
  - sideslip maximum 0.015 rad: 3.02 m, against 26.28 m.

  Both are below the 4.7 m family envelope.
- **What the reaction buys.** The reactive policy does not realize the
  expectation stated here before implementation, that following the NRMM
  deviation needs little authority and leaves only the per-hold estimate error.
  Three structural reasons:
  1. The estimator errors are worst-case and independent per hold, so the
     reaction also answers sign-switching errors.
  2. The remainder cannot be followed.
  3. Keeping the relative deviation small moves the target's spread into the
     ego's absolute position, where the lane, actuator and terminal cruise
     constraints bind.

  The report gives the measured supports and outcomes.
