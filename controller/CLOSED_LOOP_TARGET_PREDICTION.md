# Closed-loop prediction of the target: reaction and acceleration limits

Status: implemented 2026-09-23. Companion to
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
2. **The target's own future maneuver** (`J t^3/6`). No estimator can measure it
   in advance; it is the target driver's freedom.

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

The target's absolute future position still spreads out. The reaction only
keeps that spread out of the collision and exit records.

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

A carried family is void if the declared maximum increases or disappears
(`hardEncounterBarrier.prepare`).

## 4. Target-reactive policy

### Policy

From the second hold on, the executed input is

    u_k = v_k + K_k (xhat_k - z_k) + L_k (shat_k - s0_k) + N_k (ahat_k - a0),

where:
- `shat` is the target position/velocity estimate and `ahat` its acceleration
  estimate;
- `s0, a0` are the nominal constant-acceleration flow of the admitted target
  (`finiteFlow` center);
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

    plan(:,1) + K_1 (xhat - z_1) + L_1 (shat - s0_1) + N_1 (ahat - a0)

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

The physical reading: the ego tube is bounded, while the target's own maneuver
freedom still grows with prediction time. Keeping the relative tube bounded
would require a reserve comparable to the target's acceleration, which the
maneuvers and the terminal cruise requirement do not leave free.

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
