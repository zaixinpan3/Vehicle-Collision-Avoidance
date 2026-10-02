# Recovery CLF for the no-risk phase

This document defines the control Lyapunov function used once no target is
within the encounter range: from that state on, the ego should dissipate to
the nominal behaviour, the given path at the reference speed. It states what
is proved, what is verified by sampling, and what remains outside the claim.

## Why the lane quadratic was not a CLF

The lane CLF `V = e'Pe` is the LQR quadratic of the hold-discretized
linearization at the cruise trim, with `e = [e_y, e_psi, e_v, v_y - v_y*, r - r*]`.
A CLF needs an admissible input that decreases `V` at every state of the region
of interest. For the Fiala bicycle that fails away from a small neighbourhood
of the trim, for three reasons:

- the approach speed to the path is bounded by the vehicle speed, while `V`
  grows quadratically;
- steering reaches `e_y` only through `r -> psi -> e_y`, and friction bounds
  the yaw acceleration;
- the quadratic's preferred heading `-(P12/P22) e_y` passes 90 degrees at
  about 16 m.

On a steady-state slice, the 1% one-step condition already fails at `V = 44`
(8 m/s) and `V = 27` (15 m/s). See `report/CLF_NO_RISK_DISSIPATION_20261001.tex`.

## Construction: the cost-to-go of a recovery feedback

A CLF can be built from any feedback that brings every state of the region to
the nominal behaviour. Let `kappa` be such a feedback, `F(x) = f(x, kappa(x))`
the closed loop of the controller's own hold model (`nonlinearBicycleModel.sample`),
and `l(e) = e'Qe` with `Q = diag(1/scales^2)` from `cfg.clf`. Then

    V(x) = sum_{k=0}^{K(x)-1} l(e(x_k)) + V_f(e(x_K(x))),   x_0 = x, x_{k+1} = F(x_k),

where `K(x)` is the first step with `V_f(e(x_k)) <= stopValue`. `V_f(e) = e'P_f e`
is the exact cost-to-go of the linearized closed loop at the trim:
`A_cl' P_f A_cl - P_f + Q = 0`.

**Decrease.** `x_1 = F(x)` follows the same closed loop one hold later and stops
at the same absolute step, so

    V(F(x)) = V(x) - l(e(x))

whenever `K(x) >= 1`. Inside the stop level, `V = V_f` decreases by `l` up to
higher-order terms of the linear loop. A decreasing admissible input therefore
exists at every state from which `kappa` converges, namely `kappa(x)` itself.
This is the CLF property, and the region is the region of convergence of
`kappa`. `V` is zero only at the trim and positive elsewhere.

`nonlinearBicycleModel.recoveryValue` evaluates `V` and returns the stacked
square roots of its terms, so that `V = |residual|^2`.
`nonlinearBicycleModel.recoveryTerminal` returns `A_cl`, `P_f` and the trim
offset.

## The recovery feedback

`nonlinearBicycleModel.recoveryInput(x, previous, lane, reference, cfg, terminal)`
has four loops, with parameters in `cfg.recovery`.

1. **Guidance.** The course relative to the path is
   `chi = wrap(psi + atan2(v_y, v_x) - psi_path)`. Its desired value is
   `chi_d = -atan(e_y / D)`, with `D = max(minimumLookaheadMeters,
   lookaheadSeconds * v_ref)`. With exact course tracking,
   `d/dt e_y = V sin(chi_d) = -V e_y / sqrt(e_y^2 + D^2)`, so `e_y^2` decreases
   at every offset. The approach angle never exceeds 90 degrees.
2. **Course loop.** The yaw-rate demand is
   `r_d = kappa_p V cos(chi)/(1 - kappa_p e_y) + d/dt chi_d - courseGain * wrap(chi - chi_d)`.
   It is limited to `lateralAccelerationFraction * mu g / V`. A reversed course
   (`|chi - chi_d|` near pi) turns at that limit.
3. **Yaw-rate loop.** The front lateral force is
   `(Iz * yawRateGain * (r_d - r) + l_r F_yr) / l_f`, with the rear force at its
   current slip. It is capped at `frontForceFraction` of the combined-slip
   capacity `mu F_zf sqrt(1 - b^2)`. The front slip angle comes from the exact
   Fiala inverse, `|F|/F_max = 1 - (1 - s)^3` with `tan|alpha| = 3 F_max s / C`,
   so the tire stays below its force peak.
   `delta = atan2(v_y + l_f r, v_x) - alpha_f + steeringOffset`.
4. **Speed loop.** `b = b* + speedGain (v_x* - v_x)`, limited to
   `+/-brakingRatioLimit`, the braking-ratio rate and the actuation bounds.

The inverse neglects the front longitudinal force and `cos(delta)`. The
constant `steeringOffset`, computed once per configuration and curvature, makes
`kappa(trim) = u*` exactly. On straight paths the offset is zero; at the 200-m
fixture curvature it is about -2e-5 rad. The trim is therefore an exact
equilibrium, `V(trim) = 0`, and `A_cl` is the Jacobian there.

## Use in the controller

When the measured state is beyond the encounter range, or there is no target,
the solve has no collision rows. Then:

- **Anchor.** The linearization anchor is `recoveryFeedbackRollout`: `kappa`
  for the primary horizon, then the terminal completion law of the flow
  initialization. Its first input is `kappa(x0)`. The shifted previous plan is
  not used. This removes the trust-region trap documented in the report.
- **CLF stage.** It requires
  `V(x1) <= V(x0) - decreaseFraction * l(e(x0)) + rho`.
  - `V(x0)` comes from one rollout.
  - Around the anchor's first node, `V` is modelled as `c + |a + R dx1|^2`
    (Gauss-Newton). The residual Jacobian comes from six rollouts of the same
    length started at perturbed states.
  - At zero correction the model equals `V(x1) = V(x0) - l`, so `rho = 0` is
    feasible with margin `(1 - decreaseFraction) l`.
- **Third stage.** Among both achieved slack levels, it minimizes
  `|W (u - u_anchor)|`, with weight `firstInputWeight` on the first input. The
  issued input is therefore `kappa(x0)` whenever the remaining constraints admit
  it, and the model decrease is then exact, `V(x1) = V(x0) - l(e(x0))`. The
  Gauss-Newton model alone is not used to certify other inputs: its error can
  exceed the margin for corrections across the whole trust box.

While a target is within range, the controller is unchanged: shifted-plan
anchor, PCBF stage, lane-quadratic CLF stage, and no third stage. Safety keeps
priority, and no dissipation is claimed during an encounter.

## What is claimed

- **Proved in the controller's model.** The cost-to-go identity, and hence
  `V(x1) = V(x0) - l(e(x0)) < V(x0)` for every target-free frame that issues
  `kappa(x0)` from a state where `kappa` converges. Consequently `V` decreases to
  zero along any sequence of such frames.
- **Verified by sampling, not proved.**
  - Convergence of `kappa` from the region: `scripts/certifyRecoveryClf.m` runs
    it from a grid of offsets up to 150 m, all headings, speed factors 0.5 to
    1.15, lateral velocities to 1.5 m/s and yaw rates to 0.5 rad/s, at 8 and
    15 m/s, on straight and 200-m-radius paths.
  - The cost-to-go identity, to rounding error, on a subset of those starts.
- **Not claimed.**
  - Behaviour of the ODE45 plant beyond the hold model; it is measured in the
    campaign.
  - Frames whose remaining constraints move the first input off `kappa(x0)`.
  - Behaviour while a target is within range.
  - States outside the sampled region, such as speeds below about 4 m/s or
    curvature charts beyond `1 - kappa_p e_y = 0.1`.
