# Hold-node safety certificate

Adopted on 2026-09-17 by project decision. The online controller certifies
safety at the sampling nodes of the exact sampled affine plant, not at every
instant inside a held command. This note states exactly what the certificate
covers, what it no longer covers, how the implementation realizes it, and
which whole-hold enclosures remain in offline tooling.

The current scenario defaults use a **50 ms** prediction-node interval,
input hold and controller update period, all driven by `controller.sampleTime`.
The exact-state experiment exposes this as `SampleTime`; its 1.6 s performance
window becomes 32 nodes and a 30 s run becomes 600 holds. Terminal encounter
completion still determines its required physical duration. Strict periodic
experiments enforce a 50 ms frame deadline. See
[the period-change measurements](../report/CONTROL_PERIOD_50MS_20260919.md).
Changing the period requires a fresh controller state and rebuilt discrete
CLF/terminal certificates. It does not change the sampling-only safety scope.

## Statement

Let `h` be the sample period and `t_k = t_0 + k h`. The declared plant of the
optimized plan `u_1, ..., u_N` is the exact sampled transition

    x_k = A_k x_{k-1} + B_k u_k + c_k,      k = 1, ..., N,

where `[A_k, B_k, c_k]` are the first six rows of `expm(h*[A(t), B(t), c(t); 0])`
of the scheduled continuous generator of hold `k`. The true state at node `k`
lies in the box `x_k + [-rho_k, rho_k]`, where `rho_0` is the current
measurement box and `rho_k = |A_k| rho_{k-1} + d_k + a_k` with the held
process reserve `d_k` (zero for the declared zero-residual plant) and a
floating-point allowance `a_k`.

The certified predicate is: for every node `k = 1, ..., N` and every state in
its box,

1. the ego rectangle and the target's bounded reachable set at time `t_k` are
   strictly separated by the selected support half-space after uncertainty
   and numerical allowances (joint support certificates; no added physical
   clearance buffer),
2. the state lies in the local Frenet chart domain of that node (pose-domain
   rows) and, for scheduled references, within the reference phase band and the
   lateral regularity radius (phase rows),
3. the terminal node `N` lies in the modal terminal set and satisfies the
   finite-exit support certificate,

together with the actuator amplitude and slew rows on the held inputs and the
soft first-hold CLF cone. The current formulation fixes each certificate's
unit direction before optimizing the complete trajectory. Under unchanged contracts, the shifted plan
and its retained directions certify nodes `2, ..., N`; touching majorants
contain that complete witness in the next optimization. The active horizon
shrinks toward the fixed exit deadline. After confirmed release, terminal
invariance supplies target-free continuation. A solver-accepted plan is
issued without a post-solve recheck, so these predicates hold to the solver's
feasibility tolerance. The carried-witness check (`solveHardCbfClf.certify`)
evaluates physical rows, cones and the true nonlinear support residuals of the
shifted suffix before the next solve. See [JOINT_SUPPORT_CERTIFICATES.md](JOINT_SUPPORT_CERTIFICATES.md).

## What is not certified

Nothing is claimed about the state between two consecutive nodes. Within one
hold the ego moves about `v h` along its path (0.4 m at 8 m/s with a 50 ms hold), the relative
position to a moving target changes by up to the sum of the speeds times `h`,
and the heading changes by the yaw rate times `h`. Two rectangles that are
separated at both nodes can, in principle, touch between them. The numerical
reserve is not a bound on this inter-node motion. The controller reports
`metadata.wholeHoldCertificate = false` and
`metadata.certificateSampling = "holdNodes"`; the smooth-reference validation
records the node clearance and, separately, an 11-point inter-node sampled
clearance as a physical diagnostic.

Before this decision the certificate enclosed the whole hold: every collision,
domain and phase row was imposed on all Bernstein coefficients of a
polynomial enclosure of the exact flow with a 1e-11 truncation remainder. At
8 m/s that enclosure needed 25 to 26 coefficients per hold and about 15,000
rows for a 48-hold admission; the node certificate imposes the same row
families once per node. The whole-hold statements in
`INFORMATION_STATE_PCBF.md`, `TERMINAL_CBF_PROOF.md`,
`CURVED_CRUISE_CERTIFICATE.md` and `LTV_BICYCLE_MODEL.md` now hold at the
nodes only.

## Implementation

- `ltvBicycleModel.finitePredict` emits one cell per hold whose single point is
  the node: `map`, `offset` and `radius` are the plan-affine node state and
  its box; `localStateMap`, `localInputMap` and `localOffset` are `A_k`,
  `B_k` and `c_k`; `start = time = k h` and `duration = 0`. No held-interval
  enclosure is computed online.
- `avoidanceSafetyGeometry.localCellRows` evaluates a one-point cell against
  the target's bounded set at the node time (center, radius and yaw radius of
  `targetPrediction.finiteFlow`); multi-point cells keep the Bernstein path
  for offline audits. The native kernels were regenerated.
- `formulateAvoidanceProblem.localShift` reconstructs the node clock as
  `stage*h` when the executed hold is eliminated.
- `avoidanceSafetyGeometry.jointProgram` uses those node enclosures to build
  collision and exit records. `avoidanceStageQp.fixedDirections` constructs
  their fixed-direction support majorants. Every successful fresh admission
  requires one full trajectory solve; an active successor attempts one hard
  improvement using retained directions. Scalar initializers are not issued.
- `stateUncertainty.heldInterval` and `ltvBicycleModel.fixedPredict` remain
  for the scheduled terminal-family synthesis (whose whole-hold phase
  condition is stronger than the node condition and therefore still valid)
  and for offline swept audits. `encounter.taylorOrder` affects only those.
