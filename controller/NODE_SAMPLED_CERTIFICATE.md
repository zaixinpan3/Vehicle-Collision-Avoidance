# Hold-node safety certificate

Adopted on 2026-09-17 by project decision. The online controller certifies
safety at the sampling nodes of the exact sampled affine plant, not at every
instant inside a held command. This note states exactly what the certificate
covers, what it no longer covers, how the implementation realizes it, and
which whole-hold enclosures remain in offline tooling.

## Trajectory-linearized encounter update (September 25, 2026)

With the default `model.linearizationPolicy="trajectory"`, active encounters
now rebuild their affine generators once per frame. The current measured
state initializes a nonlinear Fiala rollout under the shifted previous input
plan (cruise inputs initialize the first frame). Each hold is linearized at
its own rollout state and input, including spatial curvature sensitivity on
varying references. The stages stay fixed during the existing direction
search and optimization; there is no dynamics-relinearization iteration,
new trust-region restriction, or nonlinear acceptance check.

The node certificates below concern that frame's frozen affine prediction.
They are not a nonlinear-plant certificate, and the shifted-witness recursive
argument below does not apply across these refreshed encounter models.
`recursiveFeasibilityGuaranteed` is false for trajectory-linearized predictions;
each new active frame must pass fresh admission, even if a previous plan was
feasible. The old plan supplies an anchor, not an executable fallback.
Explicit `linearizationPolicy="cruise"` retains the carried-model study path.
Target-free cruise and the existing terminal continuation are unchanged.
The first-hold CLF cone and reported generator use the actual prediction
stage; the cruise reference and Lyapunov metric remain the tracking objective.
Trajectory error feedback is now designed backward along the same stage
matrices; all error, input and slew supports use that gain sequence. This
replaces the incompatible cruise gain on active trajectory models without
removing arithmetic reserves. Stored controller state version 46 rejects
older generator/feedback contracts. See [the feedback design](FEEDBACK_TUBE_PREDICTION.md).

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
of the scheduled continuous generator of hold `k`. The plan is a feedback
policy ([FEEDBACK_TUBE_PREDICTION.md](FEEDBACK_TUBE_PREDICTION.md)): `u_1` is
exact and `u_k = v_k + K (xhat_{k-1} - x_{k-1})` from the second hold on, with
the estimator error bounded at every hold. The true state at node `k` lies in
`x_k + E_k`, where `E_0` is the current measurement box,
`E_1 = A_1 E_0`, and `E_k = (A_k + B_k K) E_{k-1} + B_k K H + D_k` for `k >= 2`.
`E_k` is a zonotope whose held process reserve `d_k` (zero for the declared
zero-residual plant) and floating-point allowance `a_k` form an interval part.

The certified predicate is: for every node `k = 1, ..., N` and every state in
its box,

1. the ego rectangle and the target's bounded reachable set at time `t_k` are
   strictly separated by the selected support half-space after uncertainty
   and numerical allowances. The default physical clearance is 0.10 m at
   periods up to 50 ms and scales proportionally for longer holds. An explicit
   `collision.safetyMarginMeters` overrides that default, including zero.
   This engineering reserve does not certify inter-node clearance,
2. for scheduled references, the state lies within the reference phase band
   and the lateral regularity radius (phase rows); curved-road chart boxes are
   not enforced, so the chart remainder is not guaranteed outside
   `poseTrustRadius` of the seed pose,
3. the terminal node `N` lies in the modal terminal set and satisfies the
   finite-exit support certificate,

together with the actuator amplitude and slew rows on the held inputs and the
soft first-hold CLF cone. The current formulation fixes each certificate's
unit direction before optimizing the complete trajectory. Under unchanged contracts, the shifted plan
and its retained directions certify nodes `2, ..., N`; touching majorants
contain that complete witness in the next optimization. The active horizon
shrinks toward the fixed exit deadline. After confirmed release, terminal
invariance supplies target-free continuation. Since the September 25 stress
campaign, solver status alone cannot authorize execution. The complete lifted
decision must satisfy its equalities, inequalities, and cones within numerical
tolerance; the original-coordinate actuator, road, terminal, and joint support
constraints are checked independently before the unchanged command is issued.
The physical checks retain all pre-solve reserves. See
[JOINT_SUPPORT_CERTIFICATES.md](JOINT_SUPPORT_CERTIFICATES.md).

If both geometric passing seeds fail on a fresh frame, at most six phase-I
direction updates may run under the same work deadline. These searches soften
only separation/exit rows to locate useful directions. Their points have no
execution authority: a separate hard SOCP and the checks above must succeed.
The normal inherited-family path and its certified-incumbent fallback remain.
The encounter horizon retains its proposed physical duration; a configurable
512-hold allocation limit causes an explicit rejection instead of truncating
the horizon before the encounter or allocating an unbounded program.

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
  the target's bounded set at the node time (the center, radius and yaw
  radius that `avoidanceSafetyGeometry.build` takes from
  `targetPrediction.finiteFlow` at the node); a multi-point cell with a target
  is refused (`unsupportedWholeHoldTarget`, since 2026-09-24), while
  multi-point road-boundary and domain rows keep their Bernstein form for
  offline audits. The native kernels were regenerated.
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
