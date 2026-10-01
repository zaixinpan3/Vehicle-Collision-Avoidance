# Controller source map

The controller and configuration contain **nine MATLAB source files**.
The current formulation and its guarantee scope are in
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md).

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Fixed target epoch, absolute clock, input memory and first control output |
| `readControllerInputs.m` | Ego, one target and road input normalization |
| `solvePredictiveControl.m` | PCBF safety restoration followed by CLF-slack optimization, moving-target flow initialization, encounter-range exit and nonlinear admission |
| `terminalContinuation.m` | Rigidly placed augmented terminal core, Schur pose elimination, RK4 contraction bounds, encounter departure under the endpoint policy and indefinite target separation |
| `nonlinearBicycleModel.m` | Fiala bicycle RK4, variational tangents, road load, trim and lane CLF |
| `modifiedFialaTire.m` | Combined-slip tire forces and derivatives |
| `predictiveSafetyGeometry.m` | Target prediction, transported flow guidance, polygon duals and adaptive interval separation |
| `laneGeometry.m` | Straight and circular lane coordinates |
| `../config/collisionAvoidanceControllerConfig.m` | Defaults, merging and validation |

The time-indexed tube is stored implicitly as a feasible hard completion
and its endpoint family. A valid witness establishes the zero minimum of the
primary PCBF objective. The next priority is actual nonlinear CLF relaxation.
A zero-slack witness reaches its global nonnegative lower bound; tracking and
input-deviation costs cannot purchase positive CLF slack. Nonlinear constraint
correction restores the CLF sublevel first and then repairs later inputs while
fixing the first input. Conic optimization and a minimum-deviation correction
QP remain parts of the same solve. Positive local stationarity and incomplete
budget-limited work are reported separately from zero-slack dissipation.
Open-loop proposals that leave the model domain, or defined full conic
proposals with an admissible first stage but an inadmissible continuation,
use linear-quadratic defect feedback to construct a nonlinear candidate.
The first input is preserved and the same complete admission is required
before execution; a defined raw candidate remains eligible.
At positive relaxation, predictive-cost refinement fixes the minimizing first
input and optimizes later controls, preserving that first-step CLF value.
The moving horizon retains the configured completion allowance, extending by
the admitted endpoint policy rather than counting that allowance down to zero.
The terminal core is built at straight cruise and may be translated and
rotated independently of the nominal given path. Completing the square in its
eight-dimensional metric eliminates the three pose coordinates exactly for
membership. A target constrains a prediction only up to its first node
farther than `cfg.collision.encounterRangeMeters`. An endpoint still inside
that range must show that its policy leaves it while separated at matching
times; indefinite target separation is the alternative for a persistent
encounter. An accepted pose stays fixed when shifting/appending the MPC
solution for the recursive-feasibility argument. There is no separate runtime
backup or nominal-versus-backup selector.
Positive-slack trajectories are search candidates only. Execution requires
zero prefix collision slack and all original hard constraints. Moving-flow
seeds need not be safe or terminal-admissible; their limited inputs generate
one nonlinear rollout shared by dynamics, tire and collision linearizations. The last applied input
belongs to the terminal state. The fixed lane ellipsoid, terminal input band,
1-norm polytope and clipped lane-feedback append have been removed.

Road boundaries are excluded from both prediction constraints and terminal
admission. The given path remains the nominal lane/CLF reference; terminal placement
does not redefine it.
Optional road widths do not affect the retained problem. Controller metadata
reports `roadConstraintsEnforced=false`; road-margin measurements belong to
the offline experiment audit.

MATLAB Optimization Toolbox and Control System Toolbox are required. There
is no native build or MPFR dependency. Endpoint Jacobian enclosures are cached
construction work. The online solve evaluates its defining nonlinear RK4
constraints and the separation of its pose interpolants. See the architecture for numerical limitations.

Behavior tests include `freePoseTerminalTest`, `terminalContinuationTest`,
`nonlinearPredictiveSafetyTest`, `clfNominalRecoveryTest`, `sweptCollisionIntervalTest`, `controllerInputGeometryTest`,
`longitudinalRoadLoadTest`, `modifiedFialaTireTest` and
`collisionAvoidanceControllerConfigTest`. `controllerSourceBudgetTest`
includes the separate endpoint-construction module in its nine-file limit.
