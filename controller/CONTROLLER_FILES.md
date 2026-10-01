# Controller source map

The controller and configuration contain **nine MATLAB source files**.
The current formulation and its guarantee scope are in
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md).

| Source | Responsibility |
| --- | --- |
| `collisionAvoidanceController.m` | Fixed target epoch, absolute clock, input memory and first control output |
| `readControllerInputs.m` | Ego, one target and road input normalization |
| `solvePredictiveControl.m` | First feasible PCBF continuation, moving-target flow initialization, conic restoration and retained witness |
| `terminalContinuation.m` | Rigidly placed augmented terminal core, Schur pose elimination, RK4 contraction bounds and indefinite target separation |
| `nonlinearBicycleModel.m` | Fiala bicycle RK4, variational tangents, road load, trim and lane CLF |
| `modifiedFialaTire.m` | Combined-slip tire forces and derivatives |
| `predictiveSafetyGeometry.m` | Target prediction, transported flow guidance, signed polygon duals and road frame |
| `laneGeometry.m` | Straight and circular lane coordinates |
| `../config/collisionAvoidanceControllerConfig.m` | Defaults, merging and validation |

The time-indexed tube is stored implicitly as a feasible hard completion
and its endpoint family. A valid witness is executed immediately; search runs
only when nonlinear revalidation finds no admissible continuation.
The terminal core is built at straight cruise and may be translated and
rotated independently of the nominal given path. Completing the square in its
eight-dimensional metric eliminates the three pose coordinates exactly for
membership. Every reconstructed pose must still pass indefinite target
separation. An accepted pose stays fixed when shifting/appending the MPC
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
constraints directly. See the architecture for numerical limitations.

Behavior tests include `freePoseTerminalTest`, `terminalContinuationTest`,
`nonlinearPredictiveSafetyTest`, `controllerInputGeometryTest`,
`longitudinalRoadLoadTest`, `modifiedFialaTireTest` and
`collisionAvoidanceControllerConfigTest`. `controllerSourceBudgetTest`
includes the separate endpoint-construction module in its nine-file limit.
