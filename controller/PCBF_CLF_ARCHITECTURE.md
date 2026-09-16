# Predictive CBF and soft CLF controller

Current implementation, September 16, 2026: the format-31 controller uses
distance-dual convexification as its sole collision-convexification method. It
retains predictive continuation across active encounters, confirmed partial
or full target release, and prediction exhaustion. The terminal information
set uses the same held affine generator as online prediction. The CLF retains
its squared slack penalty, and input effort remains relative to the certificate
operating input. The high-gain target observer is unchanged.

Each held command has one complete Bernstein certificate, with adaptive
polynomial order and no internal time subdivision. State-box and tire-slip
constraints are removed; actuator amplitude and finite slew bounds remain.
Geometry ranges follow actuator reachability. The terminal invariant set is
synthesized from the same actuator limits, without hidden state/slip bounds.

Admission has no selected maneuver side or prescribed lateral trajectory.
Li et al.'s distance dual supplies continuous separating directions at computed
anchor poses. Those directions enter the complete robust swept constraints;
collision remains hard. See [DISTANCE_DUAL_CONVEXIFICATION.md](DISTANCE_DUAL_CONVEXIFICATION.md).
Every fresh anchor midpoint must yield a valid distance-dual direction before
the hard trajectory SOCP is built. Overlap/contact makes ordinary distance zero;
unavailable initialization stops control without claiming physical infeasibility.
No direction-grid enumeration, integer search or alternate initializer remains.
The stationary and oncoming scenarios that previously depended on finite
initialization must therefore be assessed afresh. No claim of global performance
optimality or complete coverage of all physically safe trajectories is made.

After admission, first preserve the actual accepted constraint family: substitute the
executed input in its affine family, retain the terminal cones and continuous
swept enclosures, and condition successor information by inclusion. The
resulting suffix supplies a feasible candidate for the next convex solve.
New distance-dual collision rows may replace the inherited rows only after
that same suffix passes every proposed hard row and CLF/terminal cone.
Otherwise the inherited program remains the optimization problem.
Confirmed removal deletes only that target's obligations. A fresh target-free
horizon may replace the suffix only when a full feasible witness is retained.
At prediction exhaustion, the permanent invariant-set optimization still
chooses its control freely. A terminal policy is a mathematical witness and
never an executed fallback.

```matlab
cfg = collisionAvoidanceControllerConfig();
certificate = [];
[command, inputs, problem, certificate] = ...
    collisionAvoidanceController(ego, targets, road, cfg, certificate);
```

Supply timestamped complete circular perception, stable target identifiers,
finite motion bounds, and the actual previous held input. The conditional
recursive-feasibility proof and sensing/model assumptions are in
[TERMINAL_CBF_PROOF.md](TERMINAL_CBF_PROOF.md). It applies after complete
admission. Fresh target entry is a new feasibility obligation. The declared
plant remains the zero-residual held affine model; separate nonlinear Fiala
studies do not silently extend the online theorem.

No command is issued after unavailable distance geometry, failed convex solving
or failed hard verification. Frame latency remains a separate requirement;
allowing a longer offline search does not establish 100 ms operation.
See [SINGLE_SOLVE_CBF_CLF.md](SINGLE_SOLVE_CBF_CLF.md) for the CLF derivation,
terminal details and certificate interface, and the dated reports under
`report/` for executed experiments.
