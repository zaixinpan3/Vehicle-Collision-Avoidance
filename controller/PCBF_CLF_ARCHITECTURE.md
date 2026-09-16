# Predictive CBF and soft CLF controller

Current implementation, September 16, 2026: the format-28 controller searches
a finite conservative polyhedral family at initial encounter admission. It
retains predictive continuation across active encounters, confirmed partial
or full target release, and prediction exhaustion. The terminal information
set uses the same held affine generator as online prediction. The CLF retains
its squared slack penalty, and input effort remains relative to the certificate
operating input. The high-gain target observer is unchanged.

Admission has no selected maneuver side or prescribed lateral trajectory.
Each cell-target pair and each finite-exit condition has a finite set of
geometric directions. Integer branch-and-bound with lazy constraint generation
searches their assignments. Every accepted assignment is solved as a complete
convex problem and independently certified. Infeasible approximations and
unfinished searches are distinct outcomes. No claim of global performance
optimality or complete coverage of all physically safe trajectories is made.
See [FINITE_CONVEX_BRANCHES.md](FINITE_CONVEX_BRANCHES.md) for the approximation,
branch count, search logic, sound pruning and limitations.

After admission, the actual accepted branch is preserved: substitute the
executed input in its affine family, retain the terminal cones and continuous
swept enclosures, and condition successor information by inclusion. The
resulting suffix supplies a feasible candidate for the next convex solve.
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

No command is issued after exhausted/incomplete search, failed convex solving
or failed hard verification. Frame latency remains a separate requirement;
allowing a longer offline search does not establish 100 ms operation.
See [SINGLE_SOLVE_CBF_CLF.md](SINGLE_SOLVE_CBF_CLF.md) for the CLF derivation,
terminal details and certificate interface, and the dated reports under
`report/` for executed experiments.
