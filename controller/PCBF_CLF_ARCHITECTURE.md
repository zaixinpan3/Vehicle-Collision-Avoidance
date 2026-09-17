# Predictive CBF and soft CLF controller

Current implementation, September 17, 2026: the format-33 controller uses
overlap-aware support geometry and feasibility restoration. It
retains predictive continuation across active encounters, confirmed partial
or full target release, and prediction exhaustion. The terminal information
set uses the same held affine generator as online prediction. The CLF retains
its squared slack penalty, and input effort remains relative to the certificate
operating input. The high-gain target observer is unchanged.

Each held command has one complete Bernstein certificate, with adaptive
polynomial order and no internal time subdivision. State-box and tire-slip
constraints are removed; actuator amplitude and finite slew bounds remain.
Active circular encounters use local affine pose maps with hard whole-hold
validity domains. These domains certify the geometric approximation; they are
not physical road boundaries or nonlinear plant-validity claims. Straight and
target-free geometry retains actuator reachability. The terminal invariant set is
synthesized from the same actuator limits, without hidden state/slip bounds.

Fresh admission searches bounded geometric support families. An analytic signed
configuration-obstacle query supplies a finite unit direction even at overlap
or contact. A direction defines a safe half-space; it does not certify its
anchor. Joint sector alternatives resolve symmetric ties consistently across
holds. No lateral path, amplitude, passing time or avoidance acceleration is
prescribed. Controls remain optimization variables.

A colliding seed enters search-only feasibility restoration: minimize grouped,
squared normalized collision, exit and terminal deficits with actuator limits
and local pose domains hard.
Whole-hold margin proposals update the geometry. A final hard-safety, soft-CLF
solve removes every search deficit and passes independent physical verification.
Bounded sector, exit-direction and horizon searches are incomplete; failure is
not proof of physical infeasibility. See
[SUPPORT_CONVEXIFICATION.md](SUPPORT_CONVEXIFICATION.md).

After admission, first preserve the actual accepted constraint family: substitute the
executed input in its affine family, retain the terminal cones and continuous
swept enclosures, and condition successor information by inclusion. The
resulting suffix supplies a feasible candidate for the next convex solve.
New support-direction collision rows may replace the inherited rows only after
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

No command is issued after invalid geometry, failed convex solving
or failed hard verification. Frame latency remains a separate requirement;
allowing a longer offline search does not establish 100 ms operation.
See [SINGLE_SOLVE_CBF_CLF.md](SINGLE_SOLVE_CBF_CLF.md) for the CLF derivation,
terminal details and certificate interface, and the dated reports under
`report/` for executed experiments.
