# Nonlinear prediction in the nominal PCBF controller

The current formulation and its assumptions are documented in
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md).

The September 28 simplification removed nonlinear interval admission, MPFR
terminal synthesis, immutable target epochs and exact-successor contracts from
the public controller. `nonlinearSafetyCertificate.m` and
`nonlinearSafetyMex.cpp` have been removed. Nominal geometry and terminal rows
now live in `predictiveSafetyGeometry.m`.

`nonlinearBicycleModel.m` retains the nonlinear combined-slip Fiala equations,
RK4 held-input prediction, realizable lane trims, and LQR lane-error metric.
Analytic tire/body derivatives replace repeated finite differences in the
variational flow. `jointSample` combines the six ego states with the three
autonomous target pose states.

The implementation's nonlinear rollout is part of SCvx's merit and residual
calculation. It is not a rigorous nonlinear-flow enclosure. The optimizer's
first control is applied directly, and all reported slacks and margins have
nominal sampled-model meaning. See the main architecture for the conditional
recursive-feasibility argument and terminal-set assumptions.

The dated nonlinear-certificate reports preserve the results of those earlier
experiments. They do not describe the current controller or its guarantees.
