# Predictive CBF and soft CLF controller

The format-35 controller uses one convexification on the shifted previous
nominal trajectory and one complete native SOCP solve per frame. At each
hold node it computes an analytic rectangle support direction and fixes it
for the solve. The first frame uses cruise initialization. There is no
passing-side selection, branch enumeration, feasibility restoration or
post-solve direction iteration.

The predictive continuation, fixed active-encounter exit deadline, invariant
terminal set, actuator amplitude/slew limits, curved pose domains and squared
CLF slack penalty remain. Input effort is relative to the CLF/LQR operating
input. The observer is unchanged. No physical road boundaries are enabled
in the current validation scenarios. The declared plant is the exact sampled
held affine model, with zero process residual.

Every returned command passes independent hard-row, terminal-cone and CLF
verification. A solver failure or deadline miss issues no command. The
terminal feedback remains a prediction certificate only.

A newly computed normal family can exclude the old feasible continuation.
The controller reports that inclusion result and does not claim an automatic
active-encounter recursive-feasibility guarantee. Target-free continuation
retains its conditional guarantee. See
[the convexification policy and paper comparison](SUPPORT_CONVEXIFICATION.md),
[the conditional proof](TERMINAL_CBF_PROOF.md) and
[CLF details](SINGLE_SOLVE_CBF_CLF.md).

Safety is certified at 100 ms hold nodes only; no continuous-time safety
claim is made between them. See [NODE_SAMPLED_CERTIFICATE.md](NODE_SAMPLED_CERTIFICATE.md).

```matlab
cfg = collisionAvoidanceControllerConfig();
certificate = [];
[command, inputs, problem, certificate] = ...
    collisionAvoidanceController(ego, targets, road, cfg, certificate);
```

Supply timestamped ego measurements, complete current perception declarations,
stable target identities, bounded target-motion contracts and the actual
previous held actuator input. Reset stored certificates from earlier formats.
Dated experiment outcomes and limitations belong under `report/`.
