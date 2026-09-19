# Predictive CBF and soft CLF controller

The format-39 controller jointly optimizes the input sequence and separation
angles, including terminal encounter-exit angles. New encounters use one
projected geometric initialization and at most three
conic admission solves by default. Independently verified admission witnesses
are issued directly; subsequent frames optimize performance. An accepted certificate
is shifted with its complete geometry; one hard SOCP can then improve it.
Joint support is the sole algorithm. Target-free frames solve its convex
dynamics/CLF/terminal base directly.

The predictive continuation, fixed active-encounter exit deadline, invariant
terminal set, actuator amplitude/slew limits, curved pose domains and squared
CLF slack penalty remain. Input effort is relative to the CLF/LQR operating
input. The observer is unchanged. No physical road boundaries are enabled
in the current validation scenarios. The declared plant is the exact sampled
held affine model, with zero process residual.

Every returned command passes independent physical-row, support-residual,
terminal-cone and CLF verification. Failed joint improvement can return only
an independently verified incumbent. A full-frame deadline miss or absence
of a hard certificate issues no command. The terminal feedback remains a
prediction certificate only.

Retained occupied sets and angles plus touching majorants constructively
preserve subproblem feasibility under unchanged sensing/execution contracts.
Admission from every seed and completion within a sample period are not
guaranteed. See
[the joint formulation and research assessment](JOINT_SUPPORT_CERTIFICATES.md),
[the conditional proof](TERMINAL_CBF_PROOF.md) and
[CLF details](SAMPLED_CLF.md).

Safety is certified at the configured hold nodes only; no continuous-time safety
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
