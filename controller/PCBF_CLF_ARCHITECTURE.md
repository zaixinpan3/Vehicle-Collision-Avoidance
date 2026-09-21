# Predictive CBF and soft CLF controller

The format-40 controller first tries to admit a new encounter using one signed affine
control section and forbidden-amplitude interval subtraction. A fixed support
dictionary and fixed yaw enclosures on amplitude cells bound the geometric
work. The scalar objective retains input effort and squared CLF slack.
An independently verified scalar witness is issued directly. After section
exclusion, a least-violated geometric proposal can initialize one hard joint
SOCP with the complete control sequence free. That proposal cannot be issued;
only an independently verified result can become the first incumbent. A nominal
already carrying a complete certificate, or a shifted accepted certificate,
can be improved with one hard joint trajectory/support SOCP. Target-free frames
solve the convex dynamics/CLF/terminal base. Admission recovery adds no collision
slack, route catalog, method selector or executable backup controller.

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
