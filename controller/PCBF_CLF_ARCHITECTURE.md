# Predictive CBF and soft CLF controller

The format-41 controller first initializes separation directions, then fixes
them and solves one convex problem for the complete control sequence. Fresh
initialization uses an [NRMM-based time-dependent VFFM reference](NRMM_VFFM_INITIALIZATION.md)
in the selected road chart. Analytical target poses set moving Gaussian centers
at ego arrival times; all active target contributions are superposed. Two passing
assignments share a terminal-preserving fit to the affine vehicle model.
Physical base-row feasibility and robust support residuals select one seed;
analytic rectangle normals initialize the subsequent full trajectory solve. No direction dictionary, amplitude cells or scalar
objective search remains. The seed may violate constraints and never authorizes
execution. Every new admission, including an already certified nominal, must
come from the full trajectory solver and pass independent hard verification.

Inherited frames retain their certificate directions and attempt one full
trajectory improvement. Target-free frames solve the convex dynamics, CLF
and terminal base. The fixed-direction architecture follows the two-stage
principle of Li et al. (2023), while the retained yaw support majorant, terminal
cones and soft CLF make this implementation an SOCP. It does not add the
paper's collision slack. See [the formulation](JOINT_SUPPORT_CERTIFICATES.md).

The predictive continuation, fixed active-encounter exit deadline, invariant
terminal set, actuator amplitude/slew limits, curved pose domains and squared
CLF slack penalty remain. Input effort is relative to the CLF/LQR operating
input. The observer is unchanged. No physical road boundaries are enabled
in the current validation scenarios. The declared plant is the exact sampled
held affine model, with zero process residual.

Every returned command passes independent physical-row, support-residual,
terminal-cone and CLF verification. Failed fixed-direction improvement can return only
the previously optimized, independently verified suffix. A full-frame deadline miss or absence
of a hard certificate issues no command. The terminal feedback remains a
prediction certificate only.

Retained occupied sets and angles plus touching majorants constructively
preserve subproblem feasibility under unchanged sensing/execution contracts.
Admission from every seed and completion within a sample period are not
guaranteed. See
[the fixed-direction formulation and research assessment](JOINT_SUPPORT_CERTIFICATES.md),
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
