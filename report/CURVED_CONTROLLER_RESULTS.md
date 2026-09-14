# Curved-controller validation (September 8, 2026)

The nonlinear cruise trim, local Riccati certificate and input-cost center now use the same road curvature and declared longitudinal bias. The Predictive CBF/CLF framework, squared slack and scheduled-actuation contract remain. See [the derivation](../controller/CURVED_CRUISE_CERTIFICATE.md).

## Reproduction and scope

```matlab
addpath('scripts');
report = runCurvedControllerValidation(OutputDirectory="/absolute/output/path");
assert(report.passed);
```

This is an exact-state nonlinear modified-Fiala bicycle experiment, with independent Cartesian ODE integration. It is not a PassVeh14DOF or curved observer/controller validation. All eight 20-second trials completed. The release run contains 1,600 online frames; every frame met the 100 ms deadline. Timing includes online input assembly and the controller, and excludes offline road construction, plant integration and the post-step audit. Three discarded empty-target controller calls precede each run. It is not a hardware worst-case execution-time guarantee.

Settings: 10 m/s body longitudinal cruise speed; 100 ms sample period; one sample of scheduled input delay; two certified intervals; 20 prediction stages (2 s); steering-rate limit 0.5 rad/s; signed-force-ratio rate limit 2/s. Vehicle mass 1,650 kg, yaw inertia 1,700 kg m^2, axle distances 1.4/1.65 m, footprint 4.8 by 1.9 m. Other tire/road-load parameters are the committed controller defaults and are exported with each trial.

The road has half-width 6 m. The allowed center strip is `6 - hypot(2.4,0.95)` m. The 1-Lipschitz signed distance to a line or circle then contains the complete rectangle for any orientation. Independently evaluate all rectangle corners along the ODE trace. Targets follow the corresponding offset arc at a constant station rate of -10 m/s, lateral offset 0.8 m, footprint 5 by 2 m, and appear only within 30 m. Their true curvature and speed are constant; supplied jerk bounds cover Cartesian acceleration rotation. No random noise or random seed is used.

Recovery trials start with `[d,ePsi,vx,vy,r]` errors `[0.2,0.01,-0.5,0,0]` relative to the curved trim, and contain no target. Avoidance trials start at that trim. The last three observed seconds are checked against 0.2 m lateral, 0.02 rad heading and 0.5 m/s speed tolerances; this is an observed final window, not an imposed early recovery deadline or an asymptotic proof.

## Release results

| Road | Target | Minimum sampled SAT margin (m) | Minimum sampled road margin (m) | Maximum frame (ms) | Completed |
|---|---|---:|---:|---:|---|
| Straight | No | Not applicable | 4.808345 | 48.748 | Yes |
| Straight | Yes | 0.361089 | 1.750251 | 49.740 | Yes |
| Left R=400 m | No | Not applicable | 4.820303 | 46.160 | Yes |
| Left R=400 m | Yes | 0.403372 | 1.993168 | 53.912 | Yes |
| Left R=100 m | No | Not applicable | 4.831583 | 39.734 | Yes |
| Left R=100 m | Yes | 0.566262 | 1.621623 | 40.442 | Yes |
| Right R=100 m | No | Not applicable | 4.762087 | 42.044 | Yes |
| Right R=100 m | Yes | 0.594243 | 1.504423 | 40.416 | Yes |

Across all eight final windows, maximum absolute errors relative to the appropriate cruise trim are 0.004118883 m lateral, 3.10490214e-06 rad heading, and 0.001969894 m/s speed. All returned plans passed the existing independent acceptance checks. No fallback was applied. Positive CLF slack and the small remaining offsets do not establish exact asymptotic convergence.

## Diagnostic findings and limitations

The old curved working point balanced the simplified frozen linear model. At 15 m/s and radius 100 m, its maximum nonstation derivative residual in the nonlinear model was 0.04093058, whereas the corrected trim residual is about 2.36e-12. Individual derivative components have their respective state units; the residual norm is a numerical consistency diagnostic. The corrected point includes front-force rotation, full nonlinear lateral forces, exact sideslip/heading kinematics and passive resistance.

A 1.6 s horizon on the right 100 m arc repeatedly exceeded the frame budget at first acquisition (approximately 135 ms). A captured frame needed eight numerical calls and three failed nonlinear nominal checks before acceptance. The 2 s horizon completed the corresponding wide-strip diagnostic with the original geometric seed and a 52.763 ms maximum. The final common configuration additionally uses the explicit 6 m half-width road strip. A shorter 1.2 s horizon failed, and an experimental input trust region was infeasible. A nonlinear seed calibration reduced retries but did not meet the deadline in that case. Both additional mechanisms were removed; they are not part of the final controller.

The diagnostic residual allowance is `[0.05,0.03,0.01,0.2,0.5,0.5]` in the derivative units of `[s,d,ePsi,vx,vy,r]`. It was selected after failed calibration runs: an initial dynamic allowance of 0.1 was exceeded, and later station residuals exceeded 0.02. Every executed release ODE trace stayed within the selected allowance. These eight runs are consistency checks after calibration, not independent proof of a uniform residual bound. The separate, larger PassVeh14DOF allowance made the generic bicycle setup infeasible and was not represented as validated for this model.

All 215 distinct regression tests in 16 classes pass, including nonlinear trim and both turn directions, continuous CLF bounds, input-objective equivalence, native kernels, finite sensing, delay contracts and the existing straight exact/noisy-estimate recovery tests. The new unattainable-turn test initially received the lower-level tire-domain error; an explicit necessary lateral-force feasibility check now rejects that request at the cruise-working-point interface. Eight edited/new MATLAB files pass factory Code Analyzer checks without findings.

The constant-curvature experiments do not establish convergence for arbitrary curvature changes. The implementation freezes its local metric within each horizon and reports metric/reference value jumps between frames; it does not constrain those jumps. The previous high-fidelity delayed-execution failure and the absence of curved joint validation remain unresolved.

Raw originals: `/home/zai/.cache/collisionAvoidance/curved-certificate-20260908/`. The `release-validation` directory contains final trials and their aggregate report. Earlier failed attempts remain separate diagnostic records. Generated experiment data and native dependencies are excluded from the project commit.
