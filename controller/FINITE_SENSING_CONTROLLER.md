# Complete finite-sensing encounter controller

Current-scope note, September 11, 2026: finite perception is no longer part of
the controller problem. The active version-17 exact scheduled-model controller
uses one persistent target and an invariant terminal continuation. This document
records earlier finite-encounter analysis; current use and guarantees are in
[SINGLE_PATH_RECURSIVE_FEASIBILITY.md](SINGLE_PATH_RECURSIVE_FEASIBILITY.md).

As of September 10, 2026, the only controller is the version-16 hard predictive
certificate described in [PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md).
All prediction intervals are certified. Cruise and avoidance share one
continuous-input optimization, with no discrete maneuver or completion-policy
selection. The partial-horizon and delayed-input implementations are removed.

The sensor and target interface is specified in
[TARGET_PREDICTION_CONTRACT.md](TARGET_PREDICTION_CONTRACT.md). First-detection
joint feasibility is an explicit research premise; ordinary continuation uses
the stored witness. No repeated successful-solve assumption is substituted for
that construction. Completed encounters return no further command.

The remaining model notes below describe geometry and physical modeling.
Dated calibration or runtime measurements predate the single-path cleanup and
do not validate its timing, full-horizon feasible domain or post-exit behavior.

The configured planning window is not a target exit deadline. Version 15
searches for a complete witness extending beyond that window where necessary.
It never executes a terminal-free prefix as an encounter guarantee. Actual
verified exit ends the encounter; target-free expiry renews the same controller.
See [FREE_COMPLETION_TIME.md](FREE_COMPLETION_TIME.md).

## Geometry and vehicle model

The ego state is Frenet station, lateral displacement, heading error, body
longitudinal/lateral velocity and yaw rate. Inputs are front-wheel steering
angle in radians and dimensionless signed braking ratio. Physical parameters,
modified Fiala force scales and passive road load retain their existing units.

A scheduled affine bicycle inclusion propagates every held input through an
exact matrix exponential and a Bernstein enclosure. Initial-state errors,
declared process/model residuals and numerical allowances are charged on every
cell. There are no endpoint-only safety chords or nominal-only future stages.
Reference-curve chart errors enter the geometric footprint supports. Road
coverage must contain the complete admitted footprint horizon; the controller
does not shorten its deadline to bypass missing coverage.

A separating normal is fixed within each cell, with uncertain target and ego
rectangle supports evaluated at every Bernstein point. The target's full
finite-motion enclosure supplies collision rows and the terminal sensor-exit
condition. Initial geometry is selected from one reference anchor, without
discrete passing or yielding corridors. This convex inner approximation may
exclude physically feasible maneuvers; no completeness claim is made.

`model.linearizationPolicy` selects the declared schedule construction; it
does not select a different safety or execution policy. Nonlinear rollouts are
model anchors and diagnostics. Every online safety interval still uses the
scheduled inclusion. Its validity for a nonlinear physical plant depends on
the declared residual bounds, which are not established by linearization alone.

The complete nonlinear modified Fiala force law already limits each axle's
combined force through `eta=sqrt(1-beta^2)`. Version 14 removes the additional
force polygon from every safety interval and both numerical transcriptions.
Tire-slip domains, actuator bounds, road/collision geometry and slew limits
remain hard. The affine prediction does not automatically inherit nonlinear
saturation; its declared residual must still enclose the nonlinear dynamics.
No nonlinear or physical enclosure is established merely by removing rows.

## Numerical implementation

The sole online program uses sparse cell states, controls and CLF slacks.
Named row maps preserve dynamics equalities on margin updates and rebuilds.
Complete original physical constraints remain in the independent checker.
A hard-margin LP precedes a subordinate CLF SOCP. The independent checker
charges the immutable numerical reserve and rejects negative hard margins.
The test-only condensed oracle remains a numerical comparison, with tests
for identical hard decisions, CLF cone values and objective differences; it
is not an alternate controller configuration.

Generated cell-support and projection kernels share their MATLAB source in
`avoidanceSafetyGeometry.m`. Build them with
`scripts/buildAvoidanceGeometryKernel.m`. The former nominal-check kernel,
future reserve-allocation policy and delayed-controller replay driver are
removed. Historical physical and runtime reports remain dated evidence of
their original implementations, not validation of this controller.
