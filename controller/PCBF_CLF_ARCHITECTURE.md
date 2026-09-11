# One hard predictive CBF–CLF controller

The version-16 implementation uses one hard safety formulation for cruise and
avoidance. Every held interval in the admitted horizon includes uncertain
collision geometry, road boundaries, model-domain, actuator and slew limits.
The only optimization relaxation belongs to CLF performance.

The nonlinear modified Fiala force law retains its intrinsic combined-force
saturation. No additional axle-force polygon is imposed on its affine
approximation. Tire-slip model domains remain hard. Certificate version 16
rejects earlier witnesses and the removed `model.frictionPolygonSides` option.
Nonlinear model enclosure remains a separate premise.

The configured horizon seeds admission search; it does not prescribe a target
exit time. The one complete certificate may contain more intervals. A checked
complete witness is retained through exit, target-free expiry renews the same
formulation, and first detection does not inherit an old cruise expiry. See
[the free-completion-time correction](FREE_COMPLETION_TIME.md).

The controller first maximizes a nonnegative certified safety margin by LP,
then minimizes tracking, input effort and CLF relaxation in an SOCP while
preserving that margin. Independent verification authorizes the chosen input.
There is no track/yield/pass-left/pass-right selection, partial certification,
nominal-only safety suffix, delayed-input path, exit-route contract, or
fresh-solve-only execution policy. Earlier certificate formats and removed
configuration fields are rejected.

```matlab
cfg = collisionAvoidanceControllerConfig();
certificate = [];
[command, inputs, problem, certificate] = ...
    collisionAvoidanceController(ego, targets, road, cfg, certificate);
```

Supply timestamped complete circular perception at admission, stable target
identities and finite motion bounds. Zero targets use the same formulation.
Subsequent calls carry the certificate, scheduled timestamp and actually held
input. The four-argument convenience interface stores the same certificate;
resetting convenience state is not a mathematical successor construction.

During an active encounter, the original prediction matrices, geometry, target origins, executed prefix
and absolute deadline remain in the certificate. Between information events,
removing the executed input leaves a checked feasible suffix. Reoptimization
can replace that suffix only without reducing the certified margin. A failed
replacement leaves the checked incumbent available; no alternative control
law is invoked.

At first detection during an already active encounter, the augmented joint
state must belong to the certifiable feasible domain of the remaining original
problem. First detection during target-free execution instead starts fresh
complete admission. New constraints are added
jointly with all retained obligations. The event can reduce the nonnegative
margin, but cannot reset the deadline, discard old targets, or change executed
inputs. Failure to obtain a checked joint witness issues no control and does
not prove that the mathematical feasible set is empty.

Verified perception exit
returns an empty command. The caller stops the encounter. This implementation
establishes conditional finite-encounter recursive feasibility for the declared
inclusion; it does not establish indefinite driving, a physical vehicle model
enclosure, or a zero-latency real-time implementation.

See [the proof and runtime contract](HARD_PREDICTIVE_CBF.md),
[the target interface](TARGET_PREDICTION_CONTRACT.md), and
[the encounter scope and implementation status](SINGLE_PATH_RECURSIVE_FEASIBILITY.md).
