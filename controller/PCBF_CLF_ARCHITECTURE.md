# One hard predictive CBF–CLF controller

The version-13 implementation uses one hard safety formulation for cruise and
avoidance. Every held interval in the admitted horizon includes uncertain
collision geometry, road boundaries, model-domain, actuator and slew limits.
The only optimization relaxation belongs to CLF performance.

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

The original prediction matrices, geometry, target origins, executed prefix
and absolute deadline remain in the certificate. Between information events,
removing the executed input leaves a checked feasible suffix. Reoptimization
can replace that suffix only without reducing the certified margin. A failed
replacement leaves the checked incumbent available; no alternative control
law is invoked.

At first detection, the augmented joint state must belong to the certifiable
feasible domain of the remaining original problem. New constraints are added
jointly with all retained obligations. The event can reduce the nonnegative
margin, but cannot reset the deadline, discard old targets, or change executed
inputs. Failure to obtain a checked joint witness issues no control and does
not prove that the mathematical feasible set is empty.

Verified perception exit or exhaustion of a target-free finite certificate
returns an empty command. The caller stops the encounter. This implementation
establishes conditional finite-encounter recursive feasibility for the declared
inclusion; it does not establish indefinite driving, a physical vehicle model
enclosure, or a zero-latency real-time implementation.

See [the proof and runtime contract](HARD_PREDICTIVE_CBF.md),
[the target interface](TARGET_PREDICTION_CONTRACT.md), and
[the outstanding terminal obligation](SINGLE_PATH_RECURSIVE_FEASIBILITY.md).
