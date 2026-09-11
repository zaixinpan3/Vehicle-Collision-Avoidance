# Finite target motion and joint admission

Scope update, September 11, 2026: this document describes the existing finite-encounter
construction. The current requirement is the range-independent two-vehicle,
exact-target-prediction problem in [SINGLE_PATH_RECURSIVE_FEASIBILITY.md](SINGLE_PATH_RECURSIVE_FEASIBILITY.md),
including feasibility at every subsequent frame. Statements below that place
post-exit continuation outside the requirement are superseded. The finite
implementation and its proof do not yet establish that stronger guarantee.

Every target uses the same finite motion descriptor:

```matlab
target.predictionMotion = struct( ...
    "kind", "finite-sensing-motion-v1", ...
    "jerkBound", [jx; jy], ...             % componentwise m/s^3
    "yawAccelerationBound", yawRateRate); % rad/s^2
```

Stable track identity, position, velocity, acceleration, heading, yaw rate,
footprint dimensions and current componentwise estimation errors identify the
initial target set. The declared derivative bounds must cover the complete
remaining encounter. Observer estimation bounds alone do not establish these
future-motion premises.

`finiteFlow` encloses Cartesian constant-acceleration propagation plus bounded
jerk and yaw acceleration. Every safety cell uses that full uncertain flow.
`nominalFlow` supplies a constant-curvature, tangential-acceleration anchor for
model studies; its optional `scalarAccelerationMaximum` does not tighten the
uncertain safety enclosure.

The controller retains each target's first-detection time and original set.
Compatible measurements are checked against the retained flow. They cannot
renew its validity or replace old safety obligations. The independent
`advance` utility conditions an observation set for prediction studies; it is
not an alternative controller continuation path.

The accepted research assumption is joint feasibility at first detection,
including old targets, stored geometry, remaining deadline and executed
controls. Admission appends the new swept and endpoint constraints to the
retained program. It requires an independently checked joint witness. An
uncertified event is outside the declared research domain and produces no
command; a negative solver status is not proof of mathematical infeasibility.

Admission requires complete timestamped circular perception with a fixed
positive range. Missing observations cannot silently remove old constraints.
Verified exit ends the finite encounter; it does not establish nonreturn,
post-exit vehicle safety or indefinite cruise feasibility. The removed
`encounterContract` exit-route interface is rejected.

See [HARD_PREDICTIVE_CBF.md](HARD_PREDICTIVE_CBF.md) for the equations,
execution premises and successor-witness argument.
