# Finite target motion and encounter-exit contracts

Version 9 requires a current target enclosure, a finite motion contract, and an
exit guard. Current estimator bounds do not establish future jerk limits,
route nonreturn, or reliable detection. Those are separate caller assumptions.
The governing requirement is [ENCOUNTER_SCOPED_CBF_CLF.md](ENCOUNTER_SCOPED_CBF_CLF.md).

A target is a scalar record; the controller accepts a structure array of such
records. Existing inertial position, velocity, acceleration, body heading,
yaw-rate, extent, and current-error fields remain supported. `trackId`,
`targetId`, `objectId`, or `id` must supply a stable unique identifier. Current
`target-state-v1` estimator certificates override numeric current-bound aliases.
Their timestamps must match `ego.stateTime`.

Each target also requires:

```matlab
target.encounterContract = struct( ...
    "kind", "cartesian-jerk-exit-v1", ...
    "id", "crossing-1", ...
    "validFrom", 0.0, ...           % absolute seconds
    "validUntil", 1.6, ...          % absolute seconds
    "jerkBound", [0.0; 0.0], ...   % componentwise m/s^3
    "yawAccelerationBound", 0.0, ... % rad/s^2
    "exitNormal", [0.0; 1.0], ... % unit inertial normal
    "exitOffset", 4.6, ...         % plane offset in metres
    "postExitRoute", "nonreturningHalfspace");
```

These example values describe the declared synthetic fixture in
`tests/encounterTestFixture.m`. They are not defaults or measured contracts for
an arbitrary target. Admission rejects missing fields, expired validity,
anonymous identities, nonunit normals, and unsupported guard kinds.

## Finite Cartesian inclusion

The current state is `[pX;pY;vX;vY;aX;aY;psi;omega]`. During the active encounter,
`pDot=v`, `vDot=a`, `abs(aDot)<=jerkBound`, `psiDot=omega`, and
`abs(omegaDot)<=yawAccelerationBound`. For elapsed time `t`, the nominal position
is `p+v*t+a*t^2/2`; its radius is
`rhoP+rhoV*t+rhoA*t^2/2+jerkBound*t^3/6`. Corresponding velocity, acceleration,
yaw, and yaw-rate radii follow integration of the same inclusion. Body heading
is independent of velocity course, so zero speed causes no curvature quotient.
The legacy fixed-curvature utility methods remain available for independent
model comparisons; the controller uses `admit`, `finiteFlow`, `advance`, and
`exitMargin`.

Motion validity must cover every held interval up to that target's certified
exit. A short forecast is rejected if no guarded exit can occur before expiry.
The controller does not extrapolate an active obligation beyond its contract.
No infinite target trajectory is requested after discharge.

## Implemented guard

For unit normal `n`, exit requires

`n'*centerPosition - abs(n)'*positionRadius - rectangleSupport`

at least `exitOffset + collision.clearanceMargin`, with the configured
numerical reserve. Rectangle support is maximized over the complete yaw
interval. Admission separately checks that every segment of the allowed ego
route, its entire lateral domain, and its footprint remain upstream of the
plane. The caller's `nonreturningHalfspace` assertion requires the target's
whole footprint to stay at or beyond `exitOffset + collision.clearanceMargin`
after exit. Together these premises
constitute discharge; instantaneous separation alone does not.

This guard is conservative and principally serves crossing routes. A target
sharing the ego's indefinite route generally needs a different guard or
monitoring/handoff contract, which this runtime does not implement. Waiting,
forecast renewal, and route-contract replacement fail admission unless a new
independent supported encounter is supplied. A later encounter needs a new
stable identity and timely joint admission. The code does not itself certify
sensor detection range or environmental completeness.

## Observations and lifecycle

Missing observations retain the active target and its old motion inclusion.
A known target may omit `encounterContract` on subsequent observation records;
the controller then retains the existing contract. A supplied replacement must
match it. A valid observation intersects the carried reachable box, keeps its nominal
center, and can shrink its radius. The update must not silently replace jerk,
yaw-acceleration, extent, route, validity, or identity contracts. A contradictory
observation reports an assumption failure and authorizes no inherited fallback.

Each target exits on its own sample guard; joint optimization retains every
active target until its guard is met. Discharged records retain the route
assertion, without prediction beyond expiry. If a later observation contradicts
that assertion, the controller reports `exitRouteViolation`. Missing perception
by itself never discharges an active target.
