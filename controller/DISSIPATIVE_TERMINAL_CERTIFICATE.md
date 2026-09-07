# A dissipative terminal certificate for uncertain velocity

Implementation date: 2026-09-07. This extends the stationary-pose certificate
to nonzero six-state estimation enclosures under the declared scheduled affine
model. It does not establish robustness to an undeclared physical-model
residual, arbitrary moving targets, or noninvertible polyline projection.

## Design decision

A common open-loop input cannot drive every member of a nonzero velocity
interval to exact rest in finite time. Requiring that interval to have zero
radius made the NRMM/controller interface inadmissible even at its first
sample. A finite-horizon enclosure alone does not resolve this obstruction.

The new terminal condition admits a set of slowly moving states and reserves
space for their complete subsequent motion. It uses the existing input
`uR = [0; -accelerationBias / brakingRatioAccelerationGain]`: zero steering
and cancellation of the declared constant longitudinal bias. Passive road
load and tire damping then dissipate the remaining velocities in the
zero-speed scheduled model. No desired acceleration, ancillary feedback
gain, input-tracking cost, or second optimization is introduced. The online
objective remains raw head-input effort plus squared current CLF slack.

The exact-state and zero-velocity-radius stationary-pose cases retain their
previous terminal equalities. A nonzero velocity radius selects the new
certificate, even if the radius is numerically very small. A compatible
carried dissipative certificate keeps its type. Certificate version 7
prevents an older contract from authorizing the new continuation.

## Infinite continuation and invariant inequalities

Partition the terminal state into pose `p = [s; d; ePsi]` and velocity
`v = [vx; vy; r]`. The zero-speed scheduled continuous generator under `uR`
must have the form

\[
 \dot p = Gv,\qquad \dot v = Fv.
\]

`terminalDissipation.build` checks zero pose feedback, exact cancellation of
the affine forcing, an unchanged final zero-speed schedule, and zero
configured persistent disturbance. Longitudinal velocity must be an
independent damped channel; this preserves `vx >= 0`.

Let `C` retain the diagonal of `F` and replace its off-diagonal entries by
their absolute values. The implementation requires `C` to be Hurwitz. Then

\[
 |v(t)|\le e^{Ct}|v(0)|,\qquad
 M=|G|(-C)^{-1}\ge0,\qquad
 |p(t)-p(0)|\le M|v(0)|.
\]

For every terminal pose halfspace `a*p <= b`, impose the stronger inequality

\[
 a p+|a|M|v|\le b. \tag{1}
\]

Its upper Dini derivative is nonpositive:

\[
 D^+(ap+|a|M|v|)
 \le |a||G||v|+|a|MC|v|=0.
\]

Consequently (1) is forward invariant and implies the original halfspace
throughout the infinite terminal continuation. Velocities converge to zero;
pose converges to a finite limit. This is a stopping certificate, not a
claim that the backup converges to the performance cruise reference.

For the endpoint enclosure `x in z + [-r,r]`, the sufficient robust row is

\[
 a z_p+|a|M|z_v|+|a|r_p+|a|Mr_v\le b. \tag{2}
\]

The eight sign combinations of the three velocity components express (2)
exactly as linear inequalities. There are no extra decision variables.
`formulateAvoidanceProblem` uses these rows for terminal collision support,
road support, station, lateral position and heading domains.

Choose `q = alpha*(-C)^(-1)*ones(3,1)`, where `alpha` is the largest positive
scale allowed by the existing speed and terminal tire-slip bounds. The
numeric unit vector is a fixed positive comparison direction, with one unit
in each component's derivative units; it is not a vehicle disturbance.
Since `C*q < 0`, `|v| <= q` is forward invariant. Robust admission imposes
`|z_v| + r_v <= q` and `z_vx - r_vx >= 0`. Terminal steering and beta must
also satisfy the unchanged actuator limits. These constraints certify all
future terminal slip domains without assuming future measurements improve.

## Geometry and continuation preservation

Terminal road and target halfspaces use the ego circumradius. This covers
all terminal headings while the velocity-dependent pose budget covers
translation and heading-domain limits. The stationary-pose branch retains
its less conservative orientation-dependent rectangle support.

The terminal affine frame covers the anchor enclosure plus its complete
remaining pose excursion. Every finite-node frame also includes the
propagated station radius in addition to the nominal trust radius. A
candidate replacement frame must contain the whole current station box.
These changes prevent increasing uncertainty from being excluded merely by
a chart that was sized for an exact nominal point.

The dissipative terminal supporting chart is retained on a compatible shift.
Its invariant inequalities make the old endpoint followed by `uR` a valid
new endpoint. Current estimator and reachable boxes may be intersected while
retaining the old nominal center. Monotonicity of (2) in the radii preserves
this argument. A changed environment still requires complete readmission;
an incompatible plan never gains fallback authority merely by being stored.

Consecutive collinear polyline segments now form one initial invertible
chart. This removes rejection caused solely by sampling a straight road
every 0.1 m. Noncollinear corners and clipped route endpoints retain their
initial-chart rejection: their nearest-point coordinates need not reconstruct
every physical point in an uncertainty box.

## Validation and limits

`terminalDissipationTest` checks continuous terminal generators for straight,
left-curved and right-curved schedules, all eight velocity-box vertices,
nonnegative speed, pose-budget invariance, sparse/condensed QP equivalence,
live velocity-bound admission, and rejection without passive damping or
with persistent forcing.

`runRobustVelocityCertificateScenario` applies one initially accepted plan to
all 64 vertices of a six-state box with initial radii
`[0.04 m, 0.04 m, 0.014 rad, 0.388 m/s, 0.388 m/s, 0.0015 rad/s]`, from
15 m/s on a straight road. Every subsequent optimization attempt is forced
to fail, for a total of 1,200 held inputs over 60 s. Current measurement
enclosures vary; the carried reachable set supplies the fallback. This is
a declared-model experiment, not a nonlinear-plant or NRMM experiment.

The remaining limitations are substantive:

- Persistent velocity forcing need not decay and can produce unbounded
  position drift. It remains rejected; a measured physical residual is not
  silently set to zero.
- The terminal theorem uses the zero-speed affine scheduled model. Its
  passive damping is not a verified low-speed model of the 14-DOF plant.
- Finite prediction-node safety is the existing head/tail scope. The new
  terminal inequalities cover continuous terminal time but do not close the
  between-node gap before terminal entry.
- NRMM course-based estimation is not thereby certified at standstill.
- Speed/acceleration bounds alone allow a moving target to reach any fixed
  stopping location eventually. Complete-future target support remains
  infinite under that contract. A route/occupancy premise must be supplied
  and justified before claiming indefinite safety from such a target.
- Generic curved-polyline uncertainty still requires a sound invertible
  chart or a separately implemented smooth-path representation.

These limits distinguish successful uncertain-velocity terminal admission
from completion of the full physical estimator/controller experiment.

## Relation to primary research

[Köhler, Müller and Allgöwer (2021)](https://arxiv.org/abs/2105.03427)
distinguish online estimation bounds from the propagation and terminal
conditions needed by robust output-feedback MPC. Their framework motivates
keeping those obligations separate here; this implementation does not import
their nonlinear tube theorem. [Lorenzetti and Pavone
(2019, revised 2020)](https://arxiv.org/abs/1911.07360) construct invariant
uncertainty sets for a different output-feedback tube architecture. Their
abstract supports the architecture comparison, not the vehicle-specific
derivation (1)-(2), which is given explicitly above. Neither source proves
the physical vehicle or the current NRMM integration safe.
