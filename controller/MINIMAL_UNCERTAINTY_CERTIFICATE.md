# Minimal uncertainty extension of the maintained SOCP certificate

Implementation update (2026-09-07):
[DISSIPATIVE_TERMINAL_CERTIFICATE.md](DISSIPATIVE_TERMINAL_CERTIFICATE.md)
supersedes the velocity-uncertainty limitation below by replacing exact rest
with an invariant dissipative funnel under the declared affine model. This
file preserves the earlier stationary-pose design and its finite-time rest
obstruction; persistent forcing and the wider physical integration remain open.

Status (2026-09-05): implemented stationary-pose uncertainty, set-membership
updates, finite-horizon disturbance propagation and robust affine physical
rows. The full six-state NRMM closed loop remains **incomplete**. In particular,
the unchanged rest policy cannot admit uncertain terminal velocity or
persistent drift. This limitation is checked, not replaced by a simulation
assumption or hidden relaxation of the nonnegative-speed constraint.

The optimization still has the same two inputs per stage, the same scheduled
affine bicycle, one SOCP, hard collision/road/actuator/tire constraints, the
same nominal terminal equalities, and one performance CLF slack. There is no
new ancillary gain, optimized tube, future-observer decay assumption, or
change of objective. Certificate version 5 additionally stores reachable
state boxes, the stationary-pose certificate and the sample timestamp.

## State enclosures and propagation

Let the declared continuous model on held stage j be

\[
 \dot x=A^c_jx+B^c_ju+c^c_j+w(t),\qquad |w(t)|\le\bar w,
 \qquad x_j\in z_j+[-r_j,r_j].
\]

The input applied to every member of the reachable set is the stored input.
The exact nominal held flow gives matrices A_j, B_j, c_j already used by
the controller. Therefore

\[
 r_{j+1}=|A_j|r_j+d_j,\qquad
 d_j\ge\int_0^{T_s}|e^{A^c_j\tau}|\bar w\,d\tau. \tag{1}
\]

`stateUncertainty.heldDisturbance` constructs the Metzler comparison matrix M with
$M_{ii}=A^c_{ii}$ and $M_{ij}=|A^c_{ij}|$ for $i\ne j$. The componentwise
comparison inequality $|\exp(A^c\tau)|\le\exp(M\tau)$ proves (1) with

\[
 d_j=\left[\exp\left(T_s
 \begin{bmatrix}M_j&\bar w\\0&0\end{bmatrix}\right)\right]_{1:6,7}.
\]

The two configuration rate vectors now mean **continuous derivative-error
bounds**, in the Frenet state order [s,d,ePsi,vx,vy,r], with units
[m/s,m/s,rad/s,m/s^2,m/s^2,rad/s^2]. Their sum is used.
For a longitudinal double integrator forced by |w_v| <= a, a held interval
already contributes [a Ts^2/2, a Ts] to position and speed. The previous
Ts*w update missed the position term. Nonzero rate vectors can be propagated
and studied, but are still rejected by the current infinite continuation.

`stateUncertainty.toFrenet` encloses the Cartesian estimation box under the
polyline projection. If its Euclidean position radius is R and D_i is the
center's distance to segment i, every possible closest segment obeys
D_i <= min(D)+2R, by the triangle inequality. For each such segment the
implementation bounds clipped station, signed lateral distance and the
change in segment heading. It includes yaw wrapping conservatively.

A projection enclosure is weaker than an invertible Cartesian chart. At a
clipped endpoint, along-segment displacement is lost; at a vertex the inverse
can select a different segment. Uncertain initial admission consequently
requires the entire initial position box to lie in one strictly interior
projection chart. This is sufficient and conservative. The exact legacy
input path is retained. Supporting arbitrary uncertain chart transitions
would require a stronger coordinate-set representation.

## Hard constraints

For any affine physical row $H_xx+H_uu\le h$ and the stored open-loop input,
the implemented robust row is

\[
 H_xz+H_uu+|H_x|r\le h. \tag{2}
\]

This is sufficient because $H_x(x-z)\le |H_x|r$. It does not assume independent
errors. Domain limits already use this tightening. The extension applies it
also to every tire-slip domain row. The condensed acceptance
rows and sparse SOCP receive the same charged offsets. No input correction
is applied outside these rows; adding feedback later would require its own
state-input support calculation.

Collision and road rows use the existing fixed-normal support construction:
directional position-box support, rectangle support maximized over heading
uncertainty, and the frame/road error allowances are all included. Terminal
target support still covers the complete allowed future center trajectory,
with a target circumradius for arbitrary future heading. Current bounds plus
speed/acceleration domains alone do not give such a finite future halfspace.
No new target-motion premise is inferred from a current estimator bound.

## The smallest invariant terminal extension

Retain the existing nominal rest endpoint
$z_R=[s_R,d_R,e_{\psi,R},0,0,0]^\top$ and policy $u_R=[0,-b/g_\beta]^\top$.
At zero schedule
speed, every such admitted pose is an equilibrium of the declared model.
Replace its singleton by

\[
 \mathcal X_R=z_R+[-r_R,r_R],\qquad (r_R)_{4:6}=0. \tag{3}
\]

`stateUncertainty.terminalRest` checks the zero-speed schedule, exactly
zero velocity radii, and $|A_R|r_R+d_R\le r_R$. The controller also rejects
every nonzero configured persistent rate bound explicitly, avoiding a tiny
positive forcing being rounded into apparent invariance. For the supported
rest model, $A_R$ acts as the identity on pose perturbations and $d_R=0$.
Thus every point in (3) remains fixed under $u_R$. The existing terminal
geometry, domain and actuator rows certify this whole stationary set.

This is an invariant **set of poses**, not a claim that uncertain velocities
have stopped. A straight-road moving initial pose box with exact velocity
channels can reach (3): pose errors affect predicted positions, but do not
excite the velocity errors in that scheduled straight-road affine model.
The same absence of pose-to-velocity coupling holds on a curved schedule;
chart admission and the terminal set-containment check remain independent
requirements.

The obstruction for general velocity uncertainty remains with road load.
On a straight scheduled road, two velocities differing initially by
$2\epsilon$ under the same held beta input have a difference
$2\epsilon\exp(-\sum_j F'_{\rm road}(\bar v_j)T_s/m)$ after finitely many
stages. This difference is nonzero; both cannot reach exact zero. A symmetric
interval about nominal zero also violates the hard nonnegative-speed domain.

A physical braking-and-hold mechanism, a different certified terminal policy,
or a change of the modeled velocity domain is needed to admit that broader
case. No such premise has been enabled here. This is the outstanding design
decision for completing the requested NRMM robustness extension.

## New estimates and recursive continuation

Assume an already admitted controller state, the same model/chart/environment
contracts, execution of the stored input and exactly one sample elapsed.
Let P = z^- + [-r^-,r^-] be its next reachable box and let the new estimator
publish E = xhat + [-b,b]. Both must contain the same true state. Consequently
x belongs to P intersect E without any assumption about how a future
estimator bound decays or grows.

With delta=xhat-z^-, `stateUncertainty.intersect` computes

\[
 l=\max(-r^-,\delta-b),\quad u=\min(r^-,\delta+b),\quad
 r^{new}=\min(r^-,\max(|l|,|u|)). \tag{4}
\]

If l <= u, (4) gives
P intersect E subset z^-+[-rNew,rNew] subset P. The nominal center is retained;
the uncertainty is never reset to zero merely because the estimate changed.
A larger current measurement box can coexist with a tighter carried reachable
box. An empty intersection raises `inconsistentStateEnclosures` and issues
no command. It means at least one stated contract is false; a solver status
does not resolve it. Timestamp mismatch or changed target/environment
contracts instead follow explicit readmission, with no automatic recursive
guarantee across that change.

Conditional theorem in exact arithmetic: assume initial feasible admission,
sound current estimation sets in the controller chart, the declared affine
dynamics and exact actuator execution, zero persistent model disturbance,
the stationary-pose terminal certificate, applicable unchanged road coverage,
and target finite-horizon shift consistency with contained complete-future
support. Then the accepted controller maintains the represented hard
constraints at prediction nodes, and its shifted stored plan remains a
feasible fallback at each subsequent compatible sample.

Proof: (1) encloses all true states; (2) and the geometric supports imply
node constraint satisfaction. Equation (4) starts the next prediction inside
the old reachable box at the same nominal center. Monotonicity of |A_j| and
identical shifted dynamics give containment at every overlapping node. Old
supporting geometry therefore remains valid. Replacement geometry is used
only when it admits the witness; otherwise the contained old certificate is
retained. Equation (3) permits an appended rest step with the same box,
policy and future-target separation. The unbounded nonnegative CLF slack
admits the shifted inputs. Independent acceptance checks select either a
new feasible plan or that witness, without a second optimization. Induction
establishes the stated claim.

This theorem is conditional on valid current bounds; it does not establish
NRMM validity at zero speed. It covers prediction nodes and represented road
boundaries, not continuous-time collision avoidance, arbitrary dropouts,
changed target-motion contracts, nonlinear tire physics or actuator delay.
The SOCP and certificate calculations use ordinary floating point and the
existing acceptance tolerances. Numerical near-zero terminal velocities are
reported residuals, not interval-certified infinite-time rest.

## Reproducible validation

`uncertaintyGeometryTest`, `ltvBicyclePredictionTest` and
`robustStationaryPoseCertificateTest` cover projection changes, integrated
forcing, stationary-set admission, changing observation centers/radii,
empty intersections, rejection of a 1e-14 m/s velocity radius and persistent
drift, and all 64 vertices of a six-state box against the robust slip-domain
rows. Existing sparse-lift and current-estimator-bound tests also run.

`runRobustPoseCertificateScenario` uses all eight vertices of a pose box
[0.1 m, 0.1 m, 0.001 rad], exact velocity channels, 5 m/s initial nominal
speed, Ts=0.05 s and four head stages. One accepted SOCP is followed by
deliberate solver failures through three steps beyond the 78-stage plan.
The deterministic 81-sample run measured maximum enclosure excess
6.94e-17, minimum rectangle clearance 39.9836 m (required 0.25 m), and
maximum final velocity-channel magnitude 1.28e-12. No random seed is used.
The far stationary target makes this a containment/continuation regression,
not a challenging obstacle-maneuver or full NRMM experiment.
