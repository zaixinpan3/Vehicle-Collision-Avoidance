# NRMM-based time-dependent VFFM trajectory initialization

Implementation: September 22, 2026. This specifies the initial trajectory
generator feeding the existing fixed-normal SOCP. Its output is a model-consistent
optimization seed. A fresh executable plan still requires a successful full
trajectory optimization and the original independent hard verifier.

## Sources and scope

| Source | Used here | Project extension |
| --- | --- | --- |
| Sharma, Alai and Rajamani (2026), *Simultaneous ego-vehicle state estimation and vehicle trajectory tracking using a multistage high gain observer*, [DOI](https://doi.org/10.1016/j.trc.2025.105411), Sec. 2.3, Eqs. (16)–(17) | Relative positions/heading, absolute target speed, acceleration and sideslip | Fixed-frame closed-form propagation and its numerical evaluation |
| Cheng, Li, Liu, Li and Guo (2021), *Virtual Fluid-Flow-Model-Based Lane-Keeping Integrated With Collision Avoidance Control System Design for Autonomous Vehicles*, [DOI](https://doi.org/10.1109/TITS.2020.2990211), Eqs. (22)–(25), (34)–(36) | Road-channel reference and modified-function extremum rule | Moving Gaussian centers, quadratic-road coordinates, simultaneous-time evaluation and model fitting |
| Li, Zhang, Guo, Lenzo and Guo (2023), *Real-Time Optimal Trajectory Planning for Autonomous Driving with Collision Avoidance Using Convex Optimization*, [DOI](https://doi.org/10.1007/s42154-023-00222-7), Sec. 3, Eqs. (9), (12)–(13) | Separating geometry followed by convex trajectory optimization | Existing rectangle support directions, hard certificate and complete-input SOCP |

The local publisher PDFs in `reference/` were examined. These papers do not
derive this combined method. Cheng's Gaussian follows by differentiating the
modified scalar in Eq. (36); its second derivative is positive, so the chosen
point is a minimum. It is a reference-selection function. Neither the Gaussian
nor the quadratic-road streamfunction below is claimed to solve a moving-domain
Navier–Stokes boundary-value problem. Li's Eq. (13) includes collision slack;
this controller retains hard collision rows and only its existing soft CLF.

The current study considers zero or one obstacle vehicle. Both the controller
input parser and the VFFM reference interfaces reject multiple target records
with `collisionAvoidanceController:unsupportedTargetCount`; no nearest-target
selection or silent target omission is performed. Left/right reference
candidates describe alternative maneuvers around the same obstacle.

## 1. Analytical target motion in the fixed planning frame

For the single target, with constant speed derivative $A$ and sideslip $\beta$, set

\[
\kappa=\frac{\sin\beta}{l_r},\quad
\xi(t)=V_0t+\tfrac12At^2,\quad
\gamma_0=\psi_0+\beta.
\]

The NRMM's ego translation and rotation terms cancel on transforming relative
positions back into the fixed frame. Consequently,

\[
V=V_0+At,\quad \psi=\psi_0+\kappa\xi,\quad
p=p_0+\xi\,\operatorname{sinc}_0(\kappa\xi/2)
  \begin{bmatrix}\cos(\gamma_0+\kappa\xi/2)\\\sin(\gamma_0+\kappa\xi/2)\end{bmatrix}.
\]

Here $\operatorname{sinc}_0(z)=\sin(z)/z$ with value one at zero; it is not
MATLAB's normalized `sinc(z)`. The shared implementation uses a small-angle
series, retaining accumulated turning even for very small curvature.
With $e=(\cos\gamma,\sin\gamma)^T$ and $S$ the 90-degree rotation,

\[
\dot p=Ve,\quad \ddot p=Ae+\kappa V^2Se,\quad
\dddot p=-\kappa^2V^3e+3\kappa AVSe.
\]

`targetPrediction.nrmmFlow(initial,lr,time,stopPolicy)` accepts
`initial=[X;Y;V;A;psi;beta]` and returns
`[X;Y;vx;vy;ax;ay;psi;yawRate]` plus Cartesian jerk. The default rejects
queries beyond a braking stop. Explicit `"hold"` freezes pose after the stop;
derivatives there use the right-sided stopped branch. The acceleration jump at
that transition has no finite classical jerk; returned jerk applies within
each smooth branch. Known sideslip and axle distance support launch from rest.

The online Cartesian interface uses `targetPrediction.nominalFlow` with
$V_0=\|v_0\|$, $A=v_0^Ta_0/V_0$, $\kappa=r_0/V_0$ and
$\gamma_0=\operatorname{atan2}(v_{y0},v_{x0})$. These equal the NRMM quantities
for consistent nonzero-speed inputs. This existing nominal interface keeps its
explicit stop-and-hold behavior and optional scalar-acceleration limit. At
exact rest it chooses a straight launch along the acceleration vector; the
Cartesian interface cannot identify NRMM curvature there. Use `nrmmFlow` when
that distinction matters. No target uncertainty or safety motion bound is
removed: `finiteFlow` still defines the hard prediction enclosures.

## 2. Quadratic-road coordinates

The selected smooth route is $c(s)$, with tangent $t$, normal $n=St$ and
curvature $k$. For each route station, intersect $c(s)+dn(s)$ with the finite
quadratic boundaries. In a boundary's orthonormal frame,
$x=x_0+n_xd$, $y=y_0+n_yd$ and $y=ax^2+bx+c_b$, giving

\[
-a n_x^2d^2+[n_y-(2ax_0+b)n_x]d+y_0-ax_0^2-bx_0-c_b=0.
\]

Solve the quadratic stably (or its linear degeneration), retain roots inside
the boundary's parameter interval, and select the connected admissible
normal interval containing $d=0$. Safe-side signs determine which roots are
lower or upper endpoints. Tangencies do not delimit an open component.
For finite endpoints $d_-,d_+$, define

\[
m=(d_++d_-)/2,\quad h=(d_+-d_-)/2,\quad
F(s,\eta)=c(s)+(m+h\eta)n(s),\quad J_F=h[1-k(m+h\eta)].
\]

`laneGeometry.normalRoadChart` computes $m,h$ and their first two station
derivatives by implicit differentiation. If $G(c+dn)=0$, $v=(1-kd)t+d'n$,
then

\[
d'=-\frac{\nabla G^T(1-kd)t}{\nabla G^Tn},\qquad
d''=-\frac{v^T\nabla^2Gv+\nabla G^T[(-k'd-2kd')t+k(1-kd)n]}{\nabla G^Tn}.
\]

Regularity requires positive width and $1-kd>0$. On a straight $X$-axis route,
this gives exactly the midpoint and vertical half-gap of the two quadratic
graphs. Rotating the input frame leaves the normal-coordinate result unchanged.
Circular projection honors the station hint to avoid an arbitrary revolution.

When no finite pair bounds a section, the preference uses $m=0,h=1$ m and
reports `bounded=false`; this scale does not invent a physical road width.
All supplied road inequalities remain in the final optimizer.

The streamfunction $\Psi=Q(1/2+3\eta/4-\eta^3/4)$ gives the compatible
road template $u=Qf'(\eta)F_s/J_F$. The implementation evaluates the resulting
road coordinates and preferred reference directly; it does not integrate a
fluid PDE or its instantaneous streamlines.

## 3. Single-obstacle passing choices and timing

Roll out the current nominal control sequence through the same affine model
used by the optimizer. Its nodes supply progress $s_E(t_k)$ and one-sided
held-flow derivatives $\dot s_E,\ddot s_E$. If there is no target or no positive
reserved collision-support residual at any nominal node, retain the nominal
seed. A present nonconflicting target still keeps its hard constraints.
For a conflicting target, let $K$ be the first-to-last conflicting node
interval, and project its analytical poses into the same route chart to
obtain $s_o(t),d_o(t)$. Its Gaussian length is

\[
\ell=w_\ell\max\left(\ell_{\min},
\frac{\max_{k\in K}(s_{E,k}-s_{o,k})-\min_{k\in K}(s_{E,k}-s_{o,k})}{2}\right).
\]

The default $w_\ell=1.2$ and $\ell_{\min}=3$ m remain design choices. Relative
station, rather than just ego travel, accounts for an approaching or receding
target when selecting temporal extent.

At the middle conflicting node $t_m$, choose the two passing ordinates

\[
a^\sigma=d_o(t_m)+\sigma D,\qquad \sigma\in\{-1,+1\},\qquad
D=W_E/2+h_{Q_o}(R_o^Tn_o)+d_{\mathrm{shape}}.
\]

$d_{\mathrm{shape}}=0.2$ m is a reference-shape allowance, not a hard physical
clearance. The footprint orientation in the support term is target body yaw.
Order the first side against target lateral travel; a tie orders away
from its lateral center, then positive lateral if centered. The second
candidate uses the opposite side of the same target.

The chosen design parameterization has one Gaussian term:

\[
b^\sigma(t)=\frac{a^\sigma-m(s_o(t))}{h(s_o(t))},\quad
g^\sigma(s,t)=b^\sigma(t)\exp[-(s-s_o(t))^2/(2\ell^2)].
\]

It follows from $\partial_\eta(\eta^2/2-\eta g)=0$ that $\eta_r=g$.
The time-consistent preferred position is

\[
p_r^\sigma(t)=F(s_E(t),g^\sigma(s_E(t),t)).
\]

The passing ordinate is frozen from the predicted conflict, while the Gaussian
center and chart normalization evolve with the target. This is an explicit
design decision within the supplied formulation. Directly making the passing
ordinate follow $d_o(t)$ caused the first implementation's crossing references
to move with the transverse obstacle and fail existing admission regressions.
The supplied time-varying-amplitude example is not a feasibility theorem.
A stationary target on a straight, constant-width road recovers Cheng's Gaussian.

## 4. Derivatives and body-heading preference

For $q=s_E-s_o$, $E=\exp[-q^2/(2\ell^2)]$ and
$\lambda=-q\dot q/\ell^2$,

\[
\dot\lambda=-(\dot q^2+q\ddot q)/\ell^2,\quad
z=bE,\quad
\dot z=(\dot b+b\lambda)E,
\]
\[
\ddot z=[\ddot b+2\dot b\lambda+b(\lambda^2+\dot\lambda)]E.
\]

For $d=m(s_E)+h(s_E)z$, retain all midpoint/width derivatives:

\[
\dot d=(m'+h'z)\dot s_E+h\dot z,
\quad
\ddot d=(m''+h''z)\dot s_E^2+(m'+h'z)\ddot s_E+2h'\dot s_E\dot z+h\ddot z.
\]

The Cartesian reference velocity and acceleration are

\[
\dot p_r=(1-kd)\dot s_Et+\dot dn,
\]
\[
\ddot p_r=[(1-kd)\ddot s_E-k'd\dot s_E^2-2k\dot s_E\dot d]t
 +[k(1-kd)\dot s_E^2+\ddot d]n.
\]

Target station derivatives come from its analytical Cartesian velocity and
acceleration, with $r_o=1-k_o d_o$:

\[
\dot s_o=t_o^T\dot p_o/r_o,\quad \dot d_o=n_o^T\dot p_o,\quad
\ddot s_o=[t_o^T\ddot p_o+2k_o\dot s_o\dot d_o+k_o'd_o\dot s_o^2]/r_o.
\]

Analytical quotient derivatives of $b=(a-m_o)/h_o$ retain $\dot h_o$,
$\ddot h_o$, $\dot m_o$ and $\ddot m_o$. No constant-relative-speed
approximation is used. Derivatives are local to smooth road pieces and smooth
motion branches; knots, hold changes and a braking stop need one-sided values.

The reference course offset is
$\chi_r=\operatorname{atan2}(\dot d,(1-kd)\dot s_E)$.
The body-yaw preference is $\psi_{r,\mathrm{rel}}=\chi_r-eta_E^{\mathrm{anchor}}$,
where $\beta_E^{\mathrm{anchor}}=\operatorname{atan2}(v_E,U_E)$.
This subtraction matters on curved trim trajectories. The reference also
reports $V_r$, curvature and $a_{n,r}=V_r^2\kappa_r$ from Cartesian derivatives.
These diagnostics are not actuator feasibility checks.

## 5. Obtain the initial state and input trajectory

Fit lateral-position and body-yaw preferences at nodes 1 through $N$ to the
affine predicted states $x_k=c_k+M_kU$. For both passing assignments, solve

\[
\min_{\Delta U}\|M\Delta U-e^\sigma\|^2+lambda\Delta U^TR\Delta U,
\qquad C\Delta U=0.
\]

$C$ preserves the nominal final six-state vector and final two inputs. The
initial state is fixed by the prediction itself. $M$ includes lateral/heading
weights 1 and 4; $R$ retains actuator effort and input-difference penalties,
with default $\lambda=0.02$. One positive-definite factorization and a small
Schur-complement pseudoinverse handle the two right-hand sides. Reject a
nonfinite solution, an irregular chart, or terminal equality error above
$10^{-8}$. The fitted rollout, not the raw reference, is the initial trajectory
$(x_k^{(0)},U_k^{(0)})$ used to construct geometry.

Complete the existing allowed CLF slack and evaluate physical base-row
feasibility first. Once a finite-scored physically admissible fit has been
selected, skip a later physically inadmissible fit: it cannot win the existing
ranking. Its diagnostic support score remains `Inf` because it was not evaluated.
Otherwise, for every collision and exit record, query the analytic rectangle
signed-distance normal at that fitted pose. It remains defined at overlap,
unlike normalization of a zero ordinary-distance dual vector.
Prefer a fit satisfying all physical base rows
(including actuator, road and chart rows); within the same base-row feasibility
class, minimize the worst reserved support residual. This lexicographic rule
avoids choosing a slightly smaller collision residual at the cost of a known
base-constraint violation. It is still a heuristic for selecting a convex
inner problem, not proof of feasibility.

At fresh joint admission, geometry assembly keeps the occupied sets, normals,
road and pose rows but omits projection of preliminary collision rows that
the joint representation would immediately replace. Other geometry callers
retain full collision rows by default. The final fixed-direction SOCP and
independent support verifier still enforce every retained collision record.

Fix the selected directions and optimize the complete input sequence once.
Only `solveHardCbfClf.certify` accepting the optimized result can authorize a
fresh command. A failed fresh solve issues no seed. Previously optimized,
verified continuation retains its existing policy and does not regenerate
these references. `program.fluidReference` stores the numeric preparation
used by both MATLAB and the prepared-frame native adapter.

To inspect the initial trajectory after parsing and preparing `model`:

```matlab
program = formulateAvoidanceProblem(model);
[seed, angles, information] = solveHardCbfClf.fluidInitialize(program, model.cfg);
assert(~isempty(seed), "No usable reference fit was obtained.");
plan = seed(program.layout.planIndex);
initialInputs = reshape(plan, 2, []);
initialStates = program.prediction.egoStateOffset + ...
    reshape(pagemtimes(program.prediction.egoStateMatrix, plan), 6, []);
```

`initialStates` includes the fixed measured node at time zero. These arrays
are for initializing optimization and inspecting its geometry; they are not
an issued control plan.

## Boundaries of this reconstruction

The geometric initializer can evaluate supplied quadratic boundaries. The
current end-to-end terminal certificates reject nonempty physical road-boundary
sets; this scope change does not remove that limitation. See the
[validation rerun](../report/CONTROLLER_VALIDATION_RERUN_20260922.md).

- The implementation reconstructs initialization, not the entire nonlinear
  optimal-control problem in the supplied text. It retains the existing
  six-state Frenet affine Fiala model and held steering/braking inputs rather
  than replacing them with the text's seven-state linear-tire model.
- Quadratic boundaries describe the selected route corridor. A union of road
  patches through an intersection needs an explicit union-domain containment
  interface and transitions between regular charts. This change does not
  claim that existing intersected boundary rows implement that union.
- The current hard certificate checks prediction nodes. Neither a smooth
  reference, positive solver status nor these tests supplies continuous-time
  separation, a physical nonlinear-vehicle guarantee or a real-time deadline
  guarantee. Inter-sample margins from the supplied text are not silently
  assumed to exist in the current controller.
- Only one obstacle is in scope. Its two passing candidates need not include
  a feasible maneuver and may leave the road. Road footprint, collision,
  actuator and terminal constraints belong to the subsequent optimization and
  verifier. Local rejection is not global infeasibility.

Validation and actual results are recorded in
[the implementation report](../report/NRMM_VFFM_INITIALIZATION_20260922.md).
