# Controller mathematical theory

This is the current controller theory and source map. It records the current core algorithms and their mathematical derivation,
with a single statement of the implementation boundary. Repository paths are relative to the
repository root unless linked otherwise. Historical experiment reports describe
the versions that produced them; they are not rewritten by this consolidation.

The design has one nonlinear mathematical problem and a local numerical
implementation of that problem. The online default is `solver.method="rti"`.
An explicitly selected `solver.method="nonlinear"` is an iterated shooting
diagnostic, not a second safety definition. Both use the same continuous
separating-plane witnesses, modified Fiala transition and simplified terminal
function. The theoretical encounter controller minimizes only PCBF slack and
executes the first command of any attained minimizer. Both numerical backends
add an optional performance refinement under the attained safety cap; this
second solve belongs to implementation, not to the theoretical control law.
No world-angle grid defines collision avoidance. The geometric and
recursive-feasibility derivations are in Sections 3 and 7; numerical
approximations are identified separately.

The physical sensor contract remains position-only GNSS, IMU acceleration,
gyro and relative radar position. The observer and future feedback are part of
the joint prediction, with one copy of each persistent model parameter and
measurement-noise primitive. Bounded current error alone is not a proof of
sound future prediction or measurement-consistent shifting.

The [compact-certificate construction](#compact-certificate) below derives a
lower-dimensional alternative to simulating every future observer state and
sensor history. It uses sampled observer comparisons and nonlinear relative
physical tubes. This is a theoretical reduction with explicit admission and
shift conditions; it has not replaced the online joint-affine predictor.

The implemented active encounter minimizes the sum of collision slacks first,
then tracking and input variation without spending the attained primary budget.
Without a target, the integrated continuous-time CLF supplies a soft cruise condition.
The planner initializes the state, input and geometry at encounter onset. It
does not define the safety function or restrict the optimizer to its passing
side. The accepted horizon stays fixed throughout the encounter. Later calls
shift both the trajectory and its continuous plane witnesses. There is no
within-frame replanning, horizon-extension retry, or post-solve nonlinear
execution veto in RTI.

| Layer | Mathematical object and scope |
| --- | --- |
| Nonlinear design | PCBF slack minimization and execution of any attained minimizer's first command, with causal joint observer/controller policies, sound reachable information sets, continuous separating witnesses, hard admissibility and a shift-compatible terminal family. Theorem U gives conditional recursive feasibility and zero-deficit invariance without a secondary optimization. |
| Compact nonlinear certificate design | Sampled observer-error comparisons coupled to relative physical tubes, with inherited certificate coordinates and nested measurement updates. Theorem C specializes the shift proof to this outer representation; full-domain coupling constants and representable terminal closure must be supplied before deployment. |
| Engineering nonlinear transcription | Nonlinear Fiala motion and plane/terminal constraints with a frame-frozen joint affine uncertainty prediction. Accepted shooting updates may refresh that prediction. These margins are not certified nonlinear reachable sets. |
| Online SQP-RTI | One linearization of that transcription, including its plane variables and terminal function, with a Gauss--Newton performance Hessian. One primary LP and at most one secondary QP share the same local model. Convex solver iterations are not additional SQP linearizations. |
| Optional retained-certificate research mode | Full-domain joint enclosures and retained policy-box/terminal certificates. Its admission restrictions in Section 8 are separate from ordinary RTI. |

The defaults are `retainedTube=false`, `jointAffinePrediction=true` and
`jointNonlinearPrediction=false`. `robustNonlinearSafetyCertified` remains
false in both ordinary numerical backends. The strict theorem is conditional
on sound nonlinear joint images, consistent conditioning, a compatible
terminal append and the prescribed solution selection. Neither an NLP solver
return nor an affine zero slack establishes these conditions.

Geometry needs only two witness variables per checked instant: an angle
$\theta$ and a plane offset $b$. The exact parametrization
$n(\theta)=(\cos\theta,\sin\theta)$ prevents the zero-normal degeneracy at zero
physical clearance. Four ego-vertex and four target-vertex inequalities replace
the sampled directional maximum. Their derivatives are shared by shooting
and RTI. The nominal vertex inequalities are smooth. The engineering
uncertainty support oracle and terminal maximum are piecewise differentiable;
at feature changes the implementation uses selected/generalized derivatives.
A smooth RTI convergence theorem does not automatically apply at such ties.

Tracking and applied-input variation are both horizon sums in both backends;
optional reference proximity remains a horizon mean and defaults to zero.
There is no large penalty weight trading safety for tracking. The nonlinear
diagnostic additionally uses feasibility restoration, line search and margin
refresh, whereas RTI takes a single local update. These numerical differences
must remain visible in comparisons.

All safety statements below concern the declared checked instants unless an
explicit whole-hold enclosure is supplied. Zero signed clearance permits
contact and excludes interior overlap; strictly positive clearance is a
stronger statement. No fixed 0.2 m design buffer is introduced. Roads remain
a heuristic planner input, not a constraint in the default optimization.
The theoretical terminal assumption is the existence of an admissible,
zero-deficit, shift-closed terminal family and causal controller for the same
joint dynamics and constraints. The core proof does not construct this pair.
The simplified braking model in Section 5 is an implementation candidate;
its design and validation are separate from the abstract assumption (U4).
Terminal membership represents an available continuation, not a normal
driving objective.

## Contents

1. [Vehicle model and target prediction](#vehicle-model)
2. [Observer bounds and relative geometry](#observer-geometry)
3. [Collision constraints and the switched problem](#optimization)
4. [One nominal CLF and recovery](#nominal-clf)
5. [Joint terminal set and braking continuation](#terminal-set)
6. [Joint feedback-aware prediction](#joint-prediction)
7. [Unified observer-to-safety formulation](#recursive-feasibility), including
   [compact sampled certificates](#compact-certificate)
8. [Numerical realization and certificate boundary](#runtime)
9. [Source map and verification](#source-map)
10. [Sources](#sources)

<a id="vehicle-model"></a>
## 1. Vehicle model and target prediction

### Coordinates, inputs and tire forces

Use the physical ego state and held actuator input

$$
x=(p_x,p_y,\psi,v_x,v_y,r)^\top,\qquad u=(\delta,b)^\top,
\qquad t_k=kh.
$$

Positions are in meters, angles in radians, speeds in m/s, yaw rate in rad/s
and forces in N. The signed longitudinal-force ratio satisfies $-1\le b\le1$.
The default control hold is $h=0.05$ s. With wheelbase $L=l_f+l_r$, the static
axle loads and the force capacity remaining for lateral motion are

$$
F_{zf}=mg\frac{l_r}{L},\quad F_{zr}=mg\frac{l_f}{L},\quad
F_{xi}=b\mu_i F_{zi},\quad D_i=\mu_iF_{zi}\sqrt{1-b^2}.
$$

The tire slip angles use the code's force sign convention:

$$
\alpha_f=\operatorname{atan2}(v_y+l_fr,v_x)-\delta,\qquad
\alpha_r=\operatorname{atan2}(v_y-l_rr,v_x).
$$

For $D_i>0$, let $q_i=C_i|\tan\alpha_i|/(3D_i)$. The modified Fiala force is

$$
F_{yi}=\begin{cases}
-C_i\tan\alpha_i(1-q_i+q_i^2/3),&q_i<1,\\
-D_i\operatorname{sign}(\alpha_i),&q_i\ge1.
\end{cases}
$$

When $D_i=0$, lateral force is zero. The force remains defined at $|b|=1$,
but its joint input tangent is singular there. Only linearization references
are moved ten constraint tolerances inside these endpoints; physical actuator
bounds and already issued commands retain the full interval. Tire saturation
is part of the model. No extra rear-tire nonsaturation cone is imposed by the
optimizer or terminal membership test.

### Dynamic and low-speed equations

Rotate the front axle forces into the body frame:

$$
F_{xf}^B=F_{xf}\cos\delta-F_{yf}\sin\delta,\qquad
F_{yf}^B=F_{xf}\sin\delta+F_{yf}\cos\delta.
$$

Above the low-speed blend, the bicycle equations are

$$
\begin{aligned}
\dot p&=R(\psi)[v_x,v_y]^\top,&\dot\psi&=r,\\
\dot v_x&=(F_{xf}^B+F_{xr}-F_{\rm load}(v_x))/m+v_yr,\\
\dot v_y&=(F_{yf}^B+F_{yr})/m-v_xr,\\
\dot r&=(l_fF_{yf}^B-l_rF_{yr})/I_z.
\end{aligned}
$$

The passive road load is aerodynamic drag plus smooth rolling resistance:

$$
F_{\rm load}(v)=\tfrac12\rho_a C_d A_f v|v|
+mg(c_0+c_1|v|+c_4|v|^4)\tanh(v/v_\epsilon).
$$

At low speed define $r_{\rm kin}=v_x\tan\delta/L$,
$F_{x,\rm kin}=(\max(b,0)+\min(b,0)\tanh(v_x/v_b))
(\mu_fF_{zf}\cos\delta+\mu_rF_{zr})$. The low-speed body map is

$$
\dot v_x=(F_{x,\rm kin}-F_{\rm load}(v_x))/m+v_yr,\quad
\dot v_y=(l_rr_{\rm kin}-v_y)/\tau_\ell,\quad
\dot r=(r_{\rm kin}-r)/\tau_\ell.
$$

Blend the three body derivatives using $w=s^2(3-2s)$ and
$s=\operatorname{clip}((v_x-v_\ell)/(v_h-v_\ell),0,1)$, with default
$[v_\ell,v_h]=[2.5,4]$ m/s and $\tau_\ell=0.1$ s. The position/yaw equations
remain the same. The sampled map $F_h(x,u)$ is RK4 with variational tangents.
Under nonpositive $b$, a sample-end speed below $0.2v_b$ resets
$(v_x,v_y,r)$ to zero; default $v_b=0.25$ m/s. This numerical reset differs
from the exact within-hold stop used by the terminal model in Section 5.
A proof for a continuous physical plant must also enclose integration and
reset discrepancies.

The body sideslip $\beta_E=\operatorname{atan2}(v_y,v_x)$ is the angle of
the center-of-gravity velocity, not either tire slip angle $\alpha_f,\alpha_r$.
In low-speed turning, $v_y\simeq l_r r$ can give nonzero body sideslip while
the rear tire slip is near zero. Consequently, a large ratio $|v_y|/v_x$ at
low speed does not by itself establish tire sliding or physical infeasibility.
The shared observer cone is an estimation-domain premise. Enlarging it changes
that premise and the recomputed error bounds, rather than increasing tire
capacity. The low-speed blend and its response lag remain model assumptions,
not an experimental validation of the corresponding real-vehicle motion.

On a curved path with phase $s_p$, lateral displacement $e_y$, heading error
$e_\psi$ and curvature $\kappa_p$, the path chart satisfies

$$
\dot s_p=\frac{v_x\cos e_\psi-v_y\sin e_\psi}{1-\kappa_pe_y},\quad
\dot e_y=v_x\sin e_\psi+v_y\cos e_\psi,\quad
\dot e_\psi=r-\kappa_p\dot s_p.
$$

The chart requires $1-\kappa_pe_y>0$. Longitudinal phase is free in the CLF.

### Target flow and encounter memory

The target has constant tangential acceleration $A_C$ and sideslip $\beta_C$
within one forecast. With $\kappa_C=\sin\beta_C/l_{rC}$, its positive-speed flow is

$$
\dot p_C=V_C[\cos(\psi_C+\beta_C),\sin(\psi_C+\beta_C)]^\top,\quad
\dot\psi_C=\kappa_CV_C,\quad \dot V_C=A_C.
$$

For $A_C<0$, integrate only until $V_C=0$, then retain the stopped pose.
The published acceleration state becomes zero at rest. Do not extrapolate
negative speed or intersect this reset state as if it were an unchanged latent
pre-stop parameter. With available observer enclosures, the executable update
intersects the acceleration and curvature intervals with the preceding family
and propagates/intersects the speed interval. Possible stops propagate the
acceleration interval through the declared reset to zero before intersection.
An empty intersection reports an inconsistent target-model/observer contract;
it is not silently replaced by a new family.

The nominal is the closest continuation of the preceding nominal in the
current position ball, course interval, and motion-parameter intervals. A
point-estimate fluctuation inside those sets need not change the nominal
motion. Position and course radii are increased by the center displacement;
speed, acceleration, curvature and sideslip radii are recomputed from the
retained intervals. Raw observation and selected nominal remain separately
available in `model.targetPredictionUpdate`. Without a valid current set the
observed point is used and the set memory is cleared. This preserves a
current outer enclosure and constant-parameter interval consistency; it does
not implement the complete set-valued horizon shift of the conditional theory.

An encounter ends at its first exit from the perception disk, default radius
50 m. A complete current empty scan or an observed target outside the disk
can clear the runtime encounter; a missing observation alone retains its
forecast. Actual reacquisition creates a new encounter and a new
initial-feasibility question. A predicted center exit cannot certify that all
uncertain branches have exited.

<a id="observer-geometry"></a>
## 2. Observer bounds and relative geometry

### Current-state containment from a comparison system

The controller consumes the estimator's current certified enclosures. Its
observer derivation remains in
[OBSERVER_ISS_THEORY.md](../estimator/OBSERVER_ISS_THEORY.md), including
the shared-model force-balance measurement and its ego error bound.
For the continuous cascade, write

$$
\chi=(\|e_v\|,\sqrt{e_T^\top Q_oe_T})^\top,\quad
D^+\chi\le H\chi+d,\quad
H=\begin{bmatrix}-k_v&0\\c_{Tv}&-\lambda_T\end{bmatrix},
$$

where $k_v,\lambda_T>0$, $c_{Tv}\ge0$ and $d\ge0$. With
$Q_o=(D_oP_oD_o)\otimes I_2$ and
$D_o=\operatorname{diag}(1,\omega_o^{-1},\omega_o^{-2})$, a valid initial bound
$b_0\ge\chi(t_0)$ gives

$$
b(t)=e^{H(t-t_0)}b_0+\int_{t_0}^t e^{H(t-s)}\bar d(s)\,ds,\qquad
\chi(t)\le b(t),\quad \bar d\ge d.
$$

**Derivation.** $H$ is Metzler, hence its transition matrix is entrywise
nonnegative. Apply the comparison principle and variation of constants.
For constant forcing, the integral is $H^{-1}(e^{H\tau}-I)\bar d$.
The off-diagonal entry of $e^{H\tau}$ is
$c_{Tv}(e^{-k_v\tau}-e^{-\lambda_T\tau})/(\lambda_T-k_v)$, or
$c_{Tv}\tau e^{-k_v\tau}$ at equal rates. The steady-state value
$-H^{-1}\bar d$ cannot replace an initial transient that it does not contain.

The resulting ellipsoid has support

$$
\mathcal E_T=\{e:e^\top Q_oe\le b_T^2\},\qquad
\sigma_{\mathcal E_T}(a)=b_T\sqrt{a^\top Q_o^{-1}a}.
$$

This follows by Cauchy--Schwarz in the $Q_o$ metric. Its component radii are
$b_\rho=\sqrt{(P_o^{-1})_{11}}b_T$,
$b_q=\omega_o\sqrt{(P_o^{-1})_{22}}b_T$ and
$b_s=\omega_o^2\sqrt{(P_o^{-1})_{33}}b_T$.
Using the joint support preserves more correlation than independent balls.

The sampled observer requires its own validated finite-time enclosure or a
sampled comparison $b_{k+1}\le M_kb_k+d_k^{\rm obs}$ with $M_k\ge0$.
A continuous theorem alone does not certify RK4, held measurements, timing or
outages. Append independently valid yaw, gyro and absolute-position bounds.
An energy-gain statement alone is not a pointwise yaw enclosure. Current
publication (`futurePredictionIncluded=false`) is not horizon propagation.
A deterministic amplitude contract also cannot be inferred from unbounded
Gaussian simulation noise.

### Rotating-frame reconstruction and common-pose cancellation

Let $J=\left[\begin{smallmatrix}0&-1\\1&0\end{smallmatrix}\right]$ and

$$
\rho=R_E^\top(p_C-p_E),\quad q=R_E^\top v_C,\quad s=R_E^\top a_C.
$$

Differentiation gives the NRMM transport equations

$$
\dot\rho=q-v_E-r_EJ\rho,\quad
\dot q=s-r_EJq,\quad \dot s=\Phi(q,s)-r_EJs,
$$

$$
V_C=\|q\|,\quad A_C=q^\top s/V_C,\quad
\Omega_C=(Jq)^\top s/V_C^2,\quad
\Phi=-\Omega_C^2q+3A_C\Omega_CJq/V_C.
$$

The factor three follows by differentiating
$a_C=A_Ct+\kappa_CV_C^2Jt$ with constant $A_C,\kappa_C$.
$\Omega_C$ is course/yaw rate, not a sideslip derivative. On the
positive-speed, $|\beta_C|<\pi/2$ domain,

$$
\beta_C=\arcsin\frac{l_{rC}(Jq)^\top s}{\|q\|^3},\qquad
\psi_R=\arg q-\beta_C.
$$

A sound relative-yaw enclosure must cover the image of the entire joint
$(q,s)$ set. One conservative bound uses
$\nu=\|\hat q\|-b_q>0$, $S_*=\|\hat s\|+b_s$, and a true domain
$|\beta_C|\le\beta_{\max}<\pi/2$:

$$
b_\beta=\frac{l_{rC}}{\cos\beta_{\max}}
\left(\frac{4S_*b_q}{\nu^3}+\frac{b_s}{\nu^2}\right),\qquad
b_{\psi_R}=\min\left(\pi,\arcsin\frac{b_q}{\|\hat q\|}+b_\beta\right).
$$

Here the center's arcsine argument is clipped to the valid sideslip range.
The derivative bounds of the unclipped argument are $4l_{rC}S_*/\nu^3$ in
$q$ and $l_{rC}/\nu^2$ in $s$; clipping is nonexpansive and arcsine's slope
on that interval is at most $1/\cos\beta_{\max}$. The velocity ball excludes
zero along every connecting segment. At possible zero speed this inverse
supplies no heading certificate; retain a valid heading set or unknown yaw.
Clipping the estimate cannot establish the true domain premise.

For offset body rectangles $B_E,B_C$, rigid-motion invariance gives

$$
\operatorname{sdist}(\mathcal V_E,\mathcal V_C)
=\operatorname{sdist}(B_E,\rho+R(\psi_R)B_C).
$$

Common absolute translation and rotation cancel from instantaneous collision
geometry. Absolute pose uncertainty still affects path/road conditions, and
velocity/rate errors still affect prediction. Relative uncertainties need not
be independent. Ordinary distance is zero even for overlap; its nonnegativity
is therefore insufficient to certify nonoverlap.

<a id="optimization"></a>
## 3. Collision constraints and the switched problem

### One nonlinear geometry and its continuous witnesses

Let $v_{E,a}$ and $v_{C,b}$, $a,b=1,\ldots,4$, be body vertices measured from
the respective dynamics reference points, including rectangle offsets. Write
$e_a=p_E+R(\psi_E)v_{E,a}$ and $c_b=p_C+R(\psi_C)v_{C,b}$. Define one signed
clearance function, independent of the planner or numerical reference:

$$
 d_s(E,C)=\max_{\|n\|_2=1}\{\min_a n^Te_a-\max_b n^Tc_b\}.       \tag{G1}
$$

For disjoint convex rectangles this is Euclidean separation; for overlapping
interiors it is the negative minimum translational penetration depth. This is
the signed-distance support/dual construction of Zhang, Liniger and Borrelli,
Theorem 2, specialized to rectangle vertices. Ordinary unsigned distance
would be zero even for overlap and cannot represent the desired soft deficit.

Introduce $n(\theta)=(\cos\theta,\sin\theta)$ and a plane offset $b$ measured
relative to $p_C$. The nonlinear collision inequalities are

$$
\begin{aligned}
 n(\theta)^T[p_E-p_C+R(\psi_E)v_{E,a}]-b+\xi&\ge0,&&a=1,\ldots,4,\\
 b-n(\theta)^TR(\psi_C)v_{C,b'}&\ge0,&&b'=1,\ldots,4,\\
 \xi&\ge0. &&
\end{aligned}                                                    \tag{G2}
$$

Eliminating $b$ and $\theta$ gives exactly $d_s(E,C)\ge-\xi$. At zero slack,
the first four rows put every ego vertex on one side and the other four put
every target vertex on the other. With positive slack, the maximum permitted
penetration along the chosen unit direction is explicit. A normal constrained
only by $\|n\|\le1$ would admit $n=0,b=0,\xi=0$ at overlap: the equality
normalization is essential here. The angle is a periodic coordinate on the
unit circle, not a discrete maneuver label. A complete $[-\pi,\pi]$ angle increment
chart excludes no orientation from the nonlinear problem. RTI uses its local
tangent coordinates and the retraction below, not a global angular search.

For uncertain joint state $q\in R_i$, require (G2) for every member with a
common witness $(\theta_i,b_i)$. This common-plane robust condition is
sufficient, and can be more conservative than requiring physical nonoverlap
with a different witness for every unknown member. That quantifier order is
part of the design, not hidden inside a distance calculation. Shrinking $R_i$
retains every previously valid witness. Relative position must be formed
before projecting shared uncertainty, so common ego/target errors cancel
when the joint model actually permits their cancellation.

The online frozen-enclosure transcription uses a directional allowance
$\varepsilon_i(\theta)$ in the first four rows:

$$
 g_{E,a}=n^T[p_E-p_C+R_Ev_{E,a}]-b-\varepsilon_i(\theta),\qquad
 g_{C,b'}=b-n^TR_Cv_{C,b'}.                                    \tag{G3}
$$

It combines the relative affine position support, target-family support and
ego-yaw chord reserve. The target bound intersects the existing support
bound with a complete NRMM parameter-cell cover, retaining shared
position/yaw dependence. Both backends call the same allowance function.
This is sound with respect to its stated outer sets, but the ordinary joint
affine prediction has not established their containment of all nonlinear
future errors. No new sensor or model-mismatch budget is invented or reduced.

At a regular reference, set $t=Jn$. The derivatives of the first four rows
with respect to $(p_E,\psi_E,\theta,b)$ are

$$
 [n^T,\ n^TR_EJv_{E,a},\
 t^T(p_E-p_C+R_Ev_{E,a})-\varepsilon_i'(\theta),\ -1].           \tag{G4}
$$

For each target row they are $[0,0,0,-t^TR_Cv_{C,b'},1]$. Thus the RTI rows
are precisely $g(\bar z)+Dg(\bar z)\Delta z+\xi\ge0$, with slack only on
the ego rows. `separationWitnessRows` evaluates (G3)--(G4); `localPlaneRows`
adds the shared uncertainty oracle. Target vertices are retained as separate
inequalities rather than selecting a nonsmooth nominal support vertex.

Changing a witness does **not** change (G1) or the predictive barrier value:
it changes an auxiliary optimizer variable in the same fixed definition.
Freezing a path-dependent witness forever would instead restrict the
feasible set. Initialization supplies a useful witness, not a new safety
function. Warm starts shift the previously optimized witnesses; newly
appended samples obtain a geometric witness without a world-angle grid or
an explicit left/right maneuver bank.

A plane update has a useful exact geometric normalization. With RTI tangent
step $(a,d)$, form $\widetilde n=n+aJn$ and $\widetilde b=b+d$, and store

$$
 \theta^+=\theta+\arctan(a),\qquad
 b^+={b+d\over\sqrt{1+a^2}}.                                  \tag{G5}
$$

This is a retraction whose first derivative is the identity on the tangent
coordinates. For fixed vehicle poses and zero uncertainty, all eight exact
unrelaxed plane gaps after (G5) equal their linearized gaps divided by
$\sqrt{1+a^2}$. Since this scale is at least one, an unchanged nonnegative
slack also remains sufficient at fixed poses when the linearized gaps meet
that slack. It therefore preserves the represented halfspaces instead of
interpreting a large tangent increment as an equally large angle rotation.
It updates only the stored geometric witness, not the issued control. Moving
vehicle poses, uncertainty supports and dynamics still incur linearization
error, so this identity is not a nonlinear trajectory certificate. Shooting
uses direct angle coordinates; both have the same first-order plane rows.

The nominal rows are smooth, but uncertainty supports and the terminal
maximum can have feature ties. The present support derivative uses symmetric
finite differences of the same directional allowance in both backends.
This is a numerical derivative convention, not a differentiability theorem.
A fully smooth polytope realization can retain support-dual variables instead
of eliminating the maximum. The exact theoretical formulation (G2) needs no
such differentiation assumption for the shift proof.

A first-order row is not a nonlinear safety certificate for a finite step.
If the witness is fixed, the familiar rotation remainder is bounded by
$\|v\|(\Delta\psi)^2/2$. If the normal moves as well, mixed normal/pose terms
also require bounds; compensating only ego yaw is insufficient. Ordinary RTI
therefore makes no conservative-inner-approximation claim. The optional
whole-domain enclosure path freezes witness angles and retains its existing
rotation reserves, and must be interpreted as a separate restricted
verification experiment.

### Constraint instants and a continuous-time extension

The runtime constrains nodes and hold midpoints. On a one-RK4-step mesh its
midpoint is the endpoint interpolant, not an exact physical midpoint.
For true whole-hold speed/rate bounds, body reach radii $R_E,R_C$ give

$$
L_d=\bar v_E+\bar v_C+R_E\bar r_E+R_C\bar r_C,\qquad
\epsilon^{\rm hold}=L_dh/4.
$$

**Derivation.** Every body moves in Hausdorff distance by at most translation
plus rotational reach times elapsed time. Every instant is within $h/4$ of
start/midpoint/end. Tightening true nonoverlap certificates at those instants
by the motion reserve therefore suffices throughout the hold. Midpoint
integration/interpolation defects must also be enclosed. This extension is a
sufficient construction, not a reserve already established by current replay.
The ideal theorem below guarantees only the declared constraint instants
unless such whole-hold images are supplied.

### One RTI model and target-presence objective switching

For a reference $(\bar x_i,\bar u_i)$, variational tangents give

$$
\Delta x_0=0,\qquad
\Delta x_{i+1}=A_i\Delta x_i+B_i\Delta u_i+e_i,
\qquad e_i=F_h(\bar x_i,\bar u_i)-\bar x_{i+1}.
$$

The onset planner supplies a nonlinear rollout, so its defects vanish. Later
frames shift the previous input sequence and reintegrate it from the current
estimate before appending a tail hold. This preparation removes accumulated
affine state defects before constructing trust regions and uncertainty
tangents. It changes neither the already issued input nor the fixed encounter
horizon, and it does not invoke the planner again. The general defect term
remains in the equations for supplied research references. The tail
uses nonlinear path guidance with inverse-Fiala force allocation, even when
its entry belongs to the terminal set. It is a numerical seed, not an
uncertainty-certified terminal extension. The local cruise gain is not
extrapolated at a remote endpoint to construct this hold.

A saturated shifted steering seed can be retracted to the adhesion branch
before this rollout once the preceding solve has attained zero safety
deficit within `solver.constraintTolerance`. While its deficit is positive,
the old controls are retained: force retraction changes longitudinal timing
and can disrupt a maneuver still reducing its collision deficit. At the
current reference state, hold the longitudinal
ratio fixed and solve the scalar equation

    Fx_front sin(delta_new) + Fy_front(alpha_new) cos(delta_new)
      = Fx_front sin(delta_old) + Fy_front(alpha_old) cos(delta_old),
    alpha = atan2(vy + lf*r, vx) - delta.

The bounded Newton inversion starts at zero front slip and is used only
when it converges to a smaller-slip point with nonzero tire-force slope.
Otherwise the old seed is retained. Thus the instantaneous front lateral
force in body coordinates is preserved; its longitudinal component and
the subsequent trajectory need not be preserved and are recomputed by the
nonlinear rollout. This avoids centering RTI on a saturated branch where
further steering mainly rotates already saturated tire forces. It imposes
no additional force-capacity inequality and does not change physical input
bounds or the command returned by the optimizer. It is a numerical
initialization choice, not a retained feasible-policy certificate.

The current input estimate is fixed; current state error is handled by the
published enclosure, not a free optimizer correction. Continuous plane-variable
rows and the tangent of the shared nonlinear terminal function are built once on this model. Avoidance uses at most two
native objective solves on that same reference; cruise uses one.
The completed update supplies the first
optimized input, with the numerical bound normalization described below.
Its predicted states and PCBF slacks remain those of the affine subproblem;
no post-solve nonlinear rollout replaces or rejects
them. The planner, warm-start rollout and stagewise variational integrations
belong to preparation, before solving. The uncertainty prediction still uses
future observer updates and causal feedback; reintegrating the nominal does
not replace that joint tube by open-loop error propagation. Independent
experiment replay evaluates the issued
controls on the plant. There is no auxiliary feasibility restoration problem,
damped-step search or adaptive reference penalty.

### Implemented constraints and objective units

State and input increments have no artificial search bounds. Actual longitudinal-force
magnitude and optional slew limits remain hard. Steering has no artificial
actuator magnitude or slew limit; the Fiala angle domain is enforced. State
limits, sideslip, optional road-corner rows and terminal
rows retain their existing meanings. No shared elastic variable relaxes the
problem whose controls are issued.

Every nominal or uncertainty-tightened collision node carries a nonnegative
PCBF slack, including the prediction endpoint. `controller.horizonSteps`
sets the minimum horizon, not the end of a soft prefix. These deficits are
separate from physical state, road, handling and terminal constraints.

While `model.target` is nonempty, the first objective is `sum(xi)`. No CLF
row or CLF slack cost participates. A zero-slack affine candidate is checked
before solving: first zero correction, then state corrections obtained from
the same affine dynamics with zero input correction. A feasible candidate
attains the safety lower bound without an LP. This check uses floating-point
roundoff tolerance and does not integrate or certify the nonlinear model.
Otherwise one LP minimizes the safety sum.

A second QP is an optional implementation refinement among the attained
minimum-slack solutions. It is not part of the theoretical safety problem
(U6) and its completion is not a premise of Theorem U. Its removal leaves
the primary plan available for execution. The implementation retains it to
improve route following and input smoothness.
Its shared objective is

    J_perf = J_track + J_change + J_ref,
    J_track = w_track/2 sum_k ||De e_aff,k+1||^2,
    J_change = w_change/2 sum_k ||Du (u_k - u_k-1)||^2,
    J_ref = w_ref/(2 N) sum_k (
        ||Dx (x_aff,k+1 - xbar_k+1)||^2 + ||Du (u_k - ubar_k)||^2).

Here e contains lateral position, heading, longitudinal speed, lateral
velocity and yaw-rate errors relative to nominal path trim. Its affine
expansion is evaluated at each reference state; squaring that residual gives
a positive-semidefinite Gauss--Newton objective. The reference xbar/ubar is
the RTI linearization trajectory and is distinct from the nominal path.
The first input difference uses the previously applied command, including
actuator memory supplied with the estimate. J_change is a sum rather than a
horizon mean, so extending N does not cheapen the first command jump.

The refinement retains exactly the same constraints and dynamics and adds
`sum(xi) <= Sstar`, where Sstar is the first stage's attained safety value.
When Sstar is zero, the slacks are fixed to zero. No intentional slack
allowance or safety/performance weighted tradeoff is used. A refinement must
preserve the cap within `solver.constraintTolerance * max(1,Sstar)`; this is
numerical priority, not exact arithmetic. Nominal speed tracking is only a
secondary preference: braking remains allowed when needed for safety. No
CLF inequality is activated during avoidance.

Both stages reuse one RTI model and share the call budget. A zero performance
cost attains its global lower bound without a second native solve. If
refinement fails, exceeds the budget, violates input bounds or exceeds the
safety cap, the completed priority candidate is retained. No third solve,
replanning or nonlinear execution filter is introduced. Positive PCBF slack
remains admissible; state, input and terminal conditions retain
their declared hardness. Soft regularization does not impose a steering-rate
bound or certify nonlinear model accuracy.

Without an active target, collision slacks are fixed to zero and no PCBF LP
is solved. The continuous CLF design is unchanged:

    Vdot <= -alpha V + s,  alpha = 2/clf.convergenceTimeConstantSeconds.

Integrating this condition over a held-input interval h with constant slack s
would imply

    V(x+) <= exp(-alpha h) V(xhat) + tau s,
    tau = (1 - exp(-alpha h))/alpha.

The sole cruise QP imposes the affine tangent of this integrated condition
at the first predicted reference state and minimizes `tau*s + J_perf`.
It has no quadratic CLF constraint or second-order cone. The unbounded
nonnegative slack absorbs any finite affine CLF residual at an otherwise
feasible point. This approximated sampled condition does not establish the
continuous inequality inside the hold, nonlinear descent, or solver success.

The first in-range target observation activates PCBF and the onset planner.
Its first accepted horizon stays fixed throughout the encounter. Zero PCBF
slack does not reactivate CLF. An out-of-range target or a complete sensor
scan clearing the retained forecast restores CLF; missing observations alone
retain PCBF. A new target still requires a feasible initial encounter
problem. No common Lyapunov function or global switched-safety theorem is
asserted. After clearance, cruise references are rebuilt from the current
estimate before the one RTI linearization.

The weights `rti.nominalTrackingWeight`, `rti.inputVariationWeight` and
`rti.referenceDeviationWeight` default to 1, 20 and 0. Zero disables the
corresponding component; all three zero disable avoidance refinement.
Dx uses equal x/y units from `clf.lateralPositionErrorScale`, followed by
heading, speed, lateral-velocity and yaw-rate scales. De uses those five
transverse error units; Du uses the CLF certification steering/braking units.
These are cost units, not bounds. Tracking and input variation are sums;
only optional reference proximity is a horizon mean. The cruise objective is scaled by `max(1,V(xhat))` for the solver;
reported component costs are unscaled.

Each call returns the selected optimization's first input, apart from
tolerance-sized physical input-bound normalization. There is
no rejection of the priority solution due solely to refinement failure,
no PCBF-versus-CLF weighted sum, and no inherited
slack budget. The ideal encounter problem still minimizes its safety slack;
its conditional shift argument remains separate from the executable affine
RTI approximation.

Modified Fiala dynamics include combined-slip saturation at both axles.
The optimizer uses their local derivatives without additional tire-capacity
cones or friction polygons. A tangent can overestimate the nonlinear force
away from its expansion point; this approximation error does not indicate a
missing capacity limit in Fiala. Independent experiments roll the issued
controls through the nonlinear plant; there is no online post-solve rollout
or admission check. Removing the extra cones does not make an arbitrarily
large one-step correction accurate.
The force-balance observer encloses saturated forces too; its interval can
widen when the measurement becomes less informative. Its separate sideslip
premise `|vy| <= tan(model.sideslipMaximum)*vx` remains enforced, with observer
support margins. Removing that premise would require a new observer enclosure.
The precomputed local cruise certificate may use a conservative nonsaturated
neighborhood; that local certificate does not impose a full-horizon cone.

### State-dependent numerical trust regions

The default reference-proximity weight is zero. Nominal-path/speed tracking and
actual input-increment regularization remain active. Before either objective
solve, each hold receives a box around its dynamics linearization point in
`q = [psi, vx, vy, r, delta, b]`. Global x/y translations are exactly affine in
the transition and receive no trust bound. The box is frozen for both PCBF
stages, and is also used by the single cruise QP. It adds only variable bounds;
no extra cone, tire-capacity constraint, SQP iteration, replanning call,
post-solve rejection, or horizon extension is introduced.

Let F be the discrete held-input map, J its reference Jacobian, S a diagonal
coordinate scaling, and epsilon the six-component one-hold error budget.
Central differences of the discrete Jacobian estimate the scaled Hessians

    H_l = S (d^2 F_l / dq^2) S / epsilon_l.

The estimate is symmetrized. With

    w_i = max_l sum_j |H_l,ij|,
    s_i = min(1, sqrt(2 / (6 w_i))),
    radius_i = S_ii s_i,

the quadratic surrogate is at most one on the box. This follows from
`2 |d_i d_j| <= d_i^2 + d_j^2` and the six-coordinate allocation of its
quadratic budget. The scales are `[0.3 rad, max(2,0.25|vx|) m/s, 1 m/s,
0.4 rad/s, 0.25 rad, 0.5]`. They cap the extent of the initial local search;
they are numerical design choices, not physical steering or sideslip limits.
The resulting radii vary with speed, slip, yaw rate, steering and combined-slip
loading. Mixed derivatives account for interactions such as steering and
braking rather than treating their independent bounds as independent forces.

Because local curvature can miss a saturation transition, the implementation
also evaluates the exact discrete map at both ends of each coordinate axis and
four mixed-sign box directions. The actual remainder `F(q+d)-F(q)-J*d` is
compared componentwise with epsilon. A failed probe shrinks all radii by
`max(0.1,min(0.8,0.8/sqrt(ratio)))`; at most six probe passes are permitted.
Failure to find a passing neighborhood is reported, not silently accepted.
These are local preparation evaluations, not additional trajectory optimizer
iterations. The same Fiala transition, including its low-speed branch, is used.

The default budget is `[0.04 m, 0.04 m, 0.01 rad, 0.4 m/s, 0.4 m/s,
0.2 rad/s]` per hold. It controls numerical approximation locality; it neither
replaces nor adds another copy of the observer/model-mismatch error tube.
Radii are intersected with existing physical/model-domain bounds. The two
PCBF objectives share this intersection from the outset: the primary optimum
may increase relative to an unrestricted affine problem, and performance
still cannot buy any further PCBF slack. A CLF slack cannot relax a hard
terminal condition or a trust bound.

This is a **numerical locality estimate**, not a uniform nonlinear certificate:
a finite Hessian stencil and finite displacement probes do not bound the
remainder at every interior point. It also does not bound accumulation across
the entire horizon, certify the separate collision/terminal tangents, or
provide the accept/reject convergence theory of a full trust-region SQP
algorithm. No such guarantee is inferred from passing simulations.

Adding these hard numerical bounds restricts the executable local problem.
A shifted nonlinear witness from the ideal recursive-feasibility theorem is
not automatically inside the next box. Ordinary warm starts now reintegrate
the shifted controls from the current estimate, removing the nominal dynamics
defect. Supplied research references can still have defects; these remain
explicit in the equalities and in the consistency procedure below. A
consistent nominal alone does not certify robust hard-constraint feasibility
or retention of a shifted causal policy.

Before solving, forward interval propagation intersects the reachable
correction intervals of these affine equalities with the current variable
bounds, starting with the correction fixed by the initial-state equality.
An empty intersection detects an inconsistency between the dynamics and
boxes. A nonempty intersection is only a necessary condition, not a
feasibility certificate for the complete problem.

When this check finds an empty intersection, compute a particular affine
consistency correction with unchanged inputs:

$$
d_0=\hat x_k-\bar x_0,\qquad
d_{i+1}=A_i d_i+e_i,\qquad \Delta u_i=0.
$$

This forward sensitivity pass uses the existing tangents. It adds no
optimizer call, replanning, nonlinear trajectory rollout or SQP iteration.
Only at the next state whose incoming correction interval misses its box,
reserve the corresponding nonlinear-state radii and recompute that box with
the **same error budget**. Other stages retain their ordinary radii; a large
later open-loop sensitivity alone does not cause an unrelated enlargement.
The interval test is repeated after reallocation. Repairs advance through
the horizon, stop at an unresolved physical bound, and never exceed the
number of stages.
With $r_{\min}=(|d_{i,3:6}|,0,0)$, initialization uses
$r=\max(r_{\rm ordinary},r_{\min})$ and failed probes reduce only excess room:

$$
r\leftarrow r_{\min}+\gamma(r-r_{\min}),\qquad 0<\gamma<1.
$$

The reference Hessian seeds the reallocated box but does not veto it: its
absolute quadratic envelope can overpredict finite displacement errors across
tire saturation or low-speed model transitions. For a required correction,
the reallocated box is checked using the actual discrete-map remainder at all
64 mixed corners and both endpoints of each of the six axes. The componentwise
error budget is unchanged. A needed state correction can therefore be
accommodated by reducing other state/input freedom without also requiring the
pointwise curvature surrogate to remain below the budget. If no candidate box
passes these finite checks, preparation reports failure. This remains a
sampled numerical locality estimate, not an interior or uniform error bound.
Physical bounds are retained throughout. The
shifted reference, general equality defects and optimization objectives stay
unchanged, and an ordinarily compatible box is retained exactly.

This particular correction certifies only the affine dynamics equations;
it need not satisfy physical, collision or terminal constraints. If the
complete hard constraints cannot be satisfied inside the final domain, RTI
reports failure. Applying the ideal recursive proof to this restricted online
algorithm would additionally require an admissible shifted witness inside
every next trust domain. The conditional information-set theorem elsewhere
in this document remains a statement about its stated exact problem.

Physical input bounds include force ratios -1 and +1. After solving, input
values outside their physical bounds by at most `solver.feasibilityTolerance`
are projected to the exact bounds. Larger residuals reject the solver result
with `pcbfInputBoundsViolation` or `clfInputBoundsViolation`. In-bounds values
are not changed. The issued command, stored input sequence and input
corrections use the same normalized values. Affine states and slacks are not
recomputed; their constraint residual is evaluated with the normalized input.
`inputBoundCorrection` reports the largest correction, and
`postSolveInputModification` records whether any value changed. This numerical
interface does not relax the Fiala domain or certify nonlinear feasibility.
Only the tangent reference
uses `nonlinearBicycleModel.inputBounds(cfg,true)`, which keeps its force
ratio ten constraint tolerances inside the singular endpoints. Shifting a
saturated previous plan rebuilds this interior seed and its complete rollout;
this does not change a previously issued command. The 100 ms controller budget
reserves 2 ms for completion. Each controller call builds at most one RTI
model and performs at most two native objective solves during avoidance,
or one during cruise. Initial encounter
horizon sizing precedes this model, and later encounter calls retain that
length; primal infeasibility and numerical failure
are reported without increasing the node count or rebuilding the problem.
No auxiliary restoration solve is run.

The executable affine collision deficits must not be identified with a
robust nonlinear value. All collision nodes can be relaxed in the online
approximation; its terminal, input and model-domain conditions remain hard.
The retained verification backend supplies its separate constructive proof.

The legacy per-hold option restarts the present ego enclosure $G_0$ at each future hold and
uses $G(t_i+\tau)=\Phi_i(\tau)G_0$ over that hold. Target uncertainty similarly
restarts at its forecast state. This assumes a future posterior width; it
does not construct nonlinear reachable images or prove cross-sample inclusion.
Usable finite bounds tighten the program. Missing or stale certificates do
not block experimental execution; omitted padding does not assert zero error.

<a id="nominal-clf"></a>
## 4. One nominal CLF and recovery

### Function and its scope

For the given path and desired cruise trim, define

    e(x) = [e_y; e_psi; vx-vx_ref; vy-vy_ref; r-r_ref],
    V(x) = e(x)' P e(x),       P > 0.

Longitudinal path phase is free. `nonlinearBicycleModel.cruise` computes the trim
and reads P, the gain K of the cruise controller and their certificate from
`config/clfMatrices.json`; it never computes them. `nominalValue` evaluates
this quadratic directly. There is no online LQR design. The gain K defines
the local cruise controller `u = u* + K e` and its offline certificate;
`terminalSafeSet.cruiseInput` exposes a numerical hold probe. Ordinary
initialization instead uses nonlinear path guidance, avoiding extrapolation
of that local gain far from its equilibrium. The terminal set itself does
not use the CLF. No nominal-policy rollout, finite-difference Hessian, cost-to-go
kernel, or additional terminal value is used to construct the CLF.

### Requested decrease: a convergence time constant

The performance design uses the continuous-time nonlinear bicycle system:

    Vdot(x,u) <= -alpha V(x) + s,   alpha = 2/T,   s >= 0,

where T = `clf.convergenceTimeConstantSeconds` = 4 s. The slack has units of
CLF value per second and has no upper bound. Exact continuous enforcement
with zero slack implies `V(t) <= exp(-alpha t) V(0)` and hence a time constant
T for sqrt(V). The online implementation uses a single tangent of the corresponding
integrated held-step condition, as described below. Its affine slack does
not certify the exact nonlinear derivative or held-step contraction.

### Continuous-time CLF matrix and separate sampled cruise certificate

`scripts/synthesizeClfMatrices.m` computes P and K before experiments, for
each operating point (vehicle parameters, speed, path curvature, sample time
and CLF settings), and writes them with their key and certificate to
`config/clfMatrices.json`. A controller call at an operating point without an
entry is rejected with `collisionAvoidanceController:missingClfMatrix`, an
entry without the certificate with `collisionAvoidanceController:staleClfMatrix`;
nothing is synthesized at run time, and the run time does not need the SDP
solver.

The matrix and gain satisfy the continuous trim Lyapunov inequality

    Ac_cl = Ac + Bc K,
    Ac_cl' P + P Ac_cl <= -alpha P.

The synthesis imposes this as `Ac S + Bc Y + (Ac S + Bc Y)' + alpha S <= 0`,
with `S=P^-1` and `Y=K S`, in physical coordinates (scaled in code).
The design uses a slightly faster rate via `DesignMargin` and rechecks a
strict margin at the requested rate after synthesis.
`continuousLinearDecayRate` records the minimum decay rate found by the
normalized symmetric eigenvalue check. All six retained table entries pass
strictly; their existing P and K values therefore needed no numerical change.
Strictness and smoothness imply continuous nonlinear decrease on a sufficiently
small neighborhood of the trim, with admissible feedback `u=u_ref+K e`.
This argument does not identify that neighborhood with the sampled certificate's
whole `V<=1` set and does not establish a global CLF.

Separately, the held cruise initializer keeps its sampled-model certificate.
Its purpose is to validate a possible appended cruise hold, not to define the
online CLF row. This additional synthesis condition targets the sampled
nonlinear model through a ray-average
enclosure, rather than only the trim linearization. Its coverage premise is
made explicit below. Let `g(e, du)` be the transverse error after one hold from the
state with error `e` under the input `u_ref + du`, and `G = [dg/de, dg/d(du)]`
its Jacobian from the RK4 variational equations and the error chart
(`terminalSafeSet.jacobians`). The trim is a fixed point, so along the ray
from `0` to `(e, du)`

    g(e, du) = Gbar(e, du) [e; du],   Gbar(e, du) = integral_0^1 G(t e, t du) dt,

exactly. The recorded enclosure is constructed from ray averages `Gbar` along
the controller over the set (`terminalSafeSet.rayJacobians`, Gauss-Legendre quadrature with
8 nodes; the quadrature's residual in the identity is added to the enclosure's
residual as a rank-one term), so a discrete Lyapunov inequality that holds for every matrix of the
enclosure transfers to the nonlinear map if the enclosure covers every
required ray average. Enclosing the ray
averages rather than the pointwise Jacobians matters for a saturating tire:
the averages spread like the secants of the force curve, the pointwise
Jacobians like its tangents, and at the slips the set reaches the tangent
spread is two to three times the secant spread (the first coefficient bound
drops from about 0.13 to 0.06 at 8 m/s). The enclosure is a zonotope in
box-scaled coordinates (the trim's Jacobian, the leading principal directions
of the sampled deviations with their coefficient bounds, and a residual norm,
all widened by a margin factor of 1.25), and the inequality is imposed at its
vertices with the residual absorbed by Petersen's lemma. With `S = inv(P)` and
`Y = K S` the LMI is

    maximize t
    subject to  [rho S, N_v', Z'; N_v, S - lambda_v I, 0; Z, 0, lambda_v I] >= 0   (every vertex v),
                N_v = A_v S + B_v Y,   Z = eps [S; Y],
                [ubar_j^2, Y_j; Y_j', S] >= 0       (j = steering, braking),
                S_ii <= box_i^2,   S >= t diag(box)^2,

with `rho = exp(-2h/T)` the requested contraction, `ubar =
[clf.certificationSteeringRadians; clf.certificationBrakingRatio]` (0.075 rad
and 0.125 on `V <= 1`) and `box` the certification box of
`clf.certificationLateralMeters`, `...HeadingRadians`,
`...SpeedMetersPerSecond`, `...LateralVelocityMetersPerSecond` and
`...YawRateRadiansPerSecond`, inside which `Omega` must lie. Because the
enclosure depends on `Omega` and `K`, the script grows the set
self-consistently: the plain LMI inside half the box gives a first pair; each
round encloses the ray averages over 1.5 times the current set and over
inputs within a quarter of `ubar` of the current controller, and solves the
LMI with the new set inside that inflated set (`S' <= 1.5 S`) and the new
gain within that band of the old one on it (`|(K' - K)_j e| <= ubar_j / 4`
on the new set). Every round's pair therefore satisfies the matrix inequalities for the
enclosure it was designed with, and the set may grow by 1.5 in `V` per round until the
enclosure's growth stops it; the pair with the largest fill is kept. Its
vertex inequalities are re-verified by eigenvalues and the smallest
contraction they certify is recorded (`certifiedContraction`, at most `rho`).
The partial-hold maps are enclosed the same way, on the same samples, to give
the hold factor, the excess of the CLF radius inside a hold.
YALMIP/SeDuMi from `solver/`.

The remaining coverage premise is the enclosure hypothesis (all ray averages
over the set lie in the recorded zonotope): it is built from 1200 boundary and 600 interior
samples of the inflated set with a recorded seed. The entry records the
samples' seed, the coefficient bounds, the residual (with the quadrature's
share), the largest slip angles reached, the inputs' maxima, and an
independent sampled check with another seed (`terminalSafeSet.certificate`:
worst contraction, hold factor, the problem's rows); `nominalClfTest` repeats
that check and tests fresh ray averages against the recorded zonotope.

**This quadratic is not a global nonlinear CLF.** Large heading errors, tire
saturation, state-domain boundaries and braking memory
can make zero slack unavailable even without a target. For example, at a
20-m lateral displacement the sampled tests found positive slack despite an
actual reduction in V. The former cost-to-go argument covered a different,
feedback-reachable region; it cannot be transferred to this simpler function.
The scenario campaign and displaced-state tests therefore measure eventual
cruise recovery separately from local zero-slack decrease.

If the exact continuous nonlinear derivative inequality holds with zero slack
throughout a suitable invariant domain, it gives exponential decay of the transverse error with time constant T. Zero slack in the
**affine approximation**, together with a fixed model-error tolerance, does not
establish that exact inequality or global asymptotic convergence.

### One affine integrated continuous-CLF constraint

The CLF inequality and its slack cost act only outside an active encounter.
The continuous quadratic V=e'Pe and its offline design are retained. At the
first predicted reference state xbar_1, define

    Vbar_1 = e(xbar_1)' P e(xbar_1),
    g_1 = 2 e(xbar_1)' P J_e(xbar_1),
    Vaff_1 = Vbar_1 + g_1 dx_1,
    tau = (1 - exp(-alpha h))/alpha.

The integrated condition above becomes one affine row

    (Vaff_1 - V(xhat))/tau <= -alpha V(xhat) + s.

Its state increment dx_1 is coupled to the first input by the same affine
held-input dynamics used by RTI. This incorporates the duration of the
applied command instead of requesting descent only at its starting instant.
It adds neither another model linearization nor a post-solve nonlinear
rollout. Together with the shared positive-semidefinite performance Hessian,
cruise remains a QP. Native solver iterations are internal to that one QP.

This is a tangent approximation of the endpoint value. It is not an upper
bound: quadratic value terms, curved-path coordinate remainders and the
nonlinear dynamics remainder are not enclosed. Consequently, zero local
slack is not a guarantee of exact nonlinear held-step decay. Even the exact
endpoint inequality alone would not imply the continuous derivative condition
at all intermediate times. The continuous design, sampled implementation
and empirical closed-loop recovery must be distinguished.

Both modes use J_perf from the switched-optimization section. It gives the
controller a persistent route/speed preference and penalizes actual control
increments, alongside the reference regularizer that favors a useful local
linearization. It introduces no road-boundary constraint, hard correction
box or new tire-capacity constraint. A finite weight can still admit a large
correction. PCBF retains lexicographic priority; the new performance terms
cannot intentionally increase its attained optimum. The feasible-set shift
argument of the ideal PCBF problem is therefore unchanged by this tie-breaker.

The observer enclosure does not alter the performance row: it acts on the
estimate. The compatibility field `clfDerivative` now reports
`(Vaff_1-V(xhat))/tau`, the affine integrated-rate surrogate, and
`clfDerivativeLimit=-alpha*V(xhat)`. The explicit label
`clfLocalModel=integratedContinuousClfAffineInNextState` distinguishes it
from an instantaneous model derivative. `clfActualDerivative` and its
residual remain NaN online. `clfNextValue` evaluates V on the affine first
successor; it need not equal the tangent Vaff_1. The independent scenario
driver records `actualNextClfValue` on the plant successor.

`clfSatisfied` denotes zero affine slack within solver tolerance. During
an encounter CLF fields are disabled or NaN, and `optimizationMode=pcbf`
with `optimizationObjective=pcbfSlack`. Its second stage is labeled
`nominalTrackingAndRegularization`, with the attained safety cap recorded.
Cruise uses `integratedClfSlackAndTracking` in one QP and has no PCBF cap.

Neither the affine constraint nor the regularized objective proves exact
nonlinear or uncertain-plant convergence. The observer error, validity of
local approximations and conditional safety assumptions remain separate;
see Section 7. Future states receive tracking costs,
but no additional CLF descent inequalities. Target presence alone selects
the optimization mode.

### What the offline enclosure certifies

The LMI establishes contraction for every matrix in its recorded enclosure.
The enclosure is built from sampled ray averages, a quadrature residual and
an empirical margin factor. Covering sampled points does not alone establish
uniform coverage of every nonlinear ray on the ellipsoid. Thus an unconditional
nonlinear certificate additionally needs a validated enclosure of that complete
domain. The stored certificate and `nominalClfTest` give reproducible numerical
checks; they do not replace that coverage premise.

### Disturbance-dependent recovery: a sufficient theoretical construction

This extension retains the same $V$ but is not the current estimate-only row
or fixed reference penalty. Suppose a common recovery action on a verified
domain satisfies

$$
\sqrt{V(F(x,\kappa(x)))}\le a\sqrt{V(x)},\quad 0\le a<1,
$$

and estimate-feedback error, admitted plant defects and integration effects
perturb the successor in the $P$ norm by at most $D_k$. For $a^2<\lambda_c<1$,

$$
(a\sqrt V+D_k)^2\le\lambda_c V+
\frac{\lambda_c}{\lambda_c-a^2}D_k^2.
$$

The difference is the square
$(\lambda_c-a^2)(\sqrt V-aD_k/(\lambda_c-a^2))^2$.
Choose $Q\succ0$ and $\eta Q\preceq(1-\lambda_c)P$. With
$\ell=e^\top Qe$ and $\sigma_k=\lambda_cD_k^2/(\lambda_c-a^2)$, this yields

$$
V(x^+)-V(x)+\eta\ell(x)\le\delta_k+\sigma_k.
$$

The supply is derived from the disturbance contract; $\delta_k\ge0$ is CLF
relaxation. Zero-relaxation feasibility additionally requires that the common
recovery action satisfy collision, terminal, input and domain constraints.
If that candidate exists and the recovery objective is

$$
\min\;\delta_k+\epsilon\underline\ell_k R(\Delta U),\quad
0\le R\le1,\quad0\le\underline\ell_k\le\inf\ell,\quad0\le\epsilon<\eta,
$$

an optimization gap $\tau_k$ gives
$\delta_k\le\epsilon\ell_k+\tau_k$. Therefore

$$
V_{k+1}-V_k\le-(\eta-\epsilon)\ell_k+\sigma_k+\tau_k.
$$

For $\ell\ge\mu V$, $c=(\eta-\epsilon)\mu\in(0,1]$, iteration gives

$$
V_{k+j}\le(1-c)^jV_k+
\sum_{s=0}^{j-1}(1-c)^{j-1-s}(\sigma_{k+s}+\tau_{k+s}).
$$

Bounded supply/gap gives a practical neighborhood; a vanishing supply/gap
gives convergence on the verified invariant domain. The runtime's fixed,
unbounded quadratic reference-deviation cost is not this bounded vanishing
tie. It cannot inherit this recovery result merely because the CLF row is
soft. The runtime also does not certify the exact nonlinear dissipation row.

<a id="terminal-set"></a>
## 5. Joint terminal set and braking continuation

This section describes the implementation's simplified braking candidate and
the conditions needed to instantiate a terminal certificate with it. The
core theoretical algorithm does not prescribe braking, parking, a curvature
law or a numerical terminal-set construction. It assumes a terminal pair
with the properties in (U4). Verifying this particular candidate against
those properties is implementation validation, not an unfinished step of
the conditional recursive-feasibility proof.

### State and task boundary

The terminal condition belongs to the **joint encounter**, not to an ego speed
or a parking location. Its state is

    z = (x_E, q_C, a, u_previous),
    x_E = (p_E, psi_E, v_x, v_y, r),
    q_C = (p_C, psi_C, V_C, A_C, beta_C, l_r,C, rectangle_C).

The flag `a` is one while this encounter is active and zero after its first
exit from the ego perception disk, whose configured radius is 50 m. At each
constrained node or midpoint it evolves as

    a_next = a AND (norm(p_C_next - p_E_next) <= R).

During the parked phase the first exit is computed in continuous time. An
ended flag cannot be cleared by predicting a later re-entry. A fresh actual
observation after an actual exit creates a new encounter, with a new
initial-feasibility question. A missing measurement alone is not evidence
of an exit: only a complete current empty scan or a supplied observation
proved outside the disk clears the runtime forecast.

This boundary imposes no arbitrary time limit. If the target never leaves
the disk, it remains part of the encounter. A curved target that stays in
range cannot be dismissed just because it turns or the ego stops.

`jointState`, `jointSuccessor`, and `jointMember` expose this state and its
transition. `member(context,x,node)` and `rows` are the RTI adapters: the
context supplies the target at the same absolute time and the ended flag.
The controller's `encounterId` advances only on actual reacquisition; a
predicted exit does not erase a currently observed target.

Terminal membership certifies an available continuation. It neither commands
parking nor switches off MPC. Moving cruise states may belong to the set.
Normal operation still aims to avoid the target and recover the prescribed
path and speed. This use of a terminal candidate follows Huang et al. (2025),
Section III, equations (8) and (10),
[primary source](https://arxiv.org/abs/2502.08400).

### One terminal controller

In the declared terminal kinematic model, retain the steering and apply a fixed
braking command, with a forward-only absorbing stop:

    b_f = -a_b / longitudinalAccelerationGain,
    delta_f = latched steering,
    dV/dt = -a_b while V > 0; V(t) = 0 after stopping.

`control(context,joint)` returns this command and its modeled acceleration.
The command does not depend on an unknown true speed to choose the last
hold. There is no final weak-braking hold and no reacceleration after rest.
The default `a_b` is 4 m/s^2. With initial speed V and elapsed time t,

    tau = min(t, V/a_b),
    s(t) = V*tau - a_b*tau^2/2,
    t_stop = V/a_b,       s_stop = V^2/(2*a_b).

The stop may occur inside a sampling hold. `continuation` evaluates this
flow at nodes and midpoints; the parked-track check starts at the actual
stop instant, not at the following sample boundary. `successor` is its ego
projection and `jointSuccessor` also advances the target and exit memory.

The terminal-model assumption keeps beta_E = atan2(v_y,v_x) and kappa_E = r/V
constant while moving. Consequently

    p_E(t) = p_E + s*sinc(kappa_E*s/2)
                   * direction(psi_E + beta_E + kappa_E*s/2),
    psi_E(t) = psi_E + kappa_E*s.

At rest the pose is fixed. The target uses the same constant tangential
acceleration and sideslip prediction as the main controller, with its own
forward-only stop. There is no backward extrapolation through a stop.

This is the declared terminal model, not an identity with the Fiala plant.
In particular, a held actuator ratio and road loads do not independently
prove exact constant deceleration or curvature in the full plant. A robust
claim needs the compatible terminal uncertainty contract described below.

### Terminal set

Let Phi_f be the joint flow including the absorbing encounter flag. Let C
require the declared physical/input conditions, the reference-point deviation
bound and any enabled road constraints,
and require rectangle nonoverlap only on active branches. The extra design
clearance is zero; uncertainty inflation remains part of the required support.
Then the terminal set is the fixed-policy safe set

    X_f = { z : x_E is in the shared observer domain and Phi_f(z,t) satisfies C
                 at all declared constraint instants }.

The endpoint uses the same operating cone |v_y| <= tan(beta_max)*v_x as
the other prediction nodes, with beta_max = cfg.model.sideslipMaximum
(0.25 rad by default). There is no separate terminal sideslip parameter or
stricter endpoint cone. This domain belongs to the current observer's
force-balance inversion and error-bound contract; it is not a numerical
sideslip cap specified by Sharma et al. (2026). No additional bound on |r|*V is
imposed, either in membership or in the local optimization rows. The
braking ratio must satisfy actuator limits, and the configured braking rate
must lie inside the longitudinal tire capacity. These checks do not certify
combined-force realizability in the Fiala plant: applicability of the
simplified terminal flow remains the declared modeling assumption above.
No additional rear-tire nonsaturation condition is imposed. Since beta_E
is constant, the shared operating cone persists under the declared flow. Input
memory is part of the joint state; finite slew limits require an admissible
entry command, not just a geometric certificate.

The executable membership test checks:

1. The shared observer operating domain and terminal braking admissibility.
2. Ego reference-point deviation, and road occupancy when enabled, at nodes,
   midpoints, and its parked pose. Absolute ego position uncertainty tightens
   the deviation bound; no footprint radius or extra physical clearance is
   subtracted from it. `Inf` explicitly disables this bound.
3. Joint rectangle clearance during braking until the first exit.
4. If the ego stops before exit, its parked rectangle against the target's
   remaining in-range track, beginning at the exact stop time.

The fourth check is necessary: a target can collide with the ego shortly
after it stops while both remain in range. It excludes all motion after the
first exit. The check is analytic: target vertices sweep line segments or
circular arcs against ego edges, and vice versa. A target that stops inside
the disk contributes its stationary endpoint. If a circle stays wholly
inside the disk, its swept track still belongs to this encounter.

No integer terminal index is interpreted as an execution deadline. The
prediction endpoint can be a moving state with an available braking
continuation; the main controller need not choose that continuation.

### Shift closure and the role of uncertainty

For exact information under the declared terminal contract, take z in X_f.
The continuation of Phi_f(z,h) is the suffix of the original continuation:
the clipped-speed flow has the semigroup property, the target is advanced
by the same elapsed time, and input/encounter memory are retained. Every
active successor row is therefore an old certified row. Once the encounter
ends, no future prediction can reactivate it. Hence Phi_f(z,h) is in X_f.
Appending this rule to a shifted feasible plan supplies the usual
recursive-feasibility witness **only when the optimization transition and
policy class admit that same terminal rule**, as required in (U4). A soft performance objective does not remove
that candidate. This is an existence statement for the ideal problem;
local RTI solution accuracy and deadlines are separate.

For the unified observer-controller design, replace a point by a nonempty
information set J containing physical ego and target states, shared model
parameters, input memory, and the encounter flag. Add observer state only
when a predicted policy uses future observations. The terminal family is

    B_f = { J : one common causal terminal policy satisfies all hard rows
                on every admissible branch until that branch's first exit }.

The flag is branch-specific: a center outside range does not certify that
all possible targets have exited. A computed outer set may discard the
target only after every remaining active branch has exited. An unknown
parameter remains the same along each branch. Model mismatch already
represented in the current observer enclosure is not added again as an
independent initial error. Future physical model defects remain in the
physical images; the joint future-feedback model propagates their
shared effects on observer and controller as well.

For a common held braking rule with guaranteed realized deceleration
at least a_min > 0, every branch with V <= V_bar stops by V_bar/a_min and
travels at most V_bar^2/(2*a_min). Pose uncertainty remains after stopping.
All reachable parked poses and all still-active target branches must be
checked. A disturbance that can move a parked vehicle needs a compatible
parked invariant set. Holding an invalid low-speed observer bound constant
is not a substitute for this physical continuation certificate.

The terminal family is downward closed. Its exact joint image remains in
B_f because the same policy tail covers every successor branch, including
its input and encounter memory. A certified outer successor must likewise
belong to B_f. This is the terminal-closure step used in
[the recursive feasibility section](#recursive-feasibility) and
[the joint prediction section](#joint-prediction).

The runtime RTI is still an approximation to this information-set problem.
Its terminal geometry uses the target center and the published ego pose
padding; it does not implement a universally certified full nonlinear
terminal reachable set. This redesign does not promote those margins into
a robust certificate. `robustNonlinearSafetyCertified` remains false.

### Constraint-grid closure and implementation detail

A sampled terminal certificate must retain the constraint schedule under a
shift. The uniform nodes/midpoints are closed under a one-hold shift. The
exact stop and remaining parked track must be checked consistently as well.
An arbitrary recomputed grid would need an additional inclusion argument.
Invariance here concerns the declared constraint instants, unless a
continuous-time continuation enclosure is supplied.

The joint transition and terminal continuation use the first-exit convention.
The main RTI horizon no longer declares a future exit from a bounded reference
neighborhood: position translations remain unbounded; numerical trust bounds
on heading and dynamic states do not certify a perception-exit enclosure. It retains the active target's
collision rows until an observed exit.

Both ordinary numerical backends now use the same signed terminal function
`terminalSafeSet.nonlinearValue`. It evaluates the declared simplified braking
and parked-encounter continuation, together with the common operating-domain
conditions. Shooting imposes this function directly; RTI imposes its tangent
from `nonlinearTangent`, differentiating the same active branch. No additional
terminal turn-intensity or terminal-specific sideslip bound is introduced.
The finite continuation schedule, active feature and exit branch make this
an engineering local approximation, not a global nonlinear terminal
certificate. The older `rows` helper remains for standalone geometry and
certificate research probes; its separate fixed-side charts no longer define
the ordinary online terminal constraint.

The ordinary numerical tail uses the same nonlinear nominal-feedback seed
at moving endpoints, independently of terminal membership. Course and yaw
demands are limited before inverse-Fiala input construction; a large path
error therefore does not directly become a large linear steering demand.
An already stopped endpoint keeps a nonpositive longitudinal command.
The initializer does not mark this hold as a certified terminal extension:
the complete joint uncertainty tightening and terminal tangent remain part
of the subsequent optimization. In particular, pointwise membership alone
does not certify the front-tire domain for all future state/input errors.
This construction adds no optimizer iteration and does not change the
abstract terminal assumption or the separately retained certificate mode.
The optimizer still chooses the final input, subject only to the numerical
input-bound normalization described above. The `cruiseInput` and
`brakingInput` helpers remain available as standalone continuation probes;
they no longer construct ordinary online warm-start tails.

<a id="joint-prediction"></a>
## 6. Joint feedback-aware prediction

This supplies the causal observer/plant prediction used in Section 7.
Its held-input specialization recovers the physical information-set case. It is distinct from the runtime's
per-hold enclosure restart. Future measurements are unknown bounded variables.
Tube widths may vary, expand temporarily and retain a noise floor; a
constant-width invariant tube is optional. The model, observer, sensor and
terminal contracts must cover the same uncertainty family. Equations
(J1)--(J11) give the exact nonlinear construction, including target observation
and target-dependent ego control, before any supplementary representation.

### Exact nonlinear joint construction

The core theory uses exact nonlinear flows, a fixed decision horizon and
symbolic sensor/model bounds. Let sensor times be t_j=j*h_s and control
times t_(kL), with h=L*h_s and L a positive integer. The analysis has no
extra residual disturbance (d=0); uncertain tire stiffness and friction
remain persistent physical parameters. Additional disturbances require the
same extension in the vehicle, sensor, observer and terminal contracts.

Retain the augmented state

$$
 z=(x_E,x_T^{\rm phys},\vartheta_E,\zeta_E,\hat X_T,\hat y,m,u^-,\mathcal H).
 \tag{J1}
$$

Here x_T^phys includes target pose, speed, constant acceleration/curvature
parameters and stopping mode. The ego estimator state zeta_E contains its
position/velocity filters, heading and measurement memory. The target
estimate is Xhat_T=(rhohat,qhat,shat); yhat is its radar predictor; m contains
held sensor values; H contains the absolute observable history, clock and
policy memory. Physical parameters and their current states remain linked
by their physical evolution. They are not independent uncertainties.

A decision state is posterior to the measurements at that boundary. Its
current estimator states, measurement record and observable history are
known; physical quantities remain uncertain. A decision rule u=pi(H) is
constant on all tuples with the same H. At later decisions, different
measurement histories can generate different controls. This is an
optimization over causal rules, not just one future input sequence.

**Well-posedness premise.** Every admitted vehicle/observer flow and reset
must exist uniquely over every sensor interval, including terminal
continuations. A sufficient moving-vehicle domain has v_x bounded away
from zero, tire slip angles bounded away from their tangent singularities,
positive compact tire parameters, bounded velocities/yaw rate and locally
Lipschitz resistance. Matched Fiala saturation pieces are retained. A
low-speed/stopped extension, if used, must have a declared transition and
sensor law. No invalid branch may be dropped to obtain a smaller set.
No observation-error contraction or measured GNSS velocity is presumed.

### Sensor-period flow and causal target correction

The body-frame physical target coordinates are

$$
 \rho=R_E^\top(p_T-p_E),\quad q=R_E^\top\dot p_T,\quad
 s=R_E^\top\ddot p_T.
$$

For forward target motion, set t=R_E^T e(chi_T). Then
q=V*t, s=A*t+kappa*V^2*J*t and
j_phys=-kappa^2*V^3*t+3*A*kappa*V*J*t. Consequently,

$$
 \dot\rho=q-v_E-r_EJ\rho,\quad
 \dot q=s-r_EJq,\quad \dot s=j_{\rm phys}-r_EJs.               \tag{J2}
$$

The target course is chi_T=psi_T+beta_T, with constant
kappa=sin(beta_T)/l_rT. At a deceleration-to-rest event the physical pose
and course freeze, q^+=s^+=0, and the latent constant parameters remain
stored. Target yaw is retained at rest rather than reconstructed from
zero velocity.

Let a_1,a_2,a_3 be the target observer's effective innovation gains; for
the high-gain notation in the estimator theory, a_i=l_i*omega^i.
On a sensor interval the held gyro sample is omega_j=y_(r,j). The exact
observer flow and radar-predictor reset are

$$
\begin{aligned}
 \dot{\hat\rho}&=\hat q-\hat v_E-\omega_jJ\hat\rho
                       +a_1(\hat y-\hat\rho),\\
 \dot{\hat q}&=\hat s-\omega_jJ\hat q+a_2(\hat y-\hat\rho),\\
 \dot{\hat s}&=\Phi_e(\hat q,\hat s)-\omega_jJ\hat s
                       +a_3(\hat y-\hat\rho),\\
 \dot{\hat y}&=\hat q-\hat v_E-\omega_jJ\hat y,\qquad
 \hat y(t_j^+)=y_{\rho,j}.                                    \tag{J3}
\end{aligned}
$$

The estimated target state is continuous at a radar reset. The ego observer
has a deterministic sensor reset P_E, intersample flow f_O and outputs
H_E(zeta_E)=(phat_E,psihat_E,vhat_E), using position-only GNSS, IMU,
gyro and known inputs. These maps retain their internal memory and use
the same held sensor record as (J3).

One control transition is defined chronologically:

1. At the current posterior, select the common input u=pi(H). The IMU
   sample at this boundary was generated with the previous held input.
2. Flow both physical vehicles and both observers for h_s, holding u and
   the current sensor record. Include any declared target stop event.
3. Generate the endpoint measurements from these physical states and one
   shared noise vector. Apply the ego reset, reset yhat on detection, and
   append the complete readings and detection flag to H. Set u^-=u.
4. Repeat steps 2--3 L times without changing u. Stop just after the last
   reset, before selecting the next command.

With F^u the joint sensor-period flow and P^u_(nu,d) the endpoint
measurement/reset map, this defines

$$
 \mathcal G_h(z,\nu;\pi)=
 (P^u_{\nu_L,d_L}\circ F^u)\circ\cdots\circ
 (P^u_{\nu_1,d_1}\circ F^u)(z),\qquad u=\pi(\mathcal H).
 \tag{J4}
$$

The noise block contains fresh measurement primitives only. Physical
parameters are propagated unchanged, and each boundary sample is used once.
Detection is compulsory inside the symbolic sensing radius. With that
contract a missed detection certifies exit at its sample; otherwise exit
requires all states consistent with the history to be outside. The same
observable exit rule is retained under shifting. Ended histories have
vacuous collision obligations; reentry requires new initial feasibility.

### Exact nonlinear estimation and relative-tracking identities

Use estimate-minus-truth errors e_rho,e_q,e_s, e_y=yhat-rho and
e_v=vhat_E-v_E. Let w=omega_j-r_E(t) and lambda=e_y-e_rho.
Subtracting (J2) from (J3) gives

$$
\begin{aligned}
 \dot e_\rho&=e_q-e_v-\omega_jJe_\rho-wJ\rho+a_1\lambda,\\
 \dot e_q&=e_s-\omega_jJe_q-wJq+a_2\lambda,\\
 \dot e_s&=\Phi_e(\hat q,\hat s)-j_{\rm phys}
                       -\omega_jJe_s-wJs+a_3\lambda,\\
 \dot e_y&=e_q-e_v-\omega_jJe_y-wJ\rho.                         \tag{J5}
\end{aligned}
$$

The same gyro and ego-velocity errors enter the first and last rows.
Their cancellation implies, for 0<=tau<h_s following a detection,

$$
 \dot\lambda=-(a_1I+\omega_jJ)\lambda,\qquad
 \lambda(t_j+\tau)=e^{-a_1\tau}R(-\omega_j\tau)
                     [n_{\rho,j}-e_\rho(t_j)].                \tag{J6}
$$

This is the decay of the predictor innovation within a hold, not a claim
that the entire target estimation error contracts. The held-gyro difference
also contains true yaw-rate change:
w=n_(r,j)+r_E(t_j)-r_E(t). At target stopping,
e_s^+=e_s^-+s^- while e_rho and e_q remain continuous.

On a compact declared domain, the existing Lipschitz extension gives

$$
 \|\Phi_e(\hat q,\hat s)-j_{\rm phys}\|
 \le L_q\|e_q\|+L_s\|e_s\|+
                 \|\Phi_e(q,s)-j_{\rm phys}\|.                \tag{J7}
$$

The last term is zero only where the extension agrees with the physical
law. It and the stopping jump are part of the joint propagation.
For the ego output error e_E=H_E(zeta_E)-W(x_E), W=(p_E,psi_E,v_E),
the exact sensor reset is H_E(P_E(zeta_E^-,y,u^-))-W(x_E).
Its flow increment is the difference of the two exact output increments;
where differentiable, dot(e_E)=D H_E*f_O-D W*f_E. Thus e_v in (J5)
is generated by the same uncertain physical/measurement branch.

For a dynamically consistent comparison branch, write Delta for the
difference of corresponding quantities and alpha=psi_T-psi_E. Then

$$
\begin{aligned}
 \Delta\dot x_E&=f_E(x_E,u;\vartheta_E)
                    -f_E(\bar x_E,\bar u;\bar\vartheta_E),\\
 \Delta\dot\rho&=\Delta q-\Delta v_E-r_EJ\Delta\rho
                                      -\Delta r_EJ\bar\rho,\\
 \Delta\dot\alpha&=\Delta(\kappa_TV_T)-\Delta r_E.              \tag{J8}
\end{aligned}
$$

For a causal law u=K_i(c), c consists of available target/ego estimates and
memory. Its input difference is K_i(c)-K_i(cbar), with
Delta(rhohat)=Delta(rho)+Delta(e_rho), and similarly for q,s,v_E.
Equation (J5) therefore connects future target observations to (J8) through
the actual nonlinear force law. The first input difference is zero when
histories are indistinguishable.

### Joint reachable sets, posterior fibres and feedback benefit

For an admissible fixed rule define the exact successor

$$
 T_\pi(S)=\{\mathcal G_h(z,\nu;\pi):z\in S,\ \nu\in{\rm Adm}(z)\},
 \quad \mathcal R_0=I_k,\quad
 \mathcal R_{i+1}=T_{\pi_i}(\mathcal R_i).                      \tag{J9}
$$

Adm includes the observation modes and any noise-continuation memory.
A rule is admissible only if every branch has a well-defined flow and
satisfies its hard constraints. Sets retain history tags. A fibre
R_i^eta contains the tuples with one particular observed history eta;
the union over histories is the unconditional prediction. Each fibre
receives one common command.

For any defined relative-error projection D_i, its exact error set is
D_i(R_i). Component bounds can be defined by

$$
 B_{i+1,a}=\sup_{z\in R_i,\nu\in{\rm Adm}(z)}
       \|D_{i+1,a}(\mathcal G_h(z,\nu;\pi_i))\|.               \tag{J10}
$$

On a declared bounded finite-horizon domain these bounds are finite.
This is a sound theoretical construction, not a general theorem that a
small B exists. On a fibre with reading y_rho, true relative position has
diameter at most 2*nbar_rho, but different fibres can have different centers.
Small conditional estimation error does not shrink physical target
dispersion about one frozen forecast.

A checkable mathematical feedback criterion is obtained by grouping states
with the same observable statistic c into S_c. Let L_c(u) be the supremum of
the next relative-position error under a held u, taking the exact joint image,
and exclude commands violating any hard condition. Define

$$
 U_c(B)=\{u\in\mathcal U:\text{hard conditions on }S_c,\
                         L_c(u)\le B\}.                      \tag{J11}
$$

An admissible selector K(c) in U_c(B) guarantees that one-step relative
bound. If every fibre admits such a selector but the intersection of
all U_c(B) is empty, observation-dependent feedback achieves a bound that
no common command achieves. Nonemptiness and any required regularity of
the selector are substantive conditions, not consequences of bounded
sensor noise. The finite-horizon successor/terminal constraints enforce
their compatibility across later decisions.

A symbolic strict-improvement example stays within the Fiala model.
In straight motion with zero resistance and steering, the exact acceleration
is gamma*b, with |gamma/gamma_0-1|<=epsilon<1. Two target hypotheses start
with the same speed v and headway, zero curvature, and A=+a or -a.
Set h=2*h_s and apply b=0 during the first hold. Initialize both hypotheses
with the same exact relative position and velocity, shat=0 and yhat=rho,
and an exact ego estimate. Use zero sensor noise and choose the moving domain
so that the observer extension agrees with the straight-motion law.
With tau=h_s, the intermediate radar reset gives

$$
 \hat s_x(h)=\sigma\,\frac{a_3a\tau^2}{2a_1}
                         (1-e^{-a_1\tau}),\qquad \sigma\in\{-1,1\}.
$$

The causal next command b=sign(shat_x(h))*4*a/gamma_0 is admissible if
4*a<gamma_0 and the trajectories remain in the moving/sensing domain.
At 2*h, the target deviation from constant-speed motion is sigma*D,
D=2*a*h^2, while the ego correction is sigma*(gamma/gamma_0)*D.
The relative-error bound is epsilon*D. Any common ego endpoint has
worst-case error at least D against the two target endpoints. Small
nonzero noise and parameter perturbations preserve the strict inequality
by continuity while the observer estimate keeps its sign margin.
This proves existence of a strict information benefit, not a uniform
small-tube claim for arbitrary encounters.

The complete posterior construction and shift proof follow in Section 7.
The representations below are supplementary comparisons and enclosure
methods; they are not additional premises of the exact construction
(J1)--(J11).

### Supplementary feedback-rule representations

An open-loop plan fixes u_0,...,u_(M-1). A feedback plan fixes causal rules
pi_0,...,pi_(M-1). Each future rule maps the information available at that
sample to an input; it cannot use later measurements. The current observation
is already known, so pi_0 returns a definite optimized current input.

For nominal ego state z and input v, a local rule is u=v+K*(xhat-z).
Define observer error epsilon=x-xhat and physical tracking error eta=x-z.
For sampled plant F and a declared defect w,

    eta_next = F(z+eta,v+K*(eta-epsilon)) - F(z,v) + w.       (1)

The linear version is

    eta_next = (A+B*K)*eta - B*K*epsilon + w.                 (2)

The propagation matrix is A+B*K, not A. Observer error enters as a bounded
feedback disturbance. Input constraints must cover K*(eta-epsilon), including
input memory/slew. Clipping an input outside this model changes the policy.

### Propagate a varying closed-loop tube over the finite horizon

Let q contain physical deviations and actual observer/software state, together
with reference, input and mode memory when needed. The current software state
and observation are known. The initial set retains the observer-certified
physical uncertainty, while the current policy returns one common input.
For each future hold, the joint map G_i includes the nonlinear vehicle,
bounded sensor history, actual observer update and chosen causal policy.
Construct sound outer sets satisfying

    Omega_(i+1) contains {G_i(q,nu): q in Omega_i, nu in W_i}.

The physical tube at node i is the projection of Omega_i, with separate
sound hold/midpoint images as required by the constraint contract. There is
no requirement Omega_(i+1) subset Omega_i or that all projected widths equal
a single radius. Different nominal states, gains, observation quality and
noise contributions can produce different widths. Keep the same policy/set
meaning when shifting the plan and conditioning on the next measurement.

A comparison implementation can use time-varying certified coefficients,

    E_(i+1) <= a_(o,i)*E_i+d_(o,i),
    r_(i+1) <= rho_i*r_i+gamma_i*E_i+d_(F,i).

These describe future observation correction and physical tracking, rather
than a frozen-input plant-only growth factor. Temporary expansion is allowed.
No common contraction metric or constant invariant radius is prerequisite for
finite-horizon propagation. Compact scalar comparisons must still be sound;
joint generators can retain correlations that scalar boxing discards.

For a linear joint surrogate q_next=J_i*q+D_i*nu+c_i, propagate a center m
and generator Z by m_next=J_i*m+c_i and Z_next=[J_i*Z,D_i]. Take directional
supports after the matrix products. Repeatedly propagating coordinate widths
with abs(J_i) loses cancellation between the plant and observer and may itself
create large artificial tails. Initial uncertainty is not restarted at each
hold, and primitive measurement noise is not added again as an independent
observer-error disturbance. The first map uses the known current input;
subsequent maps include the future feedback law and its gyro feedthrough.

Small future estimation error about each branch's updated estimate does not
by itself bound physical deviation from a frozen nominal trajectory. The
feedback policy's input variation must therefore be part of G_i. Replanning
in experiments alone does not specify that prediction policy. Unknown future
measurements remain bounded variables, not known or noise-free values.

For finite outer representations, the remaining certification work is to construct sound finite-horizon images,
admit their policy-generated inputs and constraints, and connect the endpoint
set to the robust terminal continuation. Once entering the mild terminal
model, braking and a fixed physical parked pose set may be certified without
requiring continued contraction of the noisy yaw estimate. Recursive
feasibility uses the cross-sample inclusion and terminal shift argument below;
it does not require a constant prediction-tube width.

### One uncertainty source and its different consequences

Let theta be a fixed unknown vehicle-parameter vector in Theta. The initial
project family varies front/rear cornering stiffness and friction within the
observer's declared relative box (default 5%); mass, inertia and geometry are
not implicitly uncertain merely because that box is present. Let d_i be any
separately declared residual plant disturbance and nu_i the primitive sensor
noise history. The tire box restricts parameter values; constancy or a drift
rate does not follow from a value bound alone. Actuator errors and external
forces require compatible bounds if admitted; the tire box does not cover
arbitrary additional forces.

Take decision instants after the measurements used for that input are known.
Let q contain the physical ego/target state X, theta, actual observer point
and bound/history state s, the available measurement record y, previous input,
mode/encounter memory, and the policy's reference memory where necessary.
The observer uses its specified nominal model and parameter bounds. The true
plant uses theta. The chronological sampled map is

    u_i = pi_i(s_i,y_i,reference_i),
    q_(i+1) = G_(pi_i)(q_i,d_i,nu_i),   theta_(i+1)=theta_i.       (M1)

G executes the plant, measurement generation, predictor resets, observer
integration and reconstruction in their actual sample/hold order. Future
measurements come from the same physical branch. A raw gyro channel is a
measurement, not privileged true yaw rate. Its noise is sampled once and
shared by every path through the observer and control. G also defines the
held-input trajectory images at constrained instants and over the full hold.

If parameters may vary, replace the identity in (M1) by a declared transition
theta_next in Gamma(theta), including any intra-hold parameter history. A
range-only contract can use Gamma(theta)=Theta, without asserting a drift
rate. Retain one shared parameter history in plant and observer effects.
The outer-image and shift proofs apply with that same transition at both
frames. Equations (M3), (M4) and the constant-parameter term in (M8) specialize
to fixed parameters; they must not be used for a broader varying family.

At the root, current s and y are known: all physically possible branches
receive the same current input. Future s and y become branch dependent. The
root information set is

    I_k = {q consistent with the retained information history,
           X in the published physical enclosure,
           theta in the retained parameter set,
           s=s_k, y=y_k and known input/mode memory}.             (M2)

The product of a published enclosure and Theta is a sound initialization if
only marginal information is available; it can lose correlation with theta.
Subsequent propagation retains the same theta and conditions the joint set.
No new independent parameter draw is added at each hold.

The published error enclosure already contains effects of past admitted
mismatch and noise. It initializes physical uncertainty and can further
condition predicted sets. It is not another independent disturbance to inject
after G has propagated those same sources through the observer. Future plant
mismatch nevertheless acts again through the plant dynamics. Describing that
effect is not adding the past state error for a second time.

For an exact linear illustration, let

    x_next=A*x+B*u+P*deltaTheta+w,   y=C*x+n,
    xhat_next=A*xhat+B*u+L*(y-C*xhat),
    z_next=A*z+B*v,                 u=v+K*(xhat-z).

With eta=x-z and epsilon=x-xhat, the joint recursion is

    [eta_next       ]   [A+B*K  -B*K  P] [eta       ]   [I   0] [w]
    [epsilon_next   ] = [0      A-L*C P] [epsilon   ] + [I  -L] [n]
    [deltaTheta_next]   [0      0     I] [deltaTheta]   [0   0].    (M3)

The block notation uses a final zero row in each disturbance column. The
same w and deltaTheta enter both error equations. In the difference
eta_next-epsilon_next their direct contributions cancel, as the observer's
nominal point update requires. Independent copies would destroy this identity.
This is a timing-specific illustration; (M1) handles actual feedthrough.

For a scheduled affine error block r_next=J_i*r+P_i*deltaTheta+D_i*nu+c_i,
preserve the persistent parameter generator by

    H_(i+1)=J_i*H_i+P_i*G_theta,
    Z_(i+1)=[J_i*Z_i,D_i],   m_(i+1)=J_i*m_i+c_i.              (M4)

H and Z are concatenated only when taking supports, with any initial shared
columns preserved. Equivalently propagate theta as a state with identity
transition. Such an augmented matrix has unit parameter eigenvalues even
when its tracking/observer block is stable; a Schur test on the entire
parameter-augmented matrix is not a requirement. A constant parameter may
produce a bounded tracking offset. Neither (M3) nor (M4) bounds the nonlinear
remainder without the construction below.

**Observer compatibility.** Every published enclosure used in (M2) or a
posterior must be valid for the same Theta, residual-disturbance contract and
sensor bounds used in G. Extra lateral force, for example, changes the
force-balance measurement premise as well as the plant. Its force bound must
enter that measurement's interval calculation; adding it only to plant
prediction while keeping an incompatible observer certificate is unsound.
The current 5% tire family can be studied with no extra residual disturbance.
For an added disturbance, certificate validity must first be extended.

While relying on a finite observer certificate, its continuous-motion
premises must hold over each whole hold. Checking the sideslip/domain premise
only at nodes or midpoints does
not prove it between those instants. Collision clearance remains a separate
node/midpoint contract unless a continuous-time clearance image is certified.
In a terminal mode outside that certificate's domain, retain the propagated
physical set and use an unrestrictive Q instead of a finite invalid enclosure.

### Construct sound nonlinear images, then propagate by induction

Define the exact joint image for a fixed causal rule by

    T_pi(J)={G_pi(q,d,nu): q in J, (d,nu) in the declared history set}.

It retains theta and every shared source and is monotone in J. For each
stage construct an outer set R_(i+1) with

    T_(pi_i)(R_i) subset R_(i+1).                              (M5)

The initial containment (M2) and induction on (M5) prove containment of every
allowed nonlinear trajectory for the entire finite horizon. Initial errors
are not restarted at each hold. Widths can vary, and (M5) requires neither
contraction nor an invariant constant radius.

Here is a constructive way to certify (M5). Partition the relevant joint
state/parameter/noise domain into cells. On a convex cell C for the combined
argument a=(q,d,nu), choose a_C and a matrix L_C. If G is continuously
differentiable on C and a validated convex matrix-box enclosure of
DG(C)-L_C is M_C, then

    G(C) subset G(a_C)+L_C*(C-a_C)+M_C*(C-a_C).                 (M6)

**Proof.** Along the segment from a_C to a,
G(a)-G(a_C)=integral_0^1 DG(a_C+t*(a-a_C))*(a-a_C) dt.
Subtract L_C*(a-a_C); the average remaining Jacobian belongs to M_C by its
convexity. An interval hull of its matrix-vector product is a valid reserve.
For coordinate radii rho and entrywise absolute bound Delta on M_C, the
coordinate reserve is Delta*rho. Keep the affine joint generators before
adding this reserve, so shared theta/noise columns remain shared. QED

If component j has a validated Hessian bound |D^2G_j|<=H_j, a tighter smooth
cell reserve is one half rho'*H_j*rho. These bounds are over the entire cell,
including parameter-state and input-state interactions. A numerical Jacobian
at its center or finite differences at two step sizes do not establish them.
Physical/model mismatch belongs in G before linearization; the reserve covers
the error of approximating that uncertain nonlinear G, rather than a second
copy of the same parameter mismatch.

For switching or discontinuous algorithm branches, split the guards and take
the union of images of every possible branch. The Fiala saturation, low-speed
blend, stop reset, angle conventions and force-balance bisection/fallbacks
cannot all be covered by assuming one globally differentiable map. Interval
extensions of the executed primitive operations give an alternative to (M6).
If an exact inverse replaces the implemented bisection for analysis, its
finite termination error must also be enclosed. Numerical plant integration
is the declared map, or its defect from a continuous plant is included once
in the residual contract; those are distinct model choices.

A monotone outer operator can be defined on a fixed covering partition:
union the validated cell images for all cells intersecting J. A subset of J
then selects a subset of cells. Bounds must remain valid for the cell's whole
domain and the fixed policy. This supplies a constructive sufficient operator,
but can be computationally costly or too wide. An adaptive enclosure need not
be globally monotone; the certificate-carrying proof below avoids requiring
every fresh enclosure to be narrower than its predecessor.

For input/handling/collision rows, bound the input image pi(R_i) and the
appropriate held-state images from the same joint cells. A state tube alone
does not certify input reserves. Nonlinear rectangle rows require sound
geometry bounds as well. All these checks are constraints in the mathematical
optimization; the optimized current input is still executed directly.

Every certified cell must lie in a domain where the plant, observer and
policy updates are defined. An undefined branch cannot be silently omitted
from an enclosure. It must be excluded by a valid domain certificate or
handled by an explicitly declared mode/fallback in G.

### Finite-horizon widths with persistent mismatch and nonlinear reserves

On the certified cells, write the error projection as

    r_(i+1)=J_i*r_i+P_i*deltaTheta+D_i*nu_i+c_i+zeta_i,
    zeta_i in the certified remainder set U_i.                 (M7)

Each U_i encloses the actual joint-map residual on R_i, including parameter
interactions. Bounding residuals independently is a permissible outer
approximation; it must not introduce another copy of parameter or sensor
terms already explicitly present in (M7).

Let Phi(i,j)=J_(i-1)...J_j, with Phi(j,j)=I. For independent root generators
G_0 and G_theta, the parameter response and nominal center are

    H_i=sum_(j=0)^(i-1) Phi(i,j+1)*P_j,
    m_i=Phi(i,0)*m_0+sum_(j=0)^(i-1) Phi(i,j+1)*c_j.

For every direction ell, a finite-horizon support bound is

    |ell*m_i| + ||ell*Phi(i,0)*G_0||_1
       + ||ell*H_i*G_theta||_1
       + sum_j ||ell*Phi(i,j+1)*D_j||_1
       + sum_j support_abs(U_j,ell*Phi(i,j+1)).                 (M8)

Here sensor variables are normalized unit boxes, and support_abs(U,a) is
sup_(zeta in U)|a*zeta|. Shared initial/parameter/noise columns are combined
before taking absolute values; (M8) displays the simpler independent-root
case. Any declared temporal noise correlation can also be retained instead
of replaced by independent boxes.

**Proof.** Unroll (M7), collect all occurrences of the same deltaTheta into
H_i*deltaTheta, and take directional supports of the resulting Minkowski
sum. This proves a varying nonlinear outer tube whenever each remainder
bound has been certified on the tube where it is used. Taking absolute
values before collecting the parameter terms loses cross-step cancellation.
No fixed width, cap or zero future noise enters the derivation. QED

The matrices in Phi describe observer and feedback dynamics, not a
frozen-input plant. If their error-block products satisfy
||Phi(i,j)||<=c*rho^(i-j), rho<1, and the parameter/noise/residual forcing is
uniformly bounded, the geometric sum gives a bounded physical-error tail.
Individual steps may expand; persistent parameters still have identity
dynamics. This is a sufficient mechanism, not an imposed constant-radius
objective. For a finite horizon one can verify (M8) and the associated domain
and constraint inclusions step by step without such a uniform inequality.
This error-block stability condition is not a claim that the uncontrolled
target's absolute reachable set is bounded; safety uses joint/relative images.
The current local Schur matrices alone do not establish either a uniform
nonlinear transition-product bound or validated U_i for the actual 5% family.

### Include the actual observer update in the prediction

The proposed joint model should propagate the physical ego/target state X,
the actual observer runtime state s, nominal policy/reference memory z,
observer bound/set state B, and held-input memory. At a future sample:

    estimates_i = reconstruct(s_i, measurements_i, B_i),
    u_i = pi_i(estimates_i, z_i, B_i),
    X_(i+1) = plantHold(X_i,u_i,defects_i),
    s_(i+1),B_(i+1) = observerHoldAndUpdate(s_i,B_i,u_i,sensorHistory_i),
    z_(i+1) = nominalPolicyUpdate(z_i, estimates_i, u_nominal_i).

The sensor history is generated consistently from the same physical branch,
timestamps, held inputs and bounded noise. Propagate all admitted noise/defect
histories. Feedback uses available estimates, never the unknown true state.
The known current s_i is common to all initial branches; uncertainty starts
from the current physical/parameter enclosure, not arbitrary independent
perturbations of the known software state. Future s becomes branch-dependent
through its measurement update.

The present NRMM runtime's numerical point state is 15-dimensional:
body velocity (2), inertial position (2), GNSS predictor (2), target core (6),
radar predictor (2) and yaw (1). Synchronization, orientation sets, history and
bound updates add information memory. `nrmmObserverVectorField("integrate",...)`
supplies the RK4 point flow; `onlineNrmmTrackingRuntime` resets measurement
predictors, updates sets/bounds, then advances held measurements and reconstructs
outputs. The augmented predictor must follow this order and sample/hold contract.
The eight-state continuous core certificate is useful but is not itself the
complete published ego/target-state update.

This is a linear explanatory derivation, not our NRMM observer equation.
For the nonlinear implementation derive the joint map/Jacobian or certified
comparison dynamics from the actual point flow, reconstruction and bound
updates. Validate sound finite-horizon joint images over the admitted nominal,
error and input domains, including midpoints and mode transitions.
Stable observer ISS and the existing path CLF do not by themselves certify
incremental joint contraction. The uncontrolled target's absolute-position
spread remains; relative/coupled safety coordinates are still needed.

Local-map and nonlinear-replay results are recorded separately in
[the actual NRMM prediction report](../report/JOINT_NRMM_PREDICTION_20261013.md).
Those numerical probes do not provide uniform nonlinear remainder bounds.

Including the updater enables a tighter causal description; it does not
automatically guarantee a smaller tube. A poor outer approximation can still
discard its correlations. The full updater can be used for offline proof and
coefficient construction, with compact certified radius/generator recursions
online. Observer states need not all become optimization decision variables.
The shifted-policy proof below applies to this augmented transition when its
outer images are sound and their causal-policy witnesses and certificates
are retained. A monotone outer operator is one sufficient construction.

### Quantify the nonlinear and mode-dependent extension

At admissible nominal straight cruise above the blend/stop regime, sampled Fiala F
has continuous first derivatives locally. Stable A_c admits a contracting
metric. Continuity gives a sufficiently small neighborhood with contractive
eta derivative in (1) and bounded epsilon derivative. The mean-value argument
then gives (3) for compatible sufficiently small observer errors. This proves
local existence, not a quantified tube for the current NRMM bounds or the
entire 5% parameter box. Compute
and certify its domain, nonlinear/decision reserves, input reserve and hold
fractions. Empirical rollout maxima are not universal bounds.

Moving nominal trajectories need scheduled gains/metrics, for example
A_ci'*P_(i+1)*A_ci<=rho^2*P_i plus nonlinear reserves. The current five-state
path CLF omits longitudinal position; it is not automatically an incremental
certificate between two full trajectories. At low speed the model blends,
steering authority changes, and the stop reset switches maps. A positive-speed
contraction proof cannot simply continue through those modes. A certified
braking transition can end in a parked policy that keeps a nonzero pose set
fixed, without chasing noisy velocity estimates.

### Preserve correlation with the uncontrolled target

Two admitted target speeds V+/-delta produce positions separated by 2*delta*t.
Perfect later measurements distinguish the trajectories but do not make them
coincide. A small observer error about each branch's future estimate is not
a small physical reachable set about a single frozen forecast.

Future policies should depend on ego and target observations. Ego trajectories
then depend on the same target/measurement branch as target trajectories.
A target-relative reference or causal policy/tree retains this relationship.
Check relative rectangle clearance on coupled branches: independent marginal
unions can lose correlation and add unnecessary tightening. A target-relative
reference must still obey rate/input conditions; it cannot handle arbitrary
target maneuvers by assertion. Road-free translation-invariant relative
geometry is useful; the soft path-recovery objective can remain. Shared pose
and gyro uncertainty should retain its correlations.

### Braking in inertial-velocity coordinates

There is a concrete correlation already published by this project's
`nrmmControllerErrorBounds`: its yaw generator also changes inferred body
velocity by [v_y; -v_x] times the yaw radius. In the inertial velocity
R(psi) v, this first-order contribution cancels the corresponding rotation
term. A large body lateral-velocity coordinate radius can therefore mostly
represent uncertain coordinates, rather than equally large uncertain
inertial lateral motion. Dropping this column correlation charges both
terms separately. During deceleration the future velocity dependence must
be recomputed by the joint map; copying a high-speed publication to a
low-speed endpoint does not describe the future observer's uncertainty.
This observation does not authorize discarding genuine physical velocity
error or scaling its bound by a guessed speed ratio.

### Coordinate projection and its reserve

Keep inertial velocity in the joint propagation and reconstruct body velocity
at each nominal pose. Let R_i rotate body coordinates to inertial coordinates,
J=[0,-1;1,0], and vB_i=R_i' vI_i. Shared generator columns give

    G_vB,i = R_i' G_vI,i - J vB_i G_psi,i.

This is a coordinate projection of the propagated joint set, not a rule for
shrinking the present observer bound. The yaw column is multiplied by the
future nominal velocity. Its lateral contribution therefore tends to zero
as that velocity tends to zero, even if heading uncertainty does not shrink.
The inertial-velocity residual, measurement noise, parameter mismatch and
tracking uncertainty remain. Apply the same construction separately to the
physical and estimator projections of the joint set. A predicted small
estimation error alone does not bound physical departure from the nominal.

The rotation projection has a simple nonlinear reserve. If |deltaPsi| <= a
and ||deltaVI|| <= b, the norm of the omitted terms is at most

    2 sin(min(pi,a)/2) b + 0.5 ||vI_i|| a^2.

The first term bounds the product of velocity and rotation errors; the
second bounds the rotation Taylor remainder. It is valid for correlated
errors too. Add a sound enclosing ball or box for this reserve while
retaining the shared linear columns. This reserve certifies only this
coordinate transformation, not the preceding observer/vehicle propagation.

The numerical RTI implementation now retains the factors of the current
GNSS/yaw publication instead of copying its evaluated high-speed matrix to
every future hold. Let `a` be the published yaw radius, `epsV` the GNSS
velocity radius and `d` the componentwise absolute difference between the
rotated GNSS measurement and the observer velocity. At a nominal body
velocity `v_i`, the shared yaw column is `-J*v_i*a`. The independent velocity
columns enclose the GNSS ball and the residual box, with additional radius

    0.5*norm(v_i)*a^2 + min(a,2)*norm(d).

The latter term permits every residual sign in the box and includes its
rotation product. Sensor noise and residuals are not multiplied by a speed
ratio. The first hold retains the actual published matrix unchanged, and
user-supplied unfactored matrices retain their existing behavior.

This is a **conditional state-dependent geometry correction** to the
existing per-hold approximation. Future yaw and residual budgets are still
assumed equal to their present budgets; the correction does not prove that
future observer innovations satisfy that assumption. Metadata explicitly
reports `futurePrimitiveBudgetsAssumed=true` and
`futureObserverDynamicsIncluded=false`. A force-balance/model-mismatch
contribution in the current observer residual is retained, but this is not
a bound on all future mismatch effects. The full joint propagation below
remains necessary for such a claim. A smaller scheduled velocity radius is
not a new robust nonlinear recursive-feasibility certificate.

The integration sequence is: initialize from the published correlated set;
execute the definite current input; propagate the plant, persistent mismatch,
sensor samples, actual observer updates and future feedback jointly; project
the resulting physical/estimator sets into the next nominal body frame;
then form constraint supports. Future noise is introduced at its actual
sample, not replaced by a fresh copy of the current error box. A prototype
can use maneuver-dependent numerical joint maps offline, but those maps need
validated nonlinear reserves before they supply certified runtime bounds.

The sample-end rest reset is another branch of this joint map. The plant
replay and observer truth samples must apply it at the same control boundary
as prediction. If the admitted set straddles the speed threshold or the
braking/driving boundary, enclose both reset and non-reset images. A tangent
with zero velocity rows on the stopped branch cannot cover the moving
branch. This is compatible with the exact-set recursive-feasibility theorem;
it is an additional requirement for an approximate implementation.

### Optional constant-width diagnostic

Suppose the observer has a predictable envelope B_next<=a_o*B+d_o, a_o<1,
and (1) has a certified weighted-norm bound

    r_next <= rho*r + gamma*B + d_F,     rho<1.               (3)

Fixed widths can satisfy

    B_bar >= max(B_now,d_o/(1-a_o)),
    r_bar >= max(r_now,(gamma*B_bar+d_F)/(1-rho)).              (4)

If the nonlinear reserve depends on the tube/decision domain, solve
rho*r_bar+gamma*B_bar+d_F(r_bar,B_bar)<=r_bar with input/state admission.
This is an invariant-tube condition. No future measurement realization or
vanishing noise is assumed. A radius above its noise floor may decrease;
a radius below the floor can increase. Strict decrease from every arbitrarily
small initial width is therefore not promised. A fixed invariant width is
one possible sufficient design, rather than the user's requirement. Held-input midpoints need their own
envelope; it may vary with hold fraction but must repeat without secular growth.

Koehler, Mueller and Allgoewer (2021), *Robust output feedback model predictive
control using online estimation bounds*, Proposition 2, equations (24)--(26),
and Theorem 4 give a nonlinear framework with observer/tracking bounds and
recursive feasibility. Applying it here requires our own domain, actuator and
terminal certificates. [Primary source](https://arxiv.org/html/2105.03427v1).

<a id="recursive-feasibility"></a>
## 7. Unified observer-to-safety formulation

The theoretical object is a nonlinear output-feedback predictive barrier.
It uses the joint future observer/plant evolution from the beginning; it does
not first replace future feedback by open-loop error propagation. Exact-state
MPC is the singleton-information special case. This section specializes the
shift construction in Huang et al., problem (8) and Lemma III.5, while making
its additional observer, moving-target and terminal assumptions explicit.

### Joint information and one copy of uncertainty

Let $q$ be the history-tagged augmented state (J1): true ego and target
states, observer internal states, held sensor records, persistent uncertain
parameters, previous input, absolute observable history and policy memory. A future command has the causal form
$u_i=\pi_i(o_i)$, where $o_i$ consists only of information available to the
controller at that future instant. In particular, it cannot depend on the
unknown true error or future measurement noise. Affine feedback about a
nominal plan is one possible policy class, but the class must allow the shift
and terminal append used below.

Use the exact sensor-period flow/reset composition (J4), including all
intermediate readings and observer updates. Write its complete control-period
transition as

$$
 q^+=\mathcal F(q,\pi(o),\omega),\quad \omega\in\Omega,
 \qquad \vartheta^+=\vartheta.                                 \tag{U1}
$$

The same $\vartheta$ influences the plant and observer residual. A measurement
primitive appears once and is shared between every update using that sensor.
The initial estimation error belongs to the initial information set; it is
not reinjected independently at every future step. Future physical mismatch
still acts through (U1). These distinctions prevent double counting without
pretending that mismatch disappears after estimation.

Let $\mathcal T(R,\pi)$ be the exact image under (U1), and let
$\mathcal T_\tau(R,\pi)$ include a declared checked instant within a hold.
These maps are monotone in $R$ for a fixed policy. The theory uses sound outer
images $\mathcal T(R_i,\pi_i)\subseteq R_{i+1}$, with stored proofs or exact
sets. It neither assumes constant tube width nor requires contraction at every
step. Joint affine propagation without a full-domain remainder is an
engineering approximation of this object.

### Prediction and measurement conditioning must agree

Let $Q_{k+1}$ be the sound current observer enclosure, including the known
observer internal state. Retain the previous first successor and form

$$
 I_{k+1}=R_{1|k}\cap Q_{k+1}\cap A_{k+1},\qquad
 q_{k+1}\in I_{k+1}\subseteq R_{1|k},                            \tag{U2}
$$

where $A_{k+1}$ selects the **entire received sensor block**: intermediate
readings, detection flags, resulting known observer states and input history.
Induction through the physical flows and resets of (J4) keeps the true branch
in $R_{1|k}$; that branch also satisfies these observed facts and any valid
additional enclosure $Q_{k+1}$. This establishes both inclusions in (U2).
At initialization $I_0=Q_0$. A raw new observer ball need not be contained in the
previous one; the intersection with the **predicted successor**, not a
recentered ball, gives the inclusion needed for shifting. Any outer numerical
representation of the posterior must still lie inside that successor.
A missing reading imposes no numerical measurement restriction, but its
detection flag still conveys any information specified by the sensor contract.
In particular, guaranteed in-range detection makes a miss an exit certificate
at that sample. An empty intersection violates the declared model/information contract and cannot be
silently repaired by discarding the old information.

A new target, a changed uncertainty contract, or a genuinely new encounter
requires a new initial-feasibility argument. During an encounter the target's
motion parameters are propagated and conditioned, not independently reset.
An actual, or robustly established, perception exit ends this encounter;
nominal predicted exit alone cannot remove constraints for uncertain members
that remain nearby. Subsequent reentry is a new encounter. None of the
following assertions covers arbitrary unmodeled later encounters.

### One downward-closed constraint and terminal family

Let $H(R_i,\pi_i)$ express the hard input, input-memory and model-domain
conditions for all members at the declared checked instants. No road boundary
is included in the present theoretical task. Define $C(R_i,\pi_i;w_i)$ as the
least nonnegative slack for (G2) at those instants, with witnesses $w_i$ and
the target inactive only under the encounter contract. Endpoint clearance
uses $C_N(R_N;w_N)$. Retaining a witness gives

$$
 R'\subseteq R\ \Longrightarrow\
 C(R',\pi;w)\le C(R,\pi;w),\quad
 H(R,\pi)\Longrightarrow H(R',\pi).                            \tag{U3}
$$

**Abstract terminal-pair assumption.** There exists a nonempty,
downward-closed terminal family $\mathfrak B$ of nonempty joint information
sets, with an admissible causal terminal controller. For every
$R_f\in\mathfrak B$, this controller supplies a **common causal** policy
$\pi_f$, zero-deficit witnesses and a successor $R_f^+\in\mathfrak B$
satisfying

$$
 H(R_f,\pi_f),\quad C(R_f,\pi_f;w_f)=0,\quad
 \mathcal T(R_f,\pi_f)\subseteq R_f^+,\quad C_N(R_f^+;w_f^+)=0.  \tag{U4}
$$

The symbols $\mathcal T$, $H$ and $C$ in (U4) are exactly those used in the
prediction problem. Thus the assumption includes the same uncertainty,
geometry and admissibility requirements, including common separating
witnesses when (G2) is used. The policy cannot select an input from an
unknown true state; its information and memory follow the same causal rules
as the predicted policies. Restricting a certified set retains the policy
and witnesses, and terminal appending retains the required memory.

Existence with these properties is a premise of the core theorem, not a
construction that the theorem promises to deliver. No particular braking
model, stopping distance, terminal shape or terminal-mode switch is required
by the proof. Section 5 is one concrete implementation candidate. Whether
that candidate realizes (U4), including its transition and geometric
witnesses, is a separate implementation-validation question. A pointwise
collision-free braking family is not automatically the abstract
$\mathfrak B$ when the optimization requires a common separating plane.
An unverified candidate does not invalidate the conditional theorem; it
does not establish that this implementation satisfies the theorem's premise.

### The theoretical nonlinear PCBF problem and control law

Fix a finite $N$ at encounter initialization and use the same stage timing on
successive shifts. The baseline checks collision at decision instants. A plan consists of causal policies, nonempty joint sets,
continuous geometric witnesses and nonnegative slacks. Its conditions are

$$
\begin{aligned}
 R_0&=I_k,& \mathcal T(R_i,\pi_i)&\subseteq R_{i+1},\\
 H(R_i,\pi_i)&,& C(R_i,\pi_i;w_i)&\le\xi_i,
             && i=0,\ldots,N-1,\\
 C_N(R_N;w_N)&\le\xi_N,& R_N&\in\mathfrak B,
             &\xi_i&\ge0.                                    \tag{U5}
\end{aligned}
$$

Under (U4) the endpoint slack is redundant and can be chosen zero.
Decision instants, including the endpoint, define the present baseline.
Additional interior checks must use a shift-invariant schedule; whole-hold
versions require all $\tau\in[0,h]$. Neither extension is presumed here.
The paper permits a soft prefix of length $N_\xi\le N$ followed by a
zero-slack hard tail. Its shift also removes the first slack and appends zero,
so the same proof applies; $N_\xi=N$ gives (U5)--(U6).

Define $J_P=\sum_{i=0}^N\xi_i$. The theoretical encounter problem is

$$
 V_P(I_k)=\min_{\Pi,R,w,\xi} J_P,\qquad\text{subject to (U5)}.  \tag{U6}
$$

Choose any attained minimizer $P_k^*=(\Pi_k^*,R_k^*,w_k^*,\xi_k^*)$
and issue its first causal command $\pi_{0|k}^*$. No performance objective,
CLF slack, uniqueness assumption or second optimization is required by this
control law or its safety theorem. The result below holds for every attained
primary minimizer. Newly acquired targets require fresh initial feasibility.

The numerical implementation may refine performance over the same feasible
set with $J_P\le V_P(I_k)$. In exact arithmetic, any feasible refined plan
then also attains (U6), so its first command remains covered by the same
theorem. This observation permits an implementation choice; it does not add
that choice to the theoretical algorithm or require refinement to succeed.
Ordinary RTI instead caps the attained *local affine* slack. That numerical
cap does not establish exact primary optimality or nonlinear feasibility.
Outside an encounter, a soft cruise CLF supplies the separate cruise design.

Assume each encountered feasible problem attains its primary minimum.
A compact nonempty feasible sublevel, closed feasibility and
lower-semicontinuous cost suffice; merely allowing arbitrary history-dependent
policies does not establish these properties. The admissible policy class
must permit restriction to surviving absolute histories and terminal
concatenation. Retain all observer, reference and prefix memory needed to
evaluate the old tail. If coordinates are recentered, transport the policies
and witnesses by an invertible relabeling rather than resetting their meaning.

The optimal primary value $V_P(I)$ eliminates all policy, set and plane
variables. It is therefore independent of any particular plane orientation
or initialization path. Changing a plane witness while solving (U6) does not
redefine this predictive barrier function.

### Theorem U: recursive feasibility and zero-deficit invariance

Assume initial feasibility, true-state containment, sound joint images,
conditioning (U2), unchanged encounter/model contracts, existence of the
abstract terminal pair (U4), and a policy/set/certificate representation
closed under shifting.
The controller issues the first command of any attained minimizer of (U6).
There is no assumption on a secondary objective or secondary solver. Then

$$
 V_P(I_{k+1})\le V_P(I_k)-\xi_{0|k}^*\le V_P(I_k).              \tag{U7}
$$

The next mathematical problem is feasible. If $V_P(I_{k_0})=0$, all subsequent
values and checked-instant collision deficits stay zero for this encounter.
Consequently vehicle interiors do not overlap at those instants for any
allowed joint realization. This does not assert a positive physical margin.

**Proof.** Whole-block conditioning (U2) gives
$S_0=I_{k+1}\subseteq R_{1|k}^*$. Restrict the old remaining policies to
the surviving histories; this retains one command on every indistinguishable
history fibre. Exact images are inclusion-monotone for a fixed policy, hence
induction gives
\[
 S_j\subseteq R_{j+1|k}^*,\qquad 0\le j<N.
\]
For outer-set representations, the stored old sets may instead be retained
where they give the required certified inclusions. By (U3), the old hard
constraints, plane witnesses and slacks remain valid. Downward terminal
closure gives $S_{N-1}\in\mathfrak B$. Append its admissible causal terminal
rule and a successor $S_N\in\mathfrak B$ with zero-deficit witnesses.

The shifted sequence $(\xi_1^*,\ldots,\xi_N^*,0)$ is feasible and has sum
$V_P(I_k)-\xi_{0|k}^*$. Comparing the next attained minimum with this candidate
gives (U7). The true current tuple is retained in the conditioned root, so
zero slack separates its footprint interiors at the checked instants.
Nonnegativity and induction preserve zero. For $N=1$, the terminal
restriction applies directly to $S_0\subseteq R_{1|k}^*$. QED.

The sum, not each individual predicted slack, is nonincreasing. First-stage
slack can redistribute while the sum falls. Telescoping (U7) makes the
executed first-stage deficits summable on an infinite applicable sequence;
it does not by itself prove finite-time recovery or Huang's stronger invariant
set convergence conclusions without their additional compactness/regularity
hypotheses. Positive slack is an admitted collision deficit, not safety.
Hard input/domain/terminal inconsistency can still make initialization
infeasible even when collision slack is unrestricted.

### Feasible improvement can replace global optimization

The shift argument also gives a useful stronger algorithmic statement that
does not demand a globally solved nonlinear program on every sample. Store
an actually feasible plan of cost $J_k$ and its shift. Accept any **nonlinearly
certified feasible** replacement with

$$
 J_{k+1}\le J_k-\xi_{0|k}.                                     \tag{U8}
$$

The retained shift always meets this bound, so feasibility and zero-deficit
invariance follow even if an improvement search fails or is stopped early.
This proves a statement about the stored plan cost, not equality to the
optimal value $V_P$. Under exact optimization (U8) is redundant. It is a
sufficient suboptimal implementation contract, not a property bestowed on
ordinary RTI by a small QP residual. This online RTI implementation deliberately
does not add a nonlinear acceptance filter or retained backup execution and
therefore does not claim (U8).

<a id="compact-certificate"></a>
### Compact nonlinear certificates: replace history rollout by error comparisons

The exact augmented construction (U1) remains the semantic model. The
following reduction changes its **outer representation**, not the physical
vehicle, the observer, the causal information available to control, or the
meaning of collision slack. It does not introduce a linear tire model, a
friction polygon, a fixed-width tube, or a new terminal-controller design.

There are two different errors to retain:

1. Estimation error: an observer output minus the physical quantity on the
   **same** realization. Future sensing can reduce this error.
2. Physical prediction spread: variation of physical trajectories across
   different admissible realizations. Future sensing does not retrospectively
   reveal which target trajectory occurs, but future ego control can respond
   differently to the resulting estimates.

Replacing target prediction spread by a small future estimation radius is
invalid. Instead the physical relative tube and the estimation-error
comparison must be coupled through the causal policy.

**Relative physical coordinates.** Use
\[
 \rho=R(\psi_E)^T(p_T-p_E),\qquad \alpha=\psi_T-\psi_E,
 \quad v_T^E=R(\psi_E)^T\dot p_T.
\]
Common rigid translation and rotation cancel from this relative pose. For a
nominal comparison branch, (J8) implies exactly
\[
 \Delta\dot\rho=\Delta v_T^E-\Delta v_E
       -\omega_EJ\Delta\rho-\Delta\omega_EJ\bar\rho,
 \quad
 D^+\|\Delta\rho\|\le\|\Delta v_T^E-\Delta v_E\|
                      +|\Delta\omega_E|\|\bar\rho\|.         \tag{C1}
\]
The term \(\Delta\rho^TJ\Delta\rho\) vanishes. Adding
\(|\omega_E|\|\Delta\rho\|\) would create artificial norm growth from
a pure rotation. Likewise, combine the shared terms of
\(\Delta v_T^E-\Delta v_E\) before taking support bounds. Absolute yaw
and position errors still matter to world-referenced policies; their removal
from collision geometry does not remove them from those policies.

Let \(z\) contain sufficient physical coordinates to evaluate the relative
flow, admissibility, and chosen policy outputs. Relative pose alone is not a
closed vehicle state: body velocities, yaw rates and target motion variables
are also required. Keep persistent tire parameters and target encounter
parameters in \(\lambda\), with \(\lambda^+=\lambda\). Their domain may
shrink with new information but is not independently regenerated at future
steps. Constant target acceleration/sideslip is a prediction assumption for
the encounter, not an extra restriction on the standalone NRMM observer.

**A small observer interface.** Equations (S1)--(S8) in
[the observer derivation](../estimator/OBSERVER_ISS_THEORY.md#compact-sampled-comparison)
give explicit sampled bounds for the position/IMU filter, body/target core,
vector yaw innovation, and published ego position. At synchronized detected
samples a candidate interface is
\[
 e=[B_f,B_v,W_T,B_\psi,B_p]^T\ge0.                            \tag{C2}
\]
These are bounds on errors, not simulated observer estimates. Reset predictor
memory is eliminated only at reset instants, using its exact intersample
equations. Retain additional coordinates whenever an output, asynchronous
sensor phase, stop event or policy memory is not covered by (C2). In
particular an NRMM velocity/acceleration error bound is not by itself a target
orientation bound at zero target speed.

The two-state homogeneous sampled core contracts for a sufficiently short
sensor period under the explicit conditions (S4). The position filter
contracts in its Lyapunov metric under bounded acceleration discrepancy.
Neither fact establishes uniform contraction of their yaw-coupled
interconnection. A valid yaw bound may be the whole circle at standstill.

**Close the physical map with future feedback.** Restrict implementation of
the theoretical policy class, if desired, to finite-dimensional causal
rules
\[
 u_i=\kappa_i(\hat c_i,m_i;a_i),\qquad \hat c_i=c(z_i)+\epsilon_i,
                                                                  \tag{C3}
\]
where \(m_i\) is sufficient observable memory, \(a_i\) is the stored policy
parameter, and the error output \(\epsilon_i\) is enclosed by (C2).
This identity is an analysis device: the controller evaluates \(\hat c_i\),
not the unknown \(z_i\) or \(\epsilon_i\). At the current decision the
actual observer outputs and **all memory used by the controller** are fixed,
so (C3) has one common first command. Fixing observer memory alone would not
suffice for a policy depending on an unretained earlier reading. Allowing
independent physical and error coordinates must not
override that root equality. Later decisions permit different commands on
distinguishable histories. Control remains held across all intermediate
sensor updates in the decision period, as in (J4).

For the physical exact held-input transition \(F_h\), the controlled map is
\(F_h(z,\kappa_i(c(z)+\epsilon,m;a_i);\lambda)\). On a smooth chart its
derivative with respect to physical state includes the **combined** term
\[
 D_zF_h+D_uF_h\,D_{\hat c}\kappa_i\,D_zc,                    \tag{C4}
\]
and the relevant memory derivatives when memory varies. Bounding
\(D_zF_h\) and the feedback term separately before combining them can erase
stabilizing cancellation. Equation (C4) differentiates the exact held-input
map; it is not continuous feedback recomputed inside a control hold.
Whole-domain derivative bounds, or finite-increment bounds across nonsmooth
pieces, give the nonlinear comparison. A nominal Jacobian without its
remainder does not give that comparison.

**Constructive comparison coefficients.** Choose physical residual blocks
\(w_a\), metrics \(P_a>0\), and radii \(s_a\) with
\(\|w_a\|_{P_a}\le s_a\). Use the sampled observer functionals (C2) for
the remaining blocks. On a convex chart containing all comparison segments,
a valid cross-block coefficient for a differentiable output map \(\Psi_a\)
is
\[
 L_{ab}=\sup_{\mathcal D}
 \|P_a^{1/2}D_b\Psi_a P_b^{-1/2}\|_2.                        \tag{C5}
\]
The map \(\Psi\) includes (C3) before differentiation. Integrating its
derivative along a segment proves the finite-increment bound. Exact sampled
identities such as (S1), (S5), (S8), and common rotations such as (C1) should
be simplified before applying (C5). For observer blocks, signed-error
identities or comparison bounds are used rather than differentiating a norm
at zero. If no common smooth chart exists, partition the domain with valid
transition handling or use a directly verified finite-increment modulus.
Unbounded derivatives at a singular branch do not define finite coefficients.

Together with bounded forcing and persistent-parameter contributions, this
produces a monotone map of the form
\[
 \begin{bmatrix}s^+\\e^+\end{bmatrix}
 \le\mathcal Q_i(s,e,\Lambda;a_i),\qquad
 \mathcal Q_i(q,\Lambda;a_i)=M_iq+d_i(\Lambda),\quad M_i\ge0,  \tag{C6}
\]
when an affine majorant is available; a nonlinear order-preserving majorant
is also admissible. All observer forcings, held-sensor aging, inverse-map
uncertainty, target extension discrepancy and stop resets belong in this
map. Compose sensor-period maps with the input held, then the decision reset;
do not apply a new control policy after every sensor sample. The coefficient
domain must enclose the resulting trajectories and every allowed memory.
Domain containment is part of the certificate, not an outcome assumed after
an otherwise invalid bound has been calculated.

Equation (C5) is a specification for constructing certified constants, not
an inexpensive online instruction to solve new global supremum problems.
The intended reduction uses analytically bounded or offline-certified domain
tiles, precomputed sampled kernels, and online matrix/vector evaluations.
The five observer radii do not provide these whole-domain physical and
force-inverse constants automatically. Their useful tightness for the full
Fiala interconnection remains to be established before deployment.

**Retain shared uncertainty where it matters.** A more informative physical
representation is
\[
 z_i=\bar z_i+G_i\eta+w_i,\qquad \eta\in\Lambda,\qquad
 \|w_{a,i}\|_{P_{a,i}}\le s_{a,i}.                            \tag{C7}
\]
Selected columns carry common pose errors, persistent parameters or shared
sensor primitives. A column shared by two outputs is combined before taking
its directional support. Nonlinear products and unretained histories are
enclosed in residual blocks; no affine exactness is assumed. Fresh sensor
noise is introduced once per actual sensor event, and all its uses share
that primitive. A fixed generator budget may absorb old nonpersistent
columns into a sound residual bound. It must not erase the identity of
persistent physical parameters. The state dimension of the comparison can
stay fixed without preserving every statistical or geometric dependence.

A product of separately bounded plant and observer consequences of one
parameter is a conservative enlargement. It is not necessary to add a
second, independently invented mismatch disturbance on top of those same
consequences. Retaining their shared columns can reduce that enlargement.
Noise need not be independent, zero-mean, or averaged down; bounded
deterministic sensor errors are sufficient for these inequalities.

**What bounded width actually requires.** Suppose the full relevant
comparison, after removing constant parameters into the forcing/domain,
has a common weight \(v>0\) and constants \(0\le\gamma<1\), \(d<\infty\)
such that, uniformly over the admitted policies and domains,
\[
 M_i v\le\gamma v,\qquad \|d_i\|_v\le d,
 \quad\|q\|_v=\max_j |q_j|/v_j.
\]
Then induction gives
\[
 \|q_i\|_v\le\gamma^i\|q_0\|_v+
                    d(1-\gamma^i)/(1-\gamma).               \tag{C8}
\]
A radius may increase toward this noise floor without diverging. More
generally, a uniform product bound
\(\|M_{i-1}\cdots M_j\|\le C\gamma^{i-j}\) gives the analogous
geometric-series bound and permits expanding individual steps. If neither
condition is available, the finite-horizon recurrence can still be sound,
but bounded long-run width has not been proved. Position-only sensors at
rest do not justify a positive yaw-contraction rate; saturated steering does
not justify arbitrary relative-motion contraction. Constant uncertain target
parameters cannot contract under \(\lambda^+=\lambda\) in a positive
definite full-state metric. Thus (C8) is a checkable additional condition,
not a universal consequence of having an observer. It is not needed for the
recursive-feasibility argument that follows.

**Finite certificate semantics and collision constraints.** Write
\(\mathcal C=(\bar z,G,\Lambda,q,\mathcal D,\mathcal M)\), where
\(\mathcal M\) constrains observable memory. Define \(\Gamma(\mathcal C)\)
as the set of full joint states consistent with these data and their
anchored error functionals. There must be one physical/observer realization
behind each member, with shared parameters; alternatively a deliberate
outer product enlargement is allowed if all subsequent claims cover that
larger set. At the current root \(\mathcal M\) fixes actual memory. At
future stages it covers every possible memory required by the policy and
the sampled error contract.

Retained primitive values are part of this latent augmented state: all
functionals of a member use the same \(\eta\) in (C7). Selecting a separate
generator value for each inequality would discard the asserted joint
semantics. Primitive retirement is an explicitly sound set enlargement.

For fixed anchors, metrics, generators, domain and memory semantics,
smaller radii and nested parameter/memory domains imply set inclusion.
This is the order property used below. The essential admission test is
\[
 \mathcal T(\Gamma(\mathcal C_i),\pi_i)
      \subseteq\Gamma(\mathcal C_{i+1}),                      \tag{C9}
\]
uniformly over all admissible noise blocks. It is established by the sampled
comparisons, nonlinear residual enclosure, and domain checks. Merely
calculating (C6) at one reference does not establish (C9). Require
\(q_{i+1}\ge\mathcal Q_i(q_i,\Lambda_i;a_i)\), not equality: a previously
valid larger successor can then be retained after the root is tightened.
When monotonicity of a newly computed bound is unavailable, the stored
uniform certificate for the old set remains valid on its subsets.

Collision witnesses can be checked directly in the ego frame. Let \(V_E,V_T\)
be the body-fixed occupied rectangles including reference-point offsets and
\(h_V(n)=\sup_{v\in V}n^Tv\). A unit vector \(n\) from ego toward target
satisfies the sufficient separation condition
\[
 g=n^T\rho-h_{V_E}(n)-h_{V_T}(-R(\alpha)^Tn)\ge0.           \tag{C10}
\]
Use \(\inf_{\Gamma(\mathcal C_i)}g\ge-\xi_i\) for this witness.
This fixes the normal in the ego frame. If the optimization requires one
common **world-frame** normal \(n_W\), as in the corresponding common-plane
formulation of (U5), evaluate (C10) with
\(n=R(\psi_E)^Tn_W\) for each member before taking the infimum. A common
ego-frame normal is not silently interchangeable with a common world-frame
normal under uncertain ego yaw. Both give downward-closed sufficient
certificates, but they are different restricted witness families. The
chosen family must be retained under shifting and in terminal appending.
For example, position residual \(s_\rho\) and circular relative-yaw
radius \(B_\alpha\le\pi\) give, for a fixed ego-frame normal, the valid
geometric reserve
\[
 s_\rho+2r_T\sin(B_\alpha/2),\qquad
 r_T=\max_{v\in V_T}\|v\|,
\]
with position measured in the Euclidean norm and shared-generator support
added or jointly bounded. This is a conservative example, not a required
loss of all position/yaw dependence. The exact support over shared
primitives can be smaller. No positive physical stand-off is introduced.
At zero deficit (C10) excludes interior overlap at the declared nodes and
permits contact. Other hard constraints and the terminal family must use
the same certificate semantics, including any common-witness requirement.

**Nested compression after measurements.** Let \(\mathcal I^+\) be the
posterior obtained by conditioning the old first successor on the entire
received sensor block, as in (U2). It contains the true joint state and is a
subset of \(\Gamma(\mathcal C_{1|k})\). Keeping the old first-successor
anchor, metrics, generators, and domain, choose any bounds
\[
 \phi_j(S;\bar z_1,G_1)\le\beta_j(B)
       \quad\hbox{for every }S\in\mathcal I^+,
 \qquad q_j^+=\min\{q_{j,1|k},\beta_j(B)\}.                 \tag{C11}
\]
Fix the actual new memory and intersect \(\Lambda\) only with sound
parameter restrictions. These operations give
\[
 \mathcal I^+\subseteq\Gamma(\mathcal C_0^+)
              \subseteq\Gamma(\mathcal C_{1|k}).             \tag{C12}
\]
No explicit posterior set is needed if each restriction is proved sound.
Using the old radii and old parameter domain, with actual memory fixed, is
always a valid fallback under the transition contract. Ignoring some
measurements in the compressed update is conservative, but should not be
called exact conditioning. Error bounds \(\beta_j\) must refer to the
**same anchored functionals**: a radius about the new estimate cannot be
substituted for a radius about the old prediction without conversion.
An empty update signals a violated or inconsistent information/model
contract; it cannot be made sound by silently deleting the old certificate.

A smaller recentered ball does not ensure (C12). For example, the cap
\(\{x:\|x\|\le1,\ x_1\ge1/2\}\) has an enclosing ball centered at
\((1/2,0)\) of radius \(\sqrt3/2\); this ball extends outside the old
unit ball. For Euclidean balls, a sufficient replacement condition is
\(\|c_{\rm new}-c_{\rm old}\|+r_{\rm new}\le r_{\rm old}\).
General metrics, generators and nonlinear error functionals require their
own inclusion map. Recentring must also transport the policy, rather than
change what the retained feedback command means.

**Theorem C: compact-certificate recursive feasibility.** Fix the same
decision schedule and horizon \(N\) as in (U5). Replace its reachable sets
by \(\Gamma(\mathcal C_i)\), use (C9), and permit the inherited anchors,
metrics, generators, policies, domains and witnesses as feasible decisions.
Assume:

- The initial certificate contains the true state and has a feasible plan.
- All propagated certificates satisfy the uniform nonlinear contract (C9),
  hard constraints, and the stated uncertainty/detection contract.
- Actual updates satisfy (C12), with a common first command and admissible
  restriction of the old causal policy tail.
- The assumed terminal family has a zero-slack append **representable in
  this certificate and policy class**. Abstract pointwise terminal safety
  alone does not establish this representation property.
- Every optimization used for the optimal-value statement attains its
  primary minimum. For example a nonempty compact feasible sublevel in a
  finite-dimensional closed policy/certificate parametrization and lower
  semicontinuous objective suffice.

Let \(V_C(\mathcal C_0)\) be the minimum nonnegative slack sum over these
certified plans. Executing its first command gives
\[
 0\le V_C(\mathcal C_0^+)\le
       V_C(\mathcal C_0)-\xi_0^*\le V_C(\mathcal C_0).        \tag{C13}
\]
In particular, zero certified slack stays zero at every subsequent active
decision in this encounter.

**Proof.** By (C12), the new root is a subset of the old first successor.
The old second successor remains a valid image bound under the old next
policy, because (C9) was valid for every old first-successor member. Repeat
this restriction through the stored tail. Downward hard/collision conditions
retain the old witnesses and slacks. Apply the assumed representable terminal
append with zero slack. The resulting candidate has slack vector
\((\xi_1^*,\ldots,\xi_N^*,0)\), hence cost
\(V_C(\mathcal C_0)-\xi_0^*\). The next attained minimum cannot exceed it.
True-state containment and nonnegative slacks establish the stated safety
consequence. This proof does not require individual radius contraction or a
constant-width tube. QED.

For the same underlying information,
\(V_P(\mathcal I)\le V_C(\mathcal C)\) if the compact root encloses
\(\mathcal I\) and every compact plan is admissible in (U5), including
its policy and geometric-witness family. No such ordering is asserted
merely from using a different coordinate-dependent witness family.
Equation (C13) concerns the compact certificate value. It does not establish
the corresponding decrease of the exact optimal value along controls that
only minimize the compact problem. With a retained feasible plan and a
certified improvement instead of an attained optimum, (U8) applies to the
stored cost rather than to either optimal value.

**Finite representation and computational meaning.** A finite-memory Markov
policy can use the observer states, prior input, clock and a bounded set of
policy parameters as sufficient observable memory. The physical model and
future-noise contract must be Markov in the augmented state, with persistent
parameters retained. Then the inherited candidate stores \(N\) stage
certificates and policies, not an ever-growing measurement tree. Arbitrary
history-dependent policies or noise laws do not automatically admit this
finite representation. Terminal concatenation must stay in the selected
class. Keeping a literal chain of old histories is not a constant-memory
implementation of this construction.

Fresh planning may use a different reference; it cannot delete the inherited
candidate solely because the new local parameterization excludes it. One
retained candidate with its own valid certificate is sufficient for (U8).
A fixed-size outer representation can discard information conservatively;
it cannot represent every nonlinear posterior exactly. No bounded number of
generators is asserted to preserve all useful correlations without inflation.

A bounded total description also requires fixed templates or budgets for
\(\Lambda\), domains, functionals and controller-memory updates. Intersecting
new arbitrary halfspaces forever, or hiding a growing history expression
inside one functional, defeats that claim. Componentwise parameter bounds
can be intersected in a fixed box template; keeping the previous domain is
the sound fallback when no representable tighter restriction is available.
Generator reduction or domain simplification that expands the current root
must satisfy (C12), or separately recertify the retained tail on the expanded
root. The number of radii alone does not settle these issues.

With certified coefficients available, the online work consists of one
nominal physical rollout, a fixed-size comparison per sensor interval,
selected shared-generator/support propagation and a nested root update.
For fixed comparison dimension \(d\), fixed generator budget and fixed
sensor/control ratio, the radius propagation costs \(O(Nd^2)\). This is
neither a complexity claim for the optimization nor a measured speedup.
The evaluator below supplies constructive physical/observer remainders
instead of unknown whole-domain Jacobian constants. Tightness is tested at
admission. The compatible terminal family remains the terminal assumption
of Theorem C.

<a id="compact-constructive-enclosure"></a>
### Constructive nonlinear transition certificates

**Finite force increments, including full braking.** Put
\(\sigma(b)=\sqrt{1-b^2}\), \(s=C\tan\alpha\), \(Q=\mu F_z\sigma\), and
\[
 T(s,Q)=-\operatorname{sgn}(s)Q
 \{1-[1-\min(|s|/(3Q),1)]^3\},\qquad T(s,0)=0 .
\]
For \(Q>0\), with \(z=\min(|s|/(3Q),1)\),
\[
 T_s=-(1-z)^2,\quad T_Q=-\operatorname{sgn}(s)z^2(3-2z),
 \quad |T(s,Q)-T(\bar s,\bar Q)|\le|\Delta s|+|\Delta Q|.      \tag{D1}
\]
The increment bound follows by integrating the two partial derivatives,
both bounded by one, then taking the continuous extension at \(Q=0\).
For a braking interval, extrema of \(b^2\) occur at its endpoints and,
when included, zero. Taking square roots of \(1-b^2\) gives its exact
availability range. Use that finite range at saturation, rather than an
infinite braking derivative. Share one \(\sigma\) between axles and retain
the same persistent \(C_f,C_r,\mu_f,\mu_r\). The identity \(|T|\le Q\)
tightens the formula's image; it introduces no extra friction constraint.

If \(Q\ge Q_{\min}>0\), a sharper optional centered remainder is
\[
 |R_T|\le {|\Delta s|^2\over3Q_{\min}}
 +{|\Delta s\,\Delta Q|\over2Q_{\min}}
 +{4|\Delta Q|^2\over9Q_{\min}}.                            \tag{D2}
\]
This uses bounds \(2/(3Q_{\min}),1/(2Q_{\min}),8/(9Q_{\min})\) on
the second partials; the first derivative is Lipschitz across the joins.
A sign-fixed, fully saturated force is exactly linear in \(Q\).

**Explicit state derivatives and exact physical flow.** Input and
parameters are held, so a time-integration remainder needs only \(D_xf\).
On a slab with \(v_x\ge v_*>0\), \(|v_y|\le Y\), \(|r|\le R\),
\(|v_x|\le V\), and \(|\alpha_j|\le\alpha_{j,*}<\pi/2\), define
\[
 D_j=C_{j,\max}\sec^2\alpha_{j,*}
 [(Y+l_jR)/v_*^2,\ 1/v_*,\ l_j/v_*].
\]
These bound tire state derivatives directly from (D1) and the slip formula.
With \(s_\delta,c_\delta\) bounding absolute steering sine and cosine,
the body-state Jacobian has the componentwise majorant
\[
 L_b=\begin{bmatrix}
 s_\delta D_f/m+[L_{\rm load}/m,R,Y]\\
 (c_\delta D_f+D_r)/m+[R,0,V]\\
 (l_fc_\delta D_f+l_rD_r)/I_z
 \end{bmatrix}.                                             \tag{D3}
\]
For the actual load
\(\ell(v)=\rho C_d A v|v|/2+
 mg(c_0+c_1|v|+c_4|v|^4)\tanh(v/v_0)\), use
\[
 L_{\rm load}=|\rho C_d A|V+
 mg[|c_1|+4|c_4|V^3+(|c_0|+|c_1|V+|c_4|V^4)/v_0].
\]
Pose rows are bounded by \(L_{1:2,3}=V+Y\),
\(L_{1:2,4:5}=1,L_{3,6}=1\). The low-speed formula is differentiated
directly. Across its cubic blend add
\(1.5|f_{\rm Fiala}-f_{\rm kin}|e_{v_x}^T/(v_2-v_1)\) to the
componentwise maximum of the two branch bounds. Evaluate the dynamic branch
only on its actual positive-speed domain. These are finite arithmetic
expressions implemented by the stateSlope action of nrmmJointEnclosure.

For an initial set \(X\), one held causal command \(U\), and persistent
parameters \(\Theta\), certify a slab box \(Z\) satisfying
\[
 X+[0,d]f(Z,U,\Theta)\subseteq Z .
\]
Let \(F\ge|f(Z,U,\Theta)|\) and \(L\ge|D_xf|\) be the bounds above. Then
\[
 F_d(X,U,\Theta)\subseteq X+d f(X,U,\Theta)
                   +[-r_d,r_d],\quad r_d=\tfrac12d^2LF.       \tag{D4}
\]
The Picard inclusion proves containment throughout the slab. The remainder
is \(\int_0^d(d-t)D_xf\,f\,dt\), valid almost everywhere through smooth
blend/saturation joins. Thus (D4) covers the exact ODE, not just an RK step.
At full braking the held state derivative is finite. Expand or subdivide
the certification slab until inclusion succeeds; reject an unresolved
domain. Subdivision creates no new control decision or sensor measurement.
The exactPlant action implements this rule.

Evaluate the signed affine expression \(X+d f(X,U,\Theta)\) before taking
supports. In particular compute \(U=\kappa(\hat c_i)\) once from the
joint decision-boundary set and retain it throughout the hold. Shared
state/feedback columns then cancel before norm bounds, as required by (C3).
The residual bound is not a replacement open-loop propagation of those
columns.

**Small observer kernels with a complete nonlinear remainder.** During
one held sensor block use
\(y=(\hat v,\hat\rho,\hat q,\hat s,\lambda)\), where
\(\lambda=y_{\rm predictor}-\hat\rho\). Its equations are
\[
 \dot y=\mathcal A y+\mathcal B[a_m+k_v\tilde v;\Phi_e(\hat q,\hat s)],
 \quad\mathcal A=A_s\otimes I_2-I_5\otimes\omega_mJ,
\quad A_s=\begin{bmatrix}
 -k_v&0&0&0&0\\-1&0&1&0&a_1\\0&0&0&1&a_2\\
 0&0&0&0&a_3\\0&0&0&0&-a_1
 \end{bmatrix}.                                             \tag{D5}
\]
\(\mathcal B\) injects its vector inputs into blocks one and four.
On detection reset \(\lambda=y_{\rm radar}-\hat\rho\), one shared
expression. Without detection retain it and set all innovation gains to
zero. The only nonlinear forcing in (D5) is the actual saturated NRMM map.

Freeze that forcing at the slab's initial state for the center map only.
Let the already proved global NRMM Lipschitz coefficients be \(L_q,L_s\),
\(\Omega\ge|\omega_m|\), and \(\Lambda\ge\|\lambda(0)\|\).
Displacements from the initial target estimate satisfy the positive system
\[
 \dot d\le
 \begin{bmatrix}\Omega&1\\L_q&L_s+\Omega\end{bmatrix}d+
 [\|\dot{\hat q}(0)\|;\|\dot{\hat s}(0)\|]
 +t[|a_2|;|a_3|](a_1+\Omega)\Lambda,\quad d(0)=0.             \tag{D6}
\]
Initial derivatives are whole-set evaluations of (D5), not sampled maxima.
The last term follows from the exact decaying innovation.
With \(j(t)=[L_q,L_s]d(t)\), the endpoint forcing remainder in
\((\hat\rho,\hat q,\hat s)\), in vector norms, is bounded by
\[
 \left[\int_0^d{(d-t)^2\over2}j(t)\,dt;\
       \int_0^d(d-t)j(t)\,dt;\
       \int_0^d j(t)\,dt\right].                            \tag{D7}
\]
One seven-dimensional matrix exponential evaluates (D6)--(D7), including
constant and time coordinates. Rotations have norm one, so no additional
exponential factor is needed. This is targetObserverResidual.

The exactObserver action also covers uncertainty in held gyro. Append the
constant forcing to the state in (D5), apply the exponential series to
shared affine variables, and bound its tail after order \(p\) by
\[
 (I-d\widehat A/(p+2))^{-1}
 {(d\widehat A)^{p+1}\over(p+1)!}|y_{\rm augmented}(0)|,       \tag{D8}
\]
where \(\widehat A\ge|\mathcal A_{\rm augmented}|\) and
\(\rho(d\widehat A/(p+2))<1\). Later Taylor factors are bounded by that
nonnegative ratio matrix. Subdivide if the test fails.
The published-position map is exactly
\[
 \hat p(d)=p_m+d\hat v_I+e^{-k_pd}(\hat p(0)-p_m).
\]
For \(\dot{\hat\psi}=\omega_m+
k_\psi(Q_2\cos\hat\psi-Q_1\sin\hat\psi)\), with
\(\|Q\|\le Q_*\), its centered Euler remainder is bounded by
\[
 \tfrac12 d^2 k_\psi Q_*(|\omega_m|+k_\psi Q_*).              \tag{D9}
\]
This also covers standstill. A true correspondence has norm at most one;
its affine outer set may require a larger \(Q_*\). No yaw contraction is
assumed. Certification subdivisions inherit the original innovation and
held measurements.

**The forcing coefficients are computed from the same physical branch.**
At actual sensor events evaluate
\(a_B=(\dot v_x-rv_y,\dot v_y+rv_x)^T\) using the same state, command,
and persistent tire variables. Add each position, IMU, gyro, and radar
noise once. Apply the actual position/IMU filter reset and construct the
nominal force-balance pseudo measurement through its finite, clamped
inverse. Interval bisection retains both possibilities for an unresolved
predicate. There is no new independent tire-mismatch input in the observer.
The inverse and the product-normalized yaw correspondence already have
whole-domain arithmetic in nrmmJointEnclosure.

The boundary IMU uses the previous command, whereas (D4) uses the newly
issued command. Thus the acceleration jump
\(a_B(x,u^+,\theta)-a_B(x,u^-,\theta)\) is included explicitly, with
the same \(\theta\). Future target poses use the analytic constant-curvature
flow, persistent encounter parameters, and forward stopping. This remains
a controller prediction convention, not a new standalone-observer premise.

**Transition-containment proposition.** Suppose the root set contains
the true physical states, observer memories and persistent parameters;
the sensor-noise bounds hold; the held causal input is admissible; and the
whole-domain/slab tests above succeed. Compose sensor resets, the
position-filter map, (D4)--(D9), and the analytic target flow. At an
ambiguous perception guard propagate both detection outcomes and join
their observer images. The result encloses every exact continuous
plant/observer realization at the next decision.

Proof is induction over the actual sensor events. Each reset is an outer
image with shared primitives; (D4) encloses the physical flow; (D6)--(D8)
bound all target-observer forcing changes; (D9) and the exact position
formula cover the remaining observer blocks. Inputs are held, so later
measurements do not change earlier physical motion. Persistent parameters
are copied. Every composition therefore preserves inclusion. Fixed current
observer memory makes the first command common.

The executable constructor is the compactEnclosure action of
nrmmJointPrediction, delegating to compactCoupledHold in nrmmJointEnclosure.
A set crossing the perception guard has both observer continuations
enclosed. For affine images \(A,B\), the join
\((A+B)/2+[-r,r]\), \(r\ge|(A-B)/2|\), contains both images
with their shared primitive assignments. Joining loses some correlation
but cannot discard a possible detection history.
Its vehicle is the continuous theoretical model, without the engineering
sample-end velocity reset. Its certificate supplies the transition
inclusion of Theorem C. With the inherited-domain restriction (C12) and
the assumed compatible terminal append, that theorem gives recursive
feasibility and nonincreasing deficit. Small width, universal admission,
global yaw contraction, and finite-time zero deficit do not follow.

**Online approximation.** The ordinary RTI predictor now uses the signed
core (D5) for future virtual sensor holds. A fourth-order integrating-factor
Runge--Kutta update resolves the NRMM forcing on short slabs; its analytic
derivative includes the held gyro in the exponential. Freezing that forcing
for an entire virtual hold is not used. The first successor retains
the complete runtime observer and sensor block. The configuration field
controller.compactObserverPrediction selects this specialization.
It omits (D4), (D7)--(D9) as optimization tightenings, so its nonlinear
certificate flags remain false. The separate exact evaluator includes
those terms and claims real-arithmetic enclosure only; MATLAB arithmetic
is not a directed-rounding certificate.

### The explicit route from theory to SQP-RTI

For fixed frame-level uncertainty data, transcribe the nominal nonlinear
problem with decisions $z=(X,U,\theta,b,\xi,s)$. It has vehicle equalities
$X_{i+1}=F_h(X_i,U_i)$, the plane rows (G3), physical/domain inequalities,
the shared terminal function $\Phi_f(X_N)\le0$, and the applicable CLF row.
The optional nonlinear diagnostic solves these equations using shooting.
RTI performs the following approximation of this same transcription:

1. Shift the previous states, inputs and continuous witnesses; replace the
   measured root and retain its dynamics defects. Plan only at acquisition.
2. Propagate the engineering joint affine observer/feedback tube at that
   reference. Freeze its data for this local solve; its directional supports
   still depend on the plane variable.
3. Linearize **all optimized nonlinear coordinates**, including $\theta$ and
   $b$, using (G4). Use the same terminal function and the same objective units
   as shooting. Build a positive-semidefinite Gauss--Newton performance cost.
4. Minimize local PCBF slack, then performance under the attained slack cap,
   on this single local model. Issue the first input and retain the iterate.

Step 4 adds the optional implementation performance refinement to the
theoretical control law. Both solves share one local model; they are not two
SQP updates. If refinement fails, the implementation keeps the completed
primary result. The safety theorem does not depend on this extra solve.

Thus the QP/LP solves are numerical approximations, not a redefinition of
collision safety. The approximation has two separate gaps: an affine joint
uncertainty prediction need not enclose nonlinear errors, and one finite SQP
step need not satisfy nonlinear constraints or retain the shifted robust
witness. Numerical trust regions help local accuracy but do not prove either
premise. Feature ties and Fiala mode transitions additionally fall outside
ordinary smooth local RTI convergence assumptions. A successful numerical
campaign supports performance on its tested cases, not Theorem U for RTI.

The existing `verifyUnifiedObserverSafety` specialization and tests exercise
the shift argument on an analytic longitudinal model with exact held-input
images. It is a special case of (U1)--(U8), not verification of the full
NRMM/Fiala model. The retained policy-box research mode explores sufficient
nonlinear certificates separately. Neither should be conflated with the
ordinary online implementation.

<a id="runtime"></a>
## 8. Numerical realization and certificate boundary

The ordinary online implementation performs one SQP-RTI linearization per
control sample. It shifts the stored plan and separating-plane witnesses,
anchors the new root at the current estimate, and builds the sampled model
and joint uncertainty margins. A geometric initializer is used at encounter
onset; the horizon remains fixed for that encounter. The first optimized
input is issued directly. There is no post-solve nonlinear acceptance filter.

Position-only GNSS, IMU acceleration, gyro and radar feed the same causal
observer used by the joint prediction. The ego/observer lift has 21 coordinates:
physical ego (6), numerical body velocity/position/yaw observer (5), persistent
tire factors (4), previous input (2), and position/IMU filter memory (4).
The coupled lift additionally carries the physical target, target observer
and radar predictor. Five physical sensor-error channels are used; the two
legacy velocity-sensor slots are zero. Each tire factor and each simultaneous
sensor error enters once, shared through every affected equation.

Default future affine prediction may aggregate sensor updates and reuse a
joint linearization for nearby horizon stages. The first issued-command
sensor block is evaluated freshly. Policy derivatives and actuator reserves
are updated at every stage. Reuse changes future sensitivities and means;
it is an engineering approximation without a nonlinear remainder certificate.
`jointPredictionRefreshSteps=1` disables stage reuse. Native kernels are
optional compiled evaluations of the same MATLAB equations.

The following research-mode derivations explain sufficient enclosure and
retention mechanisms. They are separate from the default affine RTI claim.

### Whole-domain nonlinear enclosure and the local problem

The implementation now separates **choosing a trust domain** from **enclosing
its nonlinear image**. `nonlinearBicycleModel.trustRegion` remains a numerical
choice of domain; its sampled discrepancy is never promoted to a certified
Hessian bound. The new `controller.jointNonlinearPrediction=true` research
mode calls `nrmmJointEnclosure` through `nrmmJointPrediction`. It requires the
actual position-only observer snapshot. There is no fallback to the affine
mode when an image cannot be enclosed. The default remains false until the
full-budget prediction and terminal/shift requirements below are usable.

The state has 21 coordinates: physical ego (6), numerical observer (5),
persistent tire factors (4), previous input (2), and position/IMU filter
memory (4). There are five actual sensor-noise channels; the two former
velocity-sensor slots are identically zero. Future measurements, filter
updates, observer integration and ancillary feedback all execute on their
actual clocks. The current measurement and first input are conditioned data.
One tire-factor vector drives both the plant and synthetic IMU. It is not
injected again as an independent observer disturbance.

**Full-domain arithmetic.** `nrmmSet` represents

$$
a=c+G\zeta+e,\qquad |\zeta|\le1,\quad |e|\le r.
$$

For a scalar smooth function with a bound on its second derivative throughout
`[c-R,c+R]`, where `R=|G|1+r`, the implemented rule is

$$
\phi(a)\in\phi(c)+\phi'(c)G\zeta+
[-q,q],\qquad q=|\phi'(c)|r+\tfrac12 H R^2.
$$

For products, the signed first-order generators are retained and the
remainder is bounded by
`|c_a| r_b + |c_b| r_a + R_a R_b`. At each sensor boundary the new residual
box is lifted into independent signed columns, an exact reparameterization
that lets subsequent observer/feedback maps preserve cancellations. These are algebraic inclusions on the
whole domain, not estimates from sampled perturbations. Exact scalar range
bounds can also be intersected; when that loses correlation, the result is
explicitly an independent interval. Small range improvements are declined
when retaining a signed generator is more useful. This changes tightness,
not validity.

The normalized Fiala force uses

$$
P(q)=\begin{cases}3q-3|q|q+q^3,&|q|\le1,\\
\operatorname{sign}(q),&|q|>1.\end{cases}
$$

Its derivative is globally 6-Lipschitz. Thus its integral Taylor remainder
is at most `3 R_q^2`, even across zero slip and the saturation transitions.
At full braking, where the capacity derivative is singular, a finite force
range replaces differentiation. This uses the existing Fiala saturation;
it introduces no additional tire-force constraint. Low-speed plant branches
and mixed stopped/moving reset images are enclosed rather than selected
using only their centers.

The observer's nominal force-balance inverse uses a whole-domain monotonicity
test. When `V > max(l_f,l_r)|r| sin(beta_max)`, the steering cosine is positive,
and both tire-slip charts are valid, each axle force is nonincreasing in
lateral velocity. Interval predicates bracket every clamped root. A uniformly
interior root with derivative bounded away from zero additionally uses an
interval implicit-function step, retaining the shared IMU/velocity symbols.
Otherwise the full clamped output range is retained. The online inverse's
`1e-5 m/s` bisection tolerance is included; it is not treated as exact algebra.

These rules establish inclusions for the **declared discrete RK4/observer
map in real arithmetic**. MATLAB elementary operations do not use directed
rounding here: `floatingPointCertified=false`. This is not a validated ODE
flow, an intersample collision certificate, or a residual bound for an
arbitrary higher-fidelity vehicle plant. `realArithmeticEnclosure=true` and
`robustNonlinearSafetyCertified=true` are deliberately different statements;
the latter remains false.

**Decision corrections and physical uncertainty.** Let `L` lift a six-state
nominal correction into physical and observer coordinates. For a stage with
multiple-shooting defect `d_i`, the affine nominal equality is

$$
\delta x_{i+1}=A_i\delta x_i+B_i\delta u_i+d_i.
$$

The enclosure computes the joint image, in the corrected nominal coordinates,

$$
\left\{\Phi_i(\chi+L\delta x_i,\bar u_i+\delta u_i,v)
-L(A_i\delta x_i+B_i\delta u_i+d_i):
\chi\in\mathcal E_i,\ (\delta x_i,\delta u_i)\in\mathcal D_i,
\ v\in\mathcal V_i\right\}. \tag{R1}
$$

Here `Phi_i` includes the fixed ancillary law centered on the corrected
nominal state. At the first stage the input lift changes only the physical
coordinates: the current numerical observer is already known and must not
receive the initial correction a second time. The output is expressed in
the same corrected coordinates as subsequent stages. Previous-input and filter memory remain in the augmented
state; they are not reset to zero. The physical input used to generate the
next IMU sample is the same one used by the plant. Thus (R1) covers the sum
of numerical correction and uncertainty. It does not compute a Hessian on
`D_i` and then apply it outside that set to observation errors.

`localEnclosureDomain` first obtains finite correction intervals using the
affine dynamics, physical input bounds and trust choices. Position intervals
are propagated from the fixed initial state; no arbitrary position radius is
introduced. The QP subsequently intersects its final bounds with this same
domain. A warm-start consistency repair cannot enlarge an already validated
domain. Out-of-domain images fail explicitly. Actual input and state margins
use the projected nonlinear images. This is a sufficient, potentially very
conservative local construction, not a claim that a failed domain contains
no physical maneuver.

**Collision geometry.** Every active node, midpoint and endpoint fixed-normal
collision block now receives the rotating-vertex remainder. For a unit normal
and body vertex `v`, with `|delta psi| <= eta`,

$$
g\ge\ell-\|v\|\min(\eta/2,2)|\delta\psi|. \tag{R2}
$$

The first bound follows from the rotation Hessian; the second follows from
the global bound `|g'(a)-g'(b)| <= 2||v||`. Consequently (R2) remains valid
even if the yaw interval is unbounded. Two linear rows implement the absolute
value. The final QP bounds, after warm-start repair, determine `eta`. Their
additional reserve vanishes at zero nominal yaw correction. Existing physical
yaw uncertainty still requires its separate chord support. No extra physical
clearance is added, and collision slack is retained on both rows.

**Terminal continuation.** `terminalSafeSet.encloseContinuation` applies the
same arithmetic to the accepted constant-deceleration, held-curvature braking
law. It covers endpoint decision corrections plus physical endpoint error,
including states that stop at different times. Its residual is the enclosed
pose minus the affine terminal-pose map. The worst-case speed determines a
schedule long enough to include every stop. This establishes a pose-map
building block only. It did not certify the former `terminalSafeSet.rows`
local approximation, and does not certify the current `nonlinearTangent`:
its closest parked-target witness and first-exit selection must still be
uniform over the entire joint information set. Therefore the new API returns
`terminalMembershipCertified=false`; it does not replace the simplified
terminal model with Fiala dynamics or silently claim a complete terminal set.

**What can be proved about an actual convex local problem.** The following
is a sufficient theorem, including the often omitted inner-approximation
condition. It is not a theorem about unmodified ordinary SQP-RTI.

Assume that at frame `k` an accepted certificate retains (i) a physical joint
information set, (ii) a causal input policy, (iii) sound stage images,
(iv) fixed absolute-time separation certificates and nonnegative stage slacks,
and (v) a common robust terminal continuation. Require:

1. All stage images enclose the declared nonlinear plant/observer map,
   including the complete admitted decision domains and primitive errors.
2. The next posterior is contained in the previous one-step image after
   conditioning on the actual measurement. Persistent physical parameters
   are intersected with their previous set, not independently restarted.
3. The terminal certificate is invariant under its common continuation and
   the encounter's first-exit convention. Appending that continuation adds
   zero slack. The encounter horizon and slack weights stay fixed.
4. The **next convex local problem contains an encoding of the shifted
   certificate**. Re-linearization, new gains, new separation normals and new
   outer approximations are admissible only if they retain this encoding.

Then the next local problem is feasible: the conditioned tail policy, its
retained sets, shifted geometry/slacks and appended terminal continuation are
an explicit feasible candidate. If the primary solve minimizes total slack,

$$
J_{k+1}^{\star}\le J_{k+1}^{\rm shift}
=J_k-\xi_{0|k}\le J_k. \tag{R3}
$$

Proof: current-set inclusion and sound images put the realized successor in
the old first predicted set. Conditioning restricts that set, so every
remaining old policy/constraint certificate remains valid without recomputing
a larger outer approximation. Terminal invariance supplies the last stage.
Condition 4 places this physically valid candidate inside the **actual**
convex feasible set; minimization then yields (R3). If `J_k=0`, all nonnegative
slacks remain zero and the sampled separation conditions continue to hold
while the encounter is active. With positive slack, (R3) alone is not a
collision-free guarantee or a proof of finite-time convergence to zero.

Condition 4 cannot be inferred from "the old nonlinear plan is feasible."
For example, a fresh uniform remainder over a large correction box can be
larger than the retained stage reserve even at zero correction. The new
sufficient inequalities then exclude a physically safe old candidate.
Changing observer centers without carrying the information set causes a
similar problem. A warm-start primal vector does not retain either certificate.

A constructive way to satisfy the zero-update part of condition 4 is to
bound an **increment relative to the retained nonlinear tube**, rather than
rebounding that tube from scratch. On a smooth compact domain containing all
`z` in the old tube and all segments `z+s d`, suppose a verified Jacobian box
satisfies

$$
|D\Phi(z+s d)-A|\le M,\qquad 0\le s\le1.
$$

Then the fundamental theorem of calculus gives

$$
|\Phi(z+d)-\Phi(z)-A d|\le M|d|. \tag{R4}
$$

An already valid enclosure of `Phi(z)` is retained. Nonnegative auxiliary
variables `t >= d`, `t >= -d` add only the affine reserve `M t`; `d=t=0`
recovers the old enclosure exactly. This yields a concrete convex route to
a retained zero-update candidate while permitting variable tube widths.
The Jacobian box must cover physical uncertainty as well as decision
corrections. At nonsmooth points a proved incremental/Lipschitz bound or
explicit branch treatment is needed; finite derivatives at a sampled center
are insufficient. In particular, a finite derivative bound in braking ratio
must not be asserted through the full-braking square-root endpoint. The
current absolute-image arithmetic is a sound building block on accepted
domains, but has not yet been replaced by this incremental tube formulation.

The current implementation establishes (R1) on successfully enclosed domains
and (R2), plus the terminal pose-map enclosure. It still lacks conditions 2--4
for the production six-state RTI, notably full terminal-family membership and
a retained-certificate encoding. The scalar information-set specialization
already demonstrates the shift argument, but does not fill these vehicle
implementation gaps. Further code must retain/restrict old sets and policies,
or construct a variable-tube local formulation with a proved zero-update
candidate. Merely tightening the current QP with larger constant reserves
cannot finish that proof.

This is the distinction made useful by Cannon et al. (2010): bounding the
linearization error and retaining an admissible tube are both required.
Lishkova and Cannon (2025), Sections IV--V, provide related convexification
constructions under their dynamics assumptions; those assumptions are not
silently asserted for this output-feedback Fiala model. Koehler et al. (2021),
Section IV, supplies the corresponding output-feedback consistency framework.

### Retaining a candidate in the next convex problem: constructive audit

The question here is specifically condition 4 of (R3), assuming a valid
conditioned tail and terminal extension have already been supplied by
conditions 1--3. It must not be answered by assuming condition 4 again.
The current point-trajectory RTI does not provide this property. A different
local representation can provide it under the explicit incremental bounds
below. That representation is derived here; it is **not installed as the
online vehicle controller**.

**An exact obstruction, even with an invariant terminal set.** Consider
the scalar system `x+ = x^2 + u`, `-0.01 <= u <= 0`, with the hard state and
terminal interval `[0,1]`. The policy `u=0` makes this interval invariant.
A valid old horizon-two certificate has exact sets

$$
[\sqrt{1/2},1]\longmapsto[1/2,1]\longmapsto[1/4,1]
$$

and nonlinear nominal centers `sqrt(3/4), 3/4, 9/16`. Every constraint has
zero slack. Suppose the next measurement is exactly `x=1/2`, which belongs
to the old one-step set. The physically valid continuation is
`1/2 -> 1/4 -> 1/16` with zero input. Following the production initialization
pattern, reset the new first nominal node to `1/2`, retain the old endpoint
`9/16` as the next node, and append its zero-input image. Linearization then
imposes, in absolute coordinates,

$$
x_1=\tfrac14+u_0,\qquad
x_2=\tfrac98 x_1-\tfrac{81}{256}+u_1
    =-\tfrac9{256}+\tfrac98u_0+u_1<0.
$$

The new affine problem is infeasible because `x_2 >= 0` is hard, although
the nonlinear continuation is safe and the exact terminal set is invariant.
Relaxing that last row instead requires slack at least `9/256`; it does not
preserve zero slack. There is no trust region, new obstacle, target forecast
change, numerical tolerance, or solver failure in this example. It proves
that the point-shift/relinearize rule alone is insufficient; it does not
assert that every vehicle encounter fails.

`informationSetRecursiveFeasibilityTest` reproduces this exact calculation.
`adaptiveTrustRegionTest.aNonlinearContinuationIsNotAnAffineShiftCertificate`
also checks the actual Fiala formulation after a small measured-root update.
With two 50 ms holds, inputs `[0.04 rad; -0.3]` and a 15 m/s reference, the
nonlinear continuation does not exactly satisfy the assembled affine
equalities. The existing affine consistency repair satisfies those
equalities, but represents a different state sequence. This second test is
an equality audit, not a claim of complete vehicle-QP infeasibility or of a
robust safe Fiala witness.

**A sufficient constructive replacement.** Let the retained certificate
consist of absolute joint sets `S_i`, causal output-feedback policies `pi_i`,
and verified inclusions

$$
\Phi_i(S_i,0,\mathcal W_i)\subseteq S_{i+1}.
$$

Here `Phi_i(chi,v,w)` is the *closed-loop augmented transition* when the
actual input is `pi_i(available observer memory) + v`. The augmented state
includes physical and observer states, persistent parameters, target state,
and relevant filter/input memory. A new `v` is an admissible feedforward
increment, not access to the unknown physical state. The old and perturbed
images use the same primitive noise and persistent parameters. The retained
sets need not have dynamically consistent point centers or constant width.

For every `z in S_i`, `|e| <= r_i`, permitted `d_i,v_i`, and primitive `w`,
suppose the entire intervening domain has verified componentwise bounds

$$
|D_\chi\Phi_i-A_i|\le M_{x,i},\qquad
|D_v\Phi_i-B_i|\le M_{u,i},\qquad M_{x,i},M_{u,i}\ge0. \tag{R5}
$$

These bounds include the feedback and observer update, not just the plant.
Define the new tube and policy by

$$
T_i=S_i\oplus\{d_i\}\oplus[-r_i,r_i],\qquad
\pi_i^+(\cdot)=\pi_i(\cdot)+v_i.
$$

Introduce nonnegative `t_i,s_i` with `t_i >= +/-d_i` and `s_i >= +/-v_i`.
Use the linear constraints

$$
\begin{aligned}
d_{i+1}&=A_i d_i+B_i v_i,\\
r_{i+1}&\ge(|A_i|+M_{x,i})r_i+M_{x,i}t_i+M_{u,i}s_i.
\end{aligned} \tag{R6}
$$

To prove their soundness, couple a perturbed point `z+d_i+e` with its base
point `z` and the same `w`. The integral difference formula gives

$$
|\Phi_i(z+d_i+e,v_i,w)-\Phi_i(z,0,w)-A_i d_i-B_i v_i|
\le (|A_i|+M_{x,i})r_i+M_{x,i}|d_i|+M_{u,i}|v_i|.
$$

The retained inclusion and (R6) therefore imply
`Phi_i(T_i,v_i,W_i) subset T_{i+1}`. No old model discrepancy or old sensor
noise is inserted a second time as an independent incremental disturbance.
The new `r_i` bounds only the change relative to the retained certificate;
it does not replace the old joint observer/feedback tube by an open-loop
uncertainty prediction.

For an affine state constraint `a' chi <= b + xi_i`, the new exact support
row is

$$
\sup_{z\in S_i}a^\top z+a^\top d_i+|a|^\top r_i\le b+\xi_i.
$$

More generally, express each input, rotating-footprint or terminal condition
as `g_j(chi,v) <= xi_j`, with `xi_j=0` for a hard condition. Retain its old
verified upper bound `beta_j >= sup_{z in S_i} g_j(z,0)` and require
`beta_j <= xi_j_old`. If complete-domain derivative differences satisfy
`|D_chi g_j-c_j'| <= m_x,j'` and `|D_v g_j-e_j'| <= m_u,j'`, impose

$$
\beta_j+c_j^\top d_i+e_j^\top v_i+
(|c_j|+m_{x,j})^\top r_i+m_{x,j}^\top t_i+m_{u,j}^\top s_i
\le\xi_j. \tag{R7}
$$

The same integral argument proves this affine row sufficient. Choosing
`c_j=e_j=0` also allows valid finite Lipschitz constants to replace derivative
differences, at the cost of losing directional improvement. Thus the
zero-update property follows from an explicit formula, rather than being
postulated as a new feasibility assumption. At nonsmooth transitions such
bounds still require justification. The old support directions, absolute
encounter times and physical parameter family are retained. A newly computed
absolute bound cannot silently replace `beta_j` if it is looser. A terminal
pose enclosure alone does not supply the required terminal-membership bound.

After conditioning, the initial posterior is a subset of `S_0`. Set
`d_0=r_0=0`; the base tube center need not be reset to the new point estimate.
The known observer value enters the first policy evaluation, so this does
not ignore the measurement or execute a stale open-loop input.

Now **substitute** `d_i=r_i=t_i=s_i=v_i=0` and the shifted old slacks into
every row: (R6) holds; all incremental reserves vanish; the current posterior
is enclosed; and every retained state, input, collision and terminal bound
is recovered. Trust domains must contain the zero increment. Thus this
specific convex formulation contains the shifted candidate. If that
candidate's slacks are zero, minimizing their nonnegative sum has optimum
zero. The argument uses one convex model and does not require repeated SQP
iterations to converge. The executable scalar increment test also admits
a nonzero control update, so the illustration is not only a fixed singleton.

This closes condition 4 **for (R5)--(R7)**, conditional on the valid shifted
base certificate and the stated complete-domain bounds. It is a sufficient
construction, not a claim that every viable alternative must use this
parameterization. It is not a completed proof for the current runtime.

**Why this is a formulation change, not a smaller trust-radius patch.**

- `collisionAvoidanceController` currently retains primal states, inputs,
  slacks and side data. It does not retain the complete joint set/policy and
  its stage/terminal implication certificates.
- `localRtiInitialize` replaces the first state by the new estimate while
  retaining later affine nodes. `localFormulate` then enforces point-dynamics
  defect equalities. Equations (R6) instead propagate increments of a retained
  set inclusion; those two meanings are different.
- The current nonlinear enclosure propagates a fresh absolute image over a
  whole decision box. It supplies no uniform incremental Jacobian bounds for
  the complete joint map, and its extra reserve need not vanish at zero.
- Full braking permits a finite Fiala force but not a finite derivative in
  braking ratio: for fixed nonzero saturated slip, changing `b=-1` to
  `-1+epsilon` changes force proportionally to `sqrt(2 epsilon-epsilon^2)`.
  No finite coefficient times `epsilon` bounds this for every small positive
  `epsilon`. Moving the seed to `-0.9999` does not certify the allowed
  endpoint. An exact input reparameterization, proved branch treatment, or
  restricted increments would be needed; the tire model need not be replaced.
- `sampleEndState` also has a discrete stopped/moving reset. A derivative
  box across its discontinuity is invalid. Zero-update retention can still
  be used there, but nonzero admitted increments require a separate valid
  branch argument. A sound absolute range alone is not (R5).

An old-input fallback, a new constraint `sum(xi)<=J_old`, or an affine
consistency repair cannot supply these missing implications. Conversely,
none of the obstacles proves recursive feasibility impossible for a
linearization-based controller. The concrete next design decision is whether
to replace ordinary point-SQP-RTI by the retained-certificate tube formulation,
while keeping Fiala, fixed encounter horizons and convex local optimization.
Conditions 2--3 and whole-model incremental bounds remain separate work.
No runtime algorithm, physical limit, terminal vehicle model or default
certificate flag was changed by this audit.

### Implemented retained policy-box formulation

The following implementation supersedes the preceding audit's "not yet
implemented" statements **for the retained backend only**. It is selected with
`retainedTube=true` for synchronized joint-observer inputs during an encounter.
The online experiment uses point RTI with `retainedTube=false`.
This is a structural change to the optimization, not a claim that the
ordinary RTI affine dynamics become robustly feasible by adding a slack cap.

The derivative-difference construction (R5)--(R7) is one sufficient route.
The implemented route instead admits a complete nonlinear image of a convex
**feedforward policy box before optimizing within it**. This avoids requiring
a finite derivative at full braking or across a stop reset. The Fiala model
and the position-only observer remain unchanged. Sound finite ranges and
branch enclosures are used at those transitions; an unenclosable domain is
rejected, never replaced by sampled derivative maxima.

**Certificate and policy.** Let the augmented state contain the physical
ego state, numerical observer state, four persistent tire parameters,
previous input, and position/IMU filter memory. For a driving stage use

$$
 u_i=\bar u_i+K_i(\widehat x_i-\bar x_i)+d_i,
 \qquad \underline d_i\le d_i\le\overline d_i,\qquad
 \underline d_i\le0\le\overline d_i. \tag{R8}
$$

Angles in this policy use the implemented wrapped difference. Current
numerical observer coordinates are known; future gyro noise, observer
updates, feedback and the held-input timing are included in the joint map.
For initial admission the first measured input correction is conditioned,
so its `K_0` is zero. Later first inputs evaluate the retained causal policy
with the new numerical observer state and measured gyro.

`nrmmJointPrediction('policyTube',...)` encloses **every** history allowed
by (R8), the shared persistent parameter family, and the declared sensor
noise. For the nonlinear joint transition `Phi_i` it establishes

$$
 \Phi_i(S_i,D_i,W_i)\subseteq S_{i+1}. \tag{R9}
$$

The stored certificate contains `S_i`, the policy parameters and boxes,
the input-set enclosures, the target's original absolute epoch and parameter
family, fixed world separation normals, sampled separation lower bounds
`g_i`, and a terminal certificate. All hard state and input domains and the
braking-input slew bound must hold for the full admitted sets. No road rows
or additional tire-capacity constraints are introduced. Define

$$
 \xi_i=\max(0,-g_i),\qquad J(C)=\sum_{i=0}^{N}\xi_i. \tag{R10}
$$

Here `C` denotes the retained certificate. `J(C)` is a conservative,
**memory-dependent certified deficit**, not the globally optimal PCBF value
over all nonlinear policies and not the old point-RTI affine slack. A missing
target error family is an error, not a declaration of perfect perception.
Relative target uncertainty is conservatively lifted into absolute pose
uncertainty for this formulation; it gives up the old common-pose cancellation.
On the zero level this distinction does not weaken physical feasibility:
the admitted causal policy satisfies every sampled collision row and the hard
terminal condition. It is therefore an explicit zero-cost feasible policy
for the corresponding robust PCBF problem with those same constraints and
policy class; that problem's nonnegative optimum must also be zero. For
positive `J`, no equivalence to Huang's optimal value or its convergence
result is claimed. Both backends now relax all collision nodes, but an affine
slack is not a certified nonlinear deficit.

**Whole terminal family.** `terminalSafeSet.certifySet` checks the original
entry set through the declared constant-deceleration, held-curvature braking
law. It uses nonlinear rotating-rectangle support enclosures at the control
sample times. It continues until every member has left the perception range,
or both vehicle families have stopped and are separated. A stationary target
with positive acceleration is not classified as permanently stationary.
An exhausted check budget or an unproved gap returns `valid=false`.
This is a certificate for the simplified terminal model the project assumes;
it does not assert that a Fiala plant or a higher-fidelity vehicle executes
that terminal motion exactly. No terminal-model substitution was made.

Once certified, the terminal law retains its **original entry set and
elapsed time**. Its suffix is safe by the already checked trajectory. It is
not restarted from a new point estimate. This gives a shift-closed terminal
mode on an augmented state containing its entry and phase, without requiring
new separating-side decisions in subsequent terminal checks. The hard entry
speed, lateral-speed, yaw-rate and sideslip-cone bounds remain satisfied under
the declared proportional speed/rate decay and absorbing stop.

**Conditioning without losing containment.** `conditionRetained` retains the
old zonotope and appends affine equations on its latent variables for the
known observer, applied input, position-filter memory and current physical
error set. Thus the exact constrained set is a subset of the old prediction.
The position frontend must be treated carefully: the retained future value
is a prior, while the current runtime publishes a measurement posterior.
With position-update gain `L`, the equality is

$$
 (I-LH)\,v_{\rm prior}=v_{\rm posterior}-L p_{\rm GNSS}. \tag{R11}
$$

Equating prior and posterior would reject legitimate observations. A linear
feasibility check detects contradictory declarations; it is not a proof that
the observer's current bound contains reality. A coordinate outer projection
of this constrained intersection is computed using LP dual residual bounds.
Current raw GNSS, gyro and nonlinear IMU consistency then tighten that proposal. This outer set is used only for a **new** full admission
check; it is never asserted to lie in the old signed zonotope. If its new image
is too wide, the original certificate is retained. The previous fallback that
restored a stale physical projection, discarding the current gyro restriction,
has been removed.

**Convex problem and its explicit feasible point.** For an admitted policy
box, the local problem has input corrections `d`, affine performance states
`z`, and nonnegative deficits `s`:

$$
 \begin{aligned}
  &d\in D,\\
  &E_z z=b+E_d d,\\
  &s_i=\xi_i.
 \end{aligned} \tag{R12}
$$

`E_z` is block lower triangular with identity diagonal blocks. Point-dynamics
linearizations determine a quadratic tracking/regularization objective and
the surrogate equality; **they are not hard safety witnesses**. Its safety
meaning comes from (R9) and the full-box constraint/terminal checks. If a
performance tangent fails in a model branch, a constant/identity surrogate
can replace it without changing the safety certificate.

The primary optimum is analytically `sum(xi)`, so one performance QP suffices
inside the admitted box. This formulation does not pretend to optimize safety
over every nearby nonlinear policy. New candidate certificates can reduce
the deficit, but only after full admission and a comparison with the retained
budget. Performance cannot trade an increase of certified deficit for progress.

In particular, `d=0`, `s=xi`, and `z=E_z^{-1}b` is an explicit feasible point
of **this actual convex problem**. The implementation constructs and checks
it before calling the performance solver. An unfinished performance solve
retains this point. Small solver bound violations are projected back into
the admitted box and the surrogate states recomputed. This does not depend
on a post-solve point rollout to rescue safety.

After selecting `d*`, reparameterize `bar u <- bar u+d*` and
`D <- D-d*`. This preserves exactly the admitted family, and ensures the
next zero correction denotes the selected policy. Optional box enlargement
is validated before optimization; failure never replaces a valid certificate.

**Recursive-feasibility proposition.** Assume an initial certificate was
admitted, the true augmented state and target belong to its families, the
declared sampled transition/terminal models and noise bounds hold, current
observer bounds contain the true state, and the encounter continues with the
same horizon and consistent measurement/model contracts. At the next frame:

1. The implemented policy belongs to the admitted box, so (R9) puts the
   true successor inside the saved next joint set.
2. Conditioning restricts that set. The remaining saved policies and terminal
   suffix therefore remain applicable for every consistent successor.
3. Shift the stored deficits and append a zero terminal deficit. Keep the
   original geometry, absolute target time, parameter family and terminal law.
4. In (R12), substitute the shifted certificate, `d=0`, the shifted deficits,
   and the uniquely determined affine surrogate states. Every row is
   satisfied. A proposed new certificate is accepted only if its deficit sum
   does not exceed that shifted budget.

Consequently,

$$
 J_{k+1}\le J_k-\xi_{0|k}\le J_k,
 \qquad J_k=0\ \Longrightarrow\ J_{k+1}=0. \tag{R13}
$$

The fixed horizon may contain a shorter driving prefix and a longer suffix
of the saved terminal law; it is not enlarged or shortened across frames.
If no driving prefix remains, the mathematical candidate is the certified
terminal continuation, still at zero deficit. In the experiment entry point
this explicitly raises `terminalControlRequired` **before actuation**, as
requested. It is a driving-task failure, not loss of mathematical feasibility.
Membership in a terminal-safe set alone never triggers this exception.

Induction proves sampled rectangle nonoverlap from a zero certificate for
the remaining encounter under these assumptions, including the terminal
continuation if it is needed. This proof does not assume that a fresh local
QP happens to admit the shifted candidate: (R12) supplies it constructively.
It does assume sound model/observer containment and terminal admission, as
any robust safety result must. No claim is made about intersample collision,
future encounters after perception exit, arbitrary unmodeled mismatch,
positive-deficit convergence to zero, or ordinary point-RTI zero slack.

**Validation and practical boundary.** `retainedTubeControllerTest` exercises
nonzero ego/target error budgets, shared tire mismatch and sensor noise,
actual position-only observer updates, public-entry certificate persistence,
nontrivial admitted control boxes, explicit feasible candidates, contradictory
posterior/noise rejection, and the retained terminal suffix. These tests
validate implementation behavior; sampling is not the enclosure proof.
The tests include 20 consecutive driving holds with zero certified deficit
and physical containment under persistent front/rear cornering factors
1.01/0.99, friction factors 1.02/0.98, and bounded position, IMU and gyro
noise. These fixtures do not establish general campaign feasibility.
The stationary target 25 m ahead is a certificate regression fixture, not a
braking-insufficient bypass benchmark. Passing it does not establish successful
avoidance campaigns or real-time suitability. Arithmetic is ordinary
double precision: `floatingPointCertified` and the unconditional
`robustNonlinearSafetyCertified` flag remain false.

### Runtime scope

The native adapter requires a completed convex solve. Preprocessing, local
model construction and both objectives count toward the configured budget;
desktop MATLAB execution has no hard wall-clock guarantee. A timeout or
infeasibility is a failed controller call. Warm-up and experimental budgets
must be reported separately from the model sample period.

The issued RTI prediction is the affine SQP iterate. Its slacks and residuals
belong to that local problem. No uncomputed nonlinear residual is reported
as zero. `robustNonlinearSafetyCertified=false`: affine zero slack does not
establish true nonlinear separation, posterior inclusion, a compatible
Fiala terminal controller, or whole-hold safety. Current error containment,
future nonlinear enclosure and retained-feasible-plan shifting are distinct
proof obligations. Conditional Theorem U states exactly which are required.

Independent closed-loop replay measures actual clearance and sustained path
and speed recovery. Reaching a predicted terminal set does not complete a
driving experiment. Historical measurements remain in `report/` and Git
history; no simulation or performance result is added by this refactor.

<a id="source-map"></a>
## 9. Source map and verification

The controller has six MATLAB files. Configuration and the precomputed CLF
table remain in `config/`. Tests enforce the source budget and one Markdown
theory document in each algorithm directory.

| Source | Mathematical responsibility |
| --- | --- |
| [collisionAvoidanceController.m](collisionAvoidanceController.m) | Encounter memory, orchestration and first-command output. |
| [readControllerInputs.m](readControllerInputs.m) | State, timestamp, uncertainty and road-input contracts. |
| [solvePredictiveControl.m](solvePredictiveControl.m) | PCBF/CLF transcription, RTI, shooting diagnostics and plan shifting. |
| [nonlinearBicycleModel.m](nonlinearBicycleModel.m) | Vehicle and Fiala forces/tangents, held-input flow, trims and CLF. The tire interface uses `tireForces`, `tireParameters`, `tireAffineModel`, `tireParameterJacobian`, `tireLongitudinalForce` and `tireAccelerationGain`. |
| [predictiveSafetyGeometry.m](predictiveSafetyGeometry.m) | Target flow, uncertainty supports, rectangle witnesses, guidance and straight/circular lane charts (`project`, `referencePose`, `projectReferenceCurve`, `validateReferenceCurve`, `isStraight`). |
| [terminalSafeSet.m](terminalSafeSet.m) | Joint terminal continuation, membership and append-certificate helpers. |
| [collisionAvoidanceControllerConfig.m](../config/collisionAvoidanceControllerConfig.m) | Defaults, merging and validation. |
| [clfMatrices.json](../config/clfMatrices.json) | Offline CLF matrices, gains and certificates. |

Run `matlab -batch "results=runtests('tests'); assertSuccess(results)"`
from the repository root. Vehicle/tire, path-chart, collision-witness,
terminal, switched-objective, observer-coupling and retained-set tests cover
their respective behaviors. Finite tests do not discharge the uniform
nonlinear assumptions of Theorem U. The observer's derivation is
[OBSERVER_ISS_THEORY.md](../estimator/OBSERVER_ISS_THEORY.md).

<a id="sources"></a>
## 10. Sources

The derivations above specialize the repository's declared equations. Related
primary sources establish the general PCBF, output-feedback MPC and
high-order barrier constructions; they do not certify this NRMM/Fiala
implementation:

- Zhang, Liniger and Borrelli (2018), *Optimization-Based Collision Avoidance*: [primary manuscript](https://arxiv.org/html/1711.03449v3), Section 4 and Theorem 2, for exact signed-distance dual geometry and unit-normal normalization with negative permitted clearance. Equations (G1)--(G4) use its rectangle vertex specialization.
- Diehl, Bock and Schloeder (2005), *A Real-Time Iteration Scheme for Nonlinear Optimization in Optimal Feedback Control*: [original article](https://doi.org/10.1137/S0363012902400713), and [author-hosted overview](https://cdn.syscop.de/publications/Diehl2005c.pdf), for one Newton-type update per sample and its local regularity/initialization requirements. These results do not certify this nonsmooth engineering transcription.
- Cannon, Buerger, Kouvaritakis and Rakovic (2010), *Robust tubes in nonlinear model predictive control*: [primary manuscript](https://users.ox.ac.uk/~engs0169/pdf/cannon_nolcos10.pdf), Sections 2--4, for nonlinear remainder tubes and a retained feasible candidate.
- Lishkova and Cannon (2025), *A successive convexification approach for robust receding horizon control*: [primary manuscript](https://arxiv.org/html/2302.07744v3), Sections IV--V, for convexification-specific feasibility arguments.
- Scott, Raimondo, Marseglia and Braatz (2016), *Constrained zonotopes: A new tool for set-based estimation and fault detection*: [primary manuscript](https://web.mit.edu/braatzgroup/Scott_Automatica_2016.pdf), Definition 3 and Sections 3--5, for exact linear intersections and the cost of discarding generator constraints.
- Rego, Raffo, Terra and Scott (2025), *Set-based state estimation of nonlinear discrete-time systems using constrained zonotopes and polyhedral relaxations*: [preprint](https://arxiv.org/html/2504.00130v1), Section 3 and Proposition 3, for shared nonlinear graphs, polyhedral outer relaxations and measurement conditioning. The method does not establish timing or validity for this vehicle implementation.
- Kochdumper and Althoff (2021), *Sparse Polynomial Zonotopes: A Novel Set Representation for Reachability Analysis*: [primary manuscript](https://arxiv.org/pdf/1901.01780), for preserving dependent polynomial factors through nonlinear reachability operations.
- Huang, Wang, Margellos and Goulart (2025), *Predictive Control Barrier
  Functions: Bridging model predictive control and control barrier functions*:
  [primary manuscript](https://arxiv.org/html/2502.08400v1), Section III and
  Lemma III.5, for the predictive slack and shifted-plan argument.
- Koehler, Mueller and Allgoewer (2021), *Robust output feedback model
  predictive control using online estimation bounds*:
  [primary manuscript](https://arxiv.org/html/2105.03427v1), Sections III--IV,
  for separate estimation/tracking bounds and robust output-feedback MPC.
- Tezuka and Nakamura (2020), *Time-varying Obstacle Avoidance by Using
  High-gain Observer and Input-to-State Constraint Safe Control Barrier Function*:
  [publication](https://doi.org/10.1016/j.ifacol.2021.04.192), Sections 4--6,
  for treating estimated obstacle velocity error inside the barrier dynamics.
- Quan, Zhou, Frisk and Chung (2025), *Observer-Based Environment Robust
  Control Barrier Functions for Safety-Critical Control With Dynamic Obstacles*:
  [publication](https://doi.org/10.1109/LCSYS.2025.3581497), Section III, for
  distinct environment-state and velocity-error compensation. Their general
  bounded-error interface motivates the containment premise of Theorem U; an energy-gain LMI alone is not
  adopted as a pointwise Euclidean radius.
- Xiao and Belta (2019), *Control Barrier Functions for Systems with High
  Relative Degree*: [primary manuscript](https://arxiv.org/abs/1903.04706),
  for high-order barrier constructions related to virtual wall guidance.
