# Output-feedback collision avoidance of two rectangles during a positive-speed encounter

## 1. Problem and physical dynamics

A vehicle follows a prescribed path at reference speed $V_{\rm ref}>0$. During an encounter it observes the other vehicle's relative position continuously within sensing radius $R_d$. The objective is one causal steering and longitudinal command that preserves physical separation and allows nominal cruise whenever compatible with safety. The two footprints are congruent closed rectangles. Their dynamics reference points may be offset from their geometric centers. The analysis concerns forward rolling ego motion and identifies target motion on positive-speed arcs.

Write $e(\theta)=(\cos\theta,\sin\theta)^\top$, $J(x,y)=(-y,x)$, $R(\theta)=[e(\theta),Je(\theta)]$ and $[x]_+=\max(x,0)$. Norms are Euclidean. Time is continuous.

### Ego motion

The input is $u=(\delta,b)$ with $|b|\le1$. The state is $x=(p,\psi,v_x,v_y,r)$, with inertial reference position $p$, body heading $\psi$, body velocity $v=(v_x,v_y)^\top$ and yaw rate $r$. Thus $\dot p=R(\psi)v$ and $\dot\psi=r$. Let $m,I_z$ be mass and yaw inertia, $g$ gravity, $l_f,l_r$ the axle distances, and $L=l_f+l_r$. For front and rear axles $i=f,r$, define

$$
(F_{zf},F_{zr})=mg(l_r,l_f)/L,\qquad
K_i=\mu_i F_{zi},\quad F_{xi}=bK_i,\quad D_i=K_i\sqrt{1-b^2}.
$$

The friction coefficients $\mu_i$ and cornering stiffnesses $C_i$ stay attached to their physical hypothesis. On the rolling branch the slip angles are $\alpha_f=\operatorname{atan2}(v_y+l_fr,v_x)-\delta$ and $\alpha_r=\operatorname{atan2}(v_y-l_rr,v_x)$. With $z_i=C_i|\tan\alpha_i|/(3D_i)$ when $D_i>0$, the lateral tire law is

$$
F_{yi}=\begin{cases}
0,&D_i=0,\\
-C_i\tan\alpha_i(1-z_i+z_i^2/3),&D_i>0,\ z_i<1,\\
-D_i\operatorname{sign}(\alpha_i),&D_i>0,\ z_i\ge1 .
\end{cases}
$$

Write $(X_f,Y_f)^\top=R(\delta)(F_{xf},F_{yf})^\top$. With air density $\rho_a$, drag coefficient $C_d$, frontal area $S_f$, rolling coefficients $c_0,c_1,c_4$ and smoothing speed $v_\epsilon>0$, define
$F_d(v_x)=\rho_a C_d S_f v_x|v_x|/2+mg(c_0+c_1|v_x|+c_4|v_x|^4)\tanh(v_x/v_\epsilon)$.
Force and moment balance give the rolling derivative of $(v_x,v_y,r)$,

$$
f_d=\left((X_f+F_{xr}-F_d)/m+rv_y,\quad
(Y_f+F_{yr})/m-rv_x,\quad(l_fY_f-l_rF_{yr})/I_z\right)^\top .
$$

Use $\dot v_x,\dot v_y,\dot r$ from $f_d$ on the rolling domain where $v_x>0$ and $v_x\cos\delta+(v_y+l_fr)\sin\delta>0$. The operating domain stays away from its singular boundaries. Denote the full field by $f^\vartheta$ for persistent parameters $\vartheta$. Admissible commands respect the physical steering and force ranges and any declared actuator dynamics. A physical actuator with memory is included in the state. Assume unique flows along admissible inputs.

### The target's orbit and its clock

Let $x_C=(p_T,V,\gamma,A,\kappa)$ be the six-dimensional target state. The course $\gamma$ is the direction of motion. With known rear reference distance $l_T>0$ and constant sideslip $\beta\in(-\pi/2,\pi/2)$, curvature satisfies $\kappa=\sin\beta/l_T$ and the rectangle has body yaw $\psi_T=\gamma-\beta$. During forward motion the state evolves under one autonomous law,
$$
 \dot p_T=V e(\gamma),\qquad \dot V=A,\qquad
 \dot\gamma=\kappa V,\qquad \dot A=0,\qquad\dot\kappa=0 .
$$
Since traveled distance satisfies $ds=V\,dt$, the course turns through $\kappa\,ds$. Identify planar vectors with complex numbers and rotations with multiplication by a unit complex number. At a common encounter epoch $a$, put $b_T=p_T(a)$, $d=e^{i(\psi_T(a)+\beta)}$, $V_a=V(a)$. At elapsed time $\tau$ the distance and the unit-speed orbit are
$$
 s(\tau)=V_a\tau+\tfrac12 A\tau^2,\qquad
 F_\kappa(s)=\int_0^s e^{i\kappa h}\,dh
       =s\,e^{i\kappa s/2}\operatorname{sinc}(\kappa s/2).
$$
Here $\operatorname{sinc}(0)=1$. For a material point $q$ in the target body,
$$
 \boxed{\quad p_{T,q}(a+\tau)
 =b_T+d\big[F_\kappa(s(\tau))+e^{i(\kappa s(\tau)-\beta)}q\big].\quad} \tag{1}
$$
Taking $q=0$ gives the reference position. Thus the orbit is a circle or a straight line and its clock is quadratic. On the declared absorbing stop for $A<0$, holding $s$ at $V_a^2/(-2A)$ after $\tau=-V_a/A$ preserves the entire footprint. This continuation needs no inversion of stationary data. Target identification below concerns encounters with a moving observation interval.

In ego coordinates put $\rho=R(\psi)^\top(p_T-p)$ and $Q=R(\psi)^\top R(\psi_T)$. Differentiation gives
$$
 \dot\rho=VQe(\beta)-v-rJ\rho,\qquad \dot Q=(\kappa V-r)JQ .
$$
A common rigid displacement of both vehicles leaves $(\rho,Q)$ and every clearance unchanged. The prescribed path and ego position measurements still use the world frame.

**Joint information and separate physical flows.** Fix a member $z$ of the joint information set, the prediction origin, and inertial axes. The physical flow factors as $(\Phi_E^u(x_{E,z},\vartheta_z),\Phi_T(x_{C,z}))$. For a variation in the future ego command,
$$
 \delta_u x_{C,z}=0,\qquad
 \delta_u\rho_z=-R_E^\top\delta_u p_E-J\rho_z\,\delta_u\psi_E .
$$
The second identity follows by differentiating the relative position. Thus the world-frame target orbit can be evaluated once per member and physical time. The same member still determines the ego trajectory and both footprints in a clearance query. Factoring the flow does not factor the joint information set into independent vehicle sets.

### Equivalence to the nonlinear relative motion model (NRMM)

The target law in the observer and in every forecast is the constant-tangential-acceleration, constant-sideslip kinematic single track used by the NRMM. Equations (1) and (2) express this law in arc and Cartesian coordinates, respectively. [Sharma, Alai and Rajamani](https://doi.org/10.1016/j.trc.2025.105411)

Put $q=R(\psi)^\top\dot p_T$, $s_B=R(\psi)^\top\ddot p_T$, $d_B=q/V$, $\Omega=\kappa V$, and write $\nabla_r z=\dot z+rJz$ for differentiation of a world vector expressed in rotating ego axes. Then $s_B=A d_B+V\Omega Jd_B$ and $\nabla_r d_B=\Omega Jd_B$. Since $\dot\Omega=\kappa A$, differentiation gives the following Cartesian NRMM.
$$
 \nabla_r\rho=q-v,\qquad \nabla_r q=s_B,\qquad
 \nabla_r s_B=-\Omega^2q+3A\Omega Jd_B. \tag{2}
$$
Conversely, for $V=\|q\|>0$ recover $A=d_B^\top s_B$, $N=(Jd_B)^\top s_B$, $\Omega=N/V$ and $\kappa=N/V^2$. Along (2), $\dot V=A$, $\dot A=\Omega N-\Omega^2 V=0$ and $\dot N=-A\Omega+3A\Omega=2A\Omega$, hence $\dot\kappa=0$. The physical body yaw is $\psi_T=\psi+\arg q-\arcsin(l_T\kappa)$ modulo $2\pi$. Substitution recovers (1), the same relative-position output and the same four body corners for every admissible ego input. These maps give a bijection of moving trajectories on $|l_T\kappa|<1$. At an absorbing stop (1) carries the last orientation. The inverse map at $q=0$ is outside the identification domain. The estimation guarantee in Section 2 follows from containment under the measurement update.

## 2. One joint observer, from positions to occupied footprints

The joint observer estimates a family of target motions from the positions observed so far. Its current state estimate and its forecast are evaluations of this family at different times. Equation (3) specifies how observations constrain the family, and (7) retains their joint linear consequences in a polytope.

At the epoch $a$, let $z=(x(a),\vartheta,\eta,b_T,d,V_a,A,\kappa)$ belong to a declared compact joint prior $\Theta_a^-$, with $|d|=1$, $|l_T\kappa|<1$ and $\beta=\arcsin(l_T\kappa)$. The constant vector $\eta$ contains declared sensor biases. Any actuator initial state is included in $x(a)$. Denote the ego flow under the **actually applied** input by $x_z$. Relative position, ego position, yaw rate and body-frame inertial acceleration obey
$$
 \begin{aligned}
 y_\rho&=R(\psi_z)^\top(p_{T,z}-p_z)+\eta_\rho+n_\rho,&
 y_p&=p_z+\eta_p+n_p,\\
 y_r&=r_z+\eta_r+n_r,&
 y_a&=\dot v_z+r_zJv_z+\eta_a+n_a .
 \end{aligned}
$$
The joint residual history $n$ must satisfy its declared error contract. It can be exactly zero. If a contract has memory, carry its latent state or the compatible histories as part of $z$ so that every future extension respects the same contract. Measurements use the physical input that generated them. The relative position is available throughout the in-range encounter.

Let $\Theta_t$ consist of the members of $\Theta_a^-$ for which the physical and observation equations hold through $t$ with the received outputs and an admissible residual history. For fixed epoch coordinates and a memoryless residual bound,
$$
 \Theta_{t+h}=\Theta_t\cap
 \{z:\text{the additional outputs on }(t,t+h]\text{ agree with }z\}.
 \tag{3}
$$
For a history contract the intersection is on histories with a common prefix, followed by projection if desired. The image of the surviving histories under the joint flow is the current state estimate. The true member survives if the prior and model contain it. Every predicted ego pose, target corner and future clearance for a member uses that same member. The applied control determines the ego motion in the next observation constraint. Recursive set-membership estimation provides the general conditioning principle. [Bertsekas and Rhodes](https://doi.org/10.1109/TAC.1971.1099674)

**One flow for estimation and prediction.** Let $\Phi_h$ denote the target flow above for elapsed time $h$, including the stipulated stop. For a joint member $z$, write $x_{C,z}(t)=\Phi_{t-a}(x_{C,z}(a))$. Use fixed-epoch labels throughout, indexing full admissible histories when errors have memory. Conditional on the observations through $t$, the observer defines the family of continuations and its time sections by
$$
 \mathcal P_t=\{\,(z,\,h\mapsto\Phi_h(x_{C,z}(t))):z\in\Theta_t\,\},\qquad
 \mathcal C_{t+h\mid t}=\{\,\omega(h):(z,\omega)\in\mathcal P_t\,\},
 \qquad h\ge0 .
$$
At $h=0$ this is the current target state set. At $h>0$ it gives the future states consistent with precisely the same observations. Each curve carries its original joint member $z$ into the rectangle constraint (9). In particular $A$ and $\kappa$ stay constant along each curve, even though later observations can narrow their estimated ranges.

For a continuing encounter and $\delta>0$, (3) and the flow identity $\Phi_h\circ\Phi_\delta=\Phi_{h+\delta}$ give
$$
 \mathcal P_{t+\delta}\ \subseteq\
 \{\,(z,\,h\mapsto\omega(\delta+h)):(z,\omega)\in\mathcal P_t\,\}.
$$
Indeed, each member consistent through $t+\delta$ was already consistent through $t$ and follows the same target dynamics. A forecast at $t$ uses just the observations available at $t$. When the next observation arrives, it restricts the continuations themselves. For finite enclosures the inherited certificate in Section 6 preserves this reasoning on the retained plan.

**Eliminating the target pose.** Conditional on one ego trajectory, let $Y(t)=p_z(t)+R(\psi_z(t))(y_\rho(t)-\eta_\rho)$. For an isotropic radar residual bound $\epsilon(t)$ the radar constraint is precisely
$$
 b_T\in\bigcap_{a\le v\le t}
 \mathbb B\!\left(Y(v)-dF_\kappa(s(v-a)),\,\epsilon(v)\right).
 \tag{4}
$$
Other joint error constraints continue to apply. For exact observations write $F_j=F_\kappa(s(t_j-a))$, $Y_j=Y(t_j)$. Any two times with $F_1\ne F_0$ determine $d=(Y_1-Y_0)/(F_1-F_0)$ and $b_T=Y_0-dF_0$ for that candidate clock and curvature. Compatibility requires $|d|=1$ and $Y(v)-Y_0=d(F(v)-F_0)$ for the whole observed arc. Substitution in (1) gives the future corner directly:
$$
 p_{T,q}(w)=Y_0+(Y_1-Y_0)
 \frac{F(w)-F_0+e^{i(\kappa s(w-a)-\beta)}q}{F_1-F_0},
 \qquad F(w)=F_\kappa(s(w-a)). \tag{5}
$$
With bounded errors the same identities apply to the latent exact positions allowed by (3). Choosing the same latent positions for every corner preserves a single rigid body. Coincident endpoints, including a complete revolution, furnish no pose through this particular chord. Equation (3) remains defined.

**Identifiability.** Conditional on exact ego motion, an exact inertial target trace $Y$ on a smooth moving interval determines its intrinsic state:
$$
 V=\|\dot Y\|,\qquad A=\frac{\dot Y^\top\ddot Y}{V},\qquad
 \kappa=\frac{\det(\dot Y,\ddot Y)}{V^3},\qquad
 \psi_T=\arg\dot Y-\arcsin(l_T\kappa).
 \tag{6}
$$
Indeed $\dot Y=Vd_t$ and $\ddot Y=A d_t+\kappa V^2 Jd_t$ for unit course $d_t$. A single position carries no such identification. Equation (6) characterizes distinguishability in ideal data. Bounded errors enter (3) directly. Projection onto a future time can enlarge position uncertainty even though the fixed-epoch information sets are nested. An unknown ego pose stays in the same inference problem.

A useful symmetry limits the role of active sensing. With exactly known ego pose, continuous exact relative position determines the inertial target trace by a rigid change of coordinates. Under isotropic position errors the same transformation preserves the error disks. While the target stays in view, an ego maneuver alone cannot improve the ideal identifiability in (6) at the same observation times. Actual coupling remains through visibility, uncertain ego motion and the relative geometry on which safety depends.

**Geometric constraints on the same observer.** Conditional on the ego pose history, let $Y_i$ be three exact inertial target positions at increasing times on a forward moving arc. Write $D_{ij}=Y_j-Y_i$. For distinct positions with total turn less than one revolution, the signed circumcircle curvature is
$$
 \kappa=\frac{2\det(D_{01},D_{02})}{\|D_{01}\|\,\|D_{12}\|\,\|D_{02}\|}.
$$
The circumradius equals the product of the three side lengths divided by four times the triangle area. Forward order gives the sign. Three distinct collinear positions on this branch give zero curvature. For an arc between two positions with $|\kappa|\Delta s\le\pi$, its length as a function of chord length $l$ is $L_\kappa(l)=2\arcsin(|\kappa|l/2)/|\kappa|$, with $L_0(l)=l$. This follows by taking the modulus in (1).

For observed positions $\tilde Y_i,\tilde Y_j$ in error disks of radii $\epsilon_i,\epsilon_j$, the triangle inequality gives a chord interval centered on $\|\tilde Y_j-\tilde Y_i\|$ with half-width $\epsilon_i+\epsilon_j$. Intersect it with $[0,2/|\kappa|]$ (with infinite upper endpoint at zero curvature). An empty interval excludes this conditional motion. Otherwise denote its endpoints by $l^-,l^+$. On the stated minor-arc branch before a stop the clock satisfies the linear strip
$$
 L_\kappa(l^-)\le (t_j-t_i)V_a+
 \tfrac12\big[(t_j-a)^2-(t_i-a)^2\big]A\le L_\kappa(l^+).
$$
At fixed curvature the intersection of these strips is convex in $(V_a,A)$. For exact observations the two consecutive interval-average speeds $\Delta s_{ij}/(t_j-t_i)$ give the speeds at their time midpoints, so their difference divided by the midpoint time difference gives $A$. With bounded errors the strips are necessary consequences of (3). The original position constraints retain the shared measurement errors, turn sign and pose information. Use a strip only when its branch is established for the member in question. Feasible safety can be certified before curvature or clock parameters are uniquely identified.

The strip can also contract a chart with uncertain curvature without enumerating curvature values. Let $[\ell^-,\ell^+]$ enclose the chord length over that whole chart, accounting for ego-pose uncertainty. If the minor-arc and moving conditions hold throughout, then $\ell^-\le\Delta s\le(\pi/2)\ell^+$. Indeed $\Delta s/l=x/\sin x$ for $x=|\kappa|\Delta s/2\in[0,\pi/2]$. A sharper upper bound is $L_{k^+}(\ell^+)$ when $|\kappa|\le k^+$ and $k^+\ell^+\le2$, since $L_\kappa(l)$ increases in $|\kappa|$ and $l$ on this branch. These interval endpoints constrain the same affine combination of $V_a,A$ and can be applied by (7). Each selected chord contributes one two-sided linear constraint in the common chart.

### A joint polytope for measurement correction

Fix encounter coordinates $z=z_*+S\lambda$ in a chart with an unwrapped course angle. The columns of $S$ can affect both vehicles. Let $B_t=\prod_k I_{k,t}$ be a bounded coordinate box. At each query the observer supplies the controller with the polytope $P_t$ defined by its finite family of retained strips below. Its rows are necessary consequences of the available observations, so its image in physical coordinates encloses the exact consistent family (3). Any persistent measurement errors modeled in this finite chart must have a finite-dimensional representation.

For a scalar observation $y$, let $\bar y+a^\top\lambda$ approximate its noise-free value under the applied input. If the measurement error is bounded by $\epsilon$ and the expansion residual by $r_y$ for every compatible member, then $a^\top\lambda\in J_y=[y-\bar y-\epsilon-r_y,y-\bar y+\epsilon+r_y]$. The raw observation and the orbit-clock inequalities above contribute strips in the **same** $\lambda$. A strip derived under a conditional curvature or ego pose is valid only on the family used in its derivation. With $\mathcal S$ denoting the new valid strips, the correction is
$$
 \begin{aligned}
 P_t &= \{\lambda\in B_t:H_t\lambda\le b_t\},\\
 P_{t}^{+} &= P_t\cap\bigcap_{(a,J)\in\mathcal S}
                            \{\lambda:a^\top\lambda\in J\}.
 \end{aligned} \tag{7}
$$
Here $t^+$ denotes incorporation of the additional information in the fixed encounter chart. For $J=[l,u]$, append the rows $(a^\top,u)$ and $(-a^\top,-l)$, including the case $l=u$. The coordinate bounds may also be tightened by intersecting $I_j$ with $(J-\sum_{k\ne j}a_k I_k)/a_j$ for $a_j\ne0$. This follows by isolating $a_j\lambda_j$ in the strip and leaves the corrected polytope intact. A nonredundant strip remains part of (7) even when all coordinate bounds are unchanged. Every new strip contains the true member under the declared error contract. Induction therefore gives containment and nesting. An empty intersection signals an inconsistent information contract for this update.

For example, take a straight target with $V_a=(10+\lambda_1)\,\mathrm{m/s}$, $A=\lambda_2\,\mathrm{m/s^2}$ and $|\lambda_1|,|\lambda_2|\le1$. One exact observation of $20\,\mathrm m$ forward travel after $2\,\mathrm s$ gives $\lambda_1+\lambda_2=0$. Both coordinate ranges remain $[-1,1]$. At $3\,\mathrm s$ the displacement error from the same epoch is $(3\lambda_1+4.5\lambda_2)\,\mathrm m$. Its maximum is $1.5\,\mathrm m$ on the retained equality and $7.5\,\mathrm m$ on the box alone.

Valid strips can be derived without replaying the encounter. Under the actual input propagate the reference ego flow and its parameter sensitivities, and use (1) for the target. On a smooth chart, let $L,M$ bound the first and second derivatives of the augmented physical field, including the persistent parameters. Comparison equations $\dot U=LU$ and $\dot W_2=LW_2+MU^2$, initialized with chart derivative bounds, enclose the first and second flow derivatives. The chain rule then supplies an output Hessian bound $M_y$. For an expansion at zero, $r_y=\tfrac12 M_y\sup_{\lambda\in B_t}\|\lambda\|^2$ suffices if the bound covers the line segments from zero. Across nonsmooth branches retain only inequalities with valid residual enclosures. The chart dimension is fixed. Storage for $q$ dense retained inequalities is $O(qd)$ for $d$ latent coordinates. A row implied by the remaining rows and the box can be removed without changing $P_t$.

## 3. Exact dual clearance on the joint information set

Write $B_i=o_i+B_0$ with $B_0=[-a_L,a_L]\times[-a_W,a_W]$ for $i=E,T$. In ego axes the center displacement is $c=\rho+Qo_T-o_E$. The set of center displacements giving contact or overlap is $Z(Q)=B_0\oplus QB_0$, since $-B_0=B_0$. It is the sum of four segments along the two body frames, generically an octagon. Its support in direction $n$ is $h_0(n)+h_0(Q^\top n)$, where $h_0(n)=a_L|n_1|+a_W|n_2|$. Hence the signed Euclidean clearance is
$$
 g(c,Q)=\max_{\|n\|=1}
 \big\{n^\top c-h_0(n)-h_0(Q^\top n)\big\}. \tag{8}
$$
Outside $Z$, the projection of $c$ onto $Z$ gives an attaining unit normal. Inside $Z$, the minimum supporting-plane slack equals the radius of the largest disk centered at $c$ in $Z$, giving the negative distance to the boundary. Contact gives zero.

Explicitly let $P$ have rows $e_1^\top,-e_1^\top,e_2^\top,-e_2^\top$ for the Cartesian basis, and $k=(a_L,a_L,a_W,a_W)^\top$. Linear-program support duality gives the equivalent maximization of $n^\top c-k^\top(\lambda+\mu)$ over
$$
 \|n\|=1,\quad P^\top\lambda=n,\quad P^\top\mu=Q^\top n,\quad
 \lambda,\mu\ge0 .
$$
For a candidate common input $u$, let $(c_z(v;u),Q_z(v;u))$ be the joint prediction at absolute time $v$. The robust gap is
$$
 G_t(v;u)=\inf_{z\in\Theta_t} g(c_z(v;u),Q_z(v;u)). \tag{9}
$$
The quantifiers are one input, every consistent member, and a separating witness for each member and time. The separation margin and the infimum are interpreted on nonempty information sets. A single common normal would reject even two hypotheses placing a separated target on opposite sides of the ego. Taking separate support bounds for independently varied ego and target poses can likewise destroy a measured relative displacement. Equation (9) retains these correlations. Rotation of a witness changes neither the physical safety set nor the definition of the barrier.

### When two position constraints suffice to certify a future gap

Fix a future time, a common input, and a compatible ego trajectory, target course at the epoch, speed clock and curvature. Denote these fixed data and the sensor biases by $\gamma$. Eliminate admissible measurement residuals in the observation constraints, leaving the epoch translation $b_T$ free. By (1), changing $b_T$ translates every corner by the same vector. If the conditional translation constraints are closed convex sets (as in (4) with a convex conditional prior), the possible center displacements form a compact convex set $C_\gamma$ in ego axes. It is the rigid image of their intersection. $Q_\gamma$ and $Z_\gamma=Z(Q_\gamma)$ are now fixed. Whenever $C_\gamma$ and $Z_\gamma$ are disjoint, their positive robust gap is exactly
$$
 \min_{c\in C_\gamma}g(c,Q_\gamma)=\operatorname{dist}(C_\gamma,Z_\gamma)
 =\max_{\|n\|=1}\{-h_{C_\gamma}(-n)-h_{Z_\gamma}(n)\}.
$$
Indeed the unit direction from a closest point of $Z_\gamma$ to a closest point of $C_\gamma$ supports both sets, and every other projected gap is bounded by their distance. This permits a common normal **within this translation family** without loss for positive clearance. The outer infimum in (9) still ranges over compatible $\gamma$. For a nonconvex conditional family or signed penetration one can always use (9).

For a finite nonempty family of closed convex planar constraints $B_1,\ldots,B_m$ in the common displacement coordinates, assume $C=\bigcap_i B_i\ne\varnothing$. For the compact convex configuration obstacle $K=Z_\gamma$ one has the exact unsigned distance identity
$$
 \boxed{\displaystyle
 \operatorname{dist}(C,K)=
 \max_{1\le i\le j\le m}\operatorname{dist}(B_i\cap B_j,K).}
$$
Inclusion gives the upper bound on each pair distance. If the left side is positive, enlarge $K$ by a closed disk of any radius strictly below it. The enlarged obstacle and the constraints have empty intersection. By planar Helly at most three sets suffice to witness emptiness. One must be the obstacle because the constraints are mutually consistent. The remaining pair therefore certifies that radius. Taking the supremum over radii proves equality. At zero distance equality follows from nonnegativity. With continuously indexed closed constraints, compactness of the enlarged obstacle first gives a finite witness for each strictly smaller radius. The corresponding expression uses a supremum over pairs. Positive unsigned distance equals positive signed clearance. The identity makes no claim about penetration depth.

Each pair already gives a valid lower bound. For two intersecting observation disks, their support in a specified unit direction is attained at a disk's extreme point in that direction if it lies in the other disk, or at a circle intersection. These candidates follow from the two circular boundary arcs, including containment and tangency by their respective extreme or contact points. Thus a retained pair and direction $n$ give the lower gap $-h_{B_i\cap B_j}(-n)-h_K(n)$ for this conditional translation family. To use it for a family of clocks or ego poses it must hold across that entire family. An insufficient pair leaves the query undecided. Finding the best pair among $m$ constraints can involve up to $m(m+1)/2$ pairs. The active pair can change with motion and time. Convex set distance in support form is also derived by Boyd and Vandenberghe. [Convex Optimization, chapter 8](https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf)

For completeness, the planar theorem follows by taking a minimal finite family with empty intersection. If it had at least four sets, choose for each set a point common to all the others. Any four such points admit two disjoint nonempty groups with intersecting convex hulls. Take an affine dependence among the four planar points and normalize its positive and negative parts. A point in both hulls lies in every set, since at least one group omits each set's exceptional point. This contradicts emptiness. Thus three suffice. Here the theorem applies to the observation constraints and the actual rectangular collision geometry in the same two translation coordinates.

### A directional bound for the relative vertices

In prediction-origin axes let $E_i=p_E+R_E e_i$ and $T_j=p_T+R_T q_j$ be the four physical vertices of each body, including their reference-point offsets. At the same time $h$ under a common trial input $u$, form $D_{ij}=T_j-E_i$ on each joint member. The signed rectangle dual (8) is equivalently $\max_{\|n\|=1}\min_{ij}n^\top D_{ij}$. The exact robust gap (9) takes the infimum over members **outside** this maximum.

Fix a reference $\lambda_*$ for the plan and write $\delta=\lambda-\lambda_*$. In these coordinates the same observer polytope is $P=P_t-\lambda_*=\{\delta\in B:H\delta\le b\}$ with $B=B_t-\lambda_*$, $H=H_t$, $b=b_t-H_t\lambda_*$. A box centered at $m_B$ with half-widths $w$ has support $h_B(v)=m_B^\top v+\sum_k w_k|v_k|$. The shift of the row offsets and of the box is required even if the reference is outside $P_t$.

For the trial input and time, enclose every compatible member by
$$
 D_{ij}=\bar D_{ij}+M_{ij}\delta+R_{ij},\qquad
 M_{ij}=D_\lambda T_j-D_\lambda E_i,\qquad \|R_{ij}\|\le r_{ij}.
 \tag{10}
$$
The reference differences and derivatives are evaluated at $\lambda_*$. On a smooth enclosing chart Taylor's theorem admits $r_{ij}=\tfrac12\sum_{k,l}e_k e_l\sup\|\partial^2_{\lambda_k\lambda_l}D_{ij}\|$, where $e_k=\sup_{\delta\in B}|\delta_k|$ and the derivative bounds cover all line segments from the reference to compatible states. A joint remainder preserves cancellations in the difference. On a nonsmooth branch (10) requires a sound affine enclosure with bounded residual.

For unit $n$ put $c_{ij}=-M_{ij}^\top n$ and $h_P(c)=\max_{\delta\in P}c^\top\delta$ for nonempty $P$. Minimizing the linear projection in (10) over $P$ yields
$$
 \begin{aligned}
 L_{ij}(n)&=n^\top\bar D_{ij}-h_P(c_{ij})-r_{ij},\\
 \widehat G_t(t+h;u)&=\max_{\|n\|=1}\min_{1\le i,j\le4}L_{ij}(n)
                           \ \le\ G_t(t+h;u).
 \end{aligned} \tag{11}
$$
Indeed $n^\top M_{ij}\delta\ge-h_P(-M_{ij}^\top n)$ for every member. A common longitudinal displacement of the two vehicles cancels in $M_{ij}$. A measured relationship between *different* columns is retained by $h_P$, as in the clock example above. For identical reference, $M_{ij}$ and remainder bound, $P\subseteq B$ gives $h_P(c)\le h_B(c)$ in every direction, so the lower gap is at least as large as the box bound. The support is exact for the affine term on $P$. The nonlinear remainder, the outer approximation of (3), and the use of a common geometric witness within a family can still be conservative.

**Dual support certificate.** Write all inequalities of $P$, including the box, as $\widetilde H\delta\le\tilde b$. For a bounded nonempty polytope, linear programming duality gives $h_P(c)=\min_{\nu\ge0,\ \widetilde H^\top\nu=c}\tilde b^\top\nu$, also for a lower-dimensional polytope. To separate the observation rows from the box rows, group their multipliers as $\mu\ge0$ and the upper and lower box multipliers as $p^+,p^-\ge0$. For fixed $\mu$ the box multipliers must satisfy $p^+-p^-=c-H^\top\mu$. Their least contribution is $h_B(c-H^\top\mu)$, attained by taking the positive and negative parts. This gives
$$
 U(c;\mu)=b^\top\mu+h_B(c-H^\top\mu),\qquad
 h_P(c)=\min_{\mu\ge0}U(c;\mu).
$$
For every nonnegative $\mu$ and every $\delta\in P$, $c^\top\delta=\mu^\top H\delta+(c-H^\top\mu)^\top\delta\le U(c;\mu)$. Strong duality gives an optimal multiplier with $U(c;\mu^\star)=h_P(c)$ even when exact measurements constrain $P$ to a lower-dimensional set. [Linear programming duality](https://web.stanford.edu/class/ee364a/lectures/duality.pdf)

Consequently (11) certifies a required margin $m$ exactly when it admits the following witnesses.
$$
 \exists\, \|n\|=1,\ \mu_{ij}\ge0:\quad
 n^\top\bar D_{ij}\ \ge\ m+r_{ij}+U(-M_{ij}^\top n;\mu_{ij}),
 \qquad i,j=1,\ldots,4 . \tag{12}
$$
Each multiplier certifies a support bound. The normal serves all sixteen pairs, and the physical input remains common to all members. Geometrically, (11) is the signed separation of the origin from $\operatorname{conv}\bigcup_{ij}(\bar D_{ij}+M_{ij}P\oplus\mathbb B(0,r_{ij}))$. At zero uncertainty it reduces to the signed rectangle dual (8), using an exact expansion at the known member.

For a fresh query it suffices to exhibit any witnesses meeting (12). In particular $U(c;0)=h_B(c)$. Taking the least $U$ over zero and any retained nonnegative multipliers always gives a valid bound at least as tight as the box support. When the input or $n$ changes, evaluate $U$ at the **new** $c$ and use a valid trial remainder. The same multipliers remain admissible for this calculation on the same polytope. A feasible primal point alone supplies a lower bound on the support, which has the wrong direction for certifying clearance.

**Explicit witnesses from a radar chord.** Condition on a joint ego member and use complex inertial coordinates. At the present epoch let $Y$ be the observed target position, $d_0$ a reference course vector, $p=p_T-Y$ and $v=d-d_0$ the errors in position and unit course. For a past observation $Y_j$ with disk radius $\epsilon_j$, let $H_j$ be its signed nominal arc chord and $\eta_j$ bound its chord error. The observation gives the four real strip inequalities
$$
 \|p+H_j v-z_j\|_\infty\le r_j,\qquad
 z_j=Y_j-Y-H_jd_0,\qquad r_j=\epsilon_j+\eta_j .
$$
Here the infinity norm acts on real and imaginary parts. The enclosure follows by writing the true displacement as $d H_j^{\rm true}$ and using $|d|=1$. In particular, for a signed nominal arc length $s$ one may use $\eta_j=\sup|s'-s|+\tfrac12s^2\sup|\kappa'-\kappa|$, since $F_\kappa(s)=\int_0^s e^{i\kappa l}\,dl$. The same estimate applies to a future corner coefficient $G=F_\kappa(s)+e^{i(\kappa s-\beta)}q$ after adding the bounded rotation error of $q$.

The affine part of that corner's error is $p+Gv$. For $H_j\ne0$ put $w=G/H_j$, and write $C_w$ for the real matrix of complex multiplication by $w$. The identity $p+Gv=(1-w)p+w(p+H_jv)$ gives, for every unit direction $n$ and position box $B_p$,
$$
 n^\top(p+Gv)\le h_{B_p}(C_{1-w}^\top n)
       +(C_w^\top n)^\top z_j+r_j\|C_w^\top n\|_1 .
$$
These are witnesses of (12). The positive and negative parts of $C_w^\top n$ are multipliers for the two sides of the measurement rows, and their residual course coefficient vanishes. All rows refer to the same $(p,v)$. Taking the tightest bound for each signed projection, including the box witness, preserves containment. For any nominal rectangle, two orthogonal projection intervals enclose each affine corner in a box. Its farthest distance from the nominal rectangle in the rectangle's axes is the norm of the two maximum coordinate excesses. Adding the nonlinear corner remainder and maximizing over the four corners gives an enclosing rounded rectangle by convexity. Its separation from the ego footprint can use any normal in (8). For the translation alone the identity $p=(p+H_jv)-H_jv$ gives another explicit witness. If the original observation residual lies in a disk of radius $r_j$ and $|v|le r_v$, then $pinmathbb B(z_j,r_j+|H_j|r_v)$. The support of that disk in either signed direction can be intersected with the corresponding box support. Translation is common to all four corners. About an existing root $o$, the uniform support upper bound contributed by this witness is $|Y+z_j-o|+r_j+|H_j|r_v$. Its minimum with a prior ball radius bounds the translation of the entire body in every direction. This conditional certificate concerns target pose uncertainty within a joint member. Other uncertain ego variables still enter (10).

**One ego variation for all corners.** For body $b\in\{E,T\}$ write $P_b=D_\lambda p_b$, $a_b=D_\lambda\psi_b$ (a row vector) at the reference. Then
$$
 M_{ij}=P_T-P_E+(R_TJq_j)a_T-(R_EJe_i)a_E .
$$
The target terms follow from the NRMM flow (1) before varying the control. The ego terms are rows of the same sensitivity $S_E=D_\lambda x_E$ satisfying $\dot S_E=f_xS_E+f_\vartheta D_\lambda\vartheta$ at the trial input, initialized with the joint chart derivative in the fixed prediction axes. The input is held common when differentiating with respect to the latent variables. One ego flow and its variation supply every vertex pair. All transformations of an uncertain initial pose must act on the same $\lambda$. A new trial input can change the ego variation, queried support directions and residual bounds.

**Continuous-time clearance.** On any interval $[a,b]$ of duration $\Delta$, use one fixed unit normal for its two endpoint bounds and all intervening times. Suppose every material-point projection $\phi_{ij}=n^\top D_{ij}$ has absolutely continuous velocity and $|\ddot\phi_{ij}|\le M$ almost everywhere on that interval for all compatible members. Let $l_a,l_b$ bound all projected pairs from below at the endpoints using (11) with that same normal before maximizing. For $0\le s\le\Delta$ integration of the acceleration bound gives
$$
 g_z(a+s)\ \ge\ (1-s/\Delta)l_a+(s/\Delta)l_b-\tfrac M2 s(\Delta-s).
$$
Indeed, subtract from each projection its affine endpoint interpolant and add $Ms(\Delta-s)/2$. The result is concave with zero endpoint values. In particular endpoint lower bounds of $m+M\Delta^2/8$ certify margin $m$ throughout. For ego vertex $e_i$ the acceleration is $R_E(\dot v+rJv+\dot rJe_i-r^2e_i)$. For a moving target vertex it is $\ddot p_T+R_T(\kappa A Jq_j-(\kappa V)^2q_j)$ from (1). These expressions provide $M$ on a bounded motion and input domain. A lower gap function on the interval supplies a conservative pointwise deficit under (16), even when an endpoint margin is insufficient to certify zero deficit.

**Local witnesses.** If the exact gap strictly exceeds $m$ over a compact family of compatible members and times, each member-time pair has a normal with positive surplus. Continuity of the finitely many vertex projections preserves this surplus on a neighborhood. Compactness gives a finite cover by such neighborhoods. Enclosures that converge uniformly as their diameters vanish can certify a sufficiently fine cover of the compatible family. The proof gives neither a fixed number of witnesses nor a guarantee for an outer polytope containing additional unsafe states. Local witnesses all certify the same common control. A single chart and normal per time already give (11) without requiring a partition.

## 4. A terminal set from the joint physical flow

Choose one realizable terminal command $u_f=(0,b_f)$ with constant $|b_f|<1$. All members receive the same command. It may be chosen to sustain forward rolling on an admissible straight trim. Terminal membership imposes the physical handoff to this command, including actuator memory where applicable. It does not require tracking the route, stopping, or exiting sensor range.

**A two-dimensional forbidden set.** Use inertial axes at the prospective terminal time. Let $r_0=p_T-p$. Fix the remaining state $\zeta$, comprising the body headings, velocities, target invariants, ego parameters and actuator state. Write $w_E(h),w_T(h)$ for the reference-point displacements over elapsed time $h$ under $u_f$, and $R_E(h),R_T(h)$ for the orientations. These four quantities are determined by $\zeta$ alone, since the free planar dynamics are translation invariant. In particular $w_E,R_E$ come from the nonlinear tire dynamics, and $w_T,R_T$ from (1). Let $\mathbb B^\circ_d$ be the open disk of radius $d$, and $d_*>0$ the design clearance. Define
$$
 \begin{aligned}
 D_h(\zeta)&=(w_E(h)-w_T(h))+\big(R_E(h)B_E\oplus(-R_T(h)B_T)\big),\\
 \mathcal F_f(\zeta)&=\bigcup_{h\ge0}\big(D_h(\zeta)\oplus\mathbb B^\circ_{d_*}\big),\\
 K_f&=\{(r_0,\zeta): u_f\text{ admits an admissible forward flow},
                       \ r_0\notin\mathcal F_f(\zeta)\}.
 \end{aligned} \tag{13}
$$
The vector shift translates the whole Minkowski sum. At time $h$, any difference of world-frame material points has the form $y-r_0$ with $y\in D_h$. Therefore $\operatorname{dist}(r_0,D_h)$ is the unsigned distance between the footprints. For $d_*>0$, exclusion from the open offset is equivalent to signed clearance at least $d_*$, including equality. The union in (13) keeps both vehicles at the **same time**. Intersecting the spatial paths at different arrival times would give a different set. Each $D_h$ is a translated sum of four segments, as in (8). The forbidden union can be nonconvex.

For any elapsed $s\ge0$, autonomy and displacement addition give
$$
 D_{s+h}(\zeta)=w_E(s)-w_T(s)+D_h(\zeta_s),\qquad
 r_s=r_0+w_T(s)-w_E(s).
$$
Hence membership in $K_f$ implies membership at every subsequent time under $u_f$. Conversely, violation of the distance part of (13) supplies a time and two material points closer than $d_*$ under this command. Thus (13) is the exact safe set for the specified continuation on its admissible forward model. Other controls may allow additional states. A terminal information set belongs to $\mathfrak Z_f$ when every consistent physical state belongs to $K_f$ and the common actuator handoff is admissible. Subsets inherit this property. Applying $u_f$ and then conditioning on new measurements preserves it.

### A sufficient membership test with strict interior

The description (13) admits physical certificates without constructing an infinite swept union. For a unit inertial direction $n$, let $H_T(n)$ be the largest projection of any future target material point. If $\kappa\ne0$, let $d_t=e(\psi_T+\beta)$, $C=p_T+Jd_t/\kappa$, and $c_0=Je(\beta)/\kappa$. Equation (1) implies
$$
 p_{T,q}(s)=C+R(\psi_T+\kappa s)(q-c_0).
$$
The center $C$ stays fixed. Over a full forward revolution
$$
 H_T(n)=n^\top C+\max_{q\in{\rm vert}(B_T)}\|q-c_0\| .
 \tag{14}
$$
This is also an upper bound for a partial revolution. For braking with remaining travel $S=V^2/(-2A)$, the exact support maximizes $n^\top[C+R(\psi_T+\kappa s)(q-c_0)]$ over each of the four corners and $0\le s\le S$. Each term is a sinusoid, so the maximum occurs at an endpoint or at alignment with $n$. For $\kappa=0$, let $S$ be the remaining travel, possibly infinite. The support is $n^\top p_T+h_{B_T}(R_T^\top n)+\sup_{0\le s\le S}s\,n^\top d_t$, with $h_B(k)=\sup_{q\in B}k^\top q$. This includes a straight ray traveling away from the separating plane. The support of the remaining future footprint cannot increase under time shift.

For the ego, suppose $F_d(V_*)=b_f(K_f+K_r)$ has a positive straight trim. Its body coordinates are $(V_*,0,0)$. Check local stability **for the physical parameters of the member**. At this trim the longitudinal eigenvalue is $-F_d'(V_*)/m$. The lateral and yaw Jacobian follows from $D_{\alpha_i}F_{yi}(0)=-C_i$:
$$
 L_{\rm lat}=\begin{bmatrix}
 -(C_f+C_r)/(m V_*)&(l_r C_r-l_f C_f)/(m V_*)-V_*\\
 (l_r C_r-l_f C_f)/(I_zV_*)&-(l_f^2 C_f+l_r^2 C_r)/(I_zV_*)
 \end{bmatrix}.
$$
Its trace is negative and its determinant is
$$
 \frac{C_f C_r(l_f+l_r)^2}{m I_z V_*^2}+\frac{l_r C_r-l_f C_f}{I_z}.
$$
Suppose that the determinant and $F_d'(V_*)$ are positive. The full body Jacobian $L_b$ is Hurwitz. For $z_b=(v_x-V_*,v_y,r)$, solve the Lyapunov identity $L_b^\top P+P L_b=-I$ with $P\succ0$. With $W=\sqrt{z_b^\top Pz_b}$ and $c_j=\sqrt{(P^{-1})_{jj}}$ one has $|(z_b)_j|\le c_j W$. Since the nonlinear remainder is $o(\|z_b\|)$, on a sufficiently small sublevel wholly inside the rolling domain there is a $\lambda>0$ with $\dot W\le-\lambda W$ almost everywhere. This sublevel is invariant. The proof applies directly with ideal force and steering actuation from Section 1. Additional actuator dynamics require their own trim and local decay condition.

Consequently every future ego heading lies in $I_\psi=[\psi-c_3W/\lambda,\psi+c_3W/\lambda]$, using an unwrapped local angle. This interval nests under propagation because $\int_0^s |r|\,dh\le c_3(W(0)-W(s))/\lambda$. Let $R_B=\max_{q\in B_E}\|q\|$. The velocity of a material point is $R(\psi)(v+rJq)$. At every future instant its projection onto $n$ is bounded below by
$$
 q_n=V_*\min_{\phi\in I_\psi} n^\top e(\phi)
                    -(c_1+c_2+c_3R_B)W .
$$
A sufficient terminal certificate for this member is therefore
$$
 \boxed{\qquad q_n\ge0,\qquad
 \min_{q\in B_E} n^\top(p+R_E q)-H_T(n)\ge d_* .\qquad} \tag{15}
$$
Every ego material point stays in the forward half-plane and the target remains behind it. The lower bound $q_n$ can only improve along the terminal flow. The unit normal is a witness for the terminal state. It can differ between members sharing $u_f$. As an example, take an ego at stable straight trim, a turning target with $\kappa$ bounded away from zero, and place the ego beyond the forward supporting line of the swept target with strict clearance. Choosing $n=e(\psi)$ gives $q_n=V_*>0$. Both inequalities persist on an open neighborhood by continuity, also for sufficiently small correlated uncertainty. This establishes a nonempty component under explicit physical trim conditions.

The same-time set (13) also covers states that no fixed spatial plane separates. For example, consider parallel straight motion on one line, the ego at trim speed $V_*$, the target ahead at speed $V_0$ with $A>0$. If $g_0$ is the initial bumper-to-bumper gap, the projected gap is $g_0+(V_0-V_*)h+Ah^2/2$. Its minimum is $g_0-[V_*-V_0]_+^2/(2A)$. If this is at least $d_*$ the state belongs to $K_f$, even though the two spatial paths overlap along an entire ray. The half-plane certificate (15) is sufficient only.

### A finite time interval for terminal membership

Consider a physically admissible forward continuation under $u_f$ in the trim neighborhood above. At its origin let $m_E=\min_i n^\top E_i(0)$ and suppose $H^+(n)$ bounds all target projections for all future times. The terminal normal here points toward the ego side. Define $K_v=c_1+c_2+c_3R_B$, $c=\min_{\phi\in I_\psi}n^\top e(\phi)$ and suppose $\alpha=V_*c>0$. From the bound on material-point velocity, every ego vertex has projected speed at least $\alpha-K_vW(0)e^{-\lambda h}$. Integration gives
$$
 \min_i n^\top E_i(h)\ge m_E+\alpha h-B(1-e^{-\lambda h}),\qquad
 B=K_v W(0)/\lambda .
$$
Consequently the following time is sufficient:
$$
 H_*=\left[\frac{H^+(n)+d_*-m_E+B}{\alpha}\right]_+,\qquad
 h\ge H_*\ \Longrightarrow\ g(h)\ge d_* .
$$
For such a state the collision condition in (13) is **equivalent** to $\min_{0\le h\le H_*}g(h)\ge d_*$ under the same $u_f$. Necessity is restriction in time. Sufficiency follows from the tail inequality and the finite-interval check. The ego and target may traverse the same point at different times and still satisfy this condition. Test forward admissibility as part of (13). For an information set it suffices to use an upper bound on all member cutoffs, e.g. from uniform bounds on $H^+(n)-m_E+B$ and a positive lower bound on $\alpha$. Normals may depend on the member while $u_f$ remains common. Certifying the finite interval by (10)–(12) and the continuous-time bound in Section 3 is sufficient and can be conservative. Equation (15) already certifies the entire tail when its inequalities hold at entry.

**Target support through the straight limit.** For a turning orbit (14) gives $H^+(n)$. For a one-sided curvature family including zero there is also a bound independent of the orbit radius. Choose $\sigma\in\{-1,1\}$ with $\sigma\kappa\ge0$, let $d_t=e(\gamma)$ be the initial course and take $n=-\sigma Jd_t$. For $\kappa\ne0$, (1) gives
$$
 n^\top[p_T(s)-p_T(0)]=-\frac{1-\cos(\kappa s)}{|\kappa|}\le0 .
$$
At zero curvature the projection is identically zero. Thus $H^+(n)=n^\top p_T(0)+R_T^b$ suffices, with $R_T^b=\max_{q\in B_T}\|q\|$. The target may advance arbitrarily far. For uncertain course the tangent normal must correspond to that member's course, and the positive ego drift must hold along those normals for the uniform cutoff. If both turn signs are compatible, the same tangent-side assertion generally fails for a common normal. Finite remaining target travel $S^+$ gives the independent bound $n^\top p_T(0)+S^++R_T^b$.

A simultaneous speed bound can also prove separation directly under $u_f$. For example, when all compatible targets have $A\le0$, their future material-point speed is bounded by $V^+(1+R_T^b|\kappa|^+)$ using terminal-time upper bounds. If the projected gap at entry is at least $d_*$ and every ego material point has projected speed at least this upper bound as in (15), that gap persists. All these inequalities certify the same terminal family (13). Terminal membership requires one admissible continuation and its safety evidence. Neither a turn sign nor departure from sensing range is an encounter admission condition. The finite cutoff concerns only the continuation under $u_f$. Its size can grow as $\alpha$ approaches zero.

## 5. One-horizon predictive barrier and nominal cruise

During an active encounter the input on a prediction interval $[t,t+T]$ is a common admissible function of elapsed time selected from the information at $t$. At a later decision the controller can condition on its newly received outputs. Let $\mathcal U_t(T)$ contain the inputs for which every consistent physical trajectory is admissible and strictly separated throughout the interval, and the joint states reached at its end belong to the terminal information family with a common continuation. Every member uses (1) for the target and the rolling ego flow under the common input. Safety during a continuing encounter is established under the declared target contract. A verified, observable departure from perception range ends that encounter once all compatible positions have cleared the boundary. The analysis may retain the target under its stipulated continuation through the full preview and (13), even if it leaves sooner. After observed departure the route controller resumes. A later appearance is a new encounter requiring feasible admission. The mathematical extension used in (13) requires the physical target to follow its contract only while its safety guarantee is invoked.

With $G_t$ from (9), the minimum required common slack is $c_t(h;u)=[d_*-G_t(t+h;u)]_+$. For bounded measurable deficits the continuous predictive barrier is
$$
 \begin{aligned}
 D_t(u)&=\int_0^T c_t(h;u)\,dh,\\
 \mathcal V_t&=\inf_{\substack{u\in\mathcal U_t(T),\ \xi\ge0\\
                               G_t(t+h;u)+\xi(h)\ge d_*\ \forall h}}
                       \int_0^T\xi(h)\,dh
             =\inf_{u\in\mathcal U_t(T)}D_t(u).
 \end{aligned} \tag{16}
$$
The equality follows by taking $\xi=c_t$ for each input. The feasible family enforces physical noncontact, $g>0$. Slack spends the design margin. An empty family has value $+\infty$. The primary criterion uses the accumulated safety deficit in the predictive-barrier construction of Huang, Wang, Margellos and Goulart. [Predictive Control Barrier Functions](https://arxiv.org/abs/2502.08400)

**Directional certificate for the primary objective.** For the proposed common input, let $l(h)$ be a lower gap certified on $P_t$. At a chosen unit normal and multipliers it is $\min_{ij}\{n^\top\bar D_{ij}-r_{ij}-U(c_{ij};\mu_{ij})\}$. Optimal witnesses recover (11). Over a time interval use the lower function derived in Section 3 from the two endpoint bounds with a common normal. The multipliers may differ between endpoints. With multiple sets covering the posterior, use the least of their certified gaps. Require $l(h)>0$ and $l(h)+\xi(h)\ge d_*$ throughout the horizon and establish (13) with a common continuation for all terminal members covered by the joint certificate. The least certified deficit for these witnesses is
$$
 \widehat c_t(h)=[d_*-l(h)]_+,\qquad \widehat D_t=\int_0^T\widehat c_t(h)\,dh .
$$
A certified terminal continuation supplies $l\ge d_*$ on its entire suffix. Minimize the certified deficit first and use the route CLF preference at equal deficit. The physical deficit satisfies $D_t\le\widehat D_t$. The certified optimum can exceed the exact value (16). Safety depends on the lower support in the separating direction. A wide state estimate can certify a large positive gap, and even an exact state estimate can give a collision. The observer therefore needs no accuracy threshold before feasible admission. The control throughout a proposed continuation is common to all current members.

**Arithmetic structure.** For $N$ requested times and $d$ latent coordinates, the four target vertices and their sensitivities have $O(Nd)$ entries independent of the trial command. Each new input requires one six-state ego flow and its shared first variation. If the chart derivative of the initial ego state and parameters factors as $V_1 V_2$ of rank $d_E$, then $S_E=(D_{(x_E(0),\vartheta)}x_E\,V_1)V_2$. Only $d_E$ columns need dynamic propagation. The sixteen support queries at each time use the original joint coordinates.

For a fixed certificate on a fixed polytope, the pair $(H^\top\mu,b^\top\mu)$ determines $U(c;\mu)$ for **all** directions $c$. Once these aggregates are known, evaluating that witness at a changed direction costs $O(d)$, independent of the retained row count $q$. Exact support with all multipliers free instead has $q$ observation multipliers per query. Lifting all sixteen queries at $N$ times gives $16Nq$ such multipliers, in addition to any variables used for box absolute values. Safety needs a sufficient lower gap and the inherited deficit budget. It does not require an optimal support witness at each query. These arithmetic counts exclude constructing new witnesses, obtaining valid remainder bounds and optimizing the control trajectory. The chart dimension alone gives no worst-case latency bound.

**Why one interval suffices.** A feasible common input reaching $\mathfrak Z_f$ at any $\sigma\in[0,T]$ extends by $u_f$ for the remaining duration at zero deficit. Conversely a feasible fixed-duration plan is a free-arrival plan with $\sigma=T$. The primary infima are equal, and secondary costs can be compared over the full extensions. There is no mathematical junction separating an avoidance maneuver from a return maneuver. The terminal condition supplies an extension wherever the plan ends. The earliest feasible arrival, if attained, is determined by the dynamics, current information and terminal set. The equivalence places no preset lower bound on maneuver duration.

**Route CLF.** Let a smooth path have arc coordinate $s_\Gamma$, tangent angle $\theta(s_\Gamma)$, curvature $k_\Gamma$ and signed lateral error $e_y$. In a regular path chart, with course $\chi=\psi+\operatorname{atan2}(v_y,v_x)$ and speed $v_c=\|v\|$,
$$
 \dot s_\Gamma=\frac{v_c\cos(\chi-\theta)}{1-k_\Gamma e_y},\qquad
 \dot e_y=v_c\sin(\chi-\theta).
$$
A realizable constant-curvature trim satisfies $\|v^\star\|=V_{\rm ref}$, $r^\star=k_\Gamma V_{\rm ref}$, $f_d(x^\star,u^\star)=0$ and $\psi^\star-\theta=-\arg(v_x^\star+i v_y^\star)$. For transverse error $e_c=(e_y,\psi-\theta-\psi^\star_{\rm rel},v-v^\star,r-r^\star)$ put $\psi^\star_{\rm rel}=-\arg(v_x^\star+i v_y^\star)$, use the locally wrapped angle and let $A_c,B_c$ linearize these error dynamics. For a realizable feedback $u^\star+K_c e_c$ suppose $(A_c+B_cK_c)^\top P_c+P_c(A_c+B_cK_c)\preceq-2\alpha P_c$, $P_c\succ0$, $\alpha>0$. Differentiating $V_c=e_c^\top P_c e_c$ yields $\dot V_c\le-2\alpha V_c+o(\|e_c\|^2)$ and local exponential cruise convergence on a sufficiently small admissible neighborhood. A varying-curvature reference requires a realizable reference trajectory and the corresponding time-varying CLF inequality, including $\dot P_c$ if applicable.

Let $\bar u_t$ be the given causal nominal CLF forecast computed from available ego information. Among equal primary deficits, minimize $\int_0^T (u-\bar u_t)^\top W_u(u-\bar u_t)\,dh$ for $W_u\succ0$. If nominal cruise is feasible at the minimum deficit, it has zero deviation. For nonzero ego uncertainty, actual cruise convergence additionally requires a suitable output-feedback CLF bound. Recursive collision avoidance alone supplies no global path-attraction claim.

## 6. Recursive feasibility and the role of feedback

Suppose the initial information contains the truth, a feasible common plan exists at admission, and the physical and measurement contracts hold. During a continuing encounter, follow a prefix of length $0<\delta\le T$. At the next decision let $T'\ge T-\delta$ and assume the admissible input class allows restriction and concatenation at the certified terminal handoff. This permits a decreasing time horizon. An even earlier cut is admissible when terminal membership is established at that cut, since truncating also removes nonnegative deficit. For a replacement plan use a feasible minimum-deficit plan when attained, or a feasible plan respecting the inherited remaining deficit.

**Shift proof.** For each member surviving (3), uniqueness of flow under the old input suffix reproduces exactly the same two rectangles at the same physical time. Taking an infimum over the smaller set in (9) can only improve the gap. Thus the old witnesses remain valid even if a new maximizer in (8) is used. The terminal members form a subset of the old terminal set. Append $u_f$ for $T'-(T-\delta)$ using (13). This gives a feasible plan $u'$ and
$$
 D_{t+\delta}(u')\le D_t(u)-\int_0^\delta c_t(h;u)\,dh . \tag{17}
$$
In particular for an attained old primary optimum, $\mathcal V_{t+\delta}\le\mathcal V_t-\int_0^\delta c_t(h;u)\,dh$. For a stored feasible plan the right side of (17) is the acceptance budget. Induction gives strict physical separation on every executed prefix, for decision times covering the encounter without finite-time accumulation. With zero initial deficit, continuous physical clearance along each member also gives clearance at least $d_*$ everywhere: a strict shortfall would persist for an interval and contribute a positive integral. Feasible admission is an assumption about the information available when an encounter begins. Two individually avoidable states may admit no common safe input.

**Shift of a finite certificate.** Keep the executed plan's reference chart on its suffix. At a given physical time the inherited $n,M_{ij},r_{ij}$ and direction $c_{ij}$ are unchanged. After appending $(H_{\rm new},b_{\rm new})$ in (7), each old multiplier extends as $(\mu_{ij},0)$. Its two aggregates are identical. A nested tightening of the box can only decrease its support at the same residual direction. Thus every inherited lower gap remains valid, including on a lower-dimensional posterior. Exact removal of redundant rows also preserves support. If a nonredundant row is discarded, containment of the truth alone does not certify an old gap on the enlarged polytope. Retain the original polytope certificate, or re-establish the gap and the terminal witness on the new enclosure before replacing it.

Restrict the certified lower-gap function on each surviving interval, including an interval cut in its interior. Keep the remaining part of the terminal proof and its tail under $u_f$. By (17) this gives an admitted certificate with deficit at most $\widehat D_t-\int_0^\delta\widehat c_t(h)\,dh$. A replacement using a fresh reference and polytope must respect this budget. Once a certified plan exists, its executable suffix and terminal continuation maintain the safety claim under the same model and information assumptions. Feasible admission for a new encounter remains a separate requirement.

The coupling in (17) uses the identical latent motion in sensing, clearance and terminal propagation. It requires neither a dwell time for an estimate to settle nor a preset passing side. The common-input forecast can be conservative where different future measurements would permit different subsequent actions. The result establishes safety on its feasible information domain.
