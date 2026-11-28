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

The target reference point $p_T$ has speed $V$, body yaw $\psi_T$, constant tangential acceleration $A$, constant sideslip $\beta\in(-\pi/2,\pi/2)$ and known rear reference distance $l_T>0$. Its moving dynamics are
$$
 \dot p_T=V e(\psi_T+\beta),\qquad \dot\psi_T=\kappa V,\qquad
 \dot V=A,\qquad \kappa=\sin\beta/l_T .
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

### Equivalence to the nonlinear relative motion model (NRMM)

The target law in the observer and in every forecast is the constant-tangential-acceleration, constant-sideslip kinematic single track used by the NRMM. The following coordinate identity fixes its meaning independently of the estimator. [Sharma, Alai and Rajamani](https://doi.org/10.1016/j.trc.2025.105411)

Put $q=R(\psi)^\top\dot p_T$, $s_B=R(\psi)^\top\ddot p_T$, $d_B=q/V$, $\Omega=\kappa V$, and write $\nabla_r z=\dot z+rJz$ for differentiation of a world vector expressed in rotating ego axes. Then $s_B=A d_B+V\Omega Jd_B$ and $\nabla_r d_B=\Omega Jd_B$. Since $\dot\Omega=\kappa A$, differentiation gives the following Cartesian NRMM.
$$
 \nabla_r\rho=q-v,\qquad \nabla_r q=s_B,\qquad
 \nabla_r s_B=-\Omega^2q+3A\Omega Jd_B. \tag{2}
$$
Conversely, for $V=\|q\|>0$ recover $A=d_B^\top s_B$, $N=(Jd_B)^\top s_B$, $\Omega=N/V$ and $\kappa=N/V^2$. Along (2), $\dot V=A$, $\dot A=\Omega N-\Omega^2 V=0$ and $\dot N=-A\Omega+3A\Omega=2A\Omega$, hence $\dot\kappa=0$. The physical body yaw is $\psi_T=\psi+\arg q-\arcsin(l_T\kappa)$ modulo $2\pi$. Substitution recovers (1), the same relative-position output and the same four body corners for every admissible ego input. These maps give a bijection of moving trajectories on $|l_T\kappa|<1$. At an absorbing stop (1) carries the last orientation. The inverse map at $q=0$ is outside the identification domain. The equivalence concerns the physical model. A change of observer does not transfer an ISS bound proved for a particular innovation law.

## 2. One joint observer, from positions to occupied footprints

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
For a history contract the intersection is on histories with a common prefix, followed by projection if desired. The image of the surviving histories under the joint flow is the current state estimate. The true member survives if the prior and model contain it. Every predicted ego pose, target corner and future clearance for a member uses that same member. This is a nonlinear set-membership observer. The controller below acts on its information and its applied input determines the ego motion in the next observation constraint. Recursive set-membership estimation provides the general conditioning principle. [Bertsekas and Rhodes](https://doi.org/10.1109/TAC.1971.1099674)

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

### A bounded-size outer estimate

The exact history set (3) specifies the information to be covered. For bounded online work, fix a joint affine chart $z=z_*+S\lambda$ in independent scalar coordinates with an unwrapped course angle for the unit course. The matrix $S$ can mix ego and target coordinates. Enclose the initial set by $\lambda\in I=\prod_{j=1}^d I_j$. Fix this chart over an encounter. An affine change of frame may remove a shared rigid displacement from the relative prediction. Assume the stated noise bounds and a finite-dimensional description for any persistent biases used here.

For a scalar raw output $y$, let $\bar H+\sum_j a_j\lambda_j$ be a reference expansion of its physical observation map under the known input. Suppose a bound $r_H$ encloses the expansion remainder for all members and $\epsilon$ bounds the measurement residual. Writing $B=[y-\bar H-(r_H+\epsilon),y-\bar H+(r_H+\epsilon)]$, impose the necessary strip $\sum a_j\lambda_j\in B$. For each $a_j\ne0$ the interval update is
$$
 I'_j=I_j\ \cap\ \frac{B-\sum_{k\ne j}a_k I_k}{a_j}.
 \tag{7}
$$
The right side uses the prior box. If $a_j=0$, retain $I_j$. A disjoint strip signals inconsistency. Every compatible member survives each row and the resulting box is nested in the same chart. Row-wise residual bounds may discard information in a joint noise contract, increasing the outer set. The observer never equates such an outer set with (3).

One construction for the strips propagates the reference ego flow and its fixed-size parameter sensitivities under the applied input and evaluates (1) for the target. For example, on smooth compact charts, a uniform bound $L$ on the flow-field Jacobian and a bound $M$ on its second derivative give comparison bounds $\dot U=LU$, $\dot W_2=LW_2+MU^2$ on first and second sensitivities of the augmented flow with constant parameters. Initialize them with bounds on the chart derivatives. The chain rule with the output derivatives then bounds its Hessian by $M_H$ and $r_H=\frac12 M_H\sup_{\lambda\in I}\|\lambda\|^2$ suffices for expansion at zero. The constants must cover the line segments of the expansion. Across a nonsmooth physical branch a valid interval remainder suffices. Omitting an uninformative row retains the prior. This construction advances with the actual input and does not replay the encounter history. Once a row and its remainder are available, its full interval sum and endpoint totals with each term removed cost $O(d)$ arithmetic. For $m_y$ scalar outputs the update costs $O(m_y d)$ beyond the fixed-dimensional flow and sensitivity propagation. The chart, intervals and reference flow use storage independent of the number of past measurements. The two-constraint existence theorem below is not needed to find any active observations.

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

There is a sharper consequence of working in the plane. Suppose the conditional set is the nonempty intersection of closed convex prior and observation constraints indexed by time as in (4). Fix any $\varepsilon\ge0$ strictly below the conditional gap. In center-displacement coordinates the forbidden set $K=Z_\gamma\oplus\overline{\mathbb B}_\varepsilon$ is compact and convex. The whole family of translated constraints together with $K$ has empty intersection. Compactness of $K$ gives a finite subfamily with empty intersection. The planar Helly theorem reduces it to at most three sets. One must be $K$, since the observation and prior constraints alone are consistent. **At most two of those constraints therefore suffice to certify this particular strict margin.** The witnesses may change with the future time, input, latent motion or queried margin. At equality the result applies to every strictly smaller nonnegative margin. It gives no universal pair of observations that identifies the target.

For completeness, the planar theorem follows by taking a minimal finite family with empty intersection. If it had at least four sets, choose for each set a point common to all the others. Any four such points admit two disjoint nonempty groups with intersecting convex hulls: take an affine dependence among the four planar points and normalize its positive and negative parts. A point in both hulls lies in every set, since at least one group omits each set's exceptional point. This contradicts emptiness. Thus three suffice. Here the theorem applies to the observation constraints and the actual rectangular collision geometry in the same two translation coordinates.

### A rectangle constraint with fixed local cost

Equation (9) is the exact reference margin. A finite certificate can eliminate the inner minimization over latent motions. At the prediction origin express each member in its own current ego frame, with the common reference ego pose at the origin and heading zero. This removes the common absolute yaw and translation from collision geometry. Map the outer estimate into this frame with guaranteed bounds. Use one reference body motion and one reference NRMM target motion, with a bar denoting reference quantities. The following construction uses projections of a joint set and may lose some of its correlations.

Let $\epsilon_p,\epsilon_\gamma,\epsilon_V,\epsilon_A,\epsilon_\kappa,\epsilon_\beta$ bound the deviations of target reference position, course, speed, tangential acceleration, curvature and sideslip at the prediction origin. Enclose $\beta$ through the same curvature interval under $\arcsin(l_T\kappa)$. If a previously moving member is absorbed at a later origin, retain its last course and sideslip and set its initial speed to zero. Its persisted braking acceleration gives zero remaining travel in the positive-part integral below, including when other compatible members are still moving. These quantities also follow at a later origin from the original chart by (1), the actual-input ego flow enclosure, and a rigid transformation. Let $C(x)=2\sin(\min(\pi,x)/2)$ for $x\ge0$. For elapsed time $h$, write $\bar s$ for reference travel as in (1) and put
$$
 E_s=h\epsilon_V+\tfrac12 h^2\epsilon_A,\qquad
 E_{\psi T}=\epsilon_\gamma+|\bar\kappa|E_s+
                   (\bar s+E_s)\epsilon_\kappa+\epsilon_\beta .
$$
The bound on travel follows by integrating the 1-Lipschitz positive part of $V+Ah$, including an absorbing stop from a moving start. Since $|\partial_s F_\kappa|=1$ and $|\partial_\kappa F_\kappa(s)|\le s^2/2$, the displacement error of **every corresponding target material point** is bounded by
$$
 E_T(h)=\epsilon_p+|F_{\bar\kappa}(\bar s)|C(\epsilon_\gamma)+E_s
           +\tfrac12\bar s^2\epsilon_\kappa+R_T C(E_{\psi T}),
 \qquad R_T=\max_{q\in B_T}\|q\|. \tag{10}
$$
Indeed, change the travel from $s$ to $\bar s$ at the true curvature, then change curvature at $\bar s$, then rotate the reference displacement through the course error. The last term bounds rotation of the corner by the body yaw error. The coefficients are valid also for straight motion. For a fixed joint chart one can tighten the same scalar allowance before any trajectory query. Let $b_j+D_j\lambda$ approximate target corner $j$ in the prediction-origin frame, here using centered intervals $|\lambda_i|\le w_i$, with norm remainder at most $r_j$. Expand about the reference NRMM corner $b_j$ and retain the same joint coordinates through the relative-frame transformation. The image $D_j[-w,w]$ is a planar sum of $d$ segments. Its maximum norm occurs at one of at most $2d$ extreme vertices, obtained by ordering the segment directions and following the boundary. Thus $\max_j(r_j+\max_{v\in D_j[-w,w]}\|v\|)$ also bounds all corner displacements. Convex interpolation gives the same bound for all material points. Use the smaller valid bound for the same reference trajectory. This optional tightening costs $O(d\log d)$ per time argument for four corners and adds no trajectory-dependent terms. The error in (10) bounds uncertainty in evaluation of an NRMM orbit; the central and compatible target motions still obey (2).

A corresponding ego bound must cover the dependence of ego motion on the candidate input. On a declared rolling domain suppose the body field admits uniform incremental bounds $L_b,L_\vartheta$ in a specified body norm, for **the same** command and all allowed parameters. Let $B(0)$ bound the initial body-state error and let $E_\vartheta$ bound the parameter error. Set
$$
 \dot B=L_b B+L_\vartheta E_\vartheta,\quad
 \dot E_{\psi E}=c_r B,\quad
 \dot E_p=c_v B+\bar V_E C(E_{\psi E}),\qquad
 E_E=E_p+R_E^b C(E_{\psi E}).
$$
Here $c_v,c_r$ bound the velocity and yaw-rate coordinate projections, $\bar V_E$ bounds body speed on that domain, $R_E^b=\max_{q\in B_E}\|q\|$, and $E_{\psi E}(0)=E_p(0)=0$ in the relative chart. The one-sided constant $L_b$ can be the supremum of the logarithmic norm of the body Jacobian on an enclosing convex rolling chart. In a quadratic norm with weight $P$, it is bounded by the largest generalized eigenvalue of $(P Df_b+Df_b^\top P)/2$ relative to $P$. This retains dissipation where the physical field has it, including on stable trim neighborhoods. Grönwall's inequality bounds body error by $B$. Integrating yaw rate and inertial velocity gives the next two bounds. Apply these constants only to tubes contained in their validity domain, including physical and actuator limits. An uncertain dynamic actuator requires the corresponding augmented bound. All four scalar envelopes above depend on the information and elapsed time and can be formed before the control search. A uniform bound may be conservative over a long preview.

For nominal rectangles $\bar E,\bar T$ define
$$
 \widehat G_t(t+h;u)=g(\bar c,\bar Q)-E_E(h)-E_T(h)\ \le\ G_t(t+h;u).
 \tag{11}
$$
For any unit normal the two support projections can change by at most the respective material-point errors. Taking the maximizing nominal normal proves (11), including its sign. With exact data and model parameters these error allowances vanish. Both nominal footprints retain the exact rectangle dual. Given required clearance $m$, the inequality $\widehat G_t\ge m$ is equivalent to existence of an angle $\theta$ and an offset $\ell$ with $n=e(\theta)$ such that
$$
 \ell-n^\top\bar E_i\ge0\quad(i=1,\ldots,4),\qquad
 n^\top\bar T_j-\ell\ge m+E_E+E_T\quad(j=1,\ldots,4). \tag{12}
$$
The witnesses can vary with time. For fixed time and uncertainty data these eight smooth vertex inequalities eliminate the absolute values and active-feature search in (8). The equivalent scalar margin can also be evaluated directly on the boundary of $Z(\bar Q)$. Project $\bar c$ onto each nonzero edge segment, take the smallest distance, and assign its sign by membership in the polygon. There are at most eight segments. The projection argument for (8) proves equivalence to the full directional dual, including diagonal clearance. Where the closest feature persists smoothly and the gap is positive, its distance eliminates both witness scalars locally. At feature ties or changes in polygon combinatorics the scalar expression can be nonsmooth. Equation (12) remains a smooth existential description in the pose and witness for fixed uncertainty data. The control-dependent local calculation needs one ego trajectory. The target data and error allowances are independent of the candidate command. This gives a sound restriction of the exact feasible information domain. Wide allowances can exclude a physically safe plan.

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

### Finite terminal evidence

For the bounded-cost formulation, use a sufficient certificate for the actual continuation $u_f$. Choose a compact parameter range on which the trim neighborhood, norm decay, and the constants used in (15) have uniform, established bounds. Test its membership on the whole terminal ego enclosure. Given one unit normal $n$, let $\underline q_n$ lower-bound $q_n$ in (15) over this enclosure. Interval evaluation of its scalar coefficients and the minimum cosine over its heading interval suffices. A lower bound on the ego minimum projection is $\min_i n^\top\bar E_i-E_E(T)$.

An upper bound $H^+(n)$ on the future target support can be formed from the NRMM packet. For instance, if the curvature interval excludes zero, interval evaluation of $C=p_T+Jd_t/\kappa$ gives a planar box with center $c_C$ and coordinate radii $r_C$. Bound $\max_q\|q-Je(\beta)/\kappa\|$ from above by $R_C$ at its four body vertices using interval arithmetic. Then $H^+(n)=n^\top c_C+|n|^\top r_C+R_C$ follows from (14), with componentwise $|n|$. For uniformly finite remaining travel $S^+$, the support of the current reference-position box plus $S^++R_T$ also suffices, including intervals crossing zero curvature. For an exactly straight unbounded ray one can use the position-box support plus $R_T$ whenever the entire course interval satisfies $n^\top d_t\le0$. These bounds are evaluated at the prospective terminal time. They apply independently of the trial ego command in the prediction-origin frame.

A synchronized certificate follows from a bound on future projected speed. If all compatible targets have $A\le0$, their future material-point speed is at most $V^+(1+R_T|\kappa|^+)$, where the superscripts denote bounds at the endpoint. A current projected gap of at least $d_*$, together with $\underline q_n\ge V^+(1+R_T|\kappa|^+)$, preserves this gap for all future times under $u_f$. This follows by differentiating the projected material-point differences. For the spatial support certificate require $\underline q_n\ge0$ and $\min_i n^\top\bar E_i-E_E(T)-H^+(n)\ge d_*$. Each is a sufficient witness for the same terminal set and command. In particular an interval containing zero curvature with unbounded travel need not admit a finite swept-support bound. Failure to establish a sufficient witness leaves membership unresolved. It imposes no braking or turning assumption on the target model. These finite tests need a fixed number of scalar operations and four vertices per body, given the uniformly valid trim bounds. They do not evaluate the infinite union in (13).

## 5. One-horizon predictive barrier and nominal cruise

During an active encounter the input on a prediction interval $[t,t+T]$ is a common admissible function of elapsed time selected from the information at $t$. At a later decision the controller can condition on its newly received outputs. Let $\mathcal U_t(T)$ contain the inputs for which every consistent physical trajectory is admissible and strictly separated throughout the interval, and the joint states reached at its end belong to the terminal information family with a common continuation. All predictions use (1) and the rolling flow under the same input. Safety during a continuing encounter is established under the declared target contract. A verified, observable departure from perception range ends that encounter once all compatible positions have cleared the boundary. The analysis may retain the target under its stipulated continuation through the full preview and (13), even if it leaves sooner. After observed departure the route controller resumes. A later appearance is a new encounter requiring feasible admission. The mathematical extension used in (13) requires the physical target to follow its contract only while its safety guarantee is invoked.

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

**Finite certificate for the same primary objective.** For a fresh plan use the lower margin (11), impose $\widehat G_t>0$ and $\widehat G_t+\xi\ge d_*$ throughout the horizon, and require finite terminal evidence for every covered state under the common continuation. Equivalently, for each such input its certified deficit is $\widehat D_t=\int_0^T[d_*-\widehat G_t]_+\,dh$. A valid terminal continuation certifies a gap of at least $d_*$ on its entire suffix and contributes zero there. Minimize the certified deficit first and use the same secondary cruise criterion. Thus feasible certificates imply physical feasibility and $D_t\le\widehat D_t$. Their optimum can exceed the exact value in (16). A certificate consists of its reference, enclosing data, margin on the chosen input and terminal witness. The admitted family includes restrictions of the currently committed certificate.

**Arithmetic structure.** At $N$ requested time arguments, preparing the NRMM footprint and the two allowances takes $O(N)$ scalar work for fixed chart dimension given the root bounds, using the closed target flow and scalar comparison functions (the angular chord can also be bounded by its argument to give explicit integrals). Each further trajectory evaluation uses one six-state ego flow, two physical input channels and $N$ constant-size geometry terms. Each term in (12) uses two local witness scalars and eight rows. A variable margin can use one additional scalar slack. Neither measurement-history length nor the number of compatible target motions multiplies this count. The derivatives of each smooth local geometry term have constant size with respect to its local pose and witness. A common ego trajectory still couples times through its dynamics. These counts describe local evaluations and sparsity. A dense derivative of a fully eliminated trajectory, a variable integration workload, or an increasing number of nonlinear iterations can dominate elapsed time. The continuous path and actuator constraints still require valid coverage of the interval in any realization of the theorem. A finite list of time arguments alone gives no such coverage.

In particular, linearizing the nominal ego gives $\delta\dot x=A_E(h)\delta x+B_E(h)\delta u$ with six state components. At a smooth local clearance row $b(h)$ and fixed witness, the influence of a prior input variation at $s<h$ is $b_x(h)\Phi_E(h,s)B_E(s)$, where $\Phi_E$ is the ego transition matrix. This follows by variation of constants. There is no target sensitivity to the trial input in prediction-origin axes. With $p$ independent control coefficients the full reduced row Jacobian can still have order $Np$ entries. Keeping the local state relation exposes the fixed-width temporal coupling directly. This distinction matters when the number of input coefficients increases with the preview.

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

**Shift of a finite certificate.** A fresh outer estimate with a new center can give a smaller lower margin even after a more informative observation. For the retained input suffix keep its original chart, error envelope, margin and terminal witness, indexed by the original physical time. Every surviving physical trajectory is still covered because its applied prefix agrees and its fixed-epoch box only shrank. Beyond the old endpoint use the inherited terminal proof under $u_f$. Restriction and this extension produce an admitted certificate with remaining deficit at most $\widehat D_t-\int_0^\delta\widehat c_t(h)\,dh$, where $\widehat c_t$ is its committed deficit density. A replacement using fresh data must satisfy this same budget. Keeping a certificate for one active plan and its terminal continuation suffices; older observation histories and chains of earlier certificates are unnecessary. Once a certified plan exists, subsequent safety can therefore survive an interrupted search, provided its suffix and terminal command are executable on time and remain representable. The latency of first feasible admission is a separate requirement.

The coupling in (17) uses the identical latent motion in sensing, clearance and terminal propagation. It requires neither a dwell time for an estimate to settle nor a preset passing side. The common-input forecast can be conservative where different future measurements would permit different subsequent actions. The result establishes safety on its feasible information domain.
