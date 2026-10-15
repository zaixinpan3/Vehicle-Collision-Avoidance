# Feedback-aware prediction over finite horizons

Date: October 12, 2026. Design/theory, not the current runtime formulation.
The current theoretical scope omits road boundaries. Observer, model, input,
handling and collision premises remain. Optimized current inputs are issued
directly; feedback belongs in the optimization's policy and prediction.

Scope clarification, October 13, 2026: the user requires future observer
updates and causal feedback to enter prediction, so uncertainty is not
propagated as if the future inputs were fixed open loop. Tube widths may
vary along the horizon. A constant-width or uniformly contracting nonlinear
invariant tube is not a required design objective. Earlier fixed-width
constructions below remain optional research diagnostics.

The October 13 extension below treats persistent vehicle-parameter mismatch,
residual plant disturbances, measurement noise and the actual observer as one
joint system. It supplies a constructive outer-image argument and a shift
proof that can reuse certified tubes. These are mathematical constructions;
the existing NRMM probes still use nominal plant parameters and numerical
local maps, and are not their nonlinear implementation.

## Mathematical object and proof completion

The theorem concerns the joint nonlinear output-feedback design defined in
this note, not whether a production program implements it. Its controller
selects causal policies and sets satisfying the universal stage constraints
and the common robust terminal continuation, then executes its definite
current input. Model mismatch, observation noise and nonzero estimation
uncertainty are retained in that problem.

For a purely mathematical formulation, take every prediction set to be its
exact joint reachable image T_pi(J). Image containment is then a consequence
of the definition, and monotonicity follows by set inclusion. The terminal
family is defined by common admissible safe continuations; its downward
closure and successor closure are proved below and in the recursive-feasibility
note. Posterior conditioning, policy closure and the slack shift complete
the recursive-feasibility argument. The second soft CLF stage preserves
existence of a feasible point. This is a complete conditional theorem for
that mathematical design, under its declared model/observer contract and
initial feasibility; it does not assume exact state information.

Computing numerical tube radii, validating a particular approximate enclosure,
constructing a tractable terminal-membership routine or implementing these
operations is not required for this exact-set existence proof. If the
mathematical design instead specifies a particular approximate tube or hard
restriction, its soundness and preservation of the shifted candidate must
be proved; an unproved approximation cannot borrow the exact-set theorem.
The nonlinear bounds and carried certificates below provide sufficient
constructions for that extension. Production correspondence is outside the
requested theoretical completion criterion.

## Predict a control rule that uses future observations

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

## Propagate a varying closed-loop tube over the finite horizon

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

The remaining nonlinear work is to construct sound finite-horizon images,
admit their policy-generated inputs and constraints, and connect the endpoint
set to the robust terminal continuation. Once entering the mild terminal
model, braking and a fixed physical parked pose set may be certified without
requiring continued contraction of the noisy yaw estimate. Recursive
feasibility uses the cross-sample inclusion and terminal shift argument below;
it does not require a constant prediction-tube width.

## One uncertainty source and its different consequences

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

## Construct sound nonlinear images, then propagate by induction

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

## Finite-horizon widths with persistent mismatch and nonlinear reserves

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

## Optional fixed-width construction

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

## Include the actual observer update in the prediction

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

For clarity, consider a linear predictor-form illustration:

    x_next = A*x+B*u+w,          y=C*x+n,
    xhat_next = A*xhat+B*u+L*(y-C*xhat),
    z_next = A*z+B*v,            u=v+K*(xhat-z).

With e=x-xhat and delta=xhat-z,

    [e_next    ] = [A-L*C    0    ] [e    ] + [I  -L] [w],
    [delta_next]   [ L*C   A+B*K ] [delta]   [0   L] [n].

The actual tracking deviation is e+delta. The same measurement-noise column
enters the two components with opposite signs. Its immediate contribution to
their sum cancels under this predictor timing; noise still affects later
physical inputs through the estimate. Treating these components independently
can lose the cancellation. In the real code shared gyro and common-pose
columns likewise must remain shared; primitive sensor noise and the observer
error it produces must not be added as unrelated disturbances.

If measurement correction or reconstruction feedthrough precedes the current
control, include its immediate control-to-plant noise path as well. The runtime
publishes measured yaw rate directly in the ego output, so the real feedback
can have such feedthrough. The illustrative cancellation does not mean that
sensor noise cannot affect physical motion through the chosen control.

If both diagonal blocks are Schur stable, this illustrative augmented matrix
is Schur stable. A bounded noise set then admits an augmented invariant tube
Omega satisfying A_joint*Omega+noiseImage subset Omega. The physical tube is
the projection [I I]*Omega, and input deviation is the projection [0 K]*Omega.
At initialization z=xhat implies delta=0; e retains the observer-certified
nonzero initial error. These correlations should be retained in the initial set.

This stability statement concerns the error blocks in this illustration.
Including persistent unknown parameters adds identity parameter dynamics,
as in (M3); full augmented Schur stability must not be inferred from it.

This is a linear explanatory derivation, not our NRMM observer equation.
For the nonlinear implementation derive the joint map/Jacobian or certified
comparison dynamics from the actual point flow, reconstruction and bound
updates. Validate sound finite-horizon joint images over the admitted nominal,
error and input domains, including midpoints and mode transitions.
Stable observer ISS and the existing path CLF do not by themselves certify
incremental joint contraction. The uncontrolled target's absolute-position
spread remains; relative/coupled safety coordinates are still needed.

The October 13 research probes now execute the actual NRMM update and bound
memory, rather than injecting arbitrary bounded errors in place of an observer.
For an ego-only cruise policy, the point map reduces to six physical states
and five persistent ego observer states: GNSS predictor resets remove its
memory, and the target point subsystem does not feed this ego-only policy.
Four shared sensor-noise vectors are retained per controller hold. The known
current software state and the common current-input first-hold image are
included in the initial set. At 8 and 15 m/s, the numerical local joint maps
are Schur stable and admit fixed linear physical position half widths of
about 0.244 m longitudinally and 0.20 m laterally. Actual nonlinear replay
over 25.6 s keeps sampled position errors below 0.081 m, with available
current observer bounds containing the tested errors.

An additional actual-target-tracker following probe preserves relative
separation despite 5.12 m branch spread in both vehicles' absolute position.
It is evidence for using relative/joint safety coordinates, not a guarantee
over the entire published target-parameter set. The numerical maps are
secants above the force-balance inverse's bisection resolution; no uniform
nonlinear remainder or interval arithmetic certificate is supplied.
See [actual NRMM probe methods and limits](../report/JOINT_NRMM_PREDICTION_20261013.md)
and `scripts/verifyJointNrmmPrediction.m` / `scripts/verifyNrmmRelativeFeedback.m`.
Production optimization and the complete recursive-feasibility claim remain
unchanged by these research probes.

Including the updater enables a tighter causal description; it does not
automatically guarantee a smaller tube. A poor outer approximation can still
discard its correlations. The full updater can be used for offline proof and
coefficient construction, with compact certified radius/generator recursions
online. Observer states need not all become optimization decision variables.
The shifted-policy proof below applies to this augmented transition when its
outer images are sound and their causal-policy witnesses and certificates
are retained. A monotone outer operator is one sufficient construction.

## Optional invariant set for the sampled tangent model

The research verifier uses straight-cruise sampled A,B and a fixed six-state
gain K. Observer error lies in G*U, U=[-1,1]^6; sign symmetry gives disturbance
D*U, D=B*K*G. Write A_c=A+B*K. Choose positive P, C'*C=P and
rho=||C*A_c/C||_2<1. Let E(t)={e: ||C*e||_2<=t}. For finite m define

    Z_m = Minkowski sum_{j=0}^{m-1} A_c^j*D*U,
    sigma_m = max_{xi in U} ||C*A_c^m*D*xi||_2,
    R_D = Z_m + E(sigma_m/(1-rho)).

The disturbance images shift by one, their new last term lies in E(sigma_m),
and the tail ellipsoid contracts by rho. Hence A_c*R_D+D*U subset R_D.
To retain the initial observer box without a large scalar inflation, use

    sigma_0m = max_{xi in U} ||C*A_c^m*G*xi||_2,
    H = convex hull(G*U,A_c*G*U,...,A_c^(m-1)*G*U,E(sigma_0m)).

Each box maps to the next, the last maps into E(sigma_0m), and the ellipsoid
contracts into itself. Thus A_c*H subset H. R=R_D+H contains G*U and satisfies

    A_c*R+D*U subset R.                                     (5)

Its directional support is the disturbance-support sum, an ellipsoid support
and the maximum support of the initial orbit hull. Coordinate-wise widths
are outer displays, not independent boxes asserted invariant. Input admission
uses support along rows of K plus support of K*G*U. Correlated observer
generators can replace G, retaining shared columns. Measurement noise must not
also be invented as a physical plant force. The numerical verifier constructs
(5) for tangent matrices; it is not an outward-rounded machine certificate or
a nonlinear Fiala certificate.

## Quantify the nonlinear and mode-dependent extension

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

## Preserve correlation with the uncontrolled target

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

The verifier also constructs a joint relative tube for the declared straight
following tangent model. Target speed is an unknown constant in [14.9,15.1]
m/s. Define eta from relative position (desired separation 6 m), ego heading,
ego speed minus target speed, lateral velocity and yaw rate. Use the causal
rule u=u_*+g_ff*(Vhat_T-V_*)+K*etahat, where g_ff is the trim-input derivative
with respect to speed. The sampled trim derivative satisfies

    A*e_4+B*g_ff=e_4+h*e_1.

Thus the unknown true constant speed cancels from relative dynamics. With
ego measurement-error generator G_E, target position errors n_p and speed
error n_V, the disturbance is

    B*(K*G_E*xi - K(:,1:2)*n_p + (g_ff-K(:,4))*n_V).

The invariant-set construction covers all nine bounded measurement coordinates
with a fixed relative tube despite growing absolute target position diameter.
For zero rectangle offsets and a straight target, exact rectangle projection
gives clearance lower bound

    separation - r_px - vehicleLength - vehicleWidth/2*r_yaw.

This certifies geometry for every trajectory of this declared tangent model
in the joint tube; it does not certify nonlinear dynamics or arbitrary target
acceleration/curvature. The safety-relevant relative tube can be invariant
even when uncontrolled absolute-position marginals grow.

## Shift policies to prove recursive feasibility

Augment the information state with true ego/target state, observer internal
and bound state, previous input and required mode/history. Its transition
includes plant defects, measurement noise, observer update and policy input.
Let T_pi be the exact image under the same causal policy and uncertainty
contract. A certified plan consists of policies, sets R_0,...,R_M, hard-row
certificates, prefix slacks and a terminal witness, with I_k=R_0 and (M5).
Condition the old first successor on the valid new observer enclosure and
the actual known observer/measurement/input memory:

    I_(k+1)=R_1 intersect Q_(k+1) intersect knownMemory_(k+1),
    true q_(k+1) in I_(k+1) subset R_1.                         (6)

Both sets contain the same true augmented state; intersection is therefore
nonempty. At initialization use (M2). If no valid observer enclosure is
available, Q is the unrestrictive set, not an invalid finite ball. Reference
memory of the shifted policy must remain an allowed candidate. An absorbing
encounter-ended flag is propagated with the same convention on both frames.

Hard rows and collision deficits are universal over joint images, including
policy-generated inputs and declared held-input constraint instants. For each
J in a downward-closed terminal family B, there is a causal rule pi_f and
a certified outer successor R_f(J) in B such that T_(pi_f)(J) subset R_f(J),
all hard rows hold on J under that rule, and its collision deficit is zero.

A concrete admissible policy class can contain scheduled affine output laws
u_i=v_i+K_i*(publishedState_i-z_i) together with the held terminal rule.
Nominal centers/reference memory and stored gains remain free to inherit the
old tail; z_0 is not forced to equal the new estimate in the carried candidate.
The initial physical set still uses the current observer information.
Certificates are evaluated on physical joint sets, irrespective of which
nominal center is used to represent them. This class is closed under dropping
the first rule and appending the allowed terminal rule. A narrower policy
class requires its own closure argument.

**Certificate-carrying theorem.** Suppose a feasible certified plan exists
with 1<=N<=M<=M_max; the observer enclosures obey the same uncertainty
contract; the policy/certificate class permits shifting and terminal
appending; and terminal B has the properties just stated. Execute the
definite optimized current input without changing it after the solve.
Then the next mathematical problem is feasible for every admitted parameter,
disturbance and sensor history.

**Proof.** Shift pi_1,...,pi_(M-1) and append a witness pi_f for R_M. Set
R'_0=I_(k+1), retain R'_i=R_(i+1) for 1<=i<=M-1, and take R'_M to be the
certified terminal successor of R_M. For M>=2 the first new image satisfies

    T_(pi_1)(I_(k+1)) subset T_(pi_1)(R_1) subset R_2.

All subsequent old transitions and row certificates are unchanged. The
last hold starts in R_M; its terminal witness certifies the last rows and
R'_M in B. If M=1, the new root is a subset of R_M, and the same terminal
witness applies by downward closure. Input memory in q includes the applied
input, so the shifted and appended slew rows are included. The new plan has
the same M and respects both horizon limits.

Let old prefix slacks be xi_0,...,xi_(N-1). Use xi'_i=xi_(i+1) for
0<=i<N-1 and xi'_(N-1)=0. That last stage was either an old hard-tail stage
or the appended terminal stage. Its candidate budget is therefore the old
budget minus xi_0. If the executed plan is a primary minimizer, comparison
with the next minimum proves V*(I_(k+1))<=V*(I_k)-xi_0*. Feasibility itself
requires only a feasible executed plan. Zero budget implies the specified
collision clearances; positive slacks do not give a zero-deficit safety
claim. QED

This proof retains valid outer sets, rather than demanding that a freshly
computed adaptive approximation be a subset of the previous approximation.
Any refinement can intersect sound images of the same shifted rule with the
retained old tube; the intersection remains sound. A numerical optimization
failure is outside this existence statement.

For a convex implementation the feasible representation must retain this
carried candidate and its valid separating/geometry/domain certificates.
Replacing them by arbitrary new tangents, gain changes or trust regions can
remove the candidate even though the nonlinear robust problem is feasible.
The current production point-estimate programs do not yet implement this
certificate-carrying formulation.

## The second CLF stage preserves mathematical feasibility

Suppose the primary minimum is attained in the certified formulation above.
The second stage retains the same hard rows and certificates, caps the
primary slack sum at that minimum, and imposes a CLF residual g<=rho_CLF with
rho_CLF>=0 and no finite upper bound. For any primary minimizer whose CLF
residual is finite on its certified set, choose

    rho_CLF=max(0,sup g).

This is a feasible second-stage point: an unbounded soft CLF row cannot remove
that primary candidate. A point/nominal CLF residual is the scalar special
case. A hard CLF row, a bounded CLF slack or a new restrictive trust region
needs an additional feasibility proof. Attainment of a secondary optimum is
separate from this existence statement.

Executing a feasible secondary point preserves recursive feasibility by the
theorem. Optimal primary-value descent additionally requires the exact
primary cap; a relaxed cap with numerical tolerance only yields the
corresponding cost-error inequality. Neither conclusion requires a solver
to successfully return a point in every experiment.

## Issue the optimized current input directly

Optimize u_0 and include u_0=v_0+K_0*(xhat_k-z_0) as a program equality. Future
pi_i are policy parameters, rather than promises to execute nominal v_i
unchanged after observations arrive. The completed current u_0 is issued
unchanged. Keep/free the nominal center consistently so the shifted policy
remains an allowed candidate. This requires no post-solve scaling, clipping
or constraint veto.

Outstanding runtime work: consume current/predicted observer enclosures;
certify scheduled full-state/relative feedback and finite-horizon tubes; add policy
and input-margin constraints; retain target/observer branch correlations;
establish robust predicted-endpoint terminal membership; preserve shifted witnesses.
The terminal continuation is a feasibility witness, not a command to stop
nominal driving when the current state belongs to the terminal family.
The research verifier
demonstrates mechanisms, not completion of this runtime.

Independent exact-arithmetic specializations of (M3)--(M6), persistent
parameter supports, posterior/certificate reuse and a common uncertain-rate
braking law are checked by scripts/checkJointMismatchDerivations.py and
tests/jointMismatchDerivationsTest.py. They check the stated algebra and
specializations, not a quantified full-domain NRMM/Fiala certificate.

## Implementation design: finite feedback prediction and terminal certificates

October 14, 2026; terminal semantics corrected October 15, 2026. This section
specifies a proposed implementation; it does not change the runtime's
uncertainty propagation. The predicted endpoint information set belongs to a
robust terminal family with a common admissible safe continuation. That
continuation supplies the shifted feasibility witness; it need not be
executed by the optimized closed-loop trajectory. Current-state membership
does not end the driving task or require terminal braking. Normal cruising
may belong to this family while the controller continues to optimize for
lane keeping and cruise control. No condition excludes nominal states from
the terminal family, and no terminal-set attraction objective is introduced.

### State and interface contract

Use a joint deviation state containing physical ego/target deviations,
observer persistent states, persistent uncertain parameters, and all memory
needed by the sampled maps. The actual NRMM predictor resets, bound updates,
angle wrapping, held-input memory and measurement availability belong to
that map. Four 12.5-ms sensor updates occur within the default 50-ms control
hold. Sampling order must match `onlineNrmmTrackingRuntime` and its adapter;
replacing these updates by one generic correction gain is only a surrogate.

A current six-dimensional error radius or generator is insufficient to
reconstruct this map. An integration packet needs the known observer state,
its gain/model version, relevant predictor and bound memory, sensor-noise and
dropout contracts, persistent parameter set and allowed drift, and the joint
root generators. Exact software-state history need not be copied when a
proved sufficient state or sound enclosure exists. A proposed packet must
identify which of these two meanings it carries.

One convenient reduced derivation uses r=x-z and e=x-xhat. With
u=v+K(r-e), the root generator must use the *same* columns for r and e:

    [r_0; e_0] = [G_0; G_0] xi,   |xi| <= 1,
    K(r_0-e_0) = 0.

Here z_0=xhat_0; known offsets can be included explicitly. Therefore every
root branch receives the same optimized current input. Independently boxing
r_0 and e_0 would introduce fictitious uncertainty in that definite input.
Subsequent branches receive different causal feedback inputs after their
own measurements, exactly as intended by the policy prediction.

For illustration only, consider a linear plant and a predict/correct observer
with the same nominal model, output matrix C and correction gain L. With a
constant parameter deviation theta and residual process error w,

    r_next = (A+B K) r - B K e + E theta + w,
    e_next = (I-L C) A e + (I-L C)(E theta+w) - L nu,
    theta_next = theta.

The same theta and w occur in both equations. They are not two independent
physical mismatches. The actual NRMM joint map must be derived from its own
updates, including measured acceleration and gyro feedthrough; these
illustrative observer equations cannot be substituted for it.

### Correlation and mismatch accounting

Linearize the *joint* sampled map along the planned maneuver, obtaining

    q_next = m_i + M_i q + N_i nu + D_i d + R_i(q,nu,d).

A fixed parameter is a state block with identity transition. Its generator
is introduced once at the root; a declared parameter drift introduces only
the allowed increment. Residual process disturbances and newly sampled
measurement noise introduce new columns at their actual sample times. Past
mismatch is already represented in the current observer enclosure. Future
mismatch changes future physical motion and observer residuals and must
still enter the future joint map.

For a sound remainder enclosure Rset and independent new primitive boxes,

    G_next = [M_i G_i, N_i G_noise, D_i G_residual, G_remainder].

Shared sources use one common column before these operations. Take absolute
values only when evaluating supports of constraint directions. Do not first
replace G_i by a diagonal box or inject a fresh arbitrary observer-error box
at every hold. Either operation discards correlations that the joint map was
introduced to retain. Full histories can be compressed with a sound outer
approximation, while protecting the common pose and persistent-parameter
columns. An intersection with an independently valid observer enclosure can
reduce a set; the intersection must actually be represented or soundly
outer-approximated. Taking the smaller of unrelated coordinate radii is not a
general joint-set intersection algorithm.

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

### Concrete velocity-coordinate construction during braking

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

### Constraints, nonlinear reserves, and next-frame reuse

Use separate physical-state, estimate, relative-pose, input and hold-image
projections of the same joint set. Rear adhesion, front tire domain, actuator
bounds and slew constraints must include the feedback input projection,
not only physical pose inflation. Target model uncertainty remains in the
relative map even though ego steering cannot control the target. Future
radar loss follows its declared availability contract, rather than silently
assuming that every future measurement is informative.

For the nonlinear remainder, bound derivatives or the direct map on the
entire admitted state/input/parameter/noise region, accounting for every
reachable model branch. A finite-difference Jacobian and a collection of
nonlinear replays are useful diagnostics but provide no uniform remainder
certificate. Offline interval derivative tables or validated local map
bounds are plausible implementations. If an admitted tube leaves their
validated cells, the computation needs an enclosing branch or must report
that no certified tube is available. It must not reduce the radius to stay
inside a table.

Next-frame feasibility uses posterior conditioning of the old joint policy
and its carried sets. Retain a previous certified witness when a fresh
approximate propagation is wider. A planner's new reference and newly
scheduled gain do not automatically preserve that witness: the admissible
policy/set parameterization must still include it. The terminal condition
is a common causal continuation for every branch of the endpoint information
set; individual pointwise stopping trajectories are insufficient. This
requirement concerns the predicted-endpoint feasibility certificate. Actual
MPC operation continues on subsequent measurements, including when the
current information set already belongs to the terminal family.

### Bounded online computation and validation gates

For the real-time design, schedule gains and validated map/remainder bounds
offline. Propagate support directions or a bounded generator representation
online, alongside the one RTI model. Do not run dozens of finite-difference
observer replays inside each control frame. Preserve the known-current-input
root map and the subsequent feedback maps as distinct operations.

The next implementation gates are: (1) an observer prediction packet and
maneuver-dependent joint local-map prototype with persistent mismatch;
(2) validated nonlinear outer images and feedback input margins over the
admitted maneuver cells; (3) terminal information-set membership and carried
policy/set witnesses across frames. Local map agreement, support attainment
and nonlinear sampled replays should be reported separately from uniform
coverage. None of these gates requires a constant-width invariant tube or
monotonic shrinkage at every node. Their objective is finite-horizon
closed-loop prediction with honest uncertainty accounting and manageable
supports, while retaining the actual noise floor.

This implementation direction is consistent with the distinction between
current estimation error, future estimation error and estimated-to-nominal
prediction error in Koehler, Mueller and Allgoewer, *Robust output feedback
model predictive control using online estimation bounds*, Sections III--IV
([author manuscript](https://www.ist.uni-stuttgart.de/dokumente/public/Output.pdf)).
That paper provides related sufficient constructions; it does not establish
the required NRMM/Fiala bounds or numerical timing for this repository.
