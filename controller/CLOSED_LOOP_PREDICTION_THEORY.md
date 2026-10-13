# Feedback-aware prediction and invariant uncertainty tubes

Date: October 12, 2026. Design/theory, not the current runtime formulation.
The current theoretical scope omits road boundaries. Observer, model, input,
handling and collision premises remain. Optimized current inputs are issued
directly; feedback belongs in the optimization's policy and prediction.

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
small initial width is therefore not promised. A fixed invariant width meets
the nonexpanding prediction requirement. Held-input midpoints need their own
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

This is a linear explanatory derivation, not our NRMM observer equation.
For the nonlinear implementation derive the joint map/Jacobian or certified
comparison dynamics from the actual point flow, reconstruction and bound
updates. Validate a joint invariant/contraction condition over the admitted
nominal, error and input domains, including midpoints and mode transitions.
Stable observer ISS and the existing path CLF do not by themselves certify
incremental joint contraction. The uncontrolled target's absolute-position
spread remains; relative/coupled safety coordinates are still needed.

Including the updater enables a tighter causal description; it does not
automatically guarantee a smaller tube. A poor outer approximation can still
discard its correlations. The full updater can be used for offline proof and
coefficient construction, with compact certified radius/generator recursions
online. Observer states need not all become optimization decision variables.
The shifted-policy proof below applies to this augmented transition when its
outer images are sound, monotone and retain the same causal-policy witness.

## Construct a fixed invariant set for the sampled tangent model

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

At admissible straight cruise above the blend/stop regime, sampled Fiala F
has continuous first derivatives locally. Stable A_c admits a contracting
metric. Continuity gives a sufficiently small neighborhood with contractive
eta derivative in (1) and bounded epsilon derivative. The mean-value argument
then gives (3) for compatible sufficiently small observer errors. This proves
local existence, not a quantified tube for the current NRMM bounds. Compute
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
Let T#_pi(J) be a sound monotone outer image under the same causal policy for
all admitted observation histories. Conditioning on the actual new observation
gives

    I_(k+1) subset T#_(pi_0)(I_k).                           (6)

Hard rows and collision deficits are universal over joint images, including
policy-generated inputs and declared held-input constraint instants. A
downward-closed robust terminal family B admits a causal policy pi_f with all
hard rows, zero collision deficit and T#_(pi_f)(J) in B for J in B.

Given feasible pi_0,...,pi_(M-1), execute its definite optimized input pi_0(o_k).
At the next sample shift pi_1,...,pi_(M-1) and append pi_f. Monotonicity and (6)
make every new joint image a subset of its old image at the same absolute time.
Shifted rows remain valid; terminal closure certifies the appended stage and
endpoint. The prefix slack shift gives the old budget minus its first deficit.
Feasibility follows for any feasible executed policy; optimal-value descent
additionally needs a primary minimizer. The allowed policy class and each new
convex restriction must retain this witness. Arbitrary relinearizations, trust
regions, separating planes or gain changes do not establish that property.

## Issue the optimized current input directly

Optimize u_0 and include u_0=v_0+K_0*(xhat_k-z_0) as a program equality. Future
pi_i are policy parameters, rather than promises to execute nominal v_i
unchanged after observations arrive. The completed current u_0 is issued
unchanged. Keep/free the nominal center consistently so the shifted policy
remains an allowed candidate. This requires no post-solve scaling, clipping
or constraint veto.

Outstanding runtime work: consume current/predicted observer enclosures;
certify scheduled full-state/relative feedback and invariant tubes; add policy
and input-margin constraints; retain target/observer branch correlations;
certify braking/parked modes; preserve shifted witnesses. The research verifier
demonstrates mechanisms, not completion of this runtime.
