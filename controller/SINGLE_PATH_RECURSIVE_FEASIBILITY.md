# Exact two-vehicle recursive feasibility

Implemented September 11, 2026, certificate version 17. The user explicitly
confirmed that the ego follows the controller's prediction dynamics exactly
for this study. This replaces the finite-perception-encounter requirement.

## Plant and scope of the guarantee

There are exactly two vehicles. Their full states are available at every
scheduled sample. The same target is present throughout the study and follows
one immutable Cartesian constant-acceleration, constant-yaw-rate trajectory.
The public controller does not read perception range, visibility or detection
completeness. A distant target is never discharged and control never ends
because the target passes the ego.

The **mathematical ego plant is the admitted scheduled affine bicycle**. The
finite approach uses the matrices returned by `ltvBicycleModel.finitePredict`.
At its admitted terminal-entry time it uses the zero-scheduled-speed bicycle
matrices and sampled terminal feedback defined below. This terminal schedule
is an explicit part of the admitted exact plant; it is not a proof that
changing a linearization changes the physics of a nonlinear vehicle.
The stage schedule, terminal generator, road geometry and limits are retained.
An online improvement changes inputs within that schedule, not the dynamics.

The augmented state includes the ego state, original target trajectory and
clock, previous applied input, retained schedule and terminal-entry counter.
Initial ego/target error bounds and ego process-residual bounds must be zero.
Arithmetic enclosures in the finite prediction are retained. The terminal
implementation currently requires zero independent acceleration bias. Passive
road resistance is retained, including the case of zero passive damping.

Under this exact-plant/execution premise, fixed physical geometry and limits,
and one accepted complete initial certificate, the controller supplies a
feasible hard-safe continuation at every subsequent frame. The finite prefix
has certified continuous held-interval road, footprint, state, input and slew
constraints; the infinite suffix has an invariant-set proof. This is a
conditional mathematical-model guarantee. It does not establish nonlinear
Fiala vehicle safety, disturbance robustness or a computation-time deadline.
A failure to find the first certificate is not proof of global infeasibility.

## One admission problem and its successor

Admission finds a finite sequence and an invariant terminal continuation:

\[
 x_{i+1}=A_i^d x_i+B_i^d u_i+c_i^d,\quad
 C_i(x_i,u_i,u_{i-1}),\quad x_N\in X_f.                 \tag{1}
\]

Each `C_i` includes every held-interval hard constraint. Only the CLF performance
rows have unrestricted nonnegative relaxation. `controller.horizonSteps` seeds
the search; candidate length may increase until a complete certificate is
found or the numerical search budget expires. No safe prefix alone is accepted.
The finite search seed uses decreasing scheduled speed to approach the terminal
domain. Its braking anchor stays inside `|beta|<=0.95` to avoid singular Fiala
derivatives at saturation; the optimization retains the configured actuator
limits. A reference used for initialization is not an executable control.

After an accepted input executes, the remaining old inputs and the same
terminal policy form a feasible successor. A replacement solve fixes all
executed inputs and must preserve the original hard constraints and certified
margin. Failure or rejection of the replacement leaves the existing feasible
witness available. No global optimality premise is needed. At terminal entry,
the measured state supplies terminal feedback and a finite preview of that
infinite continuation is returned on every call; a new numerical optimization
is unnecessary to establish existence of a feasible continuation.

The stored QP remains in admission coordinates. Its finite prefix gets shorter,
but its invariant suffix never expires. After prefix exhaustion, its complete
certificate is still retained and the current measured state is checked in the
same terminal set. `inputPlan` then exposes a fresh preview of the analytic
terminal controls. `remainingSteps` counts finite prefix intervals, not the
number of safe commands remaining. `deadline` denotes terminal entry, not
perception exit. `certifiedDuration=Inf` includes the invariant suffix.

Equivalently, a fixed-length feasible preview can always be shifted and
extended by one terminal input. The terminal inputs satisfy all hard rows and
can be assigned finite CLF slacks. Feasibility at an admitted frame therefore
implies feasibility of a complete continuation at every later frame. The first
feasible candidate must satisfy (1), not merely collision constraints over an
arbitrary finite window without a terminal condition.

## Terminal dynamics and invariant set

Write `x=(p,v)` with `p=(s,d,ePsi)` and `v=(vx,vy,r)`. The declared terminal
schedule has no pose-dependent drift:

\[
 \dot p=Gv,\qquad
 \dot v_x=-a v_x+g_\beta\beta,\qquad
 \begin{bmatrix}\dot v_y\\\dot r\end{bmatrix}
 =F_\ell\begin{bmatrix}v_y\\r\end{bmatrix},\qquad a\ge0.      \tag{2}
\]

Steering is zero. At each sample of length `h`, hold

\[
 \beta_k=-k_b v_{x,k}/g_\beta,\quad
 \phi_h=\begin{cases}(1-e^{-ah})/a&a>0,\\h&a=0,\end{cases}
 \quad k_b=\frac{e^{-ah}(1-e^{-h})}{\phi_h}.                 \tag{3}
\]

The additional unit decay rate in `exp(-h)` has units 1/s. This gives
`vx_(k+1)=rho vx_k`, `rho=exp(-(a+1)h)` in `(0,1)`. Within the hold,
`vx(t)=(exp(-at)-k_b phi_t) vx_k` stays nonnegative and nonincreasing, and
`vxDot <= -(a+k_b) vx`. Thus active terminal braking also works when passive
road-load damping is zero. No velocity is rounded or clipped to rest.

Let `C` be the Metzler comparison matrix for the velocity subsystem: its
longitudinal diagonal is `-(a+k_b)` and its lateral/yaw block is the diagonal
of `F_l` with absolute off-diagonal entries. The constructor verifies a
positive vector `q` with `Cq<0` and a nonnegative excursion matrix `M` with

\[
 MC+|G|\le0.                                               \tag{4}
\]

`M` is obtained from `|G|(-C)^(-1)` with a conservative arithmetic reserve.
The numerical residual inequalities are checked. Scale `q` to satisfy speed,
lateral-velocity, yaw-rate, terminal slip, braking-ratio and subsequent
braking-rate limits. The finite program separately constrains the slew from
the last prefix input to the first terminal input, including endpoint
arithmetic uncertainty.

For fixed pose halfspaces `Ap<=b`, the terminal set is

\[
 X_f=\{(p,v):v_x\ge0,\ |v|\le q,\ Ap+|A|M|v|\le b\}.       \tag{5}
\]

The pose rows include the station chart, lateral and heading domains, full
vehicle road containment and all-future target separation. Enumerating the
eight velocity signs makes (5) a finite set of linear hard constraints on the
predicted terminal state. The initial endpoint enclosure must lie in this set.

**Invariance.** Comparison gives `D+|v|<=C|v|`, so `|v|<=q` persists. For every
pose row, the upper Dini derivative of `A_j p+|A_j|M|v|` is at most
`|A_j|(|G|+MC)|v|<=0`. The scalar held-flow formula preserves `vx>=0`.
Terminal input/slip bounds follow from the chosen velocity box. Between
successive terminal commands the braking change is at most
`(k_b/g_beta)(1-rho)q_x`, already included in the box construction. Thus (5)
and every physical terminal obligation persist through every held interval.
The feedback uses the current measured velocity; replaying a rounded nominal
terminal center with open-loop braking is not this construction.

This proves the suffix step. Finite induction through the remaining admitted
prefix, followed by terminal invariance, establishes the required implication
for all subsequent frames. The construction is an instance of the invariant
terminal MPC successor argument described by Rawlings, Mayne and Diehl,
*Model Predictive Control: Theory, Computation, and Design*, second edition,
Sections 2.3 and 2.4.5, available from the
[authors' book page](https://sites.engineering.ucsb.edu/~jbraw/mpc/).
The model-specific comparison and target support arguments here are project
analysis, not a theorem about this repository imported from that source.

## Target safety for all future time

For each terminal separating direction `n`, define the target position after
terminal entry as `p_t(t_N+tau)=p_N+v_N tau+a_t tau^2/2`. Compute

\[
 S_n=\sup_{\tau\ge0}n^T p_t(t_N+\tau).                      \tag{6}
\]

For the scalar quadratic `c+b tau+a tau^2`, this is `c` if `a=0,b<=0` or
`a<0,b<=0`; it is `c-b^2/(4a)` if `a<0,b>0`; otherwise it is infinite.
An infinite support does not admit that terminal halfspace. The implementation
forms upper coefficients directly from the original absolute-time trajectory,
charges arithmetic, and does not round a small positive derivative to zero.
The proposed direction is chosen on the side where future acceleration (or
velocity for zero acceleration) has nonpositive projection. This is a
conservative continuous supporting geometry, not a complete maneuver oracle.

The ego support majorants, physical clearance and target footprint support
are added to (6). A target with nonzero constant yaw rate uses its full rotating
rectangle's circumradius; fixed heading uses its directional rectangle support.
Thus terminal safety covers the target forever, including later approach or
reversal under the exact acceleration law. It does not depend on sensing range
or an assertion that a stopped ego alone is safe. Road rows cover the entire
terminal station/lateral/heading chart, including the full ego footprint.

## Implementation, validation and limits

`hardEncounterBarrier` owns complete admission, invariant rows, retained-prefix
execution and terminal feedback/flow. `formulateAvoidanceProblem` includes the
terminal and entry-slew rows as hard constraints. `targetPrediction.admitExact`
validates the strict motion contract. `readPlanningInputs` ignores perception
metadata, and `perceptionExitBuffer` is removed from controller configuration.
The public controller rejects missing/additional targets, nonzero ego/target
error premises, changed target motion and incompatible execution.

`runExactStateRecursiveFeasibilityScenario` propagates each published continuous
generator independently with matrix exponentials and audits 11 intermediate
samples per interval for rectangle separation and road margins. The September
11 runs used a 0.1 s period, ego speed 8 m/s, 16-step initial search window,
4.8 by 1.9 m rectangles, straight road boundaries at `y=+/-5 m`, and 0.25 m
required clearance. Every optimization after admission was forced to fail.
There is no randomness.

| Target initial position and velocity | Certified commands over 12 s | Terminal-entry step | Minimum additional separation (m) | Minimum additional road margin (m) |
| --- | --- | --- | --- | --- |
| Stationary: `(15,0)`, `(0,0)` | 121/121 | 16 | 2.81566051 | 3.80000000 |
| Oncoming: `(60,0)`, `(-8,0)` | 121/121 | 21 | 0.05674081 | 0.27414418 |
| Crossing: `(15,-4)`, `(0,32)` | 121/121 | 16 | 9.46157516 | 3.80000000 |

Positions are meters and velocities are meters per second. Margins in the table
are above the required 0.25 m clearance. Additional regressions cover finite
actuator slew, zero passive damping, rotating and accelerating target motion,
and optimizer-enabled continuation. These finite tests support implementation
behavior; the infinite claim rests on the conditional proof.

Reproduce from the repository root:

```matlab
addpath('scripts');
for scenario = ["stationary", "oncoming", "crossing"]
    report = runExactStateRecursiveFeasibilityScenario(Scenario=scenario, ...
        SampleCount=120, FailAfterAdmission=true);
    assert(report.passed);
end
results = runtests('tests');
assertSuccess(results);
```

Final validation: all 625 repository tests passed with zero failed or incomplete
cases. Factory Code Analyzer found zero issues in 36 changed MATLAB files. An
additional independent integration/coordinate-roundtrip check on a 100 m radius
arc, initial ego speed 2 m/s and distant stationary target returned 121 certified
commands with post-admission solver failure; this extra check used the reference
curve and no road-boundary constraints.

The core still has 20 source files. Nonlinear Fiala inclusion/feedback research
remains available as separate analysis; it is not a second online controller
mode. The old affine-rest nonlinear counterexample remains valid: the present
proof explicitly assumes the scheduled affine plant, including terminal
schedule (2), instead of transferring that rest set to nonlinear dynamics.
Safety has priority over the cruise objective: admission can command slowing
even near the reference speed, and terminal feedback converges toward rest.
No physical-vehicle, robust-disturbance, global admission-completeness,
reference-speed convergence or worst-case execution-time guarantee is claimed.
