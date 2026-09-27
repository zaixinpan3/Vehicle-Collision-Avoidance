# Nonlinear avoidance and free-progress recovery design

> Historical record: the experimental nonlinear shooting controller and its dedicated execution/audit scripts were removed on 2026-09-26 at the user's request. Commands and implementation descriptions below refer to the recorded experiment, not the current controller. See [removal decision](NONLINEAR_SHOOTING_REMOVAL_20260926.md).

Date: September 26, 2026. Status: implemented opt-in research controller; experimental outcomes are reported separately in [NONLINEAR_RECOVERY_EXPERIMENT_20260926.md](NONLINEAR_RECOVERY_EXPERIMENT_20260926.md). This design does not establish robust nonlinear safety, recursive feasibility, terminal invariance, or real-time execution.

## Decision and failure mechanism

The experimental branch optimizes the physical held steering angle and signed longitudinal tire utilization through the nonlinear bicycle rollout. It checks actual predicted rectangle geometry and recovers the path-following trim at the predicted terminal station. It therefore removes both unrestricted affine tire-force extrapolation and the absolute cruise-time station requirement from this branch.

The [September 25 solve audit](SOLVE_FAILURE_AUDIT_20260925.md) identified two separate defects. First, previously accepted affine plans predicted front lateral forces of approximately -41.790 kN and +27.272 kN where the corresponding nonlinear capacities were 2.492 kN and 1.409 kN. The straight example also reversed the force sign. Second, the S-curve actuator, station-corridor, and terminal constraints were mutually inconsistent even without obstacles or road rows. Increasing objective weights or solver iterations cannot repair either mathematical conflict.

The implementation is in `controller/solveNonlinearAvoidancePlan.m`, with domain-valid initial guesses from `controller/nonlinearAvoidanceSeed.m` and an independent numerical domain replay from `controller/nonlinearAvoidanceRollout.m`. `collisionAvoidanceController` selects it only when `cfg.solver.method="nonlinearShooting"`; the default remains `"affineSocp"`.

## Physical-input single shooting

Let

\[
 x=(s,d,e_\psi,v_x,v_y,r),\qquad u=(\delta_f,\beta).
\]

Here beta is signed longitudinal tire-force utilization, with axle longitudinal forces `Fx_i = beta*mu_i*Fz_i`. It is not a brake-pedal percentage. The experiment retains the current nonlinear modified-Fiala law, front-force rotation, passive road load, Frenet kinematics, and the supplied longitudinal acceleration bias.

For a held input, denote the implemented nonlinear RK4 map by `Phi_h`. The input sequence determines every state:

\[
 x_{k+1}=\Phi_h(x_k,u_k),\qquad x_0=\hat x.
\]

There are no independent optimized states that can satisfy a fictitious affine trajectory while the nonlinear rollout goes elsewhere. With move blocking, `u = E z`, where E repeats each decision pair for the specified number of command holds. Blocking reduces optimization dimension and restricts the candidate set; it can reduce feasibility and is recorded in each experiment configuration.

`ltvBicycleModel.nominalRollout` supplies centered finite-difference Jacobians of its discrete RK4 map. The optimizer propagates shooting sensitivities through

\[
 S_{k+1}=A_k S_k+B_k E_k,\qquad S_0=0.
\]

These are numerical derivatives of the sampled nonlinear map, not a proof of an exact continuous-time flow derivative. For varying reference curvature, the flow evaluates curvature at the progressing station inside integration. The optimizer uses MATLAB `fmincon` with SQP and supplied objective/constraint gradients; only the fitted-road pose derivative and the terminal trim's curvature derivative use small local finite differences in the constraint assembly.

The audit splits each command hold into `auditSubsteps` pieces. Each piece uses the existing RK4 integration step no greater than 0.01 s. Thus 50 ms holds with two audit subdivisions are checked every 25 ms, and each 25 ms map uses three RK4 subdivisions. The independent domain replay uses the same mesh. Original command slew limits continue to use the 50 ms command period through `commandSampleTime`.

## Objective and free terminal station

The running objective penalizes lateral path error, velocity-course error, longitudinal speed error, and yaw-rate error:

\[
 e_k=\begin{bmatrix}
 d_k\\
 e_{\psi,k}+\operatorname{atan2}(v_{y,k},v_{x,k})\\
 v_{x,k}-v_{\mathrm{ref}}\\
 r_k-\kappa(s_k)\sqrt{v_{x,k}^2+v_{y,k}^2}
 \end{bmatrix},
 \qquad Q=\operatorname{diag}(1,12,1,0.5).
\]

The accumulated running cost is multiplied by audit sample time. A nonnegative input-change penalty uses the measured/previous held input for its first difference. These terms shape performance; they do not relax physical constraints.

At the actual predicted terminal station `s_N`, the code solves the nonlinear constant-curvature cruise equilibrium for `kappa(s_N)` and the requested forward speed. Define

\[
 e_N=x_N(2{:}6)-x_{\mathrm{trim}}\bigl(\kappa(s_N),v_{\mathrm{ref}}\bigr)(2{:}6).
\]

The hard recovery requirement is componentwise `abs(e_N) <= terminalTolerance`. A weighted terminal cost also penalizes these errors. Station is deliberately absent from this recovery vector: braking or taking a longer avoidance path does not require the vehicle to catch up to an artificial absolute clock.

This is a terminal recovery condition, not an invariant terminal set. On varying curvature, a local constant-curvature trim is only a local recovery reference, and its continuation must still be recomputed as the vehicle progresses. The previous scheduled affine terminal proof is not inherited. No recursive-feasibility claim is made for the nonlinear branch.

## Hard finite-horizon conditions and independent acceptance

The nonlinear optimization enforces the following conditions at its audit nodes, within the declared numerical constraint tolerance:

- Physical steering/utilization bounds and command-to-command slew bounds.
- Forward-speed, lateral-position, heading-error, lateral-velocity and yaw-rate domains specified by the model configuration, together with a regular Frenet-chart reserve `1-kappa*d >= 0.1`.
- Front and rear tire-slip limits from `cfg.model.slipAngleMaximum`, bounded away from the invalid `abs(alpha)=pi/2` tire domain. Both sides of command switches are checked.
- Oriented ego/target rectangle separation, including the configured physical clearance.
- Declared road containment and terminal recovery tolerances.

Collision constraints use the separating axes of both actual predicted rectangles. A positive separating-axis gap supplies a sufficient physical separation margin. Its pose derivative includes the derivative of ego-fixed axes as the vehicle yaws; target-axis derivatives are zero. Future target position radii enter through their directional box support. Future target yaw uncertainty is enclosed by the circumradius displacement bound `R*min(2,yawRadius)`. These reserves include the outward arithmetic radii produced by `targetPrediction.finiteFlow`.

For a finite fitted quadratic road boundary, the implementation clips the ego rectangle to the boundary's longitudinal validity interval when its coverage policy permits partial coverage. It minimizes the signed quadratic boundary value over polygon edges, checking edge endpoints and all interior edge stationary points. Because the boundary function is linear in its lateral coordinate, a minimum over the polygon occurs on its boundary. Strict coverage policies additionally require the full rectangle to remain within the declared parameter interval. Boundary normal-distance uncertainty is retained through the maximum graph-slope allowance.

The global path-offset road clearance receives a conservative footprint curvature allowance. For vehicle circumradius R, curvature bound `kMax`, and lateral chart limit `dMax`, the code requires positive

\[
 q=1-k_{\max}(d_{\max}+R)
\]

and adds `kMax*R^2/(2*q)` to the tangent-frame rectangle lateral support. This comes from the Hessian bound of signed distance in a regular normal chart. Its use presumes the declared continued reference and its normal chart describe the selected road corridor; it is not a global guarantee for a self-intersecting route or an unobserved road branch.

A candidate becomes a returned executable plan only after independent reevaluation of the nonlinear constraints and original input/slew bounds. `nonlinearAvoidanceRollout` additionally checks every RK4 intermediate state and endpoint for finite values, valid tire slip, physical utilization, and regular Frenet coordinates; it never clips an invalid state or tire force. The accepted replay must agree with the optimization rollout within 1e-8 in maximum absolute state difference.

The solver retains the best verified feasible optimizer iterate, so an interrupted refinement can retain a previously verified candidate. A generated seed can also be retained only after the same complete nonlinear gate. A no-target seed with a numerically zero nonnegative objective may return without an optimization call. Neither seed generation, an optimizer success flag, nor a merit decrease alone establishes acceptance.

If all attempts fail, no plan is returned. Diagnostics retain the best finite infeasible candidate, the worst constraint family, separation/road/domain minima, terminal errors, optimizer outcomes, and elapsed time. This makes a failed experiment inspectable without softening physical constraints.

## Initial guesses and effort allocation

Initial guesses combine the shifted previous plan with nonlinear feedback-generated passing maneuvers on both sides. The feedback guesses obey actuator limits while being integrated through the nonlinear model and audited for model-domain validity. A Gaussian passing reference is an initialization preference and has no execution authority.

For a fresh encounter, the passing sides are ordered by the lateral displacement needed at the predicted closest longitudinal approach. A shifted plan remains first when its nonlinear sampled collision margin is near feasible. A clearly colliding shifted cruise guess moves behind the passing guesses. Exact duplicate input sequences are removed. Move-block and derivative-domain projections affect only initial guesses; every candidate must be rerolled and admitted independently.

Once a seed or accepted SQP iterate passes the complete nonlinear gate, at most `feasibleRefinementIterations` further SQP iterations refine its cost. The default is four; a verified feasible seed starts this count at iteration zero. The best verified incumbent is retained. If no feasible candidate exists, this refinement limit does not shorten the ordinary iteration or wall-time budget. The attempt diagnostics record the first feasible iteration and actual refinement count. This trades additional objective improvement for computation without weakening the acceptance constraints.

Each remaining seed receives an equal share of the remaining wall-time budget. This avoids allowing the first infeasible side to consume the entire search period. It does not ensure that a feasible side will be discovered before the deadline.

## Configuration and experimental scope

The repository configuration defaults and the standalone solver defaults differ only where the controller wrapper specifies the horizon/blocking policy:

| Quantity | Controller configuration default | Meaning |
| --- | ---: | --- |
| Method | `affineSocp` | Nonlinear branch remains opt-in |
| Nonlinear horizon | 6.0 s | Wrapper converts duration to command holds |
| Move block | 2 holds | Standalone solver defaults to 1 when no options are supplied |
| Maximum SQP iterations per seed | 100 | Wall-time allocation can stop earlier |
| Iterations after first verified feasible candidate | 4 | Objective refinement limit; infeasible search retains its ordinary budget |
| Total search budget | 30 s | Research computation limit, not a 50 ms real-time guarantee |
| Audit subdivisions | 2 per hold | Node checks, not a whole-hold enclosure |
| Terminal tolerances | `[0.25 m; 0.04 rad; 0.35 m/s; 0.25 m/s; 0.08 rad/s]` | `[d,ePsi,vx,vy,r]` relative to terminal trim |
| Nonlinear constraint tolerance | `1e-5` | Maximum accepted assembled residual |
| Utilization derivative reserve | `1e-4` | Search bounds intersect physical beta bounds with `[-0.9999,0.9999]` |
| Maximum seeds | 3 | After validity filtering and exact deduplication |
| Terminal cost weight | 50 | Error scaled by the declared terminal tolerances |
| Input-change weights | `[0.1;0.01]` | Steering/utilization changes |

The first development experiments use explicit overrides including a 5 s horizon and four-hold move blocks. Their complete configurations, deadlines, revision timing, and results belong in the experiment report; these overrides are not new physical assumptions or proof conditions.

The beta endpoint reserve is explicit because the original beta-coordinate Fiala derivative is undefined at exactly +/-1. It reduces numerical search authority by a small amount and is reported in diagnostics. It must not be described as a new physical friction law.

Published nonzero ego-state uncertainty, target-state uncertainty, or model-residual rate bounds are rejected explicitly by this experimental branch. They are never replaced by zeros. Existing future target-flow reserves are retained even with exact initial observations. Estimator-fed cases therefore require a separate robust nonlinear formulation before this branch can admit their nonzero state bounds.

Acceptance covers a nominal nonlinear discrete trajectory at the audited spacing and numerical model-domain checks. It does not cover all continuous-time points, all uncertain plants, or the future beyond its finite horizon. Dense simulation, integration refinement, and positive observed clearances are useful experimental evidence but do not replace validated reachability tubes. The branch reports robust certification, whole-hold certification, terminal invariance and recursive feasibility as false.

## Why simpler alternatives are insufficient

### A friction cone intersects a saturated tire tangent at one point

With axle capacity M, physical forces satisfy

\[
 \left\|\begin{bmatrix}\beta\\F_y/M\end{bmatrix}\right\|_2\le1.
\]

Adding this cone to the existing affine force prediction would exclude the observed excessive forces. It would not establish that the commanded steering produces that force.

There is also an exact obstruction on a saturated Fiala branch. Let `eta=sqrt(1-betaBar^2)`, `s=sign(FyBar)`, and `Delta=beta-betaBar`. Its normalized force tangent is

\[
 a(\Delta)=\begin{bmatrix}\bar\beta\\s\eta\end{bmatrix}
       +\begin{bmatrix}1\\-s\bar\beta/\eta\end{bmatrix}\Delta.
\]

Direct expansion gives

\[
 \|a(\Delta)\|_2^2=1+\Delta^2/\eta^2.
\]

Consequently, the disk permits only `Delta=0`. Tightening the disk radius below one excludes the saturated anchor itself. Because the saturated slip derivative also vanishes, this construction freezes longitudinal utilization while its local force model cannot represent a steering reversal. The cone is physically meaningful, but a cone-only patch can make the convex program infeasible or poorly conditioned. This finding motivates the chosen nonlinear shooting route.

### Utilization-angle coordinates regularize one singularity

Set `beta=sin(theta)`. On a fixed saturated slip-sign branch,

\[
 F_y=-\operatorname{sign}(\alpha)M\cos\theta,
 \qquad |F_y''(\theta)|\le M.
\]

The first-order force remainder is therefore bounded by `M*DeltaTheta^2/2`. For the adhesion branch, with `q=C*abs(tan(alpha))/(3*M*cos(theta))<1`, direct differentiation gives

\[
 \frac{\partial F_y}{\partial\theta}
 =\operatorname{sign}(\alpha)M\sin\theta(3q^2-2q^3),
\]

whose magnitude is at most M and which matches the saturated derivative at `q=1`. Direct formulas avoid multiplying a divergent beta derivative by a vanishing cosine.

This is an attractive future conditioning improvement. It does not provide a global bound across the full Fiala corner near `abs(theta)=pi/2, alpha=0`, fix slip-sign reversal, or make tire forces globally affine. Longitudinal dynamics, command mapping, slew limits, feedback gains, and warm starts would all need consistent transformation. Simultaneously taking affine tangents of sin and cos retains the same tangent-line/disk obstruction.

### Virtual force inputs restore force authority but require a different actuator map

With front lateral force f as a virtual input, let `L=M*sqrt(1-beta^2)` and `r=abs(f)/L<1`. The exact interior inverse Fiala map is

\[
 \alpha=-\operatorname{sign}(f)\arctan\left[
 \frac{3L}{C}\{1-(1-r)^{1/3}\}\right],
 \qquad
 \delta=\operatorname{atan2}(v_y+l_f r_{\mathrm{yaw}},v_x)-\alpha.
\]

Its force sensitivity contains `(1-r)^(-2/3)/C`, so inverse conditioning deteriorates near saturation. A force reserve `r<=1-epsilon^3` bounds inverse sensitivity by `1/(C*epsilon^2)`. Rear force remains state dependent on this front-steered vehicle, and steering/rate limits and front-force rotation remain nonlinear. This is a promising later architecture, not an exact-convexity shortcut for the current model.

### Successive convexification remains a future numerical alternative

Sampled-map linearization, scaled trust regions, actual/predicted merit comparisons, and a retained nonlinear-feasible incumbent can replace the current dense SQP backend. Virtual controls or geometric slack may assist search but must vanish, or be removed and followed by a hard solve, before execution. An infeasible initial trajectory and a small trust region can contain no valid maneuver, so multiple physically plausible initial guesses remain necessary.

The literature's convergence statements require their stated regularity, penalty, initialization, and terminal assumptions. Finite-budget convergence to a local stationary point does not imply a globally optimal maneuver or a safe command. Robust nonlinear safety additionally requires bounded approximation errors propagated with disturbances inside enforced model domains.

## Verification performed for this design note

An independent Python/NumPy calculation checked the analytic SAT pose-gradient formula against centered finite differences on 2,000 random rectangle configurations, using seed 260926. Its maximum absolute component error was `2.1189430210455384e-9`. This checked the mirrored mathematical formula, not the complete MATLAB shooting Jacobian or a closed-loop vehicle run.

A separate deterministic grid checked the saturated tangent/disk identity at 151 anchor beta values in `[-0.9999,0.9999]` and 31 beta increments in `[-0.1,0.1]`: 4,681 pairs, with maximum absolute identity residual `1.4210854715202004e-14`. Algebra establishes the identity; finite sampling only checks numerical evaluation.

Behavior tests in `tests/nonlinearAvoidancePlanTest.m` cover returned nonlinear rollout consistency, explicit ego/target uncertainty rejection, free terminal station, rejection of initially overlapping rectangles, and hard terminal recovery. Seed/domain tests are separate. Actual commands, check outcomes, simulation results, and any subsequent changes are listed in the experiment report.

## Verified primary sources and their limits

1. Zhang, Thornton and Gerdes, *Tire Modeling to Enable Model Predictive Control of Automated Vehicles From Standstill to the Limits of Handling*, AVEC 2018, Sections 2 and 4. The paper distinguishes friction capacity from its affine tire approximation; its convex lateral controller assumes separately supplied longitudinal information. That assumption does not match the old simultaneous beta optimization. [Author-hosted paper](https://ddl.stanford.edu/sites/g/files/sbiybj25996/files/media/file/zhang_2018_avec_0.pdf).
2. Stephen M. Erlien, *Shared Vehicle Control Using Safe Driving Envelopes for Obstacle Avoidance and Stability*, Stanford PhD thesis, 2015, Chapter 5, Eqs. (5.15)-(5.17). It provides precedent for front-force optimization, a coupled-force constraint, and inverse tire mapping. Its environmental-envelope treatment and model assumptions differ from this controller. [Author-hosted thesis](https://ddl.stanford.edu/sites/g/files/sbiybj25996/files/media/file/thesis_erlien_0.pdf).
3. Yuanqi Mao, Michael Szmuk, Xiangru Xu and Behcet Acikmese, *Successive Convexification: A Superlinearly Convergent Algorithm for Non-convex Optimal Control Problems*, arXiv:1804.06539v2, 2019, Section 2 and Algorithm 2.1. Virtual controls, penalty terms, and trust regions address artificial infeasibility and inaccurate local models. Original feasibility requires zero virtual violations; its theorem is not automatically a theorem for this repository. [Primary preprint](https://arxiv.org/pdf/1804.06539v2).
4. Yana Lishkova and Mark Cannon, *A successive convexification approach for robust receding horizon control*, arXiv:2302.07744v3, 2025, Sections II-V. Its guarantees rely on explicit nonlinear-error tubes and suitable seed/terminal assumptions. These distinguish robust nonlinear certification from nominal rollout acceptance. [Primary preprint](https://arxiv.org/pdf/2302.07744v3).
5. Alexander Liniger, Alexander Domahidi and Manfred Morari, *Optimization-Based Autonomous Racing of 1:43 Scale RC Cars*, arXiv:1711.07300, Section III-B, Eqs. (12)-(14). Progress is an additional decision-related state, coupled to the vehicle through contouring and lag errors. This supports separating path recovery from a prescribed time schedule; the racing objective and constraint softening are not adopted here. [Primary preprint](https://arxiv.org/pdf/1711.07300).

All five sources were opened and checked directly for this targeted comparison. This is a scoped engineering synthesis, not an exhaustive literature review. The coordinate and inverse mappings, shooting approach, and free-progress idea are not claimed as novel. The tangent/disk obstruction and their connection to the recorded controller failures are direct mathematical and code analysis in this work.
