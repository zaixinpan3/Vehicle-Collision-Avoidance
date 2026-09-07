# Closing the NRMM estimator and collision-avoidance control loop

Current scope update (September 7, 2026): the user specifies approximately
constant target curvature and tangential acceleration only during the short
encounter, and stops considering a target after current radar exit. The
infinite-future target occupancy/corridor proposals below are not requirements
of the implemented experiment. The current implementation uses finite-node
prediction and fresh appended-node acceptance; permanent target separation
and indefinite traffic-dependent recursive feasibility are not claimed.
See [TARGET_PREDICTION_CONTRACT.md](TARGET_PREDICTION_CONTRACT.md). The remaining
material records the earlier broader robustness investigation.


Research date: 2026-09-05. Status: proposed architecture and proof obligations,
with reproducible diagnostics; not an implemented robust vehicle controller.

Implementation update (2026-09-07):
[DISSIPATIVE_TERMINAL_CERTIFICATE.md](DISSIPATIVE_TERMINAL_CERTIFICATE.md)
extends the earlier stationary-pose implementation to uncertain terminal
velocities using passive dissipation in the declared affine model. The current
implementation retains open-loop inputs and set-membership intersections.
The broader physical-model, curved-chart, target-motion and low-speed observer
obligations below remain open; the proposals are not completed components.

The recommended first implementation is a scheduled output-feedback tube
around the existing sparse SOCP, with a verified braking-to-hold backup and a
set-valued target-motion contract. Keep the present uncertainty rejection
until those ingredients pass acceptance together. Publishing a current error
radius is necessary, but cannot establish recursive safety by itself.

The target claim is continuous-time collision and road-constraint satisfaction
after initial admission, conditional on explicit bounded sensing, actuation,
plant and environment assumptions. Passive safety is a separate, weaker
alternative and is not silently substituted for this claim. Convergence,
successful passing and real-time completion require separate validation.

## Research scope and evidence

The question is how to preserve a safe continuation when the state estimate,
its uncertainty, and target observations change between control samples.
The study combines primary-source theory, inspection of this repository,
counterexamples, and deterministic MATLAB diagnostics. Searches covered
online estimation bounds, robust output-feedback MPC, predictive safety
filters, measurement-robust barriers, and dynamic-obstacle terminal safety,
including a 2026 update. This is a focused engineering synthesis, not an
exhaustive systematic review or a novelty claim.

The inspected interface is now committed in
`d4da3ce23054fdb084345c138e572dbc71df36af`. In particular,
`ESTIMATOR_BOUND_INTERFACE.md`, `stateUncertainty.m`,
`targetPrediction.m` and `nrmmControllerErrorBounds.m` are
existing implementation inputs, not contributions of this study. Their
snapshot hashes belong in the external research record. The numerical
diagnostics use the existing model/configuration functions and do not invoke
the collision controller.

| Primary source and reading depth | Relevant result | Transfer limit and decision |
| --- | --- | --- |
| Köhler, Müller and Allgöwer (2021), [Robust output feedback model predictive control using online estimation bounds](https://arxiv.org/abs/2105.03427), full-text Theorem 1, Proposition 2, Section IV, Assumptions 7/9, Theorems 4/5 | Online observer bounds, prediction of future bounds, and an observer-to-nominal tube support robust MPC. | Adopt the two-error structure. Its observer transition and terminal hypotheses must be established for our digital NRMM and vehicle. |
| Brunke, Zhou and Schoellig (2022), [Robust Predictive Output-Feedback Safety Filter for Uncertain Nonlinear Control Systems](https://arxiv.org/abs/2212.08900), full-text Section V, Algorithm 1 and Theorem 1 | A robust observer and retained backup can provide safety despite optimizer failure. | Its general safe-set version need not make every subsequent optimization feasible. Distinguish executable backup safety from fixed-horizon recursive feasibility. |
| Leeman, Köhler, Bennani and Zeilinger (2023), [Predictive safety filter using system level synthesis](https://proceedings.mlr.press/v211/leeman23a.html), full-text Section 3 and Proposition 2 | Optimizing closed-loop disturbance responses can reduce conservatism for linear systems. | Reserve for a later extension if fixed-gain tubes are too conservative; output feedback and our stopping model still need their own derivations. |
| Cosner et al. (2021), [Measurement-Robust Control Barrier Functions: Certainty in Safety with Uncertainty in State](https://arxiv.org/abs/2104.14030), full-text backup-set and measurement-robust conditions | Measurement error can enter a barrier-based backup construction. | A viable secondary guard, but requires a sound backup set, derivative bounds and sampled-data treatment. It does not automatically repair our terminal set. |
| Mitsch, Ghorbal, Vogelbacher and Platzer (2017; 2016 preprint), [Formal Verification of Obstacle Avoidance and Navigation of Ground Robots](https://arxiv.org/abs/1605.00604), full-text safety definitions and Sections 6/7 | Separates static, passive and passive-friendly safety, including sensor/actuator and reaction-delay premises. | Stopping without collision while moving is weaker than remaining collision-free after stopping. Preserve that distinction in tests and metadata. |
| Althoff and Magdici (2016), [Set-based prediction of traffic participants on arbitrary road networks](https://doi.org/10.1109/TIV.2016.2622920), institutional abstract and metadata | Predicts occupancy sets under stated motion assumptions. | Supports a road/route-constrained occupancy interface. Detailed vehicle-specific bounds still require derivation; no theorem is imported from the abstract. |
| Dey and Bhasin (2026), [Output Feedback MPC with Adaptive Tubes](https://arxiv.org/abs/2605.23661), arXiv abstract and metadata only | Proposes adaptive tubes for LTI parametric and additive uncertainty, without requiring a common quadratically stabilizing gain across the parameter set. | A current alternative for later parameter adaptation. The abstract is insufficient to transplant its theorem to our scheduled nonlinear plant. |

An additional search identified Pek and Althoff's 2018 invariably-safe-set
work, but the full-text host returned HTTP 429. It is not used as a
theorem-level dependency. None of these papers proves this repository safe.

The apparent disagreement between terminal equality and robust terminal
sets is resolved by the controlled object: a nominal equilibrium can be
surrounded by a nonzero invariant feedback tube, while forcing the *actual*
uncertain velocity to zero using a common open-loop sequence is generally
impossible. Similarly, a safe backup can survive optimizer failure without
making a newly recentered optimization feasible. These distinctions determine
the implementation below.

## Gaps established from the current code

1. **The stored continuation is an input sequence.**
   `collisionAvoidanceController` rejects nonzero ego/model errors. Its
   nominal rest endpoint and appended fixed input do not counter disturbances
   or estimation-induced feedback errors.
2. **Estimation validity ends before the braking tail.** Both
   `nrmmTrackingConfig` and the current integration configuration require a
   true ego speed of at least 5 m/s. The yaw correction uses GNSS course.
   `nrmmPositionErrorBound` invalidates an uninformative/inconsistent course
   measurement. The controller tail reaches 0 m/s. A stopped target also lies
   outside the current positive-speed target reconstruction domain.
3. **The zero-speed model cannot contract all six coordinates.** Numerical
   PBH checks at eigenvalue one have rank 5 at zero speed on both tested
   roads. Positive-speed checks have rank 6. A full-state LQR tube cannot
   simply be continued to zero speed. Frozen position/heading uncertainty can
   remain bounded under a physically valid hold mode; it need not contract.
4. **Current bounds are not predictions of future observer outputs.** The
   sampled NRMM bound encloses accepted numerical states and includes
   integration defects. Its source explicitly avoids a sampled exponential
   stability claim. Its update depends on sensor age, radar mode, chart
   validity and realized defects. Substituting the continuous ISS decay rate
   for a digital future transition would leave a proof gap.
5. **Several dormant uncertainty paths need stronger mathematics.**
   `ltvBicycleModel.predict` currently maps position radii to Frenet coordinates
   with sums and leaves the yaw radius unchanged. On a curved road the
   tangent also changes. Its `Ts*w` disturbance update requires an endpoint
   enclosure interpretation; continuous derivative-error bounds generally
   need transition-weighted integration.
6. **Environment and timing are part of the certificate.** Domain-only
   moving-target bounds produce infinite terminal support in the current
   controller. The adapter stops publishing a target when current radar
   visibility is lost. Existing geometry certifies prediction nodes only.
   The earlier 50 ms experiments still had startup/acquisition deadline
   misses. A robust tube alone changes none of those facts.

The diagnostics and derivations here motivate changes; the uncertainty guard
currently prevents the dormant propagation issues from being presented as
an admitted uncertain-ego safety guarantee.

## Proposed state and error contract

Use a common timestamp, coordinate chart and actuator convention for true
state `x`, estimate `xhat`, nominal state `z`, and applied input. Define

\[
e=x-\hat x,\qquad q=\hat x-z,\qquad d=x-z=q+e.
\]

The safety state must include the observer mode and bound state, last sensor
times, stored nominal trajectory, feedback gains, tube sets, target reachable
sets, road chart, pending actuator input, and backup mode. A smaller radius
alone is not an admission criterion. For example, the sets `[-1,1]` and
`[0.5,1.5]` can contain the same truth while the latter has a smaller radius
and is not contained in the former.

Extend the estimator certificate with a separately validated future contract:

- An estimator-error transition enclosing all admissible measurements,
  dropouts, input histories, integration defects and mode transitions.
- An enclosure of the observer correction relative to the controller model.
- A maximum certified sensor age and a mode-specific validity horizon.
- The geometry, initialization and disturbance assumptions supporting these
  enclosures. Current timestamps and provenance remain mandatory.

One possible offline-bounded comparison is

\[
\bar b_{j+1|k}=\Phi_{m_j}\bar b_{j|k}+\bar d_{m_j},
\quad \Phi_{m_j}\ge0,\qquad b_{k+j}\le\bar b_{j|k}.
\]

This is a **required property**, not an established property of the current
runtime. A continuous Metzler comparison can contribute to it, but predictor
resets, discretization defects, chart changes and dropout intervals must be
included. Publish bounds on the component vector rather than reusing a single
relative-position radius for all state components. A bounded transition need
not be contractive at every step; infinite backup operation needs a suitable
invariant envelope or a hold mode that no longer relies on that channel.

For absolute geometry on a smooth route, if projection is unique and
`1-kappa*d` stays bounded away from zero, derive bounds for the nonlinear
coordinate map over the entire uncertainty set. In particular,

\[
|e_{\psi,\mathrm{true}}-\hat e_\psi|\le b_\psi+\bar\kappa b_s
\]

is a sufficient heading-error enclosure once `b_s` itself is certified.
Station sensitivity includes the projection denominator. For polyline
vertices, explicitly cover all possible segments or constrain the whole
state set to one valid chart. Road registration uncertainty is additional.

For collision geometry, retaining a joint ego/target set or the directly
estimated relative vector can avoid double-counting common GNSS translation
error. Independent absolute boxes are a sound conservative fallback only
when each box is valid. Do not remove ego uncertainty from absolute road
constraints, or presume future relative correlation without propagating it.

## Output-feedback tubes and the single SOCP

On a certified scheduled affine model, write

\[
x^+=Ax+Bu+c+w,\quad z^+=Az+Bv+c,\quad
u=v+K(\hat x-z),\quad w\in\mathcal W.
\]

Direct subtraction gives

\[
d^+=(A+BK)d-BKe+w. \tag{1}
\]

The `-BK e` term is essential. Replacing `A` by `A+BK` in the old radius
recursion while ignoring measurement error in the applied feedback is unsound.
One valid outer recursion, without assuming independence, is

\[
\mathcal D^+=(A+BK)\mathcal D\oplus(-BK)\mathcal E\oplus\mathcal W.
\]

For the preferred estimate-to-nominal representation, define the actual
observer innovation

\[
\ell=\hat x^+-(A\hat x+Bu+c)=Ae+w-e^+.
\]

Then `q+ = (A+BK)q + ell`. A general conservative enclosure is
`L = A E (+) W (+) (-E+)`; a direct NRMM innovation bound can be tighter.
Propagate `S+ = (A+BK)S (+) L` and use `x in z (+) S (+) E`.
This identity does not assume that the NRMM internally uses the same model;
model mismatch must be covered by `w`. Re-centering and new bound admission
must preserve containment, not merely equality of source labels.

The initial nominal center must be allowed to differ from the new estimate:
retain the shifted center or optimize it subject to `xhat-z0 in S0`.
For an ellipsoid this admission is another small SOC. Forcing `z0=xhat`
at every sample requires a different recursive-feasibility argument; resetting
the tube radius to zero does not provide one. Keep the stored schedule and
chart for the witness unless a replacement encloses their mismatch.

For a hard row `H_x x + H_u u <= h`, the resulting tightening is

\[
H_xz+H_uv+
h_{\mathcal S}((H_x+H_uK)^\top)
+h_{\mathcal E}(H_x^\top)\le h, \tag{2}
\]

where `h_C(a)=sup_{c in C} a^T c`, applied row by row. This directly covers
state-input coupling, including affine tire-slip domain rows. Input-only rows
reserve the correction `Kq`; no clipping is permitted after certification.
State-dependent tire/load terms need robust bounds on their coefficients too.
For a direct `D` tube, derive its matching input tightening using `q=d-e`;
do not mix the two error conventions.

For collision separation along a fixed unit normal `n`, require

\[
n^\top(z_{p,E}-\bar p_T)\ge
h_{\mathrm{ego\ footprint}}(-n)+h_{\mathrm{target\ footprint}}(n)
+h_{\mathcal R_E}(-n)+h_{\mathcal R_T}(n)
+d_{\mathrm{clear}}+m_{\mathrm{interval}}. \tag{3}
\]

Footprints must include their respective orientation ranges; `R_E` is the
position projection of `S (+) E`. Bound curvature/chart effects when mapping
these sets to Cartesian space. A joint relative-set support can replace the
two position terms only with an established joint enclosure.

Precompute gains, shape matrices, certified model domains and contraction
constants offline. Fixed normal directions, feedback gains and tube shapes
make the support coefficients data. Homothetic radii with linear upper
recursions add linear constraints; fixed ellipsoidal supports and suitable
terminal norms are SOCP representable. Retain the sparse nominal equalities
and the soft first-step CLF. Safety remains hard. This preserves the *problem
class*, not a demonstrated 50 ms runtime.

Schur stability of `A+BK` does not imply contraction of `abs(A+BK)`.
Use a verified metric, transformed box, or zonotope with certified outer
reduction. For a metric `P`, verify a common contraction or a scheduled
inequality `F_j^T P_(j+1) F_j <= rho_j^2 P_j` over the declared model domain.
Pointwise LQR gains on a speed grid do not prove stability under switching.
Optimize feedback responses with SLS only if the simpler shape family fails
admission benchmarks; its larger optimization is not the first migration.

For a continuous additive disturbance box `w_c`, the sampled disturbance
must contain

\[
\int_0^{T_s}e^{A_c(T_s-\tau)}w_c(\tau)\,d\tau.
\]

The componentwise integral of `abs(expm(A_c*tau))*wbar_c` is sufficient.
Treat gain uncertainty, tire nonlinearities, actuator delay, numerical flow
error and linearization residuals consistently. A finite observed maximum
residual is useful for falsification, but is not a worst-case physical bound.
The existing gain 0.80 is not a certified effectiveness interval.

## Braking, low-speed estimation and terminal hold

Use an explicit mode sequence: normal motion, certified braking/crawl,
then hold. The sequence can be selected before the SOCP to avoid mixed-integer
decisions. Each transition needs set inclusion in the next mode's certified
domain. Early versions may use a fixed terminal hold region on a selected
roadside or upstream corridor; they need not solve unrestricted parking.

During braking, switch away from course-based yaw correction **before** its
domain is lost. Propagate yaw using gyro integration and its bounded error,
with a circular radius and finite braking horizon. GNSS position/velocity
remain bounded measurements even when course is uninformative. Do not obtain
this mode by simply setting the old positive speed minimum to zero. A
low-speed target needs a separate enclosure that does not invert speed; a
position reachable set and orientation circumradius can suffice for safety.

At hold, define a physical brake policy and a set of possible resting poses.
The current plant adapter converts negative axle force to brake pressure,
but zero acceleration at zero bias commands no such holding pressure. A
dedicated hold interface must specify brake engagement, response delay,
available holding force, slope/external-force domain and lateral slip limits.
Once physical rest is established, hold geometry may use the frozen pose
enclosure from entry, avoiding unbounded gyro-integrated heading drift.
An estimated zero velocity alone does not establish physical rest.

For a longitudinal conservative stopping construction, with an upper speed
`vU`, reaction time `Delta`, possible acceleration `aU` during the delay and
a guaranteed net braking deceleration `bL > 0`, use

\[
T_{stop}\le\Delta+(v_U+a_U\Delta)/b_L,
\quad D_{stop}\le v_U\Delta+\tfrac12a_U\Delta^2
+\frac{(v_U+a_U\Delta)^2}{2b_L}. \tag{4}
\]

These bounds assume nonnegative speed and a valid constant deceleration
lower bound; turning, tire-force sharing and lateral excursions require
additional reachability bounds. The scheduled continuation length must cover
this robust stopping time, replacing its current nominal derivation.

The terminal requirement is a set condition: all reachable braking endpoints
enter a region where the hold policy keeps the complete true-state set inside
road and actuator limits and outside target occupancy for all future time.
A small velocity tolerance or longer nominal tail does not establish this.
If disturbances can move an unactuated rest coordinate indefinitely, no
bounded hold region exists under those assumptions. Restrict the disturbance
domain on physical grounds or use a different terminal policy; do not claim
that a generic six-state RPI ellipsoid always exists.

## Future target motion, visibility and consistency

A target with only finite speed/acceleration bounds may eventually reach a
parked ego. Even zero current estimation error cannot prevent that. No
controller can guarantee perpetual collision avoidance in every admissible
environment, including an unavoidable collision or a trapped ego.

For strict safety, introduce a conditional environment contract containing
allowed route/corridor sets, direction/reversal rules, speed/acceleration
domains, and all branches that remain possible. Predict occupancy over each
time interval, not only target centers at nodes. For straight and circular
research scenarios, an analytic route tube can be a declared *scenario
assumption*, separately identified from estimated motion. A full periodic
route must retain all future revolutions. For unbounded straight travel,
longitudinal support may diverge while lateral support stays finite; a safe
side region can still exist.

There are also narrower analytical alternatives to a full road predictor.
If a persistent motion contract ensures `n^T v_T(t) <= 0` for all future
time, then the complete future position support along `n` is bounded by
its current support. For example, a target guaranteed to keep inertial
longitudinal velocity in `[7,9]` m/s has finite support in the negative
longitudinal direction, despite nonzero velocity uncertainty. A generic
nominal-support-plus-growing-error calculation misses that cancellation.
For a target guaranteed to retain constant curvature with `|kappa| >= kMin > 0`,
the full orbit is bounded: displacement from its initial position is at most
`2/kMin`, before adding initial-position uncertainty. A curvature interval
containing zero does not give that bound. These are useful specialized
terminal-support implementations, but an instantaneous velocity or curvature
estimate does not establish either persistent contract. In particular,
the target's nominal NRMM evolution should not be mistaken for an observed
promise that its future maneuver parameters never change.

The stored terminal region must be disjoint from the union of every permitted
future target occupancy, or be part of a verified interactive terminal policy.
Road confinement alone is insufficient if both vehicles can occupy the same
terminal region. If the contract cannot establish either construction,
report lack of strict-safety admission. Passive safety can be offered as a
separate experiment with a different claim, not as a successful strict case.

Maintain target reachable sets across samples. Under unchanged assumptions,
update a prior prediction by intersecting it with the new measurement set,
then propagate. Required overlapping occupancies satisfy

\[
\mathcal O_{j|k+1}\subseteq\mathcal O_{j+1|k}. \tag{5}
\]

Preserve intersection information or verify that any outer approximation
still fits inside the old envelope. Containment of the true target by two
unrelated boxes does not imply (5). A failed consistency check is an
assumption violation or a reason to retain a valid prior envelope, not a
reason to silently inflate a certificate already executed.

Loss of radar visibility must not delete a physical obstacle. Keep a coasting
reachable set until a certified exit/non-return condition allows removal.
New objects, track reassociation and road updates also need admission rules.
Protect against unseen traffic using a stopping/escape region contained in
certified observed free space plus bounds on entrants. A range threshold or
successful acquisition in a previous simulation is not that proof.

## Sampling, deadlines and safe continuation

For a fixed separator over one sample, a sufficient interval margin is

\[
m_{interval}\ge T_s(\bar V_E+\bar V_T
+R_E\bar\omega_E+R_T\bar\omega_T), \tag{6}
\]

where the speed bounds cover the whole interval and `R` denotes footprint
circumradius. This conservative Lipschitz bound covers center translation
and rotating support; certify road rows similarly. Exact affine flowpipes
and interval target reachability should later reduce this margin. If normals
change within an interval, their variation must also be enclosed. Dense
simulation is a useful counterexample search, not a continuous-time proof.

Model measured-to-applied delay explicitly. Timestamp the future application
state, propagate its uncertainty through the pending input, and check the
entire delay interval. Store an executable backup **policy**, its nominal
reference and valid sensor-age envelope before starting the solver. At a
missed deadline, apply the corresponding certified feedback action, or the
certified braking/hold action for that mode. Holding an old raw input or
raising an exception is not automatically safe.

The observer must remain available within its stated age bound during
fallback. Unbounded sensing or execution outage cannot inherit the same
certificate. A watchdog can execute a pre-certified backup without adding
a second optimization per control sample.

## Conditional proof and implementation sequence

Let the admitted augmented state contain a valid observer enclosure, a
feedback tube, an actuator queue, target/road contracts and a safe terminal
backup. Under the declared assumptions, the proof must establish:

1. The actual observer error and next innovation lie inside their predicted
   sets for every allowed sensor/mode realization.
2. The applied feedback keeps the true trajectory and inputs in the tube;
   tightened rows and interval margins imply physical constraint satisfaction.
3. A shifted nominal reference and its feedback policy remain available.
   New centers, coordinate charts and model schedules either preserve that
   witness or pass a separately checked replacement admission.
4. Target/road updates preserve the stored occupied/free-space envelopes.
5. The terminal transition reaches an invariant safe hold/policy domain.
6. Solver failure or deadline expiry executes the already verified witness.

These statements imply safety by induction after initial admission. Fixed
horizon recursive feasibility additionally requires that the shifted policy
be representable in the next optimization, including terminal and geometry
choices. A fallback policy's existence alone is insufficient. No global
navigation or robustness theorem is claimed until these obligations are met.

| Stage | Concrete implementation | Required exit evidence |
| --- | --- | --- |
| 1. Estimator and frame contract | Add a bounded digital future transition and innovation enclosure; add low-speed/hold modes; repair Frenet uncertainty mapping. Keep `controllerErrorBound` as the current-state interface. | Noisy, biased-within-premise, dropout, yaw-wrap, braking-to-zero, stopped-target and curved-chart containment; bound availability checked through the full backup. |
| 2. Ego tube | Add offline gain/shape synthesis and a single tube-propagation module; robustly tighten all state/input/geometry rows in prediction/formulation. | Equation (1) corner checks; shape inclusion and model-domain proofs; actuator execution within its reserved set. |
| 3. Terminal and environment | Add a braking-to-hold certificate, hold actuator mode and target occupancy contract; preserve targets through temporary invisibility. | Nonempty safe terminal region under explicit assumptions; route-union and non-return checks; reject adversarial/no-safe-region examples. |
| 4. Certificate migration | Version the stored certificate; include policy/tubes/observer mode/application time; update independent acceptance, compatibility and fallback. | Shifted witness survives changed measurements, failed solve and deadline expiry. A forced recenter that breaks inclusion is rejected. |
| 5. Vehicle validation | Extend existing straight/arc drivers with the genuine estimator and bounded plant mismatch; log all containment and physical margins. | Full-duration noisy avoidance and stop/hold runs; no zeroed uncertainty, dropped targets or safety slack; explicit worst-case timing and guarantee scope. |

In the first prototype fix the route branch, terminal region, gain schedule
and tube shapes outside the optimizer. Retain the existing one-solve sparse
formulation and performance objective. More flexible maneuvers, gain
optimization and online parameter adaptation follow only if measured
conservatism warrants the cost.

## Reproducible diagnostics and proposed validation

Run from the repository root:

```matlab
addpath('scripts');
result = analyzeRobustOutputFeedbackDesign();
```

The completed diagnostics use 50 ms sampling. For straight and radius-400 m
scheduled bicycle models, speeds 0/1/5/15 m/s give PBH ranks 5/6/6/6 at the
unit eigenvalue. The two-state longitudinal example uses input gain 0.8,
`Q=I`, `R=1`, and `u=v+K(xhat-z)`:

| Completed diagnostic | Result | Interpretation |
| --- | --- | --- |
| Feedback gain | `[-0.963277394, -1.826498514]` | Illustrative LTI feedback only. |
| Spectral radii | `rho(F)=0.963277394`; `rho(abs(F))=1.019163653` | Stable dynamics can have an expanding componentwise box recursion. |
| Metric contraction | `0.986811144` | Verified from the discrete Lyapunov metric for this LTI example. |
| 200-step final position bounds | Box: 8.520775532 m; metric projection: 0.134011031 m | Same uncertainty premises; difference is representation conservatism, not estimator improvement. |
| Sampled containment | 512 trajectories, seed 7, maximum normalized error 0.919544482 | Supports implementation arithmetic; does not prove vehicle containment. The LTI norm inequality is the analytical basis. |
| Curved-coordinate example | 0.12 m station error on radius 60 m produces 0.002 rad Frenet heading error despite zero inertial yaw error. | Position and heading uncertainty cannot be transformed independently. |
| Held acceleration error | 1 m/s^2 over 0.05 s changes position by 0.00125 m. | A zero position component in `Ts*w_c` misses within-step coupling. |

All diagnostic assertions passed. Factory Code Analyzer reports zero findings
after removing an unnecessary suppression comment flagged on its first run.
The 17 interface/reconstruction tests run immediately before this study passed;
they do not test the proposed architecture. No new NRMM vehicle closed-loop
simulation, invariant-set construction or timing benchmark has been completed
for the proposal.

Future experiments should cross straight/radius-400 m roads with nominal
cruise, crossing, oncoming, braking-to-hold and stationary/slow targets. Use
bounded adversarial noise and domain extremes alongside seeded randomized
trials; perturb braking effectiveness, delay, dropouts and road registration.
Compare fixed worst-case bounds, current online bounds, and the proposed
feedback tube under identical physical assumptions. Report true-error
containment for every component, interval SAT/road margins, actuator reserve,
terminal inclusion, successful witness reuse, acquisition failures, deadline
misses and end-to-end runtime distributions. Monte Carlo collision counts
cannot replace universal set inclusion.

Review checkpoints found three tempting but invalid shortcuts: assuming
future NRMM contraction from the continuous theorem, extrapolating a
positive-speed feedback to physical rest, and replacing an unrestricted
moving target by a permanently frozen current radius. The proposed design
explicitly avoids them. It remains conditional on establishing feasible
physical and environment contracts; the research does not show those
contracts hold for PassVeh14DOF or a deployed vehicle.

AI assistance was used for source retrieval, code inspection, derivation,
diagnostic scripting and drafting. This is an internal research design note,
not an independent peer review or a claim of human verification of all cited
full texts.
