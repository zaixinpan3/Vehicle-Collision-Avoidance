# Sampled backup CBF and cruise CLF

September 14, 2026. Certificate format 23 introduces a bounded-candidate
execution policy alongside the predictive SOCP policy (format 22).
`controller.executionPolicy="backup"` selects it explicitly; `"predictive"`
selects the optimization policy. The default `"auto"` selects backup when a
finite `solver.frameDeadlineSeconds` is supplied, and keeps using it for a
format-23 witness. Moving-offset references retain predictive mode under
`auto`; explicit backup requires a positive constant-speed trim.
The experiment's default 100 ms budget selects backup.

## Problem and implemented change

The previous experiment found multi-second admission, positive-slack CLF
acceptance and persistent post-encounter oscillation. Its stopping backup
could preserve safety while defeating cruise recovery. Profiling the repaired
implementation shows that rebuilding prediction and constraints costs more
than the solver in an ordinary cruising frame. Changing the solver alone
does not address either problem.

The new policy uses three operations:

1. At encounter admission, generate at most two prescribed passing/cruise
   continuations, each at most 64 holds, and verify the entire continuation.
   The proposals are saturated, slew-limited sampled feedback rollouts about
   the cruise trim, with passing offsets and optional speed changes. These
   are proposals only: neither the feedback gain nor nominal separation
   authorizes a command.
2. During an admitted encounter, execute the verified suffix. Current
   measurements condition the carried boxes and the existing current-scan
   guard confirms release. No online optimization is needed. The absolute
   encounter deadline cannot move forward.
3. After release, propose a sampled cruise input and verify its full hold,
   sampled CLF decrease and membership of the successor box in the road
   stopping set. When this immediate handoff is unavailable near the end of
   a witness, a finite return-to-path rollout can be admitted by the same
   full safety check. Otherwise execute the carried witness or its invariant
   stopping continuation.

There is no online optimizer in this policy. The SOCP implementation remains
available for experiments requiring a larger search family. Removing the
optimizer reduces the feasible search family; a failed bank does not prove
that the encounter is impossible. No positive safety slack is executable.

If a direct terminal input would violate slew limits, at most 16 braking
transition holds are appended, within the same 64-hold total cap. These
holds are fully verified with the proposal. Ordinary cruise therefore need
not jump instantaneously from positive drive to the stopping brake.

## Declared model and continuous safety

This remains an experiment with a **declared affine ego plant**, not a
validated nonlinear vehicle. Each issued hold executes the recorded
`executedContinuousGenerator`. Backup proposals use the affine generator
linearized at the requested cruise trim. They do not silently claim inclusion
of a trajectory-linearized or nonlinear Fiala plant in that generator.
Cruise references have zero configured moving offset/rate. A fixed curvature
has a common error metric; changing curvature creates a new local metric and
does not establish one global path-tracking Lyapunov function.

`ltvBicycleModel.fixedPredict` keeps only the current hold's two input columns
in each tube. `avoidanceSafetyGeometry.build` substitutes that hold's actual
input before assembling scalar margins. It retains every original road,
chart, model-domain, slip and oriented-footprint separation inequality at
every Bernstein control point. Substitution includes a floating-point
evaluation reserve. Safety margins require an additional numerical reserve;
exact hard-domain/input equalities remain admissible.

The held-flow template is cached with six initial-state columns and two
input columns, using the full declared absolute domain in its arithmetic
reserve. Substituting a stage's initial nominal state leaves the two input
columns needed by geometry. The template's starting radius is the
componentwise maximum over the candidate's stage-start boxes, so it also
covers every stage's uncertainty without assuming future measurement
shrinkage. This common radius can make an uncertain proposal more
conservative. Endpoint boxes still use their individual propagated radii.

The cell count is `max(minimumCells,ceil(norm(A,inf)*h/.9))`. This respects
the geometric-series remainder premise `norm(A,inf)*cellDuration<1`.
The checked Taylor remainder, not the density of a plotted trajectory,
establishes intersample containment. Endpoints use the exact affine sampled
map and monotonically propagated boxes. Continuous geometry is verified
through the finite confirmation deadline; the road terminal set and first
terminal-input slew are also checked. Uncertainty is never capped or reset
in anticipation of a favorable observation.

## Predictive CBF guarantee

Let `S(W)` be the sum of nonnegative per-hold physical safety violations of
a complete witness `W`, including its hard terminal/confirmation obligations.
Only a verified `S(W)=0` is admitted. In the information state augmented by
the stored witness and its remaining deadline, the predictive barrier is
`h_B=-S`. On its certified zero level, the executed shift satisfies

    h_B(I_(k+1), W_shift) >= (1-alpha) h_B(I_k,W_k) = 0,

for any `alpha` in `(0,1]`. The reason is the same inclusion/shift argument
as [the information-state proof](INFORMATION_STATE_PCBF.md): conditioned
boxes lie in the old successor boxes, every remaining hold keeps its verified
generator and controls, and the terminal law preserves permanent obligations.
Confirmed release removes target obligations while retaining the road suffix.
The empty-suffix step is checked by terminal membership.

This is a predictive, sampled information-state barrier certificate. It is
not a claim that the finite bank computes the globally optimal PCBF, or that
a smooth instantaneous distance CBF with a globally feasible derivative QP
has been constructed. Safety is conditional on admission, consistent bounded
motion/measurements, timely execution of the recorded held inputs and valid
departure confirmation. New targets or enlarged motion bounds require new
admission. Re-entry and unobserved targets require additional sensing and
admission assumptions.

## Hard sampled CLF dissipation

For fixed curvature, let the five-dimensional error from the constant-speed
path trim be `e`. With held-input error `v=u-u_*`, the exact sampled error
map is

    e_(k+1) = Phi e_k + Gamma v_k.

`ltvBicycleModel.sampledCruise` synthesizes a discrete LQR gain `K` and
positive-definite `P`. It independently checks the contraction of
`F=Phi-Gamma*K` in the `P` metric, including a numerical reserve:

    F' P F <= q2 P,   0 <= q2 < 1,   V(e)=e' P e.

With exact feedback and exact arithmetic,

    V(e_(k+1)) - V(e_k) <= -(1-q2) V(e_k).

The selected dissipation fraction uses a rate no greater than this bound.
If input or slew saturation changes the feedback, the gain identity alone
cannot authorize dissipation: the implemented quadratic difference is
checked separately on the entire current information box.

For the observed center `e_bar` and true error `e=e_bar+eta`,
`|eta|<=r`, applying the unsaturated input `-K e_bar` gives

    e_(k+1) = F e_k + Gamma K eta.

Let `epsP` enclose `norm(P^(1/2) Gamma K eta)`, including the trim/arithmetic
reserve. For any `c` with `q2<c<1`, Young's inequality gives

    V(e_(k+1)) <= c V(e_k) + c/(c-q2) epsP^2.                 (1)

The uncertain case keeps at least half the available contraction gap for
this disturbance bound. The implementation also bounds the direct quadratic
residual on `e_bar +/- r`, using its linear support and an absolute-matrix
quadratic enclosure. The smaller analytical justification may authorize the
exact unsaturated feedback even if that interval expansion is conservative.
The direct test is mandatory for a saturated proposal.

`clfDissipationCertified` means the issued hold has this checked sampled
dissipation property. Its rate and disturbance bound are reported separately
from collision/road safety. For consecutive certified cruise holds with a
common metric and a uniform bound `b` on the additive term,

    V_k <= c^k V_0 + b (1-c^k)/(1-c).

Exact-state cruise is exponentially convergent in the ideal declared model;
bounded nonvanishing estimation error gives practical dissipation to a
neighborhood, with a numerical floor in floating-point execution. A clear
path does not make an arbitrary state admissible: continued cruise decrease
also requires compatibility with road/input/slew/terminal constraints.
Near a trim strictly inside those constraints, continuity and the contracting
feedback provide a local cruising neighborhood. This implementation does not
compute a maximal cruising attraction region.

During avoidance, a finite recovery rollout or terminal braking, safety can
prevent this decrease. Those commands explicitly report the CLF flag false.
The controller does not label a large optimization slack as a dissipation
guarantee, nor promise monotonically decreasing path error during a maneuver
that must leave an obstructed path. Equation (1) concerns actual sampled
states. It does not assert that a fixed quadratic `V` decreases pointwise
at every intersample instant.
For a common generator, bounded linear flow within each hold also extends
sampled exponential convergence to the continuous tracking error, with a
multiplicative intersample bound.

## Stopping set at normal cruising speeds

The previous fixed brake gain excluded nominal speeds above approximately
8.7 m/s for the default vehicle, even though the model speed domain reached
18 m/s. Write passive longitudinal damping as `d`, hold as `h`, acceleration
gain as `g`, and `phi=(1-exp(-d*h))/d` with its continuous limit at zero.
The updated gain is

    b = min(exp(-d*h)*(1-exp(-h))/phi,
            .98*(u_f,beta-beta_min)*g/v_max).

The terminal brake still acts on the lower speed endpoint `l=z_vx-r_vx`:
`beta=u_f,beta-b*l/g`. Its successor satisfies

    l^+ = (exp(-d*h)-b*phi)*l,   0 < exp(-d*h)-b*phi < 1.

The coupled nominal/error comparison matrices and stopping-excursion budgets
are recomputed with this gain. Slower braking therefore consumes a longer
certified road excursion; it does not obtain a larger speed domain for free.
Chart size uses the proposed entry speed, while the invariant pose rows
themselves enforce remaining within the chosen chart. All configured input,
slip and slew caps and uncertainty terms remain in the terminal certificate.
Previously stored format-22 terminal laws retain their own verified gains.

## Runtime contract

The mathematical work is bounded by the finite bank and tube construction;
active carried-witness steps need only conditioning and inclusion checks.
There is no unbounded horizon extension or iterative solver in the backup
policy. An already expired frame starts no new candidate work. A failed
proposal retains a verified incumbent when one exists. Admission has no
executable command until a complete candidate passes.

A previously verified format-22 plan can also feed the fast executor by
using `auto` with an infinite budget for offline admission and a finite
budget during execution. Its own stage generators and inputs are retained;
the executor does not reinterpret that witness using the cruise generator.

MATLAB, native allocation, first-use compilation and operating-system
scheduling do not have certified worst-case execution times here. Measured
sub-100 ms frames are runtime evidence, not a hard deadline theorem. These
drivers measure computation time but execute the ideal sample/hold clock;
they do not validate sensing-to-actuation latency. A deployed executor must
apply the preverified buffered command on schedule and retain a valid witness
when a planner misses its deadline; that separate real-time deployment and
nonlinear flow inclusion remain required for a physical-vehicle claim.

## Primary literature checked

- Breeden, Garg and Panagou, *Control Barrier Functions in Sampled-Data
  Systems* (2021/2022), distinguish sampled constraints from continuous
  invariance under held controls. This motivates keeping the swept geometry
  proof rather than checking only sampled distances.
  [Paper](https://arxiv.org/abs/2103.03677).
- Taylor, Dorobantu, Yue, Tabuada and Ames, *Sampled-Data Stabilization with
  Control Lyapunov Functions via Quadratically Constrained Quadratic
  Programs* (2021), examine the gap between continuous design and sampled
  implementation. Our linear exact-map inequality above is derived directly
  for this declared plant; their nonlinear practical-stability assumptions
  are not imported automatically.
  [Author manuscript](https://andrewjamestaylor.github.io/assets/paper_materials/taylor2021sampled_data_stabilization_with_control_lyapunov_functions_via_quadratically_constrained_quadratic_programs/paper.pdf).
