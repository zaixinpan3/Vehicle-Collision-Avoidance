# Information-state safe MPC with finite encounter completion

Certificate version 22, September 14, 2026. The controller carries a
finite, independently verified encounter witness followed by a
**target-independent road terminal controller**. Its executable safety value
is exactly zero. Positive safety-value solutions remain solver diagnostics;
they cannot authorize a command. The finite reach-avoid derivation and global
entry limitations are developed in
[FINITE_ENCOUNTER_TERMINAL_DESIGN.md](FINITE_ENCOUNTER_TERMINAL_DESIGN.md).

## Executed specification and premises

The target-specific obligation is rectangle separation, including configured
clearance, during an admitted active encounter. Only a sound current
observation discharges that obligation. Road, chart, model-domain, actuator
and slew obligations remain after discharge.

The implemented certificate uses `N=L=M`: every finite held interval has
swept collision and road rows, the final node has robust geometric exit rows,
and that same node belongs to the road-only terminal set. Ego velocity need
not be zero there. Early confirmed release retains the remaining road-safe
inputs. A separately optimized road-only tail with `L<M` is not implemented.

The conditional guarantee requires:

- One ego and zero or one admitted target. An active target's identity and
  footprint are fixed. New targets and enlarged future motion bounds require
  fresh admission; an old witness does not certify these new obligations.
- Current ego and target error boxes contain the true states. Ego and target
  measurements are intersected with the previously published reachable sets.
  Empty intersections invalidate the assumptions and stop control.
- Each held ego interval executes exactly the issued input and the declared
  affine generator `xdot=A*x+B*u+c`. Ego process residual bounds are zero.
  Road geometry, physical configuration, sample spacing and reported previous
  input agree with the stored execution contract.
- While active, the target follows `pdot=v`, `vdot=a`, `abs(adot)<=J`,
  `psidot=omega`, `abs(omegadot)<=H`. `J` and `H` are fixed or smaller on
  continuation. Zero derivative bounds do not extend model validity beyond
  the active encounter.
- The circular confirmation region is centered at the ego reference point,
  with the radius supplied in `ego.perception.range`. A truthful, current
  complete observation is available at the planned confirmation sample.
  The implementation supports zero confirmation delay. Missing confirmation
  at the deadline invalidates the completion premise; elapsed time alone
  never releases the target.
- The retained sampled road terminal model, comparison contraction and
  invariant-domain premises hold. These are the existing declared rest-model
  assumptions, not a validated nonlinear vehicle inclusion.

These are not claims of nonlinear vehicle safety, sensing reliability,
feasibility of arbitrary encounters, or real-time execution.

## Finite propagation and swept safety

The ego endpoint is computed by the exact held affine exponential. If the
current box is `z +/- rho`, its successor center is
`Phi*z+Gamma*u+eta`, with radius `abs(Phi)*rho`. The carried witness retains
its own generators, controls, charts, cell normals and published node boxes.
It does not replace those generators with a fresh linearization and assume
that the resulting enclosure is nested.

Target centers use constant Cartesian acceleration; target position radii
are

    rp(t) = rp(0) + rv(0)*t + ra(0)*t^2/2 + J*t^3/6.

Velocity, acceleration and yaw bounds are propagated by the corresponding
integrals. They remain finite at every finite prediction time, including
when both jerk components are positive. No cap or anticipated observer reset
is applied. `nominalFlow` provides curved optimization anchors only;
`finiteFlow` is the safety model.

For each swept cell, a separating normal and rectangle support majorants
produce affine inequalities at every Bernstein point. Nonnegative Bernstein
basis weights summing to one imply the inequality throughout the cell.
Taylor remainders, propagated uncertainty, yaw/footprint bounds and chart
errors are included before acceptance. Collision constraints cover **all**
finite cells, including the final hold ending at confirmation. Road and
model-domain rows likewise cover the complete finite witness.

## Convex finite exit

For final chart `pE=o+L*z+epsilon`, fixed exit direction `m`, ego radius
`rho`, target center `pT`, and target position radius `rp`, impose

    m' * L * z <= m' * (pT-o)
                   - abs(m)' * rp - abs(m'*L) * rho
                   - abs(m)' * chartPositionError
                   - targetRectangleSupport(-m)
                   - (range + exitReserve) * norm(m).

The target support is maximized over its complete final yaw interval. Thus
the **entire target footprint**, rather than just its nominal center, lies
outside the declared range ball. Chart validity follows from the same road
terminal chart constraints. The norm factor makes the exterior implication
valid even with floating-point normalization. Independent arithmetic reserves
are subtracted as well.

This affine directional condition is a conservative inner approximation of
the nonconvex exterior of a ball. For a predicted close approach, admission
proposes an exit direction along relative motion, so an observed approaching
exterior target must pass before a position observation can release it. Other
encounters use the final anchor's relative direction. A carried rebuild keeps
the original exit direction and chart. All exit rows are hard in both
lexicographic optimization stages. Only verified finite completions admit
control; a search failure does not establish physical collision inevitability
or global infeasibility.

At admission the declared range must exceed both body circumradii plus
collision clearance. This makes a complete absence observation compatible
with the local collision region. A complete scan reporting absence while
the carried target reference box is wholly inside that range is rejected
as inconsistent.

## Observation release and deadlines

A current complete scan requires matching `perception.time`, a positive
finite `range`, and logical `completeWithinRange=true`. Active certificates
require exactly the same range as their carried confirmation contract.
An active target is discharged by either:

1. Current complete absence consistent with its carried reachable set; or
2. A current target observation, conditioned against that reachable set,
   whose whole footprint is robustly exterior to the region.

Prediction, nominal range crossing and a timer never trigger release.
Absent or stale confirmation cannot authorize road-only terminal execution.
A previously discharged exterior object can need fresh admission when its
current velocity predicts an approach. Other robustly exterior observations
create no new inside-region obligation. Complete absence still follows the
stated scan contract; this does not prove safety across re-entry, temporary
loss of an exterior track, or detection latency.

Admission estimates an initial horizon from relative motion and the required
departure side. This is a search proposal only: finite exit must still be
verified. Smooth left/right offset paths propose consistent cell normals,
while every held-input stage retains the same swept verification. A positive
safety-value diagnostic or an approximate solver result that fails verification
can try another proposal. The timed search extends the horizon after exhausted
seeds; it reserves time for alternative normal families. During an active encounter a fresh
horizon is capped at the stored absolute exit deadline. One remaining hold
is supported. A replacement cannot move that deadline later. A failed fresh
solve leaves the independently verified carried suffix available. After
release the road-only problem can replan with its usual rolling horizon.

## Road-only invariant terminal set

Write `x=(q,v)`, where `q=(s,d,ePsi)` and `v=(vx,vy,r)`. Let `d0>=0` be
longitudinal passive damping, `gBeta>0` the acceleration gain, and `h` the
hold duration. With `phi(t)=(1-exp(-d0*t))/d0` (or `t` when `d0=0`), set

    b0 = exp(-d0*h)*(1-exp(-h))/phi(h),
    l_k = z_vx,k - rho_vx,k >= 0,
    beta_k = -b0*l_k/gBeta.

The held input acts on the **certified lower speed endpoint**. Its trajectory
is `l(t)=(exp(-d0*t)-b0*phi(t))*l_k`, which remains nonnegative throughout
the hold, and `l_(k+1)=exp(-(d0+1)*h)*l_k`. The box radius decays as
`exp(-d0*t)*rho_vx,k`. Braking the center alone fails this property because
the radius can decay more slowly than the center and the true speed can
become negative. The endpoint calculation uses the proven nonnegative lower
endpoint to enclose roundoff at rest; no negative lower endpoint is admitted.

For the retained rest model, the open-loop velocity-error comparison is the
Metzler matrix `Ce`. The nominal comparison is `C=Ce-B`, with
`B=diag(b0,0,0)`. Throughout each hold,

    d|vbar|/dt <= C*|vbar| + B*rho_v,
    d rho_v/dt <= Ce*rho_v.

An uncertain terminal velocity requires a Hurwitz `Ce`, hence finite passive
error excursion. Exact-state terminal operation may use a contracting `C`
with zero error even when the passive longitudinal damping is zero.

For each road or chart pose row `a*q<=b`, let `g` bound the positive pose
growth and set `R=g/(-C)`. Nonnegative nominal longitudinal speed permits
using `max((a*D)(1),0)` for the longitudinal term. The other terms use
absolute values. The radius-dependent input requires the coupled error budget
`Re=(abs(a*D)+R*B)/(-Ce)`. Verified nonnegative budgets with arithmetic reserves
satisfy `R*C+g<=0` and `Re*Ce+abs(a*D)+R*B<=0`. Their derivatives show that

    a*qbar + R*abs(vbar) + abs(a)*rho_q + Re*rho_v <= b

is invariant under the declared comparison assumptions. Enumerating nominal
velocity signs gives the linear rows `Af*z+Ef*rho<=bf`. Velocity-domain,
terminal-input and first-terminal-input slew constraints are retained. There
are **no target rows or infinite-time target support calculations** in this
set. Its controller may reduce speed after handoff; cruise remains a soft
CLF performance objective.

The input is stored as `uf+K*z+Kr*rho`, where `Kr=-K` in the longitudinal
brake channel. Both contributions enter the first-terminal-input slew rows.
The upper speed endpoint contracts under passive damping; velocity and tire
slip limits use a verified invariant comparison box. The lower endpoint
contracts by a fixed factor, which also bounds subsequent input changes.

The terminal input uses the carried nominal and its certified box. In
particular, conditioning a new measurement does not silently recenter that
nominal and change the verified transition from the last planned input.

## Independent acceptance and recursive argument

The verifier evaluates physical rows with an arithmetic allowance charged
against their margins. For stage `i`, `xi_i` is the nonnegative maximum
collision/road row violation, and `S=sum(xi_i)`. It reports:

- `candidateAccepted`: finite decision, hard domain/exit/road-terminal rows
  and relaxed CLF checks passed;
- `safetyCertified`: those checks passed **and `S==0`**;
- `accepted`: an alias for `safetyCertified` used by execution.

Numerical solve reserves provide space for solver error. Neither solver
feasibility tolerances nor the lexicographic tie tolerance permit positive
physical safety violation. The CLF remains soft and convex. Only
`clf.samplePoints="controlPoints"` enforces a common majorant at every
Bernstein point and supports an all-time cell dissipation interpretation;
endpoints or stage nodes support their stated sample checks only.

For a zero-value admitted witness, the swept rows certify its first hold.
The next conditioned ego and target sets are subsets of the carried
successor enclosures. With the same controls and generators, reachable sets
are monotone under inclusion, so the stored suffix still covers all valid
successors. `transferCandidate` reuses that verification;
`verifyCandidate` rebuilds with the carried data and independently checks it.
A fresh solve can replace this witness only after zero-value verification
and without extending its active exit deadline.

If a current observation releases the target early, removing its constraints
preserves the road-safe suffix and terminal set. Otherwise the finite exit
row and the confirmation premise imply release at the fixed deadline. The
road terminal law then preserves the permanent obligations. Induction gives
swept collision safety during the admitted active encounter and road
continuation afterward, conditional on the stated premises.

For the same remaining witness and unchanged obligations, dropping the first
stage yields `S_next <= S_current-xi_0`; at the executable level both values
are exactly zero. A positive diagnostic value or locally selected normals
do not establish an exact global PCBF value function. No indefinite old-target
model or invariant target halfspace enters this proof.

## Runtime metadata and implementation

`targetCertifiedUntil` and `certifiedDuration` are finite while a target is
active. `confirmedRelease`, `encounterComplete`, `roadTailCertified`, and
`confirmationContract` report separate obligations. After release the target
validity field is `NaN`; the road continuation duration may be infinite.
`indefiniteRecursiveFeasibilityClaimed` applies only to the target-free road
problem. `physicalVehicleGuaranteeEstablished` remains false.

`hardEncounterBarrier` owns finite completion and observation guards alongside
road terminal construction and witness transfer. `targetPrediction` retains
finite bounded motion and conditioning. `formulateAvoidanceProblem` appends
hard exit and road terminal rows. `solveHardCbfClf` distinguishes numerical
candidates from executable certificates; `collisionAvoidanceController`
checks and publishes the carried state.

Before fresh work, the planner checks the frame budget, including the first
attempt. The maximum observed attempt cost screens long horizons. Native
Clarabel receives the remaining program time via an optional fourth options
entry; all omitted-row checks and independent verification still apply to
any returned decision. An expired frame uses the carried witness. Initial
admission instead uses `certificateSearchTimeLimit` and cannot execute until
it obtains a complete witness. Native time limits do not bound MATLAB
formulation, factorization setup, verification, or operating-system delays;
this is not a hard 100 ms execution guarantee.

`runExactStateRecursiveFeasibilityScenario` independently samples the true
state domains and checks true ego/target containment in the published boxes.
Its pass criterion cannot be satisfied by controller certification flags
alone. Descent and carried-candidate checks restart for a new admission.

See `tests/controllerRepairTest.m`,
[report/CONTROLLER_REPAIR_RESULTS_20260914.md](../report/CONTROLLER_REPAIR_RESULTS_20260914.md),
`tests/finiteEncounterCompletionTest.m` and
[report/FINITE_COMPLETION_RESULTS_20260913.md](../report/FINITE_COMPLETION_RESULTS_20260913.md)
for actual implementation checks and their limits. Earlier timing and
all-future-terminal experiment records describe their identified source
versions and are not evidence of the new controller's feasibility.
