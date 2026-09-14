# Controller repair and validation, September 14, 2026

Controller certificate version 22. This implementation follows the
[exact-state audit](EXACT_STATE_CONTROLLER_AUDIT_20260914.md), whose tested
source was `0ad244f6e129e9f57dacf546173321d7a58d80b7`. The immediate parent is
the diagnosis-only commit `3d97645140632887472b9c873862500cd6cc091a`. The commit
containing this report contains the tested implementation; the external
activity ledger records its full SHA and verified push.

## Algorithm changes

1. **Preserve the actual speed domain during uncertain terminal braking.**
   The brake now acts on `z_vx-rho_vx`, and terminal membership requires this
   lower endpoint to be nonnegative. Its held-input successor ratio is
   `exp(-(damping+1)*h)`. Radius feedback also enters the first terminal slew
   rows and the coupled nominal/error road-excursion budgets. Merely adding
   a lower-speed row to the old center-feedback law would not be invariant.
   The derivation is in [the information-state proof](../controller/INFORMATION_STATE_PCBF.md).
   Version-21 witnesses are rejected because their terminal law differs.
2. **Search useful passing paths and finite horizons.** Relative motion
   proposes the departure side and initial duration. Smooth left/right
   offset paths propose consistent cell normals. The declared generators,
   held controls and complete swept verification are retained. Positive
   safety-value diagnostics and rejected approximate solver results can try
   another proposal. Work time is shared among normal families. A fresh
   witness cannot postpone the active absolute completion deadline.
3. **Retain a currently observed approaching exterior target.** The exit row
   requires departure on the proposed passing side, preventing the default
   oncoming experiment's immediate release and unprepared readmission.
   Exterior approach detection uses current ego velocity. This is conditional
   on the stated sensing contract; complete absence and later reappearance
   still need the separate entry assumptions in the theory document.
4. **Budget fresh work before its first attempt.** Observed costs screen
   expensive horizons, and remaining work time reaches Clarabel. Expired
   work retains the verified witness. The bridge's fourth options entry is
   optional, preserving its three-entry interface.
5. **Audit truth independently in the experiment.** The pass condition now
   checks sampled lateral/heading/speed/lateral-velocity/yaw-rate domains and
   true ego/active-target containment in the published information sets.
   It checks the old ego successor before conditioning could discard truth.
   New admissions restart the descent obligation. Failed admissions retain
   their seed, bounds, target motion and confirmation range.

The executable safety value remains exactly zero. No uncertainty cap,
favorable measurement reset or elapsed-time target release was introduced.
Core controller source count remains 20.

## Final scenario campaign

Each trial calls `scripts/runExactStateRecursiveFeasibilityScenario.m` for
300 holds (30 s), with a 0.1 s hold, 16-stage performance window, 16 m
confirmation range, 8 m/s reference speed, road boundaries at +/-5 m and
0.25 m collision/road clearance. Finite admission can extend the window.
Each issued affine generator is integrated exactly with `expm` from truth.
An independent audit samples 11 points per hold; continuous safety relies
on the swept certificate. Successful trials include one final issued command
that is not executed.

Exact trials use default seed 20260912 and zero uncertainty/disturbances.
Noisy trials use seed 20260914 and componentwise bounds:

- Ego `[s,d,ePsi,vx,vy,r]`: `[.05,.05,.005,.05,.02,.005]`, in state units.
- Target `[px,py,vx,vy,ax,ay,psi,omega]`:
  `[.1,.1,.1,.1,.05,.05,.01,.01]`, in state units.
- Cartesian jerk amplitudes/bounds `[.1,.1] m/s^3`, yaw-acceleration
  amplitude/bound `.05 rad/s^2`, sinusoidal frequency `1 rad/s`.

The forced noisy case has the same complete noise/disturbance configuration
and rejects every fresh solve after admission.

| Trial | Executed holds | Admission stages | Extra separation (m) | Extra road margin (m) | Final speed (m/s) | Outcome |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| exact stationary | 300/300 | 48 | 0.001107285 | 1.118265 | 8 | Pass |
| exact oncoming | 300/300 | 56 | 0.005100649 | 1.302365 | 8 | Pass |
| exact crossing | 300/300 | 16 | 9.269858213 | 3.800000 | 8 | Pass |
| noisy stationary | 0/300 | — | — | — | — | Admission rejected |
| noisy oncoming | 0/300 | — | — | — | — | Admission rejected |
| noisy crossing | 300/300 | 16 | 9.276598066 | 3.790400 | 8.0000739 | Pass |
| forcedNoisy crossing | 300/300 | 16 | 9.276633151 | 3.797222 | 1.63449819e-06 | Pass |

Geometric margins are **in addition to** the required 0.25 m clearance.
Every passing trial has zero verified safety value and passes the independent
state-domain, truth-containment and input-slew audits. Containment comparisons
allow `1e-9` for arithmetic only; the physical separation and speed audits
require nonnegative margins.

The three exact scenes now complete at cruise; previously stationary admission
failed and oncoming control stopped at 2.7 s. The fully noisy forced backup
finishes at `1.6345e-6 m/s`, with no negative true speed, instead of reversing
at 6.7 s and failing at 9.1 s. In the final stationary trial, speed temporarily
falls to 4.95684 m/s before recovering to cruise.

## Timing and remaining limits

Final timings come from one fresh MATLAB batch after this task's regression
jobs finished. They are workstation observations, not worst-case bounds.
Frame timing includes input assembly and the controller, excluding plant
integration and the independent geometry audit. Overruns are recorded, but
the experiment continues at ideal sample times; delayed actuation is not
validated.

| Trial | Admission (s) | Maximum successor (s) | Frames above 100 ms | Confirmed release (s) |
| --- | ---: | ---: | ---: | ---: |
| exact stationary | 8.140131 | 0.231546 | 20 | 4.2 |
| exact oncoming | 1.401525 | 0.083539 | 1 | 5.0 |
| exact crossing | 0.063062 | 0.084261 | 0 | 0.7 |
| noisy stationary | 30.105813 | — | 1 | none |
| noisy oncoming | 33.946355 | — | 1 | none |
| noisy crossing | 0.061138 | 0.091718 | 0 | 0.7 |
| forcedNoisy crossing | 0.058991 | 0.056137 | 0 | 0.7 |

Initial admission retains a separate search budget and is not guaranteed
within 100 ms. Native time limits cannot bound MATLAB formulation, solver
setup, verification or operating-system scheduling. Keep `passed` and
`runtimeQualified` distinct.

The difficult noisy stationary and oncoming trials still obtain no finite
zero-violation certificate within the admission budget. They execute no
uncertified prefix; search failure is not proof of inevitable collision.
Additional probes of the estimator-fed straight and circular fixtures both
stop at admission with `collisionAvoidanceController:roadBoundaryCoverageGap`:
the supplied boundary charts do not cover the complete requested certificate.
That coverage requirement was retained. General bounded-noise admission,
exterior-track loss/re-entry, every-frame timing and nonlinear vehicle
transfer remain open. No physical Fiala/14-DOF vehicle guarantee is claimed.

## Verification and reproduction

Final coverage is **693/693 passing**, with zero failed or incomplete cases
across the full-suite run and the affected-class rerun. The initial suite had
682 passes, 11 failures and four incomplete cases. The affected-class rerun
passed 66/66. These updates remove stale pre-finite-completion expectations:
positive-safety commands are rejected, active exit deadlines stay fixed, and
absence requires a valid scan. The Frenet-radius fixture translates the whole
scene to preserve geometry; the pipeline-deadline fixture uses a target-free
admission to isolate timing. Earlier failed result files remain preserved.

The 14 new repair regressions cover all 64 terminal-box vertices over 600
holds (60 s), 300-hold speed-only and full-ego-error forced backups, passing
admission, retained approaching targets, an exterior rear approach, expired
frame/native budgets, false-pass rejection and old-witness incompatibility.
The independent vertex integrations retain speed/velocity/slew constraints
within floating-point arithmetic allowances; noisy scenario audits require
nonnegative sampled true speed directly.

Factory Code Analyzer checked all 19 changed MATLAB files with zero findings.
The source-budget regression passes. The native bridge was rebuilt with
`addpath('scripts'); buildAvoidanceSocpSolver`; independent load/execute smoke
tests and the optional budget tests pass. Environment: MATLAB R2026a Update 3,
Clarabel.cpp `0de6259a3edfd5cc041ec42b2148599ce63e73cb`, Clarabel.rs
`25540f559592068d0c8a80e46ded1b21760212a1`. Existing R2026a post-link inspection
messages were handled by the build script's documented load-and-execute check.
Generated binaries, solver dependencies and unrelated user files are excluded
from the project commit.

Run from the repository root:

```matlab
results = runtests('tests');
assertSuccess(results);
addpath('scripts');
r = runExactStateRecursiveFeasibilityScenario( ...
    Scenario="stationary", SampleCount=300); % also "oncoming", "crossing"
```

Reproduce the fully noisy terminal backup:

```matlab
r = runExactStateRecursiveFeasibilityScenario( ...
    Scenario="crossing", SampleCount=300, Seed=20260914, ...
    EgoErrorBound=[.05;.05;.005;.05;.02;.005], ...
    TargetErrorBound=[.1;.1;.1;.1;.05;.05;.01;.01], ...
    TargetJerkAmplitude=[.1;.1], TargetYawAccelerationAmplitude=.05, ...
    FailAfterAdmission=true);
```

Original MAT/JSON traces, commands, build/test logs, analysis results,
admission diagnostics and the commit record remain at
`/home/zai/.cache/collisionAvoidance/controller-repair-20260914`.
`runRepairCampaign.m` records all seven final configurations. Intermediate
experiments are preserved separately and do not replace the final campaign.
