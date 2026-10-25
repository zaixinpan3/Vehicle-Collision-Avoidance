# Unified observer to safety formulation

Prepared October 25, 2026, America/Chicago. Baseline repository commit:
`1975206c2d54e37ce25c6349c05fec3f524ba9a8`.

The mathematical algorithm now has one information-set interface between the
observer and predictive safety controller. A current pointwise enclosure
initializes physical uncertainty. Candidate held inputs propagate that
uncertainty, and the next measurement conditions the retained first successor.
Future observer simulation is required only when the predicted policy itself
depends on future observations. Both representations use the same shift proof.

## Algorithm and proof changes

The canonical derivation is
[controller theory, Section 7](../controller/CONTROLLER_THEORY.md#recursive-feasibility).
Its core objects are the current enclosure, retained physical successor,
signed minimum gap over a complete input hold, and a common admissible
terminal continuation. The two PCBF/CLF priorities are retained.

The update `I_next = R_first intersect Q_next` contains the truth and preserves
the inclusion needed by the shifted-plan proof. Moving raw observer balls need
not be nested. Dropout supplies no new observation restriction. An empty
intersection invalidates the certificate instead of justifying a reset to the
latest estimate. The current error set is not reinjected or reset at future
nominal centers. Persistent model uncertainty remains inside physical prediction.

The primary value is the minimum sum of nonnegative hold deficits. Under sound
images, compatible uncertainty, an initially feasible plan, retained witnesses
and terminal closure, it satisfies

`V_star(I_next) <= V_star(I_now) - first_hold_deficit`.

Zero value implies invariant zero-deficit safety over the times actually
certified. Positive deficits remain relaxation. Summability does not imply
finite-time entry into the zero-value set. A finite unbounded-slack CLF row
preserves a primary feasible point, but does not by itself establish recovery.
The older discrete-CLF example in Section 3 was aligned with the implemented
continuous derivative residual.

Directional support functions connect the observer set to signed separating
gaps. Zero ordinary distance cannot distinguish contact from interior overlap.
Whole-hold bounds or valid motion reserves are necessary for continuous safety;
nodes and midpoints alone are insufficient. The optional joint observer and
future-feedback construction remains in Section 6 as a lift of the same theorem.
The physical held-input core may be more conservative, and makes no claim of
robust feasibility for every initially separated encounter.

## Executable research specialization

[verifyUnifiedObserverSafety.m](../scripts/verifyUnifiedObserverSafety.m)
implements the physical core on a longitudinal model with a constant-speed
moving target and forward-only ego braking with absorbing rest. Each root has
four interval coordinates: ego position/speed and target position/speed.
The interval operator is monotone. It can lose correlation while remaining an
outer enclosure. The optimizer does not receive the simulated true state.

The input alphabet is `{0, 2, 4}` m/s^2, the command slew bound is 10 m/s^3,
and the hold is 0.2 s. The usual horizon has four holds and a two-hold slack
prefix. All 81 sequences are enumerated before selecting the minimum deficit
and then the minimum squared speed/braking cost. One-hold and parked cases
are also exercised. The summed longitudinal half-lengths are 4.8 m and the
additional design clearance is zero. A common maximum-braking terminal rule
must satisfy the input-memory constraint; parked states may retain their input.

For each hold, the worst gap is attained at the ego upper position/speed and
target lower position/speed. Its only possible interior minimum occurs when
their speeds cross. Endpoints and that analytic critical time therefore give
the whole-hold minimum; no dense time sampling supplies the certificate.
After ego rest, a nonnegative-speed target cannot reduce this gap.

Observation centers alternate by `(0.2 m, -0.15 m/s, -0.2 m, 0.15 m/s)`
and its negative. Their component radii are `(0.25 m, 0.2 m/s, 0.25 m,
0.2 m/s)`. Every observation contains truth, including when its raw box is
not a subset of the old prediction. The safe scenario omits observations
after the fourth and fifth executed holds. No random generator is used.

| Check | Result |
| --- | --- |
| Moving-target safe run, 12 holds | All primary values zero; minimum true gap 0.56 m; 10 raw nonnested observation boxes; two dropouts; all shifted witnesses valid. |
| Initially overlapping recovery example, six holds | Initial robust deficit 0.4 m, then zero; initial true gap -0.2 m. This is recovery from an unsafe state, not a safe trajectory from initialization. |
| One-hold horizon | All four shifted/appended plans valid with zero deficit. |
| Parked uncertain state | Positive position width remains; all four plans valid with zero deficit, including two dropouts. |
| Hard braking and slew | Every issued command is in [0,4] m/s^2 and every change is at most 2 m/s^2 per hold. A separated but input-limited case has no feasible plan. |
| Between-sample collision | Minimum of start/midpoint/end gaps is +0.004 m; analytic whole-hold minimum is -0.001 m. |
| Interior overlap at zero design clearance | Signed margin -0.8 m; overlap is not declared safe. |
| Disjoint observation and prediction | Explicit inconsistent-information error. |

The pointwise-observer check also uses `P=diag(1,4)` and
`A=[-1,-20;5,-1]`. The matrix `A'*P+P*A+I=diag(-1,-7)` is negative
definite. With zero disturbance injection, initial error `(0,1)`, and a
declared unit noise bound and attenuation level, the exact error norm reaches
1.7092719983 at `t=pi/20`. Thus the energy inequality alone does not establish
the unit Euclidean ball; a radius of 2 from the metric comparison contains
this trajectory. This checks a general inference, not the vehicle experiment
reported by Quan and colleagues.

## Validation and limits

Executed with MATLAB R2025b from the repository root:

```matlab
files = {'tests/unifiedObserverSafetyTest.m', ...
    'tests/informationSetRecursiveFeasibilityTest.m', ...
    'tests/nrmmControllerErrorBoundsTest.m', ...
    'tests/rectangleRotationSupportTest.m'};
results = runtests(files); assertSuccess(results)
```

All 64 tests passed, including 11 new behavior checks. Those 11 were rerun
successfully after a property rename and addition of configuration metadata.
Code Analyzer reports only its existing missing-settings fallback; there are
no remaining source diagnostics for the two new MATLAB files.

`python3 -m unittest discover -s tests -p jointMismatchDerivationsTest.py`
passed seven existing exact-arithmetic checks of the optional joint extension.
`git diff --check` passed. Compact results are retained in
[the verification JSON](UNIFIED_OBSERVER_SAFETY_20261025.json).

This work changes the canonical mathematical algorithm and supplies an
executable finite-input specialization. It does not change the production
Fiala optimizer, NRMM gains, manuscript or historical vehicle campaign.
The source uses ordinary floating point and small feasibility tolerances;
there is no directed-rounding claim. Transferring the core to the vehicle
runtime still requires sound nonlinear hold images, retained posterior sets
and robust terminal membership for that vehicle model. No full-vehicle
performance or collision-avoidance certificate is inferred from these tests.

The literature motivation is Tezuka and Nakamura (2020), Sections 4--6,
[DOI](https://doi.org/10.1016/j.ifacol.2021.04.192), and Quan et al. (2025),
Section III, [DOI](https://doi.org/10.1109/LCSYS.2025.3581497).
The information-set formulation is a project synthesis of those observer-error
ideas with the predictive-slack shift construction; no general novelty claim
or unchanged application of either paper's theorem is made.
