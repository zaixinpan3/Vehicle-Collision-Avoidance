# Feedback (tube) prediction of the ego uncertainty — design proposal

Status: design proposal, 2026-09-23. **Not implemented.** It addresses the
open-loop growth of the ego uncertainty found in
[ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923.md](../report/ADMISSION_INFEASIBILITY_DIAGNOSIS_20260923.md).

## 1. The problem

The controller plans an input sequence `u_0..u_{N-1}` and certifies it for
every true initial state in the estimator box `x_0 - xhat_0 in H_0 = [-eps, eps]`.
The prediction assumes this whole sequence is executed **open loop**, with no
later measurement, so the box is propagated by
`rho_{k+1} = |A_k| rho_k` (`ltvBicycleModel.finitePredict`). Heading and speed
errors integrate into lateral and station errors, and over a 4.8 s encounter
horizon the ten-fold-uncertainty box grows from 0.1 m to 0.52-0.74 m
laterally. Every node constraint, the exit row and the terminal cone must hold
for that whole box. In the three ten-fold cases the box no longer fits the
terminal set, even with the state exactly on the reference.

In execution the controller re-plans every hold from a new estimate whose
error stays within the declared bound. The open-loop box describes an
execution that never happens.

## 2. Formulation

Plan a **feedback policy** instead of an input sequence:

    u_k = v_k + K_k (xhat_k - z_k),        z_{k+1} = A_k z_k + B_k v_k + c_k,

with nominal inputs `v_k` as the decision variables (same count as today),
nominal states `z_k`, and fixed gains `K_k`. `xhat_k` is the estimate that will
be available at hold k.

Estimator contract (new, explicit): at every future hold,
`x_k - xhat_k in H_k = [-epsbar, epsbar]`, with `epsbar` the declared
per-hold estimator bound. The plant contract is unchanged (declared held affine
plant).

The deviation `e_k = x_k - z_k` obeys, with `eta_k = xhat_k - x_k`:

    e_{k+1} = (A_k + B_k K_k) e_k + B_k K_k eta_k,

so `e_k` lies in a zonotope `E_k` with generator matrix

    G_{k+1} = [ (A_k + B_k K_k) G_k ,  B_k K_k diag(epsbar) ].

`E_k` does not depend on the decision variables, so it is computed once per
frame before the solve. With a stabilizing `K`, `E_k` stays bounded instead of
growing with the horizon: the per-hold measurement error is fed back and
re-injected, but it is not integrated open loop.

## 3. Constraint tightening

Every tightening is a constant, because the separation directions are fixed
before the solve. Nothing new enters the SOCP except different constants.

| Constraint | Current tightening | Feedback tightening |
| --- | --- | --- |
| collision record at node k | generators `positionMap*diag(rho_k)`, yaw radius `\|yawRow\| rho_k` | generators `positionMap*G_k`, yaw radius `sum \|yawRow G_k\|` |
| exit row | `\|row\| rho_N` | `sum \|row G_N\|` |
| terminal modal cone | `\|modal\| rho_N` | `sum_j \|modal g_j\|` over the columns of `G_N` |
| actuator amplitude | none (inputs exact) | `\|v_k\| <= umax - sum \|K_k [G_k, diag(epsbar)]\|` |
| slew | none | support of `K_k(e_k+eta_k) - K_{k-1}(e_{k-1}+eta_{k-1})`, written in `G_{k-1}`, `eta_{k-1}`, `eta_k` |

The cost of feedback is the actuator authority it reserves for correcting the
measurement error.

## 4. Recursive feasibility

Fresh admission: `z_0 = xhat_0`, `E_0 = H_0`.

To keep the carried set exact, the prediction replaces `E_1` by its interval
hull (one box, at node 1 only). Later nodes keep full zonotopes.

Inherited frame at the next hold: the nominal is **not** reset to the new
estimate. Set `z'_0 = z_1` and
`E'_0 = E_1 ∩ ((xhat'_0 - z_1) + H'_0)`, which is an intersection of two
boxes and therefore exact and contained in `E_1`. With the same gains shifted,
the candidate `v'_j = v_{j+1}` gives `z'_j = z_{j+1}` and `E'_j ⊆ E_{j+1}`. Every
constraint of the previous certificate for nodes `2..N` therefore still holds,
and the shifted policy is a feasible point of the new SOCP. The executed input
`u'_0 = v'_0 + K_1 (xhat'_0 - z_1)` includes the known correction. This is the
output-feedback tube argument of Mayne et al. (2006) in the form this
controller already uses for conditioning: successor set intersected with the
measurement box.

The terminal continuation must also hold. The terminal set is already robustly
invariant under the terminal feedback law with per-hold measurement noise
(`holdNoise`). Choosing `K_N` equal to that law keeps appended steps inside it;
this has to be verified in the implementation.

## 5. Evidence (offline, captured frames)

Computed on the frames captured in the infeasibility diagnosis. `K` is the
existing cruise LQR gain (`terminal.feedback`), and the ten-fold estimator box
is used as `epsbar` at every hold. The margin is the terminal modal margin with
the nominal exactly on the reference, i.e. whether the error set alone fits the
terminal set:

| Case | Open loop | LQR K | 0.5 K | Feedback input use (K): steering / braking ratio |
| --- | --- | --- | --- | --- |
| 39 straight, x10 | -0.029 | **+0.153** | +0.131 | 0.080 rad / 0.37 |
| 42 curved 0.01, x10 | -0.064 | **+0.136** | +0.109 | 0.103 rad / 0.39 |
| 53 curved 0.01, x3 | +0.103 | +0.162 | +0.154 | 0.031 rad / 0.12 |
| curved 0.01, x1 | +0.150 | +0.170 | +0.167 | 0.010 rad / 0.04 |

Terminal lateral radius (x10, straight) falls from 0.515 m to 0.210 m.
Heading radius rises from 0.011 to 0.035 rad because the measurement error is
re-injected every hold. With exact state feedback (no noise) the error set
converges to zero. Zonotope supports matter: the interval-hull measure the
controller uses today would report -0.178 for the same LQR tube.

## 6. Not addressed

- **Target uncertainty.** In case 54 the target's declared reachable set grows
  to 18 m radius. The ego policy does not feed back future target
  measurements, so this growth remains.
- **Model mismatch.** The PassVeh14DOF plant violates the zero-residual
  contract; nothing here changes that.
- **Gain design.** A time-varying LQR along the anchor schedule may use less
  actuator authority than the constant cruise gain; a gain scale is a
  configuration trade-off between tube size and reserved authority.

## 7. Implementation outline

1. `config`: `tube.gainScale` and the future estimator bound `epsbar`
   (default: the current declared bound).
2. `ltvBicycleModel.finitePredict`: gains `K_k` and generators `G_k`, with the
   node-1 interval hull; keep the box `rho_k` as a derived quantity for
   diagnostics.
3. `avoidanceSafetyGeometry.localJointRecord`: generators and yaw radius from
   `G_k`.
4. `hardEncounterBarrier`: terminal cone and exit supports from `G_N`; carried
   nominal `z_1`, `E_1` and gains; conditioning `E'_0 = E_1 ∩ measurement box`.
5. `formulateAvoidanceProblem`: actuator and slew tightening; inherited frames
   start from the carried nominal.
6. `collisionAvoidanceController`: executed input `v_0 + K_0 (xhat_0 - z_0)`;
   controller state stores nominal, tube and gains.
7. Tests for tube containment (sampled noise sequences stay inside `E_k`),
   recursive feasibility of the shifted policy, and actuator/slew tightening;
   then rerun the recursion campaign and the 118-case sweep.
