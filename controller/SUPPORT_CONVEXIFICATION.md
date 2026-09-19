# Single convexification on the shifted nominal

Controller format 35, September 18, 2026.

This remains the default `fixedNormal` baseline. The selectable format-36
`jointSupport` research policy and its admission/continuation proof are described
in [JOINT_SUPPORT_CERTIFICATES.md](JOINT_SUPPORT_CERTIFICATES.md).

## Online algorithm

The controller shifts the previous accepted input/state prediction by one
100 ms hold. During an unchanged active encounter its remaining horizon
counts down to the stored absolute exit deadline. It does not reset that
deadline or discard the terminal continuation. A newly admitted target uses
the shifted old controls as a numerical seed, without treating them as a
certificate for the new target. The seed repeats the last stored input once;
if a longer horizon is needed, its remaining inputs are the reference trim.
Without a previous solution, initialization uses cruise continuation.

At each predicted node, evaluate the nominal ego Cartesian position and yaw
from the exact reference geometry, and evaluate the nominal target rectangle
at the same time. Obtain one signed configuration-obstacle distance normal.
Fix all these normals, construct the complete hard-safety/soft-CLF SOCP, and
call the trajectory solver once. There is no maneuver sector, branch
enumeration, direction-candidate scoring, feasibility restoration, horizon
retry after infeasibility, or working-set sequence of native solves.

The geometric query has deterministic polygon tie-breaking. This is a local
geometric convention; it does not impose a common passing side across nodes
or prescribe any ego trajectory. Different nodes can return incompatible
half-spaces. A finite normal at overlap is not a feasibility guarantee.

## Relation to Li et al. (2023)

Guoqiang Li, Xudong Zhang, Hongliang Guo, Basilio Lenzo and Ningyuan Guo,
*Real-Time Optimal Trajectory Planning for Autonomous Driving with Collision
Avoidance Using Convex Optimization*, Automotive Innovation 6, 481--491,
[DOI 10.1007/s42154-023-00222-7](https://doi.org/10.1007/s42154-023-00222-7),
Sections 3.1--3.2, equations (9)--(13).

The adopted architecture computes geometry from a nominal trajectory and
then fixes it for the trajectory optimization. For a configuration obstacle
C, outside C the analytic closest-point normal supplies the same supporting
hyperplane as the Euclidean distance dual. No per-node conic solve is needed
for these rectangles. At overlap/contact the implementation uses the analytic
signed-distance support normal; ordinary unsigned-distance duality can return
zero there. This is an explicit extension of the paper's direction step.

Equation (13) of the paper permits collision slack. This implementation retains
hard collision rows and only the existing CLF slack. It also retains the
bicycle study plant, rectangle uncertainty charges, curved pose domains and
finite-exit/invariant-terminal certificate. It is therefore an adaptation of
the convexification sequence, not a reproduction of every paper assumption
or its reported performance.

For any unit n, the inequality

    n' * pE >= support(C, n) + clearance

certifies separation. The anchor need not satisfy it for the inequality to be
constructible. Existence of a solution satisfying all such inequalities,
actuator limits and completion conditions is a separate question.

## Curved geometry and hard verification

Exact nominal curved poses select directions. Local affine position/yaw maps,
certified remainder charges and hard local pose domains construct convex
rows. Accepted active continuations retain their original charts and model
generators. The finite exit direction/deadline and terminal cones are retained;
normal recomputation changes collision geometry only. Fresh straight-reference
exit proposals retain the existing relative-motion completion calculation;
curved exit proposals use the exact nominal endpoint. Neither is enumerated.

All current safety rows certify hold nodes, not the inter-node continuous
motion. See [NODE_SAMPLED_CERTIFICATE.md](NODE_SAMPLED_CERTIFICATE.md).
The complete original physical rows, terminal cones and sampled CLF cone are
independently checked before returning a command. No failed or uncertified
solve executes a stored control or terminal feedback policy.

## Recursive feasibility scope

The stored prediction and invariant terminal construction still certify a
finite continuation. However, recomputing directions may exclude its shifted
witness because nominal distance and robust affine support bounds are
different criteria. The controller always selects the newly computed normals,
as requested; it does not fall back to the old geometry when inclusion fails.

`supportGeometry.witnessPreserved` and `shiftedWitnessContained` report a
complete current-program inclusion check. `inheritedPredictionFamily` reports
retained generators and charts. `inheritedFeasibleFamily` is true only when
the shifted witness also satisfies the new complete program. None of these
observations proves that every future direction update preserves feasibility.

Consequently, `recursiveFeasibilityGuaranteed` is false while an encounter is
active. Target-free witness-preserving horizon renewal and invariant one-hold
optimization retain their existing conditional guarantee. The conditional
active-encounter theorem in [TERMINAL_CBF_PROOF.md](TERMINAL_CBF_PROOF.md)
requires inclusion at every future update as an additional premise. This
implementation does not establish that premise automatically. A failed
single convexification is not evidence of unavoidable physical collision.
