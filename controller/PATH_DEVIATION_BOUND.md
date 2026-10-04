# Finite-prediction lateral-deviation constraint

`controller.maximumLateralDeviationMeters` defaults to 10 m. It bounds the
absolute lateral projection of the ego model's center position onto the given
path, independently of the road's displayed width and the collision margin.
`Inf` disables this optional constraint for unconstrained research comparisons.
It adds no lateral cost, CLF, controller mode or actuator limit.

Both the PCBF and CLF problems impose the bound at hold starts, model midpoints
and the endpoint, over the prefix and completion tail. No safety slack relaxes
it. Every node uses `L`. A measured state already outside the bound cannot be made
feasible by changing future controls. Primary zero-correction detection and
inherited-anchor residuals include every corridor cone, not just the terminal
membership cone.

For a straight path, the two lateral half-planes are exact. For a circle with
center `c` and radius `R`, the corridor is the annulus

    max(0,R-L) <= norm(p-c) <= R+L.

With radial unit vector `n` taken from the current anchor, the convex subproblem
uses

    n'*(p-c) >= max(0,R-L),
    norm(p-c) <= R+L.

The outer disk is exact; the inner half-plane is a sufficient approximation of
the annulus exterior. This avoids treating a curved path as a global straight
strip or approximating both sides by unconstrained tangent planes. The same
single trajectory model supplies the affine positions in these rows/cones.

Rows and disks already implied by the correction box are removed exactly. If
`p=p_bar+M*z` and `abs(z)<=b`, then `abs(M*z)<=abs(M)*b`. Consequently a linear
row whose maximum over the box satisfies its bound is redundant, and the disk
is redundant when

    norm(abs(p_bar-c) + abs(M)*b) <= R+L.

The box used for this argument includes the existing maximum input-box
expansion, so later expansion cannot invalidate removal. State and actuator
limits only shrink this box further.

The potential-field guidance velocity is projected onto

    (-L-e_y)/T <= v_y_guidance <= (L-e_y)/T,

where `T` is the existing nominal guidance lookahead. This is a seed-generation
condition for a first-order point guidance law, not a bicycle safety proof.
The default completion is four seconds, with the last 1.5 seconds devoted to
intrinsic terminal settling; the remaining guidance interval allows the seed
to bend back before settling on a freely placed straight continuation. The
initial input correction scale is 0.125; normal frames still inherit their
previous accepted scale. These changes keep a single initialization/model per
attempt. The single CLF and the two-stage/inherited-budget objectives are
unchanged.

The issued plan is not replayed online, so the corridor holds for the affine
prediction only. Offline ODE45 validation separately reports the maximum across
31 samples per issued hold; this sampled check is not a continuous-plant
certificate. The free-pose invariant terminal core retains its original
intrinsic-dynamics role; it is not proved invariant inside this new path
corridor. Thus no new infinite-horizon or recursive-feasibility claim for the
corridor is made. A failed optimization still reports failure rather than
issuing a control from an unaccepted plan.
