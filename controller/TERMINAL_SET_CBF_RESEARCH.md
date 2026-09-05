# Terminal continuation certificate

The implemented terminal condition is described in
[PCBF_CLF_ARCHITECTURE.md](PCBF_CLF_ARCHITECTURE.md). This note records the
specific model-design decision and its limits.

The former proof-side tail froze lateral states and used
`sNext = s + Ts*v + 0.5*Ts^2*a`, while the executable head used a dynamic
forward-Euler bicycle with `sNext = s + Ts*v` on a straight road. Shifting
braking acceleration into the head moved the newly predicted station forward
by `-0.5*Ts^2*a`. Bounds on heading, lateral speed and yaw rate did not prove
that the kinematic lateral certificate contained the dynamic handoff.

The replacement uses steering and acceleration decisions, the same scheduled
six-state bicycle with exact held-input affine integration, and physical
request rows at every stage. The declared longitudinal input gain is the
same across this boundary. The schedule's speed
may reach zero; only tire-force denominators retain a positive floor. For a
zero-speed schedule, arbitrary admitted station/lateral offset/heading and
zero velocities form an equilibrium under zero steering and cancellation of
the declared constant longitudinal bias: the resting command is
`[0; -bias/longitudinalInputGain]`. Three exact terminal velocity
equalities and a fixed last input close the continuation. The shifted tail
therefore enters the head without changing its dynamics or constraints.
The conic solver now parameterizes these equalities and fixed inputs
algebraically, rather than relying on its scaled stopping test to enforce
absolute terminal-rest acceptance. A station domain can cross polyline
vertices using certified chart-error bounds; a resting endpoint is no longer
pinned to one 0.1 m route segment or separated from its predecessor by an
artificial endpoint guard.

`brakingSchedule` derives a finite continuation length and constructs an
initial reference; it is not a backup controller or a safety proof. The
optimizer may use steering throughout this continuation, which avoids the
operating-domain restriction of a prescribed braking-only backup policy.

The terminal target halfspace uses support over the entire future target
trajectory, not a finite horizon or only the final predicted target pose.
The target circumradius covers future rotation. The resting ego's orientation
is unchanged, so a directional rectangle support with a certified heading
bound replaces its former circumradius. Persistent directional target motion
uncertainty currently makes that direction inadmissible. A feedback tube and
robust invariant terminal set would be needed for disturbed ego dynamics;
the trajectory implementation rejects nonzero ego/model error bounds.

Halfspace separability is conservative: a target orbit can surround a safe
resting ego without meeting any separating halfspace. This construction does
not equate certificate infeasibility with physical inevitability of collision.
The rest proof is for the declared discrete model in exact arithmetic.
Numerical terminal residuals, model mismatch, changing observations and
intersample motion are separate validation obligations.
