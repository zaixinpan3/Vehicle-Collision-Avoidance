# Cheng VFFM as a trajectory initializer: source review and admission experiment

Date: September 21, 2026. Controller source revision:
`98cdf9df909bd5c4210ba05d9fe0649ad78fc432`.
[All numerical outcomes and artifact hashes](CHENG_FLUID_INITIALIZATION_REVIEW_20260921.json).
AI-assisted source examination, mathematical derivation and numerical investigation
were used. This is a bounded feasibility study, not a production replacement.

## Finding

Cheng's modified virtual-fluid construction can supply reference paths and
separation directions for the existing fixed-direction convex trajectory
optimizer. An offline prototype obtains hard-verified initial plans for six
mirrored circular-crossing fixtures using a shared selected path-length scale.
The result establishes a concrete integration example. It does not establish
that the initial path itself is safe, that the initializer generalizes to all
encounters, or that it improves complete-frame runtime.

No production controller or configuration is changed. The experimental driver
is [analyzeChengFluidInitialization.m](../scripts/analyzeChengFluidInitialization.m).

## What the paper actually constructs

Source: Shuo Cheng, Liang Li, Yong-Gang Liu, Wei-Bing Li and Hong-Qiang Guo,
*Virtual Fluid-Flow-Model-Based Lane-Keeping Integrated With Collision Avoidance
Control System Design for Autonomous Vehicles*, IEEE Transactions on Intelligent
Transportation Systems 22(10), 6232–6241 (2021),
[DOI 10.1109/TITS.2020.2990211](https://doi.org/10.1109/TITS.2020.2990211).
The local publisher PDF was read; page 6237 was rendered to verify Eqs. (34)–(36).
Crossref and Semantic Scholar DOI records independently match the title and year.
The first page reports online publication in May 2020 and issue publication in
October 2021; the DOI year therefore does not contradict the 2021 citation.

Section II.B, Eqs. (14)–(20), introduces virtual fluid equations, a stream
function and a potential function. Section III.A obtains a parabolic channel
velocity profile and selects the lane center through its velocity extremum.
Section III.B combines uniform and doublet flow around a circular obstacle.
For a cylinder of radius a centered at the origin, Eq. (32) gives

\[
\psi(x,y)=V_\infty y\left(1-\frac{a^2}{x^2+y^2}\right).
\]

Its constant-value curves provide geometric streamlines. However, the final
implementation described in Eqs. (34)–(36) uses a **modified function and a
velocity-extremum path rule**, rather than simply integrating the original
cylinder field. In the paper's notation,

\[
\phi_{mo}=\tfrac12y^2x-y\int l_{obs}
  \exp\left[-\frac{(x-x_{obs})^2}{2l_{lon}^2}\right]dx,
\]
\[
u_{x,mo}=\frac{\partial\phi_{mo}}{\partial x}
 =\tfrac12y^2-y l_{obs}
  \exp\left[-\frac{(x-x_{obs})^2}{2l_{lon}^2}\right].
\]

Applying Eq. (34)'s extremum condition to Eq. (36) yields

\[
\frac{\partial u_{x,mo}}{\partial y}=0
\quad\Longrightarrow\quad
\bar y(x)=l_{obs}\exp\left[-\frac{(x-x_{obs})^2}{2l_{lon}^2}\right].
\]

This last expression is a direct derivation from the printed equations, not a
separately numbered equation quoted from the paper. It is a Gaussian lateral
excursion. The derivative vanishes at a minimum of the scalar Eq. (36), since
its second derivative in y is 1; the paper's general extremum terminology is
retained. The authors call Eq. (35) a modified stream function, while its
x-derivative follows their potential-function convention. The prototype uses
the explicit algebraic path rule, not a claim of physical fluid equivalence.

The path amplitude and longitudinal scale still need selection: fluid-inspired
initialization does not eliminate these design quantities. It does eliminate
the necessity of the current 16-direction dictionary and 8-cell amplitude
search when analytic geometry supplies normals from the constructed reference.
No numerical PDE solve is needed to evaluate this Gaussian path.

## Limits relevant to this project

The paper's Section IV simulation has four obstacles located along the ego lane;
the bus experiment uses one obstacle bus. Section V explicitly places moving
obstacles among future work. Its results therefore do not establish behavior
for our moving transverse target. The bus experiment also permits the planned
path to leave the ego lane around the obstacle; it is not evidence of a hard
lane-boundary guarantee.

Several adaptations are necessary for this controller:

- A moving obstacle needs a time-dependent conflict assessment. A path around
  its current position alone need not avoid its future position. If a
  time-varying fluid field is used instead, an instantaneous streamline and the
  pathline obtained from `dp/dt = u(p,t)` are different constructions.
- A curved road needs consistent Frenet-to-Cartesian geometry and body yaw.
  A straight-road Gaussian is not directly a curved-road vehicle state.
- A geometric path needs timing, model-compatible states and controls, actuator
  amplitude/slew checks, pose-domain checks and the required terminal state.
  Smoothness by itself supplies none of those certificates.
- A perfectly centered upstream initial point in symmetric cylinder flow has
  zero transverse velocity and approaches the front stagnation point. Direct
  field integration needs a side-selection mechanism or symmetry breaking;
  flow geometry alone does not choose left versus right.

The stagnation observation follows directly by differentiating Eq. (32)'s
potential: on y=0 outside the cylinder, the transverse velocity is zero and
longitudinal velocity is `V_inf * (1 - a^2/x^2)`. These are project deductions,
not reported additional experiments from Cheng et al.

## Executed integration experiment

The script builds the same initial public controller problem directly through
input parsing, encounter preparation and problem formulation. It **does not
call the production scalar initializer or use its optimized trajectory**.
Only initial admission is examined; no control is executed.

For each fixture:

1. Propagate the nominal affine cruise plan. Use target predictions and its
   existing conservative support records to identify first and last potentially
   conflicting nodes. All six selected fixtures have nodes 29–46 at 50 ms.
2. Center the Gaussian at the nominal station corresponding to the midpoint of
   that interval, about 15.001 m. Set the base longitudinal scale to the larger
   of 3 m and half the nominal conflict-interval station span, about 3.400 m.
3. Test lateral amplitudes of either sign, magnitude 3.55 m: ego half-width
   0.95 m plus the transverse target half-length 2.4 m plus a 0.2 m seed-shape
   allowance. This allowance does not modify the final hard collision margin.
4. Set the curved reference offset to the Gaussian. Its reference heading
   perturbation is `atan2(d'(s), 1 - kappa*d(s))`, added to nominal body yaw.
   Fit lateral position and heading to the affine prediction using
   equality-constrained least squares, retaining the nominal terminal state
   and final control. The fit includes the existing actuator-effort and
   input-difference metric. It does not enforce all inequalities.
5. Obtain one continuous signed-distance support normal per collision or exit
   record from the fitted rectangle geometry. Fix those normals, solve the
   existing full trajectory SOCP, then run the unchanged physical/support/
   terminal/CLF verifier. No collision slack is added.

The curvature set is ±0.008, ±0.010 and ±0.012 per meter, with mirror-consistent
crossing direction; radius is approximately 125, 100 and 83.3 m. Ego speed is
8 m/s, target transverse speed 4 m/s, target station 15 m, initial lateral
offset 7.5 m. Exact sensing and the declared held affine model are used.
The horizon has 96 holds (4.8 s), with a diagnostic 30 s computation budget.
All inputs/configurations, seeds, selected normals and returned decisions are
saved in the external MAT artifact. The experiment is deterministic; no random
sampling or new random seed is used.

## Results and sensitivity

The study retains **all 60 attempts**: six curvatures, two sides and five
longitudinal-scale factors. There is one full SOCP per candidate. The table is
an offline parameter study, not an online ten-solve admission policy.

| Width factor | Candidates accepted / 12 | Fixtures with a passing side / 6 |
| --- | ---: | ---: |
| 0.8 | 2 | 2 |
| 1.0 | 4 | 4 |
| 1.2 | 6 | 6 |
| 1.5 | 6 | 6 |
| 2.0 | 2 | 2 |

At factors 1.2 and 1.5, the passing side is opposite the target's transverse
motion in each mirror fixture. Factor 1.2 corresponds to about 4.080 m width.
At this selected factor, all six full plans pass every original certificate;
the least negative maximum support residual is approximately -8.856e-5 m.
The selected fitting terminal-state/input residual is below 1.7e-12.

All six selected **initial fitted plans violate physical rows and collision
support conditions**. Their maximum support residuals range from about 1.336 m
to 1.586 m. The final fixed-direction optimization repairs these violations;
no seed itself is certified or executed. Thus the result supports using the
construction as an initialization, not advertising it as a safe controller.
The global touching majorant is valid even at an infeasible anchor.

Both width and side were assessed on the same six fixtures. There is no
independent held-out validation or evidence that a single selected factor
works for all encounters. Small or large width can both fail. The Gaussian's
raw tails also need not match initial and terminal conditions exactly, which
is why the model/terminal fit is a separate explicit step.

The script completed all 60 attempts. MATLAB Code Analyzer reported no issues.
An independent comparison with the public controller gives identical occupied-set
certificates and base matrices/vectors agreeing within 2.3e-13. An initial
bitwise-equality assertion failed across the single-thread batch and MCP
sessions; the recorded numerical comparison uses a 1e-10 tolerance.
Aggregation checks retain failures and verify all accepted support and physical
residuals are nonpositive. No full regression suite was rerun because production
sources are unchanged. No warm timing benchmark, closed-loop simulation,
inter-node footprint audit, uncertainty robustness test, multi-target study or
nonlinear plant validation was performed in this task. Prior controller timing
and clearance results must not be attributed to this new prototype.

## Integration decision

The evidence supports the following candidate-generation interface:

`predicted encounter -> fluid-inspired reference -> model-compatible seed ->
analytic separation normals -> fixed-normal full trajectory optimization ->
original hard verification`.

Cheng supplies the reference-shape construction; Li's two-stage principle
supplies the separation between direction selection and final convex trajectory
optimization. The current hard verifier remains the sole admission authority.
A production replacement still needs a scene-derived side/width policy,
initial/terminal consistency, wider scene coverage, closed-loop and warmed
runtime validation. In particular, least-squares fitting and geometric normal
queries have a cost; reduced angular enumeration alone does not prove a speedup.

## Reproduction

From the repository root:

```matlab
addpath('scripts');
analyzeChengFluidInitialization( ...
    '/home/zai/.cache/collisionAvoidance/cheng-fluid-initialization-20260921');
```

This was run using `matlab -singleCompThread -batch`. Raw results, preliminary
12- and 48-attempt probes, bibliography responses and logs remain in that
external cache. The report JSON records their hashes and the source PDF hash.
The original PDF remains under `reference/`; it is not copied into a new
literature or archive directory. No new Evidence ID is assigned.
