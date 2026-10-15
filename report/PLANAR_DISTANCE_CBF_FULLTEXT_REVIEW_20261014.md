# Full-text review of distance barriers for constrained planar vehicles

Prepared on October 14, 2026. The initial review and coupling addendum were read-only discussion records. The method-description addendum below supports the subsequent Introduction revision.

## Scope and reading coverage

The question is two-dimensional obstacle avoidance by a nonholonomic vehicle with physical input limits. Longitudinal adaptive cruise control alone is not a sufficient counterexample to this concern. The review also checks the related claim that evasive ego motion requires an estimation model and error analysis compatible with the moving sensor frame.

The initial review examined eleven original PDFs attached to the local Zotero library through their main bodies, mathematical statements, proofs supplied in those PDFs, results, and conclusions. For ten papers this included every PDF page. For Tayal et al., pages 1–12 include the complete main body and appendices; its final reference/biography pages were not used as substantive evidence. The coupling addendum below examines two further original PDFs in full, bringing that review to thirteen sources. The method-description addendum examines two more original PDFs in full, bringing the current total to fifteen. This is a selected full-text review, not an exhaustive or prevalence-estimating survey of the library. Eighteen PDFs were located initially; the seven unreviewed items remain outside the substantive evidence, and the two addendum sources were located subsequently.

The companion [source and reading record](PLANAR_DISTANCE_CBF_FULLTEXT_REVIEW_20261014_sources.json) identifies the Zotero parent/attachment keys, DOI, PDF SHA-256, page counts, actual reading coverage, and paper-specific findings. Original PDFs remain in Zotero. Temporary text extraction preserved page breaks; critical statements on Tayal PDF page 4, Sharma page 6, and Alai page 3 were additionally checked against rendered original pages. No source code, solver implementation, or author-supplied experiment was reproduced. Paper-reported outcomes below are literature evidence, not new project experiments.

Page references use the local PDF page number unless a printed journal page is stated. Versions matter: the Zeng MPC paper is the library's arXiv v3, not an independently checked published typeset version.

## Finding and its mathematical scope

For an acceleration-controlled planar vehicle, a clearance function depending only on position generally fails the first-order CBF definition on the entire geometrically collision-free state set. This is a mathematical failure of the claimed certificate on that domain, rather than merely poor numerical tuning. A distance-seeded high-order barrier, an input-aware invariant-set barrier, and a predictive barrier are distinct constructions and must be judged using their own admissible state sets and control-existence conditions.

Nonholonomy alone is not a sufficient explanation. Relative degree, the selected vehicle reference point, which quantities are states versus directly commanded inputs, and the admissible input set all matter. Input bounds additionally restrict which collision-free states are viable and whether a proposed higher-order or predictive condition can continue to be satisfied.

### A direct planar counterexample

Consider a static circular obstacle centered at \(p_o\), inflated clearance radius \(R>0\), and the acceleration-controlled unicycle

\[
\dot p=v e(\theta),\qquad \dot v=a,\qquad
\dot\theta=\omega,\qquad e(\theta)=(\cos\theta,\sin\theta)^T,
\]

with admissible bounded acceleration and turning rate. Let

\[
h_d(x)=\|p-p_o\|^2-R^2,\qquad
C_d=\{(p,\theta,v):h_d(x)\geq0\}.
\]

The derivative is

\[
\dot h_d=2(p-p_o)^T v e(\theta).
\]

Neither current acceleration nor current turning-rate command appears. At a state with \(p-p_o=R e_1\), \(e(\theta)=-e_1\), and \(v>0\),

\[
h_d=0,\qquad \dot h_d=-2Rv<0
\]

for every admissible input. Every continuous extended class-K function satisfies \(\alpha(0)=0\), so

\[
\sup_{(a,\omega)\in U}
\{\dot h_d+\alpha(h_d)\}=-2Rv<0.
\]

Thus \(h_d\) is not a first-order zeroing CBF on the full set \(C_d\). By continuity the same inequality fails for sufficiently small positive clearance at the same inward heading and speed; the issue is not confined to an exactly touching boundary. The state set \(C_d\) contains positions that are collision-free at the current instant but have incompatible velocity and heading for the claimed invariance property. This derivation is independent analysis for this review. Xiao and Belta's Remark 1 gives the corresponding general obstruction when the constraint is zero and its derivative is negative.

For a dynamic bicycle whose pose derivative is determined by its current longitudinal/lateral velocity and yaw, steering and longitudinal force affect accelerations rather than instantaneously resetting the translational velocity. The same boundary obstruction applies to a position-only center-clearance function at an inward-moving state. This observation does not require claiming a constant relative degree everywhere: the higher-order decoupling terms can themselves vanish at particular configurations.

### Necessary qualifications

1. **Direct speed control is a different input model.** For \(\dot p=v e(\theta)\) with \(v\) directly selectable and zero speed admissible, a static-obstacle clearance condition can be satisfied by commanding zero speed. That may give poor progress, but it is not a disproof of safety. It does not describe a car whose current speed is a state and whose braking is finite.
2. **The chosen vehicle point matters.** In a rear-axle kinematic bicycle, steering first affects heading. In a center-of-gravity kinematic bicycle with algebraic sideslip, steering can enter the position derivative directly. He and Tayal use the latter approximation. A unicycle derivative calculation cannot be transplanted to their equations without accounting for that distinction.
3. **Distance-seeded HOCBFs are legitimate separate objects.** With \(\psi_1=\dot h_d+\alpha_1(h_d)\), a second-order construction enforces \(\dot\psi_1+\alpha_2(\psi_1)\geq0\) from initial states in \(\{h_d\geq0\}\cap\{\psi_1\geq0\}\). The certified initial set already contains velocity/heading restrictions. One must still establish existence of admissible inputs for the final condition. Merely differentiating until an input appears establishes neither that existence nor recursive feasibility.
4. **Discrete barriers require their own analysis.** The control may enter \(h_d(F(x,u))-h_d(x)\) over a finite sample even when it does not enter the instantaneous first derivative. This does not make every collision-free state viable under bounded inputs. Sample-to-sample safety also does not automatically establish safety between samples.
5. **QP failure has several causes.** An invalid candidate, incompatible barriers, changed target selection, estimation error, sampled/model mismatch, an invalid initial condition, and numerical failure are different explanations. A particular failure must be traced before attributing it to distance-only geometry.

## Original-paper findings

### Brüggemann, Steeves, and Krstic (2022)

*Simultaneous Lane-Keeping and Obstacle Avoidance by Combining Model Predictive Control and Control Barrier Functions*. [DOI](https://doi.org/10.1109/CDC51059.2022.9992613). Zotero `PHWFIKQE`; six-page published conference PDF.

The plant has constant longitudinal speed, a linear lateral bicycle model, and nonlinear global position mapping. The lane/obstacle barriers in equations (6)–(7) deform the permitted lane boundary. They have relative degree two; they are not raw radial-clearance first-order barriers.

Lemma 1's control-sharing feasibility result assumes a nonzero decoupling coefficient, matched gains, and positive lane expansion (13). Its QP optimizes over an unrestricted scalar steering command. Section III-D separately introduces input-constrained barriers, further restricts the state set, and requires the bounded-input supremum condition (18). It explicitly discusses the difficulty of validating that condition and leaves combined prescribed-time/input-constrained theory outside scope.

Section V-B and Figures 4–5 are particularly relevant: with late obstacle detection at 15 m, speed 20 m/s and steering saturated at 5 degrees, the non-input-aware design intersects the obstacle; the input-constrained counterpart avoids it in the reported simulation. This is direct planar evidence of lost safety through saturation. The simulations use zero lane expansion, so they should not be described as a direct verification of every assumption in Lemma 1. Sensing and computing effects are excluded.

### Chen, Peng, and Grizzle (2018)

*Obstacle Avoidance for Low-Speed Autonomous Vehicles With Barrier Function*. [DOI](https://doi.org/10.1109/TCST.2017.2654063). Zotero `VCX6LIPN`; 13-page accepted manuscript, subsequently published in volume 26.

Equations (1)–(3) use acceleration/yaw-rate unicycle dynamics with speed, acceleration, yaw-rate, and friction limits. The method first constructs an avoidable set in relative position, speed, and bearing, then uses reciprocal barriers for its facets (40) and mixed-integer disjunctions (44). It directly contradicts any claim that all vehicle barrier methods disregard dynamics and admissible controls.

The guarantee has an essential qualification: Section II-B adopts passive-friendly safety, excluding stopped-vehicle contact and collisions from behind from its responsibility definition (8). Accurate state measurements are assumed. Section V-E recognizes that multiple obstacles can make the single-obstacle filters incompatible and uses common emergency braking, relying on braking-safe initial states and that safety definition.

Appendix H derives the sampled constraint assuming the state derivative is constant within a sample (A.32). That derivation should not be promoted to a general exact intersample result for arbitrary nonlinear motion. The 1,000-trial comparison is useful reported validation, but its collision definition differs from unconditional geometric noncontact.

### Lee, Park, and Kim (2026)

*Turning Circle-Based Control Barrier Function for Efficient Collision Avoidance of Nonholonomic Vehicles*. [DOI](https://doi.org/10.1109/TCST.2026.3692226). Zotero `3GHTX9QC`; eight-page early-access PDF.

Equation (5) uses acceleration and yaw-rate inputs with speed as a state. The Euclidean-distance comparator is **not** just equation (6)'s position clearance: equation (7) forms \(h_e=\dot h+\alpha h\), and Remark 1 explicitly describes it as second-order. The proposed barrier instead incorporates turning radius and left/right turning-circle clearances, with a smooth conservative approximation of their maximum.

Proposition 1 supplies a geometric safe-turning witness for a fixed obstacle, constant speed, and instantaneous access to maximum yaw rate. It does not establish recursive feasibility of the entire MPC with input-rate limits and moving-obstacle predictions. Equation (13) includes those limits and finite-horizon discrete barrier constraints, but no displayed invariant terminal-set constraint.

Reported comparisons identify braking, aggressive steering, and feasibility problems for the distance-seeded comparator. Hardware validation uses an autonomous surface vessel. Remark 2 leaves robust tube tightening prospective; the conclusion excludes sensing noise/delay and calls for a stronger mathematical treatment. Use this work as evidence for turning-aware constructions and practical shortcomings, not as a proof that all distance-seeded HOCBFs are invalid.

### Tayal et al. (2026)

*A Collision Cone Approach for Control Barrier Functions*. [DOI](https://doi.org/10.1109/TCST.2026.3661160). Zotero `I55U59XZ`; printed pages 1297–1310, main body/appendices on PDF pages 1–12.

The collision-cone barrier (14) depends on relative position and velocity. Unicycle and bicycle claims assume static or constant-velocity obstacles; the bicycle construction also uses small sideslip and an approximated relative velocity, explicitly distinguished from the actual position derivative on printed page 1301.

Section II-C, printed page 1300, formulates the QP over \(\mathbb R^m\), explicitly states unlimited control authority, and leaves input limits to future work. The conclusion, page 1307, confirms that simulations impose no input bounds and that saturation causes hardware failures in high-speed corner cases. This is strong evidence that a formal CBF proof may concern a different input set from the deployed vehicle. It is not a position-only barrier example.

Remark 2, page 1303, limits the theorems to a single obstacle and leaves theory for conflicting multiple cones/perception selection unfinished. The unicycle proof is referred to an earlier paper, which was not independently reviewed here. Ground experiments use motion capture; quadrotor estimation uses an EKF. A stopping fallback does not establish universal noncollision against arbitrary moving obstacles.

### Xiao and Belta (2022)

*High-Order Control Barrier Functions*. [DOI](https://doi.org/10.1109/TAC.2021.3105491). Zotero `X3EUXWYE`; eight-page published PDF.

Definition 8 and Theorem 4 require the high-order inequality to admit controls in the actual set \(U\), initial membership in all derivative-defined sets, and a controller satisfying the inequality throughout time. Remark 1 explicitly identifies the impossibility when the original constraint is zero and its derivative is negative. Theorem 5/Corollary 1 give sufficient input-compatibility conditions; changing penalties also changes the permissible initial set.

Section V-B is directly planar and nonholonomic, rather than only ACC: the unicycle has acceleration and turning-rate inputs, limits of 0.5 m/s² and 0.2 rad/s, a speed interval, and a second-order squared-distance HOCBF. The offline parameterization and feasibility conditions are substantive attempts to address input limits. This work must be acknowledged when defining the gap.

Remark 8 separately warns that the sampled controller may violate safety between update instants. The paper supports criticism of unverified geometric candidates and unqualified sampled guarantees; it does not support dismissing the HOCBF framework as intrinsically unaware of actuation bounds.

### He, Zeng, Zhang, and Sreenath (2021)

*Rule-Based Safety-Critical Control Design Using Control Barrier Functions With Application to Autonomous Lane Change*. [DOI](https://doi.org/10.23919/ACC50511.2021.9482848). Zotero `BXD33RMX`; eight-page conference PDF.

The controller uses a small-sideslip center-of-gravity kinematic bicycle with direct sideslip and acceleration inputs (2), and imposes physical/input-rate limits. Equations (11)–(13) contain time headway, relative-speed-squared braking distance, and piecewise lateral clearance. Calling the whole method a naive Euclidean-distance CBF would misdescribe it.

The state machine changes maneuvers when a QP is unsolvable; safe switching requires membership in both relevant sets. Table VI reports unsolvable rates of 0.48% and 0.20%, but the explanation attributes them to simulated surrounding vehicles passing through one another and consequent target/barrier discontinuities. These numbers are not evidence of actuator-induced failure.

Remark 2 and Section V explicitly assume accurate surrounding-state information and no sensing delay. An estimator, noisy-distance safety margins, and communication delay are future work. This is a precise source for the estimation/control gap, without claiming that its authors never considered sensing as an issue.

### Black, Jankovic, Sharma, and Panagou (2023)

*Future-Focused Control Barrier Functions for Autonomous Vehicle Control*. [DOI](https://doi.org/10.23919/ACC55779.2023.10156163). Zotero `9D45ZPMM`; eight-page conference presentation PDF.

The vehicle has a dynamically extended kinematic bicycle model (8). The future-focused barrier uses predicted closest separation under a straight constant-velocity surrogate, and its relaxed version combines physical and virtual barriers. Assumption 1 restricts initial conditions to a set safe under the prediction. This is a velocity/prediction-aware method, not a pure present-distance example.

Theorem 1 establishes smoothness and a relation between barrier functions; the subsequent validity claim remains conditional on the admissible-control inequality. Theorem 2 also assumes a feasible boundary condition. In the one-left-turn simulations, the future-focused version is feasible in 96.3% of trials and the relaxed version in 100%; page 6 explicitly leaves theoretical feasibility guarantees to future work. Empirical success must not be relabeled a recursive-feasibility theorem.

The ground-rover experiment uses Vicon feedback and a PX4 EKF. Therefore it is false to say that this work contains no estimation. Its displayed safety formulation does not propagate a certified estimator-error enclosure into the collision condition.

### Zeng, Zhang, and Sreenath (2021)

*Safety-Critical Model Predictive Control with Discrete-Time Control Barrier Function*. [Original version](https://arxiv.org/abs/2007.11718v3). Zotero `AI2W6SI7`; nine-page arXiv v3 dated March 23, 2021; published ACC DOI `10.23919/ACC50511.2021.9483029`.

The MPC includes bounded controls and discrete barrier constraints (10). The obstacle functions are squared distance for a double integrator (17) and a quartic separation function for racing (21). These are discrete-time constraints, so the continuous first-derivative counterexample is not by itself an assessment of their implementation.

Section III-C gives a qualitative feasible-set discussion. Remark 4, PDF page 5, explicitly separates pointwise feasibility from persistent feasibility and leaves the latter open. A terminal-set symbol in the generic problem is not an explicit construction of a safe terminal policy for every vehicle example.

The double-integrator example is not nonholonomic. The racing example is relevant but assumes perfect position estimation (page 8) and uses a regressed linear prediction model against the nonlinear simulated vehicle, with prescribed surrounding-car motion. Use this as evidence for the distinction between prediction constraints and persistent feasibility, and for perfect-estimation assumptions. Do not infer a robust continuous-time vehicle guarantee from the abstract.

### Seo et al. (2022): relevant exclusion

*Safety-Critical Control With Nonaffine Control Inputs Via a Relaxed Control Barrier Function for an Autonomous Vehicle*. [DOI](https://doi.org/10.1109/LRA.2022.3142408). Zotero `2N6ZYVGF`; eight-page published PDF.

The nonlinear dynamic bicycle, flat outputs, and dynamic inversion are relevant vehicle-control techniques, but the demonstrated safety barriers are a speed limit \(v_s^2-v_x^2\) and a longitudinal time-headway function \(D-0.5\dot y_1\). The double lane change does not turn those constraints into arbitrary planar obstacle avoidance. This paper should not be used as a direct counterexample to the user's planar-distance concern.

The barrier is relaxed into a penalty. Theorem 1 and Remark 1 establish an enlarged set \(h\geq-\mu\), not exact invariance of \(h\geq0\); page 7 acknowledges small violations from relaxation and discretization. The argument assumes an invertible control mapping and existence of a tracking solution. These qualifications are more informative than treating every use of the word safety as the same guarantee.

### Alai, Sharma, Alexander, and Rajamani (2024)

*On the Importance of Accurate Relative Motion Models for Target Vehicle Trajectory Tracking*. [DOI](https://doi.org/10.1016/j.ifacol.2024.12.035). Zotero `MZVVMVMM`; six-page published PDF.

Equations (11)–(16) derive relative-position transport terms caused by ego rotation and translation. The state combines relative position/heading with absolute target speed, acceleration, and sideslip; required ego inputs are body velocity and yaw rate. Global ego position/yaw are unnecessary for this relative formulation. This is an antecedent source for the motion model, rather than attributing the entire relative formulation only to the later observer paper.

The target kinematic model has constant acceleration and slowly changing/constant sideslip assumptions. Section 3's low-cost ego-sensing alternatives have further bias, tire-slip, and kinematic qualifications. Section 5 uses an EKF with radar, IMU, and speedometer; the full-scale experiment supports the model's relevance during lane changes. It does not establish deterministic estimation-error tubes or a joint collision-avoidance theorem.

### Sharma, Alai, and Rajamani (2026)

*Simultaneous Ego-Vehicle State Estimation and Vehicle Trajectory Tracking Using a Multistage High Gain Observer*. [DOI](https://doi.org/10.1016/j.trc.2025.105411). Zotero `8P4BHHZU`; 20-page published PDF.

Equations (9)–(12) derive ego-frame transport and the error from an inconsistent relative model; equation (15) shows range-dependent error when noisy ego pose transforms measurements into inertial coordinates. NRMM equations (16)–(18) avoid that measurement transformation but retain ego-velocity/yaw-rate inputs. The target model assumes constant scalar acceleration and sideslip.

Assumption 1, page 6, sets **body-measured** acceleration derivatives and the yaw-rate derivative to zero. Equations (24)–(25) still permit nonzero inertial jerk during turning. The companion derivation uses the assumption again on page 10. The Lipschitz calculation on pages 11–12 holds ego/target yaw rates constant; it must not be reused as an unchecked uniform Jacobian bound under arbitrary maneuvers.

The nominal exponential observer theorem and the yaw observer's H-infinity attenuation are not themselves deterministic pointwise collision tubes. Road-data validation is not a closed-loop evasive-control experiment. Table 3 also shows that NRMM is not best on every metric. The paper supports adopting an ego-motion-consistent model and developing an explicit error certificate; it does not prove that NRMM is the only possible model.

## Implications for this project's argument

The existing [observer theory](../estimator/OBSERVER_ISS_THEORY.md), Section 1, retains the NRMM target model in covariant physical coordinates. It states that ego jerk and angular acceleration need not vanish, and declares a positive target-speed domain and target-model bounds. That is an identifiable distinction from the original Sharma companion derivation. Reading that section is not a new audit of the entire estimator or joint controller theorem.

The claims supported by this review, for discussion before manuscript revision, are:

- **Geometric clearance versus an actual certificate.** For the project's acceleration/force-controlled vehicle, a position-only distance function cannot certify the entire collision-free state set. A barrier claim must identify its state domain and establish compatibility with the actual dynamics and admissible inputs.
- **Higher order and prediction do not remove the feasibility obligation.** Prior work already includes HOCBFs, reachable-set barriers, turning-aware barriers, and predictive constructions. The gap must be the specific missing guarantee under the relevant combined assumptions, not the existence of those techniques.
- **Estimation must enter the safety argument.** Evasive ego motion changes relative sensing dynamics. Exact-state assumptions or an EKF used in a demonstration do not establish a bound sufficient for robust collision avoidance. A defensible contribution connects a maneuver-compatible model, certified estimation errors, admissible control, and preservation of feasibility under explicitly stated assumptions.

The following stronger generalizations are not justified: most/all CBF vehicle papers use pure positional distance; all their QPs eventually fail; all prior work lacks estimation; all methods without a displayed CBF lack formal safety; or NRMM is uniquely necessary. This selected review cannot measure how common a deficiency is in the broader literature. It also does not establish this project's novelty against every robust/output-feedback/predictive safety method or certify that the project's joint theory has no remaining gaps.

An experiment that fails under steering/force limits is a useful diagnostic. The paper argument should identify the failed mathematical condition and domain, while preserving the distinction between lack of a proof, an explicitly conditional proof, and a counterexample to a claimed guarantee.

## Coupling addendum: lateral and longitudinal control

The additional research question is whether treating lateral and longitudinal vehicle control separately is appropriate for obstacle-avoidance maneuvers. The supported criticism concerns **neglected or uncoordinated coupling under the intended maneuvering conditions**. A hierarchy, two control outputs, or mathematical decoupling through a coordinate transformation does not by itself establish that physical coupling is neglected. An integrated optimization likewise does not guarantee adequate coupling if its prediction model or feasible-force constraints omit it.

### Physical mechanisms and the feasibility consequence

In a body-frame planar model, the translational dynamics contain

\[
\dot v_x=F_x^{\mathrm{body}}/m+r v_y,\qquad
\dot v_y=F_y^{\mathrm{body}}/m-r v_x.
\]

Steering rotates the front-wheel longitudinal and lateral forces into both body axes; speed and yaw alter slip angles and subsequent tire forces. These terms are present even before considering shared tire saturation. Their effect is state- and maneuver-dependent, so there is no claim that every gentle lane-following maneuver requires the same model complexity.

A second mechanism is the common tire-force budget. Under an ideal isotropic friction-circle approximation for a tire,

\[
F_x^2+F_y^2\leq(\mu F_z)^2.
\]

If \(|F_x|=0.8\mu F_z\), then \(|F_y|\leq0.6\mu F_z\). Independently requesting longitudinal and lateral force magnitudes of \(0.8\mu F_z\) satisfies the two scalar component bounds, but the combined norm is \(\sqrt{1.28}\mu F_z>\mu F_z\). This is an illustrative calculation from the stated idealized force constraint, not experimental evidence or a claim that a friction circle captures every tire effect.

Braking also reduces subsequent speed and therefore the lateral acceleration approximately required for curvature tracking, \(a_y\simeq v^2\kappa\). Consequently, stronger braking can consume lateral force capacity immediately while making later turning demands easier to satisfy. The net result requires a trajectory-level assessment. It is incorrect to assert that braking always reduces overall avoidance capability.

Independent command bounds are not the same as independent achievable tire forces. Steering and braking commands may legitimately have a Cartesian admissible set while their joint tire-force image is strongly coupled. Thus the relevant modeling question is whether prediction and safety constraints preserve that image, rather than whether command limits have a rectangular notation.

For the earlier CBF discussion, ignoring this coupling can make a barrier inequality appear feasible under assumed lateral and longitudinal authority even though the requested force combination is unavailable. That is a mismatch between the certified model and plant; it is not a universal objection to structured or separately implemented controllers. A structured architecture can be valid if it coordinates the shared resources or bounds the coupling and establishes the required closed-loop property.

### Direct original-source evidence and counterexamples

**Guo, Hu, and Wang (2016), Nonlinear Coordinated Steering and Braking Control of Vision-Based Autonomous Vehicles in Emergency Obstacle Avoidance**, [DOI](https://doi.org/10.1109/TITS.2016.2544791), Zotero `CGS9N6G6`: all eleven original PDF pages were read, including the vision pipeline, controller derivation, Lyapunov analysis, field tests, and conclusion. Printed page 3231 identifies tire-force, load-transfer, and kinematic coupling as mechanisms; this discussion must be distinguished from its reduced plant, which ignores vertical/roll/pitch motion. Equations (1)–(3) retain cross-axis body dynamics and steering contributions; equation (36) jointly determines steering and brake pressure. Experiments compare coordinated control against separate LTV steering and sliding-mode longitudinal controllers. This supports the motivation and precludes claiming that coordinated control is new. The displayed proof concerns tracking behavior; its ultimate-bound calculation must not be advertised as a collision-free invariant-set proof.

**Beal and Boyd, Coupled Lateral-Longitudinal Vehicle Dynamics and Control Design With Three-Dimensional State Portraits**, [DOI](https://doi.org/10.1080/00423114.2018.1467019), Zotero `KJEGSTLS`: the entire 29-page original early-access PDF, including its cover and references, was read. It appeared online in 2018 and subsequently in volume 57(2), pages 286–313. Section 2, equations (1)–(6), retains variable longitudinal speed, four-wheel force projection, nonlinear slip, and a coupled brush-tire model. Section 3.2 shows that the directions reachable by steering/braking depend on state and that constant-speed portraits omit useful braking dynamics. The critical-cornering example is hierarchical yet computes steering and braking together, illustrating why hierarchy alone is not the criticism. Section 5 explicitly says state portraits do not prove stability; the example does not supply a general collision-avoidance invariant-set guarantee. Actuator bandwidth is omitted in the simulated example, and the reported experimental comparison validates a particular operating curve rather than every initial state.

The previously reviewed **Brüggemann et al.** paper provides a narrower example of simplification: Assumption 1 fixes longitudinal speed and steering is the avoidance input. This confines its maneuver class; it is not evidence that the method is wrong within that class. **Seo et al.**, introduction page 1944 and dynamic model (1), explicitly motivates coupled longitudinal/lateral control and retains cross-axis dynamics. Its use of flat-output decoupling is a control design technique, not permission to label the underlying physical model uncoupled. **Chen et al.**, equation (3), already couples acceleration and yaw-rate demands through \(a^2+v^2r^2\leq\mu^2g^2\); it is another counterexample to any blanket claim of independent maneuver budgets.

### Connection to the current project and manuscript scope

Read-only inspection confirms that `localFialaBody` in [nonlinearBicycleModel.m](../controller/nonlinearBicycleModel.m) evaluates steering and longitudinal-force commands together, rotates front-tire forces, and retains \(r v_y\) and \(-r v_x\). [modifiedFialaTire.m](../controller/modifiedFialaTire.m) uses the shared longitudinal ratio \(b\), longitudinal force \(b\mu_iF_{z,i}\), and lateral saturation scale \(\mu_iF_{z,i}\sqrt{1-b^2}\). Both input components are decision variables at each prediction hold in [solvePredictiveControl.m](../controller/solvePredictiveControl.m). The safety/CLF stages do not partition the longitudinal and lateral channels.

These facts support stating that the dynamic prediction retains the declared coupling. They do not establish a complete high-fidelity chassis model: the runtime has a separate low-speed kinematic branch and a blend; wheel inertia and dynamic normal-load effects are not represented by the reduced model. Inspection also finds that the solver explicitly omits steering magnitude/slew limits; joint input optimization must not be described as evidence that every physical actuator bound is enforced. No algorithm change or new validation was performed here.

The proposed argument is therefore that **safety and feasibility should be assessed using a maneuver-compatible coupled model and coordinated steering/longitudinal actuation, together with the declared estimation uncertainty**. It is not that every architecture with separate modules is invalid, nor that existing work has not addressed coupling. This point belongs with the physical validity of the safety model and admissible-control argument; coupling alone is not an established novelty claim. The manuscript remains outside this task's edit scope.

## Verification performed

Source identity and coverage were checked against the local attachment inventories. Main-body and supplied-appendix readings are recorded paper by paper. Three initial critical original pages and Beal/Boyd's body-force equation page were visually checked. The planar counterexample and friction-circle example were derived explicitly; neither required simulation. The report and source record received whitespace/diff, JSON parsing, source-hash, and cross-reference checks. No MATLAB tests or vehicle experiments were required or run, and neither manuscript source was edited by these review tasks.

## Method-description addendum: dynamic obstacles and contribution alignment

The Introduction needs to explain what each cited method actually does before identifying the missing property. Shared limitations can be discussed together, but only when the original evidence supports the same statement for every grouped source. The comparison is directed by this project's stated mechanisms: moving-target NRMM prediction; coupled nonlinear bicycle and combined-slip Fiala tire forces; maneuver-compatible observer coordinates and certified errors; and a conditional joint predictive safety/recursive-feasibility argument. These mechanisms do not establish uniformly better empirical performance by themselves.

### Additional full-original-PDF readings

**Cheng, Li, Liu, Li and Guo (2021), Virtual Fluid-Flow-Model-Based Lane-Keeping Integrated With Collision Avoidance Control System Design for Autonomous Vehicles**, DOI [10.1109/TITS.2020.2990211](https://doi.org/10.1109/TITS.2020.2990211), Zotero `GQ8832LY`, attachment `H2WNCQ37`: read all ten PDF pages, including the complete methods, tests, conclusion and references. Section II-B assumes two-dimensional incompressible steady flow. Section III-A derives lane-centering guidance from flow between plates; III-B superposes uniform/doublet flow around a cylinder and modifies the stream function, equations (24)–(36); III-C uses optimal-preview steering tracking, equations (37)–(43). Section IV-A uses four obstacles at fixed locations and about 20 km/h ego speed; IV-B uses an obstacle bus and ego speed below 20 km/h. Printed page 6240 explicitly leaves moving obstacles to future work. This page was also visually checked in the original PDF. The supported limitation is absence of a moving-obstacle formulation and of a closed-loop collision-invariance proof; a favorable reference path or tracking result is insufficient for the project's moving-target uncertainty problem. The design has steering-angle/rate constraints, so it must not be described as wholly unconstrained. Do not infer constant speed throughout its bus test: the measured ego speed falls to about 10 km/h during avoidance.

**Wang, Gao, Li, Sun and Cheng (2019), Obstacle Avoidance Path Planning Design for Autonomous Driving Vehicles Based on an Improved Artificial Potential Field Algorithm**, DOI [10.3390/en12122342](https://doi.org/10.3390/en12122342), Zotero `W6FZ33E7`, attachment `VBLGMJ76`: read all fourteen original PDF pages. Section 2 derives a risk/safety-distance model using relative vehicle motion and obstacle characteristics; Section 3.1 reshapes repulsion into an ellipse and introduces two virtual targets to escape local minima; Section 3.2 adds road fields and steering/lateral-acceleration restrictions. Sections 4–5 include a 15 km/h moving obstacle with 40/80 km/h ego simulation and a 10 km/h moving obstacle with 20 km/h initial ego speed in a vehicle test. PDF page 11's moving-obstacle test was visually checked. The conclusion on pages 12–13 states structured-road, constant-velocity ideal conditions and leaves multiple-obstacle/variable-speed scenarios prospective. It supplies dynamic-obstacle evidence, so it cannot be grouped with Cheng as static-only. Neither paper supplies a closed-loop collision-free invariant-set proof; that is a supported common limitation. Its virtual targets address local minima, so local-minimum susceptibility cannot be stated as an untouched omission.

### Method, supported gap, and corresponding project mechanism

| Work | Actual method and source location rechecked | Supported limitation relevant to this project | Corresponding project mechanism and scope |
| --- | --- | --- | --- |
| Cheng et al. (2021) | Steady virtual road flow plus cylindrical-obstacle flow; modified stream-function path; optimal-preview steering. Sections II–IV, equations (14)–(43); conclusion p. 6240. | Moving obstacles are explicitly future work. No closed-loop collision-invariance proof. | Predict an explicitly moving target through the declared NRMM and include its estimated state/error in collision constraints. |
| Wang et al. (2019) | Risk-shaped elliptical APF repulsion; virtual targets; road and steering/lateral-acceleration restrictions. Sections 2–5, equations (5)–(22). | Validation is structured-road constant-velocity avoidance; no closed-loop invariant-set certificate. Dynamic obstacles are already included. | Declared NRMM supports nonzero target acceleration and turning; the certificate includes uncertainty under that model and its defect contract. This is not arbitrary target behavior. |
| Guo et al. (2016) | Three-degree-of-freedom coupled plant; backstepping equivalent control jointly solves brake pressure and steering, equation (36); adaptive fuzzy sliding-mode reaching, equations (39)–(40). Rechecked PDF pp. 3–8. | Coupling is addressed. The Lyapunov analysis concerns tracking errors, not collision-free invariance against an independently moving target. | Couple the Fiala vehicle prediction with explicit moving-target collision constraints and a predictive safety/feasibility argument. Do not claim that coordinated control itself is new. |
| Xiao and Belta (2022) | Successive derivative-defined safe sets and a final admissible-input inequality. Rechecked PDF pp. 2–4, Remark 1, Definition 8, Theorems 4–5. | The guarantee requires valid initial derivative constraints and an admissible satisfying control. The framework explicitly addresses input compatibility; it is not an example of ignoring bounds. | Explain why a position-only clearance candidate is insufficient on the entire geometric safe set; use predictive continuation and the declared admissible controls. |
| Brueggemann et al. (2022) | A high-rate CBF steering filter overrides lane-tracking MPC; obstacle-dependent lane barriers with exponential/prescribed-time and input-constrained variants. Rechecked PDF pp. 2–6, equations (6)–(18). | Constant longitudinal speed excludes combined steering/braking. Input-constrained variants exist; their validation obligation and excluded combined theory must not be erased. | Joint steering/longitudinal optimization on the declared variable-speed nonlinear Fiala bicycle. |
| Tayal et al. (2026) | Relative-position/velocity collision cone; barrier filters reference control outside collision-producing velocity directions. Rechecked PDF pp. 4–7 and 11, equations (9), (14), ground-vehicle Theorems 1–2. | Ground-vehicle theorem assumes constant obstacle velocity; QP assumes infinite authority. Printed pp. 1300/1307 identify input limits and saturation failures. | Explicit admissible-control/terminal-continuation contracts and bounded shared tire-force prediction. Do not claim that every physical actuator bound is enforced in the current solver. |
| Zeng et al. (2021) | Discrete CBF decrease conditions at each MPC prediction step, equations (10); moving-car racing uses a quartic separation function, equation (21). Rechecked PDF pp. 2–5 and 7–8. | Remark 4 leaves persistent feasibility open. Racing assumes perfect ego/other-car position estimates. | A shifted causal policy, retained certificates, posterior intersections and a common terminal continuation establish conditional recursive feasibility with estimation uncertainty. |
| Black et al. (2023) | Predict the time/clearance of closest approach with a constant-velocity surrogate; relax virtual clearance using physical separation, equations (14)–(24). Rechecked PDF pp. 3–6. | Feasibility theory is explicitly future work despite good empirical results. Theorems remain conditional on admissible inputs; EKF use is not a certified collision-error tube. | Preserve feasibility under stated estimator/forecast contracts; include explicit error enclosures. Group with Zeng only for the unresolved feasibility proof, not as identical algorithms. |
| He et al. (2021) | Finite-state machine selects lane-change/abort modes; CLF–CBF QPs use time headway and braking-distance barriers, equations (11)–(14). Rechecked PDF pp. 2–8, Remark 2 and Section V. | Accurate, delay-free surrounding-state data are assumed. State estimation and noisy-distance margins are future work. | The maneuver-compatible NRMM observer supplies certified target-state errors to the safety analysis. Group with Zeng's racing example for perfect-information assumptions, while preserving their different models/controllers. |

### Limits of the comparison and verification

The rewritten Introduction adds a general opening sentence, describes the algorithms before their limitations, groups only shared deficiencies, and ends at the combined modeling/actuation/estimation/feasibility requirement. Its aim is a source-specific contribution argument. The selected literature does not prove that no other work addresses the combined problem, that every omission is a failure inside an author's stated assumptions, or that project performance is uniformly superior.

The project retains positive target speed, constant scalar target acceleration/curvature in its nominal NRMM and declared defect bounds for deviations. Its robust recursive-feasibility theorem is conditional on sound information/model contracts and initial feasibility, and currently excludes road boundaries. The runtime lacks explicit steering-magnitude/slew constraints. Thus an unrestricted claim that this implementation enforces all physical actuator limits would be unsupported. No algorithm, experiment, numerical performance result or full-theory audit was added in this revision.

Local Zotero access was read-only. Full-text extraction preserved page boundaries; two key new source pages were rendered and visually inspected. Existing source hashes identify the other reviewed originals; the companion source record adds both new PDFs, their reading coverage and the targeted rechecks. Original PDFs stay in Zotero and are not copied into the repository.
