# Full-text review of distance barriers for constrained planar vehicles

Prepared on October 14, 2026. Research discussion record; the manuscript was not edited for this review.

## Scope and reading coverage

The question is two-dimensional obstacle avoidance by a nonholonomic vehicle with physical input limits. Longitudinal adaptive cruise control alone is not a sufficient counterexample to this concern. The review also checks the related claim that evasive ego motion requires an estimation model and error analysis compatible with the moving sensor frame.

Eleven original PDFs attached to the local Zotero library were examined through their main bodies, mathematical statements, proofs supplied in those PDFs, results, and conclusions. For ten papers this included every PDF page. For Tayal et al., pages 1–12 include the complete main body and appendices; its final reference/biography pages were not used as substantive evidence. This is a selected full-text review, not an exhaustive or prevalence-estimating survey of the library. Eighteen PDFs were located, but the seven remaining items were not substantively reviewed in this activity and support no findings below.

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

## Verification performed

Source identity and coverage were checked against the local attachment inventory. Main-body and supplied-appendix readings are recorded paper by paper. Three critical original pages were visually checked. The planar counterexample was derived explicitly; it required no simulation. The report and source record received whitespace/diff, JSON parsing, source-hash, and cross-reference checks. No MATLAB tests or vehicle experiments were required or run, and neither manuscript source was edited by this task.
