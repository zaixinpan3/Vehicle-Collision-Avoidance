function cfg = collisionAvoidanceControllerConfig(userCfg)
% collisionAvoidanceControllerConfig Defaults and merge for the controller.
%
% Returns the complete configuration structure the two-stage
% approximate convex collision-avoidance controller consumes, with any supplied
% overrides merged recursively over the declared defaults. Every field
% the controller reads is defined here; a missing field is a
% configuration error at the consuming module rather than a silent
% default there.
% Longitudinal-acceleration limits are validated here and normalized to
% double scalars in m/s^2 before model and tire modules consume them.
%
% The defaults describe a mid-size passenger car with the Ge et al.
% (2022) input [deltaF; a] and the forward-Euler Frenet LTV bicycle
% stage model of LTV_BICYCLE_MODEL.md.

    if nargin < 1
        userCfg = [];
    end
    cfg = localDefaults();
    if ~isempty(userCfg)
        if ~isstruct(userCfg) || ~isscalar(userCfg)
            error("collisionAvoidanceController:invalidConfiguration", ...
                "The configuration override must be a scalar structure.");
        end
        cfg = localMergeStructure(cfg, userCfg);
    end
    cfg.actuation = localNormalizeActuation(cfg.actuation);
    localValidate(cfg);
end

function cfg = localDefaults()
    cfg = struct();

    % Route-following cruise demand of the CLF.
    cfg.referenceSpeed = 15.0;

    % Closed-loop period and prediction horizon. Every prediction
    % stage is one sample, so the plan advances one node per sample.
    cfg.controller = struct( ...
        "sampleTime", 0.05, ...
        "horizonSteps", 48);

    % The collision constraint of Li et al. (2023), Eq. (13) without
    % its slack: (A p_k - b)' lambda*_k >= d_min, HARD, with the dual
    % lambda*_k of stage 1 frozen at the linearization trajectory. A
    % plan that cannot honour every imposed row is "no solution".
    %
    % clearanceMargin is d_min in metres: every separation row is the
    % dual-linearized distance minus this margin (plus the declared
    % position, yaw and curvature budgets), so a zero margin is contact
    % plus the margin. It is a geometric clearance, not a time gap.
    %
    % disturbanceBound is the declared per-step margin disturbance d in
    % metres: the mismatch between the plant and the model that one
    % sample may put into a separation margin, and therefore what
    % bounds the shortfall a FACT may have. A separation row some plan
    % in the input reach box satisfies is imposed; a row every
    % plan in the box misses, at node k of the near window (top node
    % W), by at most (W - k + 1) d - the mismatch accumulated since the
    % node was last a hard row - is a fact of the state (reported,
    % collisionFactViolation); a row missed by more, or outside the
    % window, makes the candidate infeasible - that is also what keeps
    % a passing candidate from winning by ignoring near nodes it cannot
    % yet clear (those miss by metres). Measured on the 14-DOF plant:
    % 0.018 m at the 99th percentile, 0.026 m worst per sample; facts
    % drifted to 7 cm at node 3 while cutting back in beside the lead.
    %
    % factWindowFactor bounds the NEAR WINDOW: the nodes whose
    % authority - the largest change a reach-box step can make to the
    % margin, growing like the fourth power of the node index - is
    % below this multiple of d. Only there may a row be a fact, and
    % only there may the joint test (one least-relaxation LP when a
    % candidate is infeasible; the friction polygon takes capacity the
    % reach box credits - measured, a node the box said could reach
    % +30 mm that no admissible plan raised above -2.5 mm) declare
    % facts. Rows beyond the window are never dropped.
    cfg.collision = struct( ...
        "clearanceMargin", 0.25, ...
        "disturbanceBound", 0.0, ...
        "factWindowFactor", 10.0);

    % THE CERTIFIED EXECUTION MODE. In this mode collision and road CBF
    % rows are hard and are never removed as "facts" or assigned a safety
    % slack. The terminal continuation is a hard augmented-state condition.
    % A feasible hard-constrained solver result is accepted directly; a
    % shifted stored fallback is the exceptional path that is rechecked.
    % A nonzero disturbanceBound needs a robust reachable-set/tube design;
    % the earlier row-dropping heuristic is deliberately unavailable here.
    cfg.certification = struct( ...
        "enabled", true, ...
        "shiftConsistencyTolerance", 1.0e-8, ...
        ... % Fixed inertial support directions used to separate the
        ... % resting ego from the controller's complete predicted
        ... % target continuation. This discretizes terminal-set size,
        ... % not the target trajectory or its admissible motion class.
        "terminalSupportDirectionCount", 64, ...
        "interSampleMaxDepth", 12, ...
        "interSampleDistanceTolerance", 1.0e-8);

    % THE DISJUNCTIVE SEARCH. The exact collision-free condition at a
    % node is that the ego centre lies outside AT LEAST ONE facet of
    % the configuration obstacle - a logical OR, and the reason the
    % problem is non-convex. Freezing one facet per node makes the
    % program convex but makes the LINEARIZATION choose the homotopy
    % class, so the answer is only the best plan in the class the
    % linearization reached. With nodeBudget > 0 the OR is kept in the
    % model and the choice becomes a decision of the optimization:
    % branch and bound over the facet assignment
    % (collisionAvoidanceController>localDisjunctiveSearch), whose
    % relaxations impose the assigned nodes' rows and nothing at the
    % rest, so every bound is valid and an exhausted tree is a proof of
    % global optimality for the convexified program at this
    % linearization. There is then one linearization and no start set:
    % the shifted previous solution, with the receding horizon as the
    % sequential convex iteration over it.
    %
    % nodeBudget caps the subproblems one sample may solve. The search
    % is anytime: within the budget the incumbent is a feasible plan of
    % some class and the reported gap is the certified distance to the
    % best any class could achieve, and a budget-exhausted search with
    % no incumbent is an optimization failure, never "no solution" -
    % only a closed tree proves infeasibility. ZERO selects the
    % multi-start path below instead, which is the A/B the
    % measurements run.
    %
    % relativeGapTolerance is when a subtree is pruned against the
    % incumbent, and so the optimality the search claims.
    %
    % incumbentBudget is how many subproblems the search may spend
    % while it has NO incumbent at all: the node budget caps the work
    % spent on optimality once a feasible plan exists, and a search
    % that has none is not trading optimality for time but a plan for
    % a lost sample. With the terminal set's tail the disjunction holds
    % 128 nodes; the dive child (collisionAvoidanceController>
    % localDiveAssignment) normally reaches an incumbent in a few
    % subproblems, and this is the cap when it does not.
    cfg.disjunctive = struct( ...
        "nodeBudget", 0, ...
        "incumbentBudget", 160, ...
        "relativeGapTolerance", 1.0e-4);

    % THE SEQUENTIAL CONVEX REFINEMENT, and the multi-start that goes
    % with it. Used when disjunctive.nodeBudget is zero.
    %
    % The collision row is one supporting half-plane of the
    % configuration obstacle, and which one is decided by the
    % trajectory stage 1 is linearized at: that trajectory fixes the
    % homotopy class of everything the program can return. A braking
    % linearization yields rows with no lateral coefficient whose
    % feasible set contains no passing plan at all, and re-solving at
    % the solution reproduces it - a fixed point. One start can
    % therefore only ever return its own class, however long it is
    % iterated.
    %
    % So the controller solves from TWO starts - the shifted previous
    % solution and the obstacle-free cruise reference - and commits the
    % feasible one with the least exact objective. In certified mode both
    % candidates are hard and no refinement slack exists. There is no test
    % on the driving situation anywhere: braking, following and passing are
    % outcomes of that comparison. The second start is formed whenever the
    % obstacle imposes a row, which is when the linearization can decide the
    % answer.
    %
    % In certified mode, a nonempty penaltySchedule only enables the
    % second hard start. In legacy mode it is also the refinement schedule:
    % while a start is not yet separated from the obstacle, one elastic
    % program per entry (a priced slack per imposed target node, everything
    % else hard) is solved at it and its solution becomes the next
    % linearization. The
    % price must start low enough that the first rung follows the
    % start's own normals rather than paying to stay where it is, and
    % end high enough that the last rung comes out separated - a
    % separated linearization is a feasible point of the hard program
    % built at it, so the solve cannot then fail for want of one. The
    % CLF relaxation is priced by the linear clf.relaxationWeight in the
    % certified objective. EMPTY DISABLES THE SECOND START and legacy
    % refinement, leaving the plain receding-horizon iteration, which is
    % the A/B the measurements run.
    %
    % A rung carries only the rows that decide whether a class is
    % REACHABLE - the road rows, the speed and heading domains, and the
    % input box - and not the friction
    % polygons, the CLF rows or their trust region, which shape a
    % committed plan rather than a class. It is solved by the
    % interior-point kernel: an elastic program's optimum is far from
    % its initial point and the active set walks there one constraint
    % at a time (measured: 1960 rows and 709 pivots at 0.43 s against
    % 524 rows and 16 interior-point iterations at 0.06 s, same
    % result).
    %
    % violationTolerance is when a start counts as separated - and so
    % how the loop terminates; maxIterations is the kernel budget of
    % one rung.
    cfg.sequentialConvex = struct( ...
        "penaltySchedule", [1.0e4; 1.0e6], ...
        "violationTolerance", 1.0e-4, ...
        "maxIterations", 200);

    % Vehicle geometry and inertia.
    cfg.vehicle = struct( ...
        "m", 1650.0, ...
        "Iz", 1700.0, ...
        "lf", 1.4, ...
        "lr", 1.65, ...
        "wheelbase", 3.05, ...
        "length", 4.8, ...
        "width", 1.9, ...
        "gravity", 9.81, ...
        "centerOfGravityHeight", 0.55);
    cfg.tire = struct( ...
        "corneringStiffness", [96000.0; 96000.0], ...
        "frictionCoefficient", [0.85; 0.85]);
    cfg.actuation = struct( ...
        "brakingForceDistribution", [0.625; 0.375], ...
        "longitudinalAccelerationMinimum", -8.0, ...
        "longitudinalAccelerationMaximum", 4.0);

    % Declared model domain. speedMinimum is bounded away from zero
    % because the linear-cornering rows carry 1/vx; scheduleSpeedFloor
    % clamps the frozen schedule speed (forward-Euler stability);
    % plannedSpeedMinimum is the hard floor of the planned speed - the
    % frozen-speed linearization is only trusted near the schedule
    % speed - lowered per sample to the measured speed when the
    % vehicle is slower. headingDomainRadius bounds the plan's heading
    % error to the path (validity of the Frenet linearization).
    cfg.model = struct( ...
        "speedMinimum", 1.0, ...
        "speedMaximum", 18.0, ...
        "plannedSpeedMinimum", 12.0, ...
        "scheduleSpeedFloor", 12.0, ...
        "frontWheelSteeringAngleMaximum", deg2rad(40.0), ...
        "frictionPolygonEdgeCount", 8, ...
        "slipAngleMaximum", deg2rad([10.0; 10.0]), ...
        "headingDomainRadius", 0.40, ...
        "ltvModelErrorRateBound", zeros(6, 1), ...
        "plantModelResidualRateBound", zeros(6, 1));

    % THE TERMINAL SET (TWO_STAGE_SAFETY.md, "The terminal set"). Every
    % program ends in a KINEMATIC BRAKING TAIL: N_b extra stages, with
    % the tail accelerations as decision variables, on which the same
    % separation and road rows are imposed, ending at REST. A plan is
    % admitted only if the vehicle can come to rest from its terminal
    % state without ever violating a row - the backup-set construction
    % of the CBF literature, with the optimizer supplying the witness.
    % Rest is invariant under zero input, so the feasible set of the
    % program is control invariant under the declared model and target
    % prediction (Proposition 8), which is what turns the rows into a
    % control barrier function. The knobs:
    %
    % backupDeceleration is the tail's braking rate a_b (m/s^2,
    % positive): each tail acceleration may independently lie in
    % [-a_b, 0]. Together with lateralAccelerationMaximum it must fit
    % inside every axle's
    % friction polygon under the braking shares and the affine load
    % transfer, which is validated here.
    %
    % lateralAccelerationMaximum is what the tail's lane-keeping law may
    % use while braking; lateralBandwidth (1/m) is the law's bandwidth
    % in arclength. The two fix the heading bound of the terminal node
    % in closed form (terminalLateralCertificate): a faster law holds a
    % narrower lateral band but demands more steering, so it admits a
    % smaller terminal heading error. lateralVelocityMaximum (m/s) and
    % yawRateErrorMaximum (rad/s) are the terminal rows on the dynamic
    % states the kinematic tail does not carry; their course
    % contributions are charged against the heading bound. The tail
    % length N_b is derived (kinematicBrakingTail), not declared.
    cfg.terminal = struct( ...
        "backupDeceleration", 5.0, ...
        "lateralAccelerationMaximum", 4.0, ...
        "lateralBandwidth", 0.08, ...
        "lateralVelocityMaximum", 0.15, ...
        "yawRateErrorMaximum", 0.05);

    % Road-geometry implementation allowances.
    cfg.road = struct( ...
        "orthonormalTolerance", 1.0e-9, ...
        "parameterRangeTolerance", 1.0e-3);

    % The CLF: the Riccati certificate scales of the path-frame cruise
    % error, the input weights (also the minimum-intervention weights
    % of the objective), the demanded fraction of the certified
    % decrease, and the linear price of its relaxation. Certified mode
    % optimizes this term together with the minimum-intervention input
    % cost. There is no quadratic CLF-relaxation penalty.
    %
    % trustRegionScale bounds the CLF LINEARIZATION: at every node the
    % plan's error may differ from the nominal's by at most this
    % multiple of the channel's error scale. The CLF rows are the
    % tangent of a convex quadratic at the nominal error; the tangent
    % lies below the quadratic, so along -grad V the linear model
    % credits a decrease without bound and the optimizer will buy it to
    % cancel a real increase elsewhere - measured as a 5.35 m lateral
    % overshoot at the far nodes with zero relaxation and an exact
    % residual of 220, which is what seeded every overtake before the
    % bound existed. With the bound the loop is the damped SQP
    % iteration the CLF rows are meant to be. The value trades
    % linearization error (grows with the square of the scale) against
    % how many samples a plan needs to move to a new shape (a full
    % braking plan is ~3 m/s away from a cruise plan at the far nodes;
    % at 2 x 0.25 m/s per sample that is 6 samples). The trust region
    % binds the incumbent candidate only; a hard separation row never
    % yields to it (an infeasible trust-bounded incumbent is re-solved
    % unbounded) and the passing candidates, linearization restarts,
    % carry none. Inf removes the rows.
    cfg.clf = struct( ...
        "lateralPositionErrorScale", 0.5, ...
        "headingErrorScale", 0.1, ...
        "speedErrorScale", 0.25, ...
        "lateralVelocityErrorScale", 0.5, ...
        "yawRateErrorScale", 0.2, ...
        "frontWheelSteeringAngleWeight", 1.0, ...
        "longitudinalAccelerationWeight", 1.0, ...
        "decreaseRateFraction", 0.9, ...
        "certificateSpeedFloor", 5.0, ...
        "relaxationWeight", 100.0, ...
        "trustRegionScale", 2.0);

    % Convex kernel. An empty function selects quadprog (active set,
    % with one interior-point retry and the step-tolerance ladder on a
    % stalled solve); a supplied function must take quadprog's
    % argument list.
    cfg.solver = struct( ...
        "function", [], ...
        ... % Optional test/integration hook for the single certified
        ... % joint CLF-relaxation/input solve. The handle receives
        ... % (phase, problem),
        ... % and problem.defaultSolver executes the built-in solver.
        "jointFunction", [], ...
        "maxIterations", 400, ...
        ... % The active-set budget of a passing candidate (no retry):
        ... % ordinary solves take ~30 pivots; a candidate whose rows
        ... % the incumbent violates by metres grinds through any
        ... % budget. Retried only when no candidate resolved.
        "candidateMaxIterations", 150, ...
        "constraintTolerance", 1.0e-7, ...
        "optimalityTolerance", 1.0e-7, ...
        "stepTolerance", 1.0e-12, ...
        "stepToleranceRetry", [1.0e-10; 1.0e-8]);

    % Fallback target rectangle when an estimate publishes no extent.
    cfg.target = struct( ...
        "defaultLength", 4.8, ...
        "defaultWidth", 1.9);
end

function base = localMergeStructure(base, overrides)
    fields = fieldnames(overrides);
    for fieldIdx = 1:numel(fields)
        name = fields{fieldIdx};
        value = overrides.(name);
        if isfield(base, name) && isstruct(base.(name)) ...
                && isstruct(value) && isscalar(value) ...
                && isscalar(base.(name))
            base.(name) = localMergeStructure(base.(name), value);
        else
            base.(name) = value;
        end
    end
end

function actuation = localNormalizeActuation(actuation)
% Keep the acceleration-input contract at the configuration boundary.
    if ~isstruct(actuation) || ~isscalar(actuation)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "cfg must contain a scalar actuation structure.");
    end
    names = ["longitudinalAccelerationMinimum", ...
        "longitudinalAccelerationMaximum"];
    for name = names
        value = actuation.(name);
        if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
                || ~isfinite(value)
            error("collisionAvoidanceController:invalidConfiguration", ...
                "actuation.%s must be a finite real scalar.", name);
        end
        actuation.(name) = double(value);
    end
    minimum = actuation.longitudinalAccelerationMinimum;
    maximum = actuation.longitudinalAccelerationMaximum;
    if minimum >= maximum || minimum > 0.0 || maximum < 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "Longitudinal-acceleration limits must satisfy " ...
            + "aMinimum <= 0 <= aMaximum and aMinimum < aMaximum.");
    end
end

function localValidate(cfg)
    if cfg.model.speedMinimum <= 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "model.speedMinimum must be positive: the linear-cornering " ...
            + "rows carry 1/vx.");
    end
    if cfg.model.speedMaximum <= cfg.model.speedMinimum
        error("collisionAvoidanceController:invalidConfiguration", ...
            "model.speedMaximum must exceed model.speedMinimum.");
    end
    if cfg.model.scheduleSpeedFloor < cfg.model.speedMinimum ...
            || cfg.model.scheduleSpeedFloor > cfg.model.speedMaximum
        error("collisionAvoidanceController:invalidConfiguration", ...
            "model.scheduleSpeedFloor must lie inside the model " ...
            + "speed domain.");
    end
    if cfg.model.plannedSpeedMinimum < cfg.model.speedMinimum ...
            || cfg.model.plannedSpeedMinimum > cfg.model.speedMaximum
        error("collisionAvoidanceController:invalidConfiguration", ...
            "model.plannedSpeedMinimum must lie inside the model " ...
            + "speed domain.");
    end
    if cfg.controller.horizonSteps < 2 ...
            || cfg.controller.horizonSteps ~= round(cfg.controller.horizonSteps)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "controller.horizonSteps must be an integer of at least 2.");
    end
    if cfg.controller.sampleTime <= 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "controller.sampleTime must be positive.");
    end
    if ~isnumeric(cfg.clf.relaxationWeight) ...
            || ~isscalar(cfg.clf.relaxationWeight) ...
            || ~isfinite(cfg.clf.relaxationWeight) ...
            || cfg.clf.relaxationWeight <= 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "clf.relaxationWeight must be a positive finite scalar.");
    end
    if cfg.clf.decreaseRateFraction <= 0.0 ...
            || cfg.clf.decreaseRateFraction > 1.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "clf.decreaseRateFraction must lie in (0, 1].");
    end
    % Inf is admissible and means "no trust region"; NaN and
    % nonpositive values are not.
    if ~isnumeric(cfg.clf.trustRegionScale) ...
            || ~isreal(cfg.clf.trustRegionScale) ...
            || ~isscalar(cfg.clf.trustRegionScale) ...
            || isnan(cfg.clf.trustRegionScale) ...
            || cfg.clf.trustRegionScale <= 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "clf.trustRegionScale must be a positive scalar or Inf.");
    end
    if ~isnumeric(cfg.collision.clearanceMargin) ...
            || ~isscalar(cfg.collision.clearanceMargin) ...
            || ~isfinite(cfg.collision.clearanceMargin) ...
            || cfg.collision.clearanceMargin < 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "collision.clearanceMargin must be a nonnegative finite " ...
            + "scalar.");
    end
    if ~isnumeric(cfg.collision.disturbanceBound) ...
            || ~isscalar(cfg.collision.disturbanceBound) ...
            || ~isfinite(cfg.collision.disturbanceBound) ...
            || cfg.collision.disturbanceBound < 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "collision.disturbanceBound must be a nonnegative finite " ...
            + "scalar.");
    end
    if ~isnumeric(cfg.collision.factWindowFactor) ...
            || ~isscalar(cfg.collision.factWindowFactor) ...
            || ~isfinite(cfg.collision.factWindowFactor) ...
            || cfg.collision.factWindowFactor < 1.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "collision.factWindowFactor must be a finite scalar of at " ...
            + "least 1.");
    end
    if ~isfield(cfg, "certification") ...
            || ~isstruct(cfg.certification) ...
            || ~isscalar(cfg.certification) ...
            || ~isfield(cfg.certification, "enabled") ...
            || ~islogical(cfg.certification.enabled) ...
            || ~isscalar(cfg.certification.enabled)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "certification.enabled must be a logical scalar.");
    end
    localValidateNonnegativeInteger( ...
        cfg.certification.interSampleMaxDepth, ...
        "certification.interSampleMaxDepth");
    localValidateNonnegativeInteger( ...
        cfg.certification.terminalSupportDirectionCount, ...
        "certification.terminalSupportDirectionCount");
    if cfg.certification.terminalSupportDirectionCount < 8
        error("collisionAvoidanceController:invalidConfiguration", ...
            "certification.terminalSupportDirectionCount must be an " ...
            + "integer of at least 8.");
    end
    localValidateNonnegativeScalar( ...
        cfg.certification.shiftConsistencyTolerance, ...
        "certification.shiftConsistencyTolerance");
    localValidateNonnegativeScalar( ...
        cfg.certification.interSampleDistanceTolerance, ...
        "certification.interSampleDistanceTolerance");
    if cfg.certification.enabled && cfg.collision.disturbanceBound ~= 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "Certified mode requires collision.disturbanceBound = 0. " ...
            + "Install a robust tube before claiming a nonzero bound.");
    end
    if ~isnumeric(cfg.disjunctive.nodeBudget) ...
            || ~isscalar(cfg.disjunctive.nodeBudget) ...
            || ~isfinite(cfg.disjunctive.nodeBudget) ...
            || cfg.disjunctive.nodeBudget < 0 ...
            || cfg.disjunctive.nodeBudget ~= round(cfg.disjunctive.nodeBudget)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "disjunctive.nodeBudget must be a nonnegative integer " ...
            + "(zero selects the multi-start path).");
    end
    if ~isnumeric(cfg.disjunctive.relativeGapTolerance) ...
            || ~isscalar(cfg.disjunctive.relativeGapTolerance) ...
            || ~isfinite(cfg.disjunctive.relativeGapTolerance) ...
            || cfg.disjunctive.relativeGapTolerance < 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "disjunctive.relativeGapTolerance must be a nonnegative " ...
            + "finite scalar.");
    end
    if ~isnumeric(cfg.disjunctive.incumbentBudget) ...
            || ~isscalar(cfg.disjunctive.incumbentBudget) ...
            || ~isfinite(cfg.disjunctive.incumbentBudget) ...
            || cfg.disjunctive.incumbentBudget < 0 ...
            || cfg.disjunctive.incumbentBudget ...
                ~= round(cfg.disjunctive.incumbentBudget)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "disjunctive.incumbentBudget must be a nonnegative integer.");
    end
    if cfg.certification.enabled && cfg.disjunctive.nodeBudget > 0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "The legacy weighted disjunctive search is unavailable in " ...
            + "certified hard-CBF mode. Use nodeBudget = 0.");
    end
    if ~isnumeric(cfg.sequentialConvex.penaltySchedule) ...
            || ~isreal(cfg.sequentialConvex.penaltySchedule) ...
            || any(~isfinite(cfg.sequentialConvex.penaltySchedule)) ...
            || any(cfg.sequentialConvex.penaltySchedule <= 0.0)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "sequentialConvex.penaltySchedule must be positive and " ...
            + "finite (empty disables the refinement and the second " ...
            + "start).");
    end
    if ~isnumeric(cfg.sequentialConvex.violationTolerance) ...
            || ~isscalar(cfg.sequentialConvex.violationTolerance) ...
            || ~isfinite(cfg.sequentialConvex.violationTolerance) ...
            || cfg.sequentialConvex.violationTolerance < 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "sequentialConvex.violationTolerance must be a " ...
            + "nonnegative finite scalar.");
    end
    if ~isempty(cfg.solver.jointFunction) ...
            && ~isa(cfg.solver.jointFunction, "function_handle")
        error("collisionAvoidanceController:invalidConfiguration", ...
            "solver.jointFunction must be empty or a function handle.");
    end
    % Forward-Euler stability of the declared stage model: the lateral
    % and yaw stiffness rates scale as 1/vBar, so the sample time and
    % the schedule speed floor carry the obligation together
    % (LTV_BICYCLE_MODEL.md).
    corneringStiffness = double(cfg.tire.corneringStiffness(:));
    if isscalar(corneringStiffness)
        corneringStiffness = repmat(corneringStiffness, 2, 1);
    end
    lateralRate = sum(corneringStiffness) ...
        / (cfg.vehicle.m*cfg.model.scheduleSpeedFloor);
    yawRate = (cfg.vehicle.lf^2*corneringStiffness(1) ...
            + cfg.vehicle.lr^2*corneringStiffness(2)) ...
        / (cfg.vehicle.Iz*cfg.model.scheduleSpeedFloor);
    stiffestRate = max(lateralRate, yawRate);
    if cfg.controller.sampleTime*stiffestRate >= 2.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "The forward-Euler stage step is unstable at the declared " ...
            + "schedule speed floor: sampleTime * %.3f 1/s must stay " ...
            + "below 2.", stiffestRate);
    end
    localValidateTerminal(cfg);
end

function localValidateNonnegativeInteger(value, name)
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
            || ~isfinite(value) || value < 0.0 || value ~= round(value)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must be a nonnegative integer.", name);
    end
end

function localValidateNonnegativeScalar(value, name)
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
            || ~isfinite(value) || value < 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must be a nonnegative finite scalar.", name);
    end
end

function localValidateTerminal(cfg)
% The terminal set's tail: positive knobs, a backup deceleration inside
% the actuator box, and the braking and lateral budgets inside every
% axle's friction polygon together - the braking shares longitudinally,
% the steady-cornering shares laterally, the affine load transfer under
% the backup deceleration - at the same inscribed fraction the stage
% rows use. The heading budget itself is route-dependent (the curvature
% enters) and is checked where the program is built.
    if ~isfield(cfg, "terminal") || ~isstruct(cfg.terminal) ...
            || ~isscalar(cfg.terminal)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "cfg.terminal must be a scalar structure.");
    end
    names = ["backupDeceleration", "lateralAccelerationMaximum", ...
        "lateralBandwidth", "lateralVelocityMaximum", ...
        "yawRateErrorMaximum"];
    strictlyPositive = [true, true, true, false, false];
    for nameIdx = 1:numel(names)
        name = names(nameIdx);
        if ~isfield(cfg.terminal, name)
            error("collisionAvoidanceController:invalidConfiguration", ...
                "terminal.%s must be declared.", name);
        end
        value = cfg.terminal.(name);
        if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
                || ~isfinite(value) || value < 0.0 ...
                || (strictlyPositive(nameIdx) && value <= 0.0)
            error("collisionAvoidanceController:invalidConfiguration", ...
                "terminal.%s must be a finite nonnegative scalar%s.", ...
                name, localPositiveSuffix(strictlyPositive(nameIdx)));
        end
    end
    deceleration = cfg.terminal.backupDeceleration;
    lateralAcceleration = cfg.terminal.lateralAccelerationMaximum;
    if deceleration > -cfg.actuation.longitudinalAccelerationMinimum
        error("collisionAvoidanceController:invalidConfiguration", ...
            "terminal.backupDeceleration exceeds the actuator's " ...
            + "braking limit -actuation.longitudinalAccelerationMinimum.");
    end
    parameters = axleFrictionParameters(cfg);
    wheelbase = cfg.vehicle.lf+cfg.vehicle.lr;
    lateralShare = [cfg.vehicle.lr; cfg.vehicle.lf]/wheelbase;
    for axleIdx = 1:2
        longitudinalForce = parameters.brakeDistribution(axleIdx) ...
            * parameters.mass*deceleration;
        lateralForce = lateralShare(axleIdx)*parameters.mass ...
            * lateralAcceleration;
        normalLoad = parameters.staticNormalLoad(axleIdx) ...
            + parameters.normalLoadAccelerationSlope(axleIdx) ...
                * (-deceleration);
        capacity = parameters.inscribedFraction ...
            * parameters.frictionCoefficient(axleIdx)*normalLoad;
        if hypot(longitudinalForce, lateralForce) > capacity
            error("collisionAvoidanceController:invalidConfiguration", ...
                "terminal.backupDeceleration %.2f m/s^2 with " ...
                + "terminal.lateralAccelerationMaximum %.2f m/s^2 " ...
                + "exceeds axle %d's friction polygon (%.0f N of " ...
                + "%.0f N).", deceleration, lateralAcceleration, ...
                axleIdx, hypot(longitudinalForce, lateralForce), capacity);
        end
    end
end

function suffix = localPositiveSuffix(strictlyPositive)
    if strictlyPositive
        suffix = ", strictly positive";
    else
        suffix = "";
    end
end
