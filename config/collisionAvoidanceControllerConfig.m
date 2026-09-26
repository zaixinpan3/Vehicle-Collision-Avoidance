function cfg = collisionAvoidanceControllerConfig(userCfg)
% collisionAvoidanceControllerConfig Defaults and merge for the controller.
%
% Returns the predictive CBF and sampled CLF configuration, with supplied
% overrides merged recursively over the declared defaults. Every field
% the controller reads is defined here; a missing field is a
% configuration error at the consuming module rather than a silent
% default there.
% Signed braking-ratio limits are validated here and normalized to
% dimensionless double scalars before model and tire modules consume them.
%
% The defaults describe a mid-size passenger car with input [deltaF; beta],
% locally linearized modified Fiala tires and the held-input Frenet LTV bicycle
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
    if isempty(userCfg) || ~isfield(userCfg,'collision') ...
            || ~isfield(userCfg.collision,'safetyMarginMeters')
        % Resolve the default for the selected hold duration. An explicit
        % physical clearance remains authoritative, including an explicit 0.
        cfg.collision.safetyMarginMeters = .10*max(1,cfg.controller.sampleTime/.05);
    end
end

function cfg = localDefaults()
    cfg = struct();

    % Route-following cruise demand of the CLF.
    cfg.referenceSpeed = 15.0;

    % Joint support certificates define collision and encounter-exit constraints.
    % Continuations retain the complete certificate, including its directions.
    % sampleTime is the common prediction-node interval and input-hold period.
    % poseTrustRadius [m; m; rad] sizes the curved-road chart linearization
    % allowance around the seed pose; it is not enforced as a constraint, so
    % the allowance is not guaranteed once the plan leaves that range.
    cfg.controller = struct("sampleTime",0.05,"horizonSteps",16, ...
        "minimumHorizonSteps",4,"maximumHorizonSteps",512,"stationTrustRadius",2.0, ...
        "poseTrustRadius",[2;4;0.5]);
    % NRMM/VFFM references: geometry-only initialization for the hard SOCP.
    % Lengths are in meters; clearanceAllowanceMeters shapes the seed only.
    % It does not change the hard collision clearance or authorize execution.
    cfg.admission = struct("widthScale",1.2,"minimumWidthMeters",3.0, ...
        "clearanceAllowanceMeters",0.2,"headingWeight",4.0,"regularizationWeight",0.02);
    % Small trajectory regularization for fixed-direction convex optimization.
    cfg.jointCertificate = struct("proximalWeight",1.0e-3);
    % Prediction under per-hold feedback u_k = v_k + K_k (xhat_k - z_k) from
    % the second hold on. Active trajectory models use a backward stage-wise
    % design; cruise models retain the cruise gain. Both scale the speed
    % column by speedGainScale and omit the lateral-velocity column unless
    % lateralVelocityFeedback is on. The actual finite-horizon enclosure and
    % actuator reserves determine admission; boundedness is not assumed.
    cfg.feedbackPrediction = struct("enabled",true,"speedGainScale",0.5, ...
        "lateralVelocityFeedback",false);
    % A fresh frame with a target tries these reaction strengths in order: each
    % finite entry scales the CLF input weight of the target-reactive policy
    % (ltvBicycleModel.reactionGains; larger is weaker) and Inf is the
    % ego-only feedback tube. relevanceMeters (m): records within this support
    % residual of binding at the optimizer center get full design weight;
    % exitWeight (1) scales the exit record's weight; terminalWeight (1) scales
    % the cruise CLF matrix that penalizes the final ego deviation.
    cfg.feedbackPrediction.targetReaction = struct("inputWeightScales",[Inf,30,100], ...
        "relevanceMeters",2.0,"exitWeight",1.0,"terminalWeight",0.0);
    % safetyMarginMeters is added to every ego-target separation row: the
    % planned footprints must stay this far apart at each hold node.
    % A 0.10 m clearance prevents the observed grazing overlaps in the 50 ms
    % stress campaign. It is an engineering reserve, not a whole-hold proof.
    cfg.collision = struct("cbfRate",2.0,"safetyMarginMeters",0.10);
    % taylorOrder is the minimum order of the offline whole-hold enclosures
    % (terminal family synthesis, fixedPredict audits). The online certificate
    % is evaluated at the hold nodes and does not use it.
    cfg.encounter = struct("taylorOrder",6, ...
        "numericalMargin",1.0e-6,"maximumCarriedMargin",1.0,"inputRateWeight",0.02, ...
        "referencePhaseRadius",2.0);

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
        "brakingRatioMinimum", -1.0, ...
        "brakingRatioMaximum", 1.0);

    % Passive flat-road force, separate from gross commanded acceleration.
    % Scenario adapters replace these generic values with plant parameters.
    cfg.roadLoad = struct( ...
        "airDensity", 1.225, ...
        "dragCoefficient", 0.30, ...
        "frontalArea", 2.2, ...
        "rollingCoefficient", 0.01, ...
        "rollingSpeedCoefficient", 0.0, ...
        "rollingQuarticCoefficient", 0.0, ...
        "rollingTransitionSpeed", 0.5);

    % State/slip ranges are diagnostic envelopes, not online constraints.
    % Only actuator bounds constrain the input; scheduleSpeedFloor regularizes
    % the tire model. Geometry uses actuator-reachable state enclosures.
    cfg.model = struct( ...
        "speedMinimum", 0.0, ...
        "speedMaximum", 18.0, ...
        "lateralDomainRadius", 12.0, ...
        "scheduleSpeedFloor", 1.0, ...
        "frontWheelSteeringAngleMaximum", deg2rad(40.0), ...
        "frontWheelSteeringRateMaximum", Inf, ...
        "brakingRatioRateMaximum", Inf, ...
        "slipAngleMaximum", deg2rad([10.0; 10.0]), ...
        "headingDomainRadius", 0.40, "linearizationPolicy", "trajectory", ...
        "lateralVelocityMaximum", 12.0, ...
        "yawRateMaximum", 5.0, ...
        "ltvModelErrorRateBound", zeros(6, 1), ...
        "plantModelResidualRateBound", zeros(6, 1));

    % Road-geometry implementation allowances.
    cfg.road = struct( ...
        "orthonormalTolerance", 1.0e-9, ...
        "parameterRangeTolerance", 1.0e-3);

    % Discrete Riccati error scales and the LQR input weights of the cruise
    % gain design and the initializer metric. The SOCP objective has no
    % input-effort term (removed 2026-09-24).
    % decreaseRateFraction retains a strict gap between nominal contraction
    % and the reported robust sampled dissipation factor. relaxationWeight
    % penalizes the squared nonnegative slack in the sampled CLF norm cone.
    % Legacy certificateSpeedFloor and samplePoints do not select constraints.
    cfg.clf = struct( ...
        "lateralPositionErrorScale", 0.5, ...
        "headingErrorScale", 0.1, ...
        "speedErrorScale", 0.25, ...
        "lateralVelocityErrorScale", 0.5, ...
        "yawRateErrorScale", 0.2, ...
        "frontWheelSteeringAngleWeight", 1.0, ...
        "brakingRatioWeight", 1.0, ...
        "decreaseRateFraction", 0.9, ...
        "certificateSpeedFloor", 5.0, ...
        "relaxationWeight", 100.0, ...
        "referenceOffset", zeros(5, 1), ...
        "referenceRate", zeros(5, 1), "referenceEpoch", 0.0, ...
        "samplePoints", "stageNodes");
    % Each convex subproblem calls the hook with (phase,program), P/q/A/b/cones
    % and lifted coordinates. A normal frame uses one solve. Failed fresh
    % admissions may try alternate directions and bounded phase-I recovery;
    % every issued decision passes independent hard-constraint checks.
    % constraintTolerance enters the pre-solve physical row reserves.
    % frameDeadlineSeconds is a complete controller-frame acceptance deadline.
    % A finite exit may require more stages than the performance window.
    cfg.solver = struct( ...
        "jointFunction", [], ...
        "maxIterations", 400, ...
        "certificateSearchTimeLimit", 5.0, ...
        "frameDeadlineSeconds", inf, ...
        "constraintTolerance", 1.0e-8, ...
        "optimalityTolerance", 1.0e-7, ...
        "lexicographicTieTolerance", 1.0e-6);
    % Two actuator coordinates per hold and one first-hold CLF cone remain.

    % Default target rectangle when an estimate publishes no extent.
    cfg.target = struct( ...
        "defaultLength", 4.8, ...
        "defaultWidth", 1.9);
end

function base = localMergeStructure(base, overrides)
    fields = fieldnames(overrides);
    for fieldIdx = 1:numel(fields)
        name = fields{fieldIdx};
        value = overrides.(name);
        if ~isfield(base, name)
            error("collisionAvoidanceController:invalidConfiguration", ...
                "Unknown controller configuration field: %s.", name);
        end
        if isstruct(base.(name)) ...
                && isstruct(value) && isscalar(value) ...
                && isscalar(base.(name))
            base.(name) = localMergeStructure(base.(name), value);
        else
            base.(name) = value;
        end
    end
end

function actuation = localNormalizeActuation(actuation)
% Keep the signed braking-ratio contract at the configuration boundary.
    if ~isstruct(actuation) || ~isscalar(actuation)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "cfg must contain a scalar actuation structure.");
    end
    names = ["brakingRatioMinimum", ...
        "brakingRatioMaximum"];
    for name = names
        value = actuation.(name);
        if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
                || ~isfinite(value)
            error("collisionAvoidanceController:invalidConfiguration", ...
                "actuation.%s must be a finite real scalar.", name);
        end
        actuation.(name) = double(value);
    end
    minimum = actuation.brakingRatioMinimum;
    maximum = actuation.brakingRatioMaximum;
    if minimum < -1.0 || maximum > 1.0 ...
            || minimum >= maximum || minimum > 0.0 || maximum < 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "Signed braking-ratio limits must satisfy " ...
            + "-1 <= betaMinimum <= 0 <= betaMaximum <= 1 with positive width.");
    end
end

function localValidate(cfg)
    for name = ["widthScale","minimumWidthMeters","headingWeight","regularizationWeight"]
        value=cfg.admission.(name);
        if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ~isfinite(value) || value<=0
            error("collisionAvoidanceController:invalidConfiguration", ...
                "admission.%s must be a positive finite scalar.",name);
        end
    end
    localValidateNonnegativeScalar(cfg.admission.clearanceAllowanceMeters, ...
        "admission.clearanceAllowanceMeters");
    for name = "proximalWeight"
        validateattributes(cfg.jointCertificate.(name),{'double'}, ...
            {'scalar','real','finite','positive'});
    end
    validateattributes(cfg.collision.cbfRate,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.feedbackPrediction.enabled,{'logical'},{'scalar'});
    validateattributes(cfg.feedbackPrediction.speedGainScale,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(cfg.feedbackPrediction.lateralVelocityFeedback,{'logical'},{'scalar'});
    validateattributes(cfg.feedbackPrediction.targetReaction.inputWeightScales,{'double'},{'row','nonempty','positive','nonnan'});
    validateattributes(cfg.feedbackPrediction.targetReaction.relevanceMeters,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(cfg.feedbackPrediction.targetReaction.exitWeight,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(cfg.feedbackPrediction.targetReaction.terminalWeight,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(cfg.collision.safetyMarginMeters,{'double'},{'scalar','real','finite','nonnegative'});
    for name = ["m", "Iz", "lf", "lr", "wheelbase", "length", "width", "gravity"]
        localValidateNonnegativeScalar(cfg.vehicle.(name), "vehicle."+name);
        if cfg.vehicle.(name) == 0
            error("collisionAvoidanceController:invalidConfiguration", "vehicle.%s must be positive.", name);
        end
    end
    if ~isscalar(string(cfg.model.linearizationPolicy)) ...
            || ~any(string(cfg.model.linearizationPolicy)==["currentState","trajectory","cruise"])
        error("collisionAvoidanceController:invalidConfiguration","Unknown linearization policy.");
    end
    for name = ["speedMinimum", "speedMaximum", "scheduleSpeedFloor", "headingDomainRadius", ...
            "frontWheelSteeringAngleMaximum"]
        localValidateNonnegativeScalar(cfg.model.(name), "model."+name);
    end
    localValidateNonnegativeScalar(cfg.referenceSpeed, "referenceSpeed");
    localValidateNonnegativeScalar(cfg.controller.sampleTime, "controller.sampleTime");
    localValidateNonnegativeScalar(cfg.controller.horizonSteps, "controller.horizonSteps");
    validateattributes(cfg.solver.certificateSearchTimeLimit,{'double'},{'scalar','real','finite','positive'});
    validateattributes(cfg.solver.lexicographicTieTolerance,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(cfg.model.frontWheelSteeringRateMaximum,{'double'},{'scalar','real','positive'});
    validateattributes(cfg.model.brakingRatioRateMaximum,{'double'},{'scalar','real','positive'});
    validateattributes(cfg.encounter.taylorOrder, {'double'}, ...
        {'scalar', 'integer', '>=', 3, '<=', 10});
    validateattributes(cfg.encounter.referencePhaseRadius, {'double'}, ...
        {'scalar','real','finite','positive'});
    for name = ["numericalMargin", "maximumCarriedMargin", ...
            "inputRateWeight"]
        localValidateNonnegativeScalar(cfg.encounter.(name), "encounter."+name);
    end
    if cfg.encounter.numericalMargin <= 0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "encounter.numericalMargin must be positive.");
    end
    validateattributes(cfg.clf.referenceRate, {'double'}, ...
        {'real', 'finite', 'size', [5, 1]});
    validateattributes(cfg.clf.referenceOffset, {'double'}, ...
        {'real', 'finite', 'size', [5, 1]});
    validateattributes(cfg.clf.referenceEpoch, {'double'}, {'real', 'finite', 'scalar'});
    validateattributes(cfg.model.lateralVelocityMaximum, {'double'}, {'real', 'finite', 'scalar', 'positive'});
    validateattributes(cfg.model.yawRateMaximum, {'double'}, {'real', 'finite', 'scalar', 'positive'});
    for name = string(fieldnames(cfg.roadLoad)).'
        localValidateNonnegativeScalar(cfg.roadLoad.(name), "roadLoad."+name);
    end
    if cfg.roadLoad.rollingTransitionSpeed <= 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "roadLoad.rollingTransitionSpeed must be positive.");
    end
    if cfg.model.speedMaximum <= cfg.model.speedMinimum
        error("collisionAvoidanceController:invalidConfiguration", ...
            "The finite speed domain must have a positive width.");
    end
    if cfg.model.scheduleSpeedFloor <= 0.0 ...
            || cfg.model.scheduleSpeedFloor > cfg.model.speedMaximum
        error("collisionAvoidanceController:invalidConfiguration", ...
            "scheduleSpeedFloor must be positive and no greater than speedMaximum.");
    end
    localValidateNonnegativeScalar(cfg.controller.stationTrustRadius, ...
        "controller.stationTrustRadius");
    validateattributes(cfg.controller.poseTrustRadius,{'double'}, ...
        {'size',[3,1],'real','finite','positive'});
    localValidateNonnegativeScalar(cfg.model.lateralDomainRadius, ...
        "model.lateralDomainRadius");
    for name = ["ltvModelErrorRateBound", "plantModelResidualRateBound"]
        value = cfg.model.(name);
        if ~isnumeric(value) || ~isreal(value) || ~isvector(value) ...
                || numel(value) ~= 6 || any(~isfinite(value)) || any(value < 0)
            error("collisionAvoidanceController:invalidConfiguration", ...
                "model.%s must contain six finite nonnegative continuous-rate bounds.", name);
        end
        cfg.model.(name) = double(value(:));
    end
    if cfg.controller.stationTrustRadius == 0.0 || cfg.model.lateralDomainRadius == 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "Station and lateral domain radii must be positive.");
    end
    if cfg.controller.horizonSteps < 1 ...
            || cfg.controller.horizonSteps ~= round(cfg.controller.horizonSteps)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "controller.horizonSteps must be a positive integer.");
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
    localValidateNonnegativeScalar(cfg.clf.decreaseRateFraction, "clf.decreaseRateFraction");
    if cfg.clf.decreaseRateFraction <= 0.0 ...
            || cfg.clf.decreaseRateFraction >= 1.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "clf.decreaseRateFraction must lie in (0, 1) to retain a finite robust dissipation bound.");
    end
    if ~isempty(cfg.solver.jointFunction) ...
            && ~isa(cfg.solver.jointFunction, "function_handle")
        error("collisionAvoidanceController:invalidConfiguration", ...
            "solver.jointFunction must be empty or a function handle.");
    end
    for name = ["constraintTolerance", "optimalityTolerance"]
        localValidateNonnegativeScalar(cfg.solver.(name), "solver."+name);
        if cfg.solver.(name) == 0.0
            error("collisionAvoidanceController:invalidConfiguration", ...
                "solver.%s must be positive.", name);
        end
    end
    localValidateNonnegativeScalar(cfg.solver.maxIterations, "solver.maxIterations");
    validateattributes(cfg.solver.frameDeadlineSeconds,{'double'},{'scalar','real','positive'});
    if ~ismember(string(cfg.clf.samplePoints),["stageNodes","endpoints","controlPoints"])
        error("collisionAvoidanceController:invalidConfiguration", ...
            "clf.samplePoints must be ""stageNodes"", ""endpoints"" or ""controlPoints"".");
    end
    localValidateNonnegativeScalar(cfg.controller.minimumHorizonSteps, "controller.minimumHorizonSteps");
    localValidateNonnegativeScalar(cfg.controller.maximumHorizonSteps, "controller.maximumHorizonSteps");
    if cfg.controller.maximumHorizonSteps~=fix(cfg.controller.maximumHorizonSteps) ...
            || cfg.controller.maximumHorizonSteps<max(cfg.controller.horizonSteps,cfg.controller.minimumHorizonSteps)
        error('collisionAvoidanceController:invalidConfiguration', ...
            'controller.maximumHorizonSteps must be an integer at least as large as the requested horizon.');
    end
    if cfg.controller.minimumHorizonSteps < 1 ...
            || cfg.controller.minimumHorizonSteps ~= fix(cfg.controller.minimumHorizonSteps)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "controller.minimumHorizonSteps must be a positive integer.");
    end
    if cfg.solver.maxIterations < 1 ...
            || cfg.solver.maxIterations ~= fix(cfg.solver.maxIterations)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "solver.maxIterations must be a positive integer.");
    end
    localValidateVehicleAndTire(cfg);
end

function localValidateNonnegativeScalar(value, name)
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
            || ~isfinite(value) || value < 0.0
        error("collisionAvoidanceController:invalidConfiguration", ...
            "%s must be a nonnegative finite scalar.", name);
    end
end

function localValidateVehicleAndTire(cfg)
    friction = cfg.tire.frictionCoefficient;
    if ~isnumeric(friction) || ~isreal(friction) || ~isvector(friction) ...
            || ~ismember(numel(friction), [1, 2]) ...
            || any(~isfinite(friction)) || any(friction <= 0.0)
        error("collisionAvoidanceController:invalidConfiguration", ...
            "tire.frictionCoefficient must be positive scalar or front/rear values.");
    end
    for name = ["gravity", "lf", "lr"]
        localValidateNonnegativeScalar(cfg.vehicle.(name), "vehicle."+name);
        if cfg.vehicle.(name) == 0.0
            error("collisionAvoidanceController:invalidConfiguration", ...
                "vehicle.%s must be positive.", name);
        end
    end
end
