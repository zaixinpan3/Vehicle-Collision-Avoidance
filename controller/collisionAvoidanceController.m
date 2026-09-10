function [command, predictedInput, planningProblem, certificate] = ...
        collisionAvoidanceController(egoState, targetEstimate, laneCenterline, cfg, controllerState)
%collisionAvoidanceController Complete hard predictive certificate and sampled CLF.
% The fourth output is the accepted continuation; supply it at the next sample.
% Every prediction interval is certified. The retained certificate preserves
% its absolute deadline and all admitted target obligations. New targets need
% joint certification. Completed encounters return no further control.
    persistent previousCertificate
    if nargin == 1 && (ischar(egoState) || isstring(egoState))
        if ~isscalar(string(egoState)) || string(egoState) ~= "resetNominalTrajectory"
            error("collisionAvoidanceController:invalidAction", "Use resetNominalTrajectory.");
        end
        previousCertificate = [];
        command = []; predictedInput = []; planningProblem = []; certificate = [];
        return;
    end
    explicitState = nargin >= 5;
    if ~explicitState, controllerState = previousCertificate; end
    if nargin < 4, cfg = []; end
    if nargin < 3, laneCenterline = []; end
    if nargin < 2, targetEstimate = []; end
    timer = tic;
    cfg = localControllerConfiguration(cfg);
    [ego, lane, road, observations] = readPlanningInputs(egoState, targetEstimate, laneCenterline, cfg);
    model = localFiniteModel(ego, lane, road, cfg);
    identity = struct("configuration", rmfield(cfg, "solver"), "lane", lane, ...
        "accelerationBias", ego.longitudinalAccelerationBias);
    identity.road = road;
    identity.perceptionRange = ego.perceptionRange;
    model.perceptionRange = ego.perceptionRange;
    if ~isempty(controllerState)
        [command, predictedInput, planningProblem, certificate] = ...
            hardEncounterBarrier.advance(controllerState, ego, model, observations, identity, @localCommand);
        if ~explicitState, previousCertificate = certificate; end
        return;
    end
    hardEncounterBarrier.validateAdmission(ego, observations);
    encounters = struct("key", {}, "contract", {}, "center", {}, "radius", {}, ...
        "time", {}, "halfLength", {}, "halfWidth", {}, "nominalCenter", {});
    for index = 1:numel(observations)
        encounters(end+1) = targetPrediction.admit(observations(index), model.stateTime, lane, cfg); %#ok<AGROW>
    end
    model.encounters = encounters;
    active = ~isempty(encounters);
    [model.exitSteps, model.exitMargin] = localExitSchedule(model);
    preparationSeconds = toc(timer);
    prediction = ltvBicycleModel.finitePredict(model, []);
    predictionSeconds = toc(timer)-preparationSeconds;
    anchor = localRateLimitedAnchor(reshape(prediction.referencePlan,2,[]),model);
    phase = tic;
    try
        [model,prediction,anchor] = localPlanningWindow(model,prediction,anchor);
        qp = formulateAvoidanceProblem(model,prediction,anchor);
    catch exception
        if any(string(exception.identifier)==["collisionAvoidanceController:roadBoundaryCoverageGap", ...
                "collisionAvoidanceController:unsupportedReferenceJump"])
            error("collisionAvoidanceController:noCertifiedContinuation","%s",exception.message);
        end
        rethrow(exception);
    end
    formulationSeconds = toc(phase);
    phase = tic;
    [result,qp] = solveHardCbfClf(qp,cfg);
    solveSeconds = toc(phase);
    phase = tic;
    check = certifyAvoidancePlan(qp,prediction,model,result.decision);
    verificationSeconds = toc(phase);
    if ~result.feasible || ~check.accepted
        error("collisionAvoidanceController:noCertifiedContinuation", ...
            "The joint hard program supplied no checked witness: %s; %s.", ...
            result.message,strjoin(check.failedConditions,","));
    end
    phase = tic;
    decision = result.decision;
    source = "checkedOptimization";
    margin = check.margin;
    predictedInput = reshape(decision(qp.layout.planIndex), 2, []);
    command = localCommand(predictedInput, model, prediction, 1);
    command.measurementTime = model.stateTime;
    command.actuationTime = model.stateTime;
    command.holdSeconds = model.sampleTime;
    predictedState = reshape(pagemtimes(prediction.egoStateMatrix, predictedInput(:)), 6, [])+prediction.egoStateOffset;
    certificate = struct("version", 14, "identity", identity, "stateTime", model.stateTime, ...
        "deadline", model.stateTime+prediction.stageCount*model.sampleTime, ...
        "remainingSteps", prediction.stageCount, "margin", margin, ...
        "plan", predictedInput, "decision", decision, "qp", qp, "prediction", prediction, ...
        "predictedState", predictedState, "appliedInput", predictedInput(:,1), ...
        "scheduledInput",command.actuatorInput, ...
        "stateErrorBound", prediction.egoStateErrorBound, ...
        "encounters", encounters, "acceptance", check, "safetyScope", "completeEncounterForDeclaredInclusion", ...
        "certifiedDuration", prediction.stageCount*model.sampleTime);
    metadata = struct("planCertified", check.accepted, "certificateSource", source, ...
        "fallbackUsed", false, "solverCallCount", result.solverCalls, ...
        "carriedMargin", margin, "requiredMargin", model.requiredMargin, ...
        "horizonSteps", prediction.stageCount, "tailSteps", 0, "deadline", certificate.deadline, ...
        "activeTargetKeys", string({encounters.key}), ...
        "dischargedTargetKeys", strings(1,0), ...
        "clfRelaxation", decision(qp.layout.relaxationIndex), "clfDecayRate", qp.clf.decayRate, ...
        "collisionDiscretization", "sweptBernsteinCells", "acceptance", check, ...
        "hasTarget", active, "postSolveCertificationPerformed", true, "runtimeSeconds", toc(timer));
    metadata.runtime = struct("inputPreparationSeconds", preparationSeconds, ...
        "predictionSeconds", predictionSeconds, "formulationAndWitnessSeconds", formulationSeconds, ...
        "solveSeconds", solveSeconds, "acceptanceAndCommitSeconds", verificationSeconds+toc(phase), ...
        "diagnosticsSeconds", 0);
    metadata.solverAlgorithm = "Clarabel predictive CBF-CLF SOCP";
    metadata.setMembershipUpdate = false;
    metadata.certificateCompatible = false;
    metadata.carriedWitnessFeasible = false;
    metadata.safetyScope = certificate.safetyScope;
    metadata.certifiedDuration = certificate.certifiedDuration;
    metadata.lookaheadDuration = prediction.stageCount*model.sampleTime;
    metadata.inputDelaySeconds = 0;
    metadata.commandActuationTime = command.actuationTime;
    metadata.recursiveFeasibilityClaimed = true;
    metadata.exactPredictionAssumptionsHold = false;
    metadata.tireForceConstraintScope = "nominalScheduledForceWithSeparatePlantResidual";
    metadata.executedContinuousGenerator = [prediction.continuousA(:,:,1), ...
        prediction.continuousB(:,:,1),prediction.continuousC(:,1)];
    metadata.executedResidualRateBound = prediction.modelErrorRateBound(:,1);
    trackingError = predictedState(2:6, :)-qp.clf.referenceStart ...
        -qp.clf.referenceRate*((0:prediction.stageCount)*model.sampleTime);
    metadata.clfValueProfile = sum(trackingError.*(qp.clf.lyapunovMatrix*trackingError), 1);
    metadata.clfInitialValue = metadata.clfValueProfile(1);
    metadata.clfOperatingCurvature = qp.clf.certificate.operatingCurvature;
    metadata.clfOperatingInput = qp.clf.certificate.operatingInput;
    metadata.clfMetricChanged = false;
    metadata.clfReferenceSwitchValue = 0;
    metadata.jointObjectiveValue = 0.5*decision.'*qp.Hessian*decision+qp.linear.'*decision+qp.constant;
    metadata.clfRelaxationCost = model.sampleTime*cfg.clf.relaxationWeight*sum(metadata.clfRelaxation.^2);
    metadata.hardRowViolation = check.hardRowViolation;
    planningProblem = struct("problemClass", qp.problemClass, "qp", qp, "layout", qp.layout, ...
        "prediction", prediction, "model", model, "decision", decision, "plan", predictedInput(:), ...
        "inputPlan", predictedInput, "tailPlan", zeros(2, 0), "metadata", metadata);
    [certificate, planningProblem] = hardEncounterBarrier.admit(certificate, planningProblem);
    if ~explicitState, previousCertificate = certificate; end
end

function model = localFiniteModel(ego, lane, road, cfg)
    projection = laneGeometry.project(ego.position, lane);
    heading = atan2(sin(ego.yaw-projection.heading), cos(ego.yaw-projection.heading));
    [radius, chartValid] = stateUncertainty.toFrenet(ego.modelState, ego.stateErrorBound, lane);
    if any(ego.stateErrorBound) && (~chartValid || ~isfinite(ego.stateTime))
        error("collisionAvoidanceController:invalidUncertaintyChart", ...
            "Uncertain admission requires a timestamp and one invertible projection chart.");
    end
    previousInput = zeros(2, 1);
    if ~isempty(ego.heldActuatorInput), previousInput = ego.heldActuatorInput; end
    model = struct("cfg", cfg, "lane", lane, "road", road, ...
        "stateTime", ego.stateTime, "sampleTime", cfg.controller.sampleTime, ...
        "horizonSteps", cfg.controller.horizonSteps, "referenceSpeed", cfg.referenceSpeed, ...
        "initialEgoState", [projection.station; projection.lateralPosition; heading; ego.modelState(4:6)], ...
        "initialFrenetErrorBound", radius, "longitudinalAccelerationBias", ego.longitudinalAccelerationBias, ...
        "previousInput", previousInput, ...
        "requiredMargin", 0);
end

function [steps, margin] = localExitSchedule(model)
    steps = repmat(model.horizonSteps, numel(model.encounters), 1);
    margin = inf;
    for encounter = model.encounters(:).'
        if model.stateTime+model.horizonSteps*model.sampleTime > ...
                encounter.contract.validUntil+128*eps(max(1, abs(encounter.contract.validUntil)))
            error("collisionAvoidanceController:expiredEncounterContract", ...
                "The complete encounter must fit within target motion validity.");
        end
    end
end

function anchor = localRateLimitedAnchor(inputs,model)
% Seed the nonlinear prediction with controls inside the actuator envelope.
% This initializes optimization only; it is never an executable fallback.
    cfg = model.cfg;
    lower = [-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    upper = [cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    change = model.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    prior = model.previousInput;
    for stage = 1:size(inputs,2)
        inputs(:,stage) = min(max(inputs(:,stage),max(lower,prior-change)),min(upper,prior+change));
        prior = inputs(:,stage);
    end
    anchor = inputs(:);
end

function cfg = localControllerConfiguration(userCfg)
    persistent cachedUserConfiguration cachedConfiguration cacheValid
    if isempty(cacheValid)
        cacheValid = false;
    end
    if nargin < 1 || isempty(userCfg)
        userCfg = [];
    end
    if cacheValid && isequaln(userCfg, cachedUserConfiguration)
        cfg = cachedConfiguration;
        return;
    end
    localAddConfigurationPath();
    cfg = collisionAvoidanceControllerConfig(userCfg);
    cachedUserConfiguration = userCfg;
    cachedConfiguration = cfg;
    cacheValid = true;
end

function localAddConfigurationPath()
    if exist("collisionAvoidanceControllerConfig", "file") == 2
        return;
    end
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    configurationRoot = fullfile(repositoryRoot, "config");
    if isfolder(configurationRoot)
        addpath(configurationRoot);
    end
end


function command = localCommand(inputPlan, model, prediction,stage)
    firstInput = inputPlan(:,stage);
    cfg = model.cfg;
    state = prediction.egoStateMatrix(:,:,stage)*inputPlan(:)+prediction.egoStateOffset(:,stage);
    forceScheduleSpeed = max(prediction.scheduleSpeedProfile(stage), ...
        cfg.model.scheduleSpeedFloor);
    tire = modifiedFialaTire.parameters(cfg);
    steeringAngle = firstInput(1);
    brakingRatio = firstInput(2);
    longitudinalAcceleration = modifiedFialaTire.accelerationGain(cfg)*brakingRatio;
    % Paper Eqs. (8)-(9); use the same scheduled Fiala tangent as prediction.
    frontSlipAngle = (state(5)+cfg.vehicle.lf*state(6)) ...
        / forceScheduleSpeed-steeringAngle;
    rearSlipAngle = (state(5)-cfg.vehicle.lr*state(6)) ...
        / forceScheduleSpeed;
    [tireSlope, ratioSlope, tireIntercept] = modifiedFialaTire.linearize( ...
        prediction.scheduleCurvature(stage), prediction.scheduleSpeedProfile(stage), ...
        prediction.scheduleBrakingRatio(stage), cfg);
    axleLateralForce = tireSlope.*[frontSlipAngle; rearSlipAngle] ...
        +ratioSlope*brakingRatio+tireIntercept;
    axleLongitudinalForce = modifiedFialaTire.longitudinalForce(brakingRatio, cfg);
    [roadForce, ~, roadComponents] = longitudinalRoadLoad(state(4), cfg);
    netLongitudinalAcceleration = longitudinalAcceleration ...
        - roadForce/cfg.vehicle.m+model.longitudinalAccelerationBias;
    axleRollingResistance = roadComponents.rollingResistanceForce ...
        * tire.staticNormalLoad/(cfg.vehicle.m*cfg.vehicle.gravity);
    axleContactForce = axleLongitudinalForce-axleRollingResistance;
    command = struct();
    command.brakingRatio = brakingRatio;
    command.longitudinalAcceleration = longitudinalAcceleration;
    command.bodyLongitudinalVelocityDerivative = ...
        netLongitudinalAcceleration+state(5)*state(6);
    command.lateralAcceleration = ...
        sum(axleLateralForce)/cfg.vehicle.m-state(4)*state(6);
    command.yawAcceleration = ...
        (cfg.vehicle.lf*axleLateralForce(1) ...
            - cfg.vehicle.lr*axleLateralForce(2))/cfg.vehicle.Iz;
    command.actuatorInput = [steeringAngle; brakingRatio];
    command.actuatorInputOrder = [ ...
        "frontWheelSteeringAngle", "brakingRatio"];
    command.totalLongitudinalActuatorForce = sum(axleLongitudinalForce);
    command.totalLongitudinalTireForce = sum(axleContactForce);
    command.axleLongitudinalTireForce = axleContactForce;
    command.aerodynamicResistanceForce = roadComponents.aerodynamicForce;
    command.rollingResistanceForce = roadComponents.rollingResistanceForce;
    command.totalRoadLoadForce = roadForce;
    % Compatibility field consumed by the torque adapter: actuator force
    % before rolling loss, not contact force at the tire patch.
    command.axleLongitudinalForce = axleLongitudinalForce;
    command.axleLateralForce = axleLateralForce;
    command.axleNormalLoad = tire.staticNormalLoad;
    command.tireSideslipAngle = [frontSlipAngle; rearSlipAngle];
    command.frontWheelSteeringAngle = steeringAngle;
end

function [model, prediction, anchor] = localPlanningWindow(model, prediction, anchor)
% Require road coverage of the entire retained certificate.
    cfg = model.cfg;
    if nargin<3,anchor = prediction.referencePlan;end
    if isempty(model.road.boundaries),return;end
    count = prediction.stageCount;
    [frames,nominal] = laneGeometry.sweptCellFrames(model,prediction.cells,anchor);
    for index = 1:numel(prediction.cells)
        tube = prediction.cells(index);
        frame = frames(index);
        covered = true;
        for boundary = model.road.boundaries(:).'
            direction = boundary.longitudinalDirection;
            stations = direction.'*frame.tangent*[frame.stationLower,frame.stationUpper];
            extent = abs(direction.'*frame.lateral)*cfg.model.lateralDomainRadius ...
                +hypot(cfg.vehicle.length/2,cfg.vehicle.width/2) ...
                +abs(direction).'*frame.positionErrorBound;
            range = [min(stations)-extent,max(stations)+extent] ...
                +direction.'*(frame.origin-boundary.origin);
            covered = covered && range(1)>=boundary.parameterRange(1) ...
                && range(2)<=boundary.parameterRange(2);
        end
        if ~covered
            count = tube.stage-1;
            break;
        end
    end
    if count < prediction.stageCount
        error("collisionAvoidanceController:roadBoundaryCoverageGap", ...
            "The complete retained deadline must lie inside certified road coverage.");
    else
        prediction.geometryAnchor = anchor;
        prediction.geometryFrames = frames;
        prediction.geometryNominal = nominal;
    end
end
