function [command, predictedInput, planningProblem, certificate] = ...
        collisionAvoidanceController(egoState, targetEstimate, laneCenterline, cfg, controllerState)
%collisionAvoidanceController Information-state safe MPC with a carried recursive witness.
% Supply the fourth output at the next sample. Ego and target estimates carry
% bounded error boxes. The target follows the Cartesian constant-acceleration
% law exactly; the ego executes the accepted plan's first-stage affine
% generator. Every continuation frame conditions both boxes, verifies the
% shifted previous plan with its own carried data, and lets a fresh
% optimization replace it only with a verified plan of no larger safety
% value. The reported value is the predictive control barrier function.
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
    candidate = [];
    if ~isempty(controllerState)
        [model,candidate,initialization,originalEncounter] = hardEncounterBarrier.validateTransition( ...
            controllerState,ego,model,observations,identity);
        if ~isempty(initialization)
            model.initializationPlan = initialization;
            model.horizonSteps = max(model.horizonSteps,size(initialization,2));
        end
    else
        % Admission has no carried box to condition against, so the measured
        % speed centre itself must lie in the model domain; continuation
        % frames condition a straddling box against the successor box.
        if ego.modelState(4) < cfg.model.speedMinimum || ego.modelState(4) > cfg.model.speedMaximum
            error("collisionAvoidanceController:invalidInput", ...
                "The admitted ego speed centre must lie in the model domain.");
        end
        hardEncounterBarrier.validateAdmission(ego, observations, cfg);
        model.encounters = targetPrediction.admitExact(observations, model.stateTime, lane, cfg);
        originalEncounter = model.encounters;
    end
    encounters = model.encounters;
    preparationSeconds = toc(timer);
    witnessSeconds = 0;
    if ~isempty(candidate)
        candidate = hardEncounterBarrier.verifyCandidate(model,candidate);
        witnessSeconds = candidate.seconds;
        if ~candidate.check.accepted
            detail = "";
            if isfield(candidate.check,"violatedHardRows") && ~isempty(candidate.check.violatedHardRows)
                names = solveHardCbfClf.rowNames(candidate.qp);
                detail = sprintf("; violated hard rows %s by at most %.3g", ...
                    strjoin(unique(names(candidate.check.violatedHardRows)).',","),candidate.check.hardRowViolation);
            end
            error("collisionAvoidanceController:carriedWitnessRejected", ...
                "The carried witness failed verification on the conditioned information set (%s%s); " ...
                + "a premise of the declared plant, the target law or the measurement contract was violated.", ...
                strjoin(candidate.check.failedConditions,","),detail);
        end
    end
    fresh = [];
    freshFailure = "";
    planningTiming = struct("predictionSeconds",0,"formulationSeconds",0, ...
        "solveSeconds",0,"verificationSeconds",0,"attempts",0);
    try
        [freshModel,freshPrediction,freshQp,freshResult,freshCheck,planningTiming] = hardEncounterBarrier.plan(model);
        fresh = struct("model",freshModel,"prediction",freshPrediction,"qp",freshQp, ...
            "result",freshResult,"check",freshCheck);
    catch exception
        if isempty(candidate) || ~startsWith(string(exception.identifier),"collisionAvoidanceController:")
            rethrow(exception);
        end
        freshFailure = string(exception.identifier)+": "+string(exception.message);
    end
    if ~isempty(fresh) && ~isempty(candidate)
        % The fresh plan may replace the carried witness only without
        % raising the value function; a zero carried value is kept exactly.
        tolerance = cfg.solver.lexicographicTieTolerance*double(candidate.check.value>0);
        if fresh.check.value>candidate.check.value+tolerance
            freshFailure = sprintf("fresh value %.9g exceeds the carried value %.9g", ...
                fresh.check.value,candidate.check.value);
            fresh = [];
        end
    end
    if isempty(fresh)
        source = "carriedWitness";
        model = candidate.model;
        prediction = candidate.prediction;
        qp = candidate.qp;
        check = candidate.check;
        decision = candidate.decision;
        solverCalls = 0;
        optimizedStages = candidate.optimizedStages;
        tieResidual = 0;
    else
        source = "checkedOptimization";
        model = fresh.model;
        prediction = fresh.prediction;
        qp = fresh.qp;
        check = fresh.check;
        decision = fresh.result.decision;
        solverCalls = fresh.result.solverCalls;
        optimizedStages = prediction.stageCount;
        tieResidual = fresh.result.lexicographicTieResidual;
    end
    phase = tic;
    predictedInput = reshape(decision(qp.layout.planIndex), 2, []);
    command = localCommand(predictedInput, model, prediction, 1);
    command.measurementTime = model.stateTime;
    command.actuationTime = model.stateTime;
    command.holdSeconds = model.sampleTime;
    % Published successor nodes follow the exact held-input stage flows, the
    % declared plant, with the interval hull of the initial box as radius.
    % Conditioning against them keeps every later node box inside the
    % accepted plan's node boxes (INFORMATION_STATE_PCBF.md, Lemma 1).
    predictedState = localExactNodes(model, prediction, predictedInput);
    carried = hardEncounterBarrier.carriedData(prediction,qp,optimizedStages);
    terminalActive = optimizedStages==0;
    certificate = struct("version", 19, "identity", identity, "stateTime", model.stateTime, ...
        "deadline", model.stateTime+prediction.stageCount*model.sampleTime, ...
        "remainingSteps", optimizedStages, "margin", check.margin, ...
        "value", check.value, "stageViolation", check.stageViolation, ...
        "plan", predictedInput, "decision", decision, "qp", qp, "prediction", prediction, ...
        "predictedState", predictedState, "stateErrorBound", prediction.initialErrorBound, ...
        "appliedInput", predictedInput(:,1), "scheduledInput", command.actuatorInput, ...
        "encounters", encounters, "originalEncounter", originalEncounter, "acceptance", check, ...
        "safetyScope", "verifiedPredictionWithInvariantTerminalTail", ...
        "certifiedDuration", prediction.stageCount*model.sampleTime, ...
        "stages", carried.stages, "cellStage", carried.cellStage, "cellFrames", carried.cellFrames, ...
        "cellNormals", {carried.cellNormals}, "terminal", carried.terminal, "source", source);
    metadata = struct("planCertified", check.accepted, "certificateSource", source, ...
        "fallbackUsed", false, "solverCallCount", solverCalls, ...
        "carriedMargin", check.margin, "requiredMargin", model.requiredMargin, ...
        "horizonSteps", prediction.stageCount, "tailSteps", 0, "deadline", certificate.deadline, ...
        "activeTargetKeys", string({encounters.key}), ...
        "dischargedTargetKeys", strings(1,0), ...
        "clfRelaxation", decision(qp.layout.relaxationIndex), "clfDecayRate", qp.clf.decayRate, ...
        "collisionDiscretization", "sweptBernsteinCells", "acceptance", check, ...
        "hasTarget", true, "postSolveCertificationPerformed", true, "runtimeSeconds", toc(timer));
    metadata.runtime = struct("inputPreparationSeconds", preparationSeconds, ...
        "carriedWitnessSeconds", witnessSeconds, ...
        "predictionSeconds", planningTiming.predictionSeconds, ...
        "formulationAndWitnessSeconds", planningTiming.formulationSeconds, ...
        "solveSeconds", planningTiming.solveSeconds, ...
        "acceptanceAndCommitSeconds", planningTiming.verificationSeconds+toc(phase), ...
        "diagnosticsSeconds", 0);
    metadata.planningWindowSteps = cfg.controller.horizonSteps;
    metadata.certificateSearchAttempts = planningTiming.attempts;
    metadata.solverAlgorithm = "Clarabel safety-value LP and CLF SOCP";
    metadata.setMembershipUpdate = ~isempty(candidate);
    metadata.certificateCompatible = ~isempty(candidate);
    metadata.carriedWitnessFeasible = ~isempty(candidate);
    metadata.shiftedSafetyCandidateChecked = ~isempty(candidate);
    metadata.freshSolveFailure = freshFailure;
    metadata.optimizedStages = optimizedStages;
    metadata.terminalActive = terminalActive;
    metadata.pcbfValue = check.value;
    metadata.stageViolation = check.stageViolation;
    metadata.lexicographicTieResidual = tieResidual;
    metadata.candidateVerified = ~isempty(candidate);
    metadata.candidateValue = NaN;
    metadata.candidateSeconds = witnessSeconds;
    metadata.pcbfDescentResidual = NaN;
    metadata.verificationMethod = "freshEnclosureRows";
    if ~isempty(candidate)
        metadata.candidateValue = candidate.check.value;
        % Executed descent: V(k+1) - (V(k) - xi_0(k)) must not be positive.
        metadata.pcbfDescentResidual = check.value-candidate.carriedValue;
        if source == "carriedWitness"
            metadata.verificationMethod = candidate.verificationMethod;
        end
    end
    metadata.initialErrorBound = model.initialFrenetErrorBound;
    metadata.targetErrorBound = encounters.radius;
    metadata.safetyScope = certificate.safetyScope;
    metadata.certifiedDuration = certificate.certifiedDuration;
    metadata.lookaheadDuration = prediction.stageCount*model.sampleTime;
    metadata.inputDelaySeconds = 0;
    metadata.commandActuationTime = command.actuationTime;
    metadata.exactPredictionAssumptionsHold = true;
    metadata.tireForceConstraintScope = "declaredScheduledAffineBicycleStudy";
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
    certificate.metadata = planningProblem.metadata;
    if ~explicitState, previousCertificate = certificate; end
end

function nodes = localExactNodes(model, prediction, inputPlan)
    nodes = zeros(6, prediction.stageCount+1);
    nodes(:, 1) = model.initialEgoState;
    for stage = 1:prediction.stageCount
        nodes(:, stage+1) = prediction.stageMatrixA(:, :, stage)*nodes(:, stage) ...
            +prediction.stageMatrixB(:, :, stage)*inputPlan(:, stage)+prediction.stageAffine(:, stage);
    end
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
    [roadForce, ~, roadComponents] = ltvBicycleModel.roadLoad(state(4), cfg);
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
