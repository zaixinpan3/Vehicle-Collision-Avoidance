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
        if candidate.rebuilt
            candidate = hardEncounterBarrier.verifyCandidate(model,candidate);
        else
            candidate = hardEncounterBarrier.transferCandidate(model,candidate);
        end
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
    planningTiming = struct("predictionSeconds",0,"formulationSeconds",0, ...
        "solveSeconds",0,"verificationSeconds",0,"attempts",0,"deadlineHit",false);
    % The frame deadline applies only when a verified witness can take over.
    if ~isempty(candidate), model.frameTimer = timer; end
    try
        [freshModel,freshPrediction,freshQp,freshResult,freshCheck,planningTiming,freshFailure] = ...
            hardEncounterBarrier.plan(model);
        if strlength(freshFailure)==0
            fresh = struct("model",freshModel,"prediction",freshPrediction,"qp",freshQp, ...
                "result",freshResult,"check",freshCheck);
        elseif isempty(candidate)
            % Without a verified witness a failed search ends control.
            separator = strfind(freshFailure,": ");
            error(extractBefore(freshFailure,separator(1)),"%s",extractAfter(freshFailure,separator(1)+1));
        end
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
    h = model.sampleTime;
    phase = tic;
    if ~isempty(fresh)
        source = "checkedOptimization";
        frame = localFreshFrame(fresh,fresh.model,identity,encounters,originalEncounter);
        solverCalls = fresh.result.solverCalls;
        tieResidual = fresh.result.lexicographicTieResidual;
    else
        source = "carriedWitness";
        solverCalls = 0;
        tieResidual = 0;
        if candidate.rebuilt
            rebuilt = struct("model",candidate.model,"prediction",candidate.prediction,"qp",candidate.qp, ...
                "result",struct("decision",candidate.decision),"check",candidate.check);
            frame = localFreshFrame(rebuilt,candidate.model,identity,encounters,originalEncounter);
            frame.certificate.source = source;
        elseif candidate.optimizedStages>0
            frame = localTransferredFrame(controllerState,candidate,model,encounters);
        else
            frame = localTerminalFrame(controllerState,candidate,model,identity,encounters,originalEncounter);
        end
    end
    certificate = frame.certificate;
    certificate.source = source;
    command = frame.command;
    command.measurementTime = model.stateTime;
    command.actuationTime = model.stateTime;
    command.holdSeconds = h;
    predictedInput = frame.predictedInput;
    check = frame.check;
    optimizedStages = frame.optimizedStages;
    terminalActive = optimizedStages==0;
    metadata = struct("planCertified", check.accepted, "certificateSource", source, ...
        "fallbackUsed", false, "solverCallCount", solverCalls, ...
        "carriedMargin", check.margin, "requiredMargin", model.requiredMargin, ...
        "horizonSteps", optimizedStages, "tailSteps", 0, "deadline", certificate.deadline, ...
        "activeTargetKeys", string({encounters.key}), ...
        "dischargedTargetKeys", strings(1,0), ...
        "clfRelaxation", frame.clfRelaxation, "clfDecayRate", frame.clfDecayRate, ...
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
    metadata.frameDeadlineHit = planningTiming.deadlineHit;
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
    metadata.lookaheadDuration = optimizedStages*h;
    metadata.inputDelaySeconds = 0;
    metadata.commandActuationTime = command.actuationTime;
    metadata.exactPredictionAssumptionsHold = true;
    metadata.tireForceConstraintScope = "declaredScheduledAffineBicycleStudy";
    metadata.executedContinuousGenerator = frame.executedGenerator;
    metadata.executedResidualRateBound = frame.executedResidualRateBound;
    metadata.clfValueProfile = frame.clfValueProfile;
    metadata.clfInitialValue = frame.clfValueProfile(1);
    metadata.clfOperatingCurvature = frame.clfOperatingCurvature;
    metadata.clfOperatingInput = frame.clfOperatingInput;
    metadata.clfMetricChanged = false;
    metadata.clfReferenceSwitchValue = 0;
    metadata.jointObjectiveValue = frame.jointObjectiveValue;
    metadata.clfRelaxationCost = h*cfg.clf.relaxationWeight*sum(metadata.clfRelaxation.^2);
    metadata.hardRowViolation = check.hardRowViolation;
    planningProblem = struct("problemClass", "encounterPredictiveCbfClfSocp", "qp", frame.qp, ...
        "layout", frame.layout, "prediction", frame.prediction, "model", frame.model, ...
        "decision", frame.decision, "plan", predictedInput(:), ...
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
    if exist("collisionAvoidanceControllerConfig", "file") ~= 2
        repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
        configurationRoot = fullfile(repositoryRoot, "config");
        if isfolder(configurationRoot)
            addpath(configurationRoot);
        end
    end
    localAddKernelPath();
end

function localAddKernelPath()
% The generated tube, linearization and row kernels live beside the conic
% solver bridge. Without them every prediction and row falls back to the
% interpreted implementations, which compute the same enclosures slowly.
    if exist("bicycleHeldIntervalKernelMex", "file") == 3 ...
            && exist("avoidanceCellRowsKernelMex", "file") == 3
        return;
    end
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    kernelRoot = fullfile(repositoryRoot, "solver", "bicycle");
    if isfolder(kernelRoot)
        addpath(kernelRoot);
    end
end

function frame = localFreshFrame(fresh,model,identity,encounters,originalEncounter)
% Certificate and outputs of a plan verified in full at this frame (a fresh
% plan, or a rebuilt carried witness): consumedStages is zero.
    prediction = fresh.prediction;
    qp = fresh.qp;
    decision = fresh.result.decision;
    check = fresh.check;
    predictedInput = reshape(decision(qp.layout.planIndex),2,[]);
    optimizedStages = prediction.stageCount;
    state = model.initialEgoState;
    command = localCommand(predictedInput(:,1),state,prediction.scheduleSpeedProfile(1), ...
        prediction.scheduleCurvature(1),prediction.scheduleBrakingRatio(1),model);
    % Published successor nodes follow the exact held-input stage flows, the
    % declared plant, with the interval hull of the initial box as radius.
    % Conditioning against them keeps every later node box inside the
    % accepted plan's node boxes (INFORMATION_STATE_PCBF.md, Lemma 1).
    predictedState = localExactNodes(model,prediction,predictedInput);
    carried = hardEncounterBarrier.carriedData(prediction,qp,optimizedStages);
    certificate = struct("version",20,"identity",identity,"stateTime",model.stateTime, ...
        "deadline",model.stateTime+optimizedStages*model.sampleTime, ...
        "remainingSteps",optimizedStages,"consumedStages",0,"horizonSteps",optimizedStages, ...
        "margin",check.margin, ...
        "value",check.value,"stageViolation",check.stageViolation, ...
        "plan",predictedInput,"decision",decision,"qp",qp,"prediction",prediction, ...
        "predictedState",predictedState,"stateErrorBound",prediction.initialErrorBound, ...
        "appliedInput",predictedInput(:,1),"scheduledInput",command.actuatorInput, ...
        "encounters",encounters,"originalEncounter",originalEncounter,"acceptance",check, ...
        "safetyScope","verifiedPredictionWithInvariantTerminalTail", ...
        "certifiedDuration",optimizedStages*model.sampleTime, ...
        "stages",carried.stages,"cellStage",carried.cellStage,"cellFrames",carried.cellFrames, ...
        "cellNormals",{carried.cellNormals},"terminal",carried.terminal,"source","checkedOptimization", ...
        "tailDecelerating",predictedState(4,end)<model.initialEgoState(4)-0.25 ...
            && model.initialEgoState(4)<model.referenceSpeed);
    frame = localFrameOutputs(certificate,command,predictedInput,check,optimizedStages, ...
        qp,prediction,decision,predictedState,1,model);
end

function frame = localTransferredFrame(stored,candidate,model,encounters)
% Certificate and outputs when the carried tail of the stored plan is the
% command: the stored data are kept, one more stage is consumed.
    stage = candidate.consumed+2;
    plan = stored.plan;
    prediction = stored.prediction;
    qp = stored.qp;
    state = prediction.egoStateMatrix(:,:,stage)*plan(:)+prediction.egoStateOffset(:,stage);
    command = localCommand(plan(:,stage),state,prediction.scheduleSpeedProfile(stage), ...
        prediction.scheduleCurvature(stage),prediction.scheduleBrakingRatio(stage),model);
    predictedInput = plan(:,stage:end);
    certificate = stored;
    certificate.stateTime = model.stateTime;
    certificate.consumedStages = candidate.consumed+1;
    certificate.remainingSteps = size(plan,2)-certificate.consumedStages;
    % Node boxes are published from this frame on: column 2 is the successor.
    certificate.predictedState = stored.predictedState(:,2:end);
    certificate.stateErrorBound = stored.stateErrorBound(:,2:end);
    certificate.appliedInput = plan(:,stage);
    certificate.scheduledInput = command.actuatorInput;
    certificate.encounters = encounters;
    certificate.acceptance = candidate.check;
    certificate.margin = candidate.check.margin;
    frame = localFrameOutputs(certificate,command,predictedInput,candidate.check,candidate.optimizedStages, ...
        qp,prediction,stored.decision,certificate.predictedState,stage,model);
end

function frame = localTerminalFrame(stored,candidate,model,identity,encounters,originalEncounter)
% Certificate and outputs when no optimized stage remains: the sampled
% terminal law from the conditioned box, verified by terminal membership.
    terminal = candidate.terminal;
    center = model.initialEgoState;
    radius = model.initialFrenetErrorBound;
    step = hardEncounterBarrier.terminalStep(terminal,center,radius,model.sampleTime);
    command = localCommand(step.input,center,0,terminal.curvature,step.input(2),model);
    check = candidate.check;
    emptyFrames = stored.cellFrames(zeros(0,1));
    certificate = struct("version",20,"identity",identity,"stateTime",model.stateTime, ...
        "deadline",model.stateTime+model.sampleTime,"remainingSteps",0,"consumedStages",0, ...
        "horizonSteps",0,"margin",check.margin,"value",0,"stageViolation",0, ...
        "plan",step.input,"decision",[step.input;0],"qp",[],"prediction",[], ...
        "predictedState",[center,step.successor],"stateErrorBound",[radius,step.successorRadius], ...
        "appliedInput",step.input,"scheduledInput",command.actuatorInput, ...
        "encounters",encounters,"originalEncounter",originalEncounter,"acceptance",check, ...
        "safetyScope","verifiedPredictionWithInvariantTerminalTail", ...
        "certifiedDuration",model.sampleTime, ...
        "stages",repmat(step.stage,0,1),"cellStage",zeros(0,1),"cellFrames",emptyFrames, ...
        "cellNormals",{cell(0,1)},"terminal",terminal,"source","carriedWitness","tailDecelerating",false);
    frame = localFrameOutputs(certificate,command,step.input,check,0,[],[],[step.input;0], ...
        [center,step.successor],1,model);
    frame.executedGenerator = step.generator;
end

function frame = localFrameOutputs(certificate,command,predictedInput,check,optimizedStages, ...
        qp,prediction,decision,nodes,stage,model)
% Common outputs. nodes holds the plan's node states from the current frame
% on; stage indexes the stored (unshifted) prediction and decision. Metadata
% that describe a solved program are NaN when the frame executed the
% terminal law, which solves none.
    frame = struct("certificate",certificate,"command",command,"predictedInput",predictedInput, ...
        "check",check,"optimizedStages",optimizedStages,"qp",qp,"prediction",prediction,"decision",decision);
    % The reported model is the one the frame's rows were built on. Frames
    % without a solved program report the conditioned model with the exit
    % fields every consumer of a planning model expects.
    frame.model = model;
    if ~isfield(frame.model,"exitSteps"), frame.model.exitSteps = zeros(numel(model.encounters),1); end
    if ~isfield(frame.model,"exitMargin"), frame.model.exitMargin = inf; end
    frame.layout = [];
    frame.clfRelaxation = zeros(0,1);
    frame.clfDecayRate = NaN;
    frame.clfValueProfile = NaN;
    frame.clfOperatingCurvature = NaN;
    frame.clfOperatingInput = NaN(2,1);
    frame.jointObjectiveValue = NaN;
    frame.executedGenerator = zeros(6,9);
    frame.executedResidualRateBound = zeros(6,1);
    if ~isempty(qp)
        frame.layout = qp.layout;
        frame.clfRelaxation = decision(qp.layout.relaxationIndex(stage:end));
        frame.clfDecayRate = qp.clf.decayRate;
        trackingError = nodes(2:6,:)-qp.clf.referenceStart ...
            -qp.clf.referenceRate*((0:size(nodes,2)-1)*model.sampleTime);
        frame.clfValueProfile = sum(trackingError.*(qp.clf.lyapunovMatrix*trackingError),1);
        frame.clfOperatingCurvature = qp.clf.certificate.operatingCurvature;
        frame.clfOperatingInput = qp.clf.certificate.operatingInput;
        frame.jointObjectiveValue = 0.5*decision.'*qp.Hessian*decision+qp.linear.'*decision+qp.constant;
        frame.executedGenerator = [prediction.continuousA(:,:,stage), ...
            prediction.continuousB(:,:,stage),prediction.continuousC(:,stage)];
        frame.executedResidualRateBound = prediction.modelErrorRateBound(:,stage);
    end
end

function command = localCommand(firstInput, state, scheduleSpeed, scheduleCurvature, scheduleBrakingRatio, model)
% Derived quantities of one held input at a nominal state under the stage's
% scheduled tire tangent; the actuator input is the held input itself.
    cfg = model.cfg;
    forceScheduleSpeed = max(scheduleSpeed, cfg.model.scheduleSpeedFloor);
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
        scheduleCurvature, scheduleSpeed, scheduleBrakingRatio, cfg);
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
