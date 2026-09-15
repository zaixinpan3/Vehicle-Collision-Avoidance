function [command, predictedInput, planningProblem, controllerState] = ...
        collisionAvoidanceController(egoState, targetEstimate, laneCenterline, cfg, previousState)
%collisionAvoidanceController Predictive safety continuation with a soft CLF.
% Optimize the complete finite control plan and execute only its first hold.
% Store the accepted prediction and terminal witness for next-frame transfer.
% The terminal law is a mathematical continuation, never a runtime fallback.
% Any failed solve raises an error before a command is returned.
    persistent lastState
    if nargin == 1 && (ischar(egoState) || isstring(egoState))
        if ~isscalar(string(egoState)) || string(egoState) ~= "resetNominalTrajectory"
            error("collisionAvoidanceController:invalidAction", "Use resetNominalTrajectory.");
        end
        lastState = [];
        command = []; predictedInput = []; planningProblem = []; controllerState = [];
        return;
    end
    explicitState = nargin >= 5;
    if ~explicitState, previousState = lastState; end
    if nargin < 4, cfg = []; end
    if nargin < 3, laneCenterline = []; end
    if nargin < 2, targetEstimate = []; end
    timer = tic;
    cfg = localControllerConfiguration(cfg);
    [ego,lane,road,observations] = readPlanningInputs(egoState,targetEstimate,laneCenterline,cfg);
    model = localFiniteModel(ego,lane,road,cfg);
    if cfg.referenceSpeed<=0 || any(cfg.clf.referenceOffset) || any(cfg.clf.referenceRate)
        error("collisionAvoidanceController:unsupportedCruiseReference", ...
            "The cruise CLF requires positive constant speed and zero path-error reference.");
    end
    if any(cfg.model.ltvModelErrorRateBound) || any(cfg.model.plantModelResidualRateBound)
        error("collisionAvoidanceController:nonexactStudyInput", ...
            "This controller requires the declared zero-residual held affine plant.");
    end
    identity = struct('configuration',rmfield(cfg,'solver'),'lane',lane,'road',road, ...
        'accelerationBias',ego.longitudinalAccelerationBias);
    [model,carry] = hardEncounterBarrier.prepare(model,ego,observations,previousState,identity);
    preparationSeconds = toc(timer);
    formulationTimer = tic;
    [program,prediction,clf] = formulateAvoidanceProblem(model);
    formulationSeconds = toc(formulationTimer);
    solveTimer = tic;
    solverCfg = cfg;
    solverCfg.solver.workTimer = solveTimer;
    solverCfg.solver.workTimeLimit = cfg.solver.frameDeadlineSeconds;
    result = solveHardCbfClf.constrained(program,solverCfg);
    solveSeconds = toc(solveTimer);
    if ~result.feasible
        error("collisionAvoidanceController:optimizationFailed", ...
            "The single hard-safety, soft-CLF solve failed at t=%.9g s (%s). No command was issued.", ...
            model.stateTime,result.message);
    end
    predictedInput = reshape(result.decision(program.layout.planIndex),2,[]);
    firstInput = predictedInput(:,1);
    clfSlack = result.decision(program.layout.relaxationIndex);
    % Reporting an outward slack allowance is not a command acceptance test.
    slackAllowance = max(clfSlack,0);
    slackBound = clf.youngFactor*slackAllowance*(2*clf.normDisturbance+slackAllowance);
    cruise = clf.cruise;
    states = zeros(6,prediction.stageCount+1);
    states(:,1) = model.initialEgoState;
    if isfield(prediction,'nominalInitialState'),states(:,1)=prediction.nominalInitialState;end
    for stage = 1:prediction.stageCount
        states(:,stage+1) = prediction.stageMatrixA(:,:,stage)*states(:,stage) ...
            +prediction.stageMatrixB(:,:,stage)*predictedInput(:,stage)+prediction.stageAffine(:,stage);
    end
    command = localCommand(firstInput,model.initialEgoState,cruise.stage.speed, ...
        cruise.stage.curvature,cruise.stage.brakingRatio,model);
    command.measurementTime = model.stateTime;
    command.actuationTime = model.stateTime;
    command.holdSeconds = model.sampleTime;
    nextError = cruise.transition(2:6,:)*[model.initialEgoState;firstInput;1]-cruise.state(2:6);
    nextValue = nextError.'*cruise.matrix*nextError;
    metadata = struct('solverCallCount',1,'solverExitFlag',result.exitFlag, ...
        'solverMessage',result.message,'solverAlgorithm',"predictive hard-safety soft-CLF SOCP", ...
        'hasTarget',~isempty(model.encounters),'obstacleCbfRowCount',program.obstacleCbfRowCount, ...
        'postSolveCertificationPerformed',false,'planCertified',true, ...
        'safetyScope',"finiteEncounterThenInvariantRoadContinuation", ...
        'certifiedDuration',model.sampleTime*prediction.stageCount, ...
        'recursiveFeasibilityGuaranteed',false,'certificateSource',"constrainedOptimization", ...
        'predictionContinuationRetained',true,'inheritedFeasibleFamily',program.inheritedFeasibleFamily, ...
        'carriedWitnessAvailable',~isempty(carry), ...
        'terminalContinuationCertified',true,'terminalPolicyRole',"predictionCertificateOnly", ...
        'terminalActive',false,'fallbackUsed',false,'horizonSteps',prediction.stageCount, ...
        'confirmedRelease',~isempty(model.dischargedTargetKeys), ...
        'dischargedTargetKeys',model.dischargedTargetKeys, ...
        'targetCertifiedUntil',program.completion.deadline,'roadTailCertified',true, ...
        'clfDissipationCertified',true,'clfInitialValue',clf.initialValue,'clfNextValue',nextValue, ...
        'clfDisturbanceBound',clf.disturbanceBound,'clfDecayPerHold',clf.decayPerHold, ...
        'clfSlack',clfSlack,'clfSlackPenalty',cfg.clf.relaxationWeight*clfSlack^2, ...
        'clfSlackDissipationBound',slackBound,'clfDissipationScope',"slackDependentSampledBound", ...
        'clfMatrix',cruise.matrix,'clfOperatingCurvature',cruise.stage.curvature, ...
        'clfOperatingInput',cruise.input,'clfReferenceState',cruise.state, ...
        'initialErrorBound',model.initialFrenetErrorBound,'targetErrorBound',zeros(8,0), ...
        'executedContinuousGenerator',[cruise.stage.continuousA,cruise.stage.continuousB,cruise.stage.continuousC], ...
        'executedResidualRateBound',zeros(6,1),'runtimeSeconds',toc(timer));
    if ~isempty(model.encounters)
        metadata.targetErrorBound=[model.encounters.radius];
    else
        metadata.targetCertifiedUntil=NaN;
    end
    metadata.runtime = struct('inputPreparationSeconds',preparationSeconds, ...
        'formulationSeconds',formulationSeconds,'solveSeconds',solveSeconds);
    data = hardEncounterBarrier.carriedData(prediction,program,prediction.stageCount);
    controllerState = struct('version',26,'appliedInput',firstInput,'stateTime',model.stateTime, ...
        'identity',identity,'plan',predictedInput,'decision',result.decision, ...
        'predictedState',states,'stateErrorBound',prediction.initialErrorBound, ...
        'prediction',prediction,'stages',data.stages,'cellFrames',data.cellFrames, ...
        'cellNormals',{data.cellNormals},'terminal',program.terminal, ...
        'completion',program.completion,'confirmation',model.confirmation, ...
        'encounters',model.encounters,'program',program);
    metadata.runtimeSeconds = toc(timer);
    planningProblem = struct('problemClass',"encounterPredictiveCbfClfSocp",'program',program, ...
        'prediction',prediction,'predictedState',states,'model',model,'decision',result.decision, ...
        'inputPlan',predictedInput,'carriedWitness',carry,'metadata',metadata);
    if ~explicitState,lastState=controllerState;end
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
        "requiredMargin", 0,"confirmation",[]);
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
