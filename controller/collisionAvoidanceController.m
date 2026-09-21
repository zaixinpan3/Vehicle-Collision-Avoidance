function [command, predictedInput, planningProblem, controllerState] = ...
        collisionAvoidanceController(egoState, targetEstimate, laneCenterline, cfg, previousState)
%collisionAvoidanceController Predictive safety continuation with a soft CLF.
% Optimize the complete finite control plan and execute only its first hold.
% Safety rows are certified at the hold nodes of the exact sampled affine
% plant, not between nodes (NODE_SAMPLED_CERTIFICATE.md).
% Store the accepted prediction and terminal witness for next-frame transfer.
% The terminal law is a mathematical continuation, never a runtime fallback.
% Fresh admission searches one signed section, tapered on curved predictions,
% with a coarser certificate dictionary. It may then release
% the complete control sequence in one hard solve. Subsequent frames improve
% the complete verified continuation.
% Inherited joint-support optimization retains verified incumbents when an
% improvement is unavailable. Scalar candidates require independent checking.
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
    model = localFiniteModel(ego,lane,road,cfg,previousState);
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
    if laneGeometry.isVaryingReference(lane) && isempty(model.cruiseCertificate)
        model.cruiseCertificate=ltvBicycleModel.sampledCruise(model);
    end
    if laneGeometry.isVaryingReference(lane) ...
            && abs(model.initialEgoState(1)-model.cruiseCertificate.state(1))+model.initialFrenetErrorBound(1) ...
            >cfg.encounter.referencePhaseRadius
        error('collisionAvoidanceController:referencePhaseOutsideDomain', ...
            'The measured station is outside the declared scheduled-model phase domain.');
    end
    preparationSeconds = toc(timer);
    solverCfg=cfg;solverCfg.solver.workTimer=timer;
    solverCfg.solver.workTimeLimit=min(cfg.solver.frameDeadlineSeconds,cfg.solver.certificateSearchTimeLimit);
    [program,prediction,clf,result,search]=localSolve(model,solverCfg);
    formulationSeconds=search.formulationSeconds;solveSeconds=search.solveSeconds;
    conicCalls=search.nativeSolves;
    if ~result.feasible
        error("collisionAvoidanceController:optimizationFailed", ...
            "The hard-safety, soft-CLF search failed at t=%.9g s (%s). No command was issued.", ...
            model.stateTime,result.message);
    end
    program=solveHardCbfClf.certify(program,result.decision);
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
    nextReference=cruise.state;nextMatrix=cruise.matrix;
    if isfield(cruise,'nextState'),nextReference=cruise.nextState;nextMatrix=cruise.nextMatrix;end
    nextError = cruise.transition(2:6,:)*[model.initialEgoState;firstInput;1]-nextReference(2:6);
    nextValue = nextError.'*nextMatrix*nextError;
    metadata = struct('solverCallCount',conicCalls,'solverExitFlag',result.exitFlag, ...
        'conicSolverCallCount',conicCalls, ...
        'trajectorySolverCallCount',search.hardSolves, ...
        'restorationSolverCallCount',search.restorationSolves,'admissionSearch',search, ...
        'horizonAttemptCount',search.horizonAttempts, ...
        'approximateSolveCertified',result.exitFlag==2, ...
        'solverMessage',result.message,'solverAlgorithm',"predictive hard-safety soft-CLF SOCP", ...
        'hasTarget',~isempty(model.encounters),'obstacleCbfRowCount',program.obstacleCbfRowCount, ...
        'postSolveCertificationPerformed',true,'planCertified',true, ...
        'safetyScope',"finiteEncounterThenInvariantRoadContinuation", ...
        'certifiedDuration',model.sampleTime*prediction.stageCount, ...
        'recursiveFeasibilityGuaranteed',isempty(model.encounters),'certificateSource',"constrainedOptimization", ...
        'recursiveFeasibilityScope',"targetFreeContinuationUnderDeclaredContracts", ...
        'stateAndSlipBoundsEnforced',false,'wholeHoldCertificate',false, ...
        'certificateSampling',"holdNodes", ...
        'supportGeometry',program.supportGeometry, ...
        'convexificationPolicy',search.policy, ...
        'nominalSource',model.nominalSource, ...
        'shiftedWitnessContained',program.supportGeometry.witnessPreserved, ...
        'predictionContinuationRetained',true,'inheritedFeasibleFamily',program.inheritedFeasibleFamily, ...
        'terminalInvariantOptimization',program.terminalOptimization, ...
        'freshProblemContainsWitness',program.replacementContainsWitness, ...
        'measurementRadiusLimit',program.terminal.measurementRadiusLimit, ...
        'carriedWitnessAvailable',~isempty(carry), ...
        'measurementContractChanged',model.measurementContractChanged, ...
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
    if isfield(program,'jointCertificate')
        metadata.recursiveFeasibilityGuaranteed=true;
        metadata.recursiveFeasibilityScope="shiftedCompleteCertificateUnderUnchangedContracts";
        metadata.jointCertificateResidual=avoidanceSafetyGeometry.jointResidual( ...
            program,result.decision,program.jointCertificate.angles);
        metadata.certifiedIncumbentUsed=search.usedCertifiedIncumbent;
        if search.usedCertifiedIncumbent,metadata.certificateSource="retainedCertifiedIncumbent";end
    end
    metadata.runtime = struct('inputPreparationSeconds',preparationSeconds, ...
        'formulationSeconds',formulationSeconds,'solveSeconds',solveSeconds);
    metadata.clfNextMatrix=nextMatrix;metadata.clfNextReferenceState=nextReference;
    if isfield(cruise,'scheduled') && cruise.scheduled
        metadata.referencePhaseIndex=cruise.index;
        metadata.referencePhaseError=model.initialEgoState(1)-cruise.state(1);
        metadata.referencePhaseRadius=cfg.encounter.referencePhaseRadius;
        metadata.spatialCurvature=laneGeometry.curvature(model.initialEgoState(1),lane);
        if ~isfield(program,'jointCertificate')
            metadata.recursiveFeasibilityScope="targetFreeBoundedPhaseScheduledAffinePlant";
        end
    end
    data = hardEncounterBarrier.carriedData(prediction,program,prediction.stageCount);
    controllerState = struct('version',40, ...
        'appliedInput',firstInput,'stateTime',model.stateTime, ...
        'identity',identity,'plan',predictedInput,'decision',result.decision, ...
        'predictedState',states,'stateErrorBound',prediction.initialErrorBound, ...
        'prediction',prediction,'stages',data.stages,'cellFrames',data.cellFrames, ...
        'cellNormals',{data.cellNormals},'terminal',program.terminal, ...
        'completion',program.completion,'confirmation',model.confirmation, ...
        'encounters',model.encounters,'program',program);
    metadata.runtimeSeconds = toc(timer);
    if metadata.runtimeSeconds>cfg.solver.frameDeadlineSeconds
        error('collisionAvoidanceController:optimizationFailed', ...
            'The complete frame exceeded its execution deadline. No command was issued.');
    end
    planningProblem = struct('problemClass',"encounterPredictiveCbfClfSocp",'program',program, ...
        'prediction',prediction,'predictedState',states,'model',model,'decision',result.decision, ...
        'inputPlan',predictedInput,'carriedWitness',carry,'metadata',metadata);
    if ~explicitState,lastState=controllerState;end
end

function model = localFiniteModel(ego, lane, road, cfg, previousState)
    stationHint=[];
    if laneGeometry.isVaryingReference(lane)
        stationHint=cfg.referenceSpeed*(ego.stateTime-cfg.clf.referenceEpoch);
        if isstruct(previousState) && isfield(previousState,'predictedState') ...
                && isfield(previousState,'identity') && isequaln(previousState.identity.lane,lane)
            stationHint=previousState.predictedState(1,min(2,size(previousState.predictedState,2)));
        end
    end
    projection = laneGeometry.project(ego.position, lane,stationHint);
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
    if laneGeometry.isVaryingReference(lane)
        % One schedule lookup per frame; every phase query below reuses it.
        model.referenceBank = ltvBicycleModel.referenceSchedule(model);
    end
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

function [program,prediction,clf,result,search]=localSolve(model,cfg)
% Active encounters add variable-angle certificates to the convex base.
% Without encounters, only the common dynamics, CLF and terminal solve remains.
    phase=tic;
    [program,prediction,clf]=formulateAvoidanceProblem(model);
    formulationSeconds=toc(phase);
    if isfield(program,'jointCertificate')
        [program,result,search]=solveHardCbfClf.joint(program,model,cfg);
        prediction=program.prediction;
        search.formulationSeconds=search.formulationSeconds+formulationSeconds;
        return;
    end
    phase=tic;
    result=solveHardCbfClf.constrained(program,cfg);
    search=struct('hardSolves',1,'restorationSolves',0,'nativeSolves',1, ...
        'familyAttempts',0,'horizonAttempts',1, ...
        'formulationSeconds',formulationSeconds,'solveSeconds',toc(phase), ...
        'initialOverlappingNodes',program.supportGeometry.overlappingNodes, ...
        'policy',"jointSupport");
end
