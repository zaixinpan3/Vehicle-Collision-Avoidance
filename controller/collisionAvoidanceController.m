function [command, predictedInput, planningProblem, certificate] = ...
        collisionAvoidanceController(egoState, targetEstimate, laneCenterline, cfg, controllerState)
%collisionAvoidanceController Predictive CBF safety with a carried witness.
% A finite frame budget selects two-variable hard-constrained backup control
% with hard sampled cruise dissipation. executionPolicy="predictive" retains
% the full-horizon SOCP. Supply the fourth output at the next sample.
% Current measurements condition the stored reachable boxes. Feasibility
% transfers to the carried suffix without a separate runtime plan checker.
% A strict successful hard-constrained solve can replace it without delaying
% active encounter completion. Declared affine plant and sensor contracts apply.
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
    model.targetSetChanged = false;
    model.motionBoundsIncreased = false;
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
        model.encounters = targetPrediction.admitOnline(observations, model.stateTime, lane, cfg);
        originalEncounter = model.encounters;
        if ~isempty(model.encounters)
            model.confirmation = hardEncounterBarrier.admitConfirmation(ego,model);
        end
    end
    encounters = model.encounters;
    preparationSeconds = toc(timer);
    witnessSeconds = 0;
    if ~isempty(candidate)
        candidate = hardEncounterBarrier.transferCandidate(model,candidate);
        witnessSeconds = candidate.seconds;
    end
    policy = string(cfg.controller.executionPolicy);
    fixedStored=~isempty(controllerState) && any(controllerState.version==[23,24]);
    constantReference=cfg.referenceSpeed>0 && ~any(cfg.clf.referenceOffset) && ~any(cfg.clf.referenceRate);
    if policy=="backup" || (policy=="auto" && constantReference ...
            && (isfinite(cfg.solver.frameDeadlineSeconds) || fixedStored))
        [command,predictedInput,planningProblem,certificate] = localBackupControl( ...
            model,candidate,controllerState,identity,originalEncounter,timer,preparationSeconds,witnessSeconds);
        if ~explicitState,previousCertificate=certificate;end
        return;
    end
    fresh = [];
    planningTiming = struct("predictionSeconds",0,"formulationSeconds",0, ...
        "solveSeconds",0,"verificationSeconds",0,"attempts",0,"deadlineHit",false, ...
        "maximumAttemptSeconds",0,"lastAttemptStages",0);
    % The frame deadline applies only when a verified witness can take over.
    if ~isempty(candidate)
        model.frameTimer = timer;
        model.previousAttemptSeconds = controllerState.lastAttemptSeconds;
        model.previousAttemptStages = controllerState.lastAttemptStages;
    end
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
        % A fresh feasible plan must preserve the active exit deadline.
        if ~isempty(encounters) && fresh.qp.completion.deadline>candidate.completion.deadline ...
                +128*eps(max(1,abs(candidate.completion.deadline)))
            freshFailure = "A replacement witness cannot postpone the certified encounter exit.";
            fresh = [];
        end
    end
    h = model.sampleTime;
    phase = tic;
    if ~isempty(fresh)
        source = "constrainedOptimization";
        frame = localFreshFrame(fresh,fresh.model,identity,encounters,originalEncounter);
        solverCalls = fresh.result.solverCalls;
        tieResidual = fresh.result.lexicographicTieResidual;
    else
        source = "carriedWitness";
        solverCalls = 0;
        tieResidual = 0;
        if candidate.optimizedStages==0
            frame = localTerminalFrame(controllerState,candidate,model,identity,encounters,originalEncounter);
        else
            frame = localTransferredFrame(controllerState,candidate,model,encounters);
        end
    end
    certificate = frame.certificate;
    certificate.lastAttemptSeconds = planningTiming.maximumAttemptSeconds;
    certificate.lastAttemptStages = planningTiming.lastAttemptStages;
    if certificate.lastAttemptSeconds==0 && ~isempty(controllerState)
        certificate.lastAttemptSeconds = controllerState.lastAttemptSeconds;
        certificate.lastAttemptStages = controllerState.lastAttemptStages;
    end
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
        "hasTarget", ~isempty(encounters), "postSolveCertificationPerformed", false, "runtimeSeconds", toc(timer));
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
    metadata.verificationMethod = "hardConstraintsAndSolverStatus";
    if ~isempty(candidate)
        metadata.candidateValue = candidate.check.value;
        % Executed descent: V(k+1) - (V(k) - xi_0(k)) must not be positive.
        metadata.pcbfDescentResidual = check.value-candidate.carriedValue;
        if source == "carriedWitness"
            metadata.verificationMethod = candidate.verificationMethod;
        end
    end
    metadata.initialErrorBound = model.initialFrenetErrorBound;
    metadata.targetErrorBound = zeros(8,0);
    if ~isempty(encounters), metadata.targetErrorBound = encounters.radius; end
    metadata.targetSetChanged = model.targetSetChanged;
    metadata.motionBoundsIncreased = model.motionBoundsIncreased;
    metadata.newlyAdmittedTargetKeys = strings(1,0);
    if isempty(controllerState) || model.targetSetChanged
        metadata.newlyAdmittedTargetKeys = metadata.activeTargetKeys;
    end
    metadata.jointAdmissionPerformed = ~isempty(metadata.newlyAdmittedTargetKeys) || model.motionBoundsIncreased;
    if model.targetSetChanged && isempty(encounters)
        metadata.dischargedTargetKeys = string({controllerState.encounters.key});
    end
    metadata.safetyScope = certificate.safetyScope;
    metadata.certifiedDuration = certificate.certifiedDuration;
    metadata.lookaheadDuration = optimizedStages*h;
    metadata.inputDelaySeconds = 0;
    metadata.commandActuationTime = command.actuationTime;
    metadata.exactPredictionAssumptionsHold = all(arrayfun(@(target) ...
        ~any(target.contract.jerkBound) && target.contract.yawAccelerationBound==0,encounters));
    metadata.targetMotionBounds = {encounters.contract};
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
        "requiredMargin", 0,"confirmation",[]);
end

function [command,inputs,problem,certificate] = localBackupControl( ...
        model,candidate,stored,identity,original,timer,preparationSeconds,witnessSeconds)
% Bounded rollout proposals and a two-variable solve containing all hard rows.
    cfg=model.cfg; h=model.sampleTime;
    model.declaredLinearizationPolicy="cruiseTrim";
    if cfg.referenceSpeed<=0 || any(cfg.clf.referenceOffset) || any(cfg.clf.referenceRate)
        error('collisionAvoidanceController:unsupportedCruiseReference', ...
            'The backup policy requires the constant-speed path trim; use predictive for a moving offset reference.');
    end
    cruise=ltvBicycleModel.sampledCruise(model);
    model.cfg.solver.workTimer=timer;
    model.cfg.solver.workTimeLimit=cfg.solver.certificateSearchTimeLimit;
    if ~isempty(candidate)
        model.cfg.solver.workTimeLimit=max(0,cfg.solver.frameDeadlineSeconds-.002);
    end
    fresh=[]; attempts=0; formulationSeconds=0; solveSeconds=0; solverCalls=0; failure="";
    clf=struct('certified',false,'initialValue',NaN,'nextValue',NaN, ...
        'upperResidual',NaN,'disturbanceBound',NaN,'decayPerHold',cruise.decayPerHold);
    active=~isempty(model.encounters);
    deadlineHit=~isempty(candidate) && toc(timer)+.002>=cfg.solver.frameDeadlineSeconds;
    if deadlineHit,failure="collisionAvoidanceController:frameDeadline: The feasible witness supplies this hold.";end
    if ~active && ~deadlineHit
        input=localFeedbackProposal(model,cruise,0,cfg.referenceSpeed,1);
        input=localBrakeTransition(model,cruise,input);
        attempts=attempts+1;
        cruiseModel=model;
        if ~isempty(candidate) && isfinite(cfg.solver.frameDeadlineSeconds)
            elapsed=toc(timer);
            remaining=max(0,model.cfg.solver.workTimeLimit-elapsed);
            cruiseModel.cfg.solver.workTimeLimit=elapsed+min(.015,.2*remaining);
        end
        [check,prediction,terminal,completion,input,clf,program]=localConstrainedAttempt(cruiseModel,input,cruise,true);
        formulationSeconds=formulationSeconds+check.formulationSeconds;
        solveSeconds=solveSeconds+check.solveSeconds;solverCalls=solverCalls+check.solverCalls;
        if check.accepted
            fresh=struct('input',input,'prediction',prediction,'terminal',terminal, ...
                'completion',completion,'check',check,'program',program);
        end
    end
    if isempty(fresh) && ~deadlineHit && (isempty(candidate) || (~active && candidate.optimizedStages<=1))
        if active
            offset=0;
            if model.confirmation.passingRequired
                [targetPosition,~]=laneGeometry.fromFrenet(model.initialEgoState,model.lane);
                frame=laneGeometry.frameBounds(model.lane,model.initialEgoState(1), ...
                    cfg.controller.stationTrustRadius,cfg.model.lateralDomainRadius);
                side=-sign(frame.lateral.'*(model.encounters.center(1:2)-targetPosition));
                if side==0,side=1;end
                offset=side*min(3.2,.8*cfg.model.lateralDomainRadius);
            end
            speeds=cfg.referenceSpeed;
            if any(model.encounters.radius) || any(model.encounters.contract.jerkBound)
                speeds=[min(cfg.model.speedMaximum-0.5,1.75*cfg.referenceSpeed),cfg.referenceSpeed];
            end
            offsets=[offset,-offset];
        else
            speeds=cfg.referenceSpeed;offsets=0;
        end
        proposals=max(numel(offsets),numel(speeds));
        if ~active
            proposals=2;
            if ~isempty(candidate),proposals=1;end
        end
        for index=1:proposals
            if ~isempty(candidate) && toc(timer)+.002>=cfg.solver.frameDeadlineSeconds
                deadlineHit=true;break;
            end
            speed=speeds(min(index,numel(speeds)));
            offset=offsets(min(index,numel(offsets)));
            count=min(64,max(cfg.controller.horizonSteps,ceil(4/h)));
            if active
                count=min(64,max(cfg.controller.horizonSteps,model.confirmation.searchHorizonSteps));
                if isfield(model,'exitDeadline')
                    count=min(count,round((model.exitDeadline-model.stateTime)/h));
                end
            end
            input=localFeedbackProposal(model,cruise,offset,speed,count);
            input=localBrakeTransition(model,cruise,input);
            attempts=attempts+1;
            [check,prediction,terminal,completion,input,clf,program]=localConstrainedAttempt( ...
                model,input,cruise,~active && isempty(candidate) && index==1);
            formulationSeconds=formulationSeconds+check.formulationSeconds;
            solveSeconds=solveSeconds+check.solveSeconds;solverCalls=solverCalls+check.solverCalls;
            if check.accepted
                fresh=struct('input',input,'prediction',prediction,'terminal',terminal, ...
                    'completion',completion,'check',check,'program',program);
                break;
            end
            failure=check.message;
        end
    end
    if ~isempty(fresh)
        inputs=fresh.input;
        prediction=fresh.prediction;
        count=size(inputs,2);
        check=fresh.check;
        source="constrainedBackup";
        if ~active && clf.certified,source="sampledClfCruise";end
        certificate=struct('version',24,'kind',"constrainedBackup",'identity',identity,'stateTime',model.stateTime, ...
            'deadline',model.stateTime+count*h,'remainingSteps',count,'consumedStages',0,'horizonSteps',count, ...
            'margin',check.margin,'value',0,'stageViolation',zeros(count,1),'plan',inputs,'decision',inputs(:), ...
            'qp',[],'prediction',[],'predictedState',prediction.fixedStates, ...
            'stateErrorBound',prediction.initialErrorBound,'appliedInput',inputs(:,1),'scheduledInput',inputs(:,1), ...
            'encounters',model.encounters,'originalEncounter',original,'acceptance',check, ...
            'safetyScope',"finiteConfirmedEncounterThenInvariantRoadTail",'certifiedDuration',count*h, ...
            'stages',repmat(cruise.stage,count,1),'fixedStage',cruise.stage, ...
            'cellStage',zeros(0,1),'cellFrames',struct([]),'cellNormals',{cell(0,1)}, ...
            'terminal',fresh.terminal,'source',source,'completion',fresh.completion, ...
            'confirmation',model.confirmation,'tailDecelerating',false,'witnessModel',model);
        stage=cruise.stage;
    elseif ~isempty(candidate)
        source="carriedWitness";
        check=candidate.check;
        certificate=stored;
        certificate.stateTime=model.stateTime;
        certificate.encounters=model.encounters;
        certificate.completion=candidate.completion;
        certificate.confirmation=model.confirmation;
        if candidate.optimizedStages>0
            certificate.consumedStages=stored.consumedStages+1;
            certificate.remainingSteps=stored.remainingSteps-1;
            inputs=stored.plan(:,certificate.consumedStages+1:end);
            certificate.predictedState=stored.predictedState(:,2:end);
            certificate.stateErrorBound=stored.stateErrorBound(:,2:end);
            stage=stored.stages(certificate.consumedStages+1);
        else
            step=hardEncounterBarrier.terminalStep(candidate.terminal,candidate.terminalCenter, ...
                candidate.terminalRadius,h);
            inputs=step.input;stage=step.stage;
            certificate.plan=inputs;certificate.decision=inputs(:);
            certificate.consumedStages=0;certificate.remainingSteps=0;certificate.horizonSteps=0;
            certificate.deadline=model.stateTime+h;
            certificate.predictedState=[candidate.terminalCenter,step.successor];
            certificate.stateErrorBound=[candidate.terminalRadius,step.successorRadius];
            certificate.stages=repmat(stage,0,1);certificate.fixedStage=stage;
        end
        certificate.appliedInput=inputs(:,1);certificate.scheduledInput=inputs(:,1);
        certificate.acceptance=check;certificate.source=source;
        clf.certified=false;
        clf.nextValue=NaN;clf.upperResidual=NaN;clf.disturbanceBound=NaN;
        prediction=[];
    else
        error('collisionAvoidanceController:noCertifiedContinuation', ...
            'No feasible hard-constrained continuation was obtained in %d bounded attempts. %s',attempts,failure);
    end
    command=localCommand(inputs(:,1),model.initialEgoState,stage.speed,stage.curvature,stage.brakingRatio,model);
    command.actuationTime=model.stateTime;
    command.inputDelaySeconds=0;
    command.measurementTime=model.stateTime;
    command.holdSeconds=h;
    terminalActive=certificate.remainingSteps==0;
    released=~isempty(stored) && ~isempty(stored.encounters) && ~active;
    targetUntil=NaN;
    if active,targetUntil=certificate.completion.deadline;end
    timing=struct('inputPreparationSeconds',preparationSeconds,'carriedWitnessSeconds',witnessSeconds, ...
        'predictionSeconds',0,'formulationAndWitnessSeconds',formulationSeconds,'solveSeconds',solveSeconds, ...
        'acceptanceAndCommitSeconds',0,'diagnosticsSeconds',0);
    metadata=struct('planCertified',check.accepted,'certificateSource',source,'fallbackUsed',false, ...
        'safetyCertified',check.safetyCertified,'candidateAccepted',check.candidateAccepted, ...
        'solverCallCount',solverCalls,'acceptance',check,'pcbfValue',0,'stageViolation',zeros(size(inputs,2),1), ...
        'pcbfDescentResidual',NaN,'candidateVerified',~isempty(candidate),'candidateSeconds',witnessSeconds, ...
        'terminalActive',terminalActive,'hasTarget',active,'confirmedRelease',released, ...
        'jointAdmissionPerformed',active && (isempty(stored) || model.targetSetChanged || model.motionBoundsIncreased), ...
        'initialErrorBound',model.initialFrenetErrorBound,'targetErrorBound',zeros(8,0), ...
        'targetCertifiedUntil',targetUntil,'roadTailCertified',true,'targetSetChanged',model.targetSetChanged, ...
        'motionBoundsIncreased',model.motionBoundsIncreased,'activeTargetKeys',string({model.encounters.key}), ...
        'newlyAdmittedTargetKeys',strings(1,0),'dischargedTargetKeys',strings(1,0), ...
        'runtime',timing,'runtimeSeconds',toc(timer),'planningAttempts',attempts, ...
        'certificateSearchAttempts',attempts,'frameDeadlineHit',deadlineHit, ...
        'freshSolveFailure',failure,'inputDelaySeconds',0,'commandActuationTime',command.actuationTime, ...
        'executedContinuousGenerator',[stage.continuousA,stage.continuousB,stage.continuousC], ...
        'executedResidualRateBound',zeros(6,1),'physicalVehicleGuaranteeEstablished',false, ...
        'declaredLinearizationPolicy',"cruiseTrim", ...
        'safetyScope',certificate.safetyScope,'collisionDiscretization',"sweptBernsteinCells", ...
        'horizonSteps',certificate.remainingSteps,'deadline',certificate.deadline, ...
        'clfDissipationCertified',clf.certified,'clfDissipation',clf, ...
        'clfInitialValue',clf.initialValue,'clfValueProfile',[clf.initialValue,clf.nextValue], ...
        'clfRelaxation',[],'clfDecayRate',clf.decayPerHold/h,'clfOperatingCurvature',cruise.stage.curvature, ...
        'clfOperatingInput',cruise.input,'clfMatrix',cruise.matrix,'clfReferenceState',cruise.state(2:6), ...
        'clfMetricChanged',false,'executionPolicy',"backup",'verificationMethod',"hardConstraintsAndSolverStatus", ...
        'postSolveCertificationPerformed',false,'recursivelyFeasible',true, ...
        'recursiveFeasibilityClaimed',true,'indefiniteRecursiveFeasibilityClaimed',~active, ...
        'terminalContinuationCertified',true,'encounterComplete',~active,'carriedMargin',check.margin, ...
        'requiredMargin',0,'hardRowViolation',check.hardRowViolation,'certifiedDuration',certificate.certifiedDuration, ...
        'commandCertifiedDuration',h,'lookaheadDuration',certificate.remainingSteps*h);
    if ~isempty(candidate),metadata.pcbfDescentResidual=0;end
    if active,metadata.targetErrorBound=model.encounters.radius;end
    if released,metadata.dischargedTargetKeys=string({stored.encounters.key});end
    if metadata.jointAdmissionPerformed,metadata.newlyAdmittedTargetKeys=metadata.activeTargetKeys;end
    if source=="carriedWitness",metadata.verificationMethod=candidate.verificationMethod;end
    if terminalActive
        metadata.declaredLinearizationPolicy="terminalRest";
    elseif source=="carriedWitness" && stored.version==22
        metadata.declaredLinearizationPolicy=string(stored.witnessModel.cfg.model.linearizationPolicy);
    end
    model.declaredLinearizationPolicy=metadata.declaredLinearizationPolicy;
    if ~isempty(stored)
        metadata.clfMetricChanged=stored.metadata.clfOperatingCurvature~=cruise.stage.curvature;
    end
    metadata.deadlineMet=metadata.runtimeSeconds<=min(h,cfg.solver.frameDeadlineSeconds);
    certificate.lastAttemptSeconds=0;certificate.lastAttemptStages=max(1,size(inputs,2));
    certificate.metadata=metadata;
    if isempty(fresh),program=[];else,program=fresh.program;end
    problem=struct('model',model,'prediction',prediction,'qp',program,'layout',[], ...
        'decision',inputs(:),'inputPlan',inputs,'metadata',metadata);
end

function [check,prediction,terminal,completion,inputs,clf,program] = localConstrainedAttempt(model,inputs,cruise,required)
    attemptTimer=tic;
    try
        [check,prediction,terminal,completion,inputs,clf,program] = ...
            hardEncounterBarrier.constrainedBackup(model,inputs,cruise,required);
    catch exception
        if ~startsWith(string(exception.identifier),'collisionAvoidanceController:')
            rethrow(exception);
        end
        check=struct('accepted',false,'value',NaN,'hardRowViolation',NaN, ...
            'failedConditions',string(exception.identifier),'message',string(exception.message),'solverCalls',0, ...
            'formulationSeconds',toc(attemptTimer),'solveSeconds',0);
        prediction=[];terminal=[];completion=[];program=[];
        clf=struct('certified',false,'initialValue',NaN,'nextValue',NaN, ...
            'upperResidual',NaN,'disturbanceBound',NaN,'decayPerHold',cruise.decayPerHold);
    end
end

function inputs = localBrakeTransition(model,cruise,inputs)
% Ramp from the proposed final input to the stopping law when slew requires
% it. Every appended hold belongs to the same hard-constrained swept formulation.
    terminal=hardEncounterBarrier.roadTerminalDynamics(model,cruise.stage.curvature);
    state=model.initialEgoState;radius=model.initialFrenetErrorBound;
    transition=cruise.transition(1:6,1:6);
    for index=1:size(inputs,2)
        state=cruise.transition(1:6,:)*[state;inputs(:,index);1];
        radius=abs(transition)*radius;
    end
    cfg=model.cfg;
    rate=.99*model.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    lower=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    upper=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    count=size(inputs,2);
    maximum=min(16,64-count);
    plan=[inputs,zeros(2,maximum)];
    for index=1:maximum
        desired=terminal.input+terminal.feedback*state+terminal.radiusFeedback*radius;
        if all(abs(desired-plan(:,count))<=rate) && state(4)+radius(4)<terminal.velocityLimit(1)
            break;
        end
        input=min(max(desired,max(lower,plan(:,count)-rate)),min(upper,plan(:,count)+rate));
        count=count+1;plan(:,count)=input;
        state=cruise.transition(1:6,:)*[state;input;1];
        radius=abs(transition)*radius;
    end
    inputs=plan(:,1:count);
end

function inputs = localFeedbackProposal(model,cruise,offset,speed,count)
    cfg=model.cfg;
    state=model.initialEgoState;
    desired=cruise.state(2:6);desired(1)=offset;desired(3)=speed;
    lower=[-min(.12,cfg.model.frontWheelSteeringAngleMaximum);max(-.8,cfg.actuation.brakingRatioMinimum)];
    upper=[min(.12,cfg.model.frontWheelSteeringAngleMaximum);min(.8,cfg.actuation.brakingRatioMaximum)];
    change=.99*model.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    previous=model.previousInput;
    inputs=zeros(2,count);
    for index=1:count
        input=cruise.input-cruise.gain*(state(2:6)-desired);
        input=min(max(input,max(lower,previous-change)),min(upper,previous+change));
        inputs(:,index)=input;previous=input;
        state=cruise.transition(1:6,:)*[state;input;1];
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

function frame = localFreshFrame(fresh,model,identity,encounters,originalEncounter)
% Certificate and outputs of a hard-constrained plan solved at this frame (a fresh
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
    certificate = struct("version",22,"identity",identity,"stateTime",model.stateTime, ...
        "deadline",model.stateTime+optimizedStages*model.sampleTime, ...
        "remainingSteps",optimizedStages,"consumedStages",0,"horizonSteps",optimizedStages, ...
        "margin",check.margin, ...
        "value",check.value,"stageViolation",check.stageViolation, ...
        "plan",predictedInput,"decision",decision,"qp",qp,"prediction",prediction, ...
        "predictedState",predictedState,"stateErrorBound",prediction.initialErrorBound, ...
        "appliedInput",predictedInput(:,1),"scheduledInput",command.actuatorInput, ...
        "encounters",encounters,"originalEncounter",originalEncounter,"acceptance",check, ...
        "safetyScope","finiteConfirmedEncounterThenInvariantRoadTail", ...
        "certifiedDuration",optimizedStages*model.sampleTime, ...
        "stages",carried.stages,"cellStage",carried.cellStage,"cellFrames",carried.cellFrames, ...
        "cellNormals",{carried.cellNormals},"terminal",carried.terminal,"source","constrainedOptimization", ...
        "completion",qp.completion,"confirmation",model.confirmation, ...
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
    certificate.completion = candidate.completion;
    certificate.confirmation = model.confirmation;
    certificate.acceptance = candidate.check;
    certificate.margin = candidate.check.margin;
    frame = localFrameOutputs(certificate,command,predictedInput,candidate.check,candidate.optimizedStages, ...
        qp,prediction,stored.decision,certificate.predictedState,stage,model);
end

function frame = localTerminalFrame(stored,candidate,model,identity,encounters,originalEncounter)
% Certificate and outputs when no optimized stage remains: the sampled
% terminal law from the carried nominal box, feasible by terminal invariance.
    terminal = candidate.terminal;
    center = candidate.terminalCenter;
    radius = candidate.terminalRadius;
    step = hardEncounterBarrier.terminalStep(terminal,center,radius,model.sampleTime);
    command = localCommand(step.input,center,0,terminal.curvature,step.input(2),model);
    check = candidate.check;
    emptyFrames = stored.cellFrames(zeros(0,1));
    certificate = struct("version",22,"identity",identity,"stateTime",model.stateTime, ...
        "deadline",model.stateTime+model.sampleTime,"remainingSteps",0,"consumedStages",0, ...
        "horizonSteps",0,"margin",check.margin,"value",0,"stageViolation",0, ...
        "plan",step.input,"decision",[step.input;0],"qp",[],"prediction",[], ...
        "predictedState",[center,step.successor],"stateErrorBound",[radius,step.successorRadius], ...
        "appliedInput",step.input,"scheduledInput",command.actuatorInput, ...
        "encounters",encounters,"originalEncounter",originalEncounter,"acceptance",check, ...
        "safetyScope","finiteConfirmedEncounterThenInvariantRoadTail", ...
        "certifiedDuration",model.sampleTime, ...
        "stages",repmat(step.stage,0,1),"cellStage",zeros(0,1),"cellFrames",emptyFrames, ...
        "cellNormals",{cell(0,1)},"terminal",terminal,"source","carriedWitness","tailDecelerating",false, ...
        "completion",candidate.completion,"confirmation",model.confirmation);
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
