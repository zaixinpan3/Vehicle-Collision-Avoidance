function report = runNonlinearPredictiveSafetyValidation(options)
%runNonlinearPredictiveSafetyValidation Deterministic nonlinear closed-loop audit.
% A tight ode45 replay checks each issued hold independently of the affine prediction.
% Its endpoint uses the same discrete rest transition as the prediction model.
% The audit is offline evidence; it is not an execution admission layer.
% Terminal membership certifies a possible safe continuation of the prediction;
% it does not end the driving task or activate the terminal policy.
% By default continue until failure or sustained nominal recovery. Frames=Inf
% imposes no simulation-duration limit; finite Frames enables a bounded diagnostic.
% A finite observation cap does not by itself establish task recovery.
% Frames is a cumulative observation cap, including when resuming a saved
% continuation. A completed duration is distinct from confirmed recovery.
% TargetEstimateFunction(targetTruth,egoTruth) is an optional synthetic
% observation hook. It never changes the fixed physical target trajectory.
% EstimatorConfiguration enables the actual noisy-sensor NRMM adapter. The
% plant always starts from truth, independently of the published estimate.
% FailureFile saves experiment inputs for offline reproduction only.
% Theoretical certificate availability is not an experiment failure criterion.
% LateralDeviationLimitMeters is a truth-replay stopping threshold only; it
% never enters the planner, controller constraints or terminal certificate.
    arguments
        options.Frames (1,1) double {mustBePositive} = Inf
        options.Scenarios (1,:) string = ["recovery","oncoming","circular","turningTarget"]
        options.OutputFile (1,1) string = ""
        options.ControllerConfiguration (1,1) struct = struct('referenceSpeed',8, ...
            'controller',struct('horizonSteps',8))
        options.StateTransition (1,1) string {mustBeMember(options.StateTransition,["ode45","nominalRk4"])} = "ode45"
        options.LateralDeviationLimitMeters (1,1) double {mustBePositive} = 10
        options.RequireCollisionThreat (1,1) logical = false
        options.RequireBrakingInsufficient (1,1) logical = false
        options.RecoveryDwellSeconds (1,1) double {mustBeNonnegative,mustBeFinite} = 1
        options.RecoveryMinimumSeconds (1,1) double {mustBeNonnegative,mustBeFinite} = 0
        options.RecoveryTolerances (5,1) double {mustBePositive,mustBeFinite} = [.1;pi/180;.1;.05;.01]
        options.ContinuationFile (1,1) string = ""
        options.ResumeFrom (1,1) string = ""
        options.TargetEstimateFunction = []
        options.EstimatorConfiguration (1,1) struct = struct()
        options.FailureFile (1,1) string = ""
    end
    assert(isreal(options.Frames) && (options.Frames==Inf || ...
        (isfinite(options.Frames) && options.Frames==fix(options.Frames))), ...
        'runNonlinearPredictiveSafetyValidation:invalidFrameLimit', ...
        'Frames must be a positive integer or Inf.');
    assert(isempty(options.TargetEstimateFunction) || isa(options.TargetEstimateFunction,'function_handle'), ...
        'TargetEstimateFunction must be empty or a function handle.');
    observationModel="exactTargetState";
    if ~isempty(options.TargetEstimateFunction),observationModel=string(func2str(options.TargetEstimateFunction));end
    useEstimator=~isempty(fieldnames(options.EstimatorConfiguration));
    assert(~useEstimator || isempty(options.TargetEstimateFunction), ...
        'The actual estimator and a synthetic estimate hook cannot be combined.');
    if useEstimator,observationModel="nrmmSyntheticSensors";end
    assert((strlength(options.ContinuationFile)==0 && strlength(options.ResumeFrom)==0) ...
        || isscalar(options.Scenarios),'Continuation files require exactly one scenario.');
    assert(strlength(options.FailureFile)==0 || isscalar(options.Scenarios), ...
        'A failure snapshot requires exactly one scenario.');
    root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'controller'),fullfile(root,'config'));
    results=cell(1,numel(options.Scenarios));
    for index=1:numel(options.Scenarios)
        name=options.Scenarios(index);
        [ego,target,road,cfg,initialTarget]=localFixture(name,options.ControllerConfiguration);prior=[];
        maximumBraking=struct([]);
        if options.RequireBrakingInsufficient
            state=[ego.position;ego.yaw;ego.speed;ego.lateralVelocity;ego.yawRate];
            maximumBraking=maximumBrakingCollisionBaseline(state,initialTarget,cfg);
            if maximumBraking.initialClearanceMeters<=0 || ~maximumBraking.collisionDetected
                error('runNonlinearPredictiveSafetyValidation:brakingSuffices', ...
                    'Scenario %s does not qualify: immediate maximum straight braking must collide from a separated start.',name);
            end
        end
        estimatorContext=[];estimatorConfiguration=options.EstimatorConfiguration;estimatorInitializationSeconds=0;
        if useEstimator
            assert(~isempty(initialTarget),'The NRMM adapter requires a target truth trajectory.');
            estimatorConfiguration.sensor.radar.rangeMaximum=cfg.collision.encounterRangeMeters;
            % The estimator shares the controller's vehicle, limits and
            % sideslip cone; the target premises are the scenario contract.
            % Include this encounter in the declared scenario envelope. In
            % particular a stationary or stopping lead must not inherit the
            % strictly moving target domain of the original seven fixtures.
            contract=collisionThreatContract(WindowSeconds=8);
            contract=localIncludeTargetContract(contract,initialTarget);
            estimatorConfiguration=estimatorConfigurationFromController(estimatorConfiguration,cfg, ...
                contract);
            if strlength(options.ResumeFrom)==0
                initializationTimer=tic;
                estimatorContext=nrmmEstimatorControllerAdapter("initialize",estimatorConfiguration,ego, ...
                    @(time,unused)localTarget(predictiveSafetyGeometry.predictTarget(initialTarget,time)));
                estimatorInitializationSeconds=toc(initializationTimer);
            end
        end
        % Warm after estimator initialization/path setup. A bounded-input
        % probe uses the initial publication without advancing the observer
        % or RNG (initialFramePending is true); its command/state are discarded.
        warmUpTimer=tic;collisionAvoidanceController("warmUp",cfg,road);
        if useEstimator && strlength(options.ResumeFrom)==0
            [~,probeEgo,probeTarget]=nrmmEstimatorControllerAdapter( ...
                "sample",estimatorContext,ego.stateTime,ego,target,[]);
            probeEgo.heldActuatorInput=ego.heldActuatorInput;
            probeCfg=cfg;probeCfg.solver.timeLimitSeconds=max(1,cfg.solver.timeLimitSeconds);
            try
                collisionAvoidanceController(probeEgo,probeTarget,road,probeCfg,[]);
            catch exception
                if ~strcmp(exception.identifier,'collisionAvoidanceController:noOptimizationSolution'),rethrow(exception);end
            end
        end
        controllerWarmUpSeconds=toc(warmUpTimer);
        if strlength(options.ResumeFrom)>0
            saved=load(options.ResumeFrom,'continuation');saved=saved.continuation;
            baseline=saved.result.baselineCruise;
        else
            baselineDuration=options.Frames*cfg.controller.sampleTime;
            if isinf(baselineDuration)
                % Finite qualification of the initial encounter, not a cap
                % on closed-loop execution or future observer/controller updates.
                baselineDuration=max(cfg.controller.horizonSteps*cfg.controller.sampleTime, ...
                    2*cfg.collision.encounterRangeMeters/cfg.referenceSpeed);
            end
            baseline=givenPathCollisionBaseline(road,initialTarget,cfg,baselineDuration);
        end
        if options.RequireCollisionThreat && (~baseline.collisionDetected || baseline.initialClearanceMeters<=0)
            error('runNonlinearPredictiveSafetyValidation:notCollisionThreat', ...
                'Scenario %s must start separated and collide under given-path constant-speed cruise.',name);
        end
        frames=0;minimumClearance=Inf;minimumRoadMargin=Inf;maximumSeconds=0;failure="";
        maximumDeviation=0;
        firstClf=NaN;lastClf=NaN;maximumSlack=0;maxHorizon=0;warmStartedFrames=0;finalError=[];
        trace=struct([]);failedFrameSeconds=NaN;failedPipelineSeconds=NaN;failureTime=NaN;passedTarget=false;
        collisionDetected=false;collisionTime=NaN;controlUnavailable=false;executionError=false;
        lateralLimitExceeded=false;lateralLimitTime=NaN;
        recovery=struct('enabled',options.RecoveryDwellSeconds>0,'recovered',false, ...
            'tolerances',options.RecoveryTolerances,'errorOrder', ...
            ["lateralMeters","headingRadians","speedMetersPerSecond","lateralVelocityMetersPerSecond","yawRateRadiansPerSecond"], ...
            'dwellSeconds',options.RecoveryDwellSeconds,'minimumTimeSeconds',options.RecoveryMinimumSeconds, ...
            'entryTimeSeconds',NaN,'confirmationTimeSeconds',NaN);
        % A recovery dwell must follow the prescribed encounter, rather than
        % an arbitrary elapsed duration or the initially nominal state.
        if baseline.collisionDetected
            recovery.minimumTimeSeconds=max(recovery.minimumTimeSeconds,baseline.lastCollisionSeconds);
        end
        recoveryStart=NaN;
        if strlength(options.ResumeFrom)>0
            r=saved.result;
            assert(~isfield(r,'terminalHandoff'), ...
                'runNonlinearPredictiveSafetyValidation:incompatibleTaskSemantics', ...
                'The saved run uses the retired terminal-entry experiment schema.');
            assert(isfield(saved,'sampleEndReset') ...
                && saved.sampleEndReset=="brakedSpeedBelowOneFifthHoldSpeed-v1", ...
                'runNonlinearPredictiveSafetyValidation:incompatiblePlantTransition', ...
                'Continuation must use the shared sample-end rest transition.');
            assert(isequaln(r.configuration,cfg) && r.scenario==name ...
                && saved.stateTransition==options.StateTransition ...
                && isfield(saved,'observationModel') && saved.observationModel==observationModel ...
                && saved.prior.version==92, ...
                'Continuation scenario, configuration and transition must match.');
            assert(isequaln(saved.recoveryOptions,recovery),'Recovery settings must match.');
            assert(isempty(r.trace) || (isfield(r.trace,'terminalDistanceMeters') ...
                && isfield(r.trace,'postSolveFeasibilityCheck')), ...
                'runNonlinearPredictiveSafetyValidation:incompatibleTraceSchema', ...
                'Continuation trace must use the current RTI metadata schema.');
            assert(strlength(r.failure)==0 && ~r.recovery.recovered && r.executedFrames<options.Frames, ...
                'Only an unfinished, successful duration can be extended.');
            ego=saved.ego;target=saved.target;prior=saved.prior;frames=r.executedFrames;
            if useEstimator
                assert(isequaln(saved.estimatorConfiguration,estimatorConfiguration),'Estimator settings must match.');
                estimatorContext=saved.estimatorContext;
                estimatorInitializationSeconds=r.estimatorInitializationSeconds;
            end
            assert(r.lateralDeviationLimitMeters==options.LateralDeviationLimitMeters, ...
                'Continuation must retain its experiment lateral stopping threshold.');
            minimumClearance=r.minimumReplayClearanceMeters;minimumRoadMargin=r.minimumReplayRoadMarginMeters;
            maximumSeconds=r.maximumFrameSeconds;maximumDeviation=r.maximumReplayLateralDeviationMeters;firstClf=r.initialClfValue;lastClf=r.finalClfValue;
            maximumSlack=r.maximumClfSlack;maxHorizon=r.maximumHorizonSteps;finalError=r.finalTransverseError;
            warmStartedFrames=r.warmStartedFrames;trace=r.trace;passedTarget=r.passedTarget;
            recoveryStart=saved.recoveryStart;
        end
        recoveryOptions=recovery;truthPath=[];
        frame=frames;
        while frame<options.Frames
            frame=frame+1;
            frameTimer=tic;
            phase="observer";controllerTimer=[];
            egoTruth=ego;targetTruth=target;priorAtFrame=prior;
            egoInput=ego;estimate=[];sensorFrame=[];observerSeconds=0;
            try
                observerTimer=tic;
                if useEstimator
                    [estimatorContext,egoInput,estimate,sensorFrame]=nrmmEstimatorControllerAdapter( ...
                        "sample",estimatorContext,ego.stateTime,ego,target,truthPath);
                    egoInput.heldActuatorInput=ego.heldActuatorInput;
                    observerSeconds=toc(observerTimer);
                else
                    egoInput.perception=struct('time',ego.stateTime, ...
                        'range',cfg.collision.encounterRangeMeters,'completeWithinRange',true);
                    if ~isempty(target) && norm(target.targetPositionInertial-ego.position)<=cfg.collision.encounterRangeMeters
                        estimate=target;
                        if ~isempty(options.TargetEstimateFunction)
                            estimate=options.TargetEstimateFunction(target,ego);
                        end
                    end
                end
                phase="controller";controllerTimer=tic;
                [command,~,problem,prior]=collisionAvoidanceController(egoInput,estimate,road,cfg,prior);
                assert(problem.metadata.optimizationReturned && numel(command.actuatorInput)==2 ...
                    && all(isfinite(command.actuatorInput),'all'), ...
                    'runNonlinearPredictiveSafetyValidation:noControlOutput','No finite solved control was returned.');
                frameSeconds=toc(controllerTimer);pipelineSeconds=toc(frameTimer);
                phase="replay";
                maximumSeconds=max(maximumSeconds,frameSeconds);maxHorizon=max(maxHorizon,problem.metadata.horizonSteps);
                x=[ego.position;ego.yaw;ego.speed;ego.lateralVelocity;ego.yawRate];input=command.actuatorInput;
                h=cfg.controller.sampleTime;
                solution=ode45(@(~,state)nonlinearBicycleModel.derivative(state,input,cfg), ...
                    [0,h],x,odeset('RelTol',1e-11,'AbsTol',1e-12));
                times=linspace(0,h,31).';states=deval(solution,times).';
                states(end,:)=nonlinearBicycleModel.sampleEndState(states(end,:).',input,cfg).';
                if useEstimator
                    % The estimator's sensors sample the continuous trajectory
                    % and the same post-reset state at the control boundary:
                    % states and instantaneous body accelerations R(psi)'*pddot
                    % at its sample times, with the input held over the hold.
                    sensorPeriod=estimatorConfiguration.observer.runtime.samplePeriod;
                    pathTimes=(1:round(h/sensorPeriod))*sensorPeriod;
                    pathStates=deval(solution,pathTimes);
                    pathStates(:,end)=nonlinearBicycleModel.sampleEndState(pathStates(:,end),input,cfg);
                    pathAccelerations=zeros(2,numel(pathTimes));
                    for j=1:numel(pathTimes)
                        d=nonlinearBicycleModel.derivative(pathStates(:,j),input,cfg);
                        pathAccelerations(:,j)=[d(4)-pathStates(6,j)*pathStates(5,j);d(5)+pathStates(6,j)*pathStates(4,j)];
                    end
                    truthPath=struct('times',ego.stateTime+pathTimes,'states',pathStates, ...
                        'accelerations',pathAccelerations,'input',input);
                end
                shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
                for j=1:numel(times)
                    q=predictiveSafetyGeometry.predictTarget(initialTarget,ego.stateTime+times(j));
                    if ~isempty(q)
                        distance=predictiveSafetyGeometry.rectangle(states(j,1:3).',shape,q(1:3),q(8:11));
                        minimumClearance=min(minimumClearance,distance);
                        if distance<=0 && ~collisionDetected
                            collisionDetected=true;collisionTime=ego.stateTime+times(j);
                        end
                    end
                    if cfg.controller.enforceRoadBoundaries && ~isempty(road.lateralClearance)
                        rotation=[cos(states(j,3)),-sin(states(j,3));sin(states(j,3)),cos(states(j,3))];
                        vertices=states(j,1:2).'+rotation*(shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1]);
                        for vertex=1:4
                            projection=laneGeometry.project(vertices(:,vertex),problem.model.lane);
                            minimumRoadMargin=min([minimumRoadMargin;projection.lateralPosition+road.lateralClearance(1);road.lateralClearance(2)-projection.lateralPosition]);
                        end
                    end
                end
                centerProjection=laneGeometry.project(states(:,1:2).',problem.model.lane);
                maximumDeviation=max(maximumDeviation,max(abs(centerProjection.lateralPosition)));
                outside=find(abs(centerProjection.lateralPosition)>options.LateralDeviationLimitMeters+1e-6,1);
                if ~isempty(outside),lateralLimitExceeded=true;lateralLimitTime=ego.stateTime+times(outside);end
                if frame==1,firstClf=problem.metadata.clfInitialValue;end
                lastClf=problem.metadata.clfNextValue;maximumSlack=max(maximumSlack,problem.metadata.clfSlack);
                next=states(end,:).';
                if options.StateTransition=="nominalRk4",next=nonlinearBicycleModel.sample(x,input,cfg);end
                ego=localEgo(next,ego.stateTime+h,input);
                finalError=nonlinearBicycleModel.error(next,problem.model.lane,problem.model.nominalReference);
                warmStartedFrames=warmStartedFrames+(problem.metadata.search.initialization=="shiftedInputRollout");
                search=problem.metadata.search;
                entry=struct('time',ego.stateTime-h,'controllerSeconds',frameSeconds, ...
                    'observerSeconds',observerSeconds,'pipelineSeconds',pipelineSeconds, ...
                    'observedTarget',~isempty(estimate),'targetSet',localTargetSetSummary(estimate), ...
                    'estimatedState',problem.model.initialState, ...
                    'estimatedTarget',problem.model.target, ...
                    'horizonSteps',problem.metadata.horizonSteps,'source',search.source, ...
                    'solverCalls',search.solverCalls,'initialization',search.initialization, ...
                    'potentialFieldRestarted',search.potentialFieldRestarted,'initializationFailure',search.initializationFailure, ...
                    'linearizationCount',search.linearizationCount,'attempts',search.attempts, ...
                    'slackCap',search.slackCap,'primaryOptimum',search.primaryOptimum, ...
                    'stageSlacks',problem.solution.stageSlacks, ...
                    'primaryLowerBound',search.primaryLowerBound,'selectedAttempt',search.selectedAttempt,'inputTrustScale',search.inputTrustScale, ...
                    'trust',search.trust, ...
                    'absoluteSampleIndex',problem.model.sampleIndex,'endpointIndex',problem.metadata.endpointIndex, ...
                    'terminalDistanceMeters',problem.metadata.terminalDistanceMeters, ...
                    'terminalRoadMarginMeters',problem.metadata.terminalRoadMarginMeters, ...
                    'terminalValue',problem.metadata.terminalValue,'terminalMarginMeters',problem.metadata.terminalMarginMeters, ...
                    'terminalStopSeconds',problem.metadata.terminalStopSeconds, ...
                    'terminalExitSeconds',problem.metadata.terminalExitSeconds, ...
                    'terminalEnd',problem.metadata.terminalEnd, ...
                    'postSolveInputModification',problem.metadata.postSolveInputModification,'postSolveFeasibilityCheck',problem.metadata.postSolveFeasibilityCheck, ...
                    'anchorCertified',problem.metadata.anchorCertified, ...
                    'rolloutAvailable',search.rollout.available,'rolloutReason',search.rollout.reason, ...
                    'appendedTerminalSteps',problem.metadata.appendedTerminalSteps, ...
                    'appendedBrakingSteps',problem.metadata.appendedBrakingSteps, ...
                    'appendedGuidedSteps',problem.metadata.appendedGuidedSteps, ...
                    'extensionStopped',problem.metadata.extensionStopped, ...
                    'linearizationGapMeters',problem.metadata.linearizationGapMeters, ...
                    'completionSteps',problem.metadata.completionSteps, ...
                    'optimizationReturned',search.returned,'optimizationConverged',search.converged,'terminationReason',search.terminationReason, ...
                    'solverStages',search.stages,'affineFeasibilityTolerance',cfg.solver.feasibilityTolerance, ...
                    'affineValidationPerformed',problem.metadata.affineValidationPerformed, ...
                    'nonlinearValidationPerformed',problem.metadata.nonlinearValidationPerformed, ...
                    'rtiIterations',problem.metadata.rtiIterations, ...
                    'nominalRestarted',search.nominalRestarted, ...
                    'rtiStopReason',search.stopReason, ...
                    'predictiveBarrierValue',problem.metadata.predictiveBarrierValue, ...
                    'hardResidual',problem.solution.hard,'clfSlack',problem.metadata.clfSlack, ...
                    'clfInitialValue',problem.metadata.clfInitialValue,'clfNextValue',problem.metadata.clfNextValue, ...
                    'clfFunction',problem.metadata.clfFunction,'clfRequiredDecrease',problem.metadata.clfRequiredDecrease, ...
                    'clfTieBound',problem.metadata.clfTieBound,'clfLocalModel',problem.metadata.clfLocalModel, ...
                    'clfStageAttempted',search.clfStageAttempted,'clfStageCompleted',search.clfStageCompleted, ...
                    'clfLowerBound',search.clfLowerBound,'clfInitialSlack',search.clfInitialSlack, ...
                    'state',x,'nextState',next,'input',input,'transverseError',finalError, ...
                    'auditTimes',times,'auditStates',states, ...
                    'predictionCollisionMargin',problem.metadata.minimumCollisionMargin);
                if isempty(trace),trace=entry;else,trace(end+1)=entry;end %#ok<AGROW>
                if ~isempty(target)
                    q=predictiveSafetyGeometry.predictTarget(initialTarget,ego.stateTime);
                    target=localTarget(q);
                    passedTarget=passedTarget || next(1)-q(1)>cfg.vehicle.length/2+q(8);
                end
                frames=frames+1;
                if collisionDetected
                    failure="runNonlinearPredictiveSafetyValidation:collision: Ground-truth rectangles touched or overlapped.";
                    failureTime=collisionTime;
                    localSaveFailure(options.FailureFile,name,frame,egoTruth,targetTruth, ...
                        egoInput,estimate,sensorFrame,priorAtFrame,road,cfg,estimatorConfiguration, ...
                        failure,struct('command',command,'problem',problem,'times',times,'states',states));
                    break;
                end
                if lateralLimitExceeded
                    failure="runNonlinearPredictiveSafetyValidation:lateralLimit: Ground-truth reference-point deviation exceeded the configured limit.";
                    failureTime=lateralLimitTime;
                    localSaveFailure(options.FailureFile,name,frame,egoTruth,targetTruth, ...
                        egoInput,estimate,sensorFrame,priorAtFrame,road,cfg,estimatorConfiguration, ...
                        failure,struct('command',command,'problem',problem,'times',times,'states',states));
                    break;
                end
                if recovery.enabled
                    inside=all(abs(finalError)<=options.RecoveryTolerances) ...
                        && ego.stateTime>=recovery.minimumTimeSeconds;
                    if ~inside,recoveryStart=NaN;
                    elseif isnan(recoveryStart),recoveryStart=ego.stateTime;
                    end
                    if inside && ego.stateTime-recoveryStart>=options.RecoveryDwellSeconds-1e-10
                        recovery.recovered=true;recovery.entryTimeSeconds=recoveryStart;
                        recovery.confirmationTimeSeconds=ego.stateTime;break;
                    end
                end
                if mod(frame,4)==0,fprintf('%s: %d/%d holds, last %.3f s, source %s\n', ...
                    name,frame,options.Frames,frameSeconds,search.source);end
                if recovery.enabled && mod(frame,100)==0
                    fprintf('Recovery at %.2f s: e = [%s], W = %.6g\n', ...
                        ego.stateTime,num2str(finalError.',' %.6g'),lastClf);
                end
            catch exception
                failedFrameSeconds=NaN;
                if phase=="controller" && ~isempty(controllerTimer),failedFrameSeconds=toc(controllerTimer);end
                failedPipelineSeconds=toc(frameTimer);
                failureTime=(frame-1)*cfg.controller.sampleTime;
                controlUnavailable=phase=="controller";executionError=~controlUnavailable;
                failure=string(exception.identifier)+": "+string(exception.message);
                localSaveFailure(options.FailureFile,name,frame,egoTruth,targetTruth, ...
                    egoInput,estimate,sensorFrame,priorAtFrame,road,cfg,estimatorConfiguration,failure,[]);
                break;
            end
        end
        seconds=[];pipelineSecondsHistory=[];solverCalls=0;
        if ~isempty(trace),seconds=[trace.controllerSeconds];pipelineSecondsHistory=[trace.pipelineSeconds];solverCalls=sum([trace.solverCalls]);end
        later=seconds(2:end);
        firstSeconds=NaN;if ~isempty(seconds),firstSeconds=seconds(1);end
        medianLater=NaN;p95Later=NaN;
        if ~isempty(later),medianLater=median(later);p95Later=prctile(later,95);end
        outcome="observationLimit";
        if recovery.recovered,outcome="recovered";end
        if executionError,outcome="executionError";end
        if controlUnavailable,outcome="controlUnavailable";end
        if lateralLimitExceeded,outcome="lateralLimitExceeded";end
        if collisionDetected,outcome="collision";end
        results{index}=struct('scenario',name,'requestedFrames',options.Frames,'executedFrames',frames, ...
            'completed',strlength(failure)==0 && (frames==options.Frames || recovery.recovered), ...
            'recovery',recovery, ...
            'outcome',outcome,'experimentFailed',collisionDetected || controlUnavailable || lateralLimitExceeded, ...
            'collisionDetected',collisionDetected,'collisionTimeSeconds',collisionTime, ...
            'controlUnavailable',controlUnavailable,'executionError',executionError, ...
            'failure',failure,'minimumReplayClearanceMeters',minimumClearance, ...
            'minimumReplayRoadMarginMeters',minimumRoadMargin,'maximumFrameSeconds',maximumSeconds, ...
            'maximumHorizonSteps',maxHorizon,'initialClfValue',firstClf,'finalClfValue',lastClf, ...
            'maximumClfSlack',maximumSlack,'sampleTimeSeconds',cfg.controller.sampleTime,'randomSeed',[], ...
            'finalTransverseError',finalError,'warmStartedFrames',warmStartedFrames, ...
            'requiredClearanceMeters',0, ...
            'roadConstraintsEnforced',cfg.controller.enforceRoadBoundaries && ~isempty(road.lateralClearance), ...
            'maximumReplayLateralDeviationMeters',maximumDeviation, ...
            'lateralDeviationLimitMeters',options.LateralDeviationLimitMeters, ...
            'lateralLimitExceeded',lateralLimitExceeded,'lateralLimitTimeSeconds',lateralLimitTime, ...
            'roadDepartureDetected',minimumRoadMargin<0, ...
            'firstFrameSeconds',firstSeconds,'subsequentMedianSeconds',medianLater, ...
            'subsequentP95Seconds',p95Later,'deadlineMisses',nnz(seconds>cfg.controller.sampleTime) ...
                +double(isfinite(failedFrameSeconds) && failedFrameSeconds>cfg.controller.sampleTime), ...
            'successfulDeadlineMisses',nnz(seconds>cfg.controller.sampleTime), ...
            'failedDeadlineMiss',isfinite(failedFrameSeconds) && failedFrameSeconds>cfg.controller.sampleTime, ...
            'controllerBudgetSeconds',cfg.solver.timeLimitSeconds, ...
            'controllerBudgetExceededCalls',nnz(seconds>cfg.solver.timeLimitSeconds) ...
                +double(isfinite(failedFrameSeconds) && failedFrameSeconds>cfg.solver.timeLimitSeconds), ...
            'pipelineDeadlineMisses',nnz(pipelineSecondsHistory>cfg.controller.sampleTime) ...
                +double(isfinite(failedPipelineSeconds) && failedPipelineSeconds>cfg.controller.sampleTime), ...
            'failedPipelineSeconds',failedPipelineSeconds, ...
            'failedFrameSeconds',failedFrameSeconds,'failureTime',failureTime,'passedTarget',passedTarget, ...
            'totalSolverCalls',solverCalls,'targetInitialState',initialTarget,'configuration',cfg, ...
            'observationModel',observationModel,'perceptionRadiusMeters',cfg.collision.encounterRangeMeters, ...
            'estimatorConfiguration',estimatorConfiguration,'estimatorInitializationSeconds',estimatorInitializationSeconds, ...
            'controllerWarmUpSeconds',controllerWarmUpSeconds, ...
            'baselineMaximumBraking',maximumBraking,'baselineCruise',baseline,'trace',trace);
        if useEstimator,results{index}.randomSeed=estimatorConfiguration.randomSeed;end
        if strlength(options.ContinuationFile)>0
            continuation=struct('ego',ego,'target',target,'prior',prior,'result',results{index}, ...
                'stateTransition',options.StateTransition, ...
                'sampleEndReset',"brakedSpeedBelowOneFifthHoldSpeed-v1", ...
                'observationModel',observationModel, ...
                'estimatorConfiguration',estimatorConfiguration,'estimatorContext',estimatorContext, ...
                'recoveryOptions',recoveryOptions,'recoveryStart',recoveryStart);
            save(options.ContinuationFile,'continuation','-v7.3');
        end
        fprintf('%s: %d/%d frames, max %.3f s, failure %s\n',name,frames,options.Frames,maximumSeconds,failure);
    end
    report=struct('model',"nonlinear combined-slip Fiala; one constant-acceleration/constant-sideslip target", ...
        'targetStateOrder',["X","Y","psi","V","A","beta","lr","halfLength","halfWidth","offsetX","offsetY"], ...
        'scope',"RTI PCBF/CLF with independent offline replay", ...
        'observationLimitEnabled',isfinite(options.Frames), ...
        'stateObservation',options.StateTransition+" ego successor with sample-end rest reset; target observation: "+observationModel, ...
        'targetTruth',"fixed initial constant-A/constant-beta trajectory, independent of estimated forecasts", ...
        'failureCriteria',["groundTruthCollision","noSolvedControlOutput"], ...
        'replay',struct('integrator',"ode45",'relativeTolerance',1e-11, ...
            'absoluteTolerance',1e-12,'samplesPerHold',31, ...
            'sampleEndReset',"brakedSpeedBelowOneFifthHoldSpeed-v1"),'results',[results{:}]);
    if strlength(options.OutputFile)>0
        file=fopen(options.OutputFile,'w');assert(file>=0);cleanup=onCleanup(@()fclose(file));
        fprintf(file,'%s\n',jsonencode(report));
    end
end

function contract=localIncludeTargetContract(contract,initialTarget)
    % Use the actual fixture's declared motion, including non-threat audits.
    % This avoids constructing unrelated curved fixtures at an unsupported
    % ego trim speed just to include a stationary or stopping target.
    target=predictiveSafetyGeometry.predictTarget(initialTarget,0:.05:8);
    rate=target(4,:).*sin(target(6,:))./target(7,:);widen=1.1;
    contract.speedMinimum=min(contract.speedMinimum,min(target(4,:))/widen);
    contract.speedMaximum=max(contract.speedMaximum,max(target(4,:))*widen);
    contract.scalarAccelerationMaximum=max(contract.scalarAccelerationMaximum,max(abs(target(5,:)))*widen);
    contract.sideslipMaximum=max(contract.sideslipMaximum,max(abs(target(6,:)))*widen);
    contract.yawRateMaximum=max(contract.yawRateMaximum,max(abs(rate))*widen);
    contract.accelerationNormBound=max(contract.accelerationNormBound, ...
        max(hypot(target(5,:),target(4,:).*rate))*widen);
end

function [ego,target,road,cfg,q]=localFixture(name,controllerConfiguration)
    cfg=collisionAvoidanceControllerConfig(controllerConfiguration);
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[8.5344;12.192]);
    x=[0;0;0;cfg.referenceSpeed;0;0];target=[];q=[];lr=cfg.target.rearAxleDistance;
    switch name
        case "recovery",x(2)=.01;
        case "oncoming"
            q=[24;0;pi;8;0;0;lr;2.4;.95;0;0];
        case "circular"
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',.005,'length',200), ...
                'lateralClearance',[8.5344;12.192]);
            trim=nonlinearBicycleModel.cruise(cfg,.005);x=trim.state;x(1:2)=0;
        case "turningTarget"
            q=[24;0;pi;8;0;atan(-.1*lr);lr;2.4;.95;0;0];
        case "acceleratingTarget"
            q=[24;0;pi;8;1;0;lr;2.4;.95;0;0];
        case "acceleratingTurn"
            q=[24;0;pi;8;1;atan(-.1*lr);lr;2.4;.95;0;0];
        case "brakingTarget"
            q=[24;6;pi;2;-1;0;lr;2.4;.95;0;0];
        case {"headOn","acceleratingHeadOn","stoppedLead","hardBrakingLead","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"}
            [x,q,road,cfg]=collisionThreatScenario(name,controllerConfiguration);
        otherwise,error('runNonlinearPredictiveSafetyValidation:unknownScenario','Unknown scenario %s.',name);
    end
    if ~cfg.controller.enforceRoadBoundaries,road.lateralClearance=[];end
    if ~isempty(q),target=localTarget(q);end
    ego=localEgo(x,0,[0;0]);
end
function ego=localEgo(x,time,input)
    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
        'longitudinalVelocity',x(4),'stateTime',time,'heldActuatorInput',input);
end

function localSaveFailure(file,name,frame,egoTruth,targetTruth,egoInput,targetEstimate, ...
        sensorFrame,previousState,road,configuration,estimatorConfiguration,failure,replay)
    if strlength(file)==0,return;end
    snapshot=struct('scenario',name,'frame',frame,'time',egoTruth.stateTime, ...
        'egoTruth',egoTruth,'targetTruth',targetTruth,'egoInput',egoInput, ...
        'targetEstimate',targetEstimate,'sensorFrame',sensorFrame,'previousState',previousState, ...
        'road',road,'configuration',configuration,'estimatorConfiguration',estimatorConfiguration, ...
        'failure',failure,'replay',replay);
    save(file,'snapshot','-v7.3');
end
function target=localTarget(q)
    direction=[cos(q(3)+q(6));sin(q(3)+q(6))];velocity=q(4)*direction;
    yawRate=q(4)*sin(q(6))/q(7);
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity, ...
        'targetYawInertial',q(3),'targetYawRate',yawRate,'targetSideslip',q(6), ...
        'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7), ...
        'targetAccelerationInertial',q(5)*direction+yawRate*[-velocity(2);velocity(1)], ...
        'targetLength',2*q(8),'targetWidth',2*q(9),'targetRectangleOffset',q(10:11));
end

function summary=localTargetSetSummary(estimate)
    % Half-widths of the published constant-parameter set, for audit only.
    summary=struct('available',false,'positionRadius',NaN,'courseRadius',NaN, ...
        'speedHalfWidth',NaN,'accelerationHalfWidth',NaN,'curvatureHalfWidth',NaN);
    if isempty(estimate) || ~isstruct(estimate) || ~isfield(estimate,'predictionErrorSet') ...
            || isempty(estimate.predictionErrorSet),return;end
    set=estimate.predictionErrorSet;
    summary.available=isequal(set.available,true);
    summary.positionRadius=set.positionRadius;summary.courseRadius=set.courseRadius;
    summary.speedHalfWidth=diff(set.speedInterval)/2;
    summary.accelerationHalfWidth=diff(set.accelerationInterval)/2;
    summary.curvatureHalfWidth=diff(set.curvatureInterval)/2;
end
