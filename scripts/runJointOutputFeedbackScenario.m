function campaign = runJointOutputFeedbackScenario(options)
%runJointOutputFeedbackScenario Raw-sensor predictive-control experiments.
% Radar supplies relative position. Detected low-speed targets also supply
% perceived relative body heading. The NRMM observer estimates target motion.
% An independent ode45 plant generates position GNSS, body IMU, gyro and radar.
% Physical qualification replays a separate truth-based maneuver before the
% benchmark. RandomSeed governs sensor noise. Only diagnostic fixtures that
% intentionally exercise invalid scenes should disable RequirePhysicalWitness.
% The finite initial contract and future noise cases do not cover all bounded
% uncertainty. Reported checks are empirical, with no deterministic guarantee.
    arguments
        options.Scenarios (1,:) string = ["stoppedLead","trafficBrakingLead","trafficHeadOn","trafficCrossing"]
        options.Speeds (1,:) double {mustBePositive,mustBeFinite} = 15
        options.Frames (1,1) double {mustBeInteger,mustBePositive,mustBeFinite} = 200
        options.ControllerConfiguration (1,1) struct = struct()
        options.OutputDirectory (1,1) string = string(tempname)
        options.RandomSeed (1,1) double {mustBeInteger,mustBeNonnegative} = 20261104
        options.RequirePhysicalWitness (1,1) logical = true
        options.Warmup (1,1) logical = true
        options.DropoutHolds (1,:) double {mustBeInteger,mustBeNonnegative} = []
        options.HypothesisOffsets (1,:) double {mustBeFinite,mustBeNonempty} = [-.005,0,.005]
        options.EgoPositionOffsetMeters (2,1) double {mustBeFinite} = [0;0]
        options.EgoYawOffsetRadians (1,1) double {mustBeFinite} = 0
        options.TargetPositionOffsetMeters (2,1) double {mustBeFinite} = [0;0]
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    summaries=struct([]);compactSolversReady=false;
    for speed=options.Speeds
        for name=options.Scenarios
            cfg=collisionAvoidanceControllerConfig(options.ControllerConfiguration);
            if any(string(cfg.outputFeedback.method)==["compactNlp","ipopt"]) && exist('casadi.SX','class')~=8
                setupJointNlpSolver;
            end
            cfg.referenceSpeed=speed;cfg.model.speedMaximum=max(cfg.model.speedMaximum,1.2*speed);
            [x,q,road,cfg]=collisionThreatScenario(name,cfg);
            x(1:2)=x(1:2)+options.EgoPositionOffsetMeters;
            x(3)=x(3)+options.EgoYawOffsetRadians;
            q(1:2)=q(1:2)+options.TargetPositionOffsetMeters;
            reference=road;input=zeros(2,1);time=0;state=[];
            initialClearance=predictiveSafetyGeometry.rectangle(x(1:3), ...
                [cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset],q(1:3),q(8:11));
            truth=struct('ego',x,'target',q,'parameters',ones(4,1),'sensorBias',zeros(7,1));
            initial=localInitialContract(x,options.HypothesisOffsets);
            qualification=[];refusalWitness=[];
            if options.RequirePhysicalWitness
                qualification=bypassFeasibilityBaseline(x,q,road,cfg,HalfWidthMeters=road.scenarioHalfWidthMeters);
            end
            admitted=isempty(qualification) || qualification.qualified;
            startup=localWarmup(cfg,options,truth,reference,initial,admitted && initialClearance>0,compactSolversReady);
            compactSolversReady=startup.solversReady;

            stream=RandStream('mt19937ar','Seed',options.RandomSeed);
            sample=localSensor(truth,input,cfg,time,stream,false);
            block=localSensorBlock(time,input,sample);block.initialHypotheses=initial;
            trace=struct([]);failure="";failurePhase="";minimumClearance=Inf;minimumNodeClearance=Inf;failureCallSeconds=0;
            callSeconds=zeros(1,options.Frames);callCount=0;
            if ~admitted
                failurePhase="scenarioQualification";failure=qualification.reason;
                minimumNodeClearance=initialClearance;
            end
            for frame=1:options.Frames
                if ~admitted,break;end
                if frame==1 && initialClearance<=0
                    minimumNodeClearance=initialClearance;
                    failurePhase="replaySafety";
                    failure="The vehicle footprints touched or overlapped at the initial node.";break;
                end
                callTimer=tic;
                try
                    [command,~,prediction,state]=collisionAvoidanceController(block,[],reference,cfg,state);
                catch exception
                    failureCallSeconds=toc(callTimer);
                    callCount=callCount+1;callSeconds(callCount)=failureCallSeconds;
                    failurePhase="controller";
                    save(fullfile(options.OutputDirectory,sprintf('%g-%s-failure.mat',speed,name)), ...
                        'block','state','reference','cfg');
                    failure=string(exception.identifier)+": "+string(exception.message);
                    if options.RequirePhysicalWitness && strcmp(exception.identifier,'collisionAvoidanceController:compactNoFeasiblePlan')
                        % Use the true state reached by the executed prefix, including
                        % acquisition delay. This diagnostic never supplies a command.
                        if frame==1
                            refusalWitness=qualification;
                        else
                            refusalWitness=bypassFeasibilityBaseline(truth.ego,truth.target,road,cfg, ...
                                PreviousInput=block.heldInput,HalfWidthMeters=road.scenarioHalfWidthMeters);
                        end
                    end
                    break;
                end
                elapsed=toc(callTimer);callCount=callCount+1;callSeconds(callCount)=elapsed;
                input=command.actuatorInput;
                try
                    [truth,samples,clearance,nodeClearance]=localReplay(truth,input,cfg,time,stream, ...
                        ismember(frame,options.DropoutHolds));
                catch exception
                    failurePhase="plantReplay";
                    failure=string(exception.identifier)+": "+string(exception.message);break;
                end
                nextBlock=localSensorBlock(time+cfg.controller.sampleTime,input,samples);
                minimumClearance=min(minimumClearance,clearance);
                minimumNodeClearance=min(minimumNodeClearance,nodeClearance);
                entry=struct('time',time,'input',input,'memory',state.memory, ...
                    'posteriorCount',numel(state.hypotheses),'metadata',prediction.metadata, ...
                    'controllerCallSeconds',elapsed,'deadlineMissed',elapsed>cfg.controller.sampleTime, ...
                    'minimumReplayClearanceMeters',clearance,'minimumNodeClearanceMeters',nodeClearance, ...
                    'stateAfterHold',truth.ego, ...
                    'rawSensorBlock',samples,'targetEstimateAtDecision',localTargetEstimate(state));
                if isempty(trace),trace=entry;else,trace(end+1)=entry;end %#ok<AGROW>
                time=time+cfg.controller.sampleTime;
                % Positive clearance below the planning margin is diagnostic.
                if nodeClearance<=0
                    failurePhase="replaySafety";
                    failure="The vehicle footprints touched or overlapped at a control-boundary node.";break;
                end
                block=nextBlock;
            end
            outcome="completed";if strlength(failure)>0,outcome="failed";end
            if ~admitted,outcome="unqualifiedScenario";end
            file=fullfile(options.OutputDirectory,sprintf('%g-%s.json',speed,name));
            result=struct('scenario',name,'referenceSpeedMetersPerSecond',speed, ...
                'observationModel',"rawSensorEgoConsistencyAndNrmmTargetObserver",'controllerMethod',string(cfg.outputFeedback.method), ...
                'randomSeed',options.RandomSeed, ...
                'outcome',outcome,'requestedFrames',options.Frames,'executedFrames',numel(trace), ...
                'simulatedSeconds',time,'failure',failure,'failurePhase',failurePhase, ...
                'minimumReplayClearanceMeters',minimumClearance,'configuration',cfg, ...
                'collisionCheckScope',"controlBoundaryNodes", ...
                'minimumNodeClearanceMeters',minimumNodeClearance, ...
                'nodeClearanceSatisfied',~isempty(trace)&&minimumNodeClearance>0&&minimumNodeClearance>=cfg.outputFeedback.clearanceMeters-cfg.outputFeedback.nlpTolerance, ...
                'nodeCollisionDetected',minimumNodeClearance<=0, ...
                'sampledSafetyEvaluated',~isempty(trace), ...
                'sampledClearanceSatisfied',~isempty(trace)&&minimumClearance>0&&minimumClearance>=cfg.outputFeedback.clearanceMeters-cfg.outputFeedback.nlpTolerance, ...
                'sampledCollisionDetected',~isempty(trace)&&minimumClearance<=0, ...
                'initialFootprintClearanceMeters',initialClearance,'initialContactOrOverlap',initialClearance<=0, ...
                'finalSpeedMetersPerSecond',hypot(truth.ego(4),truth.ego(5)), ...
                'finalDistanceToTargetMeters',norm(truth.ego(1:2)-truth.target(1:2)), ...
                'finalLongitudinalProgressMeters',truth.ego(1)-x(1), ...
                'startupWarmupSeconds',startup.seconds,'warmupEnabled',options.Warmup, ...
                'warmupCompletedHolds',startup.holds,'warmupFailure',startup.failure, ...
                'compactSolversReadyBeforeFirstFrame',compactSolversReady, ...
                'warmupAvoidanceContinuation',startup.avoidanceContinuation, ...
                'failureCallSeconds',failureCallSeconds, ...
                'initialSensorSample',sample,'initialInformationContract',initial, ...
                'initialPlantState',struct('ego',x,'target',q), ...
                'physicalWitness',qualification,'physicalWitnessAtRefusal',refusalWitness, ...
                'initialContractScope',"finiteEgoPrior;radarAcquisition;nrmmPointMotion", ...
                'actualInitialEgoMemberIncluded',any(options.HypothesisOffsets==0), ...
                'actualInitialMemberIncluded',false, ...
                'experimentOptions',options, ...
                'robustSafetyCertified',false,'timing',localTiming(callSeconds(1:callCount),cfg.controller.sampleTime), ...
                'trace',trace);
            result.solverTiming=localSolverTiming(trace);
            localWriteJson(file,result);
            entry=rmfield(result,{'configuration','trace','initialInformationContract','initialPlantState', ...
                'initialSensorSample','physicalWitness','physicalWitnessAtRefusal'});entry.source=file;
            entry.bypassQualified=[];entry.refusalBypassQualified=[];
            if ~isempty(qualification),entry.bypassQualified=qualification.qualified;end
            if ~isempty(refusalWitness),entry.refusalBypassQualified=refusalWitness.qualified;end
            if isempty(summaries),summaries=entry;else,summaries(end+1)=entry;end %#ok<AGROW>
            fprintf('%s at %g m/s: %s, %d/%d holds, mean/max call %.1f/%.1f ms, %d deadline misses\n', ...
                name,speed,outcome,numel(trace),options.Frames,1000*result.timing.meanSeconds, ...
                1000*result.timing.maximumSeconds,result.timing.deadlineMisses);
        end
    end
    campaign=struct('scope',"raw-sensor control with an explicitly declared finite initial information contract", ...
        'pipeline',string(cfg.outputFeedback.method),'results',summaries);
    localWriteJson(fullfile(options.OutputDirectory,'campaign.json'),campaign);
end

function startup=localWarmup(cfg,options,truth,reference,initial,canRun,solversReady)
    startup=struct('seconds',0,'holds',0,'failure',"",'solversReady',solversReady,'avoidanceContinuation',false);
    if ~options.Warmup || ~canRun,return;end
    timer=tic;input=zeros(2,1);state=[];compact=string(cfg.outputFeedback.method)=="compactNlp";
    if compact && ~solversReady
        try
            startup.solversReady=localWarmCompactSolvers(cfg);
            if ~startup.solversReady
                startup.failure="The startup probe did not exercise both the active-set QP and sparse linear solver.";
            end
        catch exception
            startup.failure=string(exception.identifier)+": "+string(exception.message);
        end
    end
    if compact
        % Use a private sensor stream to exercise target acquisition and
        % the following active continuation with the simulation's actual configuration.
        stream=RandStream('mt19937ar','Seed',0);reference=[-100,0;1000,0];
        initial=localInitialContract([0;0;0;cfg.referenceSpeed;0;0],0);
        truth=initial;
        truth.target=[cfg.collision.encounterRangeMeters/2;cfg.vehicle.width+cfg.target.defaultWidth; ...
            0;0;0;0;cfg.target.rearAxleDistance;cfg.target.defaultLength/2;cfg.target.defaultWidth/2;0;0];
        sample=localSensor(truth,input,cfg,0,stream,false);
        observerCfg=estimatorControllerIntegrationConfig();
        samplesPerHold=round(cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod);
        count=2+ceil((observerCfg.initialization.minimumRadarSamples-1)/samplesPerHold);
    else
        % Match the original four-hold replay of the research comparators.
        stream=RandStream('mt19937ar','Seed',options.RandomSeed);
        sample=localSensor(truth,input,cfg,0,stream,false);
        reset(stream);count=4;
    end
    block=localSensorBlock(0,input,sample);block.initialHypotheses=initial;
    for k=1:count
        try
            [command,~,prediction,state]=collisionAvoidanceController(block,[],reference,cfg,state);
            input=command.actuatorInput;
            if compact
                info=prediction.metadata;
                startup.avoidanceContinuation=startup.avoidanceContinuation || ...
                    (info.collisionTargetActive && info.budgetPhase=="continuation");
            end
            [truth,samples]=localReplay(truth,input,cfg,block.time,stream,false);
            block=localSensorBlock(block.time+cfg.controller.sampleTime,input,samples);
            startup.holds=startup.holds+1;
        catch exception
            startup.failure=strtrim(startup.failure+" "+string(exception.identifier)+": "+string(exception.message));
            break;
        end
    end
    startup.seconds=toc(timer);
end

function ready=localWarmCompactSolvers(runCfg)
    % A fixed synthetic encounter exercises the projection QP, sparse LP
    % and trajectory derivatives before the experiment clock starts.
    cfg=collisionAvoidanceControllerConfig();
    % This discarded offline solve uses the solver iteration limits. An
    % online wall-clock deadline can interrupt first-time toolbox loading.
    cfg.outputFeedback.initializationSolveSeconds=realmax;
    cfg.outputFeedback.nlpJit=runCfg.outputFeedback.nlpJit;
    cfg.outputFeedback.nlpCompiler=runCfg.outputFeedback.nlpCompiler;
    cfg.outputFeedback.nlpCompilerOptimization=runCfg.outputFeedback.nlpCompilerOptimization;
    item=localInitialContract([0;0;0;15;0;0],0);
    item.target=[25;0;0;0;0;0;cfg.target.rearAxleDistance; ...
        cfg.target.defaultLength/2;cfg.target.defaultWidth/2;0;0];
    input=zeros(2,1);
    sample=solvePredictiveControl("outputFeedback",struct('operation',"sensor", ...
        'cfg',cfg,'hypothesis',item,'input',input,'time',0));
    block=localSensorBlock(0,input,sample);block.initialHypotheses=item;
    % The probe has no access to the experiment state or controller memory.
    request=struct('operation',"step",'cfg',cfg,'reference',[-100,0;1000,0], ...
        'block',block,'previousState',[]);
    result=solvePredictiveControl("outputFeedback",request);
    info=result.prediction.metadata;
    lpReady=info.nlpAttempts>0;
    if ~lpReady
        % Load the same bounded stage-deficit LP backend when the projection
        % already reached zero on the offline fixture.
        opts=optimoptions('linprog','Algorithm','dual-simplex-highs','Display','none');
        [~,~,flag]=linprog([0;1],sparse([1,-1]),-.01,[],[],[-1;0],[1;.1],opts);
        lpReady=flag>0;
    end
    ready=info.zeroDeficitQpSteps>0 && lpReady;
end

function timing=localTiming(seconds,deadline)
    ordered=sort(seconds);count=numel(seconds);
    if count==0,ordered=NaN;seconds=NaN;end
    timing=struct('scope',"wholeControllerCallIncludingRejectedCallsExcludingStartupWarmupAndOfflinePlantReplay", ...
        'calls',count,'deadlineSeconds',deadline,'meanSeconds',mean(seconds), ...
        'medianSeconds',median(seconds),'p95Seconds',ordered(max(1,ceil(.95*count))), ...
        'maximumSeconds',max(seconds),'deadlineMisses',sum(seconds>deadline));
end

function timing=localSolverTiming(trace)
    seconds=[];
    for j=1:numel(trace)
        if isfield(trace(j).metadata,'nlpSeconds'),seconds(end+1)=trace(j).metadata.nlpSeconds;end %#ok<AGROW>
    end
    timing=localTiming(seconds,.1);
    timing.scope="acceptedCallsOptimizationIncludingTrialChecksExcludingCompilationAndRetainedPlanPreparation";
    timing.meanUnder100Milliseconds=~isempty(seconds) && mean(seconds)<.1;
end

function hypotheses=localInitialContract(x,offsets)
    % This is an experimental prior, not an estimate derived from truth at
    % every decision. Common translation and friction remain correlated.
    prototype=struct('ego',x,'target',[],'parameters',ones(4,1),'sensorBias',zeros(7,1));
    hypotheses=repmat(prototype,1,numel(offsets));
    for j=1:numel(offsets)
        offset=offsets(j);
        hypotheses(j).ego(1:2)=x(1:2)+[offset;offset];
        hypotheses(j).ego(4)=x(4)+offset;
        hypotheses(j).parameters(3:4)=1+offset;
    end
end

function block=localSensorBlock(time,input,samples)
    block=struct('kind',"jointSensorBlock-v1",'time',time,'heldInput',input,'samples',samples);
end

function estimate=localTargetEstimate(state)
    estimate=[];
    if isempty(state.hypotheses),return;end
    item=state.hypotheses(1);
    if isempty(item.target),return;end
    estimate=struct('state',item.target,'member',1,'lastReturn',item.targetObserver.lastReturn, ...
        'detections',item.targetObserver.detections,'motionInitialized',item.targetObserver.initialized, ...
        'restCompatible',item.targetObserver.restCompatible);
    if isfield(item.targetObserver,'motionBounds')
        estimate.accelerationIntervalMetersPerSecondSquared=item.targetObserver.motionBounds(:,1);
        estimate.sideslipIntervalRadians=item.targetObserver.motionBounds(:,2);
    end
end

function y=localSensor(truth,input,cfg,time,stream,dropout)
    persistent sensorSettings
    if isempty(sensorSettings),sensorSettings=estimatorControllerIntegrationConfig();end
    y=solvePredictiveControl('outputFeedback',struct('operation',"sensor",'cfg',cfg, ...
        'hypothesis',truth,'input',input,'time',time));
    bounds=cfg.outputFeedback.noiseBounds;
    y.position=y.position+localNoise(stream,.5*bounds(1));
    y.acceleration=y.acceleration+localNoise(stream,.5*bounds(2));
    y.yawRate=y.yawRate+.5*bounds(3)*(2*rand(stream)-1);
    % This generator uses a uniform disk of half the general sensor envelope.
    % Publish its fixed radius with the return, independently of the drawn sample.
    y.radarPositionErrorMeters=.5*bounds(4);
    y.relativePosition=y.relativePosition+localNoise(stream,y.radarPositionErrorMeters);
    if dropout,y.radarAvailable=false;y.relativePosition=[NaN;NaN];end
    y.relativeHeading=NaN;
    historySpan=(sensorSettings.initialization.minimumRadarSamples-1)*cfg.outputFeedback.sensorPeriod;
    if y.radarAvailable && abs(truth.target(4))*historySpan<=2*sensorSettings.sensor.radar.positionNoiseMaximum
        % Supply ideal yaw before rest when motion at the current speed
        % over the acquisition window is within its two-return error bound.
        angle=truth.target(3)-truth.ego(3);
        y.relativeHeading=atan2(sin(angle),cos(angle));
    end
end

function noise=localNoise(stream,radius)
    angle=2*pi*rand(stream);noise=radius*sqrt(rand(stream))*[cos(angle);sin(angle)];
end

function [truth,samples,clearance,nodeClearance]=localReplay(truth,input,cfg,time,stream,dropout)
    h=cfg.controller.sampleTime;dt=cfg.outputFeedback.sensorPeriod;count=round(h/dt);
    start=truth;ode=ode45(@(~,x)nonlinearBicycleModel.derivative(x,input,cfg),[0,h],truth.ego, ...
        odeset('RelTol',1e-10,'AbsTol',1e-12));
    samples=struct([]);
    for j=1:count
        truth.ego=deval(ode,j*dt);
        if j==count,truth.ego=nonlinearBicycleModel.sampleEndState(truth.ego,input,cfg);end
        if start.target(4)>0
            truth.target=predictiveSafetyGeometry.predictTarget(start.target,j*dt);
            truth.target(5:6)=start.target(5:6);
        end
        y=localSensor(truth,input,cfg,time+j*dt,stream,dropout);
        if isempty(samples),samples=y;else,samples(end+1)=y;end %#ok<AGROW>
    end
    % Dense clearance remains an independent diagnostic. Only the actual
    % control-boundary nodes determine the node-safety campaign outcome.
    clearance=Inf;nodeClearance=Inf;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    for t=linspace(0,h,41)
        x=deval(ode,t);q=start.target;
        if q(4)>0,q=predictiveSafetyGeometry.predictTarget(q,t);end
        gap=predictiveSafetyGeometry.rectangle(x(1:3),shape,q(1:3),q(8:11));
        clearance=min(clearance,gap);
        if t==0 || t==h,nodeClearance=min(nodeClearance,gap);end
    end
end

function localWriteJson(path,value)
    file=fopen(path,'w');assert(file>=0);cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(value,PrettyPrint=true));
end
