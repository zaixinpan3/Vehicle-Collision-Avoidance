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
        options.EgoPositionOffsetMeters (2,1) double {mustBeFinite} = [0;0]
        options.EgoYawOffsetRadians (1,1) double {mustBeFinite} = 0
        options.TargetPositionOffsetMeters (2,1) double {mustBeFinite} = [0;0]
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'estimator'));
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    summaries=struct([]);
    base=collisionAvoidanceControllerConfig(options.ControllerConfiguration);
    if isempty(base.target.observerDomain)
        % Use the complete benchmark population before selecting or observing a target.
        base.target.observerDomain=collisionThreatContract(strings(1,0),options.Speeds, ...
            WindowSeconds=max(20,options.Frames*base.controller.sampleTime),ControllerConfiguration=base);
    end
    startupTimer=tic;
    observer=onlineNrmmTrackingRuntime("prepareTarget",base);
    if exist('casadi.SX','class')~=8,setupJointNlpSolver;end
    % Finish all configuration-specific preparation before any measured case.
    [speeds,~,speedIndex]=unique(options.Speeds,'stable');
    configurations=cell(size(speeds));preparations=cell(size(speeds));
    for i=1:numel(speeds)
        cfg=base;cfg.referenceSpeed=speeds(i);
        cfg.model.speedMaximum=max(base.model.speedMaximum,1.2*speeds(i));
        configurations{i}=cfg;
        preparations{i}=localWarmup(cfg,options.Warmup,observer.domain.speedMaximum);
    end
    campaignWarmupSeconds=0;
    if options.Warmup,campaignWarmupSeconds=toc(startupTimer);end
    for i=1:numel(options.Speeds)
        speed=options.Speeds(i);startup=preparations{speedIndex(i)};
        for name=options.Scenarios
            [x,q,road,cfg]=collisionThreatScenario(name,configurations{speedIndex(i)});
            x(1:2)=x(1:2)+options.EgoPositionOffsetMeters;
            x(3)=x(3)+options.EgoYawOffsetRadians;
            q(1:2)=q(1:2)+options.TargetPositionOffsetMeters;
            reference=road;input=zeros(2,1);time=0;state=[];
            initialClearance=predictiveSafetyGeometry.rectangle(x(1:3), ...
                [cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset],q(1:3),q(8:11));
            initial=localInitialContract(x);
            truth=initial;truth.target=q;
            qualification=[];refusalWitness=[];
            if options.RequirePhysicalWitness
                qualification=bypassFeasibilityBaseline(x,q,road,cfg,HalfWidthMeters=road.scenarioHalfWidthMeters);
            end
            admitted=isempty(qualification) || qualification.qualified;

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
                'compactSolversReadyBeforeFirstFrame',startup.solversReady,'warmupIpoptCalls',startup.ipoptCalls, ...
                'warmupIpoptIterations',startup.ipoptIterations, ...
                'warmupAvoidanceContinuation',startup.avoidanceContinuation, ...
                'failureCallSeconds',failureCallSeconds, ...
                'initialSensorSample',sample,'initialInformationContract',initial, ...
                'initialPlantState',struct('ego',x,'target',q), ...
                'physicalWitness',qualification,'physicalWitnessAtRefusal',refusalWitness, ...
                'initialContractScope',"knownInitialEgo;radarAcquisition;nrmmPointMotion", ...
                'actualInitialEgoMemberIncluded',true, ...
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
    campaign=struct('scope',"raw-sensor control from one known initial ego state", ...
        'pipeline',string(cfg.outputFeedback.method),'startupWarmupSeconds',campaignWarmupSeconds,'results',summaries);
    localWriteJson(fullfile(options.OutputDirectory,'campaign.json'),campaign);
end

function startup=localWarmup(cfg,enabled,targetSpeedMaximum)
    startup=struct('seconds',0,'holds',0,'failure',"",'solversReady',false, ...
        'avoidanceContinuation',false,'ipoptCalls',0,'ipoptIterations',0);
    if ~enabled,return;end
    timer=tic;reference=[-100,0;1000,0];
    discardState=onCleanup(@()collisionAvoidanceController("resetNominalTrajectory"));
    initial=localInitialContract([0;0;0;cfg.referenceSpeed;0;0]);
    privateTarget=[cfg.collision.encounterRangeMeters/2;cfg.vehicle.width+cfg.target.defaultWidth; ...
        0;0;0;0;cfg.target.rearAxleDistance;cfg.target.defaultLength/2;cfg.target.defaultWidth/2;0;0];
    observerCfg=estimatorControllerIntegrationConfig();
    samplesPerHold=round(cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod);
    windowSamples=observerCfg.sensor.radar.motionWindowSamples;
    windowSeconds=(windowSamples-1)*cfg.outputFeedback.sensorPeriod;
    probeSpeeds=0;
    if targetSpeedMaximum*windowSeconds>3*cfg.outputFeedback.noiseBounds(4)
        % Allow a resolvable displacement even at the two opposite noise
        % extremes of the private radar generator (half the declared radius).
        movingSpeed=min(targetSpeedMaximum,max(cfg.referenceSpeed, ...
            4*cfg.outputFeedback.noiseBounds(4)/windowSeconds));
        probeSpeeds=[0,movingSpeed];
    end
    count=2+ceil((windowSamples-1)/samplesPerHold);
    for targetSpeed=probeSpeeds
        state=[];input=zeros(2,1);truth=initial;truth.target=privateTarget;
        truth.target(4)=targetSpeed;
        if targetSpeed>0
            % A receding track exercises moving-target inference and prediction.
            truth.target(1)=-truth.target(1);
            truth.target(3)=pi;
        end
        stream=RandStream('mt19937ar','Seed',0);
        sample=localSensor(truth,input,cfg,0,stream,false);
        block=localSensorBlock(0,input,sample);block.initialHypotheses=initial;
        prepared=false;continued=false;
        for k=1:count
            if ~prepared
                request=struct('operation',"prepareStep",'cfg',cfg,'reference',reference, ...
                    'block',block,'previousState',state);
                result=solvePredictiveControl("outputFeedback",request);
                command=result.command;prediction=result.prediction;state=result.state;
            else
                [command,~,prediction,state]=collisionAvoidanceController(block,[],reference,cfg,state);
            end
            info=prediction.metadata;
            startup.ipoptCalls=startup.ipoptCalls+info.nlpAttempts;
            startup.ipoptIterations=startup.ipoptIterations+info.nlpIterations;
            continued=prepared && info.collisionTargetActive && info.budgetPhase=="continuation";
            observedMotion=info.collisionTargetActive && ((state.hypotheses(1).target(4)>0)==(targetSpeed>0));
            prepared=prepared || (observedMotion && info.nlpAttempts>0 && ...
                info.nonlinearDerivativeEvaluations>0 && info.nonlinearValidationPerformed);
            startup.holds=startup.holds+1;
            if continued,break;end
            if k<count
                input=command.actuatorInput;
                [truth,samples]=localReplay(truth,input,cfg,block.time,stream,false);
                block=localSensorBlock(block.time+cfg.controller.sampleTime,input,samples);
            end
        end
        assert(prepared && continued,'runJointOutputFeedbackScenario:incompleteWarmup', ...
            'Private startup at ego/target speeds %g/%g m/s did not exercise IPOPT, derivatives and active continuation.', ...
            cfg.referenceSpeed,targetSpeed);
    end
    startup.solversReady=true;startup.avoidanceContinuation=true;
    startup.seconds=toc(timer);
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

function initial=localInitialContract(x)
    initial=struct('ego',x,'target',[],'parameters',ones(4,1),'sensorBias',zeros(7,1));
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
    if item.targetObserver.initialized && ~isempty(item.targetObserver.motionBounds)
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
    historySpan=(sensorSettings.sensor.radar.motionWindowSamples-1)*cfg.outputFeedback.sensorPeriod;
    if y.radarAvailable && abs(truth.target(4))*historySpan<=2*sensorSettings.sensor.radar.positionNoiseMaximum
        % Supply ideal yaw before rest when motion at the current speed
        % over the motion window is within its two-return error bound.
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
