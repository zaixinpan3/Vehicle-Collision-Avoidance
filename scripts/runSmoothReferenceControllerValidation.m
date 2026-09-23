function summary=runSmoothReferenceControllerValidation(options)
%runSmoothReferenceControllerValidation Controller-only varying-curvature trials.
% Each path has an explicit constant-curvature continuation. The executed
% study plant is precisely the issued held affine generator. Physical pose
% and rectangle-distance diagnostics use the smooth spatial reference.
    arguments
        options.OutputDirectory (1,1) string
        options.SampleCount (1,1) double {mustBeInteger,mustBePositive} = 600
        options.RunStrictTiming (1,1) logical = false
        options.IncludeEncounters (1,1) logical = true
        options.Paths (1,:) string {mustBeMember(options.Paths,["sBend","transition","asymmetric"])} = ["sBend","transition","asymmetric"]
        options.DiagnosticDeadlineSeconds (1,1) double {mustBeFinite,mustBePositive} = 5
        options.SampleTime (1,1) double {mustBeFinite,mustBePositive} = .05
        options.HorizonSeconds (1,1) double {mustBeFinite,mustBePositive} = 1.6
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'solver','bicycle'));
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    previousThreads=maxNumCompThreads(1);
    cleanup=onCleanup(@()maxNumCompThreads(previousThreads));
    summary=struct('matlabVersion',string(version),'sampleTime',options.SampleTime, ...
        'horizonSteps',round(options.HorizonSeconds/options.SampleTime),'referenceSpeed',8, ...
        'computationalThreads',1,'roadBoundariesEnabled',false,'estimatorEnabled',false, ...
        'requestedHolds',options.SampleCount,'trials',{{}}, ...
        'scope',"Exact declared scheduled affine plant and exact ego/target states; physical pose follows spatial PCHIP curvature; no nonlinear vehicle claim; offline audits excluded from frame timing");
    names=options.Paths;scenarios=repmat("cruise",size(names));
    if options.IncludeEncounters && any(options.Paths=="sBend")
        names=[names,"sBend","sBend"];scenarios=[scenarios,"stationary","oncoming"];
    end
    for index=1:numel(names)
        name=names(index);scenario=scenarios(index);
        directory=fullfile(options.OutputDirectory,name,scenario,"diagnostic");
        timing=struct('sampleTime',options.SampleTime,'horizonSteps',round(options.HorizonSeconds/options.SampleTime));
        trial=localTrial(name,scenario,options.SampleCount,options.DiagnosticDeadlineSeconds,directory,timing);
        if options.RunStrictTiming
            directory=fullfile(options.OutputDirectory,name,scenario,"periodic");
            trial.strictTiming=localTrial(name,scenario,options.SampleCount,options.SampleTime,directory,timing);
        end
        summary.trials{end+1}=trial;
        localSaveJson(summary,fullfile(options.OutputDirectory,'smooth-reference-summary.json'));
    end
    summary.allCompleted=all(cellfun(@(trial)trial.completed,summary.trials));
    summary.allRuntimeQualified=all(cellfun(@(trial)trial.runtimeQualified,summary.trials));
    localSaveJson(summary,fullfile(options.OutputDirectory,'smooth-reference-summary.json'));
end

function report=localTrial(name,scenario,count,deadline,directory,timing)
% The strict trial uses the sample period as its whole-frame deadline; the
% cruise horizon keeps the same duration in seconds when the period changes.
    if ~isfolder(directory),mkdir(directory);end
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',timing.sampleTime,'horizonSteps',timing.horizonSteps,'minimumHorizonSteps',1), ...
        'solver',struct('frameDeadlineSeconds',deadline,'certificateSearchTimeLimit',deadline)));
    h=cfg.controller.sampleTime;curve=localCurve(name);road=struct('referenceCurve',curve);
    [state,heldInput]=ltvBicycleModel.cruiseEquilibrium(curve.curvature,cfg);
    ego=localEgo(state,0,heldInput,road);
    setupTimer=tic;
    [~,lane,parsedRoad]=readPlanningInputs(ego,[],road,cfg);
    model=struct('cfg',cfg,'lane',lane,'road',parsedRoad,'sampleTime',h,'stateTime',0, ...
        'initialEgoState',state,'initialFrenetErrorBound',zeros(6,1), ...
        'longitudinalAccelerationBias',0,'previousInput',heldInput,'referenceSpeed',cfg.referenceSpeed);
    setupFailure=[];preparation=[];
    try
        ltvBicycleModel.referenceSchedule(model);
        compileSeconds=toc(setupTimer);
        % Startup probes use the scenario's declared target at its first sample
        % inside the perception range, so the encounter admission paths are
        % compiled before periodic sampling; the probes issue no command.
        [target,probeTime]=localProbeTarget(scenario,curve,ego,count,h);
        startupCfg=cfg;startupCfg.solver.frameDeadlineSeconds=5;startupCfg.solver.certificateSearchTimeLimit=5;
        preparation=prepareCollisionAvoidanceController(ego,road,startupCfg,target);
        preparation.probeTargetTime=probeTime;
        if scenario=="cruise",preparation.cruiseProbes=localCruiseProbes(ego,road,startupCfg);end
    catch exception
        compileSeconds=toc(setupTimer);setupFailure=exception;
    end
    states=nan(6,count+1);states(:,1)=state;inputs=nan(2,count);seconds=nan(1,count);
    actualCurvature=nan(1,count);stageCurvature=nan(1,count);phaseError=nan(1,count);
    tracking=nan(5,count+1);tracking(:,1)=0;certified=false(1,count);inherited=false(1,count);
    released=false(1,count);runtime=cell(1,count);search=cell(1,count);clfSlack=nan(1,count);
    failure=setupFailure;stored=[];executed=0;attempted=0;minimumSeparation=inf;maximumSlew=0;
    minimumNodeSeparation=inf;
    for sample=1:count
        if ~isempty(failure),break;end
        time=(sample-1)*h;frameTimer=tic;attempted=sample;
        ego=localEgo(state,time,heldInput,lane);target=localTarget(scenario,time,curve);
        try
            [command,~,problem,stored]=collisionAvoidanceController(ego,target,road,cfg,stored);
        catch exception
            seconds(sample)=toc(frameTimer);failure=exception;break;
        end
        seconds(sample)=toc(frameTimer);metadata=problem.metadata;
        actualCurvature(sample)=laneGeometry.referenceCurvature(state(1),curve);
        stageCurvature(sample)=metadata.clfOperatingCurvature;
        reference=ltvBicycleModel.referenceAt(model,sample);
        phaseError(sample)=state(1)-reference.state(1);
        actualTrim=ltvBicycleModel.cruiseEquilibrium(actualCurvature(sample),cfg);
        tracking(:,sample)=state(2:6)-actualTrim(2:6);
        certified(sample)=metadata.planCertified;inherited(sample)=metadata.inheritedFeasibleFamily;
        released(sample)=metadata.confirmedRelease;runtime{sample}=metadata.runtime;
        search{sample}=metadata.admissionSearch;clfSlack(sample)=metadata.clfSlack;
        inputs(:,sample)=command.actuatorInput;
        slew=abs(command.actuatorInput-heldInput)-h*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
        maximumSlew=max([maximumSlew;slew]);
        generator=[metadata.executedContinuousGenerator;zeros(3,9)];
        for fraction=linspace(0,1,11)
            value=expm(fraction*h*generator)*[state;command.actuatorInput;1];
            [position,heading]=laneGeometry.fromFrenet(value(1:6),lane);
            truth=localTarget(scenario,time+fraction*h,curve);
            if ~isempty(truth)
                separation=avoidanceSafetyGeometry.rectangleDistance(position,heading, ...
                    truth.targetPositionInertial,truth.targetHeadingInertial, ...
                    [cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.target.defaultLength/2;cfg.target.defaultWidth/2]);
                minimumSeparation=min(minimumSeparation,separation);
                if fraction==0 || fraction==1
                    minimumNodeSeparation=min(minimumNodeSeparation,separation);
                end
            end
        end
        state=value(1:6);heldInput=command.actuatorInput;states(:,sample+1)=state;executed=sample;
    end
    finalTrim=ltvBicycleModel.cruiseEquilibrium(laneGeometry.referenceCurvature(state(1),curve),cfg);
    tracking(:,executed+1)=state(2:6)-finalTrim(2:6);
    report=struct('path',name,'scenario',scenario,'configuration',cfg,'road',road, ...
        'coldReferenceCompilationSeconds',compileSeconds,'preparation',preparation, ...
        'requestedHolds',count,'executedHolds',executed,'durationSeconds',executed*h, ...
        'completed',isempty(failure),'time',(0:executed)*h,'state',states(:,1:executed+1), ...
        'input',inputs(:,1:executed),'actualSpatialCurvature',actualCurvature(1:executed), ...
        'scheduledCurvature',stageCurvature(1:executed),'stationPhaseError',phaseError(1:executed), ...
        'trackingError',tracking(:,1:executed+1),'finalTrackingError',tracking(:,executed+1), ...
        'frameSeconds',seconds(1:attempted),'runtimeBreakdown',{runtime(1:executed)}, ...
        'admissionSearch',{search(1:executed)},'clfSlack',clfSlack(1:executed), ...
        'planCertified',certified(1:executed),'inheritedFeasibleFamily',inherited(1:executed), ...
        'confirmedRelease',released(1:executed),'minimumSampledSeparationMargin',minimumSeparation, ...
        'minimumNodeSeparationMargin',minimumNodeSeparation, ...
        'maximumSlewViolation',maximumSlew,'failureIdentifier',"",'failureMessage',"", ...
        'failureTime',NaN,'failureContext',struct('state',state,'heldInput',heldInput,'attemptedFrame',attempted), ...
        'scope',"Exact immutable reference-phase affine plant; the hard certificate covers the hold nodes; the 11-point sampled clearance is an inter-node physical diagnostic");
    if isempty(setupFailure)
        failureReference=ltvBicycleModel.referenceAt(model,executed+1);
        report.failureContext.phaseStation=failureReference.state(1);
        report.failureContext.stationPhaseError=state(1)-failureReference.state(1);
        report.failureContext.actualSpatialCurvature=laneGeometry.referenceCurvature(state(1),curve);
        report.failureContext.scheduledCurvature=failureReference.stage.curvature;
    end
    report.passed=report.completed && all(certified(1:executed)) && minimumSeparation>0 && maximumSlew<=1e-8;
    report.deadlineMisses=nnz(seconds(1:attempted)>h);
    report.maximumFrameMilliseconds=1000*max([0,seconds(1:attempted)]);
    report.medianFrameMilliseconds=1000*median(seconds(1:attempted));
    report.runtimeQualified=report.passed && report.deadlineMisses==0;
    report.maximumSpatialScheduleCurvatureDifference=max([0,abs(actualCurvature(1:executed)-stageCurvature(1:executed))]);
    report.maximumStationPhaseError=max([0,abs(phaseError(1:executed))]);
    if ~isempty(failure)
        report.failureIdentifier=string(failure.identifier);report.failureMessage=string(failure.message);
        report.failureTime=executed*h;
    end
    save(fullfile(directory,'smooth-reference-trial.mat'),'report');
    localSaveJson(report,fullfile(directory,'smooth-reference-trial.json'));
    fprintf('%s/%s: %d/%d holds; clearance %.6g m (nodes %.6g m); phase error %.6g m; max frame %.3f ms; %s\n', ...
        name,scenario,executed,count,minimumSeparation,minimumNodeSeparation,report.maximumStationPhaseError, ...
        report.maximumFrameMilliseconds,report.failureIdentifier);
end

function probes=localCruiseProbes(ego,road,cfg)
% Discarded target-free startup probes. Each fresh probe is followed by two
% chained calls on its own declared successor so that the carried-witness
% cruise path is compiled before the first periodic sample.
    probes=struct('seconds',zeros(1,3),'certified',false(1,3),'failureIdentifier',strings(1,3), ...
        'successorSeconds',zeros(3,2),'successorFailureIdentifier',strings(3,2));
    for repetition=1:3
        timer=tic;
        try
            [~,~,problem,stored]=collisionAvoidanceController(ego,[],road,cfg,[]);
            probes.certified(repetition)=problem.metadata.planCertified;
        catch exception
            probes.failureIdentifier(repetition)=string(exception.identifier);
        end
        probes.seconds(repetition)=toc(timer);
        if strlength(probes.failureIdentifier(repetition))>0,continue;end
        for step=1:2
            timer=tic;
            try
                lane=problem.model.lane;h=problem.model.sampleTime;
                successor=localEgo(stored.predictedState(:,2),problem.model.stateTime+h,stored.appliedInput,lane);
                [~,~,problem,stored]=collisionAvoidanceController(successor,[],road,cfg,stored);
            catch exception
                probes.successorFailureIdentifier(repetition,step)=string(exception.identifier);
            end
            probes.successorSeconds(repetition,step)=toc(timer);
            if strlength(probes.successorFailureIdentifier(repetition,step))>0,break;end
        end
    end
end

function curve=localCurve(name)
    station=(0:10:120).';
    switch name
        case "sBend",curvature=.015*sin(2*pi*station/120);curvature([1,end])=0;
        case "transition",curvature=.01*(1-cos(pi*station/120));
        otherwise
            station=[0;20;45;75;100;120];curvature=[0;.005;.018;.010;0;0];
    end
    curve=struct('origin',[0;0],'heading',0,'curvature',curvature(1),'length',120, ...
        'curvatureProfile',[station,curvature],'continuation','constantCurvature');
end

function ego=localEgo(state,time,input,lane)
    [position,heading]=laneGeometry.fromFrenet(state,lane);
    ego=struct('position',position,'yaw',heading,'speed',state(4),'lateralVelocity',state(5), ...
        'yawRate',state(6),'stateTime',time,'heldActuatorInput',input, ...
        'controllerStateErrorBound',zeros(6,1), ...
        'perception',struct('time',time,'range',16,'completeWithinRange',true));
end

function [target,probeTime]=localProbeTarget(scenario,curve,ego,count,h)
% The declared target at its first sample within the initial perception range.
    probeTime=NaN;target=localTarget(scenario,0,curve);
    if isempty(target),return;end
    for sample=0:count
        candidate=localTarget(scenario,sample*h,curve);
        if norm(candidate.targetPositionInertial-ego.position)<=ego.perception.range
            target=candidate;probeTime=sample*h;return;
        end
    end
end

function target=localTarget(scenario,time,curve)
    if scenario=="cruise",target=[];return;end
    [position,heading]=laneGeometry.referencePose(15,0,curve);velocity=zeros(2,1);
    if scenario=="oncoming"
        [point,heading]=laneGeometry.referencePose(30,0,curve);tangent=[cos(heading);sin(heading)];
        position=point+30*tangent;velocity=-8*tangent;heading=heading+pi;
    end
    target=struct('trackId',1,'targetPositionInertial',position+velocity*time, ...
        'targetVelocityInertial',velocity,'targetAccelerationInertial',zeros(2,1), ...
        'targetHeadingInertial',heading,'targetYawRate',0, ...
        'predictionMotion',struct('kind','finite-sensing-motion-v1','jerkBound',[0;0],'yawAccelerationBound',0));
end

function localSaveJson(value,path)
    file=fopen(path,'w');assert(file>=0);cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(value,PrettyPrint=true));
end
