function report = runExactStateRecursiveFeasibilityScenario(options)
%runExactStateRecursiveFeasibilityScenario Exact affine predictive-control experiment.
% Recursive feasibility is conditional on the declared model, sensing and
% encounter-admission contracts. Numerical failure still terminates the run.
% Geometry and CLF measurements below are offline experimental diagnostics;
% they do not accept, reject, repair, or select a controller command.
% Road boundaries are opt-in. The declared model domain always remains hard.
    arguments
        options.Scenario (1,1) string {mustBeMember(options.Scenario,["stationary","oncoming","crossing","cruise"])} = "stationary"
        options.SampleCount (1,1) double {mustBeInteger,mustBePositive} = 120
        options.UseRoadBoundaries (1,1) logical = false
        options.FailAfterAdmission (1,1) logical = false
        options.OutputDirectory (1,1) string = ""
        options.DeadlineSeconds (1,1) double {mustBePositive} = 0.1
        options.EgoErrorBound (6,1) double {mustBeNonnegative,mustBeFinite} = zeros(6,1)
        options.TargetErrorBound (8,1) double {mustBeNonnegative,mustBeFinite} = zeros(8,1)
        options.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 20260912
        options.TargetJerkAmplitude (2,1) double {mustBeFinite} = zeros(2,1)
        options.TargetYawAccelerationAmplitude (1,1) double {mustBeFinite} = 0
        options.TargetMotionFrequency (1,1) double {mustBeFinite,mustBePositive} = 1
        options.InitialTrackingError (5,1) double {mustBeFinite} = zeros(5,1)
        options.ConfirmationRange (1,1) double {mustBeFinite,mustBePositive} = 16
        options.MinimumHorizonSteps (1,1) double {mustBeInteger,mustBePositive} = 1
        options.ExecutionPolicy (1,1) string = "finiteBranches"
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"controller"),fullfile(root,"config"));
    cfg = collisionAvoidanceControllerConfig(struct("referenceSpeed",8, ...
        "controller",struct("sampleTime",0.1,"minimumHorizonSteps",options.MinimumHorizonSteps, ...
        "executionPolicy",options.ExecutionPolicy),"model",struct("lateralDomainRadius",4), ...
        "solver",struct("frameDeadlineSeconds",options.DeadlineSeconds)));
    originalCfg = cfg;
    stream = RandStream("mt19937ar",Seed=options.Seed);
    h = cfg.controller.sampleTime;
    truthTarget = struct("center",[15;0;0;0;0;0;0;0],"radius",zeros(8,1), ...
        "jerkAmplitude",options.TargetJerkAmplitude, ...
        "yawAccelerationAmplitude",options.TargetYawAccelerationAmplitude, ...
        "frequency",options.TargetMotionFrequency, ...
        "halfLength",cfg.target.defaultLength/2,"halfWidth",cfg.target.defaultWidth/2);
    if options.Scenario=="oncoming",truthTarget.center=[60;0;-8;0;0;0;pi;0];end
    if options.Scenario=="crossing",truthTarget.center=[15;-4;0;32;0;0;pi/2;0];end
    road = struct("centerline",[-100,0;2000,0]);
    if options.UseRoadBoundaries
        boundary = struct("origin",zeros(2,1),"longitudinalDirection",[1;0], ...
            "lateralDirection",[0;1],"coefficients",[0;0;-5], ...
            "parameterRange",[-100;2000],"safeSideSign",1);
        boundaries = [boundary;boundary];boundaries(2).coefficients(3)=5;boundaries(2).safeSideSign=-1;
        road.boundaries = boundaries;
    end
    x = [0;0;0;8;0;0]+[0;options.InitialTrackingError];
    lane = [];previousState = [];previousInput = [];
    count = options.SampleCount;
    states = nan(6,count+1);inputs=nan(2,count);seconds=nan(1,count);calls=zeros(1,count);
    branchSearch=cell(1,count);
    horizons=zeros(1,count);inherited=false(1,count);releases=false(1,count);terminalCommands=false(1,count);
    terminalOptimizations=false(1,count);replacements=false(1,count);approximate=false(1,count);
    verified=false(1,count);recursive=false(1,count);
    inheritedViolation=nan(1,count);terminalMargin=nan(1,count);
    clfResidual=nan(1,count);clfValue=nan(1,count);cbfRows=zeros(1,count);
    clfSlack=nan(1,count);clfSlackBound=nan(1,count);clfUnrelaxedResidual=nan(1,count);
    minimumSeparation=inf;minimumRoad=inf;minimumDomain=inf;maximumSlew=0;
    failure=[];executed=0;
    for sample = 1:count
        time = (sample-1)*h;
        frameTimer = tic;
        ego = localEgoMeasurement(x,time,previousInput,options.EgoErrorBound,stream,lane);
        ego.perception = struct('time',time,'range',options.ConfirmationRange,'completeWithinRange',true);
        target = localTargetMeasurement(truthTarget,time,options.TargetErrorBound,stream);
        if options.Scenario=="cruise",target=[];end
        try
            [command,~,problem,previousState] = collisionAvoidanceController(ego,target,road,cfg,previousState);
        catch exception
            seconds(sample)=toc(frameTimer);failure=exception;break;
        end
        seconds(sample)=toc(frameTimer);
        if isempty(lane)
            lane=problem.model.lane;projection=laneGeometry.project(x(1:2),lane);
            x=[projection.station;projection.lateralPosition;x(3)-projection.heading;x(4:6)];
        end
        states(:,sample)=x;
        metadata=problem.metadata;calls(sample)=metadata.solverCallCount;
        branchSearch{sample}=metadata.branchSearch;
        cbfRows(sample)=metadata.obstacleCbfRowCount;
        horizons(sample)=metadata.horizonSteps;inherited(sample)=metadata.inheritedFeasibleFamily;
        releases(sample)=metadata.confirmedRelease;terminalCommands(sample)=metadata.terminalActive;
        terminalOptimizations(sample)=metadata.terminalInvariantOptimization;
        replacements(sample)=metadata.freshProblemContainsWitness;
        approximate(sample)=metadata.approximateSolveCertified;
        verified(sample)=metadata.postSolveCertificationPerformed;
        recursive(sample)=metadata.recursiveFeasibilityGuaranteed;
        % Offline audits do not authorize or reject a command.
        if inherited(sample)
            oldDecision=[problem.carriedWitness.inputs(:);0];
            inheritedViolation(sample)=max(problem.program.physicalMatrix*oldDecision-problem.program.physicalBound);
        end
        cone=reshape(problem.program.terminalConePhysicalBound ...
            -problem.program.terminalCone.matrix*problem.decision(problem.program.layout.planIndex),3,[]);
        terminalMargin(sample)=min(cone(1,:)-vecnorm(cone(2:3,:),2,1));
        inputs(:,sample)=command.actuatorInput;
        if ~isempty(previousInput)
            rate=h*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
            maximumSlew=max([maximumSlew;abs(inputs(:,sample)-previousInput)-rate]);
        end
        generator=[metadata.executedContinuousGenerator;zeros(3,9)];
        before=x(2:6)-metadata.clfReferenceState(2:6);
        clfValue(sample)=before.'*metadata.clfMatrix*before;
        for fraction=linspace(0,1,11)
            value=expm(fraction*h*generator)*[x;command.actuatorInput;1];
            [position,heading]=laneGeometry.fromFrenet(value(1:6),lane);
            if ~isempty(target)
                targetState=localTargetTruth(truthTarget,time+fraction*h);
                separation=avoidanceSafetyGeometry.rectangleDistance(position,heading,targetState(1:2),targetState(7), ...
                    [cfg.vehicle.length/2;cfg.vehicle.width/2;truthTarget.halfLength;truthTarget.halfWidth]);
                minimumSeparation=min(minimumSeparation,separation-cfg.collision.clearanceMargin);
            end
            if options.UseRoadBoundaries
                support=cfg.vehicle.length/2*abs(sin(heading))+cfg.vehicle.width/2*abs(cos(heading));
                minimumRoad=min(minimumRoad,5-abs(position(2))-support-cfg.collision.clearanceMargin);
            end
            minimumDomain=min(minimumDomain,localDomainMargin(value(1:6),cfg));
        end
        x=value(1:6);states(:,sample+1)=x;executed=sample;
        after=x(2:6)-metadata.clfReferenceState(2:6);
        clfSlack(sample)=metadata.clfSlack;
        clfSlackBound(sample)=metadata.clfSlackDissipationBound;
        clfUnrelaxedResidual(sample)=after.'*metadata.clfMatrix*after ...
            -(1-metadata.clfDecayPerHold)*clfValue(sample)-metadata.clfDisturbanceBound;
        clfResidual(sample)=clfUnrelaxedResidual(sample)-clfSlackBound(sample);
        previousInput=command.actuatorInput;
        if options.FailAfterAdmission,cfg.solver.jointFunction=@localFailedSolve;end
    end
    attempted=executed+double(~isempty(failure));
    if ~options.UseRoadBoundaries,minimumRoad=NaN;end
    report=struct('scenario',options.Scenario,'configuration',originalCfg,'seed',options.Seed, ...
        'roadBoundariesEnabled',options.UseRoadBoundaries, ...
        'solverFailureInjected',options.FailAfterAdmission, ...
        'sampleCount',count,'executedHolds',executed,'completed',isempty(failure), ...
        'time',(0:executed)*h,'state',states(:,1:executed+1),'input',inputs(:,1:executed), ...
        'solverCallCount',calls(1:executed),'obstacleCbfRowCount',cbfRows(1:executed), ...
        'branchSearch',{branchSearch(1:executed)}, ...
        'horizonSteps',horizons(1:executed),'inheritedFeasibleFamily',inherited(1:executed), ...
        'confirmedRelease',releases(1:executed),'terminalCommands',terminalCommands(1:executed), ...
        'terminalInvariantOptimization',terminalOptimizations(1:executed), ...
        'verifiedReplacement',replacements(1:executed),'approximateSolveCertified',approximate(1:executed), ...
        'hardCertificateVerified',verified(1:executed), ...
        'inheritedWitnessViolation',inheritedViolation(1:executed),'terminalMembershipMargin',terminalMargin(1:executed), ...
        'clfValue',clfValue(1:executed),'clfDissipationResidual',clfResidual(1:executed), ...
        'clfSlack',clfSlack(1:executed),'clfSlackDissipationBound',clfSlackBound(1:executed), ...
        'clfUnrelaxedDissipationResidual',clfUnrelaxedResidual(1:executed), ...
        'minimumSampledSeparationMargin',minimumSeparation,'minimumSampledRoadMargin',minimumRoad, ...
        'minimumSampledModelDomainMargin',minimumDomain,'maximumSlewViolation',maximumSlew, ...
        'failureIdentifier',"",'failureMessage',"",'failureTime',NaN, ...
        'egoErrorBound',options.EgoErrorBound,'targetErrorBound',options.TargetErrorBound, ...
        'targetMotion',truthTarget,'recursiveFeasibilityGuaranteed',executed>0 && all(recursive(1:executed)), ...
        'scope',"Predictive finite encounter and road-terminal witness; failed optimization ends execution");
    report.passed=report.completed && minimumSeparation>=-1e-8 ...
        && (~options.UseRoadBoundaries || minimumRoad>=-1e-8) ...
        && minimumDomain>=-1e-8 && maximumSlew<=1e-8 && all(clfResidual(1:executed)<=1e-8);
    report.runtime=struct('frameSeconds',seconds(1:attempted),'deadlineSeconds',h, ...
        'searchBudgetSeconds',options.DeadlineSeconds,'deadlineMisses',nnz(seconds(1:attempted)>h), ...
        'maximumSeconds',max(seconds(1:attempted)),'medianSeconds',median(seconds(1:attempted)), ...
        'scope',"Input assembly and control against the actual hold period, excluding plant integration and offline diagnostics; offline search budget is reported separately");
    report.runtimeQualified=report.passed && report.runtime.deadlineMisses==0;
    if ~isempty(failure)
        report.failureIdentifier=string(failure.identifier);report.failureMessage=string(failure.message);
        report.failureTime=executed*h;
    end
    localSave(report,options);
    fprintf('%s: %d/%d holds; separation %.6g m; road %.6g m; max frame %.3f ms\n', ...
        options.Scenario,executed,count,minimumSeparation,minimumRoad,1000*report.runtime.maximumSeconds);
    if ~isempty(failure),rethrow(failure);end
end

function margin = localDomainMargin(state,cfg)
% Independent physical state-domain audit, separate from plan certification.
    lower = [-cfg.model.lateralDomainRadius;-cfg.model.headingDomainRadius; ...
        cfg.model.speedMinimum;-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
    upper = [cfg.model.lateralDomainRadius;cfg.model.headingDomainRadius; ...
        cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
    margin = min([state(2:6)-lower;upper-state(2:6)]);
end

function ego = localEgoMeasurement(x,time,heldInput,bound,stream,lane)
% Truth plus uniform noise inside the declared box, published with that box.
    if isempty(lane)
        position = x(1:2);
        heading = x(3);
    else
        [position,heading] = laneGeometry.fromFrenet(x,lane);
    end
    noise = bound.*(2*rand(stream,6,1)-1);
    ego = struct("position",position+noise(1:2),"yaw",heading+noise(3),"speed",x(4)+noise(4), ...
        "lateralVelocity",x(5)+noise(5),"yawRate",x(6)+noise(6),"stateTime",time, ...
        "controllerStateErrorBound",bound);
    if ~isempty(heldInput), ego.heldActuatorInput = heldInput; end
end

function target = localTargetMeasurement(truth,time,bound,stream)
    state = localTargetTruth(truth,time);
    noise = bound.*(2*rand(stream,8,1)-1);
    target = struct("trackId",1,"targetPositionInertial",state(1:2)+noise(1:2), ...
        "targetVelocityInertial",state(3:4)+noise(3:4),"targetAccelerationInertial",state(5:6)+noise(5:6), ...
        "targetHeadingInertial",state(7)+noise(7),"targetYawRate",state(8)+noise(8), ...
        "targetPositionInertialErrorBound",bound(1:2),"targetVelocityInertialErrorBound",bound(3:4), ...
        "targetAccelerationInertialErrorBound",bound(5:6),"targetYawErrorBound",bound(7), ...
        "targetYawRateErrorBound",bound(8), ...
        "predictionMotion",struct("kind","finite-sensing-motion-v1", ...
            "jerkBound",abs(truth.jerkAmplitude), ...
            "yawAccelerationBound",abs(truth.yawAccelerationAmplitude)));
end

function state = localTargetTruth(truth,time)
% Independent integrals of j(t)=J*cos(w*t), yawAcceleration(t)=H*cos(w*t).
    x = truth.center;
    w = truth.frequency;
    sine = sin(w*time);
    cosine = 1-cos(w*time);
    state = [(x(1:2)+x(3:4)*time+x(5:6)*time^2/2 ...
            +truth.jerkAmplitude*(time/w^2-sine/w^3)); ...
        x(3:4)+x(5:6)*time+truth.jerkAmplitude*cosine/w^2; ...
        x(5:6)+truth.jerkAmplitude*sine/w; ...
        x(7)+x(8)*time+truth.yawAccelerationAmplitude*cosine/w^2; ...
        x(8)+truth.yawAccelerationAmplitude*sine/w];
end

function result = localFailedSolve(~,~)
    result = struct("decision",[],"exitFlag",-999,"output",struct());
end

function localSave(report,options)
    if strlength(options.OutputDirectory)==0, return; end
    if ~isfolder(options.OutputDirectory), mkdir(options.OutputDirectory); end
    save(fullfile(options.OutputDirectory,options.Scenario+"-exact-state.mat"),"report");
    file = fopen(fullfile(options.OutputDirectory,options.Scenario+"-exact-state.json"),'w');
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(report,PrettyPrint=true));
end
