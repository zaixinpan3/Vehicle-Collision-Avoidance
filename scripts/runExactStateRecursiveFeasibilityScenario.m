function report = runExactStateRecursiveFeasibilityScenario(options)
%runExactStateRecursiveFeasibilityScenario Exact affine predictive-control experiment.
% Recursive feasibility is conditional on the declared model, sensing and
% encounter-admission contracts. Numerical failure still terminates the run.
% Geometry and CLF measurements below are offline experimental diagnostics;
% they do not accept, reject, repair, or select a controller command.
% Road boundaries are opt-in. State-domain margins are diagnostics only.
% Collision acceptance requires a strictly positive sampled body gap.
% Separation-margin fields report physical gap without an added clearance.
% SampleTime is shared by prediction nodes, executed input holds and updates.
% The target is an exact NRMM truth (nrmmTargetTruth): constant speed-rate
% TargetSpeedRate and path curvature TargetCurvature, measured with noise
% inside TargetErrorBound and declared under the nrmm-motion-v1 contract
% (nrmmTargetMeasurement).
    arguments
        options.Scenario (1,1) string {mustBeMember(options.Scenario,["stationary","oncoming","crossing","cruise"])} = "stationary"
        options.SampleCount (1,1) double {mustBeInteger,mustBePositive} = 240
        options.SampleTime (1,1) double {mustBeFinite,mustBePositive} = 0.05
        options.HorizonSeconds (1,1) double {mustBeFinite,mustBePositive} = 1.6
        options.ReferenceSpeed (1,1) double {mustBeFinite,mustBePositive} = 8
        options.FrictionCoefficient (2,1) double {mustBeFinite,mustBePositive} = [.85;.85]
        options.SteeringRateMaximum (1,1) double {mustBePositive} = Inf
        options.BrakingRatioRateMaximum (1,1) double {mustBePositive} = Inf
        options.AuditSubsteps (1,1) double {mustBeInteger,mustBePositive} = 10
        options.UseRoadBoundaries (1,1) logical = false
        options.RoadCoveragePolicy (1,1) string = "strict"
        options.RoadBoundaryParameterRange (2,1) double = [-100;2000]
        options.FailAfterAdmission (1,1) logical = false
        options.OutputDirectory (1,1) string = ""
        options.RethrowFailure (1,1) logical = true
        options.DeadlineSeconds (1,1) double {mustBePositive} = 0.05
        options.EgoErrorBound (6,1) double {mustBeNonnegative,mustBeFinite} = zeros(6,1)
        options.TargetErrorBound (8,1) double {mustBeNonnegative,mustBeFinite} = zeros(8,1)
        options.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 20260912
        options.TargetAccelerationMaximum (1,1) double {mustBePositive} = Inf
        options.TargetSpeedRate (1,1) double {mustBeFinite} = 0
        options.TargetCurvature (1,1) double {mustBeFinite} = 0
        options.TargetCurvatureMaximum (1,1) double {mustBeFinite,mustBePositive} = 0.05
        options.NrmmParameterBounds (1,1) logical = true
        options.InitialTrackingError (5,1) double {mustBeFinite} = zeros(5,1)
        options.ConfirmationRange (1,1) double {mustBeFinite,mustBePositive} = 16
        options.MinimumHorizonSteps (1,1) double {mustBeInteger,mustBePositive} = 1
        options.RoadCurvature (1,1) double {mustBeFinite} = 0
        options.SearchTimeLimitSeconds (1,1) double {mustBePositive} = 5
        options.Admission (1,1) struct = struct()
        options.FeedbackPrediction (1,1) struct = struct()
        options.LinearizationPolicy (1,1) string {mustBeMember(options.LinearizationPolicy,["cruise","trajectory"])} = "trajectory"
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"controller"),fullfile(root,"config"),fullfile(root,"estimator"),fullfile(root,"scripts"));
    cfg = collisionAvoidanceControllerConfig(struct("referenceSpeed",options.ReferenceSpeed, ...
        "controller",struct("sampleTime",options.SampleTime, ...
        "horizonSteps",ceil(options.HorizonSeconds/options.SampleTime), ...
        "minimumHorizonSteps",options.MinimumHorizonSteps), ...
        "model",struct("lateralDomainRadius",4,"linearizationPolicy",options.LinearizationPolicy, ...
        "frontWheelSteeringRateMaximum",options.SteeringRateMaximum, ...
        "brakingRatioRateMaximum",options.BrakingRatioRateMaximum), ...
        "tire",struct("frictionCoefficient",options.FrictionCoefficient),"admission",options.Admission, ...
        "feedbackPrediction",options.FeedbackPrediction, ...
        "solver",struct("frameDeadlineSeconds",options.DeadlineSeconds, ...
        "certificateSearchTimeLimit",options.SearchTimeLimitSeconds)));
    originalCfg = cfg;
    stream = RandStream("mt19937ar",Seed=options.Seed);
    h = cfg.controller.sampleTime;
    assert(abs(options.TargetCurvature)<=options.TargetCurvatureMaximum, ...
        "runExactStateRecursiveFeasibilityScenario:undeclaredCurvature", ...
        "The truth curvature must lie within the declared curvature maximum.");
    truthTarget = struct("center",[15;0;0;0;0;0;0;0],"radius",zeros(8,1), ...
        "accelerationMaximum",options.TargetAccelerationMaximum, ...
        "speedRate",options.TargetSpeedRate,"curvature",options.TargetCurvature, ...
        "curvatureMaximum",options.TargetCurvatureMaximum, ...
        "publishParameterBounds",options.NrmmParameterBounds, ...
        "halfLength",cfg.target.defaultLength/2,"halfWidth",cfg.target.defaultWidth/2);
    if options.Scenario=="oncoming",truthTarget.center=[60;0;-8;0;0;0;pi;0];end
    if options.Scenario=="crossing",truthTarget.center=[15;-4;0;32;0;0;pi/2;0];end
    road = struct("centerline",[-100,0;2000,0]);
    [cruiseState,cruiseInput] = ltvBicycleModel.cruiseEquilibrium(options.RoadCurvature,cfg);
    if options.RoadCurvature~=0
        assert(~options.UseRoadBoundaries, ...
            'runExactStateRecursiveFeasibilityScenario:unsupportedCurvedBoundaries', ...
            'This circular-arc experiment does not construct road boundaries.');
        curve = struct('origin',[0;0],'heading',0,'curvature',options.RoadCurvature, ...
            'length',min(cfg.referenceSpeed*options.SampleCount*h+100,1.9*pi/abs(options.RoadCurvature)));
        road = struct('referenceCurve',curve, ...
            'centerline',laneGeometry.referencePose(linspace(0,curve.length,201),0,curve).');
        [point,heading] = laneGeometry.referencePose(15,0,curve);
        truthTarget.center = [point;zeros(4,1);heading;0];
        if options.Scenario=="oncoming"
            [point,heading] = laneGeometry.referencePose(30,0,curve);
            tangent = [cos(heading);sin(heading)];
            truthTarget.center = [point+30*tangent;-8*tangent;zeros(2,1);heading+pi;0];
        elseif options.Scenario=="crossing"
            normal = [-sin(heading);cos(heading)];
            truthTarget.center = [point-7.5*normal;4*normal;zeros(2,1);heading+pi/2;0];
        end
    end
    if options.UseRoadBoundaries
        boundary = struct("origin",zeros(2,1),"longitudinalDirection",[1;0], ...
            "lateralDirection",[0;1],"coefficients",[0;0;-5], ...
            "parameterRange",options.RoadBoundaryParameterRange,"safeSideSign",1, ...
            "coveragePolicy",options.RoadCoveragePolicy);
        boundaries = [boundary;boundary];boundaries(2).coefficients(3)=5;boundaries(2).safeSideSign=-1;
        road.boundaries = boundaries;
        road.lateralClearance = [5;5];
    end
    x = [0;0;0;cfg.referenceSpeed;0;0]+[0;options.InitialTrackingError];
    lane = [];previousState = [];previousInput = [];
    if options.RoadCurvature~=0
        x = cruiseState+[0;options.InitialTrackingError];
        lane = road;
    end
    count = options.SampleCount;
    states = nan(6,count+1);inputs=nan(2,count);seconds=nan(1,count);calls=zeros(1,count);
    states(:,1)=x;
    supportGeometry=cell(1,count);admissionSearch=cell(1,count);runtimeBreakdown=cell(1,count);trajectoryCalls=zeros(1,count);restorationCalls=zeros(1,count);
    horizons=zeros(1,count);inherited=false(1,count);releases=false(1,count);terminalCommands=false(1,count);
    terminalOptimizations=false(1,count);replacements=false(1,count);approximate=false(1,count);
    recursive=false(1,count);
    inheritedViolation=nan(1,count);terminalMargin=nan(1,count);
    jointViolation=nan(1,count);shiftedJointViolation=nan(1,count);minimumNodeSeparation=inf;
    clfResidual=nan(1,count);clfValue=nan(1,count);cbfRows=zeros(1,count);
    clfSlack=nan(1,count);clfSlackBound=nan(1,count);clfUnrelaxedResidual=nan(1,count);
    minimumSeparation=inf;minimumRoad=inf;minimumDomain=inf;maximumSlew=0;
    failure=[];executed=0;
    for sample = 1:count
        time = (sample-1)*h;
        frameTimer = tic;
        ego = localEgoMeasurement(x,time,previousInput,options.EgoErrorBound,stream,lane);
        ego.perception = struct('time',time,'range',options.ConfirmationRange,'completeWithinRange',true);
        target = nrmmTargetMeasurement(truthTarget,time,options.TargetErrorBound,stream);
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
        supportGeometry{sample}=metadata.supportGeometry;
        admissionSearch{sample}=metadata.admissionSearch;runtimeBreakdown{sample}=metadata.runtime;
        trajectoryCalls(sample)=metadata.trajectorySolverCallCount;
        restorationCalls(sample)=metadata.restorationSolverCallCount;
        cbfRows(sample)=metadata.obstacleCbfRowCount;
        horizons(sample)=metadata.horizonSteps;inherited(sample)=metadata.inheritedFeasibleFamily;
        releases(sample)=metadata.confirmedRelease;terminalCommands(sample)=metadata.terminalActive;
        terminalOptimizations(sample)=metadata.terminalInvariantOptimization;
        replacements(sample)=metadata.freshProblemContainsWitness;
        approximate(sample)=metadata.approximateSolveAccepted;
        recursive(sample)=metadata.recursiveFeasibilityGuaranteed;
        if isfield(problem.program,'jointCertificate')
            jointViolation(sample)=max(avoidanceSafetyGeometry.jointResidual( ...
                problem.program,problem.decision,problem.program.jointCertificate.angles));
        end
        % Offline audits do not authorize or reject a command.
        if inherited(sample)
            oldDecision=[problem.carriedWitness.inputs(:);0];
            inheritedViolation(sample)=max(problem.program.physicalMatrix*oldDecision-problem.program.physicalBound);
            if isfield(problem.program,'jointCertificate')
                shiftedJointViolation(sample)=max(avoidanceSafetyGeometry.jointResidual( ...
                    problem.program,oldDecision,metadata.admissionSearch.initialCertificateAngles));
            end
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
        for fraction=linspace(0,1,options.AuditSubsteps+1)
            value=expm(fraction*h*generator)*[x;command.actuatorInput;1];
            [position,heading]=laneGeometry.fromFrenet(value(1:6),lane);
            if ~isempty(target)
                targetState=nrmmTargetTruth(truthTarget,time+fraction*h);
                separation=avoidanceSafetyGeometry.rectangleDistance(position,heading,targetState(1:2),targetState(7), ...
                    [cfg.vehicle.length/2;cfg.vehicle.width/2;truthTarget.halfLength;truthTarget.halfWidth]);
                minimumSeparation=min(minimumSeparation,separation);
                if fraction==0 || fraction==1
                    minimumNodeSeparation=min(minimumNodeSeparation,separation);
                end
            end
            if options.UseRoadBoundaries
                support=cfg.vehicle.length/2*abs(sin(heading))+cfg.vehicle.width/2*abs(cos(heading));
                minimumRoad=min(minimumRoad,5-abs(position(2))-support);
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
        'auditSubsteps',options.AuditSubsteps, ...
        'roadCurvature',options.RoadCurvature,'road',road, ...
        'cruiseState',cruiseState,'cruiseInput',cruiseInput, ...
        'trackingError',states(2:6,1:executed+1)-cruiseState(2:6), ...
        'roadBoundariesEnabled',options.UseRoadBoundaries,'stateAndSlipBoundsEnforced',false, ...
        'solverFailureInjected',options.FailAfterAdmission, ...
        'sampleCount',count,'executedHolds',executed,'completed',isempty(failure), ...
        'time',(0:executed)*h,'state',states(:,1:executed+1),'input',inputs(:,1:executed), ...
        'solverCallCount',calls(1:executed),'obstacleCbfRowCount',cbfRows(1:executed), ...
        'supportGeometry',{supportGeometry(1:executed)}, ...
        'admissionSearch',{admissionSearch(1:executed)},'runtimeBreakdown',{runtimeBreakdown(1:executed)}, ...
        'trajectorySolverCallCount',trajectoryCalls(1:executed),'restorationSolverCallCount',restorationCalls(1:executed), ...
        'horizonSteps',horizons(1:executed),'inheritedFeasibleFamily',inherited(1:executed), ...
        'confirmedRelease',releases(1:executed),'terminalCommands',terminalCommands(1:executed), ...
        'terminalInvariantOptimization',terminalOptimizations(1:executed), ...
        'verifiedReplacement',replacements(1:executed),'approximateSolveAccepted',approximate(1:executed), ...
        'inheritedWitnessViolation',inheritedViolation(1:executed),'terminalMembershipMargin',terminalMargin(1:executed), ...
        'jointSeparationViolation',jointViolation(1:executed),'shiftedJointSeparationViolation',shiftedJointViolation(1:executed), ...
        'minimumNodeSeparationMargin',minimumNodeSeparation, ...
        'clfValue',clfValue(1:executed),'clfDissipationResidual',clfResidual(1:executed), ...
        'clfSlack',clfSlack(1:executed),'clfSlackDissipationBound',clfSlackBound(1:executed), ...
        'clfUnrelaxedDissipationResidual',clfUnrelaxedResidual(1:executed), ...
        'minimumSampledSeparationMargin',minimumSeparation,'minimumSampledRoadMargin',minimumRoad, ...
        'minimumSampledModelDomainMargin',minimumDomain,'maximumSlewViolation',maximumSlew, ...
        'failureIdentifier',"",'failureMessage',"",'failureTime',NaN, ...
        'egoErrorBound',options.EgoErrorBound,'targetErrorBound',options.TargetErrorBound, ...
        'targetMotion',truthTarget,'recursiveFeasibilityGuaranteed',executed>0 && all(recursive(1:executed)), ...
        'scope',"Predictive finite encounter and road-terminal witness; failed optimization ends execution");
    report.minimumSampledBodyGap=minimumSeparation;
    report.minimumNodeBodyGap=minimumNodeSeparation;
    report.sampledCollisionFree=report.minimumSampledBodyGap>0;
    report.collisionAcceptanceCriterion="Sampled signed body gap strictly greater than zero";
    report.passed=report.completed && report.sampledCollisionFree ...
        && (~options.UseRoadBoundaries || minimumRoad>=-1e-8) ...
        && maximumSlew<=1e-8 && all(clfResidual(1:executed)<=1e-8);
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
    fprintf('%s: %d/%d holds; body gap %.6g m; road %.6g m; max frame %.3f ms\n', ...
        options.Scenario,executed,count,report.minimumSampledBodyGap, ...
        minimumRoad,1000*report.runtime.maximumSeconds);
    if ~isempty(failure) && options.RethrowFailure,rethrow(failure);end
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
