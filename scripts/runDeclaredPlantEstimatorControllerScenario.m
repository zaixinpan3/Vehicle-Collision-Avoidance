function report = runDeclaredPlantEstimatorControllerScenario(options)
%runDeclaredPlantEstimatorControllerScenario Actual NRMM output with the declared ego plant.
% Sensors and truth audits use the true state; control uses only NRMM output
% and applied-input memory. Target visibility gates its publication. The
% accepted first-hold affine generator is integrated independently by expm.
% This diagnostic measures overruns while waiting for computation; it does
% not simulate delayed actuation or establish nonlinear-vehicle safety.
% UseEstimator=false supplies exact states with the same physical range gate
% for a controller-only comparison; it does not run or reset the observer.
    arguments
        options.SampleCount (1,1) double {mustBeInteger,mustBePositive} = 300
        options.TargetInitialDistance (1,1) double {mustBePositive} = 100
        options.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 20260913
        options.OutputDirectory (1,1) string = ""
        options.UseEstimator (1,1) logical = true
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'estimator'),fullfile(root,'solver','nrmm'));
    cfg = collisionAvoidanceControllerConfig(struct('referenceSpeed',10, ...
        'controller',struct('sampleTime',0.1,'horizonSteps',16), ...
        'model',struct('lateralDomainRadius',4), ...
        'solver',struct('certificateSearchTimeLimit',3,'frameDeadlineSeconds',0.1)));
    estimator = estimatorControllerIntegrationConfig();
    estimator.randomSeed = options.Seed;
    estimator.vehicle.targetSpeed = 10;
    estimator.initialization.targetSpeedPrior = 10;
    estimator.observer.ego.yaw.rearAxleDistance = cfg.vehicle.lr;
    estimator.observer.runtime.integrationStepMaximum = estimator.observer.runtime.samplePeriod;
    targetFunction = @(t,~) localTarget(t,options.TargetInitialDistance);
    truth = [0;0;0;10;0;0];
    if options.UseEstimator
        [context,initialization] = nrmmEstimatorControllerAdapter('initialize',estimator,localTruth(truth),targetFunction);
    else
        initialization = struct('scope',"Exact-state controller comparison; no observer initialization");
    end
    boundary = struct('origin',[0;0],'longitudinalDirection',[1;0],'lateralDirection',[0;1], ...
        'coefficients',[0;0;-5],'parameterRange',[-100;2000],'safeSideSign',1);
    boundaries = [boundary;boundary];boundaries(2).coefficients(3)=5;boundaries(2).safeSideSign=-1;
    road = struct('centerline',[-100,0;2000,0],'boundaries',boundaries);
    count = options.SampleCount+1;
    time = (0:options.SampleCount)*cfg.controller.sampleTime;
    states = nan(6,count);inputs = nan(2,count);frameSeconds = nan(1,count);
    observerSeconds = nan(1,count);controllerSeconds = nan(1,count);
    published = false(1,count);certified = false(1,count);audits = cell(1,count);metadata = cell(1,count);
    egoEstimates = cell(1,count);targetEstimates = cell(1,count);
    certificate = [];command = [];
    failure = struct('identifier',"",'message',"",'time',NaN);
    failureException = [];
    minimumRoadMargin = inf;minimumSeparationMargin = inf;executedHolds = 0;
    for k = 1:count
        states(:,k) = truth;
        frameTimer = tic;
        phase = tic;
        if options.UseEstimator
            [context,ego,targets,~,audit] = nrmmEstimatorControllerAdapter('sample',context,time(k), ...
                localTruth(truth),targetFunction(time(k),[]));
        else
            ego = localTruth(truth);
            ego.stateTime = time(k);
            ego.perception = struct('time',time(k),'range',estimator.sensor.radar.rangeMaximum, ...
                'completeWithinRange',true);
            targets = targetFunction(time(k),[]);
            targets.trackId = 1;
            targets.targetHeadingInertial = targets.targetYawInertial;
            if norm(targets.targetPositionInertial-truth(1:2))>estimator.sensor.radar.rangeMaximum
                targets = [];
            end
            audit = struct('truthEnclosure',struct());
        end
        observerSeconds(k) = toc(phase);
        if ~isempty(command), ego.heldActuatorInput = command.actuatorInput; end
        egoEstimates{k}=ego;targetEstimates{k}=targets;audits{k}=audit.truthEnclosure;
        published(k) = ~isempty(targets);
        phase = tic;
        try
            [command,~,problem,certificate] = collisionAvoidanceController(ego,targets,road,cfg,certificate);
        catch exception
            controllerSeconds(k) = toc(phase);frameSeconds(k) = toc(frameTimer);
            failure = struct('identifier',string(exception.identifier),'message',string(exception.message),'time',time(k));
            failureException = exception;
            break;
        end
        controllerSeconds(k)=toc(phase);frameSeconds(k)=toc(frameTimer);
        inputs(:,k)=command.actuatorInput;metadata{k}=problem.metadata;certified(k)=problem.metadata.planCertified;
        lane=problem.model.lane;
        if k==count,break;end
        projection=laneGeometry.project(truth(1:2),lane);
        x=[projection.station;projection.lateralPosition;atan2(sin(truth(3)-projection.heading),cos(truth(3)-projection.heading));truth(4:6)];
        generator=[problem.metadata.executedContinuousGenerator;zeros(3,9)];
        for fraction=linspace(0,1,11)
            elapsed=fraction*cfg.controller.sampleTime;
            flowed=expm(elapsed*generator)*[x;command.actuatorInput;1];
            [position,heading]=laneGeometry.fromFrenet(flowed(1:6),lane);
            lateralSupport=cfg.vehicle.length/2*abs(sin(heading))+cfg.vehicle.width/2*abs(cos(heading));
            minimumRoadMargin=min(minimumRoadMargin,5-abs(position(2))-lateralSupport-cfg.collision.clearanceMargin);
            target=targetFunction(time(k)+elapsed,[]);
            separation=avoidanceSafetyGeometry.rectangleDistance(position,heading,target.targetPositionInertial,pi, ...
                [cfg.vehicle.length/2;cfg.vehicle.width/2;target.targetLength/2;target.targetWidth/2]);
            minimumSeparationMargin=min(minimumSeparationMargin,separation-cfg.collision.clearanceMargin);
        end
        truth=[position;heading;flowed(4:6)];executedHolds=executedHolds+1;
        if mod(k,25)==0
            fprintf('Declared plant (estimator=%d): %d holds, speed %.6f, target %d, frame %.3f s\n', ...
                options.UseEstimator,k,truth(4),published(k),frameSeconds(k));
        end
    end
    kept=1:k;
    report=struct('completed',strlength(failure.identifier)==0,'failure',failure,'executedHolds',executedHolds, ...
        'time',time(kept),'state',states(:,kept),'input',inputs(:,kept),'published',published(kept), ...
        'certified',certified(kept),'frameSeconds',frameSeconds(kept),'observerSeconds',observerSeconds(kept), ...
        'controllerSeconds',controllerSeconds(kept),'audit',{audits(kept)},'metadata',{metadata(kept)}, ...
        'egoEstimate',{egoEstimates(kept)},'targetEstimate',{targetEstimates(kept)}, ...
        'minimumRoadMargin',minimumRoadMargin,'minimumSeparationMargin',minimumSeparationMargin, ...
        'configuration',cfg,'estimatorConfiguration',estimator,'options',options, ...
        'scope',"Actual NRMM bounds, declared affine ego plant, fixed road; computation delay measured but not applied");
    if ~options.UseEstimator
        report.scope = "Exact states with a current 30 m range gate; declared affine ego plant; computation delay measured but not applied";
    end
    if strlength(options.OutputDirectory)>0
        if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
        save(fullfile(options.OutputDirectory,'joint-declared-plant.mat'),'report','initialization','-v7.3');
    end
    if ~isempty(failureException),rethrow(failureException);end
end

function state = localTruth(x)
    state=struct('position',x(1:2),'yawAngle',x(3),'longitudinalVelocity',x(4),'lateralVelocity',x(5),'yawRate',x(6));
end

function target = localTarget(time,distance)
    target=struct('targetPositionInertial',[distance-10*time;0.8],'targetVelocityInertial',[-10;0], ...
        'targetAccelerationInertial',[0;0],'targetYawInertial',pi,'targetYawRate',0,'targetLength',4.8,'targetWidth',1.9);
end
