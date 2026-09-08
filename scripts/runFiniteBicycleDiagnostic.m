function result = runFiniteBicycleDiagnostic(cfg,duration,options)
%runFiniteBicycleDiagnostic Isolate planning with a nonlinear bicycle plant.
% Exact observations isolate planning; EstimatorConfiguration enables the
% bounded-noise NRMM sensor adapter. A target follows the straight/arc road
% with constant speed and curvature and is observed only within 30 m.
% This is a diagnostic model, not high-fidelity validation. A failed current
% solve ends the experiment before advancing the plant.
    arguments
        cfg (1,1) struct
        duration (1,1) double {mustBePositive} = 10
        options.EstimatorConfiguration (1,1) struct = struct()
        options.PrepareController (1,1) logical = true
        options.DeadlineSeconds (1,1) double {mustBePositive} = 0.1
        options.EnforceRuntimeDeadline (1,1) logical = false
        options.RoadCurvature (1,1) double {mustBeFinite} = 0
        options.InitialFrenetError (5,1) double {mustBeFinite} = zeros(5,1)
        options.IncludeTarget (1,1) logical = true
        options.EnforceModelResidual (1,1) logical = false
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    for folder = ["controller","config","estimator"],addpath(fullfile(root,folder));end
    threadCount = maxNumCompThreads(1);
    restoreThreads = onCleanup(@() maxNumCompThreads(threadCount));
    cfg = collisionAvoidanceControllerConfig(cfg);
    h = cfg.controller.sampleTime;
    if options.EnforceRuntimeDeadline && options.DeadlineSeconds>h
        error("runFiniteBicycleDiagnostic:deadlineExceedsPeriod", ...
            "The complete-frame deadline cannot exceed the control update period.");
    end
    steps = round(duration/h);
    curvature = options.RoadCurvature;
    baseCurve = struct("origin",[0;0],"heading",0,"curvature",curvature,"length",1);
    curve = baseCurve;
    [curve.origin,curve.heading] = laneGeometry.referencePose(-20,0,baseCurve);
    curve.length = max(200,20+duration*cfg.model.speedMaximum+2*cfg.referenceSpeed*h*cfg.controller.horizonSteps);
    curve = laneGeometry.validateReferenceCurve(curve);
    [trim,trimInput] = ltvBicycleModel.cruiseEquilibrium(curvature,cfg);
    initial = trim+[20;options.InitialFrenetError];
    [position,heading] = laneGeometry.referencePose(initial(1),initial(2),curve);
    state = [position;heading+initial(3);initial(4:6)];
    states = state.';times = 0;inputs = zeros(0,2);metadata = cell(0,1);
    traceTime = 0;traceState = state.';
    failure = struct("occurred",false,"time",NaN,"identifier","","message","");
    stored = [];
    road = [-200,0;3000,0];
    if curvature~=0
        road = struct("centerline",laneGeometry.referencePose(linspace(0,curve.length,301),0,curve).', ...
            "referenceCurve",curve);
    end
    estimated = ~isempty(fieldnames(options.EstimatorConfiguration));
    if estimated && ~options.IncludeTarget
        error("runFiniteBicycleDiagnostic:unsupportedTargetFreeEstimator", ...
            "Target-free diagnostics currently require exact observations.");
    end
    residualPeak = zeros(6,1);
    residualViolation = 0;
    estimates = cell(steps,1);targetEstimates = cell(steps,1);audits = cell(steps,1);
    frameSeconds = nan(steps,1);estimatorSeconds = zeros(steps,1);controllerSeconds = nan(steps,1);
    preparation = struct("performed",false,"elapsedSeconds",0);
    committedInput = zeros(0,1);
    if cfg.controller.inputDelaySteps>0
        committedInput = trimInput;
    end
    computedCommand = cell(steps,1);
    if estimated
        estimatorCfg = options.EstimatorConfiguration;
        estimatorCfg.observer.ego.yaw.rearAxleDistance = cfg.vehicle.lr;
        tire = modifiedFialaTire.parameters(cfg);
        estimatorCfg.observer.ego.domain.yawAccelerationMaximum = ...
            dot([cfg.vehicle.lf;cfg.vehicle.lr],tire.longitudinalForceScale)/cfg.vehicle.Iz;
        estimatorContext = nrmmEstimatorControllerAdapter("initialize",estimatorCfg,localEgoTruth(state), ...
            @(time,ego)localTargetTruth(time,ego,curve));
    end
    if options.PrepareController
        initialEgo = struct("position",state(1:2),"yaw",state(3),"speed",state(4), ...
            "lateralVelocity",state(5),"yawRate",state(6),"stateTime",0);
        initialEgo.heldActuatorInput = committedInput;
        initialEgo.committedActuatorInput = committedInput;
        preparation = prepareCollisionAvoidancePipeline(initialEgo,road,cfg,options.EstimatorConfiguration);
    end
    for index = 1:steps
        frameTimer = tic;
        time = (index-1)*h;
        ego = struct("position",state(1:2),"yaw",state(3),"speed",state(4), ...
            "lateralVelocity",state(5),"yawRate",state(6),"stateTime",time, ...
            "perception",struct("time",time,"range",30,"completeWithinRange",true));
        if index>1, ego.heldActuatorInput = inputs(end,:).';end
        target = localTargetTruth(time,[],curve);
        if ~options.IncludeTarget || norm(target.targetPositionInertial-state(1:2))>30,target = [];end
        if estimated
            estimatorTimer = tic;
            [estimatorContext,ego,target,~,audit] = nrmmEstimatorControllerAdapter( ...
                "sample",estimatorContext,time,localEgoTruth(state),localTargetTruth(time,[],curve));
            estimatorSeconds(index) = toc(estimatorTimer);
            audits{index} = audit;
            if index>1,ego.heldActuatorInput = inputs(end,:).';end
        end
        estimates{index} = ego;targetEstimates{index} = target;
        if index==1 && curvature~=0,ego.heldActuatorInput = trimInput;end
        if cfg.controller.inputDelaySteps>0
            ego.committedActuatorInput = committedInput;
            if index==1,ego.heldActuatorInput = committedInput;end
            estimates{index} = ego;
        end
        controllerTimer = tic;
        lastAttemptMetadata = [];
        try
            [command,~,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,stored);
            lastAttemptMetadata = problem.metadata;
            computedCommand{index} = command;
            controllerSeconds(index) = toc(controllerTimer);
            frameSeconds(index) = toc(frameTimer);
        catch exception
            controllerSeconds(index) = toc(controllerTimer);
            frameSeconds(index) = toc(frameTimer);
            failure = struct("occurred",true,"time",time,"identifier",string(exception.identifier), ...
                "message",string(exception.message));
            break;
        end
        if options.EnforceRuntimeDeadline && frameSeconds(index)>options.DeadlineSeconds
            failure = struct("occurred",true,"time",time, ...
                "identifier","collisionAvoidanceController:runtimeDeadlineExceeded", ...
                "message","The complete online frame exceeded its deadline; no command was applied.");
            break;
        end
        input = command.actuatorInput;
        if cfg.controller.inputDelaySteps>0
            input = committedInput;
            committedInput = command.actuatorInput;
        end
        [localTime,trajectory] = ode45(@(~,x) localFlow(x,input,cfg),[0,h],state, ...
            odeset(RelTol=1e-9,AbsTol=1e-11));
        traceTime = [traceTime;time+localTime(2:end)]; %#ok<AGROW>
        traceState = [traceState;trajectory(2:end,:)]; %#ok<AGROW>
        state = trajectory(end,:).';
        states(end+1,:) = state.';times(end+1,1) = time+h; %#ok<AGROW>
        inputs(end+1,:) = input.';metadata{end+1,1} = problem.metadata; %#ok<AGROW>
        [peak,violation] = localResidualAudit(trajectory,input,problem,cfg);
        residualPeak = max(residualPeak,peak);
        residualViolation = max(residualViolation,violation);
        if options.EnforceModelResidual && violation>1e-7
            failure = struct("occurred",true,"time",time+h, ...
                "identifier","runFiniteBicycleDiagnostic:modelResidualExceeded", ...
                "message","The nonlinear plant exceeded its declared residual allowance at a trace sample.");
            break;
        end
    end
    result = struct("state",states,"time",times,"input",inputs,"metadata",{metadata}, ...
        "failure",failure,"configuration",cfg,"lastCertificate",stored, ...
        "lastEgo",ego,"lastTarget",target,"stateOrder",["x","y","yaw","vx","vy","r"], ...
        "plant","nonlinear modified-Fiala bicycle with rotated front forces and passive road load", ...
        "observations","exact current states; target only inside 30 m");
    result.estimatorEnabled = estimated;
    result.lastAttemptMetadata = lastAttemptMetadata;
    result.computedCommand = computedCommand(1:index);
    result.runtime = struct("frameSeconds",frameSeconds(1:index), ...
        "estimatorSeconds",estimatorSeconds(1:index),"controllerSeconds",controllerSeconds(1:index), ...
        "deadlineSeconds",options.DeadlineSeconds,"deadlineMet",all(frameSeconds(1:index)<=options.DeadlineSeconds), ...
        "enforced",options.EnforceRuntimeDeadline, ...
        "controlPeriodSeconds",h,"inputDelaySeconds",cfg.controller.inputDelaySteps*h, ...
        "commandReadyTime",(0:index-1).'*h+frameSeconds(1:index), ...
        "scheduledActuationTime",((0:index-1).'+cfg.controller.inputDelaySteps)*h, ...
        "appliedSourceFrame",max(0,(1:size(inputs,1)).'-cfg.controller.inputDelaySteps), ...
        "preparation",preparation,"scope","Online synthetic sensors, observer, bounds, input assembly and controller; plant and offline preparation excluded");
    result.egoEstimate = estimates(1:index);
    result.targetEstimate = targetEstimates(1:index);
    result.measurementAudit = audits(1:index);
    result.plantTrace = struct("time",traceTime,"state",traceState);
    [targetPosition,targetHeading] = laneGeometry.referencePose(120-10*traceTime.',0.8,curve);
    separation = rectangleSeparationMargin(traceState(:,1:2),traceState(:,3), ...
        targetPosition.',targetHeading.'+pi,cfg.vehicle.length,cfg.vehicle.width,5,2);
    if ~options.IncludeTarget,separation = inf(size(separation));end
    result.minimumSampledSeparationMargin = min(separation);
    result.collisionFreeAtTraceSamples = all(separation>0);
    projection = laneGeometry.projectReferenceCurve(states(:,1:2).',curve);
    result.frenetState = [projection.station.',projection.lateralPosition.', ...
        atan2(sin(states(:,3)-projection.heading.'),cos(states(:,3)-projection.heading.')),states(:,4:6)];
    result.referenceState = trim;
    result.cruiseError = result.frenetState(:,2:6)-trim(2:6).';
    result.roadCurvature = curvature;
    result.targetEnabled = options.IncludeTarget;
    result.maximumSampledModelResidual = residualPeak;
    result.sampledModelResidualViolation = residualViolation;
    result.modelResidualAuditEnforced = options.EnforceModelResidual;
    if estimated
        result.observations = "NRMM estimates from the same bounded-noise sensor adapter as the physical scenario";
        result.estimatorConfiguration = estimatorCfg;
    end
end

function [peak,violation] = localResidualAudit(trajectory,input,problem,cfg)
% Independent ODE trace audit; this is sampled evidence, not a global bound.
    peak = zeros(6,1);
    a = problem.prediction.continuousA(:,:,1);
    b = problem.prediction.continuousB(:,:,1);
    c = problem.prediction.continuousC(:,1);
    lane = problem.model.lane;
    for index = 1:size(trajectory,1)
        state = trajectory(index,:).';
        projection = laneGeometry.project(state(1:2),lane);
        heading = atan2(sin(state(3)-projection.heading),cos(state(3)-projection.heading));
        frenet = [projection.station;projection.lateralPosition;heading;state(4:6)];
        curvature = laneGeometry.curvature(projection.station,lane);
        flow = localFlow(state,input,cfg);
        stationRate = (state(4)*cos(heading)-state(5)*sin(heading))/(1-curvature*frenet(2));
        flow(1:3) = [stationRate;state(4)*sin(heading)+state(5)*cos(heading);state(6)-curvature*stationRate];
        peak = max(peak,abs(flow-a*frenet-b*input-c));
    end
    violation = max(peak-problem.prediction.modelErrorRateBound(:,1));
end

function ego = localEgoTruth(state)
    ego = struct("position",state(1:2),"yawAngle",state(3), ...
        "longitudinalVelocity",state(4),"lateralVelocity",state(5),"yawRate",state(6));
end

function target = localTargetTruth(time,~,curve)
    [position,heading] = laneGeometry.referencePose(120-10*time,0.8,curve);
    speed = 10*(1-0.8*curve.curvature);
    yawRate = -10*curve.curvature;
    velocity = -speed*[cos(heading);sin(heading)];
    target = struct("trackId","diagnostic-target","targetPositionInertial",position, ...
        "targetVelocityInertial",velocity,"targetAccelerationInertial",yawRate*[-velocity(2);velocity(1)], ...
        "targetHeadingInertial",heading+pi,"targetYawRate",yawRate,"targetLength",5,"targetWidth",2, ...
        "predictionMotion",struct("kind","finite-sensing-motion-v1", ...
            "jerkBound",speed*yawRate^2*ones(2,1),"yawAccelerationBound",0,"scalarAccelerationMaximum",0));
end

function derivative = localFlow(state,input,cfg)
    vx = state(4);vy = state(5);r = state(6);delta = input(1);
    parameters = modifiedFialaTire.parameters(cfg);
    speed = max(vx,cfg.model.scheduleSpeedFloor);
    slip = [atan2(vy+cfg.vehicle.lf*r,speed)-delta;atan2(vy-cfg.vehicle.lr*r,speed)];
    fy = modifiedFialaTire.evaluate(slip,input(2),cfg);
    fx = parameters.longitudinalForceScale*input(2);
    frontX = fx(1)*cos(delta)-fy(1)*sin(delta);
    frontY = fx(1)*sin(delta)+fy(1)*cos(delta);
    derivative = [vx*cos(state(3))-vy*sin(state(3));vx*sin(state(3))+vy*cos(state(3));r; ...
        (frontX+fx(2)-longitudinalRoadLoad(vx,cfg))/cfg.vehicle.m+vy*r; ...
        (frontY+fy(2))/cfg.vehicle.m-vx*r; ...
        (cfg.vehicle.lf*frontY-cfg.vehicle.lr*fy(2))/cfg.vehicle.Iz];
end
