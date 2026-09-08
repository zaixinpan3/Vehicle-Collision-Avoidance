function result = runFiniteBicycleDiagnostic(cfg,duration,options)
%runFiniteBicycleDiagnostic Isolate planning with a nonlinear bicycle plant.
% Exact observations isolate planning; EstimatorConfiguration enables the
% bounded-noise NRMM sensor adapter. Both use a finite-range straight target.
% This is a diagnostic model, not high-fidelity validation. A failed current
% solve ends the experiment before advancing the plant.
    arguments
        cfg (1,1) struct
        duration (1,1) double {mustBePositive} = 10
        options.EstimatorConfiguration (1,1) struct = struct()
        options.PrepareController (1,1) logical = true
        options.DeadlineSeconds (1,1) double {mustBePositive} = 0.1
        options.EnforceRuntimeDeadline (1,1) logical = false
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    for folder = ["controller","config","estimator"],addpath(fullfile(root,folder));end
    threadCount = maxNumCompThreads(1);
    restoreThreads = onCleanup(@() maxNumCompThreads(threadCount));
    cfg = collisionAvoidanceControllerConfig(cfg);
    h = cfg.controller.sampleTime;
    steps = round(duration/h);
    state = [0;0;0;cfg.referenceSpeed;0;0];
    states = state.';times = 0;inputs = zeros(0,2);metadata = cell(0,1);
    traceTime = 0;traceState = state.';
    failure = struct("occurred",false,"time",NaN,"identifier","","message","");
    stored = [];
    road = [-200,0;3000,0];
    estimated = ~isempty(fieldnames(options.EstimatorConfiguration));
    estimates = cell(steps,1);targetEstimates = cell(steps,1);audits = cell(steps,1);
    frameSeconds = nan(steps,1);estimatorSeconds = zeros(steps,1);controllerSeconds = nan(steps,1);
    preparation = struct("performed",false,"elapsedSeconds",0);
    if estimated
        estimatorCfg = options.EstimatorConfiguration;
        estimatorCfg.observer.ego.yaw.rearAxleDistance = cfg.vehicle.lr;
        tire = modifiedFialaTire.parameters(cfg);
        estimatorCfg.observer.ego.domain.yawAccelerationMaximum = ...
            dot([cfg.vehicle.lf;cfg.vehicle.lr],tire.longitudinalForceScale)/cfg.vehicle.Iz;
        estimatorContext = nrmmEstimatorControllerAdapter("initialize",estimatorCfg,localEgoTruth(state),@localTargetTruth);
    end
    if options.PrepareController
        initialEgo = struct("position",state(1:2),"yaw",state(3),"speed",state(4), ...
            "lateralVelocity",state(5),"yawRate",state(6),"stateTime",0);
        preparation = prepareCollisionAvoidancePipeline(initialEgo,road,cfg,options.EstimatorConfiguration);
    end
    for index = 1:steps
        frameTimer = tic;
        time = (index-1)*h;
        ego = struct("position",state(1:2),"yaw",state(3),"speed",state(4), ...
            "lateralVelocity",state(5),"yawRate",state(6),"stateTime",time, ...
            "perception",struct("time",time,"range",30,"completeWithinRange",true));
        if index>1, ego.heldActuatorInput = inputs(end,:).';end
        target = struct("trackId","diagnostic-target","targetPositionInertial",[100-10*time;0.8], ...
            "targetVelocityInertial",[-10;0],"targetAccelerationInertial",[0;0], ...
            "targetHeadingInertial",pi,"targetYawRate",0,"targetLength",5,"targetWidth",2, ...
            "predictionMotion",struct("kind","finite-sensing-motion-v1", ...
                "jerkBound",[0;0],"yawAccelerationBound",0,"scalarAccelerationMaximum",0));
        if norm(target.targetPositionInertial-state(1:2))>30,target = [];end
        if estimated
            estimatorTimer = tic;
            [estimatorContext,ego,target,~,audit] = nrmmEstimatorControllerAdapter( ...
                "sample",estimatorContext,time,localEgoTruth(state),localTargetTruth(time,[]));
            estimatorSeconds(index) = toc(estimatorTimer);
            audits{index} = audit;
            if index>1,ego.heldActuatorInput = inputs(end,:).';end
        end
        estimates{index} = ego;targetEstimates{index} = target;
        controllerTimer = tic;
        try
            [command,~,problem,stored] = collisionAvoidanceController(ego,target,road,cfg,stored);
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
        [localTime,trajectory] = ode45(@(~,x) localFlow(x,input,cfg),[0,h],state, ...
            odeset(RelTol=1e-9,AbsTol=1e-11));
        traceTime = [traceTime;time+localTime(2:end)]; %#ok<AGROW>
        traceState = [traceState;trajectory(2:end,:)]; %#ok<AGROW>
        state = trajectory(end,:).';
        states(end+1,:) = state.';times(end+1,1) = time+h; %#ok<AGROW>
        inputs(end+1,:) = input.';metadata{end+1,1} = problem.metadata; %#ok<AGROW>
    end
    result = struct("state",states,"time",times,"input",inputs,"metadata",{metadata}, ...
        "failure",failure,"configuration",cfg,"lastCertificate",stored, ...
        "lastEgo",ego,"lastTarget",target,"stateOrder",["x","y","yaw","vx","vy","r"], ...
        "plant","nonlinear modified-Fiala bicycle with rotated front forces and passive road load", ...
        "observations","exact current states; target only inside 30 m");
    result.estimatorEnabled = estimated;
    result.runtime = struct("frameSeconds",frameSeconds(1:index), ...
        "estimatorSeconds",estimatorSeconds(1:index),"controllerSeconds",controllerSeconds(1:index), ...
        "deadlineSeconds",options.DeadlineSeconds,"deadlineMet",all(frameSeconds(1:index)<=options.DeadlineSeconds), ...
        "enforced",options.EnforceRuntimeDeadline, ...
        "preparation",preparation,"scope","Online synthetic sensors, observer, bounds, input assembly and controller; plant and offline preparation excluded");
    result.egoEstimate = estimates(1:index);
    result.targetEstimate = targetEstimates(1:index);
    result.measurementAudit = audits(1:index);
    result.plantTrace = struct("time",traceTime,"state",traceState);
    targetPosition = [100-10*traceTime,0.8*ones(size(traceTime))];
    separation = rectangleSeparationMargin(traceState(:,1:2),traceState(:,3), ...
        targetPosition,pi*ones(size(traceTime)),cfg.vehicle.length,cfg.vehicle.width,5,2);
    result.minimumSampledSeparationMargin = min(separation);
    result.collisionFreeAtTraceSamples = all(separation>0);
    if estimated
        result.observations = "NRMM estimates from the same bounded-noise sensor adapter as the physical scenario";
        result.estimatorConfiguration = estimatorCfg;
    end
end

function ego = localEgoTruth(state)
    ego = struct("position",state(1:2),"yawAngle",state(3), ...
        "longitudinalVelocity",state(4),"lateralVelocity",state(5),"yawRate",state(6));
end

function target = localTargetTruth(time,~)
    target = struct("targetPositionInertial",[100-10*time;0.8], ...
        "targetVelocityInertial",[-10;0],"targetAccelerationInertial",[0;0]);
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
