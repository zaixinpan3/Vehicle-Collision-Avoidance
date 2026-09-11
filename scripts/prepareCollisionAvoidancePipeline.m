function preparation = prepareCollisionAvoidancePipeline(ego,road,cfg,estimatorCfg)
%prepareCollisionAvoidancePipeline Warm online code before periodic execution.
% Synthetic sensor probes exercise target acquisition and bounded outputs.
% Their random stream, observer state and independently solved commands are
% discarded. No probe advances the actual plant or supplies a running plan.
    timer = tic;
    preparation = prepareCollisionAvoidanceController(ego,road,cfg);
    cfg = collisionAvoidanceControllerConfig(cfg);
    pose = ego.position(:);
    yaw = 0;
    for field = ["yaw","yawAngle","egoYaw"]
        if isfield(ego,field),yaw = ego.(field);break;end
    end
    tangent = [cos(yaw);sin(yaw)];normal = [-tangent(2);tangent(1)];
    speed = cfg.referenceSpeed;
    if isempty(fieldnames(estimatorCfg))
        return;
    end
    targetSpeed = max(estimatorCfg.observer.target.domain.speedMinimum, ...
        min(estimatorCfg.observer.target.domain.speedMaximum,0.8*speed));
    range = estimatorCfg.sensor.radar.rangeMaximum;
    target = @(time,~) struct("targetPositionInertial",pose+(range+1-targetSpeed*time)*tangent+0.8*normal, ...
        "targetVelocityInertial",-targetSpeed*tangent,"targetAccelerationInertial",zeros(2,1));
    truth = @(time) struct("position",pose+speed*time*tangent,"yawAngle",yaw, ...
        "longitudinalVelocity",speed,"lateralVelocity",0,"yawRate",0);
    probeCfg = estimatorCfg;
    probeCfg.randomSeed = mod(double(estimatorCfg.randomSeed)+104729,2^32);
    context = nrmmEstimatorControllerAdapter("initialize",probeCfg,truth(0),target);
    probeCount = 24;
    probeSeconds = zeros(probeCount,1);certified = false(probeCount,1);failures = strings(probeCount,1);
    solverCalls = zeros(probeCount,1);
    for sample = 1:probeCount
        sampleTimer = tic;
        time = (sample-1)*cfg.controller.sampleTime;
        [context,estimate,targets] = nrmmEstimatorControllerAdapter("sample",context,time,truth(time),target(time,[]));
        try
            [~,~,problem] = collisionAvoidanceController(estimate,targets,road,cfg,[]);
            certified(sample) = problem.metadata.planCertified;
            solverCalls(sample) = problem.metadata.solverCallCount;
        catch exception
            if ~startsWith(string(exception.identifier),"collisionAvoidanceController:")
                rethrow(exception);
            end
            failures(sample) = string(exception.identifier);
        end
        probeSeconds(sample) = toc(sampleTimer);
    end
    preparation.elapsedSeconds = toc(timer);
    preparation.pipelineProbeSeconds = probeSeconds;
    preparation.pipelineProbeCertified = certified;
    preparation.pipelineProbeFailures = failures;
    preparation.pipelineProbeSolverCalls = solverCalls;
    preparation.nativeObserverAvailable = exist("nrmmObserverRk4IntervalMex","file")==3;
    preparation.scope = "Offline independent synthetic sensor and controller probes; no state or command reused online";
end
