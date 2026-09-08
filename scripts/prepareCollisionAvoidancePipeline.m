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
        probeSeconds = zeros(3,1);certified = false(3,1);failures = strings(3,1);
        for sample = 1:3
            sampleTimer = tic;
            target = struct("trackId","preparation-probe", ...
                "targetPositionInertial",pose+max(12,speed*cfg.controller.sampleTime*cfg.controller.horizonSteps)*tangent+0.8*normal, ...
                "targetVelocityInertial",-speed*tangent,"targetAccelerationInertial",zeros(2,1), ...
                "targetHeadingInertial",yaw+pi,"targetYawRate",0, ...
                "predictionMotion",struct("kind","finite-sensing-motion-v1", ...
                "jerkBound",zeros(2,1),"yawAccelerationBound",0,"scalarAccelerationMaximum",0));
            try
                [~,~,problem] = collisionAvoidanceController(ego,target,road,cfg,[]);
                certified(sample) = problem.metadata.planCertified;
            catch exception
                if ~any(string(exception.identifier)==["collisionAvoidanceController:noCertifiedContinuation", ...
                        "collisionAvoidanceController:invalidUncertaintyChart"])
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
        preparation.scope = "Offline independent empty-target and synthetic target admissions; no commands applied";
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
    refinements = zeros(probeCount,1);solverCalls = zeros(probeCount,1);
    certificate = [];
    [~,probeInitialInput] = ltvBicycleModel.cruiseEquilibrium(0,cfg);
    probeHeldInput = probeInitialInput;probeCommittedInput = probeInitialInput;
    for sample = 1:probeCount
        sampleTimer = tic;
        time = (sample-1)*cfg.controller.sampleTime;
        [context,estimate,targets] = nrmmEstimatorControllerAdapter("sample",context,time,truth(time),target(time,[]));
        if cfg.controller.inputDelaySteps>0
            estimate.heldActuatorInput = probeHeldInput;
            estimate.committedActuatorInput = probeCommittedInput;
        end
        try
            [command,~,problem,certificate] = collisionAvoidanceController(estimate,targets,road,cfg,certificate);
            certified(sample) = problem.metadata.planCertified;
            refinements(sample) = problem.metadata.nominalRefinementCount;
            solverCalls(sample) = problem.metadata.solverCallCount;
            probeHeldInput = probeCommittedInput;
            probeCommittedInput = command.actuatorInput;
        catch exception
            if ~any(string(exception.identifier)==["collisionAvoidanceController:noCertifiedContinuation", ...
                    "collisionAvoidanceController:invalidUncertaintyChart", ...
                    "collisionAvoidanceController:inconsistentObservation"])
                rethrow(exception);
            end
            failures(sample) = string(exception.identifier);
            certificate = [];
            probeHeldInput = probeInitialInput;probeCommittedInput = probeInitialInput;
        end
        probeSeconds(sample) = toc(sampleTimer);
    end
    preparation.elapsedSeconds = toc(timer);
    preparation.pipelineProbeSeconds = probeSeconds;
    preparation.pipelineProbeCertified = certified;
    preparation.pipelineProbeFailures = failures;
    preparation.pipelineProbeRefinements = refinements;
    preparation.pipelineProbeSolverCalls = solverCalls;
    preparation.nativeObserverAvailable = exist("nrmmObserverRk4IntervalMex","file")==3;
    preparation.scope = "Offline independent synthetic sensor and controller probes; no state or command reused online";
end
