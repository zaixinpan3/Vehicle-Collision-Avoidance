function preparation = prepareCollisionAvoidanceController(ego, road, cfg, target, options)
%prepareCollisionAvoidanceController Prepare validated nonlinear-flow kernels.
% Optional target probes use the supplied exact scene and discard commands.
% Each accepted fresh probe is followed by SuccessorProbes chained calls on
% the declared nonlinear successor of its accepted plan (predicted center,
% issued held input, target advanced by its constant-parameter NRMM), so the
% carried-witness code paths are compiled before periodic sampling as well.
% Without a target, only native paths are prepared. No missing observation is
% synthesized and no probe replaces a running certificate.
    arguments
        ego (1,1) struct
        road
        cfg
        target = []
        options.SuccessorProbes (1,1) double {mustBeInteger, mustBeNonnegative} = 2
    end
    timer = tic;
    nativePath = fullfile(fileparts(fileparts(mfilename("fullpath"))),"solver","nonlinear");
    if ~isfolder(nativePath) || ~isfile(fullfile(nativePath,"nonlinearSafetyMex."+mexext))
        buildFialaIntervalVerifier(string(nativePath));
    end
    addpath(nativePath);
    cfg = collisionAvoidanceControllerConfig(cfg);
    if isfield(ego, "targetEstimate"), ego = rmfield(ego, "targetEstimate"); end
    probeCount = 3*~isempty(target);
    samples = zeros(1, probeCount);
    certified = false(1, probeCount);
    failures = strings(1, probeCount);
    successorCount = options.SuccessorProbes*double(localSuccessorAvailable(ego, target));
    successorSamples = zeros(probeCount, successorCount);
    successorFailures = strings(probeCount, successorCount);
    for repetition = 1:probeCount
        sampleTimer = tic;
        try
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, road, cfg, []);
            certified(repetition) = problem.metadata.planCertified;
        catch exception
            if ~startsWith(string(exception.identifier), "collisionAvoidanceController:")
                rethrow(exception);
            end
            failures(repetition) = string(exception.identifier);
        end
        samples(repetition) = toc(sampleTimer);
        if strlength(failures(repetition)) > 0, continue; end
        successorEgo = ego; successorTarget = target;
        for step = 1:successorCount
            successorTimer = tic;
            try
                [successorEgo, successorTarget] = localDeclaredSuccessor(successorEgo, successorTarget, problem, stored);
                [~, ~, problem, stored] = collisionAvoidanceController(successorEgo, successorTarget, road, cfg, stored);
            catch exception
                if ~startsWith(string(exception.identifier), "collisionAvoidanceController:")
                    rethrow(exception);
                end
                successorFailures(repetition, step) = string(exception.identifier);
            end
            successorSamples(repetition, step) = toc(successorTimer);
            if strlength(successorFailures(repetition, step)) > 0, break; end
        end
    end
    preparation = struct("performed", true, "elapsedSeconds", toc(timer), ...
        "callSeconds", samples, "attemptedCalls", numel(samples), ...
        "discardedCommandCount", nnz(certified), "probeCertified", certified, ...
        "failureIdentifier", failures, "allProbesCertified", ~isempty(certified) && all(certified), ...
        "successorProbeSeconds", successorSamples, "successorFailureIdentifier", successorFailures, ...
        "computationalThreads", maxNumCompThreads, ...
        "nativeRolloutAvailable",exist("fialaFeedbackSampleMex","file")==3, ...
        "nativeLinearizationAvailable",exist("fialaIntervalMex","file")==3, ...
        "nativeGeometryAvailable",exist("nonlinearSafetyMex","file")==3, ...
        "scope", "Before periodic sampling; explicitly supplied target probes and their declared successors; no commands applied");
end

function available = localSuccessorAvailable(ego, target)
% Successor scenes need a plain Cartesian ego record and an inertial target
% record with a published velocity; other input forms keep fresh probes only.
    available = ~isempty(target) && isstruct(target) && isscalar(target) ...
        && all(isfield(target, ["targetPositionInertial", "targetVelocityInertial"])) ...
        && ~isfield(ego, "egoState") && isfield(ego, "position");
end

function [ego, target] = localDeclaredSuccessor(ego, target, problem, stored)
% The next scene uses the accepted nonlinear prediction and NRMM flow.
% Aliases of the updated fields are removed
% so the parser reads the successor values.
    state = stored.predictedState(:, 2);
    for name = ["positionX", "x", "positionY", "y", "yawAngle", "heading", "longitudinalVelocity"]
        if isfield(ego, name), ego = rmfield(ego, name); end
    end
    ego.position = state(1:2);
    ego.yaw = state(3);
    ego.speed = state(4);
    ego.lateralVelocity = state(5);
    ego.yawRate = state(6);
    ego.stateTime = stored.stateTime+problem.model.cfg.controller.sampleTime;
    ego.heldActuatorInput = stored.appliedInput;
    if isfield(ego, "perception") && isstruct(ego.perception) && isfield(ego.perception, "time")
        ego.perception.time = ego.stateTime;
    end
    q=nonlinearSafetyCertificate.targetFlow(problem.model.target,problem.model.cfg.controller.sampleTime);
    target.targetPositionInertial=q(1:2);target.targetYawInertial=q(3);target.targetYawRate=q(6);
    target.targetVelocityInertial=q(4)*[cos(q(3)+q(5));sin(q(3)+q(5))];
    target.targetAccelerationInertial=q(6)*[-target.targetVelocityInertial(2);target.targetVelocityInertial(1)];
end
