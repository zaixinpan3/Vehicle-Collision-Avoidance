function preparation = prepareCollisionAvoidanceController(ego, road, cfg, target, options)
%prepareCollisionAvoidanceController Exercise the held-flow and conic kernels.
% Optional target probes use the supplied exact scene and discard commands.
% Each accepted fresh probe is followed by SuccessorProbes chained calls on
% the declared affine successor of its own accepted plan (predicted center,
% issued held input, target advanced by its published velocity), so the
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
    nativePath = fullfile(fileparts(fileparts(mfilename("fullpath"))),"solver","bicycle");
    if isfolder(nativePath),addpath(nativePath);end
    cfg = collisionAvoidanceControllerConfig(cfg);
    if isfield(ego, "targetEstimates"), ego = rmfield(ego, "targetEstimates"); end
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
        "nativeRolloutAvailable",exist("bicycleNominalKernelMex","file")==3, ...
        "nativeLinearizationAvailable",exist("bicycleLinearizationKernelMex","file")==3, ...
        "nativeGeometryAvailable",exist("avoidanceCellRowsKernelMex","file")==3, ...
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
% The next scene of the declared affine plant: the accepted plan's predicted
% successor center, its issued input held, and the target advanced at its
% published velocity over one hold. Aliases of the updated fields are removed
% so the parser reads the successor values.
    state = stored.predictedState(:, 2);
    [position, heading] = laneGeometry.fromFrenet(state, problem.model.lane);
    for name = ["positionX", "x", "positionY", "y", "yawAngle", "heading", "longitudinalVelocity"]
        if isfield(ego, name), ego = rmfield(ego, name); end
    end
    ego.position = position;
    ego.yaw = heading;
    ego.speed = state(4);
    ego.lateralVelocity = state(5);
    ego.yawRate = state(6);
    ego.stateTime = problem.model.stateTime+problem.model.sampleTime;
    ego.heldActuatorInput = stored.appliedInput;
    if isfield(ego, "perception") && isstruct(ego.perception) && isfield(ego.perception, "time")
        ego.perception.time = ego.stateTime;
    end
    target.targetPositionInertial = target.targetPositionInertial(:) ...
        +problem.model.sampleTime*target.targetVelocityInertial(:);
end
