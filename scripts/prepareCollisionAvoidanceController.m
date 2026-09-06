function preparation = prepareCollisionAvoidanceController(ego, road, cfg)
%prepareCollisionAvoidanceController Load and exercise code before sampling.
% Run before enabling the periodic controller. Twelve discarded calculations
% exercise empty-target, distant-target and crossing-target paths using
% the initial ego/road and synthetic targets, never future sensor
% samples. Explicit certificate state leaves the running controller intact.
% Preparation time is reported separately and is not an online deadline
% guarantee. No command computed here is applied to a vehicle.
% A synthetic probe can be infeasible even when the real initial problem
% is feasible. Record expected admission failures without vetoing the real
% controller call; unexpected input/configuration errors still propagate.

    timer = tic;
    cfg = collisionAvoidanceControllerConfig(cfg);
    if isfield(ego, "targetEstimates"), ego = rmfield(ego, "targetEstimates"); end
    pose = readPlanningInputs(ego, [], road, cfg);
    direction = [cos(pose.yaw); sin(pose.yaw)];
    target = struct("targetId", "initializationOnly", ...
        "targetPositionInertial", pose.position+1000.0*direction, ...
        "targetVelocityInertial", cfg.referenceSpeed*direction, ...
        "targetAccelerationInertial", zeros(2, 1), ...
        "targetYawInertial", pose.yaw, "targetYawRate", 0.0, ...
        "targetLength", cfg.target.defaultLength, ...
        "targetWidth", cfg.target.defaultWidth);
    crossing = target;
    lateral = [-direction(2); direction(1)];
    distance = max(45.0, cfg.referenceSpeed^2/cfg.terminal.backupDeceleration);
    crossing.targetPositionInertial = pose.position+distance*direction+8.0*lateral;
    crossing.targetVelocityInertial = -3.0*lateral;
    crossing.targetYawInertial = pose.yaw-pi/2;
    samples = zeros(4, 3);
    certified = false(4, 3);
    failureIdentifier = strings(4, 3);
    for mode = 1:4
        if mode == 1 || mode == 4
            observation = [];
        elseif mode == 2
            observation = target;
        else
            observation = crossing;
        end
        certificate = [];
        for repetition = 1:3
            if mode == 3
                % Exercise changing reference and geometry data without
                % consulting any future observation or recorded trajectory.
                observation.targetPositionInertial = crossing.targetPositionInertial ...
                    - 5.0*(repetition-1)*direction;
            end
            sampleTimer = tic;
            try
                [~, ~, problem, certificate] = collisionAvoidanceController( ...
                    ego, observation, road, cfg, certificate);
                certified(mode, repetition) = problem.metadata.planCertified;
            catch exception
                identifier = string(exception.identifier);
                if ~any(identifier == ["collisionAvoidanceController:noSolution", ...
                        "collisionAvoidanceController:optimizationFailure", ...
                        "collisionAvoidanceController:invalidStoredCertificate"])
                    rethrow(exception);
                end
                failureIdentifier(mode, repetition) = identifier;
                certificate = [];
            end
            samples(mode, repetition) = toc(sampleTimer);
        end
    end
    preparation = struct("performed", true, "elapsedSeconds", toc(timer), ...
        "callSeconds", samples, "attemptedCalls", numel(samples), ...
        "discardedCommandCount", nnz(certified), ...
        "probeCertified", certified, "failureIdentifier", failureIdentifier, ...
        "allProbesCertified", all(certified, "all"), ...
        "computationalThreads", maxNumCompThreads, ...
        "scope", "Before periodic sampling; no commands applied; synthetic targets");
end
