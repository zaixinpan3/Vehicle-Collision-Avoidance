function result = runRobustPoseCertificateScenario()
%runRobustPoseCertificateScenario Exercise the stationary-pose certificate.
% A straight-road scheduled affine plant starts with all eight vertices of
% a pose-error box. One SOCP is followed by failed optimization attempts
% through three steps beyond rest. Current measurement boxes vary in time.
% This is a declared-model experiment, not an NRMM or nonlinear-plant run.

    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root, "controller"), fullfile(root, "config"));
    cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 4)));
    cfg.solver.jointFunction = @localFailAfterAdmission;
    localFailAfterAdmission("reset", struct());
    radius = [0.1; 0.1; 0.001; 0; 0; 0];
    ego = struct("position", [10; 0], "yawAngle", 0, ...
        "longitudinalVelocity", 5, "lateralVelocity", 0, "yawRate", 0, ...
        "stateTime", 0, "controllerStateErrorBound", radius);
    target = struct("targetId", "stationary", "targetPositionInertial", [60; 0], ...
        "targetVelocityInertial", [0; 0], "targetAccelerationInertial", [0; 0], ...
        "targetYawInertial", 0, "targetLength", 4.8, "targetWidth", 1.9);
    [~, ~, first, stored] = collisionAvoidanceController(ego, target, [0, 0; 1000, 0], cfg, []);
    count = first.prediction.stageCount+3;
    signs = 2*double(dec2bin(0:7, 3).'-'0')-1;
    truth = stored.predictedState(:, 1)+[radius(1:3).*signs; zeros(3, 8)];
    minClearance = inf;
    maxContainmentResidual = -inf;
    fallback = false(1, count);
    estimateBound = zeros(6, count);
    effectiveBound = zeros(6, count);
    for step = 1:count
        if step == 1
            problem = first;
        else
            nominal = stored.predictedState(:, 2);
            ego.position = nominal(1:2);
            ego.yawAngle = nominal(3);
            ego.longitudinalVelocity = nominal(4);
            ego.lateralVelocity = nominal(5);
            ego.yawRate = nominal(6);
            ego.stateTime = (step-1)*cfg.controller.sampleTime;
            ego.heldActuatorInput = stored.appliedInput;
            ego.controllerStateErrorBound = ...
                (1.5+0.4*sin(step))*stored.stateErrorBound(:, 2);
            [~, ~, problem, stored] = collisionAvoidanceController( ...
                ego, target, [0, 0; 1000, 0], cfg, stored);
        end
        estimateBound(:, step) = problem.metadata.currentFrenetEstimationBound;
        effectiveBound(:, step) = stored.stateErrorBound(:, 1);
        maxContainmentResidual = max(maxContainmentResidual, max( ...
            abs(truth-stored.predictedState(:, 1))-stored.stateErrorBound(:, 1), [], "all"));
        dimensions = [cfg.vehicle.length/2; cfg.vehicle.width/2; 2.4; 0.95];
        for vertex = 1:8
            distance = rectangleConfigurationDistance(truth(1:2, vertex), truth(3, vertex), ...
                [60; 0], 0, dimensions);
            minClearance = min(minClearance, distance);
        end
        fallback(step) = problem.metadata.fallbackUsed;
        truth = problem.prediction.stageMatrixA(:, :, 1)*truth ...
            + problem.prediction.stageMatrixB(:, :, 1)*stored.appliedInput ...
            + problem.prediction.stageAffine(:, 1);
    end
    result = struct("scope", "declaredAffineModelPredictionNodes", ...
        "scenario", "straight-road-eight-pose-vertices", "sampleCount", count, ...
        "actualOptimizationCount", 1, "fallbackUsed", fallback, ...
        "maximumContainmentResidual", maxContainmentResidual, ...
        "minimumRectangleClearance", minClearance, ...
        "clearanceRequirement", cfg.collision.clearanceMargin, ...
        "maximumFinalSpeedMagnitude", max(abs(truth(4:6, :)), [], "all"), ...
        "currentEstimateBound", estimateBound, "effectiveBound", effectiveBound);
    assert(all(fallback(2:end)) && ~fallback(1));
    assert(maxContainmentResidual <= 1e-10);
    assert(minClearance >= cfg.collision.clearanceMargin-1e-8);
    assert(result.maximumFinalSpeedMagnitude <= 1e-8);
end

function result = localFailAfterAdmission(phase, program)
    persistent calls
    if phase == "reset"
        calls = 0;
        result = struct();
        return;
    end
    calls = calls+1;
    if calls == 1
        result = program.defaultSolver();
    else
        result = struct("decision", NaN(numel(program.f), 1), "exitFlag", -1, ...
            "output", struct("message", "deliberate continuation test failure"));
    end
end
