function result = runCertificateContinuationScenario(frameAngle)
%runCertificateContinuationScenario Steering avoidance under solver failure.
% Deterministic declared-model experiment: 15 m/s ego, 5 m/s oncoming target
% initially 30 m ahead and 1.5 m left, and straight road edges at +/-6 m.
% The first SOCP supplies a plan. Subsequent solver attempts deliberately
% return no candidate, exercising the continuation through the encounter,
% the former head/tail boundary, and three samples beyond terminal rest.
% This is a discrete-model experiment, not a nonlinear-plant validation.

    arguments
        frameAngle (1,1) double {mustBeFinite, mustBeReal} = 0.0
    end
    rotation = [cos(frameAngle), -sin(frameAngle); sin(frameAngle), cos(frameAngle)];
    cfg = collisionAvoidanceControllerConfig(struct( ...
        "controller", struct("horizonSteps", 12), ...
        "solver", struct("jointFunction", @localFailureAfterAdmission)));
    localFailureAfterAdmission("reset", struct());
    boundary = struct("origin", [0.0; 0.0], ...
        "longitudinalDirection", rotation(:, 1), "lateralDirection", rotation(:, 2), ...
        "coefficients", [0.0, 0.0, 6.0], "parameterRange", [-10.0, 2000.0], ...
        "safeSideSign", -1.0, "boundaryId", "left");
    boundaries = repmat(boundary, 2, 1);
    boundaries(2).coefficients(3) = -6.0;
    boundaries(2).safeSideSign = 1.0;
    boundaries(2).boundaryId = "right";
    road = struct("centerline", [0.0, 0.0; 2000.0, 0.0]*rotation.', "boundaries", boundaries);
    state = [0.0; 0.0; 0.0; 15.0; 0.0; 0.0];
    certificate = [];
    sampleCount = cfg.controller.horizonSteps+brakingSchedule("steps", cfg)+4;
    time = (0:sampleCount-1)*cfg.controller.sampleTime;
    states = zeros(6, sampleCount);
    gap = inf(1, sampleCount);
    roadMargin = inf(1, sampleCount);
    calls = zeros(1, sampleCount);
    fallback = false(1, sampleCount);
    shiftError = zeros(1, sampleCount);
    expected = [];
    for sampleIdx = 1:sampleCount
        position = rotation*state(1:2);
        ego = struct("positionX", position(1), "positionY", position(2), ...
            "yawAngle", state(3)+frameAngle, "longitudinalVelocity", state(4), ...
            "lateralVelocity", state(5), "yawRate", state(6));
        target = struct("targetId", "oncoming", ...
            "targetPositionInertial", rotation*[30.0-5.0*time(sampleIdx); 1.5], ...
            "targetVelocityInertial", rotation*[-5.0; 0.0], ...
            "targetAccelerationInertial", [0.0; 0.0], ...
            "targetYawInertial", pi+frameAngle, "targetLength", 4.8, "targetWidth", 1.9);
        [command, ~, problem, certificate] = collisionAvoidanceController( ...
            ego, target, road, cfg, certificate);
        if sampleIdx == 1
            expected = certificate.predictedState;
        end
        states(:, sampleIdx) = state;
        gap(sampleIdx) = problem.metadata.measuredClearance;
        roadMargin(sampleIdx) = problem.metadata.roadMargin;
        calls(sampleIdx) = problem.metadata.solverCallCount;
        fallback(sampleIdx) = problem.metadata.fallbackUsed;
        shiftError(sampleIdx) = norm(state ...
            - expected(:, min(sampleIdx, size(expected, 2))), inf);
        state = problem.prediction.stageMatrixA(:, :, 1)*state ...
            + problem.prediction.stageMatrixB(:, :, 1)*command.actuatorInput ...
            + problem.prediction.stageAffine(:, 1);
    end
    result = struct("scope", "declaredModelPredictionNodes", "sampleTime", cfg.controller.sampleTime, ...
        "performanceSteps", cfg.controller.horizonSteps, "continuationSteps", brakingSchedule("steps", cfg), ...
        "frameAngle", frameAngle, "randomSeed", [], "time", time, "state", states, ...
        "minimumRectangleClearance", min(gap), "clearanceRequirement", cfg.collision.clearanceMargin, ...
        "minimumRoadMargin", min(roadMargin), ...
        "maximumLateralDisplacement", max(abs(states(2, :))), ...
        "terminalRestResidual", norm(states(4:6, end), inf), ...
        "maximumShiftError", max(shiftError), "solverAttempts", calls, ...
        "fallbackUsed", fallback, "actualOptimizationCount", 1);
end

function solve = localFailureAfterAdmission(phase, program)
    persistent calls
    if phase == "reset"
        calls = 0;
        solve = struct();
        return;
    end
    calls = calls+1;
    if calls == 1
        solve = program.defaultSolver();
    else
        solve = struct("decision", [], "exitFlag", 0, ...
            "output", struct("message", "Injected solver outage after admission"));
    end
end
