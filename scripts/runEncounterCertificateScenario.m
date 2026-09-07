function report = runEncounterCertificateScenario(options)
%runEncounterCertificateScenario Finite crossing under a declared inclusion.
% Tests the runtime certificate with nonzero process residuals, independent
% matrix-exponential truth integration, lost target observations and solver
% failure after admission. A failed control call ends the simulation before
% another input is applied. This is not a nonlinear physical-plant validation.
    arguments
        options.ForceSolverFailure (1, 1) logical = true
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root, "controller"), fullfile(root, "config"));
    rate = [1e-3;1e-4;1e-5;1e-3;1e-4;1e-5];
    cfg = collisionAvoidanceControllerConfig(struct("referenceSpeed", 8, ...
        "controller", struct("horizonSteps", 16, "sampleTime", 0.1), ...
        "model", struct("lateralDomainRadius", 2, "plantModelResidualRateBound", rate)));
    ego = struct("position", [0;0], "yaw", 0, "speed", 8, "stateTime", 0);
    route = [-100,0;2000,0];
    contract = struct("kind", "cartesian-jerk-exit-v1", "id", "declared-crossing", ...
        "validFrom", 0, "validUntil", 1.6, "jerkBound", [0;0], ...
        "yawAccelerationBound", 0, "exitNormal", [0;1], "exitOffset", 4.6, ...
        "postExitRoute", "nonreturningHalfspace");
    target = struct("trackId", 1, "targetPositionInertial", [15;-4], ...
        "targetVelocityInertial", [0;8], "targetAccelerationInertial", [0;0], ...
        "targetHeadingInertial", pi/2, "targetYawRate", 0, "encounterContract", contract);
    [command, ~, problem, certificate] = collisionAvoidanceController(ego, target, route, cfg, []);
    trueState = problem.model.initialEgoState;
    deadline = certificate.deadline;
    minimumDistance = inf;
    maximumClfResidual = -inf;
    maximumBoxViolation = -inf;
    failure = struct("occurred", false, "identifier", "", "message", "", "time", NaN);
    executedIntervals = 0;
    issued = zeros(2, 15);
    sampleTime = cfg.controller.sampleTime;
    for stage = 1:15
        a = problem.prediction.continuousA(:, :, 1);
        b = problem.prediction.continuousB(:, :, 1);
        c = problem.prediction.continuousC(:, 1);
        input = command.actuatorInput;
        issued(:, stage) = input;
        dt = sampleTime/20;
        transition = expm(dt*[a, eye(6); zeros(6,12)]);
        for cellIndex = 1:20
            time = (stage-1)*sampleTime+(cellIndex-1)*dt;
            forcing = rate.*sin((1:6).'+2*time);
            derivative = a*trueState+b*input+c+forcing;
            error = trueState(2:6)-problem.qp.clf.referenceStart ...
                -problem.qp.clf.referenceRate*((cellIndex-1)*dt);
            p = problem.qp.clf.lyapunovMatrix;
            residual = 2*error.'*p*(derivative(2:6)-problem.qp.clf.referenceRate) ...
                +problem.qp.clf.decayRate*(error.'*p*error)-problem.metadata.clfRelaxation(1);
            maximumClfResidual = max(maximumClfResidual, residual);
            [position, yaw] = laneGeometry.fromFrenet(trueState, problem.model.lane);
            distance = rectangleConfigurationDistance(position, yaw, [15;-4+8*time], pi/2, [2.4;.95;2.4;.95]);
            minimumDistance = min(minimumDistance, distance);
            trueState = transition(1:6,1:6)*trueState+transition(1:6,7:12)*(b*input+c+forcing);
        end
        maximumBoxViolation = max(maximumBoxViolation, max(abs(trueState-certificate.predictedState(:,2)) ...
            -certificate.stateErrorBound(:,2)));
        [position,yaw] = laneGeometry.fromFrenet(trueState,problem.model.lane);
        ego = struct("position",position,"yaw",yaw,"speed",trueState(4), ...
            "lateralVelocity",trueState(5),"yawRate",trueState(6), ...
            "stateTime",stage*sampleTime,"heldActuatorInput",input);
        executedIntervals = stage;
        if options.ForceSolverFailure, cfg.solver.jointFunction = @localFailure; end
        try
            [command,~,problem,certificate] = collisionAvoidanceController(ego,[],route,cfg,certificate);
        catch exception
            if ~startsWith(string(exception.identifier), "collisionAvoidanceController:")
                rethrow(exception);
            end
            failure = struct("occurred", true, "identifier", string(exception.identifier), ...
                "message", string(exception.message), "time", ego.stateTime);
            break;
        end
        if any(~[certificate.encounters.discharged])
            assert(abs(certificate.deadline-deadline) < 1e-12);
        end
    end
    discharged = ~failure.occurred && all([certificate.encounters.discharged]);
    exitTime = NaN;
    if discharged, exitTime = certificate.stateTime; end
    report = struct("scope","declared affine inclusion; deterministic residual; no nonlinear-plant claim", ...
        "executedIntervals",executedIntervals,"fallbackCount",0,"exitTime",exitTime, ...
        "deadline",deadline,"discharged",discharged,"failure",failure, ...
        "simulatedDuration",executedIntervals*sampleTime, ...
        "minimumSampledRectangleDistance",minimumDistance,"requiredDistance",cfg.collision.clearanceMargin, ...
        "maximumSampledClfResidual",maximumClfResidual,"maximumEndpointBoxViolation",maximumBoxViolation, ...
        "finalMargin",certificate.margin,"issuedInput",issued(:,1:executedIntervals));
    assert(minimumDistance >= cfg.collision.clearanceMargin ...
        && maximumClfResidual <= 1e-9 && maximumBoxViolation <= 1e-9);
end

function result = localFailure(~,~)
    result = struct("decision",[],"exitFlag",-999,"output",struct());
end
