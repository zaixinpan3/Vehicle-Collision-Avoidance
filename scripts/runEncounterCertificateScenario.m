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
        "model", struct("lateralDomainRadius", 2, "linearizationPolicy","cruise", "plantModelResidualRateBound", rate)));
    ego = struct("position", [0;0], "yaw", 0, "speed", 8, "stateTime", 0, ...
        "perception",struct("time",0,"range",16,"completeWithinRange",true));
    route = [-100,0;2000,0];
    motion = struct("kind","finite-sensing-motion-v1", ...
        "jerkBound",[0;0],"yawAccelerationBound",0);
    target = struct("trackId", 1, "targetPositionInertial", [15;-4], ...
        "targetVelocityInertial", [0;32], "targetAccelerationInertial", [0;0], ...
        "targetHeadingInertial", pi/2, "targetYawRate", 0, "predictionMotion", motion);
    [command, ~, problem, certificate] = collisionAvoidanceController(ego, target, route, cfg, []);
    trueState = problem.model.initialEgoState;
    deadline = certificate.deadline;
    minimumDistance = inf;
    maximumClfResidual = -inf;
    maximumBoxViolation = -inf;
    failure = struct("occurred", false, "identifier", "", "message", "", "time", NaN);
    executedIntervals = 0;
    issued = zeros(2,cfg.controller.horizonSteps);
    fallbackCount = 0;
    sampleTime = cfg.controller.sampleTime;
    for stage = 1:cfg.controller.horizonSteps
        a = problem.prediction.continuousA(:, :, stage);
        b = problem.prediction.continuousB(:, :, stage);
        c = problem.prediction.continuousC(:, stage);
        input = command.actuatorInput;
        issued(:, stage) = input;
        dt = sampleTime/20;
        transition = expm(dt*[a, eye(6); zeros(6,12)]);
        for cellIndex = 1:20
            time = (stage-1)*sampleTime+(cellIndex-1)*dt;
            forcing = rate.*sin((1:6).'+2*time);
            derivative = a*trueState+b*input+c+forcing;
            error = trueState(2:6)-problem.qp.clf.referenceStart ...
                -problem.qp.clf.referenceRate*time;
            p = problem.qp.clf.lyapunovMatrix;
            residual = 2*error.'*p*(derivative(2:6)-problem.qp.clf.referenceRate) ...
                +problem.qp.clf.decayRate*(error.'*p*error)-problem.metadata.clfRelaxation(1);
            maximumClfResidual = max(maximumClfResidual, residual);
            [position, yaw] = laneGeometry.fromFrenet(trueState, problem.model.lane);
            distance = avoidanceSafetyGeometry.rectangleDistance(position, yaw, [15;-4+32*time], pi/2, [2.4;.95;2.4;.95]);
            minimumDistance = min(minimumDistance, distance);
            trueState = transition(1:6,1:6)*trueState+transition(1:6,7:12)*(b*input+c+forcing);
        end
        maximumBoxViolation = max(maximumBoxViolation, max(abs(trueState-certificate.predictedState(:,2)) ...
            -certificate.stateErrorBound(:,2)));
        [position,yaw] = laneGeometry.fromFrenet(trueState,problem.model.lane);
        ego = struct("position",position,"yaw",yaw,"speed",trueState(4), ...
            "lateralVelocity",trueState(5),"yawRate",trueState(6), ...
            "stateTime",stage*sampleTime,"heldActuatorInput",input, ...
            "perception",struct("time",stage*sampleTime,"range",16,"completeWithinRange",false));
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
        fallbackCount = fallbackCount+double(problem.metadata.fallbackUsed);
        assert(abs(certificate.deadline-deadline) < 1e-12);
        if isempty(command), break; end
    end
    discharged = ~failure.occurred && certificate.encounterComplete;
    exitTime = NaN;
    if discharged, exitTime = certificate.stateTime; end
    report = struct("scope","declared affine inclusion; deterministic residual; no nonlinear-plant claim", ...
        "executedIntervals",executedIntervals,"fallbackCount",fallbackCount,"exitTime",exitTime, ...
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
