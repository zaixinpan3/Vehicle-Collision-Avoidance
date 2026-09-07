classdef terminalDissipationTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
        end
    end

    properties (TestParameter)
        curvature = {0, 1/400, -1/60};
    end

    methods (Test)
        function allVelocityVerticesStayInsideTheInfinitePoseBudget(testCase, curvature)
            [certificate, cfg] = localCertificate(curvature);
            [residual, speed, velocityResidual] = localContinueVertices(certificate);
            testCase.verifyTrue(certificate.accepted);
            testCase.verifyLessThanOrEqual(residual, 1e-10);
            testCase.verifyLessThanOrEqual(velocityResidual, 1e-10);
            testCase.verifyGreaterThanOrEqual(speed, 0);
            testCase.verifyLessThan(speed, cfg.model.speedMaximum);
        end

        function liveVelocityBoundsCanBeAdmittedWithoutAnInputTarget(testCase)
            [ego, cfg, lane] = localInputs();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyFalse(isfield(stored, "terminalUncertainty"));
            testCase.verifyGreaterThan(stored.stateErrorBound(4, end), 0);
            testCase.verifyGreaterThanOrEqual(stored.predictedState(4, end), stored.stateErrorBound(4, end));
        end

        function finiteAdmissionDoesNotAppendADissipativeTerminalConstraint(testCase)
            [ego, cfg, lane] = localInputs();
            [~, ~, problem] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyEqual(problem.layout.tailSteps, 0);
            testCase.verifyEqual(problem.prediction.nodeCount, cfg.controller.horizonSteps+1);
            testCase.verifyEmpty(problem.qp.equalityBound);
        end

        function straightPolylineVerticesDoNotInvalidateAnInteriorChart(testCase)
            [ego, cfg] = localInputs();
            lane = [(0:.1:150).', zeros(1501, 1)];
            [~, ~, problem] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function absentPassiveDampingCannotCertifyAnInfiniteStop(testCase)
            [~, cfg] = localCertificate(0);
            cfg.roadLoad.rollingCoefficient = 0;
            model = struct("cfg", cfg, "longitudinalAccelerationBias", 0);
            prediction = struct("scheduleCurvature", [0, 0], "scheduleSpeedProfile", [0, 0]);
            certificate = terminalDissipation.build(prediction, model);
            testCase.verifyFalse(certificate.accepted);
        end

        function persistentForcingCanBeEnclosedOverAFiniteCertificate(testCase)
            [ego, cfg, lane] = localInputs();
            cfg.model.plantModelResidualRateBound(4) = .001;
            [~, ~, problem, stored] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThan(stored.stateErrorBound(4, end), 0);
        end
    end
end

function [certificate, cfg] = localCertificate(curvature)
    cfg = collisionAvoidanceControllerConfig();
    model = struct("cfg", cfg, "longitudinalAccelerationBias", 0);
    prediction = struct("scheduleCurvature", [curvature, curvature], ...
        "scheduleSpeedProfile", [0, 0]);
    certificate = terminalDissipation.build(prediction, model);
end

function [residual, minimumSpeed, velocityResidual] = localContinueVertices(certificate)
    vertices = 2*double(dec2bin(0:7, 3).'-'0')-1;
    velocity = certificate.velocityLimit.*vertices;
    velocity(1, :) = abs(velocity(1, :));
    state = [zeros(3, 8); velocity];
    budget = certificate.poseExcursionMatrix*abs(velocity);
    residual = -inf;
    velocityResidual = -inf;
    minimumSpeed = inf;
    transition = expm(.013*certificate.continuousA);
    for step = 1:5000
        state = transition*state;
        remaining = certificate.poseExcursionMatrix*abs(state(4:6, :));
        residual = max(residual, max(abs(state(1:3, :))+remaining-budget, [], "all"));
        velocityResidual = max(velocityResidual, max(abs(state(4:6, :)) ...
            -certificate.velocityLimit, [], "all"));
        minimumSpeed = min(minimumSpeed, min(state(4, :)));
    end
end

function [ego, cfg, lane] = localInputs()
    cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 4)));
    lane = [0, 0; 1000, 0];
    ego = struct("position", [10; 0], "yawAngle", 0, ...
        "longitudinalVelocity", 10, "lateralVelocity", 0, "yawRate", 0, ...
        "stateTime", 0, "controllerStateErrorBound", [.04; .04; .014; .388; .388; .0015]);
end
