classdef cruiseRecoveryTest < matlab.unittest.TestCase
    %cruiseRecoveryTest Sampled cruise feedback under the unchanged safety QP.

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "scripts")));
        end
    end

    methods (Test)
        function aFailedDiscardedProbeDoesNotVetoTheRealController(testCase)
            cfg = localConfiguration();
            cfg.solver.jointFunction = localFailFirstSolve();
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", 15);
            road = [0, 0; 2000, 0];
            preparation = prepareCollisionAvoidanceController(ego, road, cfg);
            [command, ~, problem] = collisionAvoidanceController(ego, [], road, cfg, []);
            testCase.verifyFalse(preparation.allProbesCertified);
            testCase.verifyEqual(preparation.attemptedCalls, 12);
            testCase.verifyEqual(preparation.discardedCommandCount, 11);
            testCase.verifyEqual(preparation.failureIdentifier(1, 1), ...
                "collisionAvoidanceController:optimizationFailure");
            testCase.verifyTrue(all(preparation.probeCertified(2:end, :), "all"));
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyNotEmpty(command);
        end

        function moderateSpeedErrorRequestsRecoveryBeyondTheMinimumClfDecay(testCase)
            cfg = localConfiguration();
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", 14.4);
            [command, ~, problem] = collisionAvoidanceController(ego, [], [0, 0; 2000, 0], cfg, []);
            minimumClfInput = -problem.qp.clf.decayRate*(-0.6) ...
                /(2*cfg.model.longitudinalInputGain);
            testCase.verifyGreaterThan(command.actuatorInput(2), 2*minimumClfInput);
            testCase.verifyLessThanOrEqual(command.actuatorInput(2), ...
                cfg.actuation.longitudinalAccelerationMaximum+1.0e-5);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyLessThanOrEqual(problem.metadata.hardRowViolation, 1.0e-5);
        end

        function theUnconstrainedSampledFeedbackHasStablePoles(testCase)
            cfg = localConfiguration();
            ego = struct("position", [0; 0], "yaw", 0, "speed", 15);
            [~, ~, problem] = collisionAvoidanceController(ego, [], [0, 0; 2000, 0], cfg, []);
            certificate = problem.qp.clf.certificate;
            [a, b] = ltvBicycleModel.stageMatrices(0, cfg.referenceSpeed, cfg.controller.sampleTime, cfg);
            poles = eig(a(2:6, 2:6)-b(2:6, :)*certificate.sampledFeedbackGain);
            testCase.verifyLessThan(max(abs(poles)), 1.0);
            testCase.verifyEqual(problem.qp.preferredInput, [0; 0], AbsTol=1.0e-12);
        end

        function preparationDoesNotConsumeOrReplaceTheLiveCertificate(testCase)
            cfg = localConfiguration();
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", 15);
            road = [0, 0; 2000, 0];
            collisionAvoidanceController("resetNominalTrajectory");
            testCase.addTeardown(@() collisionAvoidanceController("resetNominalTrajectory"));
            [~, ~, ~, certificate] = collisionAvoidanceController(ego, [], road, cfg);
            next = certificate.predictedState(:, 2);
            ego.position = next(1:2);
            ego.yawAngle = next(3);
            ego.speed = next(4);
            ego.lateralVelocity = next(5);
            ego.yawRate = next(6);
            preparation = prepareCollisionAvoidanceController(ego, road, cfg);
            [actual, ~, diagnostic] = collisionAvoidanceController(ego, [], road, cfg);
            expected = collisionAvoidanceController(ego, [], road, cfg, certificate);
            testCase.verifyTrue(preparation.performed);
            testCase.verifyEqual(preparation.discardedCommandCount, 12);
            testCase.verifyTrue(diagnostic.metadata.certificateCompatible);
            testCase.verifyEqual(actual.actuatorInput, expected.actuatorInput, AbsTol=1.0e-9);
        end
    end
end

function hook = localFailFirstSolve()
    count = 0;
    hook = @solve;
    function result = solve(~, program)
        count = count+1;
        if count == 1
            result = struct("decision", [], "exitFlag", -999, "output", struct());
        else
            result = program.defaultSolver();
        end
    end
end

function cfg = localConfiguration()
    cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 4), ...
        "model", struct("longitudinalInputGain", 0.8)));
end
