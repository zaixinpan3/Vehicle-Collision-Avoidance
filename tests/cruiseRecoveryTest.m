classdef cruiseRecoveryTest < matlab.unittest.TestCase
    %cruiseRecoveryTest Slack-only cruise optimization and initialization.

    properties (TestParameter)
        speed = struct("moderateError", 14.4, "actuatorLimitedError", 8.0);
    end

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

        function everyInputPlanHasTheSameCostAtFixedRelaxation(testCase)
            cfg = localConfiguration();
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", 14.4);
            [~, ~, problem] = collisionAvoidanceController(ego, [], [0, 0; 2000, 0], cfg, []);
            decision = problem.decision;
            decision(end) = 0.7;
            alternative = decision;
            alternative(1:end-1) = sin((1:problem.layout.planCount).');
            expectedCost = cfg.clf.relaxationWeight*0.7;

            testCase.verifyEqual(localObjective(problem.qp, decision), expectedCost, AbsTol=1.0e-12);
            testCase.verifyEqual(localObjective(problem.qp, alternative), expectedCost, AbsTol=1.0e-12);
            testCase.verifyEqual(nnz(problem.qp.Hessian), 0);
            testCase.verifyEqual(nnz(problem.qp.stageProgram.P), 0);
            testCase.verifyEqual(nnz(problem.qp.stageProgram.q), 1);
            testCase.verifyFalse(isfield(problem.qp, "preferredInput"));
            testCase.verifyFalse(isfield(problem.qp.clf.certificate, "sampledFeedbackGain"));
        end

        function relaxationMatchesAnIndependentLinearProgram(testCase, speed)
            cfg = localConfiguration();
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", speed);
            road = [0, 0; 2000, 0];
            [~, ~, native] = collisionAvoidanceController(ego, [], road, cfg, []);
            cfg.solver.jointFunction = @localLinprog;
            [~, ~, independent] = collisionAvoidanceController(ego, [], road, cfg, []);

            testCase.verifyTrue(native.metadata.planCertified);
            testCase.verifyTrue(independent.metadata.planCertified);
            testCase.verifyEqual(native.metadata.clfRelaxation, ...
                independent.metadata.clfRelaxation, AbsTol=1.0e-5);
            testCase.verifyEqual(native.metadata.jointObjectiveValue, ...
                cfg.clf.relaxationWeight*native.metadata.clfRelaxation, AbsTol=1.0e-10);
            testCase.verifyLessThanOrEqual(native.metadata.hardRowViolation, 1.0e-5);
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

function value = localObjective(qp, decision)
    value = 0.5*decision.'*qp.Hessian*decision+qp.linear.'*decision+qp.constant;
end

function result = localLinprog(~, program)
    options = optimoptions("linprog", "Display", "none", ...
        "ConstraintTolerance", 1.0e-9, "OptimalityTolerance", 1.0e-9);
    [decision, ~, exitFlag, output] = linprog(program.f, ...
        program.A, program.b, program.Aeq, program.beq, program.lb, program.ub, options);
    result = struct("decision", decision, "exitFlag", exitFlag, "output", output);
end
