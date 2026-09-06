classdef sparseAvoidanceQpTest < matlab.unittest.TestCase
%sparseAvoidanceQpTest The sparse lift preserves the physical optimization.

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "solver", "clarabel", "matlab")));
        end
    end

    methods (Test)
        function liftingAnyInputPlanPreservesAllHardRowResiduals(testCase)
            problem = localProblem();
            [decision, lifted] = localDecision(problem);
            qp = problem.qp;
            native = qp.stageProgram;
            rowCount = size(qp.inequalityMatrix, 1);
            rows = native.cones(1)+(1:rowCount);

            original = qp.inequalityMatrix*decision-qp.inequalityBound;
            sparseResidual = native.A(rows, :)*lifted-native.b(rows);

            testCase.verifyEqual(sparseResidual, original, AbsTol=1.0e-10);
            testCase.verifyGreaterThan(nnz(qp.rowFamily == "collision"), 0);
            testCase.verifyGreaterThan(nnz(qp.rowFamily == "road"), 0);
            testCase.verifyGreaterThan(nnz(qp.rowFamily == "friction"), 0);
        end

        function liftingAnyInputPlanPreservesDynamicsAndRestResiduals(testCase)
            problem = localProblem();
            [decision, lifted] = localDecision(problem);
            native = problem.qp.stageProgram;
            rows = 1:native.cones(1);

            residual = native.A(rows, :)*lifted-native.b(rows);
            expected = [zeros(6*problem.prediction.stageCount, 1); ...
                problem.qp.equalityMatrix*decision-problem.qp.equalityBound; ...
                decision(problem.layout.planCount-1:problem.layout.planCount)-problem.qp.terminalInput];

            testCase.verifyEqual(residual, expected, AbsTol=1.0e-10);
        end

        function theNativeQpRetainsTheAffineClfResidual(testCase)
            problem = localProblem();
            [decision, lifted] = localDecision(problem);
            native = problem.qp.stageProgram;
            clf = problem.qp.clf;
            residual = native.A(end, :)*lifted-native.b(end);
            derivative = clf.lieDerivativeDrift+clf.lieDerivativeInput*decision(1:2);
            expected = derivative+clf.decayRate*clf.initialValue-decision(end);

            testCase.verifyEqual(residual, expected, AbsTol=1.0e-10);
            testCase.verifySize(native.cones, [2, 1]);
            testCase.verifyEqual(sum(native.cones), size(native.A, 1));
        end

        function explicitStatesDoNotChangeThePerformanceObjective(testCase)
            problem = localProblem();
            [decision, lifted] = localDecision(problem);
            native = problem.qp.stageProgram;
            hessian = native.P+triu(native.P, 1).';

            sparseValue = 0.5*lifted.'*hessian*lifted+native.q.'*lifted;
            condensedValue = 0.5*decision.'*problem.qp.Hessian*decision+problem.qp.linear.'*decision;

            testCase.verifyEqual(sparseValue, condensedValue, AbsTol=1.0e-12);
            testCase.verifyEqual(problem.metadata.solverCallCount, 1);
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function nativeSolverHandlesAnIndependentQuadraticConeProblem(testCase)
            [point, information] = solveAvoidanceSocpMex(sparse(2, 2, 2, 2, 2), [0; 0], ...
                sparse([0, 0; -2, 0; 0, -1]), [1; -2; -2], [0; 0; 3], [1e-9; 1e-9; 400]);

            testCase.verifyEqual(point, [1; 1], AbsTol=1.0e-5);
            testCase.verifyTrue(any(information.status == [1, 4]));
            testCase.verifyGreaterThan(information.iterations, 0);
        end

        function nativeSolverRejectsNonfiniteDataBeforeEnteringTheLibrary(testCase)
            testCase.verifyError(@() localInvalidSolve([NaN; 0], [0; 0; 3]), ...
                "collisionAvoidanceController:invalidSocpData");
        end

        function nativeSolverRejectsInconsistentConeSizes(testCase)
            testCase.verifyError(@() localInvalidSolve([0; 0], [0; 0; 2]), ...
                "collisionAvoidanceController:invalidSocpData");
        end
    end
end

function problem = localProblem()
    cfg = struct("controller", struct("horizonSteps", 4), ...
        "model", struct("longitudinalInputGain", 0.8));
    ego = struct("position", [0.0; 0.1], "yaw", 0.005, "speed", 8.0, ...
        "longitudinalAccelerationBias", 0.3);
    target = struct("targetId", "lead", "targetPositionInertial", [80.0; 0.0], ...
        "targetVelocityInertial", [5.0; 0.0], "targetAccelerationInertial", [0.0; 0.0], ...
        "targetYawInertial", 0.0, "targetLength", 4.8, "targetWidth", 1.9);
    boundary = struct("origin", [0; 0], "longitudinalDirection", [1; 0], ...
        "lateralDirection", [0; 1], "coefficients", [0, 0, 6], ...
        "parameterRange", [-100, 2000], "safeSideSign", -1, "boundaryId", "left");
    road = struct("centerline", [0, 0; 2000, 0], "boundaries", boundary);
    [~, ~, problem] = collisionAvoidanceController(ego, target, road, cfg, []);
end

function [decision, lifted] = localDecision(problem)
    % An arbitrary input, deliberately not required to be feasible: equality
    % of residuals must hold over the whole decision space, not just at a solve.
    plan = 0.03*sin((1:problem.layout.planCount).');
    decision = [plan; 0.7];
    state = squeeze(pagemtimes(problem.prediction.egoStateMatrix, plan)) ...
        + problem.prediction.egoStateOffset;
    future = state(:, 2:end);
    lifted = [decision; future(:)];
end

function localInvalidSolve(linear, cones)
    [~, ~] = solveAvoidanceSocpMex(sparse(eye(2)), linear, sparse(3, 2), ...
        ones(3, 1), cones, [1e-9; 1e-9; 400]);
end
