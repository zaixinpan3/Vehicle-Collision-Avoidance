classdef sparseAvoidanceQpTest < matlab.unittest.TestCase
%sparseAvoidanceQpTest The sparse lift preserves the physical optimization.

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "tests")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "solver", "clarabel", "matlab")));
        end
    end

    methods (Test)
        function conicTranscriptionPreservesEveryHardRow(testCase)
            problem = localProblem();
            decision = 0.03*sin((1:problem.layout.decisionCount).');
            qp = problem.qp;
            rows = 1:size(qp.inequalityMatrix, 1);
            residual = qp.stageProgram.A(rows, :)*decision-qp.stageProgram.b(rows);
            testCase.verifyEqual(residual, qp.inequalityMatrix*decision-qp.inequalityBound, AbsTol=1e-10);
            testCase.verifyTrue(any(startsWith(qp.geometry.label, "collision:")));
            testCase.verifyTrue(any(startsWith(qp.geometry.label, "road:")));
            testCase.verifyTrue(any(qp.geometry.label == "tireSlip"));
        end

        function anExitCertificateDoesNotImposeEgoRest(testCase)
            problem = localProblem();
            testCase.verifyEmpty(problem.qp.equalityBound);
            testCase.verifyEqual(problem.layout.tailSteps, 0);
            testCase.verifyGreaterThan(problem.prediction.egoStateOffset(4, end), 0);
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function theLorentzConesEncodeThePredictiveClfQuadratics(testCase)
            problem = localProblem();
            [difference, coneMargin] = localConeResiduals(problem);
            testCase.verifyLessThanOrEqual(difference, 1e-8);
            testCase.verifyGreaterThanOrEqual(coneMargin, -1e-8);
            testCase.verifyEqual(sum(problem.qp.stageProgram.cones), size(problem.qp.stageProgram.A, 1));
        end

        function nativeTranscriptionPreservesTheFullPredictiveObjective(testCase)
            problem = localProblem();
            decision = 0.03*sin((1:problem.layout.decisionCount).');
            native = problem.qp.stageProgram;
            hessian = native.P+triu(native.P, 1).';
            testCase.verifyEqual(0.5*decision.'*hessian*decision+native.q.'*decision, ...
                0.5*decision.'*problem.qp.Hessian*decision+problem.qp.linear.'*decision, AbsTol=1e-10);
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
    [ego, target, centerline, cfg] = encounterTestFixture.crossing();
    boundary = struct("origin", [0;0], "longitudinalDirection", [1;0], ...
        "lateralDirection", [0;1], "coefficients", [0,0,6], ...
        "parameterRange", [-200,3000], "safeSideSign", -1, "boundaryId", "left");
    road = struct("centerline", centerline, "boundaries", boundary);
    [~,~,problem] = collisionAvoidanceController(ego,target,road,cfg,[]);
    % Test the physical-decision transcription directly. The separate
    % liftedAvoidanceSocpTest compares it with auxiliary-state programs.
    problem.qp.stageProgram = condensedAvoidanceTestOracle(problem.qp);
end

function [difference, margin] = localConeResiduals(problem)
    program = problem.qp.stageProgram;
    slack = program.b-program.A*problem.decision;
    cursor = program.cones(2);
    difference = 0; margin = inf;
    for index = 1:numel(problem.qp.clf.constraints)
        cone = slack(cursor+(1:10));
        constraint = problem.qp.clf.constraints(index);
        value = constraint.map*problem.decision+constraint.offset;
        residual = problem.decision(problem.layout.relaxationIndex(constraint.stage)) ...
            -norm(constraint.root*value)^2-constraint.linear.'*value-constraint.constant;
        difference = max(difference, abs(cone(1)^2-sum(cone(2:end).^2)-4*residual));
        margin = min(margin, cone(1)-norm(cone(2:end)));
        cursor = cursor+10;
    end
end

function localInvalidSolve(linear, cones)
    [~, ~] = solveAvoidanceSocpMex(sparse(eye(2)), linear, sparse(3, 2), ...
        ones(3, 1), cones, [1e-9; 1e-9; 400]);
end
