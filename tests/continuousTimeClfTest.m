classdef continuousTimeClfTest < matlab.unittest.TestCase
    %continuousTimeClfTest Continuous CLF derivatives and hard-safety separation.

    properties (TestParameter)
        laneCase = {"straight", "curved"};
        invalidRateFraction = {0, -0.1, 1.01, NaN, Inf, [0.1, 0.2], 1i};
    end

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
        end
    end

    methods (Test)
        function theDecayFractionMustBeFiniteAndWithinItsCertifiedRange(testCase, invalidRateFraction)
            testCase.verifyError(@() collisionAvoidanceControllerConfig( ...
                struct("clf", struct("decreaseRateFraction", invalidRateFraction))), ...
                "collisionAvoidanceController:invalidConfiguration");
        end

        function lieDerivativesMatchTheBiasedHeldFlow(testCase, laneCase)
            [problem, cfg, ego] = localProblem(laneCase);
            clf = problem.qp.clf;
            prediction = problem.prediction;
            state = prediction.egoStateOffset(:, 1);
            actuatorInput = [0.013; -0.7];
            step = 1.0e-5;
            plus = localErrorAfterFlow(state, actuatorInput, step, prediction, cfg, ego, clf);
            minus = localErrorAfterFlow(state, actuatorInput, -step, prediction, cfg, ego, clf);
            finiteDifference = (plus.'*clf.lyapunovMatrix*plus ...
                - minus.'*clf.lyapunovMatrix*minus)/(2.0*step);

            derivative = clf.lieDerivativeDrift+clf.lieDerivativeInput*actuatorInput;

            testCase.verifyEqual(derivative, finiteDifference, AbsTol=1.0e-6);
            testCase.verifyEqual(clf.inequalityMatrix(3:end-1), ...
                zeros(1, problem.layout.planCount-2), AbsTol=0.0);
            testCase.verifyEqual(clf.inequalityMatrix(end), -1.0, AbsTol=0.0);
        end

        function theRiccatiCertificateGuaranteesTheConfiguredContinuousRate(testCase)
            [problem, cfg] = localProblem("straight");
            certificate = problem.qp.clf.certificate;
            [stateMatrix, inputMatrix] = ltvBicycleModel.continuousMatrices( ...
                0.0, max(cfg.referenceSpeed, cfg.clf.certificateSpeedFloor), cfg);
            closedLoop = stateMatrix(2:6, 2:6)-inputMatrix(2:6, :)*certificate.feedbackGain;
            lyapunovMatrix = certificate.lyapunovMatrix;

            derivativeMatrix = closedLoop.'*lyapunovMatrix+lyapunovMatrix*closedLoop;
            rateMargin = certificate.decreaseMatrix-problem.qp.clf.decayRate*lyapunovMatrix;

            testCase.verifyEqual(derivativeMatrix, -certificate.decreaseMatrix, AbsTol=1.0e-9);
            testCase.verifyGreaterThan(min(eig(lyapunovMatrix)), 0.0);
            testCase.verifyGreaterThan(min(eig(rateMargin)), 0.0);
            testCase.verifyEqual(certificate.timeDomain, "continuousTime");
        end

        function theContinuousClfDoesNotDependOnTheSamplePeriod(testCase)
            first = localProblem("straight", struct("controller", struct("sampleTime", 0.05)));
            second = localProblem("straight", struct("controller", struct("sampleTime", 0.1)));

            testCase.verifyEqual(first.qp.clf.lyapunovMatrix, second.qp.clf.lyapunovMatrix, AbsTol=0.0);
            testCase.verifyEqual(first.qp.clf.decayRate, second.qp.clf.decayRate, AbsTol=0.0);
            testCase.verifyEqual(first.qp.clf.lieDerivativeDrift, second.qp.clf.lieDerivativeDrift, AbsTol=0.0);
            testCase.verifyEqual(first.qp.clf.lieDerivativeInput, second.qp.clf.lieDerivativeInput, AbsTol=0.0);
        end

        function changingTheClfRateLeavesEveryHardConstraintUnchanged(testCase)
            first = localProblem("straight", struct("clf", struct("decreaseRateFraction", 0.25)));
            second = localProblem("straight", struct("clf", struct("decreaseRateFraction", 0.75)));

            testCase.verifyEqual(second.qp.clf.decayRate, 3.0*first.qp.clf.decayRate, AbsTol=1.0e-12);
            testCase.verifyEqual(first.qp.inequalityMatrix, second.qp.inequalityMatrix, AbsTol=0.0);
            testCase.verifyEqual(first.qp.inequalityBound, second.qp.inequalityBound, AbsTol=0.0);
            testCase.verifyEqual(first.qp.equalityMatrix, second.qp.equalityMatrix, AbsTol=0.0);
            testCase.verifyEqual(first.qp.equalityBound, second.qp.equalityBound, AbsTol=0.0);
            testCase.verifyEqual(first.qp.lowerBound, second.qp.lowerBound, AbsTol=0.0);
            testCase.verifyEqual(first.qp.upperBound, second.qp.upperBound, AbsTol=0.0);
            testCase.verifyEqual(first.qp.inequalityMatrix(:, end), ...
                zeros(size(first.qp.inequalityBound)), AbsTol=0.0);
        end

        function zeroCruiseErrorHasZeroDerivativeAndZeroRelaxation(testCase)
            cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 4)));
            ego = struct("position", [0; 0], "yaw", 0, "speed", cfg.referenceSpeed, ...
                "longitudinalAccelerationBias", 0.3);
            [~, ~, problem] = collisionAvoidanceController(ego, [], [0, 0; 2000, 0], cfg, []);

            testCase.verifyEqual(problem.qp.clf.initialValue, 0.0, AbsTol=0.0);
            testCase.verifyEqual(problem.qp.clf.lieDerivativeDrift, 0.0, AbsTol=0.0);
            testCase.verifyEqual(problem.qp.clf.lieDerivativeInput, [0, 0], AbsTol=0.0);
            testCase.verifyEqual(problem.metadata.clfRelaxation, 0.0, AbsTol=0.0);
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function acceptanceRejectsInsufficientDerivativeRelaxation(testCase)
            [problem, cfg, ego, route] = localProblem("straight");
            [~, lane] = readPlanningInputs(ego, [], route, cfg);
            model = struct("cfg", cfg, "lane", lane, "hasTarget", false, ...
                "egoHalfLength", cfg.vehicle.length/2, "egoHalfWidth", cfg.vehicle.width/2, ...
                "targetHalfLength", 0, "targetHalfWidth", 0);
            decision = problem.decision;
            decision(end) = 0.0;

            accepted = certifyAvoidancePlan(problem.qp, problem.prediction, model, problem.decision);
            rejected = certifyAvoidancePlan(problem.qp, problem.prediction, model, decision);

            testCase.verifyGreaterThan(problem.metadata.clfRelaxation, ...
                100*cfg.solver.constraintTolerance);
            testCase.verifyTrue(accepted.accepted);
            testCase.verifyFalse(rejected.accepted);
            testCase.verifyEqual(rejected.failedConditions, "continuousTimeClf");
            testCase.verifyEqual(rejected.clfViolation, ...
                problem.metadata.clfRelaxation, AbsTol=1.0e-10);
            testCase.verifyEqual(rejected.hardRowViolation, accepted.hardRowViolation, AbsTol=0.0);
        end

        function thePublicHookCanSolveTheStandardQpWithQuadprog(testCase)
            solver = struct("constraintTolerance", 1.0e-9, "optimalityTolerance", 1.0e-9);
            native = localProblem("straight", struct("solver", solver));
            solver.jointFunction = @localQuadprog;
            hooked = localProblem("straight", struct("solver", solver));

            testCase.verifyTrue(hooked.metadata.planCertified);
            testCase.verifyEqual(hooked.metadata.solverCallCount, 1);
            testCase.verifyEqual(hooked.metadata.jointObjectiveValue, ...
                native.metadata.jointObjectiveValue, AbsTol=1.0e-5, RelTol=1.0e-8);
            testCase.verifySize(hooked.decision, [hooked.layout.planCount+1, 1]);
        end
    end
end

function [problem, cfg, ego, route] = localProblem(laneCase, overrides)
    if nargin < 2
        overrides = struct();
    end
    cfg = collisionAvoidanceControllerConfig(overrides);
    cfg.controller.horizonSteps = 4;
    ego = struct("position", [0; 0.1], "yaw", 0.005, "speed", 8.0, ...
        "lateralVelocity", 0.01, "yawRate", 0.002, "longitudinalAccelerationBias", 0.3);
    route = [0, 0; 2000, 0];
    if laneCase == "curved"
        angle = (-0.1:0.01:0.6).';
        route = [500*sin(angle), 500*(1-cos(angle))];
    end
    [~, ~, problem] = collisionAvoidanceController(ego, [], route, cfg, []);
end

function errorState = localErrorAfterFlow(state, actuatorInput, step, prediction, cfg, ego, clf)
    [stateMatrix, inputMatrix, affine] = ltvBicycleModel.stageMatrices( ...
        prediction.scheduleCurvature(1), prediction.scheduleSpeedProfile(1), step, cfg, ...
        prediction.scheduleBrakingRatio(1), ego.longitudinalAccelerationBias);
    nextState = stateMatrix*state+inputMatrix*actuatorInput+affine;
    errorState = clf.errorOffset(:, 1)+nextState(2:6)-state(2:6);
end

function result = localQuadprog(~, program)
    options = optimoptions("quadprog", "Display", "off", ...
        "ConstraintTolerance", 1.0e-9, "OptimalityTolerance", 1.0e-9, "MaxIterations", 400);
    [decision, ~, exitFlag, output] = quadprog(program.H, program.f, ...
        program.A, program.b, program.Aeq, program.beq, program.lb, program.ub, [], options);
    result = struct("decision", decision, "exitFlag", exitFlag, "output", output);
end
