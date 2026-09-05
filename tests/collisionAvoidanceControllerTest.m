classdef collisionAvoidanceControllerTest < matlab.unittest.TestCase
    % collisionAvoidanceControllerTest Hard-CBF/soft-CLF behavior.

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repositoryRoot, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repositoryRoot, "controller")));
        end
    end

    methods (TestMethodSetup)
        function resetController(testCase)
            clear collisionAvoidanceController collisionAvoidanceControllerConfig
            clear formulateTwoStageQp targetPredictionFutureSupport
            clear solveHardCbfClf certifySweptRectangleIntervals
            collisionAvoidanceController("resetNominalTrajectory");
            localJointSolveHook("reset", struct());
            testCase.addTeardown(@() collisionAvoidanceController( ...
                "resetNominalTrajectory"));
        end
    end

    methods (Test)
        function commandUsesSteeringAccelerationInputOrder(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);

            [command, inputPlan] = collisionAvoidanceController( ...
                ego, [], localLane(), cfg);

            testCase.verifyEqual(command.actuatorInputOrder, ...
                ["frontWheelSteeringAngle", "longitudinalAcceleration"]);
            testCase.verifySize(inputPlan, [2, cfg.controller.horizonSteps]);
            testCase.verifyEqual(command.actuatorInput, inputPlan(:, 1), ...
                AbsTol=0.0);
            testCase.verifyEqual(command.actuatorInput, ...
                [command.frontWheelSteeringAngle; ...
                    command.longitudinalAcceleration], AbsTol=0.0);
        end

        function outputCountDoesNotChangeThePlan(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            first = collisionAvoidanceController(ego, [], localLane(), cfg);
            collisionAvoidanceController("resetNominalTrajectory");
            [second, secondPlan] = collisionAvoidanceController( ...
                ego, [], localLane(), cfg);
            collisionAvoidanceController("resetNominalTrajectory");

            [third, thirdPlan, problem] = collisionAvoidanceController( ...
                ego, [], localLane(), cfg);

            testCase.verifyEqual(first, second);
            testCase.verifyEqual(second, third);
            testCase.verifyEqual(secondPlan, thirdPlan, AbsTol=0.0);
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function solvedPlanRespectsConfiguredInputBounds(testCase)
            cfg = localSmallConfiguration();
            cfg.actuation = struct("longitudinalAccelerationMinimum", -8.0, ...
                "longitudinalAccelerationMaximum", 2.0);
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);

            [~, inputPlan] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);

            complete = collisionAvoidanceControllerConfig(cfg);
            testCase.verifyLessThanOrEqual(abs(inputPlan(1, :)), ...
                complete.model.frontWheelSteeringAngleMaximum+1.0e-9);
            testCase.verifyGreaterThanOrEqual(inputPlan(2, :), -8.0-1.0e-9);
            testCase.verifyLessThanOrEqual(inputPlan(2, :), 2.0+1.0e-9);
        end

        function nominalRolloutMatchesTheCondensedPrediction(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.01; 0.001; 15.0; 0.01; 0.0], ...
                [0.0; 0.0]);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, [], localLane(), cfg);

            predicted = squeeze(pagemtimes( ...
                problem.prediction.egoStateMatrix, ...
                problem.qp.linearizationPlan)) ...
                + problem.prediction.egoStateOffset;
            testCase.verifyEqual(problem.qp.nominalState, predicted, ...
                AbsTol=1.0e-11);
        end

        function legacyQpStillReturnsAHardPlan(testCase)
            cfg = localSmallConfiguration();
            cfg.certification = struct("enabled", false);
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);

            [command, inputPlan, problem] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);

            testCase.verifyEqual(problem.qp.obstacleMode, "hard");
            testCase.verifyFalse(problem.metadata.planCertified);
            testCase.verifyEqual(command.actuatorInput, inputPlan(:, 1), ...
                AbsTol=0.0);
            testCase.verifyLessThanOrEqual(max( ...
                problem.qp.inequalityMatrix*problem.decision ...
                    - problem.qp.inequalityBound), 1.0e-6);
        end

        function legacyDisjunctiveSearchStillReturnsAHardPlan(testCase)
            cfg = localSmallConfiguration();
            cfg.certification = struct("enabled", false);
            cfg.disjunctive.nodeBudget = 4;
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);

            testCase.verifyEqual(problem.metadata.selectedCandidate, ...
                "disjunctive");
            testCase.verifyFalse(problem.metadata.planCertified);
            testCase.verifyGreaterThan(problem.metadata.disjunctiveExplored, 0);
            testCase.verifyLessThanOrEqual(max( ...
                problem.qp.inequalityMatrix*problem.decision ...
                    - problem.qp.inequalityBound), 1.0e-6);
        end

        function certifiedModeRejectsFactRelaxationBudget(testCase)
            override = struct("collision", struct("disturbanceBound", 0.01));

            testCase.verifyError( ...
                @() collisionAvoidanceControllerConfig(override), ...
                "collisionAvoidanceController:invalidConfiguration");
        end

        function configurationOmitsActuatorRateLimits(testCase)
            cfg = collisionAvoidanceControllerConfig();

            testCase.verifyFalse(isfield( ...
                cfg.actuation, "frontWheelSteeringRateMaximum"));
            testCase.verifyFalse(isfield( ...
                cfg.actuation, "longitudinalJerkMaximum"));
        end

        function heldActuatorDoesNotConstrainFirstPlan(testCase)
            cfg = localSmallConfiguration();
            state = [0.0; 0.0; 0.0; 15.0; 0.0; 0.0];
            firstEgo = localEgoState(state, [0.0; 0.0]);
            secondEgo = localEgoState(state, [0.3; -1.0]);
            [firstCommand, firstPlan] = collisionAvoidanceController( ...
                firstEgo, [], localLane(), cfg);
            collisionAvoidanceController("resetNominalTrajectory");

            [secondCommand, secondPlan] = collisionAvoidanceController( ...
                secondEgo, [], localLane(), cfg);

            testCase.verifyEqual(secondCommand.actuatorInput, ...
                firstCommand.actuatorInput, AbsTol=1.0e-8);
            testCase.verifyEqual(secondPlan, firstPlan, AbsTol=1.0e-8);
        end

        function brakingTailUsesAccelerationBoundsWithoutJerk(testCase)
            cfg = collisionAvoidanceControllerConfig();
            steps = kinematicBrakingTail("steps", cfg);

            profile = kinematicBrakingTail( ...
                "profile", cfg, steps, 1.0);
            speed = 1.0+cfg.controller.sampleTime*cumsum(profile);

            testCase.verifyEqual(profile(1), ...
                -cfg.terminal.backupDeceleration, AbsTol=1.0e-12);
            testCase.verifyGreaterThanOrEqual(profile, ...
                -cfg.terminal.backupDeceleration-1.0e-12);
            testCase.verifyLessThanOrEqual(profile, 1.0e-12);
            testCase.verifyGreaterThanOrEqual(speed, -1.0e-12);
            testCase.verifyEqual(speed(end), 0.0, AbsTol=1.0e-12);
            testCase.verifyEqual(profile(end), 0.0, AbsTol=1.0e-12);
        end

        function hardCbfPlanUsesJointClfInputObjective(testCase)
            cfg = localSmallConfiguration();
            cfg.clf = struct("relaxationWeight", 75.0);
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);

            testCase.verifyEqual(problem.qp.obstacleMode, "certified");
            testCase.verifyEqual(problem.layout.slackCount, 0);
            testCase.verifyEmpty(problem.layout.slackIndex);
            testCase.verifyEmpty(problem.metadata.safetySlackProfile);
            testCase.verifyTrue(problem.metadata.cbfConstraintsHard);
            testCase.verifyTrue(problem.metadata.hardCbfSatisfied);
            testCase.verifyLessThanOrEqual( ...
                problem.metadata.clfExactResidual, 2.0e-5);
            relaxationIndex = problem.layout.relaxationIndex(1);
            testCase.verifyEqual( ...
                problem.qp.Hessian(relaxationIndex, relaxationIndex), ...
                0.0, AbsTol=0.0);
            testCase.verifyEqual(problem.qp.linear(relaxationIndex), ...
                75.0, AbsTol=0.0);
            testCase.verifyEqual(problem.metadata.clfRelaxationCost, ...
                75.0*problem.metadata.clfRelaxation, AbsTol=1.0e-8);
            testCase.verifyEqual(problem.metadata.jointObjectiveValue, ...
                problem.metadata.inputDeviationCost ...
                    + problem.metadata.clfRelaxationCost, ...
                AbsTol=1.0e-8);
            testCase.verifyEqual(problem.metadata.solverCallCount, 1);
            testCase.verifyTrue(problem.metadata.terminalInvariantCertified);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyEqual(problem.metadata.factRelaxedNodes, 0);
        end

        function freshHardSolutionSkipsDuplicateCertificate(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);

            testCase.verifyFalse( ...
                problem.metadata.postSolveCertificationPerformed);
            testCase.verifyEqual(problem.metadata.certificateSource, ...
                "hardConstrainedOptimization");
            testCase.verifyFalse(isfield(problem.metadata, ...
                "exactPredictionAssumptionsHold"));
            testCase.verifyFalse(isfield(problem.metadata, ...
                "routeCoordinateValid"));
            testCase.verifyFalse(isfield(problem.metadata, ...
                "nodeClearanceMargin"));
            testCase.verifyFalse(isfield(problem.metadata, ...
                "sweptCollisionCertificate"));
        end

        function certifiedMultiStartUsesNoElasticRefinement(testCase)
            cfg = struct();
            cfg.controller = struct("sampleTime", 0.05, ...
                "horizonSteps", 4);
            cfg.disjunctive = struct("nodeBudget", 0);
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);

            testCase.verifyEqual(problem.metadata.candidateCount, 2);
            testCase.verifyEqual( ...
                problem.metadata.candidateRefinementRungs, [0.0, 0.0]);
            testCase.verifyEqual(problem.metadata.solverCallCount, 2);
            testCase.verifyEqual(problem.layout.slackCount, 0);
            testCase.verifyTrue(problem.metadata.hardCbfSatisfied);
        end

        function initialHardCbfInfeasibilityDeclaresFailure(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 5.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("overlap", [0.0; 0.0], [20.0; 0.0]);

            testCase.verifyError(@() collisionAvoidanceController( ...
                ego, target, localLane(), cfg), ...
                "collisionAvoidanceController:noSolution");
        end

        function predictedFutureIntersectionFailsHardTerminalEnvelope( ...
                testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("oncoming", [100.0; 0.0], [-5.0; 0.0]);

            testCase.verifyError(@() collisionAvoidanceController( ...
                ego, target, localLane(), cfg), ...
                "collisionAvoidanceController:noSolution");
        end

        function curvedPredictionIsNotRejectedByMotionClass(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localPredictedTarget( ...
                "curved", [120.0; 60.0], 0.0, 8.0, 0.0, 0.02);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);

            testCase.verifyTrue(problem.metadata.terminalInvariantCertified);
            testCase.verifyEqual(problem.metadata.terminalContinuationAxis, ...
                "predictedTrajectorySupport");
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function turningBrakingPredictionIsNotRejectedByMotionClass( ...
                testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localPredictedTarget( ...
                "turningBrake", [140.0; 50.0], 0.0, 8.0, -1.0, 0.02);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);

            testCase.verifyTrue(problem.metadata.terminalInvariantCertified);
            testCase.verifyEqual(problem.metadata.terminalContinuationAxis, ...
                "predictedTrajectorySupport");
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function curvedPredictionOnPiecewiseLaneUsesTerminalSegment( ...
                testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localPredictedTarget( ...
                "curvedRoute", [120.0; 60.0], 0.0, 8.0, 0.0, 0.02);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localPiecewiseLane(), cfg);

            testCase.verifyTrue(problem.metadata.terminalInvariantCertified);
            testCase.verifyGreaterThan( ...
                problem.metadata.terminalSegmentIndex, 1);
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function exactContinuationShiftsScheduleAndTarget(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);
            [command, ~, first] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);
            nextEgo = localNextEgo(first, command.actuatorInput);
            nextTarget = localTarget("lead", [100.25; 0.0], [5.0; 0.0]);

            [~, ~, second] = collisionAvoidanceController( ...
                nextEgo, nextTarget, localLane(), cfg);

            testCase.verifyTrue(second.metadata.scheduleShifted);
            testCase.verifyTrue(second.metadata.targetContinuationShifted);
            testCase.verifyEqual(second.prediction.stageMatrixA(:, :, 1), ...
                first.prediction.stageMatrixA(:, :, 2), AbsTol=0.0);
            testCase.verifyEqual( ...
                second.prediction.scheduleStation(1:end-1), ...
                first.prediction.scheduleStation(2:end), AbsTol=0.0);
        end

        function failedImprovementExecutesStoredPlanTwice(testCase)
            cfg = localSmallConfiguration();
            cfg.solver.jointFunction = @localJointSolveHook;
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);
            [firstCommand, firstPlan, first] = ...
                collisionAvoidanceController(ego, target, localLane(), cfg);
            secondEgo = localNextEgo(first, firstCommand.actuatorInput);
            secondTarget = localTarget( ...
                "lead", [100.25; 0.0], [5.0; 0.0]);

            [secondCommand, secondPlan, second] = ...
                collisionAvoidanceController( ...
                    secondEgo, secondTarget, localLane(), cfg);
            thirdEgo = localNextEgo(second, secondCommand.actuatorInput);
            thirdTarget = localTarget( ...
                "lead", [100.50; 0.0], [5.0; 0.0]);
            [thirdCommand, ~, third] = collisionAvoidanceController( ...
                thirdEgo, thirdTarget, localLane(), cfg);

            testCase.verifyTrue(second.metadata.fallbackUsed);
            testCase.verifyTrue(third.metadata.fallbackUsed);
            testCase.verifyTrue( ...
                second.metadata.postSolveCertificationPerformed);
            testCase.verifyEqual(second.metadata.certificateSource, ...
                "shiftedStoredPlan");
            testCase.verifyEqual(secondCommand.actuatorInput, ...
                firstPlan(:, 2), AbsTol=0.0);
            testCase.verifyEqual(thirdCommand.actuatorInput, ...
                firstPlan(:, 3), AbsTol=0.0);
            testCase.verifyEqual(secondPlan(:, 1), firstPlan(:, 2), ...
                AbsTol=0.0);
        end

        function curvedContinuationShiftsIntoCertifiedFallback(testCase)
            cfg = localSmallConfiguration();
            cfg.solver.jointFunction = @localJointSolveHook;
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localPredictedTarget( ...
                "curvedFallback", [120.0; 60.0], ...
                0.0, 8.0, 0.0, 0.02);
            [command, firstPlan, first] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);
            nextEgo = localNextEgo(first, command.actuatorInput);
            nextTarget = localAdvancePredictedTarget( ...
                target, cfg.controller.sampleTime);

            [secondCommand, secondPlan, second] = ...
                collisionAvoidanceController( ...
                    nextEgo, nextTarget, localLane(), cfg);

            testCase.verifyTrue(second.metadata.targetContinuationShifted);
            testCase.verifyTrue(second.metadata.fallbackUsed);
            testCase.verifyTrue(second.metadata.terminalInvariantCertified);
            testCase.verifyEqual(secondCommand.actuatorInput, ...
                firstPlan(:, 2), AbsTol=0.0);
            testCase.verifyEqual(secondPlan(:, 1), firstPlan(:, 2), ...
                AbsTol=0.0);
        end

        function identityChangeCannotReuseStoredPlan(testCase)
            cfg = localSmallConfiguration();
            cfg.solver.jointFunction = @localJointSolveHook;
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);
            [command, ~, first] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);
            nextEgo = localNextEgo(first, command.actuatorInput);
            replacement = localTarget( ...
                "replacement", [100.25; 0.0], [5.0; 0.0]);

            testCase.verifyError(@() collisionAvoidanceController( ...
                nextEgo, replacement, localLane(), cfg), ...
                "collisionAvoidanceController:optimizationFailure");
        end

        function targetShiftMismatchCannotReuseStoredPlan(testCase)
            cfg = localSmallConfiguration();
            cfg.solver.jointFunction = @localJointSolveHook;
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);
            [command, ~, first] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);
            nextEgo = localNextEgo(first, command.actuatorInput);
            inconsistent = localTarget( ...
                "lead", [101.0; 0.0], [5.0; 0.0]);

            testCase.verifyError(@() collisionAvoidanceController( ...
                nextEgo, inconsistent, localLane(), cfg), ...
                "collisionAvoidanceController:optimizationFailure");
        end

        function sweptCheckRejectsBetweenNodeCrossing(testCase)
            egoPose = [-2.0, 2.0; 0.0, 0.0; 0.0, 0.0];
            targetPose = zeros(3, 2);

            certificate = certifySweptRectangleIntervals( ...
                egoPose, targetPose, [0.4; 0.2; 0.4; 0.2], 0.0, ...
                struct("maxDepth", 10));

            testCase.verifyFalse(certificate.certified);
            testCase.verifyEqual(certificate.failureReason, ...
                "sampledCollision");
            testCase.verifyLessThan(certificate.minimumMargin, 0.0);
        end

        function sweptCheckCertifiesSeparatedMotion(testCase)
            egoPose = [-2.0, 2.0; 2.0, 2.0; 0.0, 0.0];
            targetPose = zeros(3, 2);

            certificate = certifySweptRectangleIntervals( ...
                egoPose, targetPose, [0.4; 0.2; 0.4; 0.2], 0.1, ...
                struct("maxDepth", 10));

            testCase.verifyTrue(certificate.certified);
            testCase.verifyGreaterThan(certificate.minimumMargin, 0.0);
        end
    end
end

function cfg = localSmallConfiguration()
    cfg = struct();
    cfg.controller = struct("sampleTime", 0.05, "horizonSteps", 4);
    cfg.disjunctive = struct("nodeBudget", 0);
    cfg.sequentialConvex = struct("penaltySchedule", []);
end

function lane = localLane()
    lane = [0.0, 0.0; 2000.0, 0.0];
end

function lane = localPiecewiseLane()
    lane = [0.0, 0.0; 25.0, 0.0; 2000.0, 200.0];
end

function ego = localEgoState(state, heldInput)
    ego = struct( ...
        "positionX", state(1), ...
        "positionY", state(2), ...
        "yawAngle", state(3), ...
        "longitudinalVelocity", state(4), ...
        "lateralVelocity", state(5), ...
        "yawRate", state(6), ...
        "heldActuatorInput", heldInput);
end

function target = localTarget(id, position, velocity)
    target = struct( ...
        "targetId", id, ...
        "targetPositionInertial", position, ...
        "targetVelocityInertial", velocity, ...
        "targetAccelerationInertial", [0.0; 0.0], ...
        "targetYawInertial", atan2(velocity(2), velocity(1)), ...
        "targetLength", 4.8, ...
        "targetWidth", 1.9);
end

function target = localPredictedTarget( ...
        id, position, courseAngle, speed, ...
        tangentialAcceleration, curvature)
    courseDirection = [cos(courseAngle); sin(courseAngle)];
    courseNormal = [-courseDirection(2); courseDirection(1)];
    yawRate = curvature*speed;
    acceleration = tangentialAcceleration*courseDirection ...
        + curvature*speed^2*courseNormal;
    target = struct( ...
        "targetId", id, ...
        "targetPositionInertial", position, ...
        "targetVelocityInertial", speed*courseDirection, ...
        "targetAccelerationInertial", acceleration, ...
        "targetYawInertial", courseAngle, ...
        "targetYawRate", yawRate, ...
        "targetLength", 4.8, ...
        "targetWidth", 1.9);
end

function next = localAdvancePredictedTarget(target, sampleTime)
    velocity = target.targetVelocityInertial;
    speed = norm(velocity);
    courseAngle = target.targetYawInertial;
    courseDirection = [cos(courseAngle); sin(courseAngle)];
    courseNormal = [-courseDirection(2); courseDirection(1)];
    tangentialAcceleration = ...
        dot(target.targetAccelerationInertial, courseDirection);
    curvature = target.targetYawRate/speed;
    arcLength = speed*sampleTime ...
        + 0.5*tangentialAcceleration*sampleTime^2;
    turnAngle = curvature*arcLength;
    displacement = courseDirection*(sin(turnAngle)/curvature) ...
        + courseNormal*((1.0-cos(turnAngle))/curvature);
    next = localPredictedTarget( ...
        target.targetId, ...
        target.targetPositionInertial+displacement, ...
        courseAngle+turnAngle, ...
        speed+tangentialAcceleration*sampleTime, ...
        tangentialAcceleration, curvature);
end

function ego = localNextEgo(problem, appliedInput)
    initial = problem.prediction.egoStateOffset(:, 1);
    state = problem.prediction.stageMatrixA(:, :, 1)*initial ...
        + problem.prediction.stageMatrixB(:, :, 1)*appliedInput ...
        + problem.prediction.stageAffine(:, 1);
    ego = localEgoState(state, appliedInput);
end

function solve = localJointSolveHook(phase, problem)
    persistent solveCount
    if isempty(solveCount)
        solveCount = 0;
    end
    if string(phase) == "reset"
        solveCount = 0;
        solve = struct();
        return;
    end
    solveCount = solveCount+1;
    if solveCount <= 1
        solve = problem.defaultSolver();
        return;
    end
    solve = struct( ...
        "decision", zeros(0, 1), ...
        "objective", inf, ...
        "exitFlag", 0, ...
        "output", struct("message", "deliberate test failure"));
end
