classdef collisionAvoidanceControllerTest < matlab.unittest.TestCase
    % collisionAvoidanceControllerTest Hard-CBF/soft-CLF behavior.

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repositoryRoot, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repositoryRoot, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repositoryRoot, "scripts")));
        end
    end

    methods (TestMethodSetup)
        function resetController(testCase)
            clear collisionAvoidanceController collisionAvoidanceControllerConfig
            clear formulateAvoidanceProblem targetPrediction
            clear solveHardCbfClf
            collisionAvoidanceController("resetNominalTrajectory");
            localJointSolveHook("reset", struct());
            testCase.addTeardown(@() collisionAvoidanceController( ...
                "resetNominalTrajectory"));
        end
    end

    methods (Test)
        function commandUsesSteeringBrakingRatioInputOrder(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);

            [command, inputPlan] = collisionAvoidanceController( ...
                ego, [], localLane(), cfg);

            testCase.verifyEqual(command.actuatorInputOrder, ...
                ["frontWheelSteeringAngle", "brakingRatio"]);
            testCase.verifySize(inputPlan, [2, cfg.controller.horizonSteps]);
            testCase.verifyEqual(command.actuatorInput, inputPlan(:, 1), ...
                AbsTol=0.0);
            testCase.verifyEqual(command.actuatorInput, ...
                [command.frontWheelSteeringAngle; ...
                    command.brakingRatio], AbsTol=0.0);
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
            testCase.verifyFalse(isfield(problem.metadata, "candidateCount"));
            testCase.verifyEqual(problem.metadata.solverCallCount, 1);
        end

        function solvedPlanRespectsConfiguredInputBounds(testCase)
            cfg = localSmallConfiguration();
            cfg.actuation = struct("brakingRatioMinimum", -1.0, ...
                "brakingRatioMaximum", 0.25);
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);

            [~, inputPlan] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);

            complete = collisionAvoidanceControllerConfig(cfg);
            testCase.verifyLessThanOrEqual(abs(inputPlan(1, :)), ...
                complete.model.frontWheelSteeringAngleMaximum+1.0e-9);
            testCase.verifyGreaterThanOrEqual(inputPlan(2, :), -1.0-1.0e-9);
            testCase.verifyLessThanOrEqual(inputPlan(2, :), 0.25+1.0e-9);
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
            steps = ltvBicycleModel.brakingSchedule("steps", cfg);

            profile = ltvBicycleModel.brakingSchedule( ...
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

            testCase.verifyEqual(problem.layout.relaxationCount, 1);
            testCase.verifyEqual(problem.layout.decisionCount, ...
                problem.layout.planCount+1);
            testCase.verifyTrue(problem.metadata.cbfConstraintsHard);
            testCase.verifyTrue(problem.metadata.hardCbfSatisfied);
            testCase.verifyLessThanOrEqual( ...
                problem.metadata.clfDerivativeResidual, 2.0e-5);
            relaxationIndex = problem.layout.relaxationIndex(1);
            testCase.verifyEqual( ...
                problem.qp.Hessian(relaxationIndex, relaxationIndex), ...
                150.0, AbsTol=0.0);
            testCase.verifyEqual(problem.qp.linear(relaxationIndex), ...
                0.0, AbsTol=0.0);
            testCase.verifyEqual(problem.metadata.clfRelaxationCost, ...
                75.0*problem.metadata.clfRelaxation^2, AbsTol=1.0e-8);
            testCase.verifyEqual(problem.metadata.jointObjectiveValue, ...
                problem.metadata.inputDeviationCost ...
                    + problem.metadata.clfRelaxationCost, ...
                AbsTol=1.0e-8);
            testCase.verifyEqual(problem.metadata.solverCallCount, 1);
            testCase.verifyTrue(problem.metadata.terminalPredictionCertified);
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function freshSolutionIsCheckedBeforeCommit(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);

            testCase.verifyTrue( ...
                problem.metadata.postSolveCertificationPerformed);
            testCase.verifyEqual(problem.metadata.certificateSource, ...
                "checkedOptimization");
            testCase.verifyTrue(problem.metadata.exactPredictionAssumptionsHold);
            testCase.verifyTrue(problem.metadata.routeCoordinateValid);
            testCase.verifyGreaterThan(problem.metadata.nodeClearanceMargin, 0.0);
            testCase.verifyEqual(problem.metadata.collisionDiscretization, ...
                "predictionNodesOnly");
        end

        function terminalRestIsSatisfiedToRoundoff(testCase)
            cfg = localSmallConfiguration();
            cfg.controller.horizonSteps = 24;
            cfg.solver = struct("constraintTolerance", 1.0e-6, ...
                "optimalityTolerance", 1.0e-6);
            ego = localEgoState([15.0; 0.03; 0.005; ...
                14.982; -0.001; 0.0], [0.0; 0.0]);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, [], localLane(), cfg, []);

            testCase.verifyLessThan(problem.metadata.acceptance.equalityViolation, 1.0e-10);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyEqual(problem.metadata.solverCallCount, 1);
        end

        function repeatedMeasuredStateReadmissionRetainsCruiseScheduling(testCase)
            cfg = localSmallConfiguration();
            certificate = [];
            heldInput = [0.0; 0.0];
            measuredSpeed = 14.98;
            for sample = 0:8
                ego = localEgoState([0.05*sample*measuredSpeed; ...
                    0.0; 0.0; measuredSpeed; 0.0; 0.0], heldInput);
                [command, ~, problem, certificate] = collisionAvoidanceController( ...
                    ego, [], localLane(), cfg, certificate);
                heldInput = command.actuatorInput;

                testCase.verifyEqual(problem.prediction.scheduleSpeedProfile(1), ...
                    measuredSpeed, AbsTol=1.0e-12);
                testCase.verifyGreaterThan(command.actuatorInput(2), -0.1);
                testCase.verifyTrue(problem.metadata.planCertified);
                testCase.verifyEqual(problem.metadata.solverCallCount, 1);
            end
        end

        function readmissionModelDoesNotDependOnUnexecutedOptimizedSpeeds(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], [0.0; 0.0]);
            near = localTarget("lead", [18.0; 0.0], [5.0; 0.0]);
            far = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);
            [~, ~, ~, nearCertificate] = collisionAvoidanceController( ...
                ego, near, localLane(), cfg, []);
            [~, ~, ~, farCertificate] = collisionAvoidanceController( ...
                ego, far, localLane(), cfg, []);
            testCase.verifyGreaterThan(max(abs(nearCertificate.predictedState(4, :) ...
                - farCertificate.predictedState(4, :))), 0.01);

            measured = localEgoState([0.75; 0.0; 0.0; 14.98; 0.0; 0.0], [0.0; 0.0]);
            target = localTarget("lead", [100.25; 0.0], [5.0; 0.0]);
            [~, ~, fromNear] = collisionAvoidanceController( ...
                measured, target, localLane(), cfg, nearCertificate);
            [~, ~, fromFar] = collisionAvoidanceController( ...
                measured, target, localLane(), cfg, farCertificate);

            testCase.verifyTrue(fromNear.metadata.continuationReadmission);
            testCase.verifyTrue(fromFar.metadata.continuationReadmission);
            testCase.verifyTrue(fromNear.metadata.planCertified);
            testCase.verifyTrue(fromFar.metadata.planCertified);
            testCase.verifyEqual(fromNear.prediction.stageMatrixA, ...
                fromFar.prediction.stageMatrixA, AbsTol=0.0);
            testCase.verifyEqual(fromNear.prediction.stageMatrixB, ...
                fromFar.prediction.stageMatrixB, AbsTol=0.0);
        end

        function aChangingYieldReferencePreservesTheSafetyContinuation(testCase)
            cfg = localSmallConfiguration();
            cfg.solver.jointFunction = @localJointSolveHook;
            ego = localEgoState([46.5; 0.0; 0.0; 15.0; 0.0; 0.0], [0.0; 0.0]);
            target = localTarget("crossing", [90.0; -23.2], [0.0; 8.0]);
            [command, plan, first, certificate] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg, []);
            nextEgo = localNextEgo(first, command.actuatorInput);
            nextTarget = localTarget("crossing", [90.0; -22.8], [0.0; 8.0]);

            [nextCommand, ~, second] = collisionAvoidanceController( ...
                nextEgo, nextTarget, localLane(), cfg, certificate);

            testCase.verifyTrue(first.metadata.crossingYieldActive);
            testCase.verifyTrue(second.metadata.certificateCompatible);
            testCase.verifyTrue(second.metadata.fallbackUsed);
            testCase.verifyTrue(second.metadata.planCertified);
            testCase.verifyGreaterThan(abs(first.metadata.performanceReferenceSpeed ...
                - second.metadata.performanceReferenceSpeed), 1.0e-4);
            testCase.verifyEqual(nextCommand.actuatorInput, plan(:, 2), AbsTol=0.0);
        end

        function targetConstraintsUseOneCertificatePreservingSolve(testCase)
            cfg = struct();
            cfg.controller = struct("sampleTime", 0.05, ...
                "horizonSteps", 4);
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);

            testCase.verifyFalse(isfield(problem.metadata, "candidateLabels"));
            testCase.verifyEqual(problem.metadata.solverCallCount, 1);
            testCase.verifyEqual(problem.layout.relaxationCount, 1);
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

        function remoteOncomingTargetDoesNotRequireAvoidingItsInfiniteRay(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], [0.0; 0.0]);
            target = localTarget("oncoming", [100.0; 0.0], [-5.0; 0.0]);

            [~, ~, problem, certificate] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg, []);

            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyLessThan(abs(certificate.predictedState(2, end)), 0.1);
            testCase.verifyFalse(problem.metadata.terminalInvariantCertified);
            testCase.verifyLessThan(problem.metadata.terminalRestResidual, 1.0e-6);
            testCase.verifyGreaterThanOrEqual( ...
                problem.metadata.terminalInvariantMargin, -1.0e-6);
            testCase.verifyEqual(problem.metadata.solverCallCount, 1);

        end

        function curvedPredictionIsNotRejectedByMotionClass(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localPredictedTarget( ...
                "curved", [120.0; 60.0], 0.0, 8.0, 0.0, 0.02);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);

            testCase.verifyTrue(problem.metadata.terminalPredictionCertified);
            testCase.verifyEqual(problem.metadata.terminalContinuationAxis, ...
                "finitePredictionNode");
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

            testCase.verifyTrue(problem.metadata.terminalPredictionCertified);
            testCase.verifyEqual(problem.metadata.terminalContinuationAxis, ...
                "finitePredictionNode");
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function curvedPredictionUsesABoundedTerminalChart( ...
                testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localPredictedTarget( ...
                "curvedRoute", [120.0; 60.0], 0.0, 8.0, 0.0, 0.02);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localPiecewiseLane(), cfg);

            testCase.verifyTrue(problem.metadata.terminalPredictionCertified);
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
            testCase.verifyTrue(second.metadata.terminalPredictionCertified);
            testCase.verifyEqual(secondCommand.actuatorInput, ...
                firstPlan(:, 2), AbsTol=0.0);
            testCase.verifyEqual(secondPlan(:, 1), firstPlan(:, 2), ...
                AbsTol=0.0);
        end

        function olderInputContractCannotReuseStoredPlan(testCase)
            cfg = localSmallConfiguration();
            cfg.solver.jointFunction = @localJointSolveHook;
            ego = localEgoState([0; 0; 0; 15; 0; 0], [0; 0]);
            [command, ~, first, certificate] = collisionAvoidanceController( ...
                ego, [], localLane(), cfg, []);
            nextEgo = localNextEgo(first, command.actuatorInput);
            certificate.version = 5;

            testCase.verifyError(@() collisionAvoidanceController( ...
                nextEgo, [], localLane(), cfg, certificate), ...
                "collisionAvoidanceController:optimizationFailure");
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

        function changedTargetPredictionRequiresFreshCertification(testCase)
            cfg = localSmallConfiguration();
            % This witness remains feasible under speed rescheduling only
            % in the zero-road-load longitudinal model used by this fixture.
            cfg.roadLoad = struct("dragCoefficient", 0, "rollingCoefficient", 0);
            cfg.solver.jointFunction = @localJointSolveHook;
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("lead", [100.0; 0.0], [5.0; 0.0]);
            [command, ~, first] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);
            nextEgo = localNextEgo(first, command.actuatorInput);
            inconsistent = localTarget( ...
                "lead", [101.0; 0.0], [5.0; 0.0]);

            [~, ~, second] = collisionAvoidanceController( ...
                nextEgo, inconsistent, localLane(), cfg);

            testCase.verifyFalse(second.metadata.certificateCompatible);
            testCase.verifyTrue(second.metadata.continuationReadmission);
            testCase.verifyTrue(second.metadata.planCertified);
            testCase.verifyEqual(second.metadata.certificateSource, ...
                "revalidatedContinuation");
        end

        function rescheduledRoadLoadRequiresRecheckingTheEntireCarriedTail(testCase)
            cfg = localSmallConfiguration();
            cfg.solver.jointFunction = @localJointSolveHook;
            ego = localEgoState([0; 0; 0; 15; 0; 0], [0; 0]);
            target = localTarget("lead", [100; 0], [5; 0]);
            [command, ~, first] = collisionAvoidanceController(ego, target, localLane(), cfg);
            nextEgo = localNextEgo(first, command.actuatorInput);
            inconsistent = localTarget("lead", [101; 0], [5; 0]);

            testCase.verifyError(@() collisionAvoidanceController( ...
                nextEgo, inconsistent, localLane(), cfg), ...
                "collisionAvoidanceController:optimizationFailure");
        end

        function nodeSafeFallbackAcceptsBetweenNodeCrossing(testCase)
            % Synthetic fast crossing isolates the discrete-node convention:
            % the target crosses the ego between two separated nodes.
            cfg = localSmallConfiguration();
            cfg.solver.jointFunction = @localJointSolveHook;
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("crossing", [1.125; 30.0], [0.0; -400.0]);
            [command, firstPlan, first] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg);
            nextEgo = localNextEgo(first, command.actuatorInput);
            nextTarget = localTarget( ...
                "crossing", [1.125; 10.0], [0.0; -400.0]);

            [nextCommand, ~, next] = collisionAvoidanceController( ...
                nextEgo, nextTarget, localLane(), cfg);

            midpointEgo = 0.5*(next.prediction.egoStateOffset(1:3, 1) ...
                + next.prediction.stageMatrixA(1:3, :, 1) ...
                    * next.prediction.egoStateOffset(:, 1) ...
                + next.prediction.stageMatrixB(1:3, :, 1) ...
                    * nextCommand.actuatorInput ...
                + next.prediction.stageAffine(1:3, 1));
            midpointDistance = rectangleConfigurationDistance( ...
                midpointEgo(1:2), midpointEgo(3), ...
                [1.125; 0.0], -pi/2.0, [2.4; 0.95; 2.4; 0.95]);
            testCase.verifyLessThan(midpointDistance, 0.0);
            testCase.verifyTrue(next.metadata.fallbackUsed);
            testCase.verifyTrue(next.metadata.planCertified);
            testCase.verifyTrue(next.metadata.postSolveCertificationPerformed);
            testCase.verifyGreaterThan(next.metadata.nodeClearanceMargin, 0.0);
            testCase.verifyEqual(nextCommand.actuatorInput, firstPlan(:, 2), ...
                AbsTol=0.0);
            testCase.verifyEqual(next.metadata.collisionDiscretization, ...
                "predictionNodesOnly");
        end

        function explicitCertificateSurvivesResetOfConvenienceState(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], [0.0; 0.0]);
            [command, ~, first, certificate] = collisionAvoidanceController( ...
                ego, [], localLane(), cfg, []);
            next = localNextEgo(first, command.actuatorInput);
            collisionAvoidanceController("resetNominalTrajectory");

            [~, ~, second] = collisionAvoidanceController( ...
                next, [], localLane(), cfg, certificate);

            testCase.verifyTrue(second.metadata.carriedWitnessFeasible);
            testCase.verifyTrue(second.metadata.certificateCompatible);
        end

        function repeatedFallbackReachesRestThroughTheFormerHandoff(testCase)
            trace = localFallbackToRest();

            testCase.verifyLessThan(trace.maximumStateError, 2.0e-6);
            testCase.verifyLessThan(norm(trace.finalState(4:6), inf), 2.0e-6);
            testCase.verifyTrue(all(trace.fallbackUsed(2:end)));
            testCase.verifyEqual(trace.solverCalls, ones(size(trace.solverCalls)));
            testCase.verifyGreaterThan(trace.sampleCount, trace.headSteps+trace.tailSteps);
        end

        function carriedSteeringAvoidsAnOncomingTargetWithinRoadBounds(testCase)
            result = runCertificateContinuationScenario();

            testCase.verifyGreaterThanOrEqual(result.minimumRectangleClearance, ...
                result.clearanceRequirement-1.0e-6);
            testCase.verifyGreaterThanOrEqual(result.minimumRoadMargin, -1.0e-6);
            testCase.verifyGreaterThan(result.maximumLateralDisplacement, 0.65);
            testCase.verifyLessThan(result.terminalRestResidual, 1.0e-6);
            testCase.verifyLessThan(result.maximumShiftError, 2.0e-6);
            testCase.verifyTrue(all(result.fallbackUsed(2:end)));
            testCase.verifyEqual(result.actualOptimizationCount, 1);
        end

        function rotatedInertialFramesRetainTheRectangleCertificate(testCase)
            result = runCertificateContinuationScenario(pi);

            testCase.verifyGreaterThanOrEqual(result.minimumRectangleClearance, ...
                result.clearanceRequirement-1.0e-6);
            testCase.verifyGreaterThanOrEqual(result.minimumRoadMargin, -1.0e-6);
            testCase.verifyLessThan(result.maximumShiftError, 2.0e-6);
            testCase.verifyTrue(all(result.fallbackUsed(2:end)));
        end

        function positiveSolverStatusCannotBypassCommitChecks(testCase)
            cfg = localSmallConfiguration();
            cfg.solver.jointFunction = @localUnsafeSuccess;
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], [0.0; 0.0]);

            testCase.verifyError(@() collisionAvoidanceController( ...
                ego, [], localLane(), cfg, []), ...
                "collisionAvoidanceController:optimizationFailure");
        end

        function unsafeSuccessfulImprovementUsesTheCarriedInput(testCase)
            cfg = localSmallConfiguration();
            cfg.solver.jointFunction = @localUnsafeAfterFirst;
            localUnsafeAfterFirst("reset", struct());
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], [0.0; 0.0]);
            [command, plan, first, certificate] = collisionAvoidanceController( ...
                ego, [], localLane(), cfg, []);

            [next, ~, problem] = collisionAvoidanceController( ...
                localNextEgo(first, command.actuatorInput), [], ...
                localLane(), cfg, certificate);

            testCase.verifyTrue(problem.metadata.fallbackUsed);
            testCase.verifyEqual(next.actuatorInput, plan(:, 2), AbsTol=0.0);
            testCase.verifyEqual(problem.metadata.solverCallCount, 1);
        end

        function uncertainDepartingTargetUsesOnlyItsFiniteForecast(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], [0.0; 0.0]);
            target = localTarget("uncertain", [100.0; 0.0], [5.0; 0.0]);
            target.targetVelocityInertialErrorBound = [0.1; 0.1];

            [~, ~, problem, certificate] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyFalse(problem.metadata.terminalInvariantCertified);
            testCase.verifyFalse(isfield(certificate.targetHorizon, "terminalFuturePositionSupport"));
            testCase.verifyGreaterThan(certificate.targetHorizon.targetPositionErrorBound(:, end), 0);
        end

        function anUnpublishedTargetLeavesNoCarriedCollisionConstraints(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0; 0; 0; 15; 0; 0], [0; 0]);
            target = localTarget("departing", [100; 0], [5; 0]);
            [command, ~, first, stored] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg, []);
            nextEgo = localNextEgo(first, command.actuatorInput);
            nextEgo.targetEstimates = struct([]);

            [~, ~, second, fresh] = collisionAvoidanceController( ...
                nextEgo, [], localLane(), cfg, stored);

            testCase.verifyFalse(second.metadata.hasTarget);
            testCase.verifyEqual(second.metadata.collisionImposedCount, 0);
            testCase.verifyEqual(second.metadata.rowCounts.collision, 0);
            testCase.verifyEqual(fresh.episodeIdentity.targetKey, "");
            testCase.verifyTrue(second.metadata.planCertified);
        end

        function aNewlyUnsafeLastTargetNodeCannotReuseTheOldFallback(testCase)
            cfg = collisionAvoidanceControllerConfig(localSmallConfiguration());
            cfg.solver.jointFunction = @localJointSolveHook;
            ego = localEgoState([0; 0; 0; 0; 0; 0], [0; 0]);
            steps = cfg.controller.horizonSteps+ltvBicycleModel.brakingSchedule("steps", cfg);
            speed = 120;
            start = speed*(steps+1)*cfg.controller.sampleTime;
            target = localTarget("approaching", [start; 0], [-speed; 0]);
            [command, ~, first, stored] = collisionAvoidanceController( ...
                ego, target, localLane(), cfg, []);
            nextEgo = localNextEgo(first, command.actuatorInput);
            target.targetPositionInertial(1) = start-speed*cfg.controller.sampleTime;

            outcome = localAttemptPlan(nextEgo, target, cfg, stored);

            testCase.verifyFalse(outcome.accepted);
            testCase.verifyTrue(ismember(outcome.identifier, ...
                ["collisionAvoidanceController:noSolution", ...
                 "collisionAvoidanceController:optimizationFailure"]));
        end

        function uncertainEgoWithoutAStateTimestampIsRejected(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], [0.0; 0.0]);
            ego.controllerStateErrorBound = [0.1; 0.1; 0.01; 0.1; 0.1; 0.01];

            testCase.verifyError(@() collisionAvoidanceController( ...
                ego, [], localLane(), cfg, []), ...
                "collisionAvoidanceController:invalidUncertaintyChart");
        end

        function zeroSpeedSchedulePreservesOffsetRestOnACurve(testCase)
            cfg = collisionAvoidanceControllerConfig();
            state = [20.0; -4.0; 0.1; 0.0; 0.0; 0.0];
            [stateMatrix, inputMatrix, affine] = ltvBicycleModel.stageMatrices( ...
                0.02, 0.0, cfg.controller.sampleTime, cfg);

            next = stateMatrix*state+inputMatrix*[0.0; 0.0]+affine;

            testCase.verifyEqual(next, state, AbsTol=1.0e-14);
        end

        function terminalBrakingRatioCancelsAccelerationBias(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 4.0; 0.0; 0.0], [0.0; 0.0]);
            ego.longitudinalAccelerationBias = 0.7;
            [~, ~, problem, certificate] = collisionAvoidanceController( ...
                ego, [], localLane(), cfg, []);
            % Separate the solver's rest residual from exact rest invariance.
            testCase.verifyLessThan(norm(certificate.predictedState(4:6, end), inf), 1.0e-7);
            terminal = [certificate.predictedState(1:3, end); zeros(3, 1)];
            prediction = problem.prediction;
            next = prediction.stageMatrixA(:, :, end)*terminal ...
                + prediction.stageMatrixB(:, :, end)*problem.qp.terminalInput ...
                + prediction.stageAffine(:, end);
            testCase.verifyEqual(problem.qp.terminalInput, [0.0; -0.7/modifiedFialaTire.accelerationGain(collisionAvoidanceControllerConfig(cfg))], AbsTol=1.0e-14);
            testCase.verifyEqual(next, terminal, AbsTol=1.0e-12);
        end

        function collisionAtTheNextNodeStillRejectsThePlan(testCase)
            cfg = localSmallConfiguration();
            ego = localEgoState([0.0; 0.0; 0.0; 15.0; 0.0; 0.0], ...
                [0.0; 0.0]);
            target = localTarget("crossing", [0.75; 20.0], [0.0; -400.0]);

            testCase.verifyError(@() collisionAvoidanceController( ...
                ego, target, localLane(), cfg), ...
                "collisionAvoidanceController:noSolution");
        end
    end
end

function cfg = localSmallConfiguration()
    cfg = struct();
    cfg.controller = struct("sampleTime", 0.05, "horizonSteps", 4);
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

function outcome = localAttemptPlan(ego, target, cfg, stored)
    outcome = struct("accepted", false, "identifier", "");
    try
        [~, ~, problem] = collisionAvoidanceController(ego, target, localLane(), cfg, stored);
        outcome.accepted = problem.metadata.planCertified;
    catch exception
        outcome.identifier = string(exception.identifier);
    end
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

function trace = localFallbackToRest()
    cfg = localSmallConfiguration();
    cfg.solver.jointFunction = @localJointSolveHook;
    localJointSolveHook("reset", struct());
    ego = localEgoState([0.0; 0.1; 0.002; 15.0; 0.01; 0.0], [0.0; 0.0]);
    [command, ~, first, certificate] = collisionAvoidanceController( ...
        ego, [], localLane(), cfg, []);
    expected = certificate.predictedState;
    sampleCount = first.prediction.stageCount+3;
    trace = struct("sampleCount", sampleCount, "headSteps", cfg.controller.horizonSteps, ...
        "tailSteps", first.prediction.tailSteps, "maximumStateError", 0.0, ...
        "fallbackUsed", false(1, sampleCount), "solverCalls", ones(1, sampleCount), ...
        "finalState", zeros(6, 1));
    problem = first;
    for sampleIdx = 2:sampleCount
        ego = localNextEgo(problem, command.actuatorInput);
        [command, ~, problem, certificate] = collisionAvoidanceController( ...
            ego, [], localLane(), cfg, certificate);
        trace.maximumStateError = max(trace.maximumStateError, ...
            norm(certificate.predictedState(:, 1) ...
                - expected(:, min(sampleIdx, size(expected, 2))), inf));
        trace.fallbackUsed(sampleIdx) = problem.metadata.fallbackUsed;
        trace.solverCalls(sampleIdx) = problem.metadata.solverCallCount;
    end
    trace.finalState = certificate.predictedState(:, 1);
end

function result = localUnsafeSuccess(~, program)
    result = program.defaultSolver();
    result.exitFlag = 1;
    result.decision(1) = program.ub(1)+0.5;
end

function result = localUnsafeAfterFirst(phase, program)
    persistent calls
    if phase == "reset"
        calls = 0;
        result = struct();
        return;
    end
    calls = calls+1;
    if calls == 1
        result = program.defaultSolver();
    else
        result = struct("decision", zeros(numel(program.f), 1), ...
            "exitFlag", 1, "output", struct("message", "invalid successful iterate"));
    end
end
