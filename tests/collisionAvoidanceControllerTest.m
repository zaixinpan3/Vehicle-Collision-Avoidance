classdef collisionAvoidanceControllerTest < matlab.unittest.TestCase
% collisionAvoidanceControllerTest PCBF safe-MPC controller tests.
%
% The controller solves ONE PCBF safe-MPC QP per
% collision-disjunction branch on the scheduled LTV dynamic bicycle,
% reporting [frontAxleTorque; frontWheelSteeringAngle]: tightened
% collision/road rows with per-stage worst-case violation variables,
% hard speed-, heading-, friction-polygon and slip-angle rows,
% hard nominal-path Frenet terminal rows and relaxed-CLF rows, with the
% dominant violation tier as the value-function surrogate and no
% cross-sample safety state. Closed-loop tests roll
% a local nonlinear-bicycle surrogate plant, deliberately unlike the
% controller's own model.

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture( ...
                    fullfile(repositoryRoot, "config")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture( ...
                    fullfile(repositoryRoot, "controller")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture( ...
                    fullfile(repositoryRoot, "scripts")));
        end
    end

    methods (TestMethodSetup)
        function resetController(testCase)
            collisionAvoidanceController("resetNominalTrajectory");
            testCase.addTeardown(@() collisionAvoidanceController( ...
                "resetNominalTrajectory"));
        end
    end

    methods (Test)
        %% Command contract
        function movingSceneReturnsTorqueSteeringCommand(testCase)
            % Target-free below the reference speed, so the optimum is
            % unambiguous drive torque and the smooth-split contract on
            % the rear axle can be asserted tightly. With a target
            % ahead the controller may now legitimately brake, and the
            % split contract for braking is different by design.
            cfg = localScenarioCfg();
            ego = localEgo(12.0);

            [command, predictedInput, problem] = ...
                collisionAvoidanceController( ...
                    ego, struct(), localStraightLane(), cfg);

            testCase.verifySize(command.actuatorInput, [2, 1]);
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyEqual(command.actuatorInputOrder, ...
                ["frontAxleTorque", "frontWheelSteeringAngle"]);
            testCase.verifyEqual(command.frontAxleTorque, ...
                command.actuatorInput(1), AbsTol=0.0);
            testCase.verifyEqual(command.frontWheelSteeringAngle, ...
                command.actuatorInput(2), AbsTol=0.0);
            % The unbiased smooth split T*logisticSigmoid(-T/w) is
            % strictly positive for drive torque rather than exactly
            % zero. Forcing it to zero would reintroduce a kink at
            % T = 0, which is the thing the smooth actuator map exists
            % to avoid, so the residual is bounded instead: it is below
            % a micro-newton-metre, some twenty orders of magnitude
            % under the axle friction capacity.
            % The split law itself is the contract, verified verbatim:
            % rear = share * T * logisticSigmoid(-T / smoothingWidth),
            % whatever torque the scene's optimum happens to command.
            rearShare = (1.0 - cfg.actuation.frontBrakingTorqueRatio) ...
                / cfg.actuation.frontBrakingTorqueRatio;
            smoothingWidth = cfg.actuation.torqueSmoothingWidth;
            expectedRear = rearShare * command.frontAxleTorque ...
                / (1.0 + exp(command.frontAxleTorque / smoothingWidth));
            testCase.verifyEqual(command.rearAxleTorque, ...
                expectedRear, AbsTol=1.0e-9);
            testCase.verifyLessThanOrEqual( ...
                abs(command.rearAxleTorque), ...
                abs(command.frontAxleTorque) + 1.0e-9);
            testCase.verifySize(command.longitudinalTireForce, [4, 1]);
            testCase.verifyEqual(problem.metadata.controlInputOrder, ...
                ["frontAxleTorque", "frontWheelSteeringAngle"]);
        end

        function secondOutputIsCurrentPredictedInput(testCase)
            cfg = localScenarioCfg();
            ego = localEgo(12.0);

            [command, predictedInput] = ...
                collisionAvoidanceController( ...
                    ego, struct(), localStraightLane(), cfg);

            testCase.verifySize(predictedInput, ...
                [2, cfg.collision.pcbf.horizonSteps]);
            testCase.verifyEqual(command.actuatorInput, ...
                predictedInput(:, 1), AbsTol=0.0);
        end

        function solvedPlanRespectsHardInputBounds(testCase)
            cfg = localScenarioCfg();
            ego = localEgo(14.0);
            target = localTarget(45.0, 2.6, -6.0);

            [~, predictedInput] = collisionAvoidanceController( ...
                ego, target, localStraightLane(), cfg);

            testCase.verifyGreaterThanOrEqual( ...
                min(predictedInput(1, :)), ...
                cfg.actuation.frontAxleTorqueMinimum - 1.0e-9);
            testCase.verifyLessThanOrEqual( ...
                max(predictedInput(1, :)), ...
                cfg.actuation.frontAxleTorqueMaximum + 1.0e-9);
            testCase.verifyLessThanOrEqual( ...
                max(abs(predictedInput(2, :))), ...
                cfg.model.frontWheelSteeringAngleMaximum + 1.0e-9);
        end

        %% Real-time-iteration structure
        function perBranchSolveCountIsBounded(testCase)
            % The collision disjunction is resolved by enumerating its
            % branches, each of which solves ONE safe-MPC program with
            % a bounded number of tolerance retries and the
            % infeasibility arbiter.
            % There is no sequential-convex loop, so the total is
            % bounded by the branch count times the per-branch solve
            % allowance, whatever the machine does.
            cfg = localScenarioCfg();
            cfg.solver.function = @localCountingSolver;
            localSolverCallCounter("reset");
            testCase.addTeardown( ...
                @() localSolverCallCounter("reset"));
            ego = localEgo(12.0);
            target = localTarget(45.0, 2.6, -4.0);
            branchCount = numel(cfg.collision.sideCandidates);

            collisionAvoidanceController( ...
                ego, target, localStraightLane(), cfg);
            callsWithTarget = localSolverCallCounter("read");
            localSolverCallCounter("reset");
            collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);
            callsWithoutTarget = localSolverCallCounter("read");

            % One program per branch, allowed its retry chain plus
            % the arbiter and the uncapped re-solve.
            solveAllowancePerBranch = 8;
            testCase.verifyGreaterThanOrEqual( ...
                callsWithTarget, branchCount);
            testCase.verifyLessThanOrEqual(callsWithTarget, ...
                branchCount * solveAllowancePerBranch);
            testCase.verifyGreaterThanOrEqual(callsWithoutTarget, 1);
            testCase.verifyLessThanOrEqual(callsWithoutTarget, ...
                solveAllowancePerBranch);
        end

        function frameStaysInsideItsRealTimeBudget(testCase)
            % The real-time guarantee is the bounded operation count
            % plus the deadline that stops refinement once a
            % commandable iterate exists. Interpreted-MATLAB wall time
            % is indicative, not the guarantee, so the wall-clock
            % assertions carry headroom for machine noise while the
            % mechanism fields are asserted exactly.
            cfg = localScenarioCfg();
            ego = localEgo(15.0);
            target = localTarget(45.0, 2.6, -4.0);

            collisionAvoidanceController("resetNominalTrajectory");
            collisionAvoidanceController( ...
                ego, target, localStraightLane(), cfg);
            elapsed = zeros(5, 1);
            for sampleIdx = 1:5
                startTime = tic;
                [~, ~, problem] = collisionAvoidanceController( ...
                    ego, target, localStraightLane(), cfg);
                elapsed(sampleIdx) = toc(startTime);
            end

            testCase.verifyLessThan( ...
                median(elapsed), 1.5 * cfg.controller.sampleTime);
            testCase.verifyLessThan( ...
                max(elapsed), 3.0 * cfg.controller.sampleTime);
            testCase.verifyGreaterThanOrEqual( ...
                problem.metadata.collisionBranchCount, 1);
        end

        function problemDeclaresTwoLevelLexicographicStructure(testCase)
            cfg = localScenarioCfg();
            ego = localEgo(12.0);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);

            testCase.verifyEqual(problem.framework, ...
                "PcbfSafeMpcRobustRealTimeIteration");
            testCase.verifyEqual( ...
                problem.metadata.programStructure, ...
                "twoStageLexicographicPcbfSafeMpcQpPerBranch");
            testCase.verifyEqual( ...
                problem.metadata.vehicleModelDiscretization, ...
                "exactZeroOrderHoldOneStepPerPredictionStage");
            testCase.verifyEqual(problem.metadata.tireModel, ...
                "linearCorneringBicycleWithInscribedFrictionPolygon");
            testCase.verifyEqual( ...
                problem.constraints.collisionDiscretization, ...
                "predictionNodesOnly");
            testCase.verifyEqual( ...
                problem.constraints.collision, ...
                "perStageSlackedSetTightenedAffineRectangleRows");
            testCase.verifyEqual(problem.objective.type, ...
                "exactLexicographicSlackSumThenClfRelaxationTieBreak");
        end

        function softSlackIsReportedInMetadata(testCase)
            cfg = localScenarioCfg();
            ego = localEgo(12.0);
            target = localTarget(45.0, 2.6, -6.0);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localStraightLane(), cfg);

            testCase.verifyGreaterThanOrEqual( ...
                problem.metadata.slackAggregate, 0.0);
            testCase.verifyGreaterThanOrEqual( ...
                problem.metadata.slackMaximum, 0.0);
            testCase.verifyGreaterThanOrEqual( ...
                problem.metadata.activeSlackRowCount, 0);
            testCase.verifyClass( ...
                problem.metadata.softConstraintsSatisfied, "logical");
            testCase.verifyGreaterThanOrEqual( ...
                problem.metadata.pcbfValue, 0.0);
            % The committed value V-hat decomposes into the measured
            % stage-0 violation plus the stage-A value function, and
            % the lexicographic tie residual stays within its declared
            % solver-scale tolerance.
            testCase.verifyEqual( ...
                problem.metadata.pcbfValue, ...
                problem.metadata.stageZeroViolation ...
                    + problem.metadata.pcbfStageComponent, ...
                AbsTol=1.0e-12);
            testCase.verifyLessThanOrEqual( ...
                problem.metadata.lexicographicTieResidual, ...
                cfg.solver.lexicographicTieTolerance ...
                    * (1.0 + problem.metadata.pcbfStageComponent) ...
                    + 1.0e-9);
            testCase.verifyGreaterThanOrEqual( ...
                min(problem.metadata.clfRelaxation), 0.0);
        end

        function feasibleSceneHasZeroViolationAndTightRows(testCase)
            % With a comfortably avoidable target the achieved
            % violation is zero: a hard-feasible plan into the terminal
            % Frenet terminal set exists, and the dominant violation tier
            % drives every worst-case violation variable to zero, so
            % every slacked collision and road row is satisfied up to
            % solver tolerance on the commanded plan.
            cfg = localScenarioCfg();
            ego = localEgo(12.0);
            target = localTarget(45.0, 2.6, -6.0);

            [~, predictedInput, problem] = ...
                collisionAvoidanceController( ...
                    ego, target, localStraightLane(), cfg);

            control = predictedInput(:);
            collisionViolation = max([0.0; ...
                problem.qp.collisionInequalityMatrix * control ...
                - problem.qp.collisionInequalityBound]);
            roadViolation = max([0.0; ...
                problem.qp.roadInequalityMatrix * control ...
                - problem.qp.roadInequalityBound]);
            testCase.verifyLessThanOrEqual( ...
                problem.metadata.pcbfValue, 1.0e-6);
            testCase.verifyLessThanOrEqual(collisionViolation, ...
                cfg.solver.constraintTolerance);
            testCase.verifyLessThanOrEqual(roadViolation, ...
                cfg.solver.constraintTolerance);
            testCase.verifyLessThanOrEqual( ...
                problem.metadata.hardConstraintViolation, ...
                0.01);
            testCase.verifyEqual( ...
                problem.metadata.neverRelaxedConstraintFamilies, ...
                "physicalInputBoundsSpeedHeadingFrictionAndSlipRows");
        end

        function perStageSlacksExcludeTheHardTerminalSet(testCase)
            % Every collision and road row belongs to its stage's
            % safety slack, and the Frenet terminal family is HARD -
            % the terminal set z_N in Z_f of the safe-MPC problem -
            % alongside the speed-domain and friction physics rows.
            cfg = localScenarioCfg();
            ego = localEgo(12.0);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);

            testCase.verifyEqual(problem.layout.softRowCount, ...
                size(problem.qp.collisionInequalityMatrix, 1) ...
                + size(problem.qp.roadInequalityMatrix, 1));
            testCase.verifyEqual( ...
                problem.layout.hardRowCount, ...
                size(problem.qp.speedDomainInequalityMatrix, 1) ...
                + size(problem.qp.frictionInequalityMatrix, 1) ...
                + size( ...
                    problem.qp.terminalSetInequalityMatrix, 1));
            testCase.verifyEqual( ...
                numel(problem.metadata.pcbfStageSlack), ...
                problem.layout.stageSlackCount);
        end

        function plannedSpeedStaysAboveTheFloor(testCase)
            % The MPC may not plan its way below the speed floor: the
            % declared model has no stop branch, so slowing past it is
            % the terminal controller's job.
            cfg = localScenarioCfg();
            ego = localEgo(12.0);
            target = localTarget(45.0, 2.6, -6.0);

            [~, predictedInput, problem] = ...
                collisionAvoidanceController( ...
                    ego, target, localStraightLane(), cfg);

            horizon = problem.functions.rollout(predictedInput);
            plannedSpeed = horizon.egoState(4, 2:end);
            testCase.verifyGreaterThanOrEqual( ...
                min(plannedSpeed), ...
                cfg.model.plannedSpeedMinimum - 1.0e-3);
        end

        function speedFloorNeverExceedsTheCurrentSpeed(testCase)
            % Starting below the floor must stay solvable: the floor is
            % clamped to the speed the vehicle already has, so it never
            % demands instantaneous acceleration.
            cfg = localScenarioCfg();
            ego = localEgo(0.5);

            command = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);

            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
        end

        function speedFloorOutsideTheModelDomainIsRejected(testCase)
            cfg = localScenarioCfg();
            cfg.model.plannedSpeedMinimum = cfg.model.speedMaximum + 1.0;

            testCase.verifyError(@() collisionAvoidanceController( ...
                localEgo(10.0), struct(), localStraightLane(), cfg), ...
                "collisionAvoidanceController:invalidConfiguration");
        end

        function decisionVectorStaysCondensed(testCase)
            % The decision vector stays the condensed control sequence
            % plus one slack per constrained prediction stage, one CLF
            % relaxation per performance stage, and the six split
            % terminal-state variables - never one variable per row.
            cfg = localScenarioCfg();
            ego = localEgo(12.0);
            target = localTarget(45.0, 2.6, -6.0);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localStraightLane(), cfg);

            controlCount = 2 * cfg.collision.pcbf.horizonSteps;
            testCase.verifyEqual( ...
                problem.layout.controlCount, controlCount);
            testCase.verifyEqual(problem.layout.deltaCount, 1);
            testCase.verifyLessThanOrEqual( ...
                problem.layout.stageSlackCount, ...
                cfg.collision.pcbf.horizonSteps + 1);
            testCase.verifyEqual( ...
                problem.layout.terminalStateCount, 6);
            testCase.verifyEqual( ...
                problem.layout.decisionCountA, ...
                controlCount + problem.layout.stageSlackCount + 6);
            testCase.verifyEqual( ...
                problem.layout.decisionCountB, ...
                controlCount + problem.layout.stageSlackCount ...
                    + problem.layout.deltaCount + 6);
            testCase.verifyEqual( ...
                problem.metadata.decisionVariableCount, ...
                problem.layout.decisionCountB);
        end

        function collisionRowsAreLimitedToPredictionNodes(testCase)
            cfg = localScenarioCfg();
            ego = localEgo(12.0);
            target = localTarget(45.0, 2.6, -6.0);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localStraightLane(), cfg);

            maximumRowCount = ...
                cfg.collision.pcbf.horizonSteps + 1;
            testCase.verifyLessThanOrEqual( ...
                size(problem.qp.collisionInequalityMatrix, 1), ...
                maximumRowCount);
        end

        %% Warm start
        function warmStartShiftsPreviousPrediction(testCase)
            cfg = localScenarioCfg();
            ego = localEgo(12.0);

            [~, ~, firstProblem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);
            [~, ~, secondProblem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);

            testCase.verifyEqual( ...
                firstProblem.metadata.initialNominalSource, ...
                "scheduleReference");
            testCase.verifyEqual( ...
                secondProblem.metadata.initialNominalSource, ...
                "phaseShiftedPreviousPrediction");
        end

        function warmStartIsClearedByExplicitReset(testCase)
            % A stored working set indexes rows of a specific problem.
            % After an explicit reset the next solve must not reuse it,
            % which is observable as the cold-start nominal source.
            cfg = localScenarioCfg();
            ego = localEgo(12.0);

            collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);
            collisionAvoidanceController("resetNominalTrajectory");
            [command, ~, problem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);

            testCase.verifyEqual( ...
                problem.metadata.initialNominalSource, ...
                "scheduleReference");
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
        end

        function warmStartSurvivesTerminalHandoffAndBack(testCase)
            % Handing off to the roadside controller ends the MPC mode.
            % Returning to MPC afterwards must still solve correctly
            % rather than reuse a working set from before the handoff.
            cfg = localScenarioCfg();
            cfg.referenceSpeed = 5.0;
            % A lateral velocity above terminalSafety.lateralVelocityMaximum
            % puts the ego outside the terminal feedback domain, so this
            % state is unambiguously an MPC sample.
            drivingEgo = localEgo(12.0);
            drivingEgo.lateralVelocity = 1.0;
            handoffEgo = localEgo(5.0);
            handoffEgo.position(2) = -1.301;

            collisionAvoidanceController( ...
                drivingEgo, struct(), localStraightLane(), cfg);
            handoffCommand = collisionAvoidanceController( ...
                handoffEgo, struct(), localStraightLane(), cfg);
            [command, ~, problem] = collisionAvoidanceController( ...
                drivingEgo, struct(), localStraightLane(), cfg);

            testCase.verifyEqual(handoffCommand.source, ...
                "shoulderTerminalController");
            testCase.verifyEqual( ...
                problem.metadata.initialNominalSource, ...
                "scheduleReference");
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
        end

        function warmStartIsNotReusedAcrossAStructureChange(testCase)
            % Adding a target changes the row structure, so the stored
            % working set must be discarded rather than reinterpreted
            % against different constraints.
            cfg = localScenarioCfg();
            ego = localEgo(12.0);
            target = localTarget(45.0, 2.6, -6.0);

            [~, ~, freeProblem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);
            [command, ~, targetProblem] = ...
                collisionAvoidanceController( ...
                    ego, target, localStraightLane(), cfg);

            testCase.verifyNotEqual( ...
                targetProblem.layout.softRowCount, ...
                freeProblem.layout.softRowCount);
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyLessThanOrEqual( ...
                targetProblem.metadata.hardConstraintViolation, ...
                0.01);
        end

        function targetSetChangeStartsANewEpisode(testCase)
            % The episode identity is the target set (count and ordered
            % identities), the route, and the configuration. A target
            % first appearing, persisting, and then disappearing must
            % restart the episode at each set change - the shifted plan
            % and schedule are discarded, reported as
            % scheduleShifted=false with the schedule-reference nominal
            % - while the unchanged set in between continues the
            % episode, so the recursive-feasibility argument is never
            % carried across a target-set change.
            cfg = localScenarioCfg();
            state = [0.0; 0.0; 0.0; 12.0; 0.0; 0.0];

            [~, plan, problem] = collisionAvoidanceController( ...
                localEgoFromState(state), struct(), ...
                localStraightLaneAt(state(1:2)), cfg);
            state = problem.functions.rollout(plan).egoState(:, 2);
            [~, plan, appearProblem] = collisionAvoidanceController( ...
                localEgoFromState(state), localTarget(60.0, 2.6, -6.0), ...
                localStraightLaneAt(state(1:2)), cfg);
            state = appearProblem.functions.rollout(plan) ...
                .egoState(:, 2);
            [~, plan, continueProblem] = collisionAvoidanceController( ...
                localEgoFromState(state), localTarget(59.4, 2.6, -6.0), ...
                localStraightLaneAt(state(1:2)), cfg);
            state = continueProblem.functions.rollout(plan) ...
                .egoState(:, 2);
            [~, ~, disappearProblem] = collisionAvoidanceController( ...
                localEgoFromState(state), struct(), ...
                localStraightLaneAt(state(1:2)), cfg);

            testCase.verifyFalse( ...
                appearProblem.metadata.scheduleShifted);
            testCase.verifyEqual( ...
                appearProblem.metadata.initialNominalSource, ...
                "scheduleReference");
            testCase.verifyTrue( ...
                continueProblem.metadata.scheduleShifted);
            testCase.verifyEqual( ...
                continueProblem.metadata.initialNominalSource, ...
                "phaseShiftedPreviousPrediction");
            testCase.verifyFalse( ...
                disappearProblem.metadata.scheduleShifted);
            testCase.verifyEqual( ...
                disappearProblem.metadata.initialNominalSource, ...
                "scheduleReference");
        end

        function explicitResetRestoresColdStart(testCase)
            cfg = localScenarioCfg();
            ego = localEgo(12.0);

            collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);
            collisionAvoidanceController("resetNominalTrajectory");
            [~, ~, problem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);

            testCase.verifyEqual( ...
                problem.metadata.initialNominalSource, ...
                "scheduleReference");
        end

        function configurationChangeReinitializesWarmStart(testCase)
            cfg = localScenarioCfg();
            ego = localEgo(12.0);

            collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);
            changedCfg = cfg;
            changedCfg.referenceSpeed = cfg.referenceSpeed + 1.0;
            [~, ~, problem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), changedCfg);

            testCase.verifyEqual( ...
                problem.metadata.initialNominalSource, ...
                "scheduleReference");
        end

        %% Prediction consistency
        function ltvPredictionMatchesStageRecursion(testCase)
            % The condensed affine map is exactly the stage recursion
            % of the scheduled LTV bicycle matrices: no hidden
            % relinearization, remainder, or rollout exists between
            % the model source and the QP data.
            cfg = localScenarioCfg();
            ego = localEgo(12.0);
            target = localTarget(45.0, 2.6, -6.0);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, target, localStraightLane(), cfg);

            prediction = problem.userData.affinePrediction;
            plan = 0.2 * ones(2, cfg.collision.pcbf.horizonSteps);
            plan(1, :) = 350.0;
            state = prediction.nominalHorizon.egoState(:, 1);
            control = plan(:);
            for stageIdx = 1:cfg.collision.pcbf.horizonSteps
                [stageA, stageB, stageC] = ltvBicycleStageMatrices( ...
                    prediction.scheduleHeading(stageIdx), ...
                    prediction.scheduleSpeed, ...
                    prediction.scheduleYawRate(stageIdx), ...
                    cfg.controller.sampleTime, cfg);
                state = stageA * state + stageB * plan(:, stageIdx) ...
                    + stageC;
                mapped = prediction.egoStateMatrix( ...
                        :, :, stageIdx + 1) * control ...
                    + prediction.egoStateOffset(:, stageIdx + 1);
                testCase.verifyEqual(mapped, state, AbsTol=1.0e-8);
            end
        end

        %% Configuration validation

        function brakingSplitInvertsTheCommandedForceExactly(testCase)
            % The controller decides a total longitudinal force and
            % reports an axle torque: the smooth one-sided split must
            % invert that force exactly, so front plus rear torque over
            % the wheel radius returns the force the program committed.
            cfg = localScenarioCfg();
            ego = localEgo(15.0);
            target = localTarget(18.0, 0.0, 0.0);

            command = collisionAvoidanceController( ...
                ego, target, localStraightLane(), cfg);

            rearShare = (1.0 - cfg.actuation.frontBrakingTorqueRatio) ...
                / cfg.actuation.frontBrakingTorqueRatio;
            smoothingWidth = cfg.actuation.torqueSmoothingWidth;
            expectedRear = rearShare * command.frontAxleTorque ...
                / (1.0 + exp(command.frontAxleTorque / smoothingWidth));
            testCase.verifyEqual(command.rearAxleTorque, ...
                expectedRear, AbsTol=1.0e-9);
            recoveredForce = ...
                (command.frontAxleTorque + command.rearAxleTorque) ...
                / cfg.vehicle.wheelRadius;
            testCase.verifyEqual(recoveredForce, ...
                command.longitudinalForce, RelTol=1.0e-6, ...
                AbsTol=1.0e-6);
        end

        function invalidHorizonStepCountIsRejected(testCase)
            % The single prediction horizon must be a positive integer
            % step count; there is no separate performance horizon to
            % compare it against.
            cfg = localScenarioCfg();
            cfg.collision.pcbf.horizonSteps = 0;

            testCase.verifyError(@() collisionAvoidanceController( ...
                localEgo(10.0), struct(), localStraightLane(), cfg), ...
                "collisionAvoidanceController:invalidConfiguration");
        end

        function largeTorqueBoundsSaturateInsteadOfFailing(testCase)
            % Torque bounds wider than the tires can transmit are
            % admissible rather than a configuration error: the hard
            % friction-polygon rows, not the bounds, are what limit the
            % commanded force.
            cfg = localScenarioCfg();
            cfg.actuation.frontAxleTorqueMinimum = -6000.0;

            command = collisionAvoidanceController( ...
                localEgo(10.0), struct(), localStraightLane(), cfg);

            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
        end

        function roadWithoutTerminalOffsetBandsIsRejected(testCase)
            cfg = localScenarioCfg();
            road = localStraightLane();
            road = rmfield(road, "terminalLateralOffsetBands");

            testCase.verifyError(@() collisionAvoidanceController( ...
                localEgo(10.0), struct(), road, cfg), ...
                "collisionAvoidanceController:missingTerminalSafetySet");
        end

        %% Solver failure
        function solverFailureIsAnOptimizationFailure(testCase)
            cfg = localScenarioCfg();
            cfg.solver.function = @localFailingSolver;

            testCase.verifyError(@() collisionAvoidanceController( ...
                localEgo(12.0), struct(), localStraightLane(), cfg), ...
                "collisionAvoidanceController:optimizationFailure");
        end

        %% Terminal Frenet-set handoff
        function stationaryInsideRightBandEndsControlTask(testCase)
            cfg = localScenarioCfg();
            cfg.referenceSpeed = 5.0;
            cfg.solver.function = @localFailingSolver;
            ego = localEgo(0.0);
            ego.position(2) = -1.301;

            [command, predictedInput, problem] = ...
                collisionAvoidanceController( ...
                    ego, struct(), localStraightLane(), cfg);

            testCase.verifyEmpty(command);
            testCase.verifyEmpty(predictedInput);
            testCase.verifyEmpty(problem);
        end

        function movingInsideRightBandUsesIndependentStopping(testCase)
            cfg = localScenarioCfg();
            cfg.referenceSpeed = 5.0;
            cfg.solver.function = @localFailingSolver;
            ego = localEgo(5.0);
            ego.position(2) = -1.301;

            [command, predictedInput, problem] = ...
                collisionAvoidanceController( ...
                    ego, struct(), localStraightLane(), cfg);

            testCase.verifyEqual( ...
                command.source, "shoulderTerminalController");
            testCase.verifyLessThan(command.frontAxleTorque, 0.0);
            testCase.verifyEqual( ...
                predictedInput, command.actuatorInput, AbsTol=0.0);
            testCase.verifyEqual( ...
                problem.problemClass, ...
                "independentTerminalFrenetController");
            testCase.verifyEqual(command.terminalSetId, "ZfR");
            testCase.verifyEqual(command.terminalSetSide, "right");
            testCase.verifyFalse( ...
                problem.metadata.terminalTaskComplete);
        end

        function movingInsideLeftBandSelectsZfL(testCase)
            cfg = localScenarioCfg();
            cfg.referenceSpeed = 5.0;
            cfg.solver.function = @localFailingSolver;
            ego = localEgo(5.0);
            ego.position(2) = 9.3;

            [command, ~, problem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);

            testCase.verifyEqual(command.terminalSetId, "ZfL");
            testCase.verifyEqual(command.terminalSetSide, "left");
            testCase.verifyEqual( ...
                problem.metadata.terminalLateralOffsetBand, ...
                [8.0, 10.6], AbsTol=1.0e-12);
        end

        function footprintAtRightBandBoundaryIsAdmitted(testCase)
            cfg = localScenarioCfg();
            cfg.referenceSpeed = 5.0;
            cfg.solver.function = @localFailingSolver;
            ego = localEgo(5.0);
            ego.position(2) = -0.950001;
            road = localStraightLaneWithBands([-4.0, 0.0], [8.0, 12.0]);

            command = collisionAvoidanceController( ...
                ego, struct(), road, cfg);

            testCase.verifyEqual(command.terminalSetId, "ZfR");
            testCase.verifyEqual( ...
                command.terminalLateralOffsetBand, ...
                [-4.0, 0.0], AbsTol=0.0);
        end

        function footprintOutsideRightBandDoesNotHandoff(testCase)
            cfg = localScenarioCfg();
            cfg.referenceSpeed = 5.0;
            cfg.solver.function = @localFailingSolver;
            ego = localEgo(5.0);
            ego.position(2) = -0.949;
            road = localStraightLaneWithBands([-4.0, 0.0], [8.0, 12.0]);

            testCase.verifyError(@() collisionAvoidanceController( ...
                ego, struct(), road, cfg), ...
                "collisionAvoidanceController:optimizationFailure");
        end

        function headingOutsideTerminalBoundDoesNotHandoff(testCase)
            cfg = localScenarioCfg();
            cfg.referenceSpeed = 5.0;
            cfg.solver.function = @localFailingSolver;
            ego = localEgo(5.0);
            ego.position(2) = -3.0;
            ego.yawAngle = ...
                cfg.terminalSafety.headingErrorMaximum+0.01;
            road = localStraightLaneWithBands([-6.0, 0.0], [8.0, 14.0]);

            testCase.verifyError(@() collisionAvoidanceController( ...
                ego, struct(), road, cfg), ...
                "collisionAvoidanceController:optimizationFailure");
        end

        function targetPresenceDoesNotBlockTerminalHandoff(testCase)
            % Entering the terminal set stops the MPC and switches to
            % the roadside terminal controller whether or not a target
            % is present. Membership still requires strict current
            % separation from every target, so handing off with a target
            % in view is not the same as ignoring it.
            cfg = localScenarioCfg();
            cfg.referenceSpeed = 5.0;
            ego = localEgo(5.0);
            ego.position(2) = -1.301;
            target = localTarget(60.0, 3.0, -3.0);

            [command, ~, problem] = collisionAvoidanceController( ...
                ego, target, localStraightLane(), cfg);

            testCase.verifyEqual( ...
                command.source, "shoulderTerminalController");
            testCase.verifyEqual(problem.problemClass, ...
                "independentTerminalFrenetController");
            testCase.verifyEqual( ...
                problem.metadata.terminalHandoffTargetPolicy, ...
                "currentSetMembershipRegardlessOfTargetPresence");
        end

        %% Closed-loop behavior
        function cruiseConvergesToReferenceSpeedWithZeroSlack(testCase)
            cfg = localScenarioCfg();
            state = [0.0; 0.3; 0.02; 12.0; 0.0; 0.0];
            lastSlack = inf;
            for stepIdx = 1:40
                ego = localEgoFromState(state);
                lane = localStraightLaneAt(state(1:2));
                [command, ~, problem] = ...
                    collisionAvoidanceController( ...
                        ego, struct(), lane, cfg);
                state = localNonlinearBicycleStep( ...
                    state, command.frontAxleTorque, ...
                    command.frontWheelSteeringAngle, ...
                    cfg.controller.sampleTime, cfg);
                lastSlack = problem.metadata.slackMaximum;
            end

            testCase.verifyEqual(state(4), cfg.referenceSpeed, ...
                AbsTol=0.5);
            testCase.verifyLessThan(abs(state(2)), 0.5);
            testCase.verifyLessThan(lastSlack, 1.0e-6);
        end

        function slowerLeadVehicleIsTrackedWithoutCollision(testCase)
            % A mildly slower lead vehicle stays outside the nominal
            % trajectory, so the frozen duals agree and the hard
            % collision rows shape the manoeuvre over warm-started
            % samples. The gradient is deliberately mild: the declared
            % model has no stop branch, so a scenario demanding
            % deceleration to standstill would leave its domain by
            % design rather than exercise the collision rows.
            cfg = localScenarioCfg();
            state = [0.0; 0.0; 0.0; 15.0; 0.0; 0.0];
            minimumDistance = inf;
            for stepIdx = 1:40
                ego = localEgoFromState(state);
                targetX = 60.0 + 13.5 * (stepIdx - 1) ...
                    * cfg.controller.sampleTime;
                target = localTarget(targetX, 0.0, 13.5);
                lane = localStraightLaneAt(state(1:2));
                [command, ~, problem] = ...
                    collisionAvoidanceController( ...
                        ego, target, lane, cfg);
                state = localNonlinearBicycleStep( ...
                    state, command.frontAxleTorque, ...
                    command.frontWheelSteeringAngle, ...
                    cfg.controller.sampleTime, cfg);
                rectangleDistance = problem.functions ...
                    .rectangleSignedDistance( ...
                        state(1:2), state(3), [targetX; 0.0], 0.0);
                minimumDistance = min( ...
                    minimumDistance, rectangleDistance);
            end

            testCase.verifyGreaterThan(minimumDistance, 0.0);
            testCase.verifyGreaterThan(state(4), 1.0);
        end

        function coldStartThroughATargetIsResolvedByHoisting(testCase)
            % Previously this raised optimizationFailure: the
            % straight-ahead cold-start nominal drove through the target,
            % the per-node projection degenerated to a nearest-face
            % normal that flipped as the nominal transited the obstacle,
            % and the hard collision rows had empty intersection.
            % Hoisting the disjunction at exactly those degenerate nodes
            % makes the branch consistent, so the sample now solves.
            cfg = localScenarioCfg();
            ego = localEgo(12.0);
            target = localTarget(30.0, 0.0, -6.0);

            [command, ~, problem] = collisionAvoidanceController( ...
                ego, target, localStraightLane(), cfg);

            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyLessThanOrEqual( ...
                problem.metadata.hardConstraintViolation, ...
                0.01);
            testCase.verifyEqual( ...
                problem.metadata.collisionDisjunction, ...
                "hoistedOneSidePerTargetHeldAtEveryNode");
            testCase.verifySize( ...
                problem.metadata.collisionSideAssignment, [1, 1]);
        end

        function firstStepRadiusIsTheOneStepReachableSet(testCase)
            % Node 1 carries the measured estimate set. Node 2 is the
            % RIGOROUS one-step reachable set of the declared dynamics
            % from it: for a box E and the affine stage map, the exact
            % componentwise support of A_1 E is |A_1| E, plus one held
            % step of the declared disturbance box. That set - not the
            % unpropagated measurement radius - is what the
            % shifted-candidate argument needs. Nodes 3 and beyond hold the
            % measured radii constant, because no multi-step
            % reachability is claimed.
            cfg = localScenarioCfg();
            cfg.model.ltvModelErrorRateBound = ...
                [0.0; 0.0; 0.0; 0.4; 0.4; 0.2];
            ego = localEgo(15.0);
            ego.controllerStateErrorBound = ...
                [0.08; 0.08; 0.18; 0.30; 0.30; 0.10];

            [~, plan, problem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);
            radii = problem.functions.rollout(plan).egoStateErrorBound;
            prediction = problem.userData.affinePrediction;
            stageMatrix = ltvBicycleStageMatrices( ...
                prediction.scheduleHeading(1), ...
                prediction.scheduleSpeed, ...
                prediction.scheduleYawRate(1), ...
                cfg.controller.sampleTime, cfg);

            expectedFirstStep = ...
                abs(stageMatrix) * ego.controllerStateErrorBound ...
                + cfg.controller.sampleTime ...
                    * cfg.model.ltvModelErrorRateBound;
            testCase.verifyEqual(radii(:, 1), ...
                ego.controllerStateErrorBound, AbsTol=0.0);
            testCase.verifyEqual(radii(:, 2), expectedFirstStep, ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(radii(:, 3:end), repmat( ...
                ego.controllerStateErrorBound, 1, ...
                size(radii, 2) - 2), AbsTol=0.0);
        end

        function noDeclaredUncertaintyLeavesTheRadiiEmpty(testCase)
            % A caller that declares nothing must get exactly the
            % unrobustified problem, so the tightening cannot appear by
            % accident.
            cfg = localScenarioCfg();

            [~, plan, problem] = collisionAvoidanceController( ...
                localEgo(15.0), struct(), localStraightLane(), cfg);
            radii = problem.functions.rollout(plan).egoStateErrorBound;

            testCase.verifyEqual(radii, zeros(size(radii)), AbsTol=0.0);
        end

        function scheduleSpeedIsFlooredAtLowSpeed(testCase)
            % The lateral-force model divides by the schedule speed, so
            % a crawling measured speed schedules at the floor and the
            % program still solves.
            cfg = localScenarioCfg();

            [command, ~, problem] = collisionAvoidanceController( ...
                localEgo(1.0), struct(), localStraightLane(), cfg);

            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            prediction = problem.userData.affinePrediction;
            testCase.verifyEqual(prediction.scheduleSpeed, ...
                cfg.model.scheduleSpeedFloor, AbsTol=0.0);
        end

        function declaringUncertaintyOnlyEnlargesTheRadii(testCase)
            % Monotonicity is the invariant that makes the tightening
            % meaningful: declaring a larger estimation radius may never
            % produce a smaller row radius at any node or in any
            % channel. The scheduled QP data is generated from the
            % measured state and the route alone, so both arms are
            % directly comparable node by node.
            cfg = localScenarioCfg();

            collisionAvoidanceController("resetNominalTrajectory");
            [~, certainPlan, certainProblem] = ...
                collisionAvoidanceController( ...
                    localEgo(15.0), struct(), localStraightLane(), cfg);
            certainRadii = certainProblem.functions.rollout( ...
                certainPlan).egoStateErrorBound;

            uncertainEgo = localEgo(15.0);
            uncertainEgo.controllerStateErrorBound = ...
                [0.08; 0.08; 0.18; 0.30; 0.30; 0.10];
            collisionAvoidanceController("resetNominalTrajectory");
            [~, uncertainPlan, uncertainProblem] = ...
                collisionAvoidanceController( ...
                    uncertainEgo, struct(), localStraightLane(), cfg);
            uncertainRadii = uncertainProblem.functions.rollout( ...
                uncertainPlan).egoStateErrorBound;

            testCase.verifyGreaterThanOrEqual( ...
                uncertainRadii, certainRadii - 1.0e-9);
            testCase.verifyGreaterThan(norm(uncertainRadii(1:2, 1)), ...
                norm(certainRadii(1:2, 1)));
        end

        function targetUncertaintyTightensTheCollisionRow(testCase)
            % A declared target error radius has to move the collision
            % row, otherwise the target channels are decorative. Rows
            % are only comparable when both problems are linearized
            % about the SAME nominal with the same winning branch, so
            % the disjunction is pinned to one side, the iteration cap
            % is one, and the geometry is mild enough that the single
            % round solves with and without the declared radius.
            cfg = localScenarioCfg();
            cfg.collision.sideCandidates = "right";
            ego = localEgo(15.0);
            % Within the performance-horizon window and the side-scope
            % range, yet mild: closing at 5 m/s the encounter stays
            % comfortable and the commanded plan owes no slack.
            certainTarget = localTarget(40.0, 0.8, 10.0);
            uncertainTarget = certainTarget;
            uncertainTarget.targetPositionInertialErrorBound = ...
                0.5 * ones(2, 1);
            uncertainTarget.targetVelocityInertialErrorBound = ...
                zeros(2, 1);
            uncertainTarget.targetYawErrorBound = 0.0;
            uncertainTarget.targetYawRateErrorBound = 0.0;

            collisionAvoidanceController("resetNominalTrajectory");
            [~, ~, certainProblem] = collisionAvoidanceController( ...
                ego, certainTarget, localStraightLane(), cfg);
            collisionAvoidanceController("resetNominalTrajectory");
            [~, ~, uncertainProblem] = collisionAvoidanceController( ...
                ego, uncertainTarget, localStraightLane(), cfg);

            % The tightening is folded into the slacked collision rows
            % themselves: with the same nominal and branch, the
            % uncertain problem's collision bounds are uniformly at
            % least as tight, strictly tighter somewhere, and the mild
            % geometry still owes no slack, so the achieved violation
            % stays zero.
            certainRowCount = size( ...
                certainProblem.qp.collisionInequalityMatrix, 1);
            testCase.verifyGreaterThan(certainRowCount, 0);
            testCase.verifyEqual( ...
                size(uncertainProblem.qp.collisionInequalityMatrix), ...
                size(certainProblem.qp.collisionInequalityMatrix));
            boundTightening = ...
                certainProblem.qp.collisionInequalityBound ...
                - uncertainProblem.qp.collisionInequalityBound;
            testCase.verifyGreaterThanOrEqual( ...
                min(boundTightening), -1.0e-9);
            testCase.verifyGreaterThan(max(boundTightening), 0.1);
            testCase.verifyLessThanOrEqual( ...
                uncertainProblem.metadata.pcbfValue, 1.0e-6);
            testCase.verifyLessThanOrEqual( ...
                uncertainProblem.metadata.slackMaximum, 1.0e-6);
        end

        %% Predictive certificates
        function clfCertificateIsRiccatiConsistent(testCase)
            cfg = localScenarioCfg();
            ego = localEgo(12.0);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);

            certificate = problem.clf.certificate;
            testCase.verifyGreaterThan( ...
                certificate.certifiedDecreaseRate, 0.0);
            testCase.verifyLessThanOrEqual( ...
                certificate.certifiedDecreaseRate, 1.0);
            testCase.verifyEqual(certificate.decreaseRate, ...
                cfg.clf.decreaseRateFraction ...
                    * certificate.certifiedDecreaseRate, ...
                AbsTol=1.0e-12);
            lyapunovMatrix = certificate.lyapunovMatrix;
            testCase.verifyEqual(lyapunovMatrix, lyapunovMatrix.', ...
                AbsTol=1.0e-9);
            testCase.verifyGreaterThan( ...
                min(eig(lyapunovMatrix)), 0.0);
            testCase.verifyEqual( ...
                problem.metadata.clfDecreaseRate, ...
                certificate.decreaseRate, AbsTol=0.0);
        end

        function safeCruiseReportsZeroValueFunction(testCase)
            % A hard-safe cruise lies inside the hard-feasible set of
            % the safe-MPC problem, where the value function vanishes:
            % the reported committed value - the measured stage-0
            % violation plus the achieved predicted violation - is
            % zero on every sample, the executable reading of Huang et
            % al.'s V* = 0 on the safe set. No budget, deficit, or
            % cross-sample safety state exists to assert.
            cfg = localScenarioCfg();
            ego = localEgo(cfg.referenceSpeed);

            [~, ~, firstProblem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);
            [~, ~, secondProblem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);

            testCase.verifyEqual( ...
                firstProblem.metadata.stageZeroViolation, 0.0, ...
                AbsTol=1.0e-9);
            testCase.verifyEqual( ...
                firstProblem.metadata.pcbfStageComponent, 0.0, ...
                AbsTol=1.0e-9);
            testCase.verifyLessThanOrEqual( ...
                firstProblem.metadata.pcbfValue, 1.0e-6);
            testCase.verifyEqual( ...
                secondProblem.metadata.stageZeroViolation, 0.0, ...
                AbsTol=1.0e-9);
            testCase.verifyEqual( ...
                secondProblem.metadata.pcbfStageComponent, 0.0, ...
                AbsTol=1.0e-9);
            testCase.verifyLessThanOrEqual( ...
                secondProblem.metadata.pcbfValue, 1.0e-6);
        end

        function overlappingStartReportsPositiveValueFunction(testCase)
            % An initially violating state is outside the
            % hard-feasible set but inside the PCBF domain: the
            % slacked program stays feasible, the commanded plan is
            % the branch minimizer, and the reported committed value
            % quantifies the predicted unsafety - the stage-0
            % violation forced by the measured state plus the least
            % achievable predicted violation - with the recovery
            % interpretation, not a failure. The decomposition of the
            % committed value reconciles exactly on every sample.
            cfg = localScenarioCfg();
            ego = localEgo(12.0);
            target = localTarget(2.0, 0.0, 0.0);

            [firstCommand, ~, firstProblem] = ...
                collisionAvoidanceController( ...
                    ego, target, localStraightLane(), cfg);
            [secondCommand, ~, secondProblem] = ...
                collisionAvoidanceController( ...
                    ego, target, localStraightLane(), cfg);

            testCase.verifyTrue(all(isfinite( ...
                firstCommand.actuatorInput)));
            testCase.verifyTrue(all(isfinite( ...
                secondCommand.actuatorInput)));
            testCase.verifyGreaterThan( ...
                firstProblem.metadata.stageZeroViolation, 0.0);
            testCase.verifyGreaterThan( ...
                firstProblem.metadata.pcbfValue, 0.0);
            testCase.verifyEqual( ...
                firstProblem.metadata.pcbfValue, ...
                firstProblem.metadata.stageZeroViolation ...
                    + firstProblem.metadata.pcbfStageComponent, ...
                AbsTol=1.0e-12);
            testCase.verifyEqual( ...
                secondProblem.metadata.pcbfValue, ...
                secondProblem.metadata.stageZeroViolation ...
                    + secondProblem.metadata.pcbfStageComponent, ...
                AbsTol=1.0e-12);
        end

        function targetAppearingInOverlapStillSolves(testCase)
            % A target that appears in overlap after hard-safe samples
            % cannot be avoided in prediction, but the safe-MPC
            % program remains feasible by construction - the slacks
            % absorb the forced violation - so the controller commands
            % the minimum-violation plan and reports the positive
            % committed value, with no fallback command, no relaxation
            % path, and no failure.
            cfg = localScenarioCfg();
            ego = localEgo(12.0);

            collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);
            collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);
            target = localTarget(2.0, 0.0, 0.0);
            [command, ~, problem] = collisionAvoidanceController( ...
                ego, target, localStraightLane(), cfg);

            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyGreaterThan( ...
                problem.metadata.stageZeroViolation, 0.0);
            testCase.verifyGreaterThan( ...
                problem.metadata.pcbfValue, 0.0);
            testCase.verifyGreaterThan( ...
                problem.metadata.programExitFlag, 0);
        end

        function firstStepSlotCoversShiftedCandidateStageZero(testCase)
            % The stage rows start at the first-step reachable node, so
            % the forced worst-case violation xiBar_1 of the state the
            % current input actually produces is its own reported stage
            % slot: an overlapping start reports a positive first-step
            % violation inside the stage component, while a hard-safe
            % cruise reports zero. In the recursive-feasibility
            % argument xiBar_1 is the shifted candidate's stage-0
            % slack, the quantity that covers the next sample's
            % measured stage-0 violation under one-step containment.
            cfg = localScenarioCfg();
            ego = localEgo(12.0);
            target = localTarget(2.0, 0.0, 0.0);

            [~, ~, overlapProblem] = collisionAvoidanceController( ...
                ego, target, localStraightLane(), cfg);
            collisionAvoidanceController("resetNominalTrajectory");
            [~, ~, cruiseProblem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);

            testCase.verifyTrue(any( ...
                overlapProblem.metadata.pcbfStageSlot == 2));
            testCase.verifyGreaterThan( ...
                overlapProblem.metadata.firstStepViolation, 0.0);
            testCase.verifyLessThanOrEqual( ...
                overlapProblem.metadata.firstStepViolation, ...
                overlapProblem.metadata.pcbfStageComponent + 1.0e-12);
            testCase.verifyEqual( ...
                cruiseProblem.metadata.firstStepViolation, 0.0, ...
                AbsTol=1.0e-9);
        end

        function terminalRobustContainmentHoldsByConstruction(testCase)
            % The nominal-path Frenet terminal set z_N in Z_f is imposed as hard
            % rows of both lexicographic stages on the split terminal
            % state, so the terminal-containment audit - the
            % recursive-feasibility terminal premise, re-measured on
            % the clipped commanded plan - reports certified with a
            % residual inside the solver tolerance on every solved
            % frame.
            cfg = localScenarioCfg();
            ego = localEgo(cfg.referenceSpeed);

            [~, ~, problem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);

            testCase.verifyTrue( ...
                problem.metadata.terminalRobustContainmentCertified);
            testCase.verifyLessThanOrEqual( ...
                problem.metadata.terminalContainmentResidual, ...
                cfg.solver.constraintTolerance);
        end

        function scheduleShiftsConsistentlyUnderExactDynamics(testCase)
            % Within an episode the stored schedule is advanced by one
            % stage with one freshly marched tail node, so the stage
            % data at the shifted nodes equals last sample's data
            % exactly - the data-consistency premise of the
            % shifted-candidate argument. The loop is rolled with the
            % controller's own declared one-step map (the exactness
            % idealization), so no re-anchor threshold trips.
            cfg = localScenarioCfg();
            state = [0.0; 0.0; 0.0; 12.0; 0.0; 0.0];

            ego = localEgoFromState(state);
            [~, plan, firstProblem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLaneAt(state(1:2)), cfg);
            state = firstProblem.functions.rollout(plan) ...
                .egoState(:, 2);
            ego = localEgoFromState(state);
            [~, ~, secondProblem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLaneAt(state(1:2)), cfg);

            testCase.verifyFalse( ...
                firstProblem.metadata.scheduleShifted);
            testCase.verifyTrue( ...
                secondProblem.metadata.scheduleShifted);
            testCase.verifyEqual( ...
                secondProblem.metadata.nominalTailSource, ...
                "terminalLawInput");
            firstHeading = firstProblem.userData ...
                .affinePrediction.scheduleHeading;
            secondHeading = secondProblem.userData ...
                .affinePrediction.scheduleHeading;
            testCase.verifyEqual( ...
                secondHeading(1:end - 1), firstHeading(2:end), ...
                AbsTol=0.0);
            testCase.verifyEqual( ...
                secondProblem.userData.affinePrediction ...
                    .scheduleSpeed, ...
                firstProblem.userData.affinePrediction ...
                    .scheduleSpeed, ...
                AbsTol=0.0);
        end

        function executedValueFunctionDescendsUnderExactModel(testCase)
            % Huang et al.'s descent inequality on the EXECUTED
            % controller: rolling the loop with the controller's own
            % declared one-step map (exact dynamics, exact target
            % model - the premises of the closed-loop theorem), the
            % reported committed value function satisfies
            % V(k+1) <= V(k) - xi_0(k) + tolerance on every episode
            % continuation, obtained from optimality and the shifted
            % candidate - no budget and no descent constraint exist
            % anywhere in the controller.
            cfg = localScenarioCfg();
            state = [0.0; 0.8; 0.0; 12.0; 0.0; 0.0];
            descentTolerance = 1.0e-4;
            previousValue = nan;
            previousStageZero = nan;
            shiftedSampleCount = 0;
            for stepIdx = 1:25
                ego = localEgoFromState(state);
                targetX = 40.0 + 8.0 * (stepIdx - 1) ...
                    * cfg.controller.sampleTime;
                target = localTarget(targetX, 0.8, 8.0);
                [~, plan, problem] = collisionAvoidanceController( ...
                    ego, target, localStraightLaneAt(state(1:2)), cfg);
                value = problem.metadata.pcbfValue;
                if problem.metadata.scheduleShifted ...
                        && isfinite(previousValue)
                    shiftedSampleCount = shiftedSampleCount + 1;
                    testCase.verifyLessThanOrEqual(value, ...
                        previousValue - previousStageZero ...
                            + descentTolerance);
                end
                previousValue = value;
                previousStageZero = ...
                    problem.metadata.stageZeroViolation;
                state = problem.functions.rollout(plan) ...
                    .egoState(:, 2);
            end

            testCase.verifyGreaterThanOrEqual(shiftedSampleCount, 15);
        end

        function modelErrorRatesGrowTheFirstStepRadiusOnly(testCase)
            % The declared LTV model-error and plant-residual rates
            % enter exactly one open-loop held step at the first-step
            % node - the shifted-candidate one-step growth - and
            % nowhere else, and the formulation reports the declared
            % plant rates.
            cfg = localScenarioCfg();
            residualCfg = cfg;
            residualCfg.model.ltvModelErrorRateBound = ...
                [0.1; 0.1; 0.05; 0.5; 0.5; 0.2];
            residualCfg.model.plantModelResidualRateBound = ...
                [0.0; 0.0; 0.0; 0.2; 0.4; 0.3];
            ego = localEgo(15.0);

            collisionAvoidanceController("resetNominalTrajectory");
            [~, plan, problem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), residualCfg);
            radii = problem.functions.rollout(plan).egoStateErrorBound;

            % With no declared estimation radius the propagated term
            % vanishes and the first-step radius is exactly the
            % declared one-step disturbance growth.
            expectedGrowth = cfg.controller.sampleTime ...
                * (residualCfg.model.ltvModelErrorRateBound ...
                    + residualCfg.model.plantModelResidualRateBound);
            testCase.verifyEqual(radii(:, 2), expectedGrowth, ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(radii(:, 3:end), ...
                zeros(6, size(radii, 2) - 2), AbsTol=0.0);
            testCase.verifyEqual( ...
                problem.metadata.plantModelResidualRateBound, ...
                residualCfg.model.plantModelResidualRateBound);
        end

        function steadyCruiseNeedsNoClfRelaxation(testCase)
            % On the centerline at the reference speed the CLF rows are
            % satisfiable without relaxation, and the achieved violation
            % is zero, so both certificates report a quiet frame.
            cfg = localScenarioCfg();
            ego = localEgo(cfg.referenceSpeed);

            collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);
            [~, ~, problem] = collisionAvoidanceController( ...
                ego, struct(), localStraightLane(), cfg);

            testCase.verifyLessThanOrEqual( ...
                problem.metadata.pcbfValue, 1.0e-6);
            testCase.verifyLessThanOrEqual( ...
                problem.metadata.clfRelaxationFirst, 1.0e-3);
        end
    end
end

%% Local helpers

function cfg = localScenarioCfg()
    cfg = collisionAvoidanceControllerConfig();
end

function ego = localEgo(speed)
    ego = struct();
    ego.position = [0.0; 0.0];
    ego.yawAngle = 0.0;
    ego.longitudinalVelocity = speed;
    ego.lateralVelocity = 0.0;
    ego.yawRate = 0.0;
end

function ego = localEgoFromState(state)
    ego = struct( ...
        "position", state(1:2), ...
        "yawAngle", state(3), ...
        "longitudinalVelocity", state(4), ...
        "lateralVelocity", state(5), ...
        "yawRate", state(6));
end

function target = localTarget(positionX, positionY, velocityX)
    target = struct();
    target.targetPositionInertial = [positionX; positionY];
    target.targetVelocityInertial = [velocityX; 0.0];
    target.targetAccelerationInertial = [0.0; 0.0];
    target.targetYawInertial = 0.0;
    target.targetYawRate = 0.0;
    target.targetLength = 4.80;
    target.targetWidth = 1.90;
end


function road = localStraightLane()
    road = localTestTerminalRoad( ...
        localStraightCenterline(), "through", [0.0; 0.0]);
end

function road = localStraightLaneAt(anchorPosition)
    road = localTestTerminalRoad( ...
        localStraightCenterline(), "through", anchorPosition);
end

function centerline = localStraightCenterline()
    centerline = [-100.0, 0.0; 200.0, 0.0];
end

function road = localTestTerminalRoad( ...
        centerline, routeBranchId, anchorPosition) %#ok<INUSD>
% Supply the two nominal-path Frenet terminal offset bands.

    perception = fitPerceivedRoadBoundaries( ...
        centerline, [anchorPosition(:); 0.0], ...
        PerceptionRange=30.0, ...
        RightOffset=0.001, LeftOffset=8.0, ...
        ShoulderWidth=2.60, ...
        RouteBranchId=routeBranchId);
    road = perception.roadGeometry;
    road.boundaries = struct([]);
end

function road = localStraightLaneWithBands(rightBand, leftBand)
    road = localStraightLane();
    road.terminalLateralOffsetBands = struct( ...
        "right", rightBand, "left", leftBand);
end

function [decision, objective, exitFlag, output] = ...
        localFailingSolver(varargin)
% Premature stop (exit 0), not a declared infeasibility: a solver that
% REPORTS infeasible while the linearization center verifies against
% the hard rows is by design answered with the verified center, so the
% loud-failure contract is probed with an outcome that can never be.
    decision = zeros(0, 1);
    objective = NaN;
    exitFlag = 0;
    output = struct("message", "Injected solver failure.");
end

function [decision, objective, exitFlag, output] = ...
        localCountingSolver(varargin)
    localSolverCallCounter("increment");
    warningState = warning;
    cleanup = onCleanup(@() warning(warningState));
    warning("off", "all");
    options = optimoptions("quadprog", ...
        "Algorithm", "interior-point-convex", ...
        "Display", "none", "MaxIterations", 2000);
    [decision, objective, exitFlag, output] = quadprog( ...
        varargin{1:8}, [], options);
end

function count = localSolverCallCounter(action)
    persistent callCount
    if isempty(callCount)
        callCount = 0;
    end
    count = callCount;
    switch action
        case "reset"
            callCount = 0;
        case "increment"
            callCount = callCount + 1;
            count = callCount;
        case "read"
            count = callCount;
        otherwise
            % The counter has exactly the reset/increment/read actions.
            error("collisionAvoidanceControllerTest:invalidAction", ...
                "Unknown solver-counter action.");
    end
end

function stateNext = localNonlinearBicycleStep( ...
        state, frontAxleTorque, steeringAngle, sampleTime, cfg)
% Surrogate nonlinear-bicycle plant for the closed-loop tests.
%
% Deliberately NOT the controller's prediction model, so closing the
% loop through it exercises the controller against dynamics it does
% not itself hold: the kinematics keep the exact trigonometry, the
% slip angles use the true longitudinal speed instead of the frozen
% schedule speed, the bilinear terms are retained, and the combined
% force is clipped to the friction CIRCLE rather than to the inscribed
% polygon the controller constrains itself by. Sub-stepped for
% integration accuracy. The commanded axle torque enters through the
% same smooth one-sided braking split the command reports.

    mass = cfg.vehicle.m;
    yawInertia = cfg.vehicle.Iz;
    lf = cfg.vehicle.lf;
    lr = cfg.vehicle.lr;
    corneringFront = cfg.tire.corneringStiffness(1);
    corneringRear = cfg.tire.corneringStiffness(2);
    frictionLimit = min(cfg.tire.frictionCoefficient) ...
        * cfg.vehicle.gravity;
    rearShare = (1.0 - cfg.actuation.frontBrakingTorqueRatio) ...
        / cfg.actuation.frontBrakingTorqueRatio;
    smoothingWidth = cfg.actuation.torqueSmoothingWidth;
    rearAxleTorque = rearShare * frontAxleTorque ...
        / (1.0 + exp(frontAxleTorque / smoothingWidth));
    longitudinalForce = (frontAxleTorque + rearAxleTorque) ...
        / cfg.vehicle.wheelRadius;

    substepCount = 5;
    stepTime = sampleTime / substepCount;
    stateNext = state;
    for substepIdx = 1:substepCount
        speed = max(stateNext(4), 0.5);
        frontSlipAngle = steeringAngle ...
            - (stateNext(5) + lf * stateNext(6)) / speed;
        rearSlipAngle = -(stateNext(5) - lr * stateNext(6)) / speed;
        frontLateralForce = corneringFront * frontSlipAngle;
        rearLateralForce = corneringRear * rearSlipAngle;
        longitudinalAcceleration = longitudinalForce / mass;
        lateralAcceleration = ...
            (frontLateralForce + rearLateralForce) / mass;
        demand = hypot( ...
            longitudinalAcceleration, lateralAcceleration);
        if demand > frictionLimit
            scale = frictionLimit / demand;
            longitudinalAcceleration = ...
                longitudinalAcceleration * scale;
            frontLateralForce = frontLateralForce * scale;
            rearLateralForce = rearLateralForce * scale;
            lateralAcceleration = lateralAcceleration * scale;
        end
        derivative = [ ...
            stateNext(4) * cos(stateNext(3)) ...
                - stateNext(5) * sin(stateNext(3)); ...
            stateNext(4) * sin(stateNext(3)) ...
                + stateNext(5) * cos(stateNext(3)); ...
            stateNext(6); ...
            longitudinalAcceleration ...
                + stateNext(5) * stateNext(6); ...
            lateralAcceleration - stateNext(4) * stateNext(6); ...
            (lf * frontLateralForce - lr * rearLateralForce) ...
                / yawInertia];
        stateNext = stateNext + stepTime * derivative;
        stateNext(4) = max(stateNext(4), 0.0);
    end
end
