classdef collisionAvoidanceControllerTest < matlab.unittest.TestCase
    % Held execution, encounter discharge and carried continuation behavior.
    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "tests")));
        end
    end
    methods (TestMethodSetup)
        function resetController(~)
            collisionAvoidanceController("resetNominalTrajectory");
        end
    end
    methods (Test)
        function commandUsesSteeringAndDimensionlessBrakingRatio(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [command, plan, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            testCase.verifyEqual(command.actuatorInput, plan(:, 1), AbsTol=0);
            testCase.verifyEqual(command.longitudinalAcceleration, ...
                modifiedFialaTire.accelerationGain(cfg)*command.brakingRatio, AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(plan(2, :), cfg.actuation.brakingRatioMinimum);
            testCase.verifyLessThanOrEqual(plan(2, :), cfg.actuation.brakingRatioMaximum);
            testCase.verifyLessThanOrEqual(abs(plan(1, :)), cfg.model.frontWheelSteeringAngleMaximum);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyEqual(stored.safetyScope, "heldIntervalsUntilCertifiedEncounterExit");
        end

        function aFiniteCollisionFreePrefixNeedsAnExitContract(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            target = rmfield(target, "encounterContract");
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                "collisionAvoidanceController:missingEncounterContract");
        end

        function forecastExpiryDoesNotDischargeAnActiveEncounter(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            target.encounterContract.validUntil = 1;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                "collisionAvoidanceController:noCertifiedExit");
        end

        function aStoppedTargetRemainsActiveWithoutAnExit(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            target.targetVelocityInertial(:) = 0;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                "collisionAvoidanceController:noCertifiedExit");
        end

        function routeDischargeMustClearTheEntireAllowedEgoFootprint(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            target.encounterContract.exitOffset = 4;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                "collisionAvoidanceController:invalidExitRoute");
        end

        function aPositiveExitDistanceDoesNotReplaceTheNonreturnPremise(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            target.encounterContract = rmfield(target.encounterContract,"postExitRoute");
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                "collisionAvoidanceController:missingEncounterContract");
        end

        function anUnimplementedHoldingGuardCannotAuthorizeWaiting(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            target.encounterContract.postExitRoute = "waitingForAnotherForecast";
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                "collisionAvoidanceController:invalidEncounterContract");
        end

        function uncertainTargetFootprintMustClearTheExitPlane(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            target.targetPositionInertialErrorBound = [0; 2];
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                "collisionAvoidanceController:noCertifiedExit");
        end

        function solverFailureDoesNotExecuteTheStoredTail(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, [], route, cfg, stored), ...
                "collisionAvoidanceController:noCertifiedContinuation");
        end

        function targetDisappearanceStillRequiresAFreshCertifiedSolve(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            [~, ~, next, certificate] = collisionAvoidanceController(nextEgo, [], route, cfg, stored);
            testCase.verifyFalse(next.metadata.fallbackUsed);
            testCase.verifyEqual(next.metadata.certificateSource, "checkedOptimization");
            testCase.verifyEqual(next.metadata.activeTargetKeys, "trackId:1");
            testCase.verifyEqual(certificate.deadline, stored.deadline, AbsTol=1e-14);
            testCase.verifyEqual(certificate.remainingSteps, stored.remainingSteps-1);
        end

        function successfulReplanningKeepsTheActiveEncounterDeadline(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            [last, deadlines, activeCounts] = localFollowUntilExit(stored, problem.model.lane, route, cfg);
            testCase.verifyEqual(deadlines(1:14), repmat(stored.deadline, 1, 14), AbsTol=1e-12);
            testCase.verifyEqual(activeCounts, [ones(1, 14), 0]);
            testCase.verifyTrue(last.encounters.discharged);
            testCase.verifyGreaterThanOrEqual(last.encounters.exitMargin, 0);
        end

        function anActiveCollisionConstraintDoesNotAuthorizeFallback(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            target.targetPositionInertial(1) = 10;
            cruiseClearance = rectangleConfigurationDistance([7.2; 0], 0, ...
                [10; 3.2], pi/2, [2.4; .95; 2.4; .95]);
            testCase.verifyLessThan(cruiseClearance, 0);
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            testCase.verifyGreaterThan(stored.margin, 0);
            testCase.verifyLessThan(stored.margin, 1e-4);
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, [], route, cfg, stored), ...
                "collisionAvoidanceController:noCertifiedContinuation");
        end

        function aPositiveSolverStatusCannotAuthorizeUnsafeControls(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            cfg.solver.jointFunction = @encounterTestFixture.unsafe;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                "collisionAvoidanceController:noCertifiedContinuation");
        end

        function anUnsafeSolverResultFailsWithoutReplayingAcceptedInputs(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            cfg.solver.jointFunction = @encounterTestFixture.unsafe;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, [], route, cfg, stored), ...
                "collisionAvoidanceController:noCertifiedContinuation");
        end

        function aChangedMotionContractCannotInheritTheWitness(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial+0.1*target.targetVelocityInertial;
            target.encounterContract.jerkBound(1) = 1;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, target, route, cfg, stored), ...
                "collisionAvoidanceController:changedEncounterContract");
        end

        function aValidObservationConditionsFuturesWithoutArrayEquality(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            target.targetPositionInertialErrorBound = [0.2; 0.2];
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            target.targetPositionInertial = [15.02; -3.2];
            target.targetPositionInertialErrorBound = [0.03; 0.05];
            [~, ~, next, certificate] = collisionAvoidanceController(nextEgo, target, route, cfg, stored);
            testCase.verifyFalse(next.metadata.fallbackUsed);
            testCase.verifyEqual(certificate.encounters.center(1), 15, AbsTol=0);
            testCase.verifyEqual(certificate.encounters.radius(1:2), [0.05; 0.05], AbsTol=1e-12);
        end

        function inconsistentObservationsInvalidateTheExecutionAssumptions(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            nextEgo.position(1) = nextEgo.position(1)+1;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, [], route, cfg, stored), ...
                "collisionAvoidanceController:inconsistentObservation");
        end

        function aKnownTargetInheritsItsFutureYawContract(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            target.encounterContract.yawAccelerationBound = 0.01;
            target.targetPredictionYawAccelerationErrorBound = 0.01;
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial ...
                +cfg.controller.sampleTime*target.targetVelocityInertial;
            target = rmfield(target, "encounterContract");
            [~, ~, next, certificate] = collisionAvoidanceController(nextEgo, target, route, cfg, stored);
            testCase.verifyFalse(next.metadata.fallbackUsed);
            testCase.verifyEqual(certificate.encounters.contract, stored.encounters.contract);
            target.targetPredictionYawAccelerationErrorBound = 0.02;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, target, route, cfg, stored), ...
                "collisionAvoidanceController:invalidEncounterContract");
        end

        function measuredActuatorMismatchPreventsReuse(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            nextEgo.heldActuatorInput(2) = nextEgo.heldActuatorInput(2)+0.1;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, [], route, cfg, stored), ...
                "collisionAvoidanceController:executionContractViolation");
        end

        function lateSamplesCannotSilentlyResetTheClock(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            nextEgo.stateTime = 0.2;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, [], route, cfg, stored), ...
                "collisionAvoidanceController:executionContractViolation");
        end

        function finiteResidualsDoNotRequirePerpetualVelocityDissipation(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            cfg.model.plantModelResidualRateBound = [1e-3; 1e-4; 1e-5; 1e-3; 1e-4; 1e-5];
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThan(stored.prediction.egoStateErrorBound(:, end), zeros(6, 1));
            testCase.verifyFalse(isfield(stored, "terminalUncertainty"));
        end

        function aSecondTargetIsIncludedInJointAdmission(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            second = target;
            second.trackId = 2;
            second.encounterContract.id = "crossing-2";
            second.targetPositionInertial(1) = 25;
            [~, ~, problem] = collisionAvoidanceController(ego, [target, second], route, cfg, []);
            testCase.verifyEqual(problem.metadata.activeTargetKeys, ["trackId:1", "trackId:2"]);
            testCase.verifyTrue(any(problem.qp.geometry.label == "collision:trackId:2"));
        end

        function anInfeasibleNewTargetCannotBeOmittedFromTheActiveSet(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            target.trackId = 2;
            target.encounterContract.id = "new-conflict";
            target.targetPositionInertial = nextEgo.position;
            target.targetVelocityInertial = [0; 20];
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, target, route, cfg, stored), ...
                "collisionAvoidanceController:noCertifiedContinuation");
        end

        function betweenNodeCrossingCannotReceiveANodeOnlyCertificate(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            cfg.controller.sampleTime = 0.5;
            cfg.controller.horizonSteps = 1;
            cfg.referenceSpeed = 0;
            ego.speed = 0;
            target.targetPositionInertial = [0; -10];
            target.targetVelocityInertial = [0; 40];
            testCase.verifyGreaterThan(rectangleConfigurationDistance([0;0], 0, [0;-10], pi/2, [2.4;.95;2.4;.95]), 0);
            testCase.verifyGreaterThan(rectangleConfigurationDistance([0;0], 0, [0;10], pi/2, [2.4;.95;2.4;.95]), 0);
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                "collisionAvoidanceController:noCertifiedContinuation");
        end

        function maneuverOptimizationKeepsTheCommonTrackingReference(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem] = collisionAvoidanceController(ego, target, route, cfg, []);
            testCase.verifyEqual(problem.metadata.maneuverCandidates, ["yield", "passLeft", "passRight"]);
            testCase.verifyEqual(problem.qp.clf.referenceStart, [0;0;cfg.referenceSpeed;0;0]);
            testCase.verifyEqual(numel(problem.metadata.clfRelaxation), cfg.controller.horizonSteps);
            testCase.verifyEqual(problem.metadata.solverCallCount, 3);
        end

        function barrierDecayIsEnforcedOnTheVerifiedCarriedMargin(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            cfg.encounter.barrierFraction = 0.25;
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            [~, ~, next, certificate] = collisionAvoidanceController(nextEgo, [], route, cfg, stored);
            testCase.verifyGreaterThanOrEqual(certificate.margin, 0.75*stored.margin);
            testCase.verifyEqual(next.metadata.requiredMargin, 0.75*stored.margin, AbsTol=1e-12);
            testCase.verifyEqual(certificate.remainingSteps, stored.remainingSteps-1);
        end

        function theExplicitWitnessSurvivesConvenienceStateReset(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            collisionAvoidanceController("resetNominalTrajectory");
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            [~, ~, next] = collisionAvoidanceController(nextEgo, [], route, cfg, stored);
            testCase.verifyFalse(next.metadata.fallbackUsed);
            testCase.verifyTrue(next.metadata.certificateCompatible);
        end

        function aReferenceChartJumpNeedsAnExplicitJumpCertificate(testCase)
            [ego, ~, ~, cfg] = encounterTestFixture.crossing();
            cfg.controller.horizonSteps = 4;
            route = [-100,0;0.2,0;50,5];
            testCase.verifyError(@() collisionAvoidanceController(ego, [], route, cfg, []), ...
                "collisionAvoidanceController:noCertifiedContinuation");
        end

        function finiteExitDoesNotRequireTheOptionalRestSchedule(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            cfg.actuation.brakingRatioMinimum = -0.05;
            cfg.model.speedMinimum = 1;
            [~, plan, problem] = collisionAvoidanceController(ego, target, route, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThanOrEqual(plan(2,:), -0.05);
            testCase.verifyError(@() ltvBicycleModel.brakingSchedule("steps", cfg), ...
                "collisionAvoidanceController:invalidRestSchedule");
        end
    end
end

function [stored, deadlines, activeCounts] = localFollowUntilExit(stored, lane, route, cfg)
    deadlines = zeros(1, 15);
    activeCounts = zeros(1, 15);
    for index = 1:15
        ego = encounterTestFixture.nextEgo(stored, lane);
        [~, ~, problem, stored] = collisionAvoidanceController(ego, [], route, cfg, stored);
        deadlines(index) = stored.deadline;
        activeCounts(index) = numel(problem.metadata.activeTargetKeys);
    end
end
