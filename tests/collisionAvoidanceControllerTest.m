classdef collisionAvoidanceControllerTest < matlab.unittest.TestCase
    % Held exact-model execution and rolling prediction certificate behavior.
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
            testCase.verifyEqual(stored.safetyScope, "verifiedPredictionWithInvariantTerminalTail");
        end

        function knownStatesDefineExactMotionWithoutAnExtraDescriptor(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            target = rmfield(target,"predictionMotion");
            [~,~,~,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            testCase.verifyEqual(stored.encounters.contract.kind,"exact-motion-v1");
        end

        function anUnavoidableCrossingYieldsAPositiveValueFunction(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            target.targetPositionInertial = [2;-1];
            [command,~,problem] = collisionAvoidanceController(ego,target,route,cfg,[]);
            testCase.verifyNotEmpty(command);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThan(problem.metadata.pcbfValue,0);
            testCase.verifyGreaterThan(problem.metadata.stageViolation(1),0);
            testCase.verifyEqual(problem.metadata.hardRowViolation,0);
        end

        function aTargetPositionBoxIsCarriedIntoTheCollisionRows(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            target.targetPositionInertialErrorBound = [0.3;0.3];
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyEqual(problem.metadata.targetErrorBound(1:2),[0.3;0.3],AbsTol=0);
            testCase.verifyEqual(stored.encounters.radius(1:2),[0.3;0.3],AbsTol=0);
        end

        function anUnreachableTerminalSetExhaustsTheSearchBudget(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            target.targetPositionInertialErrorBound = [50;50];
            cfg.solver.certificateSearchTimeLimit = 0.5;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,route,cfg,[]), ...
                "collisionAvoidanceController:certificateSearchLimit");
        end

        function aFailedSolveExecutesTheCarriedWitnessAtAContinuationFrame(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            nextEgo = encounterTestFixture.nextEgo(stored,problem.model.lane);
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            [command,~,next] = collisionAvoidanceController(nextEgo, ...
                localObservation(target,stored,nextEgo.stateTime),route,cfg,stored);
            testCase.verifyEqual(next.metadata.certificateSource,"carriedWitness");
            testCase.verifyEqual(command.actuatorInput,stored.plan(:,2),AbsTol=0);
            testCase.verifyTrue(next.metadata.planCertified);
            testCase.verifyFalse(next.metadata.fallbackUsed);
        end

        function missingObservationIsOutsideTheExactStatePremise(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            nextEgo = encounterTestFixture.nextEgo(stored,problem.model.lane);
            testCase.verifyError(@() collisionAvoidanceController(nextEgo,[],route,cfg,stored), ...
                "collisionAvoidanceController:invalidExactScene");
        end

        function successfulReplanningMovesThePredictionEndBeyondTheOriginalDeadline(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            deadline = stored.deadline;
            for index = 1:stored.remainingSteps+2
                ego = encounterTestFixture.nextEgo(stored,problem.model.lane);
                observed = localObservation(target,stored,ego.stateTime);
                [command,~,problem,stored] = collisionAvoidanceController(ego,observed,route,cfg,stored);
                testCase.verifyGreaterThan(stored.deadline,deadline);
                testCase.verifyGreaterThanOrEqual(stored.remainingSteps,cfg.controller.horizonSteps);
                deadline = stored.deadline;
                testCase.verifyNotEmpty(command);
            end
            testCase.verifyFalse(problem.metadata.terminalActive);
            testCase.verifyFalse(stored.encounterComplete);
        end

        function aPositiveSolverStatusCannotAuthorizeUnsafeControls(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            cfg.solver.jointFunction = @encounterTestFixture.unsafe;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                "collisionAvoidanceController:noCertifiedContinuation");
        end

        function anUnsafeSolveIsRejectedInFavourOfTheCarriedWitness(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            nextEgo = encounterTestFixture.nextEgo(stored,problem.model.lane);
            cfg.solver.jointFunction = @encounterTestFixture.unsafe;
            [command,~,next] = collisionAvoidanceController(nextEgo, ...
                localObservation(target,stored,nextEgo.stateTime),route,cfg,stored);
            testCase.verifyEqual(next.metadata.certificateSource,"carriedWitness");
            testCase.verifyEqual(command.actuatorInput,stored.plan(:,2),AbsTol=0);
            testCase.verifyEqual(next.metadata.hardRowViolation,0);
        end

        function aChangedMotionContractCannotInheritTheWitness(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial+0.1*target.targetVelocityInertial;
            target.predictionMotion.jerkBound(1) = 1;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, target, route, cfg, stored), ...
                "collisionAvoidanceController:nonexactStudyInput");
        end

        function aValidObservationPreservesTheOriginalTargetLawWhileRefreshingTheProblem(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            nextEgo = encounterTestFixture.nextEgo(stored,problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial ...
                +cfg.controller.sampleTime*target.targetVelocityInertial;
            [~,~,next,certificate] = collisionAvoidanceController(nextEgo,target,route,cfg,stored);
            testCase.verifyTrue(next.metadata.planCertified);
            testCase.verifyEqual(certificate.originalEncounter,stored.originalEncounter);
            testCase.verifyEqual(next.model.stateTime,nextEgo.stateTime);
            testCase.verifyGreaterThanOrEqual(certificate.margin,0);
        end

        function inconsistentObservationsInvalidateTheExecutionAssumptions(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            nextEgo.position(1) = nextEgo.position(1)+1;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, localObservation(target,stored,nextEgo.stateTime), route, cfg, stored), ...
                "collisionAvoidanceController:inconsistentObservation");
        end

        function changedYawBoundsCannotRenewTheOriginalContract(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            target.predictionMotion.yawAccelerationBound = 0;
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            nextEgo = encounterTestFixture.nextEgo(stored,problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial ...
                +cfg.controller.sampleTime*target.targetVelocityInertial;
            target.predictionMotion.yawAccelerationBound = 0.02;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo,target,route,cfg,stored), ...
                "collisionAvoidanceController:nonexactStudyInput");
        end

        function measuredActuatorMismatchPreventsReuse(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            nextEgo.heldActuatorInput(2) = nextEgo.heldActuatorInput(2)+0.1;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, localObservation(target,stored,nextEgo.stateTime), route, cfg, stored), ...
                "collisionAvoidanceController:executionContractViolation");
        end

        function lateSamplesCannotSilentlyResetTheClock(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            nextEgo.stateTime = 0.2;
            nextEgo.perception.time = nextEgo.stateTime;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, localObservation(target,stored,nextEgo.stateTime), route, cfg, stored), ...
                "collisionAvoidanceController:executionContractViolation");
        end

        function finiteResidualsAreOutsideTheExactStudy(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            cfg.model.plantModelResidualRateBound = 1e-3*ones(6,1);
            testCase.verifyError(@() collisionAvoidanceController(ego,target,route,cfg,[]), ...
                "collisionAvoidanceController:nonexactStudyInput");
        end

        function aSecondTargetIsOutsideTheStrictScene(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            second = target;
            second.trackId = 2;
            testCase.verifyError(@() collisionAvoidanceController(ego,[target,second],route,cfg,[]), ...
                "collisionAvoidanceController:invalidExactScene");
        end

        function betweenNodeCrossingIsReportedAsPredictedViolation(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            cfg.controller.sampleTime = 0.5;
            cfg.controller.horizonSteps = 1;
            cfg.referenceSpeed = 0;
            ego.speed = 0;
            target.targetPositionInertial = [0; -10];
            target.targetVelocityInertial = [0; 40];
            testCase.verifyGreaterThan(avoidanceSafetyGeometry.rectangleDistance([0;0], 0, [0;-10], pi/2, [2.4;.95;2.4;.95]), 0);
            testCase.verifyGreaterThan(avoidanceSafetyGeometry.rectangleDistance([0;0], 0, [0;10], pi/2, [2.4;.95;2.4;.95]), 0);
            [~, ~, problem] = collisionAvoidanceController(ego, target, route, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThan(problem.metadata.pcbfValue, 0);
            testCase.verifyGreaterThan(problem.metadata.stageViolation(1), 0);
        end

        function oneOptimizationUsesTheCommonTrackingReference(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem] = collisionAvoidanceController(ego, target, route, cfg, []);
            testCase.verifyFalse(isfield(problem.metadata,"maneuverCandidates"));
            testCase.verifyEqual(problem.qp.clf.referenceStart, [0;0;cfg.referenceSpeed;0;0]);
            testCase.verifyEqual(numel(problem.metadata.clfRelaxation), cfg.controller.horizonSteps);
            testCase.verifyGreaterThan(problem.metadata.solverCallCount, 0);
        end

        function freshOptimizationPreservesHardSafetyWithoutLockingSurplusMargin(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            nextEgo = encounterTestFixture.nextEgo(stored,problem.model.lane);
            [~,~,next,certificate] = collisionAvoidanceController(nextEgo,localObservation(target,stored,nextEgo.stateTime),route,cfg,stored);
            testCase.verifyGreaterThanOrEqual(certificate.margin,0);
            testCase.verifyEqual(next.metadata.requiredMargin,0);
        end

        function theExplicitWitnessSurvivesConvenienceStateReset(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            collisionAvoidanceController("resetNominalTrajectory");
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            [~, ~, next] = collisionAvoidanceController(nextEgo, localObservation(target,stored,nextEgo.stateTime), route, cfg, stored);
            testCase.verifyFalse(next.metadata.fallbackUsed);
            testCase.verifyTrue(next.metadata.certificateCompatible);
        end

        function aReferenceChartJumpNeedsAnExplicitJumpCertificate(testCase)
            [ego, target, ~, cfg] = encounterTestFixture.crossing();
            cfg.controller.horizonSteps = 4;
            route = [-100,0;0.2,0;50,5];
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                "collisionAvoidanceController:unsupportedReferenceJump");
        end

        function aPositiveMinimumSpeedExcludesTheSlowingTerminalSet(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            cfg.model.speedMinimum = 1;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,route,cfg,[]), ...
                "collisionAvoidanceController:invalidExactScene");
        end
    end
end

function target = localObservation(target,stored,time)
    state = targetPrediction.finiteFlow(stored.encounters,time-stored.encounters.time);
    target.targetPositionInertial = state(1:2);
    target.targetVelocityInertial = state(3:4);
    target.targetAccelerationInertial = state(5:6);
    target.targetHeadingInertial = state(7);
    target.targetYawRate = state(8);
end
