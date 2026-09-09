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
            testCase.verifyEqual(stored.safetyScope, "completeEncounterForDeclaredInclusion");
        end

        function targetsRequireFiniteMotionBounds(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            target = rmfield(target,"predictionMotion");
            testCase.verifyError(@() collisionAvoidanceController(ego,target,route,cfg,[]), ...
                "collisionAvoidanceController:invalidBarrierAdmission");
        end

        function anUncertifiedTerminalExitCannotBeAccepted(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            target.targetVelocityInertial(:) = 0;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,route,cfg,[]), ...
                "collisionAvoidanceController:noCertifiedContinuation");
        end

        function uncertaintyCannotBeRemovedToCertifyExit(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            target.targetPositionInertialErrorBound = [50;50];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,route,cfg,[]), ...
                "collisionAvoidanceController:noCertifiedContinuation");
        end

        function failSolverResultRetainsTheIndependentlyCheckedWitness(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            nextEgo = encounterTestFixture.nextEgo(stored,problem.model.lane);
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            [command,~,next] = collisionAvoidanceController(nextEgo,[],route,cfg,stored);
            testCase.verifyEqual(command.actuatorInput,stored.plan(:,2),AbsTol=0);
            testCase.verifyTrue(next.metadata.fallbackUsed);
        end

        function missingPartialObservationRetainsTheOriginalObligations(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            [~, ~, next, certificate] = collisionAvoidanceController(nextEgo, [], route, cfg, stored);
            testCase.verifyFalse(next.metadata.fallbackUsed);
            testCase.verifyEqual(next.metadata.certificateSource, "checkedContinuationOptimization");
            testCase.verifyEqual(next.metadata.activeTargetKeys, "trackId:1");
            testCase.verifyEqual(certificate.deadline, stored.deadline, AbsTol=1e-14);
            testCase.verifyEqual(certificate.remainingSteps, stored.remainingSteps-1);
        end

        function successfulReplanningKeepsTheOriginalDeadlineUntilCompletion(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            deadline = stored.deadline;
            for index = 1:cfg.controller.horizonSteps
                ego = encounterTestFixture.nextEgo(stored,problem.model.lane);
                [command,~,problem,stored] = collisionAvoidanceController(ego,[],route,cfg,stored);
                testCase.verifyEqual(stored.deadline,deadline,AbsTol=0);
                if stored.encounterComplete
                    testCase.verifyEmpty(command);
                    break;
                end
            end
            testCase.verifyTrue(stored.encounterComplete);
        end

        function aPositiveSolverStatusCannotAuthorizeUnsafeControls(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            cfg.solver.jointFunction = @encounterTestFixture.unsafe;
            testCase.verifyError(@() collisionAvoidanceController(ego, target, route, cfg, []), ...
                "collisionAvoidanceController:noCertifiedContinuation");
        end

        function unsafeSolverResultRetainsTheIndependentlyCheckedWitness(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            nextEgo = encounterTestFixture.nextEgo(stored,problem.model.lane);
            cfg.solver.jointFunction = @encounterTestFixture.unsafe;
            [command,~,next] = collisionAvoidanceController(nextEgo,[],route,cfg,stored);
            testCase.verifyEqual(command.actuatorInput,stored.plan(:,2),AbsTol=0);
            testCase.verifyTrue(next.metadata.fallbackUsed);
        end

        function aChangedMotionContractCannotInheritTheWitness(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial+0.1*target.targetVelocityInertial;
            target.predictionMotion.jerkBound(1) = 1;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, target, route, cfg, stored), ...
                "collisionAvoidanceController:changedEncounterContract");
        end

        function aValidObservationKeepsTheOriginalJointWitness(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            target.targetPositionInertialErrorBound = [0.2;0.2];
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            nextEgo = encounterTestFixture.nextEgo(stored,problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial ...
                +cfg.controller.sampleTime*target.targetVelocityInertial+[0.02;0];
            target.targetPositionInertialErrorBound = [0.03;0.05];
            [~,~,next,certificate] = collisionAvoidanceController(nextEgo,target,route,cfg,stored);
            testCase.verifyTrue(next.metadata.planCertified);
            testCase.verifyEqual(certificate.encounters,stored.encounters);
            testCase.verifyGreaterThanOrEqual(certificate.margin,stored.margin);
        end

        function inconsistentObservationsInvalidateTheExecutionAssumptions(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, target, route, cfg, []);
            nextEgo = encounterTestFixture.nextEgo(stored, problem.model.lane);
            nextEgo.position(1) = nextEgo.position(1)+1;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo, [], route, cfg, stored), ...
                "collisionAvoidanceController:inconsistentObservation");
        end

        function changedYawBoundsCannotRenewTheOriginalContract(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            target.predictionMotion.yawAccelerationBound = 0.01;
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            nextEgo = encounterTestFixture.nextEgo(stored,problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial ...
                +cfg.controller.sampleTime*target.targetVelocityInertial;
            target.predictionMotion.yawAccelerationBound = 0.02;
            testCase.verifyError(@() collisionAvoidanceController(nextEgo,target,route,cfg,stored), ...
                "collisionAvoidanceController:changedEncounterContract");
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
            nextEgo.perception.time = nextEgo.stateTime;
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
            second.targetPositionInertial(1) = 14;
            [~, ~, problem] = collisionAvoidanceController(ego, [target, second], route, cfg, []);
            testCase.verifyEqual(problem.metadata.activeTargetKeys, ["trackId:1", "trackId:2"]);
            testCase.verifyTrue(any(problem.qp.geometry.label == "collision:trackId:2"));
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

        function oneOptimizationUsesTheCommonTrackingReference(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            [~, ~, problem] = collisionAvoidanceController(ego, target, route, cfg, []);
            testCase.verifyFalse(isfield(problem.metadata,"maneuverCandidates"));
            testCase.verifyEqual(problem.qp.clf.referenceStart, [0;0;cfg.referenceSpeed;0;0]);
            testCase.verifyEqual(numel(problem.metadata.clfRelaxation), cfg.controller.horizonSteps);
            testCase.verifyGreaterThan(problem.metadata.solverCallCount, 0);
        end

        function continuationCannotDecreaseTheVerifiedCarriedMargin(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            nextEgo = encounterTestFixture.nextEgo(stored,problem.model.lane);
            [~,~,next,certificate] = collisionAvoidanceController(nextEgo,[],route,cfg,stored);
            testCase.verifyGreaterThanOrEqual(certificate.margin,stored.margin);
            testCase.verifyEqual(next.metadata.requiredMargin,stored.margin,AbsTol=0);
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

        function finiteExitAllowsPositiveMinimumSpeedAndLimitedBraking(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            cfg.actuation.brakingRatioMinimum = -0.05;
            cfg.model.speedMinimum = 1;
            [~, plan, problem] = collisionAvoidanceController(ego, target, route, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThanOrEqual(plan(2,:), -0.05);
        end
    end
end
