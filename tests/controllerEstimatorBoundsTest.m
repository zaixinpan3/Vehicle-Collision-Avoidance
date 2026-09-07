classdef controllerEstimatorBoundsTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "tests")));
        end
    end
    methods (Test)
        function currentCertificatesOverrideFixedNumericAliases(testCase)
            [ego, target, cfg, lane] = localInputs();
            ego.controllerStateErrorBound = zeros(6, 1);
            target.targetPositionInertialErrorBound = zeros(2, 1);
            [parsed, ~, ~, targets] = readPlanningInputs(ego, target, lane, cfg);
            testCase.verifyEqual(parsed.stateErrorBound, ego.controllerErrorBound.bounds, AbsTol=0.0);
            testCase.verifyEqual(targets.positionErrorBound, target.controllerErrorBound.bounds(1:2), AbsTol=0.0);
        end

        function eachCallUsesTheLatestBoundRatherThanAPastMaximum(testCase)
            [ego, target, cfg, lane] = localInputs();
            [first, ~, ~, initial] = readPlanningInputs(ego, target, lane, cfg);
            ego.controllerErrorBound.bounds = 0.25*ego.controllerErrorBound.bounds;
            target.controllerErrorBound.bounds = 0.5*target.controllerErrorBound.bounds;
            [second, ~, ~, fresh] = readPlanningInputs(ego, target, lane, cfg);
            testCase.verifyLessThan(second.stateErrorBound, first.stateErrorBound);
            testCase.verifyLessThan(fresh.positionErrorBound, initial.positionErrorBound);
        end

        function staleBoundsAreRejected(testCase)
            [ego, target, cfg, lane] = localInputs();
            target.controllerErrorBound.time = target.stateTime-0.05;
            testCase.verifyError(@() readPlanningInputs(ego, target, lane, cfg), ...
                "collisionAvoidanceController:staleEstimatorBound");
        end

        function separatelyValidButUnalignedStatesAreRejected(testCase)
            [ego, target, cfg, lane] = localInputs();
            target.stateTime = 0.05;
            target.controllerErrorBound.time = 0.05;
            testCase.verifyError(@() readPlanningInputs(ego, target, lane, cfg), ...
                "collisionAvoidanceController:staleEstimatorBound");
        end

        function unavailableBoundsNeverFallBackToZero(testCase)
            [ego, target, cfg, lane] = localInputs();
            ego.controllerErrorBound.available = false;
            ego.controllerErrorBound.bounds(:) = inf;
            ego.controllerStateErrorBound = zeros(6, 1);
            testCase.verifyError(@() readPlanningInputs(ego, target, lane, cfg), ...
                "collisionAvoidanceController:unavailableEstimatorBound");
        end

        function aBareRelativeRadiusCannotMasqueradeAsAnAbsoluteBound(testCase)
            [ego, target, cfg, lane] = localInputs();
            target = rmfield(target, "controllerErrorBound");
            target.relativePositionErrorBound = 0.1;
            testCase.verifyError(@() readPlanningInputs(ego, target, lane, cfg), ...
                "collisionAvoidanceController:missingEstimatorBound");
        end

        function currentCertificatesUseTheControllersFixedPredictionModel(testCase)
            [ego, target, cfg, lane] = localInputs();
            [~, ~, ~, parsed] = readPlanningInputs(ego, target, lane, cfg);
            testCase.verifyEqual(parsed.positionErrorBound, ...
                target.controllerErrorBound.bounds(1:2), AbsTol=0.0);
            testCase.verifyFalse(isfield(parsed, "predictionMotionBounds"));
        end

        function theOrientationBoundUsesThePublishedBodyHeading(testCase)
            [ego, target, cfg, lane] = localInputs();
            target = rmfield(target, "targetYawInertial");
            target.targetHeadingInertial = 0.2;
            target.targetVelocityInertial = [5; 0];
            [~, ~, ~, parsed] = readPlanningInputs(ego, target, lane, cfg);
            testCase.verifyEqual(parsed.yaw, 0.2, AbsTol=1e-14);
        end

        function aLargerLiveTargetBoundTightensTheActualSocp(testCase)
            [ego, target, cfg, lane] = localInputs();
            ego = rmfield(ego, "controllerErrorBound");
            [~, ~, first] = collisionAvoidanceController(ego, target, lane, cfg, []);
            target.controllerErrorBound.bounds(1:2) = 0.4;
            [~, ~, second] = collisionAvoidanceController(ego, target, lane, cfg, []);
            firstRow = find(startsWith(first.qp.geometry.label, "collision:"), 1);
            secondRow = find(startsWith(second.qp.geometry.label, "collision:"), 1);
            testCase.verifyLessThan(second.qp.geometry.physicalBound(secondRow), ...
                first.qp.geometry.physicalBound(firstRow));
            testCase.verifyTrue(first.metadata.planCertified && second.metadata.planCertified);
            testCase.verifyGreaterThanOrEqual(second.metadata.solverCallCount,1);
            testCase.verifyFalse(second.metadata.fallbackUsed);
        end

        function changingBoundsRequireTheContinuationToBeRechecked(testCase)
            [ego, target, cfg, lane] = localInputs();
            ego = rmfield(ego, "controllerErrorBound");
            [command, ~, first, stored] = collisionAvoidanceController(ego, target, lane, cfg, []);
            state = stored.predictedState(:, 2);
            [ego.position, ego.yawAngle] = laneGeometry.fromFrenet(state, first.model.lane);
            ego.longitudinalVelocity = state(4);
            ego.lateralVelocity = state(5);
            ego.yawRate = state(6);
            ego.heldActuatorInput = command.actuatorInput;
            ego.stateTime = cfg.controller.sampleTime;
            target.stateTime = cfg.controller.sampleTime;
            target.controllerErrorBound.time = cfg.controller.sampleTime;
            target.controllerErrorBound.bounds(1:2) = 0.25;
            target.targetPositionInertial = target.targetPositionInertial ...
                +cfg.controller.sampleTime*target.targetVelocityInertial;
            [~, ~, next] = collisionAvoidanceController(ego, target, lane, cfg, stored);
            testCase.verifyTrue(first.metadata.planCertified && next.metadata.planCertified);
            testCase.verifyTrue(next.metadata.certificateCompatible);
            testCase.verifyTrue(next.metadata.setMembershipUpdate);
            testCase.verifyGreaterThanOrEqual(next.metadata.solverCallCount,1);
            testCase.verifyFalse(next.metadata.fallbackUsed);
        end

        function uncertainEgoVelocityIsRetainedInTheFiniteTube(testCase)
            [ego, ~, cfg, lane] = localInputs();
            ego.position(1) = 10;
            [~, ~, problem] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThan(problem.prediction.egoStateErrorBound(4, end), 0);
        end


    end
end

function [ego, target, cfg, lane] = localInputs()
    [~, crossing, lane, cfg] = encounterTestFixture.crossing();
    ego = struct("position", [0; 0], "yawAngle", 0, ...
        "longitudinalVelocity", 10, "lateralVelocity", 0, "yawRate", 0, ...
        "stateTime", 0, "controllerErrorBound", ...
        localCertificate("ego-state-v1", [0.1; 0.1; 0.01; 0.1; 0.1; 0.001]));
    target = struct("trackId", "static-target", "targetPositionInertial", [15; -4], ...
        "targetVelocityInertial", [0; 8], "targetAccelerationInertial", [0; 0], ...
        "targetYawInertial", pi/2, "targetYawRate", 0, "stateTime", 0, ...
        "controllerErrorBound", localCertificate("target-state-v1", [0.2; 0.2; zeros(6, 1)]), ...
        "encounterContract", crossing.encounterContract);
end

function certificate = localCertificate(kind, values)
    certificate = struct("kind", kind, "time", 0, "bounds", values, ...
        "available", true, "source", "declared-test-enclosure", ...
        "futurePredictionIncluded", false);
end
