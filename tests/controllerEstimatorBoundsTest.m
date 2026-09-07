classdef controllerEstimatorBoundsTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
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

        function currentEstimationDoesNotSupplyFutureMotionPremises(testCase)
            [ego, target, cfg, lane] = localInputs();
            target = rmfield(target, "targetPredictionMotionBounds");
            testCase.verifyError(@() readPlanningInputs(ego, target, lane, cfg), ...
                "collisionAvoidanceController:missingPredictionBound");
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
            testCase.verifyGreaterThan(second.qp.collision.nodes(1).targetSupport, ...
                first.qp.collision.nodes(1).targetSupport);
            testCase.verifyTrue(first.metadata.planCertified && second.metadata.planCertified);
            testCase.verifyEqual(second.metadata.solverCallCount, 1);
        end

        function changingBoundsRequireTheContinuationToBeRechecked(testCase)
            [ego, target, cfg, lane] = localInputs();
            ego = rmfield(ego, "controllerErrorBound");
            [command, ~, first, stored] = collisionAvoidanceController(ego, target, lane, cfg, []);
            state = stored.predictedState(:, 2);
            ego.position = state(1:2);
            ego.yawAngle = state(3);
            ego.longitudinalVelocity = state(4);
            ego.lateralVelocity = state(5);
            ego.yawRate = state(6);
            ego.heldActuatorInput = command.actuatorInput;
            ego.stateTime = cfg.controller.sampleTime;
            target.stateTime = cfg.controller.sampleTime;
            target.controllerErrorBound.time = cfg.controller.sampleTime;
            target.controllerErrorBound.bounds(1:2) = 0.25;
            [~, ~, next] = collisionAvoidanceController(ego, target, lane, cfg, stored);
            testCase.verifyTrue(first.metadata.planCertified && next.metadata.planCertified);
            testCase.verifyFalse(next.metadata.certificateCompatible);
            testCase.verifyTrue(next.metadata.continuationReadmission);
            testCase.verifyEqual(next.metadata.solverCallCount, 1);
        end

        function uncertainEgoVelocityUsesTheDissipativeTerminalCertificate(testCase)
            [ego, ~, cfg, lane] = localInputs();
            ego.position(1) = 10;
            [~, ~, problem] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyEqual(problem.metadata.uncertaintyCertificate.kind, ...
                "dissipative-rest-funnel-v1");
        end

        function velocityUncertaintyEnlargesTheFuturePositionBound(testCase)
            model = localPredictionModel();
            [position, yaw] = targetPrediction.errorEnvelope([0, 1, 2], model);
            testCase.verifyEqual(position(:, 1), model.targetPositionErrorBound, AbsTol=0.0);
            testCase.verifyGreaterThan(position(:, 3), position(:, 2));
            testCase.verifyGreaterThanOrEqual(position(:, 2), ...
                model.targetPositionErrorBound+model.targetVelocityErrorBound);
            testCase.verifyGreaterThan(yaw(3), yaw(1));
        end

        function aNominalStopDoesNotUseASmoothJerkBoundAcrossTheJump(testCase)
            model = localPredictionModel();
            model.targetSpeed = 2;
            model.targetTangentialAcceleration = -2;
            model.targetStopTime = 1;
            model.targetPositionErrorBound(:) = 0;
            model.targetVelocityErrorBound(:) = 0;
            model.targetAccelerationErrorBound(:) = 0;
            model.targetPredictionMotionBounds.jerkNormMaximum = 0;
            model.targetPredictionMotionBounds.accelerationNormMaximum = 2;
            model.targetPredictionMotionBounds.speedMaximum = 10;
            [position, ~] = targetPrediction.errorEnvelope([1, 2], model);
            testCase.verifyEqual(position(:, 1), zeros(2, 1), AbsTol=0.0);
            testCase.verifyGreaterThanOrEqual(position(:, 2), ones(2, 1));
        end
    end
end

function [ego, target, cfg, lane] = localInputs()
    cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 4)));
    lane = [0, 0; 1000, 0];
    ego = struct("position", [0; 0], "yawAngle", 0, ...
        "longitudinalVelocity", 10, "lateralVelocity", 0, "yawRate", 0, ...
        "stateTime", 0, "controllerErrorBound", ...
        localCertificate("ego-state-v1", [0.1; 0.1; 0.01; 0.1; 0.1; 0.001]));
    target = struct("trackId", "static-target", "targetPositionInertial", [60; 3], ...
        "targetVelocityInertial", [0; 0], "targetAccelerationInertial", [0; 0], ...
        "targetYawInertial", 0, "targetYawRate", 0, "stateTime", 0, ...
        "controllerErrorBound", localCertificate("target-state-v1", [0.2; 0.2; zeros(6, 1)]), ...
        "targetPredictionMotionBounds", struct("speedMaximum", 0, ...
        "accelerationNormMaximum", 0, "yawRateMaximum", 0, "jerkNormMaximum", 0));
end

function certificate = localCertificate(kind, values)
    certificate = struct("kind", kind, "time", 0, "bounds", values, ...
        "available", true, "source", "declared-test-enclosure", ...
        "futurePredictionIncluded", false);
end

function model = localPredictionModel()
    model = struct("hasTarget", true, "targetPositionErrorBound", [0.1; 0.1], ...
        "targetVelocityErrorBound", [0.2; 0.2], "targetAccelerationErrorBound", [0.1; 0.1], ...
        "targetPredictionAccelerationErrorBound", zeros(2, 1), "targetYawErrorBound", 0.03, ...
        "targetSpeed", 5, "targetTangentialAcceleration", 0, "targetCurvature", 0, ...
        "targetStopTime", inf, "targetPredictionMotionBounds", struct( ...
        "speedMaximum", 10, "accelerationNormMaximum", 2, "jerkNormMaximum", 1, "yawRateMaximum", 0.1));
end
