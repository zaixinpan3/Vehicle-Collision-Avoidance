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

        function aSpeedBoxMeetingTheDomainIsReadAndAnEmptyIntersectionIsRejected(testCase)
            % A vehicle at rest measured with speed error publishes a box
            % straddling zero; the reader admits the box, not only its centre.
            [ego, target, cfg, lane] = localInputs();
            ego.longitudinalVelocity = -0.02;
            ego.controllerErrorBound.bounds(4) = 0.05;
            parsed = readPlanningInputs(ego, target, lane, cfg);
            testCase.verifyEqual(parsed.modelState(4), -0.02);
            ego.controllerErrorBound.bounds(4) = 0.01;
            testCase.verifyError(@() readPlanningInputs(ego, target, lane, cfg), ...
                "collisionAvoidanceController:invalidInput");
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

        function egoAndTargetCertificateBoxesAreAdmittedAndCarried(testCase)
            [ego, target, cfg, lane] = localInputs();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,lane,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyEqual(problem.metadata.initialErrorBound(4:6), ego.controllerErrorBound.bounds(4:6), AbsTol=0);
            testCase.verifyEqual(problem.metadata.targetErrorBound(1:2), [0.2;0.2], AbsTol=0);
            testCase.verifyEqual(stored.encounters.radius(1:2), [0.2;0.2], AbsTol=0);
        end

        function aNewlyUncertainObservationIsConditionedByTheExactCarriedFlow(testCase)
            [ego,target,lane,cfg] = encounterTestFixture.crossing();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,lane,cfg,[]);
            ego = encounterTestFixture.nextEgo(stored,problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial ...
                +cfg.controller.sampleTime*target.targetVelocityInertial;
            target.targetPositionInertialErrorBound = [0.01;0.01];
            [~,~,next,certificate] = collisionAvoidanceController(ego,target,lane,cfg,stored);
            testCase.verifyTrue(next.metadata.planCertified);
            testCase.verifyTrue(next.metadata.candidateVerified);
            testCase.verifyLessThan(max(certificate.encounters.radius(1:2)),1e-9);
        end

        function anEgoBoxIsAdmittedWithItsFrenetRadius(testCase)
            [ego,target,cfg,lane] = localInputs();
            ego.position(1) = 10;
            [~,~,problem] = collisionAvoidanceController(ego,target,lane,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyEqual(problem.metadata.initialErrorBound(1:2),[0.1;0.1],AbsTol=1e-12);
        end


    end
end

function [ego, target, cfg, lane] = localInputs()
    [~, crossing, lane, cfg] = encounterTestFixture.crossing();
    ego = struct("position", [0; 0], "yawAngle", 0, ...
        "longitudinalVelocity", 10, "lateralVelocity", 0, "yawRate", 0, ...
        "stateTime", 0, "perception",struct("time",0,"range",16,"completeWithinRange",true), ...
        "controllerErrorBound", ...
        localCertificate("ego-state-v1", [0.1; 0.1; 0.01; 0.1; 0.1; 0.001]));
    target = struct("trackId", "static-target", "targetPositionInertial", [15; -4], ...
        "targetVelocityInertial", [0; 32], "targetAccelerationInertial", [0; 0], ...
        "targetYawInertial", pi/2, "targetYawRate", 0, "stateTime", 0, ...
        "controllerErrorBound", localCertificate("target-state-v1", [0.2; 0.2; zeros(6, 1)]), ...
        "predictionMotion", crossing.predictionMotion);
end

function certificate = localCertificate(kind, values)
    certificate = struct("kind", kind, "time", 0, "bounds", values, ...
        "available", true, "source", "declared-test-enclosure", ...
        "futurePredictionIncluded", false);
end
