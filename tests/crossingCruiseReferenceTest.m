classdef crossingCruiseReferenceTest < matlab.unittest.TestCase
% crossingCruiseReferenceTest Arrival preference and its operating domain.

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, "config")));
        end
    end

    methods (Test)
        function crossingTrafficLowersTheArrivalSpeed(testCase)
            model = localModel();
            reference = crossingCruiseReference(model);
            testCase.verifyTrue(reference.yielding);
            testCase.verifyGreaterThan(reference.speed, 10.0);
            testCase.verifyLessThan(reference.speed, 12.0);
            testCase.verifyEqual(reference.stopStation, 86.4, AbsTol=1.0e-12);
            testCase.verifyGreaterThan(reference.clearTime, 3.3);
        end

        function absentOrClearedTrafficRestoresCruise(testCase)
            model = localModel();
            model.hasTarget = false;
            absent = crossingCruiseReference(model);
            model.hasTarget = true;
            model.targetHorizon.targetPosition(2, :) = ...
                model.targetHorizon.targetPosition(2, :)+40.0;
            cleared = crossingCruiseReference(model);
            testCase.verifyEqual(absent.speed, 15.0);
            testCase.verifyEqual(cleared.speed, 15.0);
            testCase.verifyFalse(absent.yielding || cleared.yielding);
        end

        function parallelTrafficRetainsItsExistingManeuverDomain(testCase)
            model = localModel();
            model.targetHorizon.targetPosition(2, :) = 1.5;
            model.targetHorizon.targetYaw(:) = pi;
            reference = crossingCruiseReference(model);
            testCase.verifyFalse(reference.yielding);
            testCase.verifyEqual(reference.speed, 15.0);
        end

        function aCrossingBehindTheEgoDoesNotSlowIt(testCase)
            model = localModel();
            model.targetHorizon.targetPosition(1, :) = 20.0;
            reference = crossingCruiseReference(model);
            testCase.verifyFalse(reference.yielding);
            testCase.verifyEqual(reference.speed, 15.0);
        end

        function aWiderCrossingFootprintDelaysArrival(testCase)
            model = localModel();
            original = crossingCruiseReference(model);
            model.targetHalfLength = 3.0;
            larger = crossingCruiseReference(model);
            testCase.verifyLessThan(larger.speed, original.speed);
            testCase.verifyGreaterThan(larger.clearTime, original.clearTime);
        end

        function anAlreadySlowerReferenceIsNotIncreased(testCase)
            model = localModel();
            model.referenceSpeed = 8.0;
            reference = crossingCruiseReference(model);
            testCase.verifyFalse(reference.yielding);
            testCase.verifyEqual(reference.speed, 8.0);
        end
    end
end

function model = localModel()
    cfg = collisionAvoidanceControllerConfig();
    time = (0:98)*0.05;
    lane = struct("segmentStart", [0.0, 0.0], "segment", [1000.0, 0.0], ...
        "segmentLength", 1000.0, "segmentStation", 0.0, "tangent", [1.0, 0.0]);
    model = struct("cfg", cfg, "hasTarget", true, "referenceSpeed", 15.0, ...
        "horizonSteps", 24, "tailSteps", 74, "sampleTime", 0.05, ...
        "egoHalfLength", 2.4, "egoHalfWidth", 0.95, ...
        "targetHalfLength", 2.4, "targetHalfWidth", 0.95, ...
        "initialEgoState", [46.5; 0.0; 0.0; 15.0; 0.0; 0.0], ...
        "lane", lane, "targetHorizon", struct( ...
            "targetPosition", [90.0*ones(size(time)); -23.2+8.0*time], ...
            "targetYaw", (pi/2.0)*ones(size(time))));
end
