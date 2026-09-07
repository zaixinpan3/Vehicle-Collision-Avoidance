classdef benchmarkControllerRuntimeTest < matlab.unittest.TestCase
    %benchmarkControllerRuntimeTest Preserve input units in recorded replay.

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            for folder = ["controller", "config", "scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, folder)));
            end
        end
    end

    methods (Test)
        function brakingRatioAndPhysicalAccelerationHaveDistinctUnits(testCase)
            cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 4)));
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", 14.8);
            road = [0, 0; 2000, 0];
            command = collisionAvoidanceController(ego, [], road, cfg, []);
            trial = struct("command", {{command}}, "controllerEgoEstimate", {{ego}}, ...
                "targetEstimate", {{[]}}, "controllerConfiguration", cfg, ...
                "controlTime", [0; cfg.controller.sampleTime], ...
                "perception", struct("roadBoundaryFit", {{struct("roadGeometry", road)}}));

            report = benchmarkControllerRuntime(trial, Repetitions=2, PrepareController=false);
            expectedRatio = repmat(command.brakingRatio, 2, 1);
            expectedAcceleration = expectedRatio*modifiedFialaTire.accelerationGain(cfg);

            testCase.verifyGreaterThan(abs(command.brakingRatio), 1.0e-4);
            testCase.verifyEqual(report.samples.brakingRatio, expectedRatio, AbsTol=1.0e-10);
            testCase.verifyEqual(report.samples.accelerationMetersPerSecondSquared, ...
                expectedAcceleration, AbsTol=1.0e-10);
            testCase.verifyGreaterThan(abs(expectedAcceleration(1)-expectedRatio(1)), 1.0e-3);
            testCase.verifyEqual(report.summary.maximumBrakingRatioDifference, zeros(2, 1), AbsTol=1.0e-10);
            testCase.verifyEqual(report.summary.maximumAccelerationDifferenceMetersPerSecondSquared, ...
                zeros(2, 1), AbsTol=1.0e-10);
        end
    end
end
