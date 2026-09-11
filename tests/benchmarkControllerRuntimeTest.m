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
            previousThreads = maxNumCompThreads(1);
            testCase.addTeardown(@() maxNumCompThreads(previousThreads));
            cfg = collisionAvoidanceControllerConfig(struct("referenceSpeed",2,"controller", struct("horizonSteps",4,"sampleTime",0.1,"stationTrustRadius",5)));
            ego = struct("position", [0; 0], "yawAngle", 0, "speed", 1.8);
            ego.stateTime = 0;
            ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
            road = [0, 0; 2000, 0];
            command = collisionAvoidanceController(ego, encounterTestFixture.stationaryTarget(), road, cfg, []);
            trial = struct("command", {{command}}, "controllerEgoEstimate", {{ego}}, ...
                "targetEstimate", {{encounterTestFixture.stationaryTarget()}}, "controllerConfiguration", cfg, ...
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
