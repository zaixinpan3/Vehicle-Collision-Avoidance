classdef maneuverCouplingTest < matlab.unittest.TestCase
% maneuverCouplingTest Coupled-acceleration metric behavior.

    methods (TestClassSetup)
        function addRepositoryPaths(testCase)
            repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture( ...
                    fullfile(repositoryRoot, "scripts")));
        end
    end

    methods (Test)
        function failedAdmissionHasNoMeasuredManeuver(testCase)
            result = localResult(5,0);
            coupling = evaluateManeuverCoupling(result);
            testCase.verifyFalse(coupling.available);
            testCase.verifyFalse(coupling.significant);
            testCase.verifyEqual(coupling.coupledDuration,0);
            testCase.verifyTrue(isnan(coupling.maximumAbsoluteLongitudinalAcceleration));
        end

        function uninterruptedThresholdCrossingIsSignificant(testCase)
            result = localResult( ...
                [5.0; 4.8; 4.6; 4.4], ...
                [0.0; 0.4; 0.8; 1.2]);

            coupling = evaluateManeuverCoupling( ...
                result, MinimumCoupledDuration=0.3);

            testCase.verifyTrue(coupling.significant);
            testCase.verifyEqual(coupling.coupledSampleCount, 3);
            testCase.verifyEqual( ...
                coupling.maximumContinuousCoupledDuration, ...
                0.3, AbsTol=1.0e-12);
            testCase.verifyGreaterThanOrEqual( ...
                coupling.maximumAbsoluteLongitudinalAcceleration, 1.0);
            testCase.verifyGreaterThanOrEqual( ...
                coupling.maximumAbsoluteLateralAcceleration, 3.0);
        end

        function separatedThresholdCrossingsDoNotMeetDuration(testCase)
            result = localResult( ...
                [5.0; 4.8; 4.8; 4.6; 4.6], ...
                [0.0; 0.4; 0.4; 0.8; 0.8]);

            coupling = evaluateManeuverCoupling( ...
                result, MinimumCoupledDuration=0.2);

            testCase.verifyFalse(coupling.significant);
            testCase.verifyEqual(coupling.coupledSampleCount, 2);
            testCase.verifyEqual(coupling.coupledDuration, ...
                0.2, AbsTol=1.0e-12);
            testCase.verifyEqual( ...
                coupling.maximumContinuousCoupledDuration, ...
                0.1, AbsTol=1.0e-12);
        end
    end
end

function result = localResult(longitudinalVelocity, lateralVelocity)
    sampleCount = numel(longitudinalVelocity);
    result = struct( ...
        "controlTime", 0.1*(0:(sampleCount-1)).', ...
        "controlState", [ ...
            zeros(sampleCount, 3), ...
            longitudinalVelocity(:), lateralVelocity(:), ...
            zeros(sampleCount, 1)]);
end
