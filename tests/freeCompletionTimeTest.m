classdef freeCompletionTimeTest < matlab.unittest.TestCase
    % The terminal horizon is a moving certificate, not an execution deadline.
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function cruisingContinuesPastTheOriginalPredictionHorizon(testCase)
            result = runExactStateRecursiveFeasibilityScenario(Scenario="crossing",SampleCount=24);
            testCase.verifyTrue(result.passed);
            testCase.verifyGreaterThan(result.executedHolds,result.admissionSteps);
            testCase.verifyGreaterThan(diff(result.predictionEndTime),zeros(1,result.executedHolds));
            testCase.verifyGreaterThanOrEqual(result.horizonSteps,result.configuration.controller.horizonSteps);
            testCase.verifyFalse(any(result.terminalActive));
            testCase.verifyLessThan(max(abs(result.state(4,:)-8)),0.02);
        end
        function aFailedOptimizationEndsBeforeAnyTerminalTakeover(testCase)
            result = runExactStateRecursiveFeasibilityScenario(Scenario="crossing",FailAfterAdmission=true);
            testCase.verifyFalse(result.passed);
            testCase.verifyEqual(result.executedHolds,1);
            testCase.verifyFalse(any(result.terminalActive));
            testCase.verifyFalse(any(result.retainedWitnessUsed));
        end
    end
end
