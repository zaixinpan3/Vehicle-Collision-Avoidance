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
        function aFailedOptimizationExecutesTheCarriedWitnessDownToTheTerminalLaw(testCase)
            result = runExactStateRecursiveFeasibilityScenario(Scenario="crossing",FailAfterAdmission=true,SampleCount=24);
            testCase.verifyTrue(result.completed);
            testCase.verifyTrue(result.passed);
            testCase.verifyTrue(all(result.candidateExecuted(2:end)));
            testCase.verifyTrue(all(result.candidateVerified(2:end)));
            testCase.verifyEqual(result.optimizedStages(1:result.admissionSteps+1),result.admissionSteps:-1:0);
            testCase.verifyTrue(all(result.terminalActive(result.admissionSteps+2:end)));
            testCase.verifyFalse(any(result.terminalActive(1:result.admissionSteps)));
            testCase.verifyLessThan(result.state(4,end),result.state(4,1));
            testCase.verifyGreaterThanOrEqual(result.minimumSampledSeparationMargin,0);
        end
    end
end
