classdef freeCompletionTimeTest < matlab.unittest.TestCase
    % A finite admission window leads to a certified infinite continuation.
    properties
        stationary
        oncoming
    end
    methods (TestClassSetup)
        function runExactPlantScenarios(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.stationary = runExactStateRecursiveFeasibilityScenario();
            testCase.oncoming = runExactStateRecursiveFeasibilityScenario(Scenario="oncoming");
        end
    end
    methods (Test)
        function aStationaryObstacleRemainsSafeBeyondTheOriginalHorizon(testCase)
            result = testCase.stationary;
            testCase.verifyTrue(result.passed);
            testCase.verifyTrue(all(result.planCertified));
            testCase.verifyGreaterThan(result.sampleCount,5*result.admissionSteps);
            testCase.verifyGreaterThan(result.minimumSampledSeparationMargin,0);
            testCase.verifyGreaterThan(result.minimumSampledRoadMargin,0);
        end

        function anOncomingVehicleDoesNotEndControlAfterPassing(testCase)
            result = testCase.oncoming;
            testCase.verifyTrue(result.passed);
            testCase.verifyTrue(result.terminalActive(end));
            testCase.verifyGreaterThan(result.finalTargetDistance,30);
            testCase.verifySize(result.input,[2,result.sampleCount+1]);
            testCase.verifyGreaterThan(result.minimumSampledSeparationMargin,0);
        end

        function terminalAdmissionCanExtendTheConfiguredWindow(testCase)
            result = testCase.oncoming;
            testCase.verifyGreaterThan(result.admissionSteps,result.configuration.controller.horizonSteps);
            testCase.verifyFalse(result.terminalActive(result.admissionSteps));
            testCase.verifyTrue(result.terminalActive(result.admissionSteps+1));
        end

        function laterSolverFailureCannotExhaustTheAcceptedSolution(testCase)
            result = testCase.oncoming;
            testCase.verifyTrue(result.solverFailureInjected);
            testCase.verifyTrue(all(result.retainedWitnessUsed(2:result.admissionSteps)));
            testCase.verifyTrue(all(result.planCertified(result.admissionSteps+1:end)));
            testCase.verifyEqual(result.maximumSlewViolation,0,AbsTol=0);
        end

        function sampledTerminalBrakingApproachesRestWithoutReverseMotion(testCase)
            result = testCase.stationary;
            speed = result.state(4,result.admissionSteps+1:end);
            testCase.verifyGreaterThanOrEqual(speed,zeros(size(speed)));
            testCase.verifyLessThan(diff(speed),zeros(1,numel(speed)-1));
            testCase.verifyLessThan(speed(end),1e-5);
        end

        function optimizingTheRetainedSuffixAlsoPreservesTerminalFeasibility(testCase)
            result = runExactStateRecursiveFeasibilityScenario(SampleCount=24,FailAfterAdmission=false);
            testCase.verifyTrue(result.passed);
            testCase.verifyFalse(result.solverFailureInjected);
            testCase.verifyTrue(all(result.planCertified));
        end
    end
end
