classdef encounterCertificateScenarioTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
        end
    end
    methods (Test)
        function solverFailureEndsTheScenarioWithoutFallback(testCase)
            report = runEncounterCertificateScenario(ForceSolverFailure=true);
            testCase.verifyFalse(report.completed);
            testCase.verifyEqual(report.executedHolds,1);
            testCase.verifyEqual(report.failureIdentifier,"collisionAvoidanceController:noCertifiedContinuation");
            testCase.verifyFalse(any(report.retainedWitnessUsed));
            testCase.verifyTrue(all(isnan(report.input(:,end))));
        end
        function freshSolutionsContinueBeyondTheFirstPredictionEnd(testCase)
            report = runEncounterCertificateScenario();
            testCase.verifyTrue(report.passed);
            testCase.verifyFalse(any(report.retainedWitnessUsed));
            testCase.verifyFalse(any(report.terminalActive));
            testCase.verifyGreaterThan(report.executedHolds,report.admissionSteps);
            testCase.verifyLessThan(abs(report.finalCruiseError(3)),0.02);
        end
    end
end
