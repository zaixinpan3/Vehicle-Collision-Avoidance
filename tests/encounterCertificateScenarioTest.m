classdef encounterCertificateScenarioTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
        end
    end
    methods (Test)
        function solverFailureExecutesTheVerifiedCarriedWitness(testCase)
            report = runEncounterCertificateScenario(ForceSolverFailure=true);
            testCase.verifyTrue(report.completed);
            testCase.verifyTrue(report.passed);
            testCase.verifyTrue(all(report.retainedWitnessUsed(2:end)));
            testCase.verifyFalse(any(isnan(report.input(:))));
            testCase.verifyEqual(report.pcbfValue,zeros(size(report.pcbfValue)));
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
