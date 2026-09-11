classdef encounterCertificateScenarioTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
        end
    end
    methods (Test)
        function solverFailureKeepsTheCrossingAndTerminalContinuationFeasible(testCase)
            report = runEncounterCertificateScenario();
            testCase.verifyTrue(all(report.planCertified));
            testCase.verifyGreaterThan(nnz(report.retainedWitnessUsed),0);
            testCase.verifyTrue(report.terminalActive(end));
            testCase.verifyGreaterThanOrEqual(report.minimumSampledSeparationMargin,0);
        end
        function freshSolutionsRetainTheIndefiniteCrossingCertificate(testCase)
            report = runEncounterCertificateScenario(ForceSolverFailure=false);
            testCase.verifyTrue(all(report.planCertified));
            testCase.verifyLessThan(nnz(report.retainedWitnessUsed),report.admissionSteps-1);
            testCase.verifyTrue(report.terminalActive(end));
            testCase.verifyGreaterThanOrEqual(report.minimumSampledSeparationMargin,0);
        end
    end
end
