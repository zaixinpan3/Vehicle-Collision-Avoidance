classdef encounterCertificateScenarioTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
        end
    end
    methods (Test)
        function solverFailureKeepsExecutingTheCertifiedWitnessUntilExit(testCase)
            report = runEncounterCertificateScenario();
            testCase.verifyFalse(report.failure.occurred);
            testCase.verifyTrue(report.discharged);
            testCase.verifyGreaterThan(report.executedIntervals,1);
            testCase.verifyGreaterThan(report.fallbackCount,0);
            testCase.verifyEqual(report.deadline,1.6,AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(report.exitTime,report.deadline);
            testCase.verifyGreaterThanOrEqual(report.minimumSampledRectangleDistance,report.requiredDistance);
            testCase.verifyLessThanOrEqual(report.maximumSampledClfResidual,0);
            testCase.verifyLessThanOrEqual(report.maximumEndpointBoxViolation,0);
        end

        function freshSolutionsCanCompleteTheDeclaredCrossing(testCase)
            report = runEncounterCertificateScenario(ForceSolverFailure=false);
            testCase.verifyFalse(report.failure.occurred);
            testCase.verifyTrue(report.discharged);
            testCase.verifyGreaterThan(report.executedIntervals,1);
            % A solver success flag need not pass strict inherited-margin
            % checking. Some successful updates and safe completion matter;
            % rejecting a numerically weaker replacement is correct behavior.
            testCase.verifyLessThan(report.fallbackCount,report.executedIntervals-1);
            testCase.verifyLessThanOrEqual(report.exitTime,report.deadline);
            testCase.verifyGreaterThanOrEqual(report.minimumSampledRectangleDistance,report.requiredDistance);
            testCase.verifyLessThanOrEqual(report.maximumEndpointBoxViolation,0);
        end
    end
end
