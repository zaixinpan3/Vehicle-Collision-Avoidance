classdef encounterCertificateScenarioTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
        end
    end
    methods (Test)
        function boundedResidualsAndSolverFailuresContinueToGuardedExit(testCase)
            report = runEncounterCertificateScenario();
            testCase.verifyTrue(report.discharged);
            testCase.verifyEqual(report.executedIntervals,15);
            testCase.verifyEqual(report.fallbackCount,15);
            testCase.verifyEqual(report.exitTime,1.5,AbsTol=1e-12);
            testCase.verifyEqual(report.deadline,1.6,AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(report.minimumSampledRectangleDistance,report.requiredDistance);
            testCase.verifyLessThanOrEqual(report.maximumSampledClfResidual,0);
            testCase.verifyLessThanOrEqual(report.maximumEndpointBoxViolation,0);
        end
    end
end
