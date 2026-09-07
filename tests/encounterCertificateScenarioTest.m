classdef encounterCertificateScenarioTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
        end
    end
    methods (Test)
        function solverFailureStopsBeforeApplyingASecondInput(testCase)
            report = runEncounterCertificateScenario();
            testCase.verifyTrue(report.failure.occurred);
            testCase.verifyEqual(report.failure.identifier, "collisionAvoidanceController:noCertifiedContinuation");
            testCase.verifyEqual(report.failure.time,0.1,AbsTol=1e-12);
            testCase.verifyFalse(report.discharged);
            testCase.verifyEqual(report.executedIntervals,1);
            testCase.verifySize(report.issuedInput,[2,1]);
            testCase.verifyEqual(report.fallbackCount,0);
            testCase.verifyEqual(report.simulatedDuration,0.1,AbsTol=1e-12);
            testCase.verifyEqual(report.deadline,1.6,AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(report.minimumSampledRectangleDistance,report.requiredDistance);
            testCase.verifyLessThanOrEqual(report.maximumSampledClfResidual,0);
            testCase.verifyLessThanOrEqual(report.maximumEndpointBoxViolation,0);
        end

        function freshSolutionsCanCompleteTheDeclaredCrossing(testCase)
            report = runEncounterCertificateScenario(ForceSolverFailure=false);
            testCase.verifyFalse(report.failure.occurred);
            testCase.verifyTrue(report.discharged);
            testCase.verifyEqual(report.executedIntervals,15);
            testCase.verifyEqual(report.fallbackCount,0);
            testCase.verifyEqual(report.exitTime,1.5,AbsTol=1e-12);
        end
    end
end
