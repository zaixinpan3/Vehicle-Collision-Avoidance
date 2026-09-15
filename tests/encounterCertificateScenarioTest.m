classdef encounterCertificateScenarioTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
        end
    end
    methods (Test)
        function solverFailureStopsTheExperiment(testCase)
            testCase.verifyError(@() runEncounterCertificateScenario(ForceSolverFailure=true), ...
                "collisionAvoidanceController:optimizationFailed");
        end
        function everyCrossingSampleSolvesOnce(testCase)
            report=runEncounterCertificateScenario(DeadlineSeconds=inf);
            testCase.verifyTrue(report.passed);
            testCase.verifyEqual(report.executedHolds,24);
            testCase.verifyEqual(report.solverCallCount,ones(1,24));
            testCase.verifyLessThan(abs(report.state(4,end)-8),1e-4);
        end
    end
end
