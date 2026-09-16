classdef encounterCertificateScenarioTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
        end
    end
    methods (Test)
        function solverFailureStopsTheExperiment(testCase)
            testCase.verifyError(@() runEncounterCertificateScenario(ForceSolverFailure=true,DeadlineSeconds=inf), ...
                "collisionAvoidanceController:optimizationFailed");
        end
        function everyCrossingSampleSolvesOneTrajectory(testCase)
            report=runEncounterCertificateScenario(DeadlineSeconds=inf);
            testCase.verifyTrue(report.passed);
            testCase.verifyEqual(report.executedHolds,24);
            testCase.verifyEqual(report.trajectorySolverCallCount,ones(1,24));
            testCase.verifyEqual(report.solverCallCount,1+report.distanceSolverCallCount);
            % This tests solve/continuation behavior, not an early recovery
            % deadline. The dedicated long cruise-recovery test checks tracking.
            testCase.verifyFalse(any(report.terminalCommands));
        end
    end
end
