classdef straightRoadBoundaryConfigurationTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function noRoadBoundaryAllowsSpaceInsideTheModelDomain(testCase)
            report = runExactStateRecursiveFeasibilityScenario(Scenario="cruise", ...
                SampleCount=1,InitialTrackingError=[3.9;0;0;0;0]);
            testCase.verifyTrue(report.passed);
            testCase.verifyFalse(report.roadBoundariesEnabled);
            testCase.verifyTrue(isnan(report.minimumSampledRoadMargin));
            testCase.verifyGreaterThanOrEqual(report.minimumSampledModelDomainMargin,0);
        end
        function anExplicitRoadBoundaryRejectsAnOverlappingFootprint(testCase)
            testCase.verifyError(@() runExactStateRecursiveFeasibilityScenario( ...
                Scenario="cruise",SampleCount=1,UseRoadBoundaries=true, ...
                InitialTrackingError=[3.9;0;0;0;0]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
        function removingTheRoadDoesNotRemoveTheModelDomain(testCase)
            testCase.verifyError(@() runExactStateRecursiveFeasibilityScenario( ...
                Scenario="cruise",SampleCount=1,InitialTrackingError=[4.1;0;0;0;0]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
    end
end
