classdef straightRoadBoundaryConfigurationTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function aRoadFreeTrialReportsItsDiagnosticEnvelope(testCase)
            report = runExactStateRecursiveFeasibilityScenario(Scenario="cruise", ...
                SampleCount=1,DeadlineSeconds=30,InitialTrackingError=[3.9;0;0;0;0]);
            testCase.verifyTrue(report.passed);
            testCase.verifyFalse(report.roadBoundariesEnabled);
            testCase.verifyTrue(isnan(report.minimumSampledRoadMargin));
            testCase.verifyGreaterThanOrEqual(report.minimumSampledModelDomainMargin,0);
        end
        function anExplicitRoadBoundaryRejectsAnOverlappingFootprint(testCase)
            testCase.verifyError(@() runExactStateRecursiveFeasibilityScenario( ...
                Scenario="cruise",SampleCount=1,DeadlineSeconds=30,UseRoadBoundaries=true, ...
                InitialTrackingError=[3.9;0;0;0;0]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
        function aStateOutsideTheDiagnosticEnvelopeCanStillBeCertified(testCase)
            report=runExactStateRecursiveFeasibilityScenario(Scenario="cruise", ...
                SampleCount=1,DeadlineSeconds=30,InitialTrackingError=[4.1;0;0;0;0]);
            testCase.verifyTrue(report.passed);
            testCase.verifyLessThan(report.minimumSampledModelDomainMargin,0);
            testCase.verifyFalse(report.stateAndSlipBoundsEnforced);
        end
    end
end
