classdef circularArcExactStateScenarioTest < matlab.unittest.TestCase
    properties (TestParameter)
        curvature = struct('left',.01,'right',-.01);
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function circularCruiseUsesItsCurvedTrim(testCase,curvature)
            report = runExactStateRecursiveFeasibilityScenario(Scenario="cruise", ...
                RoadCurvature=curvature,SampleCount=3,DeadlineSeconds=30);
            testCase.verifyTrue(report.passed);
            testCase.verifyTrue(all(report.hardCertificateVerified));
            testCase.verifyEqual(report.trackingError(:,1),zeros(5,1),AbsTol=1e-12);
            testCase.verifyLessThan(max(abs(report.trackingError),[],'all'),1e-3);
            testCase.verifyGreaterThan(curvature*report.cruiseState(6),0);
            testCase.verifyFalse(report.roadBoundariesEnabled);
        end
    end
end
