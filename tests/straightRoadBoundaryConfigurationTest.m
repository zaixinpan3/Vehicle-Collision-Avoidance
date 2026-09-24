classdef straightRoadBoundaryConfigurationTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
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
        function anExplicitRoadBoundaryCertifiesAFootprintNearTheEdge(testCase)
            report = runExactStateRecursiveFeasibilityScenario( ...
                Scenario="cruise",SampleCount=1,DeadlineSeconds=30,UseRoadBoundaries=true, ...
                InitialTrackingError=[3.9;0;0;0;0]);
            testCase.verifyTrue(report.passed);
            testCase.verifyTrue(report.roadBoundariesEnabled);
            testCase.verifyGreaterThanOrEqual(report.minimumSampledRoadMargin,0);
        end
        function anExplicitRoadBoundaryRejectsAnOverlappingFootprint(testCase)
            testCase.verifyError(@() runExactStateRecursiveFeasibilityScenario( ...
                Scenario="cruise",SampleCount=1,DeadlineSeconds=30,UseRoadBoundaries=true, ...
                InitialTrackingError=[4.2;0;0;0;0]), ...
                'collisionAvoidanceController:optimizationFailed');
        end
        function aStrictBoundaryShorterThanTheHorizonIsRejected(testCase)
            testCase.verifyError(@() runExactStateRecursiveFeasibilityScenario( ...
                Scenario="cruise",SampleCount=1,DeadlineSeconds=30,UseRoadBoundaries=true, ...
                RoadCoveragePolicy="strict",RoadBoundaryParameterRange=[-100;12]), ...
                'collisionAvoidanceController:roadBoundaryCoverageGap');
        end
        function aPerceptionLimitedBoundaryConstrainsOnlyTheCoveredCells(testCase)
            report = runExactStateRecursiveFeasibilityScenario( ...
                Scenario="cruise",SampleCount=3,DeadlineSeconds=30,UseRoadBoundaries=true, ...
                RoadCoveragePolicy="perceptionLimited",RoadBoundaryParameterRange=[-100;12], ...
                InitialTrackingError=[2;0;0;0;0]);
            testCase.verifyTrue(report.passed);
            testCase.verifyEqual(report.executedHolds,3);
            testCase.verifyGreaterThanOrEqual(report.minimumSampledRoadMargin,0);
            collisionAvoidanceController("resetNominalTrajectory");
            cfg = collisionAvoidanceControllerConfig();
            ego = struct("position",[0;2],"yaw",0,"speed",8,"lateralVelocity",0,"yawRate",0, ...
                "stateTime",0,"controllerStateErrorBound",zeros(6,1), ...
                "perception",struct("time",0,"range",30,"completeWithinRange",true));
            boundary = struct("origin",zeros(2,1),"longitudinalDirection",[1;0],"lateralDirection",[0;1], ...
                "coefficients",[0;0;-5],"parameterRange",[-100;2000],"safeSideSign",1,"coveragePolicy","strict");
            long = boundary;short = boundary;short.parameterRange = [-100;12];short.coveragePolicy = "perceptionLimited";
            road = struct("centerline",[-100,0;2000,0],"lateralClearance",[5;5]);
            road.boundaries = long;
            [~,~,fullProblem] = collisionAvoidanceController(ego,[],road,cfg);
            collisionAvoidanceController("resetNominalTrajectory");
            road.boundaries = short;
            [~,~,partialProblem] = collisionAvoidanceController(ego,[],road,cfg);
            fullRows = nnz(startsWith(fullProblem.program.geometry.label,"road:"));
            partialRows = nnz(startsWith(partialProblem.program.geometry.label,"road:"));
            testCase.verifyGreaterThan(fullRows,0);
            testCase.verifyGreaterThan(partialRows,0);
            testCase.verifyLessThan(partialRows,fullRows);
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
