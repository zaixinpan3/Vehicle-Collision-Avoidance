classdef lateralDeviationLimitTest < matlab.unittest.TestCase
    properties (TestParameter)
        curvature={0,.005,-.005};
        operatingCurvature={0,.005};
        side={-1,1};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for directory=["controller","config","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,directory)));
            end
        end
    end
    methods (Test)
        function experimentThresholdIsNotAControllerConstraint(testCase)
            cfg=collisionAvoidanceControllerConfig();
            testCase.verifyFalse(isfield(cfg.controller,'lateralDeviationMaximumMeters'));
            testCase.verifyFalse(cfg.controller.enforceRoadBoundaries);
        end
        function measuredDeviationFollowsTheRotatedOrCurvedPath(testCase,curvature,side)
            curve=struct('origin',[20;-30],'heading',.7,'curvature',curvature,'length',100);
            p=laneGeometry.referencePose(40,side*10,curve);
            projection=laneGeometry.project(p,struct('referenceCurve',curve));
            testCase.verifyEqual(projection.lateralPosition,side*10,AbsTol=1e-11);
        end
        function terminalContinuationCanCrossTheExperimentThreshold(testCase)
            model=localModel([0;9.8;.1;8;0;0]);
            context=terminalSafeSet.context(model);
            testCase.verifyTrue(terminalSafeSet.member(context,model.initialState,0));
            path=terminalSafeSet.continuation(context,model.initialState);
            testCase.verifyGreaterThan(max(path.pose(2,:)),10);
        end
        function anOutlyingStateStillReceivesAnOptimizedCommand(testCase,operatingCurvature,side)
            model=localModel([0;side*10.5;0;8;0;0]);
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',operatingCurvature,'length',200));
            ego=struct('position',[0;side*10.5],'yaw',0,'speed',8);
            [command,~,problem]=collisionAvoidanceController(ego,[],road,model.cfg,[]);
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyEqual(problem.solution.affineStates(2,1),side*10.5,AbsTol=1e-10);
        end
        function simulationStopsAndReportsTheActualDeviation(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=5, ...
                LateralDeviationLimitMeters=.005,ControllerConfiguration=struct('solver',struct('timeLimitSeconds',5)));
            r=report.results;
            testCase.verifyEqual(r.outcome,"lateralLimitExceeded");
            testCase.verifyTrue(r.lateralLimitExceeded);
            testCase.verifyFalse(r.controlUnavailable);
            testCase.verifyLessThan(r.executedFrames,5);
            testCase.verifyGreaterThan(r.maximumReplayLateralDeviationMeters,.005);
            testCase.verifyEqual(r.lateralLimitTimeSeconds,0);
        end
        function defaultSimulationThresholdIsTenMeters(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=1, ...
                ControllerConfiguration=struct('solver',struct('timeLimitSeconds',5)));
            testCase.verifyEqual(report.results.lateralDeviationLimitMeters,10);
        end
    end
end

function model=localModel(x)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'solver',struct('timeLimitSeconds',5)));
    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
    [~,lane,road]=readControllerInputs(ego,[],struct('centerline',[-100,0;1000,0]),cfg);
    model=struct('cfg',cfg,'initialState',x,'lane',lane,'road',road,'target',[],'targetEpoch',[], ...
        'sampleIndex',0,'nominalReference',nonlinearBicycleModel.cruise(cfg,0), ...
        'uncertainty',struct('egoGenerator',zeros(6),'collisionGenerator',zeros(6),'target',[]));
end
