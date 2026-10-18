classdef lateralDeviationLimitTest < matlab.unittest.TestCase
    properties (TestParameter)
        curvature={0,.005,-.005};
        % Controller tests use operating points in the precomputed CLF table.
        operatingCurvature={0,.005};
        side={-1,1};
        invalidLimit={0,-1,NaN,[10,10]};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function defaultCenterLimitIsIndependentOfRoadEdges(testCase)
            cfg=collisionAvoidanceControllerConfig();
            testCase.verifyEqual(cfg.controller.lateralDeviationMaximumMeters,10);
            testCase.verifyFalse(cfg.controller.enforceRoadBoundaries);
        end
        function invalidLimitsAreRejected(testCase,invalidLimit)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('controller', ...
                struct('lateralDeviationMaximumMeters',invalidLimit))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
        function deviationFollowsTheRotatedOrCurvedPath(testCase,curvature,side)
            curve=struct('origin',[20;-30],'heading',.7,'curvature',curvature,'length',100);
            p=laneGeometry.referencePose(40,side*10,curve);
            margin=laneGeometry.lateralDeviationMargin(p,struct('referenceCurve',curve),10);
            testCase.verifyEqual(margin,0,AbsTol=1e-11);
        end
        function centerBoundaryDoesNotSubtractTheVehicleFootprint(testCase,side)
            model=localModel([0;side*10;pi/2;0;0;0]);
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(model),model.initialState,0);
            testCase.verifyTrue(member);
            testCase.verifyEqual(info.lateralDeviationMargin,0,AbsTol=1e-12);
        end
        function absoluteObserverPositionErrorTightensTheLimit(testCase)
            model=localModel([0;9.9;0;0;0;0]);
            model.uncertainty.egoGenerator(2,2)=.2;
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(model),model.initialState,0);
            testCase.verifyFalse(member);
            testCase.verifyEqual(info.reason,"continuationExceedsLateralLimit");
            testCase.verifyEqual(info.lateralDeviationMargin,-.1,AbsTol=1e-12);
        end
        function terminalBrakingMustRemainInsideTheLimit(testCase)
            model=localModel([0;9.8;.1;8;0;0]);
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(model),model.initialState,0);
            testCase.verifyFalse(member);
            testCase.verifyEqual(info.reason,"continuationExceedsLateralLimit");
            testCase.verifyLessThan(info.lateralDeviationMargin,0);
        end
        function anExplicitInfiniteLimitRestoresUnboundedContinuation(testCase)
            model=localModel([0;9.8;.1;8;0;0]);model.cfg.controller.lateralDeviationMaximumMeters=Inf;
            testCase.verifyTrue(terminalSafeSet.member(terminalSafeSet.context(model),model.initialState,0));
        end
        function collisionSlackCannotRelaxTheCurrentLateralLimit(testCase,operatingCurvature)
            cfg=collisionAvoidanceControllerConfig(struct('controller',struct('maximumHorizonSteps',16), ...
                'solver',struct('timeLimitSeconds',5)));
            ego=struct('position',[0;10.1],'yaw',0,'speed',8);
            target=struct('targetPositionInertial',[0;10.1],'targetVelocityInertial',[0;0], ...
                'targetYawInertial',0,'targetSideslip',0);
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',operatingCurvature,'length',200));
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function solvedAffinePredictionRespectsTheCorridor(testCase,operatingCurvature,side)
            model=localModel([0;side*9.8;0;8;0;0]);
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',operatingCurvature,'length',200));
            ego=struct('position',[0;side*9.8],'yaw',0,'speed',8);
            [~,~,problem]=collisionAvoidanceController(ego,[],road,model.cfg,[]);
            margin=laneGeometry.lateralDeviationMargin(problem.solution.affineStates(1:2,:),problem.model.lane,10);
            testCase.verifyGreaterThanOrEqual(min(margin),-1e-5);
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
