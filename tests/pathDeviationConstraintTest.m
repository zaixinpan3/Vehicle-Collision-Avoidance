classdef pathDeviationConstraintTest < matlab.unittest.TestCase
    % Center-position corridor geometry and hard finite-prediction behavior.
    properties (TestParameter)
        curvature = {0,.005,-.005}
        side = {-1,1}
        invalidLimit = {0,-1,NaN,[10,20]}
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function tenMeterDefaultCanBeExplicitlyDisabled(testCase)
            cfg=collisionAvoidanceControllerConfig();
            disabled=collisionAvoidanceControllerConfig(struct('controller', ...
                struct('maximumLateralDeviationMeters',Inf)));
            testCase.verifyEqual(cfg.controller.maximumLateralDeviationMeters,10,AbsTol=0);
            testCase.verifyEqual(disabled.controller.maximumLateralDeviationMeters,Inf);
        end
        function nonpositiveOrUndefinedLimitsAreRejected(testCase,invalidLimit)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('controller', ...
                struct('maximumLateralDeviationMeters',invalidLimit))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
        function regionContainsBothNormalBoundariesAndExcludesOutside(testCase,curvature,side)
            lane=localLane(curvature);origin=laneGeometry.referencePose(30,0,lane.referenceCurve);
            region=laneGeometry.deviationRegion(origin,lane,10);
            boundary=laneGeometry.referencePose(30,side*10,lane.referenceCurve)-origin;
            outside=laneGeometry.referencePose(30,side*10.01,lane.referenceCurve)-origin;
            testCase.verifyLessThanOrEqual(localResidual(region,boundary),1e-10);
            testCase.verifyGreaterThan(localResidual(region,outside),0);
        end
        function acceptedCurvedRegionPointsStayWithinTheTrueCorridor(testCase,curvature)
            lane=localLane(curvature);origin=laneGeometry.referencePose(70,3,lane.referenceCurve);
            region=laneGeometry.deviationRegion(origin,lane,10);
            [a,b]=meshgrid(-12:.3:12);offsets=[a(:).';b(:).'];
            accepted=localAccepted(region,offsets);
            projection=laneGeometry.project(origin+offsets(:,accepted),lane);
            testCase.verifyNotEmpty(projection.lateralPosition);
            testCase.verifyLessThanOrEqual(max(abs(projection.lateralPosition)),10+1e-10);
        end
        function targetFreeRecoveryHonorsTheLimitOnBothSides(testCase,side)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
                'controller',struct('horizonSteps',8)));
            ego=struct('position',[0;side*9.8],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
            road=struct('centerline',[-100,0;1000,0]);
            [command,inputs,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            projection=laneGeometry.project(problem.predictedState(1:2,:),problem.model.lane);
            testCase.verifyLessThanOrEqual(max(abs(projection.lateralPosition)),10+cfg.solver.feasibilityTolerance);
            testCase.verifyLessThanOrEqual(problem.metadata.predictionAgreement.maximumLateralDeviationMeters,10);
            testCase.verifyTrue(problem.metadata.search.clfStageCompleted);
            testCase.verifyEqual(problem.metadata.clfFunction,"quadraticTransverseError");
            testCase.verifyEqual(command.actuatorInput,inputs(:,1),AbsTol=0);
            testCase.verifyFalse(problem.metadata.roadConstraintsEnforced);
        end
        function fixedInitialStateOutsideTheHardCorridorCannotBeHiddenBySafetySlack(testCase,side)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
                'controller',struct('horizonSteps',8)));
            ego=struct('position',[0;side*10.1],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
            road=struct('centerline',[-100,0;1000,0]);
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function guidancePointsInwardAtTheCorridorBoundary(testCase,side)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8));lane=localLane(0);
            [position,heading]=laneGeometry.referencePose(0,side*10,lane.referenceCurve);
            x=[position;heading;8;0;0];
            q=[position+[8;0];pi;8;0;0;1.6;2.4;.95;0;0];
            desired=predictiveSafetyGeometry.potentialGuidance(x,lane,q,0,cfg,side);
            testCase.verifyLessThanOrEqual(side*sin(desired-heading),1e-12);
        end
    end
end
function lane=localLane(curvature)
    lane=struct('referenceCurve',struct('origin',[2;-3],'heading',.4, ...
        'curvature',curvature,'length',200));
end
function residual=localResidual(region,offset)
    residual=max(region.a*offset-region.b);
    if isfinite(region.radius),residual=max(residual,norm(offset-region.center)-region.radius);end
end
function accepted=localAccepted(region,offsets)
    accepted=all(region.a*offsets<=region.b+1e-12,1);
    if isfinite(region.radius),accepted=accepted & vecnorm(offsets-region.center)<=region.radius;end
end
