classdef controllerInputGeometryTest < matlab.unittest.TestCase
    properties (TestParameter)
        curvature={0,.005,-.005};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function referenceProjectionRecoversLaneCoordinates(testCase,curvature)
            curve=laneGeometry.validateReferenceCurve(struct('origin',[2;3], ...
                'heading',.4,'curvature',curvature,'length',200));
            station=[-10,0,40,210];lateral=[.2,-.5,1,0];
            position=laneGeometry.referencePose(station,lateral,curve);
            projection=laneGeometry.projectReferenceCurve(position,curve,station);
            testCase.verifyEqual(projection.station,station,AbsTol=1e-11);
            testCase.verifyEqual(projection.lateralPosition,lateral,AbsTol=1e-11);
        end
        function oneTargetNeedsNoUncertaintyOrAccelerationCertificate(testCase)
            [ego,road,cfg,target]=localFixture();
            [parsed,~,~,observation]=readControllerInputs(ego,target,road,cfg);
            testCase.verifyEqual(parsed.modelState,[0;0;0;8;0;0]);
            q=predictiveSafetyGeometry.target(observation,target,cfg);
            testCase.verifyEqual(q(1:6),[20;1;.3;6;0;.08],AbsTol=1e-14);
        end
        function multipleTargetsAreRejected(testCase)
            [ego,road,cfg,target]=localFixture();
            testCase.verifyError(@()readControllerInputs(ego,[target,target],road,cfg), ...
                'MATLAB:readControllerInputs:expectedScalar');
        end
        function aTargetRectangleNeedsAnExplicitBodyHeading(testCase)
            [ego,road,cfg,target]=localFixture();target=rmfield(target,'targetYawInertial');
            [~,~,~,obs]=readControllerInputs(ego,target,road,cfg);
            testCase.verifyError(@()predictiveSafetyGeometry.target(obs,target,cfg), ...
                'collisionAvoidanceController:missingBodyHeading');
        end
        function estimatorAccelerationCanSupplyTheConstantHeadingRate(testCase)
            [ego,road,cfg,target]=localFixture();target=rmfield(target,'targetYawRate');
            v=target.targetVelocityInertial;
            target.targetAccelerationInertial=.08*[-v(2);v(1)];
            [~,~,~,obs]=readControllerInputs(ego,target,road,cfg);
            testCase.verifyEqual(obs.yawRate,.08,AbsTol=1e-14);
        end
        function theRoadCorridorMustHavePositiveClearances(testCase)
            [ego,road,cfg,target]=localFixture();road.lateralClearance=[4;-1];
            testCase.verifyError(@()readControllerInputs(ego,target,road,cfg), ...
                'collisionAvoidanceController:invalidRoadClearance');
        end
        function varyingCurvatureIsOutsideTheCurrentTerminalModel(testCase)
            curve=struct('origin',[0;0],'heading',0,'curvature',0, ...
                'length',100,'curvatureProfile',[0,0;100,.01]);
            testCase.verifyError(@()laneGeometry.validateReferenceCurve(curve), ...
                'collisionAvoidanceController:unsupportedReference');
        end
    end
end
function [ego,road,cfg,target]=localFixture()
    cfg=collisionAvoidanceControllerConfig();
    ego=struct('position',[0;0],'yaw',0,'speed',8);
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[4;4]);
    target=struct('targetPositionInertial',[20;1],'targetVelocityInertial',6*[cos(.3);sin(.3)], ...
        'targetYawInertial',.3,'targetYawRate',.08);
end
