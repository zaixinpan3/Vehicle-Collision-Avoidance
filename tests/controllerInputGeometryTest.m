classdef controllerInputGeometryTest < matlab.unittest.TestCase
    properties (TestParameter)
        curvature={0,.005,-.005};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'estimator')));
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
            testCase.verifyEqual(q(1:7),[20;1;.3;6;0;.08;1.6],AbsTol=1e-14);
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
        function tangentialAccelerationIsProjectedFromTheInertialVector(testCase)
            [ego,road,cfg,target]=localFixture();
            v=target.targetVelocityInertial;
            target.targetAccelerationInertial=1.2*v/6+(6*sin(.08)/1.6)*[-v(2);v(1)];
            [~,~,~,obs]=readControllerInputs(ego,target,road,cfg);
            q=predictiveSafetyGeometry.target(obs,target,cfg);
            testCase.verifyEqual(q(5),1.2,AbsTol=1e-14);
            testCase.verifyEqual(q(6),.08,AbsTol=1e-14);
        end
        function estimatorPublishesItsRearAxleDistance(testCase)
            settings=nrmmTrackingConfig();domain=settings.target.domain;domain.rearAxleDistance=1.8;
            domain.speedMinimum=1;domain.sideslipMaximum=.1;
            domain.yawRateMaximum=domain.speedMaximum*sin(domain.sideslipMaximum)/domain.rearAxleDistance;
            domain.accelerationNormBound=hypot(domain.scalarAccelerationMaximum,domain.speedMaximum*domain.yawRateMaximum);
            v=6*[cos(.38);sin(.38)];acceleration=1.2*v/6+6*sin(.08)/1.8*[-v(2);v(1)];
            [~,estimate]=nrmmTargetTrackerDerivative([20;1;v;acceleration], ...
                struct('bodyVelocity',[8;0],'yawRate',0),domain);
            testCase.verifyEqual(estimate.targetRearAxleDistance,1.8);
            testCase.verifyEqual(estimate.targetScalarAcceleration,1.2,AbsTol=1e-14);
            testCase.verifyEqual(estimate.targetSideslip,.08,AbsTol=1e-14);
        end
        function explicitAccelerationAndGeometryOverrideTheDefaults(testCase)
            [ego,road,cfg,target]=localFixture();target.targetScalarAcceleration=-1.2;
            target.targetRearAxleDistance=1.8;
            [~,~,~,obs]=readControllerInputs(ego,target,road,cfg);
            q=predictiveSafetyGeometry.target(obs,target,cfg);
            testCase.verifyEqual(q(5:7),[-1.2;.08;1.8],AbsTol=1e-14);
        end
        function suppliedHeadingRateIsNotAConstantPredictionParameter(testCase)
            [ego,road,cfg,target]=localFixture();target.targetYawRate=99;
            [~,~,~,obs]=readControllerInputs(ego,target,road,cfg);
            q=predictiveSafetyGeometry.target(obs,target,cfg);
            testCase.verifyEqual(q(4)*sin(q(6))/q(7),6*sin(.08)/1.6,AbsTol=1e-14);
        end
        function stationaryTargetRetainsSideslipAndTangentialAcceleration(testCase)
            [ego,road,cfg,target]=localFixture();target.targetVelocityInertial=[0;0];
            target.targetSideslip=.08;target.targetAccelerationInertial=1.2*[cos(.38);sin(.38)];
            [~,~,~,obs]=readControllerInputs(ego,target,road,cfg);
            q=predictiveSafetyGeometry.target(obs,target,cfg);
            testCase.verifyEqual(q(4:6),[0;1.2;.08],AbsTol=1e-14);
            next=predictiveSafetyGeometry.targetFlow(q,1);
            testCase.verifyEqual(next(4),1.2,AbsTol=1e-14);
        end
        function reverseVelocityKeepsTheSameSideslip(testCase)
            [ego,road,cfg,target]=localFixture();target.targetVelocityInertial=-target.targetVelocityInertial;
            target.targetSideslip=.08;
            [~,~,~,obs]=readControllerInputs(ego,target,road,cfg);
            q=predictiveSafetyGeometry.target(obs,target,cfg);
            testCase.verifyEqual(q(4:6),[-6;0;.08],AbsTol=1e-14);
        end
        function inconsistentVelocityAndSideslipAreRejected(testCase)
            [ego,road,cfg,target]=localFixture();target.targetSideslip=.2;
            [~,~,~,obs]=readControllerInputs(ego,target,road,cfg);
            testCase.verifyError(@()predictiveSafetyGeometry.target(obs,target,cfg), ...
                'collisionAvoidanceController:inconsistentTargetMotion');
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
    target=struct('targetPositionInertial',[20;1],'targetVelocityInertial',6*[cos(.38);sin(.38)], ...
        'targetYawInertial',.3);
end
