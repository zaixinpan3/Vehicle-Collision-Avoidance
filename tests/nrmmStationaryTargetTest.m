classdef nrmmStationaryTargetTest < matlab.unittest.TestCase
    % A zero physical speed never becomes a fictitious moving target.
    properties
        Design
    end
    properties (TestParameter)
        speed = {0,0.01,0.5,2,4.99,5,12};
        turnSign = {-1,1};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'estimator')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            cfg=nrmmTrackingConfig();cfg.target.domain.speedMinimum=0;
            testCase.Design=synthesizeNrmmObserverGains(cfg);
        end
    end
    methods (Test)
        function lowSpeedModelDiscrepancyIsCovered(testCase,speed,turnSign)
            domain=testCase.Design.target.domain;
            curvature=turnSign*sin(domain.sideslipMaximum)/domain.rearAxleDistance;
            acceleration=domain.scalarAccelerationMaximum;
            tangent=[cos(.7);sin(.7)];normal=[-tangent(2);tangent(1)];
            velocity=speed*tangent;
            targetAcceleration=acceleration*tangent+curvature*speed^2*normal;
            trueJerk=-curvature^2*speed^3*tangent+3*acceleration*curvature*speed*normal;
            [derivative,estimate]=nrmmTargetTrackerDerivative([30;2;velocity;targetAcceleration], ...
                struct('bodyVelocity',[15;0],'yawRate',0),domain);
            testCase.verifyTrue(estimate.speedDomainValid);
            testCase.verifyEqual(estimate.targetVelocity,velocity,AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(norm(derivative(5:6)-trueJerk), ...
                testCase.Design.target.regularizationJerkMaximum+1e-12);
            testCase.verifyEqual(derivative(5:6), ...
                min(1,speed/domain.regularizationSpeedMinimum)^4*trueJerk,AbsTol=1e-11);
        end
        function stationaryMeasurementsPublishPositionAndUnknownHeading(testCase)
            [~,output,target]=localStationaryPublication();
            testCase.verifyTrue(output.controllerErrorBound.available);
            testCase.verifyTrue(target.controllerErrorBound.available);
            testCase.verifyTrue(target.predictionErrorSet.available);
            testCase.verifyLessThan(target.targetSpeed,.2);
            testCase.verifyEqual(target.predictionErrorSet.speedInterval(1),0,AbsTol=1e-12);
            testCase.verifyEqual(target.targetYawErrorBound,pi,AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(abs(target.targetPositionInertial-[34;0]), ...
                target.targetPositionInertialErrorBound+1e-12);
            testCase.verifyLessThanOrEqual(abs(target.targetVelocityInertial), ...
                target.targetVelocityInertialErrorBound+1e-12);
            testCase.verifyLessThanOrEqual(abs(target.targetAccelerationInertial), ...
                target.targetAccelerationInertialErrorBound+1e-12);
            testCase.verifyEqual(target.predictionMotion.kind,"nrmm-motion-v1");
        end
        function stationaryTrackRemainsAvailableAfterObserverPropagation(testCase)
            [context,~,~]=localStationaryPublication();
            ego=struct('position',[.1;0],'yaw',0,'longitudinalVelocity',8, ...
                'lateralVelocity',0,'yawRate',0);
            [~,~,target]=nrmmEstimatorControllerAdapter('sample',context,.0125,ego, ...
                struct('targetPositionInertial',[34;0]));
            testCase.assertNotEmpty(target);
            testCase.verifyTrue(target.predictionErrorSet.available);
            testCase.verifyLessThan(target.targetSpeed,.2);
            testCase.verifyEqual(target.predictionErrorSet.speedInterval(1),0,AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(abs(target.targetVelocityInertial), ...
                target.targetVelocityInertialErrorBound+1e-12);
        end
        function stationaryScenarioContractContainsZeroSpeed(testCase)
            contract=collisionThreatContract("stoppedLead",25,WindowSeconds=8);
            testCase.verifyEqual(contract.speedMinimum,0);
            testCase.verifyEqual(contract.speedMaximum,0);
        end
        function stoppingScenarioContractIncludesItsParkedTail(testCase)
            contract=collisionThreatContract("hardBrakingLead",25,WindowSeconds=8);
            testCase.verifyEqual(contract.speedMinimum,0);
            testCase.verifyGreaterThanOrEqual(contract.scalarAccelerationMaximum,2);
        end
    end
end

function [context,output,target]=localStationaryPublication()
    cfg=estimatorControllerIntegrationConfig();
    ego=struct('position',[0;0],'yaw',0,'longitudinalVelocity',8, ...
        'lateralVelocity',0,'yawRate',0);
    position=@(time,unused)struct('targetPositionInertial',[34;0]);
    context=nrmmEstimatorControllerAdapter('initialize',cfg,ego,position);
    [context,output,target]=nrmmEstimatorControllerAdapter('sample',context,0,ego,position(0,ego));
end
