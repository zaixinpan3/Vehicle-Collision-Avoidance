classdef nrmmVffmReferenceTest < matlab.unittest.TestCase
    %nrmmVffmReferenceTest Analytical motion, road charts and timed preferences.
    properties (TestParameter)
        curvature = struct('straight',0,'left',.01,'right',-.01);
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function acceleratingSideslipMotionMatchesIndependentIntegration(testCase)
            initial=[2;-1;6;1.3;.25;.08];axle=1.7;time=0:.1:2;
            [actual,jerk]=targetPrediction.nrmmFlow(initial,axle,time);
            expected=localIntegratedNrmm(initial,axle,time);
            testCase.verifyEqual(actual([1,2,7],:),expected,AbsTol=2e-10);
            testCase.verifyGreaterThan(norm(actual(5:6,end)-actual(5:6,1)),1);
            step=1e-5;
            left=targetPrediction.nrmmFlow(initial,axle,time(2:end)-step);
            right=targetPrediction.nrmmFlow(initial,axle,time(2:end)+step);
            testCase.verifyEqual((right(5:6,:)-left(5:6,:))/(2*step),jerk(:,2:end),AbsTol=2e-9);
        end

        function aTinyCurvatureRetainsAccumulatedLateralDisplacement(testCase)
            initial=[0;0;100;0;0;1e-12];time=1e4;
            actual=targetPrediction.nrmmFlow(initial,1,time);
            testCase.verifyEqual(actual(2),.500001,AbsTol=1e-10);
            testCase.verifyEqual(actual(1),1e6-1e6*(1e-6)^2/6,AbsTol=1e-6);
        end

        function brakingRequiresAnExplicitPostStopPolicy(testCase)
            initial=[0;0;2;-2;0;.1];
            testCase.verifyError(@()targetPrediction.nrmmFlow(initial,1.6,[0,2]), ...
                'collisionAvoidanceController:nrmmPastStop');
            actual=targetPrediction.nrmmFlow(initial,1.6,[1,2,10],"hold");
            testCase.verifyEqual(actual,repmat(actual(:,1),1,3),AbsTol=0);
            testCase.verifyEqual(actual([3:6,8],:),zeros(5,3),AbsTol=0);
        end

        function knownSideslipSupportsAccelerationFromRest(testCase)
            actual=targetPrediction.nrmmFlow([0;0;0;2;.1;.2],1.6,[0,1]);
            testCase.verifyGreaterThan(actual(7,end),.1);
            testCase.verifyEqual(norm(actual(3:4,end)),2,AbsTol=1e-13);
        end

        function nominalCartesianInterfaceMatchesConsistentNrmmStates(testCase)
            initial=[3;-2;7;.5;.2;.04];axle=1.6;time=0:.2:3;
            center=targetPrediction.nrmmFlow(initial,axle,0);
            encounter=struct('center',center,'contract',struct());
            actual=targetPrediction.nominalFlow(encounter,time);
            expected=targetPrediction.nrmmFlow(initial,axle,time);
            testCase.verifyEqual(actual,expected,AbsTol=2e-13);
        end

        function quadraticSectionsRetainMidpointAndWidthDerivatives(testCase)
            [lane,road]=localRoad(0,true);station=[0,3,10];
            chart=laneGeometry.normalRoadChart(station,lane,road);
            testCase.verifyTrue(all(chart.valid & chart.bounded));
            testCase.verifyEqual(chart.midpoint,[.02*station.^2;.04*station;.04+0*station],AbsTol=1e-13);
            testCase.verifyEqual(chart.halfWidth,[5+.01*station.^2;.02*station;.02+0*station],AbsTol=1e-13);
        end

        function rotatedQuadraticBoundariesGiveTheSameNormalChart(testCase)
            [lane,road]=localRoad(0,true);rotation=[cos(.7),-sin(.7);sin(.7),cos(.7)];
            [rotatedLane,rotatedRoad]=localRotate(lane,road,rotation,[3;-4]);
            actual=laneGeometry.normalRoadChart([0,3,10],rotatedLane,rotatedRoad);
            expected=laneGeometry.normalRoadChart([0,3,10],lane,road);
            testCase.verifyEqual(actual.midpoint,expected.midpoint,AbsTol=1e-12);
            testCase.verifyEqual(actual.halfWidth,expected.halfWidth,AbsTol=1e-12);
        end

        function curvedNormalIntersectionsHaveCorrectSpatialDerivatives(testCase)
            [lane,road]=localRoad(.01,true);station=8;step=1e-3;
            chart=laneGeometry.normalRoadChart(station+[-step,0,step],lane,road);
            testCase.verifyTrue(all(chart.valid & chart.bounded));
            testCase.verifyEqual(diff(chart.midpoint(1,[1,3]))/(2*step),chart.midpoint(2,2),AbsTol=1e-8);
            testCase.verifyEqual(diff(chart.midpoint(1,:),2)/step^2,chart.midpoint(3,2),AbsTol=2e-7);
            testCase.verifyEqual(diff(chart.halfWidth(1,:),2)/step^2,chart.halfWidth(3,2),AbsTol=2e-7);
        end

        function anIrregularNormalChartIsRejected(testCase)
            [lane,road]=localRoad(.3,false);
            encounter=localTarget(zeros(8,1));
            field=solveHardCbfClf.vffmReference(0,[0;8;0],encounter,lane,road,10,10);
            testCase.verifyFalse(field.valid);
        end

        function stationaryStraightRoadRecoversChengGaussian(testCase)
            [lane,road]=localRoad(0,false);time=0:.1:3;
            encounter=localTarget([12;0;0;0;0;0;0;0]);
            actual=solveHardCbfClf.vffmReference(time,[8*time;8+0*time;0*time],encounter,lane,road,4,[3,-3]);
            expected=[3;-3]*exp(-.5*((8*time-12)/4).^2);
            testCase.verifyEqual(actual.lateralPosition,expected,AbsTol=1e-14);
            testCase.verifyTrue(all(actual.valid));
        end

        function matchingLongitudinalMotionProducesAConstantTimeReference(testCase)
            [lane,road]=localRoad(0,false);time=0:.2:3;
            encounter=localTarget([4;0;8;0;0;0;0;0]);
            field=solveHardCbfClf.vffmReference(time,[8*time;8+0*time;0*time],encounter,lane,road,4,3);
            testCase.verifyEqual(field.lateralPosition,3*exp(-.5)+0*time,AbsTol=1e-14);
            testCase.verifyEqual(field.lateralRate,0*time,AbsTol=1e-14);
            testCase.verifyEqual(field.courseOffset,0*time,AbsTol=1e-14);
        end

        function movingTurningReferencesHaveCorrectTimeDerivatives(testCase,curvature)
            [lane,road]=localRoad(curvature,true);time=1.1;step=1e-4;
            initial=[10;-1;3;.8;.1;.04];
            target=localTarget(targetPrediction.nrmmFlow(initial,1.6,0));
            times=time+[-step,0,step];progress=[4*times+.3*times.^2;4+.6*times;.6+0*times];
            field=solveHardCbfClf.vffmReference(times,progress,target,lane,road,6,[2,-2]);
            testCase.verifyTrue(all(field.valid));
            testCase.verifyEqual((field.position(:,3,:)-field.position(:,1,:))/(2*step), ...
                field.velocity(:,2,:),AbsTol=2e-7);
            testCase.verifyEqual((field.position(:,3,:)-2*field.position(:,2,:)+field.position(:,1,:))/step^2, ...
                field.acceleration(:,2,:),AbsTol=5e-6);
            velocity=field.velocity(:,2,1);acceleration=field.acceleration(:,2,1);
            expected=(velocity(1)*acceleration(2)-velocity(2)*acceleration(1))/norm(velocity);
            testCase.verifyEqual(field.normalAcceleration(1,2),expected,AbsTol=1e-12);
        end



        function invalidWidthsAreRejectedByTheReferenceInterface(testCase)
            [lane,road]=localRoad(0,false);target=localTarget(zeros(8,1));
            testCase.verifyError(@()solveHardCbfClf.vffmReference(0,[0;8;0],target,lane,road,0,3), ...
                'MATLAB:expectedPositive');
        end
    end
end

function [lane,road]=localRoad(curvature,bounded)
    lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',100));
    template=struct('origin',[0;0],'longitudinalDirection',[1;0],'lateralDirection',[0;1], ...
        'coefficients',[.01,0,-5],'parameterRange',[-100,100],'safeSideSign',1);
    road=struct('boundaries',repmat(template,0,1));
    if bounded
        road.boundaries=repmat(template,2,1);
        road.boundaries(2).coefficients=[.03,0,5];road.boundaries(2).safeSideSign=-1;
    end
end

function encounter=localTarget(center)
    encounter=struct('center',center,'contract',struct());
end

function expected=localIntegratedNrmm(initial,axle,time)
    k=sin(initial(6))/axle;
    flow=@(~,x)[x(3)*cos(x(4)+initial(6));x(3)*sin(x(4)+initial(6));initial(4);k*x(3)];
    [~,states]=ode45(flow,time,initial([1,2,3,5]),odeset('RelTol',1e-12,'AbsTol',1e-13));
    expected=states(:,[1,2,4]).';
end

function [lane,road]=localRotate(lane,road,rotation,translation)
    lane.referenceCurve.origin=rotation*lane.referenceCurve.origin+translation;
    lane.referenceCurve.heading=lane.referenceCurve.heading+atan2(rotation(2,1),rotation(1,1));
    for index=1:numel(road.boundaries)
        road.boundaries(index).origin=rotation*road.boundaries(index).origin+translation;
        road.boundaries(index).longitudinalDirection=rotation*road.boundaries(index).longitudinalDirection;
        road.boundaries(index).lateralDirection=rotation*road.boundaries(index).lateralDirection;
    end
end
