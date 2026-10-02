classdef movingTargetFlowTest < matlab.unittest.TestCase
    % Moving-boundary identities for initialization guidance.
    properties (TestParameter)
        angle = {0,.7,1.9};
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
        function movingEllipseHasNoRelativeNormalPenetration(testCase,angle)
            rotation=[cos(.4),-sin(.4);sin(.4),cos(.4)];j=[0,-1;1,0];
            map=rotation*diag([5,3]);rate=.3*j*map+rotation*diag([.1,-.2]);
            center=[2;4];translation=[3;-2];q=[cos(angle);sin(angle)];
            v=predictiveSafetyGeometry.movingFlowVelocity(center+map*q,[8;1], ...
                center,translation,map,rate,.5);
            relative=map\(v-translation-rate*q);
            testCase.verifyEqual(q.'*relative,0,AbsTol=2e-14);
        end
        function changingWorldVelocityTransportsTheSameRelativeFlow(testCase)
            p=[-9;2];nominal=[8;0];center=[2;0];targetVelocity=[-3;1];boost=[5;-7];
            first=predictiveSafetyGeometry.movingFlowVelocity(p,nominal,center,targetVelocity,5*eye(2),zeros(2),.7);
            second=predictiveSafetyGeometry.movingFlowVelocity(p,nominal+boost,center,targetVelocity+boost,5*eye(2),zeros(2),.7);
            testCase.verifyEqual(second-first,boost,AbsTol=1e-14);
        end
        function rotatingTheWorldRotatesTheGuidance(testCase,angle)
            rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];
            p=[-9;2];nominal=[8;0];center=[2;0];translation=[-3;1];map=diag([5,3]);rate=[0,-.6;1,0];
            first=predictiveSafetyGeometry.movingFlowVelocity(p,nominal,center,translation,map,rate,.7);
            second=predictiveSafetyGeometry.movingFlowVelocity(rotation*p,rotation*nominal,rotation*center, ...
                rotation*translation,rotation*map,rotation*rate,.7);
            testCase.verifyEqual(second,rotation*first,AbsTol=2e-14);
        end
        function anInteriorSearchPointDoesNotMakeTheGuidanceSingular(testCase)
            velocity=predictiveSafetyGeometry.movingFlowVelocity([2;3],[8;0],[2;3],[1;0],eye(2),zeros(2),.7);
            testCase.verifyTrue(all(isfinite(velocity)));
        end
        function staticObstaclePlacesGaussianAtItsArrivalTime(testCase)
            [state,lane,cfg]=localGuideFixture();
            target=[16;0;0;0;0;0;1.6;2.4;.95;0;0];
            guide=predictiveSafetyGeometry.movingGaussianGuide(state,lane,target,0,4,cfg);
            testCase.verifyEqual(guide.centerTime,2,AbsTol=1e-12);
            testCase.verifyEqual(guide.amplitude,guide.radius,AbsTol=1e-12);
        end
        function approachingTargetMovesTheGaussianEarlier(testCase)
            [state,lane,cfg]=localGuideFixture();
            target=[16;0;pi;8;0;0;1.6;2.4;.95;0;0];
            guide=predictiveSafetyGeometry.movingGaussianGuide(state,lane,target,0,4,cfg);
            testCase.verifyEqual(guide.centerTime,1,AbsTol=1e-12);
            testCase.verifyGreaterThan(guide.amplitude,0);
        end
        function matchingSpeedsDoNotRequireDivisionByClosingSpeed(testCase)
            [state,lane,cfg]=localGuideFixture();
            far=[16;0;0;8;0;0;1.6;2.4;.95;0;0];near=far;near(1)=3;
            quiet=predictiveSafetyGeometry.movingGaussianGuide(state,lane,far,0,4,cfg);
            guide=predictiveSafetyGeometry.movingGaussianGuide(state,lane,near,0,4,cfg);
            testCase.verifyEqual(quiet.amplitude,0,AbsTol=0);
            testCase.verifyTrue(all(isfinite([guide.amplitude,guide.width,guide.centerTime])));
            testCase.verifyGreaterThan(guide.amplitude,0);
        end
        function equalGeometryAtDifferentTimesDoesNotCreateAConflict(testCase)
            [state,lane,cfg]=localGuideFixture();
            target=[16;0;-pi/2;8;0;0;1.6;2.4;.95;0;0];
            guide=predictiveSafetyGeometry.movingGaussianGuide(state,lane,target,0,4,cfg);
            testCase.verifyEqual(guide.amplitude,0,AbsTol=0);
        end
        function accelerationAndSideslipUseTheSameAbsoluteClock(testCase)
            [state,lane,cfg]=localGuideFixture();
            target=predictiveSafetyGeometry.targetFlow([16;0;-pi/2-.05;9;1;.05;1.6;2.4;.95;.2;-.1],-2);
            epoch=predictiveSafetyGeometry.targetFlow(target,-.8);
            first=predictiveSafetyGeometry.movingGaussianGuide(state,lane,target,0,4,cfg);
            second=predictiveSafetyGeometry.movingGaussianGuide(state,lane,epoch,.8,4,cfg);
            testCase.verifyGreaterThan(abs(first.amplitude),0);
            testCase.verifyEqual(second.amplitude,first.amplitude,AbsTol=1e-10);
            testCase.verifyEqual(second.centerTime,first.centerTime,AbsTol=1e-12);
            testCase.verifyEqual(second.endTime,first.endTime,AbsTol=1e-12);
        end
        function rigidFrameChangesPreserveTheTimedGuide(testCase,angle)
            [state,lane,cfg]=localGuideFixture();
            target=predictiveSafetyGeometry.targetFlow([16;0;-pi/2-.05;9;1;.05;1.6;2.4;.95;.2;-.1],-2);
            rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];offset=[31;-9];
            moved=state;moved(1:2)=rotation*state(1:2)+offset;moved(3)=state(3)+angle;
            obstacle=target;obstacle(1:2)=rotation*target(1:2)+offset;obstacle(3)=target(3)+angle;
            road=lane;road.referenceCurve.origin=offset;road.referenceCurve.heading=angle;
            first=predictiveSafetyGeometry.movingGaussianGuide(state,lane,target,0,4,cfg);
            second=predictiveSafetyGeometry.movingGaussianGuide(moved,road,obstacle,0,4,cfg);
            testCase.verifyEqual(second.amplitude,first.amplitude,AbsTol=1e-10);
            testCase.verifyEqual(second.centerTime,first.centerTime,AbsTol=1e-12);
        end
    end
end

function [state,lane,cfg]=localGuideFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8));state=[0;0;0;8;0;0];
    lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',0,'length',200));
end
