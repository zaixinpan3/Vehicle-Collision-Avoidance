classdef movingTargetFlowTest < matlab.unittest.TestCase
    % Behavior of the motion-aware potential used only for initialization.
    properties (TestParameter)
        angle = {0,.7,1.9};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function absentTargetGivesPathAndSpeedAttraction(testCase)
            [state,lane,cfg]=localFixture();state(2)=2;
            [heading,speed,~,risk]=predictiveSafetyGeometry.potentialGuidance(state,lane,[],0,cfg,0);
            testCase.verifyLessThan(heading,0);
            testCase.verifyEqual(speed,cfg.referenceSpeed);
            testCase.verifyEqual(risk,0);
        end
        function approachingMotionPromptsEarlierAvoidanceThanStaticGeometry(testCase)
            [state,lane,cfg]=localFixture();target=[24;0;pi;8;0;0;1.6;2.4;.95;0;0];
            stationary=target;stationary(4)=0;
            [heading,speed,side,risk]=predictiveSafetyGeometry.potentialGuidance(state,lane,target,0,cfg,0);
            [~,~,~,staticRisk]=predictiveSafetyGeometry.potentialGuidance(state,lane,stationary,0,cfg,0);
            testCase.verifyGreaterThan(risk,staticRisk);
            testCase.verifyGreaterThan(heading,0);
            testCase.verifyEqual(side,1);
            testCase.verifyLessThan(speed,cfg.referenceSpeed);
        end
        function aPastCrossingHasLessRiskThanASimultaneousCrossing(testCase)
            [state,lane,cfg]=localFixture();target=[12.8;-12.8;pi/2;8;0;0;1.6;2.4;.95;0;0];
            past=target;past(2)=8;
            [~,~,~,risk]=predictiveSafetyGeometry.potentialGuidance(state,lane,target,0,cfg,0);
            [~,~,~,pastRisk]=predictiveSafetyGeometry.potentialGuidance(state,lane,past,0,cfg,0);
            testCase.verifyGreaterThan(risk,10*pastRisk);
        end
        function orientationChangesThePredictedExclusionWidth(testCase)
            [state,lane,cfg]=localFixture();target=[0;5;0;0;0;0;1.6;2.4;.95;0;0];
            turned=target;turned(3)=pi/2;
            [~,~,~,aligned]=predictiveSafetyGeometry.potentialGuidance(state,lane,target,0,cfg,0);
            [~,~,~,crosswise]=predictiveSafetyGeometry.potentialGuidance(state,lane,turned,0,cfg,0);
            testCase.verifyGreaterThan(crosswise,aligned);
        end
        function matchedSpeedsAndOverlapRemainFinite(testCase)
            [state,lane,cfg]=localFixture();target=[0;0;0;8;0;0;1.6;2.4;.95;0;0];
            [heading,speed,side,risk]=predictiveSafetyGeometry.potentialGuidance(state,lane,target,0,cfg,0);
            testCase.verifyTrue(all(isfinite([heading,speed,side,risk])));
            testCase.verifyGreaterThan(speed,cfg.model.scheduleSpeedFloor);
        end
        function chosenPassingSideIsRetainedWithinASeed(testCase)
            [state,lane,cfg]=localFixture();target=[24;0;pi;8;0;0;1.6;2.4;.95;0;0];
            [heading,~,side]=predictiveSafetyGeometry.potentialGuidance(state,lane,target,0,cfg,-1);
            testCase.verifyEqual(side,-1);
            testCase.verifyLessThan(heading,0);
        end
        function accelerationAndSideslipUseTheSameAbsoluteClock(testCase)
            [state,lane,cfg]=localFixture();target=[16;-12;pi/2-.05;9;1;.05;1.6;2.4;.95;.2;-.1];
            epoch=predictiveSafetyGeometry.targetFlow(target,-.8);
            [h1,v1,s1,r1]=predictiveSafetyGeometry.potentialGuidance(state,lane,target,0,cfg,0);
            [h2,v2,s2,r2]=predictiveSafetyGeometry.potentialGuidance(state,lane,epoch,.8,cfg,0);
            testCase.verifyEqual([h2,v2,s2,r2],[h1,v1,s1,r1],AbsTol=1e-11);
        end
        function rigidFrameChangesPreserveTheField(testCase,angle)
            [state,lane,cfg]=localFixture();state(2)=.4;
            target=[16;-12;pi/2-.05;9;1;.05;1.6;2.4;.95;.2;-.1];
            rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];offset=[31;-9];
            moved=state;moved(1:2)=rotation*state(1:2)+offset;moved(3)=state(3)+angle;
            obstacle=target;obstacle(1:2)=rotation*target(1:2)+offset;obstacle(3)=target(3)+angle;
            road=lane;road.referenceCurve.origin=offset;road.referenceCurve.heading=angle;
            [h1,v1,s1,r1]=predictiveSafetyGeometry.potentialGuidance(state,lane,target,0,cfg,0);
            [h2,v2,s2,r2]=predictiveSafetyGeometry.potentialGuidance(moved,road,obstacle,0,cfg,0);
            testCase.verifyEqual([atan2(sin(h2-h1-angle),cos(h2-h1-angle)),v2,s2,r2], ...
                [0,v1,s1,r1],AbsTol=1e-11);
        end
        function batchedTargetFlowAgreesWithScalarPrediction(testCase)
            q=[16;-12;pi/2-.05;9;1;.05;1.6;2.4;.95;.2;-.1];
            batch=predictiveSafetyGeometry.targetFlow(q,[-.2,0,.7]);
            testCase.verifyEqual(batch,[predictiveSafetyGeometry.targetFlow(q,-.2),q, ...
                predictiveSafetyGeometry.targetFlow(q,.7)],AbsTol=1e-13);
        end
    end
end
function [state,lane,cfg]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8));state=[0;0;0;8;0;0];
    lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',0,'length',200));
end
