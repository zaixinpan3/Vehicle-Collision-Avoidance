classdef analyticAvoidanceGuideTest < matlab.unittest.TestCase
    % Single-path geometry supplies a reference, not a safety certificate.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for directory=["controller","config","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,directory)));
            end
        end
    end
    methods (Test)
        function separationCanUseLongitudinalSpaceBeforeTheLateralPass(testCase)
            shape=[2.4;.95;0;0];position=[-8,0,8;0,0,0];
            target=repmat([0;0;pi;8;0;0;1.6;shape],1,3);
            offsets=predictiveSafetyGeometry.separationOffsets(position,zeros(1,3),shape,target,[0;1],.2);
            testCase.verifyEqual(offsets([1,3]),[0,0],AbsTol=1e-12);
            testCase.verifyEqual(offsets(2),2.1,AbsTol=1e-12);
        end
        function rayExitsClearRotatedRectanglesWithCenterOffsets(testCase)
            shape=[2.4;.95;.3;-.1];normal=[.6;.8];
            headings=[-.7,.2,1.1];position=[.2,-.3,.1;.1,.4,-.2];
            target=repmat([0;0;0;8;0;0;1.6;2.1;.85;-.2;.15],1,3);
            target(3,:)=[.4,-.8,1.5];
            offsets=predictiveSafetyGeometry.separationOffsets(position,headings,shape,target,normal,.3);
            for index=1:3
                pose=[position(:,index)+normal*(offsets(index)+1e-8);headings(index)];
                gap=predictiveSafetyGeometry.rectangle(pose,shape,target(1:3,index),target(8:11,index));
                testCase.verifyGreaterThanOrEqual(gap,.3-1e-9);
            end
        end
        function anEarlyConflictDoesNotForceTheWholeOffsetImmediately(testCase)
            model=localModel("crossing",8);guide=predictiveSafetyGeometry.avoidanceGuide(model,40,1);
            testCase.verifyGreaterThan(guide.amplitude,0);
            testCase.verifyGreaterThan(guide.riseSeconds,guide.conflictStartSeconds);
            testCase.verifyGreaterThan(guide.conflictEndSeconds,guide.conflictStartSeconds);
            testCase.verifyEqual(guide.position(:,1),model.initialState(1:2),AbsTol=1e-12);
            testCase.verifyEqual(guide.velocity(:,1),model.initialState(4:5),AbsTol=1e-12);
        end
        function theGuideHoldsItsOffsetInsteadOfReturningInsideTheHorizon(testCase)
            model=localModel("crossing",8);guide=predictiveSafetyGeometry.avoidanceGuide(model,40,1);
            t=guide.time;speed=model.initialState(4);a=model.cfg.planner.decelerationMetersPerSecondSquared;
            moving=min(t,speed/a);base=[speed*moving-.5*a*moving.^2;zeros(size(t))];
            offset=guide.direction.'*(guide.position-base);
            testCase.verifyGreaterThanOrEqual(diff(offset),-1e-12);
            testCase.verifyEqual(offset(end),guide.amplitude,AbsTol=1e-12);
            testCase.verifyGreaterThan(offset(end),0);
            testCase.verifyEqual(guide.velocity(:,end),[speed-a*t(end);0],AbsTol=1e-11);
        end
        function brakingAndDeformationShareOneAccelerationBudget(testCase)
            for speed=[8,15]
                model=localModel("curvedCrossing",speed);
                guide=predictiveSafetyGeometry.avoidanceGuide(model,40,1);
                testCase.verifyLessThanOrEqual(max(vecnorm(guide.acceleration,2,1)),guide.accelerationBudget+1e-9);
                testCase.verifyTrue(all(isfinite([guide.position;guide.velocity;guide.acceleration]),'all'));
            end
        end
        function aSmallPositiveGapNeedsNoArbitraryPlannerBuffer(testCase)
            model=localModel("headOn",8);
            model.target(1:6)=[0;2;0;8;0;0];
            guide=predictiveSafetyGeometry.avoidanceGuide(model,16,1);
            testCase.verifyEqual(guide.minimumGap,.1,AbsTol=1e-12);
            testCase.verifyEqual(guide.amplitude,0);
            testCase.verifyTrue(isnan(guide.conflictStartSeconds));
            model.uncertainty.collisionGenerator=diag([0,.15,0,0,0,0]);
            bounded=predictiveSafetyGeometry.avoidanceGuide(model,16,1);
            testCase.verifyEqual(bounded.conflictStartSeconds,0);
            testCase.verifyTrue(bounded.initialConflict);
        end
        function currentObserverBoundsWidenTheConflictWindow(testCase)
            model=localModel("headOn",8);nominal=predictiveSafetyGeometry.avoidanceGuide(model,40,1);
            model.uncertainty.collisionGenerator=diag([.2,.2,.04,0,0,0]);
            bounded=predictiveSafetyGeometry.avoidanceGuide(model,40,1);
            testCase.verifyLessThanOrEqual(bounded.conflictStartSeconds,nominal.conflictStartSeconds);
            testCase.verifyGreaterThanOrEqual(bounded.conflictEndSeconds,nominal.conflictEndSeconds);
        end
        function anInitialOverlapProducesFiniteBrakingGuidance(testCase)
            model=localModel("headOn",8);model.target(1:2)=model.initialState(1:2);
            guide=predictiveSafetyGeometry.avoidanceGuide(model,40,1);
            testCase.verifyTrue(guide.initialConflict);
            testCase.verifyTrue(guide.brakingOnly);
            testCase.verifyFalse(guide.demandSatisfied);
            testCase.verifyEqual(guide.amplitude,0);
            testCase.verifyTrue(all(isfinite([guide.position;guide.velocity;guide.acceleration]),'all'));
        end
        function aSeparatedTargetPreservesTheCurrentVelocityReference(testCase)
            model=localModel("headOn",8);model.target(2)=30;
            guide=predictiveSafetyGeometry.avoidanceGuide(model,40,1);
            testCase.verifyEqual(guide.amplitude,0);
            testCase.verifyTrue(isnan(guide.conflictStartSeconds));
            testCase.verifyEqual(guide.position,[8*guide.time;zeros(size(guide.time))],AbsTol=1e-12);
            testCase.verifyEqual(guide.velocity,repmat([8;0],1,numel(guide.time)),AbsTol=1e-12);
        end
        function aCurvedReferenceKeepsTheGivenPathCurvature(testCase)
            model=localModel("curvedHeadOn",8);model.target(1:2)=[80;80];
            guide=predictiveSafetyGeometry.avoidanceGuide(model,40,1);
            x=model.initialState;course=x(3)+atan2(x(5),x(4));
            curve=struct('origin',x(1:2),'heading',course,'curvature',.005,'length',200);
            expected=laneGeometry.referencePose(hypot(x(4),x(5))*guide.time,0,curve);
            testCase.verifyEqual(guide.amplitude,0);
            testCase.verifyEqual(guide.position,expected,AbsTol=1e-11);
            rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
            testCase.verifyEqual(guide.velocity(:,1),rotation*x(4:5),AbsTol=1e-12);
        end
        function aCurvedBrakingGuideStaysAtRestAfterItsStop(testCase)
            model=localModel("curvedHeadOn",8);model.target(1:2)=model.initialState(1:2);
            guide=predictiveSafetyGeometry.avoidanceGuide(model,60,1);
            testCase.verifyTrue(guide.brakingOnly);
            stopped=guide.time>=2.1;
            testCase.verifyEqual(guide.position(:,stopped),repmat(guide.position(:,end),1,nnz(stopped)),AbsTol=1e-12);
            testCase.verifyEqual(guide.velocity(:,stopped),zeros(2,nnz(stopped)),AbsTol=0);
            testCase.verifyEqual(guide.acceleration(:,stopped),zeros(2,nnz(stopped)),AbsTol=0);
        end
    end
end
function model=localModel(name,speed)
    [state,target,road,cfg]=collisionThreatScenario(name,struct('referenceSpeed',speed));
    curvature=0;if isfield(road,'referenceCurve'),curvature=road.referenceCurve.curvature;end
    model=struct('cfg',cfg,'initialState',state,'target',target, ...
        'nominalReference',struct('curvature',curvature), ...
        'uncertainty',struct('collisionGenerator',zeros(6),'target',[]));
end
