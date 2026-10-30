classdef targetPredictionConsistencyTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function preparePaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'estimator')));
        end
    end
    methods (Test)
        function targetPositionAndTravelErrorsRemainIndependent(testCase)
            target=[10;0;pi;1;0;0;1.6;2.4;.95;0;0];
            error=struct('positionRadius',1,'courseRadius',0,'curvatureRadius',0,'sideslipRadius',0);
            image=predictiveSafetyGeometry.targetArcSet(target,error,[0,2]);[lo,hi]=bounds(image);
            testCase.verifyLessThanOrEqual(lo(1),7);
            testCase.verifyGreaterThanOrEqual(hi(1),11);
            testCase.verifyLessThanOrEqual(lo(2),-1);
            testCase.verifyGreaterThanOrEqual(hi(2),1);
        end
        function spatialArcImagesRetainBackwardProgressAndContainParameterCorners(testCase)
            [image,samples]=localSpatialArcSamples();[lo,hi]=bounds(image);
            testCase.verifyLessThan(hi(1),25);
            testCase.verifyGreaterThanOrEqual(samples,repmat(lo,1,size(samples,2))-1e-11);
            testCase.verifyLessThanOrEqual(samples,repmat(hi,1,size(samples,2))+1e-11);
        end
        function nonlinearTargetImagesRetainAsymmetricPhysicalIntervals(testCase)
            target=[0;0;0;.9;0;0;1.6;2.4;.95;0;0];
            error=struct('positionRadius',0,'courseRadius',0,'speedRadius',.9, ...
                'accelerationRadius',1,'curvatureRadius',.05,'sideslipRadius',.1, ...
                'speedInterval',[0;1],'accelerationInterval',[0;0],'curvatureInterval',[0;0]);
            pose=predictiveSafetyGeometry.targetPoseSet(target,error,1);[lo,hi]=bounds(pose);
            testCase.verifyEqual(lo,[0;0;0],AbsTol=1e-12);
            testCase.verifyEqual(hi,[1;0;0],AbsTol=1e-12);
        end
        function currentPointUpdatesAreNotFrozenByAnOlderAdmissiblePoint(testCase)
            [q,prior,u,memory]=localFixture();
            [selected,~,next,info]=predictiveSafetyGeometry.reconcileTarget(q,prior,u,memory,.05);
            testCase.verifyEqual(selected,q,AbsTol=1e-12);
            testCase.verifyTrue(info.parameterIntersection);
            testCase.verifyEqual(info.observed,q);
            testCase.verifyEqual(next.accelerationInterval,[-.1;.1]);
        end
        function recenteringRetainsTheCurrentPositionAndCourseEnclosure(testCase)
            [q,prior,u,memory]=localFixture();
            [selected,recentered]=predictiveSafetyGeometry.reconcileTarget(q,prior,u,memory,.05);
            distance=norm(selected(1:2)-q(1:2));
            testCase.verifyEqual(recentered.target.positionRadius,u.predictionSet.positionRadius+distance,AbsTol=1e-12);
            change=abs(atan2(sin(sum(selected([3,6]))-sum(q([3,6]))),cos(sum(selected([3,6]))-sum(q([3,6])))));
            testCase.verifyEqual(recentered.target.courseRadius,u.predictionSet.courseRadius+change,AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(recentered.target.sideslipRadius, ...
                max(abs(asin(u.predictionSet.curvatureInterval*q(7))-selected(6))));
        end
        function newEvidenceCanForceARealCurvatureUpdate(testCase)
            [q,prior,u,memory]=localFixture();
            u.predictionSet.curvatureInterval=[.02;.03];
            [selected,~,next]=predictiveSafetyGeometry.reconcileTarget(q,prior,u,memory,.05);
            testCase.verifyEqual(sin(selected(6))/selected(7),.03,AbsTol=1e-12);
            testCase.verifyEqual(next.curvatureInterval,[.02;.03]);
            testCase.verifyEqual(selected(3)+selected(6),q(3)+q(6),AbsTol=1e-12);
        end
        function theSpeedIntervalPropagatesBeforeIntersectingTheNewMeasurement(testCase)
            [q,prior,u,memory]=localFixture();
            memory.speedInterval=[4;5];memory.accelerationInterval=[-1;-.5];
            u.predictionSet.accelerationInterval=[-2;0];u.predictionSet.speedInterval=[3;6];
            [~,~,next]=predictiveSafetyGeometry.reconcileTarget(q,prior,u,memory,.2);
            testCase.verifyEqual(next.speedInterval,[3.8;4.9],AbsTol=1e-12);
        end
        function contradictoryConstantParameterSetsAreNotSilentlyDiscarded(testCase)
            [q,prior,u,memory]=localFixture();
            memory.curvatureInterval=[-.1;-.05];u.predictionSet.curvatureInterval=[.01;.02];
            testCase.verifyError(@()predictiveSafetyGeometry.reconcileTarget(q,prior,u,memory,.05), ...
                'collisionAvoidanceController:inconsistentTargetPredictionSet');
        end
        function theDeclaredTargetStopResetIsNotAParameterContradiction(testCase)
            [q,prior,u,memory]=localFixture();
            memory.speedInterval=[.02;.02];memory.accelerationInterval=[-1;-1];
            u.predictionSet.speedInterval=[0;0];u.predictionSet.accelerationInterval=[0;0];
            q(4:5)=0;prior(4:5)=0;
            [selected,~,next]=predictiveSafetyGeometry.reconcileTarget(q,prior,u,memory,.05);
            testCase.verifyEqual(selected(4:5),[0;0]);
            testCase.verifyEqual(next.speedInterval,[0;0]);
            testCase.verifyEqual(next.accelerationInterval,[0;0]);
        end
        function aPartiallyStoppedFamilyRetainsBothAccelerationBranches(testCase)
            [q,prior,u,memory]=localFixture();
            memory.speedInterval=[0;.2];memory.accelerationInterval=[-1;-.5];
            u.predictionSet.speedInterval=[0;.2];u.predictionSet.accelerationInterval=[-1;0];
            [~,~,next]=predictiveSafetyGeometry.reconcileTarget(q,prior,u,memory,.05);
            testCase.verifyEqual(next.speedInterval,[0;.175],AbsTol=1e-12);
            testCase.verifyEqual(next.accelerationInterval,[-1;0]);
        end
        function unavailableEnclosuresDoNotFreezeAnUnsupportedPrediction(testCase)
            [q,prior,u,memory]=localFixture();u=rmfield(u,'predictionSet');
            [selected,~,next,info]=predictiveSafetyGeometry.reconcileTarget(q,prior,u,memory,.05);
            testCase.verifyEqual(selected,q);
            testCase.verifyEmpty(next);
            testCase.verifyFalse(info.available);
        end
    end
end

function [q,prior,u,memory]=localFixture()
    prior=[0;0;0;4;0;0;1.6;2.4;.95;0;0];
    q=prior;q(1:6)=[.03;.02;-.03;4.03;.08;.05];
    set=struct('positionRadius',.1,'courseRadius',.1,'speedInterval',[3.8;4.2], ...
        'accelerationInterval',[-.2;.2],'curvatureInterval',[-.1;.1]);
    u=struct('predictionSet',set);
    memory=struct('time',0,'speedInterval',[3.9;4.1], ...
        'accelerationInterval',[-.1;.1],'curvatureInterval',[-.1;.1]);
end

function [image,samples]=localSpatialArcSamples()
    target=[25;0;pi;8;0;0;1.6;2.4;.95;0;0];
    error=struct('positionRadius',.1,'courseRadius',.3,'speedRadius',2, ...
        'accelerationRadius',1.65,'curvatureRadius',.0344,'sideslipRadius',.056, ...
        'speedInterval',[6;10],'accelerationInterval',[-1.65;1.65], ...
        'curvatureInterval',[-.0344;.0344]);
    image=predictiveSafetyGeometry.targetPoseSet(target,error,2.45);samples=zeros(3,81);index=0;
    for speed=[6,8,10]
        for acceleration=[-1.65,0,1.65]
            for curvature=[-.0344,0,.0344]
                for course=pi+[-.3,0,.3]
                    sample=target;sample(6)=asin(1.6*curvature);sample(3)=course-sample(6);
                    sample(4:5)=[speed;acceleration];value=predictiveSafetyGeometry.predictTarget(sample,2.45);
                    index=index+1;samples(:,index)=value(1:3);
                end
            end
        end
    end
end
