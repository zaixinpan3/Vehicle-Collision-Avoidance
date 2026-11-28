classdef targetPredictionConsistencyTest < matlab.unittest.TestCase
    properties (TestParameter)
        slipInterval={[-.2,.17],[-.2,-.05],[.03,.17]}
    end
    methods (TestClassSetup)
        function preparePaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'estimator')));
        end
    end
    methods (Test)
        function declaredTurnRateEnclosesTheAdmissibleConstantCurvatureArc(testCase)
            q=[2;-5;.7;3;1;.015;1.6;2.4;.95;0;0];times=[0,1,2,4];
            bounds=[-2,-.05;3,.05];
            domain=struct('speedMaximum',12,'yawRateMaximum',.07);
            [box,centers]=predictiveSafetyGeometry.targetMotionEnvelope(q,bounds,times,[],domain);
            truth=predictiveSafetyGeometry.predictTarget(q,times);
            ego=[-3;12]+zeros(2,numel(times));
            lower=predictiveSafetyGeometry.targetEnvelopeRange(ego,box,centers);
            testCase.verifyLessThanOrEqual(lower,vecnorm(ego-truth(1:2,:))+1e-10);
            testCase.verifyLessThanOrEqual(abs(box(3,:)-q(3)),domain.yawRateMaximum*times+1e-12);
        end

        function aTurningCellRemainsBoundedAfterSeveralRevolutions(testCase)
            root=[0;0;.3;15;1;.09;1.6;2.4;.95;0;0];
            limits=[.5,.08;1.5,.10];time=[20,40];
            [~,center]=predictiveSafetyGeometry.targetMotionEnvelope(root,limits,time);
            nominal=predictiveSafetyGeometry.predictTarget(root,time);
            diameter=2*root(7)/sin(limits(1,2));
            testCase.verifyLessThanOrEqual(center(2,:)-center(1,:),2*diameter+1e-8);
            testCase.verifyLessThanOrEqual(center(4,:)-center(3,:),2*diameter+1e-8);
            singleton=repmat(root(5:6).',2,1);
            box=predictiveSafetyGeometry.targetMotionEnvelope(root,singleton,time);
            testCase.verifyEqual(box,nominal,AbsTol=1e-8);
        end

        function theArcChordRestrictsSpeedWithoutAssumingAnUnobservedHeading(testCase)
            q=[0;0;.8;5.8;2;0;1.6;2.2;.9;0;0];limits=[2,0;2,0];
            track=struct('times',[1,2],'time',2,'positions',[0,3;0,4], ...
                'positionRadii',[0,0],'speedUpper',10,'errorBound',struct('targetComponents',[0;2;1]));
            obs=nrmmControllerErrorBounds("footprintObservation",track,q,limits,struct);
            testCase.verifyEqual(obs.speed,[6;6],AbsTol=1e-12);
            track.time=2.1;
            obs=nrmmControllerErrorBounds("footprintObservation",track,q,limits,struct);
            testCase.verifyEmpty(obs.current);
            testCase.verifyEqual(obs.speed,[3.8;7.8],AbsTol=1e-12);
            % The same formula permits absorption before the last position.
            track.time=2;track.positions=[0,.1;0,0];q(4)=.4;limits(:,1)=-4;
            obs=nrmmControllerErrorBounds("footprintObservation",track,q,limits,struct);
            testCase.verifyEqual(obs.speed,[0;0],AbsTol=1e-12);
        end

        function aNoiselessChordReconstructsAllFourFutureCornersThroughAStop(testCase)
            truth=[3;-2;2.8;4;-1.5;.12;1.6;2.3;.88;.35;-.13];times=[0,.4,2,4];
            past=predictiveSafetyGeometry.predictTarget(truth,-.3);
            obs=struct('speed',[4;4],'current',truth(1:2),'radius',0, ...
                'previous',past(1:2),'previousRadius',0,'span',.3);
            root=truth;root(1:3)=[-18;14;-.9];limits=repmat(root(5:6).',2,1);
            [box,r]=predictiveSafetyGeometry.observedFootprint(root,limits,times,obs,struct);
            target=predictiveSafetyGeometry.predictTarget(truth,times);
            testCase.verifyEqual(localCorners(box),localCorners(target),AbsTol=2e-12);
            testCase.verifyLessThan(max(r),2e-12);
            obs.previous=obs.current;
            [box,r]=predictiveSafetyGeometry.observedFootprint(root,limits,times,obs,struct);
            testCase.verifyEmpty(box);testCase.verifyEmpty(r);
        end

        function aRetainedRadarStripTightensFutureClearanceForTheSameMotion(testCase)
            q=[70;-12;.61;6;0;0;1.6;2.4;.9;.32;-.18];
            limits=[0,0;0,0];times=[0,.1,1,2,3];span=.12;
            past=predictiveSafetyGeometry.predictTarget(q,[-1,-span]);
            current=q(1:2)+[.02;-.01];previous=past(1:2,2)+[-.01;.02];
            older=past(1:2,1)+[.005;.005];
            track=struct('times',[2-span,2],'time',2,'positions',[previous,current], ...
                'positionRadii',[.05,.05],'arcAnchors',[1,2-span,2;older,previous,current;.01,.05,.05]);
            obs=nrmmControllerErrorBounds("footprintObservation",track,q,limits,struct);
            testCase.verifyEqual(obs.chords,[1;older-current;.01],AbsTol=1e-14);
            [box,r]=predictiveSafetyGeometry.observedFootprint(q,limits,times,obs,struct);
            without=obs;without.chords=zeros(4,0);
            [~,base]=predictiveSafetyGeometry.observedFootprint(q,limits,times,without,struct);
            testCase.verifyLessThan(r(end),base(end)/2);
            target=predictiveSafetyGeometry.predictTarget(q,times);
            localVerifyCornerContainment(testCase,target,box,r);
            % Only the middle prior protrudes. Swept checks on both sides
            % must still get the full joint bound at their other endpoint.
            prior=box;prior(8:9,:)=prior(8:9,:)/2;prior(8,3)=box(8,3)+10;
            [~,screened,relevant]=predictiveSafetyGeometry.observedFootprint(q,limits,times,obs,struct,prior);
            testCase.verifyEqual(relevant,[false,false,true,false,false]);
            testCase.verifyEqual(screened(2:4),r(2:4),AbsTol=1e-12);
        end

        function correlatedNoisyChordsEncloseRotatingCornersAndAbsorption(testCase)
            root=[3;-2;.7;4;-1.4;.04;1.6;2.2;.9;.3;-.2];
            times=[0,.1,1,4,8];limits=[-2.1,-.035;-1.1,.12];
            for beta=[-.035,.12]
                for acceleration=limits(:,1).'
                    q=root;q(3:6)=[-2.1;4.2;acceleration;beta];
                    past=predictiveSafetyGeometry.predictTarget(q,[-.8,-.3,-.1]);
                    current=q(1:2)+[.015;-.01];older=past(1:2,1:2)+[-.01;.01];
                    obs=struct('speed',[3.8;4.3],'current',current,'radius',.02, ...
                        'previous',past(1:2,3)+[.02;-.015],'previousRadius',.03,'span',.1, ...
                        'positionBounds',current+[-.02,.02], ...
                        'chords',[.8,.3;older-current;.02,.02]);
                    [box,r]=predictiveSafetyGeometry.observedFootprint(root,limits,times,obs,struct);
                    localVerifyCornerContainment(testCase,predictiveSafetyGeometry.predictTarget(q,times),box,r);
                    [~,prior]=predictiveSafetyGeometry.observedFootprint(root,limits,times, ...
                        rmfield(obs,{'positionBounds','chords'}),struct);
                    testCase.verifyLessThanOrEqual(r,prior+1e-11);
                end
            end
        end

        function aJointChordReducesTheMainMotionEnvelopeTranslationRadius(testCase)
            origin=[73;-8];prior=.5;obs=struct('current',origin+[.02;-.01]);
            chart=struct('H',-2+1i,'offsets',.19-.11i,'radii',.025,'courseRadius',.005,'direction',1i);
            radius=predictiveSafetyGeometry.translationRadius(origin,prior,obs,chart);
            d=chart.radii+abs(chart.H)*chart.courseRadius;center=.21-.12i;
            testCase.verifyEqual(radius,abs(center)+d,AbsTol=2e-12);
            testCase.verifyLessThan(radius,prior);
            testCase.verifyGreaterThanOrEqual(radius+1e-12,max(abs(center+d*exp(1i*linspace(-pi,pi,51)))));
            chart.radii=2;
            testCase.verifyEqual(predictiveSafetyGeometry.translationRadius(origin,prior,obs,chart),prior);
        end

        function oneRoundedRectangleCoversInteriorClocksAndNoisyPositionPairs(testCase)
            root=[3;-2;.7;4;-1;.04;1.6;2.2;.9;.3;-.2];times=[0,.12,1,3,7];
            limits=[-3,-.09;.5,.13];span=.25;shape=[2.1;.8;.25;-.1];
            ego=[1,-1,5,3,22;2,1,-3,4,3;.7,-1,.05,.3,.8];
            for speed=[2.8,4.9]
                for a=[-2.6,.2]
                    for beta=[-.075,.09]
                        truth=root;truth(3:6)=[1.1;speed;a;beta];
                        past=predictiveSafetyGeometry.predictTarget(truth,-span);
                        obs=struct('speed',[2.6;5],'current',truth(1:2)+[.03;-.01],'radius',.04, ...
                            'previous',past(1:2)+[-.02;.01],'previousRadius',.03,'span',span);
                        [box,r]=predictiveSafetyGeometry.observedFootprint(root,limits,times,obs,struct);
                        target=predictiveSafetyGeometry.predictTarget(truth,times);
                        testCase.verifyLessThanOrEqual(max(abs(localCorners(target)-localCorners(box)),[],1),r+1e-11);
                        gap=predictiveSafetyGeometry.rectangleClearance(ego,box,shape)-r;
                        actual=predictiveSafetyGeometry.rectangleClearance(ego,target,shape);
                        testCase.verifyLessThanOrEqual(gap,actual+1e-10);
                    end
                end
            end
            root(4)=0;obs.previous=[];obs.current=[-.5;.8];obs.radius=.04;
            [box,r]=predictiveSafetyGeometry.observedFootprint(root,limits,times,obs,struct);
            testCase.verifyEqual(box(1:2,:),repmat(obs.current,1,numel(times)));
            testCase.verifyEqual(r,repmat(obs.radius,1,numel(times)));
        end

        function valueOnlyDistanceAgreesWithTheDualWitnessEvaluation(testCase)
            rngStream=RandStream('mt19937ar',Seed=1523);
            n=300;ego=randn(rngStream,3,n);q=zeros(11,n);
            q(1:3,:)=4*randn(rngStream,3,n);q(8:9,:)=.1+2*rand(rngStream,2,n);
            q(10:11,:)=randn(rngStream,2,n);
            shape=[2.1;.9;.37;-.13];
            value=predictiveSafetyGeometry.rectangleClearance(ego,q,shape);
            [withWitness,gradient]=predictiveSafetyGeometry.rectangleClearance(ego,q,shape);
            [interpreted,referenceGradient]=predictiveSafetyGeometry.rectangleClearanceNumeric(ego,q,shape);
            testCase.verifyEqual(withWitness,interpreted,AbsTol=1e-12);
            testCase.verifyEqual(gradient,referenceGradient,AbsTol=1e-8);
            testCase.verifyGreaterThan(max(value),0);
            testCase.verifyLessThan(min(value),0);
            testCase.verifyEqual(value,withWitness,AbsTol=1e-12);
        end

        function continuousArcEnvelopeContainsInteriorFootprintsThroughStopping(testCase,slipInterval)
            root=[12;-4;2.8;4;-.5;.03;1.6;2.4;.95;.35;-.21];
            root(6)=mean(slipInterval);
            times=[0,.1,1,2,4,12];limits=[[-3;1],slipInterval(:)];
            [box,centers]=predictiveSafetyGeometry.targetMotionEnvelope(root,limits,times);
            ego=[7;8]+zeros(2,numel(times));
            d=predictiveSafetyGeometry.targetEnvelopeRange(ego,box,centers);
            vertices=root(10:11)+root(8:9).*[-1,1,1,-1;-1,-1,1,1];
            for acceleration=linspace(limits(1,1),limits(2,1),5)
                for slip=linspace(limits(1,2),limits(2,2),7)
                    trial=root;trial(5:6)=[acceleration;slip];
                    pose=predictiveSafetyGeometry.predictTarget(trial,times);
                    testCase.verifyLessThanOrEqual(d,vecnorm(ego-pose(1:2,:))+1e-10);
                    for j=1:numel(times)
                        yaw=pose(3,j);basis=box(3,j);
                        world=pose(1:2,j)+[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)]*vertices;
                        local=[cos(basis),sin(basis);-sin(basis),cos(basis)]*(world-box(1:2,j));
                        extent=max(abs(local-box(10:11,j)),[],2);
                        testCase.verifyLessThanOrEqual(extent,box(8:9,j)+1e-10);
                    end
                end
            end
            exact=predictiveSafetyGeometry.predictTarget(root,times);
            singleton=repmat(root(5:6).',2,1);
            box=predictiveSafetyGeometry.targetMotionEnvelope(root,singleton,times);
            testCase.verifyEqual(box,exact,AbsTol=1e-10);
            root(4)=0;
            box=predictiveSafetyGeometry.targetMotionEnvelope(root,limits,times);
            testCase.verifyEqual(box,repmat(root,1,numel(times)));
        end

        function conditionedFamilySupportsContainMovingAndStoppedTargets(testCase)
            q=[3;-2;.4;8;-1;.02;1.6;2.4;.95;.2;-.1];
            kappa=sin(q(6))/q(7);course=q(3)+q(6);
            intervals=[7,-3,kappa-.025,course-.15;9,1,kappa+.025,course+.15];
            family=nrmmTargetHistory("constantMotion", 'condition',[],0,intervals,[],zeros(2,0),zeros(2,0));
            uncertainty=struct('positionRadius',.04,'courseRadius',.15,'speedRadius',1, ...
                'accelerationRadius',2,'curvatureRadius',.025,'sideslipRadius',.05, ...
                'speedInterval',intervals(:,1),'accelerationInterval',intervals(:,2), ...
                'curvatureInterval',intervals(:,3),'family',family,'familyTime',0,'familyYawRadius',0);
            stream=RandStream('mt19937ar',Seed=35);
            for time=[0,.1,2,5]
                data=predictiveSafetyGeometry.targetSupportData(q,uncertainty,time);
                nominal=predictiveSafetyGeometry.predictTarget(q,time);
                for angle=linspace(-pi,pi,9)
                    n=1.7*[cos(angle);sin(angle)];
                    bound=predictiveSafetyGeometry.targetDirectionalSupport(q,uncertainty,time,n);
                    positionBound=predictiveSafetyGeometry.targetSupportFromData(data,1,n.');
                    for sample=1:20
                        parameter=intervals(1,:).'+diff(intervals,1,1).'.*rand(stream,4,1);
                        trial=q;trial(4:5)=parameter(1:2);trial(6)=asin(q(7)*parameter(3));
                        trial(3)=parameter(4)-trial(6);
                        actual=predictiveSafetyGeometry.targetDirectionalSupport(trial,[],time,n);
                        pose=predictiveSafetyGeometry.predictTarget(trial,time);
                        testCase.verifyLessThanOrEqual(actual,bound+1e-11);
                        testCase.verifyLessThanOrEqual(n.'*(pose(1:2)-nominal(1:2)),positionBound+1e-11);
                    end
                end
            end
        end
        function directionalTargetSupportContainsCurvedAndStoppedParameterFamilies(testCase)
            q=[3;-2;.4;8;-1;.02;1.6;2.4;.95;.2;-.1];
            error=struct('positionRadius',.04,'courseRadius',.2,'speedRadius',1, ...
                'accelerationRadius',2,'curvatureRadius',.03,'sideslipRadius',.06, ...
                'speedInterval',[7;9],'accelerationInterval',[-3;1],'curvatureInterval',[-.02;.04]);
            stream=RandStream('mt19937ar',Seed=34);
            for time=[0,.1,2,5]
                for angle=linspace(-pi,pi,9)
                    n=[cos(angle);sin(angle)];
                    bound=predictiveSafetyGeometry.targetDirectionalSupport(q,error,time,n);
                    for sample=1:30
                        a=2*rand(stream,6,1)-1;trial=q;
                        trial(1:2)=q(1:2)+error.positionRadius*a(1:2)/max(1,norm(a(1:2)));
                        trial(4)=8+a(3);trial(5)=-1+2*a(4);
                        trial(6)=asin(q(7)*(.01+.03*a(5)));
                        trial(3)=q(3)+q(6)+error.courseRadius*a(6)-trial(6);
                        actual=predictiveSafetyGeometry.targetDirectionalSupport(trial,[],time,n);
                        testCase.verifyLessThanOrEqual(actual,bound+1e-12);
                    end
                end
            end
        end
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
        function gyroTransportKeepsEarlierCourseInformationWhenANewSetWidens(testCase)
            [q,prior,u,memory]=localFixture();
            memory.relativeCourseInterval=[.01;.03];memory.curvatureInterval=[0;0];
            u.predictionSet.courseRadius=.8;
            records=struct('time',{0,.025,.05},'yawRate',{.2,.2,.2});
            u.predictionSet.measurementHistory=struct('records',records, ...
                'gyroscopeNoise',.001,'yawAccelerationMaximum',.1);
            [selected,bound,next,info]=predictiveSafetyGeometry.reconcileTarget(q,prior,u,memory,.05);
            testCase.verifyTrue(info.courseIntersection);
            testCase.verifyLessThan(bound.target.courseRadius,.021);
            expected=memory.relativeCourseInterval-.01+[-1;1]*(.001*.05+2*.1*.025^2/4);
            testCase.verifyEqual(next.relativeCourseInterval,expected,AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(selected(3)+selected(6),expected(1));
            testCase.verifyLessThanOrEqual(selected(3)+selected(6),expected(2));
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
        'accelerationInterval',[-.2;.2],'curvatureInterval',[-.1;.1], ...
        'courseCenter',sum(q([3,6])),'referenceEgoPose',[0;0;0]);
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

function corners=localCorners(q)
    x=q(8,:).*[-1;1;1;-1]+q(10,:);y=q(9,:).*[-1;-1;1;1]+q(11,:);
    corners=complex(q(1,:),q(2,:))+exp(1i*q(3,:)).*complex(x,y);
end

function localVerifyCornerContainment(testCase,target,box,radius)
    w=(localCorners(target)-complex(box(1,:),box(2,:))).*exp(-1i*box(3,:));
    dx=max(0,abs(real(w)-box(10,:))-box(8,:));
    dy=max(0,abs(imag(w)-box(11,:))-box(9,:));
    testCase.verifyLessThanOrEqual(max(hypot(dx,dy),[],1),radius+1e-10);
end
