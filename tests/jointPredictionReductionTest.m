classdef jointPredictionReductionTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for folder=["controller","estimator","config"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,folder)));
            end
        end
    end
    methods (Test)
        function reducedJointFamilyContainsEveryDirectionalSupport(testCase)
            rng(1104);base=randn(12,20);g=base(:,randi(20,1,2000)).*(.1+rand(1,2000));
            g=g+.001*randn(size(g));transform=eye(12);transform(7:9,1:3)=-eye(3);
            small=reduceOrder(nrmmSet(zeros(12,1),g),80,transform);
            directions=[eye(12),randn(12,200)];
            testCase.verifyLessThanOrEqual(sum(abs(directions'*g),2),sum(abs(directions'*small.generator),2)+1e-10);
            testCase.verifyLessThanOrEqual(size(small.generator,2),80);
        end
        function repeatedSharedParameterUpdatesHaveBoundedStorage(testCase)
            rng(1105);g=randn(12,50)*.01;
            for k=1:100
                g=[.99*g,randn(12,20)*.001];
                small=reduceOrder(nrmmSet(zeros(12,1),g),80);
                testCase.verifyLessThanOrEqual(size(small.generator,2),80);g=small.generator;
            end
            testCase.verifyTrue(all(isfinite(g),'all'));
        end
        function batchedFamilyImagesEqualScalarImagesAcrossStopping(testCase)
            family=struct('epochTime',0,'lower',[2,5;-3,-.2;-.03,.01;-.2,.4], ...
                'upper',[3,6;-2,.1;-.01,.04;.1,.5]);
            for start=[-.1,0,.3]
                times=[0,.1,.5,2,5]+max(0,start);
                batch=nrmmTargetHistory('constantMotion','image',family,start,times,.02);
                for k=1:numel(times)
                    scalar=nrmmTargetHistory('constantMotion','image',family,start,times(k),.02);
                    testCase.verifyEqual(batch.lower(:,2*k-1:2*k),scalar.lower,AbsTol=1e-12);
                    testCase.verifyEqual(batch.upper(:,2*k-1:2*k),scalar.upper,AbsTol=1e-12);
                end
            end
        end
        function batchedTargetTubesMatchIndependentNodeQueries(testCase)
            q=[3;-2;.4;8;-1;.02;1.6;2.4;.95;.2;-.1];
            curvature=sin(q(6))/q(7);course=q(3)+q(6);
            intervals=[7,-3,curvature-.025,course-.15;9,1,curvature+.025,course+.15];
            family=nrmmTargetHistory('constantMotion','condition',[],0,intervals,[],zeros(2,0),zeros(2,0));
            uncertainty=struct('positionRadius',.04,'courseRadius',.15,'speedRadius',1, ...
                'accelerationRadius',2,'curvatureRadius',.025,'sideslipRadius',.05, ...
                'family',family,'familyTime',0,'familyYawRadius',0);
            time=[0,.1,2,5];
            for perNode=[false,true]
                states=q;
                if perNode,states=predictiveSafetyGeometry.predictTarget(q,[0,.2,.4,.6]);end
                batch=predictiveSafetyGeometry.targetErrorTube(states,uncertainty,time);
                for k=1:numel(time)
                    scalar=predictiveSafetyGeometry.targetErrorTube(states(:,min(k,size(states,2))),uncertainty,time(k));
                    testCase.verifyEqual(batch.positionRadius(k),scalar.positionRadius,AbsTol=1e-12);
                    testCase.verifyEqual(batch.yawRadius(k),scalar.yawRadius,AbsTol=1e-12);
                end
            end
        end
    end
end
