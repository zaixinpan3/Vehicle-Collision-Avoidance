classdef sweptCollisionIntervalTest < matlab.unittest.TestCase
    % Continuous separation of the declared nominal pose interpolant.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
        end
    end
    methods (Test)
        function sweptDualFindsOverlapAndGivesAnEndpointGradient(testCase)
            shape=[.2;.1;.025;-.01];target=repmat([0;0;.17;0;0;0;1.6;.15;.09;.02;-.01],1,2);
            x=[-1,1;.03,-.02;-.2,.4];d=predictiveSafetyGeometry.sweptRectangleDual(x,target,shape);
            testCase.verifyLessThan(d,0);
            for k=1:2
                testCase.verifyGreaterThan(predictiveSafetyGeometry.rectangle(x(:,k),shape,target(1:3,k),target(8:11,k)),0);
            end
            x(2,:)=x(2,:)+.8;
            [d,left,right]=predictiveSafetyGeometry.sweptRectangleDual(x,target,shape);
            step=1e-6;direction=[.3,-.2;-.2,.4;.4,.11];
            plus=predictiveSafetyGeometry.sweptRectangleDual(x+step*direction,target,shape);
            minus=predictiveSafetyGeometry.sweptRectangleDual(x-step*direction,target,shape);
            testCase.verifyGreaterThan(d,0);
            testCase.verifyEqual(sum([left,right].*direction,'all'),(plus-minus)/(2*step),AbsTol=1e-7);
        end
        function separatedEndpointsDoNotAuthorizeAnIntersampleCollision(testCase)
            shape=[.1;.1;0;0];target=[0;0;0;0;0;0;1.6;shape];
            result=predictiveSafetyGeometry.intervalClearance([-1;0;0],[1;0;0],shape,target,0,.05,.006);
            testCase.verifyFalse(result.certified);
            testCase.verifyEqual(result.minimumSampledClearance,0);
            testCase.verifyGreaterThan(numel(result.fractions),0);
        end
        function commonTranslationPreservesATightSafeGap(testCase)
            shape=[2.4;.95;0;0];target=[5;0;0;8;0;0;1.6;shape];
            result=predictiveSafetyGeometry.intervalClearance([0;0;0],[.4;0;0],shape,target,0,.05,.006);
            testCase.verifyTrue(result.certified);
            testCase.verifyGreaterThan(result.lowerBound,.19);
        end
        function aCloseSafePassIsResolvedByAdaptiveRefinement(testCase)
            shape=[.1;.1;0;0];target=[0;0;0;0;0;0;1.6;shape];
            result=predictiveSafetyGeometry.intervalClearance([-.5;.2062;0],[.5;.2062;0],shape,target,0,1,.006);
            testCase.verifyTrue(result.certified);
            testCase.verifyGreaterThan(result.lowerBound,0);
            testCase.verifyGreaterThanOrEqual(result.minimumSampledClearance,.006);
            testCase.verifyGreaterThan(numel(result.fractions),0);
        end
        function touchingIsRejectedEvenWithoutAnAddedBuffer(testCase)
            shape=[.1;.1;0;0];target=[0;0;0;0;0;0;1.6;shape];
            result=predictiveSafetyGeometry.intervalClearance([.2;0;0],[.4;0;0],shape,target,0,.05,0);
            testCase.verifyFalse(result.certified);
        end
        function rotatingAcceleratingTargetRespectsTheIntervalBound(testCase)
            shape=[.2;.1;.02;-.01];target=[2;2;pi;5;1;.1;1.6;shape];
            first=[0;0;0];last=[.3;.1;.15];duration=.05;
            result=predictiveSafetyGeometry.intervalClearance(first,last,shape,target,0,duration,.006);
            testCase.verifyTrue(result.certified);
            for fraction=linspace(0,1,101)
                pose=(1-fraction)*first+fraction*last;
                q=predictiveSafetyGeometry.predictTarget(target,fraction*duration);
                gap=predictiveSafetyGeometry.rectangle(pose,shape,q(1:3),q(8:11));
                testCase.verifyGreaterThanOrEqual(gap,result.lowerBound-1e-12);
            end
        end
    end
end
