classdef rectangleRotationSupportTest < matlab.unittest.TestCase
    properties (TestParameter)
        yawRadius = {0, .02, .4, 1.8, pi};
        yaw = {0, pi-.03, -2.4};
        offset = {[0;0], [.6;-.35]};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
        end
    end
    methods (Test)
        function enclosesAllSampledRotationsAndAttainsTheirLimit(testCase,yawRadius,yaw,offset)
            shape=[2.4;.95;offset];normal=[1,0;0,1;-.6,.8;2,-3;0,0];
            [support,inflation]=predictiveSafetyGeometry.rotationSupport(shape,yaw,yawRadius,normal);
            sampled=localSampledSupport(shape,yaw,yawRadius,normal);
            reach=max(vecnorm(offset+shape(1:2).*[-1,1,1,-1;-1,-1,1,1]));
            discretization=vecnorm(normal,2,2)*reach*yawRadius/2000;
            testCase.verifyGreaterThanOrEqual(support-sampled,-1e-12*ones(size(support)));
            testCase.verifyLessThanOrEqual(support-sampled,discretization+1e-12);
            testCase.verifyGreaterThanOrEqual(inflation,zeros(size(inflation)));
            testCase.verifyLessThanOrEqual(inflation,vecnorm(normal,2,2)*2*reach*sin(yawRadius/2)+1e-12);
        end
        function unknownYawNeedsOnlyTheCircumradius(testCase)
            radius=hypot(2.4,.95);
            [support,inflation]=predictiveSafetyGeometry.rotationSupport([2.4;.95;0;0],0,pi,[0,1]);
            testCase.verifyEqual(support,radius,AbsTol=1e-14);
            testCase.verifyEqual(inflation,radius-.95,AbsTol=1e-14);
            testCase.verifyLessThan(support,.95+2*radius-3.5);
        end
        function knownYawAddsNoInflationEvenWithAnOffset(testCase)
            [support,inflation]=predictiveSafetyGeometry.rotationSupport([2.4;.95;.6;-.35],0,0,[0,1;0,-1]);
            testCase.verifyEqual(support,[.6;1.3],AbsTol=1e-14);
            testCase.verifyEqual(inflation,[0;0],AbsTol=0);
        end
        function aCommonWorldRotationPreservesTheSupport(testCase)
            normal=[1,0;.6,-.8];angle=2.3;r=[cos(angle),-sin(angle);sin(angle),cos(angle)];
            a=predictiveSafetyGeometry.rotationSupport([2.4;.95;.2;-.1],-.4,.7,normal);
            b=predictiveSafetyGeometry.rotationSupport([2.4;.95;.2;-.1],-.4+angle,.7,normal*r.');
            testCase.verifyEqual(a,b,AbsTol=1e-13);
        end
    end
end

function support=localSampledSupport(shape,yaw,radius,normal)
    angles=yaw+linspace(-radius,radius,4001);
    corners=shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1];
    px=cos(angles).'*corners(1,:)-sin(angles).'*corners(2,:);
    py=sin(angles).'*corners(1,:)+cos(angles).'*corners(2,:);
    support=max(normal*[px(:).';py(:).'],[],2);
end
