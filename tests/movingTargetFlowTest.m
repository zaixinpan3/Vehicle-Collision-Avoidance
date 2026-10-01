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
    end
end
