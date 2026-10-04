classdef ordinaryDistanceDualTest < matlab.unittest.TestCase
    % Ordinary finite-body distance and the fixed-multiplier trajectory rows.
    properties (TestParameter)
        targetPose = struct('separated',[10;3;-.2], ...
            'oblique',[3;8;1.3],'overlapping',[.5;.2;.4], ...
            'touching',[4.8;0;0],'coincident',[0;0;0]);
        shapeOffset = {[0;0],[.3;-.1]};
        poseCoordinate = {1,2,3};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
        end
    end
    methods (Test)
        function optimizedDualEqualsIndependentRectangleDistance(testCase,targetPose,shapeOffset)
            pose=[0;0;0];shape=[2.4;.95;shapeOffset];
            rows=predictiveSafetyGeometry.dualLinearization(pose,shape,targetPose,shape);
            expected=predictiveSafetyGeometry.rectangleNumeric(pose,shape,targetPose,shape);
            testCase.verifyEqual(rows.distance,expected,AbsTol=1e-5);
            testCase.verifyGreaterThanOrEqual(rows.lambda,-1e-9*ones(4,1));
            testCase.verifyLessThanOrEqual(norm(rows.normal),1+1e-8);
            testCase.verifyLessThanOrEqual(abs(rows.dualityGap),1e-6);
        end
        function distantRecoveryGeometryDoesNotDependOnConicPrecision(testCase)
            shape=[2.4;.95;0;0];pose=[100.2370617530728;-1.0322098338241903;-.05532038315761686];
            target=[67.769375;0;0];
            rows=predictiveSafetyGeometry.dualLinearization(pose,shape,target,shape);
            testCase.verifyEqual(rows.distance,27.633427417605073,AbsTol=1e-11);
            testCase.verifyLessThanOrEqual(abs(rows.dualityGap),1e-11);
            testCase.verifyGreaterThanOrEqual(rows.lambda,zeros(4,1));
        end
        function fixedMultiplierRowsRespectFullEgoSize(testCase)
            shape=[2;1;0;0];target=[10;0;0];lambda=[0;0;1;0];
            values=predictiveSafetyGeometry.fixedDualRows([0;0;0],shape,target,shape,lambda);
            % Front corners are at x=2; target rear face is at x=8.
            testCase.verifyEqual(values,[10;6;6;10],AbsTol=1e-12);
            testCase.verifyEqual(min(values),6,AbsTol=1e-12);
        end
        function fixedMultiplierYawTangentMatchesFiniteDifferences(testCase,poseCoordinate)
            pose=[1;.2;.3];shape=[2.4;.95;.2;-.1];target=[10;3;-.2];
            rows=predictiveSafetyGeometry.dualLinearization(pose,shape,target,shape);
            delta=zeros(3,1);delta(poseCoordinate)=1e-6;
            high=predictiveSafetyGeometry.fixedDualRows(pose+delta,shape,target,shape,rows.lambda);
            low=predictiveSafetyGeometry.fixedDualRows(pose-delta,shape,target,shape,rows.lambda);
            testCase.verifyEqual((high-low)/(2e-6),rows.jacobian(:,poseCoordinate),AbsTol=3e-9);
        end
        function zeroMultipliersRemainZeroAfterMovingTheEgo(testCase)
            shape=[2.4;.95;0;0];
            [value,jacobian]=predictiveSafetyGeometry.fixedDualRows([30;5;.7],shape, ...
                [0;0;0],shape,zeros(4,1));
            testCase.verifyEqual(value,zeros(4,1),AbsTol=0);
            testCase.verifyEqual(jacobian,zeros(4,3),AbsTol=0);
        end
        function overlappingRectanglesHaveAFeasibleDualEscapeDirection(testCase)
            shape=[2.4;.95;0;0];pose=[0;0;0];target=[.5;.2;.4];
            rows=predictiveSafetyGeometry.supportLinearization(pose,shape,target,shape);
            testCase.verifyEqual(norm(rows.normal),1,AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(rows.lambda,zeros(4,1));
            testCase.verifyLessThan(min(rows.value),0);
            translation=-min(rows.value)+.2;
            moved=pose+[translation*rows.normal;0];
            value=predictiveSafetyGeometry.fixedDualRows(moved,shape,target,shape,rows.lambda);
            distance=predictiveSafetyGeometry.rectangleNumeric(moved,shape,target,shape);
            testCase.verifyEqual(min(value),.2,AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(distance,min(value)-1e-12);
        end
        function optimizedMultipliersAreInvariantToWorldTranslation(testCase)
            pose=[1;.2;.3];target=[10;3;-.2];shape=[2.4;.95;.2;-.1];offset=[100;-70;0];
            first=predictiveSafetyGeometry.dualLinearization(pose,shape,target,shape);
            translated=predictiveSafetyGeometry.dualLinearization(pose+offset,shape,target+offset,shape);
            testCase.verifyEqual(translated.distance,first.distance,AbsTol=1e-5);
            testCase.verifyEqual(translated.normal,first.normal,AbsTol=2e-6);
        end
    end
end
