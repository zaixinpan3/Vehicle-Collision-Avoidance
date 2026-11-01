classdef separationWitnessTest < matlab.unittest.TestCase
    % Continuous plane witnesses represent rectangle separation and deficit.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
        end
    end
    methods (Test)
        function obliqueSeparationMatchesEuclideanDistance(testCase)
            shape=[2.4;.95;.2;-.1];ego=[6;4;.193];target=[0;0;-.317];
            [distance,certificate]=predictiveSafetyGeometry.rectangle(ego,shape,target,shape);
            normal=certificate.normal;witness=localWitness(normal,target,shape);
            value=predictiveSafetyGeometry.separationWitnessRows(ego,shape,target,shape,witness);
            testCase.verifyEqual(min(value(1:4)),distance,AbsTol=1e-10);
            testCase.verifyGreaterThanOrEqual(min(value(5:8)),-1e-10);
        end
        function positiveSlackMeasuresTranslationalPenetration(testCase)
            shape=[2.4;.95;0;0];ego=[4;0;0];target=[0;0;0];
            value=predictiveSafetyGeometry.separationWitnessRows(ego,shape,target,shape,[0;2.4]);
            testCase.verifyEqual(min(value(1:4)),-.8,AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(min(value+[.8*ones(4,1);zeros(4,1)]),-1e-12);
        end
        function anOverlappingPairCannotUseAZeroNormal(testCase)
            shape=[2.4;.95;0;0];pose=zeros(3,1);
            for angle=linspace(-pi,pi,101)
                normal=[cos(angle);sin(angle)];witness=localWitness(normal,pose,shape);
                value=predictiveSafetyGeometry.separationWitnessRows(pose,shape,pose,shape,witness);
                testCase.verifyLessThan(min(value(1:4)),-1.8);
            end
        end
        function worldRotationDoesNotChangeTheSafetyFunction(testCase)
            shapeE=[2.4;.95;.2;-.1];shapeT=[2;.8;-.3;.15];
            ego=[6;4;.193];target=[1;-2;-.317];angle=.271;witness=[angle;3];
            original=predictiveSafetyGeometry.separationWitnessRows(ego,shapeE,target,shapeT,witness);
            turn=.137;rotation=[cos(turn),-sin(turn);sin(turn),cos(turn)];translation=[43;-27];
            ego=[rotation*ego(1:2)+translation;ego(3)+turn];
            target=[rotation*target(1:2)+translation;target(3)+turn];witness(1)=angle+turn;
            changed=predictiveSafetyGeometry.separationWitnessRows(ego,shapeE,target,shapeT,witness);
            testCase.verifyEqual(changed,original,AbsTol=2e-12);
        end
        function derivativesIncludeBothBodyYawAndPlaneRotation(testCase)
            shape=[2.4;.95;.2;-.1];target=[1;-2;-.317];z=[6;4;.193;.271;3];
            [~,jacobian]=predictiveSafetyGeometry.separationWitnessRows(z(1:3),shape,target,shape,z(4:5));
            for k=1:5
                step=zeros(5,1);step(k)=1e-6;plus=z+step;minus=z-step;
                vp=predictiveSafetyGeometry.separationWitnessRows(plus(1:3),shape,target,shape,plus(4:5));
                vm=predictiveSafetyGeometry.separationWitnessRows(minus(1:3),shape,target,shape,minus(4:5));
                testCase.verifyEqual((vp-vm)/2e-6,jacobian(:,k),AbsTol=2e-8);
            end
        end
        function normalizationPreservesTheTangentPlaneForALargeWitnessStep(testCase)
            shape=[2.4;.95;.2;-.1];ego=[6;4;.193];target=[1;-2;-.317];
            witness=[.271;3];step=[1.7;-.4];
            [value,jacobian]=predictiveSafetyGeometry.separationWitnessRows(ego,shape,target,shape,witness);
            updated=predictiveSafetyGeometry.retractSeparationWitness(witness,step);
            actual=predictiveSafetyGeometry.separationWitnessRows(ego,shape,target,shape,updated);
            testCase.verifyEqual(actual,(value+jacobian(:,4:5)*step)/hypot(1,step(1)),AbsTol=2e-12);
        end
    end
end

function witness=localWitness(normal,target,shape)
    rotation=[cos(target(3)),-sin(target(3));sin(target(3)),cos(target(3))];
    vertices=shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1];
    witness=[atan2(normal(2),normal(1));max(normal.'*rotation*vertices)];
end
