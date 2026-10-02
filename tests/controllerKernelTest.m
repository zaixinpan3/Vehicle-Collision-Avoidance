classdef controllerKernelTest < matlab.unittest.TestCase
    % The optional compiled kernels against the interpreted controller source.
    properties (TestParameter)
        integrationStep = {.005,.025,.05}
        fraction = {1,.5}
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            native=fullfile(root,'solver','controller');
            for name=["bicycleSampleKernelMex","rectangleKernelMex"]
                testCase.assumeTrue(isfile(fullfile(native,name+"."+mexext)),'The optional kernels have not been built.');
            end
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(native));
        end
    end
    methods (Test)
        function kernelReproducesTheModelAndItsTangents(testCase,integrationStep,fraction)
            cfg=collisionAvoidanceControllerConfig(struct('nonlinear',struct('integrationStep',integrationStep)));
            duration=fraction*cfg.controller.sampleTime;
            count=max(1,round(nonlinearBicycleModel.meshCount(cfg)*fraction));
            stream=RandStream('mt19937ar','Seed',7);
            for trial=1:40
                x=[10*randn(stream,2,1);randn(stream);4+10*rand(stream);.8*randn(stream);.4*randn(stream)];
                u=[.15*randn(stream);max(-.9,min(.9,.4*randn(stream)))];
                [expected,a,b]=localReference(x,u,cfg,duration,count);
                [next,an,bn]=nonlinearBicycleModel.sample(x,u,cfg,[],duration);
                testCase.verifyEqual(next,expected,AbsTol=1e-12);
                testCase.verifyEqual(an,a,AbsTol=1e-11);
                testCase.verifyEqual(bn,b,AbsTol=1e-9);
                testCase.verifyEqual(nonlinearBicycleModel.sample(x,u,cfg,[],duration),next,AbsTol=0);
            end
        end
        function geometryKernelReproducesTheInterpretedDistance(testCase)
            stream=RandStream('mt19937ar','Seed',11);shape=[2.4;.95;.3;-.1];target=[2.4;.95;0;0];overlaps=0;
            for trial=1:400
                poseE=[4*randn(stream,2,1);pi*(2*rand(stream)-1)];
                poseT=[4*randn(stream,2,1);pi*(2*rand(stream)-1)];
                [distance,normal]=predictiveSafetyGeometry.rectangleNumeric(poseE,shape,poseT,target);
                [kernelDistance,kernelNormal]=rectangleKernelMex(poseE,shape,poseT,target);
                testCase.verifyEqual({kernelDistance,kernelNormal},{distance,normal});
                [wrapped,certificate]=predictiveSafetyGeometry.rectangle(poseE,shape,poseT,target);
                testCase.verifyEqual({wrapped,certificate.normal},{distance,normal});
                overlaps=overlaps+(distance==0);
            end
            testCase.verifyGreaterThan(overlaps,20);
            testCase.verifyLessThan(overlaps,380);
        end
        function kernelKeepsTheModelDomainErrors(testCase)
            cfg=collisionAvoidanceControllerConfig();
            testCase.verifyError(@()nonlinearBicycleModel.sample([0;0;0;.5;0;0],[0;0],cfg), ...
                'collisionAvoidanceController:nonlinearDomain');
            testCase.verifyError(@()nonlinearBicycleModel.sample([0;0;0;8;0;0],[0;1.5],cfg), ...
                'collisionAvoidanceController:nonlinearDomain');
        end
        function nominalValueKernelMatchesTheInterpretedPolicyEvaluation(testCase)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8));
            reference=nonlinearBicycleModel.cruise(cfg,.005);terminal=nonlinearBicycleModel.nominalTail(cfg,.005);
            native=fullfile(fileparts(fileparts(mfilename('fullpath'))),'solver','controller');
            testCase.assumeTrue(isfile(fullfile(native,"nominalClfKernelMex."+mexext)));
            e=[-20;.3;-1;.2;-.1];steps=60;
            [expected,tail]=nonlinearBicycleModel.nominalResidual(e,reference.input,reference,terminal,cfg,steps);
            [actual,nativeTail]=nominalClfKernelMex(e,reference.input,reference,terminal,cfg,steps);
            testCase.verifyEqual(actual,expected,AbsTol=1e-9);
            testCase.verifyEqual(nativeTail,tail);
        end
    end
end

function [x,a,b]=localReference(x,u,cfg,duration,count)
    h=duration/count;a=eye(6);b=zeros(6,2);
    for step=1:count
        [k1,a1,b1]=localStage(x,a,b,u,cfg);
        [k2,a2,b2]=localStage(x+h*k1/2,a+h*a1/2,b+h*b1/2,u,cfg);
        [k3,a3,b3]=localStage(x+h*k2/2,a+h*a2/2,b+h*b2/2,u,cfg);
        [k4,a4,b4]=localStage(x+h*k3,a+h*a3,b+h*b3,u,cfg);
        x=x+h*(k1+2*k2+2*k3+k4)/6;a=a+h*(a1+2*a2+2*a3+a4)/6;b=b+h*(b1+2*b2+2*b3+b4)/6;
    end
end
function [dx,da,db]=localStage(x,a,b,u,cfg)
    dx=nonlinearBicycleModel.derivative(x,u,cfg);[jx,ju]=nonlinearBicycleModel.jacobian(x,u,cfg);
    da=jx*a;db=jx*b+ju;
end
