classdef nrmmCompactPredictionTest < matlab.unittest.TestCase
    properties (SetAccess=private)
        observerDesign
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for name=["controller","config","estimator","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,name)));
            end
            cfg=collisionAvoidanceControllerConfig();
            estimator=estimatorConfigurationFromController(estimatorControllerIntegrationConfig(),cfg);
            testCase.observerDesign=synthesizeNrmmObserverGains(estimator.observer);
        end
    end
    methods (Test)
        function sampledResetContractsWithoutRepeatedOpenLoopInflation(testCase)
            period=.0125;q=exp(-8*period);gain=[1-q^2;(1-q)^2/period];
            c=nrmmObserverCertificate('sampled',testCase.observerDesign,period,gain);
            testCase.verifyLessThan(c.coreContraction,1);
            testCase.verifyLessThan(c.filterContraction,1);
            scaled=c.filterScaling*c.filterTransition/c.filterScaling;
            testCase.verifyEqual(scaled'*c.filterMetric*scaled-c.filterMetric,-eye(2),AbsTol=1e-10);
            forcing=[.01;.03];limit=(eye(2)-c.resetMatrix)\forcing;state=zeros(2,1);
            for k=1:1000,state=c.resetMatrix*state+forcing;end
            testCase.verifyLessThanOrEqual(state,limit+1e-10);
            testCase.verifyFalse(c.forcingCertified);
        end
        function completeStateSlopeContainsIndependentVehicleJacobians(testCase)
            cfg=collisionAvoidanceControllerConfig();rng(901);
            for speed=[.1,3.2,8]
                center=[0;0;.2;speed;.03;.01];radius=[.01;.01;.02;.01;.005;.002];
                x=nrmmSet(center,diag(radius));u=nrmmSet([.03;-.8],diag([.005;.1]));
                theta=nrmmSet(zeros(4,1),.02*eye(4));bound=nrmmJointEnclosure('stateSlope',x,u,theta,cfg);
                for k=1:12
                    p=.02*(2*rand(4,1)-1);plant=nrmmJointPrediction('plantConfiguration',cfg,p);
                    actual=center+radius.*(2*rand(6,1)-1);input=u.center+u.generator*(2*rand(2,1)-1);
                    jac=nonlinearBicycleModel.jacobian(actual,input,plant);
                    testCase.verifyLessThanOrEqual(abs(jac),bound+1e-10);
                end
            end
        end
        function exactPhysicalSlabContainsOdeSolutionsAtFullBraking(testCase)
            cfg=collisionAvoidanceControllerConfig();rng(902);
            for brake=[-.3,-1]
                center=[0;0;.1;10;.03;.01];g=zeros(6,2);g(4,1)=.005;g(5,2)=.002;
                input=nrmmSet([.015;brake]);parameters=nrmmSet(zeros(4,1),[.005,0;0,-.005;.005,0;0,.005]);
                result=nrmmJointEnclosure('exactPlant',nrmmSet(center,g),input,parameters,cfg,.0125);
                [lo,hi]=bounds(result.endpoint);[sl,sh]=bounds(result.slab);
                for k=1:5
                    z=2*rand(2,1)-1;plant=nrmmJointPrediction('plantConfiguration',cfg,parameters.generator*z);
                    [~,path]=ode45(@(t,x)nonlinearBicycleModel.derivative(x,input.center,plant), ...
                        [0,.0125],center+g*z,odeset('RelTol',1e-10,'AbsTol',1e-12));
                    testCase.verifyGreaterThanOrEqual(path(end,:)',lo-1e-9);
                    testCase.verifyLessThanOrEqual(path(end,:)',hi+1e-9);
                    testCase.verifyGreaterThanOrEqual(min(path,[],1)',sl-1e-9);
                    testCase.verifyLessThanOrEqual(max(path,[],1)',sh+1e-9);
                end
                testCase.verifyTrue(result.realArithmeticEnclosure);
                testCase.verifyFalse(result.floatingPointCertified);
            end
        end
        function compactKernelRetainsExactRadarInnovationDecay(testCase)
            [state,input,pseudo,design]=localObserver(testCase.observerDesign);
            h=.025;dm=zeros(11,1);dp=zeros(4,1);g=zeros(15,1);
            compact=nrmmJointPrediction('observerCompact',state,g,input,dm,pseudo,dp,design,h);
            a=design.target.innovationGains(1);w=input.yawRate;
            rotation=[cos(w*h),sin(w*h);-sin(w*h),cos(w*h)];
            testCase.verifyEqual(compact.point(14:15)-compact.point(8:9), ...
                exp(-a*h)*rotation*(state(14:15)-state(8:9)),AbsTol=1e-12);
        end
        function compactKernelResolvesChangingTargetJerk(testCase)
            [state,input,pseudo,design]=localObserver(testCase.observerDesign);
            state(10:13)=[3;2;.5;-.3];input.yawRate=.3;h=.025;
            compact=nrmmJointPrediction('observerCompact',state,eye(15),input,zeros(11,15), ...
                pseudo,zeros(4,15),design,h);
            reference=nrmmJointPrediction('observerRk4',state,eye(15),input,zeros(11,15), ...
                pseudo,zeros(4,15),design,h/200,200);
            testCase.verifyEqual(compact.point,reference.point,AbsTol=2e-6);
            testCase.verifyEqual(compact.gradient,reference.gradient,AbsTol=2e-6);
        end
        function compactKernelDerivativeIncludesSharedGyroAndTargetFeedback(testCase)
            [state,input,pseudo,design]=localObserver(testCase.observerDesign);rng(903);
            g=.03*randn(15,1);dm=.03*randn(11,1);dp=.03*randn(4,1);h=.025;
            analytic=nrmmJointPrediction('observerCompact',state,g,input,dm,pseudo,dp,design,h);
            step=1e-5;values=zeros(15,2);
            for k=1:2
                delta=(2*k-3)*step;m=input;
                m.gnssVelocity=m.gnssVelocity+delta*dm(3:4);
                m.bodyAcceleration=m.bodyAcceleration+delta*dm(5:6);m.yawRate=m.yawRate+delta*dm(7);
                out=nrmmJointPrediction('observerCompact',state+delta*g,zeros(15,1),m,zeros(11,1), ...
                    pseudo+delta*dp,zeros(4,1),design,h);values(:,k)=out.point;
            end
            testCase.verifyEqual(analytic.gradient,(values(:,2)-values(:,1))/(2*step),AbsTol=2e-8,RelTol=2e-6);
        end
        function nonlinearObserverRemainderContainsAccurateReferenceFlow(testCase)
            [state,input,pseudo,design]=localObserver(testCase.observerDesign);h=.0125;
            observer=nrmmSet(state([3,4,7,1,2,8:15]));
            p=struct('body',nrmmSet(pseudo(1:2)),'rotation',nrmmSet(pseudo(3:4)), ...
                'phase',[],'magnitude',[]);
            bound=nrmmJointEnclosure('exactObserver',observer,nrmmSet(state(5:6)), ...
                nrmmSet(input.gnssVelocity),nrmmSet(input.bodyAcceleration),nrmmSet(input.yawRate),p,design,h);
            reference=nrmmJointPrediction('observerRk4',state,zeros(15,1),input,zeros(11,1), ...
                pseudo,zeros(4,1),design,h/200,200);
            expected=reference.point([3,4,7,1,2,8:15]);[lo,hi]=bounds(bound.endpoint);
            testCase.verifyGreaterThanOrEqual(expected,lo-1e-10);
            testCase.verifyLessThanOrEqual(expected,hi+1e-10);
            testCase.verifyLessThan(max(hi-lo),.02);
        end
        function targetExtensionRemainsFiniteAtZeroVelocity(testCase)
            domain=testCase.observerDesign.target.domain;
            q=nrmmSet([0;0],.001*eye(2));s=nrmmSet([0;0],.001*eye(2));
            phi=nrmmJointEnclosure('targetPhi',q,s,domain);
            testCase.verifyTrue(all(isfinite(radius(phi))));
            testCase.verifyLessThan(max(radius(phi)),1);
        end
        function completeJointCertificateRetainsFutureFeedbackAndSharedParameters(testCase)
            [center,context]=localJoint(testCase.observerDesign);
            generator=zeros(35,2);generator(12,1)=1e-6;generator(25,2)=1e-6;
            context.first=true;
            first=nrmmJointPrediction('compactEnclosure',center,generator,context);
            testCase.verifyEqual(first.inputRadius,zeros(2,1));
            testCase.verifyEqual(radius(first.endpoint(12:15)),[1e-6;0;0;0],AbsTol=1e-14);
            testCase.verifyTrue(first.futureTargetObserverIncluded);
            testCase.verifyTrue(first.realArithmeticEnclosure);
            context.first=false;context.gain(2,4)=-.03;context.targetGain(1,2)=-.01;
            generator(29,2)=1e-5;
            future=nrmmJointPrediction('compactEnclosure',center,generator,context);
            testCase.verifyGreaterThan(norm(future.inputRadius),0);
            testCase.verifyFalse(future.floatingPointCertified);
            [lo,hi]=bounds(future.endpoint);
            for sign=[-1,1]
                point=nrmmJointPrediction('coupledLinearize',center+sign*sum(generator,2),context);
                testCase.verifyGreaterThanOrEqual(point.point(1:35),lo-1e-8);
                testCase.verifyLessThanOrEqual(point.point(1:35),hi+1e-8);
            end
        end
        function uncertainPerceptionBoundaryRetainsBothDetectionOutcomes(testCase)
            [center,context]=localJoint(testCase.observerDesign);context.first=false;
            context.cfg.collision.encounterRangeMeters=20;
            generator=zeros(35,1);generator(22)=.001;
            image=nrmmJointPrediction('compactEnclosure',center,generator,context);
            testCase.verifyGreaterThan(image.joinedDetectionSamples,0);
            [lo,hi]=bounds(image.endpoint);
            for sign=[-1,1]
                sample=nrmmJointPrediction('coupledLinearize',center+sign*generator,context);
                testCase.verifyGreaterThanOrEqual(sample.point(1:35),lo-1e-8);
                testCase.verifyLessThanOrEqual(sample.point(1:35),hi+1e-8);
            end
        end
    end
end

function [center,context]=localJoint(design)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8));
    trim=nonlinearBicycleModel.cruise(cfg,0);x=trim.state;u=trim.input;
    h=.0125;q=exp(-design.positionVelocity.bandwidth*h);
    m=struct('gnssPosition',x(1:2),'gnssVelocity',x(4:5), ...
        'bodyAcceleration',zeros(2,1),'yawRate',0,'heldInput',u,'velocityErrorMaximum',0, ...
        'radarDetectionAvailable',true,'radarRelativePosition',[20;0]);
    c=struct('design',design,'samplePeriod',h,'integrationStep',h/8,'integrationSubstepCount',8, ...
        'noiseRadius',zeros(7,1),'heldInputAvailable',false,'currentMeasurement',m, ...
        'positionVelocity',struct('gain',[1-q^2;(1-q)^2/h],'point',[x(1:2);x(4:5)]), ...
        'targetObserver',struct('noiseRadius',1e-7));
    center=[x;x(1:5);zeros(4,1);u;x(1:2);x(4:5);20;0;0;8;0;0;20;0;8;0;0;0;20;0];
    context=struct('cfg',cfg,'contract',c,'input',u,'gain',zeros(2,6),'targetGain',zeros(2,6), ...
        'reference',[-20;0;0;0;0;0],'egoReference',x,'referenceYaw',0, ...
        'duration',2*h,'first',true,'targetRearAxleDistance',1.5);
end

function [state,input,pseudo,design]=localObserver(design)
    % Packed vector-field coordinates, including the moving position prior.
    state=[8;.03;1;2;1.01;1.99;.1;20;1;7;.2;.15;.03;20.02;.98];
    input=struct('bodyAcceleration',[-.1;.2],'yawRate',.015, ...
        'gnssVelocity',[8;.8],'radarDetectionAvailable',true);
    pseudo=[8;.025;.98;.08];
    if design.target.domain.speedMinimum==0
        design.target.domain.speedMinimum=design.target.domain.regularizationSpeedMinimum;
    end
end
