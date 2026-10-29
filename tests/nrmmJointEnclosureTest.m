classdef nrmmJointEnclosureTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for name=["controller","config","estimator"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,name)));
            end
        end
    end
    methods (Test)
        function sharedSymbolsCancelBeforeNonlinearRemainders(testCase)
            a=nrmmSet([1;2],diag([.1;.2]));
            testCase.verifyEqual(radius(a-a),zeros(2,1));
            b=sin(a)+a.^2;
            for s=linspace(-1,1,51)
                x=a.center+a.generator*[s;-s];
                localContains(testCase,b,sin(x)+x.^2);
            end
        end
        function fullDomainPlantImageContainsInteriorSamples(testCase)
            cfg=collisionAvoidanceControllerConfig();
            x=[0;0;.1;12;.1;.04];u=[.03;-.25];
            xc=nrmmSet(x,[diag([.02;.02;.002;.03;.008;.002]),zeros(6,6)]);
            uc=nrmmSet(u,[zeros(2,6),diag([.002;.01]),zeros(2,4)]);
            pc=nrmmSet(zeros(4,1),[zeros(4,8),.01*eye(4)]);
            image=nrmmJointEnclosure('plant',xc,uc,pc,cfg,.0125);
            rng(39);
            for k=1:25
                z=2*rand(12,1)-1;sampleCfg=cfg;p=pc.generator*z;
                sampleCfg.tire.corneringStiffness=cfg.tire.corneringStiffness.*(1+p(1:2));
                sampleCfg.tire.frictionCoefficient=cfg.tire.frictionCoefficient.*(1+p(3:4));
                actual=localFlow(x+xc.generator*z,u+uc.generator*z,sampleCfg,.0125);
                localContains(testCase,image,actual);
            end
        end
        function fullBrakingUsesFiniteForceRanges(testCase)
            cfg=collisionAvoidanceControllerConfig();x=[0;0;0;12;.2;.05];
            for brake=[-1,1]
                u=[.03;brake];image=nrmmJointEnclosure('field',nrmmSet(x),nrmmSet(u),nrmmSet(zeros(4,1)),cfg);
                localContains(testCase,image,nonlinearBicycleModel.derivative(x,u,cfg));
                testCase.verifyTrue(all(isfinite(radius(image))));
            end
        end
        function invalidInputSetsDoNotReceiveCertificates(testCase)
            cfg=collisionAvoidanceControllerConfig();x=nrmmSet([0;0;0;10;0;0]);
            testCase.verifyError(@()nrmmJointEnclosure('field',x,nrmmSet([0;-.99],diag([0;.02])), ...
                nrmmSet(zeros(4,1)),cfg),'nrmmJointEnclosure:domain');
        end
        function jointImageContainsTheActualObserverAndFeedbackMap(testCase)
            [center,context]=localFixture();context.contract.noiseRadius(:)=0;
            g=zeros(21,4);g(1:2,1:2)=.0001*eye(2);g(4,3)=.0001;g(12,4)=.0001;
            image=nrmmJointEnclosure('hold',center,g,context);
            rng(41);
            for k=1:6
                z=2*rand(4,1)-1;
                actual=nrmmJointPrediction('hold',center+g*z,zeros(7,4),context);
                actual(28:29)=actual(28:29)-context.input;
                localContains(testCase,nrmmSet(image.point,image.generator),actual);
            end
            testCase.verifyTrue(image.realArithmeticEnclosure);
            testCase.verifyFalse(image.floatingPointCertified);
        end
        function decisionCorrectionsAndNoiseAreEnclosedTogether(testCase)
            [center,context]=localFixture();context.contract.noiseRadius(:)=1e-6;
            context.contract.noiseRadius(3:4)=0;
            context.gain=[0,-.01,-.02,0,-.01,-.01;0,0,0,-.01,0,0];
            [middle,am,bm,next,a,b]=nonlinearBicycleModel.hold(center(1:6),context.input,context.cfg);
            context.a=a;context.b=b;context.am=am;context.bm=bm;context.defect=zeros(6,1);
            decisionRadius=[1e-5;1e-5;1e-5;1e-4;1e-5;1e-5;1e-5;1e-5];
            generator=zeros(21,1);generator(12)=1e-5;
            image=nrmmJointEnclosure('hold',center,generator,context,decisionRadius);
            lift=zeros(21,6);lift(1:6,:)=eye(6);lift(7:11,1:5)=eye(5);
            rng(53);
            for k=1:8
                correction=decisionRadius.*(2*rand(8,1)-1);sample=context;
                sample.input=context.input+correction(7:8);sample.reference=context.reference+correction(1:6);
                noise=context.contract.noiseRadius.*(2*rand(7,4)-1);
                actual=nrmmJointPrediction('hold',center+lift*correction(1:6)+generator*(2*rand-1),noise,sample);
                actual(1:21)=actual(1:21)-lift*(a*correction(1:6)+b*correction(7:8));
                actual(22:27)=actual(22:27)-am*correction(1:6)-bm*correction(7:8);
                actual(28:29)=actual(28:29)-sample.input;
                localContains(testCase,nrmmSet(image.point,image.generator),actual);
            end
            testCase.verifyTrue(all(isfinite([next;middle;image.remainderRadius])));
        end
        function aKnownCurrentObserverIsNotShiftedTwice(testCase)
            [center,context]=localFixture();context.first=true;context.contract.noiseRadius(:)=0;
            correction=[.01;-.02;.001;0;0;0];old=center;old(1:6)=old(1:6)-correction;
            context.a=eye(6);context.b=zeros(6,2);context.am=eye(6);context.bm=zeros(6,2);context.defect=zeros(6,1);
            domain=struct('lower',[correction;0;0],'upper',[correction;0;0]);
            image=nrmmJointEnclosure('hold',old,zeros(21,0),context,domain);
            actual=nrmmJointPrediction('hold',center,zeros(7,4),context);
            actual(1:6)=actual(1:6)-correction;actual(7:11)=actual(7:11)-correction(1:5);
            actual(22:27)=actual(22:27)-correction;actual(28:29)=actual(28:29)-context.input;
            localContains(testCase,nrmmSet(image.point,image.generator),actual);
        end
        function rotationRowsImplyExactVertexSeparation(testCase)
            shape=[2.4;.95;.2;-.1];target=[1;1;0;0];pose=[7;1;.4];q=[0;0;-.2];
            lambda=[1;0;0;0];[g,j]=predictiveSafetyGeometry.fixedDualRows(pose,shape,q,target,lambda);
            vertices=shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1];
            for eta=[.2,2,Inf]
                [rows,bounds]=predictiveSafetyGeometry.conservativeRotationRows(-j,g,[0,0,1],vecnorm(vertices).',eta);
                testCase.verifyEqual(bounds,[g;g]);
                for angle=linspace(-min(eta,6),min(eta,6),41)
                    correction=[15;0;angle];
                    if all(rows*correction<=bounds)
                        exact=predictiveSafetyGeometry.fixedDualRows(pose+correction,shape,q,target,lambda);
                        testCase.verifyGreaterThanOrEqual(exact,-1e-10);
                    end
                end
            end
        end
        function terminalImageContainsStopAndMovingBranches(testCase)
            context=struct('deceleration',4,'sampleTime',.05);
            center=[2;1;.1;8;.3;.1];g=diag([.01;.01;.002;.2;.02;.01]);
            rng(52);
            for t=[.5,2,3]
                image=nrmmJointEnclosure('terminalPose',nrmmSet(center,g),context,t);
                for k=1:10
                    x=center+g*(2*rand(6,1)-1);path=terminalSafeSet.continuation(context,x,round(2*t/.05)/2);
                    localContains(testCase,image,path.pose(:,end));
                end
            end
        end
        function terminalResidualCoversDecisionAndPhysicalError(testCase)
            context=struct('deceleration',4,'sampleTime',.05);
            y=[2;1;.1;8;.3;.1];r=[.01;.01;.002;.02;.002;.001];g=diag(r/2);
            image=terminalSafeSet.encloseContinuation(context,y,-r,r,g,4);
            rng(71);
            for k=1:8
                decision=r.*(2*rand(6,1)-1);x=y+decision+g*(2*rand(6,1)-1);
                path=terminalSafeSet.continuation(context,x,4);
                for j=1:numel(path.time)
                    residual=path.pose(:,j)-image.referencePose(:,j)-image.jacobian(:,:,j)*decision;
                    localContains(testCase,image.residual{j},residual);
                end
            end
            testCase.verifyFalse(image.terminalMembershipCertified);
        end
        function nonlinearModeRequiresAnObserverSnapshot(testCase)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
                'controller',struct('horizonSteps',2,'jointNonlinearPrediction',true), ...
                'solver',struct('timeLimitSeconds',30)));
            x=nonlinearBicycleModel.cruise(cfg,0).state;
            ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
            testCase.verifyError(@()collisionAvoidanceController(ego,[],[],cfg,[]), ...
                'collisionAvoidanceController:missingJointPrediction');
        end
        function controllerEnclosesTheSameDecisionDomainItAdmits(testCase)
            [~,context,output]=localFixture();cfg=context.cfg;
            cfg.controller.horizonSteps=2;cfg.controller.jointNonlinearPrediction=true;
            cfg.solver.timeLimitSeconds=30;cfg.rti.trustRegion.errorBudget(:)=1e-8;
            output.targetEstimate=[];output.controllerErrorBound.generator(:)=0;
            output.controllerErrorBound.bounds(:)=0;
            output.controllerErrorBound.jointPrediction.parameterRadius(:)=0;
            output.controllerErrorBound.jointPrediction.noiseRadius(:)=0;
            [~,~,prediction]=collisionAvoidanceController(output,[],[],cfg,[]);
            testCase.verifyTrue(prediction.model.jointPredictionTube.realArithmeticEnclosure);
            testCase.verifyFalse(prediction.metadata.robustNonlinearSafetyCertified);
            testCase.verifyGreaterThan(max(prediction.model.jointPredictionTube.physicalRadius,[],'all'),0);
        end
    end
end
function localContains(testCase,image,value)
    [lo,hi]=bounds(image);
    testCase.verifyGreaterThanOrEqual(value,lo-2e-10);
    testCase.verifyLessThanOrEqual(value,hi+2e-10);
end
function next=localFlow(x,u,cfg,h)
    f=@(z)nonlinearBicycleModel.derivative(z,u,cfg);
    a=f(x);b=f(x+h*a/2);c=f(x+h*b/2);d=f(x+h*c);next=x+h*(a+2*b+2*c+d)/6;
end
function [center,context,output]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8));
    estimator=estimatorConfigurationFromController(estimatorControllerIntegrationConfig(),cfg);
    design=synthesizeNrmmObserverGains(estimator.observer);
    reference=nonlinearBicycleModel.cruise(cfg,0);x=reference.state;u=reference.input;
    options=struct('initialTime',0,'egoInitialPosition',x(1:2),'egoInitialYaw',x(3), ...
        'egoInitialBodyVelocity',x(4:5),'targetInitialState',[20;0;8;0;0;0], ...
        'inertialVelocityInitialization',struct('value',x(4:5),'radius',.05));
    runtime=onlineNrmmTrackingRuntime('initialize',estimator.observer,options,design);
    frame=struct('time',0,'xGps',x(1),'yGps',x(2), ...
        'longitudinalAcceleration',0,'lateralAcceleration',0,'yawRateMeasured',0, ...
        'radarDetectionAvailable',false,'radarRelativePosition',[NaN;NaN], ...
        'heldSteeringAngle',u(1),'heldBrakingRatio',u(2));
    output=onlineNrmmTrackingRuntime('output',runtime,frame);contract=output.controllerErrorBound.jointPrediction;
    center=[x;contract.point;zeros(4,1);u;contract.positionVelocity.point];
    context=struct('cfg',cfg,'contract',contract,'reference',x,'input',u, ...
        'gain',zeros(2,6),'duration',cfg.controller.sampleTime,'first',false);
end
