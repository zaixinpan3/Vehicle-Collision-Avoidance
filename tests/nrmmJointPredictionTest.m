classdef nrmmJointPredictionTest < matlab.unittest.TestCase
    properties (SetAccess=private)
        fixture
    end
    properties (TestParameter)
        modelAided=struct('kinematic',false,'forceBalance',true)
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'estimator')));
            testCase.fixture=localFixture();
        end
    end
    methods (Test)
        function predictionExecutesTheSameEgoObserverAsTheRuntime(testCase,modelAided)
            f=testCase.fixture;frame=f.frame;
            frame=localHeldInput(frame,f.input,modelAided);
            output=onlineNrmmTrackingRuntime('output',f.runtime,frame);
            testCase.assertTrue(output.controllerErrorBound.available);
            contract=output.controllerErrorBound.jointPrediction;
            predicted=nrmmJointPrediction('observerStep',contract.point,contract.currentMeasurement,contract);
            actual=onlineNrmmTrackingRuntime('sample',f.runtime,frame);
            point=[actual.positionEstimate;actual.yawEstimate;actual.bodyVelocityEstimate];
            testCase.verifyEqual(predicted,point,AbsTol=1e-12);
        end
        function futurePositionSamplesExecuteTheActualVelocityFilter(testCase)
            f=testCase.fixture;runtime=f.runtime;x=f.state;u=[.02;-.1];
            frame=localHeldInput(f.frame,f.input,true);
            output=onlineNrmmTrackingRuntime('output',runtime,frame);
            contract=output.controllerErrorBound.jointPrediction;
            testCase.verifyEqual(contract.noiseRadius(3:4),zeros(2,1));
            context=struct('cfg',f.cfg,'contract',contract,'reference',f.state, ...
                'input',u,'gain',zeros(2,6),'duration',f.cfg.controller.sampleTime,'first',true);
            count=round(context.duration/contract.samplePeriod);
            point=nrmmJointPrediction('hold', ...
                [x;contract.point;zeros(4,1);f.input;contract.positionVelocity.point],zeros(7,count),context);
            for k=1:count
                if k>1
                    derivative=nonlinearBicycleModel.derivative(x,u,f.cfg);
                    frame.time=runtime.currentTime;frame.xGps=x(1);frame.yGps=x(2);
                    frame.longitudinalAcceleration=derivative(4)-x(6)*x(5);
                    frame.lateralAcceleration=derivative(5)+x(6)*x(4);
                    frame.yawRateMeasured=x(6);frame=localHeldInput(frame,u,true);
                end
                runtime=onlineNrmmTrackingRuntime('sample',runtime,frame);
                dt=contract.samplePeriod;flow=@(state)nonlinearBicycleModel.derivative(state,u,f.cfg);
                a=flow(x);b=flow(x+dt*a/2);c=flow(x+dt*b/2);d=flow(x+dt*c);
                x=x+dt*(a+2*b+2*c+d)/6;
            end
            testCase.verifyEqual(point(7:11),[runtime.positionEstimate;runtime.yawEstimate;runtime.bodyVelocityEstimate],AbsTol=1e-11);
            testCase.verifyEqual(point(18:21),runtime.positionVelocity.point,AbsTol=1e-11);
        end
        function currentInputIsKnownAndPersistentMismatchIsNotReinjected(testCase)
            f=testCase.fixture;[tube,contract]=localTube(f,6,true);
            testCase.verifyEqual(tube.firstInputRadius,zeros(2,1),AbsTol=1e-12);
            testCase.verifyEqual(tube.finalJointGenerator(12:15,tube.persistentParameterColumns), ...
                diag(contract.parameterRadius),AbsTol=1e-12);
            testCase.verifyEqual(sum(abs(tube.finalJointGenerator(12:15,:)),2),contract.parameterRadius,AbsTol=1e-12);
            testCase.verifyTrue(tube.futureObserverDynamicsIncluded);
            testCase.verifyFalse(tube.futurePrimitiveBudgetsAssumed);
            testCase.verifyFalse(tube.nonlinearCertificate);
        end
        function futureNoiseChangesBothEstimationAndPhysicalPrediction(testCase)
            [tube,~]=localTube(testCase.fixture,6,true);
            testCase.verifyGreaterThan(norm(tube.noiseMap{end},'fro'),0);
            testCase.verifyGreaterThan(norm(tube.inputRadius(:,end)),0);
            testCase.verifyGreaterThan(norm(tube.observerErrorRadius(:,end)),0);
            testCase.verifyNotEqual(tube.observerErrorRadius(:,end),tube.observerErrorRadius(:,1));
        end
        function controllerUsesTheJointTubeAndReservesFutureFeedbackAuthority(testCase)
            f=testCase.fixture;f.cfg.controller.horizonSteps=6;f.cfg.controller.jointAffinePrediction=true;
            output=onlineNrmmTrackingRuntime('output',f.runtime,f.frame);
            output.targetEstimate=[];
            [~,~,prediction]=collisionAvoidanceController(output,[],[],f.cfg,[]);
            tube=prediction.model.jointPredictionTube;
            testCase.verifyTrue(prediction.model.uncertaintyPrediction.futureObserverDynamicsIncluded);
            testCase.verifyFalse(prediction.model.uncertaintyPrediction.futurePrimitiveBudgetsAssumed);
            testCase.verifyGreaterThan(norm(tube.inputRadius(:,end)),0);
            testCase.verifyEqual(tube.firstInputRadius,zeros(2,1),AbsTol=1e-12);
        end
        function affinePredictionIsAnExplicitExperimentRatherThanANonlinearCertificate(testCase)
            testCase.verifyFalse(testCase.fixture.cfg.controller.jointAffinePrediction);
            [tube,~]=localTube(testCase.fixture,4,true);
            testCase.verifyFalse(tube.nonlinearCertificate);
        end
        function chainedDerivativesAgreeWithIndependentWholeHoldDifferences(testCase,modelAided)
            f=testCase.fixture;frame=localHeldInput(f.frame,f.input,modelAided);
            output=onlineNrmmTrackingRuntime('output',f.runtime,frame);
            contract=output.controllerErrorBound.jointPrediction;
            context=struct('cfg',f.cfg,'contract',contract,'reference',f.state, ...
                'input',f.input,'gain',[0,-.01,-.1,0,-.02,-.05;-.02,0,0,-.2,0,0], ...
                'duration',f.cfg.controller.sampleTime,'first',false);
            center=[f.state;contract.point;zeros(4,1);f.input;contract.positionVelocity.point];
            fast=nrmmJointPrediction('linearize',center,context);
            independent=nrmmJointPrediction('finiteDifference',center,context);
            testCase.verifyEqual(fast.point,independent.point,AbsTol=1e-12);
            testCase.verifyLessThan(norm(fast.matrix-independent.matrix,'fro')/norm(independent.matrix,'fro'),.005);
            testCase.verifyLessThan(norm(fast.noise-independent.noise,'fro')/norm(independent.noise,'fro'),.005);
        end
        function stoppingTheNominalDoesNotEraseTheStillMovingBranch(testCase)
            cfg=testCase.fixture.cfg;point=zeros(25,1);point(25)=-.2;
            point(4)=.19*cfg.model.brakeHoldSpeed;
            generator=zeros(25,1);generator(4)=.05*cfg.model.brakeHoldSpeed;
            result=nrmmJointPrediction('resetEnvelope',point,generator,cfg);
            testCase.verifyTrue(result.mixed);
            for coefficient=linspace(-1,1,21)
                x=nonlinearBicycleModel.sampleEndState(point(1:6)+generator(1:6)*coefficient,point(24:25),cfg);
                testCase.verifyLessThanOrEqual(abs(x-result.point(1:6)),sum(abs(result.generator(1:6,:)),2)+1e-14);
            end
            testCase.verifyGreaterThan(sum(abs(result.generator(4,:))),0);
        end
        function onlyAUniformStopGuardCanEraseVelocityUncertainty(testCase)
            cfg=testCase.fixture.cfg;point=zeros(25,1);point(25)=-.2;
            generator=zeros(25,1);generator(4)=.01*cfg.model.brakeHoldSpeed;
            result=nrmmJointPrediction('resetEnvelope',point,generator,cfg);
            testCase.verifyFalse(result.mixed);
            testCase.verifyEqual(result.generator(4:6,:),zeros(3,1));
        end
        function mixedStopEnclosesTheJointStateWithoutChangingItsObserverOrParameters(testCase)
            cfg=testCase.fixture.cfg;point=zeros(25,1);point(25)=-.2;
            point(4:5)=[.19*cfg.model.brakeHoldSpeed;.02];
            generator=zeros(25,2);
            generator([4,10,12],1)=[.05*cfg.model.brakeHoldSpeed;.1;.05];
            generator([5,11,13],2)=[.03;.05;.05];
            result=nrmmJointPrediction('resetEnvelope',point,generator,cfg);
            for coefficient=[-1,-1;1,1].'
                actual=point+generator*coefficient;
                actual(1:6)=nonlinearBicycleModel.sampleEndState(actual(1:6),actual(24:25),cfg);
                count=size(result.generator,2);
                [~,~,flag]=linprog(zeros(count,1),[],[],result.generator,actual-result.point, ...
                    -ones(count,1),ones(count,1),optimoptions('linprog','Display','none'));
                testCase.verifyGreaterThan(flag,0);
            end
        end
        function physicalParameterTangentsMatchTheNonlinearModelAcrossItsBranches(testCase)
            cfg=testCase.fixture.cfg;range=cfg.model.kinematicSpeedRange;
            for speed=[range(1)/2,mean(range),8]
                x=[0;0;.2;speed;.03;.01];
                for ratio=[-.7,0,.1,-1]
                    u=[.03;ratio];analytic=nonlinearBicycleModel.parameterJacobian(x,u,cfg);
                    numerical=zeros(6,4);step=1e-5;
                    for j=1:4
                        plus=cfg;minus=cfg;
                        if j<=2
                            plus.tire.corneringStiffness(j)=cfg.tire.corneringStiffness(j)*(1+step);
                            minus.tire.corneringStiffness(j)=cfg.tire.corneringStiffness(j)*(1-step);
                        else
                            plus.tire.frictionCoefficient(j-2)=cfg.tire.frictionCoefficient(j-2)*(1+step);
                            minus.tire.frictionCoefficient(j-2)=cfg.tire.frictionCoefficient(j-2)*(1-step);
                        end
                        numerical(:,j)=(nonlinearBicycleModel.derivative(x,u,plus)-nonlinearBicycleModel.derivative(x,u,minus))/(2*step);
                    end
                    testCase.verifyEqual(analytic,numerical,AbsTol=2e-8,RelTol=1e-6);
                end
            end
        end
        function staleObserverSnapshotsCannotSilentlyReplaceTheCurrentInformation(testCase)
            f=testCase.fixture;output=onlineNrmmTrackingRuntime('output',f.runtime,f.frame);
            f.cfg.controller.jointAffinePrediction=true;
            output.controllerErrorBound.jointPrediction.time=-f.runtime.samplePeriod;
            testCase.verifyError(@()readControllerInputs(output,[],[],f.cfg), ...
                'collisionAvoidanceController:invalidJointPrediction');
        end
        function boundaryImuUsesThePreviousInputWhileThePlantUsesTheNewCommand(testCase)
            f=testCase.fixture;output=onlineNrmmTrackingRuntime('output',f.runtime,f.frame);
            contract=output.controllerErrorBound.jointPrediction;
            context=struct('cfg',f.cfg,'contract',contract,'reference',f.state, ...
                'input',f.input,'gain',zeros(2,6),'duration',f.cfg.controller.sampleTime,'first',false);
            first=[f.state;contract.point;zeros(4,1);0;-.2;contract.positionVelocity.point];second=first;second(17)=.1;
            noise=zeros(7,round(context.duration/contract.samplePeriod));
            first=nrmmJointPrediction('hold',first,noise,context);
            second=nrmmJointPrediction('hold',second,noise,context);
            testCase.verifyEqual(first(1:6),second(1:6),AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(first(7:11)-second(7:11)),0);
            testCase.verifyEqual(first(16:17),context.input,AbsTol=1e-12);
        end
    end
end

function frame=localHeldInput(frame,input,enabled)
    if enabled
        frame.heldSteeringAngle=input(1);frame.heldBrakingRatio=input(2);
    end
end

function f=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'solver',struct('timeLimitSeconds',30)));
    estimator=estimatorConfigurationFromController(estimatorControllerIntegrationConfig(),cfg);
    design=synthesizeNrmmObserverGains(estimator.observer);
    reference=nonlinearBicycleModel.cruise(cfg,0);x=reference.state;u=reference.input;
    options=struct('initialTime',0,'egoInitialPosition',x(1:2), ...
        'egoInitialYaw',x(3),'egoInitialBodyVelocity',x(4:5), ...
        'targetInitialState',[20;0;8;0;0;0], ...
        'inertialVelocityInitialization',struct('value',x(4:5),'radius',.05));
    runtime=onlineNrmmTrackingRuntime('initialize',estimator.observer,options,design);
    frame=struct('time',0,'xGps',x(1),'yGps',x(2),'vxGps',x(4),'vyGps',x(5), ...
        'longitudinalAcceleration',0,'lateralAcceleration',0,'yawRateMeasured',0, ...
        'radarDetectionAvailable',false,'radarRelativePosition',[NaN;NaN]);
    f=struct('cfg',cfg,'runtime',runtime,'frame',frame,'input',u,'state',x);
end

function [tube,contract]=localTube(f,count,feedback)
    output=onlineNrmmTrackingRuntime('output',f.runtime,f.frame);
    contract=output.controllerErrorBound.jointPrediction;
    anchor=struct('states',repmat(f.state,1,count+1),'inputs',repmat(f.input,1,count));
    for k=1:count
        anchor.states(:,k+1)=nonlinearBicycleModel.sample(anchor.states(:,k),anchor.inputs(:,k),f.cfg);
    end
    tube=nrmmJointPrediction('propagate',anchor,f.cfg,output.controllerErrorBound.generator, ...
        contract,repmat(f.cfg.controller.sampleTime,1,count),struct('feedback',feedback));
end
