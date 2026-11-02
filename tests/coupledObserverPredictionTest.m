classdef coupledObserverPredictionTest < matlab.unittest.TestCase
    properties (SetAccess=private)
        fixture
    end
    properties (TestParameter)
        coordinate={1,3,5,12,16,22,24,25,26,27,28,30,32}
        retainedValue=struct('lowerSpeed',[5;1],'upperSpeed',[7;-1])
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for folder=["controller","config","estimator"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,folder)));
            end
            testCase.fixture=localFixture();
        end
    end
    methods (Test)
        function predictedJointObserversMatchTheSensorClockRuntime(testCase)
            f=testCase.fixture;u=[.02;-.1];context=localContext(f,u);
            prediction=nrmmJointPrediction('coupledLinearize',f.root.center,context);
            [runtime,contract]=localReplay(f,context,prediction.branch);
            actual=[runtime.positionEstimate;runtime.yawEstimate;runtime.bodyVelocityEstimate; ...
                runtime.targetState;runtime.targetOutputPredictor];
            testCase.verifyEqual(prediction.point([7:11,28:35]),actual,AbsTol=2e-11);
            testCase.verifyEqual(prediction.point(18:21),runtime.positionVelocity.point,AbsTol=2e-11);
            testCase.verifyEqual(contract.measurementHistory.time,prediction.branch.sampleTimes,AbsTol=1e-12);
        end
        function futureRadarUpdatesChangeFutureEgoCommandsOnTheSameBranch(testCase)
            f=testCase.fixture;anchor=localAnchor(f,4);
            f.contract.noiseRadius(:)=0;f.contract.parameterRadius(:)=0;
            error=localZeroError(f.error);
            tube=nrmmJointPrediction('coupledPropagate',anchor,f.cfg,zeros(6,0), ...
                f.contract,repmat(.05,1,4),f.target,error);
            testCase.verifyEqual(tube.firstInputRadius,zeros(2,1),AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(tube.inputRadius(:,2:end),'fro'),0);
            testCase.verifyGreaterThan(norm(tube.physicalRadius(:,5:end),'fro'),0);
            testCase.verifyTrue(tube.futureTargetObserverDynamicsIncluded);
            testCase.verifyFalse(tube.nonlinearCertificate);
        end
        function noAdditionalAuthorityDisablesTargetResponseWithoutInventingCertainty(testCase)
            f=testCase.fixture;anchor=localAnchor(f,4);
            f.contract.noiseRadius(:)=0;f.contract.parameterRadius(:)=0;
            error=localZeroError(f.error);
            options=struct('feedbackInputRadius',zeros(2,4));
            tube=nrmmJointPrediction('coupledPropagate',anchor,f.cfg,zeros(6,0), ...
                f.contract,repmat(.05,1,4),f.target,error,[],options);
            testCase.verifyEqual(tube.inputRadius,zeros(2,4),AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(tube.targetObserverErrorRadius(:,end)),0);
            testCase.verifyGreaterThan(norm(tube.feedbackGain(:)),0);
        end
        function retainedParameterCorrelationIsNotReplacedByIndependentRadii(testCase,retainedValue)
            f=testCase.fixture;error=f.error;
            error.family=struct('available',true,'epochTime',0, ...
                'lower',[5,7;1,-1;0,0;.1,.1],'upper',[5,7;1,-1;0,0;.1,.1]);
            root=nrmmJointPrediction('coupledRoot',f.state,f.generator,f.contract,f.target,error);
            testCase.verifyTrue(root.targetFamilyRetained);
            map=root.generator([25,26],:);point=root.center([25,26]);
            [~,~,flag]=linprog(zeros(size(map,2),1),[],[],map,retainedValue-point, ...
                -ones(size(map,2),1),ones(size(map,2),1),optimoptions('linprog','Display','none'));
            testCase.verifyGreaterThan(flag,0);
        end
        function jointTangentsAgreeWithIndependentPointPerturbations(testCase,coordinate)
            f=testCase.fixture;context=localContext(f,[.02;-.1]);
            prediction=nrmmJointPrediction('coupledLinearize',f.root.center,context);
            step=1e-5;delta=zeros(35,1);delta(coordinate)=step;
            plus=nrmmJointPrediction('coupledLinearize',f.root.center+delta,context);
            minus=nrmmJointPrediction('coupledLinearize',f.root.center-delta,context);
            derivative=(plus.point-minus.point)/(2*step);
            testCase.verifyLessThan(norm(derivative-prediction.matrix(:,coordinate))/max(1,norm(derivative)),.005,sprintf('Coordinate %d',coordinate));
        end
        function sharedTranslationCancelsBeforeRelativeSupportIsTaken(testCase)
            f=testCase.fixture;g=zeros(6,1);g(1)=10;
            root=nrmmJointPrediction('coupledRoot',f.state,g,f.contract,f.target,f.error);
            testCase.verifyEqual(root.generator(1:2,1)-root.generator(22:23,1),zeros(2,1),AbsTol=1e-12);
            tube=nrmmJointPrediction('coupledPropagate',localAnchor(f,2),f.cfg,g, ...
                f.contract,[.05,.05],f.target,f.error,root);
            testCase.verifyEqual(tube.relativeGenerator{1}(1:2,1),zeros(2,1),AbsTol=1e-12);
            testCase.verifyEqual(tube.finalJointGenerator(12:15,root.parameterColumns), ...
                diag(f.contract.parameterRadius),AbsTol=1e-12);
            testCase.verifyEqual(sum(abs(tube.finalJointGenerator(26:27,:)),2), ...
                [f.error.accelerationRadius;f.error.sideslipRadius],AbsTol=1e-12);
        end
        function jointPoseMarginCoversSharedTranslationAndRotatingCorners(testCase)
            [reserve,deficit]=localPoseSamples();
            testCase.verifyGreaterThanOrEqual(reserve-deficit,-1e-12);
        end
        function completeSensorBlockConditionsTheAppliedCommandSuccessor(testCase)
            f=testCase.fixture;context=localContext(f,[.02;-.1]);
            branch=nrmmJointPrediction('coupledLinearize',f.root.center,context);
            [~,contract]=localReplay(f,context,branch.branch);
            anchor=localAnchor(f,1);anchor.inputs=context.input;
            tube=nrmmJointPrediction('coupledPropagate',anchor,f.cfg,f.generator,f.contract,.05,f.target,f.error);
            target=predictiveSafetyGeometry.predictTarget(f.target,.05);
            current=nrmmJointPrediction('coupledRoot',branch.point(1:6),f.generator,contract,target,f.error);
            result=nrmmJointPrediction('conditionCoupled',tube.firstSuccessor,current,contract);
            testCase.verifyTrue(result.diagnostics.accepted,result.diagnostics.reason);
            testCase.verifyTrue(result.diagnostics.usedAllSamples);
            before=sum(abs(tube.firstSuccessor.generator(22:23,:)-tube.firstSuccessor.generator(1:2,:)),2);
            after=sum(abs(result.root.generator(22:23,:)-result.root.generator(1:2,:)),2);
            testCase.verifyLessThan(norm(after),norm(before));
            testCase.verifyEqual(result.diagnostics.sampleCount,round(.05/f.contract.samplePeriod));
            testCase.verifyEqual(tube.firstSuccessor.appliedInput,context.input);
            changed=contract;
            changed.measurementHistory.value(6,2)=changed.measurementHistory.value(6,2)+100;
            rejected=nrmmJointPrediction('conditionCoupled',tube.firstSuccessor,current,changed);
            testCase.verifyFalse(rejected.diagnostics.accepted);
            testCase.verifyEqual(rejected.diagnostics.reason,"observerHistoryInconsistent");
            changed=contract;changed.currentMeasurement.heldInput(1)=.1;
            rejected=nrmmJointPrediction('conditionCoupled',tube.firstSuccessor,current,changed);
            testCase.verifyEqual(rejected.diagnostics.reason,"timeOrAppliedInputChanged");
        end
        function controllerRetainsItsIssuedCommandForTheJointSuccessor(testCase)
            f=testCase.fixture;cfg=f.cfg;cfg.controller.horizonSteps=6;cfg.solver.timeLimitSeconds=60;
            output=onlineNrmmTrackingRuntime('output',f.runtime,f.frame);
            [command,~,prediction,state]=collisionAvoidanceController(output,[],[],cfg,[]);
            testCase.verifyTrue(prediction.metadata.futureTargetObserverDynamicsIncluded);
            testCase.verifyTrue(prediction.metadata.targetFeedbackIncluded);
            testCase.verifyEqual(state.coupledJointPrediction.appliedInput,command.actuatorInput);
            testCase.verifyFalse(prediction.metadata.robustNonlinearSafetyCertified);
            testCase.verifyEqual(state.coupledJointPrediction.time,output.stateTime+cfg.controller.sampleTime,AbsTol=1e-12);
        end
        function rotatingTheBodyFrameDoesNotInventATargetVelocityChange(testCase)
            f=testCase.fixture;context=localContext(f,f.input);context.first=false;
            context.referenceYaw=f.root.center(9);context.gain=zeros(2,6);
            context.gain(1,5)=.1;context.gain(2,4)=.1;
            center=f.root.center;context.reference=[zeros(3,1);center(10:11)-center(30:31);center(6)];
            first=nrmmJointPrediction('coupledLinearize',center,context);
            angle=.01;rotation=[cos(angle),sin(angle);-sin(angle),cos(angle)];
            center(9)=center(9)+angle;center(30:31)=rotation*center(30:31);
            second=nrmmJointPrediction('coupledLinearize',center,context);
            testCase.verifyEqual(first.point(48:49),second.point(48:49),AbsTol=1e-12);
        end
        function anUncertainStopRetainsTheStillMovingBranch(testCase)
            f=testCase.fixture;f.state(4)=.05;f.state(5:6)=0;f.input=[0;-.1];
            f.contract.point=f.state(1:5);f.contract.currentMeasurement.heldInput=f.input;
            g=zeros(6,1);g(4)=.1;
            tube=nrmmJointPrediction('coupledPropagate',localAnchor(f,1),f.cfg,g,f.contract,.05,f.target,f.error);
            testCase.verifyTrue(tube.mixedStopBranches(1));
            testCase.verifyGreaterThan(tube.physicalRadius(4,end),0);
            result=nrmmJointPrediction('conditionCoupled',tube.firstSuccessor,tube.root,f.contract);
            testCase.verifyEqual(result.diagnostics.reason,"mixedStopMeasurementMapUnavailable");
        end
        function missingIntermediateMeasurementsDoNotMasqueradeAsAFullBlock(testCase)
            f=testCase.fixture;context=localContext(f,f.input);
            branch=nrmmJointPrediction('coupledLinearize',f.root.center,context);
            [~,contract]=localReplay(f,context,branch.branch);
            tube=nrmmJointPrediction('coupledPropagate',localAnchor(f,1),f.cfg, ...
                f.generator,f.contract,.05,f.target,f.error);
            contract.measurementHistory.time(2)=[];contract.measurementHistory.value(:,2)=[];
            result=nrmmJointPrediction('conditionCoupled',tube.firstSuccessor,f.root,contract);
            testCase.verifyFalse(result.diagnostics.accepted);
            testCase.verifyEqual(result.diagnostics.reason,"incompleteSensorBlock");
        end
    end
end

function f=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8));
    estimator=estimatorConfigurationFromController(estimatorControllerIntegrationConfig(),cfg);
    design=synthesizeNrmmObserverGains(estimator.observer);
    reference=nonlinearBicycleModel.cruise(cfg,0);x=reference.state;u=reference.input;
    target=[20;2;.1;6;.2;.01;1.5;2.4;.95;0;0];
    e=[cos(target(3)+target(6));sin(target(3)+target(6))];
    acceleration=target(5)*e+sin(target(6))/target(7)*target(4)^2*[-e(2);e(1)];
    options=struct('initialTime',0,'egoInitialPosition',x(1:2), ...
        'egoInitialYaw',x(3),'egoInitialBodyVelocity',x(4:5), ...
        'targetInitialState',[target(1:2);target(4)*e;acceleration], ...
        'inertialVelocityInitialization',struct('value',x(4:5),'radius',.05));
    runtime=onlineNrmmTrackingRuntime('initialize',estimator.observer,options,design);
    frame=struct('time',0,'xGps',x(1),'yGps',x(2), ...
        'longitudinalAcceleration',0,'lateralAcceleration',0,'yawRateMeasured',0, ...
        'radarDetectionAvailable',true,'radarRelativePosition',target(1:2), ...
        'heldSteeringAngle',u(1),'heldBrakingRatio',u(2));
    output=onlineNrmmTrackingRuntime('output',runtime,frame);
    contract=output.controllerErrorBound.jointPrediction;generator=.001*eye(6);
    error=struct('positionRadius',.1,'courseRadius',.01,'speedRadius',.1, ...
        'accelerationRadius',.1,'sideslipRadius',.01);
    root=nrmmJointPrediction('coupledRoot',x,generator,contract,target,error);
    f=struct('cfg',cfg,'runtime',runtime,'frame',frame,'input',u,'state',x, ...
        'target',target,'error',error,'contract',contract,'generator',generator,'root',root);
end

function context=localContext(f,u)
    context=struct('cfg',f.cfg,'contract',f.contract,'reference',zeros(6,1), ...
        'input',u,'gain',zeros(2,6),'duration',.05,'first',true,'targetRearAxleDistance',f.target(7));
end

function anchor=localAnchor(f,count)
    anchor=struct('states',repmat(f.state,1,count+1),'inputs',repmat(f.input,1,count));
    for k=1:count,anchor.states(:,k+1)=nonlinearBicycleModel.sample(anchor.states(:,k),anchor.inputs(:,k),f.cfg);end
end

function [runtime,contract]=localReplay(f,context,branch)
    runtime=f.runtime;frame=f.frame;
    for k=1:numel(branch.sampleTimes)
        measurement=branch.measurementCenter(:,k);frame.time=branch.sampleTimes(k);
        frame.xGps=measurement(1);frame.yGps=measurement(2);
        frame.longitudinalAcceleration=measurement(3);frame.lateralAcceleration=measurement(4);
        frame.yawRateMeasured=measurement(5);frame.radarRelativePosition=measurement(6:7);
        if k>1,frame.heldSteeringAngle=context.input(1);frame.heldBrakingRatio=context.input(2);end
        if k<numel(branch.sampleTimes)
            runtime=onlineNrmmTrackingRuntime('sample',runtime,frame);
        else
            output=onlineNrmmTrackingRuntime('output',runtime,frame);
            contract=output.controllerErrorBound.jointPrediction;
        end
    end
end

function error=localZeroError(error)
    fields=fieldnames(error);
    for k=1:numel(fields),error.(fields{k})=0;end
end

function [reserve,deficit]=localPoseSamples()
    pe=[0;0;.4];pt=[10;2;-.2];shape=[2.4;.95;.2;0];normal=[cos(.3);sin(.3)];
    g=zeros(6,3);g([1,4],1)=.2;g([3,6],2)=.05;g(2,3)=.04;
    reserve=predictiveSafetyGeometry.jointPoseSupport(normal,pe,shape,pt,shape,g);
    vertices=shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1];
    direction=@(pose) normal'*(pose(1:2)+[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))]*vertices);
    baseE=direction(pe);baseT=direction(pt);deficit=0;
    for a=[-1,0,1]
        for b=[-1,0,1]
            for c=[-1,0,1]
                delta=g*[a;b;c];de=direction(pe+delta(1:3))-baseE;dt=direction(pt+delta(4:6))-baseT;
                deficit=max(deficit,max(dt-de',[],'all'));
            end
        end
    end
end
