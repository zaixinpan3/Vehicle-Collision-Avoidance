classdef retainedTubeControllerTest < matlab.unittest.TestCase
    properties
        fixture
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for name=["controller","config","estimator"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,name)));
            end
            testCase.fixture=localFixture();
        end
    end
    methods (Test)
        function aWholePolicyBoxContainsNonlinearObserverFeedbackHistories(testCase)
            f=testCase.fixture;root=nrmmJointPrediction('rootSet',f.x,f.generator,f.contract);
            context=struct('cfg',f.cfg,'contract',f.contract,'reference',f.x,'input',f.u, ...
                'gain',zeros(2,6),'duration',f.cfg.controller.sampleTime,'first',true);
            policy=struct('context',context,'lower',[-.001;-.005],'upper',[.001;.005]);
            tube=nrmmJointPrediction('policyTube',root,policy);
            random=RandStream('mt19937ar','Seed',67);
            for k=1:6
                point=root.center+root.generator*(2*rand(random,size(root.generator,2),1)-1);
                sample=context;sample.input=sample.input+policy.upper.*(2*rand(random,2,1)-1);
                noise=f.contract.noiseRadius.*(2*rand(random,7,4)-1);
                next=nrmmJointPrediction('hold',point,noise,sample);
                [lo,hi]=bounds(tube.sets{2});
                testCase.verifyGreaterThanOrEqual(next(1:21),lo-1e-10);
                testCase.verifyLessThanOrEqual(next(1:21),hi+1e-10);
            end
        end
        function aStationaryTargetHasAWholeSetTerminalCertificate(testCase)
            f=testCase.fixture;context=terminalSafeSet.context(f.model);
            state=nrmmSet(f.x,f.generator);
            result=terminalSafeSet.certifySet(context,state,f.model.target,[],0,100);
            testCase.verifyTrue(result.valid);
            testCase.verifyEqual(result.endReason,"separatedStationaryFamilies");
            testCase.verifyGreaterThan(result.minimumGap,0);
            testCase.verifyFalse(result.floatingPointCertified);
        end
        function runningOutOfTerminalChecksIsNeverACertificate(testCase)
            f=testCase.fixture;context=terminalSafeSet.context(f.model);
            result=terminalSafeSet.certifySet(context,nrmmSet(f.x,f.generator),f.model.target,[],0,0);
            testCase.verifyFalse(result.valid);
            testCase.verifyEqual(result.endReason,"workLimit");
        end
        function firstAdmissionCreatesAFeasibleNonlinearCertificate(testCase)
            f=testCase.fixture;
            [solution,search]=solvePredictiveControl('probe',struct('operation','retained', ...
                'model',f.model,'previousState',[],'anchor',f.anchor));
            testCase.verifyTrue(solution.retainedCertificate.valid);
            testCase.verifyTrue(search.retainedCandidateFeasible);
            testCase.verifyEqual(solution.safety,0,AbsTol=0);
            testCase.verifyGreaterThan(max(sum(abs(solution.retainedCertificate.tube.sets{1}.generator),2)),0);
            policy=solution.retainedCertificate.policies(1);
            testCase.verifyGreaterThan(max(policy.upper-policy.lower),0);
        end
        function aRealObserverUpdateRetainsZeroSlackInTheNextConvexProblem(testCase)
            f=testCase.fixture;
            [first,~]=solvePredictiveControl('probe',struct('operation','retained', ...
                'model',f.model,'previousState',[],'anchor',f.anchor));
            [runtime,x,output]=localAdvance(f.runtime,f.x,f.u,first.inputs(:,1),f.cfg,0);
            cfg=f.cfg;cfg.controller.retainedTube=true;
            [ego,~,~,observation]=readControllerInputs(output,[],[],cfg);
            model=f.model;model.initialState=ego.modelState;model.stateTime=output.stateTime;
            model.previousInput=first.inputs(:,1);model.sampleIndex=1;model.targetOnset=false;
            model.uncertainty=predictiveSafetyGeometry.observerUncertainty(ego,observation,model.target);
            model.uncertainty.target=f.model.uncertainty.target;
            previous=struct('retainedCertificate',first.retainedCertificate);
            [second,search]=solvePredictiveControl('probe',struct('operation','retained', ...
                'model',model,'previousState',previous,'anchor',f.anchor));
            testCase.verifyTrue(search.retainedCandidateFeasible);
            testCase.verifyTrue(search.posterior.subsetByConstruction);
            testCase.verifyEqual(second.safety,0,AbsTol=0);
            testCase.verifyEqual(second.retainedCertificate.horizonSteps,first.retainedCertificate.horizonSteps);
            testCase.verifyEqual(second.retainedCertificate.target,first.retainedCertificate.target);
            testCase.verifyEqual(runtime.currentTime,model.stateTime,AbsTol=1e-12);
            testCase.verifyGreaterThan(x(1),f.x(1));
        end
        function repeatedUpdatesRetainZeroSlackOrAnExplicitTerminalContinuation(testCase)
            f=testCase.fixture;model=f.model;previous=[];runtime=f.runtime;x=f.x;held=f.u;
            for k=0:7
                [solution,search]=solvePredictiveControl('probe',struct('operation','retained', ...
                    'model',model,'previousState',previous,'anchor',f.anchor));
                testCase.verifyTrue(search.retainedCandidateFeasible);
                testCase.verifyEqual(solution.safety,0,AbsTol=0);
                [lo,hi]=bounds(solution.retainedCertificate.tube.sets{1}(1:6));
                testCase.verifyGreaterThanOrEqual(x,lo-1e-10);
                testCase.verifyLessThanOrEqual(x,hi+1e-10);
                if solution.terminalControlRequired
                    % This is a safe mathematical continuation, NOT a
                    % successful driving-scenario outcome. The public
                    % controller stops the experiment before takeover.
                    testCase.verifyTrue(solution.retainedCertificate.terminal.valid);
                    testCase.verifyNotEmpty(search.refinementFailure);
                    model.sampleIndex=k+1;
                    saved=struct('retainedCertificate',solution.retainedCertificate);
                    [tail,nextSearch]=solvePredictiveControl('probe',struct('operation','retained', ...
                        'model',model,'previousState',saved,'anchor',f.anchor));
                    testCase.verifyTrue(nextSearch.retainedCandidateFeasible);
                    testCase.verifyTrue(tail.terminalControlRequired);
                    testCase.verifyEqual(tail.safety,0,AbsTol=0);
                    testCase.verifyEqual(tail.retainedCertificate.terminal,solution.retainedCertificate.terminal);
                    testCase.verifyEqual(tail.retainedCertificate.terminalElapsed,f.cfg.controller.sampleTime);
                    fprintf('Retained fixture: terminal continuation required at frame %d, %.3f s (%s).\n', ...
                        k,k*f.cfg.controller.sampleTime,search.refinementFailure);
                    return;
                end
                [runtime,x,output]=localAdvance(runtime,x,held,solution.inputs(:,1),f.cfg,k*f.cfg.controller.sampleTime);
                cfg=f.cfg;cfg.controller.retainedTube=true;
                [ego,~,~,observation]=readControllerInputs(output,[],[],cfg);
                held=solution.inputs(:,1);model.initialState=ego.modelState;model.previousInput=held;
                model.stateTime=output.stateTime;model.sampleIndex=k+1;model.targetOnset=false;
                model.uncertainty=predictiveSafetyGeometry.observerUncertainty(ego,observation,model.target);
                model.uncertainty.target=f.model.uncertainty.target;
                previous=struct('retainedCertificate',solution.retainedCertificate);
            end
            fprintf('Retained fixture: eight driving solves kept zero certified deficit.\n');
        end
        function thePublicEntryRetainsTheCertificateAndReportsItsScope(testCase)
            f=testCase.fixture;cfg=f.cfg;cfg.controller.retainedTube=true;
            target=localObservedTarget(f.output,cfg);
            lane=[-100,0;1000,0];
            [command,~,prediction,memory]=collisionAvoidanceController(f.output,target,lane,cfg,[]);
            testCase.verifyTrue(prediction.metadata.zeroSlack);
            testCase.verifyTrue(prediction.metadata.retainedCandidateFeasible);
            testCase.verifyEqual(prediction.metadata.solverAlgorithm,"retainedJointPolicyTube");
            testCase.verifyFalse(prediction.metadata.robustNonlinearSafetyCertified);
            [~,~,output]=localAdvance(f.runtime,f.x,f.u,command.actuatorInput,cfg,0);
            target=localObservedTarget(output,cfg);
            [~,~,next,~]=collisionAvoidanceController(output,target,lane,cfg,memory);
            testCase.verifyTrue(next.metadata.zeroSlack);
            testCase.verifyEqual(next.metadata.horizonSteps,prediction.metadata.horizonSteps);
        end
        function anInconsistentMeasurementDoesNotSilentlyResetTheCertificate(testCase)
            f=testCase.fixture;root=nrmmJointPrediction('rootSet',f.x,f.generator,f.contract);
            contract=f.contract;contract.point(1)=contract.point(1)+10;
            testCase.verifyError(@()nrmmJointPrediction('conditionRetained',root, ...
                f.x,f.generator,contract,f.contract),'nrmmJointPrediction:inconsistentPosterior');
        end
        function aLargerNoiseContractCannotBeSilentlyAdopted(testCase)
            f=testCase.fixture;root=nrmmJointPrediction('rootSet',f.x,f.generator,f.contract);
            contract=f.contract;contract.noiseRadius(1)=2*contract.noiseRadius(1);
            testCase.verifyError(@()nrmmJointPrediction('conditionRetained',root, ...
                f.x,f.generator,contract,f.contract),'nrmmJointPrediction:changedNoiseContract');
        end
        function aStoppedButAcceleratingTargetIsNotDeclaredPermanentlySafe(testCase)
            f=testCase.fixture;target=f.model.target;target(3)=pi;target(5)=2;
            x=f.x;x(4:6)=0;
            result=terminalSafeSet.certifySet(terminalSafeSet.context(f.model),nrmmSet(x),target,[],0,100);
            testCase.verifyFalse(result.valid);
            testCase.verifyEqual(result.endReason,"unprovedSeparation");
        end
        function missingTargetBoundsAreNotTreatedAsPerfectObservation(testCase)
            f=testCase.fixture;model=f.model;model.uncertainty.target=[];
            testCase.verifyError(@()solvePredictiveControl('probe',struct('operation','retained', ...
                'model',model,'previousState',[],'anchor',f.anchor)), ...
                'collisionAvoidanceController:missingTargetCertificate');
        end
        function anUnfinishedPerformanceSolveKeepsTheFeasiblePolicy(testCase)
            f=testCase.fixture;model=f.model;model.cfg.solver.maxIterations=1;
            [solution,search]=solvePredictiveControl('probe',struct('operation','retained', ...
                'model',model,'previousState',[],'anchor',f.anchor));
            testCase.verifyFalse(search.secondaryStageCompleted);
            testCase.verifyTrue(search.retainedCandidateFeasible);
            testCase.verifyTrue(solution.retainedCertificate.valid);
            testCase.verifyEqual(solution.safety,0,AbsTol=0);
        end
        function theTargetFamilyEnclosesTurningAccelerationAndPossibleStops(testCase)
            f=testCase.fixture;q=f.model.target;q(3:6)=[.3;2;-.8;.02];
            error=struct('positionRadius',.05,'courseRadius',.015,'speedRadius',.2, ...
                'accelerationRadius',.1,'curvatureRadius',.002,'sideslipRadius',.01);
            random=RandStream('mt19937ar','Seed',71);
            for time=[0,.25,1,3,5]
                predicted=predictiveSafetyGeometry.targetPoseSet(q,error,time);[lo,hi]=bounds(predicted);
                for k=1:12
                    z=2*rand(random,6,1)-1;sample=q;
                    sample(1:2)=q(1:2)+error.positionRadius*z(1:2);
                    curvature=sin(q(6))/q(7)+error.curvatureRadius*z(3);
                    sample(6)=asin(curvature*q(7));
                    sample(3)=q(3)+q(6)+error.courseRadius*z(4)-sample(6);
                    sample(4)=q(4)+error.speedRadius*z(5);sample(5)=q(5)+error.accelerationRadius*z(6);
                    point=predictiveSafetyGeometry.predictTarget(sample,time);
                    testCase.verifyGreaterThanOrEqual(point(1:3),lo-1e-10);
                    testCase.verifyLessThanOrEqual(point(1:3),hi+1e-10);
                end
            end
        end
        function aTerminalDomainMustIncludeItsAbsorbingStop(testCase)
            f=testCase.fixture;context=terminalSafeSet.context(f.model);context.stateLower(1)=1;
            result=terminalSafeSet.certifySet(context,nrmmSet(f.x,f.generator),f.model.target,[],0,100);
            testCase.verifyFalse(result.valid);
            testCase.verifyEqual(result.endReason,"terminalDomainExcludesStop");
        end
    end
end

function f=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('horizonSteps',2),'solver',struct('timeLimitSeconds',120)));
    estimator=estimatorConfigurationFromController(estimatorControllerIntegrationConfig(),cfg);
    design=synthesizeNrmmObserverGains(estimator.observer);
    reference=nonlinearBicycleModel.cruise(cfg,0);x=reference.state;u=reference.input;
    options=struct('initialTime',0,'egoInitialPosition',x(1:2),'egoInitialYaw',x(3), ...
        'egoInitialBodyVelocity',x(4:5),'targetInitialState',[25;0;0;0;0;0], ...
        'inertialVelocityInitialization',struct('value',x(4:5),'radius',.05));
    runtime=onlineNrmmTrackingRuntime('initialize',estimator.observer,options,design);
    frame=struct('time',0,'xGps',x(1),'yGps',x(2),'longitudinalAcceleration',0, ...
        'lateralAcceleration',0,'yawRateMeasured',0,'radarDetectionAvailable',false, ...
        'radarRelativePosition',[NaN;NaN],'heldSteeringAngle',u(1),'heldBrakingRatio',u(2));
    output=onlineNrmmTrackingRuntime('output',runtime,frame);output.targetEstimate=[];
    [~,~,prediction]=collisionAvoidanceController(output,[],[],cfg,[]);
    model=prediction.model;model.target=[25;0;0;0;0;0;cfg.vehicle.lr;cfg.vehicle.length/2;cfg.vehicle.width/2;0;0];
    model.targetEpoch=model.target;model.targetOnset=true;model.cfg.controller.retainedTube=true;
    model.uncertainty.jointPrediction=output.controllerErrorBound.jointPrediction;
    model.uncertainty.target=struct('positionRadius',.02,'courseRadius',.001, ...
        'speedRadius',0,'accelerationRadius',0,'curvatureRadius',0,'sideslipRadius',0);
    anchor=struct('states',repmat(x,1,3),'inputs',repmat(u,1,2));
    for k=1:2,anchor.states(:,k+1)=nonlinearBicycleModel.sample(anchor.states(:,k),u,cfg);end
    f=struct('cfg',cfg,'x',x,'u',u,'contract',output.controllerErrorBound.jointPrediction, ...
        'generator',output.controllerErrorBound.generator,'model',model,'anchor',anchor,'output',output,'runtime',runtime);
end

function target=localObservedTarget(output,cfg)
    [ego,~,~,~]=readControllerInputs(output,[],[],cfg);
    pose=ego.modelState(1:3);rotation=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
    target=struct('targetPositionInertial',[25;0],'targetVelocityInertial',[0;0],'targetYawInertial',0);
    target.controllerErrorBound=struct('kind',"target-state-v1",'time',output.stateTime, ...
        'available',true,'bounds',zeros(8,1));
    target.predictionErrorSet=struct('kind',"nrmm-constant-parameter-set-v1",'time',output.stateTime, ...
        'available',true,'referenceEgoPose',pose,'relativePosition',rotation.'*([25;0]-pose(1:2)), ...
        'positionRadius',.02,'courseCenter',-pose(3),'courseRadius',.001, ...
        'speedInterval',[0;0],'accelerationInterval',[0;0],'curvatureInterval',[0;0], ...
        'rearAxleDistance',cfg.target.rearAxleDistance);
end

function [runtime,x,output]=localAdvance(runtime,x,previous,input,cfg,time)
    plant=cfg;plant.tire.corneringStiffness=cfg.tire.corneringStiffness.*[1.01;.99];
    plant.tire.frictionCoefficient=cfg.tire.frictionCoefficient.*[1.02;.98];
    dt=runtime.samplePeriod;count=round(cfg.controller.sampleTime/dt);
    for k=1:count
        held=input;if k==1,held=previous;end
        frame=localFrame(x,held,plant,time+(k-1)*dt);
        runtime=onlineNrmmTrackingRuntime('step',runtime,frame);
        field=@(z)nonlinearBicycleModel.derivative(z,input,plant);
        a=field(x);b=field(x+dt*a/2);c=field(x+dt*b/2);d=field(x+dt*c);
        x=x+dt*(a+2*b+2*c+d)/6;
    end
    x=nonlinearBicycleModel.sampleEndState(x,input,plant);
    output=onlineNrmmTrackingRuntime('output',runtime,localFrame(x,input,plant,time+cfg.controller.sampleTime));
    output.targetEstimate=[];
end

function frame=localFrame(x,input,cfg,time)
    field=nonlinearBicycleModel.derivative(x,input,cfg);
    frame=struct('time',time,'xGps',x(1)+.004*sin(11*time),'yGps',x(2)+.004*sin(13*time), ...
        'longitudinalAcceleration',field(4)-x(5)*x(6)+.003*sin(17*time), ...
        'lateralAcceleration',field(5)+x(4)*x(6)+.003*sin(19*time), ...
        'yawRateMeasured',x(6)+.0001*sin(23*time),'radarDetectionAvailable',false, ...
        'radarRelativePosition',[NaN;NaN],'heldSteeringAngle',input(1),'heldBrakingRatio',input(2));
end
