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
        function anAuxiliaryProposalMayUseCollisionSlackAfterTheOldPrefix(testCase)
            f=testCase.fixture;count=20;anchor=localLongAnchor(f,count);
            model=f.model;model.retainedProposal=true;
            model.target(1)=anchor.states(1,end);model.targetEpoch=model.target;
            model.uncertainty.egoGenerator=zeros(6);
            model.uncertainty.collisionGenerator=zeros(6);
            model.uncertainty.target=[];
            [problem,~,model]=solvePredictiveControl('probe',struct( ...
                'operation','formulate','model',model,'anchor',anchor,'inherited',false));
            % Hold the nominal trajectory fixed: only PCBF slack can repair
            % its overlap at the end, beyond the old two-node soft prefix.
            fixed=[problem.stateIndices(:);problem.inputIndices(:)];
            problem.lower(fixed)=0;problem.upper(fixed)=0;
            [proposal,~]=solvePredictiveControl('probe',struct( ...
                'operation','solve','model',model,'anchor',anchor,'problem',problem));
            testCase.assertNotEmpty(proposal);
            testCase.verifyGreaterThan(proposal.safety,0);
            testCase.verifyEqual(proposal.inputs,anchor.inputs,AbsTol=1e-8);
            % Optional point proposals are not certificates. Their actual
            % terminal collision must still fail whole-set admission.
            terminal=terminalSafeSet.certifySet(terminalSafeSet.context(model), ...
                nrmmSet(anchor.states(:,end)),model.target,[],count*f.cfg.controller.sampleTime,100);
            testCase.verifyFalse(terminal.valid);
        end
        function projectedSupportsCoverCoupledPositionAndYaw(testCase)
            ego=nrmmSet([0;2;.4],[.1,.02;.05,.2;.25,.05]);
            target=nrmmSet([3;0;-.2],[.2,-.03;.05,.05;-.1,.2]);
            shape=[2.4;.95;.3;-.1];normal=[-.6;.8];
            gap=predictiveSafetyGeometry.setSeparation(ego,shape,target,shape,normal);
            vertices=shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1];
            for a=linspace(-1,1,5)
                for b=linspace(-1,1,5)
                    e=ego.center+ego.generator*[a;b];
                    t=target.center+target.generator*[b;-a];
                    re=[cos(e(3)),-sin(e(3));sin(e(3)),cos(e(3))];
                    rt=[cos(t(3)),-sin(t(3));sin(t(3)),cos(t(3))];
                    actual=min(normal.'*(e(1:2)+re*vertices))-max(normal.'*(t(1:2)+rt*vertices));
                    testCase.verifyLessThanOrEqual(gap,actual+1e-12);
                end
            end
        end
        function aCompleteTargetCoverCanSeparateWithoutOneCommonHalfspace(testCase)
            [coarse,covered]=localCurvedTargetCover(false);
            testCase.verifyLessThan(coarse,0);
            testCase.verifyGreaterThan(covered,0);
        end
        function aTargetCoverRetainsEveryCollidingParameterCell(testCase)
            [~,covered]=localCurvedTargetCover(true);
            testCase.verifyLessThanOrEqual(covered,0);
        end
        function aWiderPublicationDoesNotEraseTheOldTargetInformation(testCase)
            [old,next]=localWiderTargetContinuation(testCase.fixture);
            testCase.verifyEqual(next.safety,0,AbsTol=1e-12);
            testCase.verifyFalse(next.terminalControlRequired);
            testCase.verifyEqual(numel(next.retainedCertificate.policies),old.retainedCertificate.horizonSteps);
            testCase.verifyEqual(next.retainedCertificate.targetError,old.retainedCertificate.targetError);
            testCase.verifyGreaterThanOrEqual(numel(next.retainedCertificate.targetRefinements),1);
        end
        function actuatorChangeRetainsSharedFeedbackBeforeCompression(testCase)
            interval=localActuatorChange(testCase.fixture,1);
            testCase.verifyEqual(interval,[-.01,.01],AbsTol=1e-12);
        end
        function actuatorChangeDoesNotCancelOppositeUncertainty(testCase)
            interval=localActuatorChange(testCase.fixture,-1);
            testCase.verifyEqual(interval,[-.11,.11],AbsTol=1e-12);
        end
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
        function independentPositionErrorsCannotCancelToInventAPerceptionExit(testCase)
            f=testCase.fixture;context=terminalSafeSet.context(f.model);context.range=5;
            target=f.model.target;target(1:6)=[6;0;0;0;0;0];
            errors=struct('positionRadius',1,'courseRadius',0,'speedRadius',0, ...
                'accelerationRadius',0,'curvatureRadius',0,'sideslipRadius',0);
            state=nrmmSet(zeros(6,1),[1;0;0;0;0;0]);
            result=terminalSafeSet.certifySet(context,state,target,errors,0,20);
            testCase.verifyFalse(result.valid);
            testCase.verifyNotEqual(result.endReason,"uniformPerceptionExit");
        end
        function aProvenPrefixExitEndsThatTargetFamilyBeforeALaterReturn(testCase)
            f=testCase.fixture;context=terminalSafeSet.context(f.model);context.range=10;
            target=f.model.target;target(1:6)=[5;0;pi;5;0;asin(.1*target(7))];
            target(3)=pi-target(6);period=2*pi/(.1*5);
            context.encounterHistory=struct('time',[0,period/2,period], ...
                'positionLower',zeros(2,3),'positionUpper',zeros(2,3));
            result=terminalSafeSet.certifySet(context,nrmmSet(zeros(6,1)),target,[],period,100);
            testCase.verifyTrue(result.valid);
            testCase.verifyEqual(result.endReason,"perceptionExitDuringDrivingPrefix");
            testCase.verifyEqual(result.exitTime,period/2,AbsTol=1e-12);
        end
        function parameterSubdivisionCannotTurnAnActualOverlapIntoSafety(testCase)
            f=testCase.fixture;context=terminalSafeSet.context(f.model);
            context.encounterHistory=struct('time',0, ...
                'positionLower',zeros(2,1),'positionUpper',zeros(2,1));
            target=f.model.target;target(1:6)=[5.5;0;0;0;0;0];
            errors=struct('positionRadius',1,'courseRadius',.1,'speedRadius',0, ...
                'accelerationRadius',0,'curvatureRadius',.01,'sideslipRadius',.02, ...
                'speedInterval',[0;0],'accelerationInterval',[0;0],'curvatureInterval',[-.01;.01]);
            result=terminalSafeSet.certifySet(context,nrmmSet(zeros(6,1)),target,errors,0,20);
            testCase.verifyFalse(result.valid);
            testCase.verifyTrue(any(result.endReason==["unprovedSeparation","entryFamilyContainsCollision"]));
        end
        function runningOutOfTerminalChecksIsNeverACertificate(testCase)
            f=testCase.fixture;context=terminalSafeSet.context(f.model);
            result=terminalSafeSet.certifySet(context,nrmmSet(f.x,f.generator),f.model.target,[],0,0);
            testCase.verifyFalse(result.valid);
            testCase.verifyEqual(result.endReason,"workLimit");
        end
        function mixedStoppingAndMovingTargetsNeedNotExitSimultaneously(testCase)
            f=testCase.fixture;context=terminalSafeSet.context(f.model);
            target=f.model.target;target(1:6)=[20;5;pi;1;-.2;0];
            errors=struct('positionRadius',.02,'courseRadius',0,'speedRadius',1, ...
                'accelerationRadius',.2,'curvatureRadius',0,'sideslipRadius',0, ...
                'speedInterval',[0;2],'accelerationInterval',[-.4;0],'curvatureInterval',[0;0]);
            result=terminalSafeSet.certifySet(context,nrmmSet(zeros(6,1)),target,errors,0,150);
            testCase.verifyTrue(result.valid);
            testCase.verifyEqual(result.endReason,"parkedArcToPerceptionExit");
            testCase.verifyGreaterThan(result.minimumGap,0);
        end
        function aParkedArcStillRejectsAnApproachingCollision(testCase)
            f=testCase.fixture;context=terminalSafeSet.context(f.model);
            target=f.model.target;target(1:6)=[20;0;pi;1;0;0];
            result=terminalSafeSet.certifySet(context,nrmmSet(zeros(6,1)),target,[],0,150);
            testCase.verifyFalse(result.valid);
            testCase.verifyEqual(result.endReason,"unprovedSeparation");
        end
        function anOutgoingFamilyHasACompleteMonotoneExitCertificate(testCase)
            f=testCase.fixture;context=terminalSafeSet.context(f.model);
            target=f.model.target;target(1:6)=[20;8;0;2;-.2;0];
            errors=struct('positionRadius',.1,'courseRadius',.02,'speedRadius',2, ...
                'accelerationRadius',.2,'curvatureRadius',.001,'sideslipRadius',.002, ...
                'speedInterval',[0;4],'accelerationInterval',[-.4;0],'curvatureInterval',[-.001;.001]);
            result=terminalSafeSet.certifySet(context,nrmmSet(zeros(6,1)),target,errors,0,150);
            testCase.verifyTrue(result.valid);
            testCase.verifyEqual(result.endReason,"parkedMonotoneToPerceptionExit");
            testCase.verifyGreaterThan(result.minimumGap,0);
            testCase.verifyGreaterThan(result.parkedArc.exitProjectionLower,context.range);
            testCase.verifyGreaterThanOrEqual(result.parkedArc.minimumCourseCosine,0);
        end
        function firstAdmissionCreatesAFeasibleNonlinearCertificate(testCase)
            f=testCase.fixture;
            [solution,search]=solvePredictiveControl('probe',struct('operation','retained', ...
                'model',f.model,'previousState',[],'anchor',f.anchor));
            testCase.verifyTrue(solution.retainedCertificate.valid);
            testCase.verifyTrue(search.retainedCandidateFeasible);
            testCase.verifyEqual(solution.safety,0,AbsTol=0);
            testCase.verifyGreaterThan(max(radius(solution.retainedCertificate.tube.sets{1})),0);
            policy=solution.retainedCertificate.policies(1);
            testCase.verifyGreaterThan(max(policy.upper-policy.lower),0);
        end
        function performanceFreedomRetainsTheFutureFeedbackPolicy(testCase)
            f=testCase.fixture;
            [solution,~]=solvePredictiveControl('probe',struct('operation','retained', ...
                'model',f.model,'previousState',[],'anchor',f.anchor));
            policies=solution.retainedCertificate.policies;
            testCase.verifyGreaterThan(max(policies(1).upper-policies(1).lower),0);
            testCase.verifyEqual([policies(2:end).lower],[policies(2:end).upper]);
            testCase.verifyGreaterThan(norm(policies(2).context.gain,'fro'),0);
            testCase.verifyFalse(policies(2).context.first);
        end
        function changingToCruiseKeepsTheJointStateInformation(testCase)
            result=localCruiseInformationContinuation(testCase.fixture);
            testCase.verifyTrue(all(result.clfEnabled));
            testCase.verifyTrue(all(result.informationRetained));
            testCase.verifyLessThan(max(result.lateralRadius),.1);
            testCase.verifyLessThanOrEqual(result.containmentExcess,1e-9);
        end
        function initialCruiseAlreadyPropagatesTheJointInformation(testCase)
            [first,second,radius]=localInitialCruiseInformation(testCase.fixture);
            testCase.verifyTrue(isfield(first,'egoJointPrediction'));
            testCase.verifyTrue(isfield(second,'egoJointPrediction'));
            testCase.verifyEqual(second.egoJointPrediction.sampleIndex,2);
            testCase.verifyLessThan(radius,.1);
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
        function aLongDrivingWitnessIsReusedWithoutRepeatingItsTerminalProof(testCase)
            f=testCase.fixture;model=f.model;model.target(2)=15;model.targetEpoch=model.target;
            terminal=terminalSafeSet.context(model);
            count=ceil(norm(f.x(4:5))/(terminal.deceleration*f.cfg.controller.sampleTime))+10;
            anchor=localLongAnchor(f,count);
            [first,~]=solvePredictiveControl('probe',struct('operation','retained', ...
                'model',model,'previousState',[],'anchor',anchor));
            [~,~,output]=localAdvance(f.runtime,f.x,f.u,first.inputs(:,1),f.cfg,0);
            [ego,~,~,observation]=readControllerInputs(output,[],[],f.cfg);
            model.initialState=ego.modelState;model.previousInput=first.inputs(:,1);
            model.stateTime=output.stateTime;model.sampleIndex=1;model.targetOnset=false;
            model.uncertainty=predictiveSafetyGeometry.observerUncertainty(ego,observation,model.target);
            model.uncertainty.target=f.model.uncertainty.target;
            previous=struct('retainedCertificate',first.retainedCertificate);
            [next,search]=solvePredictiveControl('probe',struct('operation','retained', ...
                'model',model,'previousState',previous,'anchor',anchor));
            testCase.verifyTrue(search.certificateReused);
            testCase.verifyTrue(search.retainedCandidateFeasible);
            testCase.verifyFalse(search.nominalProposal.available);
            testCase.verifyEqual(next.safety,0,AbsTol=0);
            testCase.verifyEqual(next.retainedCertificate.terminal,first.retainedCertificate.terminal);
            testCase.verifyEqual(next.retainedCertificate.horizonSteps,count);
            testCase.verifyEqual(numel(next.retainedCertificate.policies),count-1);
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
            f=testCase.fixture;cfg=f.cfg;
            target=localObservedTarget(f.output,cfg);
            lane=[-100,0;1000,0];
            [command,~,prediction,memory]=collisionAvoidanceController(f.output,target,lane,cfg,[]);
            testCase.verifyTrue(prediction.metadata.zeroSlack);
            testCase.verifyTrue(prediction.metadata.retainedCandidateFeasible);
            testCase.verifyEqual(prediction.metadata.solverAlgorithm,"retainedJointPolicyTube");
            testCase.verifyFalse(prediction.metadata.robustNonlinearSafetyCertified);
            testCase.verifyGreaterThanOrEqual(prediction.metadata.search.linearizationCount,0);
            testCase.verifyFalse(prediction.metadata.search.nominalRestarted);
            [~,~,output]=localAdvance(f.runtime,f.x,f.u,command.actuatorInput,cfg,0);
            target=localObservedTarget(output,cfg);
            [~,~,next,~]=collisionAvoidanceController(output,target,lane,cfg,memory);
            testCase.verifyTrue(next.metadata.zeroSlack);
            testCase.verifyEqual(next.metadata.horizonSteps,prediction.metadata.horizonSteps);
            testCase.verifyTrue(next.metadata.search.nominalProposal.available);
            testCase.verifyGreaterThan(next.metadata.search.solverCalls,1);
            testCase.verifyGreaterThanOrEqual(next.metadata.search.linearizationCount,0);
            testCase.verifyFalse(next.metadata.search.nominalRestarted);
        end
        function currentImuDataRemoveInconsistentLateralVelocities(testCase)
            f=testCase.fixture;
            root=nrmmJointPrediction('rootSet',f.x,f.generator,f.contract);
            root(5)=nrmmSet.range(-1.9,1.9);root(6)=nrmmSet.range(-.64,.64);
            result=nrmmJointPrediction('conditionMeasurements',root,f.contract,f.cfg);
            testCase.verifyLessThan(radius(result.set(5)),.02);
            testCase.verifyEqual(radius(result.set(6)),f.contract.noiseRadius(7),AbsTol=1e-12);
            testCase.verifyEqual(result.diagnostics.parameterRadius,f.contract.parameterRadius,AbsTol=0);
            testCase.verifyEqual(result.diagnostics.noiseRadius,f.contract.noiseRadius,AbsTol=0);
        end
        function aBroadPublicationDoesNotUndoTheProposalsMeasurementConditioning(testCase)
            f=testCase.fixture;lane=[-100,0;1000,0];
            target=localObservedTarget(f.output,f.cfg);
            [command,~,~,memory]=collisionAvoidanceController(f.output,target,lane,f.cfg,[]);
            [~,~,output]=localAdvance(f.runtime,f.x,f.u,command.actuatorInput,f.cfg,0);
            output.controllerErrorBound.generator=[output.controllerErrorBound.generator,[0;0;0;0;4;0]];
            output.controllerErrorBound.bounds=sum(abs(output.controllerErrorBound.generator),2);
            target=localObservedTarget(output,f.cfg);
            [~,~,next]=collisionAvoidanceController(output,target,lane,f.cfg,memory);
            testCase.verifyTrue(next.metadata.search.nominalProposal.available);
            testCase.verifyTrue(next.metadata.retainedCertificateValid);
            testCase.verifyTrue(next.metadata.zeroSlack);
            testCase.verifyLessThan(next.metadata.search.nominalProposal.initialPhysicalRadius(5),.1);
        end
        function currentMeasurementContractionContainsAllowedMismatchAndNoise(testCase)
            excess=localContractionSamples(testCase.fixture);
            testCase.verifyLessThanOrEqual(excess,1e-10);
        end
        function impossibleCurrentAccelerationIsReportedAsInconsistent(testCase)
            f=testCase.fixture;root=nrmmJointPrediction('rootSet',f.x,f.generator,f.contract);
            contract=f.contract;contract.currentMeasurement.bodyAcceleration(2)=100;
            testCase.verifyError(@()nrmmJointPrediction('conditionMeasurements',root,contract,f.cfg), ...
                'nrmmJointPrediction:inconsistentMeasurement');
        end
        function fullBrakingDoesNotInventAnInformativeLateralMeasurement(testCase)
            f=testCase.fixture;contract=f.contract;contract.parameterRadius(:)=0;
            input=[0;-1];field=nonlinearBicycleModel.derivative(f.x,input,f.cfg);
            contract.currentMeasurement.heldInput=input;
            contract.currentMeasurement.bodyAcceleration=[field(4);field(5)];
            root=nrmmJointPrediction('rootSet',f.x,zeros(6),contract);root(5)=nrmmSet.range(-1.9,1.9);
            result=nrmmJointPrediction('conditionMeasurements',root,contract,f.cfg);
            testCase.verifyEqual(radius(result.set(5)),1.9,AbsTol=1e-12);
        end
        function currentMeasurementsKeepDrivingBeyondTheFormerFailure(testCase)
            result=localRepeatedConditionedDriving(testCase.fixture,20);
            testCase.verifyEqual(result.drivingFrames,20);
            testCase.verifyEqual(result.maximumSlack,0,AbsTol=0);
            testCase.verifyLessThanOrEqual(result.maximumContainmentExcess,1e-9);
            testCase.verifyLessThan(result.maximumLateralRadius,.03);
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

function interval=localActuatorChange(f,correlation)
    root=nrmmJointPrediction('rootSet',f.x,f.generator,f.contract);
    column=zeros(21,1);column(10)=.2;column(17)=correlation*.05;
    root.generator(:,end+1)=column;
    gain=zeros(2,6);gain(2,4)=.25;
    context=struct('cfg',f.cfg,'contract',f.contract, ...
        'reference',[root.center(7:11);root.center(6)],'input',f.u, ...
        'gain',gain,'duration',f.cfg.controller.sampleTime,'first',false);
    policy=struct('context',context,'lower',[0;-.01],'upper',[0;.01]);
    tube=nrmmJointPrediction('policyTube',root,policy);
    [lo,hi]=bounds(tube.changes{1}(2));interval=[lo,hi];
end

function [coarse,covered]=localCurvedTargetCover(overlap)
    beta=asin(.06*1.6);target=[0;0;pi-beta;8;0;beta;1.6;2.4;.95;0;0];
    error=struct('positionRadius',.1,'courseRadius',0,'speedRadius',4, ...
        'accelerationRadius',0,'curvatureRadius',.04,'sideslipRadius',.07, ...
        'curvatureInterval',[.02;.1]);
    ego=nrmmSet([30;0;0]);
    if overlap
        point=predictiveSafetyGeometry.predictTarget(target,5);ego=nrmmSet(point(1:3));
    end
    shape=[2.4;.95;0;0];image=predictiveSafetyGeometry.targetPoseSet(target,error,5);
    coarse=predictiveSafetyGeometry.setSeparation(ego,shape,image,target(8:11));
    covered=predictiveSafetyGeometry.targetFamilySeparation(ego,shape,target,error,5);
end

function [first,next]=localWiderTargetContinuation(f)
    [first,~]=solvePredictiveControl('probe',struct('operation','retained', ...
        'model',f.model,'previousState',[],'anchor',f.anchor));
    [~,~,output]=localAdvance(f.runtime,f.x,f.u,first.inputs(:,1),f.cfg,0);
    [ego,~,~,observation]=readControllerInputs(output,[],[],f.cfg);
    model=f.model;model.initialState=ego.modelState;model.previousInput=first.inputs(:,1);
    model.stateTime=output.stateTime;model.sampleIndex=1;model.targetOnset=false;
    model.uncertainty=predictiveSafetyGeometry.observerUncertainty(ego,observation,model.target);
    model.uncertainty.target=f.model.uncertainty.target;
    model.uncertainty.target.positionRadius=40;
    previous=struct('retainedCertificate',first.retainedCertificate);
    [next,~]=solvePredictiveControl('probe',struct('operation','retained', ...
        'model',model,'previousState',previous,'anchor',f.anchor));
end

function excess=localContractionSamples(f)
    excess=-Inf;
    for j=1:6
        x=f.x;x(5:6)=[.05*sin(j);.02*cos(j)];input=[.02;-.3];
        parameter=.05*[sin(j);cos(j);sin(2*j);cos(2*j)];plant=f.cfg;
        plant.tire.corneringStiffness=f.cfg.tire.corneringStiffness.*(1+parameter(1:2));
        plant.tire.frictionCoefficient=f.cfg.tire.frictionCoefficient.*(1+parameter(3:4));
        field=nonlinearBicycleModel.derivative(x,input,plant);contract=f.contract;
        contract.currentMeasurement.gnssPosition=x(1:2)+[.02;-.01];
        contract.currentMeasurement.yawRate=x(6)+.001*sin(j);
        contract.currentMeasurement.heldInput=input;
        contract.currentMeasurement.bodyAcceleration=[field(4)-x(5)*x(6);field(5)+x(4)*x(6)]+[.01;-.02];
        root=nrmmJointPrediction('rootSet',f.x,diag([.04;.04;.01;.1;.5;.1]),contract);
        result=nrmmJointPrediction('conditionMeasurements',root,contract,f.cfg);
        [lo,hi]=bounds(result.set([1:6,12:15]));actual=[x;parameter];
        excess=max(excess,max([lo-actual;actual-hi]));
    end
end

function result=localRepeatedConditionedDriving(f,count)
    model=f.model;previous=[];runtime=f.runtime;x=f.x;held=f.u;
    result=struct('drivingFrames',0,'maximumSlack',0,'maximumContainmentExcess',-Inf,'maximumLateralRadius',0);
    for k=0:count-1
        [solution,~]=solvePredictiveControl('probe',struct('operation','retained', ...
            'model',model,'previousState',previous,'anchor',f.anchor));
        if solution.terminalControlRequired,return;end
        root=solution.retainedCertificate.tube.sets{1};[lo,hi]=bounds(root(1:6));
        result.maximumContainmentExcess=max(result.maximumContainmentExcess,max([lo-x;x-hi]));
        result.maximumLateralRadius=max(result.maximumLateralRadius,radius(root(5)));
        result.maximumSlack=max(result.maximumSlack,solution.safety);result.drivingFrames=k+1;
        [runtime,x,output]=localAdvance(runtime,x,held,solution.inputs(:,1),f.cfg,k*f.cfg.controller.sampleTime);
        [ego,~,~,observation]=readControllerInputs(output,[],[],f.cfg);
        held=solution.inputs(:,1);model.initialState=ego.modelState;model.previousInput=held;
        model.stateTime=output.stateTime;model.sampleIndex=k+1;model.targetOnset=false;
        model.uncertainty=predictiveSafetyGeometry.observerUncertainty(ego,observation,model.target);
        model.uncertainty.target=f.model.uncertainty.target;
        previous=struct('retainedCertificate',solution.retainedCertificate);
    end
end

function result=localCruiseInformationContinuation(f)
    cfg=f.cfg;cfg.controller.retainedTube=true;
    lane=[0,0;500,0];
    [command,~,~,previous]=collisionAvoidanceController(f.output,localObservedTarget(f.output,cfg),lane,cfg,[]);
    runtime=f.runtime;x=f.x;held=f.u;
    result=struct('clfEnabled',false(1,2),'informationRetained',false(1,2), ...
        'lateralRadius',zeros(1,2),'containmentExcess',-Inf);
    for k=1:2
        [runtime,x,output]=localAdvance(runtime,x,held,command.actuatorInput,cfg,(k-1)*cfg.controller.sampleTime);
        held=command.actuatorInput;output.heldActuatorInput=held;
        % Exercise the complete-visibility mode-switch interface and a
        % deliberately loose, but still valid, standalone publication.
        output.perception=struct('time',output.stateTime,'completeWithinRange',true);
        output.controllerErrorBound.generator(:,end+1)=[0;0;0;0;4;0];
        output.controllerErrorBound.bounds=sum(abs(output.controllerErrorBound.generator),2);
        [command,~,prediction,previous]=collisionAvoidanceController(output,[],lane,cfg,previous);
        result.clfEnabled(k)=prediction.metadata.clfEnabled;
        result.informationRetained(k)=prediction.metadata.egoInformationRetainedAcrossModeSwitch;
        result.lateralRadius(k)=sum(abs(prediction.model.uncertainty.egoGenerator(5,:)));
        actual=nonlinearBicycleModel.sample(x,command.actuatorInput,cfg);
        [lo,hi]=bounds(previous.egoJointPrediction.set(1:6));
        result.containmentExcess=max(result.containmentExcess,max([lo-actual;actual-hi]));
    end
end

function anchor=localLongAnchor(f,count)
    anchor=struct('states',repmat(f.x,1,count+1),'inputs',repmat(f.u,1,count));
    for k=1:count
        anchor.states(:,k+1)=nonlinearBicycleModel.sample(anchor.states(:,k),f.u,f.cfg);
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

function [first,second,lateralRadius]=localInitialCruiseInformation(f)
    road=struct('centerline',[-100,0;1000,0]);
    [command,~,~,first]=collisionAvoidanceController(f.output,[],road,f.cfg,[]);
    [~,~,output]=localAdvance(f.runtime,f.x,f.u,command.actuatorInput,f.cfg,0);
    output.targetEstimate=[];
    [~,~,prediction,second]=collisionAvoidanceController(output,[],road,f.cfg,first);
    lateralRadius=sum(abs(prediction.model.uncertainty.egoGenerator(5,:)));
end
