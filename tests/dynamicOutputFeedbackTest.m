classdef dynamicOutputFeedbackTest < matlab.unittest.TestCase
    % Causal raw-block inference, policy execution, and physical backup.
    properties (Access=private)
        cfg
        hypothesis
        block
        state
        prediction
    end
    properties (TestParameter)
        targetAcceleration = {0,.5,-.01}
        targetSideslip = {0,.04}
    end
    methods (TestClassSetup)
        function prepareJointController(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            [cfg,hypothesis,block]=localFixture();
            [~,~,prediction,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.cfg=cfg;testCase.hypothesis=hypothesis;testCase.block=block;
            testCase.state=state;testCase.prediction=prediction;
        end
    end
    methods (Test)
        function theFirstCommandIsSharedAndLaterCommandsUseReceivedInformation(testCase)
            hypotheses=repmat(testCase.hypothesis,1,2);hypotheses(2).ego(1)=.03;
            policies=testCase.state.policies;
            policies(1).B(1,1)=1;policies(2).K(1,1)=.2;
            request=struct('operation',"predict",'cfg',testCase.cfg,'hypotheses',hypotheses, ...
                'memory',zeros(2,1),'input',testCase.block.heldInput,'policies',policies,'reference',[]);
            traces=solvePredictiveControl('outputFeedback',request);
            testCase.verifyEqual(traces{1}.inputs(:,1),traces{2}.inputs(:,1),AbsTol=1e-14);
            testCase.verifyGreaterThan(norm(traces{1}.inputs(:,2)-traces{2}.inputs(:,2)),1e-7);
            testCase.verifyEqual(traces{1}.memory(:,1),traces{2}.memory(:,1),AbsTol=0);
        end
        function theCommittedMapUpdatesMemoryExactlyOnceForTheNextBlock(testCase)
            block=localNextBlock(testCase.state,testCase.cfg);
            expected=solvePredictiveControl('outputFeedback',struct('operation',"memory", ...
                'cfg',testCase.cfg,'policy',testCase.state.committedPolicy, ...
                'memory',testCase.state.memory,'samples',block.samples));
            [command,~,prediction,state]=collisionAvoidanceController(block,[],[],testCase.cfg,testCase.state);
            testCase.verifyEqual(state.memory,expected,AbsTol=1e-14);
            testCase.verifyEqual(command.actuatorInput,state.committedPolicy.v+state.committedPolicy.K*expected,AbsTol=1e-14);
            testCase.verifyFalse(prediction.metadata.nonlinearValidationPerformed);
            testCase.verifyLessThanOrEqual(abs(state.memory),ones(2,1));
        end
        function everyRawReadingConstrainsThePosteriorEvenWithIdenticalFeatures(testCase)
            block=localNextBlock(testCase.state,testCase.cfg);bad=block;
            bad.samples(1).position(1)=bad.samples(1).position(1)+1;
            bad.samples(2).position(1)=bad.samples(2).position(1)-1;
            feature=localOperation('features',testCase.cfg,struct('samples',block.samples));
            other=localOperation('features',testCase.cfg,struct('samples',bad.samples));
            testCase.verifyEqual(feature,other,AbsTol=1e-14);
            testCase.verifyError(@()collisionAvoidanceController(bad,[],[],testCase.cfg,testCase.state), ...
                'collisionAvoidanceController:emptyJointInformation');
        end
        function accelerationUsesThePreviouslyAppliedCommand(testCase)
            bad=testCase.block;bad.heldInput=[0;0];
            testCase.verifyError(@()collisionAvoidanceController(bad,[],[],testCase.cfg,[]), ...
                'collisionAvoidanceController:emptyJointInformation');
        end
        function aDifferentAppliedCommandInvalidatesContinuation(testCase)
            bad=localNextBlock(testCase.state,testCase.cfg);bad.heldInput(2)=bad.heldInput(2)+.01;
            testCase.verifyError(@()collisionAvoidanceController(bad,[],[],testCase.cfg,testCase.state), ...
                'collisionAvoidanceController:actuationMismatch');
        end
        function missingIntermediateReadingsCannotBeSilentlySkipped(testCase)
            bad=localNextBlock(testCase.state,testCase.cfg);bad.samples(2)=[];
            testCase.verifyError(@()collisionAvoidanceController(bad,[],[],testCase.cfg,testCase.state), ...
                'collisionAvoidanceController:invalidSensorGrid');
        end
        function aDropoutRetainsThePhysicalTargetAndItsLatentParameters(testCase)
            block=testCase.block;block.samples.radarAvailable=false;block.samples.relativePosition=[NaN;NaN];
            posterior=localOperation('condition',testCase.cfg,struct('hypotheses',testCase.hypothesis,'block',block,'initial',true));
            testCase.verifyEqual(posterior.target,testCase.hypothesis.target,AbsTol=0);
            testCase.verifyTrue(all(isfinite(localOperation('features',testCase.cfg,struct('samples',block.samples)))));
        end
        function parameterAndSensorBiasHypothesesPersistThroughMotion(testCase)
            hypothesis=testCase.hypothesis;hypothesis.parameters=[.9;1.1;.95;1.05];hypothesis.sensorBias=(1:7)'*.001;
            next=localOperation('advance',testCase.cfg,struct('hypothesis',hypothesis,'input',[0;-.6], ...
                'duration',.05,'endOfHold',true));
            testCase.verifyEqual(next.parameters,hypothesis.parameters,AbsTol=0);
            testCase.verifyEqual(next.sensorBias,hypothesis.sensorBias,AbsTol=0);
        end
        function sensorBoundariesDoNotApplyTheControlBoundaryRestReset(testCase)
            hypothesis=testCase.hypothesis;hypothesis.ego(4:6)=[.04;.2;.1];
            request=struct('hypothesis',hypothesis,'input',[0;-.6],'duration',.0125,'endOfHold',false);
            middle=localOperation('advance',testCase.cfg,request);request.endOfHold=true;
            boundary=localOperation('advance',testCase.cfg,request);
            testCase.verifyGreaterThan(norm(middle.ego(4:6)),0);
            testCase.verifyEqual(boundary.ego(4:6),zeros(3,1),AbsTol=0);
            testCase.verifyEqual(middle.ego(1:3),boundary.ego(1:3),AbsTol=0);
        end
        function anAbsorbedTargetCannotRestartFromItsLatentAcceleration(testCase)
            hypothesis=testCase.hypothesis;hypothesis.target(5)=2;
            next=localOperation('advance',testCase.cfg,struct('hypothesis',hypothesis,'input',[0;-.6], ...
                'duration',.05,'endOfHold',true));
            testCase.verifyEqual(next.target,hypothesis.target,AbsTol=0);
        end
        function constantAccelerationAndSideslipTargetsReachTheQp(testCase,targetAcceleration,targetSideslip)
            [cfg,hypothesis,block]=localFixture();
            hypothesis.target(4:6)=[1;targetAcceleration;targetSideslip];
            block.initialHypotheses=hypothesis;
            block.samples=localOperation('sensor',cfg,struct('hypothesis',hypothesis,'input',block.heldInput,'time',0));
            [command,~,prediction,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            nextBlock=localNextBlock(state,cfg);
            [nextCommand,~,nextPrediction,nextState]=collisionAvoidanceController(nextBlock,[],[],cfg,state);
            testCase.verifyTrue(all(isfinite([command.actuatorInput;nextCommand.actuatorInput])));
            testCase.verifyEqual([prediction.metadata.sqpIterations,nextPrediction.metadata.sqpIterations],[1,1]);
            testCase.verifyEqual(nextState.hypotheses.target(5:6),[targetAcceleration;targetSideslip],AbsTol=0);
            expected=predictiveSafetyGeometry.predictTarget(hypothesis.target, ...
                prediction.metadata.predictionWindow.allocatedSeconds);
            testCase.verifyEqual(prediction.scenarios{1}.targets(:,end),expected,AbsTol=1e-10);
        end
        function aSingleMeasurementDoesNotInventAnInitialInformationSet(testCase)
            block=rmfield(testCase.block,'initialHypotheses');
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],testCase.cfg,[]), ...
                'collisionAvoidanceController:missingInitialInformation');
        end
        function aContinuationCannotReplaceItsJointInformation(testCase)
            block=localNextBlock(testCase.state,testCase.cfg);block.initialHypotheses=testCase.hypothesis;
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],testCase.cfg,testCase.state), ...
                'collisionAvoidanceController:informationReset');
        end
        function oneLinearizedQpDoesNotClaimNonlinearSafety(testCase)
            metadata=testCase.prediction.metadata;
            testCase.verifyFalse(metadata.nonlinearValidationPerformed);
            testCase.verifyTrue(metadata.sparseQp);
            testCase.verifyFalse(metadata.robustNonlinearSafetyCertified);
            testCase.verifyFalse(metadata.recursiveFeasibilityCertified);
            testCase.verifyLessThanOrEqual(metadata.maximumLinearizedConstraintViolation,testCase.cfg.solver.constraintTolerance);
            testCase.verifyEqual(metadata.sqpIterations,1);
            testCase.verifyEqual(metadata.acceptedSteps,1);
        end
        function theBackupReserveBoundsTheActualLowSpeedTravel(testCase)
            [minimumReserve,travel,bound,minimumSpeed]=localBackupAudit(testCase.cfg);
            testCase.verifyGreaterThanOrEqual(minimumReserve,-1e-8);
            testCase.verifyLessThanOrEqual(travel,bound+1e-8);
            testCase.verifyGreaterThanOrEqual(minimumSpeed,-1e-10);
        end
        function theTerminalSetRejectsReverseForcingAtZeroForwardSpeed(testCase)
            x=[0;0;0;0;.1;-.1];
            [margins,info]=terminalSafeSet.lowSpeedReserve(x,testCase.hypothesis.target,testCase.cfg,[0;-.6]);
            testCase.verifyLessThan(min(margins),0);
            testCase.verifyFalse(info.member);
        end
        function thePhysicalRestResetIncreasesTheSeparationReserve(testCase)
            x=[0;0;0;.04;.2;.1];q=testCase.hypothesis.target;
            [~,before]=terminalSafeSet.lowSpeedReserve(x,q,testCase.cfg,[0;-.6]);
            reset=nonlinearBicycleModel.sampleEndState(x,[0;-.6],testCase.cfg);
            [~,after]=terminalSafeSet.lowSpeedReserve(reset,q,testCase.cfg,[0;-.6]);
            testCase.verifyGreaterThan(after.separationReserveMeters,before.separationReserveMeters);
            testCase.verifyEqual(reset(1:3),x(1:3),AbsTol=0);
        end
        function aMissedDeadlineIsReportedWithoutClaimingTimelyExecution(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.maximumSolveSeconds=1e-12;
            [~,~,~,prior]=collisionAvoidanceController(testCase.block,[],[],cfg,[]);
            block=localNextBlock(prior,cfg);
            [~,~,prediction,next]=collisionAvoidanceController(block,[],[],cfg,prior);
            testCase.verifyTrue(prediction.metadata.deadlineMissed);
            testCase.verifyEqual(prediction.metadata.sqpIterations,1);
            testCase.verifyEqual(next.appliedInput,next.committedPolicy.v+next.committedPolicy.K*next.memory,AbsTol=1e-12);
        end
        function overlappingHypothesesCannotExecuteAnUncheckedPlan(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.maximumSolveSeconds=1e-12;
            hypothesis=testCase.hypothesis;hypothesis.target(1:2)=hypothesis.ego(1:2);
            block=testCase.block;block.initialHypotheses=hypothesis;
            block.samples=localOperation('sensor',cfg,struct('hypothesis',hypothesis,'input',block.heldInput,'time',0));
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],cfg,[]), ...
                'collisionAvoidanceController:noOutputFeedbackPlan');
        end
        function localShootingTangentsMatchIndependentPolicyPerturbations(testCase)
            [stateError,memoryError]=localTangentAudit();
            testCase.verifyLessThan(stateError,2e-7);
            testCase.verifyLessThan(memoryError,2e-7);
        end
        function anExplicitDurationControlsThePredictionCoverage(testCase)
            testCase.verifyGreaterThanOrEqual(numel(testCase.state.policies)*testCase.cfg.controller.sampleTime, ...
                testCase.cfg.outputFeedback.horizonSeconds);
            testCase.verifyTrue(testCase.prediction.metadata.predictionIsLinearized);
            testCase.verifyLessThan(testCase.prediction.metadata.maximumShootingEqualityResidual,1e-8);
        end
        function automaticCoverageGrowsWithTheCurrentSpeed(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.horizonSeconds=[];
            slow=testCase.hypothesis;medium=slow;fast=slow;
            medium.ego(4)=8;fast.ego(4)=25;
            a=localOperation('window',cfg,struct('hypotheses',slow));
            b=localOperation('window',cfg,struct('hypotheses',medium));
            c=localOperation('window',cfg,struct('hypotheses',fast));
            testCase.verifyGreaterThan(b.horizonSteps,a.horizonSteps);
            testCase.verifyGreaterThan(c.horizonSteps,b.horizonSteps);
            testCase.verifyGreaterThanOrEqual(c.allocatedSeconds,c.requestedSeconds);
            testCase.verifyFalse(c.feasibilityCertified);
        end
        function automaticCoverageUsesEveryRetainedHypothesis(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.horizonSeconds=[];
            hypotheses=repmat(testCase.hypothesis,1,2);hypotheses(2).ego(4:5)=[8;1];
            window=localOperation('window',cfg,struct('hypotheses',hypotheses));
            testCase.verifyEqual(window.speedUpperMetersPerSecond,hypot(8,1));
            testCase.verifyGreaterThanOrEqual(window.allocatedSeconds, ...
                hypot(8,1)/cfg.outputFeedback.previewDecelerationMetersPerSecondSquared ...
                +cfg.outputFeedback.previewSettlingSeconds);
        end
        function theAutomaticWindowActuallySizesTheExecutedPlan(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.horizonSeconds=[];
            [~,~,prediction,state]=collisionAvoidanceController(testCase.block,[],[],cfg,[]);
            window=prediction.metadata.predictionWindow;
            testCase.verifyEqual(window.source,"speedBasedPreview");
            testCase.verifyEqual(numel(state.policies),window.horizonSteps);
            testCase.verifyGreaterThan(window.allocatedSeconds,.2);
        end
        function aCapacityLimitCannotSilentlyShortenThePreview(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.horizonSeconds=[];
            cfg.controller.maximumHorizonSteps=10;
            testCase.verifyError(@()collisionAvoidanceController(testCase.block,[],[],cfg,[]), ...
                'collisionAvoidanceController:predictionWindowCapacity');
        end
        function brakingFeedforwardCanMoveBeyondTheNumericalBoxButNotActuatorLimits(testCase)
            policies=testCase.state.policies;
            for i=1:numel(policies)
                policies(i).v=[0;-.6];policies(i).K(:)=0;
            end
            request=struct('operation',"linearize",'cfg',testCase.cfg, ...
                'hypotheses',testCase.hypothesis,'memory',zeros(2,1), ...
                'input',testCase.block.heldInput,'policies',policies,'reference',[]);
            qp=solvePredictiveControl('outputFeedback',request);
            [flag,step]=localBrakeStep(qp,-.35);
            testCase.verifyGreaterThan(flag,0);
            testCase.verifyEqual(step(qp.policyIndices(2,1)),-.35,AbsTol=1e-9);
            testCase.verifyEqual(localBrakeStep(qp,-.5),-2);
            request.cfg.model.brakingRatioRateMaximum=1;
            limited=solvePredictiveControl('outputFeedback',request);
            testCase.verifyEqual(localBrakeStep(limited,-.35),-2);
        end
        function theFinalPolicyIsOptimizedInsteadOfFixedToBraking(testCase)
            testCase.verifyGreaterThan(testCase.state.policies(end).v(2),-.5);
            testCase.verifyLessThanOrEqual(abs(testCase.prediction.scenarios{1}.inputs(2,end)),1+1e-8);
        end
        function endpointClearanceDoesNotRequireRemainingStoppingTravel(testCase)
            [cfg,hypothesis,block]=localFixture();
            cfg.referenceSpeed=25;cfg.model.speedMaximum=max(30,cfg.model.speedMaximum);
            hypothesis.ego(4)=25;block.initialHypotheses=hypothesis;
            block.samples=localOperation('sensor',cfg,struct('hypothesis',hypothesis,'input',block.heldInput,'time',0));
            [command,~,prediction]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyGreaterThan(prediction.scenarios{1}.ego(4,end),20);
            testCase.verifyLessThan(prediction.metadata.maximumLinearizedConstraintViolation,cfg.solver.constraintTolerance);
        end
        function theSelectedRtiCampaignUsesRawSensorsWithoutTheNrmmPipeline(testCase)
            fixture=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            campaign=runCollisionThreatValidation(Frames=1,Speeds=1.5,Scenarios="stoppedLead", ...
                ControllerConfiguration=testCase.cfg,OutputDirectory=string(fixture.Folder));
            testCase.verifyEqual(campaign.pipeline,"sqpRti");
            testCase.verifyEqual(campaign.results.observationModel,"rawSensorFixedConsistencyInference");
            testCase.verifyEqual(campaign.results.executedFrames,1);
            testCase.verifyEqual(campaign.results.outcome,"completed");
            testCase.verifyEqual(campaign.results.timing.calls,1);
            testCase.verifyGreaterThan(campaign.results.timing.maximumSeconds,0);
            testCase.verifyTrue(campaign.results.sampledSafetyEvaluated);
            testCase.verifyTrue(campaign.results.sampledClearanceSatisfied);
        end
        function aRejectedCallIsCountedWithoutClaimingAPlantReplay(testCase)
            fixture=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            cfg=testCase.cfg;cfg.outputFeedback.horizonSeconds=[];cfg.controller.maximumHorizonSteps=10;
            campaign=runJointOutputFeedbackScenario(Frames=2,Speeds=1.5,Scenarios="stoppedLead", ...
                Warmup=false,ControllerConfiguration=cfg,OutputDirectory=string(fixture.Folder));
            testCase.verifyEqual(campaign.results.executedFrames,0);
            testCase.verifyEqual(campaign.results.failurePhase,"controller");
            testCase.verifyEqual(campaign.results.timing.calls,1);
            testCase.verifyGreaterThan(campaign.results.failureCallSeconds,0);
            testCase.verifyFalse(campaign.results.sampledSafetyEvaluated);
            testCase.verifyFalse(campaign.results.sampledClearanceSatisfied);
        end
    end
end

function result=localOperation(operation,cfg,request)
    request.operation=string(operation);request.cfg=cfg;
    result=solvePredictiveControl('outputFeedback',request);
end

function [cfg,hypothesis,block]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',1.5, ...
        'controller',struct('horizonSteps',3), ...
        'outputFeedback',struct('method',"sqpRti",'horizonSeconds',.15,'memoryDimension',2, ...
        'noiseDirections',zeros(7,1),'maximumSolveSeconds',120)));
    hypothesis=struct('ego',[0;0;0;1.5;0;0],'target',[15;0;0;0;0;0;1.6;2.4;.95;0;0], ...
        'parameters',ones(4,1),'sensorBias',zeros(7,1));
    input=[0;-.6];
    y=localOperation('sensor',cfg,struct('hypothesis',hypothesis,'input',input,'time',0));
    block=struct('kind',"jointSensorBlock-v1",'time',0,'samples',y,'heldInput',input,'initialHypotheses',hypothesis);
end

function block=localNextBlock(state,cfg)
    hypothesis=state.hypotheses(1);input=state.appliedInput;
    count=round(cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod);samples=struct([]);
    for j=1:count
        hypothesis=localOperation('advance',cfg,struct('hypothesis',hypothesis,'input',input, ...
            'duration',cfg.outputFeedback.sensorPeriod,'endOfHold',j==count));
        y=localOperation('sensor',cfg,struct('hypothesis',hypothesis,'input',input, ...
            'time',state.time+j*cfg.outputFeedback.sensorPeriod));
        if isempty(samples),samples=y;else,samples(end+1)=y;end %#ok<AGROW>
    end
    block=struct('kind',"jointSensorBlock-v1",'time',state.time+cfg.controller.sampleTime, ...
        'samples',samples,'heldInput',input);
end

function [reserve,travel,bound,minimumSpeed]=localBackupAudit(cfg)
    x=[0;0;0;1.5;.15;.1];q=[15;0;pi;1;-1;0;1.6;2.4;.95;0;0];u=[0;-.6];
    [~,initial]=terminalSafeSet.lowSpeedReserve(x,q,cfg,u);bound=initial.egoTravelMeters;
    solution=ode45(@(~,x)nonlinearBicycleModel.derivative(x,u,cfg),[0,8],x, ...
        odeset('RelTol',1e-10,'AbsTol',1e-12));
    times=linspace(0,8,4001);states=deval(solution,times);values=zeros(size(times));
    for j=1:numel(times)
        target=predictiveSafetyGeometry.predictTarget(q,times(j));
        [~,info]=terminalSafeSet.lowSpeedReserve(states(:,j),target,cfg,u);
        values(j)=info.separationReserveMeters-initial.separationReserveMeters;
    end
    reserve=min(values);travel=trapz(times,vecnorm(states(4:5,:)));minimumSpeed=min(states(4,:));
end

function [stateError,memoryError]=localTangentAudit()
    [cfg,hypothesis,block]=localFixture();
    [~,~,~,state]=collisionAvoidanceController(block,[],[],cfg,[]);
    request=struct('hypotheses',hypothesis,'memory',state.memory,'input',block.heldInput, ...
        'policies',state.policies,'reference',[]);
    qp=localOperation('linearize',cfg,request);indices=qp.stateIndices(:);
    delta=zeros(size(qp.equality,2),1);delta(qp.policyIndices(1,1))=1e-5;
    delta(qp.policyIndices(11,1))=2e-5;
    delta(indices)=qp.equality(:,indices)\(-qp.equality*delta);
    request.policies(1).v(1)=request.policies(1).v(1)+1e-5;
    request.policies(1).B(1,1)=request.policies(1).B(1,1)+2e-5;
    trace=localOperation('predict',cfg,request);
    changes=reshape(delta(indices),10,[]);
    stateError=max(abs(trace{1}.ego(:,2:end)-qp.nominal{1}.ego(:,2:end)-changes(1:6,:)),[],'all');
    memoryError=max(abs(trace{1}.memory(:,2:end)-qp.nominal{1}.memory(:,2:end)-changes(7:8,:)),[],'all');
end

function [flag,step]=localBrakeStep(qp,brakeIncrement)
    lower=qp.lower;upper=qp.upper;index=qp.policyIndices(2,1);
    lower(index)=max(lower(index),brakeIncrement);
    upper(index)=min(upper(index),brakeIncrement);
    options=optimoptions('linprog','Display','none','ConstraintTolerance',1e-9);
    [step,~,flag]=linprog(zeros(size(lower)),qp.inequality,qp.bounds, ...
        qp.equality,zeros(size(qp.equality,1),1),lower,upper,options);
end
