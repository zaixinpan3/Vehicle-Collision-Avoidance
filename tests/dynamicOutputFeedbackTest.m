classdef dynamicOutputFeedbackTest < matlab.unittest.TestCase
    % Causal raw-block inference, policy execution, and physical backup.
    properties (Access=private)
        cfg
        hypothesis
        block
        state
        prediction
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
            testCase.verifyTrue(prediction.metadata.shiftedPlanRevalidated);
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
        function aContinuingTargetNeedsAnotherTerminalCertificate(testCase)
            [cfg,hypothesis,block]=localFixture();hypothesis.target(4)=1;
            block.initialHypotheses=hypothesis;
            block.samples=localOperation('sensor',cfg,struct('hypothesis',hypothesis,'input',block.heldInput,'time',0));
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],cfg,[]), ...
                'collisionAvoidanceController:unsupportedTerminalMode');
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
        function nonlinearAcceptedScenariosDoNotClaimUniversalSafety(testCase)
            metadata=testCase.prediction.metadata;
            testCase.verifyTrue(metadata.nonlinearValidationPerformed);
            testCase.verifyFalse(metadata.robustNonlinearSafetyCertified);
            testCase.verifyFalse(metadata.recursiveFeasibilityCertified);
            testCase.verifyLessThanOrEqual(metadata.maximumConstraintViolation,testCase.cfg.solver.constraintTolerance);
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
        function anInterruptedImprovementUsesOnlyARevalidatedShift(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.maximumSolveSeconds=1e-12;
            [~,~,~,prior]=collisionAvoidanceController(testCase.block,[],[],cfg,[]);
            block=localNextBlock(prior,cfg);
            [~,~,prediction,next]=collisionAvoidanceController(block,[],[],cfg,prior);
            testCase.verifyTrue(prediction.metadata.usedRevalidatedFallback);
            testCase.verifyEqual(prediction.metadata.sqpIterations,0);
            testCase.verifyEqual(next.committedPolicy,prior.policies(2));
        end
        function overlappingHypothesesCannotExecuteAnUncheckedPlan(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.maximumSolveSeconds=1e-12;
            hypothesis=testCase.hypothesis;hypothesis.target(1:2)=hypothesis.ego(1:2);
            block=testCase.block;block.initialHypotheses=hypothesis;
            block.samples=localOperation('sensor',cfg,struct('hypothesis',hypothesis,'input',block.heldInput,'time',0));
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],cfg,[]), ...
                'collisionAvoidanceController:noOutputFeedbackPlan');
        end
        function theDefaultCampaignUsesRawSensorsWithoutTheNrmmPipeline(testCase)
            fixture=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            campaign=runCollisionThreatValidation(Frames=1,Speeds=1.5,Scenarios="stoppedLead", ...
                ControllerConfiguration=testCase.cfg,OutputDirectory=string(fixture.Folder));
            testCase.verifyEqual(campaign.pipeline,"dynamicOutputFeedback");
            testCase.verifyEqual(campaign.results.observationModel,"rawSensorJointDynamicOutputFeedback");
            testCase.verifyEqual(campaign.results.executedFrames,1);
            testCase.verifyEqual(campaign.results.outcome,"completed");
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
        'outputFeedback',struct('noiseDirections',zeros(7,1),'maximumSolveSeconds',120)));
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
