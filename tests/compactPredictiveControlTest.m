classdef compactPredictiveControlTest < matlab.unittest.TestCase
    % Direct-input planning with fixed inference and dense policy replay.
    properties (Access=private)
        cfg
        block
        command
        prediction
        state
    end
    properties (TestParameter)
        rectangleYaw={0,.2,pi/4,pi/2}
        refinedHolds={24,60}
        requestedSpeed={4,12}
        integrationMesh={4,64}
        movingTarget={false,true}
    end
    methods (TestClassSetup)
        function prepareController(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            setupJointNlpSolver;
            testCase.cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
                'model',struct('speedMaximum',16,'lateralVelocityMaximum',1), ...
                'outputFeedback',struct('method',"compactNlp",'nlpJit',false,'maximumSolveSeconds',1)));
            testCase.block=localBlock(testCase.cfg,8);
            [testCase.command,~,testCase.prediction,testCase.state]= ...
                collisionAvoidanceController(testCase.block,[],[],testCase.cfg,[]);
        end
    end
    methods (Test)
        function aSlowEncounterReceivesSpatialPreviewDirectly(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=1.5;
            item=testCase.block.initialHypotheses;item.ego(4)=1.5;
            item.target=[12;0;pi/2;0;0;0;1.6;2.4;.95;0;0];
            window=solvePredictiveControl('outputFeedback',struct('operation',"window",'cfg',cfg,'hypotheses',item));
            radius=hypot(cfg.vehicle.length/2,cfg.vehicle.width/2)+norm(cfg.vehicle.rectangleOffset) ...
                +norm(item.target(8:9))+norm(item.target(10:11));
            testCase.verifyGreaterThanOrEqual(window.allocatedSeconds*cfg.referenceSpeed,2*radius);
            testCase.verifyGreaterThan(window.allocatedSeconds,cfg.outputFeedback.minimumPreviewSeconds);
            testCase.verifyFalse(window.feasibilityCertified);
        end
        function neutralRestExposesTheOneSidedStartingDirection(testCase,integrationMesh)
            cfg=testCase.cfg;cfg.referenceSpeed=1.5;
            cfg.nonlinear.integrationStep=cfg.controller.sampleTime/integrationMesh;
            item=testCase.block.initialHypotheses;item.ego=zeros(6,1);item.target=[];
            request=struct('operation',"evaluateControlSequence",'cfg',cfg, ...
                'hypotheses',item,'input',zeros(2,1),'holds',[1,2,2],'inputs',zeros(2,3));
            value=solvePredictiveControl('outputFeedback',request);
            step=1e-7;request.inputs(2,:)=step;
            moved=solvePredictiveControl('outputFeedback',request);
            testCase.verifyLessThan(sum(value.costGradient(2:2:end)),0);
            testCase.verifyEqual(sum(value.costGradient(2:2:end)), ...
                (moved.cost-value.cost)/step,AbsTol=1e-5);
        end
        function aSafeContinuationCannotLosePerformanceToCompression(testCase)
            block=localNextBlock(testCase.state,testCase.cfg);
            [~,~,prediction]=collisionAvoidanceController(block,[],[],testCase.cfg,testCase.state);
            info=prediction.metadata;
            testCase.verifyTrue(isfinite(info.inheritedPerformanceCost));
            testCase.verifyLessThanOrEqual(info.acceptedPerformanceCost,info.inheritedPerformanceCost+1e-10);
        end
        function aShrinkingPreviewRetainsTheExecutableSuffixAndTerminalPoint(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=1.5;cfg.outputFeedback.maximumSolveSeconds=1e-12;
            block=localBlock(cfg,10.01);
            [~,~,first,prior]=collisionAvoidanceController(block,[],[],cfg,[]);
            block=localNextBlock(prior,cfg);
            [command,~,prediction,state]=collisionAvoidanceController(block,[],[],cfg,prior);
            testCase.verifyEqual(state.horizonSteps,prior.horizonSteps-1);
            testCase.verifyEqual(state.fineControls,prior.fineControls(:,2:end),AbsTol=1e-12);
            testCase.verifyEqual(command.actuatorInput,prior.fineControls(:,2),AbsTol=1e-12);
            testCase.verifyEqual(prediction.scenarios{1}.ego(:,end),first.scenarios{1}.ego(:,end),AbsTol=1e-5);
            testCase.verifyTrue(prediction.metadata.retainedFinePlanUsed);
            testCase.verifyTrue(prediction.scenarios{1}.terminal.member);
            testCase.verifyEqual(prediction.metadata.pcbfValue,0);
            testCase.verifyEqual(prediction.metadata.nonlinearOracleEvaluations,0);
        end
        function shrinkingBelowTheKnotCountKeepsPositiveControlIntervals(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=1.5;cfg.outputFeedback.maximumSolveSeconds=1e-12;
            cfg.outputFeedback.minimumPreviewSeconds=.1;
            cfg.outputFeedback.previewSettlingSeconds=.05;cfg.outputFeedback.horizonResolutionSeconds=.05;
            block=localBlock(cfg,2.01);
            [~,~,~,prior]=collisionAvoidanceController(block,[],[],cfg,[]);
            initialCount=prior.horizonSteps;
            for j=1:initialCount
                block=localNextBlock(prior,cfg);
                [~,~,prediction,state]=collisionAvoidanceController(block,[],[],cfg,prior);
                testCase.verifyEqual(state.horizonSteps,max(2,prior.horizonSteps-1));
                testCase.verifyGreaterThanOrEqual(diff(state.boundaries),ones(1,size(state.controls,2)));
                testCase.verifyLessThanOrEqual(size(state.controls,2),state.horizonSteps);
                testCase.verifyTrue(prediction.scenarios{1}.terminal.member);
                testCase.verifyEqual(prediction.metadata.pcbfValue,0);
                prior=state;
            end
            testCase.verifyEqual(state.horizonSteps,2);
        end
        function releasingBrakesWithoutProgressCannotReplaceTheRetainedPlan(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=1.5;cfg.outputFeedback.maximumSolveSeconds=1e-12;
            block=localBlock(cfg,0);
            [~,~,~,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            expected=state.fineControls(:,2);
            block=localNextBlock(state,cfg);
            [command,~,prediction]=collisionAvoidanceController(block,[],[],cfg,state);
            testCase.verifyEqual(command.actuatorInput,expected,AbsTol=1e-12);
            testCase.verifyTrue(prediction.metadata.retainedFinePlanUsed);
            testCase.verifyEqual(prediction.metadata.performanceAcceptedSteps,0);
            testCase.verifyEqual(prediction.metadata.nonlinearOracleEvaluations,0);
            testCase.verifyTrue(prediction.metadata.budgetStopped);
            testCase.verifyEqual(prediction.metadata.budgetScope,"completeWarmControllerCall");
        end
        function anOpenRoadRestStateCanResumeCruise(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=1.5;cfg.outputFeedback.nlpJit=true;
            cfg.outputFeedback.maximumSolveSeconds=2;
            block=localBlock(cfg,0);block.initialHypotheses.target=[];
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',block.initialHypotheses,'input',block.heldInput,'time',0));
            [command,~,prediction]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyGreaterThan(command.actuatorInput(2),.01);
            testCase.verifyGreaterThan(prediction.scenarios{1}.ego(4,end),1.4);
            testCase.verifyLessThanOrEqual(prediction.metadata.maximumConstraintViolation,1e-6);
        end
        function anExpiredCallBudgetCannotStartTheInitialOracle(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.maximumSolveSeconds=1e-12;
            block=localBlock(cfg,0);
            [command,~,prediction]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyEqual(prediction.metadata.nonlinearOracleEvaluations,0);
            testCase.verifyTrue(prediction.metadata.rootTerminalPlanUsed);
            testCase.verifyTrue(prediction.metadata.budgetStopped);
            testCase.verifyEqual(prediction.metadata.pcbfValue,0);
            testCase.verifyEqual(prediction.metadata.callBudgetSeconds,cfg.outputFeedback.maximumSolveSeconds,AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(prediction.metadata.maximumConstraintViolation,cfg.outputFeedback.nlpTolerance);
            testCase.verifyEqual(command.actuatorInput,prediction.scenarios{1}.inputs(:,1));
        end
        function aDeferredTrialCannotReplaceThePlanWithoutImprovement(testCase)
            state=testCase.state;
            state.deferredFineControls=repmat([testCase.cfg.outputFeedback.steeringMaximum;-1],1,state.horizonSteps);
            block=localNextBlock(state,testCase.cfg);
            [command,inputs,prediction]=collisionAvoidanceController(block,[],[],testCase.cfg,state);
            testCase.verifyFalse(prediction.metadata.deferredTrialAccepted);
            testCase.verifyLessThanOrEqual(prediction.metadata.acceptedPerformanceCost, ...
                prediction.metadata.inheritedPerformanceCost+1e-10);
            testCase.verifyGreaterThan(norm(command.actuatorInput-state.deferredFineControls(:,2)),1e-3);
            replay=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',prediction.information,'input',block.heldInput,'inputs',inputs));
            testCase.verifyTrue(replay.accepted);
        end
        function aCheckedDeferredImprovementCanResumeFromTheRetainedStop(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=1.5;cfg.outputFeedback.nlpJit=true;
            cfg.outputFeedback.maximumSolveSeconds=2;
            block=localBlock(cfg,0);block.initialHypotheses.target=[];
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',block.initialHypotheses,'input',block.heldInput,'time',0));
            [~,~,~,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            state.deferredFineControls=state.fineControls;
            backup=[0;-cfg.outputFeedback.backupBrakingRatio];
            state.fineControls=repmat(backup,1,state.horizonSteps);
            state.searchFineControls=state.fineControls;
            state.controls=repmat(backup,1,size(state.controls,2));state.appliedInput=backup;
            block=localNextBlock(state,cfg);
            [command,~,prediction]=collisionAvoidanceController(block,[],[],cfg,state);
            testCase.verifyTrue(prediction.metadata.deferredTrialAccepted);
            testCase.verifyLessThan(prediction.metadata.acceptedPerformanceCost,prediction.metadata.inheritedPerformanceCost);
            testCase.verifyGreaterThan(command.actuatorInput(2),0);
            testCase.verifyLessThanOrEqual(prediction.metadata.maximumConstraintViolation,cfg.outputFeedback.nlpTolerance);
            testCase.verifyEqual(prediction.metadata.pcbfValue,0);
            testCase.verifyTrue(prediction.scenarios{1}.terminal.member);
        end
        function anExpiredAdmissionCannotAuthorizeAnUncheckedSeed(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.initializationSolveSeconds=1e-12;
            block=localBlock(cfg,0);item=block.initialHypotheses;
            item.target=[0;0;0;0;0;0;1.6;2.4;.95;0;0];block.initialHypotheses=item;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',0));
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],cfg,[]), ...
                'collisionAvoidanceController:compactNoFeasiblePlan');
        end
        function theDefaultOptimizesDirectInputsWithoutMemoryMaps(testCase)
            cfg=collisionAvoidanceControllerConfig();
            testCase.verifyEqual(cfg.outputFeedback.method,"compactNlp");
            testCase.verifyEmpty(testCase.state.memory);
            testCase.verifyFalse(isfield(testCase.state,'committedPolicy'));
            testCase.verifyTrue(testCase.prediction.metadata.nonlinearValidationPerformed);
            testCase.verifyFalse(testCase.prediction.metadata.robustNonlinearSafetyCertified);
            testCase.verifyEqual(testCase.prediction.metadata.nlpVariables, ...
                2*testCase.prediction.metadata.controlNodes+testCase.prediction.metadata.horizonSteps);
        end
        function previewGrowthReusesTheCompiledProblem(testCase)
            block=localBlock(testCase.cfg,12);
            [~,~,prediction]=collisionAvoidanceController(block,[],[],testCase.cfg,[]);
            testCase.verifyGreaterThan(prediction.metadata.predictionWindow.allocatedSeconds, ...
                testCase.prediction.metadata.predictionWindow.allocatedSeconds);
            testCase.verifyEqual(prediction.metadata.controlNodes,testCase.prediction.metadata.controlNodes);
            testCase.verifyTrue(prediction.metadata.nlpCacheHit);
        end
        function everyAppliedHoldUsesThePredictedCommand(testCase)
            u=testCase.prediction.scenarios{1}.inputs;
            testCase.verifyEqual(testCase.command.actuatorInput,u(:,1),AbsTol=1e-12);
            testCase.verifyEqual(size(u,2),testCase.prediction.metadata.horizonSteps);
            testCase.verifyEqual(size(testCase.prediction.scenarios{1}.denseEgo,2), ...
                testCase.cfg.outputFeedback.validationSubsteps*testCase.prediction.metadata.horizonSteps+1);
        end
        function finiteActuatorRatesRemainBoundedInBothDirections(testCase,requestedSpeed,integrationMesh)
            cfg=testCase.cfg;cfg.referenceSpeed=requestedSpeed;
            cfg.nonlinear.integrationStep=cfg.controller.sampleTime/integrationMesh;
            cfg.outputFeedback.steeringRateMaximum=1;
            cfg.model.brakingRatioRateMaximum=.8;
            block=localBlock(cfg,8);
            [~,~,prediction]=collisionAvoidanceController(block,[],[],cfg,[]);
            u=prediction.scenarios{1}.inputs;
            rates=diff([block.heldInput,u],1,2)/cfg.controller.sampleTime;
            limits=[cfg.outputFeedback.steeringRateMaximum;cfg.model.brakingRatioRateMaximum];
            testCase.verifyLessThanOrEqual(max(rates,[],2),limits+1e-8);
            testCase.verifyGreaterThanOrEqual(min(rates,[],2),-limits-1e-8);
            states=prediction.scenarios{1}.denseEgo(4:6,:);
            testCase.verifyGreaterThanOrEqual(min(states(1,:)), -1e-6);
        end
        function defaultReplayAllowsSteeringChangesWithoutAnActuatorRateContract(testCase)
            item=testCase.block.initialHypotheses;item.ego(4:6)=0;
            inputs=[.5,-.5;-1,-1];
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',item,'input',zeros(2,1),'inputs',inputs));
            testCase.verifyTrue(result.accepted);
            testCase.verifyEqual(result.trajectories{1}.inputs,inputs,AbsTol=1e-14);
        end
        function replayRejectsChangesExceedingAnExplicitSteeringRate(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.steeringRateMaximum=1;
            item=testCase.block.initialHypotheses;item.ego(4:6)=0;
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',cfg,'hypotheses',item,'input',zeros(2,1),'inputs',[.5,-.5;-1,-1]));
            testCase.verifyFalse(result.accepted);
            testCase.verifyGreaterThan(result.maximumViolation,.4);
        end
        function removingSteeringSlewPreservesAngleBounds(testCase)
            item=testCase.block.initialHypotheses;item.ego(4:6)=0;
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',item,'input',zeros(2,1),'inputs',[.7;-1]));
            testCase.verifyFalse(result.accepted);
            testCase.verifyGreaterThan(result.maximumViolation,.08);
        end
        function diagonalClearanceUsesTheFullDistanceDual(testCase)
            item=testCase.block.initialHypotheses;item.ego(:)=0;
            item.target=[4.88;1.98;0;0;0;0;1.6;2.4;.95;0;0];
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',item,'input',zeros(2,1),'inputs',[0;-1]));
            testCase.verifyTrue(result.accepted);
        end
        function stabilityStateCapsDoNotRejectCollisionFreeMotion(testCase)
            cfg=testCase.cfg;cfg.model.speedMaximum=10;cfg.model.lateralVelocityMaximum=.1;cfg.model.yawRateMaximum=.1;
            item=testCase.block.initialHypotheses;item.ego(4:6)=[19;.2;.2];
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',cfg,'hypotheses',item,'input',zeros(2,1),'inputs',[0;0]));
            testCase.verifyTrue(result.accepted);
        end
        function symbolicTireClippingCannotAuthorizeAnUndefinedPhysicalFlow(testCase)
            item=testCase.block.initialHypotheses;item.ego=[0;0;0;4;-10;0];
            input=[testCase.cfg.outputFeedback.steeringMaximum;-1];
            testCase.verifyError(@()nonlinearBicycleModel.derivative(item.ego,input,testCase.cfg), ...
                'collisionAvoidanceController:invalidTireOperatingPoint');
            request=struct('operation',"validateInputSequence",'cfg',testCase.cfg, ...
                'hypotheses',item,'input',input,'inputs',input);
            value=solvePredictiveControl('outputFeedback',request);
            testCase.verifyFalse(value.accepted);
            request.operation="evaluateControlSequence";request.holds=1;
            value=solvePredictiveControl('outputFeedback',request);
            testCase.verifyGreaterThan(value.forward,1);
            step=1e-6;plus=request;minus=request;
            plus.inputs(1)=plus.inputs(1)+step;minus.inputs(1)=minus.inputs(1)-step;
            p=solvePredictiveControl('outputFeedback',plus);m=solvePredictiveControl('outputFeedback',minus);
            testCase.verifyEqual(value.forwardGradient(1),(p.forward-m.forward)/(2*step),AbsTol=1e-4);
        end
        function continuationReplaysTheUpdatedPhysicalState(testCase)
            block=localNextBlock(testCase.state,testCase.cfg);
            [command,~,prediction,state]=collisionAvoidanceController(block,[],[],testCase.cfg,testCase.state);
            expected=localAdvanceHold(state.hypotheses(1),command.actuatorInput,testCase.cfg);
            testCase.verifyEqual(prediction.scenarios{1}.ego(:,2),expected.ego,AbsTol=1e-5);
            testCase.verifyTrue(prediction.metadata.nlpCacheHit);
        end
        function inferenceResolvesTheDynamicToLowSpeedTransition(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.nlpJit=true;
            item=testCase.block.initialHypotheses;
            item.ego=[0;0;-1.7876160012;4.5485240535;9.0552937408;-3.1554270904];
            input=[-.6106706604;.8325661619];
            exact=ode45(@(~,x)nonlinearBicycleModel.derivative(x,input,cfg), ...
                [0,cfg.controller.sampleTime],item.ego,odeset('RelTol',1e-11,'AbsTol',1e-13));
            count=round(cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod);
            for j=1:count
                item=solvePredictiveControl('outputFeedback',struct('operation',"advance", ...
                    'cfg',cfg,'hypothesis',item,'input',input,'duration',cfg.outputFeedback.sensorPeriod,'endOfHold',j==count));
                reference=deval(exact,j*cfg.outputFeedback.sensorPeriod);
                if j==count,reference=nonlinearBicycleModel.sampleEndState(reference,input,cfg);end
                testCase.verifyEqual(item.ego,reference,AbsTol=1e-5);
                actual=nonlinearBicycleModel.derivative(item.ego,input,cfg);
                expected=nonlinearBicycleModel.derivative(reference,input,cfg);
                testCase.verifyEqual(actual(4:6),expected(4:6),AbsTol=1e-4);
            end
        end
        function endpointClearanceCannotReplaceTerminalMembership(testCase)
            item=testCase.block.initialHypotheses;item.ego(:)=0;
            item.target=[100;0;pi;5;0;0;1.6;2.4;.95;0;0];
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',item,'input',zeros(2,1),'inputs',[0;0]));
            testCase.verifyFalse(result.accepted);
            testCase.verifyFalse(result.trajectories{1}.terminal.member);
        end
        function everyReturnedPlanHasAnExplicitTerminalContinuation(testCase)
            testCase.verifyTrue(testCase.prediction.metadata.terminalSetEnforced);
            testCase.verifyTrue(testCase.state.terminal.member);
            testCase.verifyTrue(testCase.state.terminal.safetyAssumed);
            testCase.verifyEqual(testCase.state.terminal.input, ...
                terminalSafeSet.compactControl(testCase.state.fineControls(:,end),testCase.cfg));
        end
        function theTerminalBackupStraightensAndReachesDeclaredRest(testCase)
            item=testCase.block.initialHypotheses;item.ego=[0;0;.1;3;.4;.2];
            request=struct('operation',"validateInputSequence",'cfg',testCase.cfg, ...
                'hypotheses',item,'input',[.2;0],'inputs',[.2;0]);
            value=solvePredictiveControl('outputFeedback',request);
            testCase.verifyTrue(value.accepted);
            terminal=value.trajectories{1}.terminal;
            testCase.verifyEqual(terminal.input(1),0);
            testCase.verifyGreaterThan(terminal.backupHolds,0);
            testCase.verifyEqual(terminal.model,"straighteningBrakingThenParked");
            testCase.verifyTrue(isfinite(terminal.stopSeconds));
        end
        function terminalEntryHonorsAnExplicitSteeringSlewContract(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.steeringRateMaximum=.1;
            item=testCase.block.initialHypotheses;item.ego(4:6)=0;
            request=struct('operation',"validateInputSequence",'cfg',cfg, ...
                'hypotheses',item,'input',[.4;0],'inputs',[.4;0]);
            value=solvePredictiveControl('outputFeedback',request);
            testCase.verifyFalse(value.accepted);
            testCase.verifyGreaterThan(value.trajectories{1}.terminal.value,.39);
        end
        function anUnsafeWarmIterateCannotOverwriteTheExecutablePlan(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=3;cfg.outputFeedback.maximumSolveSeconds=1e-9;
            block=localBlock(cfg,0);item=block.initialHypotheses;
            item.target=[8;0;0;0;0;0;1.6;2.4;.95;0;0];block.initialHypotheses=item;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',0));
            [~,~,~,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            expected=state.fineControls(:,2);
            state.searchFineControls=repmat([0;1],1,state.horizonSteps);
            block=localNextBlock(state,cfg);
            [command,~,prediction]=collisionAvoidanceController(block,[],[],cfg,state);
            testCase.verifyEqual(command.actuatorInput,expected,AbsTol=1e-12);
            testCase.verifyTrue(prediction.metadata.retainedFinePlanUsed);
            testCase.verifyEqual(prediction.metadata.pcbfValue,0);
            testCase.verifyLessThanOrEqual(prediction.metadata.maximumConstraintViolation,1e-6);
        end
        function aMissingRawReadingIsRejected(testCase)
            block=localNextBlock(testCase.state,testCase.cfg);block.samples(2)=[];
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],testCase.cfg,testCase.state), ...
                'collisionAvoidanceController:invalidSensorGrid');
        end
        function anInconsistentMeasurementDoesNotResetTheEstimate(testCase)
            block=testCase.block;block.samples.position(1)=block.samples.position(1)+1;
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],testCase.cfg,[]), ...
                'collisionAvoidanceController:emptyJointInformation');
        end
        function optimizedMemorySeedsCannotBeSilentlyIgnored(testCase)
            block=testCase.block;block.initialMemory=0;
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],testCase.cfg,[]), ...
                'collisionAvoidanceController:unsupportedCompactMemory');
        end
        function theMeasuredInputMustMatchTheCommittedCommand(testCase)
            block=localNextBlock(testCase.state,testCase.cfg);block.heldInput(1)=block.heldInput(1)+.01;
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],testCase.cfg,testCase.state), ...
                'collisionAvoidanceController:actuationMismatch');
        end
        function thePredictionMatchesSensorPeriodPropagation(testCase)
            block=localNextBlock(testCase.state,testCase.cfg);
            item=testCase.state.hypotheses(1);
            item=localAdvanceHold(item,testCase.state.appliedInput,testCase.cfg);
            testCase.verifyEqual(testCase.prediction.scenarios{1}.ego(:,2),item.ego,AbsTol=1e-5);
            testCase.verifyEqual(block.time,testCase.cfg.controller.sampleTime,AbsTol=1e-12);
        end
        function aFinePlanIsValidatedWithoutCoarseReinterpolation(testCase)
            inputs=repmat([.025,-.025;-.1,-.1],1,30);
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',testCase.block.initialHypotheses, ...
                'input',testCase.block.heldInput,'inputs',inputs));
            expected=localRefinedSequence(testCase.block.initialHypotheses.ego,inputs,testCase.cfg);
            testCase.verifyTrue(result.accepted);
            testCase.verifyEqual(result.trajectories{1}.inputs,inputs,AbsTol=1e-14);
            testCase.verifyEqual(result.trajectories{1}.ego(:,end),[expected(1:3,end);expected(4:6,end)],AbsTol=1e-10);
        end
        function anUnsafeSequenceCannotPassReplayAcceptance(testCase)
            item=testCase.block.initialHypotheses;item.target=[8;0;0;0;0;0;1.6;2.4;.95;0;0];
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',item,'input',zeros(2,1),'inputs',zeros(2,60)));
            testCase.verifyFalse(result.accepted);
            testCase.verifyGreaterThan(result.maximumViolation,0);
        end
        function restorationCanRepairAnUnsafeBrakingInitialization(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=3;
            cfg.outputFeedback.initializationSolveSeconds=10;cfg.outputFeedback.nlpJit=true;
            block=localBlock(cfg,3);item=block.initialHypotheses;
            item.target=[5.6;0;0;0;0;0;1.6;2.4;.95;0;0];
            block.initialHypotheses=item;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',0));
            request=struct('operation',"validateInputSequence",'cfg',cfg,'hypotheses',item, ...
                'input',block.heldInput,'inputs',repmat([0;-.6],1,60));
            unsafe=solvePredictiveControl('outputFeedback',request);
            request.inputs=repmat([0;-.8],1,60);
            witness=solvePredictiveControl('outputFeedback',request);
            testCase.assertFalse(unsafe.accepted);
            testCase.assertTrue(witness.accepted);
            [~,~,prediction]=collisionAvoidanceController(block,[],[],cfg,[]);
            info=prediction.metadata;
            testCase.verifyTrue(info.restorationAttempted);
            testCase.verifyGreaterThan(info.initialRestorationSlack,0);
            testCase.verifyGreaterThan(info.initialNlpViolation,0);
            testCase.verifyTrue(info.terminalSetEnforced);
            testCase.verifyLessThanOrEqual(info.terminalValue,1e-6);
            testCase.verifyGreaterThanOrEqual(info.nlpAttempts,1);
            testCase.verifyEqual(info.pcbfValue,0);
            testCase.verifyTrue(info.primaryOptimumComputed);
            testCase.verifyEqual(info.initializationTrajectoryCount,1);
            inputs=prediction.scenarios{1}.inputs;
            testCase.verifyLessThanOrEqual(max(abs(inputs(1,:))),cfg.outputFeedback.steeringMaximum);
            testCase.verifyLessThanOrEqual(max(abs(inputs(2,:))),1);
            request.inputs=prediction.scenarios{1}.inputs;
            result=solvePredictiveControl('outputFeedback',request);
            testCase.verifyTrue(result.accepted);
            testCase.verifyLessThanOrEqual(result.maximumViolation,1e-6);
        end
        function condensedGradientsDifferentiateTheActualConstraintValues(testCase,integrationMesh)
            cfg=testCase.cfg;cfg.outputFeedback.nlpJit=true;
            cfg.nonlinear.integrationStep=cfg.controller.sampleTime/integrationMesh;
            item=testCase.block.initialHypotheses;item.ego=[0;0;.12;4;.1;.03];
            item.target=[8;3;.8;2;0;.04;1.6;2.4;.95;0;0];
            request=struct('operation',"evaluateControlSequence",'cfg',cfg, ...
                'hypotheses',item,'input',[0;0],'holds',[1,2,2],'inputs',[.12,.1,.08;.1,.12,.1]);
            value=solvePredictiveControl('outputFeedback',request);
            direction=[.2;-.3;.1;.2;-.15;.4];step=1e-5;
            plus=request;minus=request;
            plus.inputs=request.inputs+step*reshape(direction,2,[]);
            minus.inputs=request.inputs-step*reshape(direction,2,[]);
            p=solvePredictiveControl('outputFeedback',plus);m=solvePredictiveControl('outputFeedback',minus);
            gradient=[value.stageGradient,value.forwardGradient,value.terminalGradient].'*direction;
            difference=([p.stage;p.forward;p.terminal]-[m.stage;m.forward;m.terminal])/(2*step);
            testCase.verifyEqual(gradient,difference,AbsTol=2e-4);
            testCase.verifyEqual(value.costGradient.'*direction,(p.cost-m.cost)/(2*step),AbsTol=2e-4);
        end
        function finePredictionsAgreeWithFreshReplayAcrossFrames(testCase,movingTarget)
            cfg=testCase.cfg;cfg.referenceSpeed=3;cfg.outputFeedback.nlpJit=true;
            cfg.nonlinear.integrationStep=cfg.controller.sampleTime/cfg.outputFeedback.validationSubsteps;
            block=localBlock(cfg,2);item=block.initialHypotheses;
            if movingTarget,item.target=[35;15;pi/2;2;0;.04;1.6;2.4;.95;0;0];end
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',0));
            other=item;other.ego(1)=other.ego(1)+.005;
            block.initialHypotheses=[item,other];state=[];
            for k=1:2
                [~,inputs,prediction,state]=collisionAvoidanceController(block,[],[],cfg,state);
                replay=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                    'cfg',cfg,'hypotheses',state.hypotheses,'input',block.heldInput,'inputs',inputs));
                testCase.verifyTrue(replay.accepted);
                testCase.verifyEqual(numel(prediction.scenarios),2);
                testCase.verifyEqual(replay.maximumViolation,prediction.metadata.maximumConstraintViolation,AbsTol=1e-12);
                for s=1:numel(prediction.scenarios)
                    actual=prediction.scenarios{s};expected=replay.trajectories{s};
                    testCase.verifyEqual(actual.denseEgo,expected.denseEgo,AbsTol=1e-12);
                    testCase.verifyEqual(actual.ego,expected.ego,AbsTol=1e-12);
                    testCase.verifyEqual(actual.targets,expected.targets,AbsTol=1e-12);
                    testCase.verifyEqual(actual.terminal,expected.terminal);
                    testCase.verifyTrue(actual.terminal.member);
                end
                block=localNextBlock(state,cfg);
            end
        end
        function nonuniformInputKnotsPreserveDirectionalDerivatives(testCase,integrationMesh,movingTarget)
            cfg=testCase.cfg;cfg.outputFeedback.nlpJit=true;
            cfg.nonlinear.integrationStep=cfg.controller.sampleTime/integrationMesh;
            item=testCase.block.initialHypotheses;item.ego=[0;0;.12;3.3;.08;.03];
            item.target=[];
            if movingTarget,item.target=[12;4;.8;2;0;.04;1.6;2.4;.95;0;0];end
            other=item;other.ego(2)=other.ego(2)+.03;other.ego(4)=other.ego(4)+.04;
            other.parameters(3:4)=.98;
            request=struct('operation',"evaluateControlSequence",'cfg',cfg, ...
                'hypotheses',[item,other],'input',zeros(2,1),'holds',[1,2,4,3,6], ...
                'inputs',[.02,.07,.1,.04,-.01;-.05,.08,.12,.02,-.04]);
            value=solvePredictiveControl('outputFeedback',request);
            direction=[.2;-.3;.1;.2;-.15;.4;.12;-.1;-.2;.05];step=1e-6;
            plus=request;minus=request;
            plus.inputs=request.inputs+step*reshape(direction,2,[]);
            minus.inputs=request.inputs-step*reshape(direction,2,[]);
            p=solvePredictiveControl('outputFeedback',plus);m=solvePredictiveControl('outputFeedback',minus);
            actual=[value.stageGradient,value.forwardGradient,value.terminalGradient].'*direction;
            expected=([p.stage;p.forward;p.terminal]-[m.stage;m.forward;m.terminal])/(2*step);
            testCase.verifyEqual(actual,expected,AbsTol=3e-4);
            testCase.verifyEqual(value.costGradient.'*direction,(p.cost-m.cost)/(2*step),AbsTol=3e-4);
            testCase.verifyEqual(value.forwardGradient(3:end,1),zeros(8,1));
            testCase.verifyGreaterThan(abs(value.forwardGradient(2,1)),1e-3);
        end
        function nativeDerivativesAgreeAcrossChangingHorizons(testCase,integrationMesh,movingTarget)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.assumeTrue(isfile(fullfile(root,'solver','controller',['condensedSensitivityKernelMex.',mexext])), ...
                'Run buildControllerKernels to test the optional native sensitivity implementation.');
            cfg=testCase.cfg;cfg.nonlinear.integrationStep=cfg.controller.sampleTime/integrationMesh;
            item=testCase.block.initialHypotheses;item.ego=[0;0;.12;3.3;.08;.03];item.target=[];
            if movingTarget,item.target=[12;4;.8;2;0;.04;1.6;2.4;.95;0;0];end
            other=item;other.ego(2)=other.ego(2)+.03;other.parameters(3:4)=.98;
            request=struct('operation',"evaluateControlSequence",'cfg',cfg, ...
                'hypotheses',[item,other],'input',zeros(2,1),'holds',[1,2,4,3,6], ...
                'inputs',[.02,.07,.1,.04,-.01;-.05,.08,.12,.02,-.04]);
            fields={'stage','forward','terminal','cost','stageGradient','forwardGradient', ...
                'terminalGradient','costGradient','costHessian'};
            windows={[1,2,4,3,6],[1,2,61,3,60],[2,1,1,2,1]};
            for i=1:numel(windows)
                request.holds=windows{i};request.cfg.outputFeedback.nlpJit=false;
                interpreted=solvePredictiveControl('outputFeedback',request);
                request.cfg.outputFeedback.nlpJit=true;
                native=solvePredictiveControl('outputFeedback',request);
                for j=1:numel(fields)
                    testCase.verifyEqual(native.(fields{j}),interpreted.(fields{j}),AbsTol=1e-9,RelTol=1e-12);
                end
            end
        end
        function denseStopSamplesKeepTheirPreResetSensitivity(testCase,integrationMesh)
            cfg=testCase.cfg;cfg.outputFeedback.nlpJit=true;
            cfg.nonlinear.integrationStep=cfg.controller.sampleTime/integrationMesh;
            item=testCase.block.initialHypotheses;item.ego=[0;0;0;.051;0;0];
            item.target=[8;3;.7;2;0;.02;1.6;2.4;.95;0;0];input=[0;-.1];
            stopped=solvePredictiveControl('outputFeedback',struct('operation',"advance",'cfg',cfg, ...
                'hypothesis',item,'input',input,'duration',cfg.controller.sampleTime,'endOfHold',true));
            testCase.assertEqual(stopped.ego(4:6),zeros(3,1));
            request=struct('operation',"evaluateControlSequence",'cfg',cfg, ...
                'hypotheses',item,'input',input,'holds',1,'inputs',input);
            value=solvePredictiveControl('outputFeedback',request);step=1e-6;
            plus=request;minus=request;plus.inputs(2)=input(2)+step;minus.inputs(2)=input(2)-step;
            p=solvePredictiveControl('outputFeedback',plus);m=solvePredictiveControl('outputFeedback',minus);
            testCase.verifyGreaterThan(abs(value.forwardGradient(2)),1e-3);
            testCase.verifyEqual([value.stageGradient(2,:).';value.forwardGradient(2,:).';value.terminalGradient(2,:).'], ...
                ([p.stage;p.forward;p.terminal]-[m.stage;m.forward;m.terminal])/(2*step),AbsTol=3e-4);
        end
        function penetrationCannotDisappearIntoAZeroDualWitness(testCase)
            item=testCase.block.initialHypotheses;item.ego=[0;0;0;1;0;0];
            item.target=[3;.2;0;0;0;0;1.6;2.4;.95;0;0];
            request=struct('operation',"evaluateControlSequence",'cfg',testCase.cfg, ...
                'hypotheses',item,'input',[0;0],'holds',[1,1],'inputs',zeros(2,2));
            value=solvePredictiveControl('outputFeedback',request);
            testCase.verifyGreaterThan(min(value.stage),testCase.cfg.outputFeedback.clearanceMeters+1);
            testCase.verifyGreaterThan(norm(value.stageGradient,'fro'),1e-5);
        end
        function elasticRestorationCannotAuthorizeAnOverlappingTrajectory(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.initializationSolveSeconds=.1;
            block=localBlock(cfg,0);item=block.initialHypotheses;
            item.target=[0;0;0;0;0;0;1.6;2.4;.95;0;0];
            block.initialHypotheses=item;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',0));
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],cfg,[]), ...
                'collisionAvoidanceController:compactNoFeasiblePlan');
        end
        function optimizationFromRestRespectsPhysicalVelocityBounds(testCase)
            block=localBlock(testCase.cfg,0);
            [~,~,prediction]=collisionAvoidanceController(block,[],[],testCase.cfg,[]);
            trace=prediction.scenarios{1};
            testCase.verifyGreaterThanOrEqual(min(trace.denseEgo(4,:)),-1e-6);
            request=struct('operation',"validateInputSequence",'cfg',testCase.cfg, ...
                'hypotheses',block.initialHypotheses,'input',block.heldInput,'inputs',trace.inputs);
            replay=solvePredictiveControl('outputFeedback',request);
            testCase.verifyTrue(replay.accepted);
            testCase.verifyEqual(replay.trajectories{1}.denseEgo,trace.denseEgo,AbsTol=1e-10);
        end
        function anInconsistentMemberCannotBeDroppedDuringPlanValidation(testCase)
            safe=testCase.block.initialHypotheses;safe.target=[80;0;0;0;0;0;1.6;2.4;.95;0;0];
            unsafe=safe;unsafe.target(1)=8;
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',[safe,unsafe],'input',zeros(2,1),'inputs',zeros(2,60)));
            testCase.verifyFalse(result.accepted);
            testCase.verifyEqual(numel(result.trajectories),2);
        end
        function stationarySafeVehiclesDoNotNeedAMovingVehicleReserve(testCase)
            item=testCase.block.initialHypotheses;item.ego(4:6)=0;
            item.target=[4.95;0;0;0;0;0;1.6;2.4;.95;0;0];
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',item,'input',zeros(2,1),'inputs',repmat([0;-.1],1,60)));
            testCase.verifyTrue(result.accepted);
            testCase.verifyEqual(result.trajectories{1}.validationSubsteps,testCase.cfg.outputFeedback.validationSubsteps);
            testCase.verifyEqual(result.trajectories{1}.denseEgo(4:6,:), ...
                zeros(3,1+60*testCase.cfg.outputFeedback.validationSubsteps),AbsTol=1e-14);
        end
        function refinementPreservesTheMovingPrefixAndHoldBoundaryStop(testCase,refinedHolds)
            item=testCase.block.initialHypotheses;item.ego(4)=.5;
            item.target=[5.07;0;0;0;0;0;1.6;2.4;.95;0;0];
            inputs=repmat([0;-.8],1,refinedHolds);
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',item,'input',zeros(2,1),'inputs',inputs));
            expected=localRefinedSequence(item.ego,inputs,testCase.cfg);
            testCase.verifyTrue(result.accepted);
            testCase.verifyEqual(result.trajectories{1}.validationSubsteps,testCase.cfg.outputFeedback.validationSubsteps);
            testCase.verifyEqual(result.trajectories{1}.denseEgo,expected,AbsTol=1e-10);
            testCase.verifyGreaterThan(expected(1,end),item.ego(1));
            testCase.verifyEqual(result.trajectories{1}.ego(4:6,end),zeros(3,1),AbsTol=1e-14);
        end
        function collisionBetweenControlNodesDoesNotRejectThePlan(testCase)
            item=testCase.block.initialHypotheses;item.ego(4:6)=0;
            % The target crosses the stationary ego between two control
            % boundaries, then moves away from the parked terminal set.
            item.target=[0;-6;pi/2;480;0;0;1.6;2.4;.95;0;0];
            request=struct('operation',"validateInputSequence",'cfg',testCase.cfg, ...
                'hypotheses',item,'input',zeros(2,1),'inputs',zeros(2,1));
            result=solvePredictiveControl('outputFeedback',request);
            testCase.verifyTrue(result.accepted);
            testCase.verifySize(result.trajectories{1}.collisionEgo,[6,2]);
            testCase.verifyEqual(result.trajectories{1}.collisionTimes,[0,testCase.cfg.controller.sampleTime]);
            testCase.verifyEqual(result.trajectories{1}.validationSubsteps,testCase.cfg.outputFeedback.validationSubsteps);
            request.operation="evaluateControlSequence";request.holds=1;
            value=solvePredictiveControl('outputFeedback',request);
            testCase.verifyLessThanOrEqual(max(value.stage),0);
            crossing=predictiveSafetyGeometry.predictTarget(item.target,.0125);
            shape=[testCase.cfg.vehicle.length/2;testCase.cfg.vehicle.width/2;testCase.cfg.vehicle.rectangleOffset];
            testCase.verifyLessThanOrEqual(predictiveSafetyGeometry.rectangle(item.ego(1:3),shape,crossing(1:3),crossing(8:11)),0);
        end
        function collisionAtAnIntermediateControlNodeStillRejectsThePlan(testCase)
            item=testCase.block.initialHypotheses;item.ego(4:6)=0;
            item.target=[0;-12;pi/2;240;0;0;1.6;2.4;.95;0;0];
            request=struct('operation',"validateInputSequence",'cfg',testCase.cfg, ...
                'hypotheses',item,'input',zeros(2,1),'inputs',zeros(2,2));
            result=solvePredictiveControl('outputFeedback',request);
            testCase.verifyFalse(result.accepted);
            % A single control knot spans both holds. Its interior control
            % boundary must remain a collision node in the condensed oracle.
            request.operation="evaluateControlSequence";request.holds=2;request.inputs=zeros(2,1);
            value=solvePredictiveControl('outputFeedback',request);
            testCase.verifyGreaterThan(min(value.stage),testCase.cfg.outputFeedback.clearanceMeters);
        end
        function nodeSafetyScopeDoesNotClaimIntersampleProtection(testCase)
            info=testCase.prediction.metadata;
            testCase.verifyEqual(info.safetyScope,"finiteHypothesesControlBoundaryCollisionChecks");
            testCase.verifyEqual(info.wholeHoldReserveScope,"notAppliedNodeCollisionContract");
            testCase.verifyEqual(info.collisionNodeCount,info.horizonSteps+1);
            testCase.verifyEqual(info.collisionCheckPeriodSeconds,testCase.cfg.controller.sampleTime);
        end
        function aSeparationWitnessBoundsActualFootprintDistance(testCase,rectangleYaw)
            ego=[0;0;rectangleYaw];target=[7;.3;-.3];shape=[2.4;.95;.1;-.2];
            gap=predictiveSafetyGeometry.rectangleAxisGaps(ego,shape,target,shape);
            smooth=predictiveSafetyGeometry.rectangleAxisGaps(ego,shape,target,shape,1e-5);
            distance=predictiveSafetyGeometry.rectangleNumeric(ego,shape,target,shape);
            testCase.verifyGreaterThan(max(smooth),0);
            testCase.verifyLessThanOrEqual(smooth,gap+1e-12);
            testCase.verifyLessThanOrEqual(max(gap),distance+1e-10);
        end
        function contactCannotHaveAPositiveSeparationWitness(testCase)
            shape=[2.4;.95;0;0];
            gap=predictiveSafetyGeometry.rectangleAxisGaps([0;0;0],shape,[4.8;0;0],shape);
            testCase.verifyEqual(max(gap),0,AbsTol=1e-12);
        end
        function aCommonTranslationCancelsBeforeSeparation(testCase)
            shape=[2.4;.95;.1;-.2];a=[0;0;.2];b=[7;.3;-.3];shift=[1e4;-1e4;0];
            gap=predictiveSafetyGeometry.rectangleAxisGaps(a,shape,b,shape);
            shifted=predictiveSafetyGeometry.rectangleAxisGaps(a+shift,shape,b+shift,shape);
            testCase.verifyEqual(shifted,gap,AbsTol=1e-10);
        end
        function anAbsorbingStopPreservesLatentTargetParameters(testCase)
            block=localBlock(testCase.cfg,8);item=block.initialHypotheses;
            item.target=[30;0;.2;.01;-1;.04;1.6;2.4;.95;0;0];
            next=localAdvanceHold(item,[0;-.1],testCase.cfg);
            testCase.verifyEqual(next.target(4),0,AbsTol=1e-12);
            testCase.verifyEqual(next.target(5:6),item.target(5:6),AbsTol=0);
        end
    end
end

function item=localAdvanceHold(item,input,cfg)
    count=round(cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod);
    for j=1:count
        item=solvePredictiveControl('outputFeedback',struct('operation',"advance",'cfg',cfg, ...
            'hypothesis',item,'input',input,'duration',cfg.outputFeedback.sensorPeriod,'endOfHold',j==count));
    end
end

function item=localAdvanceSequence(item,inputs,cfg)
    for j=1:size(inputs,2),item=localAdvanceHold(item,inputs(:,j),cfg);end
end

function dense=localRefinedSequence(x,inputs,cfg)
    substeps=cfg.outputFeedback.validationSubsteps;dense=zeros(6,1+substeps*size(inputs,2));dense(:,1)=x;
    for i=1:size(inputs,2)
        for j=1:substeps
            x=nonlinearBicycleModel.sensorFlow(x,inputs(:,i),cfg,cfg.controller.sampleTime/substeps,false);
            dense(:,1+(i-1)*substeps+j)=x;
        end
        x=nonlinearBicycleModel.sampleEndState(x,inputs(:,i),cfg);
    end
end

function block=localBlock(cfg,speed)
    item=struct('ego',[0;0;0;speed;0;0],'target',[], ...
        'parameters',ones(4,1),'sensorBias',zeros(7,1));
    input=zeros(2,1);
    y=solvePredictiveControl('outputFeedback',struct('operation',"sensor",'cfg',cfg, ...
        'hypothesis',item,'input',input,'time',0));
    block=struct('kind',"jointSensorBlock-v1",'time',0,'heldInput',input, ...
        'samples',y,'initialHypotheses',item);
end

function block=localNextBlock(state,cfg)
    item=state.hypotheses(1);input=state.appliedInput;samples=struct([]);
    count=round(cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod);
    for j=1:count
        item=solvePredictiveControl('outputFeedback',struct('operation',"advance",'cfg',cfg, ...
            'hypothesis',item,'input',input,'duration',cfg.outputFeedback.sensorPeriod,'endOfHold',j==count));
        y=solvePredictiveControl('outputFeedback',struct('operation',"sensor",'cfg',cfg, ...
            'hypothesis',item,'input',input,'time',state.time+j*cfg.outputFeedback.sensorPeriod));
        if isempty(samples),samples=y;else,samples(end+1)=y;end %#ok<AGROW>
    end
    block=struct('kind',"jointSensorBlock-v1",'time',state.time+cfg.controller.sampleTime, ...
        'heldInput',input,'samples',samples);
end
