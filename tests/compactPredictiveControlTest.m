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
        function theDefaultOptimizesDirectInputsWithoutMemoryMaps(testCase)
            cfg=collisionAvoidanceControllerConfig();
            testCase.verifyEqual(cfg.outputFeedback.method,"compactNlp");
            testCase.verifyEmpty(testCase.state.memory);
            testCase.verifyFalse(isfield(testCase.state,'committedPolicy'));
            testCase.verifyTrue(testCase.prediction.metadata.nonlinearValidationPerformed);
            testCase.verifyFalse(testCase.prediction.metadata.robustNonlinearSafetyCertified);
            testCase.verifyEqual(testCase.prediction.metadata.nlpVariables, ...
                8*testCase.prediction.metadata.controlNodes+1);
        end
        function previewGrowthReusesTheCompiledProblem(testCase)
            block=localBlock(testCase.cfg,12);
            [~,~,prediction]=collisionAvoidanceController(block,[],[],testCase.cfg,[]);
            testCase.verifyGreaterThan(prediction.metadata.predictionWindow.allocatedSeconds, ...
                testCase.prediction.metadata.predictionWindow.allocatedSeconds);
            testCase.verifyEqual(prediction.metadata.nlpVariables,testCase.prediction.metadata.nlpVariables);
            testCase.verifyTrue(prediction.metadata.nlpCacheHit);
        end
        function everyAppliedHoldRespectsSteeringSlew(testCase)
            u=testCase.prediction.scenarios{1}.inputs;
            change=diff([testCase.block.heldInput,u],1,2);
            testCase.verifyLessThanOrEqual(max(abs(change(1,:))), ...
                testCase.cfg.controller.sampleTime*testCase.cfg.outputFeedback.steeringRateMaximum+1e-8);
            testCase.verifyEqual(testCase.command.actuatorInput,u(:,1),AbsTol=1e-12);
            testCase.verifyEqual(size(u,2),testCase.prediction.metadata.horizonSteps);
            testCase.verifyEqual(size(testCase.prediction.scenarios{1}.denseEgo,2), ...
                2*testCase.prediction.metadata.horizonSteps+1);
        end
        function finiteActuatorRatesRemainBoundedInBothDirections(testCase,requestedSpeed)
            cfg=testCase.cfg;cfg.referenceSpeed=requestedSpeed;
            cfg.model.brakingRatioRateMaximum=.8;
            block=localBlock(cfg,8);
            [~,~,prediction]=collisionAvoidanceController(block,[],[],cfg,[]);
            u=prediction.scenarios{1}.inputs;
            rates=diff([block.heldInput,u],1,2)/cfg.controller.sampleTime;
            limits=[cfg.outputFeedback.steeringRateMaximum;cfg.model.brakingRatioRateMaximum];
            testCase.verifyLessThanOrEqual(max(rates,[],2),limits+1e-8);
            testCase.verifyGreaterThanOrEqual(min(rates,[],2),-limits-1e-8);
            states=prediction.scenarios{1}.denseEgo(4:6,:);
            testCase.verifyGreaterThanOrEqual(min(states,[],2), ...
                [0;-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum]-1e-6);
            testCase.verifyLessThanOrEqual(max(states,[],2), ...
                [cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum]+1e-6);
        end
        function continuationReplaysTheUpdatedPhysicalState(testCase)
            block=localNextBlock(testCase.state,testCase.cfg);
            [command,~,prediction,state]=collisionAvoidanceController(block,[],[],testCase.cfg,testCase.state);
            expected=localAdvanceHold(state.hypotheses(1),command.actuatorInput,testCase.cfg);
            testCase.verifyEqual(prediction.scenarios{1}.ego(:,2),expected.ego,AbsTol=1e-10);
            testCase.verifyTrue(prediction.metadata.nlpCacheHit);
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
            testCase.verifyEqual(testCase.prediction.scenarios{1}.ego(:,2),item.ego,AbsTol=1e-10);
            testCase.verifyEqual(block.time,testCase.cfg.controller.sampleTime,AbsTol=1e-12);
        end
        function aFinePlanIsValidatedWithoutCoarseReinterpolation(testCase)
            inputs=repmat([.025,-.025;-.1,-.1],1,30);
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',testCase.block.initialHypotheses, ...
                'input',testCase.block.heldInput,'inputs',inputs));
            expected=localAdvanceSequence(testCase.block.initialHypotheses,inputs,testCase.cfg);
            testCase.verifyTrue(result.accepted);
            testCase.verifyEqual(result.trajectories{1}.inputs,inputs,AbsTol=1e-14);
            testCase.verifyEqual(result.trajectories{1}.ego(:,end),expected.ego,AbsTol=1e-10);
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
            cfg.outputFeedback.initializationSolveSeconds=10;
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
            testCase.verifyLessThanOrEqual(info.initialNlpViolation,1e-9);
            testCase.verifyLessThanOrEqual(info.nlpAttempts,5);
            request.inputs=prediction.scenarios{1}.inputs;
            result=solvePredictiveControl('outputFeedback',request);
            testCase.verifyTrue(result.accepted);
            testCase.verifyLessThanOrEqual(result.maximumViolation,1e-6);
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
        function aTargetCannotCrossBetweenCoarseChecksUndetected(testCase)
            item=testCase.block.initialHypotheses;item.ego(4:6)=0;
            % A temporal-aliasing fixture: all half-hold poses are separated.
            item.target=[0;-6;pi/2;480;0;0;1.6;2.4;.95;0;0];
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',item,'input',zeros(2,1),'inputs',zeros(2,1)));
            testCase.verifyFalse(result.accepted);
            testCase.verifyEqual(result.trajectories{1}.validationSubsteps,testCase.cfg.outputFeedback.validationSubsteps);
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
