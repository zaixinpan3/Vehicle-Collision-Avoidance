classdef compactPredictiveControlTest < matlab.unittest.TestCase
    % Direct-input planning with one fixed flow and checked SQP proposals.
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
        leadEncounter={"stoppedLead","hardBrakingLead"}
        cacheSpeed={.04,3}
        cacheMesh={8,24,64}
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
        function confirmedExitReleasesOnlyTheCollisionObligation(testCase)
            cfg=testCase.cfg;block=localBlock(cfg,8);item=block.initialHypotheses;
            radius=cfg.collision.encounterRangeMeters;
            item.target=[-radius+.46;0;pi;1;0;0;1.6;2.4;.95;0;0];
            other=item;other.target(1)=item.target(1)-.02;
            block.initialHypotheses=[item,other];
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',0));
            [~,~,p,prior]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyTrue(p.metadata.collisionTargetActive);
            block=localNextBlock(prior,cfg);
            for j=1:numel(block.samples),block.samples(j).radarAvailable=false;end
            [~,~,p,prior]=collisionAvoidanceController(block,[],[],cfg,prior);
            testCase.verifyTrue(p.metadata.collisionTargetActive);
            testCase.verifyNumElements(prior.hypotheses,2);
            block=localNextBlock(prior,cfg);
            for j=1:numel(block.samples),block.samples(j).radarAvailable=false;end
            [~,~,p,state]=collisionAvoidanceController(block,[],[],cfg,prior);
            testCase.verifyFalse(p.metadata.collisionTargetActive);
            testCase.verifyNumElements(state.hypotheses,2);
            testCase.verifyEqual(p.information,state.hypotheses);
            for j=1:2
                testCase.verifyNotEmpty(state.hypotheses(j).target);
                testCase.verifyEmpty(p.scenarios{j}.targets);
                testCase.verifyTrue(p.scenarios{j}.terminal.member);
            end
        end
        function reentryChecksTheRetainedTargetAgain(testCase)
            cfg=testCase.cfg;block=localBlock(cfg,8);
            edge=sqrt(cfg.collision.encounterRangeMeters^2-3^2);
            target=[edge-.01;3;0;9;-20;0;1.6;2.4;.95;0;0];
            block.initialHypotheses.target=target;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',block.initialHypotheses,'input',block.heldInput,'time',0));
            [~,~,p,prior]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyTrue(p.metadata.collisionTargetActive);
            [~,~,p,prior]=collisionAvoidanceController(localNextBlock(prior,cfg),[],[],cfg,prior);
            testCase.verifyFalse(p.metadata.collisionTargetActive);
            [~,~,p,state]=collisionAvoidanceController(localNextBlock(prior,cfg),[],[],cfg,prior);
            testCase.verifyTrue(p.metadata.collisionTargetActive);
            testCase.verifyEqual(p.scenarios{1}.targets(:,1),state.hypotheses.target);
            testCase.verifyTrue(p.scenarios{1}.terminal.member);
        end
        function anUnseenTargetCannotChangeCruiseOrSurviveInControllerMemory(testCase)
            cfg=testCase.cfg;block=localBlock(cfg,8);
            [~,cruise,baseline]=collisionAvoidanceController(block,[],[],cfg,[]);
            item=block.initialHypotheses;
            item.target=[cfg.collision.encounterRangeMeters+.2;0;pi;5;0;0;1.6;2.4;.95;0;0];
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',0));
            testCase.assertFalse(block.samples.radarAvailable);
            testCase.verifyTrue(all(isnan(block.samples.relativePosition)));
            block.initialHypotheses=item;
            [~,inputs,p,prior]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyEqual(inputs,cruise);
            testCase.verifyEqual(p.metadata.predictionWindow,baseline.metadata.predictionWindow);
            testCase.verifyEmpty(p.information.target);
            testCase.verifyEmpty(prior.hypotheses.target);
            testCase.verifyEmpty(prior.firstRadarTime);
            testCase.verifyFalse(p.metadata.collisionTargetActive);

            % The plant enters range during the first hold, but all returns
            % are dropped. Its actual motion remains private to the fixture.
            plant=prior;plant.hypotheses=item;
            block=localNextBlock(plant,cfg);
            testCase.assertTrue(any([block.samples.radarAvailable]));
            for j=1:numel(block.samples),block.samples(j).radarAvailable=false;end
            [~,~,p,prior]=collisionAvoidanceController(block,[],[],cfg,prior);
            testCase.verifyEmpty(prior.hypotheses.target);
            testCase.verifyFalse(p.metadata.collisionTargetActive);

            item=localAdvanceHold(item,plant.appliedInput,cfg);
            plant=prior;plant.hypotheses=item;
            block=localNextBlock(plant,cfg);
            block.samples(1).radarAvailable=false;
            block.acquisitionMotionPrior=item.target(3:end);
            [~,~,p,state]=collisionAvoidanceController(block,[],[],cfg,prior);
            testCase.verifyEqual(state.firstRadarTime,block.samples(2).time);
            testCase.verifyTrue(p.metadata.collisionTargetActive);
            testCase.verifyEqual(state.hypotheses.target,localAdvanceHold(item,prior.appliedInput,cfg).target,AbsTol=1e-10);
            testCase.verifyTrue(p.scenarios{1}.terminal.member);
        end
        function firstRadarReturnLocalizesTheTrackWithinItsMeasuredDisk(testCase)
            cfg=testCase.cfg;block=localBlock(cfg,8);item=block.initialHypotheses;
            item.ego(1:3)=[42;-17;.3];item.sensorBias(6:7)=[.01;-.005];
            block.initialHypotheses=item;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',0));
            block.samples.radarAvailable=true;radius=cfg.collision.encounterRangeMeters;
            block.samples.relativePosition=[0;radius+.01]+item.sensorBias(6:7);
            request=struct('operation',"condition",'cfg',cfg,'initial',true, ...
                'block',block,'hypotheses',item);
            testCase.verifyError(@()solvePredictiveControl('outputFeedback',request), ...
                'collisionAvoidanceController:missingAcquisitionPrior');
            request.block.acquisitionMotionPrior=[pi;2;0;0;1.6;2.4;.95;0;0];
            posterior=solvePredictiveControl('outputFeedback',request);
            position=item.ego(1:2)+radius*[-sin(item.ego(3));cos(item.ego(3))];
            testCase.verifyEqual(posterior.target,[position;request.block.acquisitionMotionPrior],AbsTol=1e-12);
            [~,~,prediction]=collisionAvoidanceController(request.block,[],[],cfg,[]);
            testCase.verifyTrue(prediction.metadata.collisionTargetActive);
            request.block.samples.relativePosition(2)=radius+1;
            testCase.verifyError(@()solvePredictiveControl('outputFeedback',request), ...
                'collisionAvoidanceController:emptyJointInformation');
        end
        function campaignSuppliesOnlyEgoInformationBeforeFirstContact(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            run=runJointOutputFeedbackScenario(Frames=4,Speeds=25,Scenarios="headOn", ...
                RandomSeed=20261107,OutputDirectory=string(folder.Folder), ...
                ControllerConfiguration=struct('outputFeedback',struct('maximumSolveSeconds',2)));
            testCase.assertEqual(run.results.outcome,"completed");
            data=jsondecode(fileread(run.results.source));
            testCase.verifyTrue(all(arrayfun(@(s)isempty(s.target),data.initialInformationContract)));
            testCase.verifyEmpty(data.initialAcquisitionMotionPrior);
            testCase.verifyFalse(data.initialSensorSample.radarAvailable);
            testCase.verifyTrue(all(isnan(data.initialSensorSample.relativePosition)));
            first=[];
            for k=1:numel(data.trace)
                current=data.trace(k);
                testCase.verifyEqual(current.metadata.collisionTargetActive,~isempty(first));
                testCase.verifyEqual(current.metadata.firstRadarTime,first);
                hits=[current.rawSensorBlock.radarAvailable];
                if isempty(first) && any(hits)
                    first=current.rawSensorBlock(find(hits,1)).time;
                    testCase.verifyNumElements(current.nextAcquisitionMotionPrior,9);
                else
                    testCase.verifyEmpty(current.nextAcquisitionMotionPrior);
                end
            end
            testCase.verifyNotEmpty(first);
        end
        function clearRoadSteadyTurnIsAPublicControllerEquilibrium(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=30;cfg.tire.frictionCoefficient=[.65;.65];
            [x,~,road,cfg]=collisionThreatScenario("curvedHeadOn",cfg);
            trim=nonlinearBicycleModel.operatingPoint(cfg,road.referenceCurve.curvature);
            block=localBlock(cfg,30);block.initialHypotheses.ego=x;block.heldInput=trim.input;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor",'cfg',cfg, ...
                'hypothesis',block.initialHypotheses,'input',trim.input,'time',0));
            [command,~,p,prior]=collisionAvoidanceController(block,[],road,cfg,[]);
            testCase.verifyEqual(command.actuatorInput,trim.input,AbsTol=1e-12);
            testCase.verifyFalse(p.metadata.collisionTargetActive);
            [command,~,p]=collisionAvoidanceController(localNextBlock(prior,cfg),[],road,cfg,prior);
            testCase.verifyEqual(command.actuatorInput,trim.input,AbsTol=1e-10);
            testCase.verifyTrue(p.scenarios{1}.terminal.member);
        end
        function sensorBlockPropagationPreservesEachSurvivingHypothesis(testCase)
            cfg=testCase.cfg;
            cfg.outputFeedback.noiseBounds=[.1;1;.1;.1];
            first=testCase.block.initialHypotheses;
            first.target=[12;3;pi/2;2;0;.01;1.6;2.4;.95;0;0];
            second=first;second.parameters=[.98;1;1.02;1];
            displaced=first;displaced.target(1)=displaced.target(1)+1;
            input=[.03;-.12];
            prior=struct('time',0,'appliedInput',input,'hypotheses',[first,displaced,second]);
            for samplesPerHold=[2,4]
                cfg.outputFeedback.sensorPeriod=cfg.controller.sampleTime/samplesPerHold;
                block=localNextBlock(prior,cfg);
                for j=2:samplesPerHold,block.samples(j).radarAvailable=false;end
                [posterior,info]=solvePredictiveControl('outputFeedback',struct('operation',"condition", ...
                    'cfg',cfg,'hypotheses',prior.hypotheses,'block',block,'initial',false));
                expected=[localAdvanceHold(first,input,cfg),localAdvanceHold(second,input,cfg)];
                testCase.verifyEqual(info.retainedIndices,[1,3]);
                testCase.verifyEqual(posterior,expected);
                testCase.verifyNotEqual(posterior(1).ego,posterior(2).ego);
            end
        end
        function aShiftedCachedPlanEqualsFreshPropagation(testCase,cacheSpeed,cacheMesh)
            cfg=testCase.cfg;cfg.outputFeedback.validationSubsteps=cacheMesh;
            [cfg,prior]=localRetainedPlan(cfg,cacheSpeed);
            block=localNextBlock(prior,cfg);
            [~,inputs,prediction]=collisionAvoidanceController(block,[],[],cfg,prior);
            fresh=rmfield(prior,'retainedTrajectories');
            [~,expectedInputs,expected]=collisionAvoidanceController(block,[],[],cfg,fresh);
            testCase.verifyEqual(inputs,expectedInputs);
            testCase.verifyEqual(prediction.scenarios,expected.scenarios);
            testCase.verifyGreaterThan(prediction.metadata.inheritedReusedHolds,0);
            testCase.verifyTrue(prediction.scenarios{1}.terminal.member);
            testCase.verifyEqual(prediction.metadata.pcbfValue,0);
        end
        function staleTrajectoryMetadataRequiresFreshPropagation(testCase)
            [cfg,prior]=localRetainedPlan(testCase.cfg,3);
            block=localNextBlock(prior,cfg);
            [~,expectedInputs,expected]=collisionAvoidanceController(block,[],[],cfg, ...
                rmfield(prior,'retainedTrajectories'));
            for change=["root","parameters","mesh","input"]
                changed=prior;old=changed.retainedTrajectories{1};
                switch change
                    case "root",old.collisionEgo(1,2)=old.collisionEgo(1,2)+.01;
                    case "parameters",old.parameters(3)=old.parameters(3)+.01;
                    case "mesh",old.validationSubsteps=2*old.validationSubsteps;
                    case "input",old.inputs(1,2)=old.inputs(1,2)+.01;
                end
                changed.retainedTrajectories{1}=old;
                [~,inputs,prediction]=collisionAvoidanceController(block,[],[],cfg,changed);
                testCase.verifyEqual(inputs,expectedInputs);
                testCase.verifyEqual(prediction.scenarios,expected.scenarios);
                testCase.verifyEqual(prediction.metadata.inheritedReusedHolds,0);
            end
        end
        function aChangedLaterInputReusesOnlyTheUnchangedPrefix(testCase)
            [cfg,prior]=localRetainedPlan(testCase.cfg,3);
            prior.fineControls(1,8)=.03;
            block=localNextBlock(prior,cfg);
            [~,inputs,prediction]=collisionAvoidanceController(block,[],[],cfg,prior);
            [~,expectedInputs,expected]=collisionAvoidanceController(block,[],[],cfg, ...
                rmfield(prior,'retainedTrajectories'));
            testCase.verifyEqual(inputs,expectedInputs);
            testCase.verifyEqual(prediction.scenarios,expected.scenarios);
            testCase.verifyGreaterThan(prediction.metadata.inheritedReusedHolds,0);
            testCase.verifyLessThan(prediction.metadata.inheritedReusedHolds,7);
        end
        function posteriorPruningUsesTheSurvivingTrajectory(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.maximumSolveSeconds=1e-12;
            cfg.outputFeedback.initializationSolveSeconds=2;
            block=localBlock(cfg,3);other=block.initialHypotheses;other.ego(1)=.03;
            block.initialHypotheses=[block.initialHypotheses,other];
            [~,~,~,prior]=collisionAvoidanceController(block,[],[],cfg,[]);
            block=localNextBlock(prior,cfg);
            for j=1:numel(block.samples),block.samples(j).position(1)=block.samples(j).position(1)+.05;end
            [~,inputs,prediction,state]=collisionAvoidanceController(block,[],[],cfg,prior);
            [~,expectedInputs,expected]=collisionAvoidanceController(block,[],[],cfg, ...
                rmfield(prior,'retainedTrajectories'));
            testCase.verifyNumElements(state.hypotheses,1);
            testCase.verifyEqual(prediction.metadata.conditioning.retainedIndices,2);
            testCase.verifyGreaterThan(prediction.metadata.inheritedReusedHolds,0);
            testCase.verifyEqual(inputs,expectedInputs);
            testCase.verifyEqual(prediction.scenarios,expected.scenarios);
        end
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
            cfg.outputFeedback.validationSubsteps=integrationMesh;
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
        function legacyIntegrationHistoryCannotChangeTheContinuation(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=3;cfg.outputFeedback.nlpJit=true;
            cfg.outputFeedback.maximumSolveSeconds=2;cfg.outputFeedback.nlpMaximumIterations=1;
            block=localBlock(cfg,2);
            [~,~,~,prior]=collisionAvoidanceController(block,[],[],cfg,[]);
            block=localNextBlock(prior,cfg);
            prior.requiresFineIntegration=false;
            [~,first,firstPrediction]=collisionAvoidanceController(block,[],[],cfg,prior);
            prior.requiresFineIntegration=true;
            [~,second,secondPrediction,state]=collisionAvoidanceController(block,[],[],cfg,prior);
            testCase.verifyEqual(second,first,AbsTol=1e-12);
            testCase.verifyEqual(secondPrediction.scenarios{1}.collisionEgo, ...
                firstPrediction.scenarios{1}.collisionEgo,AbsTol=1e-12);
            testCase.verifyTrue(secondPrediction.scenarios{1}.terminal.member);
            testCase.verifyEqual(secondPrediction.metadata.pcbfValue,0);
            testCase.verifyFalse(isfield(state,'requiresFineIntegration'));
        end
        function aValidatedDrivingPlanEndsTheSearch(testCase)
            for prediction={testCase.prediction,localNextPrediction(testCase.state,testCase.cfg)}
                info=prediction{1}.metadata;
                testCase.verifyEqual(info.performanceAcceptedSteps,0);
                testCase.verifyEqual(info.nlpAttempts,0);
                testCase.verifyEqual(info.nonlinearOracleEvaluations,0);
                testCase.verifyEqual(info.pcbfValue,0);
                testCase.verifyFalse(info.rootTerminalPlanUsed);
                testCase.verifyTrue(prediction{1}.scenarios{1}.terminal.member);
            end
        end
        function aShrinkingPreviewRetainsTheExecutableSuffixAndTerminalPoint(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=1.5;cfg.outputFeedback.maximumSolveSeconds=1e-12;
            cfg.outputFeedback.initializationSolveSeconds=2;
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
            cfg.outputFeedback.initializationSolveSeconds=2;
            cfg.outputFeedback.horizonSeconds=.1;
            block=localBlock(cfg,cfg.referenceSpeed);
            [~,~,~,prior]=collisionAvoidanceController(block,[],[],cfg,[]);
            % A retained fine sequence can exceed the currently requested
            % preview. Supply a replay-checked longer sequence at cruise.
            initialCount=8;prior.horizonSteps=initialCount;
            prior.fineControls=repmat(prior.fineControls(:,end),1,initialCount);
            prior.controls=prior.fineControls;prior.boundaries=0:initialCount;
            check=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',cfg,'hypotheses',prior.hypotheses,'input',block.heldInput,'inputs',prior.fineControls));
            testCase.assertTrue(check.accepted);
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
        function anOpenRoadRestStateCanResumeCruise(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=1.5;cfg.outputFeedback.nlpJit=true;
            cfg.outputFeedback.maximumSolveSeconds=2;cfg.outputFeedback.horizonSeconds=8;
            block=localBlock(cfg,0);block.initialHypotheses.target=[];
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',block.initialHypotheses,'input',block.heldInput,'time',0));
            [command,~,prediction]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyGreaterThan(command.actuatorInput(2),.01);
            testCase.verifyGreaterThan(prediction.scenarios{1}.ego(4,end),1.4);
            testCase.verifyLessThanOrEqual(prediction.metadata.maximumConstraintViolation,1e-6);
        end
        function stabilityCostIsSymmetricAndRespondsToTireLoading(testCase)
            cfg=testCase.cfg;
            item=testCase.block.initialHypotheses;item.target=[];item.ego=[0;0;0;14;0;0];
            request=struct('operation',"evaluateControlSequence",'cfg',cfg, ...
                'hypotheses',item,'input',zeros(2,1),'holds',[1,2,3],'inputs',zeros(2,3));
            straight=solvePredictiveControl('outputFeedback',request);
            request.inputs(2,:)=-.8;
            braking=solvePredictiveControl('outputFeedback',request);
            request.inputs(2,:)=.8;
            traction=solvePredictiveControl('outputFeedback',request);
            testCase.verifyEqual(straight.stabilityCost,0,AbsTol=1e-12);
            testCase.verifyGreaterThan(braking.stabilityCost,straight.stabilityCost);
            testCase.verifyEqual(traction.stabilityCost,braking.stabilityCost,AbsTol=1e-12);
            request.inputs(1,:)=[.08,.05,-.01];
            left=solvePredictiveControl('outputFeedback',request);
            request.inputs(1,:)=-request.inputs(1,:);
            right=solvePredictiveControl('outputFeedback',request);
            testCase.verifyGreaterThan(left.stabilityCost,traction.stabilityCost);
            testCase.verifyEqual(right.stabilityCost,left.stabilityCost,AbsTol=1e-12);
        end
        function aPassingDetourCostsLessThanRemainingParked(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=1.5;
            item=testCase.block.initialHypotheses;item.target=[];
            item.ego=[0;6;0;cfg.referenceSpeed;0;0];
            request=struct('operation',"evaluateControlSequence",'cfg',cfg, ...
                'hypotheses',item,'input',zeros(2,1),'holds',[1,1,1],'inputs',zeros(2,3));
            passing=solvePredictiveControl('outputFeedback',request);
            request.hypotheses.ego=zeros(6,1);
            parked=solvePredictiveControl('outputFeedback',request);
            testCase.verifyLessThan(passing.cost,parked.cost);
        end
        function theTerminalCruiseInputRenewsTheRequestedHorizon(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            campaign=runJointOutputFeedbackScenario(Frames=18,Speeds=8,Scenarios="hardBrakingLead", ...
                RandomSeed=20261107,OutputDirectory=string(folder.Folder), ...
                ControllerConfiguration=struct('outputFeedback',struct('maximumSolveSeconds',1)));
            testCase.assertEqual(campaign.results.outcome,"completed");
            data=jsondecode(fileread(campaign.results.source));
            current=data.trace(17).metadata;prior=data.trace(16).metadata;
            testCase.verifyEqual(current.horizonSteps,prior.horizonSteps);
            testCase.verifyEqual(current.predictionWindow.allocatedSeconds, ...
                current.horizonSteps*data.configuration.controller.sampleTime);
            testCase.verifyEqual(current.pcbfValue,0);
            testCase.verifyLessThanOrEqual(current.terminalValue,data.configuration.outputFeedback.nlpTolerance);
            testCase.verifyEqual(current.nlpAttempts,0);
            testCase.verifyTrue(current.retainedFinePlanUsed);
        end
        function aCheckedPlanCanShiftPastItsOriginalEndWithoutAnotherSolve(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=3;cfg.outputFeedback.horizonSeconds=.15;
            cfg.outputFeedback.maximumSolveSeconds=1e-12;block=localBlock(cfg,3);
            [~,~,~,prior]=collisionAvoidanceController(block,[],[],cfg,[]);
            terminalInput=terminalSafeSet.movingControl(cfg);
            for k=1:12
                block=localNextBlock(prior,cfg);
                [~,inputs,prediction,next]=collisionAvoidanceController(block,[],[],cfg,prior);
                testCase.verifyEqual(inputs,[prior.fineControls(:,2:end),terminalInput]);
                testCase.verifyEqual(prediction.metadata.nlpAttempts,0);
                testCase.verifyTrue(next.terminal.member);
                testCase.verifyGreaterThan(min(prediction.scenarios{1}.collisionEgo(4,:)),0);
                prior=next;
            end
        end
        function theMovingCoreChecksTheEntireFutureTargetTrack(testCase)
            x=[0;0;0;3;0;0];
            target=[100;0;pi;1;0;0;1.6;2.4;.95;0;0];
            testCase.verifyGreaterThan(terminalSafeSet.movingValue(x,target,testCase.cfg),0);
            target(2)=10;
            [value,n]=terminalSafeSet.movingValue(x,target,testCase.cfg);
            testCase.verifyLessThan(value,0);
            next=x;next(1)=x(1)+3;
            target=predictiveSafetyGeometry.predictTarget(target,1);
            testCase.verifyLessThanOrEqual(terminalSafeSet.movingValue(next,target,testCase.cfg,n),value+1e-9);
        end
        function aBrakeOnlyActuatorCannotCertifyPerpetualCruiseAgainstDrag(testCase)
            cfg=testCase.cfg;cfg.actuation.brakingRatioMaximum=0;
            item=testCase.block.initialHypotheses;item.target=[];
            checked=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',cfg,'hypotheses',item,'input',zeros(2,1),'inputs',zeros(2,1)));
            testCase.verifyFalse(checked.accepted);
            testCase.verifyFalse(checked.trajectories{1}.terminal.member);
        end
        function turningTargetSupportIncludesLaterRevolutions(testCase)
            x=[0;0;0;2;0;0];
            target=[20;0;0;2;0;.2;1.6;2.4;.95;.3;-.2];
            testCase.verifyGreaterThan(terminalSafeSet.movingValue(x,target,testCase.cfg),0);
            x(2)=-20;
            [value,normal]=terminalSafeSet.movingValue(x,target,testCase.cfg);
            testCase.verifyLessThan(value,0);
            testCase.verifyGreaterThanOrEqual(normal(1),-eps);
            % The analytic stationary circular support also covers the target
            % when it accelerates and traverses the circle repeatedly.
            target(5)=1;
            testCase.verifyEqual(terminalSafeSet.movingValue(x,target,testCase.cfg),value,AbsTol=1e-10);
        end
        function expiredContinuationCannotStartASecondSearchBudget(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.maximumSolveSeconds=1e-12;
            [~,~,~,prior]=collisionAvoidanceController(localBlock(cfg,3),[],[],cfg,[]);
            block=localNextBlock(prior,cfg);
            prior=rmfield(prior,'movingTerminalPlan');
            cfg.outputFeedback.recoverySolveSeconds=100;
            prior.configuration.outputFeedback.recoverySolveSeconds=100;
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],cfg,prior), ...
                'collisionAvoidanceController:compactNoFeasiblePlan');
        end
        function anExpiredAdmissionCannotDispatchTheTerminalBrake(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.initializationSolveSeconds=1e-12;
            block=localBlock(cfg,0);
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],cfg,[]), ...
                'collisionAvoidanceController:compactNoFeasiblePlan');
        end
        function aLegacyParkedPlanIsReplannedFromTheConditionedState(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=1.5;
            [~,~,~,old]=collisionAvoidanceController(localBlock(cfg,0),[],[],cfg,[]);
            old=rmfield(old,'movingTerminalPlan');
            brake=terminalSafeSet.compactControl(old.appliedInput,cfg);
            old.fineControls=repmat(brake,1,old.horizonSteps);old.appliedInput=brake;
            block=localNextBlock(old,cfg);
            [command,~,prediction,state]=collisionAvoidanceController(block,[],[],cfg,old);
            testCase.verifyGreaterThan(command.actuatorInput(2),0);
            testCase.verifyTrue(state.firstFeasibleDrivingPlan);
            testCase.verifyFalse(prediction.metadata.rootTerminalPlanUsed);
            testCase.verifyFalse(prediction.metadata.retainedFinePlanUsed);
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
            cfg=testCase.cfg;cfg.outputFeedback.horizonResolutionSeconds=.9;
            [~,~,baseline]=collisionAvoidanceController(localBlock(cfg,8),[],[],cfg,[]);
            speed=hypot(cfg.model.speedMaximum,cfg.model.lateralVelocityMaximum);
            [~,~,prediction]=collisionAvoidanceController(localBlock(cfg,speed),[],[],cfg,[]);
            testCase.verifyGreaterThan(prediction.metadata.predictionWindow.allocatedSeconds, ...
                baseline.metadata.predictionWindow.allocatedSeconds);
            testCase.verifyEqual(prediction.metadata.controlNodes,baseline.metadata.controlNodes);
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
            cfg.outputFeedback.validationSubsteps=integrationMesh;
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
            item=testCase.block.initialHypotheses;item.ego(4:6)=[2;0;0];
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
            item=testCase.block.initialHypotheses;item.ego=[0;0;pi;.3;0;0];
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
        function transitionPredictionErrorsStayWithinTheSensorNoiseBounds(testCase)
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
                predicted=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                    'cfg',cfg,'hypothesis',item,'input',input,'time',j*cfg.outputFeedback.sensorPeriod));
                physical=item;physical.ego=reference;
                measured=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                    'cfg',cfg,'hypothesis',physical,'input',input,'time',predicted.time));
                testCase.verifyLessThanOrEqual(norm(predicted.position-measured.position),cfg.outputFeedback.noiseBounds(1));
                testCase.verifyLessThanOrEqual(norm(predicted.acceleration-measured.acceleration),cfg.outputFeedback.noiseBounds(2));
                testCase.verifyLessThanOrEqual(abs(predicted.yawRate-measured.yawRate),cfg.outputFeedback.noiseBounds(3));
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
                terminalSafeSet.movingControl(testCase.cfg));
        end
        function theTerminalContinuationStraightensAndKeepsDriving(testCase)
            item=testCase.block.initialHypotheses;item.ego=[0;0;.1;3;.4;.2];
            request=struct('operation',"validateInputSequence",'cfg',testCase.cfg, ...
                'hypotheses',item,'input',[.2;0],'inputs',[.2;0]);
            value=solvePredictiveControl('outputFeedback',request);
            testCase.verifyTrue(value.accepted);
            terminal=value.trajectories{1}.terminal;
            testCase.verifyEqual(terminal.input(1),0);
            testCase.verifyGreaterThan(terminal.bridgeHolds,0);
            testCase.verifyEqual(terminal.model,"straightCruiseEscape");
            testCase.verifyGreaterThan(terminal.input(2),0);
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
        function obsoleteWarmIteratesCannotOverwriteTheExecutablePlan(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=3;cfg.outputFeedback.maximumSolveSeconds=1e-9;
            cfg.outputFeedback.initializationSolveSeconds=2;
            block=localBlock(cfg,0);item=block.initialHypotheses;
            item.target=[8;5;0;0;0;0;1.6;2.4;.95;0;0];block.initialHypotheses=item;
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
        function stabilityPreferencePreservesZeroDeficitAndHardTerminalMembership(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            campaign=runJointOutputFeedbackScenario(Frames=1,Speeds=15,Scenarios="headOn", ...
                RandomSeed=20261107,Warmup=false,OutputDirectory=string(folder.Folder), ...
                ControllerConfiguration=struct('outputFeedback',struct('initializationSolveSeconds',10)));
            testCase.assertEqual(campaign.results.outcome,"completed");
            data=jsondecode(fileread(campaign.results.source));info=data.trace.metadata;
            testCase.verifyTrue(info.restorationAttempted);
            testCase.verifyGreaterThan(info.initialRestorationSlack,0);
            testCase.verifyLessThanOrEqual(info.acceptedStabilityCost,info.firstFeasibleStabilityCost);
            testCase.verifyEqual(info.pcbfValue,0);
            testCase.verifyTrue(info.terminalSetEnforced);
            testCase.verifyLessThanOrEqual(info.terminalValue,data.configuration.outputFeedback.nlpTolerance);
        end
        function highSpeedLeadAvoidanceKeepsRearWheelSlipBelowThePureCorneringPeak(testCase,leadEncounter)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            campaign=runJointOutputFeedbackScenario(Frames=160,Speeds=25,Scenarios=leadEncounter, ...
                RandomSeed=20261107,OutputDirectory=string(folder.Folder), ...
                ControllerConfiguration=struct('outputFeedback',struct('initializationSolveSeconds',10,'maximumSolveSeconds',1)));
            testCase.assertEqual(campaign.results.outcome,"completed");
            data=jsondecode(fileread(campaign.results.source));cfg=data.configuration;
            x=[data.trace.stateAfterHold];
            rearSlip=atan2(x(5,:)-cfg.vehicle.lr*x(6,:),x(4,:));
            tire=nonlinearBicycleModel.tireParameters(cfg);
            pureCorneringPeak=atan(3*tire.longitudinalForceScale(2)/tire.corneringStiffness(2));
            testCase.verifyLessThan(max(abs(rearSlip)),pureCorneringPeak);
            metadata=[data.trace.metadata];
            compared=[metadata.performanceAcceptedSteps]>0;
            testCase.verifyTrue(any(compared));
            testCase.verifyTrue(all([metadata(compared).acceptedStabilityCost]<[metadata(compared).firstFeasibleStabilityCost]));
            testCase.verifyTrue(all([metadata.pcbfValue]==0));
            testCase.verifyFalse(data.nodeCollisionDetected);
            testCase.verifyLessThan(abs(x(2,end)),cfg.vehicle.width/2);
        end
        function aMovingSuffixCanBeExtendedWhenRepeatingItsLastInputWouldStop(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=1.5;cfg.outputFeedback.maximumSolveSeconds=1e-12;
            block=localBlock(cfg,.3);item=block.initialHypotheses;
            brake=[0;-.1];count=0;
            while item.ego(4)>0 && count<60
                item=localAdvanceHold(item,brake,cfg);
                count=count+1;
            end
            testCase.assertEqual(item.ego(4),0);count=count-1;
            testCase.assertGreaterThan(count,2);
            cfg.outputFeedback.horizonSeconds=(count-.25)*cfg.controller.sampleTime;
            [~,~,~,prior]=collisionAvoidanceController(block,[],[],cfg,[]);
            prior.fineControls=repmat(brake,1,count);prior.appliedInput=brake;
            block=localNextBlock(prior,cfg);
            [command,inputs,prediction]=collisionAvoidanceController(block,[],[],cfg,prior);
            testCase.verifyEqual(command.actuatorInput,brake);
            testCase.verifyEqual(inputs(:,1:end-1),prior.fineControls(:,2:end));
            testCase.verifyEqual(inputs(:,end),terminalSafeSet.movingControl(cfg));
            testCase.verifyFalse(prediction.metadata.restorationAttempted);
            testCase.verifyGreaterThan(min(prediction.scenarios{1}.collisionEgo(4,:)),0);
            testCase.verifyTrue(prediction.scenarios{1}.terminal.member);
        end
        function aParkingWitnessCannotBeDispatchedAsADrivingPlan(testCase)
            cfg=testCase.cfg;cfg.referenceSpeed=3;
            block=localBlock(cfg,3);item=block.initialHypotheses;
            item.target=[5.6;0;0;0;0;0;1.6;2.4;.95;0;0];
            request=struct('operation',"validateInputSequence",'cfg',cfg, ...
                'hypotheses',item,'input',block.heldInput,'inputs',repmat([0;-.8],1,60));
            parked=solvePredictiveControl('outputFeedback',request);
            testCase.verifyFalse(parked.trajectories{1}.terminal.member);
            testCase.verifyFalse(parked.accepted);
        end
        function restorationCanLeaveTheStopResetAlongTheDriveInput(testCase)
            cfg=testCase.cfg;item=testCase.block.initialHypotheses;
            item.ego(4)=.05*cfg.model.brakeHoldSpeed;
            value=solvePredictiveControl('outputFeedback',struct('operation',"evaluateControlSequence", ...
                'cfg',cfg,'hypotheses',item,'input',zeros(2,1),'holds',1,'inputs',[0;-.001]));
            testCase.verifyEqual(value.forward,.001,AbsTol=1e-12);
            testCase.verifyEqual(value.forwardGradient(:,1),[0;-1],AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(value.terminal,cfg.outputFeedback.nlpTolerance);
        end
        function condensedGradientsDifferentiateTheActualConstraintValues(testCase,integrationMesh)
            cfg=testCase.cfg;cfg.outputFeedback.nlpJit=true;
            cfg.outputFeedback.validationSubsteps=integrationMesh;
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
        function predictionsAgreeWithFreshReplayAcrossFrames(testCase,movingTarget)
            cfg=testCase.cfg;cfg.referenceSpeed=3;cfg.outputFeedback.nlpJit=true;
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
            cfg.outputFeedback.validationSubsteps=integrationMesh;
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
            testCase.verifyEqual(value.stabilityGradient.'*direction, ...
                (p.stabilityCost-m.stabilityCost)/(2*step),AbsTol=3e-4);
            testCase.verifyEqual(value.forwardGradient(3:end,1),zeros(8,1));
            testCase.verifyGreaterThan(abs(value.forwardGradient(2,1)),1e-3);
        end
        function nativeDerivativesAgreeAcrossChangingHorizons(testCase,integrationMesh,movingTarget)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.assumeTrue(isfile(fullfile(root,'solver','controller',['condensedSensitivityKernelMex.',mexext])), ...
                'Run buildControllerKernels to test the optional native sensitivity implementation.');
            cfg=testCase.cfg;cfg.outputFeedback.validationSubsteps=integrationMesh;
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
            cfg.outputFeedback.validationSubsteps=integrationMesh;
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
        function centeredPenetrationHasAnImprovingTranslation(testCase,rectangleYaw)
            pose=[3;-2;rectangleYaw];shape=[2.4;.95;.2;-.1];
            [gap,gradient]=predictiveSafetyGeometry.rectangleAxisGaps(pose,shape,pose,shape);
            normal=gradient(1:2);step=1e-5;
            testCase.verifyEqual(norm(normal),1,AbsTol=1e-12);
            for side=[-1,1]
                moved=pose+[side*step*normal;0];
                gapMoved=predictiveSafetyGeometry.rectangleAxisGaps(moved,shape,pose,shape);
                testCase.verifyEqual(max(gapMoved)-max(gap),step,AbsTol=1e-10);
            end
        end
        function penetrationGradientFollowsTheActiveFacetOnEitherSide(testCase)
            ego=[1;2;.38];targets=[1.1,1.1;2.4,1.6;-.17,-.17];shape=[2.4;.95;.2;-.1];
            targetShape=[2.1;.8;-.15;.05];step=1e-6;
            [gap,gradient]=predictiveSafetyGeometry.rectangleAxisGaps(ego,shape,targets,targetShape);
            testCase.verifyLessThan(max(gap,[],1),0);
            for j=1:3
                delta=zeros(3,1);delta(j)=step;
                plus=predictiveSafetyGeometry.rectangleAxisGaps(ego+delta,shape,targets,targetShape);
                minus=predictiveSafetyGeometry.rectangleAxisGaps(ego-delta,shape,targets,targetShape);
                testCase.verifyEqual(gradient(j,:),(max(plus)-max(minus))/(2*step),AbsTol=1e-7);
            end
        end
        function centeredTerminalBridgeExposesASteeringCorrection(testCase)
            cfg=testCase.cfg;item=localBlock(cfg,8).initialHypotheses;
            item.target=[26;0;0;0;0;0;1.6;2.4;.95;0;0];
            request=struct('operation',"evaluateControlSequence",'cfg',cfg,'hypotheses',item, ...
                'input',zeros(2,1),'holds',[10,10],'inputs',zeros(2,2));
            value=solvePredictiveControl('outputFeedback',request);
            testCase.verifyLessThan(max(value.stage),0);
            testCase.verifyGreaterThan(value.terminal,1);
            centered=value.terminalGradient(1:2:end);
            testCase.verifyGreaterThan(norm(centered),1e-3);
            offsets=[-1e-8,1e-8];steering=zeros(numel(centered),numel(offsets));
            for k=1:numel(offsets)
                request.hypotheses.target(2)=offsets(k);
                near=solvePredictiveControl('outputFeedback',request);
                steering(:,k)=near.terminalGradient(1:2:end);
                testCase.verifyEqual(norm(steering(:,k)),norm(centered),AbsTol=1e-6);
            end
            testCase.verifyEqual(steering(:,1),-steering(:,2),AbsTol=1e-6);
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
        function stationaryEndpointsCannotSupplyMovingContinuation(testCase)
            item=testCase.block.initialHypotheses;item.ego(4:6)=0;
            item.target=[4.95;0;0;0;0;0;1.6;2.4;.95;0;0];
            result=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',testCase.cfg,'hypotheses',item,'input',zeros(2,1),'inputs',repmat([0;-.1],1,60)));
            testCase.verifyFalse(result.accepted);
            testCase.verifyFalse(result.trajectories{1}.terminal.member);
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
            testCase.verifyFalse(result.accepted);
            testCase.verifyFalse(result.trajectories{1}.terminal.member);
            testCase.verifyEqual(result.trajectories{1}.validationSubsteps,testCase.cfg.outputFeedback.validationSubsteps);
            testCase.verifyEqual(result.trajectories{1}.denseEgo,expected,AbsTol=1e-10);
            testCase.verifyGreaterThan(expected(1,end),item.ego(1));
            testCase.verifyEqual(result.trajectories{1}.ego(4:6,end),zeros(3,1),AbsTol=1e-14);
        end
        function collisionBetweenControlNodesDoesNotRejectThePlan(testCase)
            item=testCase.block.initialHypotheses;item.ego(4:6)=[.1;0;0];
            % The target crosses the slowly moving ego between two control
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

function [cfg,prior]=localRetainedPlan(cfg,speed)
    cfg.referenceSpeed=1.5;
    cfg.outputFeedback.maximumSolveSeconds=1e-12;
    cfg.outputFeedback.initializationSolveSeconds=2;
    block=localBlock(cfg,speed);
    [~,~,~,prior]=collisionAvoidanceController(block,[],[],cfg,[]);
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

function prediction=localNextPrediction(prior,cfg)
    [~,~,prediction]=collisionAvoidanceController(localNextBlock(prior,cfg),[],[],cfg,prior);
end
