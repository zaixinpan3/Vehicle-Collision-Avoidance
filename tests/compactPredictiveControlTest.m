classdef compactPredictiveControlTest < matlab.unittest.TestCase
    % Direct-input planning with one fixed flow and checked CasADi/IPOPT proposals.
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
        admissionMeasurements={"boundedRadar","exactRadar","exactRadarWithDropout"}
        leadEncounter={"stoppedLead","hardBrakingLead"}
        acquisitionEncounter={"brakingLead","acceleratingHeadOn"}
        uncertainEncounter={"brakingLead","trafficBrakingLead","curvedCrossing", ...
            "headOn","trafficHeadOn","curvedHeadOn","acceleratingHeadOn"}
        cacheSpeed={.04,3}
        cacheMesh={8,24,64}
    end
    methods (TestClassSetup)
        function prepareController(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'estimator')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            setupJointNlpSolver;
            testCase.cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
                'model',struct('speedMaximum',16,'lateralVelocityMaximum',1), ...
                'outputFeedback',struct('method',"ipopt",'nlpJit',false,'maximumSolveSeconds',1)));
            testCase.block=localBlock(testCase.cfg,8);
            [testCase.command,~,testCase.prediction,testCase.state]= ...
                collisionAvoidanceController(testCase.block,[],[],testCase.cfg,[]);
        end
    end
    methods (Test)
        function theMainEntryRejectsStateEstimatesAndRetiredSolverRequests(testCase)
            block=testCase.block;cfg=testCase.cfg;
            testCase.verifyError(@()collisionAvoidanceController(struct('speed',8),[],[],cfg,[]), ...
                'collisionAvoidanceController:invalidSensorBlock');
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],struct('solver',struct('method',"rti")),[]), ...
                'collisionAvoidanceController:invalidConfiguration');
            old=cfg;old.outputFeedback.method="sqpRti";
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],old,[]), ...
                'collisionAvoidanceController:invalidConfiguration');
            prior=testCase.state;prior.configuration=old;
            testCase.verifyError(@()collisionAvoidanceController(localNextBlock(prior,cfg),[],[],cfg,prior), ...
                'collisionAvoidanceController:changedContinuationProblem');
            for operation=["linearize","linearizeStep"]
                request=struct('operation',operation,'cfg',cfg);
                testCase.verifyError(@()solvePredictiveControl("outputFeedback",request), ...
                    'collisionAvoidanceController:invalidOutputFeedbackOperation');
            end
            testCase.verifyError(@()solvePredictiveControl("probe",struct()), ...
                'collisionAvoidanceController:invalidSolverInterface');
        end
        function discardedPreparationUsesTheRawSensorSolver(testCase)
            step=prepareCollisionAvoidanceController(testCase.block,[],testCase.cfg);
            testCase.verifyEqual(step.solver,"casadiIpoptCompactPcbf");
            testCase.verifyEqual(step.predictiveBarrierValue,testCase.prediction.metadata.pcbfValue);
            testCase.verifyError(@()prepareCollisionAvoidanceController(struct('speed',8),[],testCase.cfg), ...
                'collisionAvoidanceController:invalidSensorBlock');
        end
        function anUnsupportedCheckpointRequiresNewInitialization(testCase)
            prior=testCase.state;prior.kind="unsupported";
            testCase.verifyError(@()collisionAvoidanceController(localNextBlock(prior,testCase.cfg), ...
                [],[],testCase.cfg,prior),'collisionAvoidanceController:incompatibleContinuation');
        end
        function aLongerSearchBudgetCanResumeAnExistingDrivingPlan(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.maximumSolveSeconds=2;
            [~,~,p,state]=collisionAvoidanceController(localNextBlock(testCase.state,cfg),[],[],cfg,testCase.state);
            testCase.verifyEqual(p.metadata.callBudgetSeconds,2);
            testCase.verifyEqual(state.pcbfDeficit,0);
            testCase.verifyTrue(p.scenarios{1}.terminal.member);
        end

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
            [~,~,p,state]=collisionAvoidanceController(block,[],[],cfg,prior);
            testCase.verifyEqual(state.firstRadarTime,block.samples(2).time);
            testCase.verifyFalse(p.metadata.collisionTargetActive);
            testCase.verifyEmpty(state.hypotheses.target);
            testCase.verifyEmpty(p.scenarios{1}.targets);
            testCase.verifyEqual(state.hypotheses.targetObserver.detections,numel(block.samples)-1);
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
            posterior=solvePredictiveControl('outputFeedback',request);
            position=item.ego(1:2)+radius*[-sin(item.ego(3));cos(item.ego(3))];
            testCase.verifyEqual(posterior.targetObserver.state(1:2),position,AbsTol=1e-12);
            testCase.verifyFalse(posterior.targetObserver.initialized);
            testCase.verifyEmpty(posterior.target);
            [~,~,prediction]=collisionAvoidanceController(request.block,[],[],cfg,[]);
            testCase.verifyFalse(prediction.metadata.collisionTargetActive);
            request.block.samples.relativePosition(2)=radius+1;
            testCase.verifyError(@()solvePredictiveControl('outputFeedback',request), ...
                'collisionAvoidanceController:emptyJointInformation');
        end
        function firstCompleteTargetEstimateActivatesPlanningAtTheSameBoundary(testCase,admissionMeasurements)
            cfg=testCase.cfg;block=localBlock(cfg,8);item=block.initialHypotheses;
            item.target=[10;22;.3;9;-.4;.01;1.6;2.4;.95;0;0];
            exactRadar=admissionMeasurements~="boundedRadar";
            dropoutAtAdmission=admissionMeasurements=="exactRadarWithDropout";
            block.time=3;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',block.time));
            % Without a reported radius the original bounded sensor contract applies.
            if exactRadar,block.samples.radarPositionErrorMeters=0;end
            [~,~,p,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyEqual(p.metadata.initializationSource,"nominalCruiseRollout");
            admission=block.time+cfg.controller.sampleTime;
            expected=admission+double(dropoutAtAdmission)*cfg.controller.sampleTime;
            while state.time<expected-1e-9
                plant=state;plant.hypotheses=item;
                block=localNextBlock(plant,cfg);
                if exactRadar,[block.samples.radarPositionErrorMeters]=deal(0);end
                if dropoutAtAdmission && abs(block.time-admission)<1e-9
                    [block.samples.radarAvailable]=deal(false);
                end
                item=localAdvanceHold(item,state.appliedInput,cfg);
                [~,~,p,state]=collisionAvoidanceController(block,[],[],cfg,state);
                testCase.verifyEqual(p.metadata.collisionTargetActive,p.metadata.targetEstimateReady);
                if state.time<expected-1e-9
                    testCase.verifyFalse(p.metadata.collisionTargetActive);
                    testCase.verifyEmpty(p.scenarios{1}.targets);
                end
            end
            testCase.verifyEqual(state.time,expected,AbsTol=1e-12);
            testCase.verifyTrue(p.metadata.collisionTargetActive);
            testCase.verifyEqual(p.metadata.budgetPhase,"initialization");
            testCase.verifyNotEmpty(p.scenarios{1}.targets);
            testCase.verifyEqual(p.information.target(3:6),item.target(3:6),AbsTol=.03);
        end

        function aStationaryYawMeasurementCorrectsAPositionConsistentForecast(testCase)
            cfg=testCase.cfg;block=localBlock(cfg,8);item=block.initialHypotheses;
            item.ego(3)=.4;item.target=[10;8;-.7;0;0;0;1.6;2.4;.95;0;0];
            settings=estimatorControllerIntegrationConfig();track=[];
            for k=1:settings.initialization.minimumRadarSamples
                time=(k-1)*cfg.outputFeedback.sensorPeriod;
                [track,~]=onlineNrmmTrackingRuntime("targetPoint",track,time,item.target(1:2),item.ego(3),cfg,-1.1);
            end
            item.targetObserver=track;
            block.time=time+cfg.outputFeedback.sensorPeriod;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',block.time));
            block.samples.relativeHeading=-.1;
            posterior=solvePredictiveControl('outputFeedback',struct('operation',"condition", ...
                'cfg',cfg,'initial',true,'block',block,'hypotheses',item));
            testCase.verifyEqual(posterior.target(1:3),[10;8;.3],AbsTol=1e-12);
            testCase.verifyEqual(posterior.target(4:6),zeros(3,1));
        end

        function perceivedHeadingAnchorsTheMovingForecastBeforeTheStop(testCase)
            cfg=testCase.cfg;block=localBlock(cfg,8);item=block.initialHypotheses;
            item.ego(3)=2.95;
            target=[12;4;-3.05;2;-.6;.035;1.6;2.4;.95;0;0];
            settings=estimatorControllerIntegrationConfig();track=[];
            for j=0:settings.initialization.minimumRadarSamples-1
                t=j*cfg.outputFeedback.sensorPeriod;
                q=predictiveSafetyGeometry.predictTarget(target,t);
                [track,~]=onlineNrmmTrackingRuntime("targetPoint",track,t,q(1:2),item.ego(3),cfg);
            end
            item.targetObserver=track;block.time=t+cfg.outputFeedback.sensorPeriod;
            q=predictiveSafetyGeometry.predictTarget(target,block.time);
            item.target=q;item.target(3)=q(3)+.2;
            block.samples=solvePredictiveControl("outputFeedback",struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',block.time));
            angle=q(3)-item.ego(3);
            block.samples.relativeHeading=atan2(sin(angle),cos(angle));
            posterior=solvePredictiveControl("outputFeedback",struct('operation',"condition", ...
                'cfg',cfg,'initial',true,'block',block,'hypotheses',item));
            testCase.verifyFalse(posterior.targetObserver.restCompatible);
            testCase.verifyEqual(posterior.targetObserver.heading,q(3),AbsTol=1e-12);
            testCase.verifyEqual(posterior.target(3),q(3),AbsTol=1e-12);
            replay=solvePredictiveControl("outputFeedback",struct('operation',"validateInputSequence", ...
                'cfg',cfg,'hypotheses',posterior,'input',block.heldInput,'inputs',zeros(2,1)));
            testCase.assertNumElements(replay.trajectories,1);
            testCase.verifyEqual(replay.trajectories{1}.targets(3,1),q(3),AbsTol=1e-12);
        end

        function positionOnlyUpdatesCarryTheAdmittedMotionIntoTheNextHold(testCase)
            cfg=testCase.cfg;block=localBlock(cfg,8);item=block.initialHypotheses;
            initial=[12;4;.3;9;-.6;.012;1.6;2.4;.95;0;0];track=[];
            for t=0:cfg.outputFeedback.sensorPeriod:1
                q=predictiveSafetyGeometry.predictTarget(initial,t);
                track=onlineNrmmTrackingRuntime('targetPoint',track,t,q(1:2),0,cfg);
            end
            item.targetObserver=track;t=t+cfg.outputFeedback.sensorPeriod;
            item.target=predictiveSafetyGeometry.predictTarget(initial,t);
            item.target(5:6)=[5;-.1];
            block.time=t;block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',t));
            posterior=solvePredictiveControl('outputFeedback',struct('operation',"condition", ...
                'cfg',cfg,'initial',true,'block',block,'hypotheses',item));
            [~,motion]=onlineNrmmTrackingRuntime("targetPoint",track,t,item.target(1:2),item.ego(3),cfg);
            testCase.verifyEqual(posterior.target(3:6),motion,AbsTol=1e-10);
            replay=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',cfg,'hypotheses',posterior,'input',block.heldInput,'inputs',zeros(2,1)));
            root=replay.trajectories{1}.targets(:,1);
            testCase.verifyEqual(root(3:7),posterior.target(3:7),AbsTol=1e-12);
            testCase.verifyEqual(root(1:2),mean(posterior.targetObserver.positionBounds,2),AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(root(8:9),posterior.target(8:9));
            next=predictiveSafetyGeometry.predictTarget(root,cfg.controller.sampleTime);
            testCase.verifyEqual(replay.trajectories{1}.targets(:,end),next,AbsTol=1e-12);
        end

        function anExactRelativeHeadingConditionsKnownTargetHypotheses(testCase)
            cfg=testCase.cfg;block=localBlock(cfg,8);item=block.initialHypotheses;
            item.ego(3)=2.95;item.target=[12;4;-3.05;2;-.6;.035;1.6;2.4;.95;0;0];
            block.samples=solvePredictiveControl("outputFeedback",struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',0));
            block.samples.relativeHeading=item.target(3)-item.ego(3)+2*pi;
            other=item;other.target(3)=other.target(3)+.2;
            posterior=solvePredictiveControl("outputFeedback",struct('operation',"condition", ...
                'cfg',cfg,'initial',true,'block',block,'hypotheses',[item,other]));
            testCase.verifyEqual(posterior,item);
        end

        function campaignSuppliesOnlyEgoInformationBeforeFirstContact(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            run=runJointOutputFeedbackScenario(Frames=4,Speeds=25,Scenarios="headOn", ...
                RandomSeed=20261107,OutputDirectory=string(folder.Folder), ...
                ControllerConfiguration=struct('outputFeedback',struct('maximumSolveSeconds',2)));
            testCase.assertEqual(run.results.outcome,"completed");
            data=jsondecode(fileread(run.results.source));
            testCase.verifyTrue(all(arrayfun(@(s)isempty(s.target),data.initialInformationContract)));
            testCase.verifyFalse(isfield(data,'initialAcquisitionMotionPrior'));
            testCase.verifyFalse(data.initialSensorSample.radarAvailable);
            testCase.verifyTrue(all(isnan(data.initialSensorSample.relativePosition)));
            first=[];
            for k=1:numel(data.trace)
                current=data.trace(k);
                testCase.verifyFalse(current.metadata.collisionTargetActive && ~current.metadata.targetEstimateReady);
                testCase.verifyEqual(current.metadata.firstRadarTime,first);
                hits=[current.rawSensorBlock.radarAvailable];
                if isempty(first) && any(hits)
                    first=current.rawSensorBlock(find(hits,1)).time;
                end
                if ~current.metadata.targetEstimateReady
                    testCase.verifyEmpty(current.targetEstimateAtDecision);
                else
                    testCase.verifyGreaterThanOrEqual(current.targetEstimateAtDecision.detections,20);
                    testCase.verifyTrue(current.targetEstimateAtDecision.motionInitialized);
                end
            end
            testCase.verifyNotEmpty(first);
        end
        function aMovingBrakingLeadSuppliesPerceptionYawOnlyOnDetections(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            % This sensor fixture places the target outside the benchmark road.
            run=runJointOutputFeedbackScenario(Scenarios="hardBrakingLead",Speeds=15,Frames=49, ...
                Warmup=false,RequirePhysicalWitness=false,DropoutHolds=48,EgoYawOffsetRadians=.02,TargetPositionOffsetMeters=[0;15], ...
                RandomSeed=20261107,OutputDirectory=string(folder.Folder));
            testCase.assertEqual(run.results.outcome,"completed");
            data=jsondecode(fileread(run.results.source));
            seen=data.trace(47).rawSensorBlock(end);
            speed=data.initialPlantState.target(4)+data.initialPlantState.target(5)*seen.time;
            testCase.verifyGreaterThan(speed,0);
            testCase.verifyTrue(seen.radarAvailable);
            testCase.verifyEqual(seen.relativeHeading,-data.trace(47).stateAfterHold(3),AbsTol=1e-10);
            missed=data.trace(48).rawSensorBlock;
            testCase.verifyFalse(any([missed.radarAvailable]));
            testCase.verifyTrue(all(arrayfun(@(y) isempty(y.relativeHeading) || isnan(y.relativeHeading),missed)));
            seen=data.trace(49).rawSensorBlock(end);
            testCase.verifyEqual(seen.relativeHeading,-data.trace(49).stateAfterHold(3),AbsTol=1e-10);
        end
        function startupExercisesAvoidanceWithoutSeedingTheExperiment(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            for warmup=[true,false]
                campaign=runJointOutputFeedbackScenario(Frames=8,Speeds=15,Scenarios="stoppedLead", ...
                    Warmup=warmup,RandomSeed=20261107,OutputDirectory=string(folder.Folder));
                result=campaign.results;
                testCase.assertEqual(result.outcome,"completed");
                testCase.verifyEqual(result.compactSolversReadyBeforeFirstFrame,warmup);
                testCase.verifyEqual(result.warmupAvoidanceContinuation,warmup);
                testCase.verifyEqual(result.warmupFailure,"");
                testCase.verifyEqual(result.timing.calls,8);
                data=jsondecode(fileread(result.source));
                if warmup
                    prepared=data;
                    testCase.verifyGreaterThan(result.startupWarmupSeconds,0);
                else
                    testCase.verifyEqual(result.startupWarmupSeconds,0);
                    testCase.verifyEqual(prepared.initialSensorSample,data.initialSensorSample);
                    testCase.verifyEqual([prepared.trace.rawSensorBlock],[data.trace.rawSensorBlock]);
                    testCase.verifyEqual([prepared.trace.input],[data.trace.input]);
                    testCase.verifyEqual([prepared.trace.stateAfterHold],[data.trace.stateAfterHold]);
                end
            end
        end
        function firstStoppedLeadEncounterReachesZeroDeficitWithIpopt(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            campaign=runJointOutputFeedbackScenario(Frames=8,Speeds=15,Scenarios="stoppedLead", ...
                RandomSeed=20261107,OutputDirectory=string(folder.Folder));
            testCase.assertEqual(campaign.results.outcome,"completed");
            data=jsondecode(fileread(campaign.results.source));
            active=find(arrayfun(@(frame) frame.metadata.collisionTargetActive,data.trace),1);
            testCase.assertNotEmpty(active);
            info=data.trace(active).metadata;
            testCase.verifyEqual(active,find(arrayfun(@(frame)frame.metadata.targetEstimateReady,data.trace),1));
            testCase.verifyEqual(info.callBudgetSeconds,data.configuration.outputFeedback.initializationSolveSeconds);
            testCase.verifyEqual(string(info.solver),"casadiIpoptCompactPcbf");
            testCase.verifyGreaterThan(info.nlpAttempts,0);
            testCase.verifyGreaterThan(info.nlpIterations,0);
            testCase.verifyNotEqual(string(info.ipoptStatus),"notCalled");
            testCase.verifyTrue(info.nonlinearValidationPerformed && info.terminalSetEnforced);
            testCase.verifyEqual(info.pcbfValue,0,AbsTol=128*eps(1)*info.horizonSteps);
            testCase.verifyLessThanOrEqual(info.maximumConstraintViolation,data.configuration.outputFeedback.nlpTolerance);
            testCase.verifyLessThanOrEqual(info.terminalValue,data.configuration.outputFeedback.nlpTolerance);
        end
        function difficultAcquisitionAndWarmupReturnAValidatedCommand(testCase,acquisitionEncounter)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            writelines(["function varargout=fmincon(varargin)"
                "error('compactPredictiveControlTest:forbiddenSolver','Unexpected fmincon call');"
                "end"],fullfile(folder.Folder,'fmincon.m'));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(folder.Folder));
            campaign=runJointOutputFeedbackScenario(Frames=9,Speeds=15,Scenarios=acquisitionEncounter, ...
                RandomSeed=20261108,OutputDirectory=string(folder.Folder));
            data=jsondecode(fileread(campaign.results.source));
            testCase.verifyTrue(data.compactSolversReadyBeforeFirstFrame);
            testCase.verifyEmpty(data.warmupFailure);
            testCase.assertEqual(string(data.outcome),"completed",string(data.failure));
            active=find(arrayfun(@(step)step.metadata.collisionTargetActive,data.trace));
            testCase.assertNotEmpty(active);
            info=data.trace(active(end)).metadata;
            testCase.verifyTrue(info.deficitDescentMet && info.terminalSetEnforced);
            testCase.verifyEqual(data.trace(active(1)).metadata.initializationTrajectoryCount,1);
            testCase.verifyEqual(info.initializationTrajectoryCount,double(numel(active)==1));
            testCase.verifyLessThanOrEqual(info.terminalValue,data.configuration.outputFeedback.nlpTolerance);
            testCase.verifyGreaterThan(data.minimumNodeClearanceMeters,0);
        end
        function aPenetratingFieldRolloutInitializesTheDualSolve(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            campaign=runJointOutputFeedbackScenario(Frames=12,Speeds=15,Scenarios="intersectionCrossing", ...
                RandomSeed=20261107,OutputDirectory=string(folder.Folder));
            testCase.assertEqual(campaign.results.outcome,"completed");
            data=jsondecode(fileread(campaign.results.source));
            active=find(arrayfun(@(step)step.metadata.collisionTargetActive,data.trace),1);
            testCase.assertNotEmpty(active);
            info=data.trace(active).metadata;
            testCase.verifyGreaterThan(info.initialPeakClearanceDeficitMeters,data.configuration.outputFeedback.clearanceMeters);
            testCase.verifyEqual(info.initializationTrajectoryCount,1);
            testCase.verifyTrue(info.deficitDescentMet);
            testCase.verifyTrue(info.nonlinearValidationPerformed && info.terminalSetEnforced);
            testCase.verifyLessThanOrEqual(info.pcbfValue,data.configuration.outputFeedback.nlpTolerance);
            testCase.verifyLessThanOrEqual(info.maximumConstraintViolation,data.configuration.outputFeedback.nlpTolerance);
        end
        function theConnectionCanSteerAfterTheSameAvoidancePrefixAndShift(testCase)
            cfg=testCase.cfg;
            cfg.outputFeedback.horizonSeconds=.30;
            cfg.outputFeedback.initializationSolveSeconds=8;
            block=localBlock(cfg,8);
            block.initialHypotheses.target=[12;.4;0;0;0;0;1.6;2.4;.95;0;0];
            block.samples=solvePredictiveControl("outputFeedback",struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',block.initialHypotheses,'input',block.heldInput,'time',0));
            [~,inputs,prediction,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            window=prediction.metadata.predictionWindow;
            testCase.assertEqual(window.avoidanceSteps,6);
            testCase.assertGreaterThan(window.connectionSteps,0);
            testCase.verifyEqual(size(inputs,2),window.avoidanceSteps+window.connectionSteps);
            testCase.verifyTrue(ismember(window.avoidanceSteps,state.boundaries));
            testCase.verifyTrue(state.terminal.member);
            testCase.verifyEqual(state.pcbfDeficit,0,AbsTol=cfg.outputFeedback.nlpTolerance);
            law=terminalSafeSet.movingControl(cfg);
            fixed=[inputs(:,1:window.avoidanceSteps),repmat(law,1,window.connectionSteps)];
            replay=struct('operation',"validateInputSequence",'cfg',cfg,'hypotheses',prediction.information, ...
                'input',block.heldInput,'inputs',inputs);
            valid=solvePredictiveControl("outputFeedback",replay);
            testCase.verifyTrue(valid.accepted);
            replay.inputs=fixed;
            fixedResult=solvePredictiveControl("outputFeedback",replay);
            testCase.verifyFalse(fixedResult.accepted);
            cfg.outputFeedback.maximumSolveSeconds=1e-12;
            [~,shifted,next]=collisionAvoidanceController(localNextBlock(state,cfg),[],[],cfg,state);
            testCase.verifyEqual(shifted(:,1:end-1),inputs(:,2:end));
            testCase.verifyEqual(shifted(:,end),law);
            testCase.verifyTrue(next.scenarios{1}.terminal.member);
        end
        function uncertainEncounterReturnsToTheGivenCruise(testCase,uncertainEncounter)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            frames=400;
            if uncertainEncounter=="brakingLead"
                % The gently braking lead can remain in range past 20 s.
                frames=800;
            end
            campaign=runJointOutputFeedbackScenario(Scenarios=uncertainEncounter,Speeds=15, ...
                Frames=frames,RandomSeed=20261107,OutputDirectory=string(folder.Folder));
            record=campaign.results;
            testCase.assertEqual(record.outcome,"completed",record.failure);
            data=jsondecode(fileread(record.source));
            testCase.verifyGreaterThan(data.minimumNodeClearanceMeters,0);
            testCase.verifyGreaterThan(data.minimumReplayClearanceMeters,0);
            meta=[data.trace.metadata];
            testCase.verifyTrue(all([meta.terminalSetEnforced] & [meta.deficitDescentMet]));
            testCase.verifyLessThanOrEqual(max([meta.initializationTrajectoryCount]),1);
            if any(uncertainEncounter==["headOn","trafficHeadOn","curvedHeadOn","acceleratingHeadOn"])
                testCase.verifyEqual([meta.activeScenarioCount],[meta.hypothesisCount]);
                for k=1:numel(data.trace)
                    samples=data.trace(k).rawSensorBlock;
                    testCase.assertFalse(any(arrayfun(@(s) ~isempty(s.relativeHeading) && isfinite(s.relativeHeading),samples)));
                end
            end
            cfg=collisionAvoidanceControllerConfig();[x,~,road,cfg]=collisionThreatScenario(uncertainEncounter,cfg);
            stub=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
            [~,lane]=readControllerInputs(stub,[],road,cfg);
            frame=predictiveSafetyGeometry.roadFrame(lane,struct);
            trim=nonlinearBicycleModel.operatingPoint(cfg,frame(4));
            samples=round(1/cfg.controller.sampleTime);
            for k=numel(data.trace)-samples:numel(data.trace)
                err=nonlinearBicycleModel.error(data.trace(k).stateAfterHold,lane,trim);
                testCase.verifyLessThanOrEqual(abs(err),[.1;pi/180;.1;.05;.01]);
            end
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
            testCase.verifyTrue(current.retainedFinePlanUsed || current.initializationSource=="guideWithDrivingSuffix");
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
        function targetMustClearTheSensorBoundaryWithoutAnEarlierCollision(testCase)
            cfg=testCase.cfg;x=[0;0;0;3;0;0];
            target=[40;0;pi;1;0;0;1.6;2.4;.95;0;0];
            testCase.verifyGreaterThan(terminalSafeSet.movingValue(x,target,cfg),0);
            target(2)=10;
            [value,~,~,steps]=terminalSafeSet.movingValue(x,target,cfg);
            testCase.verifyLessThan(value,0);
            h=cfg.controller.sampleTime;x(1)=h*x(4);
            target=predictiveSafetyGeometry.predictTarget(target,h);
            [next,~,~,remaining]=terminalSafeSet.movingValue(x,target,cfg);
            testCase.verifyLessThanOrEqual(next,value+1e-9);
            testCase.verifyEqual(remaining,steps-1);
        end
        function crossingAtDifferentTimesCanBelongToTheTerminalSet(testCase)
            cfg=testCase.cfg;
            ego=[0;0;0;5;0;0];
            target=[15;-5;pi/2;5;0;0;1.6;2.4;.95;0;0];
            % Both centerlines pass through (15,0). The target arrives at
            % 1 s and the ego at 3 s. Delaying the target produces contact.
            [safe,~,~,exitSteps]=terminalSafeSet.movingValue(ego,target,cfg);
            testCase.verifyLessThan(safe,0);
            testCase.verifyGreaterThan(exitSteps,0);
            testCase.verifyTrue(isfinite(exitSteps));
            target(2)=-15;
            testCase.verifyGreaterThan(terminalSafeSet.movingValue(ego,target,cfg), ...
                cfg.outputFeedback.clearanceMeters);
        end
        function coMovingInRangeTargetCannotClaimExit(testCase)
            cfg=testCase.cfg;x=[0;0;0;3;0;0];
            target=[cfg.collision.encounterRangeMeters;0;0;3;0;0;1.6;2.4;.95;0;0];
            [v,~,~,steps]=terminalSafeSet.movingValue(x,target,cfg);
            testCase.verifyGreaterThan(v,cfg.outputFeedback.nlpTolerance);
            testCase.verifyEqual(steps,Inf);
            rotated=x;rotated(3)=pi/7;
            wrapped=target;wrapped(3)=rotated(3)+2*pi;
            [~,~,~,steps]=terminalSafeSet.movingValue(rotated,wrapped,cfg);
            testCase.verifyEqual(steps,Inf);
            target(1)=target(1)+.01;
            [v,~,~,steps]=terminalSafeSet.movingValue(x,target,cfg);
            testCase.verifyLessThanOrEqual(v,0);testCase.verifyEqual(steps,0);
            target(1)=10;target(4)=3.0001;
            [v,~,~,steps]=terminalSafeSet.movingValue(x,target,cfg);
            testCase.verifyLessThan(v,0);testCase.verifyGreaterThan(steps,1e6);
        end
        function aReleasedEncounterDoesNotRequirePerpetualCruise(testCase)
            cfg=testCase.cfg;cfg.actuation.brakingRatioMaximum=0;
            item=testCase.block.initialHypotheses;item.target=[];
            checked=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                'cfg',cfg,'hypotheses',item,'input',zeros(2,1),'inputs',zeros(2,1)));
            testCase.verifyTrue(checked.accepted);
            testCase.verifyEqual(checked.trajectories{1}.terminal.exitSteps,0);
        end
        function terminalCoreUsesTheCourseAndSpeedAtHandoff(testCase)
            cfg=testCase.cfg;x=[0;0;0;.5;0;0];
            target=[49;0;-.2;10;.2;.2;1.6;2.4;.95;.3;-.2];
            [value,normal,~,steps]=terminalSafeSet.movingValue(x,target,cfg);
            velocity=target(4)*predictiveSafetyGeometry.direction(target(3)+target(6))-[x(4);0];
            h=cfg.controller.sampleTime;R=cfg.collision.encounterRangeMeters;
            testCase.verifyLessThan(value,0);
            testCase.verifyLessThanOrEqual(normal.'*velocity,1e-12);
            testCase.verifyGreaterThan(norm(target(1:2)+steps*h*velocity),R);
            testCase.verifyLessThanOrEqual(norm(target(1:2)+(steps-1)*h*velocity),R);
        end
        function eachTerminalPlaneSeparatesTheWholeHandoffVelocityDisk(testCase)
            cfg=testCase.cfg;x=[0;0;.1;5;0;0];
            q=[12;8;-.12;1;0;0;1.6;2.4;.95;0;0];
            c=[1;0];r=.35;
            [value,n,g,steps]=terminalSafeSet.separatingMotion(x,q,c,r,0,cfg);
            testCase.verifyLessThan(value,0);testCase.verifyTrue(isfinite(steps));
            relative=x(4)*predictiveSafetyGeometry.direction(x(3))-c;
            testCase.verifyEqual(norm(n),1,AbsTol=1e-12);
            testCase.verifyEqual(n.'*relative,r,AbsTol=1e-12);
            for k=1:4
                dx=zeros(6,1);dx(k)=1e-6;
                plus=terminalSafeSet.separatingMotion(x+dx,q,c,r,0,cfg);
                minus=terminalSafeSet.separatingMotion(x-dx,q,c,r,0,cfg);
                testCase.verifyEqual(g(k),(plus-minus)/(2e-6),AbsTol=1e-6);
            end
            [invalid,~,~,steps]=terminalSafeSet.separatingMotion(x,q,c,norm(relative)+.1,0,cfg);
            testCase.verifyGreaterThan(invalid,cfg.outputFeedback.nlpTolerance);
            testCase.verifyEqual(steps,Inf);
        end
        function aSinglePhysicalHandoffHoldPrecedesTheAnalyticExit(testCase)
            cfg=testCase.cfg;item=testCase.block.initialHypotheses;
            item.ego=[0;0;0;.5;0;0];
            item.target=[49;0;-.2;10;0;.2;1.6;2.4;.95;0;0];
            request=struct('operation',"validateInputSequence",'cfg',cfg,'hypotheses',item, ...
                'input',zeros(2,1),'inputs',zeros(2,1));
            result=solvePredictiveControl("outputFeedback",request);
            testCase.assertTrue(result.accepted);
            term=result.trajectories{1}.terminal;
            testCase.verifyEqual(term.exitSteps,2);
            testCase.verifyEqual(term.bridgeHolds,1);
            testCase.verifyGreaterThan(term.exitSteps,term.bridgeHolds);
            request.operation="evaluateControlSequence";request.holds=1;
            evaluated=solvePredictiveControl("outputFeedback",request);
            testCase.verifyLessThanOrEqual(evaluated.terminal,cfg.outputFeedback.nlpTolerance);
        end
        function anExpiredAdmissionCannotDispatchTheTerminalBrake(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.initializationSolveSeconds=1e-12;
            block=localBlock(cfg,0);
            % Admission of an observed, separated stopped lead vehicle.
            item=block.initialHypotheses;
            item.target=[15;0;0;0;0;0;1.6;2.4;.95;0;0];
            block.initialHypotheses=item;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',0));
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],cfg,[]), ...
                'collisionAvoidanceController:compactNoFeasiblePlan');
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
            testCase.verifyEqual(cfg.outputFeedback.method,"ipopt");
            testCase.verifyEmpty(testCase.state.memory);
            testCase.verifyFalse(isfield(testCase.state,'committedPolicy'));
            testCase.verifyTrue(testCase.prediction.metadata.nonlinearValidationPerformed);
            testCase.verifyFalse(testCase.prediction.metadata.robustNonlinearSafetyCertified);
            testCase.verifyEqual(testCase.prediction.metadata.nlpVariables, ...
                2*testCase.prediction.metadata.controlNodes);
        end
        function previewGrowthReusesTheCompiledProblem(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.horizonResolutionSeconds=.5;
            cfg.outputFeedback.minimumPreviewSeconds=1;
            cfg.model.speedMaximum=8;
            [~,~,baseline]=collisionAvoidanceController(localBlock(cfg,8),[],[],cfg,[]);
            % The driving model permits speeds above the build-time sizing
            % speed. The longer horizon must span more than one compiled block.
            block=localBlock(cfg,24);
            [~,inputs,prediction]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyGreaterThan(prediction.metadata.predictionWindow.allocatedSeconds, ...
                baseline.metadata.predictionWindow.allocatedSeconds);
            testCase.verifyEqual(prediction.metadata.controlNodes,baseline.metadata.controlNodes);
            testCase.verifyTrue(prediction.metadata.nlpCacheHit);
            % Compare every boundary and integration sample with a sequence
            % of short rollouts started at their actual preceding endpoints.
            item=block.initialHypotheses;previous=block.heldInput;trace=prediction.scenarios{1};
            substeps=cfg.outputFeedback.validationSubsteps;
            for first=1:20:size(inputs,2)
                last=min(first+19,size(inputs,2));
                replay=solvePredictiveControl('outputFeedback',struct('operation',"validateInputSequence", ...
                    'cfg',cfg,'hypotheses',item,'input',previous,'inputs',inputs(:,first:last)));
                part=replay.trajectories{1};
                testCase.verifyEqual(trace.collisionEgo(:,first+1:last+1),part.collisionEgo(:,2:end),AbsTol=1e-12);
                samples=substeps*(first-1)+2:substeps*last+1;
                testCase.verifyEqual(trace.denseEgo(:,samples),part.denseEgo(:,2:end),AbsTol=1e-12);
                item.ego=part.collisionEgo(:,end);previous=inputs(:,last);
            end
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
            % Both axis gaps fall below the margin while the diagonal exceeds it.
            axisGap=.8*testCase.cfg.outputFeedback.clearanceMeters;
            center=[testCase.cfg.vehicle.length/2+2.4;testCase.cfg.vehicle.width/2+.95]+axisGap;
            item.target=[center;0;0;0;0;1.6;2.4;.95;0;0];
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
            item=testCase.block.initialHypotheses;item.ego=[0;0;0;3;0;0];
            item.target=[30;0;pi;5;0;0;1.6;2.4;.95;0;0];
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
            item=testCase.block.initialHypotheses;item.ego=[0;0;.1;3;.4;.2];item.target=[0;20;pi/2;10;0;0;1.6;2.4;.95;0;0];
            request=struct('operation',"validateInputSequence",'cfg',testCase.cfg, ...
                'hypotheses',item,'input',[.2;0],'inputs',[.2;0]);
            value=solvePredictiveControl('outputFeedback',request);
            testCase.verifyTrue(value.accepted);
            terminal=value.trajectories{1}.terminal;
            testCase.verifyEqual(terminal.input(1),0);
            testCase.verifyGreaterThan(terminal.bridgeHolds,0);
            testCase.verifyEqual(terminal.model,"safeRangeExit");
            testCase.verifyGreaterThan(terminal.input(2),0);
        end
        function terminalEntryHonorsAnExplicitSteeringSlewContract(testCase)
            cfg=testCase.cfg;cfg.outputFeedback.steeringRateMaximum=.1;
            item=testCase.block.initialHypotheses;item.ego(4:6)=[3;0;0];item.target=[0;20;pi/2;10;0;0;1.6;2.4;.95;0;0];
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
            % A previously tracked opposing vehicle exercises the nonzero
            % initial deficit independently of cold target acquisition.
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',15,'outputFeedback', ...
                struct('initializationSolveSeconds',10)));
            [ego,target,road]=collisionThreatScenario("headOn",cfg);
            block=localBlock(cfg,15);
            block.initialHypotheses.ego=ego;block.initialHypotheses.target=target;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor",'cfg',cfg, ...
                'hypothesis',block.initialHypotheses,'input',block.heldInput,'time',0));
            [~,~,prediction]=collisionAvoidanceController(block,[],road,cfg,[]);
            info=prediction.metadata;
            testCase.verifyTrue(info.restorationAttempted);
            testCase.verifyGreaterThan(info.initialRestorationSlack,0);
            testCase.verifyEqual(string(info.solver),"casadiIpoptCompactPcbf");
            testCase.verifyGreaterThan(info.nlpAttempts,0);
            testCase.verifyGreaterThan(info.nlpIterations,0);
            testCase.verifyLessThanOrEqual(info.acceptedStabilityCost,info.firstFeasibleStabilityCost);
            testCase.verifyEqual(info.pcbfValue,0);
            testCase.verifyTrue(info.terminalSetEnforced);
            testCase.verifyLessThanOrEqual(info.terminalValue,cfg.outputFeedback.nlpTolerance);
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
            testCase.verifyTrue(all([metadata.deficitDescentMet]));
            testCase.verifyLessThanOrEqual([metadata.pcbfValue],128*eps*[metadata.horizonSteps]);
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
            testCase.assumeTrue(isfile(fullfile(root,'solver','controller',['domainSensitivityKernelMex.',mexext])), ...
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
        function thePlanningMarginCanRelaxAtNodesAndRemainsHardAtTheHandoff(testCase)
            cfg=testCase.cfg;cfg.vehicle.width=2;
            margin=cfg.outputFeedback.clearanceMeters;
            testCase.verifyEqual(margin,.1);
            request=struct('operation',"validateInputSequence",'cfg',cfg, ...
                'hypotheses',testCase.block.initialHypotheses,'input',zeros(2,1),'inputs',zeros(2,1));
            % Each stationary target is alongside only at the root, on the
            % terminal bridge, or later in range under the handoff model.
            for station=[-4.75,10,42]
                for gap=[.2,margin+.01,margin,margin-.01,.014,0,-1e-9]
                    request.hypotheses.target=[station;2+gap;0;0;0;0;1.6;2.4;1;0;0];
                    result=solvePredictiveControl("outputFeedback",request);
                    if station<0
                        testCase.verifyEqual(result.accepted,gap>0);
                        testCase.verifyEqual(result.pcbfDeficit,max(0,margin-gap),AbsTol=1e-10);
                    else
                        testCase.verifyEqual(result.accepted,gap>=margin);
                    end
                    if station>0
                        testCase.verifyEqual(result.trajectories{1}.terminal.member,gap>=margin);
                    end
                end
            end
            request.cfg.outputFeedback.clearanceMeters=.3;
            request.hypotheses.target=[-4.75;2.25;0;0;0;0;1.6;2.4;1;0;0];
            result=solvePredictiveControl("outputFeedback",request);
            testCase.verifyTrue(result.accepted);
            testCase.verifyEqual(result.stageDeficits,.05,AbsTol=1e-10);
            request.operation="evaluateControlSequence";request.holds=1;
            value=solvePredictiveControl("outputFeedback",request);
            testCase.verifyEqual(value.stage,.3-.25,AbsTol=1e-10);
        end
        function aPositiveRootDeficitCanBeCommittedAndThenSpent(testCase)
            cfg=testCase.cfg;cfg.vehicle.width=2;cfg.outputFeedback.horizonSeconds=.1;
            block=localBlock(cfg,8);item=block.initialHypotheses;
            item.target=[-4.75;2.05;0;0;0;0;1.6;2.4;1;0;0];
            block.initialHypotheses=item;
            block.samples=solvePredictiveControl("outputFeedback",struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',item,'input',block.heldInput,'time',0));
            [cmd,~,prediction,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyEqual(cmd.actuatorInput,state.fineControls(:,1));
            testCase.verifyEqual(state.stageDeficits(1),.05,AbsTol=1e-10);
            testCase.verifyEqual(prediction.metadata.pcbfValue,sum(state.stageDeficits));
            [~,~,next]=collisionAvoidanceController(localNextBlock(state,cfg),[],[],cfg,state);
            testCase.verifyEqual(next.metadata.inheritedDeficitCap,sum(state.stageDeficits(2:end)));
            testCase.verifyLessThanOrEqual(next.metadata.pcbfValue,next.metadata.inheritedDeficitCap+1e-12);
        end
        function eachPhysicalMemberHasOnePersistentTargetForecast(testCase)
            cfg=testCase.cfg;item=testCase.block.initialHypotheses;
            item.target=[14;5;0;6;-1;0;1.6;2.4;1;0;0];
            item.targetObserver=struct('motionBounds',[-2,-.015;0,.015]);
            request=struct('operation',"validateInputSequence",'cfg',cfg,'hypotheses',item, ...
                'input',zeros(2,1),'inputs',zeros(2,5));
            replay=solvePredictiveControl("outputFeedback",request);
            testCase.assertNumElements(replay.trajectories,1);
            expected=predictiveSafetyGeometry.predictTarget(item.target,(0:5)*cfg.controller.sampleTime);
            testCase.verifyEqual(replay.trajectories{1}.targets,expected,AbsTol=1e-12);
            testCase.verifyFalse(replay.trajectories{1}.terminal.member);
            testCase.verifyGreaterThan(replay.trajectories{1}.terminal.value,1);
            point=request;point.hypotheses=rmfield(item,'targetObserver');
            nominal=solvePredictiveControl("outputFeedback",point);
            testCase.verifyTrue(nominal.accepted && nominal.trajectories{1}.terminal.member);
            request.operation="evaluateControlSequence";request.holds=ones(1,5);
            oracle=solvePredictiveControl("outputFeedback",request);
            testCase.verifyEqual(max(0,oracle.stage),replay.stageDeficits,AbsTol=1e-11);
        end
        function continuousMotionIntervalsContributeToThePrimaryDeficit(testCase)
            item=testCase.block.initialHypotheses;
            item.target=[10;3.4;pi;8;0;0;1.6;2.4;1;0;0];
            request=struct('operation',"validateInputSequence",'cfg',testCase.cfg,'hypotheses',item, ...
                'input',zeros(2,1),'inputs',zeros(2,28));
            nominal=solvePredictiveControl("outputFeedback",request);
            testCase.verifyTrue(nominal.accepted);
            testCase.verifyEqual(max(nominal.stageDeficits),0);
            % A permitted turn physically intersects the ego path while the
            % straight target clears it. Compare with that concrete member.
            turn=request;turn.hypotheses.target(6)=.13;
            witness=solvePredictiveControl("outputFeedback",turn);
            testCase.assertGreaterThan(max(witness.stageDeficits),.05);
            request.hypotheses.targetObserver=struct('motionBounds',[-1,-.13;1,.13]);
            replay=solvePredictiveControl("outputFeedback",request);
            testCase.assertNumElements(replay.trajectories,1);
            testCase.verifyGreaterThanOrEqual(replay.stageDeficits+1e-10,witness.stageDeficits);
            request.operation="evaluateControlSequence";request.holds=ones(1,28);
            oracle=solvePredictiveControl("outputFeedback",request);
            testCase.verifyEqual(max(0,oracle.stage),replay.stageDeficits,AbsTol=1e-11);
        end
        function measuredJointMotionBoundsEnterTheSameStageAndTerminalReplay(testCase)
            item=testCase.block.initialHypotheses;
            item.target=[10;3.4;pi;8;0;0;1.6;2.4;1;0;0];
            r=struct('operation',"validateInputSequence",'cfg',testCase.cfg,'hypotheses',item, ...
                'input',zeros(2,1),'inputs',zeros(2,28));
            straight=solvePredictiveControl("outputFeedback",r);
            r.hypotheses.targetObserver=struct('motionBounds',[-1,-.13;1,.13]);
            broad=solvePredictiveControl("outputFeedback",r);
            testCase.assertGreaterThan(broad.pcbfDeficit,straight.pcbfDeficit);
            r.hypotheses.targetObserver.motionBounds=zeros(2,2);
            restricted=solvePredictiveControl("outputFeedback",r);
            testCase.verifyLessThan(restricted.pcbfDeficit,broad.pcbfDeficit);
            testCase.verifyEqual(restricted.trajectories{1}.targets,straight.trajectories{1}.targets);
            testCase.verifyTrue(restricted.accepted && restricted.trajectories{1}.terminal.member);
            r.operation="evaluateControlSequence";r.holds=ones(1,28);
            oracle=solvePredictiveControl("outputFeedback",r);
            testCase.verifyEqual(max(0,oracle.stage),restricted.stageDeficits,AbsTol=1e-11);
        end
        function aStoredPlanCanResumeAfterReplayWithTheCurrentMargin(testCase)
            cfg=testCase.cfg;prior=testCase.state;
            for previousMargin=[0,.2]
                prior.configuration.outputFeedback.clearanceMeters=previousMargin;
                [~,~,prediction,state]=collisionAvoidanceController(localNextBlock(prior,cfg),[],[],cfg,prior);
                testCase.verifyTrue(prediction.scenarios{1}.terminal.member);
                testCase.verifyEqual(state.configuration.outputFeedback.clearanceMeters,cfg.outputFeedback.clearanceMeters);
            end
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
            % boundaries, then leaves sensing range.
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
