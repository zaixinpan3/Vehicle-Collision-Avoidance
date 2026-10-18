classdef nominalRecoveryValidationTest < matlab.unittest.TestCase
    % Terminal certificates do not end the task; nominal recovery uses a dwell.
    methods (TestClassSetup)
        function preparePaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function defaultObservationHasNoFrameLimitAndStopsOnRecovery(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",RecoveryDwellSeconds=.1);
            r=report.results;
            testCase.verifyEqual(r.requestedFrames,Inf);
            testCase.verifyFalse(report.observationLimitEnabled);
            testCase.verifyTrue(r.recovery.recovered);
            testCase.verifyEqual(r.outcome,"recovered");
            testCase.verifyTrue(isfinite(r.baselineCruise.durationSeconds));
            testCase.verifyFalse(r.roadConstraintsEnforced);
            testCase.verifyEqual(r.controllerBudgetSeconds,.1);
            testCase.verifyEqual(r.sampleTimeSeconds,.05);
            seconds=[r.trace.controllerSeconds,r.failedFrameSeconds];
            testCase.verifyEqual(r.controllerBudgetExceededCalls,nnz(seconds>.1));
        end
        function aLimitedObservationCanContinueWithoutAFrameLimit(testCase)
            file=string(tempname)+".mat";testCase.addTeardown(@()delete(file));
            first=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=2, ...
                RecoveryDwellSeconds=.2,ContinuationFile=file,StateTransition="nominalRk4");
            resumed=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=Inf, ...
                RecoveryDwellSeconds=.2,ResumeFrom=file,StateTransition="nominalRk4");
            r=resumed.results;
            testCase.verifyTrue(r.recovery.recovered);
            testCase.verifyFalse(resumed.observationLimitEnabled);
            testCase.verifyEqual([r.trace(1:2).nextState],[first.results.trace.nextState]);
            testCase.verifyEqual(r.baselineCruise,first.results.baselineCruise);
        end
        function frameLimitsMustBeWholeNumbersOrUnlimited(testCase)
            testCase.verifyError(@()runNonlinearPredictiveSafetyValidation( ...
                Scenarios="recovery",Frames=1.5), ...
                'runNonlinearPredictiveSafetyValidation:invalidFrameLimit');
        end
        function anInitiallyTerminalCruiseStateStillRunsPredictiveControl(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=4);
            r=report.results;x=r.trace(1).estimatedState;cfg=r.configuration;
            road=struct('centerline',[-100,0;1000,0]);
            [~,lane,metadata]=readControllerInputs(struct('position',x(1:2),'yaw',x(3), ...
                'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6)),[],road,cfg);
            model=struct('cfg',cfg,'initialState',x,'lane',lane,'road',metadata, ...
                'target',[],'targetEpoch',[],'sampleIndex',0, ...
                'nominalReference',nonlinearBicycleModel.cruise(cfg,0));
            testCase.verifyTrue(terminalSafeSet.member(terminalSafeSet.context(model),x,0));
            testCase.verifyEqual(r.executedFrames,4);
            testCase.verifyEqual(r.outcome,"observationLimit");
            testCase.verifyTrue(all([r.trace.optimizationReturned]));
            testCase.verifyEqual([r.trace.absoluteSampleIndex],0:3);
            testCase.verifyGreaterThan(r.trace(end).nextState(4),cfg.referenceSpeed-.1);
        end
        function aForecastTerminalEndpointDoesNotCompleteTheDrivingTask(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="crossing",Frames=1, ...
                ControllerConfiguration=struct('solver',struct('timeLimitSeconds',5), ...
                'controller',struct('lateralDeviationMaximumMeters',Inf)));
            testCase.verifyEqual(report.results.executedFrames,1);
            testCase.verifyEqual(report.results.outcome,"observationLimit");
            testCase.verifyFalse(report.results.recovery.recovered);
        end
        function oldTerminalEntryScoresCannotResumeAsDrivingObservations(testCase)
            file=string(tempname)+".mat";testCase.addTeardown(@()delete(file));
            runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=1,ContinuationFile=file);
            saved=load(file,'continuation');continuation=saved.continuation;
            continuation.result.terminalHandoff=struct('enabled',true);
            continuation.prior=[];continuation.result.executedFrames=0;save(file,'continuation');
            testCase.verifyError(@()runNonlinearPredictiveSafetyValidation( ...
                Scenarios="recovery",Frames=2,ResumeFrom=file), ...
                'runNonlinearPredictiveSafetyValidation:incompatibleTaskSemantics');
        end
        function defaultRecoveryHasNoArbitraryMinimumDuration(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=12, ...
                RecoveryDwellSeconds=.1);
            testCase.verifyEqual(report.results.recovery.minimumTimeSeconds,0);
            testCase.verifyTrue(report.results.recovery.recovered);
            testCase.verifyLessThan(report.results.executedFrames,12);
        end
        function continuationsWithTheOldPlantTransitionAreRejected(testCase)
            file=string(tempname)+".mat";testCase.addTeardown(@()delete(file));
            report=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=1,ContinuationFile=file);
            testCase.verifyEqual(report.replay.sampleEndReset,"brakedSpeedBelowOneFifthHoldSpeed-v1");
            saved=load(file,'continuation');continuation=rmfield(saved.continuation,'sampleEndReset');
            save(file,'continuation');
            testCase.verifyError(@()runNonlinearPredictiveSafetyValidation( ...
                Scenarios="recovery",Frames=2,ResumeFrom=file), ...
                'runNonlinearPredictiveSafetyValidation:incompatiblePlantTransition');
        end
        function recoveryRequiresMinimumTimeAndSustainedDwell(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=12, ...
                RecoveryDwellSeconds=.1,RecoveryMinimumSeconds=.2);
            result=report.results;
            testCase.verifyTrue(result.completed);
            testCase.verifyTrue(result.recovery.recovered);
            testCase.verifyLessThan(result.executedFrames,result.requestedFrames);
            testCase.verifyGreaterThanOrEqual(result.recovery.entryTimeSeconds,.2);
            testCase.verifyGreaterThanOrEqual(result.recovery.confirmationTimeSeconds, .3-1e-10);
        end
        function completedDurationDoesNotMeanRecovered(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=2, ...
                RecoveryDwellSeconds=.1,RecoveryMinimumSeconds=0,RecoveryTolerances=1e-9*ones(5,1));
            testCase.verifyTrue(report.results.completed);
            testCase.verifyFalse(report.results.recovery.recovered);
            testCase.verifyEqual(report.results.executedFrames,2);
            testCase.verifyFalse(report.results.experimentFailed);
            testCase.verifyEqual(report.results.outcome,"observationLimit");
        end
        function explicitFixedDurationDoesNotClaimRecovery(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=2,RecoveryDwellSeconds=0);
            testCase.verifyTrue(report.results.completed);
            testCase.verifyFalse(report.results.recovery.enabled);
            testCase.verifyFalse(report.results.recovery.recovered);
        end
        function extendingDurationPreservesTheControllerAndPlantState(testCase)
            file=string(tempname)+".mat";
            testCase.addTeardown(@()delete(file));
            runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=2, ...
                StateTransition="nominalRk4",ContinuationFile=file);
            resumed=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=4, ...
                StateTransition="nominalRk4",ResumeFrom=file);
            uninterrupted=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=4, ...
                StateTransition="nominalRk4");
            testCase.verifyEqual([resumed.results.trace.nextState],[uninterrupted.results.trace.nextState]);
            testCase.verifyEqual([resumed.results.trace.input],[uninterrupted.results.trace.input]);
            testCase.verifyEqual([resumed.results.trace.absoluteSampleIndex],[uninterrupted.results.trace.absoluteSampleIndex]);
        end
        function incompatibleContinuationIsRejectedBeforeAdvancingThePlant(testCase)
            file=string(tempname)+".mat";
            testCase.addTeardown(@()delete(file));
            runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=1,ContinuationFile=file);
            saved=load(file,'continuation');continuation=saved.continuation;
            continuation.result.trace=rmfield(continuation.result.trace,'terminalDistanceMeters');
            save(file,'continuation');
            testCase.verifyError(@()runNonlinearPredictiveSafetyValidation( ...
                Scenarios="recovery",Frames=2,ResumeFrom=file), ...
                'runNonlinearPredictiveSafetyValidation:incompatibleTraceSchema');
        end
        function revisedUncertifiedForecastsDoNotChangeTargetTruthOrFailTheExperiment(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="oncoming",Frames=4, ...
                TargetEstimateFunction=@localRevisedTarget);
            r=report.results;
            testCase.verifyEqual(r.targetInitialState(5:6),zeros(2,1),AbsTol=0);
            testCase.verifyEqual(r.targetInitialState(2),0,AbsTol=0);
            testCase.verifyEqual(r.executedFrames,4);
            testCase.verifyFalse(r.experimentFailed);
            testCase.verifyEqual(r.outcome,"observationLimit");
            testCase.verifyTrue(all([r.trace.optimizationReturned]));
        end
        function aCollisionWithTruthFailsEvenWhenTheEstimatedTargetIsFarAway(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="oncoming",Frames=40, ...
                TargetEstimateFunction=@localFarTarget);
            r=report.results;
            testCase.verifyTrue(r.collisionDetected);
            testCase.verifyTrue(r.experimentFailed);
            testCase.verifyFalse(r.controlUnavailable);
            testCase.verifyFalse(r.completed);
            testCase.verifyEqual(r.outcome,"collision");
            testCase.verifyLessThan(r.executedFrames,40);
            testCase.verifyEqual(r.minimumReplayClearanceMeters,0,AbsTol=1e-12);
            testCase.verifyTrue(all([r.trace.optimizationReturned]));
        end
        function failedControllerTimeExcludesEarlierPipelineWork(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="oncoming",Frames=1, ...
                TargetEstimateFunction=@localDelayedImpossibleTarget);
            result=report.results;
            testCase.verifyTrue(result.controlUnavailable);
            testCase.verifyGreaterThanOrEqual(result.failedPipelineSeconds,result.failedFrameSeconds+.02);
            testCase.verifyEqual(result.deadlineMisses,double(result.failedFrameSeconds>.05));
            testCase.verifyEqual(result.pipelineDeadlineMisses,double(result.failedPipelineSeconds>.05));
        end
        function anActualOptimizationFailureRemainsAnExperimentalFailure(testCase)
            cfg=struct('referenceSpeed',15,'controller',struct('horizonSteps',16));
            report=runNonlinearPredictiveSafetyValidation(Scenarios="oncoming",Frames=1, ...
                ControllerConfiguration=cfg,TargetEstimateFunction=@localImpossibleTarget);
            testCase.verifyTrue(report.results.controlUnavailable);
            testCase.verifyGreaterThanOrEqual(report.results.failedPipelineSeconds,report.results.failedFrameSeconds);
            testCase.verifyTrue(report.results.experimentFailed);
            testCase.verifyFalse(report.results.collisionDetected);
            testCase.verifyEqual(report.results.outcome,"controlUnavailable");
        end
    end
end

function target=localRevisedTarget(target,ego)
    time=ego.stateTime;
    % Keep this information-flow fixture locally feasible for one-step RTI.
    % The physical target remains on the oncoming centerline.
    target.targetPositionInertial(2)=target.targetPositionInertial(2)+6;
    target.targetTangentialAcceleration=.02*cos(9*time);
    target.targetSideslip=.001*sin(9*time);
    course=target.targetYawInertial+target.targetSideslip;
    target.targetVelocityInertial=norm(target.targetVelocityInertial)*[cos(course);sin(course)];
    target.controllerErrorBound=struct('kind',"target-state-v1",'time',time, ...
        'bounds',Inf(8,1),'available',false);
end

function target=localFarTarget(target,~)
    target.targetPositionInertial(2)=1000;
end

function target=localImpossibleTarget(~,~)
    % A numerical constraint contradiction, rather than a missing theorem.
    target=struct('targetPositionInertial',[20;0],'targetVelocityInertial',[-15;0], ...
        'targetYawInertial',pi,'targetTangentialAcceleration',0,'targetSideslip',0, ...
        'targetLength',20000,'targetWidth',20000);
end

function target=localDelayedImpossibleTarget(targetTruth,egoTruth)
    pause(.03);
    target=localImpossibleTarget(targetTruth,egoTruth);
end
