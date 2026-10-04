classdef nominalRecoveryValidationTest < matlab.unittest.TestCase
    % Recovery is an observed dwell in all transverse coordinates.
    methods (TestClassSetup)
        function preparePaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function defaultRecoveryHasNoArbitraryMinimumDuration(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=12, ...
                RecoveryDwellSeconds=.1);
            testCase.verifyEqual(report.results.recovery.minimumTimeSeconds,0);
            testCase.verifyTrue(report.results.recovery.recovered);
            testCase.verifyLessThan(report.results.executedFrames,12);
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
        function defaultFixedDurationDoesNotClaimRecovery(testCase)
            report=runNonlinearPredictiveSafetyValidation(Scenarios="recovery",Frames=2);
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
        function anActualOptimizationFailureRemainsAnExperimentalFailure(testCase)
            cfg=struct('referenceSpeed',15,'controller',struct('horizonSteps',16));
            report=runNonlinearPredictiveSafetyValidation(Scenarios="oncoming",Frames=1, ...
                ControllerConfiguration=cfg,TargetEstimateFunction=@localImpossibleTarget);
            testCase.verifyTrue(report.results.controlUnavailable);
            testCase.verifyTrue(report.results.experimentFailed);
            testCase.verifyFalse(report.results.collisionDetected);
            testCase.verifyEqual(report.results.outcome,"controlUnavailable");
        end
    end
end

function target=localRevisedTarget(target,ego)
    time=ego.stateTime;
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
