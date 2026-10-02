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
    end
end
