classdef informationStatePcbfCampaignTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addScenarioPath(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function normalTrialCountsTerminalCommandsFromItsTrace(testCase)
            outputDirectory = string(tempname);
            mkdir(outputDirectory);
            testCase.addTeardown(@() rmdir(outputDirectory,'s'));

            summary = verifyInformationStatePcbf(outputDirectory, ...
                Scenes="stationary",Estimations="exact",SampleCount=40, ...
                ForcedFailure=false,Seed=20260912);
            saved = load(fullfile(outputDirectory,"stationary-exact", ...
                "stationary-exact-state.mat"),"report");

            testCase.verifyTrue(saved.report.completed);
            testCase.verifyFalse(saved.report.solverFailureInjected);
            testCase.verifyGreaterThan(nnz(saved.report.terminalActive),0);
            testCase.verifyEqual(summary.terminalLawFrames,nnz(saved.report.terminalActive));
            testCase.verifyEqual(summary.carriedWitnessCommands,nnz(saved.report.candidateExecuted));
        end
    end
end
