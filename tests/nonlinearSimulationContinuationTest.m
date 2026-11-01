classdef nonlinearSimulationContinuationTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for directory=["controller","config","estimator","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,directory)));
            end
        end
    end
    methods (Test)
        function checkpointDoesNotChangeTheObservedMotion(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            file=string(fullfile(folder.Folder,'continuation.mat'));
            cfg=struct('referenceSpeed',8,'controller',struct('horizonSteps',8), ...
                'solver',struct('method',"nonlinear",'timeLimitSeconds',120));
            options={'Scenarios',"brakingTarget",'ControllerConfiguration',cfg, ...
                'EstimatorConfiguration',estimatorControllerIntegrationConfig()};
            continuous=runNonlinearPredictiveSafetyValidation(options{:},Frames=2);
            testCase.assertEqual(continuous.results.failure,"");
            first=runNonlinearPredictiveSafetyValidation(options{:},Frames=1,ContinuationFile=file);
            testCase.assertEqual(first.results.failure,"");
            resumed=runNonlinearPredictiveSafetyValidation(options{:},Frames=2,ResumeFrom=file);
            testCase.assertEqual(resumed.results.failure,"");
            testCase.verifyEqual(resumed.results.trace(2).estimatedState, ...
                continuous.results.trace(2).estimatedState,AbsTol=1e-11);
            testCase.verifyEqual(resumed.results.trace(2).input, ...
                continuous.results.trace(2).input,AbsTol=1e-8);
            saved=load(file,'continuation');continuation=rmfield(saved.continuation,'truthPath');
            save(file,'continuation','-v7.3');
            legacy=runNonlinearPredictiveSafetyValidation(options{:},Frames=2,ResumeFrom=file);
            testCase.assertEqual(legacy.results.failure,"");
            testCase.verifyEqual(legacy.results.trace(2).estimatedState, ...
                continuous.results.trace(2).estimatedState,AbsTol=1e-11);
            testCase.verifyEqual(legacy.results.trace(2).input, ...
                continuous.results.trace(2).input,AbsTol=1e-8);
        end
    end
end
