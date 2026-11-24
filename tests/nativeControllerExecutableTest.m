classdef nativeControllerExecutableTest < matlab.unittest.TestCase
    % Set PCBF_CONTROLLER_EXECUTABLE to test a compiled, configured package.
    methods (Test)
        function sensorBlocksDriveTheIndependentPlant(testCase)
            executable=string(getenv('PCBF_CONTROLLER_EXECUTABLE'));
            testCase.assumeTrue(isfile(executable),'Build the native controller and supply its path.');
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg=load(fullfile(fileparts(fileparts(executable)),'share','configuration.mat'),'cfg');
            campaign=runJointOutputFeedbackScenario(ControllerExecutable=executable,Scenarios="stoppedLead", ...
                Speeds=cfg.cfg.referenceSpeed,Frames=12,OutputDirectory=string(folder.Folder),RandomSeed=20261107);
            testCase.verifyEqual(campaign.results.outcome,"completed");
            testCase.verifyGreaterThan(campaign.results.minimumNodeClearanceMeters,0);
            result=jsondecode(fileread(campaign.results.source));
            metadata=[result.trace.metadata];
            testCase.verifyFalse(metadata(1).targetActive);
            testCase.verifyTrue(any([metadata.targetActive]));
            testCase.verifyGreaterThan(sum([metadata.ipoptAttempts]),0);
        end
        function continuationRequiresAnInitializedSession(testCase)
            executable=string(getenv('PCBF_CONTROLLER_EXECUTABLE'));
            testCase.assumeTrue(isfile(executable),'Build the native controller and supply its path.');
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            client=nativeControllerProcess(executable);cleanup=onCleanup(@()delete(client));
            testCase.verifyError(@()client.exchange(struct('type',"step")), ...
                'nativeControllerProcess:refused');
        end
    end
end
