classdef immediateActuationTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            for folder = ["controller","config","scripts","tests"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,folder)));
            end
        end
    end
    methods (Test)
        function aQueuedInputCannotAcquireTheImmediateExecutionGuarantee(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            ego.committedActuatorInput = [0;0];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,route,cfg,[]), ...
                "collisionAvoidanceController:invalidInput");
        end
        function commandExecutesTheFirstCertifiedHeldInterval(testCase)
            [ego,target,route,cfg] = encounterTestFixture.crossing();
            [command,plan,~,certificate] = collisionAvoidanceController(ego,target,route,cfg,[]);
            testCase.verifyEqual(command.actuatorInput,plan(:,1),AbsTol=0);
            testCase.verifyEqual(command.actuationTime,ego.stateTime,AbsTol=0);
            testCase.verifyEqual(certificate.appliedInput,plan(:,1),AbsTol=0);
        end
        function aFrameBudgetCannotExceedTheControlPeriod(testCase)
            cfg = collisionAvoidanceControllerConfig();
            testCase.verifyError(@() runFiniteBicycleDiagnostic(cfg,.1, ...
                DeadlineSeconds=.1,EnforceRuntimeDeadline=true), ...
                "runFiniteBicycleDiagnostic:deadlineExceedsPeriod");
        end
        function aTargetFreeCertificateEndsWithoutApplyingAnEmptyCommand(testCase)
            cfg = collisionAvoidanceControllerConfig(struct("controller", ...
                struct("horizonSteps",2,"sampleTime",0.1), ...
                "model",struct("linearizationPolicy","cruise", ...
                    "plantModelResidualRateBound",1e-3*ones(6,1)),"referenceSpeed",10));
            result = runFiniteBicycleDiagnostic(cfg,.4,IncludeTarget=false,PrepareController=false);
            testCase.verifyFalse(result.failure.occurred);
            testCase.verifyEqual(size(result.input,1),2);
            testCase.verifyEqual(result.runtime.appliedSourceFrame,[1;2]);
        end
    end
end
