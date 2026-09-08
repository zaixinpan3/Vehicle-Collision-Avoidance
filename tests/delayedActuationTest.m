classdef delayedActuationTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
        end
    end
    methods (Test)
        function certificationIncludesTheNewCommandAfterItsDelay(testCase)
            invalid = struct("controller",struct("inputDelaySteps",1,"certifiedSteps",1));
            testCase.verifyError(@() collisionAvoidanceControllerConfig(invalid), ...
                "collisionAvoidanceController:invalidConfiguration");
        end

        function anAbsentCommittedInputIsRejected(testCase)
            [ego,cfg,road] = localFixture();
            ego = rmfield(ego,"committedActuatorInput");
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,[]), ...
                "collisionAvoidanceController:missingCommittedInput");
        end

        function theFirstInputIsFixedAndTheSecondIsScheduled(testCase)
            [ego,cfg,road] = localFixture();
            [command,plan,problem,certificate] = collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyEqual(plan(:,1),ego.committedActuatorInput,AbsTol=0);
            testCase.verifyEqual(command.actuatorInput,plan(:,2),AbsTol=0);
            testCase.verifyEqual(command.delayIntervalCommand.actuatorInput,plan(:,1),AbsTol=0);
            testCase.verifyEqual(command.actuationTime,.1,AbsTol=1e-15);
            testCase.verifyEqual(certificate.appliedInput,plan(:,1),AbsTol=0);
            testCase.verifyEqual(certificate.scheduledInput,plan(:,2),AbsTol=0);
            testCase.verifyGreaterThanOrEqual(problem.metadata.certifiedDuration,.2);
            changed = problem.decision;changed(1) = changed(1)+1e-12;
            check = certifyAvoidancePlan(problem.qp,problem.prediction,problem.model,changed);
            testCase.verifyFalse(check.accepted);
            testCase.verifyEqual(check.failedConditions,"committedInput");
        end

        function aScheduledCommandCannotBeChangedAtTheNextSample(testCase)
            [ego,cfg,road] = localFixture();
            [command,~,~,certificate] = collisionAvoidanceController(ego,[],road,cfg,[]);
            ego.stateTime = .1;ego.position = [1;0];
            ego.committedActuatorInput = command.actuatorInput+[1e-4;0];
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,certificate), ...
                "collisionAvoidanceController:executionContractViolation");
        end

        function aFrameBudgetCannotExceedTheControlPeriod(testCase)
            cfg = collisionAvoidanceControllerConfig();
            testCase.verifyError(@() runFiniteBicycleDiagnostic(cfg,.1, ...
                DeadlineSeconds=.1,EnforceRuntimeDeadline=true), ...
                "runFiniteBicycleDiagnostic:deadlineExceedsPeriod");
        end

        function aCorruptedStoredScheduleCannotAuthorizeAnInput(testCase)
            [ego,cfg,road] = localFixture();
            [command,~,~,certificate] = collisionAvoidanceController(ego,[],road,cfg,[]);
            ego.stateTime = .1;ego.position = [1;0];
            certificate.scheduledInput = command.actuatorInput+[1e-4;0];
            ego.committedActuatorInput = certificate.scheduledInput;
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,certificate), ...
                "collisionAvoidanceController:invalidStoredCertificate");
        end

        function thePlantUsesThePreviousFramesScheduledCommand(testCase)
            [~,cfg] = localFixture();
            trial = runFiniteBicycleDiagnostic(cfg,.3,PrepareController=false);
            testCase.verifyFalse(trial.failure.occurred);
            testCase.verifyEqual(trial.runtime.appliedSourceFrame,[0;1;2]);
            testCase.verifyEqual(trial.input(2,:).',trial.computedCommand{1}.actuatorInput,AbsTol=0);
            testCase.verifyEqual(trial.input(3,:).',trial.computedCommand{2}.actuatorInput,AbsTol=0);
            testCase.verifyEqual(trial.runtime.scheduledActuationTime,[.1;.2;.3],AbsTol=1e-15);
        end
    end
end

function [ego,cfg,road] = localFixture()
    cfg = finiteSensingValidationConfig();
    cfg.referenceSpeed = 10;
    cfg.controller.sampleTime = .1;
    cfg.controller.horizonSteps = 8;
    cfg.controller.certifiedSteps = 2;
    cfg.controller.inputDelaySteps = 1;
    cfg = collisionAvoidanceControllerConfig(cfg);
    [~,input] = ltvBicycleModel.cruiseEquilibrium(0,cfg);
    ego = struct("position",[0;0],"yaw",0,"speed",10,"stateTime",0, ...
        "heldActuatorInput",input,"committedActuatorInput",input);
    road = [-200,0;2000,0];
end
