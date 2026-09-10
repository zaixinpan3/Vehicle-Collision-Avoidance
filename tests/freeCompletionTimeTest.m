classdef freeCompletionTimeTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function completeWitnessCanExceedTheConfiguredWindow(testCase)
            [ego,target,route,cfg]=localFixture();
            [command,~,problem,stored]=collisionAvoidanceController(ego,target,route,cfg,[]);
            testCase.verifyNotEmpty(command);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThan(stored.prediction.stageCount,cfg.controller.horizonSteps);
            testCase.verifyGreaterThan(stored.acceptance.exitMargin,0);
            testCase.verifyEqual(problem.metadata.planningWindowSteps,2);
            testCase.verifyGreaterThan(problem.metadata.certificateExtensionSteps,0);
        end
        function aOnePointSixSecondWindowAllowsALaterCertifiedExit(testCase)
            [ego,target,route,cfg]=localFixture();
            cfg.controller.horizonSteps=16;
            ego.perception.range=37;
            [~,~,problem,stored]=collisionAvoidanceController(ego,target,route,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThan(stored.certifiedDuration,1.6);
            testCase.verifyGreaterThan(stored.acceptance.exitMargin,0);
            testCase.verifyEqual(problem.metadata.planningWindowSteps,16);
        end
        function solverFailureCannotEndAnEncounterAtTheShortWindow(testCase)
            result=localContinueEncounter();
            testCase.verifyGreaterThan(result.duration,result.windowDuration);
            testCase.verifyTrue(all(result.certified));
            testCase.verifyTrue(result.commandBeyondWindow);
            testCase.verifyTrue(result.completed);
            testCase.verifyGreaterThanOrEqual(result.finalDistance,13.6);
        end
        function targetFreeExecutionSurvivesSeveralWindows(testCase)
            result=localCruise(false);
            testCase.verifyTrue(result.allCommands);
            testCase.verifyFalse(result.stored.encounterComplete);
            testCase.verifyEqual(result.stored.stateTime,0.6,AbsTol=1e-12);
            testCase.verifyGreaterThan(result.stored.deadline,result.stored.stateTime);
        end
        function lateFirstDetectionDoesNotInheritACruiseExpiry(testCase)
            result=localCruise(true);
            testCase.verifyTrue(result.allCommands);
            testCase.verifyTrue(result.problem.metadata.jointAdmissionPerformed);
            testCase.verifyGreaterThan(result.stored.certifiedDuration,0.2);
            testCase.verifyEqual(result.stored.admissionTime,0.7,AbsTol=1e-12);
            testCase.verifyGreaterThan(result.stored.acceptance.exitMargin,0);
        end
        function aCoarseMeasurementDoesNotEraseCertifiedTerminalExit(testCase)
            result=localTerminalObservation(false);
            testCase.verifyTrue(result.stored.encounterComplete);
            testCase.verifyEmpty(result.command);
            testCase.verifyTrue(result.problem.metadata.acceptance.accepted);
        end
        function aNewDetectionAfterOldExitStartsAFreshCertificate(testCase)
            result=localTerminalObservation(true);
            testCase.verifyNotEmpty(result.command);
            testCase.verifyFalse(result.stored.encounterComplete);
            testCase.verifyTrue(result.problem.metadata.jointAdmissionPerformed);
            testCase.verifyGreaterThan(result.stored.deadline,result.oldDeadline);
        end
        function targetMotionValidityIsNotInventedFromWindowLength(testCase)
            [ego,target,route,cfg]=localFixture();
            [~,~,~,stored]=collisionAvoidanceController(ego,target,route,cfg,[]);
            testCase.verifyFalse(isfield(stored.encounters.contract,'validUntil'));
            testCase.verifyEqual(stored.encounters.contract.validityScope,"whileEncounterActive");
            testCase.verifyEqual(stored.encounters.contract.validFrom,0);
        end
    end
end

function [ego,target,route,cfg]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('horizonSteps',2,'sampleTime',0.1), ...
        'model',struct('linearizationPolicy','cruise','lateralDomainRadius',2)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0, ...
        'perception',struct('time',0,'range',13.5,'completeWithinRange',true));
    target=struct('trackId',1,'targetPositionInertial',[-8;0], ...
        'targetVelocityInertial',[-8;0],'targetAccelerationInertial',[0;0], ...
        'targetHeadingInertial',pi,'targetYawRate',0, ...
        'predictionMotion',struct('kind','finite-sensing-motion-v1','jerkBound',[0;0],'yawAccelerationBound',0));
    route=[-100,0;2000,0];
end

function result=localContinueEncounter()
    [ego,target,route,cfg]=localFixture();
    [~,~,problem,stored]=collisionAvoidanceController(ego,target,route,cfg,[]);
    count=stored.remainingSteps;certified=false(1,count);commandBeyondWindow=false;
    duration=stored.certifiedDuration;
    cfg.solver.jointFunction=@encounterTestFixture.fail;
    for step=1:count
        ego=encounterTestFixture.nextEgo(stored,problem.model.lane);
        ego.perception.completeWithinRange=true;
        target.targetPositionInertial=[-8-8*ego.stateTime;0];
        [command,~,problem,stored]=collisionAvoidanceController(ego,target,route,cfg,stored);
        certified(step)=problem.metadata.planCertified || stored.encounterComplete;
        if step==cfg.controller.horizonSteps,commandBeyondWindow=~isempty(command);end
    end
    result=struct('duration',duration,'windowDuration',cfg.controller.sampleTime*cfg.controller.horizonSteps, ...
        'certified',certified,'commandBeyondWindow',commandBeyondWindow,'completed',stored.encounterComplete, ...
        'finalDistance',norm(ego.position-target.targetPositionInertial));
end

function result=localCruise(acquire)
    [ego,target,route,cfg]=localFixture();
    [command,~,problem,stored]=collisionAvoidanceController(ego,[],route,cfg,[]);
    allCommands=~isempty(command);
    for step=1:6
        ego=encounterTestFixture.nextEgo(stored,problem.model.lane);
        ego.perception.completeWithinRange=true;
        [command,~,problem,stored]=collisionAvoidanceController(ego,[],route,cfg,stored);
        allCommands=allCommands && ~isempty(command);
    end
    if acquire
        ego=encounterTestFixture.nextEgo(stored,problem.model.lane);
        ego.perception.completeWithinRange=true;
        target.targetPositionInertial=ego.position+[-8;0];
        [command,~,problem,stored]=collisionAvoidanceController(ego,target,route,cfg,stored);
        allCommands=allCommands && ~isempty(command);
    end
    result=struct('allCommands',allCommands,'stored',stored,'problem',problem);
end

function result=localTerminalObservation(acquire)
    [ego,target,route,cfg]=localFixture();
    [~,~,problem,stored]=collisionAvoidanceController(ego,target,route,cfg,[]);
    count=stored.remainingSteps;oldDeadline=stored.deadline;
    for step=1:count
        ego=encounterTestFixture.nextEgo(stored,problem.model.lane);
        ego.perception.completeWithinRange=true;
        target.targetPositionInertial=[-8-8*ego.stateTime;0];
        observations=target;
        if step==count
            if acquire
                nextTarget=target;nextTarget.trackId=2;
                nextTarget.targetPositionInertial=ego.position+[-13.4;0];
                observations=[target;nextTarget];
            else
                ego.controllerStateErrorBound=[1;1;0;0;0;0];
            end
        end
        [command,~,problem,stored]=collisionAvoidanceController(ego,observations,route,cfg,stored);
    end
    result=struct('command',command,'problem',problem,'stored',stored,'oldDeadline',oldDeadline);
end
