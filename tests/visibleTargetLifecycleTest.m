classdef visibleTargetLifecycleTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function noTargetRetainsClfAndPhysicalControl(testCase)
            [ego,~,road,cfg] = encounterTestFixture.crossing();
            ego.speed = 7;
            [command,~,p,c] = collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(p.metadata.planCertified);
            testCase.verifyFalse(p.metadata.hasTarget);
            testCase.verifyEmpty(c.encounters);
            testCase.verifySize(p.metadata.targetErrorBound,[8,0]);
            testCase.verifyGreaterThan(p.metadata.clfInitialValue,0);
            testCase.verifyGreaterThan(command.actuatorInput(2),0);
            testCase.verifyLessThanOrEqual(abs(command.actuatorInput(1)),cfg.model.frontWheelSteeringAngleMaximum);
        end
        function noTargetContinuationConditionsTheExistingWitness(testCase)
            [ego,~,road,cfg,c] = localContinuation(false);
            [~,~,p] = collisionAvoidanceController(ego,[],road,cfg,c);
            testCase.verifyTrue(p.metadata.candidateVerified);
            testCase.verifyFalse(p.metadata.targetSetChanged);
            testCase.verifyLessThanOrEqual(p.metadata.pcbfDescentResidual,0);
        end
        function arrivalRequiresANewCertificateForTheVisibleTarget(testCase)
            [ego,target,road,cfg,c] = localContinuation(false);
            [~,~,p,next] = collisionAvoidanceController(ego,target,road,cfg,c);
            testCase.verifyTrue(p.metadata.hasTarget);
            testCase.verifyTrue(p.metadata.targetSetChanged);
            testCase.verifyFalse(p.metadata.candidateVerified);
            testCase.verifyTrue(isnan(p.metadata.pcbfDescentResidual));
            testCase.verifyEqual(p.metadata.certificateSource,"checkedOptimization");
            testCase.verifyEqual(p.metadata.newlyAdmittedTargetKeys,string({next.encounters.key}));
        end
        function arrivalFailureDoesNotExecuteTheNoTargetWitness(testCase)
            [ego,target,road,cfg,c] = localContinuation(false);
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,c), ...
                'collisionAvoidanceController:noCertifiedContinuation');
        end
        function departureRetainsTheVerifiedRoadContinuation(testCase)
            [ego,~,road,cfg,c] = localContinuation(true);
            ego.perception = struct('time',ego.stateTime,'range',16,'completeWithinRange',true);
            [~,~,p,next] = collisionAvoidanceController(ego,[],road,cfg,c);
            testCase.verifyFalse(p.metadata.hasTarget);
            testCase.verifyTrue(p.metadata.targetSetChanged);
            testCase.verifyEmpty(next.encounters);
            testCase.verifyEqual(p.metadata.dischargedTargetKeys,string({c.encounters.key}));
            testCase.verifyTrue(p.metadata.candidateVerified);
            testCase.verifyTrue(p.metadata.planCertified);
        end
        function aStaleVisibilityDeclarationCannotReleaseATarget(testCase)
            [ego,~,road,cfg,c] = localContinuation(true);
            ego.perception = struct('time',0,'range',16,'completeWithinRange',true);
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,c), ...
                'collisionAvoidanceController:unconfirmedTargetDeparture');
        end
        function arrivalCannotBypassTheAppliedInputContract(testCase)
            [ego,target,road,cfg,c] = localContinuation(false);
            ego.heldActuatorInput = c.appliedInput+[0;0.01];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,c), ...
                'collisionAvoidanceController:executionContractViolation');
        end
        function noTargetDoesNotDiscardModelResiduals(testCase)
            [ego,~,road,cfg] = encounterTestFixture.crossing();
            cfg.model.plantModelResidualRateBound(4) = 0.01;
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:nonexactStudyInput');
        end
    end
end

function [ego,target,road,cfg,c] = localContinuation(hasTarget)
    [ego,target,road,cfg] = encounterTestFixture.crossing();
    if hasTarget, target.targetVelocityInertial = [0;300]; end
    initial = target;
    if ~hasTarget, initial = []; end
    [~,~,p,c] = collisionAvoidanceController(ego,initial,road,cfg,[]);
    ego = encounterTestFixture.nextEgo(c,p.model.lane);
    ego.perception = struct('time',ego.stateTime,'range',16,'completeWithinRange',true);
    target.targetPositionInertial = target.targetPositionInertial+cfg.controller.sampleTime*target.targetVelocityInertial;
end
