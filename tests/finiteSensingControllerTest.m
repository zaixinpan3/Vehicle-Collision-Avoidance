classdef finiteSensingControllerTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            for folder = ["controller","config","tests"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,folder)));
            end
        end
    end
    methods (Test)
        function exactConstantVelocityUpdatesSurviveArithmeticRoundoff(testCase)
            [ego,target,route,cfg] = localFixture();
            target.predictionMotion.jerkBound = [0;0];
            target.predictionMotion.yawAccelerationBound = 0;
            [~,lane,~,parsed] = readPlanningInputs(ego,target,route,cfg);
            encounter = targetPrediction.admit(parsed,0,lane,cfg);
            position = parsed.position;
            for step = 1:150
                parsed.position = position+step*cfg.controller.sampleTime*parsed.velocity;
                encounter = targetPrediction.advance(encounter,cfg.controller.sampleTime,parsed,lane,cfg);
            end
            testCase.verifyEqual(encounter.center(1:2),parsed.position,AbsTol=1e-12);
            testCase.verifyLessThan(max(encounter.radius),1e-9);
        end
        function completePerceptionCannotOmitAStillVisibleTarget(testCase)
            [ego,target,route,cfg] = localFixture();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            next = encounterTestFixture.nextEgo(stored,problem.model.lane);
            next.perception = struct("time",next.stateTime,"range",30,"completeWithinRange",true);
            testCase.verifyError(@() collisionAvoidanceController(next,[],route,cfg,stored), ...
                "collisionAvoidanceController:inconsistentPerception");
        end
        function aCompatibleNominalPredictionSurvivesNoisyVelocityUpdates(testCase)
            [ego,target,route,cfg] = localFixture();
            [~,lane,~,parsed] = readPlanningInputs(ego,target,route,cfg);
            parsed.velocityErrorBound = ones(2,1);
            encounter = targetPrediction.admit(parsed,0,lane,cfg);
            expected = targetPrediction.nominalFlow(encounter,0.1);
            parsed.position = expected(1:2);
            parsed.velocity(2) = parsed.velocity(2)+0.2;
            updated = targetPrediction.advance(encounter,0.1,parsed,lane,cfg);
            testCase.verifyEqual(updated.center(3:4),parsed.velocity,AbsTol=0);
            testCase.verifyEqual(updated.nominalCenter,expected,AbsTol=1e-12);
            parsed.velocity(2) = parsed.velocity(2)+0.3;
            parsed.velocityErrorBound = 0.01*ones(2,1);
            replaced = targetPrediction.advance(encounter,0.1,parsed,lane,cfg);
            testCase.verifyEqual(replaced.nominalCenter,replaced.center,AbsTol=0);
        end
        function localMotionDoesNotRequireAPerpetualExitRoute(testCase)
            [ego,target,route,cfg] = localFixture();
            [~,~,problem,certificate] = collisionAvoidanceController(ego,target,route,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyFalse(problem.metadata.recursiveFeasibilityClaimed);
            testCase.verifyEqual(certificate.safetyScope,"executedIntervalWithNominalLookahead");
            testCase.verifyEqual(certificate.certifiedDuration,cfg.controller.sampleTime);
            testCase.verifyGreaterThan(problem.metadata.lookaheadDuration,certificate.certifiedDuration);
        end
        function completeCurrentPerceptionEndsAnAbsentEncounter(testCase)
            [ego,target,route,cfg] = localFixture();
            target.targetPositionInertial = [-29.5;0];
            target.targetVelocityInertial = [-8;0];
            target.targetHeadingInertial = pi;
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            next = encounterTestFixture.nextEgo(stored,problem.model.lane);
            next.perception = struct("time",next.stateTime,"range",30,"completeWithinRange",true);
            [~,~,updated,certificate] = collisionAvoidanceController(next,[],route,cfg,stored);
            testCase.verifyTrue(certificate.encounters.discharged);
            testCase.verifyEmpty(updated.metadata.activeTargetKeys);
            testCase.verifyFalse(updated.metadata.fallbackUsed);
            testCase.verifyEqual(updated.metadata.certificateSource,"checkedOptimization");
        end
        function aMissingObservationAloneDoesNotEndAnEncounter(testCase)
            [ego,target,route,cfg] = localFixture();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            next = encounterTestFixture.nextEgo(stored,problem.model.lane);
            testCase.verifyError(@() collisionAvoidanceController(next,[],route,cfg,stored), ...
                "collisionAvoidanceController:expiredEncounterContract");
        end
        function solverFailureStillStopsTheFiniteSensingController(testCase)
            [ego,target,route,cfg] = localFixture();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            next = encounterTestFixture.nextEgo(stored,problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial+cfg.controller.sampleTime*target.targetVelocityInertial;
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            testCase.verifyError(@() collisionAvoidanceController(next,target,route,cfg,stored), ...
                "collisionAvoidanceController:noCertifiedContinuation");
        end
        function nonfinitePerceptionTimeCannotAuthorizeDischarge(testCase)
            [ego,target,route,cfg] = localFixture();
            ego.perception = struct("time",NaN,"range",30,"completeWithinRange",true);
            testCase.verifyError(@() collisionAvoidanceController(ego,target,route,cfg,[]), ...
                "MATLAB:expectedFinite");
        end
        function nominalLookaheadKeepsConstantCurvatureAndTangentialAcceleration(testCase)
            [ego,target,route,cfg] = localFixture();
            target.targetVelocityInertial = [10;0];
            target.targetAccelerationInertial = [1;1];
            target.targetYawRate = 0.1;
            [~,lane,~,parsed] = readPlanningInputs(ego,target,route,cfg);
            encounter = targetPrediction.admit(parsed,0,lane,cfg);
            state = targetPrediction.nominalFlow(encounter,2);
            testCase.verifyEqual(norm(state(3:4)),12,AbsTol=1e-12);
            testCase.verifyEqual(state(8)/norm(state(3:4)),0.01,AbsTol=1e-12);
            testCase.verifyEqual(dot(state(3:4),state(5:6))/norm(state(3:4)),1,AbsTol=1e-12);
        end
    end
end

function [ego,target,route,cfg] = localFixture()
    [ego,target,route,cfg] = encounterTestFixture.crossing();
    cfg.controller.certifiedSteps = 1;
    target = rmfield(target,"encounterContract");
    target.predictionMotion = struct("kind","finite-sensing-motion-v1", ...
        "jerkBound",[2;2],"yawAccelerationBound",1);
end
