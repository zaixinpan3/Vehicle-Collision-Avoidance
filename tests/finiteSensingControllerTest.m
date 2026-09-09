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
        function liveVelocityBoundsCanBeAdmittedWithoutAnInputTarget(testCase)
            [ego, cfg, lane] = localUncertainInputs();
            [~, ~, problem, stored] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyFalse(isfield(stored, "terminalUncertainty"));
            testCase.verifyGreaterThan(stored.stateErrorBound(4, end), 0);
            testCase.verifyGreaterThanOrEqual(stored.predictedState(4, end), stored.stateErrorBound(4, end));
        end

        function finiteAdmissionDoesNotAppendADissipativeTerminalConstraint(testCase)
            [ego, cfg, lane] = localUncertainInputs();
            [~, ~, problem] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyEqual(problem.layout.tailSteps, 0);
            testCase.verifyEqual(problem.prediction.nodeCount, cfg.controller.horizonSteps+1);
            testCase.verifyEmpty(problem.qp.equalityBound);
        end

        function straightPolylineVerticesDoNotInvalidateAnInteriorChart(testCase)
            [ego, cfg] = localUncertainInputs();
            lane = [(0:.1:150).', zeros(1501, 1)];
            [~, ~, problem] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function persistentForcingCanBeEnclosedOverAFiniteCertificate(testCase)
            [ego, cfg, lane] = localUncertainInputs();
            cfg.model.plantModelResidualRateBound(4) = .001;
            [~, ~, problem, stored] = collisionAvoidanceController(ego, [], lane, cfg, []);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyGreaterThan(stored.stateErrorBound(4, end), 0);
        end
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
            next.perception = struct("time",next.stateTime,"range",16,"completeWithinRange",true);
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
            testCase.verifyEqual(updated.nominalCenter,expected,AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(abs(updated.center(3:4)-parsed.velocity) ...
                +updated.radius(3:4),parsed.velocityErrorBound+1e-12);
            parsed.velocity(2) = parsed.velocity(2)+0.3;
            parsed.velocityErrorBound = 0.01*ones(2,1);
            replaced = targetPrediction.advance(encounter,0.1,parsed,lane,cfg);
            testCase.verifyEqual(replaced.nominalCenter([1:3,5:8]),expected([1:3,5:8]),AbsTol=1e-12);
            testCase.verifyEqual(replaced.nominalCenter(4), ...
                replaced.center(4)-replaced.radius(4),AbsTol=1e-12);
        end
        function targetIntersectionPreservesBothEnclosures(testCase)
            [ego,target,route,cfg] = localFixture();
            [~,lane,~,parsed] = readPlanningInputs(ego,target,route,cfg);
            parsed.positionErrorBound = ones(2,1);
            encounter = targetPrediction.admit(parsed,0,lane,cfg);
            [predicted,radius] = targetPrediction.finiteFlow(encounter,0.1);
            parsed.position = predicted(1:2)+[0.5;0];
            parsed.positionErrorBound = [0.8;0.8];
            updated = targetPrediction.advance(encounter,0.1,parsed,lane,cfg);
            lower = max(predicted(1:2)-radius(1:2),parsed.position-parsed.positionErrorBound);
            upper = min(predicted(1:2)+radius(1:2),parsed.position+parsed.positionErrorBound);
            testCase.verifyEqual(updated.center(1:2)-updated.radius(1:2),lower,AbsTol=1e-12);
            testCase.verifyEqual(updated.center(1:2)+updated.radius(1:2),upper,AbsTol=1e-12);
        end
        function localMotionDoesNotRequireAPerpetualExitRoute(testCase)
            [ego,target,route,cfg] = localFixture();
            [~,~,problem,certificate] = collisionAvoidanceController(ego,target,route,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyTrue(problem.metadata.recursiveFeasibilityClaimed);
            testCase.verifyEqual(certificate.safetyScope,"completeEncounterForDeclaredInclusion");
            testCase.verifyEqual(certificate.certifiedDuration,cfg.controller.sampleTime*cfg.controller.horizonSteps);
            testCase.verifyEqual(problem.metadata.lookaheadDuration,certificate.certifiedDuration);
        end
        function completeCurrentPerceptionEndsAnAbsentEncounter(testCase)
            [ego,target,route,cfg] = localFixture();
            target.targetPositionInertial = [-15.5;0];
            target.targetVelocityInertial = [-8;0];
            target.targetHeadingInertial = pi;
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            next = encounterTestFixture.nextEgo(stored,problem.model.lane);
            next.perception = struct("time",next.stateTime,"range",16,"completeWithinRange",true);
            [~,~,updated,certificate] = collisionAvoidanceController(next,[],route,cfg,stored);
            testCase.verifyTrue(certificate.encounterComplete);
            testCase.verifyEmpty(updated.metadata.activeTargetKeys);
            testCase.verifyFalse(updated.metadata.fallbackUsed);
            testCase.verifyTrue(updated.metadata.encounterComplete);
        end
        function aMissingObservationAloneDoesNotEndAnEncounter(testCase)
            [ego,target,route,cfg] = localFixture();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            next = encounterTestFixture.nextEgo(stored,problem.model.lane);
            [command,~,~,continued] = collisionAvoidanceController(next,[],route,cfg,stored);
            testCase.verifyNotEmpty(command);
            testCase.verifyFalse(continued.encounterComplete);
        end
        function solverFailureRetainsTheCertifiedFiniteWitness(testCase)
            [ego,target,route,cfg] = localFixture();
            [~,~,problem,stored] = collisionAvoidanceController(ego,target,route,cfg,[]);
            next = encounterTestFixture.nextEgo(stored,problem.model.lane);
            target.targetPositionInertial = target.targetPositionInertial+cfg.controller.sampleTime*target.targetVelocityInertial;
            cfg.solver.jointFunction = @encounterTestFixture.fail;
            [command,~,problem] = collisionAvoidanceController(next,target,route,cfg,stored);
            testCase.verifyEqual(command.actuatorInput,stored.plan(:,2),AbsTol=0);
            testCase.verifyTrue(problem.metadata.fallbackUsed);
        end
        function nonfinitePerceptionTimeCannotAuthorizeDischarge(testCase)
            [ego,target,route,cfg] = localFixture();
            ego.perception = struct("time",NaN,"range",16,"completeWithinRange",true);
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
    target.predictionMotion = struct("kind","finite-sensing-motion-v1", ...
        "jerkBound",[0.02;0.02],"yawAccelerationBound",0.01);
end

function [ego, cfg, lane] = localUncertainInputs()
    cfg = collisionAvoidanceControllerConfig(struct("controller", struct("horizonSteps", 4)));
    lane = [0, 0; 1000, 0];
    ego = struct("position", [10; 0], "yawAngle", 0, ...
        "longitudinalVelocity", 10, "lateralVelocity", 0, "yawRate", 0, ...
        "stateTime", 0, "perception", struct("time",0,"range",30,"completeWithinRange",true), ...
        "controllerStateErrorBound", [.04; .04; .014; .388; .388; .0015]);
end
