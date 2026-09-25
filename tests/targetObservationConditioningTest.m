classdef targetObservationConditioningTest < matlab.unittest.TestCase
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
        function aTighterContractIsAdoptedAndALargerOneIsRefused(testCase)
            % A continuation may declare tighter maxima; a larger declared
            % maximum is not a continuation of the carried encounter.
            [ego,target,route,cfg] = localFixture();
            [~,lane,~,parsed] = readPlanningInputs(ego,target,route,cfg);
            encounter = targetPrediction.admit(parsed,0,lane,cfg);
            tighter = parsed;
            tighter.position = parsed.position+0.1*parsed.velocity;
            tighter.predictionMotion.curvatureMaximum = 0.02;
            tighter.predictionMotion.speedRateMaximum = 1;
            updated = targetPrediction.advance(encounter,0.1,tighter,lane,cfg);
            testCase.verifyEqual(updated.contract.curvatureMaximum,0.02);
            testCase.verifyEqual(updated.contract.speedRateMaximum,1);
            testCase.verifyLessThanOrEqual(abs(updated.parameters.curvature),0.02);
            testCase.verifyLessThanOrEqual(abs(updated.parameters.speedRate),1);
            larger = tighter;
            larger.predictionMotion.curvatureMaximum = 0.1;
            testCase.verifyError(@() targetPrediction.advance(encounter,0.1,larger,lane,cfg), ...
                "collisionAvoidanceController:changedEncounterContract");
            declared = tighter;
            declared.predictionMotion.curvatureMaximum = 0.05;
            declared.predictionMotion.scalarAccelerationMaximum = 2;
            adopted = targetPrediction.advance(encounter,0.1,declared,lane,cfg);
            testCase.verifyEqual(adopted.contract.scalarAccelerationMaximum,2);
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
    target.predictionMotion = struct("kind","nrmm-motion-v1","curvatureMaximum",0.05);
end
