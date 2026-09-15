classdef boundedTargetMotionTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)

        function boundedFlowContainsIndependentChangingMotion(testCase)
            [ego,target,road,cfg] = localFixture();
            [~,lane,~,parsed] = readPlanningInputs(ego,target,road,cfg);
            encounter = targetPrediction.admitOnline(parsed,0,lane,cfg);
            time = linspace(0,2,51);
            [center,radius] = targetPrediction.finiteFlow(encounter,time);
            truth = [15+0.08*time.^3/6;-4+32*time;0.08*time.^2/2;32+0*time; ...
                0.08*time;0*time;pi/2+0.04*time.^2/2;0.04*time];
            testCase.verifyLessThanOrEqual(max(abs(truth-center)-radius,[],'all'),1e-12);
        end

    end
end

function [ego,target,road,cfg] = localFixture()
    [ego,target,road,cfg] = encounterTestFixture.crossing();
    target.predictionMotion.jerkBound = [0.1;0];
    target.predictionMotion.yawAccelerationBound = 0.05;
end
