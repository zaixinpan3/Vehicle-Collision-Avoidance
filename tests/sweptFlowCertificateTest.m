classdef sweptFlowCertificateTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "tests")));
        end
    end
    methods (Test)
        function polynomialTubesEncloseIndependentExactFlowsAndDisturbanceSwitches(testCase)
            [residual, endResidual] = localFlowResiduals();
            testCase.verifyLessThanOrEqual(residual, 1e-12);
            testCase.verifyLessThanOrEqual(endResidual, 1e-12);
        end

        function lowSpeedYawUncertaintyHasAFiniteCartesianTube(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            target.targetVelocityInertial = [0.01;0];
            target.targetVelocityInertialErrorBound = [0.1;0.1];
            target.targetYawRate = 0.2;
            target.targetYawRateErrorBound = 0.1;
            target.predictionMotion.jerkBound = [0.2;0.3];
            target.predictionMotion.yawAccelerationBound = 0.1;
            [~, lane, ~, parsed] = readPlanningInputs(ego, target, route, cfg);
            encounter = targetPrediction.admit(parsed, 0, lane, cfg);
            [center, radius] = targetPrediction.finiteFlow(encounter, [0,0.5,1.6]);
            testCase.verifyTrue(all(isfinite([center;radius]), "all"));
            testCase.verifyEqual(radius(7,end), 1.6*0.1+1.6^2*0.1/2, AbsTol=1e-11);
            testCase.verifyGreaterThan(radius(1:2,end), radius(1:2,2));
        end

        function jerkVerticesReachTheAnalyticPositionBounds(testCase)
            [ego, target, route, cfg] = encounterTestFixture.crossing();
            target.predictionMotion.jerkBound = [0.2;0.3];
            [~, lane, ~, parsed] = readPlanningInputs(ego, target, route, cfg);
            encounter = targetPrediction.admit(parsed, 0, lane, cfg);
            [~, radius] = targetPrediction.finiteFlow(encounter, 1.5);
            testCase.verifyEqual(radius(1:2), [0.2;0.3]*1.5^3/6, AbsTol=1e-11);
            testCase.verifyEqual(radius(3:4), [0.2;0.3]*1.5^2/2, AbsTol=1e-11);
            testCase.verifyEqual(radius(5:6), [0.2;0.3]*1.5, AbsTol=1e-11);
        end
    end
end

function [worst, endWorst] = localFlowResiduals()
    a = [0,1;-2,-3]; b = [0;1]; c = [0.2;0];
    initial = [0.3;-0.2]; radius = [0.02;0.03]; rate = [0.01;0.02];
    duration = 0.1; order = 6;
    tube = stateUncertainty.flowTube(a,b,c,zeros(2,1),initial,radius,rate,duration,order,[2;2],1);
    signs = [1,1,-1,-1;1,-1,1,-1];
    initialStates = initial+radius.*signs;
    inputs = [-0.7,0.7];
    worst = -inf; endWorst = -inf;
    for input = inputs
        for time = linspace(0,duration,101)
            basis = arrayfun(@(k) nchoosek(order+1,k)*(time/duration)^k*(1-time/duration)^(order+1-k),0:order+1).';
            nominal = (reshape(tube.map,2,[])*input+tube.offset)*basis;
            bound = tube.radius*basis;
            firstTime = min(time,duration/2); secondTime = max(0,time-duration/2);
            first = expm(firstTime*[a,eye(2);zeros(2,4)]);
            second = expm(secondTime*[a,eye(2);zeros(2,4)]);
            states = first(1:2,1:2)*initialStates+first(1:2,3:4)*(b*input+c+rate.*signs);
            states = second(1:2,1:2)*states+second(1:2,3:4)*(b*input+c-rate.*signs);
            worst = max(worst,max(abs(states-nominal)-bound,[],"all"));
        end
        endWorst = max(endWorst,max(abs(states-(tube.endMap*input+tube.endOffset))-tube.endRadius,[],"all"));
    end
end
