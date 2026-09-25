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
            [residual, endResidual] = localFlowResiduals(1);
            testCase.verifyLessThanOrEqual(residual, 1e-12);
            testCase.verifyLessThanOrEqual(endResidual, 1e-12);
        end

        function oneWholeHoldEnclosesAFlowWithNormTimesDurationAboveOne(testCase)
            [residual,endResidual] = localFlowResiduals(12);
            testCase.verifyLessThanOrEqual(residual,1e-11);
            testCase.verifyLessThanOrEqual(endResidual,1e-11);
        end
    end
end

function [worst, endWorst] = localFlowResiduals(scale)
    a = scale*[0,1;-2,-3]; b = [0;1]; c = [0.2;0];
    initial = [0.3;-0.2]; radius = [0.02;0.03]; rate = [0.01;0.02];
    duration = 0.1; order = 6;
    tube = stateUncertainty.flowTube(a,b,c,zeros(2,1),initial,radius,rate,duration,order,1);
    degree = size(tube.offset,2)-1;
    signs = [1,1,-1,-1;1,-1,1,-1];
    initialStates = initial+radius.*signs;
    inputs = [-0.7,0.7];
    worst = -inf; endWorst = -inf;
    for input = inputs
        for time = linspace(0,duration,101)
            basis = arrayfun(@(k) nchoosek(degree,k)*(time/duration)^k*(1-time/duration)^(degree-k),0:degree).';
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
