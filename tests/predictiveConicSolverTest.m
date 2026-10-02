classdef predictiveConicSolverTest < matlab.unittest.TestCase
    % Mathematical programs exercise the compiled sparse numerical adapter.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'solver','controller')));
        end
    end
    methods (Test)
        function quadraticCostRespectsLinearBounds(testCase)
            [x,flag]=predictiveConicSolverMex(sparse(1),-2,sparse([1;-1]),[1;1],2,1,localOptions());
            testCase.verifyGreaterThan(flag,0);
            testCase.verifyEqual(x,1,AbsTol=1e-7);
        end
        function secondOrderConeAndEqualityHaveTheExpectedOptimum(testCase)
            % Minimize t with [t;x;2] in the cone and x=1.
            a=sparse([0,1;-1,0;0,-1;0,0]);b=[1;0;0;2];
            [x,flag]=predictiveConicSolverMex(sparse(2,2),[1;0],a,b,[1,3],[0,2],localOptions());
            testCase.verifyGreaterThan(flag,0);
            testCase.verifyEqual(x,[sqrt(5);1],AbsTol=1e-7);
        end
        function nativeQuadraticMatchesItsEpigraph(testCase)
            [native,flag]=predictiveConicSolverMex(sparse(2),-1,sparse([-1;1]),[1;1],2,1,localOptions());
            % sigma>=x^2, min sigma-x, |x|<=1.
            a=sparse([-1,0;1,0;0,-1;-2,0;0,-1]);b=[1;1;1;0;-1];
            [epigraph,otherFlag]=predictiveConicSolverMex(sparse(2,2),[-1;1],a,b,[2,3],[1,2],localOptions());
            testCase.verifyGreaterThan([flag,otherFlag],0);
            testCase.verifyEqual(native,epigraph(1),AbsTol=2e-5);
            testCase.verifyEqual(native^2-native,epigraph(2)-epigraph(1),AbsTol=1e-8);
        end
        function infeasibilityCertificateIsNotReturnedAsAnInput(testCase)
            [x,flag]=predictiveConicSolverMex(sparse(1,1),0,sparse([1;-1]),[0;-1],2,1,localOptions());
            testCase.verifyEmpty(x);
            testCase.verifyEqual(flag,-2);
        end
        function iterationLimitDoesNotReturnAnUnfinishedPoint(testCase)
            options=localOptions();options(1)=1;
            [x,flag]=predictiveConicSolverMex(sparse(1),-2,sparse([1;-1]),[1;1],2,1,options);
            testCase.verifyEmpty(x);
            testCase.verifyEqual(flag,0);
        end
        function malformedConeDimensionsRaiseAMatlabError(testCase)
            testCase.verifyError(@()localMalformedCall(),'predictiveConicSolver:invalidData');
        end
    end
end
function options=localOptions()
    options=[100;5;1e-9;1e-9;1e-9];
end
function localMalformedCall()
    [~,~]=predictiveConicSolverMex(sparse(1),0,sparse(1),0,3,2,localOptions());
end
