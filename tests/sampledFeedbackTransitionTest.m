classdef sampledFeedbackTransitionTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
        end
    end
    methods (Test)
        function heldFeedbackDiffersFromContinuouslyUpdatedFeedback(testCase)
            [transition,measurement]=sampledFeedbackTransition(0,1,-1,1);
            testCase.verifyEqual(transition,0,AbsTol=1e-14);
            testCase.verifyEqual(measurement,-1,AbsTol=1e-14);
            testCase.verifyGreaterThan(abs(transition-exp(-1)),.3);
        end
        function doubleIntegratorMatchesDirectHeldInputIntegration(testCase)
            a=[0,1;0,0];b=[0;1];gain=[-2,-3];h=.2;
            [transition,measurement]=sampledFeedbackTransition(a,b,gain,h);
            initial=[.3;-.2];noise=[.02;-.03];u=gain*(initial+noise);
            expected=initial+[initial(2)*h+.5*u*h^2;u*h];
            testCase.verifyEqual(transition*initial+measurement*noise,expected,AbsTol=1e-14);
        end
        function zeroDurationPreservesStateAndIgnoresMeasurement(testCase)
            [transition,measurement]=sampledFeedbackTransition(eye(2),ones(2,1),[1,2],0);
            testCase.verifyEqual(transition,eye(2));
            testCase.verifyEqual(measurement,zeros(2));
        end
    end
end
