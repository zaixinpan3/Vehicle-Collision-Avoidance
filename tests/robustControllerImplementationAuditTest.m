classdef robustControllerImplementationAuditTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function admittedTargetMotionEscapesARepeatedOneHoldEnclosure(testCase)
            result=auditRobustControllerImplementation();target=result.target;
            testCase.verifyGreaterThan(target.exactPositionErrorMeters, ...
                target.resetEachHoldPositionRadiusMeters);
            testCase.verifyLessThanOrEqual(target.exactPositionErrorMeters, ...
                target.fullHorizonPositionRadiusMeters);
        end
        function aTangentImageMissesAnAdmittedYawPerturbation(testCase)
            result=auditRobustControllerImplementation();image=result.nonlinearImage;
            testCase.verifyGreaterThan(abs(image.exactLongitudinalErrorMeters), ...
                image.tangentLongitudinalRadiusMeters+1e-4);
        end
        function settledMembershipDoesNotMakeTheTwoTerminalModelsEqual(testCase)
            result=auditRobustControllerImplementation();terminal=result.terminal;
            testCase.verifyTrue(all([terminal.terminalMember]));
            testCase.verifyLessThan(max(abs([terminal.sideslipRadians])),.05);
            testCase.verifyLessThan(max([terminal.lateralAccelerationMetersPerSecondSquared]),2);
            testCase.verifyGreaterThan(min([terminal.positionDifferenceMeters]),1e-5);
        end
    end
end
