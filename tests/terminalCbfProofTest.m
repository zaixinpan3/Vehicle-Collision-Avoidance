classdef terminalCbfProofTest < matlab.unittest.TestCase
    properties (TestParameter)
        curvature = struct("straight",0,"gentleLeft",1/400,"left",1/100,"right",-1/100);
    end
    properties
        audit
    end
    methods (TestClassSetup)
        function addPathsAndRunTheAuxiliaryAudit(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
            testCase.audit = runTerminalCbfProofAudit();
        end
    end
    methods (Test)
        function affineMarginsIncreaseAndMatchIndependentVertices(testCase,curvature)
            checks = testCase.audit.affineChecks;
            result = checks([checks.curvature]==curvature);
            testCase.verifyGreaterThan(result.barrierValues(1),0);
            testCase.verifyGreaterThanOrEqual(result.minimumBarrierIncrement,-1e-12);
            testCase.verifyLessThan(result.supportIdentityError,1e-12);
            testCase.verifyLessThan(result.comparisonSpectralAbscissa,0);
            testCase.verifyFalse(result.reverseSpeedInDomain);
        end

        function aHuangShiftDropsTheExecutedSafetySlack(testCase)
            testCase.verifyLessThanOrEqual(testCase.audit.recovery.maximumDecreaseViolation,1e-7);
            testCase.verifyLessThanOrEqual(testCase.audit.recovery.maximumShiftViolation,1e-7);
            testCase.verifyGreaterThan(testCase.audit.recovery.values(1),0);
            testCase.verifyEqual(testCase.audit.recovery.values(end),0,AbsTol=1e-7);
        end

        function theZeroSafetyValueRemainsZeroUnderTheAuxiliaryMpc(testCase)
            testCase.verifyEqual(testCase.audit.safe.values,zeros(5,1),AbsTol=1e-7);
            testCase.verifyLessThanOrEqual(testCase.audit.safe.maximumShiftViolation,1e-7);
        end

        function persistentForcingBreaksTheUndisturbedPoseBudget(testCase)
            result = testCase.audit.persistentForcing;
            testCase.verifyGreaterThan(result.initialBarrier,0);
            testCase.verifyLessThan(result.finalBarrier,0);
            testCase.verifyGreaterThan(result.budgetDerivative,0);
        end

        function nonlinearHeadingTransportBreaksTheAffineCertificate(testCase)
            result = testCase.audit.nonlinearTransport;
            testCase.verifyGreaterThan(result.initialBarrier,0);
            testCase.verifyGreaterThanOrEqual(result.affineBarrier,result.initialBarrier);
            testCase.verifyLessThan(result.nonlinearBarrier,0);
            testCase.verifyGreaterThan(result.nonlinearState(2),.05);
        end

        function theOldSetContainsANonlinearBoundaryWithUnavoidableOutflow(testCase)
            result = testCase.audit.nonlinearTransport;
            testCase.verifyEqual(result.boundaryBarrier,0,AbsTol=1e-12);
            testCase.verifyGreaterThan(result.unavoidableOutwardRate,0);
            testCase.verifyEqual(result.boundaryState(5:6),zeros(2,1),AbsTol=1e-12);
        end

        function egoRestDoesNotDischargeAMovingTarget(testCase)
            result = testCase.audit.movingTarget;
            testCase.verifyGreaterThan(result.egoTerminalBarrier,0);
            testCase.verifyGreaterThan(result.sampledSatMargin(1),0);
            testCase.verifyLessThan(result.sampledSatMargin(end),0);
        end

        function aSuccessfulAuxiliaryAuditDoesNotClaimAnOnlineProof(testCase)
            testCase.verifyTrue(testCase.audit.auditPassed);
            testCase.verifyFalse(testCase.audit.fullControllerCbfEstablished);
        end
    end
end
