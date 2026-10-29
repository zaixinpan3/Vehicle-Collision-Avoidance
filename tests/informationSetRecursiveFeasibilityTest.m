classdef informationSetRecursiveFeasibilityTest < matlab.unittest.TestCase
    % Exact scalar specializations retain estimation error in the shift.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function aNonNestedMeasurementPreservesTheZeroSlackCandidateAfterIntersection(testCase)
            result=verifyInformationSetRecursiveFeasibility();
            settled=result(1);
            testCase.verifyFalse(settled.rawMeasurementNested);
            testCase.verifyTrue(settled.posteriorNested);
            testCase.verifyTrue(settled.shiftSetsIncluded);
            testCase.verifyGreaterThan(settled.initialPositionWidthMeters,0);
            testCase.verifyGreaterThan(settled.positionInnovationMeters,0);
            testCase.verifyEqual(settled.candidateCostMeters,0,AbsTol=1e-12);
            testCase.verifyGreaterThan(settled.candidateTerminalMarginMeters,0);
            testCase.verifyLessThan(settled.rawResetTerminalMarginMeters,0);
        end
        function theShiftedPositiveSlackBudgetLosesTheAppliedHoldDeficit(testCase)
            result=verifyInformationSetRecursiveFeasibility();
            relaxed=result(2);
            testCase.verifyTrue(relaxed.shiftSetsIncluded);
            testCase.verifyEqual(relaxed.oldCostMeters,.6,AbsTol=1e-12);
            testCase.verifyEqual(relaxed.firstDeficitMeters,.2,AbsTol=1e-12);
            testCase.verifyEqual(relaxed.candidateCostMeters,.4,AbsTol=1e-12);
            testCase.verifyEqual(relaxed.candidateCostMeters, ...
                relaxed.oldCostMeters-relaxed.firstDeficitMeters,AbsTol=1e-12);
            testCase.verifyGreaterThan(relaxed.candidateTerminalMarginMeters,0);
        end
        function aUniformNewRemainderCanExcludeAFeasibleShiftedWitness(testCase)
            % Exact scalar specialization: F(1+d)=(1+d)^2 <= 1.05.
            % The old witness d=0 is feasible. A sound uniform remainder
            % over |d|<=.5 nevertheless excludes it. Zero-update-compatible
            % rows retain it, and still imply the nonlinear constraint.
            rho=.5;limit=1.05;witness=0;
            testCase.verifyLessThanOrEqual((1+witness)^2,limit);
            testCase.verifyGreaterThan(1+2*witness+rho^2,limit);
            rows=[2+rho;2-rho];bounds=(limit-1)*ones(2,1);
            testCase.verifyLessThanOrEqual(rows*witness,bounds);
            for d=linspace(-rho,rho,101)
                if all(rows*d<=bounds),testCase.verifyLessThanOrEqual((1+d)^2,limit+1e-12);end
            end
        end
    end
end
