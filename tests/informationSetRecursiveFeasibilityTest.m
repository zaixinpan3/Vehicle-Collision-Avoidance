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
    end
end
