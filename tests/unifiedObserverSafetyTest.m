classdef unifiedObserverSafetyTest < matlab.unittest.TestCase
    % Behaviors of the observer-conditioned physical predictive barrier.
    properties
        verification
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'scripts')));
            testCase.verification = verifyUnifiedObserverSafety();
        end
    end
    methods (Test)
        function boundedNonNestedObservationsPreserveSafePhysicalPlans(testCase)
            result = testCase.verification.safe;
            testCase.verifyGreaterThan(result.rawNonNestedObservationCount, 0);
            testCase.verifyTrue(result.shiftedSetsIncluded);
            testCase.verifyEqual(result.primaryValuesMeters, zeros(1, 12), AbsTol=1e-10);
            testCase.verifyGreaterThanOrEqual(result.minimumTrueGapMeters, 0);
        end
        function dropoutRetainsPredictionWithoutAnInventedFutureObserverReset(testCase)
            result = testCase.verification.safe;
            testCase.verifyEqual(result.dropoutCount, 2);
            testCase.verifyTrue(result.predictionWidthGrows);
            testCase.verifyGreaterThan(result.posteriorPositionWidthsMeters, 0);
        end
        function positiveDeficitsRemainRelaxationAndTheirBudgetDecreases(testCase)
            result = testCase.verification.recovery;
            testCase.verifyEqual(result.primaryValuesMeters(1), .4, AbsTol=1e-10);
            testCase.verifyLessThan(result.minimumTrueGapMeters, 0);
            testCase.verifyLessThanOrEqual(result.primaryValuesMeters(2:end), ...
                result.primaryValuesMeters(1:end-1)-result.firstHoldDeficitsMeters(1:end-1)+1e-10);
            testCase.verifyEqual(result.primaryValuesMeters(2:end), zeros(1, 5), AbsTol=1e-10);
        end
        function oneHoldHorizonCanShiftAndAppendTheCommonTerminalInput(testCase)
            result = testCase.verification.oneHold;
            testCase.verifyEqual(result.horizonSteps, 1);
            testCase.verifyTrue(result.shiftedSetsIncluded);
            testCase.verifyEqual(result.primaryValuesMeters, zeros(1, 4), AbsTol=1e-10);
        end
        function actuatorSlewAndMagnitudeRemainHardConditions(testCase)
            result = testCase.verification.safe;
            testCase.verifyLessThanOrEqual(result.appliedBrakingMetersPerSecondSquared, 4);
            testCase.verifyGreaterThanOrEqual(result.appliedBrakingMetersPerSecondSquared, 0);
            testCase.verifyLessThanOrEqual(result.maximumInputChange, 2+1e-12);
            testCase.verifyFalse(testCase.verification.inputLimitedPlanFound);
        end
        function parkedUncertaintyDoesNotRequireAnObserverContraction(testCase)
            result = testCase.verification.parked;
            testCase.verifyGreaterThan(result.posteriorPositionWidthsMeters, 0);
            testCase.verifyTrue(result.shiftedSetsIncluded);
            testCase.verifyGreaterThan(result.minimumTrueGapMeters, 0);
        end
        function analyticHoldMinimumDetectsCollisionMissedByThreeSamples(testCase)
            result = testCase.verification.betweenSamples;
            testCase.verifyEqual(result.sampledMinimumGapMeters, .004, AbsTol=1e-12);
            testCase.verifyEqual(result.wholeHoldMinimumGapMeters, -.001, AbsTol=1e-12);
        end
        function zeroClearanceStillRejectsInteriorOverlap(testCase)
            testCase.verifyEqual(testCase.verification.overlapGapMeters, -.8, AbsTol=1e-12);
        end
        function contradictoryInformationCannotProduceACertificate(testCase)
            testCase.verifyTrue(testCase.verification.emptyIntersectionRejected);
        end
        function energyAttenuationCannotReplacePointwiseObserverContainment(testCase)
            result = testCase.verification.energyVsPointwise;
            testCase.verifyLessThan(result.dissipationMaximumEigenvalue, 0);
            testCase.verifyEqual(result.initialErrorNorm, result.claimedEuclideanRadius, AbsTol=1e-12);
            testCase.verifyGreaterThan(result.laterErrorNorm, result.claimedEuclideanRadius);
            testCase.verifyLessThan(result.laterErrorNorm, result.validMetricComparisonRadius);
        end
        function scalarEvidenceDoesNotClaimCertificationOfTheVehicleRuntime(testCase)
            testCase.verifyFalse(testCase.verification.fullVehicleNonlinearCertificate);
        end
    end
end
