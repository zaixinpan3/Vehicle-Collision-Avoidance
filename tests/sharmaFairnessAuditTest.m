classdef sharmaFairnessAuditTest < matlab.unittest.TestCase
% sharmaFairnessAuditTest Behavioral checks for comparison interventions.
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "scripts")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, "estimator"), IncludingSubfolders=true));
        end
    end
    methods (Test)
        function campaignRejectsTrainingTestOverlap(testCase)
            testCase.verifyError(@() runSharmaFairnessAudit(OutputDirectory=string(tempname), ...
                TrainingSeeds=201, TestSeeds=201), "");
        end
        function trainingGainsRemainStableAndDoNotClaimNonlinearCertification(testCase)
            result = runObserverComparisonScenario(Estimators="sharmaMatchedPredictor", ...
                Duration=0.1, TransientDuration=0, Report=false);
            reference = rmfield(result.runs, ["estimate", "metrics"]);
            reference = orderfields(reference, ["name", "runtime", "design", "description"]);
            candidates = prepareSharmaGainCandidates(reference);
            testCase.verifyNumElements(candidates, 54);
            largestPole = arrayfun(@(candidate) max(real( ...
                [candidate.design.ego.physicalPoles; candidate.design.target.physicalPoles])), candidates);
            claimed = arrayfun(@(candidate) candidate.design.audit.theoremOneClaimed, candidates);
            testCase.verifyLessThan(max(largestPole), 0);
            testCase.verifyFalse(any(claimed));
        end
        function boundedKinematicMismatchIsActuallyAppliedToTruth(testCase)
            result = runObserverComparisonScenario(Scenario="circularFollowing", ...
                Estimators=strings(1, 0), SingleTrackMismatch=0.02, Duration=0.1, Report=false);
            residual = result.truth.egoYawRate - result.truth.egoSpeed ...
                .*sin(result.truth.egoSideslip)/result.config.ego.yaw.rearAxleDistance;
            testCase.verifyEqual(residual, 0.02*ones(size(residual)), AbsTol=1.0e-13);
            testCase.verifyTrue(result.truthDomainValid);
        end
        function mismatchOutsideDeclaredBoundIsFlagged(testCase)
            result = runObserverComparisonScenario(Estimators=strings(1, 0), ...
                SingleTrackMismatch=0.021, Duration=0.1, Report=false);
            testCase.verifyFalse(result.truthDomainValid);
        end
        function oraclePreservesExactStraightMotionForBothCoordinateSystems(testCase)
            result = runObserverComparisonScenario(Scenario="straightOncoming", ...
                Estimators=["structured", "sharmaMatchedPredictor"], NoiseModel="none", ...
                InitialOffsetScale=0, Duration=0.1, TransientDuration=0, Report=false);
            prepared = rmfield(result.runs, ["estimate", "metrics"]);
            oracle = runNrmmOracleTargetComparison(result, prepared);
            testCase.verifyEqual(oracle(1).targetState, result.truth.targetTransformedState, AbsTol=1.0e-10);
            testCase.verifyEqual(oracle(2).targetState, result.truth.targetTransformedState, AbsTol=1.0e-10);
        end
        function oracleRejectsTimeVaryingEgoDynamics(testCase)
            result = runObserverComparisonScenario(Scenario="aggressiveEgo", ...
                Estimators="structured", Duration=0.1, Report=false);
            prepared = rmfield(result.runs, ["estimate", "metrics"]);
            testCase.verifyError(@() runNrmmOracleTargetComparison(result, prepared), ...
                "runNrmmOracleTargetComparison:nonconstantEgo");
        end
    end
end
