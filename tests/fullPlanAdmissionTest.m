classdef fullPlanAdmissionTest < matlab.unittest.TestCase
    %fullPlanAdmissionTest Optimize fluid seeds without issuing unsafe initial trajectories.
    properties (TestParameter)
        curvature=struct('left',.01,'right',-.01,'gentler',.008,'tighter',.012);
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (Test)
        function curvedCrossingRetainsEveryOriginalHardConstraint(testCase,curvature)
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(curvature);
            [command,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
            testCase.verifyLessThanOrEqual(max(problem.metadata.jointCertificateResidual),0);
            testCase.verifyLessThanOrEqual(max(problem.program.physicalMatrix*problem.decision ...
                -problem.program.physicalBound),0);
            testCase.verifyLessThanOrEqual(problem.metadata.solverCallCount,1);
            testCase.verifyFalse(problem.metadata.fallbackUsed);
            testCase.verifyEqual(command.holdSeconds,.05,AbsTol=0);
        end

        function fullOptimizationRepairsTheUnsafeFluidSeed(testCase)
            [ego,target,road,cfg]=localRecoveryFixture();
            [~,~,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.admissionSearch.usedFullPlanAdmission);
            testCase.verifyEqual(problem.metadata.admissionSearch.initialization.status,"candidate");
            testCase.verifyGreaterThan(problem.metadata.admissionSearch.initialization.maximumSupportResidual,0);
            testCase.verifyLessThanOrEqual(max(problem.metadata.jointCertificateResidual),0);
            testCase.verifyEqual(problem.metadata.solverCallCount, ...
                problem.metadata.trajectorySolverCallCount+problem.metadata.restorationSolverCallCount);
        end

        function failedAdmissionSolveCannotIssueItsUncertifiedCenter(testCase)
            [ego,target,road,cfg]=localRecoveryFixture();
            cfg.solver.jointFunction=@encounterTestFixture.fail;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function aPositiveSolverFlagCannotBypassAdmissionVerification(testCase)
            [ego,target,road,cfg]=localRecoveryFixture();
            cfg.solver.jointFunction=@encounterTestFixture.unsafe;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function anExpiredBudgetCannotStartUnrestrictedAdmission(testCase)
            [ego,target,road,cfg]=localRecoveryFixture();
            cfg.solver.certificateSearchTimeLimit=1e-9;
            [cfg.solver.jointFunction,count]=localCountedFailure();
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
            testCase.verifyEqual(count(),0);
        end

        function admittedPlanRemainsAvailableAfterASuccessorSolveFailure(testCase)
            [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
            [~,~,~,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            nextEgo=encounterTestFixture.nextEgo(stored,road);
            target.targetPositionInertial=target.targetPositionInertial+.05*target.targetVelocityInertial;
            cfg.solver.jointFunction=@encounterTestFixture.fail;
            [~,~,next]=collisionAvoidanceController(nextEgo,target,road,cfg,stored);
            testCase.verifyTrue(next.metadata.certifiedIncumbentUsed);
            testCase.verifyTrue(next.metadata.planCertified);
            testCase.verifyFalse(next.metadata.admissionSearch.usedFullPlanAdmission);
            testCase.verifyEqual(next.metadata.admissionSearch.directionSeedSource,"inheritedWitness");
            testCase.verifyFalse(isfield(next.metadata.admissionSearch,'initialization'));
            testCase.verifyLessThanOrEqual(max(next.metadata.jointCertificateResidual),0);
        end

        function circularCrossingCompletesTheEncounterWithPositiveSampledGap(testCase)
            report=runExactStateRecursiveFeasibilityScenario(Scenario="crossing", ...
                RoadCurvature=.01,SampleCount=140,DeadlineSeconds=30,SearchTimeLimitSeconds=30);
            testCase.verifyTrue(report.passed);
            testCase.verifyTrue(all(report.hardCertificateVerified));
            testCase.verifyTrue(any(report.confirmedRelease));
            testCase.verifyGreaterThan(report.minimumSampledBodyGap,0);
            testCase.verifyEqual(sum(report.restorationSolverCallCount),0);
        end
    end
end

function [ego,target,road,cfg]=localRecoveryFixture()
    [ego,target,road,cfg]=encounterTestFixture.circularCrossing(.01);
end

function [hook,count]=localCountedFailure()
    calls=0;hook=@reject;count=@()calls;
    function result=reject(~,~)
        calls=calls+1;
        result=struct('decision',[],'exitFlag',-999,'output',struct());
    end
end
