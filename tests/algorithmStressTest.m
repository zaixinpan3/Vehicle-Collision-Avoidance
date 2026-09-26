classdef algorithmStressTest < matlab.unittest.TestCase
    %algorithmStressTest Regressions discovered by the broad stress campaign.
    properties (TestParameter)
        solvedStatus = struct('solved',1,'almostSolved',2);
        coarsePeriod = struct('quarterSecond',.25,'threeTenths',.3);
    end
    properties
        ObserverDesign
    end
    methods (TestClassSetup)
        function addPathsAndDesignObserver(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'estimator')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
            testCase.ObserverDesign = synthesizeNrmmObserverGains(nrmmTrackingConfig());
        end
    end
    methods (Test)
        function aSolvedFlagCannotAuthorizeAnActuatorViolation(testCase,solvedStatus)
            [ego,~,road,cfg] = encounterTestFixture.crossing();
            cfg.solver.jointFunction = @(phase,program) localCorruptSolve(phase,program,solvedStatus);
            testCase.verifyError(@() collisionAvoidanceController(ego,[],road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function aCorruptedSuccessRetainsTheCertifiedInheritedPlan(testCase,solvedStatus)
            [ego,target,road,cfg] = encounterTestFixture.circularCrossing(.01);
            cfg.model.linearizationPolicy="cruise";
            [~,~,~,stored] = collisionAvoidanceController(ego,target,road,cfg,[]);
            ego = encounterTestFixture.nextEgo(stored,road);
            target.targetPositionInertial = target.targetPositionInertial ...
                +cfg.controller.sampleTime*target.targetVelocityInertial;
            cfg.solver.jointFunction = @(phase,program) localCorruptSolve(phase,program,solvedStatus);

            [command,~,problem] = collisionAvoidanceController(ego,target,road,cfg,stored);

            testCase.verifyTrue(problem.metadata.certifiedIncumbentUsed);
            testCase.verifyLessThanOrEqual(abs(command.actuatorInput(1)),cfg.model.frontWheelSteeringAngleMaximum);
            testCase.verifyLessThanOrEqual(max(avoidanceSafetyGeometry.jointResidual( ...
                problem.program,problem.decision,problem.program.jointCertificate.angles)),0);
        end

        function coarseRequestedIntegrationPreservesTheFineStepEstimate(testCase,coarsePeriod)
            cfg = nrmmTrackingConfig();
            cfg.runtime.samplePeriod = coarsePeriod;
            fine = localEstimatorRun(cfg,testCase.ObserverDesign);
            cfg.runtime.integrationStepMaximum = coarsePeriod;

            actual = localEstimatorRun(cfg,testCase.ObserverDesign);

            testCase.verifyTrue(actual.metrics.allSamplesFinite);
            testCase.verifyLessThan(actual.metrics.relativePositionRmse,.2);
            testCase.verifyEqual(actual.metrics.relativePositionRmse, ...
                fine.metrics.relativePositionRmse,AbsTol=.002);
            testCase.verifyEqual(actual.metrics.targetVelocityRmse, ...
                fine.metrics.targetVelocityRmse,AbsTol=.005);
        end

        function unstableSensorSamplingIsRejectedBeforePublishingEstimates(testCase)
            cfg = nrmmTrackingConfig();
            cfg.runtime.samplePeriod = 1;
            options = struct('egoInitialPosition',[0;0],'egoInitialBodyVelocity',[12;0], ...
                'targetInitialState',[25;4;12.5;0;0;0]);

            testCase.verifyError(@() onlineNrmmTrackingRuntime('initialize',cfg,options,testCase.ObserverDesign), ...
                'onlineNrmmTrackingRuntime:unstableSamplePeriod');
        end

        function slowAvoidanceRetainsThePhysicalCompletionHorizon(testCase)
            result = runExactStateRecursiveFeasibilityScenario(ReferenceSpeed=2,SampleCount=1, ...
                DeadlineSeconds=Inf,SearchTimeLimitSeconds=30);
            testCase.verifyTrue(result.passed);
            testCase.verifyGreaterThan(result.horizonSteps,4*result.configuration.controller.horizonSteps);
        end

        function nearlyMatchedTargetSpeedCannotAllocateAnUnboundedHorizon(testCase)
            [ego,target,road,cfg] = encounterTestFixture.crossing();
            target.targetPositionInertial = [15;0];
            target.targetVelocityInertial = [7.999;0];
            target.targetHeadingInertial = 0;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:encounterHorizonLimit');
        end

        function directionRecoveryFindsASafeRateLimitedPlan(testCase)
            [ego,target,road,cfg] = localRateLimitedEncounter();
            [~,~,problem] = collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyGreaterThan(problem.metadata.restorationSolverCallCount,0);
            testCase.verifyEqual(problem.metadata.admissionSearch.directionSeedSource,"restoredSeparationDirections");
            testCase.verifyLessThanOrEqual(max(problem.program.physicalMatrix*problem.decision ...
                -problem.program.physicalBound),0);
            testCase.verifyLessThanOrEqual(max(avoidanceSafetyGeometry.jointResidual( ...
                problem.program,problem.decision,problem.program.jointCertificate.angles)),0);
        end

        function aRelaxedSearchPointCannotReplaceAFailedHardSolve(testCase)
            [ego,target,road,cfg] = localRateLimitedEncounter();
            cfg.solver.jointFunction = @localRejectHardSolve;
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:optimizationFailed');
        end

        function theDefaultClearanceGrowsWithTheHoldButExplicitValuesRemainAuthoritative(testCase)
            coarse = collisionAvoidanceControllerConfig(struct('controller',struct('sampleTime',.2)));
            explicit = collisionAvoidanceControllerConfig(struct('controller',struct('sampleTime',.2), ...
                'collision',struct('safetyMarginMeters',0)));
            testCase.verifyEqual(coarse.collision.safetyMarginMeters,.4,AbsTol=1e-12);
            testCase.verifyEqual(explicit.collision.safetyMarginMeters,0,AbsTol=0);
            testCase.verifyError(@() collisionAvoidanceControllerConfig(struct('controller', ...
                struct('horizonSteps',33,'maximumHorizonSteps',32))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
    end
end

function [ego,target,road,cfg] = localRateLimitedEncounter()
    [ego,target,road,cfg] = encounterTestFixture.crossing();
    cfg.controller.sampleTime = .05;cfg.controller.horizonSteps = 64;
    cfg.model.frontWheelSteeringRateMaximum = .7;
    cfg.model.brakingRatioRateMaximum = 2;
    cfg.solver.certificateSearchTimeLimit = 30;
    target.targetPositionInertial = [18.4;0];
    target.targetVelocityInertial = [-8;0];
    target.targetHeadingInertial = pi;
end

function solve = localRejectHardSolve(~,program)
    if nnz(program.P)==0 && nnz(program.q)==1 && program.q(end)==1
        solve = program.defaultSolver();
    else
        solve = struct('decision',[],'exitFlag',-2,'output',struct());
    end
end

function solve = localCorruptSolve(~,program,status)
    solve = program.defaultSolver();
    solve.decision(1) = 50;
    solve.exitFlag = status;
end

function result = localEstimatorRun(cfg,design)
    result = runOnlineNrmmComplexManeuverScenario(Plot=false,Report=false, ...
        Config=cfg,Duration=6,Seed=20260925,DesignFunction=@(~) design);
end
