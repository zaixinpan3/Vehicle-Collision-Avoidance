classdef collisionThreatScenarioTest < matlab.unittest.TestCase
    % Baseline danger is required independently of avoidance performance.
    properties (TestParameter)
        scenario = {"headOn","acceleratingHeadOn","brakingLead","stoppedLead","hardBrakingLead","crossing", ...
            "turningCrossing","curvedHeadOn","curvedCrossing"};
        referenceSpeed = {8,15};
    end
    methods (TestClassSetup)
        function preparePaths(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function campaignRunsStationaryAndBrakingTargetsWithTheObserver(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            campaign = runCollisionThreatValidation(Frames=40,OutputDirectory=string(fixture.Folder));
            testCase.assertNumElements(campaign.results,2);
            testCase.verifyEqual([campaign.results.scenario],["stoppedLead","hardBrakingLead"]);
            for ran=campaign.results
                testCase.verifyNotEqual(ran.outcome,"setupError");
                testCase.assertGreaterThan(ran.executedFrames,0);
                report = jsondecode(fileread(ran.source));
                result = report.results;
                testCase.verifyEqual(string(result.observationModel),"nrmmSyntheticSensors");
                defaults = estimatorControllerIntegrationConfig();
                testCase.verifyEqual(result.randomSeed,defaults.randomSeed);
                testCase.verifyGreaterThan(norm(result.trace(1).estimatedState-result.trace(1).state),0);
                testCase.verifyGreaterThan(result.estimatorConfiguration.sensor.radar.positionNoiseMaximum,0);
                testCase.verifyEqual(result.estimatorConfiguration.observer.target.domain.speedMinimum,0);
            end
        end
        function everyThreatStartsSeparatedAndCollidesWithoutAvoidance(testCase,scenario,referenceSpeed)
            [~,target,road,cfg] = collisionThreatScenario(scenario,struct('referenceSpeed',referenceSpeed));
            baseline = givenPathCollisionBaseline(road,target,cfg,8);
            testCase.verifyGreaterThan(baseline.initialClearanceMeters,0);
            testCase.verifyTrue(baseline.collisionDetected);
            testCase.verifyGreaterThan(baseline.strictOverlapSamples,0);
            testCase.verifyGreaterThan(baseline.maximumOverlapDepthMeters,.1);
            testCase.verifyGreaterThan(baseline.firstCollisionSeconds,0);
            testCase.verifyLessThan(baseline.firstCollisionSeconds,8);
        end
        function targetActuallyReachesTheCruisingEgo(testCase,scenario,referenceSpeed)
            [~,target,road,cfg,impactTime] = collisionThreatScenario(scenario,struct('referenceSpeed',referenceSpeed));
            baseline = givenPathCollisionBaseline(road,target,cfg,8);
            future = predictiveSafetyGeometry.predictTarget(target,impactTime);
            egoPosition = laneGeometry.referencePose(baseline.initialStationMeters+referenceSpeed*impactTime,0,baseline.referenceCurve);
            testCase.verifyEqual(future(1:2),egoPosition,AbsTol=1e-11);
        end
        function fastStoppedLeadRequiresMoreThanStraightMaximumBraking(testCase)
            [x,q,~,cfg]=collisionThreatScenario("stoppedLead", ...
                struct('referenceSpeed',25,'model',struct('speedMaximum',30)));
            baseline=maximumBrakingCollisionBaseline(x,q,cfg);
            testCase.verifyTrue(baseline.collisionDetected);
            testCase.verifyEqual(baseline.collisionPhase,"braking");
            testCase.verifyTrue(baseline.allLongitudinalPoliciesCertified);
            testCase.verifyGreaterThan(baseline.stoppingDistanceMeters,q(1)-cfg.vehicle.length);
            testCase.verifyEqual(baseline.finalState(4),0);
        end
        function aSlowLeadIsExcludedFromTheSteeringChallenge(testCase)
            testCase.verifyError(@()runNonlinearPredictiveSafetyValidation( ...
                Scenarios="brakingLead",RequireBrakingInsufficient=true), ...
                'runNonlinearPredictiveSafetyValidation:brakingSuffices');
        end
        function stoppingDoesNotDiscardAnOncomingTarget(testCase)
            [x,q,~,cfg]=collisionThreatScenario("headOn",struct('referenceSpeed',8));
            baseline=maximumBrakingCollisionBaseline(x,q,cfg);
            testCase.verifyTrue(baseline.collisionDetected);
            testCase.verifyEqual(baseline.collisionPhase,"afterStop");
            testCase.verifyTrue(baseline.allLongitudinalPoliciesCertified);
        end
        function benignSideBrakingIsNotAdmitted(testCase)
            testCase.verifyError(@()runNonlinearPredictiveSafetyValidation( ...
                Scenarios="brakingTarget",Frames=160,RequireCollisionThreat=true), ...
                'runNonlinearPredictiveSafetyValidation:notCollisionThreat');
        end
        function touchingWithoutOverlapDoesNotQualifyAsActualDanger(testCase)
            cfg = collisionAvoidanceControllerConfig(struct('referenceSpeed',8));
            road = struct('centerline',[-100,0;1000,0]);
            target = [4.8;0;0;8;0;0;1.6;2.4;.95;0;0];
            baseline = givenPathCollisionBaseline(road,target,cfg,2);
            testCase.verifyFalse(baseline.collisionDetected);
            testCase.verifyEqual(baseline.strictOverlapSamples,0);
        end
        function noTargetIsNotAnAvoidanceThreat(testCase)
            cfg = collisionAvoidanceControllerConfig();
            baseline = givenPathCollisionBaseline(struct('centerline',[-100,0;1000,0]),[],cfg,2);
            testCase.verifyFalse(baseline.collisionDetected);
            testCase.verifyEqual(baseline.strictOverlapSamples,0);
        end
    end
end
