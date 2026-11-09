classdef collisionThreatScenarioTest < matlab.unittest.TestCase
    % Baseline danger is required independently of avoidance performance.
    properties (TestParameter)
        scenario = {"headOn","acceleratingHeadOn","brakingLead","stoppedLead","hardBrakingLead","crossing", ...
            "turningCrossing","curvedHeadOn","curvedCrossing"};
        referenceSpeed = {8,15};
        crossingScenario = {"crossing","turningCrossing","curvedCrossing"};
        crossingSpeed = {1.5,3,6,15,25};
        crossingPressure = {false,true};
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
        function crossingsLeaveReactionTimeButStillIntersectCruise(testCase,crossingScenario,crossingSpeed)
            [~,q,road,cfg,impactTime]=collisionThreatScenario(crossingScenario, ...
                struct('referenceSpeed',crossingSpeed));
            baseline=givenPathCollisionBaseline(road,q,cfg,impactTime+2);
            testCase.verifyGreaterThan(baseline.initialClearanceMeters,5);
            testCase.verifyTrue(baseline.collisionDetected);
            testCase.verifyGreaterThan(baseline.firstCollisionSeconds,2);
            atImpact=predictiveSafetyGeometry.predictTarget(q,impactTime);
            position=predictiveSafetyGeometry.referencePose( ...
                baseline.initialStationMeters+crossingSpeed*impactTime,0,baseline.referenceCurve);
            testCase.verifyEqual(atImpact(1:2),position,AbsTol=1e-10);
            testCase.verifyGreaterThan(atImpact(4),0);
            testCase.verifyLessThan(q(5),0);
        end
        function crossingsAllowDelayedGentleBraking(testCase,crossingScenario,crossingSpeed,crossingPressure)
            configuration=struct('referenceSpeed',crossingSpeed);
            offsets=[-.005,0,.005];
            if crossingPressure
                configuration.tire=struct('frictionCoefficient',[.65;.65]);
                offsets=[-.01,-.005,0,.005,.01];
            end
            [x,q,~,cfg]=collisionThreatScenario(crossingScenario,configuration);
            if crossingPressure
                x(1:2)=x(1:2)+[0;.3];x(3)=x(3)+.02;
                q(1:2)=q(1:2)+[-1;.2];
            end
            minimum=Inf;
            for offset=offsets
                member=x;member(1:2)=member(1:2)+offset;member(4)=member(4)+offset;
                target=q;target(1:2)=target(1:2)+offset;
                physical=cfg;physical.tire.frictionCoefficient=cfg.tire.frictionCoefficient*(1+offset);
                witness=localDelayedBraking(member,target,physical);
                testCase.assertTrue(witness.stopped,'The independent gentle-braking witness did not reach rest.');
                testCase.verifyGreaterThanOrEqual(witness.minimumForwardSpeed,-1e-7);
                minimum=min(minimum,witness.minimumClearanceMeters);
            end
            % This is a scenario qualification threshold, not the 0.1 m
            % collision constraint used by the actual controller.
            testCase.verifyGreaterThan(minimum,2);
            fprintf('Gentle-braking witness: %s, %.1f m/s, pressure=%d, minimum %.6f m\n', ...
                crossingScenario,crossingSpeed,crossingPressure,minimum);
        end
        function curvedInitialConditionsDoNotRequireAControllerCertificate(testCase)
            for speed=[8,15,25]
                for name=["curvedHeadOn","curvedCrossing"]
                    cfg=struct('referenceSpeed',speed, ...
                        'clf',struct('convergenceTimeConstantSeconds',1.2345));
                    [x,~,road]=collisionThreatScenario(name,cfg);
                    testCase.verifyEqual(norm(x(4:5)),speed,AbsTol=1e-10);
                    testCase.verifyEqual(x(6),speed*road.referenceCurve.curvature,AbsTol=1e-10);
                    testCase.verifyEqual(x(3)+atan2(x(5),x(4)),0,AbsTol=1e-10);
                end
            end
        end
        function anApproachingTargetCanEnterTheThirtyMetreSensorLater(testCase)
            [x,q,road,cfg]=collisionThreatScenario("stoppedLead",struct('referenceSpeed',25, ...
                'model',struct('speedMaximum',30),'collision',struct('encounterRangeMeters',30)));
            testCase.assertGreaterThan(norm(q(1:2)-x(1:2)),30);
            bypass=bypassFeasibilityBaseline(x,q,road,cfg);
            testCase.assertTrue(bypass.qualified,bypass.reason);
            testCase.verifyGreaterThan(bypass.encounterEntrySeconds,0);
            testCase.verifyEqual(bypass.states(2,bypass.times<=bypass.encounterEntrySeconds), ...
                zeros(1,nnz(bypass.times<=bypass.encounterEntrySeconds)),AbsTol=1e-10);
            testCase.verifyGreaterThan(bypass.minimumClearanceMeters,0);
            braking=maximumBrakingCollisionBaseline(x,q,cfg);
            testCase.verifyTrue(braking.collisionDetected);
            testCase.verifyTrue(braking.allLongitudinalPoliciesCertified);
        end
        function campaignRunsStationaryAndBrakingTargetsWithTheObserver(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            % This checks noisy joint execution, independently of machine
            % load and the separate real-time deadline experiment.
            campaign = runCollisionThreatValidation(Pipeline="nrmmComparator",Frames=40,OutputDirectory=string(fixture.Folder), ...
                ControllerConfiguration=struct('solver',struct('method',"rti",'timeLimitSeconds',5)));
            testCase.assertNumElements(campaign.results,2);
            testCase.verifyEqual([campaign.results.scenario],["stoppedLead","hardBrakingLead"]);
            testCase.verifyTrue(all([campaign.results.bypassQualified]));
            testCase.verifyTrue(all([campaign.results.allLongitudinalPoliciesCertified]));
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
            egoPosition = predictiveSafetyGeometry.referencePose(baseline.initialStationMeters+referenceSpeed*impactTime,0,baseline.referenceCurve);
            testCase.verifyEqual(future(1:2),egoPosition,AbsTol=1e-11);
        end
        function everyThreatHasAnIndependentInRoadBypass(testCase,scenario,referenceSpeed)
            [x,target,road,cfg]=collisionThreatScenario(scenario,struct('referenceSpeed',referenceSpeed));
            witness=bypassFeasibilityBaseline(x,target,road,cfg);
            testCase.assertTrue(witness.qualified,witness.reason);
            testCase.verifyFalse(witness.robustFeasibilityClaimed);
            testCase.verifyGreaterThan(witness.minimumClearanceMeters,0);
            testCase.verifyGreaterThanOrEqual(witness.minimumEgoRoadMarginMeters,0);
            testCase.verifyGreaterThanOrEqual(witness.minimumTargetRoadMarginMeters,0);
            testCase.verifyLessThan(witness.maximumLateralDeviationMeters,9);
            testCase.verifyGreaterThan(witness.recoverySeconds,witness.encounterExitSeconds);
            testCase.verifyEqual(witness.states(:,1),x,AbsTol=0);
            testCase.verifyEqual(size(witness.states,2),size(witness.inputs,2)+1);
        end
        function targetFootprintsOutsideTheRoadAreNotQualified(testCase)
            [x,q,road,cfg]=collisionThreatScenario("stoppedLead");q(2)=8.5;
            witness=bypassFeasibilityBaseline(x,q,road,cfg);
            testCase.verifyFalse(witness.qualified);
            testCase.verifyEqual(witness.reason,"targetOutsideRoad");
            testCase.verifyEmpty(witness.inputs);
        end
        function startingInsideDoesNotExcuseLeavingDuringTheEncounter(testCase)
            [x,q,road,cfg]=collisionThreatScenario("stoppedLead");q(1:6)=[20;0;pi/2;10;0;0];
            witness=bypassFeasibilityBaseline(x,q,road,cfg);
            testCase.verifyFalse(witness.qualified);
        end
        function invalidScenarioGeometryIsNotAControllerFailure(testCase)
            cfg=struct('collision',struct('encounterRangeMeters',1));
            testCase.verifyError(@()runNonlinearPredictiveSafetyValidation( ...
                Scenarios="stoppedLead",Frames=1,ControllerConfiguration=cfg), ...
                'runNonlinearPredictiveSafetyValidation:unqualifiedScenario');
        end
        function aVehicleEdgeCannotCutThroughTheInsideOfACurvedRoad(testCase)
            curve=struct('origin',[0;0],'heading',0,'curvature',.005,'length',200);
            shape=[2.4;.95;0;0];pose=[0;8.055;0];
            corners=pose(1:2)+shape(1:2).*[1,1,-1,-1;1,-1,1,-1];
            projected=predictiveSafetyGeometry.projectReferenceCurve(corners,curve);
            testCase.verifyLessThan(max(abs(projected.lateralPosition)),9);
            margin=scenarioRoadFootprintMargin(pose,shape,curve,9);
            testCase.verifyEqual(margin,-.005,AbsTol=1e-10);
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
        function aBrakingSufficientScenarioIsNotAControllerFailure(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            campaign = runCollisionThreatValidation(Pipeline="nrmmComparator",Scenarios="brakingLead",Speeds=8, ...
                OutputDirectory=string(fixture.Folder));
            testCase.verifyEqual(campaign.results.outcome,"unqualifiedScenario");
            testCase.verifyEqual(campaign.results.executedFrames,0);
            testCase.verifyFalse(campaign.results.allLongitudinalPoliciesCertified);
            testCase.verifyEqual(campaign.results.source,"");
        end
        function oneFailedBrakingPolicyDoesNotProveSteeringIsNecessary(testCase)
            [x,q,~,cfg] = collisionThreatScenario("curvedHeadOn",struct('referenceSpeed',8));
            baseline = maximumBrakingCollisionBaseline(x,q,cfg);
            testCase.assertTrue(baseline.collisionDetected);
            testCase.verifyFalse(baseline.allLongitudinalPoliciesCertified);
            testCase.verifyError(@()runNonlinearPredictiveSafetyValidation( ...
                Scenarios="curvedHeadOn",RequireBrakingInsufficient=true, ...
                ControllerConfiguration=struct('referenceSpeed',8)), ...
                'runNonlinearPredictiveSafetyValidation:brakingInsufficiencyUnproven');
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

function witness=localDelayedBraking(x,target,cfg)
    % Independent numerical witness: one second without braking, then 35%
    % braking and zero steering. Check the complete target motion through its
    % absorbing stop, including the time after the ego has already stopped.
    delaySeconds=1;brakingRatio=.35;h=cfg.controller.sampleTime;
    maximumSeconds=delaySeconds+2*norm(x(4:5))/(brakingRatio ...
        *min(cfg.tire.frictionCoefficient)*cfg.vehicle.gravity)+5;
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    minimum=Inf;minimumSpeed=x(4);time=0;stopped=false;
    for hold=1:ceil(maximumSeconds/h)
        input=[0;-brakingRatio*double(time>=delaySeconds-1e-10)];
        solution=ode45(@(~,z)nonlinearBicycleModel.derivative(z,input,cfg),[0,h],x, ...
            odeset('RelTol',1e-10,'AbsTol',1e-12));
        times=linspace(0,h,21);states=deval(solution,times);
        targets=predictiveSafetyGeometry.predictTarget(target,time+times);
        minimumSpeed=min(minimumSpeed,min(states(4,:)));
        for node=1:numel(times)
            gap=predictiveSafetyGeometry.rectangle(states(1:3,node),shape,targets(1:3,node),target(8:11));
            minimum=min(minimum,gap);
        end
        x=nonlinearBicycleModel.sampleEndState(states(:,end),input,cfg);time=time+h;
        if time>delaySeconds && all(x(4:6)==0),stopped=true;break;end
    end
    if stopped
        targetStopSeconds=-target(4)/target(5);
        remainingTimes=unique([time:.01:max(time,targetStopSeconds),max(time,targetStopSeconds)]);
        targets=predictiveSafetyGeometry.predictTarget(target,remainingTimes);
        for node=1:numel(remainingTimes)
            gap=predictiveSafetyGeometry.rectangle(x(1:3),shape,targets(1:3,node),target(8:11));
            minimum=min(minimum,gap);
        end
    end
    witness=struct('stopped',stopped,'minimumClearanceMeters',minimum, ...
        'minimumForwardSpeed',minimumSpeed);
end
