classdef collisionThreatScenarioTest < matlab.unittest.TestCase
    % Baseline danger is required independently of avoidance performance.
    properties (TestParameter)
        scenario = {"headOn","acceleratingHeadOn","brakingLead","stoppedLead","hardBrakingLead","crossing", ...
            "turningCrossing","curvedHeadOn","curvedCrossing"};
        referenceSpeed = {8,15};
        crossingScenario = {"crossing","turningCrossing","curvedCrossing"};
        crossingSpeed = {1.5,3,6,15,20};
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
        function brakingLeadMeetsCruiseEvenIfItStopsBeforeTheConflict(testCase)
            for speed=[1.5,2.5,8]
                [ego,target,~,~,time]=collisionThreatScenario("brakingLead",struct('referenceSpeed',speed));
                conflict=predictiveSafetyGeometry.predictTarget(target,time);
                testCase.verifyEqual(conflict(1:2),ego(1:2)+[speed*time;0],'AbsTol',1e-12);
                testCase.verifyEqual(conflict(4),max(0,speed+target(5)*time),'AbsTol',1e-12);
            end
        end

        function replayContinuesBelowPlanningClearanceUntilContact(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture());
            % Coast into a stationary target to isolate the replay verdict
            % from the optimizer, which would otherwise avoid this trajectory.
            writelines(["function [command,info,prediction,state]=collisionAvoidanceController(varargin)"
                "command=struct('actuatorInput',[0;0]);"
                "info=[];prediction=struct('metadata',struct());"
                "state=struct('memory',[],'hypotheses',[]);"
                "end"],fullfile(folder.Folder,'collisionAvoidanceController.m'));
            rehash;
            [ego,target,~,cfg]=collisionThreatScenario("stoppedLead",struct('referenceSpeed',1.5));
            gap=.2;
            offset=[ego(1)+cfg.vehicle.length/2+target(8)+gap-target(1);0];
            for frames=[2,4]
                campaign=runJointOutputFeedbackScenario(Frames=frames,Speeds=1.5,Scenarios="stoppedLead", ...
                    TargetPositionOffsetMeters=offset,Warmup=false,RequirePhysicalWitness=false,OutputDirectory=string(folder.Folder));
                result=campaign.results;
                testCase.verifyFalse(result.nodeClearanceSatisfied);
                if frames==2
                    testCase.verifyEqual(result.outcome,"completed");
                    testCase.verifyEqual(result.executedFrames,2);
                    testCase.verifyFalse(result.nodeCollisionDetected);
                    testCase.verifyGreaterThan(result.minimumNodeClearanceMeters,0);
                else
                    testCase.verifyEqual(result.failurePhase,"replaySafety");
                    testCase.verifyEqual(result.executedFrames,3);
                    testCase.verifyTrue(result.nodeCollisionDetected);
                end
            end
        end

        function initialOverlapIsReportedBeforeControllerAdmission(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            [ego,target]=collisionThreatScenario("stoppedLead");
            campaign=runJointOutputFeedbackScenario(Frames=1,Speeds=1.5,Scenarios="stoppedLead", ...
                TargetPositionOffsetMeters=ego(1:2)-target(1:2),OutputDirectory=string(folder.Folder));
            result=campaign.results;
            testCase.verifyTrue(result.initialContactOrOverlap);
            testCase.verifyTrue(result.nodeCollisionDetected);
            testCase.verifyEqual(result.outcome,"unqualifiedScenario");
            testCase.verifyEqual(result.failurePhase,"scenarioQualification");
            testCase.verifyFalse(result.bypassQualified);
            testCase.verifyEqual(result.timing.calls,0);
        end

        function sensorSeedsShareAPhysicallyVerifiedScene(testCase)
            root=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            seeds=[20261107,20261108];data=cell(1,2);
            for k=1:2
                campaign=runJointOutputFeedbackScenario(Frames=1,Speeds=15,Scenarios="trafficHeadOn", ...
                    RandomSeed=seeds(k),Warmup=false,OutputDirectory=fullfile(string(root.Folder),string(seeds(k))));
                testCase.assertTrue(campaign.results.bypassQualified);
                data{k}=jsondecode(fileread(campaign.results.source));
            end
            testCase.verifyEqual(data{1}.initialPlantState,data{2}.initialPlantState);
            testCase.verifyEqual(data{1}.physicalWitness.inputs,data{2}.physicalWitness.inputs);
            testCase.verifyGreaterThan(data{1}.physicalWitness.minimumClearanceMeters,0);
            testCase.verifyNotEqual(data{1}.initialSensorSample.position,data{2}.initialSensorSample.position);
        end

        function aControllerRefusalIsCheckedFromTheReachedState(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture());
            writelines(["function [command,info,prediction,state]=collisionAvoidanceController(block,varargin)"
                "if block.time>0,error('collisionAvoidanceController:compactNoFeasiblePlan','Test refusal');end"
                "command=struct('actuatorInput',[.01;0]);info=[];prediction=struct('metadata',struct());"
                "state=struct('memory',[],'hypotheses',[]);"
                "end"],fullfile(folder.Folder,'collisionAvoidanceController.m'));
            rehash;
            campaign=runJointOutputFeedbackScenario(Frames=2,Speeds=15,Scenarios="stoppedLead", ...
                Warmup=false,OutputDirectory=string(folder.Folder));
            result=campaign.results;
            testCase.assertEqual(result.failurePhase,"controller");
            testCase.verifyEqual(result.executedFrames,1);
            testCase.verifyTrue(result.bypassQualified && result.refusalBypassQualified);
            run=jsondecode(fileread(result.source));w=run.physicalWitnessAtRefusal;
            testCase.verifyEqual(w.initialState,run.trace.stateAfterHold);
            testCase.verifyEqual(w.previousInput,run.trace.input);
            testCase.verifyGreaterThan(w.minimumClearanceMeters,0);
        end

        function trafficTargetsReachTheUnmodifiedCruiseAtRoadSpeeds(testCase)
            speeds=[50,60,40]/3.6;
            names=["trafficBrakingLead","trafficHeadOn","trafficCrossing"];
            for i=1:numel(names)
                [~,q,road,cfg,impact]=collisionThreatScenario(names(i),struct('referenceSpeed',15));
                testCase.verifyEqual(q(4),speeds(i),AbsTol=1e-12);
                baseline=givenPathCollisionBaseline(road,q,cfg,impact+1);
                testCase.verifyTrue(baseline.collisionDetected);
                target=predictiveSafetyGeometry.predictTarget(q,impact);
                testCase.verifyEqual(target(1:2),[impact*15;0],AbsTol=1e-10);
                if names(i)=="trafficCrossing"
                    testCase.verifyTrue(isfield(road,'drivablePolygons'));
                    testCase.verifyEqual(target(4),speeds(i),AbsTol=1e-12);
                end
            end
        end
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
            % Qualify the scenario with an independent 2 m braking reserve.
            testCase.verifyGreaterThan(minimum,2);
            fprintf('Gentle-braking witness: %s, %.1f m/s, pressure=%d, minimum %.6f m\n', ...
                crossingScenario,crossingSpeed,crossingPressure,minimum);
        end
        function curvedInitialConditionsDoNotRequireAControllerCertificate(testCase)
            for speed=[8,15,20]
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
        function qualifiedCommandsRespectTheStoredActuatorAndRequestedClearance(testCase)
            cfg=struct('referenceSpeed',15,'outputFeedback',struct('steeringRateMaximum',.8));
            [x,q,road,cfg]=collisionThreatScenario("stoppedLead",cfg);
            previous=[.03;0];
            w=bypassFeasibilityBaseline(x,q,road,cfg,PreviousInput=previous, ...
                MinimumClearanceMeters=cfg.outputFeedback.clearanceMeters);
            testCase.assertTrue(w.qualified,w.reason);
            testCase.verifyGreaterThanOrEqual(w.minimumClearanceMeters,cfg.outputFeedback.clearanceMeters);
            delta=diff([previous,w.inputs],1,2);
            testCase.verifyLessThanOrEqual(max(abs(delta(1,:))),cfg.outputFeedback.steeringRateMaximum*cfg.controller.sampleTime+1e-12);
            testCase.verifyLessThanOrEqual(max(abs(w.inputs(1,:))),cfg.outputFeedback.steeringMaximum);
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
        function stoppingDoesNotDiscardAnOncomingTarget(testCase)
            [x,q,~,cfg]=collisionThreatScenario("headOn",struct('referenceSpeed',8));
            baseline=maximumBrakingCollisionBaseline(x,q,cfg);
            testCase.verifyTrue(baseline.collisionDetected);
            testCase.verifyEqual(baseline.collisionPhase,"afterStop");
            testCase.verifyTrue(baseline.allLongitudinalPoliciesCertified);
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
