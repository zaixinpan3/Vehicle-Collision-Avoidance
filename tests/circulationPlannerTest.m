classdef circulationPlannerTest < matlab.unittest.TestCase
    properties (TestParameter)
        crossingSpeed={8,15};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for directory=["controller","config","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,directory)));
            end
        end
    end
    methods (Test)
        function aSafeSeedFollowsTheChosenRoute(testCase)
            model=localModel("headOn",8);
            model.target(1:6)=[-30;0;0;2;0;0];
            model.road.boundaryObservation=struct('chains',struct([]));
            original=predictiveSafetyGeometry.avoidanceGuide(model,40);
            model.lane=struct('referenceCurve',struct('origin',[0;4], ...
                'heading',0,'curvature',0,'length',100));
            changed=predictiveSafetyGeometry.avoidanceGuide(model,40);
            testCase.verifyEqual(original.states(2,:),zeros(1,41),AbsTol=1e-10);
            testCase.verifyGreaterThan(changed.states(2,end),1);
            testCase.verifyLessThan(changed.states(2,end),4);
            testCase.verifyFalse(changed.fieldActive);
        end
        function freeMotionRecoversCruiseSpeedAndLateralOffset(testCase)
            model=localModel("headOn",8);model.initialState=[0;3;0;12;0;0];
            model.target(1:6)=[-30;0;0;2;0;0];
            guide=predictiveSafetyGeometry.avoidanceGuide(model,16);
            testCase.verifyLessThan(guide.states(2,end),2.7);
            testCase.verifyLessThan(guide.states(4,end),11);
            testCase.verifyLessThan(guide.inputs(1,1),0);
            testCase.verifyFalse(guide.fieldActive);
        end
        function aSeparatedPassDoesNotEvadeAnOffPathTarget(testCase)
            model=localModel("headOn",8);model.target(1:6)=[15;5;0;0;0;0];
            guide=predictiveSafetyGeometry.avoidanceGuide(model,60);
            testCase.verifyGreaterThan(guide.minimumGap,0);
            testCase.verifyEqual(guide.states(2,:),zeros(1,61),AbsTol=1e-10);
            testCase.verifyEqual(guide.states(4,:),8*ones(1,61),AbsTol=1e-10);
            testCase.verifyFalse(guide.fieldActive);
        end
        function relativeMotionChangesTheFieldWithoutATrajectoryDecision(testCase)
            model=localModel("headOn",8);target=model.target;target(1:6)=[15;5;0;0;0;0];
            p=struct('speed',8,'inflation',0,'preferredAcceleration',zeros(2,1));
            straight=predictiveSafetyGeometry.circulationField(model.initialState,target,model.cfg,p);
            toward=model.initialState;toward(3)=atan2(5,15);
            approaching=predictiveSafetyGeometry.circulationField(toward,target,model.cfg,p);
            testCase.verifyEqual(straight,zeros(2,1),AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(approaching),1);
        end
        function fieldInfluenceVanishesContinuouslyAtAGrazingPass(testCase)
            model=localModel("headOn",8);target=model.target;
            p=struct('speed',8,'inflation',0,'preferredAcceleration',zeros(2,1));
            width=model.cfg.vehicle.width/2+target(9);
            target(1:6)=[15;width-1e-4;0;0;0;0];
            inside=predictiveSafetyGeometry.circulationField(model.initialState,target,model.cfg,p);
            target(2)=width+1e-4;
            outside=predictiveSafetyGeometry.circulationField(model.initialState,target,model.cfg,p);
            testCase.verifyEqual(outside,zeros(2,1),AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(inside),0);
            testCase.verifyLessThan(norm(inside-outside),1e-2);
        end
        function observerUncertaintyChangesTheFieldDirectly(testCase)
            model=localModel("headOn",8);model.target(1:6)=[0;2;0;8;0;0];
            model.uncertainty.collisionGenerator=diag([0,.2,0,0,0,0]);
            guide=predictiveSafetyGeometry.avoidanceGuide(model,16);
            testCase.verifyLessThan(guide.minimumGap,0);
            testCase.verifyTrue(guide.fieldActive);
        end
        function anApproachingObstacleCanBrakeAtTheCruiseSpeed(testCase)
            model=localModel("headOn",8);model.target(1:6)=[12;.2;0;0;0;0];
            guide=predictiveSafetyGeometry.avoidanceGuide(model,32);
            testCase.verifyLessThan(guide.inputs(2,1),-.1);
            testCase.verifyLessThan(guide.states(4,end),model.initialState(4)-.5);
            testCase.verifyGreaterThan(abs(guide.inputs(1,1)),1e-3);
        end
        function aRecedingTargetDoesNotPullTheVehicleIntoAnOrbit(testCase)
            model=localModel("headOn",8);target=model.target;target(1:6)=[-30;0;0;2;0;0];
            p=struct('speed',4,'inflation',0,'preferredAcceleration',[.2;0]);
            [a,f]=predictiveSafetyGeometry.circulationField(model.initialState,target,model.cfg,p);
            testCase.verifyEqual(a,p.preferredAcceleration,AbsTol=1e-12);
            testCase.verifyEqual(f.activation,0);
        end
        function parallelWallSlowsOutwardMotionBeforeReachingTheWall(testCase)
            model=localModel("headOn",8);width=model.cfg.planner.guideHalfWidthMeters;
            x=model.initialState;x(2)=width-2;x(5)=3;
            [a,active]=predictiveSafetyGeometry.wallGuidance(x,[1;100],model);
            testCase.verifyTrue(active);testCase.verifyLessThan(a(2),0);
            testCase.verifyEqual(a(1),1,AbsTol=1e-12);
            % Ideal acceleration realization only: a persistent outward
            % request approaches the wall without crossing it.
            [~,z]=ode45(@(t,z)localWallMotion(z,model),[0,6],[width-2;3], ...
                odeset('RelTol',1e-9,'AbsTol',1e-11));
            testCase.verifyLessThan(max(z(:,1)),width);
            testCase.verifyGreaterThan(z(end,1),width-.1);
        end
        function aWallDoesNotResistAnAlreadyInwardAcceleration(testCase)
            model=localModel("headOn",8);x=model.initialState;x(2)=8;x(5)=-1;
            [a,active]=predictiveSafetyGeometry.wallGuidance(x,[1;-1],model);
            testCase.verifyFalse(active);testCase.verifyEqual(a,[1;-1],AbsTol=0);
        end
        function rotatingFrameTransportRetainsTheRequestedFrozenGeometryAcceleration(testCase)
            model=localModel("turningCrossing",8);x=model.initialState;
            target=model.target;target(10:11)=[.4;-.2];x(2)=3;x(5)=.3;
            parameters=struct('speed',4,'inflation',.15);
            [a,f]=predictiveSafetyGeometry.circulationField(x,target,model.cfg,parameters);
            relative=f.rotation.'*a-f.transport;
            radial=f.normal.'*relative+f.curvature*f.tangentSpeed^2;
            tangential=f.tangent.'*relative-f.curvature*f.normalSpeed*f.tangentSpeed;
            rate=model.cfg.planner.radialRate;
            expected=-2*rate*f.normalSpeed-rate^2*(f.clearance-f.desiredClearance);
            testCase.verifyEqual(radial,expected,AbsTol=1e-10);
            testCase.verifyEqual(tangential,(f.rotation*f.tangent).'*f.tangentialAcceleration,AbsTol=1e-10);
        end
        function exactSymmetryDoesNotInventAPassingSide(testCase)
            model=localModel("headOn",8);model.target(1:6)=[12;0;0;0;0;0];
            p=struct('speed',4,'inflation',0,'preferredAcceleration',zeros(2,1));
            [a,f]=predictiveSafetyGeometry.circulationField(model.initialState,model.target,model.cfg,p);
            testCase.verifyEqual(a(2),0,AbsTol=1e-10);
            testCase.verifyEqual(f.tangentialAcceleration,zeros(2,1),AbsTol=1e-10);
            guide=predictiveSafetyGeometry.avoidanceGuide(model,16);
            testCase.verifyEqual(guide.states(:,1),model.initialState,AbsTol=0);
            testCase.verifyEqual(guide.states(2,:),zeros(1,17),AbsTol=1e-10);
            testCase.verifyFalse(guide.robustCertificate);
        end
        function returnedStatesComeFromTheActualHeldInputModel(testCase)
            model=localModel("crossing",8);model.cfg.model.brakingRatioRateMaximum=1;
            guide=predictiveSafetyGeometry.avoidanceGuide(model,50);
            testCase.verifyEqual(guide.candidateCount,1);
            testCase.verifySize(guide.inputs,[2,50]);
            testCase.verifySize(guide.states,[6,51]);
            for k=1:size(guide.inputs,2)
                next=nonlinearBicycleModel.sample(guide.states(:,k),guide.inputs(:,k),model.cfg);
                testCase.verifyEqual(guide.states(:,k+1),next,AbsTol=1e-12);
            end
            testCase.verifyLessThanOrEqual(max(abs(diff([model.previousInput(2),guide.inputs(2,:)]))),model.cfg.controller.sampleTime+1e-12);
            testCase.verifyTrue(all(isfinite([guide.states(:);guide.inputs(:)])));
        end
        function aNonterminalEndpointIsReturnedWithoutExtendingTheSeed(testCase)
            model=localModel("headOn",8);
            model.target(1:2)=model.initialState(1:2);model.targetEpoch=model.target;
            guide=predictiveSafetyGeometry.avoidanceGuide(model,1);
            testCase.verifySize(guide.inputs,[2,1]);
            testCase.verifySize(guide.states,[6,2]);
            testCase.verifyEqual(guide.time,[0,model.cfg.controller.sampleTime]);
            context=terminalSafeSet.context(model);
            testCase.verifyFalse(terminalSafeSet.member(context,guide.states(:,end),1));
            testCase.verifyTrue(all(isfinite(guide.states),'all'));
            testCase.verifyFalse(guide.robustCertificate);
        end
        function aStationarySeedStillCoversTheRequestedPredictionTimes(testCase)
            model=localModel("headOn",8);model.initialState(4:6)=0;
            model.target(4:6)=0;model.targetEpoch=model.target;
            guide=predictiveSafetyGeometry.avoidanceGuide(model,8);
            testCase.verifySize(guide.inputs,[2,8]);
            testCase.verifySize(guide.target,[11,9]);
            testCase.verifyEqual(guide.states,repmat(model.initialState,1,9),AbsTol=1e-12);
        end
        function aSmallPositiveRectangleGapDoesNotTriggerDiskAvoidance(testCase)
            model=localModel("headOn",8);model.target(1:6)=[0;2;0;8;0;0];
            guide=predictiveSafetyGeometry.avoidanceGuide(model,16);
            testCase.verifyEqual(guide.nominalMinimumGap,.1,AbsTol=1e-6);
            testCase.verifyFalse(guide.fieldActive);
            testCase.verifyTrue(isnan(guide.conflictStartSeconds));
        end
        function anUnknownTargetYawDoesNotRequireTwoRotationDiameters(testCase)
            model=localModel("headOn",8);model.target(1:6)=[0;4;0;0;0;0];
            model.targetEpoch=model.target;
            model.uncertainty.target=struct('positionRadius',0,'courseRadius',pi, ...
                'speedRadius',0,'accelerationRadius',0,'curvatureRadius',0,'sideslipRadius',0);
            guide=predictiveSafetyGeometry.avoidanceGuide(model,16);
            testCase.verifyGreaterThan(guide.minimumGap,0);
            testCase.verifyFalse(guide.fieldActive);
            testCase.verifyTrue(isnan(guide.conflictStartSeconds));
        end
        function crossingMotionGeneratesATransverseTrajectoryWithoutASideInput(testCase)
            model=localModel("intersectionCrossing",8);
            guide=predictiveSafetyGeometry.avoidanceGuide(model,40);
            testCase.verifyGreaterThan(guide.inputs(1,1),0);
            testCase.verifyEqual(guide.candidateCount,1);
        end
        function smallOppositeOffsetsProduceSmallOppositeFieldCorrections(testCase)
            model=localModel("headOn",8);model.target(1:6)=[12;1e-6;0;0;0;0];
            p=struct('speed',8,'inflation',0,'preferredAcceleration',zeros(2,1));
            a=predictiveSafetyGeometry.circulationField(model.initialState,model.target,model.cfg,p);
            model.target(2)=-model.target(2);
            b=predictiveSafetyGeometry.circulationField(model.initialState,model.target,model.cfg,p);
            testCase.verifyEqual(a(2),-b(2),AbsTol=1e-10);
            testCase.verifyLessThan(abs(a(2)),1e-3);
            testCase.verifyGreaterThan(abs(a(2)),0);
        end
        function mirroredTargetMotionMirrorsTheAvoidanceForce(testCase)
            model=localModel("intersectionCrossing",8);
            p=struct('speed',8,'inflation',0,'preferredAcceleration',zeros(2,1));
            left=predictiveSafetyGeometry.circulationField(model.initialState,model.target,model.cfg,p);
            mirrored=model.target;mirrored([2,3,6,11])=-mirrored([2,3,6,11]);
            right=predictiveSafetyGeometry.circulationField(model.initialState,mirrored,model.cfg,p);
            testCase.verifyGreaterThan(left(2),0);
            testCase.verifyEqual(right,[left(1);-left(2)],AbsTol=1e-10);
        end
        function aTargetThatClearsBeforeArrivalDoesNotLeaveAVirtualWall(testCase)
            model=localModel("intersectionCrossing",8);
            model.target(4)=30;
            p=struct('speed',8,'inflation',0,'preferredAcceleration',zeros(2,1));
            [force,field]=predictiveSafetyGeometry.circulationField(model.initialState,model.target,model.cfg,p);
            testCase.verifyEqual(force,zeros(2,1),AbsTol=1e-12);
            testCase.verifyEqual(field.incidence,0);
        end
        function predictedDecelerationCanCreateAThreatWithTheSamePresentPoseAndSpeed(testCase)
            model=localModel("intersectionCrossing",8);model.target(4)=16;
            p=struct('speed',8,'inflation',0,'preferredAcceleration',zeros(2,1));
            [free,~]=predictiveSafetyGeometry.circulationField(model.initialState,model.target,model.cfg,p);
            model.target(5)=-16^2/(2*17.6);
            [braking,field]=predictiveSafetyGeometry.circulationField(model.initialState,model.target,model.cfg,p);
            testCase.verifyEqual(free,zeros(2,1),AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(braking),1);
            testCase.verifyGreaterThan(field.incidence,.5);
        end
        function predictedTurningCanClearAnInitiallyHeadOnApproach(testCase)
            model=localModel("headOn",8);
            p=struct('speed',8,'inflation',0,'preferredAcceleration',zeros(2,1));
            [straight,~]=predictiveSafetyGeometry.circulationField(model.initialState,model.target,model.cfg,p);
            % Preserve position, speed and velocity course; change NRMM curvature.
            model.target(6)=.08;model.target(3)=pi-model.target(6);
            [turning,field]=predictiveSafetyGeometry.circulationField(model.initialState,model.target,model.cfg,p);
            testCase.verifyGreaterThan(norm(straight),1);
            testCase.verifyEqual(turning,zeros(2,1),AbsTol=1e-12);
            testCase.verifyEqual(field.incidence,0);
        end
        function predictionGuidanceDoesNotDependOnTheWorldCoordinateFrame(testCase)
            model=localModel("turningCrossing",8);
            model.target(10:11)=[.3;-.2];model.cfg.vehicle.rectangleOffset=[.2;.1];
            p=struct('speed',8,'inflation',.1,'preferredAcceleration',[.2;-.1]);
            [original,field]=predictiveSafetyGeometry.circulationField(model.initialState,model.target,model.cfg,p);
            angle=.7;rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];shift=[30;-20];
            x=model.initialState;x(1:2)=rotation*x(1:2)+shift;x(3)=x(3)+angle;
            target=model.target;target(1:2)=rotation*target(1:2)+shift;target(3)=target(3)+angle;
            p.preferredAcceleration=rotation*p.preferredAcceleration;
            [transformed,other]=predictiveSafetyGeometry.circulationField(x,target,model.cfg,p);
            testCase.verifyEqual(transformed,rotation*original,AbsTol=1e-10);
            testCase.verifyEqual(other.incidence,field.incidence,AbsTol=1e-10);
            testCase.verifyEqual(other.transportGradient,rotation*field.transportGradient,AbsTol=1e-10);
        end
        function aLongerRolloutExtendsExactlyTheSameFieldMotion(testCase,crossingSpeed)
            model=localModel("intersectionCrossing",crossingSpeed);
            model.uncertainty.collisionGenerator=diag([.05,.05,.01,0,0,0]);
            short=predictiveSafetyGeometry.avoidanceGuide(model,8);
            extended=predictiveSafetyGeometry.avoidanceGuide(model,64);
            testCase.verifyEqual(extended.states(:,1:9),short.states,AbsTol=1e-12);
            testCase.verifyEqual(extended.inputs(:,1:8),short.inputs,AbsTol=1e-12);
            testCase.verifyEqual(extended.candidateCount,1);
        end
        function anOverlappingSeedIsFiniteAndNeverCertified(testCase)
            model=localModel("headOn",8);model.target(1:2)=model.initialState(1:2);
            guide=predictiveSafetyGeometry.avoidanceGuide(model,16);
            testCase.verifyTrue(guide.initialConflict);
            testCase.verifyLessThanOrEqual(guide.nominalMinimumGap,0);
            testCase.verifyFalse(guide.robustCertificate);
            testCase.verifyTrue(all(isfinite([guide.states(:);guide.inputs(:)])));
        end
    end
end

function derivative=localWallMotion(z,model)
    x=model.initialState;x(2)=z(1);x(5)=z(2);
    a=predictiveSafetyGeometry.wallGuidance(x,[0;100],model);
    derivative=[z(2);a(2)];
end

function model=localModel(name,speed)
    [x,target,road,cfg]=collisionThreatScenario(name,struct('referenceSpeed',speed));
    raw=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
    [~,lane,road]=readControllerInputs(raw,[],road,cfg);
    frame=predictiveSafetyGeometry.roadFrame(lane,road);
    model=struct('cfg',cfg,'initialState',x,'target',target,'targetEpoch',target, ...
        'targetEpochIndex',0,'sampleIndex',0,'targetOnset',true,'previousInput',[0;0], ...
        'lane',lane,'road',road,'nominalReference',nonlinearBicycleModel.cruise(cfg,frame(4)), ...
        'uncertainty',struct('collisionGenerator',zeros(6),'egoGenerator',zeros(6),'target',[]));
end
