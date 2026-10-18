classdef circulationPlannerTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for directory=["controller","config","scripts"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,directory)));
            end
        end
    end
    methods (Test)
        function pathAttractionPullsBackAndDampsOutwardMotion(testCase)
            model=localModel("headOn",8);x=model.initialState;x(2)=3;
            a=predictiveSafetyGeometry.pathAttraction(x,model);
            x(5)=2;b=predictiveSafetyGeometry.pathAttraction(x,model);
            testCase.verifyLessThan(a(2),0);
            testCase.verifyLessThan(b(2),a(2));
            x(2)=0;x(5)=0;
            c=predictiveSafetyGeometry.pathAttraction(x,model);
            testCase.verifyEqual(c,zeros(2,1),AbsTol=1e-12);
        end
        function aRecedingTargetDoesNotPullTheVehicleIntoAnOrbit(testCase)
            model=localModel("headOn",8);target=model.target;target(1:6)=[-30;0;0;2;0;0];
            p=struct('direction',1,'speed',4,'inflation',0,'pathAcceleration',[.2;0]);
            [a,f]=predictiveSafetyGeometry.circulationField(model.initialState,target,model.cfg,p);
            testCase.verifyEqual(a,p.pathAcceleration,AbsTol=1e-12);
            testCase.verifyEqual(f.activation,0);
        end
        function parallelWallSlowsOutwardMotionBeforeReachingTheWall(testCase)
            model=localModel("headOn",8);x=model.initialState;x(2)=8;x(5)=3;
            [a,active]=predictiveSafetyGeometry.wallGuidance(x,[1;100],model);
            testCase.verifyTrue(active);testCase.verifyLessThan(a(2),0);
            testCase.verifyEqual(a(1),1,AbsTol=1e-12);
            % Ideal acceleration realization only: a persistent outward
            % request approaches the wall without crossing it.
            [~,z]=ode45(@(t,z)localWallMotion(z,model),[0,6],[8;3], ...
                odeset('RelTol',1e-9,'AbsTol',1e-11));
            testCase.verifyLessThan(max(z(:,1)),10);
            testCase.verifyGreaterThan(z(end,1),9.9);
        end
        function aWallDoesNotResistAnAlreadyInwardAcceleration(testCase)
            model=localModel("headOn",8);x=model.initialState;x(2)=8;x(5)=-1;
            [a,active]=predictiveSafetyGeometry.wallGuidance(x,[1;-1],model);
            testCase.verifyFalse(active);testCase.verifyEqual(a,[1;-1],AbsTol=0);
        end
        function rotatingFrameTransportRetainsTheRequestedFrozenGeometryAcceleration(testCase)
            model=localModel("turningCrossing",8);x=model.initialState;
            target=model.target;target(10:11)=[.4;-.2];x(2)=3;x(5)=.3;
            parameters=struct('direction',-1,'speed',4,'inflation',.15);
            [a,f]=predictiveSafetyGeometry.circulationField(x,target,model.cfg,parameters);
            relative=f.rotation.'*a-f.transport;
            radial=f.normal.'*relative+f.curvature*f.tangentSpeed^2;
            tangential=f.tangent.'*relative-f.curvature*f.normalSpeed*f.tangentSpeed;
            rate=model.cfg.planner.radialRate;
            expected=-2*rate*f.normalSpeed-rate^2*(f.clearance-f.desiredClearance);
            testCase.verifyEqual(radial,expected,AbsTol=1e-10);
            testCase.verifyEqual(tangential,model.cfg.planner.tangentialRate*(-4-f.tangentSpeed),AbsTol=1e-10);
        end
        function headOnGuidanceBreaksSymmetryWithoutChangingTheInitialState(testCase)
            model=localModel("headOn",8);p=struct('direction',1,'speed',4,'inflation',0);
            a=predictiveSafetyGeometry.circulationField(model.initialState,model.target,model.cfg,p);
            p.direction=-1;b=predictiveSafetyGeometry.circulationField(model.initialState,model.target,model.cfg,p);
            testCase.verifyGreaterThan(abs(a(2)),1);
            testCase.verifyEqual(a(2),-b(2),AbsTol=1e-10);
            guide=predictiveSafetyGeometry.avoidanceGuide(model,40,1);
            testCase.verifyEqual(guide.states(:,1),model.initialState,AbsTol=0);
            testCase.verifyFalse(guide.robustCertificate);
        end
        function returnedStatesComeFromTheActualHeldInputModel(testCase)
            model=localModel("crossing",8);model.cfg.model.brakingRatioRateMaximum=1;
            guide=predictiveSafetyGeometry.avoidanceGuide(model,50,1);
            testCase.verifyEqual(guide.candidateCount,1);
            for k=1:size(guide.inputs,2)
                next=nonlinearBicycleModel.sample(guide.states(:,k),guide.inputs(:,k),model.cfg);
                testCase.verifyEqual(guide.states(:,k+1),next,AbsTol=1e-12);
            end
            testCase.verifyLessThanOrEqual(max(abs(diff([model.previousInput(2),guide.inputs(2,:)]))),model.cfg.controller.sampleTime+1e-12);
            testCase.verifyTrue(all(isfinite([guide.states(:);guide.inputs(:)])));
        end
        function aSmallPositiveRectangleGapDoesNotTriggerDiskAvoidance(testCase)
            model=localModel("headOn",8);model.target(1:6)=[0;2;0;8;0;0];
            guide=predictiveSafetyGeometry.avoidanceGuide(model,16,1);
            testCase.verifyEqual(guide.nominalMinimumGap,.1,AbsTol=1e-6);
            testCase.verifyEqual(guide.directionSign,0);
            testCase.verifyTrue(isnan(guide.conflictStartSeconds));
        end
        function anUnknownTargetYawDoesNotRequireTwoRotationDiameters(testCase)
            model=localModel("headOn",8);model.target(1:6)=[0;4;0;0;0;0];
            model.targetEpoch=model.target;
            model.uncertainty.target=struct('positionRadius',0,'courseRadius',pi, ...
                'speedRadius',0,'accelerationRadius',0,'curvatureRadius',0,'sideslipRadius',0);
            guide=predictiveSafetyGeometry.avoidanceGuide(model,16,1);
            testCase.verifyGreaterThan(guide.minimumGap,0);
            testCase.verifyEqual(guide.directionSign,0);
            testCase.verifyTrue(isnan(guide.conflictStartSeconds));
        end
        function replanningKeepsTheSelectedCirculationDirection(testCase)
            model=localModel("headOn",8);model.circulationDirection=-1;
            guide=predictiveSafetyGeometry.avoidanceGuide(model,40,1);
            directions=[guide.candidates.direction];
            testCase.verifyTrue(all(directions==0 | directions==-1));
        end
        function anOverlappingSeedIsFiniteAndNeverCertified(testCase)
            model=localModel("headOn",8);model.target(1:2)=model.initialState(1:2);
            guide=predictiveSafetyGeometry.avoidanceGuide(model,16,1);
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
