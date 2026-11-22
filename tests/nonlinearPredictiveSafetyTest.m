classdef nonlinearPredictiveSafetyTest < matlab.unittest.TestCase
    % Target and vehicle dynamics, geometry and held-input integration.
    properties (TestParameter)
        targetSideslip={0,1e-12,.08,-.08};
        targetAcceleration={0,1.2,-1.2};
        orientation={0,.5,2.9};
        curvature={0,.005,-.005};
        referenceSpeed={8,15};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function predictTargetHasTheSemigroupProperty(testCase,targetSideslip,targetAcceleration)
            q=[3;1;.2;7;targetAcceleration;targetSideslip;1.6;2.4;.95;0;0];
            first=predictiveSafetyGeometry.predictTarget(q,.37);
            actual=predictiveSafetyGeometry.predictTarget(first,.29);
            expected=predictiveSafetyGeometry.predictTarget(q,.66);
            testCase.verifyEqual(actual,expected,AbsTol=2e-14);
        end
        function predictTargetMatchesIndependentIntegration(testCase,targetSideslip,targetAcceleration)
            q=[3;1;.2;7;targetAcceleration;targetSideslip;1.6;2.4;.95;.2;-.1];
            [~,states]=ode45(@(~,x)[x(4)*cos(x(3)+targetSideslip); ...
                x(4)*sin(x(3)+targetSideslip);x(4)*sin(targetSideslip)/1.6;targetAcceleration], ...
                [0,1.3],q(1:4),odeset('RelTol',1e-12,'AbsTol',1e-13));
            actual=predictiveSafetyGeometry.predictTarget(q,1.3);
            testCase.verifyEqual(actual(1:4),states(end,:).',AbsTol=2e-10);
            testCase.verifyEqual(actual(5:end),q(5:end));
        end
        function accelerationChangesSpeedAndHeadingRate(testCase)
            q=[0;0;.2;6;1.5;.1;1.6;2.4;.95;0;0];
            next=predictiveSafetyGeometry.predictTarget(q,2);
            testCase.verifyEqual(next(4),9);
            testCase.verifyEqual(next(3),q(3)+15*sin(.1)/1.6,AbsTol=1e-14);
            testCase.verifyEqual(next(4)*sin(next(6))/next(7),1.5*q(4)*sin(q(6))/q(7),AbsTol=1e-14);
        end
        function brakingEndsAtRest(testCase)
            % Braking stops the target and never turns it back; the state at
            % rest carries A = 0, so the forecast keeps the semigroup property.
            q=[0;0;0;2;-2;0;1.6;2.4;.95;0;0];
            stopped=predictiveSafetyGeometry.predictTarget(q,1);
            later=predictiveSafetyGeometry.predictTarget(stopped,2);
            testCase.verifyEqual(stopped(1:5),[1;0;0;0;0]);
            testCase.verifyEqual(later,stopped);
            testCase.verifyEqual(later,predictiveSafetyGeometry.predictTarget(q,3));
            testCase.verifyEqual(predictiveSafetyGeometry.predictTarget(q,[.5,1,7]), ...
                [predictiveSafetyGeometry.predictTarget(q,.5),stopped,stopped]);
            % A reversing target that brakes stops as well.
            back=predictiveSafetyGeometry.predictTarget([0;0;0;-3;1;0;1.6;2.4;.95;0;0],10);
            testCase.verifyEqual(back(1:5),[-4.5;0;0;0;0]);
        end
        function aTargetAtRestMovesOffOnlyForward(testCase)
            q=[0;0;0;0;-2;0;1.6;2.4;.95;0;0];
            still=predictiveSafetyGeometry.predictTarget(q,4);
            testCase.verifyEqual(still(1:5),[0;0;0;0;0]);
            q(5)=2;moving=predictiveSafetyGeometry.predictTarget(q,1);
            testCase.verifyEqual(moving(1:5),[1;0;0;2;2]);
        end
        function aTurningTargetStopsOnItsArc(testCase,targetSideslip)
            q=[3;1;.2;2;-2;targetSideslip;1.6;2.4;.95;0;0];
            stopped=predictiveSafetyGeometry.predictTarget(q,1);
            testCase.verifyEqual(predictiveSafetyGeometry.predictTarget(q,5),stopped);
            testCase.verifyEqual(stopped(4:7),[0;0;targetSideslip;1.6]);
            % It rests at the end of its stopping arc, V^2/(2|A|) = 1 m long.
            moving=q;moving(4)=1;moving(5)=0;along=predictiveSafetyGeometry.predictTarget(moving,1);
            testCase.verifyEqual(stopped(1:3),along(1:3),AbsTol=2e-14);
        end
        function aStopCannotBeRunBackward(testCase)
            q=[0;0;0;1;2;0;1.6;2.4;.95;0;0];
            testCase.verifyError(@()predictiveSafetyGeometry.predictTarget(q,-1), ...
                'collisionAvoidanceController:irreversibleTargetStop');
            back=predictiveSafetyGeometry.predictTarget(q,-.25);
            testCase.verifyEqual(predictiveSafetyGeometry.predictTarget(back,.25),q,AbsTol=1e-15);
        end
        function rectangleDualMatchesDistance(testCase,orientation)
            e=[0;0;orientation];t=[10;3;-.2];shape=[2.4;.95;0;0];
            [distance,dual]=predictiveSafetyGeometry.rectangle(e,shape,t,shape);
            testCase.verifyGreaterThan(distance,0);
            testCase.verifyEqual(dual.value,distance,AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual([dual.mu;dual.lambda],zeros(8,1));
            testCase.verifyEqual(norm(dual.normal),1,AbsTol=1e-12);
        end
        function overlapDoesNotInventARestorationDirection(testCase)
            shape=[2.4;.95;0;0];
            rows=predictiveSafetyGeometry.dualLinearization([0;0;0],shape,[0;0;0],shape);
            testCase.verifyEqual(rows.distance,0,AbsTol=1e-7);
            testCase.verifyEqual(rows.normal,zeros(2,1),AbsTol=1e-7);
            testCase.verifyEqual(rows.jacobian,zeros(4,3),AbsTol=1e-7);
        end
        function analyticDynamicsTangentMatchesFiniteDifferences(testCase,curvature)
            [~,~,cfg]=localFixture();x=[2;.2;.1;8;.3;.1];u=[.03;-.12];
            [a,b]=nonlinearBicycleModel.jacobian(x,u,cfg,curvature);
            finite=zeros(6,8);point=[x;u];h=1e-6;
            for j=1:8
                lo=point;hi=point;lo(j)=lo(j)-h;hi(j)=hi(j)+h;
                finite(:,j)=(nonlinearBicycleModel.derivative(hi(1:6),hi(7:8),cfg,curvature) ...
                    -nonlinearBicycleModel.derivative(lo(1:6),lo(7:8),cfg,curvature))/(2*h);
            end
            testCase.verifyEqual([a,b],finite,AbsTol=2e-6);
        end
        function lowSpeedTangentMatchesFiniteDifferences(testCase,curvature)
            % The blended (3.2 m/s) and kinematic (1.5 and 0.1 m/s) regimes,
            % braking and driving.
            [~,~,cfg]=localFixture();
            for state=[[2;.2;.1;3.2;.1;.05],[2;.2;.1;1.5;.05;.04],[2;.2;.1;.1;.002;.01]]
                for u=[[.03;-.4],[-.05;.2]]
                    [a,b]=nonlinearBicycleModel.jacobian(state,u,cfg,curvature);
                    finite=zeros(6,8);point=[state;u];h=1e-7;
                    for j=1:8
                        lo=point;hi=point;lo(j)=lo(j)-h;hi(j)=hi(j)+h;
                        finite(:,j)=(nonlinearBicycleModel.derivative(hi(1:6),hi(7:8),cfg,curvature) ...
                            -nonlinearBicycleModel.derivative(lo(1:6),lo(7:8),cfg,curvature))/(2*h);
                    end
                    testCase.verifyEqual([a,b],finite,AbsTol=1e-5);
                end
            end
        end
        function independentReplayAndPredictionShareTheRestTransition(testCase)
            [~,~,cfg]=localFixture();cfg.nonlinear.integrationStep=.005;
            x=[2;-.1;.2;.04;.001;-.002];u=[.02;-.48];
            [~,replay]=ode45(@(~,state)nonlinearBicycleModel.derivative(state,u,cfg), ...
                [0,cfg.controller.sampleTime],x,odeset('RelTol',1e-11,'AbsTol',1e-12));
            integrated=replay(end,:).';
            testCase.verifyGreaterThan(integrated(4),0);
            [actual,stopped]=nonlinearBicycleModel.sampleEndState(integrated,u,cfg);
            [predicted,a,b]=nonlinearBicycleModel.sample(x,u,cfg);
            testCase.verifyTrue(stopped);
            testCase.verifyEqual(actual(1:3),integrated(1:3));
            testCase.verifyEqual(actual(4:6),predicted(4:6));
            testCase.verifyEqual(actual(4:6),zeros(3,1));
            testCase.verifyEqual([a(4:6,:),b(4:6,:)],zeros(3,8));
            testCase.verifyEqual(actual(1:3),predicted(1:3),AbsTol=1e-6);
        end
        function theRestTransitionPreservesDrivingAndFasterMotion(testCase)
            [~,~,cfg]=localFixture();x=[2;-.1;.2;.01;.001;-.002];
            [next,stopped]=nonlinearBicycleModel.sampleEndState(x,[.02;.1],cfg);
            testCase.verifyFalse(stopped);testCase.verifyEqual(next,x);
            x(4)=.2*cfg.model.brakeHoldSpeed;
            [next,stopped]=nonlinearBicycleModel.sampleEndState(x,[.02;-.48],cfg);
            testCase.verifyFalse(stopped);testCase.verifyEqual(next,x);
        end
        function theVehicleBrakesToRestAndMovesOffFromIt(testCase)
            % Braking at a nominal 4 m/s^2 with the steering held ends
            % at rest, never in reverse, with the handling rows met.
            [~,~,cfg]=localFixture();
            u=[.05;-4/nonlinearBicycleModel.tireAccelerationGain(cfg)];
            x=[0;0;0;6;0;0];
            for k=1:60
                x=nonlinearBicycleModel.sample(x,u,cfg);
                testCase.verifyGreaterThanOrEqual(x(4),0);
                testCase.verifyTrue(terminalSafeSet.stateRows(x,cfg) && terminalSafeSet.handlingRows(x,u,cfg));
            end
            % At rest exactly, with zero sensitivities of the body velocities.
            testCase.verifyEqual(x(4:6),zeros(3,1));
            [~,a,b]=nonlinearBicycleModel.sample(x,u,cfg);
            testCase.verifyEqual([a(4:6,:),b(4:6,:)],zeros(3,8));
            % 4.4 m against 4.5 m for constant braking at a_b: road load
            % adds to the braking and the braking fades near rest.
            testCase.verifyEqual(x(1),4.4,AbsTol=.1);
            rest=nonlinearBicycleModel.sample([0;0;0;0;0;0],[0;-.5],cfg);
            testCase.verifyEqual(rest,zeros(6,1));
            moving=[0;0;0;0;0;0];
            for k=1:20,moving=nonlinearBicycleModel.sample(moving,[0;.2],cfg);end
            testCase.verifyGreaterThan(moving(4),1);
            % Above the kinematic range the model is the Fiala bicycle.
            x=[0;0;.1;6;.2;.1];
            testCase.verifyEqual(nonlinearBicycleModel.derivative(x,[.02;-.1],cfg), ...
                nonlinearBicycleModel.derivative(x,[.02;-.1],setfield(cfg,'model',setfield(cfg.model,'kinematicSpeedRange',[2,3]))));
        end
        function jointTargetCoordinatesAreUnactuated(testCase)
            [~,~,cfg]=localFixture();z=[0;0;0;8;0;0;20;3;.4;6];parameters=[1.2;.08;1.6;2.4;.95;0;0];
            [next,a,b]=nonlinearBicycleModel.jointSample(z,[.01;.02],parameters,cfg);
            target=predictiveSafetyGeometry.predictTarget([z(7:10);parameters],cfg.controller.sampleTime);
            testCase.verifyEqual(next(7:10),target(1:4),AbsTol=1e-14);
            testCase.verifyEqual(a(1:6,7:10),zeros(6,4));
            testCase.verifyEqual(b(7:10,:),zeros(4,2));
            testCase.verifyEqual(nonlinearBicycleModel.jointSample(z,[.01;.02],parameters,cfg),next);
        end
        function jointTargetTangentIncludesSpeedDependentTurning(testCase,targetSideslip)
            [~,~,cfg]=localFixture();z=[0;0;0;8;0;0;20;3;.4;6];parameters=[1.2;targetSideslip;1.6;2.4;.95;0;0];
            [~,a]=nonlinearBicycleModel.jointSample(z,[0;0],parameters,cfg);
            finite=localTargetTangent([z(7:10);parameters],cfg.controller.sampleTime);
            testCase.verifyEqual(a(7:10,7:10),finite,AbsTol=2e-9);
        end
        function fullAndHalfHoldsUseTheSamePredictionMap(testCase)
            [~,~,cfg]=localFixture();x=[0;.2;.1;8;.3;.1];u=[.03;-.12];
            cfg.nonlinear.integrationStep=.005; % a mesh that has the midpoint as a node
            [full,a,b]=nonlinearBicycleModel.sample(x,u,cfg);
            [middle,am,bm]=nonlinearBicycleModel.sample(x,u,cfg,[],cfg.controller.sampleTime/2);
            [last,an,bn]=nonlinearBicycleModel.sample(middle,u,cfg,[],cfg.controller.sampleTime/2);
            testCase.verifyEqual(last,full,AbsTol=1e-14);
            testCase.verifyEqual(an*am,a,AbsTol=1e-13);
            testCase.verifyEqual(an*bm+bn,b,AbsTol=1e-13);
        end
        function aOneStepMeshUsesItsInterpolantAsTheMidpoint(testCase)
            [~,~,cfg]=localFixture();cfg.nonlinear.integrationStep=cfg.controller.sampleTime;
            x=[0;.2;.1;8;.3;.1];u=[.03;-.12];h=1e-6;
            [middle,am,bm,next,a,b]=nonlinearBicycleModel.hold(x,u,cfg);
            [full,af,bf]=nonlinearBicycleModel.sample(x,u,cfg);
            finite=zeros(6);
            for j=1:6
                d=zeros(6,1);d(j)=h;
                finite(:,j)=(nonlinearBicycleModel.sample(x+d,u,cfg)-nonlinearBicycleModel.sample(x-d,u,cfg))/(2*h);
            end
            testCase.verifyEqual(nonlinearBicycleModel.meshCount(cfg),1);
            testCase.verifyEqual(next,full,AbsTol=0);
            testCase.verifyEqual(a,af,AbsTol=0);
            testCase.verifyEqual(b,bf,AbsTol=0);
            testCase.verifyEqual(a,finite,AbsTol=2e-8);
            testCase.verifyEqual(middle,(x+next)/2,AbsTol=0);
            testCase.verifyEqual(am,(eye(6)+a)/2,AbsTol=0);
            testCase.verifyEqual(bm,b/2,AbsTol=0);
        end
        function opposingHeadingPreservesExactlyStraightTargetMotion(testCase)
            q=[24;6;pi;2;1;0;1.6;2.4;.95;0;0];
            next=predictiveSafetyGeometry.predictTarget(q,1e8);
            testCase.verifyEqual(next(2),q(2));
            q(3)=pi-1e-12;
            next=predictiveSafetyGeometry.predictTarget(q,1e8);
            testCase.verifyGreaterThan(next(2),q(2)+1);
        end
    end
end

function [ego,road,cfg]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[8.5344;12.192]);
end

function finite=localTargetTangent(q,time)
    finite=zeros(4);h=1e-6;
    for j=1:4
        lo=q;hi=q;lo(j)=lo(j)-h;hi(j)=hi(j)+h;
        a=predictiveSafetyGeometry.predictTarget(hi,time);b=predictiveSafetyGeometry.predictTarget(lo,time);
        finite(:,j)=(a(1:4)-b(1:4))/(2*h);
    end
end
