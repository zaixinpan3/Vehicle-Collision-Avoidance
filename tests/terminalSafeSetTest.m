classdef terminalSafeSetTest < matlab.unittest.TestCase
    % Terminal set of the braking continuation: the closed-form continuation,
    % invariance under the terminal law, the exact parked check against the
    % target's track, the end of an encounter, the linearized rows, and the
    % declared-model holds that extend the anchor.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function jointMembershipChecksTheTerminalEntrySlew(testCase)
            model=localModel([0;0;0;8;0;0],[]);
            model.cfg.model.brakingRatioRateMaximum=1;
            context=terminalSafeSet.context(model);
            joint=terminalSafeSet.jointState(context,model.initialState,0);
            [member,info]=terminalSafeSet.jointMember(context,joint);
            testCase.verifyFalse(member);testCase.verifyEqual(info.reason,"terminalInputSlew");
            joint.heldInput(2)=context.brakingRatio;
            testCase.verifyTrue(terminalSafeSet.jointMember(context,joint));
        end
        function terminalMembershipRequiresAdmissibleRestAndHoldingInput(testCase)
            model=localModel([0;0;0;8;0;0],[]);model.cfg.model.speedMinimum=1;
            context=terminalSafeSet.context(model);
            [member,~,info]=terminalSafeSet.member(context,model.initialState,0);
            testCase.verifyFalse(member);testCase.verifyEqual(info.reason,"terminalStateBounds");
            model=localModel(zeros(6,1),[]);model.cfg.actuation.brakingRatioMinimum=0;
            context=terminalSafeSet.context(model);
            [member,~,info]=terminalSafeSet.member(context,model.initialState,0);
            testCase.verifyFalse(member);testCase.verifyEqual(info.reason,"terminalInputBounds");
        end
        function aSubsampleStopUsesConstantBrakingWithoutReversing(testCase)
            model=localModel([0;0;0;.1;0;0],[]);context=terminalSafeSet.context(model);
            joint=terminalSafeSet.jointState(context,model.initialState,0);
            command=terminalSafeSet.control(context,joint);
            [next,middle]=terminalSafeSet.jointSuccessor(context,joint);
            testCase.verifyEqual(command.tangentialAcceleration,-4);
            testCase.verifyEqual(next.ego,[.1^2/8;0;0;0;0;0],AbsTol=1e-14);
            testCase.verifyEqual(middle.ego,next.ego,AbsTol=1e-14);
            stopped=terminalSafeSet.jointSuccessor(context,next);
            testCase.verifyEqual(stopped.ego,next.ego,AbsTol=1e-14);
        end
        function anEndedJointEncounterCannotReactivateOnPredictedReentry(testCase)
            beta=asin(1.6/30);phase=-pi+.35;
            q=[30+30*cos(phase);30*sin(phase);phase+pi/2-beta;8;0;beta;1.6;2.4;.95;0;0];
            model=localModel(zeros(6,1),q);context=terminalSafeSet.context(model);
            joint=terminalSafeSet.jointState(context,model.initialState,0);
            [member,info]=terminalSafeSet.jointMember(context,joint);
            testCase.verifyTrue(member);testCase.verifyTrue(isfinite(info.exitSeconds));
            ended=false;
            for step=1:ceil(23/context.sampleTime)
                joint=terminalSafeSet.jointSuccessor(context,joint);
                if ~joint.active,ended=true;end
                if ended,testCase.verifyFalse(joint.active);end
            end
            testCase.verifyTrue(ended);
            testCase.verifyLessThan(norm(joint.target(1:2)-joint.ego(1:2)),context.range);
            testCase.verifyTrue(terminalSafeSet.jointMember(context,joint));
            fresh=joint;fresh.active=true;
            % Returning just before contact belongs to a different problem.
            fresh.target=predictiveSafetyGeometry.predictTarget(q,21.5);
            testCase.verifyFalse(terminalSafeSet.jointMember(context,fresh));
        end
        function theParkedTrackRejectsContainmentAtThePhysicalStop(testCase)
            q=[0;0;0;.1;-1;0;1.6;.5;.3;0;0];
            model=localModel(zeros(6,1),q);context=terminalSafeSet.context(model);
            [distance,~]=terminalSafeSet.parkedClearance(context,zeros(3,1),q);
            testCase.verifyEqual(distance,0,AbsTol=1e-12);
        end
        function aTargetBeforeItsFirstExitStillThreatensTheStoppedEgo(testCase)
            q=[40;0;pi;8;0;0;1.6;2.4;.95;0;0];
            model=localModel(zeros(6,1),q);context=terminalSafeSet.context(model);
            joint=terminalSafeSet.jointState(context,model.initialState,0);
            [member,info]=terminalSafeSet.jointMember(context,joint);
            testCase.verifyFalse(member);testCase.verifyEqual(info.reason,"parkedEgoMeetsTarget");
        end
        function theJointSetDependsOnTargetMotionAndPerceptionRange(testCase)
            q=[40;0;pi;8;0;0;1.6;2.4;.95;0;0];
            model=localModel(zeros(6,1),q);context=terminalSafeSet.context(model);
            joint=terminalSafeSet.jointState(context,model.initialState,0);
            testCase.verifyFalse(terminalSafeSet.jointMember(context,joint));
            away=joint;away.target(3)=0;
            testCase.verifyTrue(terminalSafeSet.jointMember(context,away));
            context.range=30;
            outside=terminalSafeSet.jointState(context,model.initialState,0);
            testCase.verifyFalse(outside.active);
            testCase.verifyTrue(terminalSafeSet.jointMember(context,outside));
        end
        function theContinuationIsTheExactFlowOfTheTerminalLaw(testCase)
            % The closed-form poses at nodes and hold midpoints equal a fine
            % RK4 integration of the constant-(A, beta) kinematics with
            % A = -a_b until a forward-only stop and the course curvature
            % held, including a stop inside the last hold.
            model=localModel([3;-1;.2;8;.3;.12],[]);context=terminalSafeSet.context(model);
            x=model.initialState;path=terminalSafeSet.continuation(context,x);
            h=context.sampleTime;a=context.deceleration;V=hypot(x(4),x(5));
            testCase.verifyEqual(path.steps,ceil(V/(a*h)));
            beta=atan2(x(5),x(4));kappa=x(6)/V;state=[x(1:3);V];steps=400;
            for hold=1:path.steps
                A=-a;
                for half=1:2
                    for k=1:steps
                        f=@(s)[s(4)*cos(s(3)+beta);s(4)*sin(s(3)+beta);kappa*s(4);A];
                        d=min(h/2/steps,max(0,state(4))/a);k1=f(state);k2=f(state+d/2*k1);k3=f(state+d/2*k2);k4=f(state+d*k3);
                        state=state+d/6*(k1+2*k2+2*k3+k4);state(4)=max(0,state(4));
                    end
                    testCase.verifyEqual(path.pose(:,2*(hold-1)+half+1),state(1:3),AbsTol=1e-9);
                end
            end
            testCase.verifyEqual(state(4),0,AbsTol=1e-9);
        end
        function theSetIsInvariantUnderTheTerminalLaw(testCase)
            % Every state of the continuation of a member is a member, until
            % the ego is parked, and its continuation is the remainder.
            q=[30;3.66;pi;8;0;0;1.6;2.4;.95;0;0];
            model=localModel([0;0;0;8;.1;.05],q);context=terminalSafeSet.context(model);
            x=model.initialState;[member,~,info]=terminalSafeSet.member(context,x,0);
            testCase.verifyTrue(member);testCase.verifyEqual(info.reason,"parked");
            path=terminalSafeSet.continuation(context,x);
            for node=1:path.steps
                x=terminalSafeSet.successor(context,x);
                [member,~,next]=terminalSafeSet.member(context,x,node);
                testCase.verifyTrue(member);
                testCase.verifyEqual(next.stopSeconds,max(0,info.stopSeconds-node*context.sampleTime),AbsTol=1e-9);
                testCase.verifyEqual(next.exitSeconds,info.exitSeconds-node*context.sampleTime,AbsTol=1e-6);
                rest=terminalSafeSet.continuation(context,x);
                testCase.verifyEqual(rest.pose,path.pose(:,2*node+1:end),AbsTol=1e-9);
            end
            testCase.verifyEqual(hypot(x(4),x(5)),0);
        end
        function theParkedCheckIsExactOverTheTargetsTrack(testCase)
            % Straight, circling, braking (forward and in reverse) and
            % stationary targets: the closed form is attained on the swept
            % stretch and no sampled pose of the track comes closer.
            context=struct('range',50,'shape',[2.4;.95;0;0]);
            targets={[12;-3;2.6;4;0;0;1.6;2.4;.95;0;0],[-9;4;-.4;6;-1.5;0;1.6;2.4;.95;0;0], ...
                [5;10;.3;5;0;.15;1.6;2.4;.95;0;0],[-8;-6;1;-3;.8;-.1;1.6;2.4;.95;0;0], ...
                [7;2;.5;0;0;0;1.6;2.4;.95;0;0]};
            for j=1:numel(targets)
                q=targets{j};pose=[0;0;.4];
                [distance,witness,track]=terminalSafeSet.parkedClearance(context,pose,q);
                testCase.verifyEqual(predictiveSafetyGeometry.rectangle(pose,context.shape,witness(1:3),witness(8:11)), ...
                    distance,AbsTol=1e-9);
                horizon=min(track.exitSeconds,60);
                times=linspace(0,horizon,20001);poses=predictiveSafetyGeometry.predictTarget(q,times);
                inside=find(vecnorm(poses(1:2,:)-pose(1:2),2,1)>context.range,1);
                if ~isempty(inside),poses=poses(:,1:inside-1);end
                sampled=Inf;
                for k=1:size(poses,2)
                    sampled=min(sampled,predictiveSafetyGeometry.rectangle(pose,context.shape,poses(1:3,k),poses(8:11,k)));
                end
                testCase.verifyLessThanOrEqual(distance,sampled+1e-12);
                testCase.verifyLessThanOrEqual(sampled-distance,.02);
            end
            % A braking target comes to rest after V^2/(2|A|) and stays there:
            % its track ends at the rest point, inside the range for ever.
            q=[0;-6;pi/2;3;-1;0;1.6;2.4;.95;0;0];
            [~,~,track]=terminalSafeSet.parkedClearance(context,[30;0;0],q);
            testCase.verifyEqual([track.lower,track.upper],[0,4.5],AbsTol=1e-12);
            testCase.verifyEqual(track.exitSeconds,Inf);
            % At rest with A < 0 it stays; with A > 0 it moves off forward.
            [~,~,track]=terminalSafeSet.parkedClearance(context,[30;0;0],[0;-6;pi/2;0;-1;0;1.6;2.4;.95;0;0]);
            testCase.verifyEqual([track.lower,track.upper],[0,0]);
            [~,~,track]=terminalSafeSet.parkedClearance(context,[30;0;0],[0;-6;pi/2;0;1;0;1.6;2.4;.95;0;0]);
            testCase.verifyEqual(track.lower,0);testCase.verifyGreaterThan(track.upper,30);
        end
        function parkedLineCollisionRestoresAcrossTheTrackInsteadOfAlongIt(testCase)
            x=[0;0;0;8;0;0];q=[40;0;pi;8;0;0;1.6;2.4;.95;0;0];
            model=localModel(x,q);context=terminalSafeSet.context(model);
            [rows,bounds]=terminalSafeSet.rows(context,x,0);
            along=[5;0;0;0;0;0];across=[0;3;0;0;0;0];
            testCase.verifyFalse(terminalSafeSet.member(context,x+along,0));
            testCase.verifyTrue(terminalSafeSet.member(context,x+across,0));
            testCase.verifyGreaterThanOrEqual(max(rows*along-bounds),max(-bounds)-1e-8);
            testCase.verifyLessThanOrEqual(max(rows*across-bounds),1e-8);
        end
        function parkedCircleCollisionRestoresBeyondTheSweptArc(testCase)
            x=[11;0;0;8;0;0];beta=asin(1.6/8);
            q=[8;8;pi-beta;6;0;beta;1.6;2.4;.95;0;0];
            model=localModel(x,q);context=terminalSafeSet.context(model);
            testCase.verifyFalse(terminalSafeSet.member(context,x,0));
            [rows,bounds,context]=terminalSafeSet.rows(context,x,0);
            shift=[2;0;0;0;0;0];
            testCase.verifyTrue(terminalSafeSet.member(context,x+shift,0));
            testCase.verifyLessThanOrEqual(max(rows*shift-bounds),1e-6);
            testCase.verifyEqual(context.trackSide,1);
        end
        function positiveClearanceNeedsOnlyTheActualUncertaintyReserve(testCase)
            model=localModel([0;0;0;0;0;0],[4.9;0;0;0;0;0;1.6;2.4;.95;0;0]);
            context=terminalSafeSet.context(model);
            [member,~,info]=terminalSafeSet.member(context,model.initialState,0);
            testCase.verifyTrue(member);
            testCase.verifyEqual(info.margin,.1,AbsTol=1e-9);
            model.uncertainty.egoGenerator=diag([.15,0,0,0,0,0]);
            context=terminalSafeSet.context(model);
            testCase.verifyEqual(context.required,.15,AbsTol=1e-12);
            testCase.verifyFalse(terminalSafeSet.member(context,model.initialState,0));
        end
        function theEncounterEndsAtTheExitOrAfterTheParkedCheck(testCase)
            % A lead far enough ahead: the ego stops behind it and the lead
            % leaves the range of the parked ego. A slower lead only a little
            % ahead still leaves room to stop.
            % The ego stops in 2 s after 8 m (4 m/s^2 from 8 m/s).
            lead=localDecision(testCase,[0;0;0;8;0;0],[40;0;0;6;0;0;1.6;2.4;.95;0;0],true,"parked");
            testCase.verifyEqual(lead.exitSeconds,2+(50-(40+6*2-8))/6,AbsTol=1e-9);
            close=localDecision(testCase,[0;0;0;8;0;0],[8;0;0;6;0;0;1.6;2.4;.95;0;0],true,"parked");
            % The gap closes until the speeds match (0.5 s): 3.2 - 0.5 m of
            % clearance with no fixed buffer.
            testCase.verifyEqual(close.margin,2.7,AbsTol=1e-6);
            % Too close to stop behind a stationary target.
            localDecision(testCase,[0;0;0;8;0;0],[6;0;0;0;0;0;1.6;2.4;.95;0;0],false,"continuationMeetsTarget");
            % A stationary target beyond the stopping distance stays in range
            % for ever and is cleared by the parked ego.
            still=localDecision(testCase,[0;0;0;8;0;0],[20;0;0;0;0;0;1.6;2.4;.95;0;0],true,"parked");
            testCase.verifyEqual(still.exitSeconds,Inf);
            % An oncoming target in the ego lane meets the parked ego; one lane
            % to the left it passes.
            localDecision(testCase,[0;0;0;8;0;0],[40;0;pi;8;0;0;1.6;2.4;.95;0;0],false,"parkedEgoMeetsTarget");
            localDecision(testCase,[0;0;0;8;0;0],[40;3.66;pi;8;0;0;1.6;2.4;.95;0;0],true,"parked");
            % A lead that brakes hard comes to rest ahead of the parked ego
            % and stays there; one that rests too close is met while the ego
            % still brakes; a gently braking lead leaves the range first.
            rest=localDecision(testCase,[0;0;0;8;0;0],[10;0;0;6;-4;0;1.6;2.4;.95;0;0],true,"parked");
            testCase.verifyEqual(rest.margin,10+4.5-8-4.8,AbsTol=1e-6);
            testCase.verifyEqual(rest.exitSeconds,Inf);
            localDecision(testCase,[0;0;0;8;0;0],[6;0;0;4;-4;0;1.6;2.4;.95;0;0],false,"continuationMeetsTarget");
            localDecision(testCase,[0;0;0;8;0;0],[20;0;0;7;-.5;0;1.6;2.4;.95;0;0],true,"parked");
            % A target leaving the range while the ego still brakes ends the
            % encounter at that sample.
            gone=localDecision(testCase,[0;0;0;8;0;0],[45;0;0;15;0;0;1.6;2.4;.95;0;0],true,"exit");
            testCase.verifyGreaterThan(gone.exitSeconds,0);testCase.verifyLessThan(gone.exitSeconds,gone.stopSeconds);
            % A target circling beside the road never comes near.
            localDecision(testCase,[0;0;0;8;0;0],[0;40;0;5;0;.05;1.6;2.4;.95;0;0],true,"parked");
        end
        function theEndpointMustBeMildAndTheContinuationOnTheRoad(testCase)
            model=localModel([0;.3;0;8;0;0],[]);
            model.cfg.controller.enforceRoadBoundaries=true;
            model.road.lateralClearance=[8.5344;12.192];context=terminalSafeSet.context(model);
            [member,~,info]=terminalSafeSet.member(context,model.initialState,0);
            testCase.verifyTrue(member);testCase.verifyEqual(info.reason,"noTarget");
            % Heading toward the left edge: the braking arc leaves the road.
            [member,~,info]=terminalSafeSet.member(context,[0;9;.6;8;0;0],0);
            testCase.verifyFalse(member);testCase.verifyEqual(info.reason,"continuationLeavesRoad");
            % The held arc's lateral acceleration r V above the mild maximum
            % (2.4 m/s^2), although well inside the tire friction; at
            % 1.6 m/s^2 the same arc is mild.
            [member,~,info]=terminalSafeSet.member(context,[0;0;0;8;0;.3],0);
            testCase.verifyGreaterThan(.3*8,context.lateralMaximum);
            testCase.verifyFalse(member);testCase.verifyEqual(info.reason,"notMild");
            [~,~,info]=terminalSafeSet.member(context,[0;0;0;8;0;.2],0);
            testCase.verifyNotEqual(info.reason,"notMild");
            % Outside the sideslip cone.
            [member,~,info]=terminalSafeSet.member(context,[0;0;0;8;3;0],0);
            testCase.verifyFalse(member);testCase.verifyEqual(info.reason,"notMild");
        end
        function terminalSideslipIsStricterThanTheEstimatorOperatingCone(testCase)
            model=localModel([0;0;0;15;2.7;0],[]);
            model.road.lateralClearance=[100;100];
            context=terminalSafeSet.context(model);
            beta=atan2(model.initialState(5),model.initialState(4));
            testCase.verifyLessThan(beta,model.cfg.model.sideslipMaximum);
            testCase.verifyGreaterThan(beta,context.sideslipMaximum);
            [member,~,info]=terminalSafeSet.member(context,model.initialState,0);
            testCase.verifyFalse(member);
            testCase.verifyEqual(info.reason,"notMild");
        end
        function smallForwardSpeedCannotHideLargeLateralMotion(testCase)
            model=localModel([0;0;0;.1;.3;0],[]);
            context=terminalSafeSet.context(model);
            testCase.verifyLessThan(model.initialState(4),context.holdSpeed);
            testCase.verifyGreaterThan(hypot(model.initialState(4),model.initialState(5)),context.holdSpeed);
            [member,~,info]=terminalSafeSet.member(context,model.initialState,0);
            testCase.verifyFalse(member);
            testCase.verifyEqual(info.reason,"notMild");
        end
        function theSettledConeRemainsConsistentWithItsRowsNearRest(testCase)
            model=localModel([0;0;0;.1;.02;0],[]);
            context=terminalSafeSet.context(model);
            testCase.verifyLessThan(hypot(.1,.02),context.holdSpeed);
            [member,~,info]=terminalSafeSet.member(context,model.initialState,0);
            testCase.verifyFalse(member);
            testCase.verifyEqual(info.reason,"notMild");
            [~,bounds]=terminalSafeSet.rows(context,model.initialState,0);
            testCase.verifyLessThan(min(bounds),0);
        end
        function mildTerminalGeometryDoesNotImposeAnExtraRearSaturationCone(testCase)
            model=localModel([0;0;0;3;.1;-.27],[]);
            model.road.lateralClearance=[100;100];
            context=terminalSafeSet.context(model);
            testCase.verifyTrue(terminalSafeSet.handlingRows(model.initialState,[0;0],model.cfg));
            testCase.verifyLessThan(atan2(.1,3),context.sideslipMaximum);
            [member,~,info]=terminalSafeSet.member(context,model.initialState,0);
            testCase.verifyTrue(member);
            testCase.verifyEqual(info.reason,"noTarget");
            [rows,bounds]=terminalSafeSet.rows(context,model.initialState,0);
            testCase.verifyGreaterThanOrEqual(min(bounds),0);
            testCase.verifyEqual(size(rows,2),6);
        end
        function theRowsLinearizeTheSetAtTheAnchor(testCase)
            % A member meets every row with the zero correction; each row
            % keeps 5 cm of the margin it has, and the rows predict the change
            % of the binding clearance to first order.
            q=[9;0;0;6;0;0;1.6;2.4;.95;0;0];
            model=localModel([0;0;0;8;0;0],q);context=terminalSafeSet.context(model);
            y=model.initialState;
            [rows,bounds,~,info]=terminalSafeSet.rows(context,y,0);
            testCase.verifyTrue(terminalSafeSet.member(context,y,0));
            testCase.verifyNotEmpty(rows);testCase.verifyGreaterThanOrEqual(min(bounds),-1e-9);
            geometryRows=any(abs(rows(:,1:3))>1e-9,2);
            testCase.verifyEqual(min(bounds(geometryRows))+.05,info.margin,AbsTol=1e-6);
            dz=[0;0;0;.2;0;0];
            [~,~,after]=terminalSafeSet.member(context,y+dz,0);
            testCase.verifyEqual(min(bounds(geometryRows)-rows(geometryRows,:)*dz)+.05,after.margin,AbsTol=5e-3);
            % At the boundary the rows keep what is left and no more.
            close=localModel([0;0;0;8;0;0],[5.32;0;0;6;0;0;1.6;2.4;.95;0;0]);
            closeContext=terminalSafeSet.context(close);
            [~,bounds,~,info]=terminalSafeSet.rows(closeContext,close.initialState,0);
            testCase.verifyLessThan(info.margin,.05);testCase.verifyGreaterThan(info.margin,0);
            testCase.verifyEqual(min(bounds),0,AbsTol=1e-12);
        end
        function theAnchorHoldsAreHoldsOfTheDeclaredModel(testCase)
            % The cruise hold decreases the CLF on its certified set.
            model=localModel([0;.25;.01;8;0;0],[]);x=model.initialState;
            [~,next,ok,value]=terminalSafeSet.cruiseInput(x,model);
            testCase.verifyTrue(ok);
            testCase.verifyLessThanOrEqual(value, ...
                model.nominalReference.contraction*nonlinearBicycleModel.nominalValue(x,model.lane,model.nominalReference)*(1+1e-9)+1e-12);
            testCase.verifyTrue(terminalSafeSet.member(terminalSafeSet.context(model),next,1));
            % Behind a slower lead whose clearance the cruise hold no longer
            % keeps, the appended hold is the one nearest the cruise hold
            % toward the law's realization that keeps the endpoint terminal;
            % the realization itself brakes at about a_b with the steering
            % held.
            % 0.55 m apart, closing at 2 m/s: braking at once keeps 0.05 m;
            % one cruise hold first creates about 0.05 m of overlap.
            model=localModel([0;0;0;8;0;0],[5.35;0;0;6;0;0;1.6;2.4;.95;0;0]);x=model.initialState;
            context=terminalSafeSet.context(model);
            testCase.verifyTrue(terminalSafeSet.member(context,x,0));
            [cruise,next]=terminalSafeSet.cruiseInput(x,model);
            testCase.verifyFalse(terminalSafeSet.member(context,next,1));
            [input,next,ok,~,fraction]=terminalSafeSet.brakingInput(x,[.01;0],cruise,model,context,0);
            testCase.verifyTrue(ok);testCase.verifyTrue(terminalSafeSet.member(context,next,1));
            testCase.verifyGreaterThan(fraction,0);testCase.verifyLessThanOrEqual(fraction,1);
            policy=terminalSafeSet.control(context,struct('heldInput',[.01;0],'active',true));
            testCase.verifyEqual(policy.actuatorInput,[.01;context.brakingRatio]);
            braking=-model.cfg.terminal.brakingDecelerationMetersPerSecondSquared/modifiedFialaTire.accelerationGain(model.cfg);
            [~,next]=terminalSafeSet.holdStates(x,[.01;braking],model.cfg);
            deceleration=(x(4)-next(4))/model.cfg.controller.sampleTime;
            testCase.verifyEqual(deceleration,model.cfg.terminal.brakingDecelerationMetersPerSecondSquared,RelTol=.1);
        end
        function holdStatesMatchTheLinearizedHold(testCase)
            model=localModel([0;.3;.02;8;.1;.05],[]);
            x=model.initialState;u=[.02;.05];
            [middle,next]=terminalSafeSet.holdStates(x,u,model.cfg);
            [m,~,~,n]=nonlinearBicycleModel.hold(x,u,model.cfg);
            testCase.verifyEqual(middle,m,AbsTol=0);
            testCase.verifyEqual(next,n,AbsTol=0);
        end
        function theBrakingRateMustLieInsideTheTireFriction(testCase)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('terminal', ...
                struct('brakingDecelerationMetersPerSecondSquared',9))),'collisionAvoidanceController:invalidConfiguration');
            % Together with the mild lateral acceleration (8.34 m/s^2 here).
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('terminal', ...
                struct('brakingDecelerationMetersPerSecondSquared',7,'lateralAccelerationMaximumMetersPerSecondSquared',5))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
    end
end

function info=localDecision(testCase,x,q,expected,reason)
    model=localModel(x,q);
    [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(model),model.initialState,0);
    testCase.verifyEqual(member,expected);testCase.verifyEqual(info.reason,reason);
end

function model=localModel(x,q)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('horizonSteps',8)));
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[8.5344;12.192]);
    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
        'longitudinalVelocity',x(4),'stateTime',0,'heldActuatorInput',[0;0]);
    [~,lane,roadOut]=readControllerInputs(ego,[],road,cfg);
    frame=predictiveSafetyGeometry.roadFrame(lane,roadOut);
    reference=nonlinearBicycleModel.cruise(cfg,frame(4));
    model=struct('cfg',cfg,'initialState',x,'target',q,'targetEpoch',q,'sampleIndex',0, ...
        'lane',lane,'road',roadOut,'nominalReference',reference);
end
