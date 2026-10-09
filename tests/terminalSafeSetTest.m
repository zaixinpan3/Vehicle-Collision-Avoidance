classdef terminalSafeSetTest < matlab.unittest.TestCase
    % CLF-tube terminal set: invariance under the terminal controller, nested
    % levels, state-row containment and the end of an encounter (exit or a
    % relative motion outside the collision cone).
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function theSetIsInvariantUnderTheTerminalController(testCase)
            % A receding target behind the ego: an offset start is in the set,
            % and every hold of the terminal controller stays in it.
            q=[-12;3.66;0;6;0;0;1.6;2.4;.95;0;0];
            model=localModel([0;1;.05;8;0;0],q);
            context=terminalSafeSet.context(model);
            x=model.initialState;previous=[0;0];
            [member,context]=terminalSafeSet.member(context,x,0);
            testCase.verifyTrue(member);
            for node=1:60
                [u,next,ok,value]=terminalSafeSet.terminalInput(x,previous,model);
                testCase.verifyTrue(ok);
                testCase.verifyLessThanOrEqual(value, ...
                    model.nominalReference.contraction*nonlinearBicycleModel.nominalValue(x,model.lane,model.nominalReference)*(1+1e-9)+1e-12);
                [member,context]=terminalSafeSet.member(context,next,node);
                testCase.verifyTrue(member);
                x=next;previous=u;
            end
        end
        function clearanceIsMonotoneInTheLevel(testCase)
            % Tubes are nested in the level, so the bisected level separates
            % clear levels from blocked ones.
            q=[30;3.2;pi;8;0;0;1.6;2.4;.95;0;0];
            model=localModel([0;0;0;8;0;0],q);
            context=terminalSafeSet.context(model);
            [level,context]=terminalSafeSet.level(context,model.initialState,0);
            testCase.verifyGreaterThan(level,0);
            testCase.verifyLessThan(level,context.levelMaximum);
            [value,station]=terminalSafeSet.coordinates(context,model.initialState);
            testCase.verifyEqual(value,0,AbsTol=1e-20);
            for fraction=[0,.25,.5,1]
                testCase.verifyTrue(terminalSafeSet.clear(context,fraction*level,station,0));
            end
            testCase.verifyFalse(terminalSafeSet.clear(context,min(context.levelMaximum,1.05*level+1e-3),station,0));
        end
        function theLevelEllipsoidLiesInsideTheStateRows(testCase)
            model=localModel([0;0;0;8;0;0],[]);
            reference=model.nominalReference;cfg=model.cfg;
            level=terminalSafeSet.stateLevel(reference,cfg);
            testCase.verifyGreaterThan(level,0);
            factor=chol(reference.matrix);
            rng(7);directions=randn(5,2000);
            tire=modifiedFialaTire.parameters(cfg);
            k=3*tire.longitudinalForceScale(2)/tire.corneringStiffness(2);
            braking=min(1,abs(reference.input(2))+cfg.clf.certificationBrakingRatio);
            for j=1:size(directions,2)
                e=factor\(directions(:,j)/norm(directions(:,j)))*sqrt(level)*(1-1e-9);
                x=reference.state+[0;0;0;e(3:5)];
                testCase.verifyTrue(terminalSafeSet.stateRows(x,cfg));
                testCase.verifyLessThanOrEqual(abs(x(5)),tan(cfg.model.sideslipMaximum)*x(4)*(1+1e-9));
                testCase.verifyLessThanOrEqual(abs(x(5)-cfg.vehicle.lr*x(6)),k*sqrt(1-braking^2)*x(4)*(1+1e-9));
            end
        end
        function withoutATargetOnlyTheLevelAndTheRoadCount(testCase)
            model=localModel([0;.5;0;8;0;0],[]);
            context=terminalSafeSet.context(model);
            testCase.verifyTrue(terminalSafeSet.member(context,model.initialState,0));
            far=model.initialState;far(2)=6;
            [member,~,info]=terminalSafeSet.member(context,far,0);
            testCase.verifyFalse(member);
            testCase.verifyEqual(info.reason,"levelAboveCertifiedRegion");
            narrow=model;narrow.road.lateralClearance=[1;1];
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(narrow),narrow.initialState,0);
            testCase.verifyFalse(member);
            testCase.verifyEqual(info.reason,"tubeLeavesRoad");
        end
        function theEncounterEndsAtItsExitOrOutsideTheCollisionCone(testCase)
            % A parallel target at the same speed never leaves the range, but
            % its relative motion is outside the collision cone: the encounter
            % ends at once.
            parallel=localModel([0;0;0;8;0;0],[0;49.9;0;8;0;0;1.6;2.4;.95;0;0]);
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(parallel),parallel.initialState,0);
            testCase.verifyTrue(member);
            testCase.verifyEqual(info.reason,"outsideCollisionCone");
            % A slower lead in the ego lane meets the tube.
            ahead=localModel([0;0;0;8;0;0],[20;0;0;6;0;0;1.6;2.4;.95;0;0]);
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(ahead),ahead.initialState,0);
            testCase.verifyFalse(member);
            testCase.verifyEqual(info.reason,"tubeMeetsTarget");
            % A faster lead never comes closer.
            receding=localModel([0;0;0;8;0;0],[20;0;0;10;0;0;1.6;2.4;.95;0;0]);
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(receding),receding.initialState,0);
            testCase.verifyTrue(member);
            testCase.verifyEqual(info.reason,"outsideCollisionCone");
            % A slightly slower lead far ahead closes only after the computed
            % horizon; with neither event within it the state is not terminal.
            slow=localModel([0;0;0;8;0;0],[40;0;0;7.5;0;0;1.6;2.4;.95;0;0]);
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(slow),slow.initialState,0);
            testCase.verifyFalse(member);
            testCase.verifyEqual(info.reason,"noExitOrSeparation");
            % A braking lead's first estimate (6.2 m ahead, 0.1 m/s slower,
            % heading 3.8 mrad, as measured): in the ego lane it meets the
            % tube. In the lane to the right, drifting further right, it is
            % outside the cone at once; drifting left, toward the ego, it
            % closes along and across the road and is inside it.
            lane=3.6576;
            inLane=localModel([0;0;0;8;0;0],[6.2;0;.0038;7.9;0;0;1.6;2.4;.95;0;0]);
            testCase.verifyFalse(terminalSafeSet.member(terminalSafeSet.context(inLane),inLane.initialState,0));
            away=localModel([0;0;0;8;0;0],[6.2;-lane;-.0038;7.9;0;0;1.6;2.4;.95;0;0]);
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(away),away.initialState,0);
            testCase.verifyTrue(member);
            testCase.verifyEqual(info.reason,"outsideCollisionCone");
            testCase.verifyEqual(info.exitSeconds,0);
            toward=localModel([0;0;0;8;0;0],[6.2;-lane;.0038;7.9;0;0;1.6;2.4;.95;0;0]);
            testCase.verifyFalse(terminalSafeSet.member(terminalSafeSet.context(toward),toward.initialState,0));
            % A slow diagonal crosser: 20 m ahead on the right, closing at
            % 0.1 m/s along the road and 0.12 m/s across it. Its box crosses
            % the ego band between 32 s and 68 s and reaches the ego box's
            % station only after 150 s: outside the cone now, although no
            % single gap is monotone and the crossing ends after the bound.
            crosser=localModel([0;0;0;8;0;0],[20;-6;atan2(.12,7.9);hypot(7.9,.12);0;0;1.6;2.4;.95;0;0]);
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(crosser),crosser.initialState,0);
            testCase.verifyTrue(member);
            testCase.verifyEqual(info.reason,"outsideCollisionCone");
            testCase.verifyEqual(info.exitSeconds,0);
            % Closing across the road a little faster meets the box: inside.
            hitter=localModel([0;0;0;8;0;0],[20;-6;atan2(.05,7.9);hypot(7.9,.05);0;0;1.6;2.4;.95;0;0]);
            testCase.verifyFalse(terminalSafeSet.member(terminalSafeSet.context(hitter),hitter.initialState,0));
            % A target circling beside the road stays in its disk, clear of the
            % ego band.
            circling=localModel([0;0;0;8;0;0],[0;40;0;5;0;.05;1.6;2.4;.95;0;0]);
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(circling),circling.initialState,0);
            testCase.verifyTrue(member);
            testCase.verifyEqual(info.reason,"outsideCollisionCone");
        end
        function theGridMustContainTheHoldMidpoints(testCase)
            model=localModel([0;0;0;8;0;0],[]);
            model.cfg.terminal.timeStepSeconds=.01;
            testCase.verifyError(@()terminalSafeSet.context(model),'collisionAvoidanceController:invalidTerminalGrid');
        end
        function holdStatesMatchTheLinearizedHold(testCase)
            model=localModel([0;.3;.02;8;.1;.05],[]);
            x=model.initialState;u=[.02;.05];
            [middle,next]=terminalSafeSet.holdStates(x,u,model.cfg);
            [m,~,~,n]=nonlinearBicycleModel.hold(x,u,model.cfg);
            testCase.verifyEqual(middle,m,AbsTol=0);
            testCase.verifyEqual(next,n,AbsTol=0);
        end
    end
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
