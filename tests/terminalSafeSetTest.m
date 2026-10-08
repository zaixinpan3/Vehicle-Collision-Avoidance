classdef terminalSafeSetTest < matlab.unittest.TestCase
    % Safe-exit terminal set of lane-hold CLF backups: invariance under a
    % backup, nested levels, state-row containment, the fixed encounter window
    % and the lane-hold backups.
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
        function theEncounterEndsAtItsExitOrAtTheFixedWindowEnd(testCase)
            % A parallel target at the same speed never leaves the range: it is
            % cleared until the encounter window ends (absorbing); the same
            % target in the ego lane ahead meets the tube; a receding target
            % ends the encounter by leaving the range.
            parallel=localModel([0;0;0;8;0;0],[0;49.9;0;8;0;0;1.6;2.4;.95;0;0]);
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(parallel),parallel.initialState,0);
            testCase.verifyTrue(member);
            testCase.verifyEqual(info.reason,"clearToWindowEnd");
            testCase.verifyEqual(info.exitSeconds,60,AbsTol=1e-9);
            ahead=localModel([0;0;0;8;0;0],[20;0;0;6;0;0;1.6;2.4;.95;0;0]);
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(ahead),ahead.initialState,0);
            testCase.verifyFalse(member);
            testCase.verifyEqual(info.reason,"tubeMeetsTarget");
            receding=localModel([0;0;0;8;0;0],[20;0;0;10;0;0;1.6;2.4;.95;0;0]);
            [member,~,info]=terminalSafeSet.member(terminalSafeSet.context(receding),receding.initialState,0);
            testCase.verifyTrue(member);
            testCase.verifyLessThan(info.exitSeconds,60);
            % The window end is fixed in time: with 1 s left the slower lead
            % ahead cannot be reached before the encounter ends, and a node
            % after the end is past the encounter.
            ahead.encounterWindowSeconds=1;
            context=terminalSafeSet.context(ahead);
            [member,context,info]=terminalSafeSet.member(context,ahead.initialState,0);
            testCase.verifyTrue(member);
            testCase.verifyEqual(info.reason,"clearToWindowEnd");
            [member,~,info]=terminalSafeSet.member(context,ahead.initialState,21);
            testCase.verifyTrue(member);
            testCase.verifyEqual(info.reason,"encounterWindowEnded");
        end
        function aLaneHoldBackupCompletesAnEncounterTheNominalOneCannot(testCase)
            % Settled in the left lane beside a slower lead in the nominal
            % lane: returning to the nominal lane meets the lead, holding the
            % left lane passes it until it leaves the range.
            lane=3.6576;x=[0;lane;0;8;0;0];lead=[15;0;0;6;0;0;1.6;2.4;.95;0;0];
            without=localModel(x,lead);
            testCase.verifyFalse(terminalSafeSet.member(terminalSafeSet.context(without),x,0));
            with=localModel(x,lead,lane*[-1,0,1,2]);
            context=terminalSafeSet.context(with);
            testCase.verifyEqual([context.modes.lateralOffset],[0,-lane,lane,2*lane],AbsTol=1e-12);
            [member,context,info]=terminalSafeSet.member(context,x,0);
            testCase.verifyTrue(member);
            testCase.verifyEqual(info.lateralOffset,lane,AbsTol=1e-12);
            testCase.verifyLessThan(info.exitSeconds,60);
            % Its backup keeps the state in its set until the lead leaves.
            reference=context.modes(info.mode).reference;previous=[0;0];
            for node=1:40
                [u,x,ok]=terminalSafeSet.terminalInput(x,previous,with,reference);
                testCase.assertTrue(ok);previous=u;
                [member,context]=terminalSafeSet.member(context,x,node,info.mode);
                testCase.verifyTrue(member);
            end
        end
        function theBackupErrorIsMeasuredFromItsLaneCentre(testCase)
            model=localModel([0;3.6576;.01;8;.1;.02],[]);
            reference=model.nominalReference;
            nominal=nonlinearBicycleModel.error(model.initialState,model.lane,reference);
            reference.lateralOffset=3.6576;
            held=nonlinearBicycleModel.error(model.initialState,model.lane,reference);
            testCase.verifyEqual(held(1),nominal(1)-3.6576,AbsTol=1e-12);
            testCase.verifyEqual(held(2:5),nominal(2:5),AbsTol=0);
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

function model=localModel(x,q,laneOffsets)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('horizonSteps',8)));
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[8.5344;12.192]);
    if nargin>2,road.laneOffsets=laneOffsets;end
    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
        'longitudinalVelocity',x(4),'stateTime',0,'heldActuatorInput',[0;0]);
    [~,lane,roadOut]=readControllerInputs(ego,[],road,cfg);
    frame=predictiveSafetyGeometry.roadFrame(lane,roadOut);
    reference=nonlinearBicycleModel.cruise(cfg,frame(4));
    model=struct('cfg',cfg,'initialState',x,'target',q,'targetEpoch',q,'sampleIndex',0, ...
        'lane',lane,'road',roadOut,'nominalReference',reference);
end
