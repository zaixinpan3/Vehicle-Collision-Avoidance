classdef terminalContinuationTest < matlab.unittest.TestCase
    %terminalContinuationTest Endpoint closure and retained-witness behavior.
    properties (TestParameter)
        curvature = {0,-.005,.005}
        phaseMeters = {-20,0,20}
    end
    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function nonlinearContractionIncludesTheReferenceDefect(testCase,curvature)
            [seed,~]=localSeed(curvature);
            testCase.verifyGreaterThan(seed.radius,0);
            testCase.verifyLessThan(seed.contractionBound*seed.radius+seed.defectBound,seed.radius);
            testCase.verifySize(seed.factor,[8,8]);
        end
        function endpointControlsReachTheNextAbsoluteSlice(testCase,curvature,phaseMeters)
            [seed,cfg]=localSeed(curvature);
            seed.phaseMeters=phaseMeters;
            [values,inputs,slew]=localClosureSamples(seed,cfg);
            testCase.verifyLessThanOrEqual(values,zeros(size(values)));
            testCase.verifyLessThan(abs(inputs(1,:)),cfg.model.frontWheelSteeringAngleMaximum);
            testCase.verifyLessThan(abs(inputs(2,:)),1);
            testCase.verifyLessThan(slew,.5*cfg.controller.sampleTime);
        end
        function shiftedReferenceStatesBelongToTheirOwnTerminalFamily(testCase,curvature)
            [seed,~]=localSeed(curvature);shifted=seed;shifted.phaseMeters=-20;
            y=terminalContinuation.referenceAt(shifted,seed.epochIndex);
            testCase.verifyGreaterThan(terminalContinuation.membership(y,seed.epochIndex,seed),0);
            testCase.verifyLessThan(terminalContinuation.membership(y,seed.epochIndex,shifted),0);
            testCase.verifyEqual(terminalContinuation.control(y,seed.epochIndex,shifted),seed.reference.input,AbsTol=1e-12);
        end
        function phaseTangentsMatchTheReferenceAndMembership(testCase,curvature)
            [seed,~]=localSeed(curvature);seed.phaseMeters=-12.3;index=seed.epochIndex+17;
            [y,tangent]=terminalContinuation.referenceAt(seed,index);
            y=y+[.01;-.02;.001;.03;-.01;.002;0;0];
            [~,~,~,derivative]=terminalContinuation.membership(y,index,seed);
            lo=seed;hi=seed;h=1e-4;lo.phaseMeters=lo.phaseMeters-h;hi.phaseMeters=hi.phaseMeters+h;
            finite=(terminalContinuation.referenceAt(hi,index)-terminalContinuation.referenceAt(lo,index))/(2*h);
            [~,~,lowError]=terminalContinuation.membership(y,index,lo);
            [~,~,highError]=terminalContinuation.membership(y,index,hi);
            testCase.verifyEqual(tangent,finite,AbsTol=2e-8);
            testCase.verifyEqual(derivative,(highError-lowError)/(2*h),AbsTol=2e-8);
        end
        function aDifferentPhaseMustRecheckFutureTargetSeparation(testCase)
            [seed,cfg]=localSeed(0);t=[cos(.2);sin(.2)];frame=[0;0;.2;0];
            q=[seed.epochState(1:2)+20*t;.2;0;0;0;1.6;2.4;.95;0;0];
            unsafe=terminalContinuation.separation(seed,q,frame,cfg);
            seed.phaseMeters=40;
            [safe,derivative]=terminalContinuation.separation(seed,q,frame,cfg);
            testCase.verifyLessThan(unsafe,0);
            testCase.verifyGreaterThan(safe,0);
            testCase.verifyEqual(derivative,1,AbsTol=1e-12);
        end
        function separationUsesTheSameAbsoluteTimeAsTheEndpoint(testCase)
            [seed,cfg]=localSeed(0);seed.phaseMeters=40;t=[cos(.2);sin(.2)];frame=[0;0;.2;0];
            q=[seed.epochState(1:2)+20*t;.2;4;0;0;1.6;2.4;.95;0;0];
            initial=terminalContinuation.separation(seed,q,frame,cfg);
            qNext=predictiveSafetyGeometry.targetFlow(q,10*cfg.controller.sampleTime);
            later=terminalContinuation.separation(seed,qNext,frame,cfg,seed.epochIndex+10);
            before=terminalContinuation.referenceAt(seed,seed.epochIndex);
            after=terminalContinuation.referenceAt(seed,seed.epochIndex+10);
            expected=initial+t.'*(after(1:2)-before(1:2)-qNext(1:2)+q(1:2));
            testCase.verifyEqual(later,expected,AbsTol=1e-11);
        end
        function aLongitudinalDelayCanTranslateTheTerminalCore(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=10;
            [~,~,~,first]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,first);ego.position(1)=ego.position(1)-5;
            [~,~,problem,delayed]=collisionAvoidanceController(ego,[],road,cfg,first);
            cfg.solver.timeLimitSeconds=1e-12;
            [last,hard,~,counts]=localContinue(delayed,ego,road,cfg,24);
            testCase.verifyLessThan(delayed.terminal.epochState(1)-first.terminal.epochState(1),-4);
            testCase.verifyEqual(problem.solution.hard,0,AbsTol=0);
            testCase.verifyEqual(last.terminal.epochState,delayed.terminal.epochState,AbsTol=1e-12);
            testCase.verifyEqual(hard,zeros(size(hard)),AbsTol=0);
            testCase.verifyEqual(counts,zeros(size(counts)));
        end
        function inputMemoryIsPartOfMembership(testCase)
            [seed,~]=localSeed(0);y=terminalContinuation.referenceAt(seed,seed.epochIndex);
            y(7)=y(7)+.1;
            testCase.verifyGreaterThan(terminalContinuation.membership(y,seed.epochIndex,seed),0);
            testCase.verifyError(@()terminalContinuation.control(y,seed.epochIndex,seed), ...
                'collisionAvoidanceController:outsideContinuation');
        end
        function retainedWitnessContinuesPastItsOriginalEndpoint(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=1e-12;
            [~,~,~,first]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [last,hard,cost,counts,sources]=localContinue(first,ego,road,cfg,24);
            testCase.verifyGreaterThan(last.sampleIndex,first.terminal.epochIndex);
            testCase.verifyEqual(last.terminal.epochIndex,first.terminal.epochIndex);
            testCase.verifyEqual(hard,zeros(size(hard)));
            testCase.verifyEqual(cost,zeros(size(cost)));
            testCase.verifyEqual(counts,zeros(size(counts)));
            testCase.verifyEqual(sources,repmat("retainedContinuation",size(sources)));
        end
        function aDeadlineCannotAuthorizeAPositiveSlackInitialization(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=1e-12;
            target=struct('targetPositionInertial',[0;0],'targetVelocityInertial',[-8;0],'targetYawInertial',pi);
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:noFeasibleContinuation');
        end
        function aChangedInputMemoryCannotUseTheShiftedWitness(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=1e-12;
            [~,~,~,first]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,first);ego.heldActuatorInput=first.appliedInput+[.01;0];
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,first);
            testCase.verifyFalse(problem.metadata.search.shiftAvailable);
            testCase.verifyEqual(problem.metadata.search.initialization,"restorationFromChangedEgoState");
        end
        function aChangedConstraintSetRequiresReinitialization(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=1e-12;
            [~,~,~,first]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,first);cfg.model.speedMaximum=17;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,first), ...
                'collisionAvoidanceController:changedContinuationProblem');
        end
        function changingRoadWidthPreservesTheRetainedWitness(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=1e-12;
            [~,~,~,first]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,first);road.lateralClearance=[.01;.02];
            [command,~,problem]=collisionAvoidanceController(ego,[],road,cfg,first);
            testCase.verifyEqual(command.actuatorInput,first.inputTrajectory(:,2),AbsTol=1e-14);
            testCase.verifyTrue(problem.metadata.search.shiftAvailable);
            testCase.verifyEqual(problem.metadata.controlSource,"retainedContinuation");
            testCase.verifyEqual(problem.solution.hard,0,AbsTol=0);
            testCase.verifyFalse(problem.metadata.roadConstraintsEnforced);
        end
        function changingTheGivenPathStillRequiresReinitialization(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=1e-12;
            [~,~,~,first]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,first);road.centerline(:,2)=1;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,first), ...
                'collisionAvoidanceController:changedContinuationProblem');
        end
        function targetSeparationDoesNotDependOnRoadWidth(testCase,curvature)
            [seed,cfg]=localSeed(curvature);
            q=[1000;1000;0;0;0;0;1.6;2.4;.95;0;0];
            wide=terminalContinuation.separation(seed,q,[0;0;.2;curvature;4;4],cfg);
            narrow=terminalContinuation.separation(seed,q,[0;0;.2;curvature;.01;.02],cfg);
            testCase.verifyGreaterThan(narrow,0);
            testCase.verifyEqual(narrow,wide,AbsTol=1e-12);
        end
        function noResultIsIssuedForAnInfeasibleHardCompletion(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=1e-12;
            target=struct('targetPositionInertial',[12;0],'targetVelocityInertial',[-8;0],'targetYawInertial',pi);
            cfg.controller.maximumHorizonSteps=128;
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:noFeasibleContinuation');
        end
    end
end

function [seed,cfg]=localSeed(curvature)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'model',struct('frontWheelSteeringRateMaximum',.5,'brakingRatioRateMaximum',.5)));
    seed=terminalContinuation.build(cfg,curvature);
    lane=struct('referenceCurve',struct('origin',[0;0],'heading',.2,'curvature',curvature,'length',200));
    seed=terminalContinuation.anchor(seed,seed.reference.state,11,lane);
end
function [values,inputs,slew]=localClosureSamples(seed,cfg)
    directions=[eye(8),-eye(8)];values=zeros(1,size(directions,2));inputs=zeros(2,size(directions,2));slew=inputs;
    for j=1:size(directions,2)
        index=11+101*j;reference=terminalContinuation.referenceAt(seed,index);
        rotation=[cos(reference(3)),-sin(reference(3));sin(reference(3)),cos(reference(3))];
        y=reference+blkdiag(rotation,eye(6))*(seed.factor\(directions(:,j)*seed.radius*.999));
        u=terminalContinuation.control(y,index,seed);inputs(:,j)=u;slew(:,j)=abs(u-y(7:8));
        next=[nonlinearBicycleModel.sample(y(1:6),u,cfg);u];
        values(j)=terminalContinuation.membership(next,index+1,seed);
    end
end
function [ego,road,cfg]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('horizonSteps',8,'maximumHorizonSteps',16), ...
        'nonlinear',struct('recoveryHorizonSeconds',.05)));
    ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
    road=struct('centerline',[-100,0;1000,0],'lateralClearance',[4;4]);
end
function [state,hard,cost,counts,sources]=localContinue(state,ego,road,cfg,count)
    hard=zeros(1,count);cost=hard;counts=hard;sources=strings(1,count);
    for j=1:count
        ego=localSuccessor(ego,state);
        [~,~,problem,state]=collisionAvoidanceController(ego,[],road,cfg,state);
        hard(j)=problem.solution.hard;cost(j)=problem.solution.safety;
        counts(j)=problem.metadata.solverCallCount;sources(j)=problem.metadata.controlSource;
    end
end
function ego=localSuccessor(ego,state)
    x=state.stateTrajectory(:,2);ego.position=x(1:2);ego.yaw=x(3);ego.speed=x(4);
    ego.lateralVelocity=x(5);ego.yawRate=x(6);ego.heldActuatorInput=state.appliedInput;
end
