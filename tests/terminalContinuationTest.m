classdef terminalContinuationTest < matlab.unittest.TestCase
    %terminalContinuationTest Endpoint geometry and controller context contracts.
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
            testCase.verifyLessThan(abs(inputs(2,:)),1);
            testCase.verifyLessThan(slew(2,:),.5*cfg.controller.sampleTime);
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
            qNext=predictiveSafetyGeometry.predictTarget(q,10*cfg.controller.sampleTime);
            later=terminalContinuation.separation(seed,qNext,frame,cfg,seed.epochIndex+10);
            before=terminalContinuation.referenceAt(seed,seed.epochIndex);
            after=terminalContinuation.referenceAt(seed,seed.epochIndex+10);
            expected=initial+t.'*(after(1:2)-before(1:2)-qNext(1:2)+q(1:2));
            testCase.verifyEqual(later,expected,AbsTol=1e-11);
        end
        function departureFollowsTheEndpointPolicyUntilTheTargetIsOutOfRange(testCase)
            [seed,cfg]=localSeed(0);t=[cos(.2);sin(.2)];n=[-t(2);t(1)];
            index=seed.epochIndex;time=index*cfg.controller.sampleTime;origin=seed.epochState(1:2);
            receding=predictiveSafetyGeometry.predictTarget([origin-15*t;.2+pi;8;0;0;1.6;2.4;.95;0;0],-time);
            blocking=predictiveSafetyGeometry.predictTarget([origin+20*t;.2;0;0;0;1.6;2.4;.95;0;0],-time);
            alongside=predictiveSafetyGeometry.predictTarget([origin+5*n;.2;8;0;0;1.6;2.4;.95;0;0],-time);
            [margin,value]=terminalContinuation.departure(seed,receding,index,cfg);
            geometricMargin=15-(cfg.vehicle.length/2+2.4)-cfg.collision.safetyMarginMeters;
            % The invariant tube adds a small reserve to the requested buffer.
            testCase.verifyGreaterThan(margin,geometricMargin-.01);
            testCase.verifyLessThan(margin,geometricMargin);
            testCase.verifyEqual(min(value),margin,AbsTol=1e-12);
            testCase.verifyLessThan(terminalContinuation.departure(seed,blocking,index,cfg),0);
            testCase.verifyEqual(terminalContinuation.departure(seed,alongside,index,cfg),-Inf);
            cfg.collision.encounterRangeMeters=Inf;
            testCase.verifyEqual(terminalContinuation.departure(seed,receding,index,cfg),-Inf);
        end
        function inputMemoryIsPartOfMembership(testCase)
            [seed,~]=localSeed(0);y=terminalContinuation.referenceAt(seed,seed.epochIndex);
            y(7)=y(7)+.1;
            testCase.verifyGreaterThan(terminalContinuation.membership(y,seed.epochIndex,seed),0);
            testCase.verifyError(@()terminalContinuation.control(y,seed.epochIndex,seed), ...
                'collisionAvoidanceController:outsideContinuation');
        end
        function aDeadlineCannotAuthorizeAnUnoptimizedInitialization(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=1e-12;
            target=[];
            testCase.verifyError(@()collisionAvoidanceController(ego,target,road,cfg,[]), ...
                'collisionAvoidanceController:noOptimizationSolution');
        end
        function aChangedConstraintSetRequiresReinitialization(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=5;
            [~,~,~,first]=collisionAvoidanceController(ego,[],road,cfg,[]);
            ego=localSuccessor(ego,first);cfg.model.speedMaximum=17;
            testCase.verifyError(@()collisionAvoidanceController(ego,[],road,cfg,first), ...
                'collisionAvoidanceController:changedContinuationProblem');
        end
        function changingTheGivenPathStillRequiresReinitialization(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=5;
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
    end
end

function [seed,cfg]=localSeed(curvature)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'model',struct('brakingRatioRateMaximum',.5)));
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
function ego=localSuccessor(ego,state)
    x=state.stateTrajectory(:,2);ego.position=x(1:2);ego.yaw=x(3);ego.speed=x(4);
    ego.lateralVelocity=x(5);ego.yawRate=x(6);ego.heldActuatorInput=state.appliedInput;
end
