classdef terminalContinuationTest < matlab.unittest.TestCase
    %terminalContinuationTest Endpoint closure and retained-witness behavior.
    properties (TestParameter)
        curvature = {0,-.005,.005}
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
        function endpointControlsReachTheNextAbsoluteSlice(testCase,curvature)
            [seed,cfg]=localSeed(curvature);
            [values,inputs,slew]=localClosureSamples(seed,cfg);
            testCase.verifyLessThanOrEqual(values,zeros(size(values)));
            testCase.verifyLessThan(abs(inputs(1,:)),cfg.model.frontWheelSteeringAngleMaximum);
            testCase.verifyLessThan(abs(inputs(2,:)),1);
            testCase.verifyLessThan(slew,.5*cfg.controller.sampleTime);
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
        function positiveSlackIsExecutedAndItsShiftedSumDoesNotIncrease(testCase)
            [ego,road,cfg]=localFixture();cfg.solver.timeLimitSeconds=1e-12;
            target=struct('targetPositionInertial',[0;0],'targetVelocityInertial',[-8;0],'targetYawInertial',pi);
            [command,~,problem,first]=collisionAvoidanceController(ego,target,road,cfg,[]);
            [~,hard,cost]=localContinue(first,ego,road,cfg,12);
            testCase.verifyEqual(command.actuatorInput,first.witness.inputs(:,1));
            testCase.verifyGreaterThan(problem.solution.safety,0);
            testCase.verifyFalse(problem.metadata.zeroSlack);
            testCase.verifyEqual(hard,zeros(size(hard)));
            testCase.verifyLessThanOrEqual(cost(1),sum(first.witness.stageSlacks(2:end)));
            testCase.verifyLessThanOrEqual(diff(cost),zeros(1,numel(cost)-1));
            testCase.verifyEqual(cost(end),0);
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
