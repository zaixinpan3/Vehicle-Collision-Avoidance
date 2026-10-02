classdef clfNominalRecoveryTest < matlab.unittest.TestCase
    % The same cost-to-go is used everywhere. A target-free optimized input
    % should reduce its nonlinear value without requiring equality to the construction feedback.
    properties (TestParameter)
        referenceSpeed={8,15};
        curvature={0,.005};
        lateralError={0,.1,-20};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function targetFreeFrameMeetsTheNominalClfWithZeroSlack(testCase,referenceSpeed,curvature,lateralError)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',referenceSpeed, ...
                'controller',struct('horizonSteps',referenceSpeed+mod(referenceSpeed,2))));
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
            reference=nonlinearBicycleModel.cruise(cfg,curvature);
            [position,heading]=laneGeometry.referencePose(0,lateralError,road.referenceCurve);
            x=[position;heading+reference.state(3)+.2*(lateralError<0);reference.state(4:6)];
            ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5), ...
                'yawRate',x(6),'heldActuatorInput',reference.input);
            [command,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            metadata=problem.metadata;lane=problem.model.lane;
            terminal=nonlinearBicycleModel.nominalTail(cfg,curvature);
            value=nonlinearBicycleModel.nominalValue(x,reference.input,lane,reference,terminal,cfg);
            next=nonlinearBicycleModel.sample(x,command.actuatorInput,cfg);
            nextValue=nonlinearBicycleModel.nominalValue(next,command.actuatorInput,lane,reference,terminal,cfg);
            tol=cfg.solver.feasibilityTolerance*max(1,value);
            testCase.verifyEqual(metadata.clfFunction,"nominalCostToGo");
            testCase.verifyEqual(metadata.clfInitialValue,value,AbsTol=1e-12*max(1,value));
            testCase.verifyLessThanOrEqual(metadata.clfSlack,tol);
            testCase.verifyLessThanOrEqual(nextValue,value-.5*metadata.clfRequiredDecrease+tol);
            testCase.verifyTrue(metadata.clfTailReached);
            testCase.verifyGreaterThanOrEqual(metadata.clfNextValue,0);
            testCase.verifyTrue(metadata.search.clfStageCompleted);
            testCase.verifyEqual(metadata.solverCallCount,sum([metadata.search.stages.numericalSolve]));
            testCase.verifyEqual(metadata.tertiaryObjective,"none");
        end
    end
end
