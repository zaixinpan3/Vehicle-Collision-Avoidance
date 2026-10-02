classdef clfNominalRecoveryTest < matlab.unittest.TestCase
    % Without a target in range the CLF is the recovery cost-to-go. One frame
    % issues the recovery feedback with zero CLF slack, and the nonlinear model
    % decreases the CLF by at least the required fraction of the stage cost.
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
        function targetFreeFrameMeetsTheRecoveryClfWithZeroSlack(testCase,referenceSpeed,curvature,lateralError)
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
            terminal=nonlinearBicycleModel.recoveryTerminal(cfg,curvature);
            value=nonlinearBicycleModel.recoveryValue(x,reference.input,lane,reference,terminal,cfg);
            feedback=nonlinearBicycleModel.recoveryInput(x,reference.input,lane,reference,cfg,terminal);
            next=nonlinearBicycleModel.sample(x,command.actuatorInput,cfg);
            nextValue=nonlinearBicycleModel.recoveryValue(next,command.actuatorInput,lane,reference,terminal,cfg);
            tol=cfg.solver.feasibilityTolerance*max(1,value);
            testCase.verifyEqual(metadata.clfFunction,"recoveryCostToGo");
            testCase.verifyEqual(metadata.clfInitialValue,value,AbsTol=1e-12*max(1,value));
            testCase.verifyLessThanOrEqual(metadata.clfSlack,tol);
            testCase.verifyEqual(command.actuatorInput,feedback,AbsTol=1e-6);
            testCase.verifyLessThanOrEqual(nextValue,value-metadata.clfRequiredDecrease+tol);
            testCase.verifyTrue(metadata.recoveryRolloutConverged);
            testCase.verifyEqual(metadata.solverCallCount,3);
            testCase.verifyEqual(metadata.tertiaryObjective,"minimumAnchorDeviationAtBothOptima");
        end
    end
end
