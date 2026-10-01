classdef clfNominalRecoveryTest < matlab.unittest.TestCase
    % One-step CLF dissipation in the affine model; no nonlinear stability claim.
    properties (TestParameter)
        referenceSpeed={8,15};
        curvature={0,.005};
        lateralError={0,.1};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function secondStageReachesZeroSlackForAnUnobstructedClf(testCase,referenceSpeed,curvature,lateralError)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',referenceSpeed, ...
                'controller',struct('horizonSteps',referenceSpeed+mod(referenceSpeed,2))));
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
            reference=nonlinearBicycleModel.cruise(cfg,curvature);x=reference.state;x(2)=lateralError;
            ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5), ...
                'yawRate',x(6),'heldActuatorInput',reference.input);
            [~,~,problem]=collisionAvoidanceController(ego,[],road,cfg,[]);
            anchor=problem.model.linearization;
            [error,jacobian]=nonlinearBicycleModel.errorLinearization(anchor.states(:,2),problem.model.lane,reference);
            error=error+jacobian*(problem.predictedState(:,2)-anchor.states(:,2));
            nextValue=norm(reference.factor*error)^2;
            tol=cfg.solver.feasibilityTolerance*max(1,problem.solution.clfInitialValue);
            testCase.verifyEqual(problem.solution.clfNextValue,nextValue,AbsTol=1e-8);
            testCase.verifyLessThanOrEqual(problem.solution.clfSlack,tol);
            testCase.verifyLessThanOrEqual(nextValue,(1-cfg.nonlinear.clfDecay)*problem.solution.clfInitialValue+tol);
            testCase.verifyLessThanOrEqual(problem.solution.safety,problem.metadata.search.slackCap+tol);
            testCase.verifyTrue(problem.metadata.search.clfStageCompleted);
            testCase.verifyEqual(problem.metadata.solverCallCount,2);
        end
    end
end
