classdef clfNominalRecoveryTest < matlab.unittest.TestCase
    % The same analytic transverse quadratic is used throughout an encounter.
    % Test local zero-slack decrease and eventual recovery from displaced states.
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
            value=sum((reference.factor*nonlinearBicycleModel.error(x,lane,reference)).^2);
            next=nonlinearBicycleModel.sample(x,command.actuatorInput,cfg);
            nextValue=sum((reference.factor*nonlinearBicycleModel.error(next,lane,reference)).^2);
            tol=cfg.solver.feasibilityTolerance*max(1,value);
            testCase.verifyEqual(metadata.clfFunction,"quadraticTransverseError");
            testCase.verifyEqual(metadata.clfInitialValue,value,AbsTol=1e-12*max(1,value));
            testCase.verifyLessThanOrEqual(metadata.clfSlack,tol);
            testCase.verifyLessThanOrEqual(nextValue,value-.5*metadata.clfRequiredDecrease+tol);
            testCase.verifyGreaterThanOrEqual(metadata.clfNextValue,0);
            testCase.verifyTrue(metadata.search.clfStageCompleted);
            testCase.verifyEqual(metadata.solverCallCount,sum([metadata.search.stages.numericalSolve]));
            testCase.verifyEqual(metadata.tertiaryObjective,"none");
        end
        function displacedAndReversedStatesEventuallyReturnToCruise(testCase,referenceSpeed,curvature)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',referenceSpeed, ...
                'controller',struct('horizonSteps',referenceSpeed+mod(referenceSpeed,2))));
            % These CLF fixtures start 20 or 40 m away; no road clearance is declared.
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
            reference=nonlinearBicycleModel.cruise(cfg,curvature);tolerance=[.1;pi/180;.1;.05;.01];
            for initial=[-20,.2,reference.state(4),0,0;-40,2.5,6,.3,.1].'
                x=[0;initial(1);reference.state(3)+initial(2);initial(3:5)];
                prior=[];input=reference.input;inside=0;initialValue=nonlinearBicycleModel.nominalValue(x,road,reference);
                for frame=1:1500
                    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5), ...
                        'yawRate',x(6),'heldActuatorInput',input,'stateTime',(frame-1)*cfg.controller.sampleTime);
                    [command,~,problem,prior]=collisionAvoidanceController(ego,[],road,cfg,prior);
                    testCase.assertTrue(problem.metadata.search.clfStageCompleted);
                    input=command.actuatorInput;x=nonlinearBicycleModel.sample(x,input,cfg);
                    if all(abs(nonlinearBicycleModel.error(x,road,reference))<=tolerance),inside=inside+1;else,inside=0;end
                    if inside*cfg.controller.sampleTime>=1,break;end
                end
                testCase.verifyGreaterThanOrEqual(inside*cfg.controller.sampleTime,1);
                testCase.verifyLessThan(nonlinearBicycleModel.nominalValue(x,road,reference),initialValue);
            end
        end
    end
end
