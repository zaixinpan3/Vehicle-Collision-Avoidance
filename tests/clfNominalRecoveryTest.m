classdef clfNominalRecoveryTest < matlab.unittest.TestCase
    % The analytic transverse quadratic acts outside active encounters.
    % Test local zero-slack derivative decrease and eventual recovery from displaced states.
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
                'controller',struct('horizonSteps',referenceSpeed+mod(referenceSpeed,2)), ...
                'solver',struct('timeLimitSeconds',5)));
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
            testCase.verifyLessThanOrEqual(metadata.clfDerivative,metadata.clfDerivativeLimit+tol);
            testCase.verifyTrue(isnan(metadata.clfActualDerivative));
            testCase.verifyLessThanOrEqual(nextValue,value+tol);
            testCase.verifyGreaterThanOrEqual(metadata.clfNextValue,0);
            testCase.verifyEqual(metadata.clfNextValueScope,"affinePredictedState");
            testCase.verifyTrue(metadata.search.clfStageCompleted);
            testCase.verifyEqual(metadata.solverCallCount,sum([metadata.search.stages.numericalSolve]));
            testCase.verifyEqual(metadata.tertiaryObjective,"none");
        end
        function moderatelyDisplacedStatesReturnToCruiseWithOneStepPerFrame(testCase,referenceSpeed,curvature)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',referenceSpeed, ...
                'controller',struct('horizonSteps',referenceSpeed+mod(referenceSpeed,2)), ...
                'solver',struct('timeLimitSeconds',5)));
            % RTI recovery fixtures remain near an admissible local reference.
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
            reference=nonlinearBicycleModel.cruise(cfg,curvature);tolerance=[.1;pi/180;.1;.05;.01];
            for initial=[-2,.1,reference.state(4),0,0;-4,.3,referenceSpeed-1,.1,.05].'
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
        function aDistantReversedSeedCanSolveWithoutArtificialHorizonGrowth(testCase)
            % Removing correction bounds lets this reference solve at the
            % requested length; terminal rows still constrain its endpoint.
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
                'controller',struct('horizonSteps',8), ...
                'solver',struct('timeLimitSeconds',5)));
            ego=struct('position',[0;0],'yaw',0,'speed',8, ...
                'lateralVelocity',0,'yawRate',0,'heldActuatorInput',[0;0]);
            [~,~,problem]=collisionAvoidanceController(ego,[],[-100,0;1000,0],cfg,[]);
            model=problem.model;model.initialState=[0;-40;2.5;6;.3;.1];
            [solution,search,model]=solvePredictiveControl(model,[],tic);
            testCase.assertNotEmpty(solution);
            testCase.verifyEqual(search.horizonExtensions,0);
            testCase.verifyNumElements(search.attempts,1);
            anchor=model.linearization;count=size(anchor.inputs,2);
            testCase.verifyEqual(count,cfg.controller.horizonSteps);
            [rows,bounds]=terminalSafeSet.rows(model.terminalContext,anchor.states(:,end),count);
            correction=solution.affineStates(:,end)-anchor.states(:,end);
            testCase.verifyLessThanOrEqual(rows*correction,bounds+cfg.solver.reducedTolerance);
        end
        function cruiseStaysOnTheReference(testCase,referenceSpeed,curvature)
            % Started on the reference, the cruise stays there: the plan's
            % later inputs, which the CLF row does not see, must not wander
            % between samples (they did at 15 m/s while the tie vanished).
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',referenceSpeed, ...
                'controller',struct('horizonSteps',referenceSpeed+mod(referenceSpeed,2)), ...
                'solver',struct('timeLimitSeconds',5)));
            road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',600));
            reference=nonlinearBicycleModel.cruise(cfg,curvature);
            [position,heading]=laneGeometry.referencePose(0,0,road.referenceCurve);
            x=[position;heading+reference.state(3);reference.state(4:6)];
            prior=[];input=reference.input;worst=zeros(5,1);slack=0;
            for frame=1:300
                ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5), ...
                    'yawRate',x(6),'heldActuatorInput',input,'stateTime',(frame-1)*cfg.controller.sampleTime);
                [command,~,problem,prior]=collisionAvoidanceController(ego,[],road,cfg,prior);
                input=command.actuatorInput;x=nonlinearBicycleModel.sample(x,input,cfg);
                if frame>100
                    worst=max(worst,abs(nonlinearBicycleModel.error(x,road,reference)));
                    slack=max(slack,problem.metadata.clfSlack);
                end
            end
            testCase.verifyLessThanOrEqual(worst,[.01;2e-3;.01;.01;2e-3]);
            testCase.verifyLessThanOrEqual(slack,cfg.solver.feasibilityTolerance);
        end
    end
end
