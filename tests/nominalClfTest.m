classdef nominalClfTest < matlab.unittest.TestCase
    % Finite nominal policy evaluation with a strict local Lyapunov tail.
    % These sampled behaviors do not certify the whole nonlinear domain.
    properties (TestParameter)
        referenceSpeed={8,15};
        curvature={0,.005};
        coordinate={1,2,3,4,5};
        direction={-1,1};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function trimIsAnExactEquilibriumOfTheFeedback(testCase,referenceSpeed,curvature)
            [cfg,lane,reference,terminal]=localSetup(referenceSpeed,curvature);
            trim=[0;0;reference.state(3:6)];
            u=nonlinearBicycleModel.nominalFeedback(trim,reference.input,lane,reference,cfg,terminal);
            testCase.verifyEqual(u,reference.input,AbsTol=1e-12);
            testCase.verifyEqual(nonlinearBicycleModel.nominalValue(trim,u,lane,reference,terminal,cfg),0,AbsTol=1e-20);
            testCase.verifyLessThan(terminal.spectralRadius,1);
            A=terminal.closedLoop;P=terminal.matrix;
            testCase.verifyLessThanOrEqual(norm(A.'*P*A-P+2*terminal.stageFactor^2),1e-8*norm(P));
        end
        function nominalValueDecreasesByAtLeastTheStageCost(testCase,referenceSpeed,curvature)
            [cfg,lane,reference,terminal]=localSetup(referenceSpeed,curvature);
            for state=localStates(lane,reference)
                x=state;u=nonlinearBicycleModel.nominalFeedback(x,reference.input,lane,reference,cfg,terminal);
                [value,~,~,converged]=nonlinearBicycleModel.nominalValue(x,reference.input,lane,reference,terminal,cfg);
                stage=sum((terminal.stageFactor*nonlinearBicycleModel.error(x,lane,reference)).^2);
                next=nonlinearBicycleModel.nominalValue(nonlinearBicycleModel.sample(x,u,cfg),u,lane,reference,terminal,cfg);
                testCase.verifyTrue(converged);
                testCase.verifyLessThanOrEqual(next,value-stage+1e-8*max(1,value));
            end
        end
        function strictTailDecreasesForNonlinearCorePerturbations(testCase,referenceSpeed,curvature,coordinate,direction)
            [cfg,lane,reference,terminal]=localSetup(referenceSpeed,curvature);
            e=zeros(5,1);e(coordinate)=direction*sqrt(cfg.nominalClf.tailLevel/terminal.matrix(coordinate,coordinate));
            x=[0;e(1);reference.state(3)+e(2);reference.state(4:6)+e(3:5)];
            u=nonlinearBicycleModel.nominalFeedback(x,reference.input,lane,reference,cfg,terminal);
            next=nonlinearBicycleModel.error(nonlinearBicycleModel.sample(x,u,cfg),lane,reference);
            testCase.verifyLessThanOrEqual(next.'*terminal.matrix*next-e.'*terminal.matrix*e, ...
                -sum((terminal.stageFactor*e).^2)+1e-12);
        end
        function fixedEvaluationHorizonDoesNotSwitchNearTheTrim(testCase)
            [cfg,lane,reference,terminal]=localSetup(8,0);
            trim=[0;0;reference.state(3:6)];away=trim;away(2)=20;
            [zero,~,nearSteps]=nonlinearBicycleModel.nominalValue(trim,reference.input,lane,reference,terminal,cfg);
            [positive,~,farSteps]=nonlinearBicycleModel.nominalValue(away,reference.input,lane,reference,terminal,cfg);
            testCase.verifyEqual(nearSteps,farSteps);
            testCase.verifyEqual(nearSteps,round(cfg.nominalClf.evaluationSeconds/cfg.controller.sampleTime));
            testCase.verifyLessThan(zero,1e-16);
            testCase.verifyGreaterThan(positive,1);
        end
        function finiteBrakingSlewMakesTheValueDependOnInputMemory(testCase)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'model',struct('brakingRatioRateMaximum',.25)));
            reference=nonlinearBicycleModel.cruise(cfg,0);terminal=nonlinearBicycleModel.nominalTail(cfg,0);
            lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',0,'length',200));
            trim=[0;0;reference.state(3:6)];
            nominal=nonlinearBicycleModel.nominalValue(trim,reference.input,lane,reference,terminal,cfg);
            different=nonlinearBicycleModel.nominalValue(trim,[0;-.3],lane,reference,terminal,cfg);
            testCase.verifyLessThan(nominal,1e-16);
            testCase.verifyGreaterThan(different,1);
        end
        function longitudinalPathPhaseDoesNotChangeTheNominalValue(testCase,referenceSpeed,curvature)
            [cfg,lane,reference,terminal]=localSetup(referenceSpeed,curvature);
            values=zeros(1,3);stations=[0,47,-23];
            for index=1:numel(stations)
                [position,heading]=laneGeometry.referencePose(stations(index),.8,lane.referenceCurve);
                x=[position;heading+reference.state(3)+.2;reference.state(4:6)+[-.7;.1;.05]];
                values(index)=nonlinearBicycleModel.nominalValue(x,reference.input,lane,reference,terminal,cfg);
            end
            testCase.verifyEqual(values,repmat(values(1),size(values)),RelTol=1e-9);
        end
        function feedbackReturnsFromDistantAndReversedStates(testCase,referenceSpeed,curvature)
            [cfg,lane,reference,terminal]=localSetup(referenceSpeed,curvature);
            tolerance=[.1;pi/180;.1;.05;.01];
            for state=localStates(lane,reference)
                x=state;u=reference.input;inside=0;
                for step=1:round(60/cfg.controller.sampleTime)
                    u=nonlinearBicycleModel.nominalFeedback(x,u,lane,reference,cfg,terminal);
                    x=nonlinearBicycleModel.sample(x,u,cfg);
                    if all(abs(nonlinearBicycleModel.error(x,lane,reference))<=tolerance),inside=inside+1;else,inside=0;end
                    if inside*cfg.controller.sampleTime>=5,break;end
                end
                testCase.verifyGreaterThanOrEqual(inside*cfg.controller.sampleTime,5-1e-9);
            end
        end
        function targetFreeClosedLoopDecreasesTheClfEveryFrame(testCase)
            [cfg,lane,reference,terminal]=localSetup(8,0);
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('horizonSteps',8)));
            road=struct('referenceCurve',lane.referenceCurve);
            [position,heading]=laneGeometry.referencePose(0,-40,lane.referenceCurve);
            x=[position;heading+2.5;6;.3;.1];u=reference.input;prior=[];values=zeros(1,21);
            for frame=1:21
                ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
                    'heldActuatorInput',u,'stateTime',(frame-1)*cfg.controller.sampleTime);
                [command,~,problem,prior]=collisionAvoidanceController(ego,[],road,cfg,prior);
                metadata=problem.metadata;values(frame)=metadata.clfInitialValue;
                testCase.verifyEqual(metadata.clfFunction,"nominalCostToGo");
                testCase.verifyEqual(metadata.solverCallCount,2);
                if frame>1
                    testCase.verifyLessThanOrEqual(values(frame),values(frame-1)-.5*required+1e-6*values(frame-1));
                end
                required=metadata.clfRequiredDecrease;u=command.actuatorInput;x=nonlinearBicycleModel.sample(x,u,cfg);
            end
        end
        function nominalFractionsMustLieInsideTheUnitInterval(testCase)
            for name=["decreaseFraction","lateralAccelerationFraction","frontForceFraction","brakingRatioLimit"]
                testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('nominalClf',struct(name,1))), ...
                    'collisionAvoidanceController:invalidConfiguration');
            end
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('nominalClf',struct('courseGain',0))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
    end
end

function [cfg,lane,reference,terminal]=localSetup(speed,curvature)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
    lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
    reference=nonlinearBicycleModel.cruise(cfg,curvature);
    terminal=nonlinearBicycleModel.nominalTail(cfg,curvature);
end

function states=localStates(lane,reference)
    % Distant, reversed, slow, sliding and yawing starts.
    cases=[-100,pi,1,0,0;100,-2.5,.5,1.5,.5;-30,1.6,1.15,-1,-.3;5,-.4,1,0,0;0,pi-.1,.8,.5,0];
    states=zeros(6,size(cases,1));
    for index=1:size(cases,1)
        c=cases(index,:);[position,heading]=laneGeometry.referencePose(0,c(1),lane.referenceCurve);
        states(:,index)=[position;heading+reference.state(3)+c(2);reference.state(4)*c(3);c(4);c(5)];
    end
end
