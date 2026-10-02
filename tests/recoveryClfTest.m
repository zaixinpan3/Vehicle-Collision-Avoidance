classdef recoveryClfTest < matlab.unittest.TestCase
    % The recovery CLF is the cost-to-go of the path-guidance feedback
    % (controller/RECOVERY_CLF.md). These tests check its defining properties in
    % the controller's own model: an exact trim equilibrium, a terminal quadratic
    % that solves the linear-loop Lyapunov equation, a decrease by exactly the
    % stage cost along the feedback, convergence from distant and reversed
    % states, and a target-free closed loop that decreases it at every frame.
    properties (TestParameter)
        referenceSpeed={8,15};
        curvature={0,.005};
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
            u=nonlinearBicycleModel.recoveryInput(trim,reference.input,lane,reference,cfg,terminal);
            testCase.verifyEqual(u,reference.input,AbsTol=1e-12);
            testCase.verifyEqual(nonlinearBicycleModel.recoveryValue(trim,u,lane,reference,terminal,cfg),0,AbsTol=1e-20);
            testCase.verifyLessThan(terminal.spectralRadius,1);
            A=terminal.closedLoop;P=terminal.matrix;
            testCase.verifyLessThanOrEqual(norm(A.'*P*A-P+terminal.stageFactor^2),1e-8*norm(P));
        end
        function costToGoDecreasesByExactlyTheStageCost(testCase,referenceSpeed,curvature)
            [cfg,lane,reference,terminal]=localSetup(referenceSpeed,curvature);
            for state=localStates(lane,reference)
                x=state;u=nonlinearBicycleModel.recoveryInput(x,reference.input,lane,reference,cfg,terminal);
                [value,~,~,converged]=nonlinearBicycleModel.recoveryValue(x,reference.input,lane,reference,terminal,cfg);
                stage=sum((terminal.stageFactor*nonlinearBicycleModel.error(x,lane,reference)).^2);
                next=nonlinearBicycleModel.recoveryValue(nonlinearBicycleModel.sample(x,u,cfg),u,lane,reference,terminal,cfg);
                testCase.verifyTrue(converged);
                testCase.verifyEqual(next,value-stage,AbsTol=1e-8*max(1,value));
            end
        end
        function feedbackReturnsFromDistantAndReversedStates(testCase,referenceSpeed,curvature)
            [cfg,lane,reference,terminal]=localSetup(referenceSpeed,curvature);
            tolerance=[.1;pi/180;.1;.05;.01];
            for state=localStates(lane,reference)
                x=state;u=reference.input;inside=0;
                for step=1:round(60/cfg.controller.sampleTime)
                    u=nonlinearBicycleModel.recoveryInput(x,u,lane,reference,cfg,terminal);
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
                feedback=nonlinearBicycleModel.recoveryInput(x,u,problem.model.lane,reference,cfg,terminal);
                testCase.verifyEqual(metadata.search.initialization,"recoveryFeedbackRollout");
                testCase.verifyEqual(command.actuatorInput,feedback,AbsTol=1e-6);
                if frame>1
                    testCase.verifyLessThanOrEqual(values(frame),values(frame-1)-required+1e-6*values(frame-1));
                end
                required=metadata.clfRequiredDecrease;u=command.actuatorInput;x=nonlinearBicycleModel.sample(x,u,cfg);
            end
        end
        function recoveryFractionsMustLieInsideTheUnitInterval(testCase)
            for name=["decreaseFraction","lateralAccelerationFraction","frontForceFraction","brakingRatioLimit"]
                testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('recovery',struct(name,1))), ...
                    'collisionAvoidanceController:invalidConfiguration');
            end
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('recovery',struct('courseGain',0))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
    end
end

function [cfg,lane,reference,terminal]=localSetup(speed,curvature)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
    lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
    reference=nonlinearBicycleModel.cruise(cfg,curvature);
    terminal=nonlinearBicycleModel.recoveryTerminal(cfg,curvature);
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
