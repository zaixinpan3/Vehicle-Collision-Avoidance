classdef wholeHoldActuatorConstraintsTest < matlab.unittest.TestCase
    %wholeHoldActuatorConstraintsTest Whole-period safety without state boxes.
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function diagnosticStateAndSlipLimitsDoNotRestrictTheOptimization(testCase)
            [ego,cfg]=localFixture();
            [~,~,baseline]=collisionAvoidanceController(ego,[],[-100,0;2000,0],cfg,[]);
            cfg.model.speedMaximum=1;
            cfg.model.lateralDomainRadius=.001;
            cfg.model.headingDomainRadius=.001;
            cfg.model.lateralVelocityMaximum=.001;
            cfg.model.yawRateMaximum=.001;
            cfg.model.slipAngleMaximum=[1e-6;1e-6];
            [command,~,problem]=collisionAvoidanceController(ego,[],[-100,0;2000,0],cfg,[]);
            testCase.verifyEqual(problem.inputPlan,baseline.inputPlan,AbsTol=1e-7);
            testCase.verifyTrue(all(problem.program.physicalLabels=="actuator"));
            testCase.verifyLessThanOrEqual(abs(command.actuatorInput),[deg2rad(40);1]);
            testCase.verifyFalse(problem.metadata.stateAndSlipBoundsEnforced);
        end

        function everyPredictionNodeCoversExactlyOneHeldCommand(testCase)
            [ego,cfg]=localFixture();
            target=struct('trackId',1,'targetPositionInertial',[12;4], ...
                'targetVelocityInertial',[20;0],'targetAccelerationInertial',[0;0], ...
                'targetHeadingInertial',0,'targetYawRate',0, ...
                'predictionMotion',struct('kind',"nrmm-motion-v1",'curvatureMaximum',0.05));
            [~,~,problem]=collisionAvoidanceController(ego,target,[-100,0;2000,0],cfg,[]);
            testCase.verifyNumElements(problem.prediction.cells,problem.prediction.stageCount);
            testCase.verifyEqual([problem.prediction.cells.time], ...
                .1*(1:problem.prediction.stageCount),AbsTol=1e-12);
            testCase.verifyEqual([problem.prediction.cells.stage],1:problem.prediction.stageCount);
            testCase.verifyFalse(any(ismember(problem.program.physicalLabels,["modelDomain","tireSlip"])));
        end

        function unrestrictedHeadingSupportContainsEveryRectangleOrientation(testCase)
            [offset,slope]=targetPrediction.rectangleSupportMajorant([0;1],0,2.4,.95,0,50);
            yaw=linspace(-50,50,10001);
            actual=2.4*abs(sin(yaw))+.95*abs(cos(yaw));
            testCase.verifyLessThanOrEqual(max(actual-offset-slope*yaw),0);
        end

        function wholeHoldPredictionPropagatesEveryFutureInput(testCase)
            [ego,cfg]=localFixture();
            [~,~,problem]=collisionAvoidanceController(ego,[],[-100,0;2000,0],cfg,[]);
            residual=localPredictionResidual(problem);
            testCase.verifyLessThanOrEqual(residual,1e-8);
        end
    end
end

function residual=localPredictionResidual(problem)
    prediction=problem.prediction;count=prediction.stageCount;
    inputs=[.2*sin(1:count);.3*cos(1:count)];
    state=problem.model.initialEgoState;residual=0;
    for index=1:count
        flow=expm(problem.model.sampleTime*[prediction.continuousA(:,:,index), ...
            prediction.continuousB(:,:,index),prediction.continuousC(:,index);zeros(3,9)]);
        state=flow(1:6,:)*[state;inputs(:,index);1];
        mapped=prediction.egoStateMatrix(:,:,index+1)*inputs(:)+prediction.egoStateOffset(:,index+1);
        residual=max(residual,norm(state-mapped,inf));
    end
end

function [ego,cfg]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.1),'solver',struct('frameDeadlineSeconds',30)));
    ego=struct('position',[0;.2],'yaw',.03,'speed',8,'lateralVelocity',.05, ...
        'yawRate',.01,'stateTime',0, ...
        'perception',struct('time',0,'range',16,'completeWithinRange',true));
end
