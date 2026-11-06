classdef nonlinearOutputFeedbackTest < matlab.unittest.TestCase
    properties (TestParameter)
        speed = {.04, 1.5, 3.3, 8, 25}
        braking = {-1, -.6, 0, .5, 1}
        targetAcceleration = {-.2, 0, .5}
    end
    methods (TestClassSetup)
        function loadSolver(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            setupJointNlpSolver;
            testCase.addTeardown(@()clear('solvePredictiveControl'));
        end
    end
    methods (Test)
        function symbolicFlowMatchesTheDeclaredPlantAcrossModes(testCase,speed,braking)
            cfg=collisionAvoidanceControllerConfig();
            scales=[.95;1.02;.99;1.005];physical=cfg;
            physical.tire.corneringStiffness=physical.tire.corneringStiffness.*scales(1:2);
            physical.tire.frictionCoefficient=physical.tire.frictionCoefficient.*scales(3:4);
            x=[2;-1;.2;speed;.03;.02];u=[.02;braking];
            sx=casadi.SX.sym('x',6);su=casadi.SX.sym('u',2);
            flow=casadi.Function('flow',{sx,su},{nonlinearBicycleModel.symbolicSensorFlow(sx,su,scales,cfg,.025,true)});
            testCase.verifyEqual(full(flow(x,u)),nonlinearBicycleModel.sensorFlow(x,u,physical,.025,true),AbsTol=1e-10);
        end
        function automaticDerivativesMatchTheExistingAnalyticModel(testCase)
            cfg=collisionAvoidanceControllerConfig();x=[2;-1;.2;8;.1;.02];u=[.02;-.4];
            sx=casadi.SX.sym('x',6);su=casadi.SX.sym('u',2);
            f=nonlinearBicycleModel.symbolicDerivative(sx,su,ones(4,1),cfg);
            tangent=casadi.Function('tangent',{sx,su},{jacobian(f,[sx;su])});
            [a,b]=nonlinearBicycleModel.jacobian(x,u,cfg);
            testCase.verifyEqual(full(tangent(x,u)),[a,b],AbsTol=1e-9);
        end
        function acceptedPoliciesPassNonlinearReplayAndReuseTheCommittedMemory(testCase)
            [cfg,block]=localFixture();
            [~,~,prediction,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            next=localBlock(state,cfg);
            expected=solvePredictiveControl('outputFeedback',struct('operation',"memory", ...
                'cfg',cfg,'memory',state.memory,'policy',state.committedPolicy,'samples',next.samples));
            [command,~,second,state2]=collisionAvoidanceController(next,[],[],cfg,state);
            testCase.verifyEqual(state2.memory,expected,AbsTol=1e-14);
            testCase.verifyEqual(command.actuatorInput,state2.committedPolicy.v+state2.committedPolicy.K*expected,AbsTol=1e-14);
            testCase.verifyTrue(prediction.metadata.nonlinearValidationPerformed);
            testCase.verifyFalse(prediction.metadata.predictionIsLinearized);
            testCase.verifyLessThanOrEqual(second.metadata.maximumConstraintViolation,cfg.outputFeedback.nlpTolerance);
            testCase.verifyTrue(second.metadata.nlpCacheHit);
            testCase.verifyTrue(second.metadata.nlpDualWarmStart);
            testCase.verifyFalse(second.metadata.robustNonlinearSafetyCertified);
        end
        function hypothesesReceiveOneCommonCurrentCommand(testCase)
            [cfg,block]=localFixture();
            block.initialHypotheses(2)=block.initialHypotheses(1);
            block.initialHypotheses(2).ego(1)=.01;
            [~,~,prediction]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyEqual(prediction.scenarios{1}.inputs(:,1),prediction.scenarios{2}.inputs(:,1),AbsTol=1e-14);
            testCase.verifyEqual(prediction.metadata.scenarioCount,2);
        end
        function continuingTargetsHaveNoBrakingAdmissionGate(testCase,targetAcceleration)
            [cfg,block]=localFixture();
            block.initialHypotheses.target(4:6)=[2;targetAcceleration;.04];
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',block.initialHypotheses,'input',block.heldInput,'time',0));
            [command,~,prediction]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyEqual(prediction.scenarios{1}.targets(5:6,end),[targetAcceleration;.04],AbsTol=1e-14);
            testCase.verifyLessThanOrEqual(prediction.metadata.maximumConstraintViolation,cfg.outputFeedback.nlpTolerance);
        end
        function posteriorPruningReusesTheCompiledProblemWithoutStaleDuals(testCase)
            [cfg,block]=localFixture();
            block.initialHypotheses(2)=block.initialHypotheses(1);
            block.initialHypotheses(2).ego(1)=.035;
            block.initialHypotheses(2).target(1)=block.initialHypotheses(2).target(1)+.035;
            [~,~,~,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            next=localBlock(state,cfg,-.02);
            [~,~,prediction,state]=collisionAvoidanceController(next,[],[],cfg,state);
            testCase.verifyNumElements(state.hypotheses,1);
            testCase.verifyTrue(prediction.metadata.nlpCacheHit);
            testCase.verifyFalse(prediction.metadata.nlpDualWarmStart);
            testCase.verifyEqual(prediction.metadata.scenarioCount,2);
            testCase.verifyEqual(prediction.metadata.activeScenarioCount,1);
        end
        function anEmptyTargetUsesTheSameNonlinearPolicyInterface(testCase)
            [cfg,block]=localFixture();block.initialHypotheses.target=[];
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',block.initialHypotheses,'input',block.heldInput,'time',0));
            [command,~,prediction]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyLessThanOrEqual(prediction.metadata.maximumConstraintViolation,cfg.outputFeedback.nlpTolerance);
        end
        function theNonlinearBackendUsesAutomaticTimeCoverage(testCase)
            [cfg,block]=localFixture();cfg.outputFeedback.horizonSeconds=[];cfg.referenceSpeed=1.5;
            block.initialHypotheses.ego(4)=1.5;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',block.initialHypotheses,'input',block.heldInput,'time',0));
            [~,~,prediction,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyEqual(prediction.metadata.predictionWindow.source,"speedBasedPreview");
            testCase.verifyEqual(numel(state.policies),prediction.metadata.predictionWindow.horizonSteps);
            testCase.verifyGreaterThan(prediction.metadata.predictionWindow.allocatedSeconds,.2);
            testCase.verifyLessThanOrEqual(prediction.metadata.maximumConstraintViolation,cfg.outputFeedback.nlpTolerance);
        end
        function risingSpeedExtendsTheShiftedPlanWithoutResettingMemory(testCase)
            [cfg,block]=localFixture();cfg.outputFeedback.horizonSeconds=[];
            cfg.outputFeedback.maximumSolveSeconds=5;block.initialHypotheses.ego(4)=1.95;
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',block.initialHypotheses,'input',block.heldInput,'time',0));
            [~,~,first,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            next=localBlock(state,cfg);
            expected=solvePredictiveControl('outputFeedback',struct('operation',"memory", ...
                'cfg',cfg,'memory',state.memory,'policy',state.committedPolicy,'samples',next.samples));
            [~,~,second,state]=collisionAvoidanceController(next,[],[],cfg,state);
            testCase.verifyGreaterThan(second.metadata.horizonSteps,first.metadata.horizonSteps);
            testCase.verifyEqual(numel(state.policies),second.metadata.horizonSteps);
            testCase.verifyEqual(state.memory,expected,AbsTol=1e-14);
            testCase.verifyFalse(second.metadata.nlpDualWarmStart);
            testCase.verifyLessThanOrEqual(second.metadata.maximumConstraintViolation,cfg.outputFeedback.nlpTolerance);
        end
        function anInvalidNumericalSeedDoesNotResetMeasuredMemory(testCase)
            [cfg,block]=localFixture();block.initialMemory=.9;
            policy=struct('v',[0;.95],'K',[0;.25],'A',0,'B',zeros(1,8),'c',0);
            block.initialPolicies=repmat(policy,1,ceil(cfg.outputFeedback.horizonSeconds/cfg.controller.sampleTime));
            [command,~,prediction,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyTrue(prediction.metadata.numericalSeedRepaired);
            testCase.verifyEqual(state.memory,.9);
            testCase.verifyLessThanOrEqual(abs(command.brakingRatio),1);
            testCase.verifyLessThanOrEqual(prediction.metadata.maximumConstraintViolation,cfg.outputFeedback.nlpTolerance);
        end
        function nonlinearIterationsEscapeTheRtiCoefficientStepBox(testCase)
            [cfg,block]=localFixture();block.heldInput=[-.4;0];
            policy=struct('v',[.4;0],'K',zeros(2,1),'A',0,'B',zeros(1,8),'c',0);
            block.initialPolicies=repmat(policy,1,ceil(cfg.outputFeedback.horizonSeconds/cfg.controller.sampleTime));
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',block.initialHypotheses,'input',block.heldInput,'time',0));
            rti=cfg;rti.outputFeedback.method="sqpRti";
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],rti,[]), ...
                'collisionAvoidanceController:noOutputFeedbackPlan');
            [command,~,prediction]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.verifyLessThan(command.steeringAngle,-.35+cfg.outputFeedback.nlpTolerance);
            testCase.verifyLessThanOrEqual(prediction.metadata.maximumConstraintViolation,cfg.outputFeedback.nlpTolerance);
        end
        function anInitiallyOverlappingTargetIsNotMadeFeasibleBySofteningSafety(testCase)
            [cfg,block]=localFixture();
            block.initialHypotheses.target(1:2)=block.initialHypotheses.ego(1:2);
            block.samples=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
                'cfg',cfg,'hypothesis',block.initialHypotheses,'input',block.heldInput,'time',0));
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],cfg,[]), ...
                'collisionAvoidanceController:noOutputFeedbackPlan');
        end
    end
end

function [cfg,block]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'outputFeedback',struct('method',"ipopt",'horizonSeconds',.15,'maximumSolveSeconds',2,'nlpJit',false)));
    hypothesis=struct('ego',[0;0;0;8;0;0],'target',[30;0;0;0;0;0;1.6;2.4;.95;0;0], ...
        'parameters',ones(4,1),'sensorBias',zeros(7,1));
    input=[0;-.6];sample=solvePredictiveControl('outputFeedback',struct('operation',"sensor", ...
        'cfg',cfg,'hypothesis',hypothesis,'input',input,'time',0));
    block=struct('kind',"jointSensorBlock-v1",'time',0,'samples',sample, ...
        'heldInput',input,'initialHypotheses',hypothesis);
end

function block=localBlock(state,cfg,positionOffset)
    if nargin<3,positionOffset=0;end
    item=state.hypotheses(1);u=state.appliedInput;count=round(cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod);
    samples=struct([]);
    for j=1:count
        item=solvePredictiveControl('outputFeedback',struct('operation',"advance",'cfg',cfg, ...
            'hypothesis',item,'input',u,'duration',cfg.outputFeedback.sensorPeriod,'endOfHold',j==count));
        sample=solvePredictiveControl('outputFeedback',struct('operation',"sensor",'cfg',cfg, ...
            'hypothesis',item,'input',u,'time',state.time+j*cfg.outputFeedback.sensorPeriod));
        sample.position(1)=sample.position(1)+positionOffset;
        if isempty(samples),samples=sample;else,samples(j)=sample;end
    end
    block=struct('kind',"jointSensorBlock-v1",'time',state.time+cfg.controller.sampleTime, ...
        'heldInput',u,'samples',samples);
end
