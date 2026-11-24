classdef rawSensorContractTest < matlab.unittest.TestCase
    % Raw sensor timing, observation consistency, and persistent physical state.
    properties (Access=private)
        cfg
        hypothesis
        block
        state
        prediction
    end
    methods (TestClassSetup)
        function prepareJointController(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            setupJointNlpSolver;
            [cfg,hypothesis,block]=localFixture();
            [~,~,prediction,state]=collisionAvoidanceController(block,[],[],cfg,[]);
            testCase.cfg=cfg;testCase.hypothesis=hypothesis;testCase.block=block;
            testCase.state=state;testCase.prediction=prediction;
        end
    end
    methods (Test)
        function accelerationUsesThePreviouslyAppliedCommand(testCase)
            bad=testCase.block;bad.heldInput=[0;0];
            testCase.verifyError(@()collisionAvoidanceController(bad,[],[],testCase.cfg,[]), ...
                'collisionAvoidanceController:emptyJointInformation');
        end
        function aDifferentAppliedCommandInvalidatesContinuation(testCase)
            bad=localNextBlock(testCase.state,testCase.cfg);bad.heldInput(2)=bad.heldInput(2)+.01;
            testCase.verifyError(@()collisionAvoidanceController(bad,[],[],testCase.cfg,testCase.state), ...
                'collisionAvoidanceController:actuationMismatch');
        end
        function missingIntermediateReadingsCannotBeSilentlySkipped(testCase)
            bad=localNextBlock(testCase.state,testCase.cfg);bad.samples(2)=[];
            testCase.verifyError(@()collisionAvoidanceController(bad,[],[],testCase.cfg,testCase.state), ...
                'collisionAvoidanceController:invalidSensorGrid');
        end
        function parameterAndSensorBiasHypothesesPersistThroughMotion(testCase)
            hypothesis=testCase.hypothesis;hypothesis.parameters=[.9;1.1;.95;1.05];hypothesis.sensorBias=(1:7)'*.001;
            next=localOperation('advance',testCase.cfg,struct('hypothesis',hypothesis,'input',[0;-.6], ...
                'duration',.05,'endOfHold',true));
            testCase.verifyEqual(next.parameters,hypothesis.parameters,AbsTol=0);
            testCase.verifyEqual(next.sensorBias,hypothesis.sensorBias,AbsTol=0);
        end
        function sensorBoundariesDoNotApplyTheControlBoundaryRestReset(testCase)
            hypothesis=testCase.hypothesis;hypothesis.ego(4:6)=[.04;.2;.1];
            request=struct('hypothesis',hypothesis,'input',[0;-.6],'duration',.0125,'endOfHold',false);
            middle=localOperation('advance',testCase.cfg,request);request.endOfHold=true;
            boundary=localOperation('advance',testCase.cfg,request);
            testCase.verifyGreaterThan(norm(middle.ego(4:6)),0);
            testCase.verifyEqual(boundary.ego(4:6),zeros(3,1),AbsTol=0);
            testCase.verifyEqual(middle.ego(1:3),boundary.ego(1:3),AbsTol=0);
        end
        function anAbsorbedTargetCannotRestartFromItsLatentAcceleration(testCase)
            hypothesis=testCase.hypothesis;hypothesis.target(5)=2;
            next=localOperation('advance',testCase.cfg,struct('hypothesis',hypothesis,'input',[0;-.6], ...
                'duration',.05,'endOfHold',true));
            testCase.verifyEqual(next.target,hypothesis.target,AbsTol=0);
        end
        function aSingleMeasurementDoesNotInventAnInitialInformationSet(testCase)
            block=rmfield(testCase.block,'initialHypotheses');
            testCase.verifyError(@()collisionAvoidanceController(block,[],[],testCase.cfg,[]), ...
                'collisionAvoidanceController:missingInitialInformation');
        end
        function aCapacityLimitCannotSilentlyShortenThePreview(testCase)
            cfg=testCase.cfg;
            cfg.controller.maximumHorizonSteps=10;
            testCase.verifyError(@()collisionAvoidanceController(testCase.block,[],[],cfg,[]), ...
                'collisionAvoidanceController:predictionWindowCapacity');
        end
    end
end

function result=localOperation(operation,cfg,request)
    request.operation=string(operation);request.cfg=cfg;
    result=solvePredictiveControl('outputFeedback',request);
end

function [cfg,hypothesis,block]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',1.5, ...
        'outputFeedback',struct('nlpJit',false,'maximumSolveSeconds',120)));
    hypothesis=struct('ego',[0;0;0;1.5;0;0],'target',[15;8;0;0;0;0;1.6;2.4;.95;0;0], ...
        'parameters',ones(4,1),'sensorBias',zeros(7,1));
    input=[0;-.6];
    y=localOperation('sensor',cfg,struct('hypothesis',hypothesis,'input',input,'time',0));
    block=struct('kind',"jointSensorBlock-v1",'time',0,'samples',y,'heldInput',input,'initialHypotheses',hypothesis);
end

function block=localNextBlock(state,cfg)
    hypothesis=state.hypotheses(1);input=state.appliedInput;
    count=round(cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod);samples=struct([]);
    for j=1:count
        hypothesis=localOperation('advance',cfg,struct('hypothesis',hypothesis,'input',input, ...
            'duration',cfg.outputFeedback.sensorPeriod,'endOfHold',j==count));
        y=localOperation('sensor',cfg,struct('hypothesis',hypothesis,'input',input, ...
            'time',state.time+j*cfg.outputFeedback.sensorPeriod));
        if isempty(samples),samples=y;else,samples(end+1)=y;end %#ok<AGROW>
    end
    block=struct('kind',"jointSensorBlock-v1",'time',state.time+cfg.controller.sampleTime, ...
        'samples',samples,'heldInput',input);
end
