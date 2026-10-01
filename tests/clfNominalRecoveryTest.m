classdef clfNominalRecoveryTest < matlab.unittest.TestCase
    % Closed-loop dissipation beyond the original initialization/completion tail.
    properties (TestParameter)
        referenceSpeed={8,15};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function circularCruiseRemainsNominalAfterTheInitialTail(testCase,referenceSpeed)
            result=localClosedLoop(referenceSpeed,0,140);
            testCase.verifyLessThan(max(result.energy),1e-10);
            testCase.verifyLessThan(max(abs(result.finalError)),1e-6);
            testCase.verifyEqual(result.maximumHard,0,AbsTol=0);
            testCase.verifyEqual(result.maximumSafety,0,AbsTol=0);
            testCase.verifyTrue(all(result.clfCompleted));
            testCase.verifyGreaterThanOrEqual(min(result.horizon),referenceSpeed+mod(referenceSpeed,2)+60);
        end
        function curvedTrackingErrorDissipatesAfterTheInitialTail(testCase,referenceSpeed)
            result=localClosedLoop(referenceSpeed,.1,240);
            testCase.verifyLessThanOrEqual(result.energy(end), ...
                (1-.01)^240*result.energy(1)+1e-10);
            testCase.verifyLessThanOrEqual(max(result.relaxation),1e-10);
            testCase.verifyEqual(result.maximumHard,0,AbsTol=0);
            testCase.verifyEqual(result.maximumSafety,0,AbsTol=0);
            testCase.verifyTrue(all(result.clfCompleted));
            testCase.verifyGreaterThanOrEqual(min(result.horizon),referenceSpeed+mod(referenceSpeed,2)+60);
        end
    end
end

function result=localClosedLoop(speed,lateralError,holds)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed, ...
        'controller',struct('horizonSteps',speed+mod(speed,2))));
    road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',.005,'length',200), ...
        'lateralClearance',[4;4]);
    reference=nonlinearBicycleModel.cruise(cfg,.005);x=reference.state;x(2)=lateralError;
    previous=[];input=zeros(2,1);energy=zeros(1,holds+1);relaxation=zeros(1,holds);
    hard=0;safety=0;completed=false(1,holds);horizon=zeros(1,holds);
    for step=1:holds
        ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
            'stateTime',(step-1)*cfg.controller.sampleTime,'heldActuatorInput',input);
        [command,~,prediction,previous]=collisionAvoidanceController(ego,[],road,cfg,previous);
        input=command.actuatorInput;x=nonlinearBicycleModel.sample(x,input,cfg);
        energy(step)=prediction.solution.clfInitialValue;energy(step+1)=prediction.solution.clfNextValue;
        relaxation(step)=prediction.solution.clfSlack;
        hard=max(hard,prediction.solution.hard);safety=max(safety,prediction.solution.safety);
        completed(step)=prediction.metadata.search.clfLowerBound;horizon(step)=size(previous.inputTrajectory,2);
    end
    result=struct('energy',energy,'relaxation',relaxation,'maximumHard',hard, ...
        'maximumSafety',safety,'clfCompleted',completed,'horizon',horizon, ...
        'finalError',nonlinearBicycleModel.error(x,prediction.model.lane,reference));
end
