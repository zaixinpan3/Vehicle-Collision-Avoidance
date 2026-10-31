classdef nonlinearOptimizationBackendTest < matlab.unittest.TestCase
    % Direct NLP solves nonlinear equations instead of one affine update.
    properties (Access=private)
        prediction
        config
    end
    methods (TestClassSetup)
        function solveNontrivialCruise(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
                'controller',struct('horizonSteps',8), ...
                'solver',struct('method',"nonlinear",'maxIterations',150,'timeLimitSeconds',120)));
            ego=struct('position',[0;.4],'yaw',.03,'speed',8,'lateralVelocity',0,'yawRate',0);
            [~,~,testCase.prediction]=collisionAvoidanceController(ego,[],[],cfg,[]);
            testCase.config=cfg;
        end
    end
    methods (Test)
        function optimizedNodesSatisfyTheNonlinearTransition(testCase)
            p=testCase.prediction;
            residual=localDynamicsResidual(p.solution.states,p.solution.inputs,testCase.config);
            testCase.verifyLessThanOrEqual(residual,testCase.config.solver.constraintTolerance);
            testCase.verifyEqual(p.metadata.predictionModel,"nonlinearMultipleShooting");
            testCase.verifyEqual(p.metadata.rtiIterations,0);
            testCase.verifyTrue(p.metadata.nonlinearValidationPerformed);
            testCase.verifyFalse(p.metadata.robustNonlinearSafetyCertified);
        end
        function theIntegratedClfUsesTheNonlinearNextState(testCase)
            p=testCase.prediction;cfg=testCase.config;alpha=p.model.nominalReference.decayRate;
            nextValue=nonlinearBicycleModel.nominalValue(p.solution.states(:,2),p.model.lane,p.model.nominalReference);
            limit=exp(-alpha*cfg.controller.sampleTime)*p.solution.clfInitialValue ...
                -expm1(-alpha*cfg.controller.sampleTime)/alpha*p.solution.clfSlack;
            testCase.verifyLessThanOrEqual(nextValue-limit,cfg.solver.constraintTolerance*max(1,p.solution.clfInitialValue));
            testCase.verifyFalse(p.metadata.trustRegionEnabled);
            testCase.verifyTrue(p.metadata.optimizationConverged);
        end
        function nonlinearAndRetainedModesCannotBeConfused(testCase)
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct( ...
                'solver',struct('method',"nonlinear"),'controller',struct('retainedTube',true))), ...
                'collisionAvoidanceController:invalidConfiguration');
            cfg=collisionAvoidanceControllerConfig();
            testCase.verifyEqual(cfg.solver.method,"rti");
        end
        function constraintDerivativesMatchNonlinearPerturbations(testCase)
            model=testCase.prediction.model;anchor=model.linearization;
            model.solveTimeLimitSeconds=Inf;
            model.target=[10;2;pi;8;0;0;1.6;2.4;.95;0;0];
            model.targetEpoch=model.target;model.targetEpochIndex=model.sampleIndex;
            model.terminalContext=terminalSafeSet.context(model);
            problem=solvePredictiveControl('probe',struct('operation','nonlinearProblem', ...
                'model',model,'anchor',anchor));
            point=problem.point;point(problem.stateIndices(5,end))=.02;
            [~,~,gc,ge]=problem.constraint(point);[~,g]=problem.performance(point);
            columns=unique([problem.stateIndices(:,end);problem.inputIndices(:,1);problem.stateIndices(:,1)]);
            for index=columns.'
                delta=zeros(size(point));delta(index)=1e-6;
                [cp,ep]=problem.constraint(point+delta);[cm,em]=problem.constraint(point-delta);
                testCase.verifyEqual((cp-cm)/2e-6,full(gc(index,:)).',AbsTol=2e-4);
                testCase.verifyEqual((ep-em)/2e-6,full(ge(index,:)).',AbsTol=2e-4);
                derivative=(problem.performance(point+delta)-problem.performance(point-delta))/2e-6;
                testCase.verifyEqual(derivative,g(index),AbsTol=2e-4);
            end
        end
        function positiveSafetySlackPermitsAFeasibleNonlinearPlan(testCase)
            model=testCase.prediction.model;anchor=model.linearization;
            % Overlap without coincident vertices or a sampling-time contact.
            model.target=[anchor.states(1:2,1)+[1.37;.41];0;40;0;0;1.6;2.4;.95;0;0];
            model.targetEpoch=model.target;model.targetEpochIndex=model.sampleIndex;
            model.terminalContext=terminalSafeSet.context(model);
            problem=solvePredictiveControl('probe',struct('operation','nonlinearProblem', ...
                'model',model,'anchor',anchor));
            point=problem.point;[unrelaxed,~]=problem.constraint(point);
            testCase.verifyGreaterThan(max(unrelaxed),0);
            point(problem.slackIndices)=10;
            [relaxed,equality]=problem.constraint(point);
            testCase.verifyLessThanOrEqual(max(relaxed),1e-5);
            testCase.verifyLessThanOrEqual(max(abs(equality)),1e-5);
            testCase.verifyGreaterThan(problem.safety(point),0);
        end
    end
end

function residual=localDynamicsResidual(states,inputs,cfg)
    residual=0;
    for k=1:size(inputs,2)
        next=nonlinearBicycleModel.sample(states(:,k),inputs(:,k),cfg);
        residual=max(residual,norm(states(:,k+1)-next,Inf));
    end
end
