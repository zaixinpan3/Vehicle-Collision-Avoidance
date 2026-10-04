classdef trustInnovationTest < matlab.unittest.TestCase
    % Input trust scale estimated from the plan innovation e = L*s^2.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (TestMethodSetup)
        function reset(~)
            collisionAvoidanceController("resetNominalTrajectory");
        end
    end
    methods (Test)
        function startupUsesTheInitialScaleAndRecordsTheCorrection(testCase)
            [ego,road,cfg]=localFixture();
            [~,inputs,problem,state]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyEqual(state.trust.scale,cfg.nonlinear.trustInitialScale);
            testCase.verifyFalse(state.trust.updated);
            unit=cfg.nonlinear.trustRadius*[.15;.25];
            anchor=problem.model.linearization.inputs;
            testCase.verifyEqual(state.trust.correction,max(abs(inputs-anchor)./unit,[],'all'),AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(state.trust.correction, ...
                state.trust.scale+cfg.solver.feasibilityTolerance);
            testCase.verifyTrue(isnan(state.trust.modelInnovation));
            testCase.verifyEqual(problem.metadata.trust.scale,state.trust.scale);
        end
        function anAccuratePlanGrowsTheScaleByAtMostTheRetentionBound(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,~,problem,state]=collisionAvoidanceController(localSuccessor(ego,prior),[],road,cfg,prior);
            trust=problem.metadata.trust;
            testCase.verifyTrue(trust.updated);
            testCase.verifyLessThan(trust.modelInnovation,cfg.nonlinear.trustInnovationMeters);
            testCase.verifyEqual(trust.scale,localExpectedScale(prior.trust,trust.modelInnovation,cfg),RelTol=1e-12);
            testCase.verifyGreaterThan(trust.scale,prior.trust.scale);
            testCase.verifyLessThanOrEqual(trust.scale, ...
                prior.trust.scale/sqrt(cfg.nonlinear.trustRetention)*(1+1e-12));
            testCase.verifyEqual(state.trust.scale,trust.scale);
        end
        function aContradictedPredictionShrinksTheScaleBySquareRootLaw(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            offset=8*cfg.nonlinear.trustInnovationMeters;
            next=localSuccessor(ego,prior);
            prior.stateTrajectory(2,5)=prior.stateTrajectory(2,5)+offset;
            [~,~,problem]=collisionAvoidanceController(next,[],road,cfg,prior);
            trust=problem.metadata.trust;
            testCase.verifyGreaterThanOrEqual(trust.modelInnovation,offset*(1-1e-3));
            testCase.verifyEqual(trust.scale,localExpectedScale(prior.trust,trust.modelInnovation,cfg),RelTol=1e-12);
            testCase.verifyLessThan(trust.scale,prior.trust.scale);
        end
        function aPosteriorDepartureIsAttributedToTheObserverNotTheModel(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            exact=localSuccessor(ego,prior);
            [~,~,reference]=collisionAvoidanceController(exact,[],road,cfg,prior);
            offset=.3;displaced=exact;displaced.position(2)=displaced.position(2)+offset;
            [~,~,problem]=collisionAvoidanceController(displaced,[],road,cfg,prior);
            trust=problem.metadata.trust;
            % The model part replays the plan from its own prediction, so the
            % posterior cannot change it; the departure enters the observer part.
            testCase.verifyEqual(trust.modelInnovation,reference.metadata.trust.modelInnovation,AbsTol=0);
            testCase.verifyEqual(trust.scale,reference.metadata.trust.scale,AbsTol=0);
            testCase.verifyGreaterThanOrEqual(trust.observationInnovation,offset*(1-1e-3));
            testCase.verifyLessThanOrEqual(trust.innovation, ...
                trust.observationInnovation+trust.modelInnovation+1e-12);
        end
        function aGrossContradictionStopsAtTheMinimumScale(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            next=localSuccessor(ego,prior);
            prior.stateTrajectory(2,5)=prior.stateTrajectory(2,5)+10;
            [~,~,problem]=collisionAvoidanceController(next,[],road,cfg,prior);
            testCase.verifyEqual(problem.metadata.trust.scale,cfg.nonlinear.trustMinimumScale);
        end
        function invalidTrustSettingsAreRejected(testCase)
            for override={struct('trustRetention',1),struct('trustRetention',0), ...
                    struct('trustMinimumScale',.5,'trustInitialScale',.25), ...
                    struct('trustInitialScale',2,'trustMaximumScale',1), ...
                    struct('trustInnovationMeters',0)}
                testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('nonlinear',override{1})), ...
                    'collisionAvoidanceController:invalidConfiguration');
            end
        end
    end
end

function scale=localExpectedScale(previous,innovation,cfg)
    step=max(previous.correction,previous.scale/2);
    curvature=max(max(innovation/step^2,cfg.nonlinear.trustRetention*previous.curvature),eps);
    scale=min(cfg.nonlinear.trustMaximumScale,max(cfg.nonlinear.trustMinimumScale, ...
        sqrt(cfg.nonlinear.trustInnovationMeters/curvature)));
end
function [ego,road,cfg]=localFixture()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('horizonSteps',8)));
    ego=struct('position',[0;.5],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0,'stateTime',0, ...
        'heldActuatorInput',[0;0]);
    road=struct('centerline',[-100,0;1000,0]);
end
function ego=localSuccessor(ego,prior)
    x=prior.stateTrajectory(:,2);ego.position=x(1:2);ego.yaw=x(3);ego.speed=x(4);
    ego.lateralVelocity=x(5);ego.yawRate=x(6);ego.heldActuatorInput=prior.appliedInput;
    ego.stateTime=ego.stateTime+prior.sampleTime;
end
