classdef trustInnovationTest < matlab.unittest.TestCase
    % Input trust scale estimated from the full-step remainder e = L*s^2.
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
        function theFullStepRemainderSetsTheNextScale(testCase)
            % The frame solves with the initial scale; the nonlinear rollout of
            % its full affine step measures the remainder L*s^2 that sets the
            % scale of the next frame.
            [ego,road,cfg]=localFixture();
            [~,inputs,problem,state]=collisionAvoidanceController(ego,[],road,cfg,[]);
            search=problem.metadata.search;acceptance=search.acceptance;
            testCase.verifyEqual(search.inputTrustScale,cfg.nonlinear.trustInitialScale);
            testCase.verifyTrue(state.trust.updated);
            testCase.verifyEqual(state.trust.modelInnovation,acceptance.fullGap,AbsTol=0);
            prior=struct('scale',cfg.nonlinear.trustInitialScale,'correction',acceptance.fullCorrection, ...
                'curvature',cfg.nonlinear.trustInnovationMeters/cfg.nonlinear.trustInitialScale^2);
            testCase.verifyEqual(state.trust.scale,localExpectedScale(prior,acceptance.fullGap,cfg),RelTol=1e-12);
            unit=cfg.nonlinear.trustRadius*[.15;.25];
            anchor=problem.model.linearization.inputs;
            testCase.verifyEqual(state.trust.correction,max(abs(inputs-anchor)./unit,[],'all'),AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(acceptance.fullCorrection, ...
                search.inputTrustScale+cfg.solver.feasibilityTolerance);
            testCase.verifyEqual(problem.metadata.trust.scale,state.trust.scale);
        end
        function anAccuratePlanGrowsTheScaleByAtMostTheRetentionBound(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,~,problem,state]=collisionAvoidanceController(localSuccessor(ego,prior),[],road,cfg,prior);
            trust=problem.metadata.trust;acceptance=problem.metadata.search.acceptance;
            testCase.verifyEqual(problem.metadata.search.inputTrustScale,prior.trust.scale);
            testCase.verifyLessThan(trust.modelInnovation,cfg.nonlinear.trustInnovationMeters);
            previous=prior.trust;previous.correction=acceptance.fullCorrection;
            testCase.verifyEqual(trust.scale,localExpectedScale(previous,trust.modelInnovation,cfg),RelTol=1e-12);
            testCase.verifyLessThanOrEqual(trust.scale, ...
                prior.trust.scale/sqrt(cfg.nonlinear.trustRetention)*(1+1e-12));
            testCase.verifyEqual(state.trust.scale,trust.scale);
        end
        function aPosteriorDepartureIsAttributedToTheObserverNotTheModel(testCase)
            [ego,road,cfg]=localFixture();
            [~,~,~,prior]=collisionAvoidanceController(ego,[],road,cfg,[]);
            exact=localSuccessor(ego,prior);
            [~,~,reference]=collisionAvoidanceController(exact,[],road,cfg,prior);
            offset=.3;displaced=exact;displaced.position(2)=displaced.position(2)+offset;
            [~,~,problem]=collisionAvoidanceController(displaced,[],road,cfg,prior);
            % The plan is a nonlinear rollout, so a posterior equal to its second
            % node has no innovation; a displaced one enters the observer part.
            testCase.verifyEqual(reference.metadata.trust.observationInnovation,0,AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(problem.metadata.trust.observationInnovation,offset*(1-1e-9));
            testCase.verifyEqual(problem.metadata.search.inputTrustScale,prior.trust.scale);
        end
        function theScaleFollowsTheClampedLawWithinItsBounds(testCase)
            for target=[1e-12,10]
                [ego,road,cfg]=localFixture();cfg.nonlinear.trustInnovationMeters=target;
                [~,~,problem,state]=collisionAvoidanceController(ego,[],road,cfg,[]);
                acceptance=problem.metadata.search.acceptance;
                prior=struct('scale',cfg.nonlinear.trustInitialScale,'correction',acceptance.fullCorrection, ...
                    'curvature',target/cfg.nonlinear.trustInitialScale^2);
                testCase.verifyEqual(state.trust.scale,localExpectedScale(prior,acceptance.fullGap,cfg),RelTol=1e-12);
                testCase.verifyGreaterThanOrEqual(state.trust.scale,cfg.nonlinear.trustMinimumScale);
                testCase.verifyLessThanOrEqual(state.trust.scale,cfg.nonlinear.trustMaximumScale);
            end
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
