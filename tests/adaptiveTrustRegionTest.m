classdef adaptiveTrustRegionTest < matlab.unittest.TestCase
    % Local model validity is distinct from observer error or tire capacity.
    properties (Access=private)
        cfg
        model
        anchor
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'solver',struct('timeLimitSeconds',10)));
            ego=struct('position',[0;0],'yaw',0,'speed',8,'lateralVelocity',0,'yawRate',0);
            [~,~,p]=collisionAvoidanceController(ego,[],[],cfg,[]);
            testCase.cfg=cfg;testCase.model=p.model;testCase.anchor=p.model.linearization;
        end
    end
    methods (Test)
        function referenceProximityIsOffAndNumericalTrustIsOnByDefault(testCase)
            testCase.verifyTrue(testCase.cfg.rti.trustRegion.enabled);
            testCase.verifyEqual(testCase.cfg.rti.referenceDeviationWeight,0);
            testCase.verifyGreaterThan(testCase.cfg.rti.inputVariationWeight,0);
        end
        function worldTranslationDoesNotConsumeLocalModelTrust(testCase)
            x=[0;0;.1;15;.2;.05];u=[.05;-.3];
            radius=localRadius(x,u,testCase.cfg);
            shifted=localRadius(x+[100;200;0;0;0;0],u,testCase.cfg);
            testCase.verifyEqual(radius(1:2),[Inf;Inf]);
            testCase.verifyEqual(radius(3:end),shifted(3:end),AbsTol=1e-5);
        end
        function combinedSlipChangesTheAllowedSteeringCorrection(testCase)
            x=[0;0;0;15;0;0];
            cruise=localRadius(x,[0;0],testCase.cfg);
            loaded=localRadius(x,[0;.9],testCase.cfg);
            testCase.verifyLessThan(loaded(7),cruise(7));
            testCase.verifyLessThan(loaded(8),cruise(8));
            testCase.verifyGreaterThan(loaded(7:8),0);
        end
        function tighterPredictionAccuracyShrinksTheNeighborhood(testCase)
            x=[0;0;0;15;0;0];u=[0;.5];cfg=testCase.cfg;
            wide=localRadius(x,u,cfg);
            cfg.rti.trustRegion.errorBudget=cfg.rti.trustRegion.errorBudget/4;
            narrow=localRadius(x,u,cfg);
            testCase.verifyLessThanOrEqual(narrow(3:end),wide(3:end));
            testCase.verifyLessThan(norm(narrow(3:end)),norm(wide(3:end)));
        end
        function finiteMixedDisplacementsAreCheckedBeforeTheOptimization(testCase)
            [radius,info]=localRadius([0;0;0;15;0;0],[0;0],testCase.cfg);
            testCase.verifyTrue(all(isfinite(radius(3:end))));
            testCase.verifyLessThanOrEqual(info.sampledErrorRatio,1);
            testCase.verifyGreaterThanOrEqual(info.probePasses,1);
        end
        function aCombinedInteriorCorrectionMatchesTheDiscreteModelBudget(testCase)
            x=[0;0;.1;15;.2;.05];u=[.03;-.4];
            [next,a,b]=nonlinearBicycleModel.sample(x,u,testCase.cfg);
            radius=nonlinearBicycleModel.trustRegion(x,u,testCase.cfg,next,a,b);
            change=[0;0;radius(3:end).*[.2;-.3;.4;-.2;.6;-.5]];
            actual=nonlinearBicycleModel.sample(x+change(1:6),u+change(7:8),testCase.cfg);
            remainder=actual-next-[a,b]*change;
            testCase.verifyLessThanOrEqual(abs(remainder),testCase.cfg.rti.trustRegion.errorBudget);
        end
        function primarySlackOptimizationAlreadyRespectsTrust(testCase)
            request=localPriorityRequest(testCase.model,testCase.anchor);
            limit=request.problem.upper(request.problem.inputIndices(1,1));
            [solution,search]=solvePredictiveControl('probe',request);
            testCase.assertNotEmpty(solution);
            testCase.verifyEqual(solution.inputCorrection(1,1),limit,AbsTol=1e-5);
            testCase.verifyEqual(search.primaryOptimum,limit,AbsTol=1e-5);
            testCase.verifyEqual(solution.safety,search.primaryOptimum,AbsTol=1e-5);
            testCase.verifyEqual(search.slackCap,search.primaryOptimum,AbsTol=0);
            testCase.verifyLessThanOrEqual(search.solverCalls,2);
        end
        function trustBoundsDoNotChangePhysicalActuatorSaturation(testCase)
            [lo,hi]=nonlinearBicycleModel.inputBounds(testCase.cfg);
            testCase.verifyEqual(lo,[-Inf;-1]);testCase.verifyEqual(hi,[Inf;1]);
        end
        function aRequiredStateCorrectionSharesTheExistingErrorBudget(testCase)
            x=[0;0;.1;15;.2;.05];u=[.03;.9999];
            [next,a,b]=nonlinearBicycleModel.sample(x,u,testCase.cfg);
            ordinary=nonlinearBicycleModel.trustRegion(x,u,testCase.cfg,next,a,b);
            required=[0;0;0;0;.23;.13];
            [radius,info]=nonlinearBicycleModel.trustRegion(x,u,testCase.cfg,next,a,b,required);
            testCase.verifyGreaterThanOrEqual(radius(1:6),required);
            testCase.verifyLessThanOrEqual(radius(7:8),ordinary(7:8));
            testCase.verifyLessThanOrEqual(info.sampledErrorRatio,1);
        end
        function anInaccurateRequiredCorrectionCannotBypassTheErrorBudget(testCase)
            x=[0;0;0;15;0;0];u=[0;0];
            [next,a,b]=nonlinearBicycleModel.sample(x,u,testCase.cfg);
            testCase.verifyError(@()nonlinearBicycleModel.trustRegion( ...
                x,u,testCase.cfg,next,a,b,[0;0;1;0;0;0]), ...
                'collisionAvoidanceController:trustRegionUnavailable');
        end
        function shiftedStateDefectsCanBeRepairedInsideTheNumericalDomain(testCase)
            model=testCase.model;cfg=testCase.cfg;anchor=testCase.anchor;
            anchor.inputs=repmat([0;.9999],1,size(anchor.inputs,2));
            for k=1:size(anchor.inputs,2)
                [~,anchor.states(:,k+1)]=terminalSafeSet.holdStates(anchor.states(:,k),anchor.inputs(:,k),cfg);
            end
            anchor.states(5:6,2)=anchor.states(5:6,2)+[-cfg.vehicle.lf*.2;.2];
            p=solvePredictiveControl('probe',struct('operation','formulate', ...
                'model',model,'anchor',anchor,'inherited',true));
            testCase.verifyTrue(p.trustWarmStartAdjusted);
            correction=reshape(p.rhs,6,[]);
            for k=1:numel(p.stages)
                correction(:,k+1)=correction(:,k+1)+p.stages(k).a*correction(:,k);
            end
            point=zeros(size(p.lower));point(p.stateIndices)=correction;
            testCase.verifyEqual(p.equal*point,p.rhs,AbsTol=1e-10);
            testCase.verifyGreaterThanOrEqual(point,p.lower-1e-10);
            testCase.verifyLessThanOrEqual(point,p.upper+1e-10);
            testCase.verifyLessThanOrEqual(p.trustSampledErrorRatio,1);
        end
        function errorBudgetsMustBePositiveAndUseSixStateUnits(testCase)
            bad=struct('rti',struct('trustRegion',struct('errorBudget',zeros(6,1))));
            testCase.verifyError(@()collisionAvoidanceControllerConfig(bad), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
    end
end

function [radius,info]=localRadius(x,u,cfg)
    [next,a,b]=nonlinearBicycleModel.sample(x,u,cfg);
    [radius,info]=nonlinearBicycleModel.trustRegion(x,u,cfg,next,a,b);
end

function request=localPriorityRequest(model,anchor)
    model.target=[30;0;pi;8;0;0;model.cfg.vehicle.lr;2.4;.95;0;0];model.targetEpoch=model.target;
    model.cfg.rti.nominalTrackingWeight=0;model.cfg.rti.referenceDeviationWeight=0;
    model.cfg.rti.inputVariationWeight=20;
    [p,~,model]=solvePredictiveControl('probe',struct('operation','formulate','model',model,'anchor',anchor,'inherited',false));
    column=p.inputIndices(1,1);slack=p.slackIndices(1);limit=p.upper(column);n=numel(p.lower);
    p.equal=sparse(0,n);p.rhs=zeros(0,1);p.cones=[];p.lower(:)=0;p.upper(:)=0;
    p.upper(column)=limit;p.upper(slack)=Inf;
    p.a=sparse([1,1],[column,slack],[-1,-1],1,n);p.b=-2*limit;
    p.primaryLowerBound=0;
    request=struct('operation','solve','model',model,'anchor',anchor,'problem',p,'source','trustPriorityTest');
end
