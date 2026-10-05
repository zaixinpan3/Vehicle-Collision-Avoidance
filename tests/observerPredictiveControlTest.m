classdef observerPredictiveControlTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function constantParameterTubeContainsExactTurningAndAcceleratingPaths(testCase)
            [positionExcess,yawExcess]=localTubeSamples();
            testCase.verifyLessThanOrEqual(positionExcess,1e-12);
            testCase.verifyLessThanOrEqual(yawExcess,1e-12);
        end
        function uncertaintyDoesNotContractWithoutFutureMeasurements(testCase)
            q=[0;0;.2;8;.3;.02;1.6;2.4;.95;0;0];
            set=struct('positionRadius',.03,'courseRadius',.01,'speedRadius',.1, ...
                'accelerationRadius',.02,'curvatureRadius',.001,'sideslipRadius',.002);
            tube=predictiveSafetyGeometry.targetErrorTube(q,set,0:.1:4);
            testCase.verifyGreaterThanOrEqual(diff(tube.positionRadius),0);
            testCase.verifyGreaterThan(tube.positionRadius(end),tube.positionRadius(1));
        end
        function largerTargetEnclosuresActuallyChangeTheAvoidanceOptimization(testCase)
            [ego,target,road,cfg]=localFixture(.01);
            [~,smallInputs,small]=collisionAvoidanceController(ego,target,road,cfg,[]);
            target.predictionErrorSet.positionRadius=.10;
            target.controllerErrorBound.bounds(1:2)=.10;
            [~,largeInputs,large]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyGreaterThan(large.metadata.maximumCollisionTighteningMeters, ...
                small.metadata.maximumCollisionTighteningMeters+.08);
            testCase.verifyGreaterThan(norm(largeInputs-smallInputs,'fro'),1e-5);
            testCase.verifyTrue(large.metadata.uncertaintyIncluded);
            testCase.verifyFalse(large.metadata.robustNonlinearSafetyCertified);
            testCase.verifyEqual(large.metadata.safetyScope,"hardNominalGeometryWithRelaxedObserverMargins");
        end
        function zeroRadiusEnclosuresRecoverTheExactStateProblem(testCase)
            [ego,target,road,cfg]=localFixture(0);
            exactEgo=rmfield(ego,'controllerErrorBound');
            exactTarget=rmfield(target,{'controllerErrorBound','predictionErrorSet'});
            [~,exactInputs]=collisionAvoidanceController(exactEgo,exactTarget,road,cfg,[]);
            [~,boundedInputs,bounded]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyEqual(boundedInputs,exactInputs,AbsTol=1e-9);
            testCase.verifyEqual(bounded.metadata.maximumCollisionTighteningMeters,0,AbsTol=1e-12);
        end
        function commonGlobalPoseUncertaintyCancelsOnlyFromCollisionGeometry(testCase)
            [raw,target,road,cfg]=localFixture(.01);
            raw.controllerErrorBound.bounds(1:3)=[.3;.3;.1];
            [ego,~,~,observation]=readControllerInputs(raw,target,road,cfg);
            q=predictiveSafetyGeometry.target(observation,target,cfg);
            set=predictiveSafetyGeometry.observerUncertainty(ego,observation,q);
            testCase.verifyEqual(set.collisionGenerator(1:3,:),zeros(3,6),AbsTol=0);
            testCase.verifyEqual(diag(set.egoGenerator),raw.controllerErrorBound.bounds,AbsTol=0);
            testCase.verifyEqual(set.target.positionRadius,.01,AbsTol=0);
            testCase.verifyTrue(set.relativeFrame);
        end
        function estimatedConstantParametersCanUpdateWithoutDiscardingTheInputWarmStart(testCase)
            [ego,target,road,cfg]=localFixture(.001);
            target.predictionErrorSet.accelerationInterval=[-.002;.002];
            target.controllerErrorBound.bounds(5:6)=.002;
            [command,~,first,prior]=collisionAvoidanceController(ego,target,road,cfg,[]);
            next=nonlinearBicycleModel.sample(first.model.initialState,command.actuatorInput,cfg);
            q=predictiveSafetyGeometry.predictTarget(first.model.target,cfg.controller.sampleTime);
            q(1)=q(1)+.001;q(5)=q(5)+.001;
            [ego,target]=localPublication(next,q,cfg.controller.sampleTime,.001,zeros(6,1));
            % Both frames contain the same true A=0 despite a shifted estimate.
            target.predictionErrorSet.accelerationInterval=q(5)+[-.002;.002];
            target.controllerErrorBound.bounds(5:6)=.002;
            ego.heldActuatorInput=command.actuatorInput;
            [~,~,second]=collisionAvoidanceController(ego,target,road,cfg,prior);
            testCase.verifyEqual(second.model.target,q,AbsTol=1e-12);
            testCase.verifyEqual(second.metadata.search.initialization,"shiftedInputRollout");
            testCase.verifyEqual([second.metadata.search.stages.objective],["pcbfSlack","clfSlack"]);
            testCase.verifyEqual(second.model.linearization.states(:,1),next,AbsTol=1e-12);
        end
        function theSingleClfActsOnTheEstimateWithoutAnEnclosureBudget(testCase)
            % The CLF row is V(xhat+) <= rho V(xhat) + slack with or without
            % an observer enclosure: estimation error enters as an input.
            [ego,~,road,cfg]=localFixture(0);
            ego.position(2)=.1;
            exactEgo=rmfield(ego,'controllerErrorBound');
            ego.controllerErrorBound.bounds=[.001;.001;1e-4;1e-6;1e-6;1e-6];
            [~,~,prediction]=collisionAvoidanceController(ego,[],road,cfg,[]);
            [~,~,exact]=collisionAvoidanceController(exactEgo,[],road,cfg,[]);
            testCase.verifyTrue(prediction.metadata.uncertaintyIncluded);
            testCase.verifyEqual(prediction.metadata.clfCurrentBudget, ...
                prediction.metadata.clfContraction*prediction.metadata.clfInitialValue,RelTol=1e-12);
            testCase.verifyEqual(prediction.metadata.clfCurrentBudget,exact.metadata.clfCurrentBudget,RelTol=1e-12);
            testCase.verifyEqual(prediction.metadata.clfWorstNextValue,prediction.metadata.clfNextValue,RelTol=1e-12);
            testCase.verifyEqual(prediction.metadata.clfFunction,"quadraticTransverseError");
            model=prediction.model;
            scales=[cfg.clf.lateralPositionErrorScale;cfg.clf.headingErrorScale;cfg.clf.speedErrorScale; ...
                cfg.clf.lateralVelocityErrorScale;cfg.clf.yawRateErrorScale];
            loss=sum((nonlinearBicycleModel.error(model.initialState,model.lane,model.nominalReference)./scales).^2);
            testCase.verifyLessThanOrEqual(prediction.metadata.clfTieBound,cfg.solver.clfTieTolerance*loss+1e-12);
        end
        function unavailableBoundsDoNotPreventASolvedControl(testCase)
            [ego,target,road,cfg]=localFixture(.01);
            ego.controllerErrorBound.available=false;
            ego.controllerErrorBound.bounds(:)=Inf;
            [command,~,prediction]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyFalse(prediction.metadata.uncertaintyIncluded);
            testCase.verifyFalse(prediction.metadata.robustNonlinearSafetyCertified);
        end
        function feedbackErrorIsCarriedToTheEndpoint(testCase)
            [ego,~,road,cfg]=localFixture(0);
            ego.controllerErrorBound.bounds(4)=.001;
            [command,~,prediction]=collisionAvoidanceController(ego,[],road,cfg,[]);
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyGreaterThan(prediction.model.uncertaintyPrediction.egoStateRadius(4,end),0);
            testCase.verifyFalse(prediction.metadata.robustNonlinearSafetyCertified);
        end
        function directionalTargetSupportContainsTheConstantParameterFamily(testCase)
            q=[0;0;.2;8;.3;.02;1.6;2.4;.95;0;0];
            set=struct('positionRadius',.03,'courseRadius',.04,'speedRadius',.4, ...
                'accelerationRadius',.1,'curvatureRadius',.002,'sideslipRadius',.004);
            directions=[cos(linspace(0,2*pi,17));sin(linspace(0,2*pi,17))].';
            for beta=[0,.02]
                q(6)=beta;
                for time=[-.5,.2,1,4]
                    nominal=predictiveSafetyGeometry.predictTarget(q,time);
                    bound=predictiveSafetyGeometry.targetPositionSupport(q,set,time,directions);
                    for bits=0:15
                        signs=2*double(bitget(bits,1:4)).'-1;
                        member=q;member(4:5)=q(4:5)+signs(1:2).*[set.speedRadius;set.accelerationRadius];
                        member(6)=asin((sin(beta)/q(7)+signs(3)*set.curvatureRadius)*q(7));
                        member(3)=q(3)+beta+signs(4)*set.courseRadius-member(6);
                        actual=predictiveSafetyGeometry.predictTarget(member,time);
                        testCase.verifyLessThanOrEqual(abs(directions*(actual(1:2)-nominal(1:2))) ...
                            +set.positionRadius,bound+1e-10);
                    end
                end
            end
        end
        function longitudinalTargetSpeedUncertaintyDoesNotBecomeLateralWidth(testCase)
            q=[0;0;0;8;0;0;1.6;2.4;.95;0;0];
            set=struct('positionRadius',.03,'courseRadius',0,'speedRadius',2, ...
                'accelerationRadius',1,'curvatureRadius',0,'sideslipRadius',0);
            support=predictiveSafetyGeometry.targetPositionSupport(q,set,3,eye(2));
            testCase.verifyEqual(support,[10.53;.03],AbsTol=1e-12);
        end
        function staleBoundsCannotBeUsedAtTheCurrentStateTime(testCase)
            [ego,target,road,cfg]=localFixture(.01);
            target.controllerErrorBound.time=-cfg.controller.sampleTime;
            [command,~,prediction]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyEmpty(prediction.model.uncertainty.target);
        end
        function aRelativeSetCannotBeAttachedToADifferentEgoPose(testCase)
            [ego,target,road,cfg]=localFixture(.01);
            target.predictionErrorSet.referenceEgoPose(1)=1;
            [command,~,prediction]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyFalse(prediction.metadata.commonPoseCancelled);
            testCase.verifyEmpty(prediction.model.uncertainty.target);
        end
        function lossOfAnEnclosureDoesNotDiscardTheShiftedOptimization(testCase)
            [ego,target,road,cfg]=localFixture(.001);
            [command,~,first,prior]=collisionAvoidanceController(ego,target,road,cfg,[]);
            next=nonlinearBicycleModel.sample(first.model.initialState,command.actuatorInput,cfg);
            q=predictiveSafetyGeometry.predictTarget(first.model.target,cfg.controller.sampleTime);
            [ego,target]=localPublication(next,q,cfg.controller.sampleTime,.001,zeros(6,1));
            ego.controllerErrorBound.available=false;target.predictionErrorSet.available=false;
            ego.heldActuatorInput=command.actuatorInput;
            [command,~,second]=collisionAvoidanceController(ego,target,road,cfg,prior);
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyEqual(second.metadata.search.initialization,"shiftedInputRollout");
            testCase.verifyFalse(second.metadata.robustNonlinearSafetyCertified);
        end
        function aMissingTargetPredictionSetDoesNotStopTheController(testCase)
            [ego,target,road,cfg]=localFixture(.01);
            target=rmfield(target,'predictionErrorSet');
            [command,~,prediction]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyTrue(all(isfinite(command.actuatorInput)));
            testCase.verifyEmpty(prediction.model.uncertainty.target);
        end
    end
end

function [ego,target,road,cfg]=localFixture(radius)
    [x,q,road,cfg]=collisionThreatScenario("headOn",struct('referenceSpeed',8,'controller',struct('horizonSteps',8)));
    [ego,target]=localPublication(x,q,0,radius,zeros(6,1));
end

function [ego,target]=localPublication(x,q,time,radius,egoBounds)
    ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6), ...
        'stateTime',time,'heldActuatorInput',zeros(2,1),'controllerErrorBound',localCertificate("ego-state-v1",time,egoBounds));
    direction=[cos(q(3)+q(6));sin(q(3)+q(6))];
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',q(4)*direction, ...
        'targetAccelerationInertial',q(5)*direction+q(4)^2*sin(q(6))/q(7)*[-direction(2);direction(1)], ...
        'targetYawInertial',q(3),'targetSideslip',q(6),'targetTangentialAcceleration',q(5), ...
        'targetRearAxleDistance',q(7),'targetLength',2*q(8),'targetWidth',2*q(9),'targetRectangleOffset',q(10:11), ...
        'controllerErrorBound',localCertificate("target-state-v1",time,[radius;radius;zeros(6,1)]));
    rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
    target.predictionErrorSet=struct('kind',"nrmm-constant-parameter-set-v1",'time',time,'available',true, ...
        'referenceEgoPose',x(1:3),'relativePosition',rotation.'*(q(1:2)-x(1:2)), ...
        'positionRadius',radius,'courseCenter',q(3)+q(6)-x(3),'courseRadius',0, ...
        'speedInterval',q(4)*ones(2,1),'accelerationInterval',q(5)*ones(2,1), ...
        'curvatureInterval',sin(q(6))/q(7)*ones(2,1),'rearAxleDistance',q(7));
end

function certificate=localCertificate(kind,time,bounds)
    certificate=struct('kind',kind,'time',time,'bounds',bounds,'available',true);
end

function [positionExcess,yawExcess]=localTubeSamples()
    q=[2;-1;.4-asin(1.6*.02);8;.3;asin(1.6*.02);1.6;2.4;.95;0;0];
    betaRadius=max(abs(asin(1.6*(.02+[-.004;.004]))-q(6)));
    set=struct('positionRadius',.1,'courseRadius',.03,'speedRadius',.2, ...
        'accelerationRadius',.1,'curvatureRadius',.004,'sideslipRadius',betaRadius);
    time=0:.05:3;nominal=predictiveSafetyGeometry.predictTarget(q,time);
    tube=predictiveSafetyGeometry.targetErrorTube(q,set,time);
    [s1,s2,s3,s4]=ndgrid([-1,0,1]);samples=[s1(:),s2(:),s3(:),s4(:)];
    positionExcess=-Inf;yawExcess=-Inf;
    for row=samples.'
        actual=q;actual(1:2)=q(1:2)+.1*[cos(row(1));sin(row(1))];
        actual(4)=q(4)+row(1)*.2;actual(5)=q(5)+row(2)*.1;
        actual(6)=asin(1.6*(.02+row(3)*.004));actual(3)=.4+row(4)*.03-actual(6);
        truth=predictiveSafetyGeometry.predictTarget(actual,time);
        positionExcess=max(positionExcess,max(vecnorm(truth(1:2,:)-nominal(1:2,:))-tube.positionRadius));
        yawExcess=max(yawExcess,max(abs(atan2(sin(truth(3,:)-nominal(3,:)),cos(truth(3,:)-nominal(3,:))))-tube.yawRadius));
    end
end

