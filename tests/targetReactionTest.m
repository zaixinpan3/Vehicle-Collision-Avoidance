classdef targetReactionTest < matlab.unittest.TestCase
    %targetReactionTest Target-reactive feedback policy and capped target reachability.
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (TestMethodSetup)
        function resetController(~)
            collisionAvoidanceController("resetNominalTrajectory");
        end
    end
    methods (Test)
        function sampledReactiveTrajectoriesStayInsideTheJointTube(testCase)
            [ego,target,road,cfg]=localEncounter(30);
            [~,plan,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.assertEqual(problem.metadata.targetReactionStrength,30);
            testCase.verifyTrue(any(problem.prediction.targetGainSequence(:)) ...
                && any(problem.prediction.targetAccelerationGainSequence(:)));
            excess=localSampledExcess(problem,plan,20260924);
            testCase.verifyLessThanOrEqual(excess.joint,1e-9);
            testCase.verifyLessThanOrEqual(excess.record,1e-9);
            testCase.verifyLessThanOrEqual(excess.input,1e-9);
            testCase.verifyLessThanOrEqual(excess.slew,1e-9);
        end

        function reactionShrinksTheRelativeSupportOfLateRecords(testCase)
            [ego,target,road,cfg]=localEncounter(30);
            [~,~,reactive]=collisionAvoidanceController(ego,target,road,cfg,[]);
            collisionAvoidanceController("resetNominalTrajectory");
            cfg.feedbackPrediction.targetReaction.inputWeightScales=Inf;
            [~,~,egoOnly]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.verifyFalse(isfield(egoOnly.prediction,'targetGainSequence'));
            reactiveSupport=localRecordSupport(reactive.program);
            egoOnlySupport=localRecordSupport(egoOnly.program);
            testCase.verifyLessThan(reactiveSupport(end),egoOnlySupport(end));
        end

        function anInheritedFrameAlsoCorrectsByTheTargetDeviation(testCase)
            [ego,target,road,cfg]=localEncounter(30);
            [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.assertEqual(first.metadata.targetReactionStrength,30);
            next=localDisplacedSuccessor(ego,stored,first.model.lane);
            nextTarget=localNextTarget(target,first.prediction.targetNominal(:,2),[.03;-.02;.01;0;.005;0]);
            [command,plan,problem,nextStored]=collisionAvoidanceController(next,nextTarget,road,cfg,stored);
            testCase.verifyTrue(problem.metadata.inheritedFeasibleFamily);
            prediction=problem.prediction;
            deviation=problem.model.encounter.center(1:6)-prediction.targetNominal(:,1);
            correction=prediction.feedbackGainSequence(:,:,1) ...
                *(problem.model.initialEgoState-prediction.nominalInitialState) ...
                +prediction.targetGainSequence(:,:,1)*deviation(1:4) ...
                +prediction.targetAccelerationGainSequence(:,:,1)*deviation(5:6);
            testCase.verifyGreaterThan(norm(prediction.targetGainSequence(:,:,1)*deviation(1:4) ...
                +prediction.targetAccelerationGainSequence(:,:,1)*deviation(5:6)),0);
            testCase.verifyEqual(problem.metadata.feedbackCorrection,correction,AbsTol=1e-14);
            testCase.verifyEqual(command.actuatorInput,plan(:,1)+problem.metadata.feedbackCorrection,AbsTol=0);
            testCase.verifyEqual(nextStored.appliedInput,command.actuatorInput,AbsTol=0);
        end

        function aLargerTargetBoundVoidsTheReactiveFamily(testCase)
            [ego,target,road,cfg]=localEncounter([30,Inf]);
            [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            next=localDisplacedSuccessor(ego,stored,first.model.lane);
            nextTarget=localNextTarget(target,first.prediction.targetNominal(:,2),zeros(6,1));
            nextTarget.targetVelocityInertialErrorBound=1.5*nextTarget.targetVelocityInertialErrorBound;
            [~,~,problem]=collisionAvoidanceController(next,nextTarget,road,cfg,stored);
            testCase.verifyFalse(problem.metadata.inheritedFeasibleFamily);
        end

        function aLargerAccelerationMaximumVoidsTheCarriedFamily(testCase)
            [ego,target,road,cfg]=localEncounter(Inf);
            [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            next=localDisplacedSuccessor(ego,stored,first.model.lane);
            nominal=targetPrediction.finiteFlow(first.model.encounter,cfg.controller.sampleTime);
            nextTarget=localNextTarget(target,nominal(1:6),zeros(6,1));
            [~,~,same]=collisionAvoidanceController(next,nextTarget,road,cfg,stored);
            testCase.verifyTrue(same.metadata.inheritedFeasibleFamily);
            nextTarget.predictionMotion.scalarAccelerationMaximum=3;
            [~,~,problem]=collisionAvoidanceController(next,nextTarget,road,cfg,stored);
            testCase.verifyFalse(problem.metadata.inheritedFeasibleFamily);
        end

        function aReleaseAfterTheReactionContinuesWithoutACorrection(testCase)
            % Curved stationary encounter with 2 m initial lateral error: the
            % reactive admission's gains end before the target is released at
            % hold 64; the released, inherited frames carry no target term.
            report=runExactStateRecursiveFeasibilityScenario(Scenario="stationary",RoadCurvature=0.01, ...
                InitialTrackingError=[2;0;0;0;0],SampleCount=80,DeadlineSeconds=Inf,SearchTimeLimitSeconds=30, ...
                FeedbackPrediction=struct('targetReaction',struct('inputWeightScales',[30,100,Inf])));
            testCase.verifyTrue(report.completed);
            testCase.verifyTrue(any(report.confirmedRelease));
            testCase.verifyTrue(any(cellfun(@(search) isfield(search,'reactionAttempts') ...
                && ~isempty(search.reactionAttempts) && isfinite(search.reactionStrength),report.admissionSearch)));
        end

        function aDeclaredAccelerationMaximumCapsTheReachableSet(testCase)
            encounter=localReachEncounter(2);uncapped=rmfield(encounter,'contract');
            uncapped.contract=rmfield(encounter.contract,'scalarAccelerationMaximum');
            time=0:.05:6;
            [~,capped]=targetPrediction.finiteFlow(encounter,time);
            [~,free]=targetPrediction.finiteFlow(uncapped,time);
            testCase.verifyLessThanOrEqual(max(capped(1:6,:)-free(1:6,:),[],'all'),1e-12);
            testCase.verifyEqual(capped(7:8,:),free(7:8,:));
            % Numerical integration of b(t) = min(r_a + J t, amax + |a0|).
            cap=2+abs(encounter.center(5:6));step=1e-4;fine=0:step:6;
            b=min(encounter.radius(5:6)+encounter.contract.jerkBound.*fine,cap);
            velocity=encounter.radius(3:4)+cumtrapz(fine,b,2);
            position=encounter.radius(1:2)+cumtrapz(fine,velocity,2);
            pick=round(time/step)+1;
            testCase.verifyEqual(capped(1:2,:),position(:,pick),RelTol=1e-6,AbsTol=1e-6);
            testCase.verifyEqual(capped(3:4,:),velocity(:,pick),RelTol=1e-6,AbsTol=1e-6);
            testCase.verifyEqual(capped(5:6,:),b(:,pick),RelTol=1e-12,AbsTol=1e-12);
        end

        function sampledCappedTargetsStayInsideTheReachableSet(testCase)
            encounter=localReachEncounter(2);stream=RandStream('mt19937ar',Seed=7);
            time=0:.05:5;[center,radius]=targetPrediction.finiteFlow(encounter,time);
            excess=-Inf;
            for trial=1:100
                state=localSampleStart(encounter,stream,mod(trial,2)==0);
                for node=1:numel(time)
                    excess=max([excess;abs(state(1:6)-center(1:6,node))-radius(1:6,node)]);
                    if node<numel(time)
                        state=localTargetHold(state,.05,encounter.contract.jerkBound, ...
                            encounter.contract.scalarAccelerationMaximum,stream,mod(trial,2)==0);
                    end
                end
            end
            testCase.verifyLessThanOrEqual(excess,1e-9);
        end
    end
end

function [ego,target,road,cfg]=localEncounter(scales)
% Straight road, stationary target 15 m ahead with jerk 1 m/s^3 and |a| <= 2 m/s^2.
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.05,'horizonSteps',32,'minimumHorizonSteps',1), ...
        'model',struct('lateralDomainRadius',4), ...
        'solver',struct('frameDeadlineSeconds',60,'certificateSearchTimeLimit',60), ...
        'feedbackPrediction',struct('targetReaction',struct('inputWeightScales',scales))));
    road=[-100,0;2000,0];
    ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0, ...
        'controllerStateErrorBound',[.076;.076;.048;.089;.497;.0015], ...
        'perception',struct('time',0,'range',16,'completeWithinRange',true));
    target=struct('trackId',1,'targetPositionInertial',[15;0],'targetVelocityInertial',[0;0], ...
        'targetAccelerationInertial',[0;0],'targetHeadingInertial',0,'targetYawRate',0, ...
        'targetPositionInertialErrorBound',[.1;.1],'targetVelocityInertialErrorBound',[.05;.05], ...
        'targetAccelerationInertialErrorBound',[.01;.01],'targetYawErrorBound',.01, ...
        'targetYawRateErrorBound',.01, ...
        'predictionMotion',struct('kind','finite-sensing-motion-v1','jerkBound',[1;1], ...
            'yawAccelerationBound',0,'scalarAccelerationMaximum',2));
end

function target=localNextTarget(target,nominal,offset)
% A measurement of the next hold: the carried nominal flow plus an offset.
    state=nominal+offset;
    target.targetPositionInertial=state(1:2);target.targetVelocityInertial=state(3:4);
    target.targetAccelerationInertial=state(5:6);
end

function ego=localDisplacedSuccessor(previous,stored,lane)
% A measurement displaced inside the stored successor box.
    state=stored.predictedState(:,2);radius=stored.stateErrorBound(:,2);
    state(2)=state(2)+.5*radius(2);state(3)=state(3)-.5*radius(3);
    [position,heading]=laneGeometry.fromFrenet(state,lane);
    ego=struct('position',position,'yaw',heading,'speed',state(4), ...
        'lateralVelocity',state(5),'yawRate',state(6), ...
        'stateTime',stored.stateTime+stored.identity.configuration.controller.sampleTime, ...
        'heldActuatorInput',stored.appliedInput, ...
        'controllerStateErrorBound',previous.controllerStateErrorBound, ...
        'perception',struct('time',stored.stateTime+stored.identity.configuration.controller.sampleTime, ...
        'range',stored.confirmation.range,'completeWithinRange',true));
end

function support=localRecordSupport(program)
    certificate=program.jointCertificate;support=zeros(numel(certificate.records),1);
    for index=1:numel(certificate.records)
        normal=[cos(certificate.angles(index));sin(certificate.angles(index))];
        support(index)=sum(abs(certificate.records(index).generators.'*normal));
    end
end

function excess=localSampledExcess(problem,plan,seed)
% Simulate the declared plant under u_k = v_k + K_k (xhat - z) + L_k (shat - s0)
% + N_k (ahat - a0), with a jerk- and acceleration-limited target and per-hold
% estimator errors (random and box vertices).
    prediction=problem.prediction;model=problem.model;encounter=model.encounter;
    count=prediction.stageCount;h=model.sampleTime;
    nominal=zeros(6,count+1);
    for node=0:count
        nominal(:,node+1)=prediction.egoStateOffset(:,node+1)+prediction.egoStateMatrix(:,:,node+1)*plan(:);
    end
    targetNominal=prediction.targetNominal;
    egoBound=prediction.estimatorBound;targetBound=prediction.targetEstimatorBound;
    records=problem.program.jointCertificate.records;angles=problem.program.jointCertificate.angles;
    stream=RandStream('mt19937ar',Seed=seed);
    excess=struct('joint',-Inf,'record',-Inf,'input',-Inf,'slew',-Inf);
    for trial=1:60
        vertex=mod(trial,2)==0;
        x=nominal(:,1)+model.initialFrenetErrorBound.*localUnit(stream,6,vertex);
        s=localSampleStart(encounter,stream,vertex);
        previous=zeros(2,1);
        for stage=1:count
            deviation=zeros(2,1);
            if stage>1
                egoEstimate=x+egoBound.*localUnit(stream,6,vertex);
                targetEstimate=s(1:6)+targetBound.*localUnit(stream,6,vertex);
                targetDeviation=targetEstimate-targetNominal(:,stage);
                deviation=prediction.feedbackGainSequence(:,:,stage)*(egoEstimate-nominal(:,stage)) ...
                    +prediction.targetGainSequence(:,:,stage)*targetDeviation(1:4) ...
                    +prediction.targetAccelerationGainSequence(:,:,stage)*targetDeviation(5:6);
            end
            excess.input=max(excess.input,max(abs(deviation)-prediction.feedbackInputSupport(:,stage)));
            excess.slew=max(excess.slew,max(abs(deviation-previous)-prediction.feedbackSlewSupport(:,stage)));
            previous=deviation;
            x=prediction.stageMatrixA(:,:,stage)*x ...
                +prediction.stageMatrixB(:,:,stage)*(plan(:,stage)+deviation)+prediction.stageAffine(:,stage);
            s=localTargetHold(s,h,encounter.contract.jerkBound,encounter.contract.scalarAccelerationMaximum,stream,vertex);
            cell=prediction.cells(stage);
            joint=[x-nominal(:,stage+1);s(1:4)-targetNominal(1:4,stage+1)];
            generators=[cell.generators;cell.targetGenerators];
            for direction=[eye(10),-eye(10),randn(stream,10,8)]
                excess.joint=max(excess.joint,direction.'*joint-sum(abs(direction.'*generators)));
            end
            for index=find([records.stage]==stage)
                normal=[cos(angles(index));sin(angles(index))];
                relative=records(index).positionMap*joint(1:6)-joint(7:8);
                excess.record=max(excess.record,normal.'*relative-sum(abs(records(index).generators.'*normal)));
            end
        end
    end
end

function state=localSampleStart(encounter,stream,vertex)
% A true target state inside the admitted box with |a_i| <= amax/sqrt(2).
    cap=encounter.contract.scalarAccelerationMaximum/sqrt(2);
    state=encounter.center(1:6)+encounter.radius(1:6).*localUnit(stream,6,vertex);
    state(5:6)=min(max(state(5:6),-cap),cap);
end

function state=localTargetHold(state,h,jerk,maximum,stream,vertex)
% One hold of piecewise-constant jerk |j_i| <= J with |a_i| <= amax/sqrt(2).
    cap=maximum/sqrt(2);steps=10;dt=h/steps;
    for step=1:steps
        j=jerk.*localUnit(stream,2,vertex);
        j=min(max(j,(-cap-state(5:6))/dt),(cap-state(5:6))/dt);
        j=min(max(j,-jerk),jerk);
        state(1:2)=state(1:2)+state(3:4)*dt+state(5:6)*dt^2/2+j*dt^3/6;
        state(3:4)=state(3:4)+state(5:6)*dt+j*dt^2/2;
        state(5:6)=state(5:6)+j*dt;
    end
end

function value=localUnit(stream,count,vertex)
    if vertex
        value=sign(randn(stream,count,1));
    else
        value=2*rand(stream,count,1)-1;
    end
end

function encounter=localReachEncounter(maximum)
    encounter=struct('center',[3;-2;4;1;.3;-.2;0;0],'radius',[.1;.1;.05;.05;.01;.01;.01;.01], ...
        'contract',struct('jerkBound',[1;.7],'yawAccelerationBound',0,'scalarAccelerationMaximum',maximum));
end
