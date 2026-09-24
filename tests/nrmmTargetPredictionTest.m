classdef nrmmTargetPredictionTest < matlab.unittest.TestCase
    %nrmmTargetPredictionTest NRMM target reachability and its reactive joint tube.
    properties (TestParameter)
        % [speed; course; speed-rate; curvature] of the estimated NRMM path.
        motion = struct('turning',[6;0.3;0.4;0.02],'reversedCurvature',[4;1.2;0;-0.03], ...
            'braking',[12;-0.5;-0.8;0.01],'nearlyStopped',[0.05;0.2;0.05;0],'stopping',[2;0;-0.6;0.04]);
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
        end
    end
    methods (TestMethodSetup)
        function resetController(~)
            collisionAvoidanceController("resetNominalTrajectory");
        end
    end
    methods (Test)
        function sampledNrmmPathsStayInsideBothEnclosures(testCase,motion)
            encounter=localPathEncounter(motion,[.2;.2;.1;.1;.1;.1;.02;.02],.05);
            time=[0:.1:4.8,3.3,3.35];stream=RandStream('mt19937ar',Seed=11);
            [center,box]=targetPrediction.finiteFlow(encounter,time);
            model=targetPrediction.deviationModel(encounter,time);
            boxExcess=-Inf;linearExcess=-Inf;
            for trial=1:300
                truth=localSampleNrmm(encounter,stream,mod(trial,3)==0);
                for node=1:numel(time)
                    state=localNrmmState(truth,time(node));
                    deviation=state-center(:,node);
                    deviation(7)=atan2(sin(deviation(7)),cos(deviation(7)));
                    boxExcess=max([boxExcess;abs(deviation)-box(:,node)]);
                    deviation=state(1:6)-model.center(1:6,node);
                    generators=model.parameterGenerators(:,:,node);
                    for direction=[eye(6),-eye(6),randn(stream,6,4)]
                        linearExcess=max(linearExcess,direction.'*deviation ...
                            -sum(abs(direction.'*generators))-abs(direction).'*model.remainder(:,node));
                    end
                end
            end
            testCase.verifyLessThanOrEqual(boxExcess,1e-9);
            testCase.verifyLessThanOrEqual(linearExcess,1e-9);
            testCase.verifyEqual(model.holdBound,zeros(2,numel(time)));
        end

        function thePathLengthBallCoversAPossibleStop(testCase)
            % Braking at 0.6 m/s^2 from 2 m/s may stop after 3.3 s; from then on
            % only the initial position keeps its generators.
            encounter=localPathEncounter([2;0;-.6;.04],[.2;.2;.1;.1;.1;.1;.02;.02],.05);
            model=targetPrediction.deviationModel(encounter,[1,4.8]);
            testCase.verifyTrue(any(model.parameterGenerators(:,3:6,1),'all'));
            testCase.verifyFalse(any(model.parameterGenerators(:,3:6,2),'all'));
            testCase.verifyEqual(model.parameterGenerators(1:2,1:2,2),diag([.2;.2]));
            slow=localPathEncounter([.05;.2;.05;0],[.2;.2;.1;.1;.1;.1;.02;.02],.05);
            model=targetPrediction.deviationModel(slow,0:.1:4.8);
            testCase.verifyFalse(any(model.parameterGenerators(:,3:6,:),'all'));
        end

        function theNrmmBoxIsTighterThanTheCartesianTurningJerkBox(testCase)
            % A 10 m/s target of the declared domain (|kappa| <= 0.03, |A| <= 1):
            % the Cartesian contract must cover the jerk of NRMM turning,
            % hypot(V*omega^2, 3*A*omega), and grows with t^3/6.
            encounter=localPathEncounter([10;0;0;.02],[.1;.1;.05;.05;.01;.01;.01;.01],.03);
            cartesian=encounter;
            jerk=hypot(10*.3^2,3*1*.3);
            cartesian.contract=struct('kind',"finite-sensing-motion-v1",'jerkBound',[jerk;jerk], ...
                'yawAccelerationBound',.03,'scalarAccelerationMaximum',1);
            time=[2,4.8];
            [~,nrmm]=targetPrediction.finiteFlow(encounter,time);
            [~,box]=targetPrediction.finiteFlow(cartesian,time);
            testCase.verifyLessThan(max(nrmm(1:2,:)),max(box(1:2,:)));
            testCase.verifyLessThan(max(nrmm(1:2,2)),max(box(1:2,2))/10);
        end

        function aSlowTargetKeepsTheCartesianEnclosureOfItsTurning(testCase)
            % At 4 m/s with ten-fold estimate errors the NRMM parameter enclosure
            % is wider than the Cartesian box whose jerk covers the turning; the
            % flow keeps the tighter of the two on every axis.
            encounter=localPathEncounter([4;pi/2;0;0],[1;1;.5;.5;.1;.1;.1;.1],.03);
            time=0:.1:4.8;
            [~,box]=targetPrediction.finiteFlow(encounter,time);
            model=targetPrediction.deviationModel(encounter,time);
            nrmm=reshape(sum(abs(model.parameterGenerators),2),6,[])+model.remainder;
            speed=4+norm([.5;.5]);rate=norm([.1;.1]);
            cartesian=encounter;jerk=hypot(.03^2*(speed+rate*4.8)^3,3*rate*.03*(speed+rate*4.8));
            cartesian.contract=struct('kind',"finite-sensing-motion-v1",'jerkBound',[jerk;jerk], ...
                'yawAccelerationBound',rate*.03);
            [~,reference]=targetPrediction.finiteFlow(cartesian,time);
            testCase.verifyLessThanOrEqual(box-reference,1e-12);
            testCase.verifyLessThanOrEqual(box(1:6,:)-nrmm,1e-12);
            testCase.verifyLessThan(max(box(1:2,end)),.7*max(nrmm(1:2,end)));
        end

        function aTargetThatMayBeAtRestKeepsTheCartesianBox(testCase)
            % A target estimated at rest may stop at any time; the stop is an
            % acceleration jump to zero, covered because its estimate is zero.
            radius=[1;1;.5;.5;.1;.1;.1;.1];
            encounter=localPathEncounter([0;0;0;0],radius,.03);
            time=0:.1:4.8;
            [~,box]=targetPrediction.finiteFlow(encounter,time);
            speed=.71+.15*time;jerk=hypot(.03^2*speed.^3,3*.15*.03*speed);
            reference=radius(1:2)+radius(3:4).*time+radius(5:6).*time.^2/2+jerk.*time.^3/6;
            testCase.verifyLessThanOrEqual(box(1:2,:)-reference,1e-12);
        end

        function aContradictoryYawRateIsAnInconsistentObservation(testCase)
            encounter=localPathEncounter([10;0;0;.02],[.1;.1;.05;.05;.01;.01;.01;.01],.05);
            encounter.center(8)=1;
            testCase.verifyError(@() targetPrediction.finiteFlow(encounter,1), ...
                "collisionAvoidanceController:inconsistentObservation");
        end

        function anNrmmContractCannotCarryAModelError(testCase)
            [ego,target,road,cfg]=localLeadEncounter(Inf);
            target.predictionMotion.jerkBound=[.1;.1];
            testCase.verifyError(@() collisionAvoidanceController(ego,target,road,cfg,[]), ...
                "collisionAvoidanceController:invalidEncounterContract");
        end

        function sampledReactiveNrmmTrajectoriesStayInsideTheJointTube(testCase)
            % The reactive policy of the admitted directions, closed on NRMM
            % truths and per-hold estimator errors on the declared plant.
            [ego,target,road,cfg]=localLeadEncounter(Inf);
            [~,plan,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.assertTrue(problem.metadata.planCertified);
            reactive=localReactiveProgram(problem,30);
            testCase.assertTrue(any(reactive.prediction.targetGainSequence(:)));
            excess=localSampledExcess(reactive,problem.model,plan,20260924);
            testCase.verifyLessThanOrEqual(excess.joint,1e-9);
            testCase.verifyLessThanOrEqual(excess.record,1e-9);
            testCase.verifyLessThanOrEqual(excess.input,1e-9);
            testCase.verifyLessThanOrEqual(excess.slew,1e-9);
        end

        function aMotionModelChangeVoidsTheCarriedFamily(testCase)
            [ego,target,road,cfg]=localLeadEncounter(Inf);
            [~,~,first,stored]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.assertTrue(first.metadata.planCertified);
            next=localSuccessor(ego,stored);
            nominal=targetPrediction.finiteFlow(first.model.encounter,cfg.controller.sampleTime);
            nextTarget=target;
            nextTarget.targetPositionInertial=nominal(1:2);nextTarget.targetVelocityInertial=nominal(3:4);
            nextTarget.targetAccelerationInertial=nominal(5:6);
            nextTarget.targetHeadingInertial=nominal(7);nextTarget.targetYawRate=nominal(8);
            [~,~,same]=collisionAvoidanceController(next,nextTarget,road,cfg,stored);
            testCase.verifyTrue(same.metadata.inheritedFeasibleFamily);
            wider=nextTarget;wider.predictionMotion.curvatureMaximum=.05;
            [~,~,problem]=collisionAvoidanceController(next,wider,road,cfg,stored);
            testCase.verifyFalse(problem.metadata.inheritedFeasibleFamily);
            cartesian=nextTarget;
            cartesian.predictionMotion=struct('kind',"finite-sensing-motion-v1",'jerkBound',[0;0], ...
                'yawAccelerationBound',0);
            [~,~,problem]=collisionAvoidanceController(next,cartesian,road,cfg,stored);
            testCase.verifyFalse(problem.metadata.inheritedFeasibleFamily);
        end
    end
end

function encounter=localPathEncounter(parameters,radius,curvatureMaximum)
% Estimate box centred on the NRMM state [1; 2] + path(parameters) at t = 0.
    truth=struct('p0',[1;2],'speed',parameters(1),'course',parameters(2),'A',parameters(3), ...
        'kappa',parameters(4),'yaw0',parameters(2));
    encounter=struct('center',localNrmmState(truth,0),'radius',radius, ...
        'contract',struct('kind',"nrmm-motion-v1",'jerkBound',[0;0],'yawAccelerationBound',0, ...
        'curvatureMaximum',curvatureMaximum));
end

function truth=localSampleNrmm(encounter,stream,extreme)
% NRMM parameters whose state at t = 0 lies in the admitted box; extreme draws
% put the position, velocity, yaw and yaw rate at box vertices (uniform draws
% after 200 vertices without an NRMM-consistent acceleration).
    x=encounter.center;r=encounter.radius;maximum=encounter.contract.curvatureMaximum;
    for attempt=1:50000
        vertex=extreme && attempt<=200;
        if vertex,u=sign(randn(stream,8,1));else,u=2*rand(stream,8,1)-1;end
        p0=x(1:2)+r(1:2).*u(1:2);v=x(3:4)+r(3:4).*u(3:4);speed=norm(v);
        if speed==0,continue;end
        kappa=(x(8)+r(8)*u(8))/speed;
        if abs(kappa)>maximum,continue;end
        course=atan2(v(2),v(1));tangent=[cos(course);sin(course)];
        normal=kappa*speed^2*[-tangent(2);tangent(1)];
        % Speed-rates A with A*tangent + normal inside the acceleration box.
        low=-Inf;high=Inf;feasible=true;
        for axis=1:2
            lower=x(4+axis)-r(4+axis)-normal(axis);upper=x(4+axis)+r(4+axis)-normal(axis);
            if abs(tangent(axis))<1e-12
                feasible=feasible && lower<=0 && upper>=0;
            else
                bounds=sort([lower,upper]/tangent(axis));low=max(low,bounds(1));high=min(high,bounds(2));
            end
        end
        if ~feasible || low>high,continue;end
        if vertex,A=low+(high-low)*(randn(stream)>0);else,A=low+(high-low)*rand(stream);end
        truth=struct('p0',p0,'speed',speed,'course',course,'A',A,'kappa',kappa,'yaw0',x(7)+r(7)*u(7));
        return;
    end
    error('nrmmTargetPredictionTest:noSample','No NRMM state found in the box.');
end

function state=localNrmmState(truth,t)
% Exact NRMM state [p; v; a; yaw; yaw rate] at time t; stops and holds at zero speed.
    moving=t;
    if truth.A<0,moving=min(t,truth.speed/-truth.A);end
    arc=truth.speed*moving+truth.A*moving^2/2;rate=max(0,truth.speed+truth.A*moving);
    heading=truth.course+truth.kappa*arc;tangent=[cos(heading);sin(heading)];
    half=truth.kappa*arc/2;scale=1;
    if abs(half)>1e-9,scale=sin(half)/half;end
    acceleration=truth.A*tangent+truth.kappa*rate^2*[-tangent(2);tangent(1)];
    if rate==0,acceleration=zeros(2,1);end
    state=[truth.p0+arc*scale*[cos(truth.course+half);sin(truth.course+half)];rate*tangent;acceleration; ...
        truth.yaw0+truth.kappa*arc;truth.kappa*rate];
end

function [ego,target,road,cfg]=localLeadEncounter(scales)
% Straight road; a 2 m/s target 15 m ahead on a 50 m-radius NRMM arc.
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'controller',struct('sampleTime',.05,'horizonSteps',32,'minimumHorizonSteps',1), ...
        'model',struct('lateralDomainRadius',4), ...
        'solver',struct('frameDeadlineSeconds',60,'certificateSearchTimeLimit',60), ...
        'feedbackPrediction',struct('targetReaction',struct('inputWeightScales',scales))));
    road=[-100,0;2000,0];
    ego=struct('position',[0;0],'yaw',0,'speed',8,'stateTime',0, ...
        'controllerStateErrorBound',[.076;.076;.048;.089;.497;.0015], ...
        'perception',struct('time',0,'range',16,'completeWithinRange',true));
    target=struct('trackId',1,'targetPositionInertial',[15;0],'targetVelocityInertial',[2;0], ...
        'targetAccelerationInertial',[0;.08],'targetHeadingInertial',0,'targetYawRate',.04, ...
        'targetPositionInertialErrorBound',[.1;.1],'targetVelocityInertialErrorBound',[.05;.05], ...
        'targetAccelerationInertialErrorBound',[.02;.02],'targetYawErrorBound',.01, ...
        'targetYawRateErrorBound',.01, ...
        'predictionMotion',struct('kind','nrmm-motion-v1','jerkBound',[0;0], ...
            'yawAccelerationBound',0,'curvatureMaximum',.03));
end

function program=localReactiveProgram(problem,strength)
% The accepted directions on the target-reactive tube of the given strength.
    model=problem.model;records=problem.program.jointCertificate.records;
    angles=problem.program.jointCertificate.angles;
    design=ltvBicycleModel.reactionGains(model,problem.program.prediction,records,angles, ...
        ones(numel(records),1),strength);
    model.reactionDesign=design;
    program=avoidanceSafetyGeometry.setDirections(formulateAvoidanceProblem(model),angles);
end

function ego=localSuccessor(previous,stored)
% The stored successor state, measured exactly one hold later.
    state=stored.predictedState(:,2);
    [position,heading]=laneGeometry.fromFrenet(state,stored.identity.lane);
    h=stored.identity.configuration.controller.sampleTime;
    ego=struct('position',position,'yaw',heading,'speed',state(4), ...
        'lateralVelocity',state(5),'yawRate',state(6),'stateTime',stored.stateTime+h, ...
        'heldActuatorInput',stored.appliedInput, ...
        'controllerStateErrorBound',previous.controllerStateErrorBound, ...
        'perception',struct('time',stored.stateTime+h,'range',stored.confirmation.range,'completeWithinRange',true));
end

function excess=localSampledExcess(program,model,plan,seed)
% Simulate the declared plant under u_k = v_k + K_k (xhat - z) + L_k (shat - s0)
% + N_k (ahat - a0) with exact NRMM targets and per-hold estimator errors.
    prediction=program.prediction;encounter=model.encounter;
    count=prediction.stageCount;h=model.sampleTime;
    nominal=zeros(6,count+1);
    for node=0:count
        nominal(:,node+1)=prediction.egoStateOffset(:,node+1)+prediction.egoStateMatrix(:,:,node+1)*plan(:);
    end
    targetNominal=prediction.targetNominal;
    egoBound=prediction.estimatorBound;targetBound=prediction.targetEstimatorBound;
    records=program.jointCertificate.records;angles=program.jointCertificate.angles;
    stream=RandStream('mt19937ar',Seed=seed);
    excess=struct('joint',-Inf,'record',-Inf,'input',-Inf,'slew',-Inf);
    for trial=1:60
        vertex=mod(trial,2)==0;
        x=nominal(:,1)+model.initialFrenetErrorBound.*localUnit(stream,6,vertex);
        truth=localSampleNrmm(encounter,stream,vertex);
        previous=zeros(2,1);
        for stage=1:count
            deviation=zeros(2,1);
            if stage>1
                egoEstimate=x+egoBound.*localUnit(stream,6,vertex);
                s=localNrmmState(truth,(stage-1)*h);
                targetDeviation=s(1:6)+targetBound.*localUnit(stream,6,vertex)-targetNominal(:,stage);
                deviation=prediction.feedbackGainSequence(:,:,stage)*(egoEstimate-nominal(:,stage)) ...
                    +prediction.targetGainSequence(:,:,stage)*targetDeviation(1:4) ...
                    +prediction.targetAccelerationGainSequence(:,:,stage)*targetDeviation(5:6);
            end
            excess.input=max(excess.input,max(abs(deviation)-prediction.feedbackInputSupport(:,stage)));
            excess.slew=max(excess.slew,max(abs(deviation-previous)-prediction.feedbackSlewSupport(:,stage)));
            previous=deviation;
            x=prediction.stageMatrixA(:,:,stage)*x ...
                +prediction.stageMatrixB(:,:,stage)*(plan(:,stage)+deviation)+prediction.stageAffine(:,stage);
            s=localNrmmState(truth,stage*h);
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

function value=localUnit(stream,count,vertex)
    if vertex
        value=sign(randn(stream,count,1));
    else
        value=2*rand(stream,count,1)-1;
    end
end
