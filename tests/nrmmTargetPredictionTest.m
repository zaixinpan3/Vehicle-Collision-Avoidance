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
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
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
                truth=nrmmTruthFixture.sampleBox(encounter,stream,mod(trial,3)==0);
                for node=1:numel(time)
                    state=nrmmTruthFixture.state(truth,time(node));
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
            testCase.verifyTrue(all(isfinite([center;box]),'all'));
            testCase.verifyTrue(all(isfinite(model.remainder),'all') && all(isfinite(model.parameterGenerators),'all'));
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
            [center,box]=targetPrediction.finiteFlow(slow,0:.1:4.8);
            testCase.verifyTrue(all(isfinite([center;box;model.remainder]),'all'));
        end

        function aDeclaredSpeedRateMaximumClipsTheParametersAndTheBox(testCase)
            % The box implies speed-rates within +-0.028 m/s^2 (the acceleration
            % radius plus the normal acceleration turned by the course error);
            % a declared maximum below that clips the interval and shrinks the
            % reachable box by the clipped error times t^2/2, and truths within
            % the maximum stay inside.
            free=localPathEncounter([10;0;0;.02],[.1;.1;.05;.05;.01;.01;.01;.01],.05);
            capped=free;capped.contract.speedRateMaximum=.01;
            capped.parameters=targetPrediction.nrmmParameters(capped.center,capped.radius,capped.contract,[]);
            testCase.verifyGreaterThan(diff(free.parameters.speedRate),.05);
            testCase.verifyEqual(capped.parameters.speedRate,[-.01;.01],AbsTol=1e-12);
            time=[1,4.8];
            [~,freeBox]=targetPrediction.finiteFlow(free,time);
            [center,cappedBox]=targetPrediction.finiteFlow(capped,time);
            testCase.verifyLessThan(cappedBox(1:6,:),freeBox(1:6,:)+1e-12);
            testCase.verifyLessThan(max(cappedBox(1:2,2)),max(freeBox(1:2,2))-.15);
            stream=RandStream('mt19937ar',Seed=5);excess=-Inf;
            for trial=1:200
                truth=nrmmTruthFixture.sampleBox(capped,stream,mod(trial,2)==0);
                testCase.assertLessThanOrEqual(abs(truth.A),.01+1e-12);
                for node=1:numel(time)
                    deviation=nrmmTruthFixture.state(truth,time(node))-center(:,node);
                    deviation(7)=atan2(sin(deviation(7)),cos(deviation(7)));
                    excess=max([excess;abs(deviation)-cappedBox(:,node)]);
                end
            end
            testCase.verifyLessThanOrEqual(excess,1e-9);
        end

        function publishedParameterBoundsTightenTheBoxAndStayValid(testCase)
            % A 4 m/s target with ten-fold errors. The declarer's velocity and
            % acceleration errors are discs of the box radii: the published
            % speed, course, speed-rate and curvature bounds are tighter than
            % the box corners, and every NRMM truth in the discs stays inside.
            radius=[1;1;.5;.5;.1;.1;.1;.1];
            boxOnly=localPathEncounter([4;pi/2;0;0],radius,.03);
            published=localBallBounds(boxOnly.center,.5,.1);
            encounter=boxOnly;
            encounter.parameters=targetPrediction.nrmmParameters(encounter.center,radius,encounter.contract,published);
            for name=["speed","course","speedRate","curvature"]
                testCase.verifyGreaterThanOrEqual(encounter.parameters.(name)(1),boxOnly.parameters.(name)(1)-1e-12);
                testCase.verifyLessThanOrEqual(encounter.parameters.(name)(2),boxOnly.parameters.(name)(2)+1e-12);
            end
            testCase.verifyLessThan(diff(encounter.parameters.speed),.8*diff(boxOnly.parameters.speed));
            testCase.verifyLessThan(diff(encounter.parameters.course),.8*diff(boxOnly.parameters.course));
            time=0:.1:4.8;
            [center,box]=targetPrediction.finiteFlow(encounter,time);
            [~,wide]=targetPrediction.finiteFlow(boxOnly,time);
            testCase.verifyLessThanOrEqual(box-wide,1e-9);
            testCase.verifyLessThan(max(box(1:2,end)),.95*max(wide(1:2,end)));
            stream=RandStream('mt19937ar',Seed=5);excess=-Inf;
            for trial=1:300
                truth=nrmmTruthFixture.sampleDisc(encounter,.5,.1,stream);
                for node=1:numel(time)
                    deviation=nrmmTruthFixture.state(truth,time(node))-center(:,node);
                    deviation(7)=atan2(sin(deviation(7)),cos(deviation(7)));
                    excess=max([excess;abs(deviation)-box(:,node)]);
                end
            end
            testCase.verifyLessThanOrEqual(excess,1e-9);
        end

        function conditioningPropagatesAndIntersectsTheParameters(testCase)
            radius=[.2;.2;.1;.1;.1;.1;.02;.02];h=.05;
            carried=localPathEncounter([6;.3;.4;.02],radius,.05);
            truth=struct('p0',[1;2],'speed',6,'course',.3,'A',.4,'kappa',.02,'yaw0',.3);
            measured=carried;measured.center=nrmmTruthFixture.state(truth,h);
            measured.parameters=targetPrediction.nrmmParameters(measured.center,radius,measured.contract, ...
                localBallBounds(measured.center,.1,.1));
            next=targetPrediction.condition(carried,h,measured);
            arc=6*h+.4*h^2/2;
            expected=struct('speed',6+.4*h,'course',.3+.02*arc,'speedRate',.4,'curvature',.02);
            for name=["speed","course","speedRate","curvature"]
                interval=next.parameters.(name);
                testCase.verifyGreaterThanOrEqual(interval(1),measured.parameters.(name)(1)-1e-12);
                testCase.verifyLessThanOrEqual(interval(2),measured.parameters.(name)(2)+1e-12);
                testCase.verifyGreaterThanOrEqual(expected.(name),interval(1)-1e-12);
                testCase.verifyLessThanOrEqual(expected.(name),interval(2)+1e-12);
            end
            testCase.verifyLessThanOrEqual(diff(next.parameters.course),diff(measured.parameters.course)+1e-12);
            testCase.verifyLessThan(diff(next.parameters.course),diff(carried.parameters.course));
            contradictory=measured;contradictory.parameters.curvature=[.04;.05];
            testCase.verifyError(@() targetPrediction.condition(carried,h,contradictory), ...
                "collisionAvoidanceController:inconsistentObservation");
        end

        function aContradictoryYawRateIsAnInconsistentObservation(testCase)
            encounter=localPathEncounter([10;0;0;.02],[.1;.1;.05;.05;.01;.01;.01;.01],.05);
            encounter.center(8)=1;
            testCase.verifyError(@() targetPrediction.nrmmParameters(encounter.center,encounter.radius, ...
                encounter.contract,[]),"collisionAvoidanceController:inconsistentObservation");
        end

        function anNrmmContractCannotCarryAModelError(testCase)
            [ego,target,road,cfg]=encounterTestFixture.nrmmLead(Inf);
            jerk=target;jerk.predictionMotion.jerkBound=[.1;.1];
            testCase.verifyError(@() collisionAvoidanceController(ego,jerk,road,cfg,[]), ...
                "collisionAvoidanceController:invalidEncounterContract");
            yawing=target;yawing.predictionMotion.yawAccelerationBound=.1;
            testCase.verifyError(@() collisionAvoidanceController(ego,yawing,road,cfg,[]), ...
                "collisionAvoidanceController:invalidEncounterContract");
            zeros_=target;zeros_.predictionMotion.jerkBound=[0;0];zeros_.predictionMotion.yawAccelerationBound=0;
            [~,~,problem]=collisionAvoidanceController(ego,zeros_,road,cfg,[]);
            testCase.verifyTrue(problem.metadata.planCertified);
        end

        function aTargetWithoutAContractIsRefused(testCase)
            % No contract describes motion outside every NRMM path (the
            % estimator publishes none for a model error).
            [ego,target,road,cfg]=encounterTestFixture.nrmmLead(Inf);
            empty=target;empty.predictionMotion=[];
            testCase.verifyError(@() collisionAvoidanceController(ego,empty,road,cfg,[]), ...
                "collisionAvoidanceController:missingPredictionMotion");
            absent=rmfield(target,'predictionMotion');
            testCase.verifyError(@() collisionAvoidanceController(ego,absent,road,cfg,[]), ...
                "collisionAvoidanceController:missingPredictionMotion");
            other=target;other.predictionMotion=struct('kind',"finite-sensing-motion-v1",'jerkBound',[0;0]);
            testCase.verifyError(@() collisionAvoidanceController(ego,other,road,cfg,[]), ...
                "collisionAvoidanceController:invalidEncounterContract");
        end

        function sampledReactiveNrmmTrajectoriesStayInsideTheJointTube(testCase)
            % The reactive policy of the admitted directions, closed on NRMM
            % truths and per-hold estimator errors on the declared plant.
            [ego,target,road,cfg]=encounterTestFixture.nrmmLead(Inf);
            [~,plan,problem]=collisionAvoidanceController(ego,target,road,cfg,[]);
            testCase.assertTrue(problem.metadata.planCertified);
            reactive=localReactiveProgram(problem,30);
            testCase.assertTrue(any(reactive.prediction.targetGainSequence(:)));
            excess=nrmmTruthFixture.sampledExcess(reactive,problem.model,plan,20260924);
            testCase.verifyLessThanOrEqual(excess.joint,1e-9);
            testCase.verifyLessThanOrEqual(excess.record,1e-9);
            testCase.verifyLessThanOrEqual(excess.input,1e-9);
            testCase.verifyLessThanOrEqual(excess.slew,1e-9);
        end

        function aMotionModelChangeVoidsTheCarriedFamily(testCase)
            [ego,target,road,cfg]=encounterTestFixture.nrmmLead(Inf);
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
            faster=nextTarget;faster.predictionMotion.speedRateMaximum=2;
            [~,~,problem]=collisionAvoidanceController(next,faster,road,cfg,stored);
            testCase.verifyFalse(problem.metadata.inheritedFeasibleFamily);
        end
    end
end

function encounter=localPathEncounter(parameters,radius,curvatureMaximum)
% Estimate box centred on the NRMM state [1; 2] + path(parameters) at t = 0.
    truth=struct('p0',[1;2],'speed',parameters(1),'course',parameters(2),'A',parameters(3), ...
        'kappa',parameters(4),'yaw0',parameters(2));
    encounter=struct('center',nrmmTruthFixture.state(truth,0),'radius',radius,'time',0, ...
        'contract',struct('kind',"nrmm-motion-v1",'curvatureMaximum',curvatureMaximum));
    encounter.parameters=targetPrediction.nrmmParameters(encounter.center,encounter.radius,encounter.contract,[]);
end

function published=localBallBounds(center,velocityRadius,accelerationRadius)
% NRMM parameter error bounds of velocity and acceleration errors inside discs,
% written out independently of the estimator helper.
    v=center(3:4);a=center(5:6);speed=norm(v);
    theta=pi;
    if velocityRadius<speed,theta=asin(velocityRadius/speed);end
    componentRadius=accelerationRadius+2*norm(a)*sin(theta/2);
    normal=(v(1)*a(2)-v(2)*a(1))/speed;
    quotients=(normal+[-1;1]*componentRadius)./[(speed-velocityRadius)^2,(speed+velocityRadius)^2];
    published=struct('speed',velocityRadius,'course',theta,'speedRate',componentRadius, ...
        'curvature',[min(quotients,[],'all');max(quotients,[],'all')]);
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
