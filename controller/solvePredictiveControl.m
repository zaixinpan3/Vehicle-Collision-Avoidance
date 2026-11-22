function [result,diagnostics,problem] = solvePredictiveControl(interface,request)
%solvePredictiveControl Raw-sensor PCBF and dynamic-memory IPOPT comparison.
    assert(nargin==2 && (ischar(interface) || isstring(interface)) && isequal(string(interface),"outputFeedback"), ...
        'collisionAvoidanceController:invalidSolverInterface','Expected the outputFeedback interface.');
    if ~isfield(request,'callTimer'),request.callTimer=tic;end
    cfg=collisionAvoidanceControllerConfig(request.cfg);
    diagnostics=[];problem=[];
    switch string(request.operation)
        case "warmUp"
            result=localWarmIpopt(cfg,request.reference);
        case "features"
            result=localOfFeatures(request.samples,cfg);
        case "memory"
            result=localOfMemory(request.policy,request.memory,request.samples,cfg);
        case "sensor"
            result=localOfSensor(request.hypothesis,request.input,cfg,request.time);
        case "advance"
            result=localOfAdvance(request.hypothesis,request.input,cfg,request.duration,request.endOfHold);
        case "condition"
            [result,diagnostics]=localOfCondition(request.hypotheses,request.block,cfg,request.initial);
        case "window"
            result=localOfWindow(localOfValidateHypotheses(request.hypotheses,cfg),0,cfg);
        case "validateInputSequence"
            hypotheses=localOfValidateHypotheses(request.hypotheses,cfg);
            validateattributes(request.inputs,{'double'},{'2d','nrows',2,'nonempty','finite','real'});
            count=size(request.inputs,2);
            connection=localConnectionSteps(hypotheses,cfg);
            avoidance=count;
            if count>connection,avoidance=count-connection;end
            if isfield(request,'avoidanceSteps')
                avoidance=request.avoidanceSteps;
                validateattributes(avoidance,{'numeric'},{'scalar','integer','positive','<=',count});
            end
            boundaries=localCompactBoundaries(count,cfg,avoidance);
            kernel=localCompactKernel(cfg,diff(boundaries),~isempty(hypotheses(1).target));
            hypotheses=localCompactPlanningMembers(hypotheses,cfg,size(request.inputs,2));
            [valid,traces,violation,~,slacks]=localCompactValidate(request.inputs,hypotheses,request.input,kernel,cfg,true);
            result=struct('accepted',valid,'maximumViolation',violation,'trajectories',{traces}, ...
                'stageDeficits',slacks,'pcbfDeficit',sum(slacks));
        case "evaluateControlSequence"
            hypotheses=localOfValidateHypotheses(request.hypotheses,cfg);
            nominal=ceil(numel(hypotheses)/2);
            hypotheses=localCompactPlanningMembers(hypotheses,cfg,sum(request.holds));
            controls=request.inputs;holds=request.holds;
            validateattributes(holds,{'double'},{'row','integer','positive','numel',size(controls,2)});
            validateattributes(controls,{'double'},{'2d','nrows',2,'nonempty','finite','real'});
            x=hypotheses(nominal).ego;
            stub=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
            reference=[-1000,0;1000,0];if isfield(request,'reference'),reference=request.reference;end
            [~,lane]=readControllerInputs(stub,[],reference,cfg);
            kernel=localCompactKernel(cfg,holds,~isempty(hypotheses(1).target));
            model=localCondensedModel(cfg,hypotheses,request.input,holds,lane,nominal,kernel);
            [~,rollouts]=localCondensedOracle(controls(:),model,false);
            result=localCondensedOracle(controls(:),model,true,rollouts);
        case "predict"
            problem=localOfProblem(request.hypotheses,request.memory,request.input, ...
                request.policies,request.reference,cfg);
            result=problem.nominalTrajectories;
        case "step"
            [result,diagnostics,problem]=localOfStep(request,cfg);
        otherwise
            error('collisionAvoidanceController:invalidOutputFeedbackOperation','Unknown joint operation.');
    end
end

function [result,diagnostics,problem] = localOfStep(request,cfg)
    if isfield(request.previousState,'configuration')
        prior=request.previousState;
        stored=prior.configuration.outputFeedback;
        if ~isfield(stored,'movingTargetObservationSeconds')
            stored.movingTargetObservationSeconds=cfg.outputFeedback.movingTargetObservationSeconds;
        end
        if isfield(prior,'kind') && prior.kind=="compactOutputFeedback-v1" && string(stored.method)=="compactNlp"
            stored.method="ipopt";
        elseif isfield(prior,'kind') && prior.kind=="jointOutputFeedback-v1" && string(stored.method)=="ipopt" ...
                && cfg.outputFeedback.method=="dynamicMemoryIpopt"
            stored.method="dynamicMemoryIpopt";
        end
        assert(any(string(stored.method)==["ipopt","dynamicMemoryIpopt"]), ...
            'collisionAvoidanceController:invalidConfiguration','The checkpoint requires an unsupported solver.');
        prior.configuration.outputFeedback=stored;
        if isfield(prior.configuration.solver,'method') || ...
                any(isfield(stored,{'qpMaximumIterations','restorationMaximumIterations','finiteDifferenceStep','recoverySolveSeconds'}))
            prior.configuration=collisionAvoidanceControllerConfig(prior.configuration);
        end
        request.previousState=prior;
    end
    if string(cfg.outputFeedback.method)=="ipopt"
        [result,diagnostics]=localCompactStep(request,cfg);problem=[];return;
    end
    timer=tic;block=request.block;prior=request.previousState;
    if isempty(request.reference),request.reference=[-1000,0;1000,0];end
    localOfValidateBlock(block,cfg);
    assert(~cfg.controller.enforceRoadBoundaries, ...
        'collisionAvoidanceController:unsupportedJointRoadConstraint', ...
        'The joint deployment does not implement hard road-boundary inequalities.');
    count=cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod;
    assert(abs(count-round(count))<1e-10 && count>=2 && mod(round(count),2)==0, ...
        'collisionAvoidanceController:invalidSensorGrid', ...
        'A control hold must contain an even, positive number of sensor periods.');
    initial=isempty(prior);
    if initial
        assert(isfield(block,'initialHypotheses') && ~isempty(block.initialHypotheses), ...
            'collisionAvoidanceController:missingInitialInformation', ...
            'An explicit joint initial hypothesis contract is required; a single sensor block cannot identify it.');
        hypotheses=localOfInitialHypotheses(block,cfg);
        assert(numel(block.samples)==1 && block.samples.time==block.time, ...
            'collisionAvoidanceController:invalidInitialBlock','Initialization uses one current-boundary reading.');
        memory=zeros(cfg.outputFeedback.memoryDimension,1);
        if isfield(block,'initialMemory'),memory=block.initialMemory;end
        validateattributes(memory,{'double'},{'size',[cfg.outputFeedback.memoryDimension,1],'finite','real'});
        assert(all(abs(memory)<=1),'collisionAvoidanceController:invalidMemory','Memory must belong to its invariant cube.');
        policies=[];
        if isfield(block,'initialPolicies'),policies=block.initialPolicies;end
        inheritedCount=numel(policies);
    else
        assert(isstruct(prior) && isfield(prior,'kind') && prior.kind=="jointOutputFeedback-v1", ...
            'collisionAvoidanceController:incompatibleContinuation','Expected a joint output-feedback continuation.');
        assert(isequaln(prior.configuration,cfg) && isequaln(prior.reference,request.reference), ...
            'collisionAvoidanceController:changedContinuationProblem', ...
            'Continuation must retain its model, uncertainty contract, memory coordinates, and reference.');
        assert(~isfield(block,'initialHypotheses') && ~isfield(block,'initialMemory') ...
            && ~isfield(block,'initialPolicies'), ...
            'collisionAvoidanceController:informationReset','A continuation cannot reset its information or memory.');
        times=prior.time+(1:round(count))*cfg.outputFeedback.sensorPeriod;
        assert(numel(block.samples)==round(count) && ...
            max(abs([block.samples.time]-times))<1e-9 && abs(block.time-times(end))<1e-9, ...
            'collisionAvoidanceController:invalidSensorGrid','Receive the complete consecutive sensor block.');
        assert(norm(block.heldInput-prior.appliedInput,Inf)<1e-12, ...
            'collisionAvoidanceController:actuationMismatch','The received block must use the previously committed actuator command.');
        hypotheses=prior.hypotheses;
        memory=localOfMemory(prior.committedPolicy,prior.memory,block.samples,cfg);
        policies=prior.policies(2:end);inheritedCount=numel(policies);
    end
    inferenceTimer=tic;
    [hypotheses,conditioning]=localOfCondition(hypotheses,block,cfg,initial);
    inferenceSeconds=toc(inferenceTimer);
    preparationTimer=tic;
    window=localOfWindow(hypotheses,inheritedCount,cfg);
    if isempty(policies)
        policies=localOfInitialPolicies(cfg,hypotheses,request.reference,block.heldInput,window.horizonSteps);
    else
        policies=[policies,repmat(localOfBackup(cfg),1,window.horizonSteps-numel(policies))];
    end
    capacity=numel(hypotheses);
    if ~initial && isfield(prior,'nlpHypothesisCapacity'),capacity=prior.nlpHypothesisCapacity;end
    problem=localOfProblem(hypotheses,memory,block.heldInput,policies,request.reference,cfg,capacity);
    preparationSeconds=toc(preparationTimer);
    warm=[];
    if ~initial && isfield(prior,'nlpWarmStart') && ...
            isequal(conditioning.retainedIndices(:),(1:numel(prior.hypotheses))')
        warm=prior.nlpWarmStart;
    end
    [point,diagnostics,trajectory,warm]=localOfNonlinearOptimize(problem,initial,timer,warm);
    diagnostics.informationUpdateSeconds=inferenceSeconds;
    diagnostics.predictionPreparationSeconds=preparationSeconds;
    diagnostics.predictionWindow=window;
    accepted=localOfUnpackPlan(point,problem);input=accepted(1).v+accepted(1).K*memory;
    % Commit the full first controller, including its next-block memory map.
    state=struct('kind',"jointOutputFeedback-v1",'time',block.time, ...
        'memory',memory,'hypotheses',hypotheses,'appliedInput',input, ...
        'committedPolicy',accepted(1),'policies',accepted,'configuration',cfg, ...
        'reference',request.reference,'conditioning',conditioning);
    state.nlpWarmStart=warm;state.nlpHypothesisCapacity=capacity;
    diagnostics.conditioning=conditioning;diagnostics.elapsedSeconds=toc(timer);
    diagnostics.deadlineMissed=diagnostics.elapsedSeconds>diagnostics.deadlineSeconds;
    diagnostics.memoryDimension=numel(memory);diagnostics.hypothesisCount=numel(hypotheses);
    diagnostics.scenarioCount=numel(problem.scenarios);
    diagnostics.activeScenarioCount=numel(hypotheses)*size(cfg.outputFeedback.noiseDirections,2);
    diagnostics.safetyScope="finiteJointScenariosNonlinearRolloutValidated";
    diagnostics.robustNonlinearSafetyCertified=false;
    diagnostics.recursiveFeasibilityCertified=false;
    diagnostics.optimizationReturned=true;
    diagnostics.wholeHoldReserveScope="engineeringIntersampleMarginNoCertificate";
    diagnostics.predictionModel="jointDynamicOutputFeedbackSparseMultipleShooting";
    diagnostics.sensorContract="positionOnlyGnssBodyImuGyroRelativeRadar";
    diagnostics.previousInputUsedForMeasurements=block.heldInput;
    command=struct('actuatorInput',input,'steeringAngle',input(1),'brakingRatio',input(2));
    prediction=struct('metadata',diagnostics,'scenarios',{trajectory}, ...
        'policies',accepted,'memory',memory,'information',hypotheses);
    result=struct('command',command,'inputs',trajectory{1}.inputs, ...
        'prediction',prediction,'state',state);
end

function localOfValidateBlock(block,cfg)
    assert(isstruct(block) && isscalar(block) && ...
        all(isfield(block,{'kind','time','heldInput','samples'})), ...
        'collisionAvoidanceController:invalidSensorBlock','A timestamped raw block and previous input are required.');
    validateattributes(block.time,{'double'},{'scalar','real','finite'});
    validateattributes(block.heldInput,{'double'},{'size',[2,1],'real','finite'});
    assert(abs(block.heldInput(2))<=1,'collisionAvoidanceController:invalidSensorBlock','Invalid previous force ratio.');
    assert(isstruct(block.samples) && ~isempty(block.samples), ...
        'collisionAvoidanceController:invalidSensorBlock','The sensor block cannot be empty.');
    for j=1:numel(block.samples)
        y=block.samples(j);
        assert(all(isfield(y,{'time','position','acceleration','yawRate','relativePosition','radarAvailable'})), ...
            'collisionAvoidanceController:invalidSensorBlock','Every raw reading must include its detection flag.');
        validateattributes(y.time,{'double'},{'scalar','real','finite'});
        validateattributes(y.position,{'double'},{'size',[2,1],'real','finite'});
        validateattributes(y.acceleration,{'double'},{'size',[2,1],'real','finite'});
        validateattributes(y.yawRate,{'double'},{'scalar','real','finite'});
        validateattributes(y.radarAvailable,{'logical'},{'scalar'});
        if isfield(y,'radarPositionErrorMeters') && ~isempty(y.radarPositionErrorMeters)
            validateattributes(y.radarPositionErrorMeters,{'double'},{'real','scalar','finite','nonnegative'});
        end
        if isfield(y,'relativeHeading') && ~isempty(y.relativeHeading)
            validateattributes(y.relativeHeading,{'double'},{'real','scalar'});
            assert(isnan(y.relativeHeading) || (y.radarAvailable && isfinite(y.relativeHeading)), ...
                'collisionAvoidanceController:invalidSensorBlock','A relative heading requires a detected target.');
        end
        if y.radarAvailable
            validateattributes(y.relativePosition,{'double'},{'size',[2,1],'real','finite'});
        end
    end
    assert(cfg.model.speedMinimum==0,'collisionAvoidanceController:terminalRestExcluded', ...
        'The joint deployment uses a forward-only physical domain with zero minimum speed.');
end

function hypotheses=localOfInitialHypotheses(block,cfg)
    hypotheses=block.initialHypotheses;
    if ~any([block.samples.radarAvailable]) && isstruct(hypotheses) && isfield(hypotheses,'target')
        [hypotheses.target]=deal([]);
    end
    hypotheses=localOfValidateHypotheses(hypotheses,cfg);
end

function hypotheses = localOfValidateHypotheses(hypotheses,cfg)
    assert(isstruct(hypotheses) && ~isempty(hypotheses) ...
        && numel(hypotheses)<=cfg.outputFeedback.maximumHypotheses, ...
        'collisionAvoidanceController:invalidInitialInformation','Supply a nonempty bounded-size joint ensemble.');
    hasTarget=false(1,numel(hypotheses));
    for j=1:numel(hypotheses)
        assert(all(isfield(hypotheses(j),{'ego','target','parameters','sensorBias'})), ...
            'collisionAvoidanceController:invalidInitialInformation', ...
            'Each hypothesis needs ego, target, persistent parameter scales, and persistent sensor bias.');
        validateattributes(hypotheses(j).ego,{'double'},{'size',[6,1],'real','finite'});
        validateattributes(hypotheses(j).parameters,{'double'},{'size',[4,1],'real','finite','positive'});
        validateattributes(hypotheses(j).sensorBias,{'double'},{'size',[7,1],'real','finite'});
        q=hypotheses(j).target;hasTarget(j)=~isempty(q);
        if hasTarget(j)
            validateattributes(q,{'double'},{'size',[11,1],'real','finite'});
            assert(q(4)>=0 && q(7)>0 && all(q(8:9)>0), ...
                'collisionAvoidanceController:invalidInitialInformation','Invalid target speed or footprint.');
        end
    end
    assert(all(hasTarget==hasTarget(1)), ...
        'collisionAvoidanceController:unsupportedTargetExistenceMixture', ...
        'This finite transcription requires a common target-existence mode.');
end

function cfg = localOfPhysicalConfiguration(hypothesis,cfg)
    % Scales remain attached to their physical hypothesis for its lifetime.
    cfg.tire.corneringStiffness=cfg.tire.corneringStiffness.*hypothesis.parameters(1:2);
    cfg.tire.frictionCoefficient=cfg.tire.frictionCoefficient.*hypothesis.parameters(3:4);
end

function next = localOfAdvance(hypothesis,input,cfg,duration,endOfHold,flow)
    if nargin<6,flow=localOfInferenceKernel(cfg,duration);end
    next=hypothesis;
    if ~isempty(flow)
        next.ego=full(flow(hypothesis.ego,input,hypothesis.parameters));
        if endOfHold,next.ego=nonlinearBicycleModel.sampleEndState(next.ego,input,cfg);end
    else
        physical=localOfPhysicalConfiguration(hypothesis,cfg);
        next.ego=nonlinearBicycleModel.sensorFlow(hypothesis.ego,input,physical,duration,endOfHold);
    end
    if ~isempty(hypothesis.target) && hypothesis.target(4)>0
        next.target=predictiveSafetyGeometry.predictTarget(hypothesis.target,duration);
        next.target(5:6)=hypothesis.target(5:6);
    end
    % V=0 is an absorbing mode, including when latent A was nonzero.
end

function flow=localOfInferenceKernel(cfg,duration)
    % Match the fixed prediction/replay mesh so conditioned successor roots
    % agree with the previously accepted held-input trajectory.
    persistent keys flows
    if string(cfg.outputFeedback.method)~="ipopt",flow=[];return;end
    key={cfg,duration};index=[];
    for j=1:numel(keys),if isequaln(keys{j},key),index=j;break;end,end
    if isempty(index)
        import casadi.*
        % Align integer sensor blocks with the held-input integration mesh.
        requested=cfg.outputFeedback.validationSubsteps*duration/cfg.controller.sampleTime;
        count=max(1,ceil(requested-8*eps(max(1,requested))));dt=duration/count;
        root=SX.sym('inference_state',6);command=SX.sym('inference_input',2);scale=SX.sym('inference_scale',4);
        k1=nonlinearBicycleModel.symbolicDerivative(root,command,scale,cfg);
        k2=nonlinearBicycleModel.symbolicDerivative(root+dt*k1/2,command,scale,cfg);
        k3=nonlinearBicycleModel.symbolicDerivative(root+dt*k2/2,command,scale,cfg);
        k4=nonlinearBicycleModel.symbolicDerivative(root+dt*k3,command,scale,cfg);
        options=struct;
        if cfg.outputFeedback.nlpJit
            directory=tempname;mkdir(directory);
            options=struct('jit',true,'compiler','shell','jit_options',struct( ...
                'flags',sprintf('-O%d',cfg.outputFeedback.nlpCompilerOptimization), ...
                'compiler',char(cfg.outputFeedback.nlpCompiler),'directory',[directory,filesep]));
        end
        step=Function('pcbf_inference_step',{root,command,scale},{root+dt*(k1+2*k2+2*k3+k4)/6},options);
        mapped=step.mapaccum(count);root=MX.sym('root',6);command=MX.sym('command',2);scale=MX.sym('scale',4);
        states=mapped(root,repmat(command,1,count),repmat(scale,1,count));
        flows{end+1}=Function('pcbf_inference',{root,command,scale},{states(:,end)});keys{end+1}=key;index=numel(keys);
    end
    flow=flows{index};
end

function y = localOfSensor(hypothesis,input,cfg,time)
    physical=localOfPhysicalConfiguration(hypothesis,cfg);x=hypothesis.ego;
    dx=nonlinearBicycleModel.derivative(x,input,physical);
    a=[dx(4)-x(6)*x(5);dx(5)+x(6)*x(4)];
    relative=zeros(2,1);detected=false;
    if ~isempty(hypothesis.target)
        rotation=[cos(x(3)),sin(x(3));-sin(x(3)),cos(x(3))];
        relative=rotation*(hypothesis.target(1:2)-x(1:2));
        detected=norm(relative)<=cfg.collision.encounterRangeMeters;
    end
    if ~detected,relative=NaN(2,1);end
    bias=hypothesis.sensorBias;
    y=struct('time',time,'position',x(1:2)+bias(1:2), ...
        'acceleration',a+bias(3:4),'yawRate',x(6)+bias(5), ...
        'relativePosition',relative+bias(6:7),'radarAvailable',detected);
end

function [posterior,info] = localOfCondition(hypotheses,block,cfg,initial)
    hypotheses=localOfValidateHypotheses(hypotheses,cfg);
    observe=isempty(hypotheses(1).target) || ...
        (isfield(hypotheses,'targetObserver') && ~isempty(hypotheses(1).targetObserver));
    if observe && ~isfield(hypotheses,'targetObserver'),[hypotheses.targetObserver]=deal([]);end
    if observe && (~isempty(hypotheses(1).target) || any([block.samples.radarAvailable])) && ...
            exist('onlineNrmmTrackingRuntime','file')~=2
        addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'estimator'));
    end
    keep=false(1,numel(hypotheses));residual=Inf(numel(hypotheses),1);
    next=hypotheses;bounds=cfg.outputFeedback.noiseBounds;
    tolerance=1e-9; % Numerical measurement comparison only; not extra noise.
    % The sensor mesh and model are shared; the persistent physical
    % parameters remain separate arguments for each hypothesis.
    if ~initial,flow=localOfInferenceKernel(cfg,cfg.outputFeedback.sensorPeriod);end
    for s=1:numel(hypotheses)
        item=hypotheses(s);excess=-Inf;
        for j=1:numel(block.samples)
            if ~initial
                item=localOfAdvance(item,block.heldInput,cfg,cfg.outputFeedback.sensorPeriod,j==numel(block.samples),flow);
            end
            measured=block.samples(j);
            if observe && (measured.radarAvailable || ~isempty(item.targetObserver))
                measuredPosition=[];
                if measured.radarAvailable
                    corrected=measured.relativePosition-item.sensorBias(6:7);
                    radius=cfg.collision.encounterRangeMeters;
                    rho=corrected*min(1,radius/max(norm(corrected),realmin));
                    excess=max(excess,norm(corrected-rho)-bounds(4));
                    yaw=item.ego(3);rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
                    measuredPosition=item.ego(1:2)+rotation*rho;
                end
                relativeHeading=NaN;
                if measured.radarAvailable && isfield(measured,'relativeHeading') && ~isempty(measured.relativeHeading)
                    relativeHeading=measured.relativeHeading;
                end
                positionError=bounds(4);
                if isfield(measured,'radarPositionErrorMeters') && ~isempty(measured.radarPositionErrorMeters)
                    positionError=min(positionError,measured.radarPositionErrorMeters);
                end
                previousTrack=item.targetObserver;
                try
                    [item.targetObserver,motion]=onlineNrmmTrackingRuntime("targetPoint", ...
                        item.targetObserver,measured.time,measuredPosition,item.ego(3),cfg,relativeHeading,positionError,j==numel(block.samples));
                catch exception
                    if ~strcmp(exception.identifier,'onlineNrmmTrackingRuntime:inconsistentTargetMotion'),rethrow(exception);end
                    excess=Inf;break
                end
                if measured.radarAvailable && ~isempty(motion)
                    % Keep the previous observer forecast while this return
                    % explains it. Replacing it with each noisy position
                    % would discard a consistent shifted collision witness.
                    refresh=isempty(item.target) || (~isempty(previousTrack) && ...
                        (isempty(previousTrack.times) || previousTrack.restCompatible~=item.targetObserver.restCompatible || ...
                        ~previousTrack.initialized && item.targetObserver.initialized));
                    if ~refresh
                        refresh=norm(item.target(1:2)-item.ego(1:2))>radius || ...
                            norm(rotation.'*(item.target(1:2)-item.ego(1:2))-corrected)>bounds(4)+tolerance;
                    end
                    if refresh
                        % Use the filtered position inside both the detection
                        % disk and the current bounded position observation.
                        filtered=rotation.'*(item.targetObserver.state(1:2)-item.ego(1:2));
                        filtered=filtered*min(1,radius/max(norm(filtered),realmin));
                        delta=filtered-rho;
                        room=max(0,bounds(4)-norm(corrected-rho));
                        center=measuredPosition+rotation*(min(1,room/max(norm(delta),realmin))*delta);
                        item.target=[center;motion;cfg.target.rearAxleDistance; ...
                            cfg.target.defaultLength/2;cfg.target.defaultWidth/2;0;0];
                    end
                    if isfinite(relativeHeading)
                        % Position consistency does not constrain body heading.
                        item.target(3)=motion(1);
                    end
                    if item.target(4)>0
                        % A position-only return can also tighten these intervals.
                        % Retain the same forecast that the controller propagates.
                        motionBounds=item.targetObserver.motionBounds;
                        item.target(5:6)=min(motionBounds(2,:).',max(motionBounds(1,:).',item.target(5:6)));
                    end
                end
            end
            expected=localOfSensor(item,block.heldInput,cfg,measured.time);
            errors=[norm(measured.position-expected.position); ...
                norm(measured.acceleration-expected.acceleration);abs(measured.yawRate-expected.yawRate)];
            excess=max(excess,max(errors-bounds(1:3)));
            if measured.radarAvailable && ~observe
                if isempty(item.target) || ~expected.radarAvailable
                    excess=Inf;
                else
                    excess=max(excess,norm(measured.relativePosition-expected.relativePosition)-bounds(4));
                    if isfield(measured,'relativeHeading') && ~isempty(measured.relativeHeading) && isfinite(measured.relativeHeading)
                        angle=item.target(3)-item.ego(3)-measured.relativeHeading;
                        excess=max(excess,abs(atan2(sin(angle),cos(angle))));
                    end
                end
            end
            % A missed return alone cannot establish target departure.
        end
        next(s)=item;residual(s)=excess;keep(s)=excess<=tolerance;
    end
    assert(any(keep),'collisionAvoidanceController:emptyJointInformation', ...
        'No retained physical hypothesis explains the complete raw block. No command or memory reset is authorized.');
    posterior=next(keep);
    if observe && any(arrayfun(@(s)isempty(s.target),posterior))
        % The common planner starts when every surviving ego hypothesis has
        % a full target estimate. Each observer keeps its own acquisition history.
        [posterior.target]=deal([]);
    end
    info=struct('priorCount',numel(hypotheses),'posteriorCount',nnz(keep), ...
        'retainedIndices',find(keep),'maximumNoiseExcess',residual, ...
        'rawReadingsConditioned',numel(block.samples),'representation',"finiteConsistentHypotheses");
    if observe,info.representation="finiteEgoHypothesesWithNrmmTargetPoint";end
end

function feature = localOfFeatures(samples,cfg)
    values=zeros(8,numel(samples));
    for j=1:numel(samples)
        y=samples(j);relative=zeros(2,1);
        if y.radarAvailable,relative=y.relativePosition;end
        values(:,j)=[[y.position;y.acceleration;y.yawRate;relative]./cfg.outputFeedback.featureScale; ...
            double(y.radarAvailable)];
    end
    feature=mean(values,2);
end

function memory = localOfMemory(policy,memory,samples,cfg)
    memory=tanh(policy.A*memory+policy.B*localOfFeatures(samples,cfg)+policy.c);
end

function policy = localOfBackup(cfg)
    n=cfg.outputFeedback.memoryDimension;
    policy=struct('v',[0;-cfg.outputFeedback.backupBrakingRatio], ...
        'K',zeros(2,n),'A',zeros(n),'B',zeros(n,8),'c',zeros(n,1));
end

function window = localOfWindow(hypotheses,inheritedCount,cfg)
    % Select coverage directly, before optimization; never try increasing
    % lengths until a solve succeeds. This is an engineering time scale,
    % not a certified Fiala stopping-time or feasibility upper bound.
    of=cfg.outputFeedback;h=cfg.controller.sampleTime;
    speed=max(arrayfun(@(s)hypot(s.ego(4),s.ego(5)),hypotheses));
    if isempty(of.horizonSeconds)
        minimum=2*h;
        if string(of.method)=="ipopt"
            minimum=max(minimum,of.minimumPreviewSeconds);
            % A low-speed preview must span the encounter's spatial scale,
            % not only braking time. This is a performance lookahead, not a
            % stopping reserve or a certificate of feasibility.
            targetRadius=0;
            for s=1:numel(hypotheses)
                q=hypotheses(s).target;
                if ~isempty(q),targetRadius=max(targetRadius,norm(q(8:9))+norm(q(10:11)));end
            end
            if targetRadius>0 && cfg.referenceSpeed>0
                egoRadius=hypot(cfg.vehicle.length/2,cfg.vehicle.width/2)+norm(cfg.vehicle.rectangleOffset);
                minimum=max(minimum,2*(egoRadius+targetRadius)/max(speed,cfg.referenceSpeed));
            end
        end
        requested=max(minimum,speed/of.previewDecelerationMetersPerSecondSquared+of.previewSettlingSeconds);
        allocated=of.horizonResolutionSeconds*ceil(requested/of.horizonResolutionSeconds);
        source="speedBasedPreview";
    else
        requested=max(2*h,of.horizonSeconds);allocated=requested;source="configuredDuration";
    end
    % Retain only the unexecuted suffix, which still reaches the old
    % terminal endpoint. The executed hold is no longer a horizon floor.
    connection=localConnectionSteps(hypotheses,cfg);
    count=max(inheritedCount,ceil(allocated/h)+connection);
    assert(count<=cfg.controller.maximumHorizonSteps, ...
        'collisionAvoidanceController:predictionWindowCapacity', ...
        'The selected %.3g s preview needs %d stages, exceeding the configured capacity %g. It was not truncated.', ...
        count*h,count,cfg.controller.maximumHorizonSteps);
    window=struct('source',source,'requestedSeconds',requested,'allocatedSeconds',count*h, ...
        'horizonSteps',count,'avoidanceSteps',count-connection,'connectionSteps',connection, ...
        'speedUpperMetersPerSecond',speed, ...
        'inheritedHorizonSteps',inheritedCount,'feasibilityCertified',false);
end

function count=localConnectionSteps(hypotheses,cfg)
    count=0;
    if string(cfg.outputFeedback.method)=="ipopt" && any(arrayfun(@(item)~isempty(item.target),hypotheses))
        count=ceil(cfg.outputFeedback.minimumPreviewSeconds/cfg.controller.sampleTime);
    end
end

function policies = localOfInitialPolicies(cfg,hypotheses,reference,previousInput,count)
    % The selected time coverage sizes this numerical initialization only.
    speed=max(arrayfun(@(s)norm(s.ego(4:5)),hypotheses));
    policy=localOfBackup(cfg);n=cfg.outputFeedback.memoryDimension;
    policy.A=.25*eye(n);policy.B=.1*eye(n,8);
    policies=repmat(policy,1,count);
    % A smooth avoidance rollout provides only a starting policy. It uses a
    % representative retained hypothesis and is checked against every finite
    % scenario by the optimizer. Its first local QP must succeed before execution.
    representative=hypotheses(ceil(numel(hypotheses)/2));
    if speed>cfg.model.kinematicSpeedRange(1) && ~isempty(representative.target)
        x=representative.ego;q=representative.target;
        stub=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
        [~,lane,road]=readControllerInputs(stub,[],reference,cfg);
        frame=predictiveSafetyGeometry.roadFrame(lane,road);
        nominal=struct('state',[0;0;0;cfg.referenceSpeed;0;0],'curvature',frame(4));
        model=struct('cfg',cfg,'initialState',x,'previousInput',previousInput, ...
            'target',q,'lane',lane,'road',road,'frame',frame, ...
            'nominalReference',nominal, ...
            'uncertainty',struct('collisionGenerator',zeros(6,0),'target',[]));
        guide=predictiveSafetyGeometry.avoidanceGuide(model,count);
        for i=1:count-1
            policies(i).v=[max(-cfg.outputFeedback.steeringMaximum,min(cfg.outputFeedback.steeringMaximum,guide.inputs(1,i))); ...
                -cfg.outputFeedback.backupBrakingRatio];
        end
    end
    policies(end)=localOfBackup(cfg);
end

function vector = localOfPack(policy)
    vector=[policy.v(:);policy.K(:);policy.A(:);policy.B(:);policy.c(:)];
end

function policy = localOfUnpack(vector,cfg)
    n=cfg.outputFeedback.memoryDimension;a=2;b=a+2*n;c=b+n*n;d=c+8*n;
    policy=struct('v',vector(1:a),'K',reshape(vector(a+1:b),2,n), ...
        'A',reshape(vector(b+1:c),n,n),'B',reshape(vector(c+1:d),n,8),'c',vector(d+1:end));
end

function policies = localOfUnpackPlan(point,problem)
    policies=repmat(localOfBackup(problem.cfg),1,problem.horizon);
    for i=1:problem.horizon
        policies(i)=localOfUnpack(point(problem.policyIndices(:,i)),problem.cfg);
    end
end

function problem = localOfProblem(hypotheses,memory,input,policies,reference,cfg,capacity)
    if nargin<7,capacity=numel(hypotheses);end
    n=numel(policies);dimension=numel(localOfPack(localOfBackup(cfg)));
    assert(isstruct(policies) && n>=2 && n<=cfg.controller.maximumHorizonSteps, ...
        'collisionAvoidanceController:invalidPolicySeed','The seed must have one complete policy per hold.');
    indices=reshape(1:dimension*n,dimension,n);point=zeros(dimension*n,1);
    for i=1:n
        vector=localOfPack(policies(i));
        assert(numel(vector)==dimension && all(isfinite(vector)), ...
            'collisionAvoidanceController:invalidPolicySeed','Invalid policy dimensions or coefficients.');
        point(indices(:,i))=vector;
    end
    scenarios=struct([]);noise=cfg.outputFeedback.noiseDirections;
    radii=cfg.outputFeedback.noiseBounds([1;1;2;2;3;4;4]);
    for s=1:numel(hypotheses)
        for j=1:size(noise,2)
            scenario=struct('initial',hypotheses(s),'noise',radii.*noise(:,j),'rootIndex',s,'weight',1);
            if isempty(scenarios),scenarios=scenario;else,scenarios(end+1)=scenario;end %#ok<AGROW>
        end
    end
    activeCount=numel(scenarios);
    if string(cfg.outputFeedback.method)=="dynamicMemoryIpopt"
        % Keep the compiled NLP shape through posterior pruning. Duplicate
        % the last surviving trajectory with zero objective weight; its hard
        % rows are redundant and do not add a new uncertainty realization.
        scenarios(activeCount+1:capacity*size(noise,2))=scenarios(end);
    end
    for s=1:numel(scenarios),scenarios(s).weight=double(s<=activeCount)/activeCount;end
    x=hypotheses(1).ego;
    stub=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
    [~,lane]=readControllerInputs(stub,[],reference,cfg);
    hasTarget=~isempty(hypotheses(1).target);policyCount=numel(point);
    witnessIndices=zeros(2,3,n,numel(scenarios));
    if hasTarget
        witnessIndices=reshape(policyCount+(1:numel(witnessIndices)),size(witnessIndices));
        point=[point;zeros(numel(witnessIndices),1)];
    end
    lower=-cfg.outputFeedback.coefficientMaximum*ones(size(point));upper=-lower;
    for i=1:n
        gain=indices(3:2+2*cfg.outputFeedback.memoryDimension,i);
        lower(gain)=-cfg.outputFeedback.gainMaximum;upper(gain)=cfg.outputFeedback.gainMaximum;
    end
    if hasTarget,lower(policyCount+1:end)=-Inf;upper(policyCount+1:end)=Inf;end
    assert(all(point(1:policyCount)>=lower(1:policyCount)) && all(point(1:policyCount)<=upper(1:policyCount)), ...
        'collisionAvoidanceController:invalidPolicySeed','Policy seed exceeds the coefficient contract.');
    problem=struct('cfg',cfg,'scenarios',scenarios,'memory',memory,'previousInput',input, ...
        'horizon',n,'lane',lane,'policyIndices',indices,'policyCount',policyCount, ...
        'witnessIndices',witnessIndices,'hasTarget',hasTarget,'lower',lower,'upper',upper, ...
        'initial',point,'regularizationAnchor',point(1:policyCount),'seedRepaired',false);
    try
        trajectories=localOfRollout(point,problem);
    catch exception
        if string(cfg.outputFeedback.method)~="dynamicMemoryIpopt" || ...
                ~any(string(exception.identifier)==["collisionAvoidanceController:jointTrialDomain", ...
                "collisionAvoidanceController:nonlinearDomain"]),rethrow(exception);end
        % A changed measured memory can invalidate a shifted numerical seed.
        % Repair only the seed's actuator maps; inference and actual memory
        % are untouched, and the optimizer must still pass nonlinear replay.
        previous=input;
        for i=1:n
            policy=policies(i);policy.K(:)=0;
            policy.v(1)=max(-cfg.outputFeedback.steeringMaximum,min(cfg.outputFeedback.steeringMaximum, ...
                max(previous(1)-cfg.controller.sampleTime*cfg.outputFeedback.steeringRateMaximum, ...
                min(previous(1)+cfg.controller.sampleTime*cfg.outputFeedback.steeringRateMaximum,policy.v(1)))));
            change=cfg.controller.sampleTime*cfg.model.brakingRatioRateMaximum;
            policy.v(2)=max(cfg.actuation.brakingRatioMinimum,min(cfg.actuation.brakingRatioMaximum, ...
                max(previous(2)-change,min(previous(2)+change,policy.v(2)))));
            point(indices(:,i))=localOfPack(policy);previous=policy.v;
        end
        problem.initial=point;problem.regularizationAnchor=point(1:policyCount);problem.seedRepaired=true;
        trajectories=localOfRollout(point,problem);
    end
    if hasTarget
        for s=1:numel(scenarios)
            for i=1:n
                for j=1:3
                    x=trajectories{s}.checks(:,j,i);q=trajectories{s}.targetChecks(:,j,i);
                    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
                    witness=predictiveSafetyGeometry.supportLinearization(x(1:3),shape,q(1:3),q(8:11));
                    normal=witness.normal;rotation=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];
                    offset=normal.'*rotation*q(10:11)+abs(normal.'*rotation)*q(8:9);
                    point(witnessIndices(:,j,i,s))=[atan2(normal(2),normal(1));offset];
                end
            end
        end
        problem.initial=point;
        problem.nominalTrajectories=trajectories;
    else
        problem.nominalTrajectories=trajectories;
    end
end

function trajectories = localOfRollout(point,problem)
    cfg=problem.cfg;n=problem.horizon;l=cfg.outputFeedback.predictionSubsteps;
    dt=cfg.controller.sampleTime/l;
    policies=localOfUnpackPlan(point,problem);trajectories=cell(1,numel(problem.scenarios));
    for s=1:numel(problem.scenarios)
        scenario=problem.scenarios(s);item=scenario.initial;m=problem.memory;
        trace=struct('ego',zeros(6,n+1),'memory',zeros(numel(m),n+1), ...
            'inputs',zeros(2,n),'targets',zeros(11,n+1),'checks',zeros(6,3,n), ...
            'targetChecks',zeros(11,3,n),'physicalNodes',zeros(6,l+1,n), ...
            'sensorBlocks',{cell(1,n)},'rootIndex',scenario.rootIndex);
        trace.ego(:,1)=item.ego;trace.memory(:,1)=m;
        if problem.hasTarget,trace.targets(:,1)=item.target;end
        for i=1:n
            policy=policies(i);u=policy.v+policy.K*m;trace.inputs(:,i)=u;
            assert(abs(u(2))<=1+1e-12,'collisionAvoidanceController:jointTrialDomain', ...
                'A trial force ratio left the physical model domain.');
            trace.checks(:,1,i)=item.ego;trace.physicalNodes(:,1,i)=item.ego;
            if problem.hasTarget,trace.targetChecks(:,1,i)=item.target;end
            readings=struct([]);
            for j=1:l
                item=localOfAdvance(item,u,cfg,dt,j==l);
                y=localOfSensor(item,u,cfg,(i-1)*cfg.controller.sampleTime+j*dt);
                y.position=y.position+scenario.noise(1:2);y.acceleration=y.acceleration+scenario.noise(3:4);
                y.yawRate=y.yawRate+scenario.noise(5);y.relativePosition=y.relativePosition+scenario.noise(6:7);
                if isempty(readings),readings=y;else,readings(end+1)=y;end %#ok<AGROW>
                trace.physicalNodes(:,j+1,i)=item.ego;
                if j==l/2 || j==l
                    slot=1+round(2*j/l);trace.checks(:,slot,i)=item.ego;
                    if problem.hasTarget,trace.targetChecks(:,slot,i)=item.target;end
                end
            end
            m=localOfMemory(policy,m,readings,cfg);
            trace.ego(:,i+1)=item.ego;trace.memory(:,i+1)=m;trace.sensorBlocks{i}=readings;
            if problem.hasTarget,trace.targets(:,i+1)=item.target;end
        end
        trajectories{s}=trace;
    end
end

function [r,c]=localOfStageValues(u,previous,witness,data,problem)
    cfg=problem.cfg;of=cfg.outputFeedback;h=cfg.controller.sampleTime;x=data.next;
    projection=predictiveSafetyGeometry.project(x(1:2),problem.lane);
    error=[projection.lateralPosition;atan2(sin(x(3)-projection.heading),cos(x(3)-projection.heading)); ...
        x(4)-cfg.referenceSpeed;x(5);x(6)];
    r=[sqrt(h*of.trackingWeights).*error;sqrt(of.inputWeight)*(u-previous)]/sqrt(numel(problem.scenarios));
    c=[u(1)-of.steeringMaximum;-u(1)-of.steeringMaximum; ...
        u(2)-cfg.actuation.brakingRatioMaximum;cfg.actuation.brakingRatioMinimum-u(2)];
    if isfinite(of.steeringRateMaximum)
        change=h*of.steeringRateMaximum;c=[c;u(1)-previous(1)-change;previous(1)-u(1)-change];
    end
    if isfinite(cfg.model.brakingRatioRateMaximum)
        change=h*cfg.model.brakingRatioRateMaximum;c=[c;u(2)-previous(2)-change;previous(2)-u(2)-change];
    end
    states=data.states;
    c=[c;reshape([-states(4,:);states(4,:)-cfg.model.speedMaximum; ...
        states(5,:)-cfg.model.lateralVelocityMaximum;-states(5,:)-cfg.model.lateralVelocityMaximum; ...
        states(6,:)-cfg.model.yawRateMaximum;-states(6,:)-cfg.model.yawRateMaximum],[],1)];
    if problem.hasTarget
        shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
        reserve=of.clearanceMeters+of.intersampleMarginMeters;
        for j=1:3
            q=data.targets(:,j);x=data.checks(:,j);
            c=[c;-predictiveSafetyGeometry.separationWitnessRows(x(1:3),shape,q(1:3),q(8:11),witness(:,j),reserve)]; %#ok<AGROW>
        end
    end
end

function [point,info,trajectory,warm] = localOfNonlinearOptimize(problem,initial,timer,priorWarm)
    % Thompson-style numerical route for OUR finite joint policy problem:
    % cached parameterized NLP, automatic derivatives, IPOPT and warm starts.
    assert(exist('casadi.SX','class')==8,'collisionAvoidanceController:missingCasadi', ...
        'Run scripts/setupJointNlpSolver before selecting outputFeedback.method=ipopt.');
    cfg=problem.cfg;of=cfg.outputFeedback;buildTimer=tic;
    [kernel,cacheHit]=localOfNlpKernel(problem);buildSeconds=toc(buildTimer);
    [parameters,seed]=localOfNlpData(problem,kernel);
    lower=[problem.lower;kernel.stateLower];upper=[problem.upper;kernel.stateUpper];
    seed=min(upper,max(lower,seed));
    solverArguments={'x0',seed,'p',parameters,'lbx',lower,'ubx',upper, ...
        'lbg',kernel.constraintLower,'ubg',kernel.constraintUpper};
    dualWarm=false;
    if ~isempty(priorWarm) && isequal(priorWarm.key,kernel.key)
        lx=zeros(size(seed));lg=zeros(size(kernel.constraintLower));
        pi=problem.policyIndices;wi=problem.witnessIndices;ix=kernel.stateIndices;
        lx(pi(:,1:end-1))=priorWarm.lambdaX(pi(:,2:end));
        if problem.hasTarget,lx(wi(:,:,1:end-1,:))=priorWarm.lambdaX(wi(:,:,2:end,:));end
        lx(ix(:,1:end-1,:))=priorWarm.lambdaX(ix(:,2:end,:));
        gi=kernel.constraintIndices;
        lg(gi(:,1:end-1,:))=priorWarm.lambdaG(gi(:,2:end,:));
        solverArguments=[solverArguments,{'lam_x0',lx,'lam_g0',lg}];dualWarm=true;
    end
    solveTimer=tic;solution=kernel.solver(solverArguments{:});solveSeconds=toc(solveTimer);
    stats=kernel.solver.stats();trial=full(solution.x);
    validationTimer=tic;
    [valid,trajectory,violation,objective]=localOfValidateNlpPolicy(trial(1:numel(problem.initial)),problem);
    trialViolation=violation;
    fallback=false;point=trial(1:numel(problem.initial));
    if ~valid
        [valid,trajectory,violation,objective]=localOfValidateNlpPolicy(problem.initial,problem);
        fallback=valid;point=problem.initial;
    end
    assert(valid,'collisionAvoidanceController:noOutputFeedbackPlan', ...
        'IPOPT returned %s; nonlinear violation: candidate %g, shifted seed %g (limit %g). No command was committed.', ...
        stats.return_status,trialViolation,violation,of.nlpTolerance);
    validationSeconds=toc(validationTimer);warm=[];
    if ~fallback
        warm=struct('key',{kernel.key},'lambdaX',full(solution.lam_x),'lambdaG',full(solution.lam_g));
    end
    residual=full(solution.g);equality=kernel.constraintLower==kernel.constraintUpper;
    info=struct('solver',"casadiIpopt",'nlpStatus',string(stats.return_status), ...
        'nlpConverged',logical(stats.success),'nlpIterations',stats.iter_count, ...
        'acceptedSteps',double(~fallback), ...
        'maximumShootingEqualityResidual',max([0;abs(residual(equality))]), ...
        'maximumConstraintViolation',violation,'objective',objective, ...
        'candidateMaximumConstraintViolation',trialViolation,'candidateObjective',full(solution.f), ...
        'shiftedPlanRevalidated',fallback,'usedRevalidatedFallback',fallback, ...
        'nonlinearValidationPerformed',true,'horizonSteps',problem.horizon, ...
        'hardCollisionConstraints',true,'terminalControllerActive',false, ...
        'nlpSeconds',solveSeconds, ...
        'nlpBuildSeconds',buildSeconds,'nonlinearValidationSeconds',validationSeconds, ...
        'nlpCacheHit',cacheHit,'nlpDualWarmStart',dualWarm, ...
        'nlpVariables',numel(seed),'stageFlowEvaluations',NaN, ...
        'predictionIsLinearized',false,'auxiliaryNodesAreNominal',false, ...
        'automaticDerivatives',true,'initialization',initial,'numericalSeedRepaired',problem.seedRepaired, ...
        'trackingReferenceScope',"nominalLocalPathTangent", ...
        'deadlineSeconds',min(of.maximumSolveSeconds,cfg.controller.sampleTime), ...
        'deadlineMissed',toc(timer)>min(of.maximumSolveSeconds,cfg.controller.sampleTime));
end

function [valid,traces,violation,objective] = localOfValidateNlpPolicy(point,problem)
    valid=false;traces={};violation=Inf;objective=Inf;
    if any(~isfinite(point)),return;end
    try
        traces=localOfRollout(point,problem);
        violation=max([0;problem.lower-point;point-problem.upper]);
        objective=.5*problem.cfg.outputFeedback.regularization* ...
            norm(point(1:problem.policyCount)-problem.initial(1:problem.policyCount))^2;
        for s=1:numel(traces)
            trace=traces{s};previous=problem.previousInput;
            for i=1:problem.horizon
                witness=[];
                if problem.hasTarget,witness=reshape(point(problem.witnessIndices(:,:,i,s)),2,3);end
                data=struct('next',trace.ego(:,i+1), ...
                    'states',trace.physicalNodes(:,:,i),'checks',trace.checks(:,:,i),'targets',trace.targetChecks(:,:,i));
                [r,c]=localOfStageValues(trace.inputs(:,i),previous,witness,data,problem);
                objective=objective+.5*(r.'*r)*numel(problem.scenarios)*problem.scenarios(s).weight;
                violation=max(violation,max(c));previous=trace.inputs(:,i);
            end
        end
        valid=isfinite(violation) && violation<=problem.cfg.outputFeedback.nlpTolerance;
    catch exception
        if ~startsWith(exception.identifier,'collisionAvoidanceController:'),rethrow(exception);end
    end
end

function [kernel,hit] = localOfNlpKernel(problem)
    persistent keys kernels
    key={problem.cfg,problem.horizon,numel(problem.scenarios),problem.hasTarget};
    hit=false;
    for j=1:numel(keys)
        if isequaln(keys{j},key),kernel=kernels{j};hit=true;return;end
    end
    import casadi.*
    cfg=problem.cfg;of=cfg.outputFeedback;n=problem.horizon;ns=numel(problem.scenarios);
    nm=of.memoryDimension;nx=8+nm;np=numel(problem.initial);
    ix=reshape(np+(1:nx*n*ns),nx,n,ns);w=MX.sym('decision',np+numel(ix));
    stage=localOfSymbolicStage(cfg,problem.hasTarget);
    stageParameterCount=stage.size1_in(3);rootCount=6*ns+nm+2;
    p=MX.sym('parameter',rootCount+stageParameterCount*n*ns+problem.policyCount);
    pi=reshape(rootCount+(1:stageParameterCount*n*ns),stageParameterCount,n,ns);
    memory=p(6*ns+(1:nm));input=p(6*ns+nm+(1:2));
    g=cell(n,ns);cost=0;
    for s=1:ns
        state=[p((s-1)*6+(1:6));memory;input];
        for i=1:n
            witness=MX.zeros(6,1);
            if problem.hasTarget,witness=w(reshape(problem.witnessIndices(:,:,i,s),[],1));end
            [next,c,r]=stage(state,w(problem.policyIndices(:,i)),witness,p(pi(:,i,s)));
            g{i,s}=[w(ix(:,i,s))-next;c];cost=cost+p(pi(end,i,s))*dot(r,r)/2;
            state=w(ix(:,i,s));
        end
    end
    anchor=p(end-problem.policyCount+1:end);
    cost=cost+.5*of.regularization*dot(w(1:problem.policyCount)-anchor,w(1:problem.policyCount)-anchor);
    nc=numel(g{1});constraint=vertcat(g{:});
    gl=repmat([zeros(nx,1);-Inf(nc-nx,1)],n*ns,1);gu=zeros(size(gl));
    options=struct('print_time',false,'error_on_fail',false,'jit',of.nlpJit,'calc_lam_p',false, ...
        'ipopt',struct('print_level',0,'sb','yes','max_iter',of.nlpMaximumIterations, ...
        'tol',of.nlpTolerance,'constr_viol_tol',of.nlpTolerance, ...
        'bound_relax_factor',0,'warm_start_init_point','yes','mu_init',1e-3));
    if isfinite(of.maximumSolveSeconds),options.ipopt.max_wall_time=of.maximumSolveSeconds;end
    if of.nlpJit
        directory=tempname;mkdir(directory);
        options.compiler='shell';options.jit_options=struct( ...
            'flags',sprintf('-O%d',of.nlpCompilerOptimization),'compiler',char(of.nlpCompiler),'directory',[directory,filesep]);
        % CasADi writes the generated source in the current directory even
        % when the compiler output directory is set. Keep both outside Git.
        previousDirectory=pwd;restoreDirectory=onCleanup(@()cd(previousDirectory));
        cd(directory);
    end
    solver=nlpsol('joint_output_feedback','ipopt',struct('x',w,'p',p,'f',cost,'g',constraint),options);
    if of.nlpJit,clear restoreDirectory;end
    sl=[-Inf(3,1);0;-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum; ...
        -ones(nm,1);-of.steeringMaximum;max(-1,cfg.actuation.brakingRatioMinimum)];
    su=[Inf(3,1);cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum; ...
        ones(nm,1);of.steeringMaximum;min(1,cfg.actuation.brakingRatioMaximum)];
    kernel=struct('key',{key},'solver',solver,'stateIndices',ix,'parameterIndices',pi, ...
        'parameterCount',numel(p),'constraintLower',gl,'constraintUpper',gu, ...
        'constraintIndices',reshape(1:numel(gl),nc,n,ns), ...
        'stateLower',repmat(sl,n*ns,1),'stateUpper',repmat(su,n*ns,1));
    if numel(keys)>=8,keys=keys(2:end);kernels=kernels(2:end);end
    keys{end+1}=key;kernels{end+1}=kernel;
end

function [parameters,seed] = localOfNlpData(problem,kernel)
    cfg=problem.cfg;ns=numel(problem.scenarios);nm=cfg.outputFeedback.memoryDimension;
    l=cfg.outputFeedback.predictionSubsteps;dt=cfg.controller.sampleTime/l;
    parameters=zeros(kernel.parameterCount,1);seed=zeros(numel(problem.initial)+numel(kernel.stateIndices),1);
    seed(1:numel(problem.initial))=problem.initial;
    parameters(6*ns+(1:nm+2))=[problem.memory;problem.previousInput];
    parameters(end-problem.policyCount+1:end)=problem.initial(1:problem.policyCount);
    for s=1:ns
        item=problem.scenarios(s).initial;trace=problem.nominalTrajectories{s};
        parameters((s-1)*6+(1:6))=item.ego;
        for i=1:problem.horizon
            targets=zeros(11,l+1);
            if problem.hasTarget
                q=trace.targets(:,i);targets(:,1)=q;
                for j=1:l
                    if q(4)>0,q=predictiveSafetyGeometry.predictTarget(q,dt);end
                    targets(:,j+1)=q;
                end
            end
            projection=predictiveSafetyGeometry.project(trace.ego(1:2,i+1),problem.lane);
            parameters(kernel.parameterIndices(:,i,s))=[item.parameters;item.sensorBias; ...
                problem.scenarios(s).noise;targets(:);projection.point;projection.heading;problem.scenarios(s).weight];
            seed(kernel.stateIndices(:,i,s))=[trace.ego(:,i+1);trace.memory(:,i+1);trace.inputs(:,i)];
        end
    end
end

function stage = localOfSymbolicStage(cfg,hasTarget)
    import casadi.*
    of=cfg.outputFeedback;nm=of.memoryDimension;l=of.predictionSubsteps;
    z=SX.sym('state',8+nm);alpha=SX.sym('policy',2+2*nm+nm^2+9*nm);
    witness=SX.sym('witness',6);p=SX.sym('data',18+11*(l+1)+4);
    policy=localOfUnpack(alpha,cfg);x=z(1:6);m=z(7:6+nm);previous=z(7+nm:8+nm);
    u=policy.v+policy.K*m;h=cfg.controller.sampleTime;dt=h/l;
    states=x;feature=SX.zeros(8,1);q=reshape(p(19:18+11*(l+1)),11,l+1);
    for j=1:l
        x=nonlinearBicycleModel.symbolicSensorFlow(x,u,p(1:4),cfg,dt,j==l);states=[states,x]; %#ok<AGROW>
        dx=nonlinearBicycleModel.symbolicDerivative(x,u,p(1:4),cfg);
        relative=SX.zeros(2,1);detected=SX(0);
        if hasTarget
            relative=[cos(x(3)),sin(x(3));-sin(x(3)),cos(x(3))]*(q(1:2,j+1)-x(1:2));
            detected=dot(relative,relative)<=cfg.collision.encounterRangeMeters^2;
        end
        y=[x(1:2);dx(4)-x(6)*x(5);dx(5)+x(6)*x(4);x(6);relative]+p(5:11)+p(12:18);
        y=[y(1:5);if_else(detected,y(6:7),zeros(2,1))];
        feature=feature+[y./of.featureScale;detected]/l;
    end
    next=[x;tanh(policy.A*m+policy.B*feature+policy.c);u];
    ref=p(end-3:end-1);normal=[-sin(ref(3));cos(ref(3))];
    error=[normal.'*(x(1:2)-ref(1:2));atan2(sin(x(3)-ref(3)),cos(x(3)-ref(3))); ...
        x(4)-cfg.referenceSpeed;x(5);x(6)];
    residual=[sqrt(h*of.trackingWeights).*error;sqrt(of.inputWeight)*(u-previous)];
    c=[u(1)-of.steeringMaximum;-u(1)-of.steeringMaximum; ...
        u(2)-min(1,cfg.actuation.brakingRatioMaximum);max(-1,cfg.actuation.brakingRatioMinimum)-u(2)];
    if isfinite(of.steeringRateMaximum)
        change=h*of.steeringRateMaximum;c=[c;u(1)-previous(1)-change;previous(1)-u(1)-change];
    end
    if isfinite(cfg.model.brakingRatioRateMaximum)
        change=h*cfg.model.brakingRatioRateMaximum;c=[c;u(2)-previous(2)-change;previous(2)-u(2)-change];
    end
    c=[c;reshape([-states(4,:);states(4,:)-cfg.model.speedMaximum; ...
        states(5,:)-cfg.model.lateralVelocityMaximum;-states(5,:)-cfg.model.lateralVelocityMaximum; ...
        states(6,:)-cfg.model.yawRateMaximum;-states(6,:)-cfg.model.yawRateMaximum],[],1)];
    if hasTarget
        signs=[-1,1,1,-1;-1,-1,1,1];body=cfg.vehicle.rectangleOffset ...
            +[cfg.vehicle.length/2;cfg.vehicle.width/2].*signs;
        reserve=of.clearanceMeters+of.intersampleMarginMeters;checks=[1,1+l/2,1+l];
        for j=1:3
            pose=states(1:3,checks(j));target=q(:,checks(j));
            re=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
            rt=[cos(target(3)),-sin(target(3));sin(target(3)),cos(target(3))];
            normal=[cos(witness(2*j-1));sin(witness(2*j-1))];offset=witness(2*j);
            ego=pose(1:2)-target(1:2)+re*body;other=rt*(target(10:11)+target(8:9).*signs);
            c=[c;(offset+reserve-normal.'*ego).';(normal.'*other-offset).']; %#ok<AGROW>
        end
    end
    stage=Function('joint_stage',{z,alpha,witness,p},{next,c,residual});
end

function [result,info] = localCompactStep(request,cfg)
    % Fixed inference and a condensed PCBF solve with exact geometry dual values.
    timer=request.callTimer;block=request.block;prior=request.previousState;initial=isempty(prior);
    localOfValidateBlock(block,cfg);
    sensorCount=cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod;
    assert(abs(sensorCount-round(sensorCount))<1e-10 && sensorCount>=2 && mod(round(sensorCount),2)==0, ...
        'collisionAvoidanceController:invalidSensorGrid','A hold needs an even integer number of sensor samples.');
    assert(~cfg.controller.enforceRoadBoundaries, ...
        'collisionAvoidanceController:unsupportedCompactRoadConstraint', ...
        'Hard road-boundary constraints are not implemented by this transcription.');
    if isempty(request.reference),request.reference=[-1000,0;1000,0];end
    if initial
        assert(isfield(block,'initialHypotheses'), ...
            'collisionAvoidanceController:missingInitialInformation','Supply an explicit initial information contract.');
        assert(~isfield(block,'initialMemory') && ~isfield(block,'initialPolicies'), ...
            'collisionAvoidanceController:unsupportedCompactMemory','The compact controller has no optimized memory map.');
        hypotheses=localOfInitialHypotheses(block,cfg);
        assert(numel(block.samples)==1 && abs(block.samples.time-block.time)<1e-9, ...
            'collisionAvoidanceController:invalidInitialBlock','Initialization uses the current reading.');
        inherited=0;
    else
        assert(isfield(prior,'kind') && prior.kind=="compactOutputFeedback-v1", ...
            'collisionAvoidanceController:incompatibleContinuation','Expected a compact controller continuation.');
        previousCfg=prior.configuration;
        % A search deadline does not enter the dynamics or plan certificate.
        previousCfg.outputFeedback.maximumSolveSeconds=cfg.outputFeedback.maximumSolveSeconds;
        previousCfg.outputFeedback.initializationSolveSeconds=cfg.outputFeedback.initializationSolveSeconds;
        % Clearance changes preserve inference and dynamics. A stored plan
        % is only a candidate until replay under the current margin passes.
        previousCfg.outputFeedback.clearanceMeters=cfg.outputFeedback.clearanceMeters;
        assert(isequaln(previousCfg,cfg) && isequaln(prior.reference,request.reference), ...
            'collisionAvoidanceController:changedContinuationProblem','The continuation contract changed.');
        assert(~isfield(block,'initialHypotheses') && ~isfield(block,'initialMemory') && ~isfield(block,'initialPolicies'), ...
            'collisionAvoidanceController:informationReset','Continuation cannot reset its information.');
        sensorCount=round(cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod);
        times=prior.time+(1:sensorCount)*cfg.outputFeedback.sensorPeriod;
        assert(abs(block.time-prior.time-cfg.controller.sampleTime)<1e-9 && ...
            numel(block.samples)==numel(times) && max(abs([block.samples.time]-times))<1e-9, ...
            'collisionAvoidanceController:invalidSensorGrid','Receive the complete consecutive block.');
        assert(norm(block.heldInput-prior.appliedInput,Inf)<1e-12, ...
            'collisionAvoidanceController:actuationMismatch','Measurements must use the committed input.');
        hypotheses=prior.hypotheses;inherited=size(prior.fineControls,2)-1;
        if ~isfield(prior,'targetEstimateReady') && isfield(hypotheses,'targetObserver') && ...
                any(arrayfun(@(s)~isempty(s.targetObserver) && ~s.targetObserver.initialized,hypotheses))
            [hypotheses.target]=deal([]);
            prior.collisionTargetActive=false;
            request.previousState=prior;
        end
        if ~isfield(prior,'firstRadarTime') && ~isempty(hypotheses(1).target) && ~any([block.samples.radarAvailable])
            % Older checkpoints could carry a target and an avoidance plan
            % without ever receiving radar. Preserve only their ego history.
            [hypotheses.target]=deal([]);
            prior.movingTerminalPlan=false;
            request.previousState=prior;
            inherited=0;
        end
    end
    clock=tic;[hypotheses,conditioning]=localOfCondition(hypotheses,block,cfg,initial);
    inferenceSeconds=toc(clock);
    % A dropout alone gives no range information. Keep the physical target
    % in the posterior even when all surviving members place it outside.
    planning=hypotheses;
    inRange=any(arrayfun(@(s)~isempty(s.target) && ...
        norm(s.target(1:2)-s.ego(1:2))<=cfg.collision.encounterRangeMeters,hypotheses));
    firstRadarTime=[];
    if ~initial && isfield(prior,'firstRadarTime'),firstRadarTime=prior.firstRadarTime;end
    if isempty(firstRadarTime)
        hit=find([block.samples.radarAvailable],1);
        if ~isempty(hit),firstRadarTime=block.samples(hit).time;end
    end
    targetReady=all(arrayfun(@(s)~isempty(s.target),hypotheses));
    active=targetReady && ~isempty(firstRadarTime) && (block.samples(end).radarAvailable || inRange);
    if active && ~initial && ~(isfield(prior,'collisionTargetActive') && prior.collisionTargetActive) ...
            && isfield(hypotheses,'targetObserver') && ...
            all(arrayfun(@(s)~isempty(s.targetObserver) && s.target(4)>0,hypotheses))
        % Count received radar history. A dropout alone adds no observation.
        observedUntil=min(arrayfun(@(s)s.targetObserver.lastReturn,hypotheses));
        active=observedUntil-firstRadarTime >= cfg.outputFeedback.movingTargetObservationSeconds-1e-9;
    end
    if ~active,[planning.target]=deal([]);end
    window=localOfWindow(planning,inherited,cfg);
    holds=diff(localCompactBoundaries(window.horizonSteps,cfg,window.avoidanceSteps));
    % A representative retained member supplies the nominal dynamics. All
    % retained members are replayed before accepting a control sequence.
    values=[hypotheses.ego];[~,nominal]=min(sum((values-mean(values,2)).^2,1));item=hypotheses(nominal);
    x=item.ego;
    stub=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
    [~,lane]=readControllerInputs(stub,[],request.reference,cfg);
    planning=localCompactPlanningMembers(planning,cfg,sum(holds));
    [result,info]=localCondensedSolve(request,cfg,planning,conditioning,window,holds,lane,nominal,timer,inferenceSeconds);
    info.conditioning=conditioning;result.state.conditioning=conditioning;
    info.hypothesisCount=numel(hypotheses);
    info.collisionTargetActive=active;
    info.targetEstimateReady=targetReady;
    info.firstRadarTime=firstRadarTime;
    result.prediction.metadata=info;
    result.prediction.information=hypotheses;
    result.state.hypotheses=hypotheses;
    result.state.targetEstimateReady=targetReady;
    result.state.firstRadarTime=firstRadarTime;
    result.state.targetEnteredRange=~isempty(firstRadarTime);
    result.state.collisionTargetActive=active;
end

function members=localCompactPlanningMembers(members,cfg,count)
    for j=1:numel(members)
        members(j).motionLimits=[];members(j).cover=[];
        q=members(j).target;track=[];
        if isfield(members,'targetObserver'),track=members(j).targetObserver;end
        if isempty(q) || q(4)==0 || isempty(track),continue;end
        if isfield(track,'motionBounds') && ~isempty(track.motionBounds)
            b=track.motionBounds;
        else
            settings=estimatorControllerIntegrationConfig();domain=settings.trafficTargetObserver.domain;
            limit=[domain.scalarAccelerationMaximum,domain.sideslipMaximum];
            b=[-limit;limit];
        end
        members(j).target(5:6)=min(b(2,:).',max(b(1,:).',q(5:6)));
        if any(b(1,:)~=b(2,:))
            members(j).motionLimits=b;
            h=cfg.controller.sampleTime;
            members(j).cover=localCompactCover(members(j),h,count+ceil(cfg.outputFeedback.minimumPreviewSeconds/h));
        end
    end
    if isfield(members,'targetObserver'),members=rmfield(members,'targetObserver');end
end

function contained=localCompactShiftContained(prior,members,indices,h)
    contained=isfield(prior,'retainedTrajectories');
    if ~contained,return;end
    for j=1:numel(members)
        old=prior.retainedTrajectories{indices(j)};item=members(j);
        if ~isequal(old.collisionEgo(:,2),item.ego) || ~isequal(old.parameters,item.parameters)
            contained=false;return
        end
        if isempty(item.target),continue;end
        if ~isempty(item.motionLimits),contained=false;return;end
        if ~isfield(old,'target') || isempty(old.target),contained=false;return;end
        pred=localCompactTargetNodes(old.target,h);
        if pred(4)==0 && item.target(4)==0,pred(5:6)=item.target(5:6);end
        delta=pred-item.target;delta(3)=atan2(sin(delta(3)),cos(delta(3)));
        if any(abs(delta)>128*eps(max(1,abs(pred)))),contained=false;return;end
    end
end

function boundaries=localCompactBoundaries(count,cfg,avoidance)
    if nargin<3,avoidance=count;end
    nodes=min(avoidance,cfg.outputFeedback.controlNodes);
    near=min(nodes-1,cfg.outputFeedback.nearControlNodes);
    boundaries=[0:near,near+round((1:nodes-near)*(avoidance-near)/(nodes-near))];
    connection=count-avoidance;
    if connection>0
        % Keep the avoidance coordinates. The connection has independent
        % endpoint inputs at the same coarse temporal resolution.
        spacing=max(diff(boundaries));
        knots=min(cfg.outputFeedback.controlNodes,ceil(connection/spacing));
        boundaries=[boundaries,avoidance+round((1:knots)*connection/knots)];
    end
end

function [result,info] = localCondensedSolve(request,cfg,hypotheses,conditioning,window,holds,lane,nominal,timer,inferenceSeconds)
    clock=tic;[kernel,cacheHit]=localCompactKernel(cfg,holds,~isempty(hypotheses(1).target));buildSeconds=toc(clock);
    count=sum(holds);nodes=numel(holds);nu=2*nodes;previous=request.block.heldInput;prior=request.previousState;
    initial=isempty(prior);h=cfg.controller.sampleTime;boundaries=[0,cumsum(holds)];
    model=localCondensedModel(cfg,hypotheses,previous,holds,lane,nominal,kernel);
    item=hypotheses(nominal);
    tolerance=cfg.outputFeedback.nlpTolerance;
    best=[];traces={};violation=Inf;validationSeconds=0;retained=false;
    inheritedReusedHolds=zeros(1,numel(hypotheses));solveClock=tic;
    startupBuildSeconds=0;
    firstEncounter=~initial && isfield(prior,'collisionTargetActive') && ~prior.collisionTargetActive && ~isempty(item.target);
    budget=cfg.outputFeedback.maximumSolveSeconds;budgetPhase="continuation";
    if initial || firstEncounter
        budget=cfg.outputFeedback.initializationSolveSeconds;budgetPhase="initialization";
        if ~cacheHit,startupBuildSeconds=buildSeconds;end
    end
    % A previous implementation could store a parking sequence. Preserve the
    % posterior when resuming such states and construct a fresh driving plan.
    inherit=~initial && isfield(prior,'movingTerminalPlan') && prior.movingTerminalPlan;
    attempts=0;iterationCount=0;calls=0;reuseCount=0;restoration=false;seedCount=0;gradientCalls=0;
    solverStatus="notCalled";converged=false;ipoptSeconds=0;constraintCount=0;bestU=[];
    cacheU=[];cache=[];rollouts=[];includeTracking=false;initialDeficit=NaN;initialTerminal=NaN;initialPeakDeficit=NaN;
    stabilitySteps=0;initialStabilitySteps=0;firstFeasibleStability=NaN;incumbentStability=NaN;
    bestCost=Inf;inheritedCost=NaN;deadline=budget+startupBuildSeconds;
    budgetStopped=false;budgetStopPhase="";
    bestLoss=Inf;bestSlacks=[];priorCap=Inf;slackRoundoff=128*eps(max(1,cfg.outputFeedback.clearanceMeters))*count;
    contained=inherit && ~firstEncounter && cfg.outputFeedback.clearanceMeters==prior.configuration.outputFeedback.clearanceMeters ...
        && localCompactShiftContained(prior,hypotheses,conditioning.retainedIndices,h);
    sameEncounter=inherit && ~firstEncounter && cfg.outputFeedback.clearanceMeters==prior.configuration.outputFeedback.clearanceMeters;
    if sameEncounter
        % Old checkpoints admitted zero-deficit plans only. A new posterior
        % needs fresh replay even when it falls outside the sampled shift.
        priorCap=0;
        if isfield(prior,'stageDeficits'),priorCap=sum(prior.stageDeficits(2:end));end
    end
    lossFloor=0;
    if ~isempty(item.target)
        roots=[hypotheses.ego];targets=[hypotheses.target];
        shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
        rootGap=min(predictiveSafetyGeometry.rectangleClearance(roots,targets,shape));
        lossFloor=max(0,cfg.outputFeedback.clearanceMeters-rootGap);
        if rootGap<=0 || lossFloor>priorCap+slackRoundoff
            error('collisionAvoidanceController:compactNoFeasiblePlan', ...
                'Current node clearance %.9g m requires deficit %.9g m with remaining budget %.9g m.',rootGap,lossFloor,priorCap);
        end
    end
    if inherit && ~firstEncounter
        suffix=prior.fineControls(:,2:end);
        extra=count-size(suffix,2);
        cruise=terminalSafeSet.movingControl(cfg);
        last=prior.fineControls(:,end);
        extension=localCompactRateLimit(repmat(cruise,1,extra),last,ones(1,extra),cfg);
        inherited=[suffix,extension];
        initializationSource="shiftedMovingTerminalLaw";
        if toc(timer)<budget
            guide=localCompactSeed(item,previous,1,lane,cfg);
            proposal=inherited;proposal(:,1)=guide;
            proposal=localCompactRateLimit(proposal,previous,ones(1,count),cfg);
            if ~isequal(proposal,inherited)
                validatePlan(proposal,[]);
                if ~isempty(best),initializationSource="guideWithDrivingSuffix";end
            end
        end
        % If the guide splice passes, compare the shifted driving plan while
        % there is time. A validated incumbent survives an interrupted replay.
        compare=~isempty(best) && ~isempty(item.target) && toc(timer)<deadline;
        if bestLoss>lossFloor+slackRoundoff || compare
            try
                clockCheck=[];
                if compare && bestLoss<=lossFloor+slackRoundoff,clockCheck=@checkClock;end
                [shifted,reused]=localCompactShiftedCache(prior,hypotheses,conditioning,inherited,kernel,cfg,clockCheck);
                [selected,inheritedCost]=validatePlan(inherited,shifted,clockCheck);
                if selected
                    retained=true;inheritedReusedHolds=reused;
                    initializationSource="shiftedMovingTerminalLaw";
                end
            catch exception
                if ~compare || ~strcmp(exception.identifier,'collisionAvoidanceController:condensedDeadline'),rethrow(exception);end
            end
        end
    end
    if bestLoss>lossFloor+slackRoundoff
        if isempty(item.target),initializationSource="nominalCruiseRollout";
        else,initializationSource="singleCirculationRollout";end
        try
            checkClock();
            [controls,guideInputs]=localCompactSeed(item,previous,holds,lane,cfg,window.avoidanceSteps);
            seedCount=1;
            if isempty(item.target)
                validatePlan(controls*model.map.'+model.offset,[],@checkClock);
            end
            if bestLoss>lossFloor+slackRoundoff
                [a,b]=localCondensedInputRows(previous,holds,cfg);
                if ~isempty(item.target)
                    % Interpolate actuator corrections about the field rollout.
                    % Zero correction reproduces its held commands at every
                    % node, including between knots of the condensed problem.
                    guideOffset=guideInputs-(controls*model.map.'+model.offset);
                    model.offset=model.offset+guideOffset;
                    [a,b]=localGuideInputRows(model,guideOffset,a,b);
                end
                lower=repmat([-cfg.outputFeedback.steeringMaximum;max(-1,cfg.actuation.brakingRatioMinimum)],nodes,1);
                upper=repmat([cfg.outputFeedback.steeringMaximum;min(1,cfg.actuation.brakingRatioMaximum)],nodes,1);
                start=controls(:);
                controlScale=ones(nu,1);
                % Express steering increments in units of the bicycle's
                % friction-limited turn angle. Bounds use the same coordinates.
                speedSquared=sum(item.ego(4:5).^2);
                lateralGrip=min(cfg.tire.frictionCoefficient)*cfg.vehicle.gravity;
                steerScale=atan2(cfg.vehicle.wheelbase*lateralGrip,speedSquared);
                controlScale(1:2:end)=min(cfg.outputFeedback.steeringMaximum,steerScale);
                e=evaluate(start,false);
                initialPeakDeficit=max(0,max(e.stage));
                initialDeficit=sum(max(0,e.stage));initialTerminal=max(e.terminal);
                considerSolution(start);
                restoration=true;
                if bestLoss>lossFloor+slackRoundoff
                    solveIpopt(start,false);
                end
                if ~isempty(bestU) && bestLoss<=lossFloor+slackRoundoff && toc(timer)<deadline
                    solveIpopt(bestU,true);
                end
            end
        catch exception
            if ~strcmp(exception.identifier,'collisionAvoidanceController:condensedDeadline'),rethrow(exception);end
        end
    end
    if isempty(best)
        error('collisionAvoidanceController:compactNoFeasiblePlan', ...
            'No validated moving PCBF plan within the %s search. No command was committed.',budgetPhase);
    end
    solveSeconds=toc(solveClock);
    retainedTrajectories=cell(1,numel(hypotheses));
    for s=1:numel(hypotheses)
        trace=traces{s};
        retainedTrajectories{s}=struct('inputs',trace.inputs,'denseEgo',trace.denseEgo, ...
            'collisionEgo',trace.collisionEgo,'validationSubsteps',kernel.validationSubsteps, ...
            'parameters',hypotheses(s).parameters,'target',hypotheses(s).target);
    end
    input=best(:,1);state=struct('kind',"compactOutputFeedback-v1",'time',request.block.time,'memory',zeros(0,1), ...
        'hypotheses',hypotheses,'appliedInput',input,'controls',best(:,boundaries(2:end)), ...
        'fineControls',best,'movingTerminalPlan',true,'firstFeasibleDrivingPlan',stabilitySteps==0,'boundaries',boundaries,'horizonSteps',count,'configuration',cfg, ...
        'reference',request.reference,'conditioning',conditioning,'terminal',traces{nominal}.terminal, ...
        'pcbfDeficit',bestLoss,'stageDeficits',bestSlacks, ...
        'retainedTrajectories',{retainedTrajectories});
    info=struct('solver',"casadiIpoptCompactPcbf",'ipoptStatus',solverStatus,'ipoptSeconds',ipoptSeconds,'nlpStatus',"feasible", ...
        'nlpConverged',converged,'nlpIterations',iterationCount,'nlpSeconds',solveSeconds, ...
        'nlpAttempts',attempts,'nlpBuildSeconds',buildSeconds,'nlpCacheHit',cacheHit, ...
        'nlpVariables',nu,'nlpConstraints',constraintCount, ...
        'informationUpdateSeconds',inferenceSeconds,'nonlinearValidationSeconds',validationSeconds, ...
        'inheritedReusedHolds',sum(inheritedReusedHolds), ...
        'elapsedSeconds',toc(timer),'deadlineSeconds',h,'predictionWindow',window, ...
        'controlNodes',nodes,'horizonSteps',count,'initializationTrajectoryCount',seedCount,'initializationSource',initializationSource, ...
        'reducedSearchEvaluations',0,'reducedSearchAccepted',false, ...
        'fallbackUsed',retained && stabilitySteps==0,'retainedFinePlanUsed',retained && ~initial,'rootTerminalPlanUsed',false, ...
        'retainedTailRepaired',false, ...
        'restorationAttempted',restoration,'initialRestorationSlack',initialDeficit, ...
        'initialPeakClearanceDeficitMeters',initialPeakDeficit, ...
        'initialNlpViolation',max(0,initialTerminal),'finalRestorationSlack',bestLoss, ...
        'validationSubsteps',kernel.validationSubsteps,'sensitivitySubsteps',kernel.sensitivitySubsteps,'nonlinearValidationPerformed',true, ...
        'maximumConstraintViolation',violation,'scenarioCount',numel(hypotheses), ...
        'activeScenarioCount',numel(hypotheses),'hypothesisCount',numel(hypotheses),'memoryDimension',0, ...
        'conditioning',conditioning,'terminalSetEnforced',true,'terminalSafetyAssumed',true, ...
        'terminalModel',"safeRangeExit",'terminalValue',max(cellfun(@(t)t.terminal.value,traces)), ...
        'terminalInput',traces{nominal}.terminal.input,'robustNonlinearSafetyCertified',false, ...
        'recursiveFeasibilityCertified',false,'safetyScope',"finiteHypothesesControlBoundaryCollisionChecks", ...
        'collisionCheckPeriodSeconds',h,'collisionNodeCount',count+1, ...
        'wholeHoldReserveScope',"notAppliedNodeCollisionContract", ...
        'predictionModel',"condensedSharedInputNonlinearRollout",'previousInputUsedForMeasurements',previous, ...
        'optimizationReturned',true,'pcbfValue',bestLoss,'primaryOptimumComputed',bestLoss<=lossFloor+slackRoundoff, ...
        'shiftContained',contained,'inheritedDeficitCap',priorCap,'deficitDescentMet',bestLoss<=priorCap+slackRoundoff, ...
        'inheritedPerformanceCost',inheritedCost,'acceptedPerformanceCost',bestCost, ...
        'firstFeasibleStabilityCost',firstFeasibleStability,'acceptedStabilityCost',incumbentStability, ...
        'performanceAcceptedSteps',stabilitySteps, ...
        'searchRestartedAtRest',false, ...
        'deferredTrialAccepted',false,'deferredTrialStored',false, ...
        'safetyObjective',"sumOfHorizonSafetySlacks",'nonlinearOracleEvaluations',calls, ...
        'oraclePropagationReuses',reuseCount,'nonlinearDerivativeEvaluations',gradientCalls,'callBudgetSeconds',budget, ...
        'startupBuildSeconds',startupBuildSeconds,'budgetPhase',budgetPhase, ...
        'budgetScope',"completeWarmControllerCall",'budgetStopped',budgetStopped,'budgetStopPhase',budgetStopPhase);
    info.callBudgetOverrunSeconds=max(0,info.elapsedSeconds-budget-startupBuildSeconds);
    info.deadlineMissed=info.elapsedSeconds>h;
    command=struct('actuatorInput',input,'steeringAngle',input(1),'brakingRatio',input(2));
    prediction=struct('metadata',info,'scenarios',{traces},'memory',zeros(0,1),'information',hypotheses);
    result=struct('command',command,'inputs',best,'prediction',prediction,'state',state);

    function [selected,tracking]=validatePlan(inputs,flows,clockCheck)
        if nargin<3,clockCheck=[];end
        clock=tic;
        [valid,checked,maximum,~,slacks]=localCompactValidate(inputs,hypotheses,previous,kernel,cfg,true,flows,clockCheck);
        validationSeconds=validationSeconds+toc(clock);
        selected=false;
        tracking=NaN;
        loss=sum(slacks);
        if ~valid || loss>bestLoss || loss>priorCap+slackRoundoff,return;end
        tracking=localCompactPerformance(checked{nominal}.ego(:,2:end),inputs,model);
        score=h/2*sum(inputs(2,:).^2+inputs(2,:).^4);
        for k=1:numel(hypotheses)
            residual=localRearSlipResidual(checked{k}.collisionEgo(:,2:end),model,k);
            score=score+h/(2*numel(hypotheses))*sum(residual.^2);
        end
        if ~isempty(best)
            if loss==bestLoss
                if score>=incumbentStability || tracking>bestCost+tolerance,return;end
                stabilitySteps=stabilitySteps+1;
            end
        else
            firstFeasibleStability=score;
        end
        selected=true;bestCost=tracking;incumbentStability=score;
        best=inputs;traces=checked;violation=maximum;bestLoss=loss;bestSlacks=slacks;
    end

    function solveIpopt(start,secondary)
        import casadi.*
        % IPOPT owns all search directions, restoration and step lengths.
        % Scale knots before exposing the exact nonlinear values to CasADi.
        checkClock();
        if exist('casadiFunctionCallback','class')~=8
            addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'scripts'));
        end
        includeTracking=secondary;
        if secondary,cacheU=[];end
        outputCount=2*count+numel(hypotheses)+1+double(secondary);
        oracle=casadiFunctionCallback('pcbf_rollout',[nu,1],[outputCount,1], ...
            @ipoptValues,@ipoptJacobian);
        oracleLifetime=onCleanup(@()oracle.release());
        v=MX.sym('knots',nu);value=oracle.call({v});value=value{1};
        inputRows=sparse(a.*controlScale.');
        terminalEnd=2*count+numel(hypotheses);
        hard=[value(count+1:terminalEnd);inputRows*v-b];
        z=v;lo=lower./controlScale;hi=upper./controlScale;z0=start./controlScale;
        % For fixed controls, xi_j=max(C_j,0) minimizes the original
        % epigraph. Its sum and the maximum stage deficit retain every cap.
        stage=value(1:count);
        deficit=sum(fmax(stage,0));
        cap=priorCap;
        maxIterations=cfg.outputFeedback.nlpMaximumIterations;
        if secondary
            % The performance pass proposes one step from the feasible
            % incumbent; primary iterations retain the configured limit.
            maxIterations=1;
            cap=bestLoss;
            hard=[hard;value(end)-bestCost-tolerance];
        end
        if cap==0
            constraints=[stage;hard];
        else
            constraints=[mmax(stage)-cfg.outputFeedback.clearanceMeters;hard];
            if isfinite(cap),constraints=[constraints;deficit-cap];end
        end
        objective=deficit;
        if secondary,objective=value(terminalEnd+1);end
        ng=size(constraints,1);nz=size(z,1);constraintCount=ng;
        % nlpsol outputs x, f, g, lam_x, lam_g, lam_p.
        initialStabilitySteps=stabilitySteps;
        stop=casadiFunctionCallback('pcbf_stop',[[nz;1;ng;nz;ng;0],ones(6,1)],[1,1], ...
            @stopIpopt);
        stopLifetime=onCleanup(@()stop.release());
        options=struct('print_time',false,'error_on_fail',false,'calc_lam_p',false, ...
            'iteration_callback',stop,'show_eval_warnings',false, ...
            'ipopt',struct('print_level',0,'sb','yes','max_iter',maxIterations, ...
            'hessian_approximation','limited-memory','bound_relax_factor',0, ...
            'mu_strategy','monotone','mu_init',1e-6,'bound_push',1e-6,'bound_frac',1e-6, ...
            'tol',tolerance,'constr_viol_tol',tolerance));
        % The offline prewarm uses realmax. Passing it through the CasADi
        % options converter can overflow; omit a wall limit for that case.
        if deadline<flintmax,options.ipopt.max_wall_time=max(.001,deadline-toc(timer));end
        nlp=nlpsol('compact_pcbf','ipopt',struct('x',z,'f',objective,'g',constraints),options);
        solverLifetime=onCleanup(@()delete(nlp));
        attempts=attempts+1;
        ipoptClock=tic;
        answer=nlp('x0',z0,'lbx',lo,'ubx',hi,'lbg',-inf(ng,1),'ubg',zeros(ng,1));
        ipoptSeconds=ipoptSeconds+toc(ipoptClock);
        stats=nlp.stats();solverStatus=string(stats.return_status);
        iterationCount=iterationCount+stats.iter_count;
        converged=stats.success;
        if toc(timer)>=deadline,budgetStopped=true;budgetStopPhase=budgetPhase;end
        % A solver status alone never authorizes an actuator command.
        point=full(answer.x);
        clear solverLifetime options objective constraints hard deficit stage value
        clear stopLifetime oracleLifetime
        if all(isfinite(point)) && toc(timer)<deadline
            evaluate(point(1:nu).*controlScale,false);
        end
    end

    % Keep callback state in the controller frame. A callback must not retain
    % the solver workspace that owns it across an encounter or MATLAB exit.
    function stop=stopIpopt(~)
        stop=double(toc(timer)>=deadline || (~includeTracking && bestLoss<=lossFloor+slackRoundoff) || ...
            (includeTracking && stabilitySteps>initialStabilitySteps));
    end

    function output=ipoptValues(args)
        e=evaluate(full(args{1}).*controlScale,false);
        output=[e.stage;e.forward;e.terminal;e.stabilityCost];
        if includeTracking,output=[output;e.cost];end
    end

    function output=ipoptJacobian(args)
        e=evaluate(full(args{1}).*controlScale,true);
        gradients=[e.stageGradient,e.forwardGradient,e.terminalGradient,e.stabilityGradient];
        if includeTracking,gradients=[gradients,e.costGradient];end
        output=(gradients.*controlScale).';
    end

    function checkClock()
        if toc(timer)>=deadline
            budgetStopped=true;budgetStopPhase=budgetPhase;
            error('collisionAvoidanceController:condensedDeadline','The feasibility search budget elapsed.');
        end
    end
    function result=evaluate(u,derivatives)
        if ~isequal(cacheU,u)
            [cache,rollouts]=localCondensedOracle(u,model,derivatives,[],[],includeTracking);
            cacheU=u;calls=calls+1;gradientCalls=gradientCalls+derivatives;
            if restoration,considerSolution(u);end
        elseif derivatives && ~cache.derivatives
            [cache,rollouts]=localCondensedOracle(u,model,true,rollouts,[],includeTracking);
            gradientCalls=gradientCalls+1;
        end
        result=cache;
    end
    function considerSolution(u)
        if sum(max(0,cache.stage))>bestLoss+tolerance || max(cache.stage)>=cfg.outputFeedback.clearanceMeters || ...
                max([cache.forward;cache.terminal;a*u-b;lower-u;u-upper])>tolerance
            return
        end
        inputs=reshape(u,2,nodes)*model.map.'+model.offset;
        [selected,~]=validatePlan(inputs,rollouts);reuseCount=reuseCount+1;
        if selected,bestU=u;end
    end
end

function model=localCondensedModel(cfg,hypotheses,previous,holds,lane,nominal,kernel)
    count=sum(holds);nodes=numel(holds);h=cfg.controller.sampleTime;boundaries=[0,cumsum(holds)];
    map=zeros(count,nodes);offset=zeros(2,count);row=0;
    for i=1:nodes
        for j=1:holds(i)
            row=row+1;fraction=j/holds(i);map(row,i)=fraction;
            if i==1,offset(:,row)=(1-fraction)*previous;else,map(row,i-1)=1-fraction;end
        end
    end
    controlMap=kron(map,eye(2));
    model=struct('cfg',cfg,'hypotheses',hypotheses,'previous',previous,'kernel',kernel, ...
        'map',map,'offset',offset,'controlMap',controlMap,'nominal',nominal,'reference',zeros(3,nodes));
    tire=nonlinearBicycleModel.tireParameters(cfg);
    scales=[hypotheses.parameters];
    model.rearSlipScale=(tire.longitudinalForceScale(2)/tire.corneringStiffness(2))*scales(4,:)./scales(2,:);
    x=hypotheses(nominal).ego;
    for i=1:nodes
        p=x(1:2)+boundaries(i+1)*h*cfg.referenceSpeed*[cos(x(3));sin(x(3))];
        projection=predictiveSafetyGeometry.project(p,lane);model.reference(:,i)=[projection.point;projection.heading];
    end
end

function [a,b]=localCondensedInputRows(previous,holds,cfg)
    n=numel(holds);a=zeros(0,2*n);b=zeros(0,1);rates=[cfg.outputFeedback.steeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    for i=1:n
        for k=1:2
            if ~isfinite(rates(k)),continue;end
            row=zeros(1,2*n);row(2*i-2+k)=1;origin=0;
            if i>1,row(2*i-4+k)=-1;else,origin=previous(k);end
            a=[a;row;-row];b=[b;rates(k)*cfg.controller.sampleTime*holds(i)+origin;rates(k)*cfg.controller.sampleTime*holds(i)-origin];
        end
    end

end

function [a,b]=localGuideInputRows(model,offset,a,b)
    % Bounds on knot values already constrain their convex interpolation.
    % Add the fine rows changed by centering the interpolation on the guide.
    cfg=model.cfg;map=model.controlMap;bias=model.offset(:);previous=model.previous;
    lower=[-cfg.outputFeedback.steeringMaximum;max(-1,cfg.actuation.brakingRatioMinimum)];
    upper=[cfg.outputFeedback.steeringMaximum;min(1,cfg.actuation.brakingRatioMaximum)];
    up=find(offset(:)>0);
    down=find(offset(:)<0);
    a=[a;map(up,:);-map(down,:)];
    b=[b;upper(1+mod(up-1,2))-bias(up);bias(down)-lower(1+mod(down-1,2))];
    rates=[cfg.outputFeedback.steeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    if any(isfinite(rates))
        delta=diff([zeros(2,1),offset],1,2);
        active=find(delta~=0 & isfinite(rates));channel=1+mod(active-1,2);
        changes=map-[zeros(2,size(map,2));map(1:end-2,:)];
        base=bias-[previous;bias(1:end-2)];
        span=cfg.controller.sampleTime*rates(channel);
        a=[a;changes(active,:);-changes(active,:)];
        b=[b;span-base(active);span+base(active)];
    end
end

function [kernel,hit] = localCompactKernel(cfg,holds,hasTarget)
    persistent keys kernels
    key=cfg;hit=false;
    for k=1:numel(keys)
        if isequaln(key,keys{k})
            kernel=kernels{k};kernel.holds=holds;hit=true;return;
        end
    end
    import casadi.*
    of=cfg.outputFeedback;h=cfg.controller.sampleTime;
    maximumSpeed=hypot(cfg.model.speedMaximum,cfg.model.lateralVelocityMaximum);
    previewTime=max(of.minimumPreviewSeconds,maximumSpeed/of.previewDecelerationMetersPerSecondSquared+of.previewSettlingSeconds);
    allocatedTime=of.horizonResolutionSeconds*ceil(previewTime/of.horizonResolutionSeconds);
    maximumCount=ceil(allocatedTime/h)+ceil(of.minimumPreviewSeconds/h);
    capacity=maximumCount;K=of.validationSubsteps;
    M=K;D=1;dt=h/M;
    x=SX.sym('x',6);u=SX.sym('u',2);p=SX.sym('p',4);
    k1=nonlinearBicycleModel.symbolicDerivative(x,u,p,cfg);
    k2=nonlinearBicycleModel.symbolicDerivative(x+dt*k1/2,u,p,cfg);
    k3=nonlinearBicycleModel.symbolicDerivative(x+dt*k2/2,u,p,cfg);
    k4=nonlinearBicycleModel.symbolicDerivative(x+dt*k3,u,p,cfg);
    next=x+dt*(k1+2*k2+2*k3+k4)/6;
    options=struct;
    if of.nlpJit
        directory=tempname;mkdir(directory);options=struct('jit',true,'compiler','shell', ...
            'jit_options',struct('flags',sprintf('-O%d',of.nlpCompilerOptimization), ...
            'compiler',char(of.nlpCompiler),'directory',[directory,filesep]));
    end
    points=cell(1,D);
    for j=1:D
        t=j/D;points{j}=x+dt*((t-1.5*t^2+2/3*t^3)*k1 ...
            +(t^2-2/3*t^3)*(k2+k3)+(-.5*t^2+2/3*t^3)*k4);
    end
    step=Function('pcbf_step',{x,u,p},{next,horzcat(points{:})},options);
    root=MX.sym('root',6);command=MX.sym('command',2);parameters=MX.sym('parameters',4);
    mappedStep=step.mapaccum(M);[ends,points]=mappedStep(root,repmat(command,1,M),repmat(parameters,1,M));
    last=ends(:,end);stopped=(command(2)<=0)&(last(4)<.2*cfg.model.brakeHoldSpeed);
    finish=[last(1:3);if_else(stopped,zeros(3,1),last(4:6))];
    hold=Function('pcbf_hold',{root,command,parameters},{finish,points});
    % RK4 continuous extension supplies dense physical-domain samples and
    % their derivatives without an additional integration at every sample.
    % IPOPT uses this approximation to the trajectory Jacobian.
    % The nonlinear oracle and acceptance gate always use the flow above.
    M=min(K,max(4,round(h/of.sensorPeriod)));D=K/M;dt=h/M;
    k1=nonlinearBicycleModel.symbolicDerivative(x,u,p,cfg);
    k2=nonlinearBicycleModel.symbolicDerivative(x+dt*k1/2,u,p,cfg);
    k3=nonlinearBicycleModel.symbolicDerivative(x+dt*k2/2,u,p,cfg);
    k4=nonlinearBicycleModel.symbolicDerivative(x+dt*k3,u,p,cfg);
    next=x+dt*(k1+2*k2+2*k3+k4)/6;
    sensitivity=SX.sym('sensitivity',6,8);
    updated=jacobian(next,x)*sensitivity+[SX.zeros(6,6),jacobian(next,u)];
    % Only the binding physical-domain sample needs a within-hold Jacobian.
    % Geometry and the terminal law use the post-reset boundary Jacobians.
    index=MX.sym('index');initialSensitivity=DM([eye(6),zeros(6,2)]);
    augmented=[x;sensitivity(:)];
    variation=Function('pcbf_variation',{augmented,u,p},{[next;updated(:)]},options);
    initialVariation=[root;initialSensitivity(:)];
    variationMap=variation.mapaccum(M);
    varying=variationMap(initialVariation,repmat(command,1,M),repmat(parameters,1,M));
    history=densify([initialVariation,varying]);
    position=index*M/K;left=floor(position);
    base=history(:,left+1);
    sample=reshape(base(7:54),6,8);
    if D~=1
        % A fine-grid sample between coarse RK4 endpoints uses the continuous
        % extension of that single substep, evaluated at its own initial state.
        t=SX.sym('fraction');
        point=x+dt*((t-1.5*t^2+2/3*t^3)*k1+(t^2-2/3*t^3)*(k2+k3)+(-.5*t^2+2/3*t^3)*k4);
        tangent=jacobian(point,x)*sensitivity+[SX.zeros(6,6),jacobian(point,u)];
        interpolate=Function('pcbf_dense_tangent',{augmented,u,p,t},{tangent},options);
        fraction=position-left;
        sample=if_else(fraction>0,interpolate(base,command,parameters,fraction),sample,true);
    end
    E=reshape(varying(7:54,end),6,8);
    % At exact rest and neutral force, select the outgoing acceleration
    % branch. The reset value is unchanged; nonlinear replay accepts steps.
    starting=(command(2)==0)&(dot(root(4:6),root(4:6))==0);
    stopped=(command(2)<=0)&(varying(4,end)<.2*cfg.model.brakeHoldSpeed)&~starting;
    E=[E(1:3,:);if_else(stopped,MX.zeros(3,8),E(4:6,:))];
    derivative=Function('pcbf_sensitivity',{root,command,parameters,index},{E,sample},options);
    identity=Function('pcbf_identity',{x,u,p},{x,repmat(x,1,K)});
    switchFlow=Function.if_else('pcbf_active',hold,identity);
    root=MX.sym('root',6);input=MX.sym('input',2);scale=MX.sym('scale',4);active=MX.sym('active');
    [endpoint,dense]=switchFlow(active,root,input,scale);
    piece=Function('pcbf_piece',{root,input,scale,active},{endpoint,dense});mapped=piece.mapaccum(capacity);
    inputs=MX.sym('inputs',2,capacity);enabled=MX.sym('enabled',1,capacity);
    [ends,points]=mapped(root,inputs,repmat(scale,1,capacity),enabled);
    rollout=Function('pcbf_rollout',{root,inputs,scale,enabled},{points,ends},options);
    kernel=struct('refinedRollout',rollout,'sensitivity',derivative,'capacity',capacity, ...
        'mappedSensitivity',containers.Map('KeyType','double','ValueType','any'), ...
        'holds',holds,'validationSubsteps',K,'integrationSubsteps',K,'sensitivitySubsteps',M,'hasTarget',hasTarget);
    keys{end+1}=key;kernels{end+1}=kernel;
end

function [e,rollouts]=localCondensedOracle(u,model,derivatives,rollouts,checkBudget,performance)
    if nargin<6,performance=true;end
    if nargin<3,derivatives=true;end
    if nargin<5,checkBudget=[];end
    cfg=model.cfg;kernel=model.kernel;N=sum(kernel.holds);n=numel(kernel.holds);nu=numel(u);K=kernel.validationSubsteps;h=cfg.controller.sampleTime;
    controls=reshape(u,2,n);inputs=controls*model.map.'+model.offset;
    if nargin<4,rollouts=[];end
    if ~localCompactCacheMatches(rollouts,inputs,model.hypotheses,model.previous,kernel,cfg)
        rollouts=struct('hypotheses',model.hypotheses,'inputs',inputs,'previous',model.previous, ...
            'cfg',cfg,'holds',kernel.holds,'integrationSubsteps',kernel.integrationSubsteps, ...
            'trajectories',{cell(1,numel(model.hypotheses))});
    end
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    e=struct('stage',-Inf(N,1),'forward',-Inf(N,1),'terminal',zeros(numel(model.hypotheses),1), ...
        'cost',0,'derivatives',derivatives);
    ratio=inputs(2,:).';
    e.stabilityCost=h/2*sum(ratio.^2+ratio.^4);
    if derivatives
        e.stageGradient=zeros(nu,N);e.forwardGradient=zeros(nu,N);
        e.terminalGradient=zeros(nu,numel(model.hypotheses));
        e.costGradient=zeros(nu,1);e.costHessian=zeros(nu);
        ratioJacobian=model.map;
        squaredRatioJacobian=2*ratio.*ratioJacobian;
        e.stabilityGradient=zeros(nu,1);
        e.stabilityHessian=zeros(nu);
        e.stabilityGradient(2:2:end)=h*(ratioJacobian.'*ratio+squaredRatioJacobian.'*(ratio.^2));
        e.stabilityHessian(2:2:end,2:2:end)=h*(ratioJacobian.'*ratioJacobian+squaredRatioJacobian.'*squaredRatioJacobian);
    end
    for s=1:numel(model.hypotheses)
        if ~isempty(checkBudget),checkBudget();end
        item=model.hypotheses(s);
        if ~isempty(rollouts.trajectories{s}) && isfield(rollouts.trajectories{s},'oracle')
            flow=rollouts.trajectories{s}.oracle;
            dense=flow.dense;
            ends=flow.ends;
            allInputs=flow.allInputs;
            total=flow.total;
            q=flow.q;boxes=flow.boxes;activeStages=flow.activeStages;
            localStage=flow.localStage;
            nodeIndices=flow.nodeIndices;
            forward=flow.forward;
            fi=flow.fi;
            domainState=flow.domainState;
            domainInput=flow.domainInput;terminal=flow.terminal;g=flow.terminalGradient;
        else
            if ~isempty(rollouts.trajectories{s}) && isfield(rollouts.trajectories{s},'ends')
                dense=rollouts.trajectories{s}.dense;ends=rollouts.trajectories{s}.ends;
            else
                [dense,ends]=localCompactHeldFlow(kernel,item.ego,inputs,item.parameters,checkBudget);
                dense=[item.ego,dense];
            end
            remaining=[];
            if ~isempty(item.cover)
                [drivingGap,drivingBoxes,remaining]=localCompactOccupied(item,[item.ego,ends],(0:N)*h,shape,cfg);
                activeAtHandoff=remaining;
            end
            [backupDense,backupEnds,backupInputs]=localCompactCruiseRollout(kernel,ends(:,end),item,N*h,cfg,checkBudget,remaining);
            rollouts.trajectories{s}=struct('dense',dense,'states',ends(:,cumsum(kernel.holds)), ...
                    'collisionStates',[item.ego,ends], ...
                    'backup',struct('dense',backupDense,'states',backupEnds,'inputs',backupInputs));
            ends=[ends,backupEnds];dense=[dense,backupDense(:,2:end)];
            allInputs=[inputs,backupInputs];total=size(allInputs,2);times=(0:total)*h;
            nodes=[item.ego,ends];q=[];boxes=[];gap=[];localStage=-ones(1,total);nodeIndices=1:total;
            activeStages=false(1,total);
            if ~isempty(item.target)
                if item.target(4)>0,q=predictiveSafetyGeometry.predictTarget(item.target,times);else,q=repmat(item.target,1,total+1);end
                boxes=q;
                if isempty(item.cover)
                    gap=predictiveSafetyGeometry.rectangleClearance(nodes,boxes,shape);
                else
                    gap=drivingGap;boxes=drivingBoxes;
                    if total>N
                        tailNodes=N+2:total+1;
                        [tailGap,tailBoxes,remaining]=localCompactOccupied(item,nodes(:,tailNodes),times(tailNodes),shape,cfg,remaining);
                        gap=[gap,tailGap];boxes=[boxes,tailBoxes];
                    end
                end
                % Each stage covers its two control-boundary nodes. No
                % collision condition is imposed on the integration mesh.
                [localStage,side]=max(cfg.outputFeedback.clearanceMeters-[gap(1:end-1);gap(2:end)],[],1);
                activeStages=isfinite(localStage);
                localStage(~activeStages)=-1;
                nodeIndices=(0:total-1)+side;
            end
            [forward,fi,domainState,domainInput]=localCompactModelDomain(dense,allInputs,cfg,K,N);
            target=[];if ~isempty(q),target=q(:,end);end
            [terminal,~,g]=localCompactMovingValue(ends(:,end),target,item,total*h,cfg,checkBudget,remaining);
            rollouts.trajectories{s}.oracle=struct('dense',dense,'ends',ends,'allInputs',allInputs,'total',total, ...
                'q',q,'boxes',boxes,'terminal',terminal,'terminalGradient',g,'activeStages',activeStages,'localStage',localStage,'nodeIndices',nodeIndices, ...
                'forward',forward,'fi',fi,'domainState',domainState, ...
                'domainInput',domainInput);
            if ~isempty(q)
                rollouts.trajectories{s}.targets=q(:,1:N+1);
                rollouts.trajectories{s}.separation=gap(1:N+1);
                if ~isempty(item.cover)
                    rollouts.trajectories{s}.remaining=activeAtHandoff;
                end
            end
        end
        if derivatives
            [residual,slipDerivative]=localRearSlipResidual(ends(:,1:N),model,s);
        else
            residual=localRearSlipResidual(ends(:,1:N),model,s);
        end
        weight=h/numel(model.hypotheses);
        e.stabilityCost=e.stabilityCost+weight/2*sum(residual.^2);
        slew=-Inf;channel=1;trim=terminalSafeSet.movingControl(cfg);
        if total>N
            rates=[cfg.outputFeedback.steeringRateMaximum;cfg.model.brakingRatioRateMaximum];
            [slew,channel]=max(abs(inputs(:,end)-trim)-h*rates);
        end
        if ~derivatives
            e.stage=max(e.stage,localStage(1:N).');e.forward=max(e.forward,forward(1:N).');
            e.terminal(s)=max([terminal,slew,localStage(N+1:end),forward(N+1:end)]);
            if performance && s==model.nominal,e.cost=localCompactPerformance(ends(:,cumsum(kernel.holds)),inputs,model);end
            continue;
        end
        nodes=[item.ego,ends];roots=nodes(:,1:end-1);derivativeModel=model;
        derivativeModel.controlMap=[model.controlMap;zeros(2*(total-N),nu)];
        % Geometry uses boundary sensitivities. Only the active physical
        % domain sample inside each hold needs a dense-sample sensitivity.
        stageNeeded=false(1,total);domainNeeded=false(1,total);
        stageNeeded(1:N)=activeStages(1:N);domainNeeded(1:N)=true;
        if total>N
            [~,backupStage]=max(localStage(N+1:end));
            [~,backupDomain]=max(forward(N+1:end));
            stageNeeded(N+backupStage)=activeStages(N+backupStage);domainNeeded(N+backupDomain)=true;
        end
        if ~isempty(checkBudget),checkBudget();end
        [SF,SE]=localCondensedSensitivities(roots,allInputs,item.parameters,fi,domainNeeded,derivativeModel);
        if ~isempty(checkBudget),checkBudget();end
        gradient=zeros(3,total);
        if any(stageNeeded)
            selected=nodeIndices(stageNeeded);
            gradient(:,stageNeeded)=localCompactDistanceGradient(nodes(:,selected),boxes(:,selected),shape);
        end
        stageGradient=zeros(nu,total);
        for i=1:total
            if stageNeeded(i)
                S=SE(:,:,nodeIndices(i));
                stageGradient(:,i)=-(gradient(:,i).'*S(1:3,:)).';
            end
            if i<=N
                domainGradient=(domainState(:,i).'*SF(:,:,i)+domainInput(:,i).'*derivativeModel.controlMap(2*i-1:2*i,:)).';
                if localStage(i)>e.stage(i),e.stage(i)=localStage(i);e.stageGradient(:,i)=stageGradient(:,i);end
                if forward(i)>e.forward(i),e.forward(i)=forward(i);e.forwardGradient(:,i)=domainGradient;end
            end
        end
        e.terminal(s)=terminal;e.terminalGradient(:,s)=(g*SE(:,:,end)).';
        if total>N
            [backupValue,index]=max(localStage(N+1:end));
            if backupValue>e.terminal(s),e.terminal(s)=backupValue;e.terminalGradient(:,s)=stageGradient(:,N+index);end
            [backupForward,index]=max(forward(N+1:end));
            if backupForward>e.terminal(s)
                j=N+index;e.terminal(s)=backupForward;
                e.terminalGradient(:,s)=(domainState(:,j).'*SF(:,:,j)+domainInput(:,j).'*derivativeModel.controlMap(2*j-1:2*j,:)).';
            end
        end
        if slew>e.terminal(s)
            e.terminal(s)=slew;
            e.terminalGradient(:,s)=0;
            e.terminalGradient(nu-2+channel,s)=sign(inputs(channel,end)-trim(channel));
        end
        jacobian=zeros(N,nu);
        for j=1:N
            sensitivity=SE(:,:,j+1);
            jacobian(j,:)=slipDerivative(:,j).'*sensitivity(4:6,:);
        end
        e.stabilityGradient=e.stabilityGradient+weight*jacobian.'*residual.';
        e.stabilityHessian=e.stabilityHessian+weight*(jacobian.'*jacobian);
        if performance && s==model.nominal
            for i=1:n
                state=ends(:,sum(kernel.holds(1:i)));S=SE(:,:,sum(kernel.holds(1:i))+1);ref=model.reference(:,i);
                normal=[-sin(ref(3));cos(ref(3))];error=[normal.'*(state(1:2)-ref(1:2));atan2(sin(state(3)-ref(3)),cos(state(3)-ref(3)));state(4)-cfg.referenceSpeed;state(5:6)];
                derivative=[normal.'*S(1:2,:);S(3:6,:)];
                e.cost=e.cost+.5*h*kernel.holds(i)*sum(cfg.outputFeedback.trackingWeights.*error.^2);
                e.costGradient=e.costGradient+h*kernel.holds(i)*derivative.'*(cfg.outputFeedback.trackingWeights.*error);
                e.costHessian=e.costHessian+h*kernel.holds(i)*derivative.'*(cfg.outputFeedback.trackingWeights.*derivative);
            end
        end
    end
    if ~derivatives || ~performance,return;end
    differences=diff([model.previous,controls],1,2);
    e.cost=e.cost+.5*cfg.outputFeedback.inputWeight*sum(diff([model.previous,inputs],1,2).^2,'all');
    % The fixed guide offset vanishes at each knot. Its fine increments
    % telescope to zero within each interval, leaving these derivatives intact.
    du=cfg.outputFeedback.inputWeight*differences./kernel.holds;
    e.costGradient=e.costGradient+reshape(du-[du(:,2:end),zeros(2,1)],[],1);
    difference=eye(n)-diag(ones(n-1,1),-1);
    e.costHessian=e.costHessian+cfg.outputFeedback.inputWeight*kron(difference.'*diag(1./kernel.holds)*difference,eye(2));
end

function [residual,derivative]=localRearSlipResidual(states,model,index)
    cfg=model.cfg;
    vx=states(4,:);
    speed=hypot(vx,cfg.model.scheduleSpeedFloor);
    lateral=states(5,:)-cfg.vehicle.lr*states(6,:);
    scale=model.rearSlipScale(index);
    residual=atan2(lateral,speed)/scale;
    if nargout>1
        factor=1./(scale*(speed.^2+lateral.^2));
        derivative=[-factor.*lateral.*vx./speed;factor.*speed;-cfg.vehicle.lr*factor.*speed];
    end
end

function [selected,ends]=localCondensedSensitivities(roots,inputs,parameters,index,required,model)
    N=size(inputs,2);nu=size(model.controlMap,2);
    maps=model.kernel.mappedSensitivity;
    if isKey(maps,N)
        mapped=maps(N);
    else
        mapped=model.kernel.sensitivity.map(N);
        % Horizons can vary with the terminal continuation. Bound the cache
        % per compiled model while reusing maps across hypotheses and iterates.
        if maps.Count>=16,remove(maps,keys(maps));end
        maps(N)=mapped;
    end
    [allEnd,allSelected]=mapped(roots,inputs,repmat(parameters,1,N),index);
    allEnd=full(allEnd);allSelected=full(allSelected);
    native=[];
    if model.cfg.outputFeedback.nlpJit,native=localCompactSensitivityKernel();end
    if ~isempty(native)
        [points,endpoints]=native(allEnd,allSelected,model.controlMap,index,required,model.kernel.validationSubsteps);
        selected=reshape(points,6,nu,N);
        ends=reshape(endpoints,6,nu,N+1);
        return
    end
    selected=zeros(6,nu,N);ends=zeros(6,nu,N+1);
    for i=1:N
        columns=8*(i-1)+(1:8);
        E=allEnd(:,columns);U=model.controlMap(2*i-1:2*i,:);
        before=ends(:,:,i);
        ends(:,:,i+1)=E(:,1:6)*before+E(:,7:8)*U;
        if required(i)
            D=allSelected(:,columns);
            if index(i)==0
                selected(:,:,i)=before;
            elseif index(i)==model.kernel.validationSubsteps && isequal(D,E)
                selected(:,:,i)=ends(:,:,i+1);
            else
                selected(:,:,i)=D(:,1:6)*before+D(:,7:8)*U;
            end
        end
    end
end

function kernel=localCompactSensitivityKernel()
    % Optional native copy of the numerical chain rule. A deterministic
    % probe covers a pre-reset sample and causal control columns.
    persistent handle checked
    if isempty(checked)
        checked=true;handle=[];
        folder=fullfile(fileparts(fileparts(mfilename('fullpath'))),'solver','controller');
        if isfile(fullfile(folder,['domainSensitivityKernelMex.',mexext]))
            previous=addpath(folder);cleanup=onCleanup(@()path(previous));candidate=@domainSensitivityKernelMex;
            E=[eye(6),ones(6,2)/8];E=[E,E];E(4:6,9:16)=0;
            D=repmat([eye(6),ones(6,2)/8],1,2);
            U=[eye(2),zeros(2);eye(2)/2,eye(2)/2];
            index=[0,64];required=true(1,2);
            [points,ends]=nonlinearBicycleModel.condenseSensitivities(E,D,U,index,required,64);
            same=false;
            try
                [a,b]=candidate(E,D,U,index,required,64);same=isequal(a,points)&&isequal(b,ends);
            catch
                % Missing or stale native interfaces use the MATLAB path.
            end
            if same,handle=candidate;
            else
                warning('collisionAvoidanceController:staleSensitivityKernel', ...
                    'The compiled sensitivity kernel differs from its source probe. Rebuild it with buildControllerKernels.');
            end
        end
    end
    kernel=handle;
end

function gradient=localCompactDistanceGradient(states,targets,shape)
    [~,gradient]=predictiveSafetyGeometry.rectangleClearance(states,targets,shape);
end

function value=localCompactPerformance(states,inputs,model)
    cfg=model.cfg;reference=model.reference;
    lateral=-sin(reference(3,:)).*(states(1,:)-reference(1,:)) ...
        +cos(reference(3,:)).*(states(2,:)-reference(2,:));
    angle=states(3,:)-reference(3,:);
    error=[lateral;atan2(sin(angle),cos(angle));states(4,:)-cfg.referenceSpeed;states(5:6,:)];
    value=.5*cfg.controller.sampleTime*sum(model.kernel.holds.*sum(cfg.outputFeedback.trackingWeights.*error.^2,1)) ...
        +.5*cfg.outputFeedback.inputWeight*sum(diff([model.previous,inputs],1,2).^2,'all');
end

function controls = localCompactRateLimit(controls,previous,holds,cfg)
    for i=1:numel(holds)
        span=cfg.controller.sampleTime*holds(i);
        change=[cfg.outputFeedback.steeringRateMaximum;cfg.model.brakingRatioRateMaximum]*span;
        controls(:,i)=max(previous-change,min(previous+change,controls(:,i)));
        controls(:,i)=max([-cfg.outputFeedback.steeringMaximum;max(-1,cfg.actuation.brakingRatioMinimum)], ...
            min([cfg.outputFeedback.steeringMaximum;min(1,cfg.actuation.brakingRatioMaximum)],controls(:,i)));
        previous=controls(:,i);
    end
end

function [controls,guideInputs] = localCompactSeed(item,previous,holds,lane,cfg,avoidance)
    % One held-input field rollout supplies the initial geometry and controls.
    count=sum(holds);x=item.ego;
    if nargin<6,avoidance=count;end
    frame=predictiveSafetyGeometry.roadFrame(lane,struct('lateralClearance',[]));
    nominal=struct('state',[0;0;0;cfg.referenceSpeed;0;0],'curvature',frame(4));
    model=struct('cfg',cfg,'initialState',x,'previousInput',previous,'target',item.target, ...
        'lane',lane,'road',struct('lateralClearance',[]),'frame',frame,'nominalReference',nominal,'compactInitialization',true,'motionLimits',item.motionLimits,'motionCover',item.cover, ...
        'uncertainty',struct('collisionGenerator',zeros(6,0),'target',[]));
    if isempty(item.target)
        trimOffset=localCompactCruiseOffset(cfg,frame(4));
        inputs=zeros(2,count);prior=previous;
        for i=1:count
            projection=predictiveSafetyGeometry.project(x(1:2),lane);
            acceleration=predictiveSafetyGeometry.routeAcceleration(x,projection,model);
            guided=nonlinearBicycleModel.accelerationGuidance(x,acceleration,prior,cfg,1,true,true);
            prior=localCompactRateLimit(guided+trimOffset,prior,1,cfg);
            inputs(:,i)=prior;
            if i<count,x=nonlinearBicycleModel.sample(x,prior,cfg);end
        end
    else
        guide=predictiveSafetyGeometry.avoidanceGuide(model,avoidance);
        % Only the first leg comes from the planner. Trim initializes the
        % remaining actuator variables, which are free in the same NLP.
        inputs=[guide.inputs,repmat(terminalSafeSet.movingControl(cfg),1,count-avoidance)];
    end
    guideInputs=localCompactRateLimit(inputs,previous,ones(1,count),cfg);
    controls=guideInputs(:,cumsum(holds));
end

function offset=localCompactCruiseOffset(cfg,curvature)
    % Anchor route feedback at the physical steady turn. Its longitudinal
    % inverse accounts for road load but omits the tangential tire forces.
    offset=zeros(2,1);
    if curvature==0,return;end
    persistent lastCfg lastCurvature lastOffset
    if isequal(lastCurvature,curvature) && isequaln(lastCfg,cfg),offset=lastOffset;return;end
    try
        trim=nonlinearBicycleModel.operatingPoint(cfg,curvature);
        acceleration=[0;curvature*cfg.referenceSpeed^2];
        guided=nonlinearBicycleModel.accelerationGuidance(trim.state,acceleration,trim.input,cfg,1,true,true);
        offset=trim.input-guided;
    catch exception
        if ~strcmp(exception.identifier,'collisionAvoidanceController:unrealizableReference'),rethrow(exception);end
    end
    lastCfg=cfg;lastCurvature=curvature;lastOffset=offset;
end

function matches=localCompactCacheMatches(rollouts,inputs,hypotheses,previous,kernel,cfg)
    matches=~isempty(rollouts) && isequal(inputs,rollouts.inputs) ...
        && isequal(previous,rollouts.previous) && isequaln(hypotheses,rollouts.hypotheses) ...
        && isequaln(cfg,rollouts.cfg) && isequal(kernel.holds,rollouts.holds) ...
        && kernel.integrationSubsteps==rollouts.integrationSubsteps;
end

function [cache,reusedHolds]=localCompactShiftedCache(prior,hypotheses,conditioning,inputs,kernel,cfg,checkBudget)
    % Reuse only the identical suffix of the accepted numerical trajectory.
    % Measurement conditioning may prune members, so use their retained indices.
    cache=[];reusedHolds=zeros(1,numel(hypotheses));
    if ~isfield(prior,'retainedTrajectories') || ~iscell(prior.retainedTrajectories),return;end
    count=size(inputs,2);substeps=kernel.validationSubsteps;
    cache=struct('hypotheses',hypotheses,'inputs',inputs,'previous',prior.appliedInput, ...
        'cfg',cfg,'holds',kernel.holds,'integrationSubsteps',kernel.integrationSubsteps, ...
        'trajectories',{cell(1,numel(hypotheses))});
    for s=1:numel(hypotheses)
        if ~isempty(checkBudget),checkBudget();end
        original=conditioning.retainedIndices(s);item=hypotheses(s);prefix=0;
        if original<=numel(prior.retainedTrajectories)
            old=prior.retainedTrajectories{original};
            compatible=isstruct(old) && isscalar(old) && all(isfield(old, ...
                {'inputs','denseEgo','collisionEgo','validationSubsteps','parameters'}));
            if compatible
                oldCount=size(old.inputs,2);
                compatible=old.validationSubsteps==substeps && oldCount>=1 ...
                    && isequal(size(old.inputs),[2,oldCount]) ...
                    && isequal(size(old.collisionEgo),[6,oldCount+1]) ...
                    && isequal(size(old.denseEgo),[6,substeps*oldCount+1]) ...
                    && isequal(old.collisionEgo(:,2),item.ego) ...
                    && isequal(old.parameters,item.parameters);
            end
            if compatible
                prefix=min(count,oldCount-1);
                mismatch=find(any(inputs(:,1:prefix)~=old.inputs(:,2:prefix+1),1),1);
                if ~isempty(mismatch),prefix=mismatch-1;end
            end
        end
        if prefix>0
            dense=old.denseEgo(:,substeps+1:substeps*(prefix+1)+1);
            % Dense samples precede the hold-end stop reset; the new root follows it.
            dense(:,1)=item.ego;
            ends=old.collisionEgo(:,3:prefix+2);root=ends(:,end);
        else
            dense=item.ego;ends=zeros(6,0);root=item.ego;
        end
        if prefix<count
            [points,states]=localCompactHeldFlow(kernel,root,inputs(:,prefix+1:end),item.parameters,checkBudget);
            dense=[dense,points];ends=[ends,states];
        end
        reusedHolds(s)=prefix;
        % Geometry, physical-domain constraints and terminal continuation are rechecked.
        cache.trajectories{s}=struct('dense',dense,'states',ends(:,cumsum(kernel.holds)), ...
            'collisionStates',[item.ego,ends],'backup',[]);
    end
end

function [valid,traces,violation,nodeStates,slacks] = localCompactValidate(controls,hypotheses,previous,kernel,cfg,fine,rollouts,checkBudget)
    if nargin<6,fine=false;end
    if nargin<7,rollouts=[];end
    if nargin<8,checkBudget=[];end
    reuse=fine && localCompactCacheMatches(rollouts,controls,hypotheses,previous,kernel,cfg);
    traces=cell(1,numel(hypotheses));nodeStates=cell(size(traces));violation=0;h=cfg.controller.sampleTime;
    durations=kernel.holds;if fine,durations=ones(1,size(controls,2));end
    slacks=zeros(sum(durations),1);
    clipped=localCompactRateLimit(controls,previous,durations,cfg);
    violation=max(violation,max(abs(controls-clipped),[],'all'));
    stateViolation=violation;separated=true;
    for s=1:numel(hypotheses)
        if ~isempty(checkBudget),checkBudget();end
        item=hypotheses(s);
        if reuse
            flow=rollouts.trajectories{s};dense=flow.dense;states=flow.states;inputs=controls;collisionStates=flow.collisionStates;
        else
            [dense,states,inputs,collisionStates]=localCompactRollout(kernel,item.ego,previous,controls,item.parameters,fine);
        end
        % The accepted trace contains the original control-knot states.
        nodeStates{s}=states;
        if any(~isfinite(dense),'all'),violation=Inf;valid=false;return;end
        [domain,~,~,~,defined]=localCompactModelDomain(dense,inputs,cfg,kernel.validationSubsteps);
        stateViolation=max(stateViolation,max(domain));
        if ~defined,stateViolation=Inf;end
        violation=max(violation,stateViolation);
        times=(0:size(inputs,2))*h;q=[];remaining=[];
        if ~isempty(item.target)
            if reuse && isfield(flow,'separation')
                q=flow.targets;separation=flow.separation;
                if ~isempty(item.cover),remaining=flow.remaining;end
            else
                if item.target(4)>0,q=predictiveSafetyGeometry.predictTarget(item.target,times);
                else,q=repmat(item.target,1,numel(times));end
                shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
                if isempty(item.cover)
                    separation=predictiveSafetyGeometry.rectangleClearance(collisionStates,q,shape);
                else
                    [separation,~,remaining]=localCompactOccupied(item,collisionStates,times,shape,cfg);
                end
            end
            separated=separated && all(separation>0);
            violation=max(violation,-min(separation));
            slacks=max(slacks,(cfg.outputFeedback.clearanceMeters-min(separation(1:end-1),separation(2:end))).');
        end
        parameters=[];if ~isempty(item.target),parameters=item.target(5:6);end
        traces{s}=struct('ego',[item.ego,states],'inputs',inputs,'denseEgo',dense,'collisionEgo',collisionStates,'collisionTimes',times, ...
            'targets',q,'remainingParameterCells',remaining,'targetMotionParameters',parameters,'validationSubsteps',kernel.validationSubsteps);
    end
    valid=separated && isfinite(violation) && violation<=cfg.outputFeedback.nlpTolerance;
    if isfinite(violation)
        for s=1:numel(hypotheses)
            if ~isempty(checkBudget),checkBudget();end
            trace=traces{s};
            backup=[];if reuse,backup=rollouts.trajectories{s}.backup;end
            [terminalValue,terminal]=localCompactTerminalCheck(trace.ego(:,end),hypotheses(s),trace.collisionTimes(end),trace.inputs(:,end),kernel,cfg,backup,checkBudget,trace.remainingParameterCells);
            traces{s}.terminal=terminal;violation=max(violation,terminalValue);
            valid=valid && terminal.member;
        end
    end
    % A stationary initial measurement may start moving. Both velocity
    % components may vanish together only at the initial sample.
    for s=1:numel(traces)
        if ~valid,break;end
        trace=traces{s};
        valid=~any(all(trace.collisionEgo(4:5,2:end)==0,1)) && ~any(all(trace.denseEgo(4:5,2:end)==0,1));
    end
end

function [dense,states,inputs,collisionStates] = localCompactRollout(kernel,x,previous,controls,parameters,fine)
    N=sum(kernel.holds);inputs=zeros(2,N);k=0;
    if fine,inputs=controls;else
        for i=1:numel(kernel.holds)
            for j=1:kernel.holds(i),k=k+1;inputs(:,k)=previous+(controls(:,i)-previous)*j/kernel.holds(i);end
            previous=controls(:,i);
        end
    end
    [points,ends]=localCompactHeldFlow(kernel,x,inputs,parameters,[]);
    dense=[x,points];states=ends(:,cumsum(kernel.holds));
    if nargout>3,collisionStates=[x,ends];end
end

function [points,ends]=localCompactHeldFlow(kernel,x,inputs,parameters,checkBudget)
    % The compiled integrator owns a block of holds. A longer online horizon
    % continues at its post-reset endpoint with the same parameters and inputs.
    count=size(inputs,2);capacity=kernel.capacity;substeps=kernel.validationSubsteps;
    points=zeros(6,substeps*count);ends=zeros(6,count);
    for first=1:capacity:count
        if first>1 && ~isempty(checkBudget),checkBudget();end
        last=min(first+capacity-1,count);blockCount=last-first+1;
        commands=[inputs(:,first:last),zeros(2,capacity-blockCount)];
        [dense,nodes]=kernel.refinedRollout(x,commands,parameters,double((1:capacity)<=blockCount));
        dense=full(dense);nodes=full(nodes);
        points(:,substeps*(first-1)+1:substeps*last)=dense(:,1:substeps*blockCount);
        ends(:,first:last)=nodes(:,1:blockCount);x=nodes(:,blockCount);
    end
end

function [terminalValue,terminal]=localCompactTerminalCheck(x,item,start,previous,kernel,cfg,backup,checkBudget,remaining)
    h=cfg.controller.sampleTime;
    if nargin<8,checkBudget=[];end
    if nargin<7 || isempty(backup)
        [tail,tailEnds,tailInputs]=localCompactCruiseRollout(kernel,x,item,start,cfg,checkBudget,remaining);
    else
        tail=backup.dense;tailEnds=backup.states;tailInputs=backup.inputs;
    end
    command=terminalSafeSet.movingControl(cfg);nBridge=size(tailInputs,2);
    normalWitness=zeros(2,1);exitSteps=0;terminalValue=0;member=true;
    if nBridge>0
        [domain,~,~,~,defined]=localCompactModelDomain(tail,tailInputs,cfg,kernel.validationSubsteps);
        times=start+(0:nBridge)*h;
        q=localCompactTargetNodes(item.target,times);
        shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
        if isempty(item.cover)
            gaps=predictiveSafetyGeometry.rectangleClearance([x,tailEnds],q,shape);
        else
            [gaps,~,remaining]=localCompactOccupied(item,[x,tailEnds],times,shape,cfg,remaining);
        end
        [core,normalWitness,~,coreSteps]=localCompactMovingValue(tailEnds(:,end),q(:,end),item,times(end),cfg,checkBudget,remaining);
        exitSteps=nBridge+coreSteps;
        rates=[cfg.outputFeedback.steeringRateMaximum;cfg.model.brakingRatioRateMaximum];
        slew=abs(command-previous)-h*rates;
        terminalValue=max([0,max(domain),cfg.outputFeedback.clearanceMeters-min(gaps),core,slew.']);
        member=defined && all(gaps>0) && isfinite(exitSteps) && ...
            terminalValue<=cfg.outputFeedback.nlpTolerance && ...
            (coreSteps==0 || core<cfg.outputFeedback.clearanceMeters);
    else
        % No encounter action is required. The next nominal input must
        % still be reachable from the preceding actuator command.
        command=localCompactRateLimit(command,previous,1,cfg);
    end
    terminal=struct('value',terminalValue,'member',member, ...
        'input',command,'bridgeHolds',nBridge,'exitSteps',exitSteps,'normal',normalWitness, ...
        'model',"safeRangeExit",'safetyAssumed',true,'safetyProved',false);
end

function [dense,ends,inputs]=localCompactCruiseRollout(kernel,x,item,start,cfg,checkBudget,remaining)
    if ~isempty(checkBudget),checkBudget();end
    if isempty(item.target) || (~isempty(item.cover) && ~any(remaining)) || localCompactRange(item,x,start)>cfg.collision.encounterRangeMeters
        dense=x;ends=zeros(6,0);inputs=zeros(2,0);return;
    end
    h=cfg.controller.sampleTime;
    count=max(1,ceil(cfg.outputFeedback.minimumPreviewSeconds/h));
    inputs=repmat(terminalSafeSet.movingControl(cfg),1,count);
    [points,ends]=localCompactHeldFlow(kernel,x,inputs,item.parameters,checkBudget);
    out=find(localCompactRange(item,ends,start+(1:count)*h)>cfg.collision.encounterRangeMeters,1);
    if ~isempty(out),count=out;ends=ends(:,1:count);inputs=inputs(:,1:count);end
    dense=[x,points(:,1:count*kernel.validationSubsteps)];
end

function cover=localCompactCover(item,h,count)
    bounds=item.motionLimits;
    acceleration=unique(linspace(bounds(1,1),bounds(2,1),5));
    slip=unique(linspace(bounds(1,2),bounds(2,2),5));
    na=max(1,numel(acceleration)-1);nb=max(1,numel(slip)-1);
    [i,j]=ndgrid(1:na,1:nb);
    % Store closed interval cells, each with its complete footprint image.
    cover=struct('h',h,'cells',zeros(2,2,numel(i)),'boxes',zeros(11,count+1,numel(i)), ...
        'centers',zeros(4,count+1,numel(i)));
    times=(0:count)*h;nominal=predictiveSafetyGeometry.predictTarget(item.target,times);
    lower=[reshape(acceleration(i),1,[]);reshape(slip(j),1,[])];
    upper=[reshape(acceleration(min(i+1,end)),1,[]);reshape(slip(min(j+1,end)),1,[])];
    track=item.targetObserver;
    if isfield(track,'motionStrips') && ~isempty(track.motionStrips)
        strips=track.motionStrips;lr=item.target(7);
        lower(2,:)=sin(lower(2,:))/lr;upper(2,:)=sin(upper(2,:))/lr;
        % Keep the published NRMM center as well as every data-compatible
        % constant motion. Each inequality only excludes impossible cells.
        center=[item.target(5);sin(item.target(6))/lr];
        strips(:,3)=max(strips(:,3),strips(:,1:2)*center);
        for row=1:size(strips,1)
            n=strips(row,1:2);rhs=strips(row,3);
            for d=1:2
                if abs(n(d))<=eps,continue;end
                other=3-d;
                bound=(rhs-min(n(other)*lower(other,:),n(other)*upper(other,:)))/n(d);
                tol=64*eps(max(1,abs(bound)));
                if n(d)>0,upper(d,:)=min(upper(d,:),bound+tol);
                else,lower(d,:)=max(lower(d,:),bound-tol);end
            end
        end
        keep=all(lower<=upper,1);lower=lower(:,keep);upper=upper(:,keep);
        lower(2,:)=asin(max(-1,min(1,lr*lower(2,:))));upper(2,:)=asin(max(-1,min(1,lr*upper(2,:))));
        cover.cells=cover.cells(:,:,1:nnz(keep));cover.boxes=cover.boxes(:,:,1:nnz(keep));
        cover.centers=cover.centers(:,:,1:nnz(keep));
    end
    for k=1:size(lower,2)
        limits=[lower(:,k).';upper(:,k).'];cover.cells(:,:,k)=limits;
        [cover.boxes(:,:,k),cover.centers(:,:,k)]=predictiveSafetyGeometry.targetMotionEnvelope(item.target,limits,times,nominal);
    end
end

function [gap,worst,remaining,last]=localCompactOccupied(item,states,times,shape,cfg,remaining,clip)
    cover=item.cover;nodes=round(times/cover.h)+1;count=numel(nodes);
    if nargin<6 || isempty(remaining),remaining=true(1,size(cover.cells,3));end
    if nargin<7,clip=Inf;end
    gap=repmat(clip,1,count);worst=repmat(item.target,1,count);last=1;
    which=find(remaining);number=numel(which);
    if number==0,return;end
    if nodes(end)<=size(cover.boxes,2)
        boxes=cover.boxes(:,nodes,which);centers=cover.centers(:,nodes,which);
    else
        boxes=zeros(11,count,number);centers=zeros(4,count,number);
        nominal=predictiveSafetyGeometry.predictTarget(item.target,times);
        for k=1:number
            [boxes(:,:,k),centers(:,:,k)]=predictiveSafetyGeometry.targetMotionEnvelope(item.target,cover.cells(:,:,which(k)),times,nominal);
        end
    end
    boxes=reshape(boxes,11,[]);centers=reshape(centers,4,[]);
    ego=repmat(states,1,number);
    range=reshape(predictiveSafetyGeometry.targetEnvelopeRange(ego,boxes,centers),count,[]);
    outside=range>cfg.collision.encounterRangeMeters;
    [hasExit,stop]=max(outside,[],1);stop(~hasExit)=count;
    remaining(which(hasExit))=false;last=max(stop);
    active=reshape((1:count).'<=stop,1,[]);
    c=cos(boxes(3,:));sn=sin(boxes(3,:));
    center=boxes(1:2,:)+[c.*boxes(10,:)-sn.*boxes(11,:);sn.*boxes(10,:)+c.*boxes(11,:)];
    radius=vecnorm(boxes(8:9,:))+norm(shape(1:2))+norm(shape(3:4));
    distance=vecnorm(ego(1:2,:)-center);
    lower=reshape(distance-radius,count,[]);
    upper=reshape(distance+radius,count,[]);upper(~reshape(active,count,[]))=Inf;
    bound=min(clip,min(upper,[],2));
    check=active & reshape(lower<=bound,1,[]);
    values=repmat(clip,1,count*number);
    if any(check)
        values(check)=min(clip,predictiveSafetyGeometry.rectangleClearance(ego(:,check),boxes(:,check),shape));
    end
    [gap,index]=min(reshape(values,count,[]),[],2);
    gap=gap.';worst=boxes(:,(index.'-1)*count+(1:count));
end

function range=localCompactRange(item,states,time)
    if isempty(item.motionLimits)
        q=localCompactTargetNodes(item.target,time);
        range=vecnorm(states(1:2,:)-q(1:2,:));
    else
        [q,centers]=predictiveSafetyGeometry.targetMotionEnvelope(item.target,item.motionLimits,time);
        range=predictiveSafetyGeometry.targetEnvelopeRange(states,q,centers);
    end
end

function [value,normal,gradient,exitSteps]=localCompactMovingValue(x,target,item,start,cfg,checkBudget,remaining)
    if isempty(item.motionLimits)
        [value,normal,gradient,exitSteps]=terminalSafeSet.movingValue(x,target,cfg,checkBudget);
        return
    end
    R=cfg.collision.encounterRangeMeters;h=cfg.controller.sampleTime;
    value=-R;normal=zeros(2,1);gradient=zeros(1,6);exitSteps=0;
    if ~any(remaining) || localCompactRange(item,x,start)>R,return;end
    exitSteps=Inf;
    if x(4)<=0,value=max(2*cfg.outputFeedback.nlpTolerance,-x(4));gradient(4)=-1;return;end
    e=predictiveSafetyGeometry.direction(x(3));lateral=[-e(2);e(1)];
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    clock=tic;first=0;best=Inf;
    while isinf(exitSteps)
        if isempty(checkBudget) && toc(clock)>cfg.outputFeedback.maximumSolveSeconds
            value=R;return
        end
        if ~isempty(checkBudget),checkBudget();end
        indices=first+(0:31);time=h*indices;
        states=[x(1:2)+x(4)*e*time;repmat(x(3),size(time))];
        [gaps,boxes,remaining,last]=localCompactOccupied(item,states,start+time,shape,cfg,remaining);
        if ~any(remaining),exitSteps=indices(last);end
        [minimum,j]=min(gaps);
        if minimum<best
            best=minimum;value=cfg.outputFeedback.clearanceMeters-best;
            [~,g]=predictiveSafetyGeometry.rectangleClearance(states(:,j),boxes(:,j),shape);
            normal=g(1:2);t=time(j);
            gradient=-[normal.',g(3)+x(4)*t*(normal.'*lateral),t*(normal.'*e),0,0];
        end
        if best<cfg.outputFeedback.clearanceMeters-cfg.outputFeedback.nlpTolerance,return;end
        first=indices(end)+1;
    end
end

function q=localCompactTargetNodes(target,times)
    if target(4)==0,q=repmat(target,1,numel(times));
    else,q=predictiveSafetyGeometry.predictTarget(target,times);end
end

function [value,index,stateGradient,inputGradient,defined]=localCompactModelDomain(states,inputs,cfg,K,drivingHolds)
    % The numerical Fiala guard must not become an accepted model extension.
    % Above the pure kinematic branch, front-wheel longitudinal velocity must
    % be positive (equivalent to |alpha_f|<pi/2 for the steering domain).
    % Check both sides of held-command jumps; no stability speed/rate cap is used.
    count=size(inputs,2);samples=(0:K).'+(0:count-1)*K+1;
    x=states(:,samples(:));u=repelem(inputs,1,K+1);c=cos(u(1,:));s=sin(u(1,:));
    lateral=x(5,:)+cfg.vehicle.lf*x(6,:);
    front=x(4,:).*c+lateral.*s;
    blend=x(4,:)-cfg.model.kinematicSpeedRange(1);
    defined=all(blend<=0 | front>0);
    [v,branch]=max([-x(4,:);min(blend,-front)],[],1);
    [value,j]=max(reshape(v,K+1,count),[],1);index=j-1;
    if nargout<3 && nargin<5,return;end
    selected=(0:count-1)*(K+1)+j;x=x(:,selected);u=u(:,selected);
    branch=branch(selected);c=c(selected);s=s(selected);lateral=lateral(selected);
    stateGradient=zeros(6,count);inputGradient=zeros(2,count);stateGradient(4,:)=-1;
    active=branch==2;low=active & blend(selected)<=-front(selected);
    stateGradient(4,low)=1;wheel=active & ~low;
    stateGradient(4,wheel)=-c(wheel);stateGradient(5,wheel)=-s(wheel);
    stateGradient(6,wheel)=-cfg.vehicle.lf*s(wheel);
    inputGradient(1,wheel)=x(4,wheel).*s(wheel)-lateral(wheel).*c(wheel);
    if nargin<5 || drivingHolds==0,return;end
    % At driving nodes exclude the interior of the model's stop-reset guard:
    % nonpositive command AND longitudinal speed below the reset threshold.
    % Use the pre-reset endpoint derivative. Exact replay resolves its boundary.
    nodes=1:drivingHolds;speedGap=.2*cfg.model.brakeHoldSpeed-states(4,1+K*nodes);
    driveGap=-inputs(2,nodes);resetGap=min(speedGap,driveGap);
    active=nodes(resetGap>value(nodes));
    value(active)=resetGap(active);index(active)=K;
    stateGradient(:,active)=0;inputGradient(:,active)=0;
    speedActive=speedGap(active)<=driveGap(active);
    stateGradient(4,active(speedActive))=-1;inputGradient(2,active(~speedActive))=-1;
end

function ready=localWarmIpopt(cfg,reference)
    % A fixed synthetic encounter exercises the CasADi/IPOPT backend
    % and trajectory derivatives before the experiment clock starts.
    % This discarded offline solve uses the solver iteration limits. An
    % online wall-clock deadline can interrupt first-time toolbox loading.
    cfg.outputFeedback.initializationSolveSeconds=realmax;
    item=struct('ego',[0;0;0;15;0;0],'parameters',ones(4,1),'sensorBias',zeros(7,1));
    item.target=[25;0;0;0;0;0;cfg.target.rearAxleDistance; ...
        cfg.target.defaultLength/2;cfg.target.defaultWidth/2;0;0];
    input=zeros(2,1);
    sample=solvePredictiveControl("outputFeedback",struct('operation',"sensor", ...
        'cfg',cfg,'hypothesis',item,'input',input,'time',0));
    block=struct('kind',"jointSensorBlock-v1",'time',0,'heldInput',input,'samples',sample,'initialHypotheses',item);
    % The probe has no access to the experiment state or controller memory.
    request=struct('operation',"step",'cfg',cfg,'reference',reference, ...
        'block',block,'previousState',[]);
    result=solvePredictiveControl("outputFeedback",request);
    info=result.prediction.metadata;
    if string(cfg.outputFeedback.method)=="ipopt"
        ready=info.solver=="casadiIpoptCompactPcbf" && info.nlpAttempts>0;
    else
        ready=info.solver=="casadiIpopt";
    end
end
