function [result,diagnostics,problem] = solvePredictiveControl(interface,request)
%solvePredictiveControl Raw-sensor, two-stage PCBF with CasADi/IPOPT.
    assert(nargin==2 && (ischar(interface) || isstring(interface)) && isequal(string(interface),"outputFeedback"), ...
        'collisionAvoidanceController:invalidSolverInterface','Expected the outputFeedback interface.');
    if ~isfield(request,'callTimer'),request.callTimer=tic;end
    cfg=collisionAvoidanceControllerConfig(request.cfg);
    diagnostics=[];problem=[];
    switch string(request.operation)
        case "warmUp"
            result=localWarmIpopt(cfg,request.reference);
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
            if isfield(request,'avoidanceSteps')
                avoidance=request.avoidanceSteps;
                validateattributes(avoidance,{'numeric'},{'scalar','integer','positive','<=',count});
            else
                [~,connection]=localPreviewDurations(hypotheses,cfg);
                avoidance=count;
                if count>connection,avoidance=count-connection;end
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
            % The sequence evaluator differentiates the reported nonlinear
            % flow. Online IPOPT proposals use the cheaper tangent mesh.
            kernel=localCompactKernel(cfg,holds,~isempty(hypotheses(1).target),cfg.outputFeedback.validationSubsteps);
            model=localPerformanceModel(cfg,hypotheses,request.input,holds,lane,nominal,kernel);
            model=localAddControlMap(model);
            [~,rollouts]=localCondensedOracle(controls(:),model,false);
            result=localCondensedOracle(controls(:),model,true,rollouts);
        case {"step","prepareStep"}
            [result,diagnostics]=localCompactStep(request,cfg);
        otherwise
            error('collisionAvoidanceController:invalidOutputFeedbackOperation','Unknown joint operation.');
    end
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

function next = localOfAdvance(hypothesis,input,cfg,duration,endOfHold,ego)
    if nargin<6
        flow=localOfInferenceKernel(cfg,duration,1);
        ego=full(flow(hypothesis.ego,input,hypothesis.parameters));
    end
    next=hypothesis;
    next.ego=ego;
    if endOfHold,next.ego=nonlinearBicycleModel.sampleEndState(next.ego,input,cfg);end
    if ~isempty(hypothesis.target) && hypothesis.target(4)>0
        next.target=predictiveSafetyGeometry.predictTarget(hypothesis.target,duration);
        next.target(5:6)=hypothesis.target(5:6);
    end
    % V=0 is an absorbing mode, including when latent A was nonzero.
end

function flow=localOfInferenceKernel(cfg,duration,sampleCount)
    % Match the fixed prediction/replay mesh so conditioned successor roots
    % agree with the previously accepted held-input trajectory.
    persistent keys flows
    key={cfg,duration,sampleCount};index=[];
    for j=1:numel(keys),if isequaln(keys{j},key),index=j;break;end,end
    if isempty(index)
        import casadi.*
        % Align integer sensor blocks with the held-input integration mesh.
        requested=cfg.outputFeedback.validationSubsteps*duration/cfg.controller.sampleTime;
        count=max(1,ceil(requested-8*eps(max(1,requested))));dt=duration/count;
        root=SX.sym('inference_state',6);command=SX.sym('inference_input',2);scale=SX.sym('inference_scale',4);
        options=struct;
        if cfg.outputFeedback.nlpJit
            directory=tempname;mkdir(directory);
            options=struct('jit',true,'compiler','shell','jit_options',struct( ...
                'flags',sprintf('-O%d',cfg.outputFeedback.nlpCompilerOptimization), ...
                'compiler',char(cfg.outputFeedback.nlpCompiler),'directory',[directory,filesep]));
        end
        step=localIntegrationStep(root,command,scale,dt,cfg,options);
        steps=count*sampleCount;
        mapped=step.mapaccum(steps);root=MX.sym('root',6);command=MX.sym('command',2);scale=MX.sym('scale',4);
        states=mapped(root,repmat(command,1,steps),repmat(scale,1,steps));
        flows{end+1}=Function('pcbf_inference',{root,command,scale},{states(:,count:count:end)});keys{end+1}=key;index=numel(keys);
    end
    flow=flows{index};
end

function step=localIntegrationStep(x,u,p,dt,cfg,options)
    import casadi.*
    nominal=localRk4(x,u,p,dt,cfg);
    if dt<=cfg.outputFeedback.sensorPeriod/2
        step=Function('pcbf_rk4',{x,u,p},{nominal},options);
        return
    end
    coarseStep=Function('pcbf_full_step',{x,u,p},{nominal});
    halfStep=Function('pcbf_half_step',{x,u,p},{localRk4(x,u,p,dt/2,cfg)});
    root=MX.sym('root',6);command=MX.sym('command',2);scale=MX.sym('scale',4);
    fine=Function('pcbf_two_halves',{root,command,scale},{halfStep(halfStep(root,command,scale),command,scale)});
    choose=Function.if_else('pcbf_select_step',fine,coarseStep);
    % Extrapolate the current body coupling and road load by one substep.
    % Allow either tire-force direction when selecting the integration mesh.
    vx=root(4);range=cfg.model.kinematicSpeedRange;
    drift=dt*(root(5)*root(6)-nonlinearBicycleModel.symbolicRoadLoad(vx,cfg)/cfg.vehicle.m);
    tireSpan=dt*cfg.vehicle.gravity*mmax(cfg.tire.frictionCoefficient(:).*scale(3:4));
    nearBlend=(fmax(vx,vx+drift+tireSpan)>=range(1)) & (fmin(vx,vx+drift-tireSpan)<=range(2));
    next=choose(nearBlend,root,command,scale);
    step=Function('pcbf_transition_step',{root,command,scale},{next},options);
end

function [next,k1,k2,k3,k4]=localRk4(x,u,p,dt,cfg)
    k1=nonlinearBicycleModel.symbolicDerivative(x,u,p,cfg);
    k2=nonlinearBicycleModel.symbolicDerivative(x+dt*k1/2,u,p,cfg);
    k3=nonlinearBicycleModel.symbolicDerivative(x+dt*k2/2,u,p,cfg);
    k4=nonlinearBicycleModel.symbolicDerivative(x+dt*k3,u,p,cfg);
    next=x+dt*(k1+2*k2+2*k3+k4)/6;
end

function y = localOfSensor(hypothesis,input,cfg,time)
    physical=localOfPhysicalConfiguration(hypothesis,cfg);
    [position,acceleration,yawRate]=localOfEgoSensor(hypothesis,input,physical);
    [relative,detected]=localOfRadar(hypothesis,cfg);
    y=struct('time',time,'position',position,'acceleration',acceleration,'yawRate',yawRate, ...
        'relativePosition',relative,'radarAvailable',detected);
end

function [position,acceleration,yawRate]=localOfEgoSensor(hypothesis,input,physical)
    x=hypothesis.ego;bias=hypothesis.sensorBias;
    dx=nonlinearBicycleModel.derivative(x,input,physical);
    position=x(1:2)+bias(1:2);
    acceleration=[dx(4)-x(6)*x(5);dx(5)+x(6)*x(4)]+bias(3:4);
    yawRate=x(6)+bias(5);
end

function [relative,detected]=localOfRadar(hypothesis,cfg)
    relative=NaN(2,1);detected=false;
    if isempty(hypothesis.target),return;end
    x=hypothesis.ego;rotation=[cos(x(3)),sin(x(3));-sin(x(3)),cos(x(3))];
    offset=rotation*(hypothesis.target(1:2)-x(1:2));
    detected=norm(offset)<=cfg.collision.encounterRangeMeters;
    if detected,relative=offset+hypothesis.sensorBias(6:7);end
end

function [posterior,info] = localOfCondition(hypotheses,block,cfg,initial)
    hypotheses=localOfValidateHypotheses(hypotheses,cfg);
    observe=isempty(hypotheses(1).target) || ...
        (isfield(hypotheses,'targetObserver') && ~isempty(hypotheses(1).targetObserver));
    if observe && ~isfield(hypotheses,'targetObserver'),[hypotheses.targetObserver]=deal([]);end
    persistent updateTarget
    if isempty(updateTarget) && observe && (any([block.samples.radarAvailable]) || any(~cellfun(@isempty,{hypotheses.targetObserver})))
        if exist('onlineNrmmTrackingRuntime','file')~=2
            addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'estimator'));
        end
        updateTarget=@onlineNrmmTrackingRuntime;
    end
    keep=false(1,numel(hypotheses));residual=Inf(numel(hypotheses),1);
    next=hypotheses;bounds=cfg.outputFeedback.noiseBounds;
    tolerance=1e-9; % Numerical measurement comparison only; not extra noise.
    % The sensor mesh and model are shared; the persistent physical
    % parameters remain separate arguments for each hypothesis.
    sampleCount=numel(block.samples);
    if ~initial,flow=localOfInferenceKernel(cfg,cfg.outputFeedback.sensorPeriod,sampleCount);end
    for s=1:numel(hypotheses)
        item=hypotheses(s);excess=-Inf;
        physical=localOfPhysicalConfiguration(item,cfg);
        % Only the target is corrected between readings of a held command.
        if ~initial,egoSamples=full(flow(item.ego,block.heldInput,item.parameters));end
        for j=1:sampleCount
            if ~initial
                item=localOfAdvance(item,block.heldInput,cfg,cfg.outputFeedback.sensorPeriod,j==sampleCount,egoSamples(:,j));
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
                [item.targetObserver,motion]=updateTarget("targetPoint", ...
                    item.targetObserver,measured.time,measuredPosition,item.ego(3),cfg,relativeHeading,positionError);
                positionBox=item.targetObserver.positionBounds;
                if ~isempty(positionBox) && any(positionBox(:,1)>positionBox(:,2)+tolerance)
                    excess=Inf;break
                end
                if item.targetObserver.initialized && isempty(motion)
                    % Disjoint certified intervals exclude this track history.
                    excess=Inf;break
                end
                if measured.radarAvailable && ~isempty(motion)
                    % Use the filtered position inside both the detection
                    % disk and the current bounded position observation.
                    filtered=rotation.'*(item.targetObserver.state(1:2)-item.ego(1:2));
                    filtered=filtered*min(1,radius/max(norm(filtered),realmin));
                    delta=filtered-rho;
                    room=max(0,bounds(4)-norm(corrected-rho));
                    center=measuredPosition+rotation*(min(1,room/max(norm(delta),realmin))*delta);
                    item.target=[center;motion;cfg.target.rearAxleDistance; ...
                        cfg.target.defaultLength/2;cfg.target.defaultWidth/2;0;0];
                elseif ~isempty(motion)
                    item.target=[item.targetObserver.state(1:2);motion;cfg.target.rearAxleDistance; ...
                        cfg.target.defaultLength/2;cfg.target.defaultWidth/2;0;0];
                end
            end
            [position,acceleration,yawRate]=localOfEgoSensor(item,block.heldInput,physical);
            errors=[norm(measured.position-position); ...
                norm(measured.acceleration-acceleration);abs(measured.yawRate-yawRate)];
            excess=max(excess,max(errors-bounds(1:3)));
            if measured.radarAvailable && ~observe
                [relative,detected]=localOfRadar(item,cfg);
                if ~detected
                    excess=Inf;
                else
                    excess=max(excess,norm(measured.relativePosition-relative)-bounds(4));
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

function window = localOfWindow(hypotheses,inheritedCount,cfg)
    % Select coverage directly, before optimization; never try increasing
    % lengths until a solve succeeds. This is an engineering time scale,
    % not a certified Fiala stopping-time or feasibility upper bound.
    h=cfg.controller.sampleTime;
    speed=max(arrayfun(@(s)hypot(s.ego(4),s.ego(5)),hypotheses));
    [avoidance,connection,requested]=localPreviewDurations(hypotheses,cfg);
    count=max(inheritedCount,avoidance+connection);
    assert(count<=cfg.controller.maximumHorizonSteps, ...
        'collisionAvoidanceController:predictionWindowCapacity', ...
        'The selected %.3g s preview needs %d stages, exceeding the configured capacity %g. It was not truncated.', ...
        count*h,count,cfg.controller.maximumHorizonSteps);
    window=struct('source',"calculatedPreview",'requestedSeconds',requested,'allocatedSeconds',count*h, ...
        'horizonSteps',count,'avoidanceSteps',count-connection,'connectionSteps',connection, ...
        'speedUpperMetersPerSecond',speed, ...
        'inheritedHorizonSteps',inheritedCount,'feasibilityCertified',false);
end

function [avoidance,connection,requested]=localPreviewDurations(hypotheses,cfg)
    of=cfg.outputFeedback;h=cfg.controller.sampleTime;
    requested=0;connectionTime=0;
    egoRadius=hypot(cfg.vehicle.length/2,cfg.vehicle.width/2)+norm(cfg.vehicle.rectangleOffset);
    lateralAcceleration=cfg.planner.lateralAccelerationFraction*min(cfg.tire.frictionCoefficient)*cfg.vehicle.gravity;
    for member=hypotheses(:).'
        x=member.ego;q=member.target;
        egoSpeed=hypot(x(4),x(5));
        brakingTime=egoSpeed/of.previewDecelerationMetersPerSecondSquared+of.previewSettlingSeconds;
        leg=brakingTime;
        if ~isempty(q)
            body=predictiveSafetyGeometry.direction(x(3));left=[-body(2);body(1)];
            targetCourse=predictiveSafetyGeometry.direction(q(3)+q(6));
            relativeVelocity=[body,left]*x(4:5)-q(4)*targetCourse;
            span=2*(egoRadius+norm(q(8:9))+norm(q(10:11)));
            % Along the target course, positive relative velocity can carry
            % the ego past the target. Crossing traffic may have no such motion.
            passingSpeed=relativeVelocity.'*targetCourse;
            passTime=Inf;approachTime=Inf;
            if passingSpeed>0
                passTime=span/passingSpeed;
                approachTime=max(0,(q(1:2)-x(1:2)).'*targetCourse)/passingSpeed;
            end
            angle=q(3)-x(3);
            width=cfg.vehicle.width/2+abs(sin(angle))*q(8)+abs(cos(angle))*q(9)+of.clearanceMeters;
            lateralTime=2*sqrt(width/lateralAcceleration);
            leg=min(max(brakingTime,lateralTime),approachTime+lateralTime);
            leg=max(leg,span/max(egoSpeed,cfg.referenceSpeed));
            connectionTime=max(connectionTime,min(passTime,2*sqrt(span/lateralAcceleration)));
        end
        requested=max(requested,max(2*h,leg));
    end
    resolution=of.horizonResolutionSeconds;
    avoidance=ceil(resolution*ceil(requested/resolution)/h);
    connection=ceil(resolution*ceil(connectionTime/resolution)/h);
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
        assert(isfield(prior,'kind') && prior.kind=="compactOutputFeedback-v2", ...
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
    end
    clock=tic;[hypotheses,conditioning]=localOfCondition(hypotheses,block,cfg,initial);
    inferenceSeconds=toc(clock);
    % A dropout alone gives no range information. Keep the physical target
    % in the posterior even when all surviving members place it outside.
    planning=hypotheses;
    inRange=false;
    for member=hypotheses(:).'
        if isempty(member.target),continue;end
        nearest=member.target(1:2);
        if isfield(member,'targetObserver') && isstruct(member.targetObserver) && ...
                isfield(member.targetObserver,'positionBounds') && ~isempty(member.targetObserver.positionBounds)
            box=member.targetObserver.positionBounds;
            nearest=min(box(:,2),max(box(:,1),member.ego(1:2)));
        end
        if norm(nearest-member.ego(1:2))<=cfg.collision.encounterRangeMeters,inRange=true;break;end
    end
    firstRadarTime=[];
    if ~initial,firstRadarTime=prior.firstRadarTime;end
    if isempty(firstRadarTime)
        hit=find([block.samples.radarAvailable],1);
        if ~isempty(hit),firstRadarTime=block.samples(hit).time;end
    end
    targetReady=all(arrayfun(@(s)~isempty(s.target),hypotheses));
    active=targetReady && ~isempty(firstRadarTime) && (block.samples(end).radarAvailable || inRange);
    if ~active,[planning.target]=deal([]);end
    % Acquisition adds collision obligations absent from a cruise-only suffix.
    if active && ~initial && ~prior.collisionTargetActive,inherited=0;end
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
        members(j).motionLimits=[];members(j).cover=[];members(j).positionRadius=0;
        q=members(j).target;track=[];
        if isfield(members,'targetObserver'),track=members(j).targetObserver;end
        if isempty(q) || isempty(track),continue;end
        if isfield(track,'positionBounds') && ~isempty(track.positionBounds)
            box=track.positionBounds;
            if q(4)==0 && isfield(track,'restBounds') && ~isempty(track.restBounds)
                box=track.restBounds;
            end
            q(1:2)=mean(box,2);
            half=max(0,(box(:,2)-box(:,1))/2);
            members(j).positionRadius=norm(half);
            if q(4)==0
                % A translation box in world coordinates, projected onto
                % the two fixed axes of the absorbing footprint.
                c=abs(cos(q(3)));s=abs(sin(q(3)));
                projection=[c,s;s,c]*half;
            else
                % The target axes can turn at later prediction nodes.
                projection=repmat(norm(half),2,1);
            end
            q(8:9)=q(8:9)+projection;
            members(j).target=q;
        end
        if q(4)==0,continue;end
        b=track.motionBounds;
        members(j).target(5:6)=min(b(2,:).',max(b(1,:).',q(5:6)));
        if any(b(1,:)~=b(2,:))
            members(j).motionLimits=b;
            members(j).cover=localCompactCover(members(j),cfg,count+1);
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
    validationModel=localPerformanceModel(cfg,hypotheses,previous,holds,lane,nominal,kernel);
    item=hypotheses(nominal);
    tolerance=cfg.outputFeedback.nlpTolerance;
    best=[];traces={};violation=Inf;validationSeconds=0;retained=false;
    inheritedReusedHolds=zeros(1,numel(hypotheses));solveClock=tic;
    startupBuildSeconds=0;
    firstEncounter=~initial && ~prior.collisionTargetActive && ~isempty(item.target);
    budget=cfg.outputFeedback.maximumSolveSeconds;budgetPhase="continuation";
    if initial || firstEncounter
        budget=cfg.outputFeedback.initializationSolveSeconds;budgetPhase="initialization";
        if ~cacheHit,startupBuildSeconds=buildSeconds;end
    end
    preparing=strcmp(request.operation,"prepareStep");
    if preparing,budget=realmax;end
    inherit=~initial;
    attempts=0;iterationCount=0;calls=0;reuseCount=0;restoration=false;seedCount=0;gradientCalls=0;
    solverStatus="notCalled";converged=false;ipoptSeconds=0;constraintCount=0;
    cacheU=[];cache=[];rollouts=[];initialDeficit=NaN;initialTerminal=NaN;initialPeakDeficit=NaN;
    stabilitySteps=0;firstFeasibleStability=NaN;incumbentStability=NaN;
    reducedSearch=false;reducedPoint=[];reducedCalls=0;reducedAccepted=false;
    domainKnots=[];midpointReplays=0;meritTransition=0;
    bestPoint=[];bestResidual=Inf;
    bestCost=Inf;inheritedCost=NaN;deadline=budget+startupBuildSeconds;
    budgetStopped=false;budgetStopPhase="";
    bestLoss=Inf;bestSlacks=[];priorCap=Inf;slackRoundoff=128*eps(max(1,cfg.outputFeedback.clearanceMeters))*count;
    contained=inherit && ~firstEncounter && cfg.outputFeedback.clearanceMeters==prior.configuration.outputFeedback.clearanceMeters ...
        && localCompactShiftContained(prior,hypotheses,conditioning.retainedIndices,h);
    sameEncounter=inherit && ~firstEncounter && cfg.outputFeedback.clearanceMeters==prior.configuration.outputFeedback.clearanceMeters;
    if sameEncounter
        priorCap=sum(prior.stageDeficits(2:end));
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
        % Extend an active encounter with the same common neutral handoff
        % command that was checked for every physical member.
        cruise=terminalSafeSet.continuationControl(cfg,~isempty(item.target));
        last=prior.fineControls(:,end);
        extension=localCompactRateLimit(repmat(cruise,1,extra),last,ones(1,extra),cfg);
        inherited=[suffix,extension];
        initializationSource="shiftedMovingTerminalLaw";
        if toc(timer)<budget
            try
                guide=localCompactSeed(item,previous,1,lane,cfg);
                proposal=inherited;proposal(:,1)=guide;
                proposal=localCompactRateLimit(proposal,previous,ones(1,count),cfg);
                if ~isequal(proposal,inherited)
                    validatePlan(proposal,[]);
                    if ~isempty(best),initializationSource="guideWithDrivingSuffix";end
                end
            catch exception
                % An optional field splice can leave the Fiala domain.
                % The shifted controls still require their own replay.
                if ~strcmp(exception.identifier,'collisionAvoidanceController:invalidTireOperatingPoint'),rethrow(exception);end
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
            model=localAddControlMap(validationModel);
            coldCruise=initial && isempty(item.target);
            if ~coldCruise,checkClock();end
            if inherit && ~firstEncounter && ~isempty(item.target)
                guideInputs=inherited;
                controls=guideInputs(:,cumsum(holds));
                initializationSource="shiftedMovingTerminalLaw";
            else
                [controls,guideInputs]=localCompactSeed(item,previous,holds,lane,cfg,window.avoidanceSteps);
                seedCount=1;
            end
            if isempty(item.target)
                clockCheck=@checkClock;
                if coldCruise,clockCheck=[];end
                validatePlan(controls*model.map.'+model.offset,[],clockCheck);
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
                physicalScale=controlScale;
                seekZero=~isempty(item.target);
                e=evaluate(start,false);
                initialPeakDeficit=max(0,max(e.stage));
                initialDeficit=sum(max(0,e.stage));initialTerminal=max(e.terminal);
                considerSolution(start);
                restoration=true;
                fullModel=model;
                if seekZero && bestLoss>lossFloor+slackRoundoff && numel(hypotheses)>1
                    working=false(1,numel(hypotheses));
                    while nnz(working)<numel(working)-1 && bestLoss>lossFloor+slackRoundoff
                        score=zeros(size(working));
                        for j=find(~working)
                            flow=rollouts.trajectories{j}.oracle;
                            score(j)=sum(max(0,flow.localStage).^2)+sum(max(0,flow.forward).^2)+max(0,flow.terminal)^2;
                        end
                        score(working)=-Inf;[~,worst]=max(score);working(worst)=true;
                        model=fullModel;model.hypotheses=hypotheses(working);
                        model.rearSlipScale=fullModel.rearSlipScale(working);model.nominal=1;
                        reducedSearch=true;reducedPoint=[];cacheU=[];rollouts=[];
                        evaluate(start,false);
                        if isempty(reducedPoint),solveIpopt(start);end
                        reducedSearch=false;model=fullModel;cacheU=[];rollouts=[];
                        if ~isempty(reducedPoint),start=reducedPoint;end
                        evaluate(start,false);
                    end
                end
                if preparing && seekZero && bestLoss<=lossFloor+slackRoundoff
                    % A feasible private seed would skip the optimizer online.
                    % Exercise its actual horizon Jacobian and IPOPT callbacks.
                    [cache,rollouts]=localCondensedOracle(start,model,true,rollouts,[],false);
                    gradientCalls=gradientCalls+1;
                end
                if bestLoss>lossFloor+slackRoundoff || (preparing && seekZero)
                    solveIpopt(start);
                end
                if seekZero && bestLoss>lossFloor+slackRoundoff && toc(timer)<deadline
                    seekZero=false;
                    % The primary NLP also needs Jacobians of satisfied constraints.
                    cache.derivatives=false;
                    if ~isempty(bestPoint),start=bestPoint;end
                    solveIpopt(start);
                end
            end
        catch exception
            if ~strcmp(exception.identifier,'collisionAvoidanceController:condensedDeadline'),rethrow(exception);end
        end
    end
    if isempty(best)
        error('collisionAvoidanceController:compactNoFeasiblePlan', ...
            'No validated PCBF plan within the %s search. No command was committed.',budgetPhase);
    end
    solveSeconds=toc(solveClock);
    retainedTrajectories=cell(1,numel(hypotheses));
    for s=1:numel(hypotheses)
        trace=traces{s};
        retainedTrajectories{s}=struct('inputs',trace.inputs,'denseEgo',trace.denseEgo, ...
            'collisionEgo',trace.collisionEgo,'validationSubsteps',kernel.validationSubsteps, ...
            'parameters',hypotheses(s).parameters,'target',hypotheses(s).target,'integrationRule',"transitionRefinedRK4");
    end
    input=best(:,1);state=struct('kind',"compactOutputFeedback-v2",'time',request.block.time,'memory',zeros(0,1), ...
        'hypotheses',hypotheses,'appliedInput',input,'controls',best(:,boundaries(2:end)), ...
        'fineControls',best,'firstFeasibleDrivingPlan',stabilitySteps==0,'boundaries',boundaries,'horizonSteps',count,'configuration',cfg, ...
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
        'reducedSearchEvaluations',reducedCalls,'reducedSearchAccepted',reducedAccepted, ...
        'midpointReplays',midpointReplays, ...
        'fallbackUsed',retained && stabilitySteps==0,'retainedFinePlanUsed',retained && ~initial,'rootTerminalPlanUsed',false, ...
        'retainedTailRepaired',false, ...
        'restorationAttempted',restoration,'initialRestorationSlack',initialDeficit, ...
        'initialPeakClearanceDeficitMeters',initialPeakDeficit, ...
        'initialNlpViolation',max(0,initialTerminal),'finalRestorationSlack',bestLoss, ...
        'validationSubsteps',kernel.validationSubsteps,'sensitivitySubsteps',kernel.sensitivitySubsteps,'nonlinearValidationPerformed',true, ...
        'maximumConstraintViolation',violation,'scenarioCount',numel(hypotheses), ...
        'activeScenarioCount',numel(hypotheses),'hypothesisCount',numel(hypotheses),'memoryDimension',0, ...
        'conditioning',conditioning,'terminalSetEnforced',true,'terminalSafetyAssumed',true, ...
        'terminalModel',"collisionFreeContinuation",'terminalValue',max(cellfun(@(t)t.terminal.value,traces)), ...
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
        tracking=localCompactPerformance(checked{nominal}.ego(:,2:end),inputs,validationModel);
        score=h/2*sum(inputs(2,:).^2+inputs(2,:).^4);
        for k=1:numel(hypotheses)
            residual=localRearSlipResidual(checked{k}.collisionEgo(:,2:end),validationModel,k);
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

    function solveIpopt(start)
        import casadi.*
        % IPOPT owns all search directions, restoration and step lengths.
        % Scale knots before exposing the exact nonlinear values to CasADi.
        checkClock();
        domainKnots=[];
        if exist('casadiFunctionCallback','class')~=8
            addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'scripts'));
        end
        residualCount=2*count+numel(model.hypotheses);
        if seekZero,outputCount=1;else,outputCount=residualCount;end
        % In a clearance-only restoration, equilibrate each actuator knot
        % by the diagonal of the regularized local least-squares model of
        % the pseudo-Huber merit. A unit scaled step then accounts for its
        % effect on every violated clearance. All bounds use the same map.
        controlScale=physicalScale;
        meritTransition=0;
        if seekZero
            seed=evaluate(start,false);
            active=seed.stage>0;
            if any(active) && all(seed.forward<=0) && all(seed.terminal<=0)
                meritTransition=cfg.outputFeedback.clearanceMeters;
                seed=evaluate(start,true);
                jacobian=seed.stageGradient(:,active).*physicalScale;
                weights=1./hypot(1,seed.stage(active));
                diagonal=1+jacobian.^2*weights;
                if all(isfinite(diagonal))
                    controlScale=physicalScale./sqrt(diagonal);
                end
            end
        end
        oracle=casadiFunctionCallback('pcbf_rollout',[nu,1],[outputCount,1], ...
            @ipoptValues,@ipoptJacobian);
        oracleLifetime=onCleanup(@()oracle.release());
        v=MX.sym('knots',nu);value=oracle.call({v});value=value{1};
        inputRows=sparse(a.*controlScale.');
        constraints=inputRows*v-b;
        if seekZero
            % The scalar merit proposes a zero-deficit trajectory. Only a
            % full nonlinear replay can authorize dispatch.
            objective=value;
        else
            stage=value(1:count);
            objective=sum(fmax(stage,0));
            hard=[value(count+1:end);constraints];
            if priorCap==0
                constraints=[stage;hard];
            else
                constraints=[mmax(stage)-cfg.outputFeedback.clearanceMeters;hard];
                if isfinite(priorCap),constraints=[constraints;objective-priorCap];end
            end
        end
        ng=size(constraints,1);constraintCount=ng;
        % nlpsol outputs x, f, g, lam_x, lam_g, lam_p.
        stop=casadiFunctionCallback('pcbf_stop',[[nu;1;ng;nu;ng;0],ones(6,1)],[1,1], ...
            @stopIpopt);
        stopLifetime=onCleanup(@()stop.release());
        options=struct('print_time',false,'error_on_fail',false,'calc_lam_p',false, ...
            'iteration_callback',stop,'show_eval_warnings',false, ...
            'ipopt',struct('print_level',0,'sb','yes','max_iter',cfg.outputFeedback.nlpMaximumIterations, ...
            'hessian_approximation','limited-memory','bound_relax_factor',0, ...
            'mu_strategy','monotone','mu_init',1e-6,'bound_push',1e-6,'bound_frac',1e-6, ...
            'tol',tolerance,'constr_viol_tol',tolerance));
        if seekZero
            % A rejected trial costs a full nonlinear rollout. Backtrack by
            % two halvings at a time when searching for a zero PCBF deficit.
            options.ipopt.alpha_red_factor=0.25;
        end
        % The offline prewarm uses realmax. Passing it through the CasADi
        % options converter can overflow; omit a wall limit for that case.
        if deadline<flintmax,options.ipopt.max_wall_time=max(.001,deadline-toc(timer));end
        nlp=nlpsol('compact_pcbf','ipopt',struct('x',v,'f',objective,'g',constraints),options);
        solverLifetime=onCleanup(@()delete(nlp));
        attempts=attempts+1;
        ipoptClock=tic;
        answer=nlp('x0',start./controlScale,'lbx',lower./controlScale,'ubx',upper./controlScale,'lbg',-inf(ng,1),'ubg',zeros(ng,1));
        ipoptSeconds=ipoptSeconds+toc(ipoptClock);
        stats=nlp.stats();solverStatus=string(stats.return_status);
        iterationCount=iterationCount+stats.iter_count;
        converged=stats.success;
        if toc(timer)>=deadline,budgetStopped=true;budgetStopPhase=budgetPhase;end
        % A solver status alone never authorizes an actuator command.
        point=full(answer.x);
        clear solverLifetime options objective constraints hard stage value
        clear stopLifetime oracleLifetime
        if bestLoss>lossFloor+slackRoundoff && all(isfinite(point)) && toc(timer)<deadline
            evaluate(point(1:nu).*controlScale,false);
        end
    end

    % Keep callback state in the controller frame. A callback must not retain
    % the solver workspace that owns it across an encounter or MATLAB exit.
    function stop=stopIpopt(args)
        stop=double(toc(timer)>=deadline || (reducedSearch && ~isempty(reducedPoint)) || bestLoss<=lossFloor+slackRoundoff);
        % Let IPOPT exercise its linear solver on the discarded startup call.
        if preparing && bestLoss<=lossFloor+slackRoundoff,stop=0;return;end
        if stop || ~seekZero || reducedSearch,return;end
        knots=full(args{1}).*controlScale;
        if ~isequal(knots,cacheU)
            domainKnots=[];
            return
        end
        geometryPassed=all(cache.stage<=0) && all(cache.terminal<=tolerance);
        if ~geometryPassed && all(cache.forward<=0)
            domainKnots=knots;
            return
        end
        % The accepted step can clear the obstacle but leave the tire domain.
        % Replay one interior control sequence before requesting another step.
        if ~isempty(domainKnots) && geometryPassed && any(cache.forward>0)
            midpoint=domainKnots+(knots-domainKnots)/2;
            if all(a*midpoint-b<=tolerance) && all(midpoint>=lower-tolerance & midpoint<=upper+tolerance)
                midpointReplays=midpointReplays+1;
                inputs=reshape(midpoint,2,nodes)*model.map.'+model.offset;
                try
                    validatePlan(inputs,[],@checkClock);
                catch exception
                    if ~strcmp(exception.identifier,'collisionAvoidanceController:condensedDeadline'),rethrow(exception);end
                end
                stop=double(bestLoss<=lossFloor+slackRoundoff || toc(timer)>=deadline);
            end
        end
        domainKnots=[];
    end

    function output=ipoptValues(args)
        e=evaluate(full(args{1}).*controlScale,false);
        output=[e.stage;e.forward;e.terminal];
        if seekZero
            residual=max(output,0);
            output=localFeasibilityMerit(residual,meritTransition,tolerance);
        end
    end

    function output=ipoptJacobian(args)
        u=full(args{1}).*controlScale;
        if seekZero
            e=evaluate(u,false);
            residual=max([e.stage;e.forward;e.terminal],0);
            % At zero every squared positive-part term has zero derivative,
            % including a constraint exactly on its boundary.
            if ~any(residual)
                output=zeros(1,nu);
                return
            end
        end
        e=evaluate(u,true);
        gradients=[e.stageGradient,e.forwardGradient,e.terminalGradient];
        if seekZero
            [~,weights]=localFeasibilityMerit(residual,meritTransition,tolerance);
            gradients=gradients*weights;
        end
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
            [cache,rollouts]=localCondensedOracle(u,model,derivatives,[],[],false);
            cacheU=u;calls=calls+1;gradientCalls=gradientCalls+derivatives;
            reducedCalls=reducedCalls+reducedSearch;
            if restoration,considerSolution(u);end
        elseif derivatives && ~cache.derivatives
            if seekZero && numel(model.hypotheses)==1 && all(cache.forward<=0) && all(cache.terminal<=0)
                cache=localClearanceMeritDerivatives(cache,rollouts.trajectories{1}.oracle,model);
            else
                [cache,rollouts]=localCondensedOracle(u,model,true,rollouts,[],false,seekZero && numel(model.hypotheses)==1);
            end
            gradientCalls=gradientCalls+1;
        end
        result=cache;
    end
    function considerSolution(u)
        if ~reducedSearch
            residual=sum(max(0,[cache.stage;cache.terminal;cache.forward;a*u-b;u-upper;lower-u]).^2);
            if residual<bestResidual,bestResidual=residual;bestPoint=u;end
        end
        loss=sum(max(0,cache.stage));
        if loss>bestLoss+tolerance || loss>priorCap+slackRoundoff || max(cache.stage)>=cfg.outputFeedback.clearanceMeters || ...
                max([cache.forward;cache.terminal;a*u-b;lower-u;u-upper])>tolerance
            return
        end
        inputs=reshape(u,2,nodes)*model.map.'+model.offset;
        if reducedSearch
            reducedPoint=u;
            [selected,~]=validatePlan(inputs,[]);
            reducedAccepted=reducedAccepted || selected;
        else
            validatePlan(inputs,rollouts);reuseCount=reuseCount+1;
        end
    end
end

function [value,weights]=localFeasibilityMerit(residual,transition,tolerance)
    scale=hypot(1,residual);
    mass=sum(residual.*(residual./(1+scale)));
    % Keep the original search slope above the clearance scale. Below it,
    % a regularized square root counters the flattening of the squared loss.
    % Both branches have the same value and first derivative at the join.
    join=sqrt(transition^2+tolerance^2);
    if 2*mass<transition^2
        root=sqrt(2*mass+tolerance^2);
        value=join*(2*mass/(root+tolerance));
        slope=join/root;
    else
        value=mass+transition^2/2-tolerance*transition^2/(join+tolerance);
        slope=1;
    end
    if nargout>1,weights=(residual./scale)*slope;end
end

function model=localPerformanceModel(cfg,hypotheses,previous,holds,lane,nominal,kernel)
    model=struct('cfg',cfg,'hypotheses',hypotheses,'previous',previous,'kernel',kernel,'nominal',nominal);
    tire=nonlinearBicycleModel.tireParameters(cfg);
    scales=[hypotheses.parameters];
    model.rearSlipScale=(tire.longitudinalForceScale(2)/tire.corneringStiffness(2))*scales(4,:)./scales(2,:);
    x=hypotheses(nominal).ego;
    distances=cumsum(holds)*cfg.controller.sampleTime*cfg.referenceSpeed;
    points=x(1:2)+[cos(x(3));sin(x(3))]*distances;
    projection=predictiveSafetyGeometry.project(points,lane);
    model.reference=[projection.point;projection.heading];
end

function model=localAddControlMap(model)
    % Shifted executable commands can be evaluated directly on the hold grid.
    holds=model.kernel.holds;count=sum(holds);nodes=numel(holds);
    map=zeros(count,nodes);offset=zeros(2,count);row=0;
    for i=1:nodes
        for j=1:holds(i)
            row=row+1;fraction=j/holds(i);map(row,i)=fraction;
            if i==1,offset(:,row)=(1-fraction)*model.previous;else,map(row,i-1)=1-fraction;end
        end
    end
    model.map=map;model.offset=offset;model.controlMap=kron(map,eye(2));
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

function [kernel,hit] = localCompactKernel(cfg,holds,hasTarget,tangentSteps)
    persistent keys kernels
    of=cfg.outputFeedback;h=cfg.controller.sampleTime;
    if nargin<4
        tangentSteps=min(of.validationSubsteps,max(2,ceil(h/(2*of.sensorPeriod))));
    end
    key=cfg;hit=false;
    for k=1:numel(keys)
        if kernels{k}.sensitivitySubsteps==tangentSteps && isequaln(key,keys{k})
            kernel=kernels{k};kernel.holds=holds;hit=true;return;
        end
    end
    import casadi.*
    maximumSpeed=hypot(cfg.model.speedMaximum,cfg.model.lateralVelocityMaximum);
    blockSeconds=maximumSpeed/of.previewDecelerationMetersPerSecondSquared+of.previewSettlingSeconds;
    capacity=max(2,ceil(blockSeconds/h));K=of.validationSubsteps;
    M=K;dt=h/M;
    x=SX.sym('x',6);u=SX.sym('u',2);p=SX.sym('p',4);
    options=struct;
    if of.nlpJit
        directory=tempname;mkdir(directory);options=struct('jit',true,'compiler','shell', ...
            'jit_options',struct('flags',sprintf('-O%d',of.nlpCompilerOptimization), ...
            'compiler',char(of.nlpCompiler),'directory',[directory,filesep]));
    end
    step=localIntegrationStep(x,u,p,dt,cfg,options);
    root=MX.sym('root',6);command=MX.sym('command',2);parameters=MX.sym('parameters',4);
    mappedStep=step.mapaccum(M);ends=mappedStep(root,repmat(command,1,M),repmat(parameters,1,M));
    points=ends;
    last=ends(:,end);stopped=(command(2)<=0)&(last(4)<.2*cfg.model.brakeHoldSpeed);
    finish=[last(1:3);if_else(stopped,zeros(3,1),last(4:6))];
    hold=Function('pcbf_hold',{root,command,parameters},{finish,points});
    % RK4 continuous extension supplies dense physical-domain samples and
    % their derivatives without an additional integration at every sample.
    % IPOPT uses this approximation to the trajectory Jacobian.
    % The nonlinear oracle and acceptance gate always use the flow above.
    M=tangentSteps;D=K/M;dt=h/M;
    [next,k1,k2,k3,k4]=localRk4(x,u,p,dt,cfg);
    sensitivity=SX.sym('sensitivity',6,8);
    updated=jacobian(next,x)*sensitivity+[SX.zeros(6,6),jacobian(next,u)];
    % Only the binding physical-domain sample needs a within-hold Jacobian.
    % Geometry and the terminal law use the post-reset boundary Jacobians.
    index=MX.sym('index');initialSensitivity=DM([eye(6),zeros(6,2)]);
    augmented=[x;sensitivity(:)];
    if M==K && dt>of.sensorPeriod/2
        exact=step(root,command,parameters);seed=MX.sym('seed',6,8);
        change=jacobian(exact,root)*seed+[MX.zeros(6,6),jacobian(exact,command)];
        variation=Function('pcbf_variation',{[root;seed(:)],command,parameters},{[exact;change(:)]},options);
    else
        variation=Function('pcbf_variation',{augmented,u,p},{[next;updated(:)]},options);
    end
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

function [e,rollouts]=localCondensedOracle(u,model,derivatives,rollouts,checkBudget,performance,positiveOnly)
    if nargin<7,positiveOnly=false;end
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
    if derivatives
        e.stageGradient=zeros(nu,N);e.forwardGradient=zeros(nu,N);
        e.terminalGradient=zeros(nu,numel(model.hypotheses));
    end
    if performance
        ratio=inputs(2,:).';
        e.stabilityCost=h/2*sum(ratio.^2+ratio.^4);
        if derivatives
            e.costGradient=zeros(nu,1);e.costHessian=zeros(nu);
            ratioJacobian=model.map;
            squaredRatioJacobian=2*ratio.*ratioJacobian;
            e.stabilityGradient=zeros(nu,1);
            e.stabilityHessian=zeros(nu);
            e.stabilityGradient(2:2:end)=h*(ratioJacobian.'*ratio+squaredRatioJacobian.'*(ratio.^2));
            e.stabilityHessian(2:2:end,2:2:end)=h*(ratioJacobian.'*ratioJacobian+squaredRatioJacobian.'*squaredRatioJacobian);
        end
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
            [forward,fi,domainState,domainInput]=localCompactModelDomain(dense,allInputs,cfg,K);
            target=[];if ~isempty(q),target=q(:,end);end
            [terminal,~,g]=localCompactMovingValue(ends(:,end),target,item,total*h,cfg,remaining);
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
        if performance
            if derivatives
                [residual,slipDerivative]=localRearSlipResidual(ends(:,1:N),model,s);
            else
                residual=localRearSlipResidual(ends(:,1:N),model,s);
            end
            weight=h/numel(model.hypotheses);
            e.stabilityCost=e.stabilityCost+weight/2*sum(residual.^2);
        end
        slew=-Inf;channel=1;
        if total>N
            trim=allInputs(:,end);
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
        if positiveOnly
            % Satisfied rows have exactly zero weight in the scalar merit.
            stageNeeded(1:N)=stageNeeded(1:N) & localStage(1:N)>0;
            domainNeeded(1:N)=forward(1:N)>0;
        end
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
        for i=find(stageNeeded)
            S=SE(:,:,nodeIndices(i));
            stageGradient(:,i)=-(gradient(:,i).'*S(1:3,:)).';
        end
        updateStage=localStage(1:N).'>e.stage;
        e.stage(updateStage)=localStage(updateStage);
        e.stageGradient(:,updateStage)=stageGradient(:,updateStage);
        updateDomain=forward(1:N).'>e.forward;
        e.forward(updateDomain)=forward(updateDomain);
        for i=find(updateDomain.' & domainNeeded(1:N))
            e.forwardGradient(:,i)=(domainState(:,i).'*SF(:,:,i)+domainInput(:,i).'*derivativeModel.controlMap(2*i-1:2*i,:)).';
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
        if ~performance,continue;end
        jacobian=zeros(N,nu);
        for j=1:N
            sensitivity=SE(:,:,j+1);
            jacobian(j,:)=slipDerivative(:,j).'*sensitivity(4:6,:);
        end
        e.stabilityGradient=e.stabilityGradient+weight*jacobian.'*residual.';
        e.stabilityHessian=e.stabilityHessian+weight*(jacobian.'*jacobian);
        if s==model.nominal
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

function e=localClearanceMeritDerivatives(e,flow,model)
    % The full rollout identifies which node determines each stage deficit.
    % Only positive deficits contribute to the zero-deficit search gradient.
    width=size(model.controlMap,2);
    e.stageGradient=zeros(width,numel(e.stage));
    e.forwardGradient=zeros(width,numel(e.forward));
    e.terminalGradient=zeros(width,1);
    e.derivatives=true;
    stages=find(e.stage>0);
    nodes=flow.nodeIndices(stages);
    % Node 1 is fixed. Propagate through the hold ending at the last
    % contributing node; later controls cannot affect its clearance.
    lastHold=max([1;nodes(:)])-1;
    if lastHold==0,return;end
    item=model.hypotheses(1);
    states=[item.ego,flow.ends(:,1:lastHold)];
    model.controlMap=model.controlMap(1:2*lastHold,:);
    [~,sensitivity]=localCondensedSensitivities(states(:,1:lastHold),flow.allInputs(:,1:lastHold), ...
        item.parameters,flow.fi(1:lastHold),false(1,lastHold),model);
    vehicle=model.cfg.vehicle;
    shape=[vehicle.length/2;vehicle.width/2;vehicle.rectangleOffset];
    gradients=localCompactDistanceGradient(states(:,nodes),flow.boxes(:,nodes),shape);
    for j=1:numel(stages)
        e.stageGradient(:,stages(j))=-(gradients(:,j).'*sensitivity(1:3,:,nodes(j))).';
    end
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
    rates=[cfg.outputFeedback.steeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    lower=[-cfg.outputFeedback.steeringMaximum;max(-1,cfg.actuation.brakingRatioMinimum)];
    upper=[cfg.outputFeedback.steeringMaximum;min(1,cfg.actuation.brakingRatioMaximum)];
    if all(isinf(rates))
        controls=max(lower,min(upper,controls));
        return
    end
    for i=1:numel(holds)
        change=rates*(cfg.controller.sampleTime*holds(i));
        controls(:,i)=max(previous-change,min(previous+change,controls(:,i)));
        controls(:,i)=max(lower,min(upper,controls(:,i)));
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
        'lane',lane,'road',struct('lateralClearance',[]),'frame',frame,'nominalReference',nominal,'motionCover',item.cover, ...
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
        fieldInputs=predictiveSafetyGeometry.avoidanceInputs(model,avoidance);
        % Only the first leg comes from the planner. Trim initializes the
        % remaining actuator variables, which are free in the same NLP.
        inputs=[fieldInputs,repmat(terminalSafeSet.movingControl(cfg),1,count-avoidance)];
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
                compatible=isfield(old,'integrationRule') && isequal(old.integrationRule,"transitionRefinedRK4") ...
                    && old.validationSubsteps==substeps && oldCount>=1 ...
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
    nBridge=size(tailInputs,2);
    normalWitness=zeros(2,1);exitSteps=0;terminalValue=0;member=true;
    if nBridge>0
        command=tailInputs(:,1);
        [domain,~,~,~,defined]=localCompactModelDomain(tail,tailInputs,cfg,kernel.validationSubsteps);
        times=start+(0:nBridge)*h;
        q=localCompactTargetNodes(item.target,times);
        shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
        if isempty(item.cover)
            gaps=predictiveSafetyGeometry.rectangleClearance([x,tailEnds],q,shape);
        else
            [gaps,~,remaining]=localCompactOccupied(item,[x,tailEnds],times,shape,cfg,remaining);
        end
        [core,normalWitness,~,coreSteps]=localCompactMovingValue(tailEnds(:,end),q(:,end),item,times(end),cfg,remaining);
        exitSteps=nBridge+coreSteps;
        rates=[cfg.outputFeedback.steeringRateMaximum;cfg.model.brakingRatioRateMaximum];
        slew=abs(command-previous)-h*rates;
        terminalValue=max([0,max(domain),cfg.outputFeedback.clearanceMeters-min(gaps),core,slew.']);
        member=defined && all(gaps>0) && ...
            terminalValue<=cfg.outputFeedback.nlpTolerance && ...
            core<cfg.outputFeedback.clearanceMeters;
    else
        % No encounter action is required. The next nominal input must
        % still be reachable from the preceding actuator command.
        command=terminalSafeSet.continuationControl(cfg,~isempty(item.target));
        command=localCompactRateLimit(command,previous,1,cfg);
    end
    terminal=struct('value',terminalValue,'member',member, ...
        'input',command,'bridgeHolds',nBridge,'exitSteps',exitSteps, ...
        'exitStepsAreUpperBound',~isempty(item.cover) && isfinite(exitSteps) && exitSteps>nBridge,'normal',normalWitness, ...
        'model',"collisionFreeContinuation",'safetyAssumed',true,'safetyProved',false);
end

function [dense,ends,inputs]=localCompactCruiseRollout(kernel,x,item,start,cfg,checkBudget,remaining)
    if ~isempty(checkBudget),checkBudget();end
    if isempty(item.cover)
        target=item.target;
        if ~isempty(target),target=localCompactTargetNodes(target,start);end
        released=isempty(target) || norm(x(1:2)-target(1:2))>cfg.collision.encounterRangeMeters+item.positionRadius;
    else
        released=~remaining;
    end
    if released
        dense=x;ends=zeros(6,0);inputs=zeros(2,0);return;
    end
    inputs=terminalSafeSet.continuationControl(cfg,true);
    [points,ends]=localCompactHeldFlow(kernel,x,inputs,item.parameters,checkBudget);
    dense=[x,points];
end

function cover=localCompactCover(item,cfg,count)
    h=cfg.controller.sampleTime;bounds=item.motionLimits;
    times=(0:count)*h;
    [boxes,centers]=predictiveSafetyGeometry.targetMotionEnvelope(item.target,bounds,times,[],cfg.target.observerDomain);
    cover=struct('h',h,'boxes',boxes,'centers',centers);
    % A single disk encloses the velocity over the full observer interval at handoff.
    t=times(end);slow=item.target;fast=slow;
    slow(5)=bounds(1,1);fast(5)=bounds(2,1);
    [sMin,vMin]=predictiveSafetyGeometry.targetArc(slow,t);
    [sMax,vMax]=predictiveSafetyGeometry.targetArc(fast,t);
    kMin=sin(bounds(1,2))/item.target(7);kMax=sin(bounds(2,2))/item.target(7);
    turns=[kMin*sMin,kMin*sMax,kMax*sMin,kMax*sMax];
    angLo=min(turns);angHi=max(turns);
    if isfield(cfg.target.observerDomain,'yawRateMaximum')
        limit=t*cfg.target.observerDomain.yawRateMaximum;
        angLo=max(-limit,min(limit,angLo));angHi=max(-limit,min(limit,angHi));
    end
    if isfield(cfg.target.observerDomain,'speedMaximum')
        vMax=min(vMax,cfg.target.observerDomain.speedMaximum);vMin=min(vMin,vMax);
    end
    lo=angLo+bounds(1,2);hi=angHi+bounds(2,2);
    angle=item.target(3)+(lo+hi)/2;half=min(pi,(hi-lo)/2);
    % Minimize the worst squared distance to the two radial endpoints
    % of the angular sector. Its enclosing disk lies on the bisector.
    cosine=cos(half);mid=0;
    if cosine>0
        mid=min(vMax*cosine,(vMin+vMax)/(2*cosine));
    end
    cover.velocity=mid.*predictiveSafetyGeometry.direction(angle);
    % Law of cosines at the angular extreme and the two speed endpoints.
    if vMin==0 && cosine>0 && mid==vMax/(2*cosine)
        % The origin and both outer corners are co-circular. Use the same
        % norm as the drift test to preserve exact tangency at rest.
        cover.velocityRadius=vecnorm(cover.velocity);
    else
        cover.velocityRadius=sqrt(max((vMin-mid).^2+4*vMin.*mid.*sin(half/2).^2, ...
            (vMax-mid).^2+4*vMax.*mid.*sin(half/2).^2));
    end
end

function [gap,boxes,remaining]=localCompactOccupied(item,states,times,shape,cfg,remaining)
    cover=item.cover;nodes=round(times/cover.h)+1;
    boxes=cover.boxes(:,nodes);gap=inf(1,numel(nodes));
    if nargin<6 || isempty(remaining),remaining=true;end
    if ~remaining,return;end
    ranges=predictiveSafetyGeometry.targetEnvelopeRange(states,boxes,cover.centers(:,nodes));
    last=find(ranges>cfg.collision.encounterRangeMeters+item.positionRadius,1);
    remaining=isempty(last);
    if remaining,last=numel(nodes);end
    gap(1:last)=predictiveSafetyGeometry.rectangleClearance(states(:,1:last),boxes(:,1:last),shape);
end

function [value,normal,gradient,exitSteps]=localCompactMovingValue(x,target,item,start,cfg,remaining)
    if isempty(item.motionLimits)
        cfg.collision.encounterRangeMeters=cfg.collision.encounterRangeMeters+item.positionRadius;
        [value,normal,gradient,exitSteps]=terminalSafeSet.movingValue(x,target,cfg);
        return
    end
    R=cfg.collision.encounterRangeMeters;
    value=-R;normal=zeros(2,1);gradient=zeros(1,6);exitSteps=0;
    if ~remaining,return;end
    node=round(start/item.cover.h)+1;
    boxes=item.cover.boxes(:,node);
    centers=item.cover.centers(:,node);
    posRadius=item.positionRadius+vecnorm(max(abs(centers([1 3],:)),abs(centers([2 4],:))));
    [value,normal,gradient,exitSteps]=terminalSafeSet.separatingMotion(x,boxes, ...
        item.cover.velocity,item.cover.velocityRadius,posRadius,cfg);
end


function q=localCompactTargetNodes(target,times)
    if target(4)==0,q=repmat(target,1,numel(times));
    else,q=predictiveSafetyGeometry.predictTarget(target,times);end
end

function [value,index,stateGradient,inputGradient,defined]=localCompactModelDomain(states,inputs,cfg,K)
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
    if nargout<3,return;end
    selected=(0:count-1)*(K+1)+j;x=x(:,selected);u=u(:,selected);
    branch=branch(selected);c=c(selected);s=s(selected);lateral=lateral(selected);
    stateGradient=zeros(6,count);inputGradient=zeros(2,count);stateGradient(4,:)=-1;
    active=branch==2;low=active & blend(selected)<=-front(selected);
    stateGradient(4,low)=1;wheel=active & ~low;
    stateGradient(4,wheel)=-c(wheel);stateGradient(5,wheel)=-s(wheel);
    stateGradient(6,wheel)=-cfg.vehicle.lf*s(wheel);
    inputGradient(1,wheel)=x(4,wheel).*s(wheel)-lateral(wheel).*c(wheel);
end

function ready=localWarmIpopt(cfg,reference)
    % A fixed synthetic encounter exercises the CasADi/IPOPT backend
    % and trajectory derivatives before the experiment clock starts.
    % This discarded offline solve uses the solver iteration limits. An
    % online wall-clock deadline can interrupt first-time toolbox loading.
    item=struct('ego',[0;0;0;cfg.referenceSpeed;0;0],'parameters',ones(4,1),'sensorBias',zeros(7,1));
    item.target=[cfg.collision.encounterRangeMeters/2;cfg.vehicle.width+cfg.target.defaultWidth;0;0;0;0;cfg.target.rearAxleDistance; ...
        cfg.target.defaultLength/2;cfg.target.defaultWidth/2;0;0];
    input=zeros(2,1);
    sample=solvePredictiveControl("outputFeedback",struct('operation',"sensor", ...
        'cfg',cfg,'hypothesis',item,'input',input,'time',0));
    block=struct('kind',"jointSensorBlock-v1",'time',0,'heldInput',input,'samples',sample,'initialHypotheses',item);
    % The probe has no access to the experiment state or controller memory.
    request=struct('operation',"prepareStep",'cfg',cfg,'reference',reference, ...
        'block',block,'previousState',[]);
    result=solvePredictiveControl("outputFeedback",request);
    info=result.prediction.metadata;
    ready=info.solver=="casadiIpoptCompactPcbf" && info.nlpAttempts>0;
end
