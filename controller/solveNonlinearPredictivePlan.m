function [incumbent,search] = solveNonlinearPredictivePlan(model,previousState)
%solveNonlinearPredictivePlan Improve witnesses inside validated control boxes.
% Directed interval propagation certifies every policy in each convex box.
% A point validation and lexicographic comparison precede incumbent replacement.
    cfg=model.cfg;timer=tic;incumbent=[];
    search=struct('certifiedCandidates',0,'rejectedCandidates',0,'solverCalls',0, ...
        'source',"",'proposalFailures',strings(0,1),'lastRejection',"",'shiftAttempted',false,'attempts',{{}});
    if localCanInherit(previousState,model)
        % The stored policy is available before any new integration or solve.
        % Its proof is inherited under the explicitly unchanged exact plant.
        search.shiftAttempted=true;
        incumbent=localInherit(previousState,model);
        if ~isempty(incumbent)
            search.source="inheritedCertifiedPolicy";
            search.certifiedCandidates=1;
            if localExpired(timer,cfg),search.elapsedSeconds=toc(timer);return;end
        end
        if size(previousState.plan,2)>1
            tail=previousState.predictedState(:,end);
            error=nonlinearBicycleModel.error(tail,model.lane,model.backup.reference);
            u=model.backup.reference.input+model.backup.reference.gain*error;
            plan=[previousState.plan(:,2:end),u];
            plan(:,1)=nonlinearSafetyMex('feedbackInput',model.initialState, ...
                [previousState.certificate.referenceStates(:,2);previousState.plan(:,2)], ...
                previousState.certificate.feedbackGains(:,:,2));
            [incumbent,search]=localAdmit(plan,"shiftedCertifiedPlan",incumbent,search,model);
        end
    end
    if ~isempty(cfg.nonlinear.initialPlan)
        [incumbent,search]=localAdmit(cfg.nonlinear.initialPlan,"suppliedInitialPlan",incumbent,search,model);
    end
    % Explicitly retain a safe zero-slack nominal anchor before local QP work.
    for side=[0,1,-1,2,-2]
        if side~=0 && (~isempty(incumbent) || isempty(model.target)),break;end
        try
            plan=localSeed(model,side);
            [incumbent,search]=localAdmit(plan,"nominalContinuation",incumbent,search,model);
        catch exception
            if ~startsWith(exception.identifier,'collisionAvoidanceController:'),rethrow(exception);end
            search.lastRejection=string(exception.message);
        end
    end
    if isempty(incumbent),return;end
    if incumbent.terminalOnly,search.elapsedSeconds=toc(timer);return;end
    for iteration=1:cfg.nonlinear.maximumImprovementIterations
        if toc(timer)>=cfg.solver.certificateSearchTimeLimit,break;end
        try
            if isempty(cfg.nonlinear.proposalFunction)
                [plan,calls]=localConvexProposal(incumbent,model,timer);search.solverCalls=search.solverCalls+calls;
            else
                search.solverCalls=search.solverCalls+1;
                plan=cfg.nonlinear.proposalFunction(incumbent,model);
            end
            if isempty(plan) || toc(timer)>=min(cfg.solver.certificateSearchTimeLimit,cfg.solver.frameDeadlineSeconds),break;end
            [incumbent,search]=localAdmit(plan,"validatedConvexProposal",incumbent,search,model);
        catch exception
            search.proposalFailures(end+1,1)=string(exception.identifier)+": "+string(exception.message);
            break;
        end
    end
    search.elapsedSeconds=toc(timer);
end

function [incumbent,search]=localAdmit(plan,source,incumbent,search,model)
    policy=[];
    if isstruct(plan),policy=plan.policy;plan=plan.inputs;end
    if size(plan,2)<model.cfg.controller.horizonSteps || size(plan,2)>model.cfg.controller.maximumHorizonSteps
        search.rejectedCandidates=search.rejectedCandidates+1;search.lastRejection="horizon";return;
    end
    target=model.target;targetStep=0;
    if isfield(model,'targetAnchor'),target=model.targetAnchor;targetStep=model.targetStep;end
    candidate=nonlinearSafetyCertificate.plan(model.initialState,plan,model.previousInput, ...
        target,model.lane,model.frame,model.backup,model.cfg,targetStep,policy);
    search.attempts{end+1}=struct('source',source,'accepted',candidate.accepted,'reason',candidate.reason, ...
        'horizonSteps',size(plan,2),'checkedStage',candidate.checkedStage,'checkedTime',candidate.checkedTime, ...
        'collisionMargin',candidate.minimumCollisionMargin,'roadMargin',candidate.minimumRoadMargin, ...
        'checkedBox',candidate.checkedBox,'terminal',candidate.terminal);
    if ~candidate.accepted
        search.rejectedCandidates=search.rejectedCandidates+1;search.lastRejection=candidate.reason;return;
    end
    search.certifiedCandidates=search.certifiedCandidates+1;
    candidate.nominalCost=localCost(candidate.states,plan,model);
    if isempty(incumbent) || candidate.clfSlack<incumbent.clfSlack ...
            || (candidate.clfSlack==incumbent.clfSlack && candidate.nominalCost<incumbent.nominalCost)
        incumbent=candidate;search.source=source;
    end
end

function plan=localSeed(model,side)
    cfg=model.cfg;reference=model.backup.reference;x=model.initialState;previous=model.previousInput;
    plan=zeros(2,cfg.controller.maximumHorizonSteps);h=cfg.controller.sampleTime;
    duration=0;offset=0;holdUntil=0;
    if side~=0
        target=model.target;projection=laneGeometry.project(x(1:2),model.lane);
        tangent=[cos(projection.heading);sin(projection.heading)];
        relativeSpeed=cfg.referenceSpeed-target(4)*cos(target(3)+target(5)-projection.heading);
        encounterTime=max(.5,tangent.'*(target(1:2)-x(1:2))/max(1,relativeSpeed));
        holdUntil=encounterTime+max(.5,(cfg.vehicle.length+2*target(7))/max(1,relativeSpeed));
        duration=holdUntil+2;
        available=min(model.frame(5:6))-cfg.vehicle.width/2-.2;
        requested=cfg.admission.widthScale*(cfg.vehicle.width/2+target(8) ...
            +cfg.collision.safetyMarginMeters+cfg.admission.clearanceAllowanceMeters);
        offset=sign(side)*min(available,requested);
    end
    for index=1:size(plan,2)
        error=nonlinearBicycleModel.error(x,model.lane,reference);
        time=(index-1)*h;
        if duration>0 && time<duration
            fraction=max(0,(time-holdUntil)/(duration-holdUntil));
            error(1)=abs(side)*(error(1)-offset*cos(pi*fraction/2)^2);
        end
        u=reference.input+reference.gain*error;
        u=localClip(u,previous,cfg);plan(:,index)=u;
        x=nonlinearBicycleModel.sample(x,u,cfg);previous=u;
        error=nonlinearBicycleModel.error(x,model.lane,reference);
        if index>=cfg.controller.horizonSteps && time>=duration ...
                && norm(reference.factor*error)<model.backup.radius*.6 ...
                && all(abs(u-reference.input)<model.backup.inputRadius*.8)
            plan=plan(:,1:index);return;
        end
    end
end

function u=localClip(u,previous,cfg)
    lower=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    upper=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    reserve=min(cfg.nonlinear.initializationInputReserve,.1*(upper-lower));lower=lower+reserve;upper=upper-reserve;
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    u=min(upper,max(lower,min(previous+rate,max(previous-rate,u))));
    % The validated positive-speed model has no derivative at |b|=1.
    u(2)=min(1-1e-8,max(-1+1e-8,u(2)));
end

function cost=localCost(states,plan,model)
    cost=0;reference=model.backup.reference;
    for index=1:size(plan,2)
        error=nonlinearBicycleModel.error(states(:,index+1),model.lane,reference);
        cost=cost+error.'*reference.matrix*error+.01*sum((plan(:,index)-reference.input).^2);
    end
    cost=cost/size(plan,2);
end

function [plan,calls]=localConvexProposal(incumbent,model,timer)
    cfg=model.cfg;anchor=incumbent.inputs;count=numel(anchor);scale=[.15;.25];
    plan=[];calls=0;
    policy=struct('referenceStates',incumbent.referenceStates,'feedbackGains',incumbent.feedbackGains, ...
        'inputRadius',zeros(size(anchor)));
    radius=min(cfg.nonlinear.trustRadius,.01*model.backup.radius/max(scale));
    target=model.target;targetStep=0;
    if isfield(model,'targetAnchor'),target=model.targetAnchor;targetStep=model.targetStep;end
    region=[];
    for attempt=1:8
        if localExpired(timer,cfg),return;end
        policy.inputRadius=repmat(scale*radius,1,size(anchor,2));
        region=nonlinearSafetyCertificate.plan(model.initialState,anchor,model.previousInput, ...
            target,model.lane,model.frame,model.backup,cfg,targetStep,policy);
        if region.accepted,break;end
        radius=radius/2;
    end
    if isempty(region) || ~region.accepted || localExpired(timer,cfg),return;end
    % Every point in this box already has a nonlinear full-interval safety
    % proof. The two extra variables majorize the first control's absolute
    % increments, giving a convex CLF bound from interval flow derivatives.
    a=zeros(4,count+3);b=zeros(4,1);
    a(1,1)=1;a(1,count+1)=-1;a(2,1)=-1;a(2,count+1)=-1;
    a(3,2)=1;a(3,count+2)=-1;a(4,2)=-1;a(4,count+2)=-1;
    if incumbent.clfAvailable
        derivative=nonlinearSafetyMex('clfGradient',region.samples{1},fialaCertificate.parameters(cfg), ...
            model.frame,model.backup.reference.state,model.backup.reference.factor,scale);
        row=zeros(1,count+3);row(1:2)=derivative(:,1);row(count+(1:2))=derivative(:,2);row(end)=-1;
        a=[a;row];b=[b;-incumbent.clfResidualUpper];
    end
    gradient=localCostGradient(anchor,policy,model,timer);
    if isempty(gradient) || localExpired(timer,cfg),return;end
    % Reserve one percent for floating-point command assembly; exact directed
    % box membership is checked after both optimization stages.
    lb=[repmat(-.99*radius,count,1);0;0;0];
    ub=[repmat(.99*radius,count,1);radius;radius;Inf];
    linearOptions=optimoptions('linprog','Display','none','MaxIterations',cfg.solver.maxIterations);
    [first,~,flag]=linprog([zeros(count+2,1);1],a,b,[],[],lb,ub,linearOptions);calls=1;
    if flag<=0 || isempty(first),return;end
    ub(end)=max(0,first(end));
    quadraticOptions=optimoptions('quadprog','Display','off','MaxIterations',cfg.solver.maxIterations, ...
        'ConstraintTolerance',cfg.solver.constraintTolerance,'OptimalityTolerance',cfg.solver.optimalityTolerance);
    [second,~,flag]=quadprog(diag([ones(count,1);0;0;0]), ...
        [repmat(scale,size(anchor,2),1).*gradient;0;0;0],a,b,[],[],lb,ub,[],quadraticOptions);calls=2;
    if flag>0 && ~isempty(second),first=second;end
    proposed=anchor+reshape(repmat(scale,size(anchor,2),1).*first(1:count),2,[]);
    if ~nonlinearSafetyMex('inputBoxContains',proposed,anchor,policy.inputRadius),return;end
    policy.inputRadius=zeros(size(anchor));
    plan=struct('inputs',proposed,'policy',policy);
end

function gradient=localCostGradient(inputs,policy,model,timer)
    cfg=model.cfg;reference=model.backup.reference;count=size(inputs,2);
    a=zeros(6,6,count);b=zeros(6,2,count);costState=zeros(6,count);x=model.initialState;
    gradient=[];
    for index=1:count
        if localExpired(timer,cfg),return;end
        gain=policy.feedbackGains(:,:,index);
        u=inputs(:,index)+gain*(x-policy.referenceStates(:,index));
        [x,transition,inputMap]=nonlinearBicycleModel.sample(x,u,cfg);
        a(:,:,index)=transition+inputMap*gain;b(:,:,index)=inputMap;
        [error,projection]=nonlinearBicycleModel.error(x,model.lane,reference);
        tangent=[cos(projection.heading);sin(projection.heading)];normal=[-tangent(2);tangent(1)];
        e=zeros(5,6);e(1,1:2)=normal.';e(2,3)=1;
        e(2,1:2)=-model.frame(4)*tangent.'/(1-model.frame(4)*projection.lateralPosition);
        e(3:5,4:6)=eye(3);costState(:,index)=2*e.'*reference.matrix*error/count;
    end
    gradient=zeros(2,count);adjoint=zeros(6,1);
    for index=count:-1:1
        adjoint=adjoint+costState(:,index);
        gradient(:,index)=b(:,:,index).'*adjoint+.02*(inputs(:,index)-reference.input)/count;
        adjoint=a(:,:,index).'*adjoint;
    end
    gradient=gradient(:);
end

function expired=localExpired(timer,cfg)
    expired=toc(timer)>=min(cfg.solver.certificateSearchTimeLimit,cfg.solver.frameDeadlineSeconds);
end

function valid=localCanInherit(prior,model)
    valid=isstruct(prior) && isfield(prior,'version') && prior.version==47 ...
        && isequaln(prior.identity,model.identity) && ~isempty(prior.plan) ...
        && isequal(prior.appliedInput,model.previousInput) ...
        && isfinite(model.stateTime) && isfinite(prior.stateTime) ...
        && abs(model.stateTime-prior.stateTime-model.cfg.controller.sampleTime) ...
            <=model.cfg.nonlinear.targetConsistencyTolerance;
    if ~valid,return;end
    box=prior.certificate.samples{1}.endpoint(1:6,:);
    valid=all(model.initialState>=box(:,1) & model.initialState<=box(:,2));
    % Box containment detects obvious contract violations. It is not evidence
    % that a disturbed state is an exact successor of the executed policy.
end

function certificate=localInherit(prior,model)
    certificate=prior.certificate;cfg=model.cfg;x=model.initialState;
    if size(prior.plan,2)>1
        certificate.inputs=prior.plan(:,2:end);
        certificate.referenceStates=certificate.referenceStates(:,2:end);
        certificate.feedbackGains=certificate.feedbackGains(:,:,2:end);
        certificate.inputs(:,1)=nonlinearSafetyMex('feedbackInput',x, ...
            [certificate.referenceStates(:,1);certificate.inputs(:,1)],certificate.feedbackGains(:,:,1));
        certificate.feedbackGains(:,:,1)=0;certificate.referenceStates(:,1)=x;
        certificate.samples=certificate.samples(2:end);
        certificate.states=[x,certificate.states(:,3:end)];
    else
        shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
        [admitted,terminal]=nonlinearSafetyCertificate.terminal([x,x],model.previousInput,model.targetAnchor, ...
            [model.targetStep;cfg.controller.sampleTime],model.backup,model.frame,shape,cfg);
        if ~admitted,certificate=[];return;end
        reference=model.backup.reference;
        input=nonlinearSafetyMex('backupInput',x,model.frame,[reference.state;reference.input],reference.gain);
        world=nonlinearSafetyMex('backupWorld',x,model.backup.domain,model.frame);
        sample=struct('accepted',true,'endpoint',world,'cells',struct('start',0, ...
            'end',cfg.controller.sampleTime,'swept',world,'domain',world), ...
            'scope',"invariantFrenetTemplateMappedToWorld");
        certificate.inputs=input;certificate.samples={sample};certificate.referenceStates=x;
        certificate.feedbackGains=zeros(2,6);certificate.states=[x,nonlinearBicycleModel.sample(x,input,cfg)];
        certificate.terminal=terminal;certificate.terminalOnly=true;
    end
    certificate.inputRadius=zeros(size(certificate.inputs));certificate.inherited=true;
    certificate.reason="inheritedUnderExactSuccessorContract";
    certificate.clfAvailable=false;certificate.clfSlack=0;certificate.clfResidualUpper=0;
    certificate.clfInitialValue=0;certificate.clfNextValueUpper=0;
    try
        before=nonlinearSafetyMex('error',[x,x],model.frame,model.backup.reference.state);
        after=nonlinearSafetyMex('error',certificate.samples{1}.endpoint(1:6,:),model.frame,model.backup.reference.state);
        bound=nonlinearSafetyMex('clf',before,after,model.backup.reference.factor,cfg.nonlinear.clfDecay);
        certificate.clfInitialValue=bound(1);certificate.clfNextValueUpper=bound(2);
        certificate.clfSlack=bound(3);certificate.clfResidualUpper=bound(4);certificate.clfAvailable=true;
    catch exception
        if ~startsWith(exception.identifier,'collisionAvoidanceController:'),rethrow(exception);end
    end
    certificate.nominalCost=localCost(certificate.states,certificate.inputs,model);
end
