function [incumbent,search] = solveNonlinearPredictivePlan(model,previousState)
%solveNonlinearPredictivePlan Huang safety priority with Li dual SCA steps.
% Infeasible nominal iterates are search data. Only a full nonlinear witness
% ending in the invariant joint-state terminal set can authorize an input.
    cfg=model.cfg;timer=tic;incumbent=[];
    search=struct('certifiedCandidates',0,'rejectedCandidates',0,'solverCalls',0, ...
        'source',"",'proposalFailures',strings(0,1),'lastRejection',"",'shiftAttempted',false,'attempts',{{}}, ...
        'sequentialIterations',{{}},'initialization',"laneFeedbackRollout", ...
        'safetyObjective',"sumOfStageSafetySlacks",'bestNominalSafetySlack',Inf);
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
    % A lane-feedback rollout supplies a numerical center, without a planned
    % passing maneuver. Safety restoration can start with an intersecting path.
    try
        anchor=localSeed(model);
        [incumbent,search]=localAdmit(anchor,"nominalContinuation",incumbent,search,model);
    catch exception
        if ~startsWith(exception.identifier,'collisionAvoidanceController:'),rethrow(exception);end
        search.lastRejection=string(exception.message);anchor=[];
    end
    if ~isempty(incumbent) && incumbent.terminalOnly
        search.elapsedSeconds=toc(timer);return;
    end
    if isempty(incumbent)
        limit=cfg.nonlinear.maximumAdmissionIterations;
        if ~isempty(cfg.nonlinear.initialPlan),anchor=cfg.nonlinear.initialPlan;end
        if ~isempty(model.target)
            shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
            distance=nonlinearSafetyCertificate.rectangle(model.initialState(1:3),shape, ...
                model.target(1:3),model.target(7:10));
            if distance<=cfg.collision.safetyMarginMeters
                search.lastRejection="initialCollisionClearance";search.elapsedSeconds=toc(timer);return;
            end
        end
    else
        limit=cfg.nonlinear.maximumImprovementIterations;anchor=incumbent.inputs;
    end
    if ~isempty(anchor) && limit>0
        try
            if isempty(cfg.nonlinear.proposalFunction)
                [incumbent,search]=localSequentialSearch(anchor,incumbent,search,model,timer,limit);
            elseif ~isempty(incumbent)
                search.solverCalls=search.solverCalls+1;
                plan=cfg.nonlinear.proposalFunction(incumbent,model);
                [incumbent,search]=localAdmit(plan,"validatedCustomProposal",incumbent,search,model);
            end
        catch exception
            search.proposalFailures(end+1,1)=string(exception.identifier)+": "+string(exception.message);
            search.lastRejection=string(exception.message);
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
    if ~candidate.accepted && candidate.reason=="terminal" && isempty(policy) ...
            && ~isempty(candidate.terminal) && candidate.terminal.tailMargin>=0 ...
            && size(plan,2)<model.cfg.controller.maximumHorizonSteps
        % The nonlinear enclosure can need more settling than the numerical
        % endpoint. Extend the same lane-feedback continuation and revalidate
        % its complete prefix; this is still only a candidate until admission.
        cfg=model.cfg;reference=model.backup.reference;
        nominal=nonlinearBicycleModel.rollout(model.initialState,plan,cfg);x=nominal(:,end);
        extra=min(ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime), ...
            cfg.controller.maximumHorizonSteps-size(plan,2));
        for index=1:extra
            error=nonlinearBicycleModel.error(x,model.lane,reference);
            u=localClip(reference.input+reference.gain*error,plan(:,end),cfg);
            plan(:,end+1)=u; %#ok<AGROW>
            x=nonlinearBicycleModel.sample(x,u,cfg);
        end
        candidate=nonlinearSafetyCertificate.plan(model.initialState,plan,model.previousInput, ...
            target,model.lane,model.frame,model.backup,cfg,targetStep);
    end
    search.attempts{end+1}=struct('source',source,'accepted',candidate.accepted,'reason',candidate.reason, ...
        'horizonSteps',size(plan,2),'checkedStage',candidate.checkedStage,'checkedTime',candidate.checkedTime, ...
        'collisionMargin',candidate.minimumCollisionMargin,'roadMargin',candidate.minimumRoadMargin, ...
        'checkedBox',candidate.checkedBox,'terminal',candidate.terminal);
    if ~candidate.accepted
        search.rejectedCandidates=search.rejectedCandidates+1;search.lastRejection=candidate.reason;return;
    end
    search.certifiedCandidates=search.certifiedCandidates+1;
    candidate.nominalCost=localCost(candidate.states,plan,model);
    candidate.performanceCost=candidate.nominalCost+model.cfg.clf.relaxationWeight*candidate.clfSlack^2;
    if isempty(incumbent) || candidate.performanceCost<incumbent.performanceCost
        incumbent=candidate;search.source=source;
    end
end

function plan=localSeed(model)
    cfg=model.cfg;reference=model.backup.reference;x=model.initialState;previous=model.previousInput;
    required=cfg.controller.horizonSteps;
    if ~isempty(model.target) && localTailMargin(x,0,model)<0
        projection=laneGeometry.project(x(1:2),model.lane);
        tangent=[cos(projection.heading);sin(projection.heading)];q=model.target;
        relativeSpeed=cfg.referenceSpeed-q(4)*cos(q(3)+q(5)-projection.heading);
        encounterTime=max(0,tangent.'*(q(1:2)-x(1:2))/max(1,relativeSpeed));
        required=max(required,ceil((encounterTime+cfg.nonlinear.recoveryHorizonSeconds)/cfg.controller.sampleTime));
        required=min(required,cfg.controller.maximumHorizonSteps);
    end
    plan=zeros(2,cfg.controller.maximumHorizonSteps);
    for index=1:size(plan,2)
        error=nonlinearBicycleModel.error(x,model.lane,reference);
        u=localClip(reference.input+reference.gain*error,previous,cfg);plan(:,index)=u;
        x=nonlinearBicycleModel.sample(x,u,cfg);previous=u;
        error=nonlinearBicycleModel.error(x,model.lane,reference);
        if index>=required && norm(reference.factor*error)<model.backup.radius*.6 ...
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

function [incumbent,search]=localSequentialSearch(anchor,incumbent,search,model,timer,limit)
    cfg=model.cfg;radius=cfg.nonlinear.trustRadius;admission=isempty(incumbent);
    baseline=localNominalEvaluation(anchor,model);
    search.bestNominalSafetySlack=baseline.safety;
    for iteration=1:limit
        if toc(timer)>=cfg.solver.frameDeadlineSeconds || (~admission && localExpired(timer,cfg)),break;end
        [plan,step]=localSequentialStep(anchor,model,radius);
        search.solverCalls=search.solverCalls+step.calls;
        step.iteration=iteration;step.trustRadius=radius;step.acceptedIterate=false;
        step.nominalSafetySlack=Inf;step.nominalHardViolation=Inf;
        if isempty(plan)
            radius=radius/2;search.sequentialIterations{end+1}=step;
            if radius<1e-4,break;end
            continue;
        end
        try
            candidate=localNominalEvaluation(plan,model);
        catch exception
            if ~startsWith(exception.identifier,'collisionAvoidanceController:'),rethrow(exception);end
            radius=radius/2;search.sequentialIterations{end+1}=step;continue;
        end
        step.nominalSafetySlack=candidate.safety;step.nominalHardViolation=candidate.hard;
        merit=baseline.safety+10*baseline.hard;
        nextMerit=candidate.safety+10*candidate.hard;
        improves=nextMerit<merit-1e-8 || (nextMerit<=1e-6 && candidate.cost<=baseline.cost+1e-10);
        if improves
            anchor=plan;baseline=candidate;step.acceptedIterate=true;
            search.bestNominalSafetySlack=min(search.bestNominalSafetySlack,candidate.safety);
            radius=min(2,1.5*radius);
        else
            radius=radius/2;
        end
        search.sequentialIterations{end+1}=step;
        if candidate.safety<=.5*cfg.nonlinear.optimizationClearanceMeters && candidate.hard<=1e-5
            before=search.certifiedCandidates;
            [incumbent,search]=localAdmit(plan,"sequentialConvexification",incumbent,search,model);
            if admission && search.certifiedCandidates>before,break;end
        end
        if radius<1e-4,break;end
    end
end

function [plan,info]=localSequentialStep(anchor,model,radius)
    cfg=model.cfg;count=size(anchor,2);reference=model.backup.reference;
    % Variables: ego state increments, input increments, stage safety slacks,
    % one CLF slack and five terminal absolute-value epigraphs. The target's
    % autonomous joint-state block is eliminated by its exact analytic flow.
    ix=reshape(1:6*(count+1),6,[]);iu=reshape(ix(end)+(1:2*count),2,[]);
    is=iu(end)+(1:count);ic=is(end)+1;it=ic+(1:5);nv=it(end);
    equal=sparse(6*(count+1),nv);rhs=zeros(6*(count+1),1);equal(1:6,ix(:,1))=eye(6);
    rows=cell(1,0);bounds=cell(1,0);hessian=sparse(nv,nv);linear=zeros(nv,1);
    lower=-Inf(nv,1);upper=Inf(nv,1);
    stateTrust=radius*[5;5;.5;5;3;1.5];inputTrust=radius*[.15;.25];
    lower(ix(:))=-repmat(stateTrust,count+1,1);upper(ix(:))=-lower(ix(:));
    lower(iu(:))=-repmat(inputTrust,count,1);upper(iu(:))=-lower(iu(:));
    lower([is,ic,it])=0;
    inputLower=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum]+cfg.nonlinear.initializationInputReserve;
    inputUpper=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum]-cfg.nonlinear.initializationInputReserve;
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    x=model.initialState;initialError=nonlinearBicycleModel.error(x,model.lane,reference);
    halfCfg=cfg;halfCfg.controller.sampleTime=cfg.controller.sampleTime/2;
    for index=1:count
        % Integrated variational dynamics are tangents to the nonlinear held
        % flow, including the affine defect (zero for this fresh rollout).
        [middle,am,bm]=nonlinearBicycleModel.sample(x,anchor(:,index),halfCfg);
        [next,an,bn]=nonlinearBicycleModel.sample(middle,anchor(:,index),halfCfg);
        a=an*am;b=an*bm+bn;eq=6*index+(1:6);
        equal(eq,ix(:,index+1))=eye(6);equal(eq,ix(:,index))=-a;equal(eq,iu(:,index))=-b;
        lower(iu(:,index))=max(lower(iu(:,index)),inputLower-anchor(:,index));
        upper(iu(:,index))=min(upper(iu(:,index)),inputUpper-anchor(:,index));
        r=sparse(2,nv);r(:,iu(:,index))=eye(2);previous=model.previousInput;
        if index>1,r(:,iu(:,index-1))=-eye(2);previous=anchor(:,index-1);end
        finite=isfinite(rate);difference=anchor(:,index)-previous;
        rows{end+1}=[r(finite,:);-r(finite,:)];bounds{end+1}=[rate(finite)-difference(finite);rate(finite)+difference(finite)];
        map=sparse(6,nv);map(:,ix(:,index))=eye(6);
        [g,j]=localSafetyRows(x,(index-1)*cfg.controller.sampleTime,model);
        r=-j*map;r(:,is(index))=-1;rows{end+1}=r;bounds{end+1}=g;
        map(:,ix(:,index))=am;map(:,iu(:,index))=bm;
        [g,j]=localSafetyRows(middle,(index-.5)*cfg.controller.sampleTime,model);
        r=-j*map;r(:,is(index))=-1;rows{end+1}=r;bounds{end+1}=g;
        [e,j]=nonlinearBicycleModel.errorLinearization(next,model.lane,reference);
        jx=ix(:,index+1);hessian(jx,jx)=hessian(jx,jx)+2*j.'*reference.matrix*j/count;
        linear(jx)=linear(jx)+2*j.'*reference.matrix*e/count;
        hessian(iu(:,index),iu(:,index))=.02*eye(2)/count;
        linear(iu(:,index))=.02*(anchor(:,index)-reference.input)/count;
        physicalLower=[max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor+1e-4); ...
            -cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
        physicalUpper=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
        lower(jx(4:6))=max(lower(jx(4:6)),physicalLower-next(4:6));
        upper(jx(4:6))=min(upper(jx(4:6)),physicalUpper-next(4:6));
        if index==1
            r=sparse(1,nv);r(jx)=2*e.'*reference.matrix*j;r(ic)=-1;
            rows{end+1}=r;bounds{end+1}=(1-cfg.nonlinear.clfDecay)*norm(reference.factor*initialError)^2-norm(reference.factor*e)^2;
        end
        x=next;
    end
    % A polyhedral subset of the terminal ellipsoid is hard in every step.
    % Independent full-flow admission checks the actual nonlinear endpoint,
    % invariant target separation, and previous-input coordinates afterwards.
    r=sparse(5,nv);r(:,ix(:,end))=reference.factor*j;
    epigraph=sparse(5,nv);epigraph(:,it)=eye(5);
    rows{end+1}=[r-epigraph;-r-epigraph];bounds{end+1}=[-reference.factor*e;reference.factor*e];
    r=sparse(1,nv);r(it)=1;rows{end+1}=r;bounds{end+1}=.7*model.backup.radius;
    lower(iu(:,end))=max(lower(iu(:,end)),reference.input-.8*model.backup.inputRadius-anchor(:,end));
    upper(iu(:,end))=min(upper(iu(:,end)),reference.input+.8*model.backup.inputRadius-anchor(:,end));
    [tail,gradient]=localTailLinearization(x,count*cfg.controller.sampleTime,model);
    r=sparse(1,nv);r(ix(:,end))=-gradient;rows{end+1}=r;bounds{end+1}=tail-cfg.nonlinear.clearanceReserve;
    [g,j]=localSafetyRows(x,count*cfg.controller.sampleTime,model);
    r=sparse(size(j,1),nv);r(:,ix(:,end))=-j;rows{end+1}=r;bounds{end+1}=g;
    hessian(ic,ic)=2*cfg.clf.relaxationWeight;
    % A small proximal term resolves unpenalized station/slack directions.
    hessian=hessian+1e-8*speye(nv);hessian=(hessian+hessian.')/2;
    a=vertcat(rows{:});b=vertcat(bounds{:});objective=zeros(nv,1);objective(is)=1;
    info=struct('calls',0,'safetyOptimum',Inf,'secondarySafety',Inf,'safetyCap',Inf, ...
        'status',"infeasibleBounds",'safetyExitFlag',NaN,'secondaryExitFlag',NaN);plan=[];
    if any(lower>upper) || any(~isfinite(b)),return;end
    options=optimoptions('linprog','Algorithm','interior-point','Display','none', ...
        'MaxIterations',cfg.solver.maxIterations,'ConstraintTolerance',cfg.solver.constraintTolerance, ...
        'OptimalityTolerance',cfg.solver.optimalityTolerance);
    [first,~,flag]=linprog(objective,a,b,equal,rhs,lower,upper,options);info.calls=1;
    info.safetyExitFlag=flag;
    if flag<=0 || isempty(first),info.status="safetySolveFailed";return;end
    info.safetyOptimum=sum(max(0,first(is)));
    % This numerical tie tolerance never relaxes executable safety. Only the
    % original nonlinear zero-slack witness is admitted by localAdmit.
    info.safetyCap=info.safetyOptimum+cfg.solver.lexicographicTieTolerance;
    a=[a;objective.'];b=[b;info.safetyCap];
    options=optimoptions('quadprog','Display','off','MaxIterations',cfg.solver.maxIterations, ...
        'ConstraintTolerance',cfg.solver.constraintTolerance,'OptimalityTolerance',cfg.solver.optimalityTolerance);
    [second,~,flag]=quadprog(hessian,linear,a,b,equal,rhs,lower,upper,[],options);info.calls=2;
    info.secondaryExitFlag=flag;
    if flag>0 && ~isempty(second),first=second;end
    if max([a*first-b;abs(equal*first-rhs);lower-first;first-upper])>10*cfg.solver.constraintTolerance
        info.status="numericalResidual";return;
    end
    info.secondarySafety=sum(max(0,first(is)));info.status="solved";
    plan=anchor+reshape(first(iu(:)),2,[]);
end

function [values,jacobian]=localSafetyRows(x,time,model)
    cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    values=zeros(0,1);jacobian=zeros(0,6);
    projection=laneGeometry.project(x(1:2),model.lane);
    preferred=[-sin(projection.heading);cos(projection.heading)];
    q=localTargetAt(model,time);
    if ~isempty(q)
        dual=nonlinearSafetyCertificate.dualLinearization(x(1:3),shape,q(1:3),q(7:10),preferred);
        values=dual.value-cfg.collision.safetyMarginMeters-cfg.nonlinear.optimizationClearanceMeters;
        jacobian=[dual.jacobian,zeros(4,3)];
    end
    rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
    body=shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1];
    for vertex=1:4
        point=x(1:2)+rotation*body(:,vertex);p=laneGeometry.project(point,model.lane);
        normal=[-sin(p.heading),cos(p.heading)];gradient=[normal,normal*rotation*[0,-1;1,0]*body(:,vertex),0,0,0];
        values=[values;p.lateralPosition+model.frame(5);model.frame(6)-p.lateralPosition]; %#ok<AGROW>
        jacobian=[jacobian;gradient;-gradient]; %#ok<AGROW>
    end
end

function evaluation=localNominalEvaluation(plan,model)
    cfg=model.cfg;x=model.initialState;count=size(plan,2);slacks=zeros(1,count);hard=0;
    halfCfg=cfg;halfCfg.controller.sampleTime=cfg.controller.sampleTime/2;
    states=zeros(6,count+1);states(:,1)=x;
    for index=1:count
        values=localSafetyRows(x,(index-1)*cfg.controller.sampleTime,model);
        middle=nonlinearBicycleModel.sample(x,plan(:,index),halfCfg);
        middleValues=localSafetyRows(middle,(index-.5)*cfg.controller.sampleTime,model);
        slacks(index)=max([0;-values;-middleValues]);
        x=nonlinearBicycleModel.sample(middle,plan(:,index),halfCfg);states(:,index+1)=x;
        hard=hard+max([0;cfg.model.speedMinimum-x(4);x(4)-cfg.model.speedMaximum; ...
            abs(x(5))-cfg.model.lateralVelocityMaximum;abs(x(6))-cfg.model.yawRateMaximum]);
    end
    e=nonlinearBicycleModel.error(x,model.lane,model.backup.reference);
    tail=localTailMargin(x,count*cfg.controller.sampleTime,model);
    hard=hard+max(0,norm(model.backup.reference.factor*e)-.8*model.backup.radius) ...
        +max(0,-tail)+max([0;-localSafetyRows(x,count*cfg.controller.sampleTime,model)]) ...
        +max([0;abs(plan(:,end)-model.backup.reference.input)-model.backup.inputRadius]);
    before=nonlinearBicycleModel.error(states(:,1),model.lane,model.backup.reference);
    after=nonlinearBicycleModel.error(states(:,2),model.lane,model.backup.reference);
    clf=max(0,norm(model.backup.reference.factor*after)^2-(1-cfg.nonlinear.clfDecay)*norm(model.backup.reference.factor*before)^2);
    evaluation=struct('safety',sum(slacks),'stageSlacks',slacks,'hard',hard, ...
        'cost',localCost(states,plan,model)+cfg.clf.relaxationWeight*clf^2);
end

function [margin,gradient]=localTailLinearization(x,time,model)
    margin=localTailMargin(x,time,model);gradient=zeros(1,6);step=1e-4;
    for index=1:2
        lo=x;hi=x;lo(index)=lo(index)-step;hi(index)=hi(index)+step;
        gradient(index)=(localTailMargin(hi,time,model)-localTailMargin(lo,time,model))/(2*step);
    end
end

function margin=localTailMargin(x,time,model)
    cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    q=localTargetAt(model,time);
    margin=nonlinearSafetyMex('tail',[x,x],model.backup.domain,shape,q,model.frame, ...
        cfg.collision.safetyMarginMeters+cfg.nonlinear.clearanceReserve,0);
end

function q=localTargetAt(model,time)
    % The target coordinates of the joint prediction are eliminated exactly.
    if isfield(model,'targetAnchor')
        q=nonlinearSafetyCertificate.targetFlow(model.targetAnchor,model.targetStep*model.cfg.controller.sampleTime+time);
    else
        q=nonlinearSafetyCertificate.targetFlow(model.target,time);
    end
end

function expired=localExpired(timer,cfg)
    expired=toc(timer)>=min(cfg.solver.certificateSearchTimeLimit,cfg.solver.frameDeadlineSeconds);
end

function valid=localCanInherit(prior,model)
    valid=isstruct(prior) && isfield(prior,'version') && prior.version==48 ...
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
    certificate.performanceCost=certificate.nominalCost+cfg.clf.relaxationWeight*certificate.clfSlack^2;
end
