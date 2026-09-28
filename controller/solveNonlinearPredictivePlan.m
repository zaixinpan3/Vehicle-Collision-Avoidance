function [solution,search] = solveNonlinearPredictivePlan(model,previousState)
%solveNonlinearPredictivePlan Nominal PCBF safety priority and lane CLF SCvx.
% Rollout residuals and trust-region updates are part of the numerical solve.
% There is no separate execution verifier or formal certificate pipeline.
    cfg=model.cfg;timer=tic;solution=[];
    search=struct('solverCalls',0,'source',"",'lastRejection',"", ...
        'shiftedPlanAvailable',false,'sequentialIterations',{{}}, ...
        'initialization',"laneFeedbackRollout",'bestSafetySlack',Inf, ...
        'converged',false,'failures',strings(0,1));
    if isstruct(previousState) && isfield(previousState,'version') && previousState.version==49 ...
            && previousState.sampleTime==cfg.controller.sampleTime ...
            && size(previousState.plan,2)>=cfg.controller.horizonSteps ...
            && size(previousState.plan,2)<=cfg.controller.maximumHorizonSteps
        % Shift controls, append the same lane terminal feedback, and start
        % this prediction from the current measured joint state.
        plan=previousState.plan(:,2:end);
        e=nonlinearBicycleModel.error(previousState.predictedState(:,end),model.lane,model.terminal.reference);
        u=model.terminal.reference.input+model.terminal.reference.gain*e;
        if isempty(plan),last=model.previousInput;else,last=plan(:,end);end
        anchor=[plan,localClip(u,last,cfg)];
        candidate=localNominalEvaluation(anchor,model);
        if candidate.hard<=cfg.solver.feasibilityTolerance
            solution=candidate;search.source="shiftedPlan";search.shiftedPlanAvailable=true;
        end
        search.initialization="shiftedPlan";
    else
        anchor=localSeed(model);
    end
    if search.initialization=="shiftedPlan",baseline=candidate;
    else,baseline=localNominalEvaluation(anchor,model);end
    if isempty(solution) && baseline.hard<=cfg.solver.feasibilityTolerance
        solution=baseline;search.source="laneFeedbackRollout";
    end
    limit=cfg.nonlinear.maximumIterations;
    if ~isempty(solution) && solution.safety<=cfg.solver.feasibilityTolerance
        limit=cfg.nonlinear.maximumImprovementIterations;
    end
    radius=cfg.nonlinear.trustRadius;
    for iteration=1:limit
        if toc(timer)>=min(cfg.solver.timeLimitSeconds,cfg.solver.frameDeadlineSeconds),break;end
        if ~isempty(solution) && solution.safety<=cfg.solver.feasibilityTolerance ...
                && solution.cost<=cfg.solver.optimalityTolerance,break;end
        try
            [plan,step]=localSequentialStep(anchor,model,radius);
            search.solverCalls=search.solverCalls+step.calls;
            step.iteration=iteration;step.trustRadius=radius;step.acceptedIterate=false;
            step.nominalSafetySlack=Inf;step.nominalHardViolation=Inf;
            if isempty(plan)
                search.lastRejection=step.status;radius=radius/2;
                search.sequentialIterations{end+1}=step;
                if radius<1e-4,break;end
                continue;
            end
            candidate=localNominalEvaluation(plan,model);
            step.nominalSafetySlack=candidate.safety;step.nominalHardViolation=candidate.hard;
            merit=baseline.safety+10*baseline.hard;
            nextMerit=candidate.safety+10*candidate.hard;
            improves=nextMerit<merit-1e-8 || ...
                (nextMerit<=cfg.solver.feasibilityTolerance && candidate.cost<=baseline.cost+1e-10);
            if improves
                anchor=plan;baseline=candidate;step.acceptedIterate=true;radius=min(2,1.5*radius);
            else
                radius=radius/2;
            end
            if candidate.hard<=cfg.solver.feasibilityTolerance && localBetter(candidate,solution,cfg)
                solution=candidate;search.source="sequentialConvexification";
            end
            search.sequentialIterations{end+1}=step;
            if ~isempty(solution) && solution.safety<=cfg.solver.feasibilityTolerance
                search.converged=true;
                % A zero-slack plan attains the global safety lower bound.
                % Its secondary CLF step has already been solved in the QP.
                break;
            end
            if radius<1e-4,break;end
        catch exception
            search.failures(end+1,1)=string(exception.identifier)+": "+string(exception.message);
            search.lastRejection=string(exception.message);break;
        end
    end
    if ~isempty(solution)
        search.lastRejection="";search.bestSafetySlack=solution.safety;
        search.converged=search.converged || solution.safety<=cfg.solver.feasibilityTolerance;
    elseif strlength(search.lastRejection)==0
        search.lastRejection="hardConstraintResidual";
    end
    search.elapsedSeconds=toc(timer);
end

function better=localBetter(candidate,current,cfg)
    if isempty(current),better=true;return;end
    tolerance=cfg.solver.feasibilityTolerance;
    better=candidate.safety<current.safety-tolerance || ...
        (candidate.safety<=max(current.safety,tolerance) && candidate.cost<current.cost);
end

function plan=localSeed(model)
    cfg=model.cfg;reference=model.terminal.reference;x=model.initialState;previous=model.previousInput;
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
        if index>=required && norm(reference.factor*error)<model.terminal.radius*.6 ...
                && localTailMargin(x,index*cfg.controller.sampleTime,model)>=0
            plan=plan(:,1:index);return;
        end
    end
end

function u=localClip(u,previous,cfg)
    lower=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    upper=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    u=min(upper,max(lower,min(previous+rate,max(previous-rate,u))));
    % The combined-slip force tangent is singular at |b|=1.
    u(2)=min(1-1e-8,max(-1+1e-8,u(2)));
end

function cost=localCost(states,plan,model)
    cost=0;reference=model.terminal.reference;
    for index=1:size(plan,2)
        error=nonlinearBicycleModel.error(states(:,index+1),model.lane,reference);
        cost=cost+error.'*reference.matrix*error+.01*sum((plan(:,index)-reference.input).^2);
    end
    cost=cost/size(plan,2);
end

function [plan,info]=localSequentialStep(anchor,model,radius)
    cfg=model.cfg;count=size(anchor,2);reference=model.terminal.reference;
    % Variables: ego state increments, input increments, stage safety slacks,
    % one CLF slack and five terminal absolute-value epigraphs. The target's
    % autonomous joint-state block is eliminated by its exact analytic flow.
    ix=reshape(1:6*(count+1),6,[]);iu=reshape(ix(end)+(1:2*count),2,[]);
    is=iu(end)+(1:count);ic=is(end)+1;it=ic+(1:5);nv=it(end);
    equal=sparse(6*(count+1),nv);rhs=zeros(6*(count+1),1);equal(1:6,ix(:,1))=eye(6);
    rows=cell(1,3*count+5);bounds=cell(1,3*count+5);rowCount=0;hessian=sparse(nv,nv);linear=zeros(nv,1);
    lower=-Inf(nv,1);upper=Inf(nv,1);
    stateTrust=radius*[5;5;.5;5;3;1.5];inputTrust=radius*[.15;.25];
    lower(ix(:))=-repmat(stateTrust,count+1,1);upper(ix(:))=-lower(ix(:));
    lower(iu(:))=-repmat(inputTrust,count,1);upper(iu(:))=-lower(iu(:));
    lower([is,ic,it])=0;
    inputLower=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    inputUpper=[cfg.model.frontWheelSteeringAngleMaximum;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    inputLower(2)=max(-1+1e-8,inputLower(2));
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    x=model.initialState;initialError=nonlinearBicycleModel.error(x,model.lane,reference);
    halfDuration=cfg.controller.sampleTime/2;
    for index=1:count
        % Integrated variational dynamics are tangents to the nonlinear held
        % flow, including the affine defect (zero for this fresh rollout).
        [middle,am,bm]=nonlinearBicycleModel.sample(x,anchor(:,index),cfg,[],halfDuration);
        [next,an,bn]=nonlinearBicycleModel.sample(middle,anchor(:,index),cfg,[],halfDuration);
        a=an*am;b=an*bm+bn;eq=6*index+(1:6);
        equal(eq,ix(:,index+1))=eye(6);equal(eq,ix(:,index))=-a;equal(eq,iu(:,index))=-b;
        lower(iu(:,index))=max(lower(iu(:,index)),inputLower-anchor(:,index));
        upper(iu(:,index))=min(upper(iu(:,index)),inputUpper-anchor(:,index));
        r=sparse(2,nv);r(:,iu(:,index))=eye(2);previous=model.previousInput;
        if index>1,r(:,iu(:,index-1))=-eye(2);previous=anchor(:,index-1);end
        finite=isfinite(rate);difference=anchor(:,index)-previous;
        rowCount=rowCount+1;rows{rowCount}=[r(finite,:);-r(finite,:)];bounds{rowCount}=[rate(finite)-difference(finite);rate(finite)+difference(finite)];
        map=sparse(6,nv);map(:,ix(:,index))=eye(6);
        [g,j]=localSafetyRows(x,(index-1)*cfg.controller.sampleTime,model);
        r=-j*map;r(:,is(index))=-1;rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
        map(:,ix(:,index))=am;map(:,iu(:,index))=bm;
        [g,j]=localSafetyRows(middle,(index-.5)*cfg.controller.sampleTime,model);
        r=-j*map;r(:,is(index))=-1;rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
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
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=(1-cfg.nonlinear.clfDecay)*norm(reference.factor*initialError)^2-norm(reference.factor*e)^2;
        end
        x=next;
    end
    % A polyhedral subset of the terminal ellipsoid is hard in every step.
    % The SCvx merit uses the corresponding nominal terminal residual.
    r=sparse(5,nv);r(:,ix(:,end))=reference.factor*j;
    epigraph=sparse(5,nv);epigraph(:,it)=eye(5);
    rowCount=rowCount+1;rows{rowCount}=[r-epigraph;-r-epigraph];bounds{rowCount}=[-reference.factor*e;reference.factor*e];
    r=sparse(1,nv);r(it)=1;rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=model.terminal.radius;
    lower(iu(:,end))=max(lower(iu(:,end)),reference.input-(rate-model.terminal.inputRadius)-anchor(:,end));
    upper(iu(:,end))=min(upper(iu(:,end)),reference.input+(rate-model.terminal.inputRadius)-anchor(:,end));
    [tail,gradient]=localTailLinearization(x,count*cfg.controller.sampleTime,model);
    r=sparse(1,nv);r(ix(:,end))=-gradient;rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=tail;
    [g,j]=localSafetyRows(x,count*cfg.controller.sampleTime,model);
    r=sparse(size(j,1),nv);r(:,ix(:,end))=-j;rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
    hessian(ic,ic)=2*cfg.clf.relaxationWeight;
    % A small proximal term resolves unpenalized station/slack directions.
    hessian=hessian+1e-8*speye(nv);hessian=(hessian+hessian.')/2;
    a=vertcat(rows{1:rowCount});b=vertcat(bounds{1:rowCount});objective=zeros(nv,1);objective(is)=1;
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
    % The secondary objective cannot trade away the primary safety optimum
    % beyond the declared numerical tie tolerance.
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
        dual=predictiveSafetyGeometry.dualLinearization(x(1:3),shape,q(1:3),q(7:10),preferred);
        values=dual.value-cfg.collision.safetyMarginMeters;
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
    halfDuration=cfg.controller.sampleTime/2;
    states=zeros(6,count+1);states(:,1)=x;collision=Inf;road=Inf;
    for index=1:count
        values=localSafetyRows(x,(index-1)*cfg.controller.sampleTime,model);
        middle=nonlinearBicycleModel.sample(x,plan(:,index),cfg,[],halfDuration);
        middleValues=localSafetyRows(middle,(index-.5)*cfg.controller.sampleTime,model);
        slacks(index)=max([0;-values;-middleValues]);
        if ~isempty(model.target)
            collision=min([collision;values(1:4);middleValues(1:4)]);
            road=min([road;values(5:end);middleValues(5:end)]);
        else
            road=min([road;values;middleValues]);
        end
        x=nonlinearBicycleModel.sample(middle,plan(:,index),cfg,[],halfDuration);states(:,index+1)=x;
        hard=max([hard;cfg.model.scheduleSpeedFloor+1e-4-x(4);cfg.model.speedMinimum-x(4); ...
            x(4)-cfg.model.speedMaximum;abs(x(5))-cfg.model.lateralVelocityMaximum;abs(x(6))-cfg.model.yawRateMaximum]);
    end
    reference=model.terminal.reference;e=nonlinearBicycleModel.error(x,model.lane,reference);
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    lo=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    hi=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    hard=max([hard;norm(reference.factor*e)-model.terminal.radius; ...
        -localTailMargin(x,count*cfg.controller.sampleTime,model); ...
        -localSafetyRows(x,count*cfg.controller.sampleTime,model); ...
        reshape(lo-plan,[],1);reshape(plan-hi,[],1); ...
        reshape(abs(diff([model.previousInput,plan],1,2))-rate,[],1); ...
        abs(plan(:,end)-reference.input)-(rate-model.terminal.inputRadius)]);
    before=nonlinearBicycleModel.error(states(:,1),model.lane,reference);
    after=nonlinearBicycleModel.error(states(:,2),model.lane,reference);
    v0=norm(reference.factor*before)^2;v1=norm(reference.factor*after)^2;
    clf=max(0,v1-(1-cfg.nonlinear.clfDecay)*v0);
    evaluation=struct('inputs',plan,'states',states,'safety',sum(slacks),'stageSlacks',slacks, ...
        'hard',hard,'clfSlack',clf,'clfInitialValue',v0,'clfNextValue',v1, ...
        'minimumCollisionMargin',collision,'minimumRoadMargin',road, ...
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
    margin=predictiveSafetyGeometry.terminalMargin(x,q,model.frame,model.terminal,shape, ...
        cfg.collision.safetyMarginMeters);
end

function q=localTargetAt(model,time)
    q=predictiveSafetyGeometry.targetFlow(model.target,time);
end
