function [solution,search,model] = solvePredictiveControl(model,previousState)
%solvePredictiveControl Lexicographic PCBF/CLF with a retained hard completion.
% Only nonlinear feasible iterates within the shifted slack budget replace
% the retained witness. The endpoint family supplies every newly appended input.
    cfg=model.cfg;timer=tic;solution=[];model.slackCap=Inf;
    search=struct('solverCalls',0,'source',"sequentialConvexification", ...
        'terminationReason',"iterationLimit",'sequentialIterations',{{}}, ...
        'initialization',"laneFeedbackRollout",'safetySlack',Inf,'shiftAvailable',false, ...
        'slackCap',Inf,'converged',false,'failures',strings(0,1));
    if isstruct(previousState)
        model.terminal=previousState.terminal;
        anchor=previousState.inputTrajectory(:,2:end);
        expected=previousState.stateTrajectory(:,2);
        sameState=isequal(model.initialState,expected) && isequal(model.previousInput,previousState.appliedInput);
        if size(anchor,2)<cfg.controller.horizonSteps
            y=[previousState.stateTrajectory(:,end);previousState.inputTrajectory(:,end)];
            oldEnd=previousState.sampleIndex+size(previousState.inputTrajectory,2);
            anchor(:,end+1)=terminalContinuation.control(y,oldEnd,model.terminal);
        end
        search.initialization="shiftedContinuation";
        if sameState
            model.slackCap=sum(previousState.witness.stageSlacks(2:end));
            solution=localNominalEvaluation(anchor,model);
            if solution.hard>0 || solution.safety>model.slackCap
                error('collisionAvoidanceController:invalidRetainedContinuation', ...
                    'The retained nominal witness failed its successor constraints or slack budget.');
            end
            search.shiftAvailable=true;search.source="retainedContinuation";
        else
            search.initialization="restorationFromChangedEgoState";
        end
    else
        [anchor,model.terminal]=localSeed(model);
    end
    search.slackCap=model.slackCap;
    if isempty(solution),baseline=localNominalEvaluation(anchor,model);
    else,baseline=solution;
    end
    if isempty(solution) && baseline.hard==0 && baseline.safety<=model.slackCap
        solution=baseline;search.source="feasibleInitialization";
    end
    radius=cfg.nonlinear.trustRadius;
    for iteration=1:cfg.nonlinear.maximumIterations
        if toc(timer)>=cfg.solver.timeLimitSeconds,search.terminationReason="timeLimit";break;end
        try
            [inputs,step]=localSequentialStep(anchor,model,radius);
            search.solverCalls=search.solverCalls+step.calls;
            step.iteration=iteration;step.trustRadius=radius;step.acceptedIterate=false;
            step.retainedAsWitness=false;step.nominalSafetySlack=Inf;step.nominalHardViolation=Inf;
            if isempty(inputs)
                search.terminationReason=step.status;radius=radius/2;
                search.sequentialIterations{end+1}=step;
                if radius<1e-7,break;end
                continue;
            end
            candidate=localNominalEvaluation(inputs,model);
            candidate=localPolishEndpoint(candidate,model);
            inputs=candidate.inputs;
            step.nominalSafetySlack=candidate.safety;step.nominalHardViolation=candidate.hard;
            merit=baseline.safety+100*baseline.hard;nextMerit=candidate.safety+100*candidate.hard;
            improves=nextMerit<merit || (candidate.hard==0 && candidate.safety<=baseline.safety && candidate.cost<baseline.cost);
            if improves
                anchor=inputs;baseline=candidate;step.acceptedIterate=true;
                radius=min(1,1.25*radius);
            else
                radius=radius/2;
            end
            feasible=candidate.hard==0 && candidate.safety<=model.slackCap;
            better=isempty(solution) || candidate.safety<solution.safety ...
                || (candidate.safety==solution.safety && candidate.cost<solution.cost);
            if feasible && better
                solution=candidate;search.source="sequentialConvexification";step.retainedAsWitness=true;
            end
            search.sequentialIterations{end+1}=step;
            search.converged=feasible && candidate.safety==0 && ~step.completionRestoration ...
                && step.secondaryReturned ...
                && candidate.clfSlack<=step.clfSlack+cfg.solver.feasibilityTolerance;
            if search.converged,search.terminationReason="zeroSlack";break;end
            if radius<1e-7,search.terminationReason="smallTrustRegion";break;end
        catch exception
            search.failures(end+1,1)=string(exception.identifier)+": "+string(exception.message);
            search.terminationReason=string(exception.message);
            if strcmp(exception.identifier,'collisionAvoidanceController:nonlinearDomain')
                radius=radius/2;if radius>=1e-7,continue;end
            end
            break;
        end
    end
    if ~isempty(solution),search.safetySlack=solution.safety;end
    search.elapsedSeconds=toc(timer);
end

function candidate=localPolishEndpoint(candidate,model)
    % A small shooting correction removes the nonlinear terminal defect of
    % an affine SCvx step. It is a search step: no corrected input is clipped,
    % and all nonlinear constraints and the PCBF budget are checked afterward.
    cfg=model.cfg;count=size(candidate.inputs,2);seed=model.terminal;
    tail=max(1,count-cfg.controller.horizonSteps);length=min(count,max(8,min(20,tail)));
    first=count-length+1;endIndex=model.sampleIndex+count;
    ref=terminalContinuation.referenceAt(seed,endIndex);
    transform=blkdiag([cos(ref(3)),sin(ref(3));-sin(ref(3)),cos(ref(3))],eye(6));
    for iteration=1:5
        [~,~,deviation]=terminalContinuation.membership([candidate.states(:,end);candidate.inputs(:,end)],endIndex,seed);
        if norm(seed.factor*deviation)<.5*seed.radius,return;end
        sensitivity=zeros(6,2*length);x=candidate.states(:,first);
        for index=first:count
            [x,a,b]=nonlinearBicycleModel.sample(x,candidate.inputs(:,index),cfg);
            sensitivity=a*sensitivity;sensitivity(:,2*(index-first)+(1:2))=b;
        end
        memory=zeros(2,2*length);memory(:,end-1:end)=eye(2);
        tangent=seed.factor*transform*[sensitivity;memory];
        correction=-pinv(tangent)*(seed.factor*deviation);
        improved=false;
        for exponent=0:5
            inputs=candidate.inputs;inputs(:,first:end)=inputs(:,first:end)+reshape(correction,2,[])*2^-exponent;
            try
                trial=localNominalEvaluation(inputs,model);
            catch exception
                if strcmp(exception.identifier,'collisionAvoidanceController:nonlinearDomain'),continue;end
                rethrow(exception);
            end
            [~,~,nextDeviation]=terminalContinuation.membership([trial.states(:,end);inputs(:,end)],endIndex,seed);
            if norm(seed.factor*nextDeviation)<norm(seed.factor*deviation)
                candidate=trial;improved=true;break;
            end
        end
        if ~improved,return;end
    end
end

function [inputs,seed]=localSeed(model)
    cfg=model.cfg;seed=model.terminal;reference=seed.reference;x=model.initialState;previous=model.previousInput;
    required=cfg.controller.horizonSteps+ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime);
    required=min(required,cfg.controller.maximumHorizonSteps);inputs=zeros(2,cfg.controller.maximumHorizonSteps);
    for index=1:size(inputs,2)
        deviation=nonlinearBicycleModel.error(x,model.lane,reference);
        u=localClip(reference.input+reference.gain*deviation,previous,cfg);inputs(:,index)=u;
        x=nonlinearBicycleModel.sample(x,u,cfg);previous=u;
        if index<required,continue;end
        seed=terminalContinuation.anchor(seed,x,model.sampleIndex+index,model.lane);
        value=terminalContinuation.membership([x;u],seed.epochIndex,seed);
        q=localTargetAt(model,index*cfg.controller.sampleTime);
        if value<0 && terminalContinuation.separation(seed,q,model.frame,cfg)>0
            inputs=inputs(:,1:index);return;
        end
    end
    error('collisionAvoidanceController:noTerminalContinuation', ...
        'No indefinitely admissible endpoint was reached within maximumHorizonSteps.');
end

function u=localClip(u,previous,cfg)
    % Clipping is used only to initialize a numerical search, never to modify
    % the retained witness or the endpoint policy.
    lower=[-cfg.model.frontWheelSteeringAngleMaximum;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
    upper=[cfg.model.frontWheelSteeringAngleMaximum;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    u=min(upper,max(lower,min(previous+rate,max(previous-rate,u))));
end

function [inputs,info]=localSequentialStep(anchor,model,radius)
    cfg=model.cfg;count=size(anchor,2);prefix=cfg.controller.horizonSteps;reference=model.terminal.reference;
    ix=reshape(1:6*(count+1),6,[]);iu=reshape(ix(end)+(1:2*count),2,[]);
    is=iu(end)+(1:count);ic=is(end)+1;it=ic+1;nv=it;
    equal=sparse(6*(count+1),nv);rhs=zeros(6*(count+1),1);equal(1:6,ix(:,1))=eye(6);
    rows=cell(1,4*count+6);bounds=cell(1,4*count+6);rowCount=0;
    costMatrix=sparse(7*count+1,nv);costOffset=zeros(7*count+1,1);
    lower=-Inf(nv,1);upper=Inf(nv,1);
    stateTrust=radius*[5;5;.5;5;3;1.5];inputTrust=radius*[.15;.25];
    lower(ix(:))=-repmat(stateTrust,count+1,1);upper(ix(:))=-lower(ix(:));
    lower(iu(:))=-repmat(inputTrust,count,1);upper(iu(:))=-lower(iu(:));lower([is,ic,it])=0;
    upper(is(prefix+1:end))=0;
    inputLower=[-cfg.model.frontWheelSteeringAngleMaximum;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
    inputUpper=[cfg.model.frontWheelSteeringAngleMaximum;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    [physicalLower,physicalUpper]=localStateLimits(cfg);
    x=model.initialState;initialError=nonlinearBicycleModel.error(x,model.lane,reference);halfDuration=cfg.controller.sampleTime/2;
    for index=1:count
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
        r=-j*map;r(:,is(index))=-1;
        rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
        map(:,ix(:,index))=am;map(:,iu(:,index))=bm;
        [g,j]=localSafetyRows(middle,(index-.5)*cfg.controller.sampleTime,model);
        r=-j*map;r(:,is(index))=-1;
        rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
        rowCount=rowCount+1;rows{rowCount}=[map(4:6,:);-map(4:6,:)];
        bounds{rowCount}=[physicalUpper-middle(4:6);middle(4:6)-physicalLower];
        [e,j]=nonlinearBicycleModel.errorLinearization(next,model.lane,reference);jx=ix(:,index+1);
        ci=7*(index-1)+(1:7);costMatrix(ci(1:5),jx)=reference.factor*j/sqrt(count);
        costOffset(ci(1:5))=reference.factor*e/sqrt(count);
        costMatrix(ci(6:7),iu(:,index))=.1*eye(2)/sqrt(count);
        costOffset(ci(6:7))=.1*(anchor(:,index)-reference.input)/sqrt(count);
        lower(jx(4:6))=max(lower(jx(4:6)),physicalLower-next(4:6));
        upper(jx(4:6))=min(upper(jx(4:6)),physicalUpper-next(4:6));
        if index==1
            r=sparse(1,nv);r(jx)=2*e.'*reference.matrix*j;r(ic)=-1;
            rowCount=rowCount+1;rows{rowCount}=r;
            bounds{rowCount}=(1-cfg.nonlinear.clfDecay)*norm(reference.factor*initialError)^2-norm(reference.factor*e)^2;
        end
        x=next;
    end
    % The same 2-norm endpoint membership is used here and in the nonlinear
    % rollout. There is no inscribed-polytope/ellipsoid mismatch.
    endIndex=model.sampleIndex+count;y=[x;anchor(:,end)];seed=model.terminal;
    ref=terminalContinuation.referenceAt(seed,endIndex);
    transform=blkdiag([cos(ref(3)),sin(ref(3));-sin(ref(3)),cos(ref(3))],eye(6));
    [~,~,deviation]=terminalContinuation.membership(y,endIndex,seed);
    map=sparse(8,nv);map(1:6,ix(:,end))=eye(6);map(7:8,iu(:,end))=eye(2);
    endpoint=secondordercone(seed.factor*transform*map,-seed.factor*deviation,zeros(nv,1),-seed.radius);
    [g,j]=localSafetyRows(x,count*cfg.controller.sampleTime,model);
    r=sparse(size(j,1),nv);r(:,ix(:,end))=-j;rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
    objective=zeros(nv,1);objective(is(1:prefix))=1;
    if isfinite(model.slackCap)
        rowCount=rowCount+1;rows{rowCount}=sparse(objective.');bounds{rowCount}=model.slackCap;
    end
    a=vertcat(rows{1:rowCount});b=vertcat(bounds{1:rowCount});
    info=struct('calls',0,'safetyOptimum',Inf,'secondarySafety',Inf,'safetyCap',Inf, ...
        'clfSlack',Inf,'status',"infeasibleBounds",'safetyExitFlag',NaN,'secondaryExitFlag',NaN, ...
        'completionRestoration',false,'secondaryReturned',false);inputs=[];
    if any(lower>upper) || any(~isfinite(b)),return;end
    options=optimoptions('coneprog','Display','none','MaxIterations',cfg.solver.maxIterations, ...
        'ConstraintTolerance',cfg.solver.constraintTolerance,'OptimalityTolerance',cfg.solver.optimalityTolerance, ...
        'MaxTime',cfg.solver.timeLimitSeconds);
    [first,~,flag]=coneprog(objective,endpoint,a,b,equal,rhs,lower,upper,options);info.calls=1;info.safetyExitFlag=flag;
    if flag<=0 || isempty(first)
        % Elastic rows are a numerical restoration step only. The completion
        % remains hard in the MPC problem and in nonlinear witness admission.
        upper(is(prefix+1:end))=Inf;restoration=objective;restoration(is(prefix+1:end))=100;
        [first,~,flag]=coneprog(restoration,endpoint,a,b,equal,rhs,lower,upper,options);info.calls=2;
        if flag<=0 || isempty(first),info.status="safetySolveFailed";return;end
        info.completionRestoration=true;info.status="restoringCompletion";
        info.secondarySafety=sum(max(0,first(is(1:prefix))));info.clfSlack=max(0,first(ic));
        inputs=anchor+reshape(first(iu(:)),2,[]);return;
    end
    info.safetyOptimum=sum(max(0,first(is(1:prefix))));info.safetyCap=min(model.slackCap,info.safetyOptimum+cfg.solver.lexicographicTieTolerance);
    a=[a;objective.'];b=[b;info.safetyCap];
    costMatrix(end,ic)=sqrt(cfg.clf.relaxationWeight);
    % Epigraph of the quadratic lane/CLF objective as a second-order cone.
    r=sparse(1,nv);r(it)=1;
    costCone=secondordercone([2*costMatrix;r],[-2*costOffset;1],r.',-1);
    objective=zeros(nv,1);objective(it)=1;
    [second,~,flag]=coneprog(objective,[endpoint,costCone],a,b,equal,rhs,lower,upper,options);
    info.calls=2;info.secondaryExitFlag=flag;
    % A finite suboptimal conic point can still improve the nonlinear MPC.
    % Its original hard constraints and achieved slack budget decide admission.
    if ~isempty(second) && all(isfinite(second))
        first=second;info.secondaryReturned=true;
    end
    info.solverResidual=max([0;a*first-b;abs(equal*first-rhs);lower-first;first-upper]);
    info.secondarySafety=sum(max(0,first(is(1:prefix))));info.clfSlack=max(0,first(ic));info.status="solved";
    inputs=anchor+reshape(first(iu(:)),2,[]);
end

function [values,jacobian]=localSafetyRows(x,time,model)
    cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    values=zeros(0,1);jacobian=zeros(0,6);
    projection=laneGeometry.project(x(1:2),model.lane);
    preferred=[-sin(projection.heading);cos(projection.heading)];
    q=localTargetAt(model,time);
    if ~isempty(q)
        dual=predictiveSafetyGeometry.dualLinearization(x(1:3),shape,q(1:3),q(8:11),preferred);
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

function evaluation=localNominalEvaluation(inputs,model)
    cfg=model.cfg;x=model.initialState;count=size(inputs,2);prefix=cfg.controller.horizonSteps;slacks=zeros(1,prefix);
    [low,high]=localStateLimits(cfg);hard=max([0;low-x(4:6);x(4:6)-high]);
    halfDuration=cfg.controller.sampleTime/2;states=zeros(6,count+1);states(:,1)=x;collision=Inf;road=Inf;cost=0;
    for index=1:count
        values=localSafetyRows(x,(index-1)*cfg.controller.sampleTime,model);
        middle=nonlinearBicycleModel.sample(x,inputs(:,index),cfg,[],halfDuration);
        middleValues=localSafetyRows(middle,(index-.5)*cfg.controller.sampleTime,model);
        violation=max([0;-values;-middleValues]);
        if index<=prefix,slacks(index)=violation;else,hard=max(hard,violation);end
        if ~isempty(model.target)
            collision=min([collision;values(1:4);middleValues(1:4)]);road=min([road;values(5:end);middleValues(5:end)]);
        else,road=min([road;values;middleValues]);
        end
        x=nonlinearBicycleModel.sample(middle,inputs(:,index),cfg,[],halfDuration);states(:,index+1)=x;
        hard=max([hard;low-x(4:6);x(4:6)-high;low-middle(4:6);middle(4:6)-high]);
        deviation=nonlinearBicycleModel.error(x,model.lane,model.terminal.reference);
        cost=cost+norm(model.terminal.reference.factor*deviation)^2+.01*norm(inputs(:,index)-model.terminal.reference.input)^2;
    end
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    lo=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    hi=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    [~,~,deviation]=terminalContinuation.membership([x;inputs(:,end)],model.sampleIndex+count,model.terminal);
    hard=max([hard;norm(model.terminal.factor*deviation)-model.terminal.radius; ...
        -localSafetyRows(x,count*cfg.controller.sampleTime,model); ...
        reshape(lo-inputs,[],1);reshape(inputs-hi,[],1); ...
        reshape(abs(diff([model.previousInput,inputs],1,2))-rate,[],1)]);
    reference=model.terminal.reference;before=nonlinearBicycleModel.error(states(:,1),model.lane,reference);
    after=nonlinearBicycleModel.error(states(:,2),model.lane,reference);
    v0=norm(reference.factor*before)^2;v1=norm(reference.factor*after)^2;clf=max(0,v1-(1-cfg.nonlinear.clfDecay)*v0);
    evaluation=struct('inputs',inputs,'states',states,'safety',sum(slacks),'stageSlacks',slacks, ...
        'hard',hard,'clfSlack',clf,'clfInitialValue',v0,'clfNextValue',v1, ...
        'minimumCollisionMargin',collision,'minimumRoadMargin',road, ...
        'cost',cost/count+cfg.clf.relaxationWeight*clf^2);
end

function [low,high]=localStateLimits(cfg)
    low=[max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor+1e-4);-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
    high=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
end

function q=localTargetAt(model,time)
    h=model.cfg.controller.sampleTime;
    halfIndex=2*model.sampleIndex+round(2*time/h);
    q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,halfIndex*(h/2));
end
