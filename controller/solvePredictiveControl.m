function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl Return the first admissible nonlinear continuation.
% Safety and lane/CLF costs guide restoration only while no witness exists.
% The endpoint family supplies every newly appended input.
    if nargin<3,timer=tic;end
    cfg=model.cfg;solution=[];model.slackCap=Inf;
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
            if isfield(previousState.witness,'stageCost')
                shifted=previousState.witness;
                shifted.inputs=shifted.inputs(:,2:end);shifted.states=shifted.states(:,2:end);
                for name=["stageSafety","stageHard","stageCost","stageCollision","stageRoad"]
                    shifted.(name)=shifted.(name)(2:end);
                end
                solution=localNominalEvaluation(anchor,model,shifted,size(shifted.inputs,2)+1);
            else
                solution=localNominalEvaluation(anchor,model);
            end
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
    if isempty(solution) && localFeasible(baseline,model)
        solution=baseline;search.source="feasibleInitialization";
    end
    if ~isempty(solution)
        search.converged=true;search.terminationReason="feasibleWitness";
        search.safetySlack=solution.safety;search.elapsedSeconds=toc(timer);return;
    end
    radius=cfg.nonlinear.trustRadius;
    for iteration=1:cfg.nonlinear.maximumIterations
        if toc(timer)>=cfg.solver.timeLimitSeconds,search.terminationReason="timeLimit";break;end
        try
            [inputs,step,candidate]=localSequentialStep(anchor,model,radius,timer);
            search.solverCalls=search.solverCalls+step.calls;
            step.iteration=iteration;step.trustRadius=radius;step.acceptedIterate=false;
            step.retainedAsWitness=false;step.nominalSafetySlack=Inf;step.nominalHardViolation=Inf;
            if isempty(candidate) && toc(timer)>=cfg.solver.timeLimitSeconds
                search.terminationReason="timeLimit";search.sequentialIterations{end+1}=step;break;
            end
            if isempty(inputs)
                search.terminationReason=step.status;radius=radius/2;
                search.sequentialIterations{end+1}=step;
                if radius<1e-7,break;end
                continue;
            end
            if isempty(candidate),candidate=localNominalEvaluation(inputs,model);end
            if ~localFeasible(candidate,model),candidate=localPolishEndpoint(candidate,model,timer);end
            inputs=candidate.inputs;
            step.nominalSafetySlack=candidate.safety;step.nominalHardViolation=candidate.hard;
            if localFeasible(candidate,model)
                solution=candidate;search.source="sequentialConvexification";
                step.acceptedIterate=true;step.retainedAsWitness=true;
                search.sequentialIterations{end+1}=step;
                search.converged=true;search.terminationReason="feasibleWitness";break;
            end
            merit=baseline.safety+100*baseline.hard;nextMerit=candidate.safety+100*candidate.hard;
            improves=nextMerit<merit || (candidate.hard==0 && candidate.safety<=baseline.safety && candidate.cost<baseline.cost);
            if improves
                anchor=inputs;baseline=candidate;step.acceptedIterate=true;
                radius=min(1,1.25*radius);
            else
                radius=radius/2;
            end
            search.sequentialIterations{end+1}=step;
            if radius<1e-7,search.terminationReason="smallTrustRegion";break;end
        catch exception
            search.failures(end+1,1)=string(exception.identifier)+": "+string(exception.message);
            search.terminationReason=string(exception.message);
            if localDomainFailure(exception)
                radius=radius/2;if radius>=1e-7,continue;end
            end
            break;
        end
    end
    if ~isempty(solution),search.safetySlack=solution.safety;end
    search.elapsedSeconds=toc(timer);
end

function feasible=localFeasible(candidate,model)
    % Preserve the original hard constraints and shifted PCBF slack budget.
    % Positive prefix slack remains recovery, not a collision-free claim.
    feasible=~isempty(candidate) && candidate.hard==0 && candidate.safety<=model.slackCap;
end

function yes=localDomainFailure(exception)
    yes=any(strcmp(exception.identifier,{'collisionAvoidanceController:nonlinearDomain', ...
        'collisionAvoidanceController:invalidTireOperatingPoint'}));
end

function candidate=localPolishEndpoint(candidate,model,timer)
    % A small shooting correction removes the nonlinear terminal defect of
    % an affine SCvx step. It is a search step: no corrected input is clipped,
    % and all nonlinear constraints and the PCBF budget are checked afterward.
    cfg=model.cfg;count=size(candidate.inputs,2);seed=model.terminal;
    tail=max(1,count-cfg.controller.horizonSteps);length=min(count,max(8,min(20,tail)));
    first=count-length+1;endIndex=model.sampleIndex+count;
    ref=terminalContinuation.referenceAt(seed,endIndex);
    transform=blkdiag([cos(ref(3)),sin(ref(3));-sin(ref(3)),cos(ref(3))],eye(6));
    for iteration=1:5
        if localFeasible(candidate,model),return;end
        if toc(timer)>=cfg.solver.timeLimitSeconds,return;end
        [~,~,deviation]=terminalContinuation.membership([candidate.states(:,end);candidate.inputs(:,end)],endIndex,seed);
        if norm(seed.factor*deviation)<=seed.radius,return;end
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
            if toc(timer)>=cfg.solver.timeLimitSeconds,return;end
            inputs=candidate.inputs;inputs(:,first:end)=inputs(:,first:end)+reshape(correction,2,[])*2^-exponent;
            lo=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
            hi=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
            rate=cfg.controller.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
            if any(inputs<lo | inputs>hi,'all') || any(abs(diff([model.previousInput,inputs],1,2))>rate,'all')
                continue;
            end
            try
                trial=localNominalEvaluation(inputs,model,candidate,first);
            catch exception
                if localDomainFailure(exception),continue;end
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

function [inputs,info,candidate]=localSequentialStep(anchor,model,radius,timer)
    info=struct('calls',0,'safetyOptimum',Inf,'secondarySafety',Inf,'safetyCap',Inf, ...
        'clfSlack',Inf,'status',"infeasibleBounds",'safetyExitFlag',NaN,'secondaryExitFlag',NaN, ...
        'completionRestoration',false,'secondaryReturned',false);inputs=[];candidate=[];
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
        if mod(index-1,8)==0 && toc(timer)>=cfg.solver.timeLimitSeconds
            info.status="timeLimit";return;
        end
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
    if any(lower>upper) || any(~isfinite(b)),return;end
    remaining=cfg.solver.timeLimitSeconds-toc(timer);
    if remaining<=0,info.status="timeLimit";return;end
    options=optimoptions('coneprog','Display','none','MaxIterations',cfg.solver.maxIterations, ...
        'ConstraintTolerance',cfg.solver.constraintTolerance,'OptimalityTolerance',cfg.solver.optimalityTolerance, ...
        'MaxTime',remaining);
    [first,~,flag]=coneprog(objective,endpoint,a,b,equal,rhs,lower,upper,options);info.calls=1;
    info.safetyExitFlag=flag;
    if ~isempty(first) && all(isfinite(first)) && toc(timer)<cfg.solver.timeLimitSeconds
        primaryInputs=anchor+reshape(first(iu(:)),2,[]);
        try
            primaryCandidate=localNominalEvaluation(primaryInputs,model);
            % Repair terminal defects before replacing a usable primary plan.
            primaryCandidate=localPolishEndpoint(primaryCandidate,model,timer);
            primaryInputs=primaryCandidate.inputs;
            if localFeasible(primaryCandidate,model)
                inputs=primaryInputs;candidate=primaryCandidate;info.status="primaryFeasible";
                info.secondarySafety=candidate.safety;info.clfSlack=candidate.clfSlack;return;
            end
        catch exception
            if ~localDomainFailure(exception),rethrow(exception);end
        end
    end
    if flag<=0 || isempty(first)
        % Elastic rows are a numerical restoration step only. The completion
        % remains hard in the MPC problem and in nonlinear witness admission.
        upper(is(prefix+1:end))=Inf;restoration=objective;restoration(is(prefix+1:end))=100;
        remaining=cfg.solver.timeLimitSeconds-toc(timer);
        if remaining<=0,info.status="timeLimit";return;end
        options.MaxTime=remaining;
        [first,~,flag]=coneprog(restoration,endpoint,a,b,equal,rhs,lower,upper,options);info.calls=info.calls+1;
        if flag<=0 || isempty(first),info.status="safetySolveFailed";return;end
        info.completionRestoration=true;info.status="restoringCompletion";
        info.secondarySafety=sum(max(0,first(is(1:prefix))));info.clfSlack=max(0,first(ic));
        inputs=anchor+reshape(first(iu(:)),2,[]);return;
    end
    if toc(timer)>=cfg.solver.timeLimitSeconds,info.status="timeLimit";return;end
    info.safetyOptimum=sum(max(0,first(is(1:prefix))));info.safetyCap=min(model.slackCap,info.safetyOptimum+cfg.solver.lexicographicTieTolerance);
    a=[a;objective.'];b=[b;info.safetyCap];
    costMatrix(end,ic)=sqrt(cfg.clf.relaxationWeight);
    % A sum of small squared-norm epigraphs is exactly the same quadratic
    % objective, without one horizon-wide cone coupling every stage.
    groups=count+1;expanded=nv+groups;
    endpoint=secondordercone([endpoint.A,sparse(8,groups)],endpoint.b, ...
        [endpoint.d;zeros(groups,1)],endpoint.gamma);
    cones=repmat(endpoint,1,groups+1);epigraph=zeros(groups,1);
    for group=1:groups
        if group<=count,selected=7*(group-1)+(1:7);else,selected=7*count+1;end
        r=sparse(1,expanded);r(nv+group)=1;
        cones(group+1)=secondordercone([[2*costMatrix(selected,:),sparse(numel(selected),groups)];r], ...
            [-2*costOffset(selected);1],r.',-1);
        epigraph(group)=norm(costMatrix(selected,:)*first+costOffset(selected))^2;
    end
    r=sparse(1,nv);r(it)=1;
    a=[a,sparse(size(a,1),groups);-r,ones(1,groups)];b=[b;0];
    equal=[equal,sparse(size(equal,1),groups)];
    lower=[lower;zeros(groups,1)];upper=[upper;Inf(groups,1)];
    first(it)=sum(epigraph);first=[first;epigraph];
    objective=zeros(expanded,1);objective(it)=1;
    remaining=cfg.solver.timeLimitSeconds-toc(timer);
    if remaining<=0,info.status="timeLimit";return;end
    options.MaxTime=remaining;
    [second,~,flag]=coneprog(objective,cones,a,b,equal,rhs,lower,upper,options);
    info.calls=info.calls+1;info.secondaryExitFlag=flag;
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
    count=4*~isempty(model.target);values=zeros(count+8,1);
    if nargout>1,jacobian=zeros(count+8,6);end
    if count>0
        projection=laneGeometry.project(x(1:2),model.lane);
        preferred=[-sin(projection.heading);cos(projection.heading)];
        q=localTargetAt(model,time);
        dual=predictiveSafetyGeometry.dualLinearization(x(1:3),shape,q(1:3),q(8:11),preferred);
        values(1:4)=dual.value-cfg.collision.safetyMarginMeters;
        if nargout>1,jacobian(1:4,1:3)=dual.jacobian;end
    end
    rotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
    body=shape(3:4)+shape(1:2).*[-1,1,1,-1;-1,-1,1,1];
    projection=laneGeometry.project(x(1:2)+rotation*body,model.lane);
    values(count+(1:8))=reshape([projection.lateralPosition+model.frame(5); ...
        model.frame(6)-projection.lateralPosition],[],1);
    if nargout>1
        normals=[-sin(projection.heading);cos(projection.heading)];
        yaw=sum(normals.*(rotation*[0,-1;1,0]*body),1);
        gradient=[normals.',yaw.',zeros(4,3)];
        jacobian(count+(1:2:8),:)=gradient;jacobian(count+(2:2:8),:)=-gradient;
    end
end

function evaluation=localNominalEvaluation(inputs,model,cached,first)
    % Reuse only an exactly unchanged input/state prefix in the same problem.
    % A shifted cache is supplied only after the nominal successor check.
    cfg=model.cfg;count=size(inputs,2);prefix=cfg.controller.horizonSteps;
    [low,high]=localStateLimits(cfg);halfDuration=cfg.controller.sampleTime/2;
    states=zeros(6,count+1);states(:,1)=model.initialState;
    stageSafety=zeros(1,count);stageHard=stageSafety;stageCost=stageSafety;
    stageCollision=Inf(1,count);stageRoad=stageCollision;
    if nargin<3,first=1;
    else
        assert(isequal(inputs(:,1:first-1),cached.inputs(:,1:first-1)) ...
            && isequal(model.initialState,cached.states(:,1)), ...
            'collisionAvoidanceController:invalidEvaluationCache','The cached trajectory prefix changed.');
        states(:,1:first)=cached.states(:,1:first);
        stageSafety(1:first-1)=cached.stageSafety(1:first-1);
        stageHard(1:first-1)=cached.stageHard(1:first-1);
        stageCost(1:first-1)=cached.stageCost(1:first-1);
        stageCollision(1:first-1)=cached.stageCollision(1:first-1);
        stageRoad(1:first-1)=cached.stageRoad(1:first-1);
    end
    x=states(:,first);
    for index=first:count
        values=localSafetyRows(x,(index-1)*cfg.controller.sampleTime,model);
        middle=nonlinearBicycleModel.sample(x,inputs(:,index),cfg,[],halfDuration);
        middleValues=localSafetyRows(middle,(index-.5)*cfg.controller.sampleTime,model);
        stageSafety(index)=max([0;-values;-middleValues]);
        if ~isempty(model.target)
            stageCollision(index)=min([values(1:4);middleValues(1:4)]);
            stageRoad(index)=min([values(5:end);middleValues(5:end)]);
        else
            stageRoad(index)=min([values;middleValues]);
        end
        x=nonlinearBicycleModel.sample(middle,inputs(:,index),cfg,[],halfDuration);states(:,index+1)=x;
        stageHard(index)=max([0;low-x(4:6);x(4:6)-high;low-middle(4:6);middle(4:6)-high]);
        deviation=nonlinearBicycleModel.error(x,model.lane,model.terminal.reference);
        stageCost(index)=norm(model.terminal.reference.factor*deviation)^2 ...
            +.01*norm(inputs(:,index)-model.terminal.reference.input)^2;
    end
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    lo=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    hi=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    [~,~,deviation]=terminalContinuation.membership([x;inputs(:,end)],model.sampleIndex+count,model.terminal);
    hard=max([0;low-model.initialState(4:6);model.initialState(4:6)-high; ...
        stageHard.';stageSafety(prefix+1:end).';norm(model.terminal.factor*deviation)-model.terminal.radius; ...
        -localSafetyRows(x,count*cfg.controller.sampleTime,model); ...
        reshape(lo-inputs,[],1);reshape(inputs-hi,[],1); ...
        reshape(abs(diff([model.previousInput,inputs],1,2))-rate,[],1)]);
    reference=model.terminal.reference;before=nonlinearBicycleModel.error(states(:,1),model.lane,reference);
    after=nonlinearBicycleModel.error(states(:,2),model.lane,reference);
    v0=norm(reference.factor*before)^2;v1=norm(reference.factor*after)^2;clf=max(0,v1-(1-cfg.nonlinear.clfDecay)*v0);
    slacks=stageSafety(1:prefix);
    evaluation=struct('inputs',inputs,'states',states,'safety',sum(slacks),'stageSlacks',slacks, ...
        'hard',hard,'clfSlack',clf,'clfInitialValue',v0,'clfNextValue',v1, ...
        'minimumCollisionMargin',min(stageCollision),'minimumRoadMargin',min(stageRoad), ...
        'cost',sum(stageCost)/count+cfg.clf.relaxationWeight*clf^2, ...
        'stageSafety',stageSafety,'stageHard',stageHard,'stageCost',stageCost, ...
        'stageCollision',stageCollision,'stageRoad',stageRoad);
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
