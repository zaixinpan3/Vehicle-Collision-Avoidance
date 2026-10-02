function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl One RTI step, with one fresh initialization on failure.
% Returned states belong to the affine prediction, not a nonlinear replay.
% Solver exit flags are recorded, not used to admit or reject returned vectors.
% There is no post-solve constraint audit or convergence loop.
    if nargin<3,timer=tic;end
    wall=tic;
    [anchor,initialization,initializationFailure]=localInitialization(model,previousState);
    initializationSeconds=toc(wall);
    [solution,search,model]=localStep(anchor,model,initialization,timer);
    search.initializationSeconds=initializationSeconds;
    attempts=localAttempt(search);firstStages=search.stages;
    restart=isempty(solution) && any(initialization==["shiftedInputRollout","recoveryFeedbackRollout"]) ...
        && any(search.terminationReason==["pcbfNoNumericalResult","clfNoNumericalResult"]) ...
        && toc(timer)<model.cfg.solver.timeLimitSeconds;
    if restart
        % Rebuild dynamics and separation directions from the new flow rollout.
        % No first-stage or previous-frame control is executed in its place.
        reason=search.terminationReason;
        wall=tic;anchor=localFlowSeed(model,size(anchor.inputs,2));initializationSeconds=toc(wall);
        initialization="movingTargetFlow";
        if isempty(model.target),initialization="laneFeedbackRollout";end
        [solution,search,model]=localStep(anchor,model,initialization,timer);
        search.initializationSeconds=initializationSeconds;
        attempts(2)=localAttempt(search);
        search.stages=[firstStages,search.stages];
        initializationFailure=reason;
    end
    search.attempts=attempts;search.linearizationCount=numel(attempts);
    search.solverCalls=sum([attempts.solverCalls]);search.flowRestarted=restart;
    search.initializationFailure=initializationFailure;
    search.elapsedSeconds=toc(timer);
end

function attempt=localAttempt(search)
    attempt=struct('initialization',search.initialization,'terminationReason',search.terminationReason, ...
        'solverCalls',search.solverCalls,'stages',search.stages, ...
        'initializationSeconds',search.initializationSeconds,'formulationSeconds',search.formulationSeconds);
end

function [solution,search,model]=localStep(anchor,model,initialization,timer)
    cfg=model.cfg;solution=[];
    wall=tic;[problem,model]=localFormulate(anchor,model);formulationSeconds=toc(wall);
    model.linearization=anchor;
    search=struct('solverCalls',0,'source',"twoStageRealTimeIteration", ...
        'initialization',initialization,'returned',false,'converged',false,'terminationReason',"timeLimit", ...
        'formulationSeconds',formulationSeconds, ...
        'primaryOptimum',NaN,'slackCap',NaN,'clfInitialSlack',problem.initialClfSlack, ...
        'clfStageAttempted',false,'clfStageCompleted',false,'clfLowerBound',false, ...
        'stages',struct('objective',{},'exitFlag',{},'seconds',{},'value',{}));
    remaining=cfg.solver.timeLimitSeconds-toc(timer);
    if remaining<=0,search.elapsedSeconds=toc(timer);return;end
    options=optimoptions('coneprog','Display','none','MaxIterations',cfg.solver.maxIterations, ...
        'ConstraintTolerance',cfg.solver.constraintTolerance,'OptimalityTolerance',cfg.solver.optimalityTolerance, ...
        'MaxTime',remaining);
    wall=tic;
    [first,~,flag]=coneprog(problem.safetyObjective,problem.cones,problem.a,problem.b, ...
        problem.equal,problem.rhs,problem.lower,problem.upper,options);
    search.solverCalls=1;
    search.stages(1)=struct('objective',"pcbfSlack",'exitFlag',flag,'seconds',toc(wall),'value',NaN);
    if isempty(first) || any(~isfinite(first))
        search.terminationReason="pcbfNoNumericalResult";search.elapsedSeconds=toc(timer);return;
    end
    optimum=sum(max(0,first(problem.slackIndices)));
    search.primaryOptimum=optimum;search.stages(1).value=optimum;
    search.slackCap=optimum+cfg.solver.lexicographicTieTolerance;
    problem.a=[problem.a;problem.safetyObjective.'];problem.b=[problem.b;search.slackCap];
    remaining=cfg.solver.timeLimitSeconds-toc(timer);
    if remaining<=0,search.elapsedSeconds=toc(timer);return;end
    options.MaxTime=remaining;search.clfStageAttempted=true;wall=tic;
    objective=zeros(size(problem.safetyObjective));objective(problem.clfIndex)=1;
    [second,~,flag]=coneprog(objective,problem.cones,problem.a,problem.b, ...
        problem.equal,problem.rhs,problem.lower,problem.upper,options);
    search.solverCalls=2;
    search.stages(2)=struct('objective',"clfSlack",'exitFlag',flag,'seconds',toc(wall),'value',NaN);
    if isempty(second) || any(~isfinite(second))
        search.terminationReason="clfNoNumericalResult";search.elapsedSeconds=toc(timer);return;
    end
    remaining=cfg.solver.timeLimitSeconds-toc(timer);
    if problem.recovery.active && remaining>0
        % Third stage without collision rows: among both achieved slack levels,
        % the plan closest to the anchor, so the issued input stays on the
        % recovery feedback whenever it is admissible (RECOVERY_CLF.md).
        nv=numel(objective);iu=problem.inputIndices;weights=ones(size(iu));
        weights(:,1)=cfg.recovery.firstInputWeight;
        deviation=sparse(1:numel(iu),iu(:),weights(:),numel(iu),nv+1);
        axis=sparse(1,nv+1);axis(end)=1;cones=problem.cones;
        for index=1:numel(cones)
            cones(index)=secondordercone([cones(index).A,sparse(size(cones(index).A,1),1)], ...
                cones(index).b,[cones(index).d;0],cones(index).gamma);
        end
        cones(end+1)=secondordercone(deviation,zeros(numel(iu),1),axis.',0);
        lower=[problem.lower;0];upper=[problem.upper;Inf];
        upper(problem.clfIndex)=max(0,second(problem.clfIndex))+cfg.solver.lexicographicTieTolerance;
        options.MaxTime=remaining;wall=tic;
        [third,~,flag]=coneprog(axis.',cones,[problem.a,sparse(size(problem.a,1),1)],problem.b, ...
            [problem.equal,sparse(size(problem.equal,1),1)],problem.rhs,lower,upper,options);
        search.solverCalls=3;
        search.stages(3)=struct('objective',"anchorDeviation",'exitFlag',flag,'seconds',toc(wall),'value',NaN);
        if ~isempty(third) && all(isfinite(third))
            second=third(1:nv);search.stages(3).value=third(end);
        end
    end
    inputs=anchor.inputs+reshape(second(problem.inputIndices),size(anchor.inputs));
    states=anchor.states+reshape(second(problem.stateIndices),size(anchor.states));
    slacks=max(0,second(problem.slackIndices));rho=problem.clfScale*max(0,second(problem.clfIndex));
    nextValue=(norm(problem.clfMap*second+problem.clfOffset)^2+problem.clfModelConstant)*problem.clfScale;
    % Fitting the free pose only packages the endpoint; it is not an admission test.
    model.terminal=terminalContinuation.fit(model.terminal,[states(:,end);inputs(:,end)], ...
        model.sampleIndex+size(inputs,2));
    solution=struct('inputs',inputs,'states',states,'stageSlacks',slacks.', ...
        'safety',sum(slacks),'hard',NaN,'clfSlack',rho, ...
        'clfInitialValue',problem.initialClfValue,'clfNextValue',nextValue, ...
        'clfFunction',problem.recovery.function,'clfRequiredDecrease',problem.recovery.requiredDecrease, ...
        'recoveryRolloutSteps',problem.recovery.rolloutSteps,'recoveryRolloutConverged',problem.recovery.converged, ...
        'minimumCollisionMargin',NaN,'terminalSeparationMargin',NaN, ...
        'encounterExit',problem.encounterExit,'affineValidationPerformed',false,'nonlinearValidationPerformed',false);
    model.linearization=anchor;
    search.stages(2).value=rho;search.clfStageCompleted=true;search.returned=true;
    search.converged=all([search.stages.exitFlag]>0);
    search.clfLowerBound=rho<=cfg.solver.feasibilityTolerance*problem.clfScale;
    search.terminationReason="twoStagesReturned";search.elapsedSeconds=toc(timer);
end

function [anchor,source,failure]=localInitialization(model,previous)
    cfg=model.cfg;
    count=min(cfg.controller.maximumHorizonSteps,cfg.controller.horizonSteps ...
        +ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime));
    source="movingTargetFlow";failure="";
    if isempty(model.target),source="laneFeedbackRollout";end
    domainErrors=["collisionAvoidanceController:nonlinearDomain", ...
        "collisionAvoidanceController:invalidTireOperatingPoint", ...
        "collisionAvoidanceController:singularTireLinearization"];
    if localBeyondRange(model.initialState,0,model)
        % No target within the encounter range: anchor every frame on the
        % recovery feedback instead of the shifted plan (RECOVERY_CLF.md).
        try
            anchor=localRecoverySeed(model,count);source="recoveryFeedbackRollout";return;
        catch exception
            if ~any(string(exception.identifier)==domainErrors),rethrow(exception);end
            failure=string(exception.identifier);anchor=localFlowSeed(model,count);return;
        end
    end
    if isstruct(previous)
        inputs=previous.inputTrajectory(:,2:end);
        usable=~isempty(inputs) && all(isfinite(inputs(:))) && all(abs(inputs(2,:))<1);
        if usable
            inputs=inputs(:,1:min(count,size(inputs,2)));
            try
                states=zeros(6,count+1);states(:,1)=model.initialState;
                for index=1:count
                    if index>size(inputs,2)
                        inputs(:,index)=localClip(model.terminal.reference.input,inputs(:,end),cfg);
                    end
                    states(:,index+1)=nonlinearBicycleModel.sample(states(:,index),inputs(:,index),cfg);
                end
                anchor=struct('inputs',inputs,'states',states);source="shiftedInputRollout";return;
            catch exception
                if ~any(string(exception.identifier)==domainErrors),rethrow(exception);end
                % This is reference construction, never a candidate admission test.
                failure=string(exception.identifier);
            end
        else
            failure="unusableShiftedInputs";
        end
    end
    anchor=localFlowSeed(model,count);
end

function anchor=localFlowSeed(model,count)
    % One flow-guided initialization, with no safety or terminal admission.
    cfg=model.cfg;reference=model.nominalReference;x=model.initialState;previous=model.previousInput;
    h=cfg.controller.sampleTime;tire=modifiedFialaTire.parameters(cfg);
    inputs=zeros(2,count);states=zeros(6,count+1);states(:,1)=x;
    radius=0;
    if ~isempty(model.target)
        radius=norm([cfg.vehicle.length;cfg.vehicle.width]/2)+norm(cfg.vehicle.rectangleOffset) ...
            +norm(model.target(8:9))+norm(model.target(10:11))+cfg.collision.safetyMarginMeters;
    end
    station=[];completionStarted=false;exited=localBeyondRange(x,0,model);
    for index=1:count
        time=(model.sampleIndex+index-1)*h;
        q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,time);
        projection=laneGeometry.project(x(1:2),model.lane,station);station=projection.station;
        direction=[cos(projection.heading);sin(projection.heading)];normal=[-direction(2);direction(1)];
        deviation=nonlinearBicycleModel.error(x,model.lane,reference);active=false;
        if ~exited
            for preview=[0,.5,1,1.5,2,3]
                future=predictiveSafetyGeometry.targetFlow(model.targetEpoch,time+preview);
                if isfield(model.lane,'referenceCurve')
                    position=laneGeometry.referencePose(station+cfg.referenceSpeed*preview,0,model.lane.referenceCurve);
                else,position=projection.point+cfg.referenceSpeed*preview*direction;
                end
                if norm(position-future(1:2))<radius+1,active=true;break;end
            end
        end
        if active && ~completionStarted
            yaw=q(3);rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];offset=rotation*q(10:11);
            omega=q(4)*sin(q(6))/q(7);
            translation=q(4)*[cos(yaw+q(6));sin(yaw+q(6))]+omega*[-offset(2);offset(1)];
            nominal=cfg.referenceSpeed*direction-.5*projection.lateralPosition*normal;
            circulation=-.6*max(norm(nominal-translation),.5*cfg.referenceSpeed)/radius;
            velocity=predictiveSafetyGeometry.movingFlowVelocity(x(1:2),nominal,q(1:2)+offset, ...
                translation,radius*eye(2),zeros(2),circulation);
            heading=atan2(velocity(2),velocity(1));
            desiredSpeed=min(cfg.referenceSpeed,max(max(1.5,.5*cfg.referenceSpeed),norm(velocity)));
            deviation(1)=0;deviation(2)=atan2(sin(x(3)-heading),cos(x(3)-heading));
            deviation(3)=x(4)-desiredSpeed;
        end
        if index>cfg.controller.horizonSteps && (~active || completionStarted)
            completionStarted=true;seed=terminalContinuation.fit(model.terminal,[x;previous],model.sampleIndex+index-1);
            [~,~,deviation]=terminalContinuation.membership([x;previous],seed.epochIndex,seed);
            u=seed.reference.input+seed.gain*deviation;
        else,u=reference.input+reference.gain*deviation;
        end
        u=localShape(u,x,previous,cfg,tire);
        inputs(:,index)=u;x=nonlinearBicycleModel.sample(x,u,cfg);states(:,index+1)=x;previous=u;
        exited=exited || localBeyondRange(x,index*h,model);
    end
    anchor=struct('inputs',inputs,'states',states);
end

function anchor=localRecoverySeed(model,count)
    % Recovery feedback over the primary horizon, then the terminal completion
    % law of the flow initialization. The first input is recoveryInput itself.
    cfg=model.cfg;x=model.initialState;previous=model.previousInput;
    terminal=nonlinearBicycleModel.recoveryTerminal(cfg,model.nominalReference.curvature);
    tire=modifiedFialaTire.parameters(cfg);inputs=zeros(2,count);states=zeros(6,count+1);states(:,1)=x;
    for index=1:count
        if index<=cfg.controller.horizonSteps
            u=nonlinearBicycleModel.recoveryInput(x,previous,model.lane,model.nominalReference,cfg,terminal);
        else
            seed=terminalContinuation.fit(model.terminal,[x;previous],model.sampleIndex+index-1);
            [~,~,deviation]=terminalContinuation.membership([x;previous],seed.epochIndex,seed);
            u=localShape(seed.reference.input+seed.gain*deviation,x,previous,cfg,tire);
        end
        inputs(:,index)=u;x=nonlinearBicycleModel.sample(x,u,cfg);states(:,index+1)=x;previous=u;
    end
    anchor=struct('inputs',inputs,'states',states);
end

function u=localShape(u,x,previous,cfg,tire)
    u(2)=min(.35,max(-.35,u(2)));u=localClip(u,previous,cfg);
    % Tire-informed seed shaping does not bound the optimized steering.
    slipLimit=atan(3*tire.longitudinalForceScale(1)*sqrt(1-u(2)^2) ...
        /tire.corneringStiffness(1)*(1-(1-.8)^(1/3)));
    zeroSlip=atan2(x(5)+cfg.vehicle.lf*x(6),max(x(4),cfg.model.scheduleSpeedFloor));
    u(1)=min(zeroSlip+slipLimit,max(zeroSlip-slipLimit,u(1)));u=localClip(u,previous,cfg);
end

function [problem,model]=localFormulate(anchor,model)
    cfg=model.cfg;count=size(anchor.inputs,2);prefix=cfg.controller.horizonSteps;reference=model.nominalReference;
    ix=reshape(1:6*(count+1),6,[]);iu=reshape(ix(end)+(1:2*count),2,[]);
    is=iu(end)+(1:prefix);ic=is(end)+1;nv=ic;
    equal=sparse(6*(count+1),nv);rhs=zeros(6*(count+1),1);equal(1:6,ix(:,1))=eye(6);
    rhs(1:6)=model.initialState-anchor.states(:,1);
    rows=cell(1,5*count+3);bounds=cell(size(rows));rowCount=0;
    lower=-Inf(nv,1);upper=Inf(nv,1);radius=cfg.nonlinear.trustRadius;
    lower(ix(:))=-repmat(radius*[5;5;.5;5;3;1.5],count+1,1);upper(ix(:))=-lower(ix(:));
    lower(iu(:))=-repmat(radius*[.15;.25],count,1);upper(iu(:))=-lower(iu(:));lower([is,ic])=0;
    % Numerical RTI corrections are local to the new anchor at every sample.
    % Steering still has no actuator magnitude or slew constraint.
    inputLower=[-Inf;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
    inputUpper=[Inf;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    rate=[Inf;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    [physicalLower,physicalUpper]=localStateLimits(cfg);
    initialError=nonlinearBicycleModel.error(model.initialState,model.lane,reference);
    v0=norm(reference.factor*initialError)^2;clfScale=max(v0,1);
    departure=Inf;if isempty(model.target),departure=0;end
    for index=1:count
        x=anchor.states(:,index);input=anchor.inputs(:,index);nextReference=anchor.states(:,index+1);
        [middle,am,bm,next,a,b]=nonlinearBicycleModel.hold(x,input,cfg);
        eq=6*index+(1:6);equal(eq,ix(:,index+1))=eye(6);equal(eq,ix(:,index))=-a;equal(eq,iu(:,index))=-b;
        rhs(eq)=next-nextReference;
        lower(iu(:,index))=max(lower(iu(:,index)),inputLower-input);
        upper(iu(:,index))=min(upper(iu(:,index)),inputUpper-input);
        r=sparse(2,nv);r(:,iu(:,index))=eye(2);previous=model.previousInput;
        if index>1,r(:,iu(:,index-1))=-eye(2);previous=anchor.inputs(:,index-1);end
        finite=isfinite(rate);difference=input-previous;
        rowCount=rowCount+1;rows{rowCount}=[r(finite,:);-r(finite,:)];bounds{rowCount}=[rate(finite)-difference(finite);rate(finite)+difference(finite)];
        if departure==Inf && localBeyondRange(x,(index-1)*cfg.controller.sampleTime,model),departure=index-1;end
        map=sparse(6,nv);map(:,ix(:,index))=eye(6);
        if index<=departure
            [g,j]=localSafetyRows(x,(index-1)*cfg.controller.sampleTime,model);
            r=-j*map;if index<=prefix,r(:,is(index))=-1;end
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
        end
        map(:,ix(:,index))=am;map(:,iu(:,index))=bm;
        if index<=departure
            [g,j]=localSafetyRows(middle,(index-.5)*cfg.controller.sampleTime,model);
            r=-j*map;if index<=prefix,r(:,is(index))=-1;end
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
        end
        rowCount=rowCount+1;rows{rowCount}=[map(4:6,:);-map(4:6,:)];
        bounds{rowCount}=[physicalUpper-middle(4:6);middle(4:6)-physicalLower];
        jx=ix(:,index+1);
        lower(jx(4:6))=max(lower(jx(4:6)),physicalLower-nextReference(4:6));
        upper(jx(4:6))=min(upper(jx(4:6)),physicalUpper-nextReference(4:6));
        if index==1
            [e,j]=nonlinearBicycleModel.errorLinearization(nextReference,model.lane,reference);
            clfMap=sparse(5,nv);clfMap(:,jx)=reference.factor*j/sqrt(clfScale);
            clfOffset=reference.factor*e/sqrt(clfScale);
        end
    end
    endIndex=model.sampleIndex+count;y=[anchor.states(:,end);anchor.inputs(:,end)];
    [seed,poseJacobian]=terminalContinuation.fit(model.terminal,y,endIndex);model.terminal=seed;
    deviation=y(4:8)-[seed.base(4:6);seed.reference.input];
    map=sparse(8,nv);map(1:6,ix(:,end))=eye(6);map(7:8,iu(:,end))=eye(2);
    endpoint=secondordercone(seed.quotientFactor*map(4:8,:),-seed.quotientFactor*deviation,zeros(nv,1),-seed.radius);
    clfAxis=sparse(1,nv);clfAxis(ic)=1;clfModelConstant=0;
    recovery=struct('active',false,'function',"laneQuadratic",'requiredDecrease',NaN,'rolloutSteps',NaN,'converged',false);
    if departure==0
        % No collision rows: the recovery CLF replaces the lane quadratic.
        [clfMap,clfOffset,clfConstant,v0,clfScale,clfModelConstant,recovery]=localRecoveryClf(anchor,model,ix(:,2),nv);
    else
        clfConstant=(1-cfg.nonlinear.clfDecay)*v0/clfScale;
    end
    clfCone=secondordercone([2*clfMap;clfAxis],[-2*clfOffset;1-clfConstant],clfAxis.',-1-clfConstant);
    terminalA=zeros(0,nv);terminalB=zeros(0,1);
    if departure>count
        if localBeyondRange(y(1:6),count*cfg.controller.sampleTime,model),departure=count;
        else
            [~,terminalB,gradient]=localTerminalClearance(seed,count,model);
            terminalA=-gradient*poseJacobian*map;
            rowCount=rowCount+1;rows{rowCount}=terminalA;bounds{rowCount}=terminalB;
            [g,j]=localSafetyRows(y(1:6),count*cfg.controller.sampleTime,model);
            r=sparse(size(j,1),nv);r(:,ix(:,end))=-j;
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
        end
    end
    objective=zeros(nv,1);objective(is)=1;
    problem=struct('a',vertcat(rows{1:rowCount}),'b',vertcat(bounds{1:rowCount}), ...
        'equal',equal,'rhs',rhs,'lower',lower,'upper',upper,'cones',[endpoint,clfCone], ...
        'stateIndices',ix,'inputIndices',iu,'slackIndices',is,'clfIndex',ic, ...
        'safetyObjective',objective,'clfScale',clfScale,'clfMap',clfMap,'clfOffset',clfOffset, ...
        'clfModelConstant',clfModelConstant,'recovery',recovery, ...
        'initialClfValue',v0,'initialClfSlack',max(0,(norm(clfOffset)^2-clfConstant)*clfScale), ...
        'terminalA',terminalA,'terminalB',terminalB,'encounterExit',departure);
end

function [map,offset,constant,value,scale,modelConstant,recovery]=localRecoveryClf(anchor,model,next,nv)
    % Recovery CLF (RECOVERY_CLF.md): V(x0) from one rollout of the recovery
    % feedback. At the anchor's first node V is modelled as c + |a + R dx1|^2
    % (Gauss-Newton), with the residual Jacobian from rollouts of the same length
    % started at perturbed states. The required decrease is a fraction of l(x0).
    cfg=model.cfg;reference=model.nominalReference;lane=model.lane;
    terminal=nonlinearBicycleModel.recoveryTerminal(cfg,reference.curvature);
    x0=model.initialState;u0=anchor.inputs(:,1);x1=anchor.states(:,2);
    [value,residual0,steps0,converged]=nonlinearBicycleModel.recoveryValue(x0,model.previousInput,lane,reference,terminal,cfg);
    stage=sum((terminal.stageFactor*nonlinearBicycleModel.error(x0,lane,reference)).^2);
    if steps0>0 && isequal(u0,nonlinearBicycleModel.recoveryInput(x0,model.previousInput,lane,reference,cfg,terminal))
        residual1=residual0(6:end);steps1=steps0-1; % the same closed loop, one hold later
    else
        [~,residual1,steps1]=nonlinearBicycleModel.recoveryValue(x1,u0,lane,reference,terminal,cfg);
    end
    delta=[1e-4;1e-4;1e-6;1e-5;1e-5;1e-6];jacobian=zeros(numel(residual1),6);
    for index=1:6
        x=x1;x(index)=x(index)+delta(index);
        [~,perturbed]=nonlinearBicycleModel.recoveryValue(x,u0,lane,reference,terminal,cfg,steps1);
        jacobian(:,index)=(perturbed-residual1)/delta(index);
    end
    [basis,factor]=qr(jacobian,0);projected=basis.'*residual1;
    orthogonal=max(0,sum(residual1.^2)-sum(projected.^2));scale=max(value,1);
    map=sparse(size(factor,1),nv);map(:,next)=factor/sqrt(scale);offset=projected/sqrt(scale);
    required=cfg.recovery.decreaseFraction*stage;
    constant=(value-required-orthogonal)/scale;modelConstant=orthogonal/scale;
    recovery=struct('active',true,'function',"recoveryCostToGo",'requiredDecrease',required, ...
        'rolloutSteps',steps0,'converged',converged);
end

function u=localClip(u,previous,cfg)
    % Only initialization braking is clipped; steering passes through unchanged.
    lower=[-Inf;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
    upper=[Inf;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    rate=[Inf;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    u=min(upper,max(lower,min(previous+rate,max(previous-rate,u))));
end

function [values,jacobian]=localSafetyRows(x,time,model)
    cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    q=localTargetAt(model,time);projection=laneGeometry.project(x(1:2),model.lane);
    preferred=[-sin(projection.heading);cos(projection.heading)];
    dual=predictiveSafetyGeometry.dualLinearization(x(1:3),shape,q(1:3),q(8:11),preferred);
    values=dual.value-cfg.collision.safetyMarginMeters;
    jacobian=[dual.jacobian,zeros(4,3)];
end

function beyond=localBeyondRange(x,time,model)
    beyond=isempty(model.target);
    if ~beyond,beyond=min(localSafetyRows(x,time,model))>model.cfg.collision.encounterRangeMeters-model.cfg.collision.safetyMarginMeters;end
end

function [margin,value,gradient]=localTerminalClearance(seed,count,model)
    cfg=model.cfg;index=model.sampleIndex+count;
    [margin,~,gradient]=terminalContinuation.separation(seed, ...
        localTargetAt(model,count*cfg.controller.sampleTime),model.frame,cfg,index);
    value=margin;if margin>0,return;end
    [leaving,rows,jacobian]=terminalContinuation.departure(seed,model.targetEpoch,index,cfg);
    if leaving>margin,margin=leaving;value=rows;gradient=jacobian;end
end

function [low,high]=localStateLimits(cfg)
    low=[max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor+1e-4);-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
    high=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
end

function q=localTargetAt(model,time)
    q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,model.sampleIndex*model.cfg.controller.sampleTime+time);
end
