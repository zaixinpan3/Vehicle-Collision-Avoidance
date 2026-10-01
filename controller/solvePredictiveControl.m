function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl One linearization, PCBF slack, then CLF slack.
% Returned states belong to the affine prediction, not a nonlinear replay.
% Solver exit flags are recorded, not used to admit or reject returned vectors.
% There is no post-solve constraint audit, correction or outer search.
    if nargin<3,timer=tic;end
    cfg=model.cfg;solution=[];
    [anchor,initialization]=localInitialization(model,previousState);
    [problem,model]=localFormulate(anchor,model);
    search=struct('solverCalls',0,'source',"twoStageConvexOptimization", ...
        'initialization',initialization,'returned',false,'converged',false,'terminationReason',"timeLimit", ...
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
    inputs=anchor.inputs+reshape(second(problem.inputIndices),size(anchor.inputs));
    states=anchor.states+reshape(second(problem.stateIndices),size(anchor.states));
    slacks=max(0,second(problem.slackIndices));rho=problem.clfScale*max(0,second(problem.clfIndex));
    nextValue=norm(problem.clfMap*second+problem.clfOffset)^2*problem.clfScale;
    % Fitting the free pose only packages the endpoint; it is not an admission test.
    model.terminal=terminalContinuation.fit(model.terminal,[states(:,end);inputs(:,end)], ...
        model.sampleIndex+size(inputs,2));
    solution=struct('inputs',inputs,'states',states,'stageSlacks',slacks.', ...
        'safety',sum(slacks),'hard',NaN,'clfSlack',rho, ...
        'clfInitialValue',problem.initialClfValue,'clfNextValue',nextValue, ...
        'minimumCollisionMargin',NaN,'terminalSeparationMargin',NaN, ...
        'encounterExit',problem.encounterExit,'affineValidationPerformed',false,'nonlinearValidationPerformed',false);
    model.linearization=anchor;
    search.stages(2).value=rho;search.clfStageCompleted=true;search.returned=true;
    search.converged=all([search.stages.exitFlag]>0);
    search.clfLowerBound=rho<=cfg.solver.feasibilityTolerance;
    search.terminationReason="twoStagesReturned";search.elapsedSeconds=toc(timer);
end

function [anchor,source]=localInitialization(model,previous)
    cfg=model.cfg;
    count=min(cfg.controller.maximumHorizonSteps,cfg.controller.horizonSteps ...
        +ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime));
    source="movingTargetFlow";
    if isempty(model.target),source="laneFeedbackRollout";end
    if isstruct(previous)
        states=previous.stateTrajectory(:,2:end);inputs=previous.inputTrajectory(:,2:end);
        usable=size(states,2)==size(inputs,2)+1 && ~isempty(inputs) ...
            && all(isfinite([states(:);inputs(:)])) ...
            && all(states(4,:)>cfg.model.scheduleSpeedFloor) && all(abs(inputs(2,:))<1);
        if usable
            inputs=inputs(:,1:min(count,size(inputs,2)));states=states(:,1:size(inputs,2)+1);
            while size(inputs,2)<count
                u=localClip(model.terminal.reference.input,inputs(:,end),cfg);
                states(:,end+1)=nonlinearBicycleModel.sample(states(:,end),u,cfg);
                inputs(:,end+1)=u;
            end
            anchor=struct('inputs',inputs,'states',states);source="shiftedLinearization";return;
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
        u(2)=min(.35,max(-.35,u(2)));u=localClip(u,previous,cfg);
        slipLimit=atan(3*tire.longitudinalForceScale(1)*sqrt(1-u(2)^2) ...
            /tire.corneringStiffness(1)*(1-(1-.8)^(1/3)));
        zeroSlip=atan2(x(5)+cfg.vehicle.lf*x(6),max(x(4),cfg.model.scheduleSpeedFloor));
        u(1)=min(zeroSlip+slipLimit,max(zeroSlip-slipLimit,u(1)));u=localClip(u,previous,cfg);
        inputs(:,index)=u;x=nonlinearBicycleModel.sample(x,u,cfg);states(:,index+1)=x;previous=u;
        exited=exited || localBeyondRange(x,index*h,model);
    end
    anchor=struct('inputs',inputs,'states',states);
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
    inputLower=[-cfg.model.frontWheelSteeringAngleMaximum;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
    inputUpper=[cfg.model.frontWheelSteeringAngleMaximum;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
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
    clfConstant=(1-cfg.nonlinear.clfDecay)*v0/clfScale;clfAxis=sparse(1,nv);clfAxis(ic)=1;
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
        'initialClfValue',v0,'initialClfSlack',max(0,(norm(clfOffset)^2-clfConstant)*clfScale), ...
        'terminalA',terminalA,'terminalB',terminalB,'encounterExit',departure);
end

function u=localClip(u,previous,cfg)
    % Only initialization controls are clipped; optimized controls are returned directly.
    lower=[-cfg.model.frontWheelSteeringAngleMaximum;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
    upper=[cfg.model.frontWheelSteeringAngleMaximum;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
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
