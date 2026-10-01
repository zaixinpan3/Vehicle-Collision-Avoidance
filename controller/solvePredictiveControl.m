function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl Solve safety first, then CLF under the attained safety level.
% Zero safety slack does not terminate a still-positive CLF problem.
% The endpoint family supplies every newly appended input.
    if nargin<3,timer=tic;end
    cfg=model.cfg;solution=[];baseline=[];model.slackCap=Inf;
    search=struct('solverCalls',0,'source',"sequentialConvexification", ...
        'terminationReason',"iterationLimit",'sequentialIterations',{{}}, ...
        'initialization',"laneFeedbackRollout",'safetySlack',Inf,'shiftAvailable',false, ...
        'slackCap',Inf,'converged',false,'failures',strings(0,1), ...
        'clfInitialSlack',Inf,'clfStageAttempted',false,'clfStageCompleted',false,'clfLowerBound',false);
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
                for name=["stageSafety","stageHard","stageCost","stageCollision","interiorViolation"]
                    shifted.(name)=shifted.(name)(2:end);
                end
                baseline=localNominalEvaluation(anchor,model,shifted,size(shifted.inputs,2)+1,false);
            else
                baseline=localNominalEvaluation(anchor,model,[],1,false);
            end
            search.shiftAvailable=localFeasible(baseline,model);
        else
            search.initialization="restorationFromChangedEgoState";
        end
    else
        [anchor,model.terminal]=localSeed(model);
    end
    search.slackCap=model.slackCap;
    search.initializationCandidates=struct([]);
    if isempty(baseline),baseline=localNominalEvaluation(anchor,model);end
    if localFeasible(baseline,model)
        solution=baseline;search.source="feasibleInitialization";
        if search.shiftAvailable,search.source="retainedContinuation";end
    end
    % Screen bounded target-aware rollouts only when no transferable witness
    % exists. Tiny replay differences still repair the retained trajectory.
    fresh=~isstruct(previousState);
    if ~fresh,fresh=norm(model.initialState-expected,inf)>1e-4;end
    if isempty(solution) && ~isempty(model.target) && fresh
        for side=[-1,1]
            if toc(timer)>=cfg.solver.timeLimitSeconds,break;end
            trialModel=model;seedTimer=tic;
            try
                [inputs,trialModel.terminal]=localFlowSeed(trialModel,side,timer);
                if isempty(inputs),break;end
                trial=localNominalEvaluation(inputs,trialModel);
            catch exception
                if localDomainFailure(exception),continue;end
                rethrow(exception);
            end
            record=struct('side',side,'holds',size(inputs,2),'seconds',toc(seedTimer), ...
                'hard',trial.hard,'safety',trial.safety,'restorationMerit',trial.restorationMerit);
            if isempty(search.initializationCandidates),search.initializationCandidates=record;
            else,search.initializationCandidates(end+1)=record;
            end
            if localBetter(trial,baseline,model)
                baseline=trial;model=trialModel;search.initialization="movingTargetFlow";
            end
            if localFeasible(baseline,model)
                solution=baseline;search.source="feasibleInitialization";break;
            end
        end
    end
    search.clfInitialSlack=baseline.clfSlack;
    if ~isempty(solution)
        baseline=solution;model.terminal=solution.terminal;model.slackCap=0;search.slackCap=0;
        if cfg.clf.relaxationWeight*solution.clfSlack^2<=cfg.solver.optimalityTolerance
            search.converged=true;search.terminationReason="clfLowerBound";
            search.clfStageCompleted=true;search.clfLowerBound=true;
            search.safetySlack=solution.safety;search.elapsedSeconds=toc(timer);return;
        end
    end
    model.collisionRefinement=baseline.interiorViolation>0;
    radius=cfg.nonlinear.trustRadius;
    for iteration=1:cfg.nonlinear.maximumIterations
        if toc(timer)>=cfg.solver.timeLimitSeconds,search.terminationReason="timeLimit";break;end
        try
            [candidate,step]=localSequentialStep(baseline,model,radius,timer);
            search.solverCalls=search.solverCalls+step.calls;
            search.clfStageAttempted=search.clfStageAttempted || step.secondaryAttempted;
            step.iteration=iteration;step.trustRadius=radius;step.acceptedIterate=false;
            step.retainedAsWitness=false;step.nominalSafetySlack=Inf;step.nominalHardViolation=Inf;step.nominalRestorationMerit=Inf;
            if isempty(candidate) && toc(timer)>=cfg.solver.timeLimitSeconds
                search.terminationReason="timeLimit";search.sequentialIterations{end+1}=step;break;
            end
            if isempty(candidate)
                search.terminationReason=step.status;
                if step.status=="restorationInfeasible"
                    nextRadius=min(1,max(cfg.nonlinear.trustRadius,2*radius));
                    if nextRadius<=radius,search.sequentialIterations{end+1}=step;break;end
                    radius=nextRadius;
                elseif step.status=="restorationSolverFailure"
                    search.sequentialIterations{end+1}=step;break;
                else
                    radius=radius/2;
                end
                search.sequentialIterations{end+1}=step;
                if radius<1e-7,break;end
                continue;
            end
            refinement=model.collisionRefinement | candidate.interiorViolation>0;
            if any(refinement & ~model.collisionRefinement)
                model.collisionRefinement=refinement;
                baseline=localNominalEvaluation(baseline.inputs,model);
                radius=max(radius,cfg.nonlinear.trustRadius);
            end
            step.nominalSafetySlack=candidate.safety;step.nominalHardViolation=candidate.hard;step.nominalRestorationMerit=candidate.restorationMerit;
            if localFeasible(candidate,model)
                if localBetter(candidate,solution,model)
                    solution=candidate;search.source="sequentialConvexification";
                    step.acceptedIterate=true;step.retainedAsWitness=true;
                end
                lowerBound=cfg.clf.relaxationWeight*solution.clfSlack^2<=cfg.solver.optimalityTolerance;
                % A finite suboptimal conic result can complete this local
                % stage after nonlinear admission; its exit flag is retained.
                secondaryComplete=step.secondaryAccepted;
                if lowerBound || secondaryComplete
                    search.clfLowerBound=lowerBound;search.clfStageCompleted=true;
                    search.converged=true;search.terminationReason="clfStageComplete";
                    if lowerBound,search.terminationReason="clfLowerBound";end
                    search.sequentialIterations{end+1}=step;break;
                end
                if step.secondaryAttempted && ~step.secondaryAccepted
                    radius=radius/2;
                end
                baseline=solution;model.terminal=solution.terminal;
                model.slackCap=0;search.slackCap=0;
                search.sequentialIterations{end+1}=step;
                if radius<1e-7,search.terminationReason="smallTrustRegion";break;end
                continue;
            end
            if localBetter(candidate,baseline,model)
                baseline=candidate;step.acceptedIterate=true;
                model.terminal=candidate.terminal;
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
    if ~isempty(solution)
        model.terminal=solution.terminal;search.safetySlack=solution.safety;
    end
    search.elapsedSeconds=toc(timer);
end

function feasible=localFeasible(candidate,~)
    % Positive restoration slack may guide search, but never authorize input.
    feasible=~isempty(candidate) && candidate.hard==0 && candidate.safety==0;
end

function better=localBetter(candidate,baseline,model)
    % Compare actual nonlinear trajectories, including raw and polished points.
    better=false;if isempty(candidate),return;end
    if isempty(baseline),better=true;return;end
    feasible=localFeasible(candidate,model);previous=localFeasible(baseline,model);
    if feasible~=previous,better=feasible;return;end
    if feasible
        better=candidate.clfSlack<baseline.clfSlack ...
            || (candidate.clfSlack==baseline.clfSlack && candidate.cost<baseline.cost);
        return;
    end
    merit=candidate.restorationMerit;before=baseline.restorationMerit;
    better=merit<before || (merit==before && candidate.terminalViolation<baseline.terminalViolation) ...
        || (candidate.hard==0 && candidate.safety<=baseline.safety && candidate.cost<baseline.cost);
end

function yes=localDomainFailure(exception)
    yes=any(strcmp(exception.identifier,{'collisionAvoidanceController:nonlinearDomain', ...
        'collisionAvoidanceController:invalidTireOperatingPoint'}));
end

function candidate=localPolishEndpoint(candidate,model,timer)
    % A small shooting correction removes the nonlinear terminal defect of
    % an affine SCvx step. It is a search step: no corrected input is clipped,
    % and all nonlinear constraints and the PCBF budget are checked afterward.
    cfg=model.cfg;count=size(candidate.inputs,2);seed=model.terminal;working=candidate;
    tail=max(1,count-cfg.controller.horizonSteps);length=min(count,max(8,min(20,tail)));
    first=count-length+1;
    for iteration=1:5
        if localFeasible(candidate,model),return;end
        if toc(timer)>=cfg.solver.timeLimitSeconds,return;end
        deviation=[working.states(4:6,end);working.inputs(:,end)]-[seed.base(4:6);seed.reference.input];
        if norm(seed.quotientFactor*deviation)<=seed.radius,return;end
        sensitivity=zeros(6,2*length);x=working.states(:,first);
        for index=first:count
            [x,a,b]=nonlinearBicycleModel.sample(x,working.inputs(:,index),cfg);
            sensitivity=a*sensitivity;sensitivity(:,2*(index-first)+(1:2))=b;
        end
        memory=zeros(2,2*length);memory(:,end-1:end)=eye(2);
        tangent=seed.quotientFactor*[sensitivity(4:6,:);memory];
        correction=-pinv(tangent)*(seed.quotientFactor*deviation);
        improved=false;
        for exponent=0:5
            if toc(timer)>=cfg.solver.timeLimitSeconds,return;end
            inputs=working.inputs;inputs(:,first:end)=inputs(:,first:end)+reshape(correction,2,[])*2^-exponent;
            lo=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
            hi=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
            rate=cfg.controller.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
            if any(inputs<lo | inputs>hi,'all') || any(abs(diff([model.previousInput,inputs],1,2))>rate,'all')
                continue;
            end
            try
                trial=localNominalEvaluation(inputs,model,working,first);
            catch exception
                if localDomainFailure(exception),continue;end
                rethrow(exception);
            end
            if localBetter(trial,candidate,model),candidate=trial;end
            if localFeasible(candidate,model),return;end
            nextDeviation=[trial.states(4:6,end);inputs(:,end)]-[seed.base(4:6);seed.reference.input];
            if norm(seed.quotientFactor*nextDeviation)<norm(seed.quotientFactor*deviation)
                working=trial;improved=true;break;
            end
        end
        if ~improved,return;end
    end
end

function [inputs,seed]=localSeed(model)
    cfg=model.cfg;seed=model.terminal;reference=model.nominalReference;x=model.initialState;previous=model.previousInput;
    required=cfg.controller.horizonSteps+ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime);
    required=min(required,cfg.controller.maximumHorizonSteps);inputs=zeros(2,cfg.controller.maximumHorizonSteps);
    for index=1:size(inputs,2)
        if index<=required
            deviation=nonlinearBicycleModel.error(x,model.lane,reference);
            u=reference.input+reference.gain*deviation;
        else
            seed=terminalContinuation.fit(seed,[x;previous],model.sampleIndex+index-1);
            [~,~,deviation]=terminalContinuation.membership([x;previous],seed.epochIndex,seed);
            u=seed.reference.input+seed.gain*deviation;
        end
        u=localClip(u,previous,cfg);inputs(:,index)=u;
        x=nonlinearBicycleModel.sample(x,u,cfg);previous=u;
        if index<required,continue;end
        seed=terminalContinuation.fit(seed,[x;u],model.sampleIndex+index);
        value=terminalContinuation.membership([x;u],seed.epochIndex,seed);
        q=localTargetAt(model,index*cfg.controller.sampleTime);
        if value<0 && terminalContinuation.separation(seed,q,model.frame,cfg)>0
            inputs=inputs(:,1:index);return;
        end
    end
    % An uncertified endpoint is a search seed, not proof of infeasibility.
    % Its original endpoint conditions remain in restoration and admission.
end

function [inputs,seed]=localFlowSeed(model,side,timer)
    % The transported fluid velocity is a temporary guide. Limited feedback
    % inputs and the actual nonlinear model, not an ideal point path, define
    % the anchor used by dynamics, tires and rectangle separation constraints.
    cfg=model.cfg;seed=model.terminal;reference=model.nominalReference;x=model.initialState;previous=model.previousInput;
    h=cfg.controller.sampleTime;count=cfg.controller.maximumHorizonSteps;
    tire=modifiedFialaTire.parameters(cfg);
    required=min(count,cfg.controller.horizonSteps+ceil(cfg.nonlinear.recoveryHorizonSeconds/h));
    inputs=zeros(2,count);
    egoRadius=norm([cfg.vehicle.length;cfg.vehicle.width]/2)+norm(cfg.vehicle.rectangleOffset);
    radius=egoRadius+norm(model.target(8:9))+norm(model.target(10:11))+cfg.collision.safetyMarginMeters;
    station=[];completionStarted=false;
    for index=1:count
        if mod(index-1,16)==0 && toc(timer)>=cfg.solver.timeLimitSeconds,inputs=[];return;end
        time=(model.sampleIndex+index-1)*h;
        q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,time);
        projection=laneGeometry.project(x(1:2),model.lane,station);station=projection.station;
        direction=[cos(projection.heading);sin(projection.heading)];normal=[-direction(2);direction(1)];
        deviation=nonlinearBicycleModel.error(x,model.lane,reference);
        active=false;
        for preview=[0,.5,1,1.5,2,3]
            future=predictiveSafetyGeometry.targetFlow(model.targetEpoch,time+preview);
            if isfield(model.lane,'referenceCurve')
                position=laneGeometry.referencePose(station+cfg.referenceSpeed*preview,0,model.lane.referenceCurve);
            else
                position=projection.point+cfg.referenceSpeed*preview*direction;
            end
            if norm(position-future(1:2))<radius+1,active=true;break;end
        end
        if active && ~completionStarted
            yaw=q(3);rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];offset=rotation*q(10:11);
            omega=q(4)*sin(q(6))/q(7);
            translation=q(4)*[cos(yaw+q(6));sin(yaw+q(6))]+omega*[-offset(2);offset(1)];
            nominal=cfg.referenceSpeed*direction-.5*projection.lateralPosition*normal;
            circulation=side*.6*max(norm(nominal-translation),.5*cfg.referenceSpeed)/radius;
            velocity=predictiveSafetyGeometry.movingFlowVelocity(x(1:2),nominal,q(1:2)+offset, ...
                translation,radius*eye(2),zeros(2),circulation);
            heading=atan2(velocity(2),velocity(1));
            desiredSpeed=min(cfg.referenceSpeed,max(max(1.5,.5*cfg.referenceSpeed),norm(velocity)));
            deviation(1)=0;deviation(2)=atan2(sin(x(3)-heading),cos(x(3)-heading));
            deviation(3)=x(4)-desiredSpeed;
        end
        if index>required && (~active || completionStarted)
            completionStarted=true;
            seed=terminalContinuation.fit(seed,[x;previous],model.sampleIndex+index-1);
            [~,~,deviation]=terminalContinuation.membership([x;previous],seed.epochIndex,seed);
            u=seed.reference.input+seed.gain*deviation;
        else
            u=reference.input+reference.gain*deviation;
        end
        u(2)=min(.35,max(-.35,u(2)));u=localClip(u,previous,cfg);
        % Invert the adhesion-branch force fraction 1-(1-q)^3. A kinematic
        % steering cap can already saturate Fiala and erase its steering
        % sensitivity. This is a seed preference, not an optimizer constraint.
        slipLimit=atan(3*tire.longitudinalForceScale(1)*sqrt(1-u(2)^2) ...
            /tire.corneringStiffness(1)*(1-(1-.8)^(1/3)));
        zeroSlip=atan2(x(5)+cfg.vehicle.lf*x(6),max(x(4),cfg.model.scheduleSpeedFloor));
        u(1)=min(zeroSlip+slipLimit,max(zeroSlip-slipLimit,u(1)));
        u=localClip(u,previous,cfg);inputs(:,index)=u;
        x=nonlinearBicycleModel.sample(x,u,cfg);previous=u;
        if index<required,continue;end
        seed=terminalContinuation.fit(seed,[x;u],model.sampleIndex+index);
        membership=terminalContinuation.membership([x;u],seed.epochIndex,seed);
        future=predictiveSafetyGeometry.targetFlow(model.targetEpoch,time+h);
        separation=terminalContinuation.separation(seed,future,model.frame,cfg);
        if membership<0 && separation>0,break;end
    end
    inputs=inputs(:,1:index);
end

function u=localClip(u,previous,cfg)
    % Clipping is used only to initialize a numerical search, never to modify
    % the retained witness or the endpoint policy.
    lower=[-cfg.model.frontWheelSteeringAngleMaximum;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
    upper=[cfg.model.frontWheelSteeringAngleMaximum;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    u=min(upper,max(lower,min(previous+rate,max(previous-rate,u))));
end

function [candidate,info]=localSequentialStep(baseline,model,radius,timer)
    info=struct('calls',0,'safetyOptimum',Inf,'secondarySafety',Inf,'safetyCap',Inf, ...
        'clfSlack',Inf,'status',"infeasibleBounds",'safetyExitFlag',NaN,'secondaryExitFlag',NaN, ...
        'completionRestoration',false,'secondaryReturned',false,'secondaryAttempted',false, ...
        'secondaryAccepted',false,'primarySkipped',false,'clfBefore',baseline.clfSlack, ...
        'terminalPhaseMeters',model.terminal.phaseMeters,'primaryRawMerit',Inf, ...
        'primaryMerit',Inf,'secondaryRawMerit',Inf,'secondaryMerit',Inf, ...
        'restorationRawMerit',Inf,'restorationMerit',Inf,'candidateStage',"none",'hardRowInfeasible',false);candidate=[];
    anchor=baseline.inputs;cfg=model.cfg;count=size(anchor,2);prefix=cfg.controller.horizonSteps;reference=model.nominalReference;
    % A change of units keeps squared-slack epigraphs bounded when the
    % physical CLF residual is large. Nonlinear admission uses original units.
    clfScale=max(1,baseline.clfSlack);
    objectiveScale=max(1,baseline.cost);
    info.clfScale=clfScale;info.secondaryObjectiveScale=objectiveScale;
    ix=reshape(1:6*(count+1),6,[]);iu=reshape(ix(end)+(1:2*count),2,[]);
    is=iu(end)+(1:count);ic=is(end)+1;it=ic+1;ie=it+(1:2);nv=ie(end);
    equal=sparse(6*(count+1),nv);rhs=zeros(6*(count+1),1);equal(1:6,ix(:,1))=eye(6);
    rows=cell(1,4*count+6);bounds=cell(1,4*count+6);rowCount=0;
    costMatrix=sparse(7*count+1,nv);costOffset=zeros(7*count+1,1);
    lower=-Inf(nv,1);upper=Inf(nv,1);
    stateTrust=radius*[5;5;.5;5;3;1.5];inputTrust=radius*[.15;.25];
    lower(ix(:))=-repmat(stateTrust,count+1,1);upper(ix(:))=-lower(ix(:));
    lower(iu(:))=-repmat(inputTrust,count,1);upper(iu(:))=-lower(iu(:));lower([is,ic,it])=0;
    upper(is(prefix+1:end))=0;lower(ie)=0;upper(ie)=0;
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
        if model.collisionRefinement(index)
            for half=0:1
                base=x;if half==1,base=middle;end
                for duration=localInteriorTimes(cfg)
                    [interior,ai,bi]=nonlinearBicycleModel.sample(base,anchor(:,index),cfg,[],duration);
                    if half==1,bi=ai*bm+bi;ai=ai*am;end
                    map=sparse(6,nv);map(:,ix(:,index))=ai;map(:,iu(:,index))=bi;
                    [gi,ji]=localSafetyRows(interior,(index-1+half/2)*cfg.controller.sampleTime+duration,model);
                    r=-ji*map;r(:,is(index))=-1;
                    rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=gi;
                end
            end
        end
        [e,j]=nonlinearBicycleModel.errorLinearization(next,model.lane,reference);
        jx=ix(:,index+1);ci=7*(index-1)+(1:7);
        costMatrix(ci(1:5),jx)=reference.factor*j/sqrt(count);
        costOffset(ci(1:5))=reference.factor*e/sqrt(count);
        costMatrix(ci(6:7),iu(:,index))=.1*diag(sqrt([cfg.clf.frontWheelSteeringAngleWeight, ...
            cfg.clf.brakingRatioWeight]))/sqrt(count);
        lower(jx(4:6))=max(lower(jx(4:6)),physicalLower-next(4:6));
        upper(jx(4:6))=min(upper(jx(4:6)),physicalUpper-next(4:6));
        if index==1
            clfMap=sparse(5,nv);clfMap(:,jx)=reference.factor*j/sqrt(clfScale);
            clfOffset=reference.factor*e/sqrt(clfScale);
        end
        x=next;
    end
    % The same 2-norm endpoint membership is used here and in the nonlinear
    % rollout. There is no inscribed-polytope/ellipsoid mismatch.
    endIndex=model.sampleIndex+count;y=[x;anchor(:,end)];
    [seed,poseJacobian]=terminalContinuation.fit(model.terminal,y,endIndex);
    deviation=y(4:8)-[seed.base(4:6);seed.reference.input];
    map=sparse(8,nv);map(1:6,ix(:,end))=eye(6);map(7:8,iu(:,end))=eye(2);
    elastic=zeros(nv,1);elastic(ie(1))=1;
    endpoint=secondordercone(seed.quotientFactor*map(4:8,:),-seed.quotientFactor*deviation,elastic,-seed.radius);
    clfConstant=(1-cfg.nonlinear.clfDecay)*norm(reference.factor*initialError)^2/clfScale;
    clfAxis=sparse(1,nv);clfAxis(ic)=1;
    clfCone=secondordercone([2*clfMap;clfAxis],[-2*clfOffset;1-clfConstant],clfAxis.',-1-clfConstant);
    endpoint=[endpoint,clfCone];
    if ~isempty(model.target)
        q=localTargetAt(model,count*cfg.controller.sampleTime);
        [separation,~,poseGradient]=terminalContinuation.separation(seed,q,model.frame,cfg,endIndex);
        r=-poseGradient*poseJacobian*map;r(ie(2))=-1;
        rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=separation;
    end
    [g,j]=localSafetyRows(x,count*cfg.controller.sampleTime,model);
    r=sparse(size(j,1),nv);r(:,ix(:,end))=-j;r(:,ie(2))=-1;rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
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
    % A row already impossible within variable bounds cannot be repaired by
    % the hard conic solve. Proceed directly to elastic restoration.
    [row,column,value]=find(a);contribution=value.*lower(column);
    negative=value<0;contribution(negative)=value(negative).*upper(column(negative));
    rowMinimum=accumarray(row,contribution,[size(a,1),1],@sum,0);
    info.hardRowInfeasible=any(rowMinimum>b+1e-10*max(1,abs(b)));
    if localFeasible(baseline,model)
        first=zeros(nv,1);first(ic)=baseline.clfSlack/clfScale;
        candidate=baseline;info.primarySkipped=true;info.candidateStage="incumbent";
        upper(is)=0;upper(ic)=baseline.clfSlack/clfScale;
    else
        first=[];flag=-2;
        if ~info.hardRowInfeasible
            [first,~,flag]=coneprog(objective,endpoint,a,b,equal,rhs,lower,upper,options);info.calls=1;
        end
        info.safetyExitFlag=flag;
        [candidate,info.primaryRawMerit]=localConicEvaluation(first,iu,baseline,model,timer);
        if ~isempty(candidate)
            info.candidateStage="primary";info.primaryMerit=candidate.restorationMerit;
            if localFeasible(candidate,model) || localBetter(candidate,baseline,model)
                info.status="primaryProgress";if localFeasible(candidate,model),info.status="primaryFeasible";end
                info.terminalPhaseMeters=candidate.terminalPhaseMeters;
                info.secondarySafety=candidate.safety;info.clfSlack=candidate.clfSlack;return;
            end
        end
        if flag<=0 || isempty(first)
            % Endpoint elastic variables are enabled only in the search problem.
            % Nonlinear witness admission still checks every original hard row.
            upper(is(prefix+1:end))=Inf;upper(ie)=Inf;
            restoration=objective;restoration(is(prefix+1:end))=100;restoration(ie)=100;
            remaining=cfg.solver.timeLimitSeconds-toc(timer);
            if remaining<=0,info.status="timeLimit";return;end
            options.MaxTime=remaining;
            [first,~,flag]=coneprog(restoration,endpoint,a,b,equal,rhs,lower,upper,options);info.calls=info.calls+1;
            info.completionRestoration=true;info.status="restoringCompletion";
            [trial,info.restorationRawMerit]=localConicEvaluation(first,iu,baseline,model,timer);
            if ~isempty(trial)
                info.restorationMerit=trial.restorationMerit;
                if localBetter(trial,candidate,model),candidate=trial;info.candidateStage="restoration";end
            end
            if isempty(candidate)
                if toc(timer)>=cfg.solver.timeLimitSeconds,info.status="timeLimit";
                elseif flag==-2,info.status="restorationInfeasible";
                else,info.status="restorationSolverFailure";
                end
            else
                info.terminalPhaseMeters=candidate.terminalPhaseMeters;
                info.secondarySafety=candidate.safety;info.clfSlack=candidate.clfSlack;
            end
            return;
        end
    end
    if toc(timer)>=cfg.solver.timeLimitSeconds,info.status="timeLimit";return;end
    info.safetyOptimum=sum(max(0,first(is(1:prefix))));
    info.safetyCap=min(model.slackCap,info.safetyOptimum+cfg.solver.lexicographicTieTolerance);
    if info.primarySkipped,info.safetyCap=0;end
    a=[a;objective.'];b=[b;info.safetyCap];
    costMatrix(end,ic)=sqrt(cfg.clf.relaxationWeight)*clfScale;
    costMatrix=costMatrix/sqrt(objectiveScale);
    costOffset=costOffset/sqrt(objectiveScale);
    % A sum of small squared-norm epigraphs is exactly the same quadratic
    % objective, without one horizon-wide cone coupling every stage.
    groups=count+1;expanded=nv+groups;
    baseCones=numel(endpoint);cones=repmat(endpoint(1),1,groups+baseCones);
    for index=1:baseCones
        cones(index)=secondordercone([endpoint(index).A,sparse(size(endpoint(index).A,1),groups)], ...
            endpoint(index).b,[endpoint(index).d;zeros(groups,1)],endpoint(index).gamma);
    end
    epigraph=zeros(groups,1);
    for group=1:groups
        if group<=count,selected=7*(group-1)+(1:7);else,selected=7*count+1;end
        r=sparse(1,expanded);r(nv+group)=1;
        cones(group+baseCones)=secondordercone([[2*costMatrix(selected,:),sparse(numel(selected),groups)];r], ...
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
    info.secondaryAttempted=true;
    [second,~,flag]=coneprog(objective,cones,a,b,equal,rhs,lower,upper,options);
    info.calls=info.calls+1;info.secondaryExitFlag=flag;
    % Keep the best nonlinear candidate across both conic stages and polish.
    info.secondaryReturned=~isempty(second) && all(isfinite(second));
    if info.secondaryReturned,first=second;end
    info.solverResidual=max([0;a*first-b;abs(equal*first-rhs);lower-first;first-upper]);
    info.secondarySafety=sum(max(0,first(is(1:prefix))));info.clfSlack=clfScale*max(0,first(ic));info.status="solved";
    [trial,info.secondaryRawMerit]=localConicEvaluation(second,iu,baseline,model,timer);
    if ~isempty(trial)
        info.secondaryMerit=trial.restorationMerit;
        info.secondaryAccepted=localFeasible(trial,model) ...
            && (~localFeasible(baseline,model) || trial.clfSlack<=baseline.clfSlack);
        if localBetter(trial,candidate,model),candidate=trial;info.candidateStage="secondary";end
    end
    if ~isempty(candidate),info.terminalPhaseMeters=candidate.terminalPhaseMeters;end
end

function [candidate,rawMerit]=localConicEvaluation(point,iu,baseline,model,timer)
    candidate=[];rawMerit=Inf;
    if isempty(point) || any(~isfinite(point)) || toc(timer)>=model.cfg.solver.timeLimitSeconds,return;end
    delta=reshape(point(iu(:)),2,[]);
    feasibleAnchor=localFeasible(baseline,model);
    for exponent=0:20
        if toc(timer)>=model.cfg.solver.timeLimitSeconds,return;end
        scale=2^-exponent;
        try
            trial=localNominalEvaluation(baseline.inputs+scale*delta,model);
            if scale==1,rawMerit=trial.restorationMerit;end
            if localBetter(trial,candidate,model),candidate=trial;end
            if localFeasible(candidate,model) && (~localFeasible(baseline,model) || candidate.clfSlack<=baseline.clfSlack),return;end
            if ~feasibleAnchor
                trial=localPolishEndpoint(trial,model,timer);
                if localBetter(trial,candidate,model),candidate=trial;end
            end
            if localBetter(candidate,baseline,model),return;end
        catch exception
            if ~localDomainFailure(exception),rethrow(exception);end
        end
    end
end

function [values,jacobian]=localSafetyRows(x,time,model)
    cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    count=4*~isempty(model.target);values=zeros(count,1);
    if nargout>1,jacobian=zeros(count,6);end
    if count>0
        projection=laneGeometry.project(x(1:2),model.lane);
        preferred=[-sin(projection.heading);cos(projection.heading)];
        q=localTargetAt(model,time);
        dual=predictiveSafetyGeometry.dualLinearization(x(1:3),shape,q(1:3),q(8:11),preferred);
        values(1:4)=dual.value-cfg.collision.safetyMarginMeters;
        if nargout>1,jacobian(1:4,1:3)=dual.jacobian;end
    end
end

function evaluation=localNominalEvaluation(inputs,model,cached,first,refit)
    % Reuse only an exactly unchanged input/state prefix in the same problem.
    % A shifted cache is supplied only after the nominal successor check.
    cfg=model.cfg;count=size(inputs,2);prefix=cfg.controller.horizonSteps;
    if nargin<5,refit=true;end
    seed=model.terminal;
    [low,high]=localStateLimits(cfg);halfDuration=cfg.controller.sampleTime/2;
    states=zeros(6,count+1);states(:,1)=model.initialState;
    stageSafety=zeros(1,count);stageHard=stageSafety;stageCost=stageSafety;
    stageCollision=Inf(1,count);
    if nargin<3 || isempty(cached),first=1;
    else
        assert(isequal(inputs(:,1:first-1),cached.inputs(:,1:first-1)) ...
            && isequal(model.initialState,cached.states(:,1)), ...
            'collisionAvoidanceController:invalidEvaluationCache','The cached trajectory prefix changed.');
        states(:,1:first)=cached.states(:,1:first);
        stageSafety(1:first-1)=cached.stageSafety(1:first-1);
        stageHard(1:first-1)=cached.stageHard(1:first-1);
        stageCost(1:first-1)=cached.stageCost(1:first-1);
        stageCollision(1:first-1)=cached.stageCollision(1:first-1);
    end
    x=states(:,first);
    for index=first:count
        values=localSafetyRows(x,(index-1)*cfg.controller.sampleTime,model);
        middle=nonlinearBicycleModel.sample(x,inputs(:,index),cfg,[],halfDuration);
        middleValues=localSafetyRows(middle,(index-.5)*cfg.controller.sampleTime,model);
        stageSafety(index)=max([0;-values;-middleValues]);
        if ~isempty(model.target)
            stageCollision(index)=min([values(1:4);middleValues(1:4)]);
        end
        x=nonlinearBicycleModel.sample(middle,inputs(:,index),cfg,[],halfDuration);states(:,index+1)=x;
        stageHard(index)=max([0;low-x(4:6);x(4:6)-high;low-middle(4:6);middle(4:6)-high]);
        deviation=nonlinearBicycleModel.error(x,model.lane,model.nominalReference);
        % Input proximity belongs to the local search step, not an absolute
        % input penalty in the retained nonlinear trajectory's nominal cost.
        stageCost(index)=norm(model.nominalReference.factor*deviation)^2;
    end
    rate=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    lo=[-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    hi=[cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    endIndex=model.sampleIndex+count;
    if refit,seed=terminalContinuation.fit(seed,[x;inputs(:,end)],endIndex);end
    [~,~,deviation]=terminalContinuation.membership([x;inputs(:,end)],endIndex,seed);
    separation=terminalContinuation.separation(seed,localTargetAt(model,count*cfg.controller.sampleTime),model.frame,cfg,endIndex);
    terminalViolation=max(0,norm(seed.factor*deviation)-seed.radius);
    terminalGeometry=max([0;-separation;-localSafetyRows(x,count*cfg.controller.sampleTime,model)]);
    physical=max([0;low-model.initialState(4:6);model.initialState(4:6)-high;stageHard.'; ...
        reshape(lo-inputs,[],1);reshape(inputs-hi,[],1); ...
        reshape(abs(diff([model.previousInput,inputs],1,2))-rate,[],1)]);
    hard=max([physical;stageSafety(prefix+1:end).';terminalViolation;terminalGeometry]);
    interiorViolation=zeros(1,count);
    if nargin>=3 && ~isempty(cached) && isfield(cached,'interiorViolation')
        interiorViolation(1:first-1)=cached.interiorViolation(1:first-1);
    end
    refinement=false(1,count);
    if isfield(model,'collisionRefinement'),refinement=model.collisionRefinement;end
    checkWitness=hard==0 && sum(stageSafety(1:prefix))<=model.slackCap;
    for index=1:count
        time=(index-1)*cfg.controller.sampleTime;
        if ~(refinement(index) || (checkWitness && localCloseEncounter(stageCollision(index),time,model))),continue;end
        middle=nonlinearBicycleModel.sample(states(:,index),inputs(:,index),cfg,[],halfDuration);
        for half=0:1
            base=states(:,index);if half==1,base=middle;end
            for duration=localInteriorTimes(cfg)
                interior=nonlinearBicycleModel.sample(base,inputs(:,index),cfg,[],duration);
                values=localSafetyRows(interior,time+half*halfDuration+duration,model);
                violation=max([0;-values]);interiorViolation(index)=max(interiorViolation(index),violation);
                stageSafety(index)=max(stageSafety(index),violation);
                stageCollision(index)=min([stageCollision(index);values]);
            end
        end
    end
    hard=max([hard;stageSafety(prefix+1:end).']);
    restorationMerit=sum(stageSafety(1:prefix))+100*(sum(stageSafety(prefix+1:end)) ...
        +terminalViolation+terminalGeometry+physical);
    reference=model.nominalReference;before=nonlinearBicycleModel.error(states(:,1),model.lane,reference);
    after=nonlinearBicycleModel.error(states(:,2),model.lane,reference);
    v0=norm(reference.factor*before)^2;v1=norm(reference.factor*after)^2;clf=max(0,v1-(1-cfg.nonlinear.clfDecay)*v0);
    slacks=stageSafety(1:prefix);
    evaluation=struct('inputs',inputs,'states',states,'safety',sum(slacks),'stageSlacks',slacks, ...
        'hard',hard,'clfSlack',clf,'clfInitialValue',v0,'clfNextValue',v1, ...
        'minimumCollisionMargin',min(stageCollision),'terminalPhaseMeters',seed.phaseMeters,'terminal',seed, ...
        'terminalSeparationMargin',separation, ...
        'terminalViolation',terminalViolation,'restorationMerit',restorationMerit,'interiorViolation',interiorViolation, ...
        'cost',sum(stageCost)/count+cfg.clf.relaxationWeight*clf^2, ...
        'stageSafety',stageSafety,'stageHard',stageHard,'stageCost',stageCost, ...
        'stageCollision',stageCollision);
end

function [low,high]=localStateLimits(cfg)
    low=[max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor+1e-4);-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
    high=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
end

function times=localInteriorTimes(cfg)
    % Reuse interior nodes of the defining half-hold RK4 mesh.
    count=max(1,ceil(cfg.controller.sampleTime/(2*cfg.nonlinear.integrationStep)));
    times=(1:count-1)*(cfg.controller.sampleTime/(2*count));
end

function close=localCloseEncounter(margin,time,model)
    close=false;if isempty(model.target),return;end
    cfg=model.cfg;h=cfg.controller.sampleTime;q=localTargetAt(model,time);
    egoRadius=norm([cfg.vehicle.length;cfg.vehicle.width]/2)+norm(cfg.vehicle.rectangleOffset);
    targetRadius=norm(q(8:9))+norm(q(10:11));speed=abs(q(4))+abs(q(5))*h;
    travel=h*(hypot(cfg.model.speedMaximum,cfg.model.lateralVelocityMaximum) ...
        +cfg.model.yawRateMaximum*egoRadius+speed*(1+abs(sin(q(6))/q(7))*targetRadius));
    close=margin<=travel;
end

function q=localTargetAt(model,time)
    h=model.cfg.controller.sampleTime;
    count=max(1,ceil(h/(2*model.cfg.nonlinear.integrationStep)));
    tick=2*count*model.sampleIndex+round(2*count*time/h);
    if mod(tick,count)==0,absoluteTime=(tick/count)*(h/2);
    else,absoluteTime=tick*(h/(2*count));
    end
    q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,absoluteTime);
end
