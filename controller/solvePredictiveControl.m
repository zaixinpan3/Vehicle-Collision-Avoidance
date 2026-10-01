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
        'clfInitialSlack',Inf,'clfStageAttempted',false,'clfStageCompleted',false,'clfLowerBound',false, ...
        'clfStationary',false,'clfCorrectionAttempted',false);
    if isstruct(previousState)
        model.terminal=previousState.terminal;
        anchor=previousState.inputTrajectory(:,2:end);
        warmStates=previousState.stateTrajectory(:,2:end);
        expected=previousState.stateTrajectory(:,2);
        sameState=isequal(model.initialState,expected) && isequal(model.previousInput,previousState.appliedInput);
        minimumHorizon=min(cfg.controller.maximumHorizonSteps,cfg.controller.horizonSteps ...
            +ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime));
        if size(anchor,2)<minimumHorizon
            y=[previousState.stateTrajectory(:,end);previousState.inputTrajectory(:,end)];
            oldEnd=previousState.sampleIndex+size(previousState.inputTrajectory,2);
            while size(anchor,2)<minimumHorizon
                appended=terminalContinuation.control(y,oldEnd,model.terminal);
                anchor(:,end+1)=appended;
                y=[nonlinearBicycleModel.sample(y(1:6),appended,cfg);appended];oldEnd=oldEnd+1;
                warmStates(:,end+1)=y(1:6);
            end
        end
        search.initialization="shiftedContinuation";
        if sameState
            model.slackCap=sum(previousState.witness.stageSlacks(2:end));
            if isfield(previousState.witness,'stageCost')
                shifted=previousState.witness;
                shifted.inputs=shifted.inputs(:,2:end);shifted.states=shifted.states(:,2:end);
                for name=["stageSafety","stageHard","stageCost","stageCollision","interiorViolation","collisionNodes","collisionTimes","intervalCertified"]
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
    if isempty(baseline)
        try
            baseline=localNominalEvaluation(anchor,model);
        catch exception
            if ~isstruct(previousState) || ~localDomainFailure(exception),rethrow(exception);end
            reference=struct('states',warmStates,'inputs',anchor);
            anchor=localStabilizedInputs(anchor,warmStates,reference,model);
            baseline=localNominalEvaluation(anchor,model);
        end
    end
    if isstruct(previousState) && ~localFeasible(baseline,model) && toc(timer)<cfg.solver.timeLimitSeconds
        % Repair a transferred iterate before rebuilding a full conic model.
        % This preserves the issued first input only when the entire repaired
        % trajectory passes the same nonlinear and interval admission.
        [repaired,calls]=localRetractConstraints(baseline,model,timer);
        search.solverCalls=search.solverCalls+calls;
        if localBetter(repaired,baseline,model),baseline=repaired;model.terminal=repaired.terminal;end
    end
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
        if ~localClfSatisfied(solution) && toc(timer)<cfg.solver.timeLimitSeconds
            % Restore the shifted iterate onto the CLF sublevel and endpoint
            % before building another affine program. A zero-slack admitted
            % point already attains the global nonnegative CLF lower bound.
            search.clfStageAttempted=true;search.clfCorrectionAttempted=true;
            trial=localPolishClf(baseline,model,timer);
            [trial,calls]=localRetractConstraints(trial,model,timer);search.solverCalls=search.solverCalls+calls;
            if localFeasible(trial,model) && localBetter(trial,solution,model)
                solution=trial;baseline=trial;model.terminal=trial.terminal;
                search.source="nonlinearConstraintCorrection";
            end
        end
        lowerBound=localClfSatisfied(solution);
        stationary=false; % A positive first-step minimum still needs predictive cost refinement.
        if lowerBound || stationary
            search.converged=true;search.terminationReason="clfStationary";
            if lowerBound,search.terminationReason="clfLowerBound";end
            search.clfStageCompleted=true;search.clfLowerBound=lowerBound;search.clfStationary=stationary;
            search.safetySlack=solution.safety;search.elapsedSeconds=toc(timer);return;
        end
    end
    model.collisionRefinement=baseline.interiorViolation>0;model.collisionTimes=baseline.collisionTimes;
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
            for node=1:numel(candidate.collisionTimes)
                model.collisionTimes{node}=reshape(union(model.collisionTimes{node},candidate.collisionTimes{node}),1,[]);
            end
            if any(refinement & ~model.collisionRefinement)
                model.collisionRefinement=refinement;
                baseline=localNominalEvaluation(baseline.inputs,model);
                radius=max(radius,cfg.nonlinear.trustRadius);
            end
            step.nominalSafetySlack=candidate.safety;step.nominalHardViolation=candidate.hard;step.nominalRestorationMerit=candidate.restorationMerit;
            step.nominalClfSlack=candidate.clfSlack;
            if localFeasible(candidate,model)
                if localBetter(candidate,solution,model)
                    solution=candidate;search.source="sequentialConvexification";
                    step.acceptedIterate=true;step.retainedAsWitness=true;
                end
                if ~step.secondaryAttempted && ~localClfSatisfied(solution)
                    search.clfStageAttempted=true;search.clfCorrectionAttempted=true;
                    corrected=localPolishClf(solution,model,timer);
                    [corrected,calls]=localRetractConstraints(corrected,model,timer);search.solverCalls=search.solverCalls+calls;
                    if localFeasible(corrected,model) && localBetter(corrected,solution,model)
                        solution=corrected;search.source="nonlinearConstraintCorrection";
                    end
                end
                lowerBound=localClfSatisfied(solution);
                % A feasible improvement is not a completed CLF stage. A
                % positive residual may stop only at local model stationarity.
                stationary=step.relaxedClfAttempted && step.secondaryExitFlag>0 ...
                    && abs(step.clfBefore-step.clfSlack)<=cfg.solver.optimalityTolerance*step.clfScale ...
                    && abs(solution.clfSlack-step.clfBefore)<=cfg.solver.optimalityTolerance*step.clfScale;
                stationary=step.nominalCostAttempted && (stationary || (~lowerBound && localClfStationary(solution,model)));
                if lowerBound || stationary
                    search.clfLowerBound=lowerBound;search.clfStageCompleted=true;search.clfStationary=stationary;
                    search.converged=true;search.terminationReason="clfStationary";
                    if lowerBound,search.terminationReason="clfLowerBound";end
                    search.sequentialIterations{end+1}=step;break;
                end
                if step.secondaryAttempted
                    predicted=step.clfBefore-step.clfSlack;
                    actual=step.clfBefore-solution.clfSlack;
                    if ~step.secondaryAccepted || (predicted>cfg.solver.optimalityTolerance*step.clfScale && actual<.25*predicted)
                        radius=radius/2;
                    end
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
    feasible=~isempty(candidate) && candidate.hard==0 && candidate.safety==0 ...
        && all(candidate.intervalCertified);
end

function satisfied=localClfSatisfied(candidate)
    % Only floating-point roundoff is ignored; this is not a squared-penalty
    % stopping tolerance that could mask a material positive CLF relaxation.
    tolerance=64*eps(max([1,candidate.clfInitialValue,candidate.clfNextValue]));
    satisfied=candidate.clfSlack<=tolerance;
end

function stationary=localClfStationary(candidate,model)
    % A feasible one-step KKT point is also stationary on the more restricted
    % predictive set. This is local stationarity, not a global positive lower bound.
    cfg=model.cfg;u=candidate.inputs(:,1);reference=model.nominalReference;
    [next,~,b]=nonlinearBicycleModel.sample(model.initialState,u,cfg);
    [e,j]=nonlinearBicycleModel.errorLinearization(next,model.lane,reference);
    map=reference.factor*j*b;weighted=reference.factor*e;
    gradient=map.'*weighted/max(1,norm(map,'fro')*norm(weighted));
    projected=u-localClip(u-gradient,model.previousInput,cfg);
    stationary=norm(projected,inf)<=cfg.solver.optimalityTolerance;
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

function candidate=localPolishClf(candidate,model,timer)
    % Restore the actual one-step CLF sublevel before endpoint retraction.
    % Analytic Gauss-Newton directions modify only u_0; this is a numerical
    % correction inside the same predictive solve, never an issued fallback.
    cfg=model.cfg;reference=model.nominalReference;input=candidate.inputs(:,1);
    anchor=input;trust=cfg.nonlinear.trustRadius*[.15;.25];
    value=candidate.clfNextValue;limit=(1-cfg.nonlinear.clfDecay)*candidate.clfInitialValue;
    tolerance=64*eps(max([1,value,candidate.clfInitialValue]));
    for iteration=1:6
        if value<=limit+tolerance || toc(timer)>=cfg.solver.timeLimitSeconds,break;end
        [next,~,b]=nonlinearBicycleModel.sample(model.initialState,input,cfg);
        [e,j]=nonlinearBicycleModel.errorLinearization(next,model.lane,reference);
        weighted=reference.factor*e;map=reference.factor*j*b;
        direction=-pinv(map)*weighted;
        projected=map*direction;aa=projected.'*projected;
        bb=2*weighted.'*projected;cc=weighted.'*weighted-limit;
        discriminant=bb^2-4*aa*cc;
        if aa>0 && bb<0 && discriminant>=0
            % The shorter intersection reaches the linearized CLF sublevel
            % without requesting the full least-squares minimizer.
            fraction=2*cc/(-bb+sqrt(discriminant));
            direction=direction*min(1,max(0,fraction));
        end
        improved=false;
        for exponent=0:8
            trial=min(anchor+trust,max(anchor-trust,input+2^-exponent*direction));
            trial=localClip(trial,model.previousInput,cfg);
            try
                next=nonlinearBicycleModel.sample(model.initialState,trial,cfg);
                nextValue=norm(reference.factor*nonlinearBicycleModel.error(next,model.lane,reference))^2;
            catch exception
                if localDomainFailure(exception),continue;end
                rethrow(exception);
            end
            if nextValue<value
                input=trial;value=nextValue;improved=true;break;
            end
        end
        if ~improved,break;end
    end
    if isequal(input,candidate.inputs(:,1)),return;end
    inputs=candidate.inputs;inputs(:,1)=input;
    try
        candidate=localNominalEvaluation(inputs,model);
    catch exception
        if ~localDomainFailure(exception),rethrow(exception);end
    end
end

function [candidate,calls]=localRetractConstraints(candidate,model,timer)
    calls=0;
    % A second-order correction preserves u_0 while repairing the complete
    % nonlinear trajectory, including contacts earlier than the endpoint tail.
    candidate=localPolishEndpoint(candidate,model,timer);
    if localFeasible(candidate,model) || candidate.stageSafety(1)>0 || candidate.stageHard(1)>0,return;end
    cfg=model.cfg;count=size(candidate.inputs,2);nv=2*(count-1);
    if nv==0,return;end
    working=candidate;
    weights=repmat([cfg.clf.frontWheelSteeringAngleWeight;cfg.clf.brakingRatioWeight],count-1,1);
    options=optimoptions('quadprog','Display','off','Algorithm','active-set', ...
        'MaxIterations',100,'ConstraintTolerance',cfg.solver.constraintTolerance, ...
        'OptimalityTolerance',cfg.solver.optimalityTolerance);
    for iteration=1:5
        if toc(timer)>=cfg.solver.timeLimitSeconds,return;end
        [a,b,equal,rhs]=localCorrectionProblem(working,model);
        lo=[-cfg.model.frontWheelSteeringAngleMaximum;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
        hi=[cfg.model.frontWheelSteeringAngleMaximum;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
        lower=reshape(lo-working.inputs(:,2:end),[],1);upper=reshape(hi-working.inputs(:,2:end),[],1);
        [direction,~,flag]=quadprog(spdiags(weights,0,nv,nv),zeros(nv,1),a,b,equal,rhs,lower,upper,zeros(nv,1),options);
        calls=calls+1;
        if flag<=0 || isempty(direction) || any(~isfinite(direction)),return;end
        improved=false;
        for exponent=0:8
            if toc(timer)>=cfg.solver.timeLimitSeconds,return;end
            inputs=working.inputs;inputs(:,2:end)=inputs(:,2:end)+2^-exponent*reshape(direction,2,[]);
            try
                trial=localNominalEvaluation(inputs,model,working,2);
                trial=localPolishEndpoint(trial,model,timer);
            catch exception
                if localDomainFailure(exception),continue;end
                rethrow(exception);
            end
            if localBetter(trial,candidate,model),candidate=trial;end
            if localFeasible(candidate,model),return;end
            if trial.restorationMerit<working.restorationMerit
                working=trial;improved=true;break;
            end
        end
        if ~improved,return;end
    end
end

function [a,b,equal,rhs]=localCorrectionProblem(candidate,model)
    cfg=model.cfg;count=size(candidate.inputs,2);nv=2*(count-1);h=cfg.controller.sampleTime;
    sensitivity=zeros(6,nv);x=model.initialState;rows=cell(1,4*count+2);bounds=rows;n=0;
    reserve=cfg.solver.feasibilityTolerance;
    rate=h*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    for index=1:count
        % Starts, midpoint and any previously violated interior audit nodes.
        [r,v]=localCorrectionRows(x,sensitivity,(index-1)*h,model,reserve);
        n=n+1;rows{n}=r;bounds{n}=v;
        [middle,am,bm]=nonlinearBicycleModel.sample(x,candidate.inputs(:,index),cfg,[],h/2);
        sm=am*sensitivity;
        if index>1,columns=2*(index-2)+(1:2);sm(:,columns)=sm(:,columns)+bm;end
        [r,v]=localCorrectionRows(middle,sm,(index-.5)*h,model,reserve);
        n=n+1;rows{n}=r;bounds{n}=v;
        if candidate.interiorViolation(index)>0
            for half=0:1
                start=x;map=sensitivity;if half==1,start=middle;map=sm;end
                for duration=localInteriorTimes(cfg)
                    [interior,ai,bi]=nonlinearBicycleModel.sample(start,candidate.inputs(:,index),cfg,[],duration);
                    si=ai*map;if index>1,si(:,columns)=si(:,columns)+bi;end
                    [r,v]=localCorrectionRows(interior,si,(index-1+half/2)*h+duration,model,reserve);
                    n=n+1;rows{n}=r;bounds{n}=v;
                end
            end
        end
        for duration=reshape(candidate.collisionTimes{index},1,[])
            [interior,ai,bi]=localCollisionInterpolation(x,candidate.inputs(:,index),duration,cfg);
            si=ai*sensitivity;if index>1,si(:,columns)=si(:,columns)+bi;end
            [g,j]=localSafetyRows(interior,(index-1)*h+duration,model);
            moving=any(j*si~=0,2);n=n+1;rows{n}=-j*si;bounds{n}=g-reserve*moving;
        end
        [x,an,bn]=nonlinearBicycleModel.sample(middle,candidate.inputs(:,index),cfg,[],h/2);
        sensitivity=an*sm;
        if index>1
            sensitivity(:,columns)=sensitivity(:,columns)+bn;
            finite=isfinite(rate);r=sparse(2,nv);r(:,columns)=eye(2);
            if index>2,r(:,columns-2)=-eye(2);end
            change=candidate.inputs(:,index)-candidate.inputs(:,index-1);
            n=n+1;rows{n}=[r(finite,:);-r(finite,:)];bounds{n}=[rate(finite)-change(finite);rate(finite)+change(finite)];
        end
    end
    [r,v]=localCorrectionRows(x,sensitivity,count*h,model,reserve);
    n=n+1;rows{n}=r;bounds{n}=v;
    memory=sparse(2,nv);memory(:,end-1:end)=eye(2);map=[sensitivity;memory];
    seed=model.terminal;deviation=[x(4:6);candidate.inputs(:,end)]-[seed.base(4:6);seed.reference.input];
    equal=seed.quotientFactor*map(4:8,:);rhs=-seed.quotientFactor*deviation;
    if ~isempty(model.target)
        [seed,poseJacobian]=terminalContinuation.fit(seed,[x;candidate.inputs(:,end)],model.sampleIndex+count);
        [separation,~,gradient]=terminalContinuation.separation(seed,localTargetAt(model,count*h),model.frame,cfg,model.sampleIndex+count);
        n=n+1;rows{n}=-gradient*poseJacobian*map;bounds{n}=separation-reserve;
    end
    a=vertcat(rows{1:n});b=vertcat(bounds{1:n});
end

function [rows,bounds]=localCorrectionRows(x,sensitivity,time,model,reserve)
    [g,j]=localSafetyRows(x,time,model);[lo,hi]=localStateLimits(model.cfg);
    rows=[-j*sensitivity;sensitivity(4:6,:);-sensitivity(4:6,:)];
    % A numerical interior avoids approaching a hard contact only from its
    % infeasible side. The original declared collision margin is unchanged.
    moving=any(j*sensitivity~=0,2);
    bounds=[g-reserve*moving;hi-x(4:6);x(4:6)-lo];
end

function candidate=localPolishEndpoint(candidate,model,timer)
    % Retract the later inputs onto the original nonlinear endpoint set.
    % The first input (and therefore its true CLF decrease) is held fixed.
    % Every corrected trajectory still passes the complete original admission.
    cfg=model.cfg;count=size(candidate.inputs,2);seed=model.terminal;working=candidate;
    tail=max(1,count-cfg.controller.horizonSteps);length=min(count-1,max(8,min(20,tail)));
    if length==0,return;end
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
        'restorationRawMerit',Inf,'restorationMerit',Inf,'candidateStage',"none",'hardRowInfeasible',false, ...
        'zeroClfExitFlag',NaN,'relaxedClfAttempted',false, ...
        'nominalCostAttempted',false,'nominalCostExitFlag',NaN);candidate=[];
    anchor=baseline.inputs;cfg=model.cfg;count=size(anchor,2);prefix=cfg.controller.horizonSteps;reference=model.nominalReference;
    % Normalize both large errors and near-zero CLF residuals. Nonlinear
    % admission remains in physical units with only a roundoff allowance.
    clfScale=max([baseline.clfInitialValue,baseline.clfSlack,sqrt(eps)]);
    objectiveScale=max(1,baseline.cost);
    info.clfScale=clfScale;info.secondaryObjectiveScale=objectiveScale;
    ix=reshape(1:6*(count+1),6,[]);iu=reshape(ix(end)+(1:2*count),2,[]);
    is=iu(end)+(1:count);ic=is(end)+1;it=ic+1;ie=it+(1:2);nv=ie(end);
    equal=sparse(6*(count+1),nv);rhs=zeros(6*(count+1),1);equal(1:6,ix(:,1))=eye(6);
    rows=cell(1,4*count+6);bounds=cell(1,4*count+6);rowCount=0;
    costMatrix=sparse(7*count,nv);costOffset=zeros(7*count,1);
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
        if isfield(model,'collisionTimes')
            for duration=reshape(model.collisionTimes{index},1,[])
                [interior,ai,bi]=localCollisionInterpolation(x,anchor(:,index),duration,cfg);
                map=sparse(6,nv);map(:,ix(:,index))=ai;map(:,iu(:,index))=bi;
                [g,j]=localSafetyRows(interior,(index-1)*cfg.controller.sampleTime+duration,model);
                r=-j*map;r(:,is(index))=-1;
                rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
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
        [candidate,info.primaryRawMerit,used]=localConicEvaluation(first,iu,baseline,model,timer);info.calls=info.calls+used;
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
            [trial,info.restorationRawMerit,used]=localConicEvaluation(first,iu,baseline,model,timer);info.calls=info.calls+used;
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
    % Zero CLF relaxation has priority over all tracking/proximity costs.
    % First optimize those costs on the zero-relaxation slice. If that local
    % slice is infeasible, minimize the relaxation itself, never a weighted sum.
    originalUpper=upper;upper(ic)=0;
    zeroEndpoint=endpoint;
    % On rho=0 the norm ball avoids subtracting nearly equal epigraph
    % constants. Its accuracy is measured in error-norm rather than W units.
    zeroEndpoint(2)=secondordercone(clfMap,-clfOffset,zeros(nv,1),-sqrt(clfConstant));
    localA=a;localB=b;localEqual=equal;localLower=lower;
    costMatrix=costMatrix/sqrt(objectiveScale);
    costOffset=costOffset/sqrt(objectiveScale);
    % Tracking and deviation from the current input anchor are optimized
    % only on the zero-CLF slice, using a separate small cone per stage.
    groups=count;expanded=nv+groups;
    baseCones=numel(endpoint);cones=repmat(zeroEndpoint(1),1,groups+baseCones);
    for index=1:baseCones
        cones(index)=secondordercone([zeroEndpoint(index).A,sparse(size(zeroEndpoint(index).A,1),groups)], ...
            zeroEndpoint(index).b,[zeroEndpoint(index).d;zeros(groups,1)],zeroEndpoint(index).gamma);
    end
    epigraph=zeros(groups,1);
    for group=1:groups
        selected=7*(group-1)+(1:7);
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
    info.calls=info.calls+1;info.zeroClfExitFlag=flag;
    if flag==-2 || isempty(second) || any(~isfinite(second))
        remaining=cfg.solver.timeLimitSeconds-toc(timer);
        if remaining<=0,info.status="timeLimit";return;end
        options.MaxTime=remaining;info.relaxedClfAttempted=true;
        objective=zeros(nv,1);objective(ic)=1;
        a=localA;b=localB;equal=localEqual;lower=localLower;upper=originalUpper;
        [second,~,flag]=coneprog(objective,endpoint,a,b,equal,rhs,lower,upper,options);
        first=first(1:nv);info.calls=info.calls+1;
    end
    info.secondaryExitFlag=flag;
    % Keep the best nonlinear candidate across both conic stages and polish.
    info.secondaryReturned=~isempty(second) && all(isfinite(second));
    if info.secondaryReturned,first=second;end
    info.solverResidual=max([0;a*first-b;abs(equal*first-rhs);lower-first;first-upper]);
    info.secondarySafety=sum(max(0,first(is(1:prefix))));info.clfSlack=clfScale*max(0,first(ic));info.status="solved";
    [trial,info.secondaryRawMerit,used]=localConicEvaluation(second,iu,baseline,model,timer,~info.relaxedClfAttempted);info.calls=info.calls+used;
    if ~isempty(trial)
        info.secondaryMerit=trial.restorationMerit;
        info.secondaryAccepted=localFeasible(trial,model) ...
            && (~localFeasible(baseline,model) || trial.clfSlack<=baseline.clfSlack);
        if localBetter(trial,candidate,model),candidate=trial;info.candidateStage="secondary";end
    end
    if info.relaxedClfAttempted && info.secondaryReturned && toc(timer)<cfg.solver.timeLimitSeconds
        % First-step relaxation leaves every later input undetermined. Resolve
        % that nullspace with the predictive cost, fixing the minimizing first
        % control so its actual nonlinear CLF value cannot be purchased away.
        cones(2)=secondordercone([endpoint(2).A,sparse(size(endpoint(2).A,1),groups)], ...
            endpoint(2).b,[endpoint(2).d;zeros(groups,1)],endpoint(2).gamma);
        r=sparse(1,nv);r(it)=1;
        costA=[localA,sparse(size(localA,1),groups);-r,ones(1,groups)];
        costB=[localB;0];costEqual=[localEqual,sparse(size(localEqual,1),groups)];
        costLower=[localLower;zeros(groups,1)];costUpper=[originalUpper;Inf(groups,1)];
        costLower(iu(:,1))=second(iu(:,1));costUpper(iu(:,1))=second(iu(:,1));
        costUpper(ic)=max(0,second(ic))+64*eps(max(1,abs(second(ic))));
        objective=zeros(expanded,1);objective(it)=1;options.MaxTime=cfg.solver.timeLimitSeconds-toc(timer);
        info.nominalCostAttempted=true;
        [third,~,info.nominalCostExitFlag]=coneprog(objective,cones,costA,costB,costEqual,rhs,costLower,costUpper,options);
        info.calls=info.calls+1;
        [trial,~,used]=localConicEvaluation(third,iu,baseline,model,timer);info.calls=info.calls+used;
        if localBetter(trial,candidate,model),candidate=trial;info.candidateStage="nominalCost";end
    end
    if ~isempty(candidate),info.terminalPhaseMeters=candidate.terminalPhaseMeters;end
end

function [candidate,rawMerit,calls]=localConicEvaluation(point,iu,baseline,model,timer,zeroClf)
    if nargin<6,zeroClf=false;end
    candidate=[];rawMerit=Inf;calls=0;
    if isempty(point) || any(~isfinite(point)) || toc(timer)>=model.cfg.solver.timeLimitSeconds,return;end
    delta=reshape(point(iu(:)),2,[]);
    for exponent=0:20
        if toc(timer)>=model.cfg.solver.timeLimitSeconds,return;end
        scale=2^-exponent;
        try
            inputs=baseline.inputs+scale*delta;retracted=false;
            try
                trial=localNominalEvaluation(inputs,model);
            catch exception
                if ~localDomainFailure(exception),rethrow(exception);end
                count=size(inputs,2);
                reference=baseline.states+scale*reshape(point(1:6*(count+1)),6,[]);
                inputs=localStabilizedInputs(inputs,reference,baseline,model);retracted=true;
                trial=localNominalEvaluation(inputs,model);
            end
            if scale==1 && ~retracted,rawMerit=trial.restorationMerit;end
            if scale==1 && ~retracted && ~localFeasible(trial,model) ...
                    && trial.stageSafety(1)==0 && trial.stageHard(1)==0 ...
                    && toc(timer)<model.cfg.solver.timeLimitSeconds
                % A defined open-loop rollout can still lose the affine
                % proposal through amplified second-order dynamic defects.
                % Preserve the raw point while offering the same retraction.
                try
                    count=size(inputs,2);
                    reference=baseline.states+reshape(point(1:6*(count+1)),6,[]);
                    corrected=localStabilizedInputs(inputs,reference,baseline,model);
                    corrected=localNominalEvaluation(corrected,model);
                    if localBetter(corrected,trial,model),trial=corrected;end
                catch exception
                    if ~localDomainFailure(exception),rethrow(exception);end
                end
            end
            if localBetter(trial,candidate,model),candidate=trial;end
            if localFeasible(candidate,model) && (~localFeasible(baseline,model) || candidate.clfSlack<=baseline.clfSlack) ...
                    && (~zeroClf || localClfSatisfied(candidate)),return;end
            if zeroClf && ~localClfSatisfied(trial)
                corrected=localPolishClf(trial,model,timer);
                if scale==1,[corrected,used]=localRetractConstraints(corrected,model,timer);calls=calls+used;
                else,corrected=localPolishEndpoint(corrected,model,timer);
                end
                if localBetter(corrected,candidate,model),candidate=corrected;end
                if localFeasible(candidate,model) && localClfSatisfied(candidate),return;end
            end
            if scale==1,[trial,used]=localRetractConstraints(trial,model,timer);calls=calls+used;
            else,trial=localPolishEndpoint(trial,model,timer);
            end
            if localBetter(trial,candidate,model),candidate=trial;end
            if localBetter(candidate,baseline,model),return;end
        catch exception
            if ~localDomainFailure(exception),rethrow(exception);end
        end
    end
end

function inputs=localStabilizedInputs(inputs,reference,anchor,model)
    % Project a locally proposed state/control path onto nonlinear dynamics.
    % Finite-horizon linear feedback damps rollout defects; it is a numerical
    % retraction, not an execution policy. The first input remains fixed and
    % the resulting controls still require complete nonlinear admission.
    cfg=model.cfg;count=size(inputs,2);gains=zeros(2,6,count);
    q=diag(1./[5;5;.5;5;3;1.5].^2);
    r=diag([cfg.clf.frontWheelSteeringAngleWeight;cfg.clf.brakingRatioWeight]);p=q;
    for index=count:-1:1
        [~,a,b]=nonlinearBicycleModel.sample(anchor.states(:,index),anchor.inputs(:,index),cfg);
        gain=-(r+b.'*p*b)\(b.'*p*a);
        gains(:,:,index)=gain;closed=a+b*gain;
        p=q+gain.'*r*gain+closed.'*p*closed;p=(p+p.')/2;
    end
    x=model.initialState;previous=model.previousInput;
    for index=1:count
        input=inputs(:,index);
        if index>1
            input=localClip(input+gains(:,:,index)*(x-reference(:,index)),previous,cfg);
        end
        inputs(:,index)=input;
        x=nonlinearBicycleModel.sample(x,input,cfg);previous=input;
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
    [low,high]=localStateLimits(cfg);
    states=zeros(6,count+1);states(:,1)=model.initialState;
    stageSafety=zeros(1,count);stageHard=stageSafety;stageCost=stageSafety;
    stageCollision=Inf(1,count);collisionNodes=cell(1,count);collisionTimes=cell(1,count);
    intervalCertified=repmat(isempty(model.target),1,count);
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
        collisionNodes(1:first-1)=cached.collisionNodes(1:first-1);
        collisionTimes(1:first-1)=cached.collisionTimes(1:first-1);
        intervalCertified(1:first-1)=cached.intervalCertified(1:first-1);
    end
    x=states(:,first);
    for index=first:count
        values=localSafetyRows(x,(index-1)*cfg.controller.sampleTime,model);
        nodes=localCollisionNodes(x,inputs(:,index),cfg);collisionNodes{index}=nodes;
        middle=nodes(:,(size(nodes,2)+1)/2);
        middleValues=localSafetyRows(middle,(index-.5)*cfg.controller.sampleTime,model);
        stageSafety(index)=max([0;-values;-middleValues]);
        if ~isempty(model.target)
            stageCollision(index)=min([values(1:4);middleValues(1:4)]);
        end
        x=nodes(:,end);states(:,index+1)=x;
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
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    for index=1:count
        if isempty(model.target),continue;end
        if index<first && intervalCertified(index),continue;end
        time=(index-1)*cfg.controller.sampleTime;nodes=collisionNodes{index};
        if ~(refinement(index) || checkWitness),continue;end
        intervals=size(nodes,2)-1;duration=cfg.controller.sampleTime/intervals;
        q=localTargetAt(model,time);reach=norm(shape(1:2))+norm(shape(3:4));
        targetReach=norm(q(8:9))+norm(q(10:11));
        speed=max(abs([q(4),q(4)+q(5)*cfg.controller.sampleTime]));
        travel=cfg.controller.sampleTime*(max(vecnorm(diff(nodes(1:2,:),1,2),2,1))/duration ...
            +max(abs(diff(nodes(3,:))))/duration*reach+speed*(1+abs(sin(q(6))/q(7))*targetReach));
        if stageCollision(index)>travel,intervalCertified(index)=true;continue;end
        for node=2:intervals+1
            values=localSafetyRows(nodes(:,node),time+(node-1)*duration,model);
            violation=max([0;-values]);interiorViolation(index)=max(interiorViolation(index),violation);
            stageSafety(index)=max(stageSafety(index),violation);
            stageCollision(index)=min([stageCollision(index);values]);
        end
        if stageSafety(index)>0,intervalCertified(index)=false;continue;end
        intervalCertified(index)=true;
        for node=1:intervals
            absolute=(model.sampleIndex+index-1)*cfg.controller.sampleTime+(node-1)*duration;
            check=predictiveSafetyGeometry.intervalClearance(nodes(1:3,node),nodes(1:3,node+1), ...
                shape,model.targetEpoch,absolute,duration,cfg.collision.safetyMarginMeters);
            intervalCertified(index)=intervalCertified(index) && check.certified;
            for fraction=check.fractions
                cut=(node-1+fraction)*duration;
                collisionTimes{index}(end+1)=cut;
                interior=(1-fraction)*nodes(:,node)+fraction*nodes(:,node+1);
                values=localSafetyRows(interior,time+cut,model);
                violation=max([0;-values]);interiorViolation(index)=max(interiorViolation(index),violation);
                stageSafety(index)=max(stageSafety(index),violation);
                stageCollision(index)=min([stageCollision(index);values]);
            end
        end
        collisionTimes{index}=reshape(unique(collisionTimes{index}),1,[]);
        if ~intervalCertified(index) && stageSafety(index)==0,hard=Inf;end
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
        'cost',sum(stageCost)/count, ...
        'stageSafety',stageSafety,'stageHard',stageHard,'stageCost',stageCost, ...
        'stageCollision',stageCollision,'collisionNodes',{collisionNodes}, ...
        'collisionTimes',{collisionTimes},'intervalCertified',intervalCertified);
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

function q=localTargetAt(model,time)
    h=model.cfg.controller.sampleTime;
    count=max(1,ceil(h/(2*model.cfg.nonlinear.integrationStep)));
    coordinate=2*count*time/h;
    if abs(coordinate-round(coordinate))>64*eps(max(1,abs(coordinate)))
        q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,model.sampleIndex*h+time);return;
    end
    tick=2*count*model.sampleIndex+round(2*count*time/h);
    if mod(tick,count)==0,absoluteTime=(tick/count)*(h/2);
    else,absoluteTime=tick*(h/(2*count));
    end
    q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,absoluteTime);
end

function nodes=localCollisionNodes(x,input,cfg)
    count=2*max(1,ceil(cfg.controller.sampleTime/(2*cfg.nonlinear.integrationStep)));
    nodes=zeros(6,count+1);nodes(:,1)=x;duration=cfg.controller.sampleTime/count;
    for index=1:count,nodes(:,index+1)=nonlinearBicycleModel.sample(nodes(:,index),input,cfg,[],duration);end
end

function [x,a,b]=localCollisionInterpolation(initial,input,time,cfg)
    count=2*max(1,ceil(cfg.controller.sampleTime/(2*cfg.nonlinear.integrationStep)));
    duration=cfg.controller.sampleTime/count;index=min(count-1,floor(time/duration));
    fraction=(time-index*duration)/duration;
    if index==0,left=initial;al=eye(6);bl=zeros(6,2);
    else,[left,al,bl]=nonlinearBicycleModel.sample(initial,input,cfg,[],index*duration);
    end
    [right,ar,br]=nonlinearBicycleModel.sample(left,input,cfg,[],duration);
    x=(1-fraction)*left+fraction*right;
    a=((1-fraction)*eye(6)+fraction*ar)*al;
    b=(1-fraction)*bl+fraction*(ar*bl+br);
end
