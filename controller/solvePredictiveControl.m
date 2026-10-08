function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl One affine model per sample; no fallback algorithm.
% Startup uses a potential-field rollout; later samples start from the
% previous accepted plan shifted by one hold. The horizon ends where the
% anchor enters the terminal set (terminalSafeSet): the CLF tube of a backup
% (the nominal path or another lane centre) from the endpoint misses the
% target until the target leaves the perception range, within the encounter
% window. A shifted plan shorter than horizonSteps, or whose endpoint is no
% longer in the set, is extended by holds of the endpoint's backup (the same
% problem without PCBF rows, CLF without slack). A plan whose endpoint holds
% another lane is extended back to the nominal set by path guidance whenever
% that extension meets every hard row.
% The PCBF stage minimizes prefix safety slack, then the CLF stage runs; the
% terminal row is the CLF level of the endpoint, V(x_N) <= c*, c* the
% largest level whose tube at the anchor endpoint is clear. A primary
% problem infeasible inside the trust region is re-solved on the same
% linearization with the trust scale doubled until it is feasible or
% reaches trustMaximumScale. A shifted plan still infeasible is then solved
% from a fresh potential-field rollout in the same way.
% The issued plan is the nonlinear rollout of anchor + alpha*correction for
% the largest alpha in terminal.acceptanceSteps whose rollout meets every
% hard row and ends in the terminal set, without increasing the prefix
% safety deficit; alpha = 0 is the shifted plan, feasible by construction
% when the forecast and the state evolved as predicted (recursive
% feasibility). Without an acceptable step the problem is linearized again
% at the full step, at most terminal.sqpIterations times, and then reports
% that it has no solution.
% The input trust scale is an estimate: the second-order error coefficient
% revealed by the nonlinear rollout of the full affine step sets the next
% step bound.
    if nargin<3,timer=tic;end
    if ~isfield(model,'uncertainty')
        model.uncertainty=struct('specified',false,'egoGenerator',zeros(6), ...
            'collisionGenerator',zeros(6),'target',[],'relativeFrame',false);
    end
    cfg=model.cfg;
    model.trust=localTrustPrior(previousState,cfg);
    model.inputTrustScale=model.trust.scale;
    model.linearizationBuilds=0;
    [solution,search,model]=localRound(model,previousState,timer);
    if ~isempty(solution)
        model.trust=localTrustRecord(model.trust,solution,model,search.acceptance);
        solution=localTerminalMetrics(solution,model,search.acceptance);
    end
    search.trust=model.trust;
    search.linearizationCount=sum([search.attempts.modelBuilt]);
    search.elapsedSeconds=toc(timer);search.returned=~isempty(solution);
end

function trust=localTrustPrior(previous,cfg)
    % Startup assigns the coefficient that makes the initial scale consistent.
    scale=cfg.nonlinear.trustInitialScale;
    trust=struct('scale',scale,'curvature',cfg.nonlinear.trustInnovationMeters/scale^2, ...
        'correction',NaN,'innovation',NaN,'observationInnovation',NaN,'modelInnovation',NaN, ...
        'updated',false);
    if isstruct(previous) && isfield(previous,'trust') && isstruct(previous.trust)
        trust=previous.trust;trust.updated=false;
        trust.innovation=NaN;trust.observationInnovation=NaN;trust.modelInnovation=NaN;
    end
end

function model=localAdaptTrust(model,previous)
    % Departure of the present estimate from the previous plan's prediction
    % of it (observation innovation). The plan is a nonlinear rollout, so this
    % is the estimator's and the plant's share; the model share is measured
    % when a step is accepted (localTrustRecord).
    trust=model.trust;
    if ~isfield(previous,'stateTrajectory') || size(previous.stateTrajectory,2)<2,return;end
    trust.observationInnovation=localPoseDiscrepancy(model.initialState,previous.stateTrajectory(:,2),model.cfg);
    trust.innovation=trust.observationInnovation;
    model.trust=trust;
end

function discrepancy=localPoseDiscrepancy(first,second,cfg)
    % Body-point pose metric of the safety rows: translation plus reach*yaw.
    reach=norm([cfg.vehicle.length;cfg.vehicle.width]/2)+norm(cfg.vehicle.rectangleOffset);
    difference=first-second;
    discrepancy=vecnorm(difference(1:2,:))+reach*abs(atan2(sin(difference(3,:)),cos(difference(3,:))));
end

function trust=localTrustRecord(trust,solution,model,acceptance)
    % Store the correction actually taken, in units of the unit input box,
    % and update the error coefficient from the full affine step: its
    % nonlinear rollout departs from the affine prediction by the
    % second-order remainder L*s^2 (s its size in the unit box).
    cfg=model.cfg;unit=cfg.nonlinear.trustRadius*[.15;.25];
    anchor=model.linearization.inputs;count=min(size(anchor,2),size(solution.inputs,2));
    trust.correction=max(abs(solution.inputs(:,1:count)-anchor(:,1:count))./unit,[],'all');
    if ~isfinite(acceptance.fullGap) || ~(acceptance.fullCorrection>0),return;end
    trust.modelInnovation=acceptance.fullGap;
    % A step far inside the box carries little information about its boundary.
    step=max(acceptance.fullCorrection,trust.scale/2);
    curvature=max(acceptance.fullGap/step^2,cfg.nonlinear.trustRetention*trust.curvature);
    curvature=max(curvature,eps);
    trust.scale=min(cfg.nonlinear.trustMaximumScale,max(cfg.nonlinear.trustMinimumScale, ...
        sqrt(cfg.nonlinear.trustInnovationMeters/curvature)));
    trust.curvature=curvature;trust.updated=true;
end

function [solution,search,model] = localRound(model,previousState,timer)
%localRound Linearize one anchor, minimize safety slack, then solve the CLF.
    if nargin<3,timer=tic;end
    wall=tic;[anchor,source,failure]=localInitialization(model,previousState);
    initializationSeconds=toc(wall);solution=[];
    if isempty(anchor)
        search=localEmptySearch(model,source,failure,initializationSeconds);return;
    end
    if source=="shiftedInputRollout",model=localAdaptTrust(model,previousState);end
    estimatedScale=model.inputTrustScale;
    [point,problem,search,model]=localPrimary(anchor,model,source,timer);
    search.initializationSeconds=initializationSeconds;
    attempts=localAttempt(search);stages=search.stages;selected=1;restarted=false;
    cfg=model.cfg;freshSource="movingTargetPotentialField";
    if isempty(model.target),freshSource="laneFeedbackRollout";end
    [point,problem,search,model,attempts,stages,selected]=localExpand( ...
        point,problem,search,model,anchor,source,timer,attempts,stages,selected);
    % Infeasible even in the largest trust region around the shifted plan.
    if source=="shiftedInputRollout" && localInfeasible(point,search) && toc(timer)<cfg.solver.timeLimitSeconds
        failure="shiftedProblemInfeasible";model.inputTrustScale=estimatedScale;
        wall=tic;anchor=localPotentialFieldSeed(model,cfg.controller.horizonSteps,cfg.controller.maximumHorizonSteps);
        [point,problem,search,model]=localPrimary(anchor,model,freshSource,timer);
        search.initializationSeconds=toc(wall);restarted=true;
        attempts(end+1)=localAttempt(search);stages=[stages,search.stages];selected=numel(attempts);
        [point,problem,search,model,attempts,stages,selected]=localExpand( ...
            point,problem,search,model,anchor,freshSource,timer,attempts,stages,selected);
    end
    [solution,search,model]=localSecondary(point,problem,anchor,model,search,timer);
    stages=[stages,search.stages(2:end)];attempts(selected)=localAttempt(search);
    [solution,search,model,stages]=localAcceptance(solution,problem,anchor,model,search,timer,stages);
    search.attempts=attempts;search.selectedAttempt=selected;search.stages=stages;
    search.solverCalls=sum([stages.numericalSolve]);search.potentialFieldRestarted=restarted;
    search.initializationFailure=failure;search.elapsedSeconds=toc(timer);
    search.anchorCertified=isfield(anchor,'certified') && anchor.certified;
    search.appendedTerminalSteps=0;if isfield(anchor,'appended'),search.appendedTerminalSteps=anchor.appended;end
    search.anchorHolds=size(anchor.inputs,2);
    search.returnedHolds=0;if isfield(anchor,'returnedHolds'),search.returnedHolds=anchor.returnedHolds;end
    search.seedReachedTerminalSet=NaN;
    if isfield(anchor,'seedReachedTerminalSet'),search.seedReachedTerminalSet=anchor.seedReachedTerminalSet;end
end

function [solution,search,model,stages]=localAcceptance(solution,problem,anchor,model,search,timer,stages)
    % Step-size rule on the nonlinear sampled model. The affine solution is
    % a search direction about the anchor. The issued plan is the nonlinear
    % rollout of anchor + alpha*correction for the largest alpha in
    % terminal.acceptanceSteps whose rollout meets every hard row and ends
    % in the terminal set, without increasing the soft prefix deficit of the
    % anchor. alpha = 0 is the anchor itself: when it is the previous plan
    % shifted by one hold (and, if needed, extended by the terminal
    % controller), it meets these rows by construction, so a feasible plan
    % exists at every sample. Without any acceptable step the problem is
    % linearized again at the full step (another iteration of the same
    % sequential convex method); after terminal.sqpIterations it reports
    % that it has no solution.
    cfg=model.cfg;
    search.acceptance=struct('step',NaN,'candidateFeasible',false,'candidateReason',"",'iterations',0, ...
        'linearizationGap',NaN,'fullGap',NaN,'fullCorrection',NaN,'terminalLevel',NaN, ...
        'terminalValue',NaN,'terminalExitSeconds',NaN,'terminalLaneOffset',NaN,'terminalEnd',"",'reason',"");
    if isempty(solution),return;end
    for iteration=0:cfg.terminal.sqpIterations
        [accepted,info]=localLineSearch(solution,anchor,model);
        if iteration==0
            search.acceptance.candidateFeasible=info.candidateFeasible;search.acceptance.candidateReason=info.candidateReason;
        end
        search.acceptance.iterations=iteration;search.acceptance.reason=info.reason;
        search.acceptance.fullGap=info.fullGap;search.acceptance.fullCorrection=info.fullCorrection;
        search.acceptance.terminalLevel=info.terminalLevel;
        if ~isempty(accepted)
            solution.inputs=accepted.inputs;solution.states=accepted.states;
            solution.stageSlacks=accepted.softDeficits;solution.safety=accepted.softSum;
            search.acceptance.step=accepted.step;search.acceptance.linearizationGap=accepted.linearizationGap;
            search.acceptance.terminalValue=accepted.terminalValue;
            search.acceptance.terminalExitSeconds=accepted.terminalExitSeconds;
            search.acceptance.terminalLaneOffset=accepted.terminalLaneOffset;
            search.acceptance.terminalEnd=accepted.terminalEnd;
            return;
        end
        if iteration==cfg.terminal.sqpIterations || toc(timer)>=cfg.solver.timeLimitSeconds,break;end
        anchor=struct('inputs',info.fullInputs,'states',info.fullStates,'terminalContext',info.context);
        [point,problem,again,model]=localPrimary(anchor,model,"acceptanceRelinearization",timer);
        stages=[stages,again.stages];
        if isempty(point),break;end
        [solution,again,model]=localSecondary(point,problem,anchor,model,again,timer);
        stages=[stages,again.stages(2:end)];
        if isempty(solution),break;end
    end
    solution=[];search.returned=false;search.terminationReason="nonlinearAcceptanceFailed";
end

function [accepted,info]=localLineSearch(solution,anchor,model)
    cfg=model.cfg;steps=cfg.terminal.acceptanceSteps;
    correction=solution.inputs-anchor.inputs;affineStates=solution.states-anchor.states;
    context=localTerminalContext(anchor,model);
    unit=cfg.nonlinear.trustRadius*[.15;.25];
    accepted=[];info=struct('candidateFeasible',false,'candidateReason',"",'reason',"",'fullInputs',[],'fullStates',[], ...
        'context',context,'terminalLevel',NaN,'fullGap',NaN, ...
        'fullCorrection',max(abs(correction)./unit,[],'all'));
    [candidate,context]=localEvaluatePlan(anchor.inputs,model,context);
    info.candidateFeasible=candidate.feasible;info.candidateReason=candidate.reason;
    reference=Inf;if candidate.feasible,reference=candidate.softSum;end
    for alpha=steps(:).'
        if alpha==0
            plan=candidate;
        else
            [plan,context]=localEvaluatePlan(anchor.inputs+alpha*correction,model,context);
        end
        if alpha==1 && ~isempty(plan.states)
            info.fullInputs=plan.inputs;info.fullStates=plan.states;
            info.fullGap=max(localPoseDiscrepancy(plan.states,solution.states,cfg));
        end
        if isempty(plan.states),info.reason=plan.reason;continue;end
        decrease=plan.softSum<=reference+cfg.solver.lexicographicTieTolerance*max(1,reference);
        if plan.feasible && (decrease || ~candidate.feasible)
            plan.step=alpha;
            prediction=anchor.states+alpha*affineStates;
            plan.linearizationGap=max(localPoseDiscrepancy(plan.states,prediction,cfg));
            accepted=plan;break;
        end
        if plan.feasible,info.reason="prefixDeficitIncrease";else,info.reason=plan.reason;end
    end
    [info.terminalLevel,context]=terminalSafeSet.level(context,anchor.states(:,end),size(anchor.inputs,2));
    info.context=context;
end

function context=localTerminalContext(anchor,model)
    if isfield(anchor,'terminalContext') && ~isempty(anchor.terminalContext)
        context=anchor.terminalContext;
    else
        context=terminalSafeSet.context(model);
    end
end

function [plan,context]=localEvaluatePlan(inputs,model,context)
    % Nonlinear rollout of one input sequence and its hard and soft rows:
    % road, state limits and handling envelope at every hold, collision
    % clearance at nodes and hold midpoints (soft in the prefix, hard after
    % it), and the terminal set at the endpoint.
    cfg=model.cfg;count=size(inputs,2);prefix=cfg.controller.horizonSteps;h=cfg.controller.sampleTime;
    plan=struct('inputs',inputs,'states',[],'feasible',false,'softSum',Inf,'softDeficits',zeros(1,0), ...
        'reason',"",'terminalValue',NaN,'terminalExitSeconds',NaN,'terminalLaneOffset',NaN,'terminalEnd',"");
    states=zeros(6,count+1);middles=zeros(6,count);states(:,1)=model.initialState;
    try
        for index=1:count
            [middles(:,index),states(:,index+1)]=terminalSafeSet.holdStates(states(:,index),inputs(:,index),cfg);
        end
    catch exception
        if ~startsWith(string(exception.identifier),"collisionAvoidanceController:"),rethrow(exception);end
        plan.reason="nonlinearDomain";return;
    end
    plan.states=states;
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    margin=cfg.collision.safetyMarginMeters;tolerance=1e-6;
    deficits=zeros(1,count+1);
    for index=1:count
        u=inputs(:,index);x=states(:,index);next=states(:,index+1);middle=middles(:,index);
        if ~terminalSafeSet.stateRows(next,cfg) || ~terminalSafeSet.stateRows(middle,cfg)
            plan.reason="stateLimits";return;
        end
        if ~terminalSafeSet.handlingRows(next,u,cfg) || (index>1 && ~terminalSafeSet.handlingRows(x,u,cfg))
            plan.reason="handlingEnvelope";return;
        end
        if ~isempty(model.road.lateralClearance)
            if (index>1 && terminalSafeSet.roadMargin(x,model)<-tolerance) || terminalSafeSet.roadMargin(middle,model)<-tolerance
                plan.reason="road";return;
            end
        end
        for point=[1,2]
            if point==1,pose=x;time=(index-1)*h;else,pose=middle;time=(index-.5)*h;end
            deficit=max(0,margin-localClearance(pose,time,model,shape));
            if index<=prefix
                deficits(index)=max(deficits(index),deficit);
            elseif deficit>tolerance
                plan.reason="collision";return;
            end
        end
    end
    if ~isempty(model.road.lateralClearance) && terminalSafeSet.roadMargin(states(:,end),model)<-tolerance
        plan.reason="road";return;
    end
    gap=localClearance(states(:,end),count*h,model,shape);
    if count+1<=prefix,deficits(count+1)=max(0,margin-gap);
    elseif margin-gap>tolerance,plan.reason="collision";return;
    end
    [member,context,terminal]=terminalSafeSet.member(context,states(:,end),count);
    plan.terminalValue=terminal.value;plan.terminalExitSeconds=terminal.exitSeconds;
    if member,plan.terminalLaneOffset=terminal.lateralOffset;plan.terminalEnd=terminal.reason;end
    if ~member,plan.reason="terminal:"+terminal.reason;return;end
    plan.softDeficits=deficits;plan.softSum=sum(deficits);plan.feasible=true;
end

function gap=localClearance(pose,time,model,shape)
    % Ordinary rectangle distance to the target forecast; Inf when no target
    % or beyond the encounter range (the target no longer exists there).
    gap=Inf;if isempty(model.target),return;end
    q=localTargetAt(model,time);
    if norm(pose(1:2)-q(1:2))>model.cfg.collision.encounterRangeMeters,return;end
    gap=predictiveSafetyGeometry.rectangle(pose(1:3),shape,q(1:3),q(8:11));
end

function [point,problem,search,model,attempts,stages,selected]=localExpand( ...
        point,problem,search,model,anchor,source,timer,attempts,stages,selected)
    % Double the input trust scale on the same anchor while the primary
    % problem is certified infeasible, never beyond trustMaximumScale: larger
    % steps leave the region where the linearization is accurate. The trust
    % estimate itself is unchanged.
    cfg=model.cfg;
    while localInfeasible(point,search) && model.inputTrustScale<cfg.nonlinear.trustMaximumScale ...
            && toc(timer)<cfg.solver.timeLimitSeconds
        model.inputTrustScale=min(2*model.inputTrustScale,cfg.nonlinear.trustMaximumScale);
        [point,problem,search,model]=localPrimary(anchor,model,source,timer);
        attempts(end+1)=localAttempt(search);stages=[stages,search.stages];selected=numel(attempts); %#ok<AGROW>
    end
end

function infeasible=localInfeasible(point,search)
    infeasible=isempty(point) && ~isempty(search.stages) && search.stages(end).exitFlag==-2;
end

function search=localEmptySearch(model,source,failure,seconds)
    search=struct('solverCalls',0,'source',"twoStageRealTimeIteration",'modelBuilt',false, ...
        'initialization',source,'returned',false,'converged',false,'terminationReason',"anchorUnavailable", ...
        'formulationSeconds',0,'clfConstructionSeconds',0,'inputTrustScale',model.inputTrustScale, ...
        'primaryOptimum',NaN,'primaryLowerBound',NaN,'slackCap',NaN,'clfInitialSlack',NaN, ...
        'clfStageAttempted',false,'clfStageCompleted',false,'clfLowerBound',false, ...
        'stages',struct('objective',{},'exitFlag',{},'seconds',{},'value',{},'numericalSolve',{},'solverInfo',{}), ...
        'initializationSeconds',seconds,'initializationFailure',failure,'elapsedSeconds',seconds, ...
        'potentialFieldRestarted',false);
    search.attempts=localAttempt(search);search.selectedAttempt=0;
end

function attempt=localAttempt(search)
    attempt=struct('initialization',search.initialization,'terminationReason',search.terminationReason, ...
        'solverCalls',search.solverCalls,'stages',search.stages,'primaryOptimum',search.primaryOptimum, ...
        'primaryLowerBound',search.primaryLowerBound,'inputTrustScale',search.inputTrustScale, ...
        'slackCap',search.slackCap,'initializationSeconds',search.initializationSeconds,'formulationSeconds',search.formulationSeconds, ...
        'clfConstructionSeconds',search.clfConstructionSeconds);
    attempt.modelBuilt=search.modelBuilt;
end

function search=localSearch(problem,model,initialization,formulationSeconds)
    search=struct('solverCalls',0,'source',"twoStageRealTimeIteration", ...
        'modelBuilt',formulationSeconds>0, ...
        'initialization',initialization,'returned',false,'converged',false,'terminationReason',"timeLimit", ...
        'formulationSeconds',formulationSeconds,'clfConstructionSeconds',0, ...
        'inputTrustScale',model.inputTrustScale,'primaryOptimum',NaN,'primaryLowerBound',problem.primaryLowerBound,'slackCap',NaN,'clfInitialSlack',NaN, ...
        'initializationSeconds',0, ...
        'clfStageAttempted',false,'clfStageCompleted',false,'clfLowerBound',false, ...
        'stages',struct('objective',{},'exitFlag',{},'seconds',{},'value',{},'numericalSolve',{},'solverInfo',{}));
end

function [point,problem,search,model]=localPrimary(anchor,model,initialization,timer)
    cfg=model.cfg;point=[];
    wall=tic;[problem,model]=localFormulate(anchor,model);formulationSeconds=toc(wall);
    model.linearization=anchor;
    search=localSearch(problem,model,initialization,formulationSeconds);
    remaining=cfg.solver.timeLimitSeconds-toc(timer);if remaining<=0,return;end
    primaryCount=problem.clfIndex-1;primaryCone=problem.cones(1:problem.primaryConeCount);wall=tic;
    coneFeasible=true;
    for index=1:numel(primaryCone)
        cone=primaryCone(index);coneFeasible=coneFeasible && norm(cone.b)<=-cone.gamma;
        primaryCone(index).A=cone.A(:,1:primaryCount);primaryCone(index).d=cone.d(1:primaryCount);
    end
    % A feasible zero correction with zero nonnegative slacks attains the
    % global lower bound. Include every primary constraint, not just safety.
    if all(problem.b>=0) && all(problem.rhs==0) ...
            && all(problem.lower(1:primaryCount)<=0) && all(problem.upper(1:primaryCount)>=0) ...
            && coneFeasible
        point=zeros(primaryCount,1);search.primaryOptimum=0;
        search.slackCap=cfg.solver.lexicographicTieTolerance;
        search.stages(1)=struct('objective',"pcbfSlack",'exitFlag',1, ...
            'seconds',toc(wall),'value',0,'numericalSolve',false,'solverInfo',struct());
        search.terminationReason="primaryReturned";return;
    end
    wall=tic;
    [point,flag,solverInfo]=localConicSolve(sparse(primaryCount,primaryCount),problem.safetyObjective(1:primaryCount),primaryCone, ...
        problem.a(:,1:primaryCount),problem.b,problem.equal(:,1:primaryCount),problem.rhs, ...
        problem.lower(1:primaryCount),problem.upper(1:primaryCount),cfg,remaining,cfg.solver.optimalityTolerance);
    search.solverCalls=1;
    search.stages(1)=struct('objective',"pcbfSlack",'exitFlag',flag,'seconds',toc(wall),'value',NaN,'numericalSolve',true,'solverInfo',solverInfo);
    if isempty(point) || any(~isfinite(point)) || flag<=0
        point=[];search.terminationReason="pcbfNoNumericalResult";return;
    end
    search.primaryOptimum=sum(max(0,point(problem.slackIndices)));
    search.stages(1).value=search.primaryOptimum;
    % Match the cap's scale to the numerical primary objective. An absolute
    % micrometer cap on a large restoration sum can exclude the computed
    % primary point within the solver's relative optimality tolerance.
    search.slackCap=search.primaryOptimum+cfg.solver.lexicographicTieTolerance*max(1,search.primaryOptimum);
    search.terminationReason="primaryReturned";
end

function [solution,search,model]=localSecondary(point,problem,anchor,model,search,timer)
    cfg=model.cfg;solution=[];if isempty(point),return;end
    if ~isfield(problem,'clf')
        wall=tic;problem=localAddClf(problem,anchor,model);search.clfConstructionSeconds=toc(wall);
        search.formulationSeconds=search.formulationSeconds+search.clfConstructionSeconds;
    end
    search.clfInitialSlack=problem.initialClfSlack;
    problem.a=[problem.a;problem.safetyObjective.'];problem.b=[problem.b;search.slackCap];
    remaining=cfg.solver.timeLimitSeconds-toc(timer);
    if remaining<=0,search.terminationReason="timeLimit";return;end
    % The input box implies ||R*dU||^2<=1. The native quadratic is exactly
    % the former epigraph penalty; it changes scaled CLF slack by at most
    % clfTieTolerance and penalizes departure from the anchor, not zero.
    objective=zeros(size(problem.safetyObjective));objective(problem.clfIndex)=1;
    search.clfStageAttempted=true;wall=tic;
    [second,flag,solverInfo]=localConicSolve(problem.quadratic,objective,problem.cones,problem.a,problem.b, ...
        problem.equal,problem.rhs,problem.lower,problem.upper,cfg,remaining, ...
        min(cfg.solver.optimalityTolerance,cfg.solver.constraintTolerance));
    search.solverCalls=search.solverCalls+1;
    search.stages(2)=struct('objective',"clfSlack",'exitFlag',flag,'seconds',toc(wall),'value',NaN,'numericalSolve',true,'solverInfo',solverInfo);
    if isempty(second) || any(~isfinite(second)) || flag<=0,search.terminationReason="clfNoNumericalResult";return;end
    solution=localDecode(second,problem,anchor);
    search.stages(2).value=solution.clfSlack;search.clfStageCompleted=true;search.returned=true;
    search.converged=all([search.stages([search.stages.numericalSolve]).exitFlag]>0);
    search.clfLowerBound=solution.clfSlack<=cfg.solver.feasibilityTolerance*problem.clfScale;
    search.terminationReason="twoStagesReturned";
end

function solution=localDecode(point,problem,anchor)
    inputs=anchor.inputs+reshape(point(problem.inputIndices),size(anchor.inputs));
    states=anchor.states+reshape(point(problem.stateIndices),size(anchor.states));
    slacks=max(0,point(problem.slackIndices));rho=problem.clfScale*max(0,point(problem.clfIndex));
    nextValue=norm(problem.nominalClfMap*point+problem.nominalClfOffset)^2*problem.clfScale;
    worstNext=(norm(problem.clfMap*point+problem.clfOffset)^2+problem.clfModelConstant)*problem.clfScale;
    solution=struct('inputs',inputs,'states',states,'stageSlacks',slacks.', ...
        'safety',sum(slacks),'hard',NaN,'clfSlack',rho, ...
        'clfInitialValue',problem.initialClfValue,'clfNextValue',nextValue, ...
        'clfFunction',problem.clf.function,'clfRequiredDecrease',problem.clf.requiredDecrease, ...
        'clfTieBound',problem.clfTieBound,'clfWorstNextValue',worstNext,'clfCurrentBudget',problem.clfCurrentBudget, ...
        'maximumCollisionTighteningMeters',problem.maximumCollisionTighteningMeters, ...
        'minimumCollisionMargin',NaN, ...
        'encounterExit',problem.encounterExit,'affineValidationPerformed',false,'nonlinearValidationPerformed',false);
end

function [point,flag,solverInfo]=localConicSolve(quadratic,objective,cones,a,b,equal,rhs,lower,upper,cfg,remaining,gapTolerance)
    persistent solver
    if isempty(solver)
        directory=fullfile(fileparts(fileparts(mfilename('fullpath'))),'solver','controller');
        if isfolder(directory),addpath(directory);end
        assert(exist('predictiveConicSolverMex','file')==3, ...
            'collisionAvoidanceController:missingConicSolver', ...
            'Run scripts/buildPredictiveConicSolver to compile the Clarabel adapter.');
        % Retain the resolved entry point when a caller restores MATLAB's
        % path (for example a test fixture). A cached availability flag alone
        % would survive that restoration while name resolution would fail.
        solver=@predictiveConicSolverMex;
    end
    count=numel(objective);identity=speye(count);low=isfinite(lower);high=isfinite(upper);
    rows=cell(1,numel(cones)+1);bounds=cell(size(rows));
    rows{1}=[equal;a;-identity(low,:);identity(high,:)];
    bounds{1}=[rhs;b;-lower(low);upper(high)];
    dimensions=[size(equal,1),size(a,1)+nnz(low)+nnz(high),zeros(1,numel(cones))];
    for k=1:numel(cones)
        cone=cones(k);rows{k+1}=[-cone.d.';-cone.A];bounds{k+1}=[-cone.gamma;-cone.b];
        dimensions(k+2)=size(cone.A,1)+1;
    end
    [point,flag,solverInfo]=solver(quadratic,objective, ...
        vertcat(rows{:}),vertcat(bounds{:}),dimensions,[0,1,2*ones(1,numel(cones))], ...
        [cfg.solver.maxIterations;remaining;cfg.solver.constraintTolerance;gapTolerance;cfg.solver.feasibilityTolerance]);
end

function cone=localCone(a,b,d,gamma)
    cone=struct('A',sparse(a),'b',full(b),'d',sparse(d),'gamma',gamma);
end

function [anchor,source,failure]=localInitialization(model,previous)
    % Startup: one potential-field rollout. Later samples: the shifted plan,
    % extended by path guidance. Either rollout stops at the first node that
    % reaches the terminal set, between horizonSteps and maximumHorizonSteps.
    % An unusable shifted plan is reported; no other anchor replaces it.
    cfg=model.cfg;minimum=cfg.controller.horizonSteps;maximum=cfg.controller.maximumHorizonSteps;
    anchor=[];failure="";
    if ~isstruct(previous)
        source="movingTargetPotentialField";if isempty(model.target),source="laneFeedbackRollout";end
        anchor=localPotentialFieldSeed(model,minimum,maximum);return;
    end
    source="shiftedInputRollout";
    shifted=previous.inputTrajectory(:,2:end);
    if isempty(shifted) || ~all(isfinite(shifted(:))) || ~all(abs(shifted(2,:))<1)
        failure="unusableShiftedInputs";return;
    end
    % The previous accepted plan, shifted by one hold. Its endpoint lies in
    % the terminal set at the same absolute time; only when the plan is
    % shorter than the prefix, or its endpoint is not in the set (a changed
    % forecast), is it extended by holds of the endpoint's backup: the one
    % whose set contains it, else the one whose CLF region it is closest to.
    domainErrors=["collisionAvoidanceController:nonlinearDomain", ...
        "collisionAvoidanceController:invalidTireOperatingPoint", ...
        "collisionAvoidanceController:singularTireLinearization"];
    count=size(shifted,2);inputs=shifted;states=zeros(6,count+1);states(:,1)=model.initialState;
    context=terminalSafeSet.context(model);appended=0;
    try
        for index=1:count
            [~,states(:,index+1)]=terminalSafeSet.holdStates(states(:,index),inputs(:,index),cfg);
        end
        [member,context]=terminalSafeSet.member(context,states(:,end),count);
        if count<minimum || ~member
            [mode,context]=terminalSafeSet.select(context,states(:,end),count);
            reference=context.modes(mode).reference;
        end
        while (count<minimum || ~member) && count<maximum
            [u,next,ok]=terminalSafeSet.terminalInput(states(:,end),inputs(:,end),model,reference);
            if ~ok,break;end
            inputs(:,end+1)=u;states(:,end+1)=next;count=count+1;appended=appended+1; %#ok<AGROW>
            [member,context]=terminalSafeSet.member(context,next,count);
        end
    catch exception
        if ~any(string(exception.identifier)==domainErrors),rethrow(exception);end
        failure=string(exception.identifier);return;
    end
    anchor=struct('inputs',inputs,'states',states,'terminalContext',context, ...
        'certified',member && count>=minimum,'appended',appended,'returnedHolds',0);
    anchor=localReturnToNominal(anchor,model);
end

function anchor=localReturnToNominal(anchor,model)
    % A plan whose endpoint is terminal only for another lane's backup is
    % extended by path guidance until a node in the nominal backup's set, and
    % the extension is kept only if every appended hold meets the hard rows
    % (state limits, handling envelope, road, collision clearance). The plan
    % before the endpoint is unchanged, so its prefix deficits are too. The
    % return is thus taken as soon as it is safe, and a plan that cannot yet
    % return keeps holding its lane.
    cfg=model.cfg;count=size(anchor.inputs,2);maximum=cfg.controller.maximumHorizonSteps;
    context=anchor.terminalContext;
    if count>=maximum || numel(context.modes)<2,return;end
    [nominal,context]=terminalSafeSet.member(context,anchor.states(:,end),count,1);
    anchor.terminalContext=context;
    if nominal,return;end
    reference=model.nominalReference;
    guidance=nonlinearBicycleModel.nominalGuidanceParameters(cfg,reference.curvature);
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    margin=cfg.collision.safetyMarginMeters;h=cfg.controller.sampleTime;
    inputs=anchor.inputs;states=anchor.states;x=states(:,end);previous=inputs(:,end);found=false;
    try
        for index=count+1:maximum
            u=nonlinearBicycleModel.nominalFeedback(x,previous,model.lane,reference,cfg,guidance);
            [middle,next]=terminalSafeSet.holdStates(x,u,cfg);
            if ~terminalSafeSet.admissible(x,middle,next,u,model) ...
                    || localClearance(middle,(index-.5)*h,model,shape)<margin ...
                    || localClearance(next,index*h,model,shape)<margin
                break;
            end
            inputs(:,index)=u;states(:,index+1)=next;x=next;previous=u;
            if mod(index-count,10)==0 || index==maximum
                [found,context]=terminalSafeSet.member(context,next,index,1);
                if found,break;end
            end
        end
    catch exception
        if ~startsWith(string(exception.identifier),"collisionAvoidanceController:"),rethrow(exception);end
        return;
    end
    if ~found,anchor.terminalContext=context;return;end
    anchor.inputs=inputs(:,1:index);anchor.states=states(:,1:index+1);anchor.terminalContext=context;
    anchor.returnedHolds=index-count;
end

function anchor=localPotentialFieldSeed(model,minimum,maximum)
    % The startup rollout: potential guidance with a target, path guidance
    % without one. It supplies a linearization, never an issued input, and
    % stops at the first node from horizonSteps on that is in the terminal set.
    cfg=model.cfg;reference=model.nominalReference;x=model.initialState;previous=model.previousInput;
    h=cfg.controller.sampleTime;
    inputs=zeros(2,maximum);states=zeros(6,maximum+1);states(:,1)=x;side=0;count=maximum;
    nominal=nonlinearBicycleModel.nominalGuidanceParameters(cfg,reference.curvature);
    context=terminalSafeSet.context(model);member=false;
    for index=1:maximum
        time=(index-1)*h;
        if isempty(model.target)
            u=nonlinearBicycleModel.nominalFeedback(x,previous,model.lane,reference,cfg,nominal);
        else
            [heading,speed,side]=predictiveSafetyGeometry.potentialGuidance(x,model.lane,model.target,time,cfg,side, ...
                model.road.lateralClearance);
            course=x(3)+atan2(x(5),x(4));
            yawRate=reference.curvature*hypot(x(4),x(5)) ...
                -cfg.nominalClf.courseGain*atan2(sin(course-heading),cos(course-heading));
            u=localGuidanceInput(x,previous,yawRate,speed,cfg,reference);
        end
        inputs(:,index)=u;[~,x]=terminalSafeSet.holdStates(x,u,cfg);states(:,index+1)=x;previous=u;
        if index>=minimum
            [member,context]=terminalSafeSet.member(context,x,index);
            if member,count=index;break;end
        end
    end
    anchor=struct('inputs',inputs(:,1:count),'states',states(:,1:count+1),'terminalContext',context, ...
        'certified',false,'appended',0,'seedReachedTerminalSet',member,'returnedHolds',0);
    if member,anchor=localReturnToNominal(anchor,model);end
end

function u=localGuidanceInput(x,previous,yawRate,speed,cfg,reference)
    tire=modifiedFialaTire.parameters(cfg);
    limit=cfg.initialization.lateralAccelerationFraction*min(tire.frictionCoefficient)*cfg.vehicle.gravity/max(hypot(x(4),x(5)),1);
    yawRate=min(limit,max(-limit,yawRate));
    b=reference.input(2)+cfg.initialization.speedGain*(speed-x(4));
    b=min(cfg.initialization.brakingRatioLimit,max(-cfg.initialization.brakingRatioLimit,b));
    clipped=localClip([0;b],previous,cfg);b=clipped(2);
    rear=modifiedFialaTire.evaluate([0;atan2(x(5)-cfg.vehicle.lr*x(6),x(4))],b,cfg);
    front=(cfg.vehicle.Iz*cfg.nominalClf.yawRateGain*(yawRate-x(6))+cfg.vehicle.lr*rear(2))/cfg.vehicle.lf;
    capacity=tire.longitudinalForceScale(1)*sqrt(1-b^2);
    fraction=min(abs(front)/capacity,cfg.initialization.frontForceFraction);
    slip=-sign(front)*atan(3*capacity*(1-(1-fraction)^(1/3))/tire.corneringStiffness(1));
    u=[atan2(x(5)+cfg.vehicle.lf*x(6),x(4))-slip;b];
end

function [problem,model]=localFormulate(anchor,model)
    model.linearizationBuilds=model.linearizationBuilds+1;
    cfg=model.cfg;count=size(anchor.inputs,2);prefix=cfg.controller.horizonSteps;
    robustRelaxation=any(model.uncertainty.egoGenerator(:));
    if ~isempty(model.uncertainty.target)
        robustRelaxation=robustRelaxation || any(structfun(@(value)any(value(:)>0),model.uncertainty.target));
    end
    % Collision rows of the first horizonSteps nodes carry slack. With an
    % observer enclosure every node also has a slack on its tightened row;
    % the untightened row stays hard after the prefix, so the formulation
    % reduces to the certainty-equivalent one as the enclosure vanishes.
    slackCount=prefix;
    if robustRelaxation,slackCount=count+1;end
    ix=reshape(1:6*(count+1),6,[]);iu=reshape(ix(end)+(1:2*count),2,[]);
    is=iu(end)+(1:slackCount);ic=is(end)+1;nv=ic;
    equal=sparse(6*(count+1),nv);rhs=zeros(6*(count+1),1);equal(1:6,ix(:,1))=eye(6);
    rhs(1:6)=model.initialState-anchor.states(:,1);
    rows=cell(1,9*count+8);bounds=cell(size(rows));rowCount=0;
    lower=-Inf(nv,1);upper=Inf(nv,1);radius=cfg.nonlinear.trustRadius;
    lower(ix(:))=-repmat(radius*[5;5;.5;5;3;1.5],count+1,1);upper(ix(:))=-lower(ix(:));
    lower(iu(:))=-repmat(model.inputTrustScale*radius*[.15;.25],count,1);upper(iu(:))=-lower(iu(:));lower([is,ic])=0;
    % Numerical RTI corrections are local to the new anchor at every sample.
    % Steering still has no actuator magnitude or slew constraint.
    inputLower=[-Inf;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
    inputUpper=[Inf;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    rate=[Inf;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    [physicalLower,physicalUpper]=localStateLimits(cfg);
    primaryLowerBound=0;departure=0;
    % Information-consistent uncertainty. The estimator publishes a fresh
    % enclosure G0 of its estimate at every sample, and every hold of the
    % plan is issued from such an estimate. Hold i therefore starts from G0
    % and propagates it through that hold only: G(t_i+tau) = Phi_i(tau) G0,
    % and the target set restarts from its predicted state at t_i. The
    % first hold is the certified one; later holds assume that the estimator's
    % enclosure at their start equals the present one. Issued inputs carry
    % no uncertainty of their own.
    initialGenerator=model.uncertainty.egoGenerator;generator=initialGenerator;
    holdCenter=model.initialState;holdStart=0;h=cfg.controller.sampleTime;
    stages=localLinearization(anchor,model);
    physicalInputLower=repmat(inputLower,1,count);physicalInputUpper=repmat(inputUpper,1,count);
    maximumTightening=0;
    times=(0:2*count)*h/2;
    % Record time j*h/2 lies in hold ceil(j/2), which starts at (ceil(j/2)-1)*h.
    targetTube=localTargetTube(model,times,max(0,ceil((0:2*count)/2)-1)*h);
    uncertaintyPrediction=struct('time',times,'target',targetTube, ...
        'egoStateRadius',zeros(6,2*count+1),'collisionStateRadius',zeros(6,2*count+1));
    primaryCones=struct('A',{},'b',{},'d',{},'gamma',{});
    for index=1:count
        x=anchor.states(:,index);input=anchor.inputs(:,index);nextReference=anchor.states(:,index+1);
        stage=stages(index);middle=stage.middle;am=stage.am;bm=stage.bm;
        next=stage.next;a=stage.a;b=stage.b;
        % Node index closes hold index-1 (or is the present sample).
        collisionGenerator=localRelativeGenerator(x,generator,model,holdCenter);
        middleGenerator=am*initialGenerator;nextGenerator=a*initialGenerator;
        middleCenter=model.initialState;if index>1,middleCenter=x;end
        middleCollisionGenerator=localRelativeGenerator(middle,middleGenerator,model,middleCenter);
        uncertaintyPrediction.egoStateRadius(:,2*index-1:2*index)=[sum(abs(generator),2),sum(abs(middleGenerator),2)];
        uncertaintyPrediction.collisionStateRadius(:,2*index-1:2*index)=[sum(abs(collisionGenerator),2),sum(abs(middleCollisionGenerator),2)];
        eq=6*index+(1:6);equal(eq,ix(:,index+1))=eye(6);equal(eq,ix(:,index))=-a;equal(eq,iu(:,index))=-b;
        rhs(eq)=next-nextReference;
        lower(iu(:,index))=max(lower(iu(:,index)),inputLower-input);
        upper(iu(:,index))=min(upper(iu(:,index)),inputUpper-input);
        r=sparse(2,nv);r(:,iu(:,index))=eye(2);previous=model.previousInput;
        if index>1,r(:,iu(:,index-1))=-eye(2);previous=anchor.inputs(:,index-1);end
        finite=isfinite(rate);difference=input-previous;
        rowCount=rowCount+1;rows{rowCount}=[r(finite,:);-r(finite,:)];bounds{rowCount}=[rate(finite)-difference(finite);rate(finite)+difference(finite)];
        map=sparse(6,nv);map(:,ix(:,index))=eye(6);
        % The current measured node is fixed; every later node stays on the road.
        if index>1 && ~isempty(model.road.lateralClearance)
            [r,bound]=localRoadRows(x,model,generator,map);
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=bound;
        end
        if ~localBeyondRange(x,(index-1)*h,model,collisionGenerator,holdStart)
            departure=index;
            [g,j,tightening]=localSafetyRows(x,(index-1)*h,model,collisionGenerator,holdStart);
            maximumTightening=max(maximumTightening,max(tightening));
            if index==1,primaryLowerBound=max(0,-min(g));end
            r=-j*map;
            if robustRelaxation && index>prefix
                rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g+tightening;
            end
            if index<=slackCount,r(:,is(index))=-1;end
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
        end
        map(:,ix(:,index))=am;map(:,iu(:,index))=bm;
        if ~isempty(model.road.lateralClearance)
            [r,bound]=localRoadRows(middle,model,middleGenerator,map);
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=bound;
        end
        if ~localBeyondRange(middle,(index-.5)*h,model,middleCollisionGenerator,(index-1)*h)
            departure=index;
            [g,j,tightening]=localSafetyRows(middle,(index-.5)*h,model,middleCollisionGenerator,(index-1)*h);
            maximumTightening=max(maximumTightening,max(tightening));
            r=-j*map;
            if robustRelaxation && index>prefix
                rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g+tightening;
            end
            if index<=slackCount,r(:,is(index))=-1;end
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
        end
        rowCount=rowCount+1;rows{rowCount}=[map(4:6,:);-map(4:6,:)];
        middleRadius=sum(abs(middleGenerator(4:6,:)),2);nextRadius=sum(abs(nextGenerator(4:6,:)),2);
        bounds{rowCount}=[physicalUpper-middle(4:6)-middleRadius;middle(4:6)-physicalLower-middleRadius];
        jx=ix(:,index+1);
        lower(jx(4:6))=max(lower(jx(4:6)),physicalLower-nextReference(4:6)+nextRadius);
        upper(jx(4:6))=min(upper(jx(4:6)),physicalUpper-nextReference(4:6)-nextRadius);
        % Handling envelope at both ends of the hold, under its own input.
        % The measured node is fixed and is not constrained.
        nodeMap=sparse(6,nv);nodeMap(:,jx)=speye(6);
        [r,bound,cone]=localHandling(nextReference,input,iu(2,index),model,nextGenerator,nodeMap);
        rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=bound;primaryCones(end+1)=cone; %#ok<AGROW>
        if index>1
            nodeMap=sparse(6,nv);nodeMap(:,ix(:,index))=speye(6);
            [~,~,cone]=localHandling(x,input,iu(2,index),model,generator,nodeMap);
            primaryCones(end+1)=cone; %#ok<AGROW>
        end
        generator=nextGenerator;holdCenter=middleCenter;holdStart=(index-1)*h;
    end
    collisionGenerator=localRelativeGenerator(anchor.states(:,end),generator,model,holdCenter);
    uncertaintyPrediction.egoStateRadius(:,end)=sum(abs(generator),2);
    uncertaintyPrediction.collisionStateRadius(:,end)=sum(abs(collisionGenerator),2);
    model.uncertaintyPrediction=uncertaintyPrediction;
    % Terminal node: the collision rows of every node inside the range, the
    % road, and the terminal set as one CLF-level cone
    %   ||F (e(y) + J dx_N)|| <= sqrt(c*),
    % c* the largest level whose tube at the anchor endpoint's station misses
    % the target (terminalSafeSet.level). The anchor endpoint meets it when
    % it is in the set, so the zero correction stays feasible.
    y=anchor.states(:,end);map=sparse(6,nv);map(:,ix(:,end))=eye(6);
    time=count*h;
    if ~localBeyondRange(y,time,model,collisionGenerator,holdStart)
        departure=count+1;
        [g,j,tightening]=localSafetyRows(y,time,model,collisionGenerator,holdStart);
        maximumTightening=max(maximumTightening,max(tightening));
        r=-j*map;
        if robustRelaxation && count+1>prefix
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g+tightening;
        end
        if count+1<=slackCount,r(:,is(count+1))=-1;end
        rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
    end
    if ~isempty(model.road.lateralClearance)
        [r,bound]=localRoadRows(y,model,generator,map);
        rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=bound;
    end
    context=localTerminalContext(anchor,model);
    % The endpoint's backup: the one whose set contains the anchor endpoint
    % (nominal first), else the one whose CLF region it is closest to.
    [mode,context]=terminalSafeSet.select(context,y,count);
    [terminalLevel,context,terminalInfo]=terminalSafeSet.level(context,y,count,mode);
    % The bisection returns a verified level within its tolerance below the
    % exact one; an endpoint verified in the set keeps its own level.
    [member,~,memberInfo]=terminalSafeSet.member(context,y,count,mode);
    if member,terminalLevel=max(terminalLevel,memberInfo.value);end
    reference=context.modes(mode).reference;
    [e,jacobian]=nonlinearBicycleModel.errorLinearization(y,model.lane,reference);
    primaryCones(end+1)=localCone(reference.factor*jacobian*map,-reference.factor*e, ...
        sparse(nv,1),-sqrt(max(terminalLevel,0)));
    objective=zeros(nv,1);objective(is)=1;
    problem=struct('a',vertcat(rows{1:rowCount}),'b',vertcat(bounds{1:rowCount}), ...
        'equal',equal,'rhs',rhs,'lower',lower,'upper',upper,'cones',primaryCones, ...
        'primaryConeCount',numel(primaryCones), ...
        'stateIndices',ix,'inputIndices',iu,'slackIndices',is,'clfIndex',ic, ...
        'safetyObjective',objective,'primaryLowerBound',primaryLowerBound, ...
        'encounterExit',departure, ...
        'maximumCollisionTighteningMeters',maximumTightening, ...
        'terminalLevel',terminalLevel,'terminalAnchorValue',terminalInfo.value, ...
        'terminalLaneOffset',context.modes(mode).lateralOffset, ...
        'terminalAnchorExitSeconds',terminalInfo.exitSeconds,'terminalAnchorReason',terminalInfo.reason);
    model.robustnessRelaxation=robustRelaxation;
    problem.physicalInputLower=physicalInputLower;problem.physicalInputUpper=physicalInputUpper;
end

function stages=localLinearization(anchor,model)
    count=size(anchor.inputs,2);stages=repmat(struct('middle',[],'am',[],'bm',[], ...
        'next',[],'a',[],'b',[]),1,count);
    for index=1:count
        [stages(index).middle,stages(index).am,stages(index).bm, ...
            stages(index).next,stages(index).a,stages(index).b]= ...
            nonlinearBicycleModel.hold(anchor.states(:,index),anchor.inputs(:,index),model.cfg);
    end
end

function relative=localRelativeGenerator(x,generator,model,center)
    relative=generator;
    if ~model.uncertainty.relativeFrame,return;end
    % The target is measured relative to the ego, so the pose error of the
    % estimate that starts a hold moves both rigidly about the ego position
    % at that hold start (center). Keep those columns through the hold map
    % and subtract the same rigid motion only for pairwise geometry.
    offset=x(1:2)-center(1:2);
    rigid=[eye(2),[-offset(2);offset(1)];0,0,1];
    relative(1:3,1:3)=relative(1:3,1:3)-rigid*model.uncertainty.egoGenerator(1:3,1:3);
end


function problem=localAddClf(problem,anchor,model)
    nv=numel(problem.safetyObjective);iu=problem.inputIndices;ic=problem.clfIndex;count=size(iu,2);
    [nominalMap,nominalOffset,~,value,scale,~,clf]=localNominalClf(anchor,model,problem.stateIndices(:,2),nv);
    e0=nonlinearBicycleModel.errorLinearization(model.initialState,model.lane,model.nominalReference);
    scales=[model.cfg.clf.lateralPositionErrorScale;model.cfg.clf.headingErrorScale; ...
        model.cfg.clf.speedErrorScale;model.cfg.clf.lateralVelocityErrorScale;model.cfg.clf.yawRateErrorScale];
    lossLower=norm(e0./scales)^2;
    % The CLF is a soft performance row on the estimate: V(xhat+) <= rho
    % V(xhat). Estimation error enters it as an input (ISS); it carries no
    % observer enclosure. A worst case over the enclosure would ask for a
    % decrease below the enclosure's own size, which no input achieves near
    % the path, and would turn this stage into a greedy one-step minimizer.
    currentBudget=model.nominalReference.contraction*value;
    map=nominalMap;offset=nominalOffset;
    modelConstant=0;constant=currentBudget/scale;
    axis=sparse(1,nv);axis(ic)=1;
    cone=localCone([2*map;axis],[-2*offset;1-constant],axis.',-1-constant);
    inputScale=repmat(1./(model.inputTrustScale*model.cfg.nonlinear.trustRadius*[.15;.25]*sqrt(2*count)),count,1);
    increment=sparse(1:numel(iu),iu(:),inputScale,numel(iu),nv);
    problem.clfTieBound=model.cfg.solver.clfTieTolerance*lossLower;
    problem.quadratic=2*problem.clfTieBound/scale*(increment.'*increment);
    problem.cones=[problem.cones,cone];
    problem.clfScale=scale;problem.clfMap=map;problem.clfOffset=offset;
    problem.nominalClfMap=nominalMap;problem.nominalClfOffset=nominalOffset;
    problem.clfCurrentBudget=currentBudget;
    problem.clfModelConstant=modelConstant;problem.clf=clf;problem.initialClfValue=value;
    problem.initialClfSlack=max(0,(norm(offset)^2-constant)*scale);
end

function [map,offset,constant,value,scale,modelConstant,clf]=localNominalClf(anchor,model,next,nv)
    % One analytic transverse quadratic on the single affine state model.
    reference=model.nominalReference;
    [e1,jacobian]=nonlinearBicycleModel.errorLinearization(anchor.states(:,2),model.lane,reference);
    value=nonlinearBicycleModel.nominalValue(model.initialState,model.lane,reference);scale=max(1,value);
    % V(next) <= rho V(now): the error sqrt(V) shrinks by at least 1/e per
    % clf.convergenceTimeConstantSeconds.
    required=(1-reference.contraction)*value;
    map=sparse(5,nv);map(:,next)=reference.factor*jacobian/sqrt(scale);
    offset=reference.factor*e1/sqrt(scale);modelConstant=0;
    constant=(value-required)/scale;
    clf=struct('function',"quadraticTransverseError",'requiredDecrease',required);
end

function u=localClip(u,previous,cfg)
    % Only initialization braking is clipped; steering passes through unchanged.
    lower=[-Inf;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
    upper=[Inf;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    rate=[Inf;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    u=min(upper,max(lower,min(previous+rate,max(previous-rate,u))));
end

function [values,jacobian,tightening]=localSafetyRows(x,time,model,generator,holdStart)
    cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    q=localTargetAt(model,time);
    dual=predictiveSafetyGeometry.supportLinearization(x(1:3),shape,q(1:3),q(8:11));
    values=dual.value-cfg.collision.safetyMarginMeters;
    jacobian=[dual.jacobian,zeros(4,3)];
    % Directional position support retains correlation across the affine
    % ego dynamics. Rotation uses an exact chord bound for each rectangle.
    normal=jacobian(:,1:2);normalNorm=vecnorm(normal,2,2);
    [tube,targetSupport]=localTargetTube(model,time,holdStart,normal);
    egoReach=norm(shape(1:2))+norm(shape(3:4));
    targetReach=norm(q(8:9))+norm(q(10:11));
    yawRadius=sum(abs(generator(3,:)));
    reserve=2*egoReach*sin(min(pi,yawRadius)/2) ...
        +2*targetReach*sin(tube.yawRadius/2);
    tightening=sum(abs(normal*generator(1:2,:)),2)+targetSupport+normalNorm*reserve;
    values=values-tightening;
end

function [tube,support]=localTargetTube(model,time,holdStart,normal)
    % Target enclosure of the hold that starts at holdStart: the observer
    % set restarted from the target's predicted state there, for the time
    % elapsed in the hold. Same constant-parameter family as before.
    tube=struct('positionRadius',zeros(size(time)),'yawRadius',zeros(size(time)), ...
        'courseRadius',zeros(size(time)),'speedRadius',zeros(size(time)));support=[];
    if isempty(model.target) || isempty(model.uncertainty.target)
        if nargin>3,support=zeros(size(normal,1),1);end
        return;
    end
    % One predicted start state per entry; the tube formulas are elementwise.
    starts=localTargetAt(model,holdStart);
    tube=predictiveSafetyGeometry.targetErrorTube(starts,model.uncertainty.target,time-holdStart);
    if nargin>3
        support=predictiveSafetyGeometry.targetPositionSupport(starts,model.uncertainty.target,time-holdStart,normal);
    end
end

function beyond=localBeyondRange(x,time,model,generator,holdStart)
    beyond=isempty(model.target);
    if ~beyond
        cfg=model.cfg;
        q=localTargetAt(model,time);
        tube=localTargetTube(model,time,holdStart);
        reserve=sum(vecnorm(generator(1:2,:)))+tube.positionRadius;
        beyond=norm(x(1:2)-q(1:2))-reserve>cfg.collision.encounterRangeMeters;
    end
end

function [rows,bounds,cone]=localHandling(y,input,brakingIndex,model,generator,map)
    % Stable handling envelope and the estimator's domain at one node, under
    % the input of an adjacent hold.
    %   Rear adhesion: the rear Fiala tire stays below saturation,
    %     |w| <= k v_x sqrt(1-b^2),  w = v_y - lr r,  k = 3 mu_r Fz_r / C_r,
    %   i.e. the cone ||[w/k; v_x b]|| <= v_x with v_x frozen at the anchor in
    %   the product v_x b. A saturated rear axle has no restoring yaw moment
    %   (the vehicle spins) and its force no longer determines the slip, so
    %   the estimator's force-balance measurement loses its information. It is
    %   a handling and observability row, not part of the safety certificate,
    %   and acts on the estimate: the certified lateral-velocity radius widens
    %   exactly as the rear slip nears saturation, and its reserve excluded
    %   states well inside the envelope.
    %   Sideslip cone: |v_y| <= tan(model.sideslipMaximum) v_x, the premise of
    %   the estimator's force-balance and course inversions, robust to the
    %   ego enclosure (generator).
    % The cone is divided by the anchor speed, so its entries are the
    % normalized rear slip tan(alpha_r)/tan(alpha_sat), the braking ratio and
    % their unit bound: a well-scaled cone for the interior-point solver.
    cfg=model.cfg;tire=modifiedFialaTire.parameters(cfg);
    k=3*tire.longitudinalForceScale(2)/tire.corneringStiffness(2);
    slip=[0,0,0,0,1,-cfg.vehicle.lr];forward=[0,0,0,1,0,0];
    brake=sparse(1,brakingIndex,1,1,size(map,2));scale=max(y(4),cfg.model.scheduleSpeedFloor);
    cone=localCone([slip*map/(k*scale);brake],-[slip*y/(k*scale);input(2)], ...
        (forward*map).'/scale,-y(4)/scale);
    tangent=tan(cfg.model.sideslipMaximum);side=[0,0,0,-tangent,1,0;0,0,0,-tangent,-1,0];
    rows=side*map;bounds=-side*y-sum(abs(side*generator),2);
end

function [rows,bounds]=localRoadRows(y,model,generator,map)
    % The ego drives on the road: every rectangle corner within
    % lateralClearance=[right;left] of the path, linearized at the anchor
    % corner on the local path normal. Applied at every predicted node.
    cfg=model.cfg;clearance=model.road.lateralClearance;
    corners=cfg.vehicle.rectangleOffset+[cfg.vehicle.length;cfg.vehicle.width]/2.*[1,1,-1,-1;1,-1,1,-1];
    c=cos(y(3));s=sin(y(3));rotation=[c,-s;s,c];derivative=[-s,-c;c,-s];
    rows=sparse(8,size(map,2));bounds=zeros(8,1);
    for k=1:4
        projection=laneGeometry.project(y(1:2)+rotation*corners(:,k),model.lane);
        normal=[-sin(projection.heading);cos(projection.heading)];
        jacobian=normal.'*[eye(2),derivative*corners(:,k)];
        reserve=sum(abs(jacobian*generator(1:3,:)));
        r=jacobian*map(1:3,:);
        rows(2*k-1,:)=r;bounds(2*k-1)=clearance(2)-projection.lateralPosition-reserve;
        rows(2*k,:)=-r;bounds(2*k)=clearance(1)+projection.lateralPosition-reserve;
    end
end

function solution=localTerminalMetrics(solution,model,acceptance)
    % Terminal quantities of the issued (nonlinear) plan, for records.
    cfg=model.cfg;y=solution.states(:,end);count=size(solution.inputs,2);
    solution.terminalValue=acceptance.terminalValue;solution.terminalLevel=acceptance.terminalLevel;
    solution.terminalExitSeconds=acceptance.terminalExitSeconds;
    solution.terminalLaneOffset=acceptance.terminalLaneOffset;solution.terminalEnd=acceptance.terminalEnd;
    solution.acceptedStep=acceptance.step;solution.candidateFeasible=acceptance.candidateFeasible;
    solution.linearizationGap=acceptance.linearizationGap;
    solution.terminalDistanceMeters=Inf;solution.terminalRoadMarginMeters=Inf;
    if ~isempty(model.target)
        q=localTargetAt(model,count*cfg.controller.sampleTime);
        solution.terminalDistanceMeters=norm(y(1:2)-q(1:2));
    end
    if ~isempty(model.road.lateralClearance)
        solution.terminalRoadMarginMeters=terminalSafeSet.roadMargin(y,model);
    end
end

function [low,high]=localStateLimits(cfg)
    low=[max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor+1e-4);-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
    high=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
end

function q=localTargetAt(model,time)
    q=predictiveSafetyGeometry.predictTarget(model.targetEpoch,model.sampleIndex*model.cfg.controller.sampleTime+time);
end
