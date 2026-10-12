function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl One affine model per sample; no fallback algorithm.
% Startup, and the sample at which a target appears that the previous plan
% did not know of, use a potential-field rollout; later samples start from
% the previous solved plan shifted by one hold. The horizon ends where the
% anchor enters the terminal set (terminalSafeSet): the joint continuation
% under the terminal law (the ego brakes at a constant rate with its
% sideslip and course curvature held until it stops, the target keeps its
% forecast) clears the target until it leaves the perception range, and the
% road throughout.
% A shifted plan shorter than horizonSteps is extended by holds of the
% declared model whose endpoint stays in the set: the CLF's certified cruise
% hold, or else the hold nearest to it toward the held steering (at the
% cruise hold's braking ratio, else braking as the terminal law); a shifted
% plan whose endpoint is no longer in the set is extended by cruise holds.
% The PCBF stage minimizes prefix safety slack, then the CLF stage runs; the
% terminal rows are the set's conditions linearized at the anchor endpoint
% (collision rows at the continuation's nodes and midpoints and at the parked
% pose, road rows along it). A primary
% problem infeasible inside the trust region is re-solved on the same
% linearization with the trust scale doubled until it is feasible or
% reaches trustMaximumScale. A shifted plan still infeasible is then solved
% from a fresh potential-field rollout in the same way.
% The completed CLF program supplies the issued inputs directly. There is no
% post-solve step scaling, feasibility veto or acceptance relinearization.
% A nonlinear rollout is prediction data only; it does not change the inputs
% or the optimized slacks. Its remainder sets the next sample's trust scale.
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
        model.trust=localTrustRecord(model.trust,solution,model,search.rollout);
        solution=localTerminalMetrics(solution,model);
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
    % of it (observation innovation). The prediction is a nonlinear rollout, so this
    % is the estimator's and the plant's share; the model share is measured
    % by the completed input's prediction (localTrustRecord).
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

function trust=localTrustRecord(trust,solution,model,rollout)
    % Store the correction actually taken, in units of the unit input box,
    % and update the error coefficient from the full affine step: its
    % nonlinear rollout departs from the affine prediction by the
    % second-order remainder L*s^2 (s its size in the unit box).
    cfg=model.cfg;unit=cfg.nonlinear.trustRadius*[.15;.25];
    anchor=model.linearization.inputs;count=min(size(anchor,2),size(solution.inputs,2));
    trust.correction=max(abs(solution.inputs(:,1:count)-anchor(:,1:count))./unit,[],'all');
    if ~isfinite(rollout.fullGap) || ~(rollout.fullCorrection>0),return;end
    trust.modelInnovation=rollout.fullGap;
    % A step far inside the box carries little information about its boundary.
    step=max(rollout.fullCorrection,trust.scale/2);
    curvature=max(rollout.fullGap/step^2,cfg.nonlinear.trustRetention*trust.curvature);
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
    [solution,search.rollout]=localPrediction(solution,anchor,model);
    search.attempts=attempts;search.selectedAttempt=selected;search.stages=stages;
    search.solverCalls=sum([stages.numericalSolve]);search.potentialFieldRestarted=restarted;
    search.initializationFailure=failure;search.elapsedSeconds=toc(timer);
    search.anchorCertified=isfield(anchor,'certified') && anchor.certified;
    search.appendedTerminalSteps=0;if isfield(anchor,'appended'),search.appendedTerminalSteps=anchor.appended;end
    search.appendedBrakingSteps=0;if isfield(anchor,'appendedBraking'),search.appendedBrakingSteps=anchor.appendedBraking;end
    search.appendedGuidedSteps=0;if isfield(anchor,'appendedGuided'),search.appendedGuidedSteps=anchor.appendedGuided;end
    search.extensionStopped="";if isfield(anchor,'extensionStopped'),search.extensionStopped=anchor.extensionStopped;end
    search.anchorHolds=size(anchor.inputs,2);
    search.seedReachedTerminalSet=NaN;
    if isfield(anchor,'seedReachedTerminalSet'),search.seedReachedTerminalSet=anchor.seedReachedTerminalSet;end
end

function [solution,rollout]=localPrediction(solution,anchor,model)
    % Prediction and trust data only. Never replace, rescale or reject the
    % completed program's inputs, slacks or objective on a nonlinear check.
    rollout=struct('available',false,'reason',"noCompletedOptimization", ...
        'fullGap',NaN,'fullCorrection',NaN);
    if isempty(solution),return;end
    solution.affineStates=solution.states;
    solution.inputCorrection=solution.inputs-anchor.inputs;
    unit=model.cfg.nonlinear.trustRadius*[.15;.25];
    rollout.fullCorrection=max(abs(solution.inputCorrection)./unit,[],'all');
    states=zeros(size(solution.states));states(:,1)=model.initialState;
    try
        for index=1:size(solution.inputs,2)
            states(:,index+1)=nonlinearBicycleModel.sample(states(:,index),solution.inputs(:,index),model.cfg);
        end
    catch exception
        if ~startsWith(string(exception.identifier),"collisionAvoidanceController:"),rethrow(exception);end
        % The affine prediction remains available even outside the nonlinear
        % model's domain. A diagnostic failure does not veto a solved input.
        rollout.reason=string(exception.identifier);return;
    end
    rollout.available=true;rollout.reason="predictionOnly";
    rollout.fullGap=max(localPoseDiscrepancy(states,solution.affineStates,model.cfg));
    solution.states=states;
end

function context=localTerminalContext(anchor,model)
    if isfield(anchor,'terminalContext') && ~isempty(anchor.terminalContext)
        context=anchor.terminalContext;
    else
        context=terminalSafeSet.context(model);
    end
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
        [point,problem,search,model]=localPrimary(anchor,model,source,timer,problem);
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

function [point,problem,search,model]=localPrimary(anchor,model,initialization,timer,problem)
    % Formulate the anchor's problem and solve its safety stage. A problem
    % already formulated on this anchor (a larger trust scale) is reused:
    % the scale changes only the input increments' bounds.
    cfg=model.cfg;point=[];formulationSeconds=0;
    if nargin<5 || isempty(problem)
        wall=tic;[problem,model]=localFormulate(anchor,model);formulationSeconds=toc(wall);
        model.linearization=anchor;
        % The next sample inherits this plan's later inputs only when the
        % anchor is the previous solved plan (localAddClf).
        problem.inheritedAnchor=initialization=="shiftedInputRollout";
    else
        problem=localInputBounds(problem,model);
    end
    search=localSearch(problem,model,initialization,formulationSeconds);
    remaining=cfg.solver.timeLimitSeconds-toc(timer);if remaining<=0,return;end
    primaryCount=problem.clfIndex-1;primaryCone=problem.cones(1:problem.primaryConeCount);wall=tic;
    coneFeasible=true;
    for index=1:numel(primaryCone)
        cone=primaryCone(index);
        coneFeasible=coneFeasible && all(vecnorm(reshape(cone.b,cone.size,[]),2,1)<=-cone.gamma(:).');
        primaryCone(index).A=cone.A(:,1:primaryCount);primaryCone(index).D=cone.D(:,1:primaryCount);
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
    % Each cone block holds cones of equal dimension; every cone enters the
    % solver as its d-row followed by its rows of A.
    coneDimensions=cell(1,numel(cones));
    for k=1:numel(cones)
        cone=cones(k);m=numel(cone.gamma);n=cone.size;
        order=reshape([1:m;m+reshape(1:m*n,n,m)],[],1);
        stacked=[-cone.D;-cone.A];offsets=[-cone.gamma(:);-cone.b];
        rows{k+1}=stacked(order,:);bounds{k+1}=offsets(order);coneDimensions{k}=repmat(n+1,1,m);
    end
    coneDimensions=[coneDimensions{:}];
    dimensions=[size(equal,1),size(a,1)+nnz(low)+nnz(high),coneDimensions];
    [point,flag,solverInfo]=solver(quadratic,objective, ...
        vertcat(rows{:}),vertcat(bounds{:}),dimensions,[0,1,2*ones(1,numel(coneDimensions))], ...
        [cfg.solver.maxIterations;remaining;cfg.solver.constraintTolerance;gapTolerance;cfg.solver.reducedTolerance]);
end

function [anchor,source,failure]=localInitialization(model,previous)
    % Startup and target onset: one potential-field rollout. Later samples:
    % the shifted plan, extended by path guidance. Either rollout stops at
    % the first node that reaches the terminal set, between horizonSteps and
    % maximumHorizonSteps.
    % An unusable shifted plan is reported; no other anchor replaces it.
    cfg=model.cfg;minimum=cfg.controller.horizonSteps;maximum=cfg.controller.maximumHorizonSteps;
    anchor=[];failure="";
    if ~isstruct(previous) || (isfield(model,'targetOnset') && model.targetOnset)
        source="movingTargetPotentialField";if isempty(model.target),source="laneFeedbackRollout";end
        if isstruct(previous),failure="targetOnset";end
        anchor=localPotentialFieldSeed(model,minimum,maximum);return;
    end
    source="shiftedInputRollout";
    shifted=previous.inputTrajectory(:,2:end);
    if isempty(shifted) || ~all(isfinite(shifted(:))) || ~all(abs(shifted(2,:))<1)
        failure="unusableShiftedInputs";return;
    end
    % The previous solved plan, shifted by one hold. Its endpoint lies in
    % the terminal set at the same absolute time. When the plan is shorter
    % than the prefix it is extended by holds of the declared model that keep
    % the endpoint in the set: the cruise hold of the CLF's certified
    % controller, or else the hold nearest to it toward the held steering
    % (terminalSafeSet.brakingInput).
    % An endpoint no longer in the set (a changed forecast) is extended by
    % cruise holds until one is.
    domainErrors=["collisionAvoidanceController:nonlinearDomain", ...
        "collisionAvoidanceController:invalidTireOperatingPoint", ...
        "collisionAvoidanceController:singularTireLinearization"];
    count=size(shifted,2);inputs=shifted;states=zeros(6,count+1);states(:,1)=model.initialState;
    context=terminalSafeSet.context(model);appended=0;braking=0;guided=0;stopped="";
    try
        for index=1:count
            [~,states(:,index+1)]=terminalSafeSet.holdStates(states(:,index),inputs(:,index),cfg);
        end
        [member,context]=terminalSafeSet.member(context,states(:,end),count);
        while (count<minimum || ~member) && count<maximum
            [u,next,ok,~,middle]=terminalSafeSet.cruiseInput(states(:,end),model);
            stays=false;
            if ok,[stays,context]=terminalSafeSet.member(context,next,count+1);end
            stays=stays && terminalSafeSet.clears(context,middle,count+.5);
            if member && ~stays
                % First, guidance through states outside the set until it
                % is reached again: a longer plan that completes the
                % maneuver instead of braking to keep the endpoint mild.
                [through,passed,context]=localGuideToSet(states(:,end),inputs(:,end),count,model,context,maximum);
                if passed
                    n=size(through.inputs,2);
                    inputs=[inputs,through.inputs];states=[states,through.states]; %#ok<AGROW>
                    count=count+n;appended=appended+n;guided=guided+n;member=true;continue;
                end
                [u,next,stays]=terminalSafeSet.brakingInput(states(:,end),inputs(:,end),u,model,context,count);
                if ~stays,stopped="noHoldStaysTerminal";break;end
                braking=braking+1;
            elseif ~ok
                stopped="cruiseHoldInadmissible";break;
            end
            inputs(:,end+1)=u;states(:,end+1)=next;count=count+1;appended=appended+1; %#ok<AGROW>
            member=stays;
        end
    catch exception
        if ~any(string(exception.identifier)==domainErrors),rethrow(exception);end
        failure=string(exception.identifier);return;
    end
    anchor=struct('inputs',inputs,'states',states,'terminalContext',context, ...
        'certified',member && count>=minimum,'appended',appended,'appendedBraking',braking, ...
        'appendedGuided',guided,'extensionStopped',stopped);
end

function [through,passed,context]=localGuideToSet(x,previous,count,model,context,maximum)
    % Holds of the startup guidance (localGuidance) from the endpoint x at
    % plan node 'count' until a node lies in the terminal set again, each
    % meeting the hard rows and clearing the target at its midpoint and
    % endpoint, at most up to maximumHorizonSteps; passed is false when a
    % hold fails or the set is not reached.
    cfg=model.cfg;through=struct('inputs',zeros(2,0),'states',zeros(6,0));passed=false;side=0;
    nominal=nonlinearBicycleModel.nominalGuidanceParameters(cfg,model.nominalReference.curvature);
    while count<maximum
        [u,side]=localGuidance(x,previous,count*cfg.controller.sampleTime,model,side,nominal);
        try
            [middle,next]=terminalSafeSet.holdStates(x,u,cfg);
        catch exception
            if startsWith(string(exception.identifier),"collisionAvoidanceController:"),return;end
            rethrow(exception);
        end
        if ~terminalSafeSet.admissible(x,middle,next,u,model) || ~terminalSafeSet.clears(context,middle,count+.5) ...
                || ~terminalSafeSet.clears(context,next,count+1)
            return;
        end
        through.inputs(:,end+1)=u;through.states(:,end+1)=next;count=count+1;x=next;previous=u;
        [passed,context]=terminalSafeSet.member(context,next,count);
        if passed,return;end
    end
end

function [u,side]=localGuidance(x,previous,time,model,side,nominal)
    % One input of the startup guidance at the time 'time' after now:
    % potential guidance with a target, path guidance without one.
    cfg=model.cfg;reference=model.nominalReference;
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
        [u,side]=localGuidance(x,previous,(index-1)*h,model,side,nominal);
        inputs(:,index)=u;[~,x]=terminalSafeSet.holdStates(x,u,cfg);states(:,index+1)=x;previous=u;
        if index>=minimum
            [member,context]=terminalSafeSet.member(context,x,index);
            if member,count=index;break;end
        end
    end
    anchor=struct('inputs',inputs(:,1:count),'states',states(:,1:count+1),'terminalContext',context, ...
        'certified',false,'appended',0,'seedReachedTerminalSet',member);
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
    % Every inequality row is a small dense block on the few variables it
    % involves (a node's state, or a hold's start state and input, and its
    % slack), in the order of the rows; the sparse matrices are built once
    % from these blocks (localAssemble). Building them row by row in sparse
    % form cost time quadratic in the plan length.
    blockRows=cell(1,12*count+8);blockColumns=blockRows;blockBounds=blockRows;blockCount=0;
    lower=-Inf(nv,1);upper=Inf(nv,1);radius=cfg.nonlinear.trustRadius;
    lower(ix(:))=-repmat(radius*[5;5;.5;5;3;1.5],count+1,1);upper(ix(:))=-lower(ix(:));
    lower([is,ic])=0;
    % Numerical RTI corrections are local to the new anchor at every sample.
    % Steering still has no actuator magnitude or slew constraint.
    inputLower=[-Inf;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
    inputUpper=[Inf;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    rate=[Inf;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;finite=isfinite(rate);
    [physicalLower,physicalUpper]=localStateLimits(cfg);
    primaryLowerBound=0;departure=0;
    % Per-hold affine uncertainty. The estimator publishes a fresh
    % enclosure G0 of its estimate at every sample, and every hold of the
    % plan is issued from such an estimate. Hold i therefore starts from G0
    % and propagates it through that hold only: G(t_i+tau) = Phi_i(tau) G0,
    % and the target set restarts from its predicted state at t_i. The
    % first hold uses the present enclosure's tangent image; nonlinear
    % remainders are not certified. Later holds assume that the estimator's
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
    % The target forecast at every node and midpoint (record times), and
    % the lane projection of the ego corners at every node and midpoint,
    % each in one call.
    middles=reshape([stages.middle],6,count);
    forecast=[];if ~isempty(model.target),forecast=localTargetAt(model,times);end
    clearance=model.road.lateralClearance;road=[];
    if ~isempty(clearance),road=localRoadProjection([anchor.states,middles],model);end
    % Equality rows: the anchor's start, then x_{i+1} - A_i x_i - B_i u_i.
    a3=reshape([stages.a],6,6,count);b3=reshape([stages.b],6,2,count);
    rowIndex=6*reshape(1:count,1,1,count)+(1:6).';
    equal=sparse([(1:6).';reshape(rowIndex,[],1);reshape(repmat(rowIndex,1,6,1),[],1);reshape(repmat(rowIndex,1,2,1),[],1)], ...
        [ix(:,1);reshape(ix(:,2:end),[],1);reshape(repmat(reshape(ix(:,1:count),1,6,count),6,1,1),[],1); ...
        reshape(repmat(reshape(iu,1,2,count),6,1,1),[],1)], ...
        [ones(6+6*count,1);-a3(:);-b3(:)],6*(count+1),nv);
    rhs=[model.initialState-anchor.states(:,1);reshape(reshape([stages.next],6,count)-anchor.states(:,2:end),[],1)];
    % Handling cones: two per hold after the first (one per hold for the
    % first), in the order of the former per-cone assembly.
    coneCount=2*count-1;coneState=zeros(6,coneCount);coneBrake=zeros(1,coneCount);
    coneSlip=zeros(6,coneCount);coneForward=zeros(6,coneCount);coneB=zeros(2,coneCount);coneGamma=zeros(1,coneCount);cones=0;
    tire=modifiedFialaTire.parameters(cfg);k=3*tire.longitudinalForceScale(2)/tire.corneringStiffness(2);
    tangent=tan(cfg.model.sideslipMaximum);side=[0,0,0,-tangent,1,0;0,0,0,-tangent,-1,0];
    % Collision rows that cannot bind are not formed (localBinding): the
    % pose moves at most this much over the variables' boxes (inputs at
    % trustMaximumScale), in |dx| + |dy| + reach |dpsi| per unit of each
    % variable's box.
    stateBox=radius*[5;5;.5;5;3;1.5];inputBox=cfg.nonlinear.trustMaximumScale*radius*[.15;.25];
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];reach=norm(shape(1:2))+norm(shape(3:4));
    nodeMovement=[1,1,reach,0,0,0]*stateBox;
    for index=1:count
        x=anchor.states(:,index);input=anchor.inputs(:,index);nextReference=anchor.states(:,index+1);
        stage=stages(index);middle=stage.middle;am=stage.am;bm=stage.bm;
        % Node index closes hold index-1 (or is the present sample).
        collisionGenerator=localRelativeGenerator(x,generator,model,holdCenter);
        middleGenerator=am*initialGenerator;nextGenerator=stage.a*initialGenerator;
        middleCenter=model.initialState;if index>1,middleCenter=x;end
        middleCollisionGenerator=localRelativeGenerator(middle,middleGenerator,model,middleCenter);
        uncertaintyPrediction.egoStateRadius(:,2*index-1:2*index)=[sum(abs(generator),2),sum(abs(middleGenerator),2)];
        uncertaintyPrediction.collisionStateRadius(:,2*index-1:2*index)=[sum(abs(collisionGenerator),2),sum(abs(middleCollisionGenerator),2)];
        if any(finite)
            previous=model.previousInput;rateRows=eye(2);rateColumns=iu(:,index);
            if index>1,previous=anchor.inputs(:,index-1);rateRows=[eye(2),-eye(2)];rateColumns=[iu(:,index);iu(:,index-1)];end
            difference=input-previous;
            blockCount=blockCount+1;blockRows{blockCount}=[rateRows(finite,:);-rateRows(finite,:)];blockColumns{blockCount}=rateColumns;
            blockBounds{blockCount}=[rate(finite)-difference(finite);rate(finite)+difference(finite)];
        end
        nodeColumns=ix(:,index);
        % The current measured node is fixed; every later node stays on the road.
        if index>1 && ~isempty(clearance)
            [r,bound]=localRoadBlock(road,index,x,eye(6),generator,clearance);
            blockCount=blockCount+1;blockRows{blockCount}=r;blockColumns{blockCount}=nodeColumns;blockBounds{blockCount}=bound;
        end
        point=2*index-1;
        if ~localBeyond(x,forecast,point,collisionGenerator,targetTube,cfg)
            departure=index;
        end
        if departure==index && localBinding(x,forecast(:,point),point,collisionGenerator,targetTube,nodeMovement,cfg)
            [g,j,tightening]=localSafety(x,forecast(:,point),(index-1)*h,model,collisionGenerator,holdStart);
            maximumTightening=max(maximumTightening,max(tightening));
            if index==1,primaryLowerBound=max(0,-min(g));end
            r=[-j,zeros(4,3)];
            if robustRelaxation && index>prefix
                blockCount=blockCount+1;blockRows{blockCount}=r;blockColumns{blockCount}=nodeColumns;blockBounds{blockCount}=g+tightening;
            end
            blockCount=blockCount+1;
            if index<=slackCount,blockRows{blockCount}=[r,-ones(4,1)];blockColumns{blockCount}=[nodeColumns;is(index)];
            else,blockRows{blockCount}=r;blockColumns{blockCount}=nodeColumns;end
            blockBounds{blockCount}=g;
        end
        middleMap=[am,bm];middleColumns=[ix(:,index);iu(:,index)];
        if ~isempty(clearance)
            [r,bound]=localRoadBlock(road,count+1+index,middle,middleMap,middleGenerator,clearance);
            blockCount=blockCount+1;blockRows{blockCount}=r;blockColumns{blockCount}=middleColumns;blockBounds{blockCount}=bound;
        end
        point=2*index;inRange=~localBeyond(middle,forecast,point,middleCollisionGenerator,targetTube,cfg);
        if inRange,departure=index;end
        middleMovement=[1,1,reach]*abs(middleMap(1:3,:))*[stateBox;inputBox];
        if inRange && localBinding(middle,forecast(:,point),point,middleCollisionGenerator,targetTube,middleMovement,cfg)
            [g,j,tightening]=localSafety(middle,forecast(:,point),(index-.5)*h,model,middleCollisionGenerator,(index-1)*h);
            maximumTightening=max(maximumTightening,max(tightening));
            r=-j*middleMap(1:3,:);
            if robustRelaxation && index>prefix
                blockCount=blockCount+1;blockRows{blockCount}=r;blockColumns{blockCount}=middleColumns;blockBounds{blockCount}=g+tightening;
            end
            blockCount=blockCount+1;
            if index<=slackCount,blockRows{blockCount}=[r,-ones(4,1)];blockColumns{blockCount}=[middleColumns;is(index)];
            else,blockRows{blockCount}=r;blockColumns{blockCount}=middleColumns;end
            blockBounds{blockCount}=g;
        end
        middleRadius=sum(abs(middleGenerator(4:6,:)),2);nextRadius=sum(abs(nextGenerator(4:6,:)),2);
        blockCount=blockCount+1;blockRows{blockCount}=[middleMap(4:6,:);-middleMap(4:6,:)];blockColumns{blockCount}=middleColumns;
        blockBounds{blockCount}=[physicalUpper-middle(4:6)-middleRadius;middle(4:6)-physicalLower-middleRadius];
        jx=ix(:,index+1);
        lower(jx(4:6))=max(lower(jx(4:6)),physicalLower-nextReference(4:6)+nextRadius);
        upper(jx(4:6))=min(upper(jx(4:6)),physicalUpper-nextReference(4:6)-nextRadius);
        % Handling envelope at both ends of the hold, under its own input
        % (localHandling). The measured node is fixed and is not constrained.
        % The sideslip cone applies above model.brakeHoldSpeed; at rest the
        % sideslip is undefined.
        if nextReference(4)>cfg.model.brakeHoldSpeed
            blockCount=blockCount+1;blockRows{blockCount}=side;blockColumns{blockCount}=jx;
            blockBounds{blockCount}=-side*nextReference-sum(abs(side*nextGenerator),2);
        end
        cones=cones+1;[coneSlip(:,cones),coneForward(:,cones),coneB(:,cones),coneGamma(cones)]=localHandling(nextReference,input,k,cfg);
        coneState(:,cones)=jx;coneBrake(cones)=iu(2,index);
        if index>1
            cones=cones+1;[coneSlip(:,cones),coneForward(:,cones),coneB(:,cones),coneGamma(cones)]=localHandling(x,input,k,cfg);
            coneState(:,cones)=nodeColumns;coneBrake(cones)=iu(2,index);
        end
        generator=nextGenerator;holdCenter=middleCenter;holdStart=(index-1)*h;
    end
    collisionGenerator=localRelativeGenerator(anchor.states(:,end),generator,model,holdCenter);
    uncertaintyPrediction.egoStateRadius(:,end)=sum(abs(generator),2);
    uncertaintyPrediction.collisionStateRadius(:,end)=sum(abs(collisionGenerator),2);
    model.uncertaintyPrediction=uncertaintyPrediction;
    % Terminal node: the collision rows of every node inside the range, the
    % road, and the terminal set's conditions linearized at the anchor
    % endpoint y (terminalSafeSet.rows): collision rows at the continuation's
    % nodes and midpoints before the target's exit and at the parked pose
    % against the target's closest track pose, and road rows along the
    % continuation, all on dx_N. The anchor endpoint meets them when it is in
    % the set, so the zero correction stays feasible.
    y=anchor.states(:,end);endColumns=ix(:,end);
    time=count*h;point=2*count+1;
    if ~localBeyond(y,forecast,point,collisionGenerator,targetTube,cfg)
        departure=count+1;
    end
    if departure==count+1 && localBinding(y,forecast(:,point),point,collisionGenerator,targetTube,nodeMovement,cfg)
        [g,j,tightening]=localSafety(y,forecast(:,point),time,model,collisionGenerator,holdStart);
        maximumTightening=max(maximumTightening,max(tightening));
        r=[-j,zeros(4,3)];
        if robustRelaxation && count+1>prefix
            blockCount=blockCount+1;blockRows{blockCount}=r;blockColumns{blockCount}=endColumns;blockBounds{blockCount}=g+tightening;
        end
        blockCount=blockCount+1;
        if count+1<=slackCount,blockRows{blockCount}=[r,-ones(4,1)];blockColumns{blockCount}=[endColumns;is(count+1)];
        else,blockRows{blockCount}=r;blockColumns{blockCount}=endColumns;end
        blockBounds{blockCount}=g;
    end
    if ~isempty(clearance)
        [r,bound]=localRoadBlock(road,count+1,y,eye(6),generator,clearance);
        blockCount=blockCount+1;blockRows{blockCount}=r;blockColumns{blockCount}=endColumns;blockBounds{blockCount}=bound;
    end
    context=localTerminalContext(anchor,model);
    [terminalRows,terminalBounds,~,terminalInfo]=terminalSafeSet.rows(context,y,count,radius*[5;5;.5;5;3;1.5]);
    if ~isempty(terminalRows)
        blockCount=blockCount+1;blockRows{blockCount}=terminalRows;blockColumns{blockCount}=endColumns;blockBounds{blockCount}=terminalBounds;
    end
    % Rows that every correction in the variables' boxes meets are left out:
    % they cannot bind, so the feasible set and the optimum are unchanged.
    % The input increments' box is taken at trustMaximumScale, so the rows
    % kept serve every trust scale of localExpand.
    reachLower=lower;reachUpper=upper;
    box=cfg.nonlinear.trustMaximumScale*radius*[.15;.25];
    reachLower(iu)=max(-box,physicalInputLower-anchor.inputs);reachUpper(iu)=min(box,physicalInputUpper-anchor.inputs);
    [inequality,bounds]=localAssemble(blockRows,blockColumns,blockBounds,blockCount,nv,reachLower,reachUpper);
    % One block of three-dimensional handling cones ||[w/k; v_x b]|| <= v_x.
    coneRows=[2*(1:cones)-1;2*(1:cones)];
    coneA=sparse([reshape(repmat(coneRows(1,:),6,1),[],1);coneRows(2,:).'], ...
        [reshape(coneState(:,1:cones),[],1);coneBrake(1:cones).'], ...
        [reshape(coneSlip(:,1:cones),[],1);ones(cones,1)],2*cones,nv);
    coneD=sparse(reshape(repmat(1:cones,6,1),[],1),reshape(coneState(:,1:cones),[],1),reshape(coneForward(:,1:cones),[],1),cones,nv);
    handling=localConeBlock(coneA,reshape(coneB(:,1:cones),[],1),coneD,coneGamma(1:cones).',2);
    objective=zeros(nv,1);objective(is)=1;
    problem=struct('a',inequality,'b',bounds, ...
        'equal',equal,'rhs',rhs,'lower',lower,'upper',upper,'cones',handling, ...
        'primaryConeCount',1, ...
        'stateIndices',ix,'inputIndices',iu,'slackIndices',is,'clfIndex',ic, ...
        'safetyObjective',objective,'primaryLowerBound',primaryLowerBound, ...
        'encounterExit',departure, ...
        'maximumCollisionTighteningMeters',maximumTightening, ...
        'terminalRowCount',size(terminalRows,1),'terminalAnchorValue',terminalInfo.value, ...
        'terminalAnchorMargin',terminalInfo.margin,'terminalAnchorExitSeconds',terminalInfo.exitSeconds, ...
        'terminalAnchorReason',terminalInfo.reason);
    model.robustnessRelaxation=robustRelaxation;
    problem.physicalInputLower=physicalInputLower;problem.physicalInputUpper=physicalInputUpper;
    problem.inputIncrementLower=physicalInputLower-anchor.inputs;problem.inputIncrementUpper=physicalInputUpper-anchor.inputs;
    problem=localInputBounds(problem,model);
end

function [matrix,vector]=localAssemble(rows,columns,bounds,count,nv,lower,upper)
    % Stack dense row blocks on their columns into one sparse matrix,
    % leaving out every row r z <= b that the box lower <= z <= upper
    % already implies (max over the box of r z below b).
    rows=rows(1:count);columns=columns(1:count);bounds=bounds(1:count);
    for j=1:count
        r=rows{j};if isempty(r),continue;end
        c=columns{j}(:);low=lower(c).';high=upper(c).';
        reach=max(r.*low,r.*high);reach(r==0)=0;
        keep=~(sum(reach,2)<bounds{j});
        if ~all(keep),rows{j}=r(keep,:);bounds{j}=bounds{j}(keep);end
    end
    m=cellfun('size',rows,1);n=cellfun('size',rows,2);
    offsets=cumsum([0,m(1:end-1)]);total=sum(m.*n);
    I=zeros(total,1);J=I;V=I;at=0;
    for j=1:count
        e=m(j)*n(j);if e==0,continue;end
        I(at+(1:e))=offsets(j)+repmat((1:m(j)).',n(j),1);
        J(at+(1:e))=reshape(repmat(reshape(columns{j},1,[]),m(j),1),[],1);
        V(at+(1:e))=rows{j}(:);at=at+e;
    end
    matrix=sparse(I,J,V,sum(m),nv);vector=vertcat(bounds{1:count});
end

function cone=localConeBlock(A,b,D,gamma,rowsPerCone)
    % A block of second-order cones of equal dimension: cone j is
    % ||A(rows j) z + b(rows j)|| <= D(j,:) z - gamma(j) in the solver's
    % convention (localConicSolve), with 'size' rows of A per cone.
    cone=struct('A',A,'b',b,'D',D,'gamma',gamma,'size',rowsPerCone);
end

function road=localRoadProjection(poses,model)
    % Lane projection of the four ego corners at every pose (columns).
    cfg=model.cfg;corners=cfg.vehicle.rectangleOffset+[cfg.vehicle.length;cfg.vehicle.width]/2.*[1,1,-1,-1;1,-1,1,-1];
    n=size(poses,2);c=cos(poses(3,:));s=sin(poses(3,:));points=zeros(2,4,n);
    for corner=1:4
        points(:,corner,:)=reshape(poses(1:2,:)+[c*corners(1,corner)-s*corners(2,corner); ...
            s*corners(1,corner)+c*corners(2,corner)],2,1,n);
    end
    projection=laneGeometry.project(reshape(points,2,[]),model.lane);
    road=struct('lateral',reshape(projection.lateralPosition,4,n),'heading',reshape(projection.heading,4,n),'corners',corners);
end

function [rows,bounds]=localRoadBlock(road,point,pose,map,generator,clearance)
    % The ego drives on the road: every rectangle corner within
    % lateralClearance=[right;left] of the path, linearized at the anchor
    % corner on the local path normal, on the columns of 'map' (the pose's
    % dependence on the problem's variables). Applied at every predicted node.
    c=cos(pose(3));s=sin(pose(3));derivative=[-s,-c;c,-s];
    rows=zeros(8,size(map,2));bounds=zeros(8,1);
    for corner=1:4
        heading=road.heading(corner,point);normal=[-sin(heading);cos(heading)];
        jacobian=normal.'*[eye(2),derivative*road.corners(:,corner)];
        reserve=sum(abs(jacobian*generator(1:3,:)));
        r=jacobian*map(1:3,:);
        rows(2*corner-1,:)=r;bounds(2*corner-1)=clearance(2)-road.lateral(corner,point)-reserve;
        rows(2*corner,:)=-r;bounds(2*corner)=clearance(1)+road.lateral(corner,point)-reserve;
    end
end

function [values,jacobian,tightening]=localSafety(x,q,time,model,generator,holdStart)
    % Fixed-dual collision rows at a pose against the target's forecast q,
    % on the pose (jacobian, 4x3), tightened by the enclosures.
    cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    dual=predictiveSafetyGeometry.supportLinearization(x(1:3),shape,q(1:3),q(8:11));
    values=dual.value-cfg.collision.safetyMarginMeters;
    jacobian=dual.jacobian;
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

function binding=localBinding(x,q,point,generator,tube,movement,cfg)
    % Can a collision row at this pose bind within the variables' boxes?
    % Every fixed-dual row value is at least the rectangle distance, hence at
    % least the reference distance less both reaches; the tightening is at
    % most the ego enclosure's position radius, the target tube's position
    % radius and both rotation chords (localSafety); a row changes by at
    % most 'movement' over the boxes (unit normal, yaw lever at most the
    % reach). Rows whose bound exceeds that change cannot bind and are not
    % formed: the problem's feasible set is the same.
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    egoReach=norm(shape(1:2))+norm(shape(3:4));targetReach=norm(q(8:9))+norm(q(10:11));
    yawRadius=sum(abs(generator(3,:)));
    tightening=sum(vecnorm(generator(1:2,:)))+tube.positionRadius(point) ...
        +2*egoReach*sin(min(pi,yawRadius)/2)+2*targetReach*sin(tube.yawRadius(point)/2);
    lower=norm(x(1:2)-q(1:2))-egoReach-targetReach-cfg.collision.safetyMarginMeters-tightening;
    binding=~(lower>movement);
end

function beyond=localBeyond(x,forecast,point,generator,tube,cfg)
    % The target's reference point beyond the encounter range at record
    % point 'point' (no target: beyond), with the enclosures as reserve.
    beyond=isempty(forecast);
    if ~beyond
        reserve=sum(vecnorm(generator(1:2,:)))+tube.positionRadius(point);
        beyond=norm(x(1:2)-forecast(1:2,point))-reserve>cfg.collision.encounterRangeMeters;
    end
end

function [slip,forward,b,gamma]=localHandling(y,input,k,cfg)
    % Stable handling envelope at one node under the input of an adjacent
    % hold: the cone ||[w/k; v_x b]|| <= v_x on the node's state (slip and
    % forward rows, 6x1 each) and its braking ratio (unit coefficient).
    %   Rear adhesion: the rear Fiala tire stays below saturation,
    %     |w| <= k v_x sqrt(1-b^2),  w = v_y - lr r,  k = 3 mu_r Fz_r / C_r,
    %   with v_x frozen at the anchor in the product v_x b. A saturated rear
    %   axle has no restoring yaw moment (the vehicle spins) and its force
    %   no longer determines the slip, so the estimator's force-balance
    %   measurement loses its information. It is a handling and
    %   observability row, not part of the safety certificate, and acts on
    %   the estimate. The cone is divided by the anchor speed, so its
    %   entries are the normalized rear slip tan(alpha_r)/tan(alpha_sat), the
    %   braking ratio and their unit bound: a well-scaled cone for the
    %   interior-point solver. In the declared model's kinematic regime
    %   (anchor speed at or below model.kinematicSpeedRange(1)) the rear tire
    %   has no lateral force to saturate: the cone is only |b| <= 1.
    slip=zeros(6,1);forward=zeros(6,1);
    if y(4)>cfg.model.kinematicSpeedRange(1)
        scale=max(y(4),cfg.model.scheduleSpeedFloor);
        slip(5:6)=[1;-cfg.vehicle.lr]/(k*scale);forward(4)=1/scale;
        b=-[(y(5)-cfg.vehicle.lr*y(6))/(k*scale);input(2)];gamma=-y(4)/scale;
    else
        b=-[0;input(2)];gamma=-1;
    end
end

function problem=localInputBounds(problem,model)
    % Bounds of the input increments: the trust box of the present scale,
    % within the actuator limits about the anchor inputs.
    iu=problem.inputIndices;box=model.inputTrustScale*model.cfg.nonlinear.trustRadius*[.15;.25];
    problem.lower(iu)=max(-box,problem.inputIncrementLower);
    problem.upper(iu)=min(box,problem.inputIncrementUpper);
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
    % On the previous solved plan the tie's loss is at least one unit of
    % the CLF error scales. Only the first successor enters the CLF row, so
    % near the path the row and the constraints leave the later inputs free,
    % and the tie is what keeps them. A tie that vanished there left them
    % anywhere in the trust box: the plan, and the next sample's anchor,
    % wandered between samples, and at 15 m/s the cruise oscillated. A fresh
    % rollout is only a search seed, and the tie is not floored on it
    % (report/SOLVER_TOLERANCE_20261011.tex).
    lossLower=max(double(problem.inheritedAnchor),norm(e0./scales)^2);
    % The CLF is a soft performance row on the estimate: V(xhat+) <= rho
    % V(xhat). Estimation error enters it as an input (ISS); it carries no
    % observer enclosure. A worst case over the enclosure would ask for a
    % decrease below the enclosure's own size, which no input achieves near
    % the path, and would turn this stage into a greedy one-step minimizer.
    currentBudget=model.nominalReference.contraction*value;
    map=nominalMap;offset=nominalOffset;
    modelConstant=0;constant=currentBudget/scale;
    axis=sparse(1,nv);axis(ic)=1;
    cone=localConeBlock([2*map;axis],[-2*offset;1-constant],axis,-1-constant,6);
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

function [tube,support]=localTargetTube(model,time,holdStart,normal)
    % Target enclosure of the hold that starts at holdStart: the observer
    % set restarted from the target's predicted state there, for the time
    % elapsed in the hold. This is not propagation of the current family
    % over the complete horizon; future measurement updates are assumed.
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

function solution=localTerminalMetrics(solution,model)
    % Simple prediction quantities; no post-solve constraint evaluation.
    cfg=model.cfg;y=solution.states(:,end);count=size(solution.inputs,2);
    solution.terminalValue=nonlinearBicycleModel.nominalValue(y,model.lane,model.nominalReference);
    solution.terminalMarginMeters=NaN;solution.terminalRoadMarginMeters=NaN;
    solution.terminalStopSeconds=ceil(hypot(y(4),y(5))/(cfg.terminal.brakingDecelerationMetersPerSecondSquared ...
        *cfg.controller.sampleTime))*cfg.controller.sampleTime;
    solution.terminalExitSeconds=NaN;solution.terminalEnd="notValidated";
    solution.linearizationGap=model.trust.modelInnovation;
    solution.terminalDistanceMeters=Inf;
    if ~isempty(model.target)
        q=localTargetAt(model,count*cfg.controller.sampleTime);
        solution.terminalDistanceMeters=norm(y(1:2)-q(1:2));
    end
end

function [low,high]=localStateLimits(cfg)
    low=[cfg.model.speedMinimum;-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
    high=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
end

function q=localTargetAt(model,time)
    q=predictiveSafetyGeometry.predictTarget(model.targetEpoch, ...
        (model.sampleIndex-localEpochIndex(model))*model.cfg.controller.sampleTime+time);
end

function index=localEpochIndex(model)
    % Sample index of the target epoch (0 when a model carries none).
    index=0;if isfield(model,'targetEpochIndex'),index=model.targetEpochIndex;end
end
