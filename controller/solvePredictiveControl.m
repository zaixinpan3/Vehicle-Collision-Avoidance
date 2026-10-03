function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl One model per shifted or freshly initialized trajectory.
% A shifted slack budget can replace the primary optimization, never the CLF.
% Damping reuses that model; only a failed shift can request a fresh seed.
    if nargin<3,timer=tic;end
    cfg=model.cfg;solution=[];allAttempts=struct([]);allStages=struct([]);history=struct([]);
    restarted=false;selected=0;initialFailure="";selectedRound=0;
    lineSearchHistory=struct([]);
    model.inputTrustScale=.125;
    model.inheritedSlacks=[];
    model.linearizationBuilds=0;
    if isstruct(previousState) && isfield(previousState,'linearizationTrustScale')
        model.inputTrustScale=previousState.linearizationTrustScale;
    end
    if isstruct(previousState) && isfield(previousState,'stageSlacks') ...
            && isnumeric(previousState.stageSlacks) && isvector(previousState.stageSlacks) ...
            && numel(previousState.stageSlacks)==cfg.controller.horizonSteps ...
            && all(isfinite(previousState.stageSlacks)) && all(previousState.stageSlacks>=0)
        model.inheritedSlacks=[reshape(previousState.stageSlacks(2:end),1,[]),0];
    end
    for iteration=1:cfg.nonlinear.maximumLinearizations
        if model.linearizationBuilds>=cfg.nonlinear.maximumLinearizations,break;end
        model.allowPotentialFieldRestart=~restarted;
        [candidate,search,trialModel]=localRound(model,previousState,timer);
        offset=numel(allAttempts);allAttempts=[allAttempts,search.attempts]; %#ok<AGROW>
        allStages=[allStages,search.stages]; %#ok<AGROW>
        restarted=restarted || search.potentialFieldRestarted;
        if strlength(search.initializationFailure)>0,initialFailure=search.initializationFailure;end
        if isempty(candidate),break;end
        segment=candidate.searchSegment;candidate=rmfield(candidate,'searchSegment');
        wall=tic;[agreement,~,~]=localPredictionAgreement(candidate,trialModel);
        agreement.seconds=toc(wall);agreement.stepFraction=1;
        scale=max(1,candidate.clfInitialValue);
        agreement.predictedClfReduction=search.clfInitialSlack-candidate.clfSlack;
        agreement.actualClfReduction=search.clfInitialSlack-agreement.actualClfSlack;
        agreement.clfReductionResolved=max(abs([agreement.predictedClfReduction, ...
            agreement.actualClfReduction]))>cfg.solver.clfTieTolerance*scale;
        agreement.trustBoundaryRefinement=false;
        history=[history,agreement]; %#ok<AGROW>
        if agreement.ratio>1 && toc(timer)<cfg.solver.timeLimitSeconds
            [damped,dampedAgreement,lineSearch]=localDampedStep(candidate,segment,trialModel,agreement,timer);
            lineSearch.round=iteration;lineSearchHistory=[lineSearchHistory,lineSearch]; %#ok<AGROW>
            if ~isempty(damped),candidate=damped;agreement=dampedAgreement;end
        end
        if agreement.ratio<=1
            candidate.predictionAgreement=agreement;
            solution=candidate;acceptedModel=trialModel;acceptedSearch=search;
            selected=offset+search.selectedAttempt;selectedRound=iteration;
            acceptedModel.nextTrustScale=min(1,trialModel.inputTrustScale ...
                *min(1.5,max(1,.8/sqrt(max(agreement.ratio,eps))))*agreement.stepFraction);
            acceptedModel.nextTrustScale=max(1/1024,acceptedModel.nextTrustScale);
            acceptedSearch.clfLowerBound=candidate.clfSlack<=cfg.solver.feasibilityTolerance*scale;
            acceptedModel.terminal=terminalContinuation.fit(acceptedModel.terminal, ...
                [candidate.states(:,end);candidate.inputs(:,end)],model.sampleIndex+size(candidate.inputs,2));
            break;
        end
        % A failed shifted correction may start one new potential-field seed.
        % Never linearize the optimized rollout again in this sampling hold.
        if restarted || search.initialization~="shiftedInputRollout" ...
                || trialModel.linearizationBuilds>=cfg.nonlinear.maximumLinearizations
            break;
        end
        model=trialModel;model.inheritedSlacks=[];model.inputTrustScale=.125;
        previousState=[];restarted=true;initialFailure="shiftedModelDisagreement";
        if toc(timer)>=cfg.solver.timeLimitSeconds,break;end
    end
    if ~isempty(solution)
        model=acceptedModel;search=acceptedSearch;search.terminationReason="twoStagesModelAgreement";
    elseif ~isempty(history)
        search.terminationReason="linearizationAccuracyNotReached";
    end
    search.attempts=allAttempts;search.stages=allStages;search.selectedAttempt=selected;
    search.linearizationCount=sum([allAttempts.modelBuilt]);search.solverCalls=sum([allStages.numericalSolve]);
    search.potentialFieldRestarted=restarted;search.initializationFailure=initialFailure;
    search.refinementCount=numel(history);search.selectedRefinement=selectedRound;search.modelAgreementHistory=history;
    search.modelAgreementSatisfied=~isempty(solution);search.elapsedSeconds=toc(timer);
    search.returned=~isempty(solution);
    search.lineSearchHistory=lineSearchHistory;search.lineSearchSeconds=0;search.lineSearchTrials=0;
    search.acceptedStepFraction=1;
    if ~isempty(lineSearchHistory)
        search.lineSearchSeconds=sum([lineSearchHistory.seconds]);
        search.lineSearchTrials=sum(arrayfun(@(entry)numel(entry.trials),lineSearchHistory));
    end
    if ~isempty(solution),search.acceptedStepFraction=solution.predictionAgreement.stepFraction;end
end

function [accepted,agreement,info]=localDampedStep(full,segment,model,fullAgreement,timer)
    wall=tic;accepted=[];agreement=fullAgreement;cfg=model.cfg;
    problem=segment.problem;base=[segment.primary;0];
    base(problem.clfIndex)=localClfSlack(base,problem)/problem.clfScale;
    residual=localConvexResidual(base,problem);
    baseSlack=problem.clfScale*base(problem.clfIndex);
    info=struct('reason',"infeasibleBase",'baseResidual',residual,'baseClfSlack',baseSlack, ...
        'fullClfSlack',full.clfSlack,'accepted',false,'seconds',0,'trials',struct([]));
    % Inherited slacks alone do not establish feasibility at zero correction.
    % Only a point in this exact assembled problem supplies a convex segment.
    if ~isfinite(residual) || residual>cfg.solver.constraintTolerance
        info.seconds=toc(wall);return;
    end
    fraction=min(.5,.8/sqrt(fullAgreement.ratio));
    if ~isfinite(fraction) || fraction<=0,fraction=.1;end
    direction=segment.secondary-base;
    zeroClf=full.clfSlack<=cfg.solver.feasibilityTolerance*problem.clfScale;
    info.reason="modelDisagreement";
    for attempt=1:3
        if toc(timer)>=cfg.solver.timeLimitSeconds,info.reason="timeLimit";break;end
        point=base+fraction*direction;
        % Tighten rho on the same convex quadratic; linear interpolation of
        % epigraph heights would report avoidable slack and fake CLF error.
        point(problem.clfIndex)=localClfSlack(point,problem)/problem.clfScale;
        trial=localDecode(point,problem,model.linearization);
        if zeroClf && trial.clfSlack>cfg.solver.feasibilityTolerance*problem.clfScale
            % The zero-slack sublevel set on this segment is an interval
            % containing alpha=1. Smaller alpha cannot restore a lost zero.
            info.reason="zeroClfSlackWouldBeLost";break;
        end
        check=tic;[trialAgreement,~,~]=localPredictionAgreement(trial,model);
        trialAgreement.seconds=toc(check);trialAgreement.stepFraction=fraction;
        trialAgreement.predictedClfReduction=baseSlack-trial.clfSlack;
        trialAgreement.actualClfReduction=baseSlack-trialAgreement.actualClfSlack;
        trialAgreement.clfReductionResolved=max(abs([trialAgreement.predictedClfReduction, ...
            trialAgreement.actualClfReduction]))>cfg.solver.clfTieTolerance*problem.clfScale;
        trialAgreement.trustBoundaryRefinement=false;
        entry=struct('fraction',fraction,'clfSlack',trial.clfSlack,'safety',trial.safety, ...
            'agreement',trialAgreement);
        info.trials=[info.trials,entry];
        if trialAgreement.ratio<=1
            accepted=trial;agreement=trialAgreement;info.accepted=true;info.reason="accepted";break;
        end
        fraction=fraction/2;
    end
    info.seconds=toc(wall);
end

function slack=localClfSlack(point,problem)
    next=(norm(problem.clfMap*point+problem.clfOffset)^2+problem.clfModelConstant)*problem.clfScale;
    slack=max(0,next-problem.initialClfValue+problem.clf.requiredDecrease);
end

function residual=localConvexResidual(point,problem)
    residual=max([0;problem.a*point-problem.b;abs(problem.equal*point-problem.rhs); ...
        problem.lower-point;point-problem.upper]);
    for index=1:numel(problem.cones)
        cone=problem.cones(index);
        residual=max(residual,norm(cone.A*point-cone.b)-cone.d.'*point+cone.gamma);
    end
end

function [agreement,rollout,valid]=localPredictionAgreement(candidate,model)
    cfg=model.cfg;[rollout,valid]=localRollout(candidate.inputs,model);
    poseError=Inf;stateError=Inf;clfError=Inf;fullPoseError=Inf;actualSlack=Inf;
    maximumDeviation=Inf;deviationRatio=0;
    poseCount=min(size(candidate.states,2),candidate.encounterExit+1);
    if valid
        [poseError,stateError,fullPoseError]=localTrajectoryError(candidate.states,rollout.states,cfg,poseCount);
        actual=nonlinearBicycleModel.nominalValue(rollout.states(:,2),model.lane,model.nominalReference);
        clfError=abs(actual-candidate.clfNextValue)/max(1,candidate.clfInitialValue);
        actualSlack=max(0,actual-candidate.clfInitialValue+candidate.clfRequiredDecrease);
        projection=laneGeometry.project(rollout.states(1:2,:),model.lane);
        maximumDeviation=max(abs(projection.lateralPosition));
        if isfinite(cfg.controller.maximumLateralDeviationMeters)
            deviationRatio=max(0,1+(maximumDeviation-cfg.controller.maximumLateralDeviationMeters) ...
                /cfg.nonlinear.predictionToleranceMeters);
        end
    end
    ratio=max([poseError/cfg.nonlinear.predictionToleranceMeters, ...
        stateError/cfg.nonlinear.statePredictionTolerance,clfError/cfg.nonlinear.clfPredictionTolerance,deviationRatio]);
    agreement=struct('poseErrorMeters',poseError,'fullPoseErrorMeters',fullPoseError, ...
        'poseConstraintNodeCount',poseCount,'scaledStateError',stateError, ...
        'scaledClfError',clfError,'ratio',ratio,'inputTrustScale',model.inputTrustScale, ...
        'actualClfSlack',actualSlack,'maximumLateralDeviationMeters',maximumDeviation, ...
        'lateralDeviationRatio',deviationRatio,'firstInputTrustActivity', ...
        max(abs(candidate.inputs(:,1)-model.linearization.inputs(:,1)) ...
        ./(model.inputTrustScale*cfg.nonlinear.trustRadius*[.15;.25])), ...
        'seconds',0,'stepFraction',1);
end

function [poseError,stateError,fullPoseError]=localTrajectoryError(affine,nonlinear,cfg,poseCount)
    reach=norm([cfg.vehicle.length;cfg.vehicle.width]/2)+norm(cfg.vehicle.rectangleOffset);
    displacement=vecnorm(affine(1:2,:)-nonlinear(1:2,:));
    rotation=abs(affine(3,:)-nonlinear(3,:));
    bodyError=displacement+reach*rotation;fullPoseError=max(bodyError);
    % Free endpoint pose and post-encounter positions impose no position
    % constraint. Preserve accuracy where pose enters the actual problem.
    poseError=max(bodyError(1:poseCount));
    stateError=max(abs(affine(4:6,:)-nonlinear(4:6,:))./[5;3;1.5],[],'all');
end

function [rollout,valid]=localRollout(inputs,model)
    states=zeros(6,size(inputs,2)+1);states(:,1)=model.initialState;valid=true;
    try
        for index=1:size(inputs,2)
            states(:,index+1)=nonlinearBicycleModel.sample(states(:,index),inputs(:,index),model.cfg);
        end
    catch exception
        domainErrors=["collisionAvoidanceController:nonlinearDomain", ...
            "collisionAvoidanceController:invalidTireOperatingPoint", ...
            "collisionAvoidanceController:singularTireLinearization"];
        if ~any(string(exception.identifier)==domainErrors),rethrow(exception);end
        valid=false;
    end
    valid=valid && all(isfinite(states),'all');
    rollout=struct('inputs',inputs,'states',states);
end

function [solution,search,model] = localRound(model,previousState,timer)
%localRound Restore one affine PCBF problem, then solve its CLF objective.
% A finite candidate is returned for model agreement and optional damping.
    if nargin<3,timer=tic;end
    wall=tic;[anchor,source,failure]=localInitialization(model,previousState);
    initializationSeconds=toc(wall);
    budgetAttempts=struct([]);budgetStages=struct([]);
    sharedProblem=[];
    if ~isempty(model.inheritedSlacks) && source=="shiftedInputRollout"
        wall=tic;[problem,model]=localFormulate(anchor,model);model.linearization=anchor;
        search=localSearch(problem,model,source,toc(wall));
        search.initializationSeconds=initializationSeconds;
        search.budgetSource="shiftedTrajectory";search.slackCap=sum(model.inheritedSlacks);
        search.firstSlackCap=model.inheritedSlacks(1);
        problem.upper(problem.slackIndices(1))=search.firstSlackCap;
        point=zeros(problem.clfIndex-1,1);point(problem.slackIndices)=model.inheritedSlacks;
        fullPoint=[point;0];
        search.budgetAnchorResidual=max([0;problem.a*fullPoint-problem.b; ...
            abs(problem.equal*fullPoint-problem.rhs);problem.lower-fullPoint;fullPoint-problem.upper]);
        for cone=problem.cones(1:problem.primaryConeCount)
            search.budgetAnchorResidual=max(search.budgetAnchorResidual, ...
                norm(cone.A*fullPoint-cone.b)-cone.d.'*fullPoint+cone.gamma);
        end
        % This cap is not a recomputed optimal PCBF value. The current problem
        % can be infeasible after state or linearization discrepancies.
        search.stages(1)=struct('objective',"inheritedSafetyBudget",'exitFlag',NaN, ...
            'seconds',0,'value',search.slackCap,'numericalSolve',false,'solverInfo',struct());
        [solution,search,model,sharedProblem]=localSecondary(point,problem,anchor,model,search,timer);
        budgetAttempts=localAttempt(search);budgetStages=search.stages;
        if ~isempty(solution)
            search.selectedAttempt=1;search.attempts=budgetAttempts;search.linearizationCount=1;
            search.potentialFieldRestarted=false;search.initializationFailure=failure;search.elapsedSeconds=toc(timer);return;
        end
        % Restore this optimization after a failed inherited-budget attempt.
        % This branch never issues an input from the previous plan.
        model.inheritedSlacks=[];initializationSeconds=0;
    end
    [point,problem,search,model]=localPrimary(anchor,model,source,timer,sharedProblem);
    search.initializationSeconds=initializationSeconds;
    attempts=localAttempt(search);stages=search.stages;selected=1;restarted=false;
    if source~="shiftedInputRollout"
        [point,problem,search,model,extra,expanded]=localExpandPrimary(point,problem,search,model,anchor,timer);
        if ~isempty(extra),attempts(end+1)=extra;stages=[stages,extra.stages];end
        if expanded,selected=numel(attempts);end
    end
    needsRestoration=isempty(point) || search.primaryOptimum>problem.primaryLowerBound+model.cfg.solver.feasibilityTolerance;
    if source=="shiftedInputRollout" && needsRestoration && model.allowPotentialFieldRestart ...
            && model.linearizationBuilds<model.cfg.nonlinear.maximumLinearizations && toc(timer)<model.cfg.solver.timeLimitSeconds
        reason=search.terminationReason;
        if ~isempty(point),reason="positiveRestorablePcbfSlack";end
        wall=tic;freshAnchor=localPotentialFieldSeed(model,size(anchor.inputs,2));initializationSeconds=toc(wall);
        freshSource="movingTargetPotentialField";if isempty(model.target),freshSource="laneFeedbackRollout";end
        [freshPoint,freshProblem,freshSearch,freshModel]=localPrimary(freshAnchor,model,freshSource,timer);
        freshSearch.initializationSeconds=initializationSeconds;
        attempts(end+1)=localAttempt(freshSearch);stages=[stages,freshSearch.stages];restarted=true;failure=reason;
        freshSelected=numel(attempts);
        [freshPoint,freshProblem,freshSearch,freshModel,extra,expanded]=localExpandPrimary( ...
            freshPoint,freshProblem,freshSearch,freshModel,freshAnchor,timer);
        if ~isempty(extra),attempts(end+1)=extra;stages=[stages,extra.stages];end
        if expanded,freshSelected=numel(attempts);end
        % Do not discard a better primary problem merely because the new
        % initialization returns a numerical point. Ties retain the shift.
        if isempty(point) || (~isempty(freshPoint) && freshSearch.primaryOptimum ...
                <search.primaryOptimum-model.cfg.solver.lexicographicTieTolerance)
            point=freshPoint;problem=freshProblem;search=freshSearch;model=freshModel;anchor=freshAnchor;selected=freshSelected;
        end
        model.linearizationBuilds=max(model.linearizationBuilds,freshModel.linearizationBuilds);
    end
    [solution,search,model]=localSecondary(point,problem,anchor,model,search,timer);
    stages=[stages,search.stages(2:end)];attempts(selected)=localAttempt(search);
    % A failed CLF solve can still use the one available fresh initialization.
    if isempty(solution) && search.initialization=="shiftedInputRollout" && ~restarted && model.allowPotentialFieldRestart ...
            && model.linearizationBuilds<model.cfg.nonlinear.maximumLinearizations ...
            && search.terminationReason=="clfNoNumericalResult" && toc(timer)<model.cfg.solver.timeLimitSeconds
        failure=search.terminationReason;wall=tic;anchor=localPotentialFieldSeed(model,size(anchor.inputs,2));initializationSeconds=toc(wall);
        source="movingTargetPotentialField";if isempty(model.target),source="laneFeedbackRollout";end
        [point,problem,search,model]=localPrimary(anchor,model,source,timer);
        search.initializationSeconds=initializationSeconds;
        attempts(end+1)=localAttempt(search);stages=[stages,search.stages];
        selected=numel(attempts);
        [point,problem,search,model,extra,expanded]=localExpandPrimary(point,problem,search,model,anchor,timer);
        if ~isempty(extra),attempts(end+1)=extra;stages=[stages,extra.stages];end
        if expanded,selected=numel(attempts);end
        [solution,search,model]=localSecondary(point,problem,anchor,model,search,timer);
        stages=[stages,search.stages(2:end)];attempts(selected)=localAttempt(search);restarted=true;
    end
    search.selectedAttempt=selected;search.stages=stages;search.attempts=attempts;
    search.linearizationCount=numel(attempts);search.solverCalls=sum([stages.numericalSolve]);search.potentialFieldRestarted=restarted;
    search.initializationFailure=failure;search.elapsedSeconds=toc(timer);
    search.selectedAttempt=search.selectedAttempt+numel(budgetAttempts);
    search.attempts=[budgetAttempts,search.attempts];search.stages=[budgetStages,search.stages];
    search.linearizationCount=numel(search.attempts);search.solverCalls=sum([search.stages.numericalSolve]);
end

function [point,problem,search,model,extra,expanded]=localExpandPrimary(point,problem,search,model,anchor,timer)
    % An infeasible affine problem cannot be repaired by shrinking its input
    % correction box. Reuse the potential-field seed and allow one bounded enlargement.
    % State trust, actuator bounds, safety rows and terminal constraints stay.
    extra=struct([]);expanded=false;
    infeasible=isempty(point) && ~isempty(search.stages) && any(search.stages(end).exitFlag==[-2,-7]);
    if infeasible && model.inputTrustScale<2 && toc(timer)<model.cfg.solver.timeLimitSeconds
        widerModel=model;widerModel.inputTrustScale=2;
        % Only the correction box changes. Reuse the dynamics, geometry and
        % terminal Jacobians; this is not another trajectory linearization.
        widerProblem=localInputBox(problem,anchor,widerModel);
        [widerPoint,widerProblem,widerSearch,widerModel]=localPrimary( ...
            anchor,widerModel,search.initialization,timer,widerProblem);
        widerSearch.initializationSeconds=0;extra=localAttempt(widerSearch);
        if isempty(point) || (~isempty(widerPoint) && widerSearch.primaryOptimum ...
                <search.primaryOptimum-model.cfg.solver.lexicographicTieTolerance)
            point=widerPoint;problem=widerProblem;search=widerSearch;model=widerModel;expanded=true;
        end
        model.linearizationBuilds=max(model.linearizationBuilds,widerModel.linearizationBuilds);
    end
end

function problem=localInputBox(problem,anchor,model)
    cfg=model.cfg;iu=problem.inputIndices;count=size(iu,2);
    radius=model.inputTrustScale*cfg.nonlinear.trustRadius*[.15;.25];
    lower=[-Inf;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)]-anchor.inputs;
    upper=[Inf;min(1-1e-8,cfg.actuation.brakingRatioMaximum)]-anchor.inputs;
    problem.lower(iu(:))=reshape(max(-radius,lower),[],1);
    problem.upper(iu(:))=reshape(min(radius,upper),[],1);
    if isfield(problem,'clf')
        scale=repmat(1./(radius*sqrt(2*count)),count,1);
        increment=sparse(1:numel(iu),iu(:),scale,numel(iu),numel(problem.lower));
        problem.quadratic=2*cfg.solver.clfTieTolerance*(increment.'*increment);
    end
end

function attempt=localAttempt(search)
    attempt=struct('initialization',search.initialization,'terminationReason',search.terminationReason, ...
        'solverCalls',search.solverCalls,'stages',search.stages,'primaryOptimum',search.primaryOptimum, ...
        'primaryLowerBound',search.primaryLowerBound,'inputTrustScale',search.inputTrustScale, ...
        'budgetSource',search.budgetSource,'slackCap',search.slackCap, ...
        'firstSlackCap',search.firstSlackCap,'budgetAnchorResidual',search.budgetAnchorResidual, ...
        'initializationSeconds',search.initializationSeconds,'formulationSeconds',search.formulationSeconds, ...
        'clfConstructionSeconds',search.clfConstructionSeconds);
    attempt.modelBuilt=search.modelBuilt;
end

function search=localSearch(problem,model,initialization,formulationSeconds)
    search=struct('solverCalls',0,'source',"twoStageRealTimeIteration", ...
        'modelBuilt',formulationSeconds>0, ...
        'initialization',initialization,'returned',false,'converged',false,'terminationReason',"timeLimit", ...
        'formulationSeconds',formulationSeconds,'clfConstructionSeconds',0, ...
        'inputTrustScale',model.inputTrustScale,'primaryOptimum',NaN,'primaryLowerBound',problem.primaryLowerBound,'slackCap',NaN,'clfInitialSlack',NaN, ...
        'budgetSource',"primaryOptimum",'firstSlackCap',Inf,'budgetAnchorResidual',NaN, ...
        'clfStageAttempted',false,'clfStageCompleted',false,'clfLowerBound',false, ...
        'stages',struct('objective',{},'exitFlag',{},'seconds',{},'value',{},'numericalSolve',{},'solverInfo',{}));
end

function [point,problem,search,model]=localPrimary(anchor,model,initialization,timer,sharedProblem)
    if ~isfield(model,'inputTrustScale'),model.inputTrustScale=1;end
    cfg=model.cfg;point=[];
    if nargin<5 || isempty(sharedProblem)
        wall=tic;[problem,model]=localFormulate(anchor,model);formulationSeconds=toc(wall);
    else
        problem=sharedProblem;formulationSeconds=0;
        problem.upper(problem.slackIndices(1))=Inf;
    end
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
    search.slackCap=search.primaryOptimum+cfg.solver.lexicographicTieTolerance;
    search.terminationReason="primaryReturned";
end

function [solution,search,model,sharedProblem]=localSecondary(point,problem,anchor,model,search,timer)
    sharedProblem=problem;
    cfg=model.cfg;solution=[];if isempty(point),return;end
    if ~isfield(problem,'clf')
        wall=tic;problem=localAddClf(problem,anchor,model);search.clfConstructionSeconds=toc(wall);
        search.formulationSeconds=search.formulationSeconds+search.clfConstructionSeconds;
    end
    sharedProblem=problem;
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
    solution.searchSegment=struct('primary',point,'secondary',second,'problem',problem);
    model.terminal=terminalContinuation.fit(model.terminal,[solution.states(:,end);solution.inputs(:,end)],model.sampleIndex+size(solution.inputs,2));
    search.stages(2).value=solution.clfSlack;search.clfStageCompleted=true;search.returned=true;
    search.converged=all([search.stages([search.stages.numericalSolve]).exitFlag]>0);
    search.clfLowerBound=solution.clfSlack<=cfg.solver.feasibilityTolerance*problem.clfScale;
    search.terminationReason="twoStagesReturned";
end

function solution=localDecode(point,problem,anchor)
    inputs=anchor.inputs+reshape(point(problem.inputIndices),size(anchor.inputs));
    states=anchor.states+reshape(point(problem.stateIndices),size(anchor.states));
    slacks=max(0,point(problem.slackIndices));rho=problem.clfScale*max(0,point(problem.clfIndex));
    nextValue=(norm(problem.clfMap*point+problem.clfOffset)^2+problem.clfModelConstant)*problem.clfScale;
    solution=struct('inputs',inputs,'states',states,'stageSlacks',slacks.', ...
        'safety',sum(slacks),'hard',NaN,'clfSlack',rho, ...
        'clfInitialValue',problem.initialClfValue,'clfNextValue',nextValue, ...
        'clfFunction',problem.clf.function,'clfRequiredDecrease',problem.clf.requiredDecrease, ...
        'clfTieBound',problem.clfTieBound, ...
        'minimumCollisionMargin',NaN,'terminalSeparationMargin',NaN, ...
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
    cfg=model.cfg;
    count=min(cfg.controller.maximumHorizonSteps,cfg.controller.horizonSteps ...
        +ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime));
    source="movingTargetPotentialField";failure="";
    if isempty(model.target),source="laneFeedbackRollout";end
    domainErrors=["collisionAvoidanceController:nonlinearDomain", ...
        "collisionAvoidanceController:invalidTireOperatingPoint", ...
        "collisionAvoidanceController:singularTireLinearization"];
    if isstruct(previous)
        inputs=previous.inputTrajectory(:,2:end);
        usable=~isempty(inputs) && all(isfinite(inputs(:))) && all(abs(inputs(2,:))<1);
        if usable
            inputs=inputs(:,1:min(count,size(inputs,2)));
            try
                states=zeros(6,count+1);states(:,1)=model.initialState;
                for index=1:count
                    if index>size(inputs,2)
                        [~,~,deviation]=terminalContinuation.membership( ...
                            [states(:,index);inputs(:,end)],model.sampleIndex+index-1,model.terminal);
                        inputs(:,index)=localClip(model.terminal.reference.input ...
                            +model.terminal.gain*deviation,inputs(:,end),cfg);
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
    anchor=localPotentialFieldSeed(model,count);
end

function anchor=localPotentialFieldSeed(model,count)
    % A single potential-guided bicycle rollout, used only without a usable
    % shift or after its optimization fails. No seed supplies an issued input.
    cfg=model.cfg;reference=model.nominalReference;x=model.initialState;previous=model.previousInput;
    h=cfg.controller.sampleTime;
    inputs=zeros(2,count);states=zeros(6,count+1);states(:,1)=x;side=0;
    nominal=nonlinearBicycleModel.nominalGuidanceParameters(cfg,reference.curvature);
    settle=max(cfg.controller.horizonSteps,count-ceil(cfg.initialization.settlingSeconds/h));
    for index=1:count
        time=(model.sampleIndex+index-1)*h;
        if index>settle
            % Settle intrinsic velocities into the existing free-pose
            % straight core; the endpoint position and heading remain free.
            u=localGuidanceInput(x,previous,0,cfg.referenceSpeed,cfg,model.terminal.reference);
        elseif isempty(model.targetEpoch)
            u=nonlinearBicycleModel.nominalFeedback(x,previous,model.lane,reference,cfg,nominal);
        else
            [heading,speed,side]=predictiveSafetyGeometry.potentialGuidance(x,model.lane,model.targetEpoch,time,cfg,side);
            course=x(3)+atan2(x(5),x(4));
            yawRate=reference.curvature*hypot(x(4),x(5)) ...
                -cfg.nominalClf.courseGain*atan2(sin(course-heading),cos(course-heading));
            u=localGuidanceInput(x,previous,yawRate,speed,cfg,reference);
        end
        inputs(:,index)=u;x=nonlinearBicycleModel.sample(x,u,cfg);states(:,index+1)=x;previous=u;
    end
    anchor=struct('inputs',inputs,'states',states);
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
    ix=reshape(1:6*(count+1),6,[]);iu=reshape(ix(end)+(1:2*count),2,[]);
    is=iu(end)+(1:prefix);ic=is(end)+1;nv=ic;
    equal=sparse(6*(count+1),nv);rhs=zeros(6*(count+1),1);equal(1:6,ix(:,1))=eye(6);
    rhs(1:6)=model.initialState-anchor.states(:,1);
    rows=cell(1,7*count+4);bounds=cell(size(rows));rowCount=0;
    pathCones=struct('A',{},'b',{},'d',{},'gamma',{});
    deviationLimit=cfg.controller.maximumLateralDeviationMeters;
    % Reserve the existing position-accuracy tolerance inside the hard bound.
    pathLimit=deviationLimit-min(cfg.nonlinear.predictionToleranceMeters,deviationLimit/100);
    lower=-Inf(nv,1);upper=Inf(nv,1);radius=cfg.nonlinear.trustRadius;
    lower(ix(:))=-repmat(radius*[5;5;.5;5;3;1.5],count+1,1);upper(ix(:))=-lower(ix(:));
    lower(iu(:))=-repmat(model.inputTrustScale*radius*[.15;.25],count,1);upper(iu(:))=-lower(iu(:));lower([is,ic])=0;
    pathBox=zeros(nv,1);pathBox(ix(:))=upper(ix(:));
    % Cover the existing maximum fresh-input box expansion as well. Removing
    % a redundant corridor row remains valid if that box is enlarged later.
    pathBox(iu(:))=repmat(max(2,model.inputTrustScale)*radius*[.15;.25],count,1);
    % Numerical RTI corrections are local to the new anchor at every sample.
    % Steering still has no actuator magnitude or slew constraint.
    inputLower=[-Inf;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
    inputUpper=[Inf;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    rate=[Inf;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
    [physicalLower,physicalUpper]=localStateLimits(cfg);
    primaryLowerBound=0;departure=Inf;if isempty(model.target),departure=0;end
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
        if isfinite(deviationLimit)
            limit=pathLimit;if index==1,limit=deviationLimit;end
            [r,bound,cone]=localPathRows(x(1:2),map(1:2,:),model.lane,limit,pathBox);
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=bound;
            pathCones=[pathCones,cone]; %#ok<AGROW>
        end
        if index<=departure
            [g,j]=localSafetyRows(x,(index-1)*cfg.controller.sampleTime,model);
            if index==1,primaryLowerBound=max(0,-min(g));end
            r=-j*map;if index<=prefix,r(:,is(index))=-1;end
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
        end
        map(:,ix(:,index))=am;map(:,iu(:,index))=bm;
        if isfinite(deviationLimit)
            [r,bound,cone]=localPathRows(middle(1:2),map(1:2,:),model.lane,pathLimit,pathBox);
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=bound;
            pathCones=[pathCones,cone]; %#ok<AGROW>
        end
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
    end
    endIndex=model.sampleIndex+count;y=[anchor.states(:,end);anchor.inputs(:,end)];
    [seed,poseJacobian]=terminalContinuation.fit(model.terminal,y,endIndex);model.terminal=seed;
    deviation=y(4:8)-[seed.base(4:6);seed.reference.input];
    map=sparse(8,nv);map(1:6,ix(:,end))=eye(6);map(7:8,iu(:,end))=eye(2);
    endpoint=localCone(seed.quotientFactor*map(4:8,:),-seed.quotientFactor*deviation,zeros(nv,1),-seed.radius);
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
    if isfinite(deviationLimit)
        [r,bound,cone]=localPathRows(y(1:2),map(1:2,:),model.lane,pathLimit,pathBox);
        rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=bound;
        pathCones=[pathCones,cone];
    end
    primaryCones=[endpoint,pathCones];
    objective=zeros(nv,1);objective(is)=1;
    problem=struct('a',vertcat(rows{1:rowCount}),'b',vertcat(bounds{1:rowCount}), ...
        'equal',equal,'rhs',rhs,'lower',lower,'upper',upper,'cones',primaryCones, ...
        'primaryConeCount',numel(primaryCones), ...
        'stateIndices',ix,'inputIndices',iu,'slackIndices',is,'clfIndex',ic, ...
        'safetyObjective',objective,'primaryLowerBound',primaryLowerBound, ...
        'terminalA',terminalA,'terminalB',terminalB,'encounterExit',departure);
end

function [rows,bounds,cone]=localPathRows(position,map,lane,limit,box)
    region=laneGeometry.deviationRegion(position,lane,limit);
    rows=region.a*map;bounds=region.b;
    active=abs(rows)*box>bounds;rows=rows(active,:);bounds=bounds(active);
    cone=struct('A',{},'b',{},'d',{},'gamma',{});
    if isfinite(region.radius) && norm(abs(region.center)+abs(map)*box)>region.radius
        cone=localCone(map,region.center,zeros(size(map,2),1),-region.radius);
    end
end

function problem=localAddClf(problem,anchor,model)
    nv=numel(problem.safetyObjective);iu=problem.inputIndices;ic=problem.clfIndex;count=size(iu,2);
    [map,offset,constant,value,scale,modelConstant,clf]=localNominalClf(anchor,model,problem.stateIndices(:,2),nv);
    axis=sparse(1,nv);axis(ic)=1;
    cone=localCone([2*map;axis],[-2*offset;1-constant],axis.',-1-constant);
    inputScale=repmat(1./(model.inputTrustScale*model.cfg.nonlinear.trustRadius*[.15;.25]*sqrt(2*count)),count,1);
    increment=sparse(1:numel(iu),iu(:),inputScale,numel(iu),nv);
    problem.quadratic=2*model.cfg.solver.clfTieTolerance*(increment.'*increment);
    problem.cones=[problem.cones,cone];
    problem.clfScale=scale;problem.clfMap=map;problem.clfOffset=offset;
    problem.clfModelConstant=modelConstant;problem.clf=clf;problem.initialClfValue=value;
    problem.clfTieBound=model.cfg.solver.clfTieTolerance*scale;
    problem.initialClfSlack=max(0,(norm(offset)^2-constant)*scale);
end

function [map,offset,constant,value,scale,modelConstant,clf]=localNominalClf(anchor,model,next,nv)
    % One analytic transverse quadratic on the single affine state model.
    cfg=model.cfg;reference=model.nominalReference;
    e0=nonlinearBicycleModel.error(model.initialState,model.lane,reference);
    [e1,jacobian]=nonlinearBicycleModel.errorLinearization(anchor.states(:,2),model.lane,reference);
    value=nonlinearBicycleModel.nominalValue(model.initialState,model.lane,reference);scale=max(1,value);
    scales=[cfg.clf.lateralPositionErrorScale;cfg.clf.headingErrorScale;cfg.clf.speedErrorScale; ...
        cfg.clf.lateralVelocityErrorScale;cfg.clf.yawRateErrorScale];
    required=cfg.nominalClf.decreaseFraction*sum((e0./scales).^2);
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

function [values,jacobian]=localSafetyRows(x,time,model)
    cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    q=localTargetAt(model,time);
    dual=predictiveSafetyGeometry.dualLinearization(x(1:3),shape,q(1:3),q(8:11));
    values=dual.value-cfg.collision.safetyMarginMeters;
    jacobian=[dual.jacobian,zeros(4,3)];
end

function beyond=localBeyondRange(x,time,model)
    beyond=isempty(model.target);
    if ~beyond
        cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
        q=localTargetAt(model,time);
        beyond=predictiveSafetyGeometry.rectangle(x(1:3),shape,q(1:3),q(8:11))>cfg.collision.encounterRangeMeters;
    end
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
    q=predictiveSafetyGeometry.predictTarget(model.targetEpoch,model.sampleIndex*model.cfg.controller.sampleTime+time);
end
