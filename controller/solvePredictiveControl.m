function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl One model per shifted or freshly initialized trajectory.
% A shifted slack budget can replace the primary optimization, never the CLF.
% Damping reuses that model; only a failed shift can request a fresh seed.
    if nargin<3,timer=tic;end
    if ~isfield(model,'uncertainty')
        model.uncertainty=struct('specified',false,'egoGenerator',zeros(6), ...
            'collisionGenerator',zeros(6),'target',[],'relativeFrame',false);
    end
    cfg=model.cfg;solution=[];allAttempts=struct([]);allStages=struct([]);history=struct([]);
    restarted=false;selected=0;initialFailure="";selectedRound=0;
    lineSearchHistory=struct([]);
    model.inputTrustScale=.125;
    model.inheritedSlacks=[];
    model.linearizationBuilds=0;
    if isstruct(previousState) && isfield(previousState,'linearizationTrustScale')
        model.inputTrustScale=previousState.linearizationTrustScale;
    end
    if ~model.uncertainty.specified && isstruct(previousState) && isfield(previousState,'stageSlacks') ...
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
        if isempty(candidate),model=trialModel;break;end
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
    % Use the same numerical feasibility allowance as the accepted solver
    % point. Convex interpolation preserves that allowance, not exact zeros.
    if ~isfinite(residual) || residual>cfg.solver.feasibilityTolerance
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
    slack=max(0,next-problem.clfCurrentBudget);
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
    if ~valid
        % Only the first hold is issued. A remote suffix outside the tire
        % model domain does not invalidate an evaluable first-hold comparison.
        [rollout,valid]=localRollout(candidate.inputs(:,1),model);
    end
    poseError=Inf;stateError=Inf;clfError=Inf;fullPoseError=Inf;actualSlack=Inf;
    maximumDeviation=Inf;deviationRatio=0;
    % RTI executes one hold before receiving another posterior. Agreement
    % over an unexecuted open-loop suffix is not an admission requirement.
    % The affine suffix still carries all planning constraints; its nonlinear
    % feasibility and recursive transfer are explicitly not certified.
    poseCount=min(size(candidate.states,2),2);
    if valid
        nodes=size(rollout.states,2);
        [poseError,stateError,fullPoseError]=localTrajectoryError(candidate.states(:,1:nodes),rollout.states,cfg,poseCount);
        if nodes<size(candidate.states,2),fullPoseError=Inf;end
        actual=nonlinearBicycleModel.nominalValue(rollout.states(:,2),model.lane,model.nominalReference);
        clfError=abs(actual-candidate.clfNextValue)/max(1,candidate.clfInitialValue);
        actualSlack=max(0,actual-candidate.clfInitialValue+candidate.clfRequiredDecrease);
        projection=laneGeometry.project(rollout.states(1:2,1:poseCount),model.lane);
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
    % Retain the full suffix discrepancy as an offline research measurement.
    poseError=max(bodyError(1:poseCount));
    stateError=max(abs(affine(4:6,1:poseCount)-nonlinear(4:6,1:poseCount))./[5;3;1.5],[],'all');
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
    if model.uncertainty.specified
        wall=tic;[problem,model]=localFormulate(anchor,model);model.linearization=anchor;
        search=localSearch(problem,model,source,toc(wall));
        search.initializationSeconds=initializationSeconds;
        point=zeros(problem.clfIndex-1,1);
        for stage=1:numel(problem.slackIndices)
            rows=problem.a(:,problem.slackIndices(stage))<0;
            point(problem.slackIndices(stage))=max([0;-problem.b(rows)]);
        end
        search.budgetAnchorResidual=localConvexResidual([point;0],problem);
        search.budgetSource="observerConditionedAnchor";
        search.slackCap=sum(point(problem.slackIndices));
        search.slackCap=search.slackCap+model.cfg.solver.lexicographicTieTolerance*max(1,search.slackCap);
        search.stages(1)=struct('objective',"conditionedSafetyBudget",'exitFlag',NaN, ...
            'seconds',0,'value',search.slackCap,'numericalSolve',false,'solverInfo',struct());
        sharedProblem=problem;
        if search.budgetAnchorResidual<=model.cfg.solver.constraintTolerance
            [solution,search,model,sharedProblem]=localSecondary(point,problem,anchor,model,search,timer);
        else
            solution=[];search.terminationReason="anchorOutsideCurrentAffineSet";
        end
        budgetAttempts=localAttempt(search);budgetStages=search.stages;
        if ~isempty(solution)
            search.selectedAttempt=1;search.attempts=budgetAttempts;search.linearizationCount=1;
            search.potentialFieldRestarted=false;search.initializationFailure=failure;search.elapsedSeconds=toc(timer);return;
        end
        % A changed posterior can invalidate this particular anchor. Restore
        % the same assembled problem before requesting a fresh APF rollout.
        model.inheritedSlacks=[];initializationSeconds=0;
    end
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
    needsRestoration=isempty(point) || (~model.robustnessRelaxation ...
        && search.primaryOptimum>problem.primaryLowerBound+model.cfg.solver.feasibilityTolerance);
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
    radius=repmat(radius,1,count);
    lower=problem.physicalInputLower-anchor.inputs;
    upper=problem.physicalInputUpper-anchor.inputs;
    problem.lower(iu(:))=reshape(max(-radius,lower),[],1);
    problem.upper(iu(:))=reshape(min(radius,upper),[],1);
    if isfield(problem,'clf')
        scale=reshape(1./(radius*sqrt(2*count)),[],1);
        increment=sparse(1:numel(iu),iu(:),scale,numel(iu),numel(problem.lower));
        problem.quadratic=2*problem.clfTieBound/problem.clfScale*(increment.'*increment);
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
    % Match the cap's scale to the numerical primary objective. An absolute
    % micrometer cap on a large restoration sum can exclude the computed
    % primary point within the solver's relative optimality tolerance.
    search.slackCap=search.primaryOptimum+cfg.solver.lexicographicTieTolerance*max(1,search.primaryOptimum);
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
    nextValue=norm(problem.nominalClfMap*point+problem.nominalClfOffset)^2*problem.clfScale;
    worstNext=(norm(problem.clfMap*point+problem.clfOffset)^2+problem.clfModelConstant)*problem.clfScale;
    solution=struct('inputs',inputs,'states',states,'stageSlacks',slacks.', ...
        'safety',sum(slacks),'hard',NaN,'clfSlack',rho, ...
        'clfInitialValue',problem.initialClfValue,'clfNextValue',nextValue, ...
        'clfFunction',problem.clf.function,'clfRequiredDecrease',problem.clf.requiredDecrease, ...
        'clfTieBound',problem.clfTieBound,'clfWorstNextValue',worstNext,'clfCurrentBudget',problem.clfCurrentBudget, ...
        'maximumCollisionTighteningMeters',problem.maximumCollisionTighteningMeters, ...
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
                    elseif isfield(previous,'feedbackGains') && index<size(previous.feedbackGains,3)
                        error=states(:,index)-previous.stateTrajectory(:,index+1);
                        error(3)=atan2(sin(error(3)),cos(error(3)));
                        last=model.previousInput;if index>1,last=inputs(:,index-1);end
                        inputs(:,index)=localClip(inputs(:,index)+previous.feedbackGains(:,:,index+1)*error, ...
                            last,cfg);
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
        time=(index-1)*h;
        if index>settle
            % Settle intrinsic velocities into the existing free-pose
            % straight core; the endpoint position and heading remain free.
            u=localGuidanceInput(x,previous,0,cfg.referenceSpeed,cfg,model.terminal.reference);
        elseif isempty(model.targetEpoch)
            u=nonlinearBicycleModel.nominalFeedback(x,previous,model.lane,reference,cfg,nominal);
        else
            [heading,speed,side]=predictiveSafetyGeometry.potentialGuidance(x,model.lane,model.target,time,cfg,side);
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
    robustRelaxation=any(model.uncertainty.egoGenerator(:));
    if ~isempty(model.uncertainty.target)
        robustRelaxation=robustRelaxation || any(structfun(@(value)any(value(:)>0),model.uncertainty.target));
    end
    slackCount=prefix;
    if robustRelaxation,slackCount=count+1;end
    ix=reshape(1:6*(count+1),6,[]);iu=reshape(ix(end)+(1:2*count),2,[]);
    is=iu(end)+(1:slackCount);ic=is(end)+1;nv=ic;
    equal=sparse(6*(count+1),nv);rhs=zeros(6*(count+1),1);equal(1:6,ix(:,1))=eye(6);
    rhs(1:6)=model.initialState-anchor.states(:,1);
    rows=cell(1,9*count+8);bounds=cell(size(rows));rowCount=0;
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
    primaryLowerBound=0;departure=0;
    generator=model.uncertainty.egoGenerator;
    posterior=generator;
    stages=localFeedbackLinearization(anchor,model,any(posterior(:)));
    model.feedbackGains=cat(3,stages.gain);
    inputGenerator=zeros(2,size(generator,2));
    physicalInputLower=zeros(2,count);physicalInputUpper=zeros(2,count);
    maximumTightening=0;
    times=(0:2*count)*cfg.controller.sampleTime/2;
    targetTube=predictiveSafetyGeometry.targetErrorTube(model.target,model.uncertainty.target,times);
    uncertaintyPrediction=struct('time',times,'target',targetTube, ...
        'egoStateRadius',zeros(6,2*count+1),'collisionStateRadius',zeros(6,2*count+1));
    for index=1:count
        x=anchor.states(:,index);input=anchor.inputs(:,index);nextReference=anchor.states(:,index+1);
        stage=stages(index);middle=stage.middle;am=stage.am;bm=stage.bm;
        next=stage.next;a=stage.a;b=stage.b;
        previousInputGenerator=inputGenerator;
        collisionGenerator=localRelativeGenerator(x,generator,model);
        if index==1 || ~any(posterior(:))
            % The current estimate and nominal center coincide. The issued
            % input is known, so d0=epsilon0 cancels the feedback terms.
            middleGenerator=am*generator;nextGenerator=a*generator;
            inputGenerator=zeros(2,size(generator,2));
        else
            gain=stage.gain;
            % Maximize alpha in [0,1] along the Riccati brake-gain direction
            % subject to its uncertainty image fitting the anchor's remaining
            % actuator range. Shrink feedback authority, never the error set.
            allowance=max(0,min(inputUpper(2)-input(2),input(2)-inputLower(2)));
            rawRadius=sum(abs([gain(2,:)*generator,-gain(2,:)*posterior]));
            if rawRadius>0,gain(2,:)=gain(2,:)*min(1,allowance/rawRadius);end
            model.feedbackGains(:,:,index)=gain;
            inputGenerator=[gain*generator,-gain*posterior];
            middleGenerator=[(am+bm*gain)*generator,-bm*gain*posterior];
            nextGenerator=[(a+b*gain)*generator,-b*gain*posterior];
        end
        middleCollisionGenerator=localRelativeGenerator(middle,middleGenerator,model);
        if index==1,firstStateGenerator=nextGenerator;end
        uncertaintyPrediction.egoStateRadius(:,2*index-1:2*index)=[sum(abs(generator),2),sum(abs(middleGenerator),2)];
        uncertaintyPrediction.collisionStateRadius(:,2*index-1:2*index)=[sum(abs(collisionGenerator),2),sum(abs(middleCollisionGenerator),2)];
        eq=6*index+(1:6);equal(eq,ix(:,index+1))=eye(6);equal(eq,ix(:,index))=-a;equal(eq,iu(:,index))=-b;
        rhs(eq)=next-nextReference;
        inputRadius=sum(abs(inputGenerator),2);
        physicalInputLower(:,index)=inputLower+inputRadius;
        physicalInputUpper(:,index)=inputUpper-inputRadius;
        lower(iu(:,index))=max(lower(iu(:,index)),inputLower-input+inputRadius);
        upper(iu(:,index))=min(upper(iu(:,index)),inputUpper-input-inputRadius);
        r=sparse(2,nv);r(:,iu(:,index))=eye(2);previous=model.previousInput;
        if index>1,r(:,iu(:,index-1))=-eye(2);previous=anchor.inputs(:,index-1);end
        previousInputGenerator(:,end+1:size(inputGenerator,2))=0;
        rateRadius=sum(abs(inputGenerator-previousInputGenerator),2);
        finite=isfinite(rate);difference=input-previous;
        rowCount=rowCount+1;rows{rowCount}=[r(finite,:);-r(finite,:)];bounds{rowCount}=[rate(finite)-difference(finite)-rateRadius(finite);rate(finite)+difference(finite)-rateRadius(finite)];
        map=sparse(6,nv);map(:,ix(:,index))=eye(6);
        if isfinite(deviationLimit)
            limit=pathLimit;if index==1,limit=deviationLimit;end
            [r,bound,cone]=localPathRows(x(1:2),map(1:2,:),model.lane,limit,pathBox,generator(1:2,:));
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=bound;
            pathCones=[pathCones,cone]; %#ok<AGROW>
        end
        if ~localBeyondRange(x,(index-1)*cfg.controller.sampleTime,model,collisionGenerator)
            departure=index;
            [g,j,tightening]=localSafetyRows(x,(index-1)*cfg.controller.sampleTime,model,collisionGenerator);
            maximumTightening=max(maximumTightening,max(tightening));
            if index==1,primaryLowerBound=max(0,-min(g));end
            r=-j*map;
            if robustRelaxation
                rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g+tightening;
            end
            if index<=slackCount,r(:,is(index))=-1;end
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
        end
        map(:,ix(:,index))=am;map(:,iu(:,index))=bm;
        if isfinite(deviationLimit)
            [r,bound,cone]=localPathRows(middle(1:2),map(1:2,:),model.lane,pathLimit,pathBox,middleGenerator(1:2,:));
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=bound;
            pathCones=[pathCones,cone]; %#ok<AGROW>
        end
        if ~localBeyondRange(middle,(index-.5)*cfg.controller.sampleTime,model,middleCollisionGenerator)
            departure=index;
            [g,j,tightening]=localSafetyRows(middle,(index-.5)*cfg.controller.sampleTime,model,middleCollisionGenerator);
            maximumTightening=max(maximumTightening,max(tightening));
            r=-j*map;
            if robustRelaxation
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
        generator=nextGenerator;
    end
    collisionGenerator=localRelativeGenerator(anchor.states(:,end),generator,model);
    uncertaintyPrediction.egoStateRadius(:,end)=sum(abs(generator),2);
    uncertaintyPrediction.collisionStateRadius(:,end)=sum(abs(collisionGenerator),2);
    model.uncertaintyPrediction=uncertaintyPrediction;
    endIndex=model.sampleIndex+count;y=[anchor.states(:,end);anchor.inputs(:,end)];
    [seed,poseJacobian]=terminalContinuation.fit(model.terminal,y,endIndex);model.terminal=seed;
    [seed,terminalDomainMargin]=terminalContinuation.feedbackCore( ...
        seed,posterior,[generator;inputGenerator],cfg,y(4:8)-[seed.base(4:6);seed.reference.input]);
    model.terminal=seed;
    deviation=y(4:8)-[seed.base(4:6);seed.reference.input];
    map=sparse(8,nv);map(1:6,ix(:,end))=eye(6);map(7:8,iu(:,end))=eye(2);
    endpointReserve=sum(vecnorm(seed.quotientFactor*[generator(4:6,:);inputGenerator]));
    model.uncertaintyPrediction.terminalCoreRadius=seed.radius;
    model.uncertaintyPrediction.terminalReserve=endpointReserve;
    % The endpoint is a nominal core plus an invariant feedback-error tube.
    % The uncertainty is retained in that tube and terminal geometry; it is
    % not required to fit inside the nominal zero-disturbance core.
    endpoint=localCone(seed.quotientFactor*map(4:8,:),-seed.quotientFactor*deviation,zeros(nv,1),-seed.radius);
    rowCount=rowCount+1;rows{rowCount}=sparse(numel(terminalDomainMargin),nv);bounds{rowCount}=terminalDomainMargin;
    terminalA=zeros(0,nv);terminalB=zeros(0,1);
    if ~localBeyondRange(y(1:6),count*cfg.controller.sampleTime,model,collisionGenerator)
        departure=Inf;
        [~,terminalB,gradient]=localTerminalClearance(seed,count,model);
        terminalA=-gradient*poseJacobian*map;
        rowCount=rowCount+1;rows{rowCount}=terminalA;bounds{rowCount}=terminalB;
        [g,j,tightening]=localSafetyRows(y(1:6),count*cfg.controller.sampleTime,model,collisionGenerator);
        maximumTightening=max(maximumTightening,max(tightening));
        r=sparse(size(j,1),nv);r(:,ix(:,end))=-j;
        if robustRelaxation
            rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g+tightening;
            r(:,is(end))=-1;
        end
        rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;
    end
    if isfinite(deviationLimit)
        [r,bound,cone]=localPathRows(y(1:2),map(1:2,:),model.lane,pathLimit,pathBox,generator(1:2,:));
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
        'terminalA',terminalA,'terminalB',terminalB,'encounterExit',departure, ...
        'maximumCollisionTighteningMeters',maximumTightening,'firstStateGenerator',firstStateGenerator);
    model.robustnessRelaxation=robustRelaxation;
    problem.physicalInputLower=physicalInputLower;problem.physicalInputUpper=physicalInputUpper;
end

function stages=localFeedbackLinearization(anchor,model,withFeedback)
    count=size(anchor.inputs,2);stages=repmat(struct('middle',[],'am',[],'bm',[], ...
        'next',[],'a',[],'b',[],'gain',zeros(2,6)),1,count);
    for index=1:count
        [stages(index).middle,stages(index).am,stages(index).bm, ...
            stages(index).next,stages(index).a,stages(index).b]= ...
            nonlinearBicycleModel.hold(anchor.states(:,index),anchor.inputs(:,index),model.cfg);
    end
    if ~withFeedback,return;end
    angle=anchor.states(3,end);rotation=[cos(angle),sin(angle);-sin(angle),cos(angle)];
    transform=blkdiag(rotation,eye(4));p=transform.'*model.terminal.matrix(1:6,1:6)*transform;
    cfg=model.cfg;scales=[1,cfg.clf.lateralPositionErrorScale,cfg.clf.headingErrorScale, ...
        cfg.clf.speedErrorScale,cfg.clf.lateralVelocityErrorScale,cfg.clf.yawRateErrorScale];
    % Normalize ancillary effort by the same steering/braking scales used
    % for the RTI correction. An unscaled unit penalty amplifies posterior
    % noise enough to make the brake tube exceed its physical range.
    effort=diag(1./[.15;.25].^2);
    for index=count:-1:1
        a=stages(index).a;b=stages(index).b;
        angle=anchor.states(3,index);rotation=[cos(angle),sin(angle);-sin(angle),cos(angle)];
        transform=blkdiag(rotation,eye(4));q=transform.'*diag(1./scales.^2)*transform;
        gain=-(effort+b.'*p*b)\(b.'*p*a);stages(index).gain=gain;
        p=q+a.'*p*a+a.'*p*b*gain;p=(p+p.')/2;
    end
end

function relative=localRelativeGenerator(x,generator,model)
    relative=generator;
    if ~model.uncertainty.relativeFrame,return;end
    % Keep the shared initial rigid-pose columns through the feedback map.
    % Subtract the same rigid motion of the ego/target configuration only
    % when evaluating pairwise geometry, not before propagating feedback.
    offset=x(1:2)-model.initialState(1:2);
    rigid=[eye(2),[-offset(2);offset(1)];0,0,1];
    relative(1:3,1:3)=relative(1:3,1:3)-rigid*model.uncertainty.egoGenerator(1:3,1:3);
end

function [rows,bounds,cone]=localPathRows(position,map,lane,limit,box,generator)
    region=laneGeometry.deviationRegion(position,lane,limit);
    rows=region.a*map;bounds=region.b-sum(abs(region.a*generator),2);
    active=abs(rows)*box>bounds;rows=rows(active,:);bounds=bounds(active);
    cone=struct('A',{},'b',{},'d',{},'gamma',{});
    radius=region.radius-sum(vecnorm(generator));
    if isfinite(radius) && norm(abs(region.center)+abs(map)*box)>radius
        cone=localCone(map,region.center,zeros(size(map,2),1),-radius);
    end
end

function problem=localAddClf(problem,anchor,model)
    nv=numel(problem.safetyObjective);iu=problem.inputIndices;ic=problem.clfIndex;count=size(iu,2);
    [nominalMap,nominalOffset,~,value,scale,~,clf]=localNominalClf(anchor,model,problem.stateIndices(:,2),nv);
    [e0,j0]=nonlinearBicycleModel.errorLinearization(model.initialState,model.lane,model.nominalReference);
    [~,j1]=nonlinearBicycleModel.errorLinearization(anchor.states(:,2),model.lane,model.nominalReference);
    factor=model.nominalReference.factor;
    initialRadius=sum(vecnorm(factor*j0*model.uncertainty.egoGenerator));
    nextRadius=sum(vecnorm(factor*j1*problem.firstStateGenerator));
    scales=[model.cfg.clf.lateralPositionErrorScale;model.cfg.clf.headingErrorScale; ...
        model.cfg.clf.speedErrorScale;model.cfg.clf.lateralVelocityErrorScale;model.cfg.clf.yawRateErrorScale];
    lossRadius=sum(vecnorm((j0*model.uncertainty.egoGenerator)./scales));
    lossCenter=norm(e0./scales);
    lossLower=max(0,lossCenter-lossRadius)^2;
    currentBudget=max(0,sqrt(value)-initialRadius)^2 ...
        -model.cfg.nominalClf.decreaseFraction*(lossCenter+lossRadius)^2;
    % Young's inequality keeps one SOC and the same CLF. Mean values remain
    % separate from these bounds for the existing model-agreement diagnostic.
    multiplier=1;bias=0;
    if nextRadius>0
        weight=min(1,nextRadius/max(norm(nominalOffset)*sqrt(scale),eps));
        multiplier=1+weight;bias=(1+1/weight)*nextRadius^2;
    end
    map=sqrt(multiplier)*nominalMap;offset=sqrt(multiplier)*nominalOffset;
    modelConstant=bias/scale;constant=(currentBudget-bias)/scale;
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

function [values,jacobian,tightening]=localSafetyRows(x,time,model,generator)
    cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    q=localTargetAt(model,time);
    dual=predictiveSafetyGeometry.supportLinearization(x(1:3),shape,q(1:3),q(8:11));
    values=dual.value-cfg.collision.safetyMarginMeters;
    jacobian=[dual.jacobian,zeros(4,3)];
    % Directional position support retains correlation across the affine
    % ego dynamics. Rotation uses an exact chord bound for each rectangle.
    normal=jacobian(:,1:2);normalNorm=vecnorm(normal,2,2);
    tube=predictiveSafetyGeometry.targetErrorTube(model.target,model.uncertainty.target,time);
    egoReach=norm(shape(1:2))+norm(shape(3:4));
    targetReach=norm(q(8:9))+norm(q(10:11));
    yawRadius=sum(abs(generator(3,:)));
    targetSupport=predictiveSafetyGeometry.targetPositionSupport(model.target,model.uncertainty.target,time,normal);
    reserve=2*egoReach*sin(min(pi,yawRadius)/2) ...
        +2*targetReach*sin(tube.yawRadius/2);
    tightening=sum(abs(normal*generator(1:2,:)),2)+targetSupport+normalNorm*reserve;
    values=values-tightening;
end

function beyond=localBeyondRange(x,time,model,generator)
    beyond=isempty(model.target);
    if ~beyond
        cfg=model.cfg;
        q=localTargetAt(model,time);
        tube=predictiveSafetyGeometry.targetErrorTube(model.target,model.uncertainty.target,time);
        reserve=sum(vecnorm(generator(1:2,:)))+tube.positionRadius;
        beyond=norm(x(1:2)-q(1:2))-reserve>cfg.collision.encounterRangeMeters;
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
