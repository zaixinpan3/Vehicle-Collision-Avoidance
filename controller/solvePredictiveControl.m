function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl Refine one PCBF/CLF controller within the current hold.
% Each issued iterate completes both objectives and meets model accuracy.
% Resolved CLF progress at the numerical input boundary triggers another round.
    if nargin<3,timer=tic;end
    cfg=model.cfg;solution=[];allAttempts=struct([]);allStages=struct([]);history=struct([]);
    restarted=false;selected=0;initialFailure="";selectedRound=0;
    model.inputTrustScale=1;
    if isstruct(previousState) && isfield(previousState,'linearizationTrustScale')
        model.inputTrustScale=previousState.linearizationTrustScale;
    end
    for iteration=1:cfg.nonlinear.maximumLinearizations
        model.allowFlowRestart=~restarted;
        [candidate,search,trialModel]=localRound(model,previousState,timer);
        offset=numel(allAttempts);allAttempts=[allAttempts,search.attempts]; %#ok<AGROW>
        allStages=[allStages,search.stages]; %#ok<AGROW>
        restarted=restarted || search.flowRestarted;
        if strlength(search.initializationFailure)>0,initialFailure=search.initializationFailure;end
        if isempty(candidate),break;end
        wall=tic;[agreement,rollout,valid]=localPredictionAgreement(candidate,trialModel);
        agreement.seconds=toc(wall);agreement.stepFraction=1;
        scale=max(1,candidate.clfInitialValue);
        agreement.predictedClfReduction=search.clfInitialSlack-candidate.clfSlack;
        agreement.actualClfReduction=search.clfInitialSlack-agreement.actualClfSlack;
        agreement.clfReductionResolved=max(abs([agreement.predictedClfReduction, ...
            agreement.actualClfReduction]))>cfg.solver.clfTieTolerance*scale;
        agreement.trustBoundaryRefinement=agreement.ratio<=1 && agreement.clfReductionResolved ...
            && candidate.clfSlack>cfg.solver.feasibilityTolerance*max(1,candidate.clfInitialValue) ...
            && agreement.firstInputTrustActivity>=1-1e-3;
        history=[history,agreement]; %#ok<AGROW>
        if agreement.ratio<=1
            candidate.predictionAgreement=agreement;
            % Retain the best accurate two-stage iterate from this frame.
            % This is optimization bookkeeping, not a previous-plan policy.
            if isempty(solution) || localBetterIterate(candidate,solution,cfg)
                solution=candidate;acceptedModel=trialModel;acceptedSearch=search;
                selected=offset+search.selectedAttempt;selectedRound=iteration;
                acceptedModel.nextTrustScale=min(1,trialModel.inputTrustScale ...
                    *min(1.5,max(1,.8/sqrt(max(agreement.ratio,eps)))));
            end
            if ~agreement.trustBoundaryRefinement,break;end
        end
        % The full nonlinear rollout closes the dynamics defect at the new
        % anchor. Damping an inaccurate but evaluable iterate can keep its
        % endpoint outside the tiny terminal core and force repeated expansion.
        model=trialModel;
        if ~valid
            anchor=trialModel.linearization;fraction=.5;
            for backtrack=1:9
                inputs=anchor.inputs+fraction*(candidate.inputs-anchor.inputs);
                [rollout,valid]=localRollout(inputs,trialModel);
                if valid,break;end
                fraction=fraction/2;
            end
            history(end).stepFraction=fraction;
        end
        if valid
            model.iterationAnchor=rollout;
            model.terminal=terminalContinuation.fit(model.terminal, ...
                [rollout.states(:,end);rollout.inputs(:,end)],model.sampleIndex+size(rollout.inputs,2));
        else
            model.iterationAnchor=trialModel.linearization;
        end
        if agreement.ratio>1
            fraction=min(.8,.8/sqrt(agreement.ratio));
            if ~isfinite(fraction),fraction=.1;end
            model.inputTrustScale=max(1/1024,model.inputTrustScale*max(fraction,.1));
        end
        if toc(timer)>=cfg.solver.timeLimitSeconds,break;end
    end
    if ~isempty(solution)
        model=acceptedModel;search=acceptedSearch;search.terminationReason="twoStagesModelAgreement";
    elseif ~isempty(history)
        search.terminationReason="linearizationAccuracyNotReached";
    end
    search.attempts=allAttempts;search.stages=allStages;search.selectedAttempt=selected;
    search.linearizationCount=numel(allAttempts);search.solverCalls=sum([allStages.numericalSolve]);
    search.flowRestarted=restarted;search.initializationFailure=initialFailure;
    search.refinementCount=numel(history);search.selectedRefinement=selectedRound;search.modelAgreementHistory=history;
    search.modelAgreementSatisfied=~isempty(solution);search.elapsedSeconds=toc(timer);
    search.returned=~isempty(solution);
end

function better=localBetterIterate(candidate,retained,cfg)
    tie=cfg.solver.lexicographicTieTolerance;
    better=candidate.safety<retained.safety-tie ...
        || (candidate.safety<=retained.safety+tie ...
        && candidate.predictionAgreement.actualClfSlack<retained.predictionAgreement.actualClfSlack);
end

function [agreement,rollout,valid]=localPredictionAgreement(candidate,model)
    cfg=model.cfg;[rollout,valid]=localRollout(candidate.inputs,model);
    poseError=Inf;stateError=Inf;clfError=Inf;fullPoseError=Inf;actualSlack=Inf;
    poseCount=min(size(candidate.states,2),candidate.encounterExit+1);
    if valid
        [poseError,stateError,fullPoseError]=localTrajectoryError(candidate.states,rollout.states,cfg,poseCount);
        terminal=nonlinearBicycleModel.nominalTail(cfg,model.nominalReference.curvature);
        actual=nonlinearBicycleModel.nominalValue(rollout.states(:,2),candidate.inputs(:,1), ...
            model.lane,model.nominalReference,terminal,cfg);
        clfError=abs(actual-candidate.clfNextValue)/max(1,candidate.clfInitialValue);
        actualSlack=max(0,actual-candidate.clfInitialValue+candidate.clfRequiredDecrease);
    end
    ratio=max([poseError/cfg.nonlinear.predictionToleranceMeters, ...
        stateError/cfg.nonlinear.statePredictionTolerance,clfError/cfg.nonlinear.clfPredictionTolerance]);
    agreement=struct('poseErrorMeters',poseError,'fullPoseErrorMeters',fullPoseError, ...
        'poseConstraintNodeCount',poseCount,'scaledStateError',stateError, ...
        'scaledClfError',clfError,'ratio',ratio,'inputTrustScale',model.inputTrustScale, ...
        'actualClfSlack',actualSlack,'firstInputTrustActivity', ...
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
% A finite candidate is returned to the outer model-agreement iteration.
    if nargin<3,timer=tic;end
    wall=tic;[anchor,source,failure]=localInitialization(model,previousState);
    initializationSeconds=toc(wall);
    [point,problem,search,model]=localPrimary(anchor,model,source,timer);
    search.initializationSeconds=initializationSeconds;
    attempts=localAttempt(search);stages=search.stages;selected=1;restarted=false;
    if source~="shiftedInputRollout"
        [point,problem,search,model,extra,expanded]=localExpandPrimary(point,problem,search,model,anchor,timer);
        if ~isempty(extra),attempts(end+1)=extra;stages=[stages,extra.stages];end
        if expanded,selected=numel(attempts);end
    end
    needsRestoration=isempty(point) || search.primaryOptimum>problem.primaryLowerBound+model.cfg.solver.feasibilityTolerance;
    if any(source==["shiftedInputRollout","sameFrameRollout"]) && needsRestoration && model.allowFlowRestart && toc(timer)<model.cfg.solver.timeLimitSeconds
        reason=search.terminationReason;
        if ~isempty(point),reason="positiveRestorablePcbfSlack";end
        wall=tic;freshAnchor=localFlowSeed(model,size(anchor.inputs,2));initializationSeconds=toc(wall);
        freshSource="movingTargetFlow";if isempty(model.target),freshSource="laneFeedbackRollout";end
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
    end
    [solution,search,model]=localSecondary(point,problem,anchor,model,search,timer);
    stages=[stages,search.stages(2:end)];attempts(selected)=localAttempt(search);
    % A failed CLF solve can still use the one available fresh initialization.
    if isempty(solution) && any(search.initialization==["shiftedInputRollout","sameFrameRollout"]) && ~restarted && model.allowFlowRestart ...
            && search.terminationReason=="clfNoNumericalResult" && toc(timer)<model.cfg.solver.timeLimitSeconds
        failure=search.terminationReason;wall=tic;anchor=localFlowSeed(model,size(anchor.inputs,2));initializationSeconds=toc(wall);
        source="movingTargetFlow";if isempty(model.target),source="laneFeedbackRollout";end
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
    search.linearizationCount=numel(attempts);search.solverCalls=sum([stages.numericalSolve]);search.flowRestarted=restarted;
    search.initializationFailure=failure;search.elapsedSeconds=toc(timer);
end

function [point,problem,search,model,extra,expanded]=localExpandPrimary(point,problem,search,model,anchor,timer)
    % An infeasible affine problem cannot be repaired by shrinking its input
    % correction box. Reuse the fresh flow and allow one bounded enlargement.
    % State trust, actuator bounds, safety rows and terminal constraints stay.
    extra=struct([]);expanded=false;
    infeasible=isempty(point) && ~isempty(search.stages) && any(search.stages(end).exitFlag==[-2,-7]);
    if infeasible && model.inputTrustScale<2 && toc(timer)<model.cfg.solver.timeLimitSeconds
        widerModel=model;widerModel.inputTrustScale=2;
        [widerPoint,widerProblem,widerSearch,widerModel]=localPrimary(anchor,widerModel,search.initialization,timer);
        widerSearch.initializationSeconds=0;extra=localAttempt(widerSearch);
        if isempty(point) || (~isempty(widerPoint) && widerSearch.primaryOptimum ...
                <search.primaryOptimum-model.cfg.solver.lexicographicTieTolerance)
            point=widerPoint;problem=widerProblem;search=widerSearch;model=widerModel;expanded=true;
        end
    end
end

function attempt=localAttempt(search)
    attempt=struct('initialization',search.initialization,'terminationReason',search.terminationReason, ...
        'solverCalls',search.solverCalls,'stages',search.stages,'primaryOptimum',search.primaryOptimum, ...
        'primaryLowerBound',search.primaryLowerBound,'inputTrustScale',search.inputTrustScale, ...
        'initializationSeconds',search.initializationSeconds,'formulationSeconds',search.formulationSeconds, ...
        'clfConstructionSeconds',search.clfConstructionSeconds);
end

function [point,problem,search,model]=localPrimary(anchor,model,initialization,timer)
    if ~isfield(model,'inputTrustScale'),model.inputTrustScale=1;end
    cfg=model.cfg;point=[];wall=tic;[problem,model]=localFormulate(anchor,model);
    model.linearization=anchor;
    search=struct('solverCalls',0,'source',"twoStageRealTimeIteration", ...
        'initialization',initialization,'returned',false,'converged',false,'terminationReason',"timeLimit", ...
        'formulationSeconds',toc(wall),'clfConstructionSeconds',0, ...
        'inputTrustScale',model.inputTrustScale,'primaryOptimum',NaN,'primaryLowerBound',problem.primaryLowerBound,'slackCap',NaN,'clfInitialSlack',NaN, ...
        'clfStageAttempted',false,'clfStageCompleted',false,'clfLowerBound',false, ...
        'stages',struct('objective',{},'exitFlag',{},'seconds',{},'value',{},'numericalSolve',{},'solverInfo',{}));
    remaining=cfg.solver.timeLimitSeconds-toc(timer);if remaining<=0,return;end
    primaryCount=problem.clfIndex-1;endpoint=problem.cones(1);wall=tic;
    % A feasible zero correction with zero nonnegative slacks attains the
    % global lower bound. Include every primary constraint, not just safety.
    if all(problem.b>=0) && all(problem.rhs==0) ...
            && all(problem.lower(1:primaryCount)<=0) && all(problem.upper(1:primaryCount)>=0) ...
            && norm(endpoint.b)<=-endpoint.gamma
        point=zeros(primaryCount,1);search.primaryOptimum=0;
        search.slackCap=cfg.solver.lexicographicTieTolerance;
        search.stages(1)=struct('objective',"pcbfSlack",'exitFlag',1, ...
            'seconds',toc(wall),'value',0,'numericalSolve',false,'solverInfo',struct());
        search.terminationReason="primaryReturned";return;
    end
    primaryCone=endpoint;primaryCone.A=endpoint.A(:,1:primaryCount);primaryCone.d=endpoint.d(1:primaryCount);
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

function [solution,search,model]=localSecondary(point,problem,anchor,model,search,timer)
    cfg=model.cfg;solution=[];if isempty(point),return;end
    wall=tic;problem=localAddClf(problem,anchor,model);
    search.clfConstructionSeconds=toc(wall);search.formulationSeconds=search.formulationSeconds+search.clfConstructionSeconds;
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
    inputs=anchor.inputs+reshape(second(problem.inputIndices),size(anchor.inputs));
    states=anchor.states+reshape(second(problem.stateIndices),size(anchor.states));
    slacks=max(0,second(problem.slackIndices));rho=problem.clfScale*max(0,second(problem.clfIndex));
    nextValue=(norm(problem.clfMap*second+problem.clfOffset)^2+problem.clfModelConstant)*problem.clfScale;
    model.terminal=terminalContinuation.fit(model.terminal,[states(:,end);inputs(:,end)],model.sampleIndex+size(inputs,2));
    solution=struct('inputs',inputs,'states',states,'stageSlacks',slacks.', ...
        'safety',sum(slacks),'hard',NaN,'clfSlack',rho, ...
        'clfInitialValue',problem.initialClfValue,'clfNextValue',nextValue, ...
        'clfFunction',problem.clf.function,'clfRequiredDecrease',problem.clf.requiredDecrease, ...
        'clfTieBound',cfg.solver.clfTieTolerance*problem.clfScale, ...
        'clfRolloutSteps',problem.clf.rolloutSteps,'clfTailReached',problem.clf.converged, ...
        'minimumCollisionMargin',NaN,'terminalSeparationMargin',NaN, ...
        'encounterExit',problem.encounterExit,'affineValidationPerformed',false,'nonlinearValidationPerformed',false);
    search.stages(2).value=rho;search.clfStageCompleted=true;search.returned=true;
    search.converged=all([search.stages.exitFlag]>0);
    search.clfLowerBound=rho<=cfg.solver.feasibilityTolerance*problem.clfScale;
    search.terminationReason="twoStagesReturned";
end

function [point,flag,solverInfo]=localConicSolve(quadratic,objective,cones,a,b,equal,rhs,lower,upper,cfg,remaining,gapTolerance)
    persistent available
    if isempty(available)
        directory=fullfile(fileparts(fileparts(mfilename('fullpath'))),'solver','controller');
        if isfolder(directory),addpath(directory);end
        assert(exist('predictiveConicSolverMex','file')==3, ...
            'collisionAvoidanceController:missingConicSolver', ...
            'Run scripts/buildPredictiveConicSolver to compile the Clarabel adapter.');
        available=true;
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
    [point,flag,solverInfo]=predictiveConicSolverMex(quadratic,objective, ...
        vertcat(rows{:}),vertcat(bounds{:}),dimensions,[0,1,2*ones(1,numel(cones))], ...
        [cfg.solver.maxIterations;remaining;cfg.solver.constraintTolerance;gapTolerance;cfg.solver.feasibilityTolerance]);
end

function cone=localCone(a,b,d,gamma)
    cone=struct('A',sparse(a),'b',full(b),'d',sparse(d),'gamma',gamma);
end

function [anchor,source,failure]=localInitialization(model,previous)
    cfg=model.cfg;
    if isfield(model,'iterationAnchor')
        anchor=model.iterationAnchor;source="sameFrameRollout";failure="";return;
    end
    count=min(cfg.controller.maximumHorizonSteps,cfg.controller.horizonSteps ...
        +ceil(cfg.nonlinear.recoveryHorizonSeconds/cfg.controller.sampleTime));
    source="movingTargetFlow";failure="";
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
    % A time-aligned Gaussian guides one transported-flow bicycle rollout.
    % The curve's nominal arrival clock is approximate; the actual rollout
    % and moving field always query the target at the same absolute time.
    % This is an initialization, with no safety or terminal admission.
    cfg=model.cfg;reference=model.nominalReference;x=model.initialState;previous=model.previousInput;
    h=cfg.controller.sampleTime;tire=modifiedFialaTire.parameters(cfg);
    inputs=zeros(2,count);states=zeros(6,count+1);states(:,1)=x;
    guide=predictiveSafetyGeometry.movingGaussianGuide(x,model.lane,model.targetEpoch, ...
        model.sampleIndex*h,count*h,cfg);radius=guide.radius;
    nominalTerminal=nonlinearBicycleModel.nominalTail(cfg,reference.curvature);
    station=[];completionStarted=false;
    for index=1:count
        time=(model.sampleIndex+index-1)*h;
        q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,time);
        projection=laneGeometry.project(x(1:2),model.lane,station);station=projection.station;
        direction=[cos(projection.heading);sin(projection.heading)];normal=[-direction(2);direction(1)];
        deviation=nonlinearBicycleModel.error(x,model.lane,reference);
        elapsed=(index-1)*h;active=guide.amplitude~=0 && elapsed<=guide.endTime;
        if active && ~completionStarted
            yaw=q(3);rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];offset=rotation*q(10:11);
            omega=q(4)*sin(q(6))/q(7);
            translation=q(4)*[cos(yaw+q(6));sin(yaw+q(6))]+omega*[-offset(2);offset(1)];
            z=(elapsed-guide.centerTime)/guide.width;
            lateral=guide.amplitude*exp(-.5*z^2);lateralRate=-z/guide.width*lateral;
            nominal=cfg.referenceSpeed*direction ...
                +(lateralRate-.5*(projection.lateralPosition-lateral))*normal;
            circulation=-sign(guide.amplitude)*.6*max(norm(nominal-translation),.5*cfg.referenceSpeed)/radius;
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
        elseif active,u=reference.input+reference.gain*deviation;
        else,u=nonlinearBicycleModel.nominalFeedback(x,previous,model.lane,reference,cfg,nominalTerminal);
        end
        u=localShape(u,x,previous,cfg,tire);
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
    lower(iu(:))=-repmat(model.inputTrustScale*radius*[.15;.25],count,1);upper(iu(:))=-lower(iu(:));lower([is,ic])=0;
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
        if index<=departure
            [g,j]=localSafetyRows(x,(index-1)*cfg.controller.sampleTime,model);
            if index==1,primaryLowerBound=max(0,-min(g));end
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
    objective=zeros(nv,1);objective(is)=1;
    problem=struct('a',vertcat(rows{1:rowCount}),'b',vertcat(bounds{1:rowCount}), ...
        'equal',equal,'rhs',rhs,'lower',lower,'upper',upper,'cones',endpoint, ...
        'stateIndices',ix,'inputIndices',iu,'slackIndices',is,'clfIndex',ic, ...
        'safetyObjective',objective,'primaryLowerBound',primaryLowerBound, ...
        'terminalA',terminalA,'terminalB',terminalB,'encounterExit',departure);
end

function problem=localAddClf(problem,anchor,model)
    nv=numel(problem.safetyObjective);iu=problem.inputIndices;ic=problem.clfIndex;count=size(iu,2);
    [map,offset,constant,value,scale,modelConstant,clf]=localNominalClf(anchor,model,iu(:,1),nv);
    axis=sparse(1,nv);axis(ic)=1;
    cone=localCone([2*map;axis],[-2*offset;1-constant],axis.',-1-constant);
    inputScale=repmat(1./(model.inputTrustScale*model.cfg.nonlinear.trustRadius*[.15;.25]*sqrt(2*count)),count,1);
    increment=sparse(1:numel(iu),iu(:),inputScale,numel(iu),nv);
    problem.quadratic=2*model.cfg.solver.clfTieTolerance*(increment.'*increment);
    problem.cones=[problem.cones,cone];
    problem.clfScale=scale;problem.clfMap=map;problem.clfOffset=offset;
    problem.clfModelConstant=modelConstant;problem.clf=clf;problem.initialClfValue=value;
    problem.initialClfSlack=max(0,(norm(offset)^2-constant)*scale);
end

function [map,offset,constant,value,scale,modelConstant,clf]=localNominalClf(anchor,model,next,nv)
    % One target-independent cost-to-go at every sample. Its first successor
    % depends on only two controls. Differentiate the complete held-input map
    % and nominal rollout; add the positive residual curvature to Gauss-Newton.
    % This is a local RTI model, not a certified nonlinear upper bound.
    cfg=model.cfg;reference=model.nominalReference;lane=model.lane;
    terminal=nonlinearBicycleModel.nominalTail(cfg,reference.curvature);
    x0=model.initialState;u0=anchor.inputs(:,1);
    [value,~,steps0,converged]=nonlinearBicycleModel.nominalValue(x0,model.previousInput,lane,reference,terminal,cfg);
    stage=sum((terminal.stageFactor*nonlinearBicycleModel.error(x0,lane,reference)).^2);
    phi=@(u)nonlinearBicycleModel.nominalValue(nonlinearBicycleModel.sample(x0,u,cfg), ...
        u,lane,reference,terminal,cfg);
    [center,residual]=phi(u0);delta=[1e-4;min(1e-4,(1-abs(u0(2)))/4)];
    hessian=zeros(2);jacobian=zeros(numel(residual),2);
    for index=1:2
        d=zeros(2,1);d(index)=delta(index);[plus,rp]=phi(u0+d);[minus,rm]=phi(u0-d);
        jacobian(:,index)=(rp-rm)/(2*delta(index));
        hessian(index,index)=(plus-2*center+minus)/delta(index)^2;
    end
    mixed=phi(u0+delta)-phi(u0+[delta(1);-delta(2)]) ...
        -phi(u0+[-delta(1);delta(2)])+phi(u0-delta);
    hessian(1,2)=mixed/(4*prod(delta));hessian(2,1)=hessian(1,2);
    % Preserve the nonnegative residual model, then add only positive
    % residual curvature. A plain PSD projection of the full Hessian can
    % have a negative quadratic minimum even though V itself is nonnegative.
    gaussNewton=2*(jacobian.'*jacobian);
    remainder=(hessian+hessian.')/2-gaussNewton;
    [basis,eigenvalues]=eig((remainder+remainder.')/2);
    curvature=diag(sqrt(max(0,diag(eigenvalues))/2))*basis.';
    [q,factor]=qr(jacobian,0);projected=q.'*residual;
    orthogonal=max(0,center-sum(projected.^2));scale=max(value,1);
    map=sparse(4,nv);map(:,next)=[factor;curvature]/sqrt(scale);
    offset=[projected;zeros(2,1)]/sqrt(scale);
    modelConstant=orthogonal/scale;
    required=cfg.nominalClf.decreaseFraction*stage;
    constant=(value-required-orthogonal)/scale;
    clf=struct('function',"nominalCostToGo",'requiredDecrease',required, ...
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
    q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,model.sampleIndex*model.cfg.controller.sampleTime+time);
end
