function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl One local PCBF/CLF RTI model per controller call.
% The planner runs once at encounter onset. Later frames shift the previous
% state/input iterate and retain its node count throughout the encounter.
% Horizon sizing occurs only at encounter onset; a failed solve never extends
% the horizon or rebuilds the problem. The completed local update returns
% its optimized inputs after tolerance-sized physical-bound normalization,
% without a post-solve nonlinear rollout.
% An active target selects PCBF slack first, then tracking and regularization
% on its optimal face. Both solves share the same linearization and budget.
% Outside an encounter one integrated-CLF/performance QP is solved. There is no
% cross-frame slack budget.
% Outside an encounter, cruise feedback generates a dynamically consistent
% reference from each new estimate before the single linearization.
% Research access: solvePredictiveControl("probe",request) exposes reference,
% formulation and solve operations to the free-time experiment in scripts/.
% It does not enable a variable grid in the online controller.
    if (isstring(model) || ischar(model)) && string(model)=="probe"
        [solution,search,model]=localProbe(previousState);return;
    end
    if nargin<3,timer=tic;end
    if ~isfield(model,'uncertainty')
        model.uncertainty=struct('specified',false,'egoGenerator',zeros(6), ...
            'collisionGenerator',zeros(6),'target',[],'relativeFrame',false);
    end
    cfg=model.cfg;model.linearizationBuilds=0;
    model.solveTimer=timer;
    model.solveTimeLimitSeconds=max(0,cfg.solver.timeLimitSeconds-cfg.solver.completionReserveSeconds);
    model.circulationDirection=0;
    if isfield(model,'targetOnset') && ~model.targetOnset && isstruct(previousState) && isfield(previousState,'circulationDirection')
        model.circulationDirection=previousState.circulationDirection;
    end
    minimumCount=cfg.controller.horizonSteps;
    if isfield(model,'referenceProbeOnly') && model.referenceProbeOnly && isfield(model,'probeMinimumHorizonSteps')
        minimumCount=max(minimumCount,model.probeMinimumHorizonSteps);
    end
    anchor=[];reference=[];plannerCalls=0;
    if isfield(model,'probeReference'),reference=model.probeReference;end
    source="nominalFeedbackRollout";
    rollout=struct('available',false,'reason',"notPerformedByRti", ...
        'fullGap',NaN,'fullCorrection',NaN);
    if ~isempty(reference),source="extendedInputRollout";
    elseif isfield(model,'targetOnset') && model.targetOnset,source="plannedAvoidanceRollout";
    elseif localCruiseFeedback(model,previousState),source="cruiseFeedbackRollout";
    elseif localHasWarmStart(previousState),source="shiftedPrimalIterate";
    elseif isfield(model,'target') && ~isempty(model.target),source="unavailableWarmStart";
    end
    search=localEmptySearch(source,"",0);
    try
        localCheckDeadline(model);
        wall=tic;
        if source=="plannedAvoidanceRollout",plannerCalls=plannerCalls+1;end
        [anchor,source]=localRtiInitialize(model,previousState,minimumCount,reference);
        search.horizonSteps=size(anchor.inputs,2);
        search.initializationSeconds=toc(wall);search.initialization=source;
        model.collisionEscapeNormals=zeros(2,2*size(anchor.inputs,2)+1);
        [model.collisionSide,model.retainCollisionSide]=localCollisionSide(model);
        if anchor.planner.used
            if anchor.planner.circulationDirection~=0,model.circulationDirection=anchor.planner.circulationDirection;end
            projection=laneGeometry.project(model.initialState(1:2),model.lane);
            lateral=anchor.guide.direction.'*[-sin(projection.heading);cos(projection.heading)];
            if abs(lateral)>1e-8,model.collisionSide=sign(lateral);end
        end
        if source~="plannedAvoidanceRollout" && ~isempty(model.target) && ~model.targetOnset && isstruct(previousState) ...
                && isfield(previousState,'collisionSide')
            model.collisionSide=previousState.collisionSide;
            if isfield(previousState,'retainCollisionSide'),model.retainCollisionSide=previousState.retainCollisionSide;end
        end
        anchor.terminalContext.collisionSide=model.collisionSide;
        model.terminalContext=anchor.terminalContext;
        if source~="plannedAvoidanceRollout" && ~isempty(model.target) && ~model.targetOnset ...
                && isfield(previousState,'collisionEscapeNormals') && isfield(previousState,'collisionSide')
            old=previousState.collisionEscapeNormals(:,3:end);
            count=min(size(old,2),size(model.collisionEscapeNormals,2));
            model.collisionEscapeNormals(:,1:count)=old(:,1:count);
        end
        localCheckDeadline(model);
        if isfield(model,'referenceProbeOnly') && model.referenceProbeOnly
            search.plannerCalls=plannerCalls;
            model.linearization=anchor;solution=[];return;
        end
        wall=tic;[problem,model]=localFormulate(anchor,model);
        search.formulationSeconds=toc(wall);search.modelBuilt=true;
        model.linearization=anchor;problem.inheritedAnchor=source=="shiftedPrimalIterate";
        initializationSeconds=search.initializationSeconds;formulationSeconds=search.formulationSeconds;
        [solution,search,model]=localSolve(anchor,model,source,timer,problem);
        search.modelBuilt=true;search.formulationSeconds=formulationSeconds+search.clfConstructionSeconds;
        search.initializationSeconds=initializationSeconds;
        if ~isempty(solution)
            solution.collisionEscapeNormals=model.collisionEscapeNormals;
            solution.collisionSide=model.collisionSide;
            solution.circulationDirection=model.circulationDirection;
            solution.retainCollisionSide=model.retainCollisionSide;
            solution.clfNextValue=nonlinearBicycleModel.nominalValue( ...
                solution.states(:,2),model.lane,model.nominalReference);
            solution=localTerminalMetrics(solution,model);
        end
    catch exception
        if isfield(model,'referenceProbeOnly') && model.referenceProbeOnly,rethrow(exception);end
        if strcmp(exception.identifier,'collisionAvoidanceController:timeBudgetExceeded')
            solution=[];search.terminationReason="timeLimit";
        elseif any(strcmp(exception.identifier,{'collisionAvoidanceController:invalidTireOperatingPoint', ...
                'collisionAvoidanceController:singularTireLinearization','collisionAvoidanceController:nonlinearDomain', ...
                'collisionAvoidanceController:missingWarmStart'}))
            % A missing or unrollable continuation is a failure. Only a
            % newly observed encounter may request avoidance planning.
            solution=[];
            search.initialization=source;search.initializationFailure=string(exception.identifier);
            search.terminationReason="referencePredictionUnavailable";
        else
            rethrow(exception);
        end
    end
    history=localAttempt(search);
    search.source="switchedRealTimeIteration";search.algorithm="realTimeIteration";
    search.initialization=source;search.returned=~isempty(solution);
    % A completed convex subproblem is not nonlinear SQP convergence.
    search.converged=false;
    search.linearizationCount=model.linearizationBuilds;
    search.rtiIterations=model.linearizationBuilds;
    search.nominalRestarted=false;search.restartCount=0;
    search.plannerCalls=plannerCalls;
    search.horizonExtensions=0;
    search.stopReason=search.terminationReason;
    search.rollout=rollout;
    search.anchorCertified=false;search.appendedTerminalSteps=0;search.appendedBrakingSteps=0;
    search.planner=localPlannerMetadata();
    if ~isempty(anchor)
        search.appendedTerminalSteps=anchor.appendedTerminalSteps;
        search.appendedBrakingSteps=anchor.appendedBrakingSteps;search.planner=anchor.planner;
    end
    search.potentialFieldRestarted=false;
    if ~isfield(search,'initializationFailure'),search.initializationFailure="";end
    search.attempts=history;search.selectedAttempt=numel(history)*double(search.returned);
    search.solverCalls=sum([history.solverCalls]);
    search.initializationSeconds=sum([history.initializationSeconds]);
    search.formulationSeconds=sum([history.formulationSeconds]);
    search.clfConstructionSeconds=sum([history.clfConstructionSeconds]);
    search.elapsedSeconds=toc(timer);
end

function localCheckDeadline(model)
    if isfield(model,'solveTimer') && toc(model.solveTimer)>=model.solveTimeLimitSeconds
        error('collisionAvoidanceController:timeBudgetExceeded','The current RTI computation budget is exhausted.');
    end
end

function [side,retain]=localCollisionSide(model)
    side=1;retain=false;if isempty(model.target),return;end
    projection=laneGeometry.project(model.target(1:2),model.lane);
    forward=[cos(projection.heading);sin(projection.heading)];
    course=predictiveSafetyGeometry.direction(model.target(3)+model.target(6));
    retain=abs(forward.'*course)>=sqrt(.5);
    tolerance=1e-6;
    if ~isempty(model.uncertainty.target),tolerance=tolerance+model.uncertainty.target.positionRadius;end
    if abs(projection.lateralPosition)>tolerance
        side=-sign(projection.lateralPosition);
    elseif ~isempty(model.road.lateralClearance) && model.road.lateralClearance(1)>model.road.lateralClearance(2)
        side=-1;
    end
end

function count=localPlanningHorizon(model,minimumCount)
    % Size the geometric path around the encounter from the measured motion.
    % This preview sizes the initial horizon once per encounter. Subsequent
    % frames retain the accepted input sequence's length without previewing.
    % Doubling the geometry grid costs O(N) in its final node count. The same
    % wall-clock deadline applies during this search and the one RTI solve.
    cfg=model.cfg;h=cfg.controller.sampleTime;minimumDuration=h*minimumCount;duration=minimumDuration;
    x=model.initialState;speed=hypot(x(4),x(5));beta=atan2(x(5),x(4));
    % A near-equal-speed target can put the next geometric intersection tens
    % of seconds away. Initial search need only span a physically meaningful
    % stopping/settling maneuver. The seed need not end in the terminal set.
    % RTI enforces endpoint constraints once on the selected horizon; an
    % infeasible result is reported without extending or rebuilding it.
    preview=max(minimumDuration,speed/cfg.terminal.brakingDecelerationMetersPerSecondSquared+cfg.rti.settlingSeconds);
    % Preview the measured course and yaw rate, not the desired cruise arc.
    curvature=x(6)/max(speed,cfg.model.scheduleSpeedFloor);
    arcBeta=asin(min(1,max(-1,curvature*cfg.vehicle.lr)));
    egoArc=[x(1:2);x(3)+beta-arcBeta;speed;0;arcBeta;cfg.vehicle.lr; ...
        cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    generator=model.uncertainty.collisionGenerator;
    reserve=sum(vecnorm(generator(1:2,:))) ...
        +2*(norm(shape(1:2))+norm(shape(3:4)))*sin(min(pi,sum(abs(generator(3,:))))/2);
    if ~isempty(model.uncertainty.target)
        set=model.uncertainty.target;
        reserve=reserve+set.positionRadius ...
            +2*(norm(model.target(8:9))+norm(model.target(10:11)))*sin(min(pi,set.courseRadius+set.sideslipRadius)/2);
    end
    exitReserve=reserve;
    reach=norm(shape(1:2))+norm(shape(3:4))+norm(model.target(8:9))+norm(model.target(10:11)) ...
        +reserve;
    while true
        localCheckDeadline(model);
        duration=min([duration,preview,h*cfg.controller.maximumHorizonSteps]);
        time=(0:ceil(duration/h))*h;
        forecast=predictiveSafetyGeometry.predictTarget(egoArc,time);
        position=forecast(1:2,:);heading=forecast(3,:)+arcBeta-beta;
        target=localTargetAt(model,time);separation=vecnorm(position-target(1:2,:),2,1);
        outside=find(separation>cfg.collision.encounterRangeMeters+exitReserve,1);
        last=numel(time);if ~isempty(outside),last=outside-1;end
        conflictNodes=false(1,last);
        for node=1:last
            if mod(node,16)==1,localCheckDeadline(model);end
            if separation(node)>reach,continue;end
            gap=predictiveSafetyGeometry.rectangle([position(:,node);heading(node)], ...
                shape,target(1:3,node),target(8:11,node));
            conflictNodes(node)=gap<=reserve;
        end
        conflict=any(conflictNodes);
        difference=diff(separation(1:last));
        stationary=isempty(difference) || all(abs(difference)<=1e-10);
        approaching=~isempty(difference) && difference(end)<-1e-10;
        bracketed=~isempty(outside) || stationary || (~approaching && ~conflict);
        if conflict && ~conflictNodes(end),bracketed=true;end
        if bracketed || duration>=preview || numel(time)-1>=cfg.controller.maximumHorizonSteps,break;end
        duration=2*duration;
    end
    duration=minimumDuration;
    if conflict
        duration=max(minimumDuration,time(find(conflictNodes,1,'last'))+cfg.rti.settlingSeconds);
    end
    count=min(ceil(duration/h),cfg.controller.maximumHorizonSteps);
end

function available=localHasWarmStart(previous)
    available=isstruct(previous) && isfield(previous,'inputTrajectory') ...
        && size(previous.inputTrajectory,1)==2 && size(previous.inputTrajectory,2)>=1 ...
        && all(isfinite(previous.inputTrajectory),'all') ...
        && isfield(previous,'stateTrajectory') ...
        && isequal(size(previous.stateTrajectory),[6,size(previous.inputTrajectory,2)+1]) ...
        && all(isfinite(previous.stateTrajectory),'all');
end

function feedback=localCruiseFeedback(model,previous)
    feedback=isfield(model,'target') && isempty(model.target) && localHasWarmStart(previous);
end

function [anchor,source]=localRtiInitialize(model,previous,minimumCount,reference)
    cfg=model.cfg;count=minimumCount;reuse=~isempty(reference);
    onset=isfield(model,'targetOnset') && model.targetOnset;
    feedback=~reuse && localCruiseFeedback(model,previous);
    inherited=~reuse && ~onset && ~feedback && localHasWarmStart(previous);
    if ~reuse && onset && ~isempty(model.target)
        count=localPlanningHorizon(model,minimumCount);
        anchor=localPlanReference(model,count);source="plannedAvoidanceRollout";return;
    end
    if ~reuse && ~inherited && ~isempty(model.target)
        error('collisionAvoidanceController:missingWarmStart', ...
            'An active encounter requires the preceding optimized inputs; planning is restricted to encounter onset.');
    end
    old=zeros(2,0);source="nominalFeedbackRollout";
    if reuse
        old=reference.inputs;source="extendedInputRollout";
        count=max(minimumCount,size(old,2));
    elseif feedback
        source="cruiseFeedbackRollout";
        % Preserve the ordinary target-free horizon schedule, but rebuild
        % its reference with cruise feedback from the current measurement.
        % Repeating this in cruise prevents its next linearization from
        % inheriting a large affine correction made on the preceding frame.
        if localHasWarmStart(previous)
            count=max(minimumCount,size(previous.inputTrajectory,2)-1);
        end
    elseif inherited
        old=previous.inputTrajectory(:,2:end);source="shiftedPrimalIterate";
        % The issued input stays unchanged; only singular endpoint references
        % move into the differentiable interior before the next linearization.
        [seedLower,seedUpper]=nonlinearBicycleModel.inputBounds(cfg,true);
        old=min(seedUpper,max(seedLower,old));
        count=max(minimumCount,size(old,2));
        if ~isempty(model.target)
            % The first accepted sequence fixes M for this encounter. Drop
            % its executed input and append one tail hold, retaining M even
            % when the target forecast or observer enclosure changes.
            count=size(previous.inputTrajectory,2);
        end
    end
    count=min(count,cfg.controller.maximumHorizonSteps);
    inputs=zeros(2,count);states=zeros(6,count+1);states(:,1)=model.initialState;
    context=terminalSafeSet.context(model);
    context.solveTimer=model.solveTimer;context.solveTimeLimitSeconds=model.solveTimeLimitSeconds;
    appended=0;braking=0;
    prior=model.previousInput;nominal=nonlinearBicycleModel.nominalGuidanceParameters(cfg,model.nominalReference.curvature);
    for index=1:count
        if index<=size(old,2)
            u=old(:,index);
            if inherited
                % Multiple-shooting warm start: retain the shifted state
                % decision instead of replaying the whole input sequence.
                % The measured first state and the shifted later nodes may
                % have dynamics defects; localFormulate includes them in rhs.
                next=previous.stateTrajectory(:,index+2);
            else
                next=nonlinearBicycleModel.sample(states(:,index),u,cfg);
            end
        elseif (inherited || reuse) && terminalSafeSet.member(context,states(:,index),index-1)
            [u,next,ok,~,middle]=terminalSafeSet.cruiseInput(states(:,index),model);
            ok=ok && abs(u(2)-prior(2))<=context.brakingChangeMaximum+1e-9 ...
                && terminalSafeSet.clears(context,middle,index-.5) ...
                && terminalSafeSet.member(context,next,index);
            if ~ok
                [u,next,ok]=terminalSafeSet.brakingInput(states(:,index),prior,u,model,context,index-1);
                braking=braking+double(ok);
            end
            if ~ok
                u=localTailSeed(states(:,index),prior,model,nominal);
                next=nonlinearBicycleModel.sample(states(:,index),u,cfg);
            end
            appended=appended+double(ok);
        else
            u=localTailSeed(states(:,index),prior,model,nominal);
            next=nonlinearBicycleModel.sample(states(:,index),u,cfg);
        end
        inputs(:,index)=u;states(:,index+1)=next;prior=u;
        if mod(index,4)==0,localCheckDeadline(model);end
    end
    anchor=struct('inputs',inputs,'states',states,'terminalContext',context, ...
        'appendedTerminalSteps',appended,'appendedBrakingSteps',braking,'planner',localPlannerMetadata());
    if reuse
        anchor.appendedTerminalSteps=anchor.appendedTerminalSteps+reference.appendedTerminalSteps;
        anchor.appendedBrakingSteps=anchor.appendedBrakingSteps+reference.appendedBrakingSteps;
        anchor.planner=reference.planner;
        if isfield(reference,'guide'),anchor.guide=reference.guide;end
    end
end

function input=localTailSeed(x,prior,model,parameters)
    input=nonlinearBicycleModel.nominalFeedback(x,prior,model.lane,model.nominalReference,model.cfg,parameters);
    if isfield(model.road,'boundaryObservation') && hypot(x(4),x(5))>model.cfg.model.brakeHoldSpeed
        input=predictiveSafetyGeometry.boundaryReferenceInput(x,input,prior,model);
    end
    % An uncertified stopped tail must not restart into a positive-input
    % reference. Moving tails still seek the nominal cruise equilibrium.
    if hypot(x(4),x(5))<=.2*model.cfg.model.brakeHoldSpeed
        [lower,~]=nonlinearBicycleModel.inputBounds(model.cfg,true);
        input=[prior(1);max(lower(2),min(0,prior(2)))];
    end
end

function anchor=localPlanReference(model,count)
    cfg=model.cfg;wall=tic;
    x=model.initialState;course=x(3)+atan2(x(5),x(4));
    lateral=[-sin(course),cos(course)]*(model.target(1:2)-x(1:2));
    tolerance=1e-6;
    if ~isempty(model.uncertainty.target),tolerance=tolerance+model.uncertainty.target.positionRadius;end
    side=1;if abs(lateral)>tolerance,side=-sign(lateral);end
    guide=predictiveSafetyGeometry.avoidanceGuide(model,count,side);
    context=terminalSafeSet.context(model);
    context.solveTimer=model.solveTimer;context.solveTimeLimitSeconds=model.solveTimeLimitSeconds;
    planner=localPlannerMetadata();planner.used=true;planner.candidateCount=guide.candidateCount;
    planner.maximumDeviationMeters=guide.maximumDeviation;
    planner.candidates=guide.candidates;planner.circulationDirection=guide.directionSign;
    planner.referenceMinimumNodeGapMeters=guide.nominalMinimumGap;
    planner.referenceMinimumInflatedGapMeters=guide.minimumGap;
    planner.referenceStateViolation=guide.stateViolation;
    planner.sampledNominalChecksPassed=guide.nominalMinimumGap>0 && guide.stateViolation<=1e-9;
    planner.conflictStartSeconds=guide.conflictStartSeconds;planner.conflictEndSeconds=guide.conflictEndSeconds;
    planner.releaseSeconds=guide.releaseNode*cfg.controller.sampleTime;
    planner.rolloutSeconds=toc(wall);
    anchor=struct('inputs',guide.inputs,'states',guide.states,'terminalContext',context,'guide',guide, ...
        'appendedTerminalSteps',0,'appendedBrakingSteps',0,'planner',planner);
end

function planner=localPlannerMetadata()
    planner=struct('used',false,'maximumDeviationMeters',NaN, ...
        'candidateCount',0,'algorithm',"safetyCirculation", ...
        'circulationDirection',0,'candidates',struct([]), ...
        'referenceMinimumNodeGapMeters',NaN,'referenceMinimumInflatedGapMeters',NaN, ...
        'referenceStateViolation',NaN,'sampledNominalChecksPassed',false, ...
        'conflictStartSeconds',NaN,'conflictEndSeconds',NaN,'releaseSeconds',NaN, ...
        'rolloutSeconds',0,'robustCertificate',false);
end

function [check,problem,formed]=localNonlinearCheck(anchor,model,normals)
    % Offline probe only; never called by the online RTI update.
    % At zero correction the reassembled rows evaluate the supplied rollout,
    % including its true yaw, posterior margins and terminal continuation.
    % Keep the solve's separating normals: validation cannot select an easier
    % collision branch. This is a sampled numerical check, not a tube proof.
    model.fixedCollisionNormals=normals;builds=model.linearizationBuilds;
    [problem,formed]=localFormulate(anchor,model);
    formed.linearizationBuilds=builds;
    check=localPrimaryResidual(problem);
end

function check=localPrimaryResidual(problem)
    % Evaluate the original primary rows at zero correction to the rollout.
    deficit=max(0,-problem.b);slack=problem.slackIndices;
    soft=problem.a(:,slack);hard=~any(soft,2);
    required=zeros(numel(slack),1);
    for k=1:numel(slack)
        required(k)=max([0;deficit(soft(:,k)<0)]);
    end
    boundViolation=max([0;problem.lower;-problem.upper]);
    check=struct('safety',sum(required),'slacks',required.', ...
        'hard',max([boundViolation;deficit(hard)]));
    check.terminalViolation=max([0;-problem.terminalBounds]);
end

function context=localTerminalContext(anchor,model)
    if isfield(model,'terminalContext') && ~isempty(model.terminalContext)
        context=model.terminalContext;
    elseif isfield(anchor,'terminalContext') && ~isempty(anchor.terminalContext)
        context=anchor.terminalContext;
    else
        context=terminalSafeSet.context(model);
    end
    if isfield(model,'solveTimer')
        context.solveTimer=model.solveTimer;context.solveTimeLimitSeconds=model.solveTimeLimitSeconds;
    end
end

function search=localEmptySearch(source,failure,seconds)
    search=struct('solverCalls',0,'source',"switchedRealTimeIteration",'modelBuilt',false, ...
        'initialization',source,'returned',false,'converged',false,'terminationReason',"anchorUnavailable", ...
        'formulationSeconds',0,'clfConstructionSeconds',0, ...
        'primaryOptimum',NaN,'primaryLowerBound',NaN,'slackCap',NaN,'clfInitialSlack',NaN, ...
        'clfStageAttempted',false,'clfStageCompleted',false,'clfLowerBound',false, ...
        'secondaryStageAttempted',false,'secondaryStageCompleted',false, ...
        'pcbfStageAttempted',false,'pcbfStageCompleted',false,'convexSolveCompleted',false, ...
        'mode',"", ...
        'stages',struct('objective',{},'exitFlag',{},'seconds',{},'value',{},'numericalSolve',{},'solverInfo',{}), ...
        'initializationSeconds',seconds,'initializationFailure',failure,'elapsedSeconds',seconds, ...
        'potentialFieldRestarted',false,'horizonSteps',0);
    search.attempts=localAttempt(search);search.selectedAttempt=0;
end

function attempt=localAttempt(search)
    attempt=struct('initialization',search.initialization,'terminationReason',search.terminationReason, ...
        'solverCalls',search.solverCalls,'stages',search.stages,'primaryOptimum',search.primaryOptimum, ...
        'primaryLowerBound',search.primaryLowerBound, ...
        'slackCap',search.slackCap,'initializationSeconds',search.initializationSeconds,'formulationSeconds',search.formulationSeconds, ...
        'clfConstructionSeconds',search.clfConstructionSeconds);
    attempt.modelBuilt=search.modelBuilt;
    attempt.horizonSteps=search.horizonSteps;
end

function search=localSearch(problem,initialization,formulationSeconds)
    search=struct('solverCalls',0,'source',"switchedRealTimeIteration", ...
        'modelBuilt',formulationSeconds>0, ...
        'initialization',initialization,'returned',false,'converged',false,'terminationReason',"timeLimit", ...
        'formulationSeconds',formulationSeconds,'clfConstructionSeconds',0, ...
        'primaryOptimum',NaN,'primaryLowerBound',problem.primaryLowerBound,'slackCap',NaN,'clfInitialSlack',NaN, ...
        'initializationSeconds',0, ...
        'clfStageAttempted',false,'clfStageCompleted',false,'clfLowerBound',false, ...
        'secondaryStageAttempted',false,'secondaryStageCompleted',false, ...
        'pcbfStageAttempted',false,'pcbfStageCompleted',false,'convexSolveCompleted',false, ...
        'mode',"", ...
        'stages',struct('objective',{},'exitFlag',{},'seconds',{},'value',{},'numericalSolve',{},'solverInfo',{}), ...
        'horizonSteps',size(problem.inputIndices,2));
end

function [solution,search,model]=localSolve(anchor,model,initialization,timer,problem)
    % One RTI model. Avoidance orders slack before quadratic performance;
    % cruise uses its integrated continuous CLF and performance in one QP.
    localCheckDeadline(model);
    cfg=model.cfg;solution=[];
    search=localSearch(problem,initialization,0);
    if ~isfield(problem,'mode')
        wall=tic;problem=localSelectObjective(problem,anchor,model);
        search.clfConstructionSeconds=toc(wall);
    end
    search.mode=problem.mode;search.clfInitialSlack=problem.initialClfSlack;
    search.clfStageAttempted=problem.clf.enabled;
    search.pcbfStageAttempted=~problem.clf.enabled;
    if problem.clf.enabled,search.primaryLowerBound=NaN;end
    remaining=model.solveTimeLimitSeconds-toc(timer);if remaining<=0,return;end
    objective=problem.safetyObjective;quadratic=sparse(numel(objective),numel(objective));
    stageObjective="pcbfSlack";
    if problem.clf.enabled
        objective=problem.performanceObjective;quadratic=problem.quadratic;
        stageObjective=problem.optimizationObjective;
    end
    wall=tic;point=[];
    if ~problem.clf.enabled,point=localZeroSlackCandidate(problem);end
    numerical=isempty(point);
    if numerical
        [point,flag,solverInfo]=localConicSolve(quadratic,objective,problem.cones, ...
            problem.a,problem.b,problem.equal,problem.rhs,problem.lower,problem.upper,cfg,remaining, ...
            min(cfg.solver.optimalityTolerance,cfg.solver.constraintTolerance));
        search.solverCalls=1;
    else
        flag=1;solverInfo=struct();
    end
    search.stages(1)=struct('objective',stageObjective,'exitFlag',flag, ...
        'seconds',toc(wall),'value',NaN,'numericalSolve',numerical,'solverInfo',solverInfo);
    if isempty(point) || any(~isfinite(point)) || flag<=0
        search.terminationReason=problem.mode+"NoNumericalResult";return;
    end
    solution=localBoundedDecode(point,problem,anchor,cfg);
    if isempty(solution)
        search.terminationReason=problem.mode+"InputBoundsViolation";return;
    end
    search.stages(1).value=solution.objectiveValue;
    search.returned=true;search.convexSolveCompleted=true;
    search.clfStageCompleted=problem.clf.enabled;
    search.pcbfStageCompleted=~problem.clf.enabled;
    search.clfLowerBound=problem.clf.enabled && solution.clfSlack<=cfg.solver.feasibilityTolerance*problem.clfScale;
    search.terminationReason=problem.mode+"Returned";
    if problem.clf.enabled,return;end
    search.primaryOptimum=solution.safety;
    if problem.referenceDeviationWeight==0 && problem.nominalTrackingWeight==0 && problem.inputVariationWeight==0,return;end

    % Preserve the attained safety optimum, without a weighted tradeoff or
    % an intentional slack allowance. Solver tolerances remain numerical.
    search.slackCap=search.primaryOptimum;
    secondary=problem;
    if search.slackCap==0
        secondary.upper(problem.slackIndices)=0;
    else
        secondary.a=[problem.a;problem.safetyObjective.'];
        secondary.b=[problem.b;search.slackCap];
    end
    search.secondaryStageAttempted=true;
    wall=tic;numerical=false;flag=1;solverInfo=struct();
    if solution.performanceCost==0
        % Zero is also the global lower bound of the quadratic tie-breaker.
        refined=solution;
    else
        remaining=model.solveTimeLimitSeconds-toc(timer);refined=[];flag=0;
        if remaining>0
            numerical=true;search.solverCalls=search.solverCalls+1;
            [second,flag,solverInfo]=localConicSolve(problem.quadratic, ...
                problem.performanceObjective,secondary.cones,secondary.a,secondary.b, ...
                secondary.equal,secondary.rhs,secondary.lower,secondary.upper,cfg,remaining, ...
                min(cfg.solver.optimalityTolerance,cfg.solver.constraintTolerance));
            if ~isempty(second) && all(isfinite(second)) && flag>0
                refined=localBoundedDecode(second,secondary,anchor,cfg);
            end
        end
    end
    search.stages(2)=struct('objective',"nominalTrackingAndRegularization",'exitFlag',flag, ...
        'seconds',toc(wall),'value',NaN,'numericalSolve',numerical,'solverInfo',solverInfo);
    tolerance=cfg.solver.constraintTolerance*max(1,search.slackCap);
    if ~isempty(refined) && refined.safety<=search.slackCap+tolerance
        solution=refined;search.secondaryStageCompleted=true;
        search.stages(2).value=solution.performanceCost;
    else
        % Keep the completed priority result; refinement failure does not
        % trigger another solve or invalidate the already available input.
        search.terminationReason="pcbfReturnedWithoutRefinement";
    end
    solution.secondaryObjective="nominalTrackingAndRegularization";
    solution.secondaryObjectiveValue=solution.performanceCost;
end

function point=localZeroSlackCandidate(problem)
    % A feasible affine candidate with zero slack establishes the primary
    % lower bound without an LP. This is not nonlinear integration.
    point=zeros(size(problem.lower));
    if localAffineFeasible(point,problem),return;end
    indices=problem.stateIndices(:);dynamics=problem.equal(:,indices);
    if size(dynamics,1)~=numel(indices) || any(diag(dynamics)~=1)
        point=[];return;
    end
    point(indices)=dynamics\problem.rhs;
    if ~localAffineFeasible(point,problem),point=[];end
end

function feasible=localAffineFeasible(point,problem)
    tolerance=100*eps(max(1,norm(point,Inf)));
    feasible=all(isfinite(point)) && all(problem.a*point<=problem.b+tolerance) ...
        && all(abs(problem.equal*point-problem.rhs)<=tolerance) ...
        && all(point>=problem.lower-tolerance) && all(point<=problem.upper+tolerance);
    for index=1:numel(problem.cones)
        cone=problem.cones(index);
        feasible=feasible && all(vecnorm(reshape(cone.A*point+cone.b,cone.size,[]),2,1) ...
            <=reshape(cone.D*point-cone.gamma,1,[])+tolerance);
    end
end

function solution=localBoundedDecode(point,problem,anchor,cfg)
    % Solver feasibility is numerical, whereas the force model requires
    % exact physical bounds. Project only tolerance-sized input residuals.
    solution=[];
    inputs=anchor.inputs+reshape(point(problem.inputIndices),size(anchor.inputs));
    bounded=min(max(inputs,problem.physicalInputLower),problem.physicalInputUpper);
    correction=bounded-inputs;boundCorrection=max(abs(correction),[],'all');
    if boundCorrection>cfg.solver.feasibilityTolerance,return;end
    changed=correction~=0;indices=problem.inputIndices(changed);
    point(indices)=bounded(changed)-anchor.inputs(changed);
    % Keep the solved affine states and report their normalized-input
    % residual. Do not round-trip an exact endpoint through anchor+increment.
    solution=localDecode(point,problem,anchor,bounded);
    solution.inputBoundCorrection=boundCorrection;
end

function solution=localDecode(point,problem,anchor,inputs)
    states=anchor.states+reshape(point(problem.stateIndices),size(anchor.states));
    slacks=max(0,point(problem.slackIndices));rho=NaN;rate=NaN;
    if problem.clf.enabled
        rho=problem.clfScale*max(0,point(problem.clfIndex));
        rate=(problem.clfDerivativeMap*point+problem.clfDerivativeOffset)*problem.clfScale;
    end
    solution=struct('inputs',inputs,'states',states,'stageSlacks',slacks.', ...
        'safety',sum(slacks),'hard',max([0;problem.a*point-problem.b; ...
            abs(problem.equal*point-problem.rhs);problem.lower-point;point-problem.upper]), ...
        'clfSlack',rho,'clfEnabled',problem.clf.enabled, ...
        'secondaryObjective',"none",'secondaryObjectiveValue',NaN,'linearizationGap',NaN, ...
        'optimizationMode',problem.mode,'optimizationObjective',problem.optimizationObjective, ...
        'clfActualDerivative',NaN,'clfActualDerivativeResidual',NaN, ...
        'clfInitialValue',problem.initialClfValue,'clfDerivative',rate, ...
        'clfFunction',problem.clf.function,'clfRequiredDecreaseRate',problem.clf.requiredDecreaseRate, ...
        'referenceDeviationWeight',problem.referenceDeviationWeight,'clfDerivativeLimit',problem.clfDerivativeLimit, ...
        'maximumCollisionTighteningMeters',problem.maximumCollisionTighteningMeters, ...
        'minimumCollisionMargin',NaN, ...
        'encounterExit',problem.encounterExit,'affineValidationPerformed',false,'nonlinearValidationPerformed',false, ...
        'referenceStateDeviation',norm(problem.referenceStateMap*point)^2, ...
        'referenceInputDeviation',norm(problem.referenceInputMap*point)^2);
    solution.referenceDeviationCost=.5*problem.referenceDeviationWeight ...
        *(solution.referenceStateDeviation+solution.referenceInputDeviation);
    solution.nominalTrackingCost=.5*problem.nominalTrackingWeight ...
        *norm(problem.nominalTrackingOffset+problem.nominalTrackingMap*point)^2;
    solution.inputVariationCost=.5*problem.inputVariationWeight ...
        *norm(problem.inputVariationOffset+problem.inputVariationMap*point)^2;
    solution.performanceCost=solution.referenceDeviationCost+solution.nominalTrackingCost+solution.inputVariationCost;
    changes=[reshape(point(problem.stateIndices(:,1:end-1)),6,[]); ...
        reshape(point(problem.inputIndices),2,[])];
    solution.trustMaximumScaledCorrection=max(abs(changes)./problem.trustRadii,[],'all');
    solution.trustFirstInputRadius=problem.trustRadii(7:8,1);
    solution.trustSampledErrorRatio=max(problem.trustSampledErrorRatio);
    solution.objectiveValue=solution.safety;
    if problem.clf.enabled,solution.objectiveValue=solution.performanceCost+problem.clfSlackDuration*solution.clfSlack;end
    if isfield(problem,'durationIndex')
        solution.durationChangeSeconds=point(problem.durationIndex);
        solution.states=solution.states+problem.durationStateDerivative*solution.durationChangeSeconds;
    end
    solution.affineStates=solution.states;
    solution.affineSafety=solution.safety;solution.affineStageSlacks=solution.stageSlacks;
    solution.inputCorrection=inputs-anchor.inputs;
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
    [point,flag,solverInfo]=solver(triu(quadratic),objective, ...
        vertcat(rows{:}),vertcat(bounds{:}),dimensions,[0,1,2*ones(1,numel(coneDimensions))], ...
        [cfg.solver.maxIterations;remaining;cfg.solver.constraintTolerance;gapTolerance;cfg.solver.reducedTolerance]);
end

function [problem,model]=localFormulate(anchor,model)
    model.linearizationBuilds=model.linearizationBuilds+1;
    cfg=model.cfg;count=size(anchor.inputs,2);prefix=cfg.controller.horizonSteps;
    if ~isfield(model,'collisionEscapeNormals') || size(model.collisionEscapeNormals,2)~=2*count+1
        model.collisionEscapeNormals=zeros(2,2*count+1);
    end
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
    blockRows=cell(1,14*count+10);blockColumns=blockRows;blockBounds=blockRows;blockCount=0;
    cones=struct('A',{},'b',{},'D',{},'gamma',{},'size',{});
    lower=-Inf(nv,1);upper=Inf(nv,1);
    lower([is,ic])=0;
    % Position translations stay unrestricted. Numerical trust bounds on the
    % nonlinear coordinates are shared by the primary and secondary solves.
    [inputLower,inputUpper]=nonlinearBicycleModel.inputBounds(cfg);
    rate=[Inf;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;finite=isfinite(rate);
    [physicalLower,physicalUpper]=localStateLimits(cfg);
    primaryLowerBound=0;departure=0;encounterEnded=isempty(model.target);exitRecord=NaN;
    fullCollisionRows=isfield(model,'durationProbe') && model.durationProbe && ~isempty(model.target);
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
    holdCenter=model.initialState;h=cfg.controller.sampleTime;
    stages=localLinearization(anchor,model);
    physicalInputLower=repmat(inputLower,1,count);physicalInputUpper=repmat(inputUpper,1,count);
    maximumTightening=0;safetyNormals=zeros(2,2*count+1);
    [durations,times]=localTimeGrid(model,count);
    % Record time j*h/2 lies in hold ceil(j/2), which starts at (ceil(j/2)-1)*h.
    holdStarts=times(2*max(0,ceil((0:2*count)/2)-1)+1);
    model.recordTimes=times;
    targetTube=localTargetTube(model,times,holdStarts);
    model.targetTube=targetTube;
    starts=localTargetAt(model,holdStarts);
    model.targetSupportData=predictiveSafetyGeometry.targetSupportData(starts,model.uncertainty.target,times-holdStarts);
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
    tangent=tan(cfg.model.sideslipMaximum);side=[0,0,0,-tangent,1,0;0,0,0,-tangent,-1,0];
    % Without a bounded pose correction, reference distance cannot justify
    % omitting a collision row or declaring a future perception exit. Retain
    % the active target over this horizon; actual exit is handled at the next
    % observation. A presently out-of-range target has no active encounter.
    if ~fullCollisionRows && ~encounterEnded
        currentGenerator=localRelativeGenerator(anchor.states(:,1),generator,model,holdCenter);
        encounterEnded=localBeyond(anchor.states(:,1),forecast,1,currentGenerator,targetTube,cfg);
        if encounterEnded,exitRecord=1;end
    end
    for index=1:count
        if mod(index,4)==1,localCheckDeadline(model);end
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
            elapsed=h;if index>1,elapsed=durations(index-1);end
            rate=[Inf;cfg.model.brakingRatioRateMaximum]*elapsed;
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
        if fullCollisionRows || ~encounterEnded
            departure=index;
            [g,j,tightening,escape]=localSafety(x,forecast(:,point),point,model,collisionGenerator);
            safetyNormals(:,point)=j(1,1:2).';
            model.collisionEscapeNormals(:,point)=escape;
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
        point=2*index;
        if fullCollisionRows || ~encounterEnded
            departure=index;
            [g,j,tightening,escape]=localSafety(middle,forecast(:,point),point,model,middleCollisionGenerator);
            safetyNormals(:,point)=j(1,1:2).';
            model.collisionEscapeNormals(:,point)=escape;
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
        [r,bound]=localFrontDomain(x,input,cfg,generator);
        blockCount=blockCount+1;blockRows{blockCount}=r;blockColumns{blockCount}=[nodeColumns;iu(:,index)];blockBounds{blockCount}=bound;
        [r,bound]=localFrontDomain(nextReference,input,cfg,nextGenerator);
        blockCount=blockCount+1;blockRows{blockCount}=r;blockColumns{blockCount}=[ix(:,index+1);iu(:,index)];blockBounds{blockCount}=bound;
        [r,bound]=localFrontDomain(middle,input,cfg,middleGenerator);
        mapped=r(:,1:6)*middleMap;mapped(:,7:8)=mapped(:,7:8)+r(:,7:8);
        blockCount=blockCount+1;blockRows{blockCount}=mapped;blockColumns{blockCount}=middleColumns;blockBounds{blockCount}=bound;
        jx=ix(:,index+1);
        lower(jx(4:6))=max(lower(jx(4:6)),physicalLower-nextReference(4:6)+nextRadius);
        upper(jx(4:6))=min(upper(jx(4:6)),physicalUpper-nextReference(4:6)-nextRadius);
        % The shared observer operating cone is distinct from tire saturation.
        % The measured node is fixed and is not constrained.
        % The sideslip cone applies above model.brakeHoldSpeed; at rest the
        % sideslip is undefined.
        if nextReference(4)>cfg.model.brakeHoldSpeed
            blockCount=blockCount+1;blockRows{blockCount}=side;blockColumns{blockCount}=jx;
            blockBounds{blockCount}=-side*nextReference-sum(abs(side*nextGenerator),2);
        end
        generator=nextGenerator;holdCenter=middleCenter;
    end
    collisionGenerator=localRelativeGenerator(anchor.states(:,end),generator,model,holdCenter);
    uncertaintyPrediction.egoStateRadius(:,end)=sum(abs(generator),2);
    uncertaintyPrediction.collisionStateRadius(:,end)=sum(abs(collisionGenerator),2);
    model.uncertaintyPrediction=uncertaintyPrediction;
    model.sampledStates=zeros(6,2*count+1);
    model.sampledStates(:,1:2:end)=anchor.states;model.sampledStates(:,2:2:end)=middles;
    % Terminal node: the collision rows of every node inside the range, the
    % road, and the terminal set's conditions linearized at the anchor
    % endpoint y (terminalSafeSet.rows): collision rows at the continuation's
    % nodes and midpoints before the target's exit and at the parked pose
    % against the target's closest track pose, and road rows along the
    % continuation, all on dx_N. The anchor endpoint meets them when it is in
    % the set, so the zero correction stays feasible.
    y=anchor.states(:,end);endColumns=ix(:,end);
    time=times(end);point=2*count+1;
    if fullCollisionRows || ~encounterEnded
        departure=count+1;
        [g,j,tightening,escape]=localSafety(y,forecast(:,point),point,model,collisionGenerator);
        safetyNormals(:,point)=j(1,1:2).';
        model.collisionEscapeNormals(:,point)=escape;
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
    context.encounterEnded=encounterEnded;
    % The appended terminal command starts from the final optimized input,
    % not from this frame's measured previous input.
    lastBrake=iu(2,end);entry=context.brakingRatio-anchor.inputs(2,end);
    entryChange=context.brakingChangeMaximum;
    if durations(end)~=h,entryChange=entryChange*durations(end)/h;end
    lower(lastBrake)=max(lower(lastBrake),entry-entryChange);
    upper(lastBrake)=min(upper(lastBrake),entry+entryChange);
    if isfinite(entryChange)
        % Keep terminal input slew explicit, including its duration derivative.
        blockCount=blockCount+1;blockRows{blockCount}=[1;-1];blockColumns{blockCount}=lastBrake;
        blockBounds{blockCount}=[entry+entryChange;-entry+entryChange];
    end
    localCheckDeadline(model);
    [terminalRows,terminalBounds,context,terminalInfo]=terminalSafeSet.rows(context,y,time/h);
    localCheckDeadline(model);
    model.terminalContext=context;model.encounterExitRecord=exitRecord;
    if ~isempty(terminalRows)
        blockCount=blockCount+1;blockRows{blockCount}=terminalRows;blockColumns{blockCount}=endColumns;blockBounds{blockCount}=terminalBounds;
    end
    % Assemble after intersecting physical and numerical locality bounds.
    lower(iu)=max(lower(iu),physicalInputLower-anchor.inputs);
    upper(iu)=min(upper(iu),physicalInputUpper-anchor.inputs);
    physicalLowerBounds=lower;physicalUpperBounds=upper;
    trustRadii=Inf(8,count);trustRatio=zeros(1,count);adjusted=false;
    if cfg.rti.trustRegion.enabled
        for k=1:count
            stageCfg=cfg;stageCfg.controller.sampleTime=durations(k);
            [radius,trustInfo]=nonlinearBicycleModel.trustRegion( ...
                anchor.states(:,k),anchor.inputs(:,k),stageCfg,stages(k).next,stages(k).a,stages(k).b);
            columns=[ix(:,k);iu(:,k)];
            lower(columns)=max(lower(columns),-radius);
            upper(columns)=min(upper(columns),radius);
            trustRadii(:,k)=radius;trustRatio(k)=trustInfo.sampledErrorRatio;
        end
        % A measured initial state and shifted affine decisions need not be
        % dynamically consistent. Detect an empty dynamics/box intersection
        % before asking PCBF slack (which cannot relax dynamics) to fix it.
        if ~localDynamicsBoxCompatible(stages,reshape(rhs,6,[]),ix,iu,lower,upper,cfg.solver.constraintTolerance)
            % Particular solution of the affine equalities with unchanged
            % inputs. This is a sensitivity propagation, not a nonlinear
            % rollout, extra SQP solve, or replacement of the shifted plan.
            correction=reshape(rhs,6,[]);
            for k=1:count
                correction(:,k+1)=correction(:,k+1)+stages(k).a*correction(:,k);
            end
            lastAdjusted=0;
            for repair=1:count
                [compatible,k]=localDynamicsBoxCompatible(stages,reshape(rhs,6,[]),ix,iu,lower,upper,cfg.solver.constraintTolerance);
                % Enlarging an unrelated later node is unnecessary and can
                % demand a poorly conditioned open-loop correction there.
                % Only repair the next interval conflict. Shrinking that
                % stage's input room affects subsequent transitions only.
                if compatible || k<=lastAdjusted || k>count,break;end
                required=abs(correction(:,k));
                if ~any(required>trustRadii(1:6,k)),break;end
                stageCfg=cfg;stageCfg.controller.sampleTime=durations(k);
                [radius,trustInfo]=nonlinearBicycleModel.trustRegion(anchor.states(:,k),anchor.inputs(:,k), ...
                    stageCfg,stages(k).next,stages(k).a,stages(k).b,required);
                columns=[ix(:,k);iu(:,k)];
                % Keep all physical bounds. The reallocated numerical box
                % still has to pass the original nonlinear error budget.
                lower(columns)=max(physicalLowerBounds(columns),-radius);
                upper(columns)=min(physicalUpperBounds(columns),radius);
                trustRadii(:,k)=radius;trustRatio(k)=trustInfo.sampledErrorRatio;adjusted=true;
                lastAdjusted=k;
            end
        end
    end
    model.trustWarmStartAdjusted=adjusted;
    reachLower=lower;reachUpper=upper;
    if isfield(model,'durationProbe') && model.durationProbe
        % Keep redundant rows while differentiating: pruning at a bound
        % crossing would change the finite-difference row identities.
        reachLower(:)=-Inf;reachUpper(:)=Inf;
    end
    [inequality,bounds]=localAssemble(blockRows,blockColumns,blockBounds,blockCount,nv,reachLower,reachUpper);
    objective=zeros(nv,1);objective(is)=1;
    problem=struct('a',inequality,'b',bounds, ...
        'equal',equal,'rhs',rhs,'lower',lower,'upper',upper,'cones',cones, ...
        'primaryConeCount',numel(cones), ...
        'stages',stages, ...
        'safetyNormals',safetyNormals, ...
        'stateIndices',ix,'inputIndices',iu,'slackIndices',is,'clfIndex',ic, ...
        'safetyObjective',objective,'primaryLowerBound',primaryLowerBound, ...
        'encounterExit',departure, ...
        'maximumCollisionTighteningMeters',maximumTightening, ...
        'terminalRowCount',size(terminalRows,1),'terminalAnchorValue',terminalInfo.value, ...
        'terminalBounds',terminalBounds, ...
        'terminalAnchorMargin',terminalInfo.margin,'terminalAnchorExitSeconds',terminalInfo.exitSeconds, ...
        'terminalAnchorReason',terminalInfo.reason);
    model.robustnessRelaxation=robustRelaxation;
    problem.physicalInputLower=physicalInputLower;problem.physicalInputUpper=physicalInputUpper;
    problem.trustRadii=trustRadii;problem.trustSampledErrorRatio=trustRatio;
    problem.trustWarmStartAdjusted=adjusted;
    problem.inputIncrementLower=physicalInputLower-anchor.inputs;problem.inputIncrementUpper=physicalInputUpper-anchor.inputs;
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

function [values,jacobian,tightening,escape]=localSafety(x,q,point,model,generator)
    % Fixed-dual collision rows at a pose against the target's forecast q,
    % on the pose (jacobian, 4x3), tightened by the enclosures.
    cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    dual=predictiveSafetyGeometry.supportLinearization(x(1:3),shape,q(1:3),q(8:11));
    escape=model.collisionEscapeNormals(:,point);
    if ~model.retainCollisionSide,escape=zeros(2,1);end
    if norm(escape)>0
        projection=laneGeometry.project(x(1:2),model.lane);
        forward=[cos(projection.heading);sin(projection.heading)];
        tube=struct('positionRadius',model.targetTube.positionRadius(point),'yawRadius',model.targetTube.yawRadius(point));
        reserve=sum(vecnorm(generator(1:2,:)))+tube.positionRadius ...
            +2*norm(shape(1:2))*sin(min(pi,sum(abs(generator(3,:))))/2) ...
            +2*norm(q(8:9))*sin(tube.yawRadius/2);
        passed=forward.'*(x(1:2)-q(1:2))>norm(shape(1:2))+norm(q(8:9)) ...
            +norm(shape(3:4))+norm(q(10:11))+reserve;
        if passed,escape=zeros(2,1);end
    end
    active=norm(escape)>0 || dual.distance<=1e-8;
    if active
        dual=localEscapeDual(x,q,model,dual);
        escape=dual.normal;
    end
    if isfield(model,'fixedCollisionNormals') || isfield(model,'probeNormals')
        if isfield(model,'fixedCollisionNormals'),normal=model.fixedCollisionNormals(:,point);
        else,normal=model.probeNormals(:,point);end
        rotation=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];body=rotation.'*normal;
        [dual.value,dual.jacobian]=predictiveSafetyGeometry.fixedDualRows( ...
            x(1:3),shape,q(1:3),q(8:11),[max(body,0);max(-body,0)]);
        dual.normal=normal;
    end
    if isfield(model,'durationProbe') && model.durationProbe,escape=dual.normal;end
    values=dual.value;
    jacobian=dual.jacobian;
    % Directional position support retains correlation across the affine
    % ego dynamics. Target yaw uses the exact support over its entire yaw
    % interval, in the SAME fixed dual normal. The ego chord is retained:
    % it remains uniform while the optimizer changes the nominal ego yaw.
    normal=jacobian(:,1:2);normalNorm=vecnorm(normal,2,2);
    tube=struct('positionRadius',model.targetTube.positionRadius(point),'yawRadius',model.targetTube.yawRadius(point));
    % All four vertex rows share one normal and target support.
    targetSupport=predictiveSafetyGeometry.targetSupportFromData(model.targetSupportData,point,normal(1,:));
    egoReach=norm(shape(1:2))+norm(shape(3:4));
    yawRadius=sum(abs(generator(3,:)));
    reserve=2*egoReach*sin(min(pi,yawRadius)/2);
    [~,targetRotation]=predictiveSafetyGeometry.rotationSupport(q(8:11),q(3),tube.yawRadius,normal(1,:));
    tightening=sum(abs(normal*generator(1:2,:)),2)+targetSupport+normalNorm*reserve+targetRotation;
    values=values-tightening;
end

function dual=localEscapeDual(x,q,model,nearest)
    shape=[model.cfg.vehicle.length/2;model.cfg.vehicle.width/2;model.cfg.vehicle.rectangleOffset];
    projection=laneGeometry.project(x(1:2),model.lane);
    forward=predictiveSafetyGeometry.direction(projection.heading);
    side=model.collisionSide*[-forward(2);forward(1)];
    if nearest.distance>1e-8 && nearest.normal.'*side>.01
        dual=nearest;return;
    end
    egoRotation=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
    targetRotation=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];
    centers=x(1:2)+egoRotation*shape(3:4)-q(1:2)-targetRotation*q(10:11);
    longitudinal=forward.'*centers;orientation=1;
    if longitudinal<0,orientation=-1;end
    first=orientation*forward;second=side;
    rotation=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];
    body=rotation.'*[first,second];lambda=[max(body,0);max(-body,0)];
    [v1,~]=predictiveSafetyGeometry.fixedDualRows(x(1:3),shape,q(1:3),q(8:11),lambda(:,1));
    [v2,~]=predictiveSafetyGeometry.fixedDualRows(x(1:3),shape,q(1:3),q(8:11),lambda(:,2));
    gap=[min(v1);min(v2)];temperature=min(shape(2),q(9))/2;
    logWeight=(gap-max(gap))/temperature;
    sideSupport=second.'*centers-min(v2);
    if longitudinal==0,logWeight(1)=-Inf;
    else,logWeight(1)=logWeight(1)+log(abs(longitudinal)/(abs(longitudinal)+sideSupport));end
    weight=exp(logWeight-max(logWeight));weight=weight/sum(weight);
    % A small lateral component gives an intersecting straight reference an
    % escape derivative before pure lateral separation is physically possible.
    % Once separated, the exact closest-feature normal can turn diagonally.
    normal=weight(1)*first+weight(2)*second;normal=normal/norm(normal);
    body=rotation.'*normal;lambda=[max(body,0);max(-body,0)];
    [value,jacobian]=predictiveSafetyGeometry.fixedDualRows(x(1:3),shape,q(1:3),q(8:11),lambda);
    dual=struct('value',value,'jacobian',jacobian,'normal',normal);
end

function beyond=localBeyond(x,forecast,point,generator,tube,cfg,movement)
    % The target's reference point beyond the encounter range at record
    % point 'point' (no target: beyond), with the enclosures as reserve.
    if nargin<7,movement=0;end
    beyond=isempty(forecast);
    if ~beyond
        reserve=sum(vecnorm(generator(1:2,:)))+tube.positionRadius(point)+movement;
        beyond=norm(x(1:2)-forecast(1:2,point))-reserve>cfg.collision.encounterRangeMeters;
    end
end

function [rows,bounds]=localFrontDomain(y,input,cfg,generator)
    w=y(5)+cfg.vehicle.lf*y(6);denominator=y(4)^2+w^2;
    gradient=zeros(1,6);
    if denominator>1e-12,gradient(4:6)=[-w,y(4),cfg.vehicle.lf*y(4)]/denominator;end
    angle=atan2(w,y(4))-input(1);row=[gradient,-1,0];
    limit=pi/2-10*cfg.solver.constraintTolerance;
    reserve=[0;0];radius=sum(abs(generator),2);
    if y(4)>radius(4)
        wRadius=sum(abs(generator(5,:)+cfg.vehicle.lf*generator(6,:)));
        corners=atan2(w+[-wRadius,-wRadius,wRadius,wRadius], ...
            y(4)+[-radius(4),radius(4),-radius(4),radius(4)]);
        center=atan2(w,y(4));reserve=[max(corners)-center;center-min(corners)];
    end
    rows=[row;-row];bounds=[limit-angle;limit+angle]-reserve;
end

function stages=localLinearization(anchor,model)
    count=size(anchor.inputs,2);stages=repmat(struct('middle',[],'am',[],'bm',[], ...
        'next',[],'a',[],'b',[]),1,count);
    durations=localTimeGrid(model,count);stageCfg=model.cfg;
    for index=1:count
        if mod(index,4)==1,localCheckDeadline(model);end
        if stageCfg.controller.sampleTime~=durations(index),stageCfg.controller.sampleTime=durations(index);end
        [stages(index).middle,stages(index).am,stages(index).bm, ...
            stages(index).next,stages(index).a,stages(index).b]= ...
            nonlinearBicycleModel.hold(anchor.states(:,index),anchor.inputs(:,index),stageCfg);
    end
end

function [durations,times]=localTimeGrid(model,count)
    h=model.cfg.controller.sampleTime;
    durations=repmat(h,1,count);times=(0:2*count)*h/2;
    if ~isfield(model,'predictionDurations'),return;end
    durations=reshape(model.predictionDurations,1,[]);
    assert(numel(durations)==count && all(isfinite(durations) & durations>0), ...
        'collisionAvoidanceController:invalidPredictionGrid','Prediction durations must be positive, one per input.');
    if all(durations==h),return;end
    nodes=[0,cumsum(durations)];times=zeros(1,2*count+1);
    times(1:2:end)=nodes;times(2:2:end)=nodes(1:end-1)+durations/2;
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


function problem=localSelectObjective(problem,anchor,model)
    nv=numel(problem.safetyObjective);iu=problem.inputIndices;ic=problem.clfIndex;count=size(iu,2);
    enabled=isempty(model.target);
    value=nonlinearBicycleModel.nominalValue(model.initialState,model.lane,model.nominalReference);
    scale=1;required=NaN;map=sparse(1,nv);offset=NaN;initialSlack=NaN;slackDuration=model.cfg.controller.sampleTime;
    problem.performanceObjective=zeros(nv,1);
    problem.mode="pcbf";problem.optimizationObjective="pcbfSlack";functionName="none";
    if enabled
        % Integrate the SAME continuous CLF over one held-input interval:
        % V(x+) <= exp(-alpha*h)*V(x) + tau*s, tau=(1-exp(-alpha*h))/alpha.
        % RTI uses its tangent at the first predicted state, keeping a QP.
        % This tangent is an approximation, not a nonlinear descent certificate.
        alpha=model.nominalReference.decayRate;h=model.cfg.controller.sampleTime;
        slackDuration=-expm1(-alpha*h)/alpha;
        [nextError,nextJacobian]=nonlinearBicycleModel.errorLinearization( ...
            anchor.states(:,2),model.lane,model.nominalReference);
        nextValue=sum((model.nominalReference.factor*nextError).^2);
        scale=max(1,value);required=alpha*value;
        map(problem.stateIndices(:,2))=2*nextError.'*model.nominalReference.matrix*nextJacobian/(slackDuration*scale);
        offset=(nextValue-value)/(slackDuration*scale);
        row=map;row(ic)=-1;
        problem.a=[problem.a;row];problem.b=[problem.b;-required/scale-offset];
        problem.clfRowIndex=size(problem.a,1);
        problem.performanceObjective(ic)=slackDuration;
        problem.mode="clf";problem.optimizationObjective="integratedClfSlackAndTracking";
        % No active collision problem: remove the unused safety objective
        % and fix its reserved slack slots rather than running a safety LP.
        problem.safetyObjective(:)=0;problem.upper(problem.slackIndices)=0;
        functionName="quadraticTransverseError";initialSlack=max(0,offset*scale+required);
    else
        % Reserve the existing index for probe compatibility, but fix it to
        % zero and omit both the CLF inequality and its objective coefficient.
        problem.lower(ic)=0;problem.upper(ic)=0;
    end
    % Unit-scaled full-state/input proximity to the affine reference, not to
    % the cruise path. Equal x/y units make position cost rotation invariant.
    % A horizon mean prevents changing N from changing the total weight.
    % These units impose no bounds, and the weight does not vanish at cruise.
    stateUnits=[repmat(model.cfg.clf.lateralPositionErrorScale,2,1); ...
        model.cfg.clf.headingErrorScale;model.cfg.clf.speedErrorScale; ...
        model.cfg.clf.lateralVelocityErrorScale;model.cfg.clf.yawRateErrorScale];
    inputUnits=[model.cfg.clf.certificationSteeringRadians;model.cfg.clf.certificationBrakingRatio];
    ix=problem.stateIndices(:,2:end);
    stateScale=repmat(1./(stateUnits*sqrt(count)),count,1);
    inputScale=repmat(1./(inputUnits*sqrt(count)),count,1);
    problem.referenceStateMap=sparse(1:numel(ix),ix(:),stateScale,numel(ix),nv);
    problem.referenceInputMap=sparse(1:numel(iu),iu(:),inputScale,numel(iu),nv);
    problem.referenceDeviationWeight=model.cfg.rti.referenceDeviationWeight;
    problem.referencePenaltyWeight=problem.referenceDeviationWeight/(2*scale);
    problem.quadratic=2*problem.referencePenaltyWeight ...
        *(problem.referenceStateMap.'*problem.referenceStateMap+problem.referenceInputMap.'*problem.referenceInputMap);
    % A persistent path objective accompanies every RTI solve. Avoidance
    % attracts transverse state and speed without a cruise CLF condition
    % during avoidance. The first PCBF optimization remains unchanged.
    components=5;
    units=stateUnits(2:end);trackingValues=zeros(components,6,count);
    trackingOffset=zeros(components*count,1);
    for k=1:count
        [error,jacobian]=nonlinearBicycleModel.errorLinearization( ...
            anchor.states(:,k+1),model.lane,model.nominalReference);
        rows=(k-1)*components+(1:components);
        trackingOffset(rows)=error(1:components)./(units(1:components)*sqrt(count));
        trackingValues(:,:,k)=jacobian(1:components,:)./(units(1:components)*sqrt(count));
    end
    trackingRows=repmat(reshape(1:components*count,components,1,count),1,6,1);
    trackingColumns=repmat(reshape(ix,1,6,count),components,1,1);
    trackingMap=sparse(trackingRows(:),trackingColumns(:),trackingValues(:),components*count,nv);
    problem.nominalTrackingWeight=model.cfg.rti.nominalTrackingWeight;
    problem.nominalTrackingMap=trackingMap;problem.nominalTrackingOffset=trackingOffset;
    weight=problem.nominalTrackingWeight/scale;
    problem.quadratic=problem.quadratic+weight*(trackingMap.'*trackingMap);
    problem.performanceObjective=problem.performanceObjective+weight*trackingMap.'*trackingOffset;
    % Penalize actual neighboring controls, including the already applied
    % input. This is a soft temporal preference, not a rate constraint.
    % Use a sum: extending the horizon must not cheapen the first jump.
    variationRows=reshape(1:2*count,2,count);
    previousRows=variationRows(:,2:end);previousColumns=iu(:,1:end-1);
    variationMap=sparse([variationRows(:);previousRows(:)], ...
        [iu(:);previousColumns(:)], ...
        [repmat(1./inputUnits,count,1);repmat(-1./inputUnits,count-1,1)],2*count,nv);
    variationOffset=reshape(diff([model.previousInput,anchor.inputs],1,2)./inputUnits,[],1);
    problem.inputVariationWeight=model.cfg.rti.inputVariationWeight;
    problem.inputVariationMap=variationMap;problem.inputVariationOffset=variationOffset;
    weight=problem.inputVariationWeight/scale;
    problem.quadratic=problem.quadratic+weight*(variationMap.'*variationMap);
    problem.performanceObjective=problem.performanceObjective+weight*variationMap.'*variationOffset;
    problem.clfSlackDuration=slackDuration;
    problem.clfScale=scale;problem.clfDerivativeMap=map;problem.clfDerivativeOffset=offset;
    problem.clfDerivativeLimit=-required;
    problem.clf=struct('enabled',enabled,'function',functionName,'requiredDecreaseRate',required);
    problem.initialClfValue=value;problem.initialClfSlack=initialSlack;
end

function tube=localTargetTube(model,time,holdStart)
    % Target enclosure of the hold that starts at holdStart: the observer
    % set restarted from the target's predicted state there, for the time
    % elapsed in the hold. This is not propagation of the current family
    % over the complete horizon; future measurement updates are assumed.
    tube=struct('positionRadius',zeros(size(time)),'yawRadius',zeros(size(time)), ...
        'courseRadius',zeros(size(time)),'speedRadius',zeros(size(time)));
    if isempty(model.target) || isempty(model.uncertainty.target)
        return;
    end
    % One predicted start state per entry; the tube formulas are elementwise.
    starts=localTargetAt(model,holdStart);
    tube=predictiveSafetyGeometry.targetErrorTube(starts,model.uncertainty.target,time-holdStart);
end

function solution=localTerminalMetrics(solution,model)
    % Descriptive metrics of the local affine endpoint, not a certificate.
    cfg=model.cfg;y=solution.states(:,end);count=size(solution.inputs,2);
    solution.terminalValue=nonlinearBicycleModel.nominalValue(y,model.lane,model.nominalReference);
    solution.terminalMarginMeters=NaN;solution.terminalRoadMarginMeters=NaN;
    solution.terminalStopSeconds=hypot(y(4),y(5))/cfg.terminal.brakingDecelerationMetersPerSecondSquared;
    solution.terminalExitSeconds=NaN;solution.terminalEnd="affineEndpoint";
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

function [result,search,model]=localProbe(request)
    % Research-only reuse of the existing formulation and conic solver.
    % The online entry point never supplies this request or a variable grid.
    model=request.model;search=struct();timer=tic;
    if isfield(request,'timer'),timer=request.timer;end
    switch string(request.operation)
        case "reference"
            model.referenceProbeOnly=true;
            if isfield(request,'minimumHorizonSteps'),model.probeMinimumHorizonSteps=request.minimumHorizonSteps;end
            if isfield(request,'reference'),model.probeReference=request.reference;end
            [~,search,model]=solvePredictiveControl(model,request.previousState,timer);
            model=rmfield(model,'referenceProbeOnly');
            if isfield(model,'probeReference'),model=rmfield(model,'probeReference');end
            if isfield(model,'probeMinimumHorizonSteps'),model=rmfield(model,'probeMinimumHorizonSteps');end
            assert(isfield(model,'linearization'),'collisionAvoidanceController:noProbeReference', ...
                'No rollable reference was constructed.');
            result=model.linearization;
        case "formulate"
            model.solveTimer=timer;model.solveTimeLimitSeconds=model.cfg.solver.timeLimitSeconds;
            [result,model]=localFormulate(request.anchor,model);
            result.inheritedAnchor=request.inherited;
            result=localSelectObjective(result,request.anchor,model);
        case "solve"
            model.solveTimer=timer;model.solveTimeLimitSeconds=model.cfg.solver.timeLimitSeconds;
            problem=request.problem;anchor=request.anchor;
            [result,search,model]=localSolve(anchor,model,"durationExperiment",timer,problem);
        case "check"
            model.solveTimer=timer;model.solveTimeLimitSeconds=model.cfg.solver.timeLimitSeconds;
            [result,search,model]=localNonlinearCheck(request.anchor,model,request.problem.safetyNormals);
        otherwise
            error('collisionAvoidanceController:invalidProbeOperation','Unknown research operation.');
    end
end

function [feasible,blockedStage]=localDynamicsBoxCompatible(stages,defects,ix,iu,lower,upper,tolerance)
    % Forward interval propagation gives a necessary condition only. A
    % nonempty interval does not certify joint feasibility or other rows.
    % The initial correction is fixed by the first equality, not its box.
    low=defects(:,1);high=low;feasible=true;blockedStage=0;
    if any(low<lower(ix(:,1))-tolerance | high>upper(ix(:,1))+tolerance)
        feasible=false;return;
    end
    for k=1:numel(stages)
        if any(lower(iu(:,k))>upper(iu(:,k))+tolerance),feasible=false;return;end
        a=stages(k).a;b=stages(k).b;
        reachLow=defects(:,k+1)+max(a,0)*low+min(a,0)*high+max(b,0)*lower(iu(:,k))+min(b,0)*upper(iu(:,k));
        reachHigh=defects(:,k+1)+max(a,0)*high+min(a,0)*low+max(b,0)*upper(iu(:,k))+min(b,0)*lower(iu(:,k));
        low=max(reachLow,lower(ix(:,k+1)));high=min(reachHigh,upper(ix(:,k+1)));
        if any(low>high+tolerance),feasible=false;blockedStage=k+1;return;end
    end
end
