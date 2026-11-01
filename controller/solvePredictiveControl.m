function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl SQP-RTI of the nonlinear predictive barrier problem.
% Both numerical backends use continuous separating-plane witnesses and the
% same nonlinear vehicle, terminal function, and primary safety objective.
% Theory executes any primary minimizer's first command. The optional second
% objective is implementation performance refinement, not a theorem premise.
% The online default performs one local model update. Iterated nonlinear
% shooting and retained nonlinear certificates are explicit research modes.
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
% controller.retainedTube selects the certificate-based formulation
% during encounters; see CONTROLLER_THEORY.md, implemented policy-box proof.
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
    hasRetainedState=isstruct(previousState) && isfield(previousState,'retainedCertificate');
    if cfg.controller.retainedTube && isfield(model,'target') && ~isempty(model.target) ...
            && (isfield(model.uncertainty,'jointPrediction') || hasRetainedState) ...
            && ~(isfield(model,'referenceProbeOnly') && model.referenceProbeOnly)
        [solution,search,model]=localRetainedSolve(model,previousState,timer);return;
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
        if string(cfg.solver.method)=="nonlinear"
            [solution,search,model]=localNonlinearSolve(anchor,model,source,timer,search.initializationSeconds);
            return;
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
    if inherited && isfield(previous,'collisionWitnesses') && ~isempty(previous.collisionWitnesses)
        anchor.collisionWitnesses=nan(2,2*count+1);
        oldWitness=previous.collisionWitnesses(:,3:end);
        kept=min(size(oldWitness,2),2*count+1);
        anchor.collisionWitnesses(:,1:kept)=oldWitness(:,1:kept);
    end
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
    guide=predictiveSafetyGeometry.avoidanceGuide(model,count);
    context=terminalSafeSet.context(model);
    context.solveTimer=model.solveTimer;context.solveTimeLimitSeconds=model.solveTimeLimitSeconds;
    planner=localPlannerMetadata();planner.used=true;planner.candidateCount=guide.candidateCount;
    planner.maximumDeviationMeters=guide.maximumDeviation;
    planner.candidates=guide.candidates;
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
        'candidateCount',0,'algorithm',"predictiveVectorField", ...
        'candidates',struct([]), ...
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
        'potentialFieldRestarted',false,'horizonSteps',0, ...
        'linearizationCount',0,'nominalRestarted',false,'restartCount',0);
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
    % One RTI model. The second avoidance solve is implementation refinement;
    % the theoretical PCBF control law needs only a primary minimizer.
    % Avoidance orders slack before quadratic performance;
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
    scaled=abs(changes)./problem.trustRadii;scaled(changes==0 & problem.trustRadii==0)=0;
    solution.trustMaximumScaledCorrection=max(scaled,[],'all');
    solution.trustFirstInputRadius=problem.trustRadii(7:8,1);
    solution.trustSampledErrorRatio=max(problem.trustSampledErrorRatio);
    solution.objectiveValue=solution.safety;
    if problem.clf.enabled,solution.objectiveValue=solution.performanceCost+problem.clfSlackDuration*solution.clfSlack;end
    if isfield(problem,'durationIndex')
        solution.durationChangeSeconds=point(problem.durationIndex);
        solution.states=solution.states+problem.durationStateDerivative*solution.durationChangeSeconds;
    end
    solution.collisionWitnesses=zeros(2,0);
    if isfield(problem,'witnessIndices') && ~isempty(problem.witnessIndices)
        witnessStep=reshape(point(problem.witnessIndices),size(problem.witnessIndices));
        if problem.retractWitnesses
            solution.collisionWitnesses=predictiveSafetyGeometry.retractSeparationWitness(problem.collisionWitnesses,witnessStep);
        else
            solution.collisionWitnesses=problem.collisionWitnesses+witnessStep;
        end
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
    cfg=model.cfg;count=size(anchor.inputs,2);
    retainedProposal=isfield(model,'retainedProposal') && model.retainedProposal;
    % Every collision stage has a PCBF deficit, including the endpoint.
    % Horizon sizing must not create an unrelated hard collision tail.
    % Input, model-domain and terminal requirements remain hard.
    if ~isfield(model,'collisionEscapeNormals') || size(model.collisionEscapeNormals,2)~=2*count+1
        model.collisionEscapeNormals=zeros(2,2*count+1);
    end
    robustRelaxation=any(model.uncertainty.egoGenerator(:));
    if ~isempty(model.uncertainty.target)
        robustRelaxation=robustRelaxation || any(structfun(@(value)any(value(:)>0),model.uncertainty.target));
    end
    % Nominal and observer-tightened collisions share the stage slack. A
    % positive optimum is an allowed safety deficit, not an avoidance claim.
    slackCount=count+1;
    ix=reshape(1:6*(count+1),6,[]);iu=reshape(ix(end)+(1:2*count),2,[]);
    is=iu(end)+(1:slackCount);ic=is(end)+1;nv=ic;
    iw=zeros(2,0);witnesses=zeros(2,0);collisionData=cell(1,2*count+1);
    if ~isempty(model.target)
        iw=reshape(ic+(1:2*(2*count+1)),2,[]);nv=iw(end);
        witnesses=nan(size(iw));
    end
    model.referenceWitnesses=witnesses;
    if isfield(anchor,'collisionWitnesses') && isequal(size(anchor.collisionWitnesses),size(witnesses))
        model.referenceWitnesses=anchor.collisionWitnesses;
    end
    % Every inequality row is a small dense block on the few variables it
    % involves (a node's state, or a hold's start state and input, and its
    % slack), in the order of the rows; the sparse matrices are built once
    % from these blocks (localAssemble). Building them row by row in sparse
    % form cost time quadratic in the plan length.
    blockRows=cell(1,14*count+10);blockColumns=blockRows;blockBounds=blockRows;blockCount=0;
    rotationBlocks=[];rotationMaps={};
    cones=struct('A',{},'b',{},'D',{},'gamma',{},'size',{});
    lower=-Inf(nv,1);upper=Inf(nv,1);
    lower([is,ic])=0;
    % One complete angular chart covers every unit normal, including negative
    % signed clearance. Zero normal is impossible, even with zero clearance.
    lower(iw(1,:))=-pi;upper(iw(1,:))=pi;
    % Position translations stay unrestricted. Numerical trust bounds on the
    % nonlinear coordinates are shared by the primary and secondary solves.
    [inputLower,inputUpper]=nonlinearBicycleModel.inputBounds(cfg);
    rate=[Inf;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;finite=isfinite(rate);
    [physicalLower,physicalUpper]=localStateLimits(cfg);
    primaryLowerBound=0;departure=0;encounterEnded=isempty(model.target);exitRecord=NaN;
    fullCollisionRows=isfield(model,'durationProbe') && model.durationProbe && ~isempty(model.target);
    % Legacy per-hold affine uncertainty. The first hold uses the published current
    % generator unchanged. Subsequent holds re-evaluate its GNSS/yaw geometry
    % at their own nominal velocity, retaining sensor and observer-residual
    % budgets. Publications without these factors retain the old generator.
    % Future yaw/residual budgets remain conditional on the current ones;
    % this scheduling is not a lifted observer/measurement prediction or a
    % nonlinear reachable-set certificate. No future innovation is presumed
    % to vanish. The target set retains its existing per-hold convention.
    initialGenerator=model.uncertainty.egoGenerator;generator=initialGenerator;
    holdCenter=model.initialState;h=cfg.controller.sampleTime;
    stages=localLinearization(anchor,model);
    physicalInputLower=repmat(inputLower,1,count);physicalInputUpper=repmat(inputUpper,1,count);
    maximumTightening=0;safetyNormals=zeros(2,2*count+1);
    [durations,times]=localTimeGrid(model,count);
    jointTube=[];enclosureDomain=[];
    if isfield(model,'jointPredictionTube'),model=rmfield(model,'jointPredictionTube');end
    jointRequested=cfg.controller.jointAffinePrediction || cfg.controller.jointNonlinearPrediction;
    if jointRequested && isfield(model.uncertainty,'jointPrediction')
        options=struct();
        if cfg.controller.jointNonlinearPrediction
            enclosureDomain=localEnclosureDomain(anchor,model,stages,durations);
            options=struct('nonlinearEnclosure',true,'decisionLower',enclosureDomain.lower, ...
                'decisionUpper',enclosureDomain.upper,'stages',stages);
        end
        jointTube=nrmmJointPrediction('propagate',anchor,cfg,initialGenerator, ...
            model.uncertainty.jointPrediction,durations,options);
        physicalInputLower=physicalInputLower+jointTube.inputRadius;
        physicalInputUpper=physicalInputUpper-jointTube.inputRadius;
        model.jointPredictionTube=jointTube;
    elseif cfg.controller.jointNonlinearPrediction
        error('collisionAvoidanceController:missingJointPrediction', ...
            'Joint prediction requires a synchronized observer snapshot, not only current error radii.');
    end
    % Record time j*h/2 lies in hold ceil(j/2), which starts at (ceil(j/2)-1)*h.
    holdStarts=times(2*max(0,ceil((0:2*count)/2)-1)+1);
    % A new future measurement changes the estimate, not the physical target
    % family. Without a target-relative policy retain its entire elapsed-time
    % spread instead of pretending each future observation resets that spread.
    if ~isempty(jointTube) || (isfield(model,'retainedProposal') && model.retainedProposal)
        holdStarts=zeros(size(times));
    end
    model.recordTimes=times;
    targetTube=localTargetTube(model,times,holdStarts);
    model.targetTube=targetTube;
    starts=localTargetAt(model,holdStarts);
    model.targetSupportData=predictiveSafetyGeometry.targetSupportData(starts,model.uncertainty.target,times-holdStarts);
    uncertaintyPrediction=struct('time',times,'target',targetTube, ...
        'egoStateRadius',zeros(6,2*count+1),'collisionStateRadius',zeros(6,2*count+1));
    uncertaintyPrediction.stateDependentVelocityGeometry=isfield(model.uncertainty,'velocityGeometry');
    uncertaintyPrediction.futureObserverDynamicsIncluded=false;
    uncertaintyPrediction.futurePrimitiveBudgetsAssumed=true;
    if ~isempty(jointTube)
        uncertaintyPrediction.futureObserverDynamicsIncluded=true;
        uncertaintyPrediction.futurePrimitiveBudgetsAssumed=false;
        uncertaintyPrediction.nonlinearCertificate=false;
        uncertaintyPrediction.egoJointRealArithmeticEnclosure=jointTube.realArithmeticEnclosure;
        uncertaintyPrediction.observerErrorRadius=jointTube.observerErrorRadius;
        uncertaintyPrediction.observerErrorTime=jointTube.observerErrorTime;
        uncertaintyPrediction.inputRadius=jointTube.inputRadius;
        uncertaintyPrediction.mixedStopBranches=jointTube.mixedStopBranches;
    end
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
        holdGenerator=initialGenerator;
        if index>1,holdGenerator=predictiveSafetyGeometry.egoErrorGenerator(x,model.uncertainty);end
        middleGenerator=am*holdGenerator;nextGenerator=stage.a*holdGenerator;
        middleCenter=model.initialState;if index>1,middleCenter=x;end
        if ~isempty(jointTube)
            middleGenerator=jointTube.generator{2*index};nextGenerator=jointTube.generator{2*index+1};
            middleCenter=model.initialState;
        end
        middleCollisionGenerator=localRelativeGenerator(middle,middleGenerator,model,middleCenter);
        uncertaintyPrediction.egoStateRadius(:,2*index-1:2*index)=[sum(abs(generator),2),sum(abs(middleGenerator),2)];
        uncertaintyPrediction.collisionStateRadius(:,2*index-1:2*index)=[sum(abs(collisionGenerator),2),sum(abs(middleCollisionGenerator),2)];
        if any(finite)
            elapsed=h;if index>1,elapsed=durations(index-1);end
            rate=[Inf;cfg.model.brakingRatioRateMaximum]*elapsed;
            previous=model.previousInput;rateRows=eye(2);rateColumns=iu(:,index);
            if index>1,previous=anchor.inputs(:,index-1);rateRows=[eye(2),-eye(2)];rateColumns=[iu(:,index);iu(:,index-1)];end
            difference=input-previous;
            reserve=zeros(2,1);
            if ~isempty(jointTube)
                reserve=jointTube.inputRadius(:,index);
                if index>1,reserve=reserve+jointTube.inputRadius(:,index-1);end
            end
            blockCount=blockCount+1;blockRows{blockCount}=[rateRows(finite,:);-rateRows(finite,:)];blockColumns{blockCount}=rateColumns;
            blockBounds{blockCount}=[rate(finite)-difference(finite)-reserve(finite);rate(finite)+difference(finite)-reserve(finite)];
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
            [g,j,tightening,escape,jw,witnesses(:,point),collisionData{point}]=localSafety(x,forecast(:,point),point,model,collisionGenerator);
            safetyNormals(:,point)=j(1,1:2).';
            model.collisionEscapeNormals(:,point)=escape;
            maximumTightening=max(maximumTightening,max(tightening));
            % A movable witness can improve this support gap; zero remains
            % the valid primary lower bound, independently of its initialization.
            r=[-j,zeros(8,3)];yawMap=[0,0,1,0,0,0];
            blockCount=blockCount+1;
            blockRows{blockCount}=[r,-jw,-[ones(4,1);zeros(4,1)]];blockColumns{blockCount}=[nodeColumns;iw(:,point);is(index)];
            blockBounds{blockCount}=g;
            rotationBlocks(end+1)=blockCount;rotationMaps{end+1}= ...
                [yawMap,zeros(1,size(blockRows{blockCount},2)-numel(yawMap))];
        end
        middleMap=[am,bm];middleColumns=[ix(:,index);iu(:,index)];
        if ~isempty(clearance)
            [r,bound]=localRoadBlock(road,count+1+index,middle,middleMap,middleGenerator,clearance);
            blockCount=blockCount+1;blockRows{blockCount}=r;blockColumns{blockCount}=middleColumns;blockBounds{blockCount}=bound;
        end
        point=2*index;
        if fullCollisionRows || ~encounterEnded
            departure=index;
            [g,j,tightening,escape,jw,witnesses(:,point),collisionData{point}]=localSafety(middle,forecast(:,point),point,model,middleCollisionGenerator);
            safetyNormals(:,point)=j(1,1:2).';
            model.collisionEscapeNormals(:,point)=escape;
            maximumTightening=max(maximumTightening,max(tightening));
            r=-j*middleMap(1:3,:);yawMap=middleMap(3,:);
            blockCount=blockCount+1;
            blockRows{blockCount}=[r,-jw,-[ones(4,1);zeros(4,1)]];blockColumns{blockCount}=[middleColumns;iw(:,point);is(index)];
            blockBounds{blockCount}=g;
            rotationBlocks(end+1)=blockCount;rotationMaps{end+1}= ...
                [yawMap,zeros(1,size(blockRows{blockCount},2)-numel(yawMap))];
        end
        middleRadius=sum(abs(middleGenerator(4:6,:)),2);nextRadius=sum(abs(nextGenerator(4:6,:)),2);
        middleLowerRadius=middleRadius;
        if stage.next(4)==0 && ~any(stage.a(4,:)) && ~any(stage.b(4,:)) ...
                && norm(am(4,:)-[0,0,0,.5,0,0],Inf)<1e-12 && ~any(bm(4,:))
            % The rest reset fixes the endpoint speed to zero; this mesh's
            % midpoint is half the nonnegative start speed. Intersect its
            % reachable set with the declared forward-motion domain instead
            % of assigning negative speed to a symmetric error box around
            % rest. Keep the nominal lower bound and every published error
            % radius; only this already-implied lower tightening is redundant.
            middleLowerRadius(1)=0;
        end
        blockCount=blockCount+1;blockRows{blockCount}=[middleMap(4:6,:);-middleMap(4:6,:)];blockColumns{blockCount}=middleColumns;
        blockBounds{blockCount}=[physicalUpper-middle(4:6)-middleRadius;middle(4:6)-physicalLower-middleLowerRadius];
        [r,bound]=localFrontDomain(x,input,cfg,generator);
        if ~isempty(jointTube),bound=bound-jointTube.inputRadius(1,index);end
        blockCount=blockCount+1;blockRows{blockCount}=r;blockColumns{blockCount}=[nodeColumns;iu(:,index)];blockBounds{blockCount}=bound;
        [r,bound]=localFrontDomain(nextReference,input,cfg,nextGenerator);
        if ~isempty(jointTube),bound=bound-jointTube.inputRadius(1,index);end
        blockCount=blockCount+1;blockRows{blockCount}=r;blockColumns{blockCount}=[ix(:,index+1);iu(:,index)];blockBounds{blockCount}=bound;
        [r,bound]=localFrontDomain(middle,input,cfg,middleGenerator);
        if ~isempty(jointTube),bound=bound-jointTube.inputRadius(1,index);end
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
    % Endpoint collision rows share the ordinary plane formulation. The
    % terminal row below is the tangent of the SAME signed continuation
    % function evaluated by shooting, with its active feature retained only
    % while taking the derivative. This is not a nonlinear safety certificate.
    y=anchor.states(:,end);endColumns=ix(:,end);
    time=times(end);point=2*count+1;
    if fullCollisionRows || ~encounterEnded
        departure=count+1;
        [g,j,tightening,escape,jw,witnesses(:,point),collisionData{point}]=localSafety(y,forecast(:,point),point,model,collisionGenerator);
        safetyNormals(:,point)=j(1,1:2).';
        model.collisionEscapeNormals(:,point)=escape;
        maximumTightening=max(maximumTightening,max(tightening));
        r=[-j,zeros(8,3)];yawMap=[0,0,1,0,0,0];
        blockCount=blockCount+1;
        blockRows{blockCount}=[r,-jw,-[ones(4,1);zeros(4,1)]];blockColumns{blockCount}=[endColumns;iw(:,point);is(count+1)];
        blockBounds{blockCount}=g;
        rotationBlocks(end+1)=blockCount;rotationMaps{end+1}= ...
            [yawMap,zeros(1,size(blockRows{blockCount},2)-numel(yawMap))];
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
    if retainedProposal
        % The joint policy's terminal set is checked after nonlinear set
        % propagation by localRetainedAssess. Do not constrain this optional
        % proposal by another, locally linearized braking continuation.
        terminalRows=zeros(0,6);terminalBounds=zeros(0,1);
        terminalInfo=struct('value',NaN,'margin',NaN,'exitSeconds',NaN, ...
            'reason',"deferredToWholeSetAdmission");
    else
        [terminalValue,terminalRows]=terminalSafeSet.nonlinearTangent(context,y,time/h);
        terminalBounds=-terminalValue;
        terminalInfo=struct('value',terminalValue,'margin',-terminalValue, ...
            'exitSeconds',NaN,'reason',"signedBrakingContinuation");
    end
    localCheckDeadline(model);
    model.terminalContext=context;model.encounterExitRecord=exitRecord;
    if ~isempty(terminalRows)
        blockCount=blockCount+1;blockRows{blockCount}=terminalRows;blockColumns{blockCount}=endColumns;blockBounds{blockCount}=terminalBounds;
    end
    % Assemble after intersecting physical and numerical locality bounds.
    lower(iu)=max(lower(iu),physicalInputLower-anchor.inputs);
    upper(iu)=min(upper(iu),physicalInputUpper-anchor.inputs);
    inactive=cellfun(@isempty,collisionData);
    if ~isempty(iw)
        lower(iw(:,inactive))=0;upper(iw(:,inactive))=0;witnesses(:,inactive)=0;
        if cfg.controller.jointNonlinearPrediction
            % The retained whole-domain rotation reserve certifies a fixed
            % normal only. This optional enclosure experiment freezes it.
            lower(iw(1,:))=0;upper(iw(1,:))=0;
        end
    end
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
    if ~isempty(enclosureDomain)
        % The nonlinear images were computed on this domain. Subsequent
        % locality/physical bounds may intersect it, but cannot enlarge it.
        for k=1:count
            columns=[ix(:,k);iu(:,k)];
            lower(columns)=max(lower(columns),enclosureDomain.lower(:,k));
            upper(columns)=min(upper(columns),enclosureDomain.upper(:,k));
        end
    end
    model.trustWarmStartAdjusted=adjusted;
    % Use the FINAL domain, including any affine warm-start consistency
    % repair. A later domain enlargement must never reuse an old yaw bound.
    vertices=cfg.vehicle.rectangleOffset+[cfg.vehicle.length;cfg.vehicle.width]/2 ...
        .*[-1,1,1,-1;-1,-1,1,1];
    vertexReach=[vecnorm(vertices).';zeros(4,1)];
    if ~cfg.controller.jointNonlinearPrediction,rotationBlocks=[];end
    for k=1:numel(rotationBlocks)
        block=rotationBlocks(k);map=rotationMaps{k};columns=blockColumns{block};
        coordinateRadius=max(abs(lower(columns)),abs(upper(columns)));
        active=map~=0;yawRadius=abs(map(active))*coordinateRadius(active);
        [blockRows{block},blockBounds{block}]=predictiveSafetyGeometry.conservativeRotationRows( ...
            blockRows{block},blockBounds{block},map,vertexReach,yawRadius);
    end
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
        'safetyNormals',safetyNormals,'witnessIndices',iw,'collisionWitnesses',witnesses, ...
        'collisionData',{collisionData}, ...
        'retractWitnesses',string(cfg.solver.method)=="rti", ...
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

function domain=localEnclosureDomain(anchor,model,stages,durations)
    % Finite outer domain for all affine decisions admitted by the QP. The
    % trust heuristic only chooses a box; it supplies NO remainder bound.
    cfg=model.cfg;count=numel(stages);domain.lower=zeros(8,count);domain.upper=zeros(8,count);
    [ul,uh]=nonlinearBicycleModel.inputBounds(cfg);
    reachableLow=model.initialState-anchor.states(:,1);reachableHigh=reachableLow;
    particular=reachableLow;
    for k=1:count
        stageCfg=cfg;stageCfg.controller.sampleTime=durations(k);
        radius=nonlinearBicycleModel.trustRegion(anchor.states(:,k),anchor.inputs(:,k), ...
            stageCfg,stages(k).next,stages(k).a,stages(k).b,abs(particular));
        low=[max(reachableLow,-radius(1:6));max(ul-anchor.inputs(:,k),-radius(7:8))];
        high=[min(reachableHigh,radius(1:6));min(uh-anchor.inputs(:,k),radius(7:8))];
        if any(low>high) || any(~isfinite([low;high]))
            error('collisionAvoidanceController:unenclosedDecisionDomain', ...
                'A finite nonempty decision domain is required for nonlinear outer images.');
        end
        domain.lower(:,k)=low;domain.upper(:,k)=high;
        map=[stages(k).a,stages(k).b];defect=stages(k).next-anchor.states(:,k+1);
        reachableLow=max(map,0)*low+min(map,0)*high+defect;
        reachableHigh=max(map,0)*high+min(map,0)*low+defect;
        particular=stages(k).a*particular+defect;
    end
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

function [values,jacobian,tightening,normal,witnessJacobian,witness,data]=localSafety(x,q,point,model,generator)
    % The same nonlinear plane rows feed both shooting and SQP-RTI.
    cfg=model.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    dual=predictiveSafetyGeometry.supportLinearization(x(1:3),shape,q(1:3),q(8:11));
    normal=dual.normal;
    if isfield(model,'fixedCollisionNormals') && norm(model.fixedCollisionNormals(:,point))>0
        normal=model.fixedCollisionNormals(:,point);
    elseif isfield(model,'probeNormals') && norm(model.probeNormals(:,point))>0
        normal=model.probeNormals(:,point);
    end
    angle=atan2(normal(2),normal(1));
    rt=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];
    vertices=q(10:11)+q(8:9).*[-1,1,1,-1;-1,-1,1,1];
    witness=[angle;max(normal.'*rt*vertices)];
    if isfield(model,'referenceWitnesses') && all(isfinite(model.referenceWitnesses(:,point)))
        witness=model.referenceWitnesses(:,point);
        normal=[cos(witness(1));sin(witness(1))];
    end
    data=struct('shape',shape,'target',q,'generator',generator, ...
        'targetSupportData',model.targetSupportData,'point',point, ...
        'targetYawRadius',model.targetTube.yawRadius(point),'epoch',model.target, ...
        'error',model.uncertainty.target,'time',model.recordTimes(point));
    [values,jacobianFull,tightening]=localPlaneRows(x,witness,data);
    jacobian=jacobianFull(:,1:3);witnessJacobian=jacobianFull(:,4:5);
end

function [values,jacobian,tightening]=localPlaneRows(x,witness,data)
    reserve=localPlaneReserve(witness(1),data);
    % The engineering support oracle is piecewise differentiable. Both
    % backends use the same symmetric derivative at support-feature ties.
    % It is not a smoothness or nonlinear-enclosure certificate at those ties.
    step=1e-6;
    derivative=(localPlaneReserve(witness(1)+step,data)-localPlaneReserve(witness(1)-step,data))/(2*step);
    [values,jacobian]=predictiveSafetyGeometry.separationWitnessRows( ...
        x(1:3),data.shape,data.target(1:3),data.target(8:11),witness,reserve,derivative);
    tightening=reserve;
end

function reserve=localPlaneReserve(angle,data)
    normal=[cos(angle),sin(angle)];q=data.target;shape=data.shape;
    position=predictiveSafetyGeometry.targetSupportFromData(data.targetSupportData,data.point,normal);
    [~,rotation]=predictiveSafetyGeometry.rotationSupport(q(8:11),q(3),data.targetYawRadius,normal);
    target=position+rotation;
    if ~isempty(data.error)
        upper=predictiveSafetyGeometry.targetDirectionalSupport(data.epoch,data.error,data.time,normal);
        nominal=predictiveSafetyGeometry.targetDirectionalSupport(q,[],0,normal);
        target=min(target,max(0,upper-nominal));
    end
    yaw=sum(abs(data.generator(3,:)));
    reserve=sum(abs(normal*data.generator(1:2,:)))+target ...
        +2*(norm(shape(1:2))+norm(shape(3:4)))*sin(min(pi,yaw)/2);
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
        trackingOffset(rows)=error(1:components)./units(1:components);
        trackingValues(:,:,k)=jacobian(1:components,:)./units(1:components);
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
        case "retained"
            model.solveTimer=timer;model.solveTimeLimitSeconds=model.cfg.solver.timeLimitSeconds;
            [result,search,model]=localRetainedSolve(model,request.previousState,timer,request.anchor);
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
        case "nonlinearProblem"
            model.solveTimer=timer;model.solveTimeLimitSeconds=model.cfg.solver.timeLimitSeconds;
            model.cfg.rti.trustRegion.enabled=false;
            [problem,model]=localFormulate(request.anchor,model);
            problem=localSelectObjective(problem,request.anchor,model);
            data=localNonlinearData(request.anchor,model,problem);
            result=struct('constraint',@(z)localNonlinearConstraints(z,data), ...
                'performance',@(z)localNonlinearObjective(z,data,false), ...
                'safety',@(z)localNonlinearObjective(z,data,true), ...
                'point',zeros(data.nv,1),'lower',data.lower,'upper',data.upper, ...
                'stateIndices',problem.stateIndices,'inputIndices',problem.inputIndices, ...
                'slackIndices',problem.slackIndices,'clfIndex',problem.clfIndex, ...
                'witnessIndices',problem.witnessIndices,'collisionWitnesses',problem.collisionWitnesses, ...
                'affineProblem',problem);
        otherwise
            error('collisionAvoidanceController:invalidProbeOperation','Unknown research operation.');
    end
end

function [solution,search,model]=localNonlinearSolve(anchor,model,source,timer,initializationSeconds)
    % Restore the shifted/planned control sequence before asking it to reduce
    % safety slack. This is an NLP phase-I initialization, not another
    % planner, maneuver family, horizon extension, or applied backup control.
    model.cfg.rti.trustRegion.enabled=false;
    iterations=0;calls=0;initialHard=NaN;finalHard=Inf;
    if isfield(model.uncertainty,'jointPrediction') && model.cfg.controller.jointAffinePrediction
        for pass=1:8
            anchor.states(:,1)=model.initialState;
            for k=1:size(anchor.inputs,2)
                anchor.states(:,k+1)=nonlinearBicycleModel.sample(anchor.states(:,k),anchor.inputs(:,k),model.cfg);
            end
            [problem,model]=localFormulate(anchor,model);problem=localSelectObjective(problem,anchor,model);
            model.fixedCollisionNormals=problem.safetyNormals;
            data=localNonlinearData(anchor,model,problem);point=zeros(size(data.lower));
            [c,~]=localNonlinearConstraints(point,data);
            for k=1:numel(problem.slackIndices)
                point(problem.slackIndices(k))=max([0;c(data.collisionRowSlack==k)]);
            end
            if problem.clf.enabled,point(problem.clfIndex)=max(0,c(end))/problem.clfSlackDuration;end
            [c,eq]=localNonlinearConstraints(point,data);
            finalHard=max([0;c;abs(eq);data.lower-point;point-data.upper]);
            if pass==1,initialHard=finalHard;end
            if finalHard<=model.cfg.solver.constraintTolerance || any(data.lower>data.upper),break;end
            data.restoration=true;data.restorationSideBuffer=2*finalHard;
            options=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
                'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true, ...
                'MaxIterations',max(80,model.cfg.solver.maxIterations), ...
                'MaxFunctionEvaluations',20000,'ConstraintTolerance',model.cfg.solver.constraintTolerance, ...
                'OptimalityTolerance',model.cfg.solver.optimalityTolerance,'StepTolerance',1e-10, ...
                'OutputFcn',@(x,v,state)toc(timer)>=model.solveTimeLimitSeconds);
            [point,~,output]=localNonlinearOptimize(point,data,options,true,Inf);
            iterations=iterations+output.iterations;calls=calls+1;
            anchor.inputs=anchor.inputs+reshape(point(problem.inputIndices),size(anchor.inputs));
            anchor.states=anchor.states+reshape(point(problem.stateIndices),size(anchor.states));
            anchor.collisionWitnesses=problem.collisionWitnesses+reshape(point(problem.witnessIndices),size(problem.witnessIndices));
        end
    end
    [solution,search,model]=localNonlinearIteration(anchor,model,source,timer,initializationSeconds);
    search.feasibilityRestoration=struct('calls',calls,'iterations',iterations, ...
        'initialViolation',initialHard,'lastEvaluatedViolation',finalHard);
    if isfield(search,'nonlinearIterations'),search.nonlinearIterations=search.nonlinearIterations+iterations;end
    search.solverCalls=search.solverCalls+calls;
end

function [solution,search,model]=localNonlinearIteration(anchor,model,source,timer,initializationSeconds)
    % Exact nonlinear shooting with causal observer/feedback margin refresh.
    % Each numerical subproblem freezes its engineering affine tube; accepted
    % updates re-evaluate that tube on the actual nonlinear trajectory. This
    % is not the complete nonlinear information-set certificate backend.
    cfg=model.cfg;model.cfg.rti.trustRegion.enabled=false;
    assert(~cfg.controller.jointNonlinearPrediction, ...
        'collisionAvoidanceController:unsupportedNonlinearBaseline', ...
        'The NLP baseline uses frozen engineering margins, not a certified decision box.');
    assert(isempty(model.road.lateralClearance), ...
        'collisionAvoidanceController:unsupportedNonlinearBaseline','The NLP baseline has no hard road constraints.');
    wall=tic;[problem,model]=localFormulate(anchor,model);
    problem=localSelectObjective(problem,anchor,model);
    data=localNonlinearData(anchor,model,problem);
    search=localEmptySearch(source,"",initializationSeconds);
    search.source="nonlinearSingleShooting";search.algorithm="fminconSqp";
    search.mode=problem.mode;search.modelBuilt=true;search.formulationSeconds=toc(wall);
    search.horizonSteps=size(anchor.inputs,2);search.planner=anchor.planner;
    search.plannerCalls=double(anchor.planner.used);search.rtiIterations=0;
    search.linearizationCount=0;search.horizonExtensions=0;
    search.anchorCertified=false;search.appendedTerminalSteps=anchor.appendedTerminalSteps;
    search.appendedBrakingSteps=anchor.appendedBrakingSteps;
    search.rollout=struct('available',true,'reason',"nonlinearShootingConstraints", ...
        'fullGap',NaN,'fullCorrection',NaN);
    search.clfInitialSlack=problem.initialClfSlack;search.clfStageAttempted=problem.clf.enabled;
    search.pcbfStageAttempted=~problem.clf.enabled;
    solution=[];point=zeros(size(problem.lower));
    if any(data.lower>data.upper)
        search.terminationReason="nonlinearFrozenBoundsInconsistent";
    else
        point=min(data.upper,max(data.lower,point));
        [c,~]=localNonlinearConstraints(point,data);
        if ~problem.clf.enabled
            for k=1:numel(problem.slackIndices)
                rows=data.collisionRowSlack==k;
                point(problem.slackIndices(k))=max([0;c(rows)]);
            end
        else
            point(problem.clfIndex)=max(0,c(end))/problem.clfSlackDuration;
        end
        options=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
            'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true, ...
            'MaxIterations',cfg.solver.maxIterations, ...
            'MaxFunctionEvaluations',100*cfg.solver.maxIterations, ...
            'ConstraintTolerance',cfg.solver.constraintTolerance, ...
            'OptimalityTolerance',cfg.solver.optimalityTolerance,'StepTolerance',1e-10, ...
            'OutputFcn',@(x,v,state)toc(timer)>=model.solveTimeLimitSeconds);
        [c,eq]=localNonlinearConstraints(point,data);
        initialFeasible=all(isfinite([c;eq])) && max([0;c;abs(eq)])<=cfg.solver.constraintTolerance;
        numerical=problem.clf.enabled || ~initialFeasible || any(point(problem.slackIndices)>0);
        wall=tic;flag=1;output=struct('iterations',0,'funcCount',0,'constrviolation',0);
        primary=~problem.clf.enabled;
        if numerical
            [point,flag,output]=localNonlinearOptimize(point,data,options,primary,Inf);
            search.solverCalls=1;
        end
        [c,eq]=localNonlinearConstraints(point,data);
        hard=max([0;c;abs(eq);data.lower-point;point-data.upper]);
        if any(~isfinite([point;c;eq])),hard=Inf;end
        search.stages(1)=struct('objective',problem.optimizationObjective,'exitFlag',flag, ...
            'seconds',toc(wall),'value',localNonlinearObjective(point,data,primary), ...
            'numericalSolve',numerical,'solverInfo',output);
        search.converged=flag>0;search.nonlinearIterations=output.iterations;
        search.terminationReason="nonlinearConstraintsNotSatisfied";
        search.finalConstraintViolation=hard;
        search.constraintResiduals=struct('inequality',max([0;c]), ...
            'dynamics',max(abs(eq)),'bounds',max([0;data.lower-point;point-data.upper]), ...
            'terminal',c(end-double(problem.clf.enabled)));
        if hard<=cfg.solver.constraintTolerance
            search.returned=true;search.terminationReason="nonlinearFeasibleReturned";
            search.clfStageCompleted=problem.clf.enabled;search.pcbfStageCompleted=~problem.clf.enabled;
            refreshMargins=isfield(model,'jointPredictionTube') && ~isempty(model.jointPredictionTube);
            if refreshMargins
                [point,anchor,problem,data,model,refresh]=localNonlinearRefresh(point,anchor,problem,data,model,primary,Inf);
                search.marginRefresh=refresh;
                search.converged=search.converged && refresh.step==1;
                if ~refresh.accepted
                    solution=[];search.returned=false;search.terminationReason="nonlinearUpdatedMarginsInfeasible";
                    search.stopReason=search.terminationReason;search.elapsedSeconds=toc(timer);
                    model.linearization=anchor;return;
                end
            end
            if primary
                search.primaryOptimum=sum(point(problem.slackIndices));
                search.primaryLowerBound=0;search.slackCap=search.primaryOptimum;
                search.secondaryStageAttempted=true;search.solverCalls=search.solverCalls+1;
                b=search.slackCap;
                upper=data.upper;
                if b==0,upper(problem.slackIndices)=0;end
                wall=tic;
                [refined,secondFlag,secondOutput]=localNonlinearOptimize(point,data,options,false,search.slackCap);
                [cc,ee]=localNonlinearConstraints(refined,data);
                violation=max([0;cc;abs(ee);data.lower-refined;refined-upper; ...
                    sum(refined(problem.slackIndices))-search.slackCap]);
                if any(~isfinite([refined;cc;ee])),violation=Inf;end
                search.stages(2)=struct('objective',"nonlinearTrackingAndRegularization", ...
                    'exitFlag',secondFlag,'seconds',toc(wall), ...
                    'value',localNonlinearObjective(refined,data,false),'numericalSolve',true,'solverInfo',secondOutput);
                search.nonlinearIterations=search.nonlinearIterations+secondOutput.iterations;
                search.converged=search.converged && secondFlag>0 && violation<=cfg.solver.constraintTolerance;
                if violation<=cfg.solver.constraintTolerance
                    point=refined;search.secondaryStageCompleted=true;
                end
            end
            if refreshMargins && primary
                [point,anchor,problem,data,model,refresh]=localNonlinearRefresh( ...
                    point,anchor,problem,data,model,false,search.slackCap);
                search.secondaryMarginRefresh=refresh;
                search.converged=search.converged && refresh.step==1;
                if ~refresh.accepted
                    solution=[];search.returned=false;search.terminationReason="nonlinearUpdatedMarginsInfeasible";
                    search.stopReason=search.terminationReason;search.elapsedSeconds=toc(timer);
                    model.linearization=anchor;return;
                end
            end
            inputs=anchor.inputs+reshape(point(problem.inputIndices),size(anchor.inputs));
            [c,eq]=localNonlinearConstraints(point,data);
            hard=max([0;c;abs(eq);data.lower-point;point-data.upper]);
            search.constraintResiduals=struct('inequality',max([0;c]),'dynamics',max(abs(eq)), ...
                'bounds',max([0;data.lower-point;point-data.upper]),'terminal',c(end-double(problem.clf.enabled)));
            solution=localDecode(point,problem,anchor,inputs);
            solution.hard=hard;solution.nonlinearValidationPerformed=true;
            search.finalConstraintViolation=hard;
            solution.affineSafety=NaN;solution.affineStates=[];solution.affineStageSlacks=[];
            solution.inputBoundCorrection=0;solution.trustMaximumScaledCorrection=NaN;
            solution.trustFirstInputRadius=[Inf;Inf];solution.trustSampledErrorRatio=NaN;
            solution.collisionEscapeNormals=model.collisionEscapeNormals;
            solution.collisionSide=model.collisionSide;solution.retainCollisionSide=model.retainCollisionSide;
            solution.clfNextValue=nonlinearBicycleModel.nominalValue(solution.states(:,2),model.lane,model.nominalReference);
            [~,~,costs]=localNonlinearObjective(point,data,false);
            names=fieldnames(costs);
            for k=1:numel(names),solution.(names{k})=costs.(names{k});end
            solution.objectiveValue=solution.safety;
            if problem.clf.enabled,solution.objectiveValue=localNonlinearObjective(point,data,false);end
            if primary
                solution.secondaryObjective="nonlinearTrackingAndRegularization";
                solution.secondaryObjectiveValue=solution.performanceCost;
            else
                solution.clfDerivative=(solution.clfNextValue-solution.clfInitialValue)/problem.clfSlackDuration;
                solution.clfActualDerivative=nonlinearBicycleModel.nominalDerivative( ...
                    model.initialState,inputs(:,1),model.lane,model.nominalReference,cfg);
                solution.clfActualDerivativeResidual=solution.clfActualDerivative ...
                    +model.nominalReference.decayRate*solution.clfInitialValue-solution.clfSlack;
            end
            solution=localTerminalMetrics(solution,model);
            solution.terminalEnd="nonlinearEndpoint";
            [member,~,info]=terminalSafeSet.member(data.terminalContext,solution.states(:,end),search.horizonSteps);
            solution.terminalMarginMeters=info.margin;solution.terminalExitSeconds=info.exitSeconds;
            solution.terminalEnd=info.end;search.nonlinearTerminalMember=member;
            if ~member
                solution=[];search.returned=false;search.terminationReason="nonlinearTerminalMembershipFailed";
            end
        end
    end
    model.linearization=anchor;model.trustWarmStartAdjusted=false;
    search.stopReason=search.terminationReason;
    if isfield(search,'finalConstraintViolation')
        search.stopReason=search.stopReason+sprintf( ...
            ' (residual %.6g, %d nonlinear iterations; inequality %.6g, dynamics %.6g, terminal %.6g)', ...
            search.finalConstraintViolation,search.nonlinearIterations, ...
            search.constraintResiduals.inequality,search.constraintResiduals.dynamics,search.constraintResiduals.terminal);
    end
    search.elapsedSeconds=toc(timer);
    search.attempts=localAttempt(search);search.selectedAttempt=double(search.returned);
end

function [point,anchor,problem,data,model,info]=localNonlinearRefresh(point,anchor,problem,data,model,primary,slackCap)
    % The candidate's observer/feedback map changes with its trajectory.
    % Backtrack controls, roll out the EXACT vehicle, and rebuild that map.
    % This is a numerical globalization of the NLP, not a new maneuver or a
    % frozen tube certificate. Never return a step using stale supports.
    base=anchor;proposal=anchor.inputs+reshape(point(problem.inputIndices),size(anchor.inputs));
    fixed=model;
    baseWitness=problem.collisionWitnesses;
    witnessStep=reshape(point(problem.witnessIndices),size(problem.witnessIndices));
    info=struct('accepted',false,'step',NaN,'evaluations',0,'initialSlack',NaN,'returnedSlack',NaN, ...
        'trials',zeros(0,4));
    best=Inf;chosen=[];steps=[1,.5,.25,.125,.0625,0];
    unchanged=isequal(proposal,base.inputs);
    if unchanged,steps=1;end
    for alpha=steps
        trial=base;trial.inputs=base.inputs+alpha*(proposal-base.inputs);
        trial.collisionWitnesses=baseWitness+alpha*witnessStep;
        trial.states(:,1)=model.initialState;info.evaluations=info.evaluations+1;
        try
            for k=1:size(trial.inputs,2)
                trial.states(:,k+1)=nonlinearBicycleModel.sample(trial.states(:,k),trial.inputs(:,k),model.cfg);
            end
            [p,m]=localFormulate(trial,fixed);p=localSelectObjective(p,trial,m);
            d=localNonlinearData(trial,m,p);z=zeros(size(p.lower));
            c=localNonlinearConstraints(z,d);
            for k=1:numel(p.slackIndices)
                z(p.slackIndices(k))=max([0;c(d.collisionRowSlack==k)]);
            end
            if p.clf.enabled,z(p.clfIndex)=max(0,c(end))/p.clfSlackDuration;end
            [c,eq]=localNonlinearConstraints(z,d);
            hard=max([0;c;abs(eq);d.lower-z;z-d.upper]);
            info.trials(end+1,:)=[alpha,hard,max([0;d.lower-z;z-d.upper]),c(end)];
            slack=sum(z(p.slackIndices));merit=slack;
            if ~primary
                correction=z;correction(problem.stateIndices(:))=reshape(trial.states-base.states,[],1);
                correction(problem.inputIndices(:))=reshape(trial.inputs-base.inputs,[],1);
                merit=localNonlinearObjective(correction,data,false);
            end
            if alpha==0 || unchanged,info.initialSlack=slack;end
            if all(isfinite([z;c;eq])) && hard<=model.cfg.solver.constraintTolerance ...
                    && slack<=slackCap+model.cfg.solver.constraintTolerance && merit<best
                best=merit;chosen=struct('point',z,'anchor',trial,'problem',p,'data',d,'model',m);
                info.accepted=true;info.step=alpha;info.returnedSlack=slack;
            end
        catch exception
            if ~any(string(exception.identifier)==["collisionAvoidanceController:nonlinearDomain", ...
                    "collisionAvoidanceController:invalidTireOperatingPoint", ...
                    "collisionAvoidanceController:singularTireLinearization", ...
                    "nrmmJointPrediction:inputDomain"])
                rethrow(exception);
            end
        end
    end
    if info.accepted
        point=chosen.point;anchor=chosen.anchor;problem=chosen.problem;data=chosen.data;model=chosen.model;
    end
end

function [point,flag,output]=localNonlinearOptimize(point,data,options,primary,slackCap)
    % Exact nonlinear single shooting eliminates dependent state variables.
    % The chain rule below is the derivative of the complete nonlinear
    % rollout, not an affine prediction used in place of that rollout.
    p=data.problem;indices=[p.inputIndices(:);p.slackIndices(:);p.clfIndex;p.witnessIndices(:)];
    units=[data.model.cfg.clf.certificationSteeringRadians;data.model.cfg.clf.certificationBrakingRatio];
    scale=[repmat(units,data.count,1);ones(numel(p.slackIndices)+1+numel(p.witnessIndices),1)];
    lower=data.lower(indices)./scale;upper=data.upper(indices)./scale;
    a=[];b=[];
    if isfinite(slackCap)
        a=double(ismember(indices,p.slackIndices)).';b=slackCap;
        if slackCap==0,upper(ismember(indices,p.slackIndices))=0;a=[];b=[];end
    end
    cached=[];full=[];chain=[];valid=false;
    motionInput=[];motionStates=[];motionJacobian=[];
    constraintPoint=[];constraintValue=[];constraintGradient=[];
    incumbent=[];best=Inf;deadline=options.OutputFcn;
    initial=point(indices)./scale;expand(initial);
    if primary && valid
        c=localNonlinearConstraints(full,data);
        for slackIndex=1:numel(p.slackIndices)
            initial(indices==p.slackIndices(slackIndex))=max([0; ...
                c(data.collisionRowSlack==slackIndex)+full(p.slackIndices(slackIndex))]);
        end
    end
    retain(initial);options.OutputFcn=@iterate;
    [reduced,~,flag,output]=fmincon(@objective,initial,a,b,[],[],lower,upper,@constraints,options);
    retain(reduced);output.feasibleIncumbentUsed=false;
    [finalRows,~]=constraints(reduced);finalValue=objective(reduced);
    finalHard=max([0;finalRows;lower-reduced;reduced-upper; ...
        sum(reduced(ismember(indices,p.slackIndices)))-slackCap]);
    tolerance=data.model.cfg.solver.optimalityTolerance*max(1,abs(best));
    if ~isempty(incumbent) && (any(~isfinite([finalRows;finalValue])) ...
            || finalHard>data.model.cfg.solver.constraintTolerance || finalValue>best+tolerance)
        reduced=incumbent;output.feasibleIncumbentUsed=true;
        output.lastIterateExitFlag=flag;flag=0;
    end
    expand(reduced);point=full;
    function retain(z)
        [c,~]=constraints(z);
        if primary && valid && all(isfinite(c))
            % Slacks are epigraph variables. Their exact least feasible
            % values are available without an optimization or motion change.
            for deficitIndex=1:numel(p.slackIndices)
                j=indices==p.slackIndices(deficitIndex);rows=data.collisionRowSlack==deficitIndex;
                z(j)=max([0;c(find(rows))+z(j)]); %#ok<FNDSB>
            end
            [c,~]=constraints(z);
        end
        value=objective(z);
        violation=max([0;c;lower-z;z-upper;sum(z(ismember(indices,p.slackIndices)))-slackCap]);
        if all(isfinite([c;value])) && violation<=data.model.cfg.solver.constraintTolerance && value<best
            best=value;incumbent=z;
        end
    end
    function stop=iterate(z,values,state)
        retain(z);
        stop=deadline(z,values,state);
    end
    function expand(z)
        if isequal(z,cached),return;end
        cached=z;full=zeros(data.nv,1);full(indices)=scale.*z;
        chain=zeros(data.nv,numel(z));chain(indices,:)=diag(scale);
        % Epigraph/slack updates do not change the nonlinear motion. Retain
        % the rollout and its exact sensitivity until an input changes.
        if isequal(full(p.inputIndices(:)),motionInput)
            full(p.stateIndices(:))=motionStates;
            chain(p.stateIndices(:),:)=motionJacobian;
            return;
        end
        motionInput=full(p.inputIndices(:));
        x=data.model.initialState;dx=zeros(6,numel(z));valid=true;
        full(p.stateIndices(:,1))=x-data.anchor.states(:,1);
        data.shooting=struct('middle',zeros(6,data.count),'am',zeros(6,6,data.count), ...
            'bm',zeros(6,2,data.count),'next',zeros(6,data.count), ...
            'a',zeros(6,6,data.count),'b',zeros(6,2,data.count));
        try
            for motionStep=1:data.count
                columns=p.inputIndices(:,motionStep);u=data.anchor.inputs(:,motionStep)+full(columns);
                [middle,am,bm,x,aa,bb]=nonlinearBicycleModel.hold(x,u,data.model.cfg);
                data.shooting.middle(:,motionStep)=middle;data.shooting.am(:,:,motionStep)=am;
                data.shooting.bm(:,:,motionStep)=bm;data.shooting.next(:,motionStep)=x;
                data.shooting.a(:,:,motionStep)=aa;data.shooting.b(:,:,motionStep)=bb;
                dx=aa*dx+bb*chain(columns,:);
                full(p.stateIndices(:,motionStep+1))=x-data.anchor.states(:,motionStep+1);
                chain(p.stateIndices(:,motionStep+1),:)=dx;
            end
        catch exception
            if ~any(string(exception.identifier)==["collisionAvoidanceController:nonlinearDomain", ...
                    "collisionAvoidanceController:invalidTireOperatingPoint", ...
                    "collisionAvoidanceController:singularTireLinearization"])
                rethrow(exception);
            end
            valid=false;
        end
        motionStates=full(p.stateIndices(:));motionJacobian=chain(p.stateIndices(:),:);
    end
    function [value,gradient]=objective(z)
        expand(z);
        if ~valid,value=NaN;gradient=zeros(size(z));return;end
        [value,g]=localNonlinearObjective(full,data,primary);gradient=chain.'*g;
    end
    function [c,e,gc,ge]=constraints(z)
        expand(z);
        if isequal(z,constraintPoint)
            c=constraintValue;gc=constraintGradient;e=zeros(0,1);ge=zeros(numel(z),0);return;
        end
        state=p.stateIndices(:);
        lo=isfinite(data.lower(state));hi=isfinite(data.upper(state));
        if valid
            [c,~,g]=localNonlinearConstraints(full,data);
            c=[c;data.lower(state(lo))-full(state(lo));full(state(hi))-data.upper(state(hi))];
            gc=[chain.'*g,-chain(state(lo),:).',chain(state(hi),:).'];
        else
            c=nan(numel(data.collisionRowSlack)+nnz(lo)+nnz(hi),1);gc=zeros(numel(z),numel(c));
        end
        constraintPoint=z;constraintValue=c;constraintGradient=gc;
        e=zeros(0,1);ge=zeros(numel(z),0);
    end
end

function data=localNonlinearData(anchor,model,problem)
    cfg=model.cfg;count=size(anchor.inputs,2);nv=numel(problem.lower);
    data=struct('anchor',anchor,'model',model,'problem',problem,'count',count,'nv',nv);
    data.lower=problem.lower;data.upper=problem.upper;
    data.stateRadius=model.uncertaintyPrediction.egoStateRadius;
    data.sideReserve=zeros(2,count);data.frontReserve=zeros(2,2*count+1);
    data.collisionData=problem.collisionData;
    data.forecast=localTargetAt(model,(0:2*count)*cfg.controller.sampleTime/2);
    data.shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    data.terminalContext=model.terminalContext;
    data.terminalContext.solveTimeLimitSeconds=Inf;
    model.fixedCollisionNormals=problem.safetyNormals;
    side=[0,0,0,-tan(cfg.model.sideslipMaximum),1,0;0,0,0,-tan(cfg.model.sideslipMaximum),-1,0];
    for p=1:2*count+1
        x=model.sampledStates(:,p);generator=diag(data.stateRadius(:,p));
        if isfield(model,'jointPredictionTube') && ~isempty(model.jointPredictionTube)
            generator=model.jointPredictionTube.generator{p};
        elseif p==1
            generator=model.uncertainty.egoGenerator;
        end
        if p>1 && mod(p,2)==1,data.sideReserve(:,(p-1)/2)=sum(abs(side*generator),2);end
        [~,bounds]=localFrontDomain(x,[0;0],cfg,generator);
        angle=atan2(x(5)+cfg.vehicle.lf*x(6),x(4));limit=pi/2-10*cfg.solver.constraintTolerance;
        data.frontReserve(:,p)=[limit-angle;limit+angle]-bounds;
    end
    % Rows have a fixed identity even when a terminal closest feature changes.
    % One scalar evaluates the terminal continuation directly.
    data.collisionRowSlack=zeros(60*count+20,1);row=0;
    for k=1:count
        if ~isempty(model.target)
            data.collisionRowSlack(row+(1:4))=k;
            data.collisionRowSlack(row+8+(1:4))=k;row=row+16;
        end
        row=row+6+6+2; % midpoint bounds, three front domains, next sideslip
        if isfinite(cfg.model.brakingRatioRateMaximum),row=row+2;end
    end
    if ~isempty(model.target),data.collisionRowSlack(row+(1:4))=count+1;row=row+8;end
    row=row+1+double(problem.clf.enabled);
    data.collisionRowSlack=data.collisionRowSlack(1:row);
end

function [c,ceq,gc,gceq]=localNonlinearConstraints(point,data)
    % SQP line search may try points outside the Fiala function domain.
    % A nonfinite evaluation requests backtracking; it must not alter the
    % tire model or turn a rejected trial into an applied vehicle command.
    try
        if nargout>2
            [c,ceq,gc,gceq]=localNonlinearConstraintValues(point,data);
        else
            [c,ceq]=localNonlinearConstraintValues(point,data);
        end
    catch exception
        if ~any(string(exception.identifier)==["collisionAvoidanceController:nonlinearDomain", ...
                "collisionAvoidanceController:invalidTireOperatingPoint", ...
                "collisionAvoidanceController:singularTireLinearization"])
            rethrow(exception);
        end
        c=nan(numel(data.collisionRowSlack),1);ceq=nan(6*(data.count+1),1);
        gc=zeros(data.nv,numel(c));gceq=zeros(data.nv,numel(ceq));
    end
end

function [c,ceq,gc,gceq]=localNonlinearConstraintValues(point,data)
    p=data.problem;m=data.model;cfg=m.cfg;n=data.count;nv=data.nv;
    x=data.anchor.states+reshape(point(p.stateIndices),size(data.anchor.states));
    u=data.anchor.inputs+reshape(point(p.inputIndices),size(data.anchor.inputs));
    c=zeros(numel(data.collisionRowSlack),1);jac=zeros(numel(c),nv);row=0;
    ceq=zeros(6*(n+1),1);jeq=zeros(numel(ceq),nv);
    ceq(1:6)=x(:,1)-m.initialState;jeq(1:6,p.stateIndices(:,1))=eye(6);
    low=[cfg.model.speedMinimum;-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
    high=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
    side=[0,0,0,-tan(cfg.model.sideslipMaximum),1,0;0,0,0,-tan(cfg.model.sideslipMaximum),-1,0];
    for k=1:n
        if isfield(data,'shooting')
            middle=data.shooting.middle(:,k);am=data.shooting.am(:,:,k);bm=data.shooting.bm(:,:,k);
            next=data.shooting.next(:,k);a=data.shooting.a(:,:,k);b=data.shooting.b(:,:,k);
        else
            [middle,am,bm,next,a,b]=nonlinearBicycleModel.hold(x(:,k),u(:,k),cfg);
        end
        cols=[p.stateIndices(:,k);p.inputIndices(:,k)];map=[am,bm];
        rows=6*k+(1:6);ceq(rows)=x(:,k+1)-next;
        jeq(rows,cols)=-[a,b];jeq(rows,p.stateIndices(:,k+1))=eye(6);
        if ~isempty(m.target)
            collision(x(:,k),2*k-1,p.stateIndices(:,k),eye(6),k);
            collision(middle,2*k,cols,map,k);
        end
        radius=data.stateRadius(4:6,2*k);
        push([middle(4:6)-high+radius;low-middle(4:6)+radius], ...
            [map(4:6,:);-map(4:6,:)],cols);
        front(x(:,k),u(:,k),2*k-1,[p.stateIndices(:,k);p.inputIndices(:,k)],eye(6,8),k);
        front(x(:,k+1),u(:,k),2*k+1,[p.stateIndices(:,k+1);p.inputIndices(:,k)],eye(6,8),k);
        front(middle,u(:,k),2*k,cols,map,k);
        value=side*x(:,k+1)+data.sideReserve(:,k);derivative=side;
        if isfield(data,'restorationSideBuffer'),value=value+data.restorationSideBuffer;end
        if x(4,k+1)<=cfg.model.brakeHoldSpeed,value(:)=-1;derivative(:)=0;end
        push(value,derivative,p.stateIndices(:,k+1));
        if isfinite(cfg.model.brakingRatioRateMaximum)
            previous=m.previousInput;columns=p.inputIndices(2,k);gradient=1;
            if k>1,previous=u(:,k-1);columns=[columns,p.inputIndices(2,k-1)];gradient=[1,-1];end
            rate=cfg.model.brakingRatioRateMaximum*cfg.controller.sampleTime;
            push([u(2,k)-previous(2)-rate;previous(2)-u(2,k)-rate],[gradient;-gradient],columns);
        end
    end
    if ~isempty(m.target),collision(x(:,end),2*n+1,p.stateIndices(:,end),eye(6),n+1);end
    [terminal,terminalGradient]=localNonlinearTerminal(x(:,end),data,nargout>2);
    push(terminal,terminalGradient,p.stateIndices(:,end));
    if p.clf.enabled
        [e,j]=nonlinearBicycleModel.errorLinearization(x(:,2),m.lane,m.nominalReference);
        value=e.'*m.nominalReference.matrix*e;
        alpha=m.nominalReference.decayRate;
        residual=(value-exp(-alpha*cfg.controller.sampleTime)*p.initialClfValue)/p.clfScale ...
            -p.clfSlackDuration*point(p.clfIndex);
        gradient=[2*e.'*m.nominalReference.matrix*j/p.clfScale,-p.clfSlackDuration];
        push(residual,gradient,[p.stateIndices(:,2);p.clfIndex]);
    end
    assert(row==numel(c));gc=sparse(jac.');gceq=sparse(jeq.');
    function push(value,gradient,columns)
        rows=row+(1:numel(value));c(rows)=value;jac(rows,columns)=gradient;row=row+numel(value);
    end
    function collision(y,index,columns,map,slack)
        planeColumns=[columns(:);p.witnessIndices(:,index);p.slackIndices(slack)];
        if isempty(data.collisionData{index})
            push(-ones(8,1),zeros(8,numel(planeColumns)),planeColumns);return;
        end
        witness=p.collisionWitnesses(:,index)+point(p.witnessIndices(:,index));
        [gap,j]=localPlaneRows(y,witness,data.collisionData{index});
        relaxed=[ones(4,1);zeros(4,1)];
        push(-gap-relaxed*point(p.slackIndices(slack)), ...
            [-j(:,1:3)*map(1:3,:),-j(:,4:5),-relaxed],planeColumns);
    end
    function front(y,input,index,columns,map,stage)
        [r,bound]=localFrontDomain(y,input,cfg,zeros(6,0));
        gradient=r(:,1:6)*map;gradient(:,end-1:end)=gradient(:,end-1:end)+r(:,7:8);
        reserve=data.frontReserve(:,index);
        if isfield(m,'jointPredictionTube') && ~isempty(m.jointPredictionTube)
            reserve=reserve+m.jointPredictionTube.inputRadius(1,stage);
        end
        push(-bound+reserve,gradient,columns);
    end
end

function [value,gradient]=localNonlinearTerminal(y,data,wantGradient)
    % Re-evaluate the declared braking continuation, never its old tangent.
    % Signed geometry keeps the row count fixed across closest-feature changes.
    if wantGradient
        [value,gradient]=terminalSafeSet.nonlinearTangent(data.terminalContext,y,data.count);
    else
        value=terminalSafeSet.nonlinearValue(data.terminalContext,y,data.count);gradient=zeros(1,6);
    end
end

function [value,gradient,costs]=localNonlinearObjective(point,data,primary)
    p=data.problem;m=data.model;cfg=m.cfg;n=data.count;
    gradient=zeros(data.nv,1);
    if primary
        if isfield(data,'restoration') && data.restoration
            units=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio];
            deviation=reshape(point(p.inputIndices),2,[])./units;
            value=.5*sum(deviation.^2,'all');gradient(p.inputIndices(:))=reshape(deviation./units,[],1);
        else
            value=sum(point(p.slackIndices));gradient(p.slackIndices)=1;
        end
        return;
    end
    x=data.anchor.states+reshape(point(p.stateIndices),size(data.anchor.states));
    u=data.anchor.inputs+reshape(point(p.inputIndices),size(data.anchor.inputs));
    stateUnits=[cfg.clf.lateralPositionErrorScale;cfg.clf.headingErrorScale;cfg.clf.speedErrorScale; ...
        cfg.clf.lateralVelocityErrorScale;cfg.clf.yawRateErrorScale];
    inputUnits=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio];
    % Both terms are stage sums. Averaging only tracking by n makes longer
    % horizons suppress corrective steering relative to input smoothing.
    tracking=0;variation=0;
    for k=1:n
        [error,j]=nonlinearBicycleModel.errorLinearization(x(:,k+1),m.lane,m.nominalReference);
        tracking=tracking+.5*cfg.rti.nominalTrackingWeight*sum((error./stateUnits).^2);
        gradient(p.stateIndices(:,k+1))=cfg.rti.nominalTrackingWeight*j.'*(error./stateUnits.^2);
        previous=m.previousInput;if k>1,previous=u(:,k-1);end
        difference=u(:,k)-previous;
        variation=variation+.5*cfg.rti.inputVariationWeight*sum((difference./inputUnits).^2);
        g=cfg.rti.inputVariationWeight*difference./inputUnits.^2;
        gradient(p.inputIndices(:,k))=gradient(p.inputIndices(:,k))+g;
        if k>1,gradient(p.inputIndices(:,k-1))=gradient(p.inputIndices(:,k-1))-g;end
    end
    refMap=p.referenceStateMap.'*p.referenceStateMap+p.referenceInputMap.'*p.referenceInputMap;
    reference=.5*cfg.rti.referenceDeviationWeight*(point.'*refMap*point);
    gradient=gradient+cfg.rti.referenceDeviationWeight*refMap*point;
    value=tracking+variation+reference;
    costs=struct('nominalTrackingCost',tracking,'inputVariationCost',variation, ...
        'referenceDeviationCost',reference,'performanceCost',value);
    value=value/p.clfScale;gradient=gradient/p.clfScale;
    if p.clf.enabled
        value=value+p.clfSlackDuration*point(p.clfIndex);
        gradient(p.clfIndex)=p.clfSlackDuration;
    end
end

function [solution,search,model]=localRetainedSolve(model,previous,timer,seed)
    % A convex policy box is admitted only after its COMPLETE nonlinear
    % joint image and terminal continuation have been enclosed. Performance
    % states are affine surrogates, never hard safety witnesses. An existing
    % certificate remains available when a new image/optimization fails.
    cfg=model.cfg;
    assert(model.uncertainty.specified && isfield(model.uncertainty,'jointPrediction'), ...
        'collisionAvoidanceController:missingJointPrediction','Retained tubes require the synchronized observer snapshot.');
    assert(~isempty(model.uncertainty.target), ...
        'collisionAvoidanceController:missingTargetCertificate', ...
        'Retained tubes require an explicit target error family; missing bounds never mean an exact target.');
    assert(~cfg.controller.enforceRoadBoundaries && isempty(model.road.lateralClearance), ...
        'collisionAvoidanceController:retainedRoadUnsupported','Road constraints are outside this certificate formulation.');
    contract=model.uncertainty.jointPrediction;h=cfg.controller.sampleTime;
    inherited=isstruct(previous) && isfield(previous,'retainedCertificate') && ~model.targetOnset;
    assert(inherited || model.targetOnset,'collisionAvoidanceController:missingRetainedCertificate', ...
        'An ongoing encounter must retain its admitted certificate; it cannot restart its safety budget.');
    search=localEmptySearch("retainedPolicyTube","",0);search.planner=localPlannerMetadata();
    search.refinementFailure="";search.posterior=[];
    search.certificateReused=false;freshAdmission=~inherited;
    search.nominalProposal=struct('available',false,'accepted',false,'solverCalls',0,'linearizationCount',0,'failure',"notRequested");
    if inherited
        old=previous.retainedCertificate;
        assert(old.valid && old.absoluteIndex+1==model.sampleIndex, ...
            'collisionAvoidanceController:invalidRetainedCertificate','Only the immediately preceding certificate can be shifted.');
        certificate=old;certificate.absoluteIndex=model.sampleIndex;certificate.epochSeconds=old.epochSeconds+h;
        certificate.slacks=[old.slacks(2:end),0];certificate.gaps=[old.gaps(2:end),Inf];
        certificate.normals=[old.normals(:,2:end),zeros(2,1)];
        if isempty(old.policies)
            % The mathematical continuation uses the previously certified
            % terminal law and its ORIGINAL entry set. Do not restart that
            % law from a fresh estimate or resume a Fiala driving policy.
            certificate.terminalElapsed=old.terminalElapsed+h;
        else
            certificate.policies=old.policies(2:end);certificate.tube.sets=old.tube.sets(2:end);
            certificate.tube.inputs=old.tube.inputs(2:end);certificate.tube.middles=old.tube.middles(2:end);
            if isfield(old.tube,'changes'),certificate.tube.changes=old.tube.changes(2:end);end
            search.posterior=nrmmJointPrediction('conditionRetained',certificate.tube.sets{1}, ...
                model.initialState,model.uncertainty.egoGenerator,contract,old.contract,cfg);
            certificate.posterior=search.posterior;
            model.uncertainty=predictiveSafetyGeometry.conditionTargetHistory( ...
                model.uncertainty,search.posterior.proposedOuter(1:6),model.stateTime,model.target);
            % Each valid observation narrows the target information set.
            % Improve supports without discarding the original terminal
            % family: the actual target belongs to BOTH descriptions.
            certificate=localRetainedTargetUpdate(certificate,model);
            % The SAVED witness keeps its own target epoch. A fresh proposal
            % may use the current measurement-consistent target family, but
            % only after re-admitting its complete continuation and deficit.
            predicted=predictiveSafetyGeometry.targetPoseSet(certificate.target,certificate.targetError,certificate.epochSeconds);
            observed=predictiveSafetyGeometry.targetPoseSet(model.target,localAbsoluteTargetError(model),0);
            [pl,ph]=bounds(predicted(1:2));[ol,oh]=bounds(observed(1:2));
            assert(all(max(pl,ol)<=min(ph,oh)), ...
                'collisionAvoidanceController:inconsistentTargetPosterior','Observed target position contradicts the retained family.');
            budget=sum(certificate.slacks);
            % A new ego policy must not forget target information that made
            % the old certificate safe. Keep its original physical family
            % and epoch, and intersect it with the current observation.
            currentBase=certificate;currentBase.contract=contract;
            currentBase.refineTargetSupports=true;
            record=struct('sampleIndex',model.sampleIndex,'target',model.target, ...
                'errorSet',localAbsoluteTargetError(model),'improvedNodes',[], ...
                'normals',zeros(2,0));
            if ~isfield(currentBase,'targetRefinements'),currentBase.targetRefinements=record;
            elseif isempty(currentBase.targetRefinements) || currentBase.targetRefinements(end).sampleIndex~=model.sampleIndex
                currentBase.targetRefinements(end+1)=record;
            end
            speed=norm(model.initialState(4:5));
            braking=terminalSafeSet.context(model);
            renewalSteps=max(cfg.controller.horizonSteps,ceil(speed/(braking.deceleration*h))+1);
            % Positive deficit is not a safe driving plan: keep seeking
            % an improvement every frame until its robust deficit is zero.
            % Only a zero-deficit driving suffix can defer that search.
            renew=budget>0 || numel(certificate.policies)<=renewalSteps;
            search.certificateReused=~renew;
            if renew
                freshAdmission=true;
                feedbackScale=1;
                if isfield(certificate,'feedbackScale'),feedbackScale=certificate.feedbackScale;end
                % Try to retain a full driving horizon. If this extension cannot be
                % certified, the old terminal continuation remains the valid tail.
                [optimized,search.nominalProposal]=localRetainedProposal(model,previous,[],search.posterior.proposedOuter);
                fractions=0;
                if ~isempty(optimized),fractions=[1,.5,0];end
                failures=strings(1,numel(fractions));
                for attempt=1:numel(fractions)
                    try
                        candidate=localRetainedRenewal(search.posterior.proposedOuter, ...
                            certificate,optimized,fractions(attempt),currentBase,model,feedbackScale,budget);
                        if candidate.valid
                            certificate=candidate;
                            search.nominalProposal.accepted=fractions(attempt)>0;
                            search.refinementFailure="";break;
                        else
                            failures(attempt)=candidate.failureReason;
                        end
                    catch exception
                        if ~localCertificateFailure(exception),rethrow(exception);end
                        failures(attempt)=string(exception.identifier)+": "+exception.message;
                    end
                    search.refinementFailure=join(failures(strlength(failures)>0),"; ");
                end
            end
        end
    else
        if isfield(model,'initialJointSet')
            root=model.initialJointSet;
            search.initialMeasurementConditioning=struct('source',"conditionedCruiseInformation");
        else
            root=nrmmJointPrediction('rootSet',model.initialState,model.uncertainty.egoGenerator,contract);
            conditioned=nrmmJointPrediction('conditionMeasurements',root,contract,cfg);
            root=conditioned.set;search.initialMeasurementConditioning=conditioned.diagnostics;
        end
        model.uncertainty=predictiveSafetyGeometry.conditionTargetHistory( ...
            model.uncertainty,root(1:6),model.stateTime,model.target);
        optimized=[];
        if nargin<4 || isempty(seed)
            [seed,~]=localRtiInitialize(model,previous,cfg.controller.horizonSteps,[]);
            search.planner=seed.planner;
            [optimized,search.nominalProposal]=localRetainedProposal(model,previous,seed,root);
        end
        inputCandidates={seed.inputs};
        if ~isempty(optimized)
            inputCandidates=[{optimized,(optimized+seed.inputs)/2},inputCandidates];
        end
        nominalCandidates=false(1,numel(inputCandidates));nominalCandidates(1)=~isempty(optimized);
        if ~isempty(optimized),nominalCandidates(2)=true;end
        % A point-state LQR gain need not suit uncertain observer feedback.
        % If the complete tube rejects both centers, retry the SAME inputs
        % with damped ancillary feedback. Bounds, models and safety tests
        % remain unchanged; no second avoidance path is planned.
        scales=2.^-(0:5);feedbackScales=repelem(scales,numel(inputCandidates));
        inputCandidates=repmat(inputCandidates,1,numel(scales));
        nominalCandidates=repmat(nominalCandidates,1,numel(scales));
        base=struct('valid',false,'absoluteIndex',model.sampleIndex,'epochSeconds',0, ...
            'target',model.target,'targetError',localAbsoluteTargetError(model),'contract',contract, ...
            'horizonSteps',0,'normals',zeros(2,0),'posterior',[],'terminalElapsed',0, ...
            'refineTargetSupports',true);
        certificate=base;failures=strings(1,numel(inputCandidates));
        search.initialAdmission=struct('terminalChecks',0,'checkedHorizons',[]);
        % Only initial admission selects N. Keep the complete encounter seed,
        % then continue the existing route feedback until its endpoint has a
        % whole-set terminal witness. This generates no second avoidance
        % plan and performs no additional RTI solve. Later frames keep N.
        for attempt=1:numel(inputCandidates)
            inputs=inputCandidates{attempt};admissionFailure="noTerminalWitness";admitted=false;
            checked=[];
            partial=struct('sets',{{root}},'inputs',{{}},'middles',{{}},'changes',{{}}, ...
                'realArithmeticEnclosure',true,'floatingPointCertified',false);
            proposal=struct('context',{},'lower',{},'upper',{});
            k=0;nextTerminalCheck=size(inputs,2);limit=cfg.controller.maximumHorizonSteps;
            while k<limit
                k=k+1;set=partial.sets{end};
                if k<=size(inputs,2)
                    input=inputs(:,k);
                else
                    input=nonlinearBicycleModel.nominalFeedback(set.center(1:6),set.center(16:17), ...
                        model.lane,model.nominalReference,cfg, ...
                        nonlinearBicycleModel.nominalGuidanceParameters(cfg,model.nominalReference.curvature));
                end
                policy=localRetainedPolicy(set,input,cfg,contract,k==1,feedbackScales(attempt));
                try
                    image=nrmmJointPrediction('policyTube',set,policy);
                catch cause
                    if ~localCertificateFailure(cause),rethrow(cause);end
                    admissionFailure=string(cause.identifier)+": "+cause.message;break;
                end
                proposal(end+1)=policy;partial.sets{end+1}=image.sets{2}; %#ok<AGROW>
                partial.inputs{end+1}=image.inputs{1};partial.middles{end+1}=image.middles{1};
                partial.changes{end+1}=image.changes{1};
                base.horizonSteps=k;
                checkTerminal=k>=nextTerminalCheck;
                [candidate,checked]=localRetainedAssess(partial,proposal,base,model,checked,checkTerminal);
                if checkTerminal
                    search.initialAdmission.terminalChecks=search.initialAdmission.terminalChecks+1;
                    search.initialAdmission.checkedHorizons(end+1)=k;
                    % Search for an admissible initial horizon geometrically;
                    % do not repeat an expensive tail proof every 50 ms.
                    % This is a search schedule, not a prescribed duration or
                    % a change to the fixed horizon after first admission.
                    nextTerminalCheck=min(cfg.controller.maximumHorizonSteps,max(k+1,ceil(1.5*k)));
                end
                if candidate.valid
                    admitted=true;certificate=candidate;
                    certificate.feedbackScale=feedbackScales(attempt);
                    search.nominalProposal.accepted=nominalCandidates(attempt);
                    break;
                end
                admissionFailure=candidate.failureReason;
                % A later endpoint cannot repair an earlier hard violation.
                if startsWith(admissionFailure,["stateDomainAtNode","inputDomainAtHold","inputSlewAtHold"]),break;end
            end
            if ~admitted && k>=cfg.controller.maximumHorizonSteps
                admissionFailure="initialHorizonWorkLimit: "+admissionFailure;
            end
            % Admission asks for one complete witness. Comparing additional
            % ancillary gains is not the online PCBF minimization and need
            % not delay its first executable plan. Later renewals retain the
            % shifted deficit cap, including an exact zero cap once attained.
            if admitted,break;end
            if ~admitted
                failures(attempt)=sprintf('candidate %d, feedback %.3g at %.3f s: %s', ...
                    attempt,feedbackScales(attempt),k*h,admissionFailure);
            end
        end
        search.initialAdmission.failures=failures;
        assert(certificate.valid,'collisionAvoidanceController:noRetainedCertificate', ...
            'Initial joint tube rejected (%s).',join(failures,"; "));
    end
    % Optional expansion is verified BEFORE the QP. Rejection keeps the
    % exact saved bounds; a fresh absolute enclosure never overwrites them.
    budget=sum(certificate.slacks);
    % A symmetric superset cannot reduce an already positive worst-case
    % deficit. First seek a better CENTER policy; reserve optional symmetric
    % performance freedom for an admitted zero-deficit family.
    if freshAdmission && budget==0 && numel(certificate.policies)==certificate.horizonSteps
        for scale=[1,.25,.0625]
            proposal=certificate.policies;
            % Only the present feedforward correction is executed. Keep
            % the admitted future feedback policy, rather than introducing
            % independent arbitrary corrections at every future hold.
            proposal(1).lower=min(proposal(1).lower,-scale*[.02;.05]);
            proposal(1).upper=max(proposal(1).upper,scale*[.02;.05]);
            try
                candidate=localRetainedPolicyBox(certificate.tube.sets{1},proposal,certificate,model,budget);
                if candidate.valid && sum(candidate.slacks)<=budget,certificate=candidate;break;end
            catch exception
                if ~localCertificateFailure(exception),rethrow(exception);end
                search.refinementFailure=string(exception.identifier)+": "+exception.message;
            end
        end
    end
    [problem,anchor,baseInputs]=localRetainedProblem(certificate,model,contract);
    point=zeros(size(problem.lower));point(problem.slackIndices)=certificate.slacks;
    stateColumns=problem.stateIndices(:);point(stateColumns)=problem.equal(:,stateColumns)\problem.rhs;
    assert(localAffineFeasible(point,problem),'collisionAvoidanceController:lostRetainedCandidate', ...
        'The saved policy must be an explicit feasible point of the new convex problem.');
    search.retainedCandidateFeasible=true;search.primaryOptimum=sum(certificate.slacks);
    search.primaryLowerBound=search.primaryOptimum;search.slackCap=search.primaryOptimum;
    % The primary minimum is analytic: every slack is bounded below by the
    % admitted tube deficit. One QP chooses performance within that level.
    singleton=all(problem.lower(problem.inputIndices)==problem.upper(problem.inputIndices),'all');
    if singleton
        % States, inputs, safety deficits and the inactive CLF slot are all
        % uniquely fixed. The already checked point is the exact optimizer.
        trial=point;flag=1;
    else
        [trial,flag,~]=localConicSolve(problem.quadratic,problem.performanceObjective,[], ...
            problem.a,problem.b,problem.equal,problem.rhs,problem.lower,problem.upper,cfg,cfg.solver.timeLimitSeconds, ...
            min(cfg.solver.optimalityTolerance,cfg.solver.constraintTolerance));
    end
    search.solverCalls=double(~singleton)+search.nominalProposal.solverCalls;
    if flag>0 && ~isempty(trial) && all(isfinite(trial))
        % Restore EXACT certificate membership after numerical tolerances.
        trial(problem.inputIndices)=min(problem.upper(problem.inputIndices), ...
            max(problem.lower(problem.inputIndices),trial(problem.inputIndices)));
        trial(problem.slackIndices)=certificate.slacks;trial(problem.clfIndex)=0;
        trial(stateColumns)=problem.equal(:,stateColumns)\(problem.rhs-problem.equal(:,problem.inputIndices(:))*trial(problem.inputIndices(:)));
        if localAffineFeasible(trial,problem),point=trial;end
    end
    correction=reshape(point(problem.inputIndices),size(baseInputs));
    solution=localDecode(point,problem,anchor,baseInputs+correction);
    for k=1:numel(certificate.policies)
        certificate.policies(k).context.input=certificate.policies(k).context.input+correction(:,k);
        certificate.policies(k).lower=certificate.policies(k).lower-correction(:,k);
        certificate.policies(k).upper=certificate.policies(k).upper-correction(:,k);
    end
    certificate.executedInput=solution.inputs(:,1);
    solution.retainedCertificate=certificate;solution.inputBoundCorrection=0;
    solution.terminalControlRequired=isempty(certificate.policies);
    solution.minimumCollisionMargin=min(certificate.gaps);solution.secondaryObjective="trackingWithinCertifiedPolicyBox";
    solution.secondaryObjectiveValue=solution.performanceCost;solution.clfNextValue=NaN;
    solution.collisionSide=0;solution.retainCollisionSide=false;solution.collisionEscapeNormals=zeros(2,2*size(baseInputs,2)+1);
    solution=localTerminalMetrics(solution,model);solution.terminalMarginMeters=certificate.terminal.minimumGap;
    solution.terminalEnd=certificate.terminal.endReason;
    model.linearization=anchor;model.trustWarmStartAdjusted=false;model.encounterExitRecord=NaN;model.robustnessRelaxation=false;
    search.returned=true;search.converged=flag>0;search.convexSolveCompleted=flag>0;
    search.modelBuilt=true;search.linearizationCount=1+search.nominalProposal.linearizationCount;search.mode=problem.mode;
    search.pcbfStageAttempted=true;search.pcbfStageCompleted=true;
    search.secondaryStageAttempted=true;search.secondaryStageCompleted=flag>0;search.terminationReason="retainedCertificateReturned";
    search.stopReason="";search.anchorCertified=true;search.appendedTerminalSteps=certificate.horizonSteps-numel(certificate.policies);
    search.appendedBrakingSteps=search.appendedTerminalSteps;search.rtiIterations=1;
    search.rollout=struct('available',true,'reason',"wholePolicyBoxEnclosure");
    search.horizonSteps=certificate.horizonSteps;search.elapsedSeconds=toc(timer);search.source="retainedJointPolicyTube";
    search.attempts=localAttempt(search);search.selectedAttempt=1;
end

function candidate=localRetainedPolicyBox(root,policies,base,model,budget)
    % A rejected causal prefix cannot become admissible by propagating its
    % suffix. This computes the same accepted image as policyTube, while
    % avoiding growth of an already unusable optional performance box.
    tube=struct('sets',{{root}},'inputs',{{}},'middles',{{}},'changes',{{}}, ...
        'realArithmeticEnclosure',true,'floatingPointCertified',false);
    checked=[];
    for k=1:numel(policies)
        image=nrmmJointPrediction('policyTube',tube.sets{end},policies(k));
        tube.sets{end+1}=image.sets{2};tube.inputs{end+1}=image.inputs{1};
        tube.middles{end+1}=image.middles{1};tube.changes{end+1}=image.changes{1};
        [candidate,checked]=localRetainedAssess(tube,policies(1:k),base,model,checked,false);
        if candidate.failureReason~="awaitingInitialEndpoint",return;end
        if sum(max(0,-checked.gaps))>budget
            candidate.failureReason="increasedCertifiedDeficit";return;
        end
    end
    [candidate,~]=localRetainedAssess(tube,policies,base,model,checked,true);
end

function candidate=localRetainedRenewal(root,saved,optimized,fraction,base,model,feedbackScale,budget)
    % Build and check causal prefixes. A hard violation, or a deficit already
    % above the inherited total budget, cannot be repaired by any suffix.
    % Reject immediately while retaining the old feasible witness unchanged.
    cfg=model.cfg;contract=base.contract;checked=[];
    tube=struct('sets',{{root}},'inputs',{{}},'middles',{{}},'changes',{{}}, ...
        'realArithmeticEnclosure',true,'floatingPointCertified',false);
    policies=struct('context',{},'lower',{},'upper',{});
    candidate=base;candidate.valid=false;candidate.failureReason="unfinishedRenewal";
    nominal=nonlinearBicycleModel.nominalGuidanceParameters(cfg,model.nominalReference.curvature);
    for k=1:saved.horizonSteps
        set=tube.sets{end};observed=[set.center(7:11);set.center(6)];
        if k==1,observed=[contract.point;contract.currentMeasurement.yawRate];end
        if k<=numel(saved.policies)
            policy=saved.policies(k);context=policy.context;
            difference=observed-context.reference;
            difference(3)=atan2(sin(difference(3)),cos(difference(3)));
            input=context.input+double(~context.first)*context.gain*difference;
            context.contract=contract;
            if k==1
                context.input=input;context.gain=zeros(2,6);context.first=true;
            end
            policy.context=context;policy.lower=zeros(2,1);policy.upper=zeros(2,1);
        else
            input=nonlinearBicycleModel.nominalFeedback(set.center(1:6),set.center(16:17), ...
                model.lane,model.nominalReference,cfg,nominal);
            policy=localRetainedPolicy(set,input,cfg,contract,k==1,feedbackScale);
        end
        if fraction>0
            input=(1-fraction)*input+fraction*optimized(:,k);
            policy=localRetainedPolicy(set,input,cfg,contract,k==1,feedbackScale);
        end
        image=nrmmJointPrediction('policyTube',set,policy);
        policies(end+1)=policy;tube.sets{end+1}=image.sets{2}; %#ok<AGROW>
        tube.inputs{end+1}=image.inputs{1};tube.middles{end+1}=image.middles{1};
        tube.changes{end+1}=image.changes{1};
        [candidate,checked]=localRetainedAssess(tube,policies,base,model,checked,false);
        if candidate.failureReason~="awaitingInitialEndpoint",return;end
        if sum(max(0,-checked.gaps))>budget
            candidate.failureReason="increasedCertifiedDeficit";return;
        end
    end
    [candidate,~]=localRetainedAssess(tube,policies,base,model,checked,true);
end

function [inputs,diagnostics]=localRetainedProposal(model,previous,seed,root)
    % Optimize a nominal candidate before requiring whole-set admission.
    % The safety certificate is never inferred from this point subproblem.
    % The planner is reused only on the first call; later RTI proposals use
    % the ordinary shifted iterate and the already fixed horizon length.
    inputs=[];diagnostics=struct('available',false,'accepted',false,'solverCalls',0,'linearizationCount',0,'failure',"noWarmStart");
    if isempty(seed) && ~localHasWarmStart(previous),return;end
    candidate=model;candidate.cfg.controller.retainedTube=false;
    % This auxiliary proposal is still admitted by the complete nonlinear
    % certificate below. Preserve its inexpensive per-hold initialization.
    candidate.cfg.controller.jointAffinePrediction=false;
    candidate.cfg.controller.jointNonlinearPrediction=false;
    % The proposal and its admission must start with the SAME measured
    % information. Reintroducing the standalone observer box here can make
    % the auxiliary QP infeasible after the joint measurement intersection
    % has already excluded its extreme lateral velocities. The bias column
    % encloses the conditioned set about the unchanged observer estimate.
    physical=root(1:6);
    bias=physical.center-model.initialState;bias(3)=atan2(sin(bias(3)),cos(bias(3)));
    generator=[physical.generator,diag(physical.remainder),bias];
    candidate.uncertainty.egoGenerator=generator;
    candidate.uncertainty.collisionGenerator=generator;
    if candidate.uncertainty.relativeFrame,candidate.uncertainty.collisionGenerator(1:3,:)=0;end
    if isfield(candidate.uncertainty,'velocityGeometry')
        candidate.uncertainty=rmfield(candidate.uncertainty,'velocityGeometry');
    end
    diagnostics.initialPhysicalRadius=sum(abs(generator),2);
    % A proposal must react to the same persistent target family that its
    % nonlinear admission will check. The legacy per-hold reset assumes
    % future target observations and can propose dangerously late passing.
    candidate.retainedProposal=true;
    if ~isempty(seed),candidate.probeReference=seed;end
    try
        [solution,search]=solvePredictiveControl(candidate,previous,tic);
        diagnostics.solverCalls=search.solverCalls;
        diagnostics.linearizationCount=search.linearizationCount;
        if isfield(search,'stages'),diagnostics.stages=search.stages;end
        if isempty(solution),diagnostics.failure=search.terminationReason;return;end
        inputs=solution.inputs;diagnostics.available=true;diagnostics.failure="";
    catch exception
        if ~startsWith(string(exception.identifier),"collisionAvoidanceController:"),rethrow(exception);end
        diagnostics.failure=string(exception.identifier)+": "+exception.message;
    end
end

function policy=localRetainedPolicy(set,input,cfg,contract,first,feedbackScale)
    if nargin<6,feedbackScale=1;end
    reference=[set.center(7:11);set.center(6)];gain=zeros(2,6);
    if ~first
        anchor=struct('states',[set.center(1:6),set.center(1:6)],'inputs',input);
        gain=feedbackScale*nrmmJointPrediction('feedback',anchor,cfg,cfg.controller.sampleTime);
        estimate=set([7:11,6]);reserve=sum(abs(gain*estimate.generator),2)+abs(gain)*estimate.remainder;
        reserve=reserve+abs(gain(:,6))*contract.noiseRadius(7);
        if reserve(2)>0
            authority=min(cfg.actuation.brakingRatioMaximum-input(2),input(2)-cfg.actuation.brakingRatioMinimum);
            % policyTube's 1e-4 support-budget reduction can enlarge a
            % coordinate radius by at most twice that fraction. Account
            % for this REPRESENTATION reserve before reaching an exact
            % actuator endpoint; the physical input limits stay [-1,1].
            gain(2,:)=gain(2,:)*min(1,max(0,authority)*(1-32*eps)/((1+2e-4)*reserve(2)));
        end
    end
    context=struct('cfg',cfg,'contract',contract,'reference',reference,'input',input, ...
        'gain',gain,'duration',cfg.controller.sampleTime,'first',first);
    policy=struct('context',context,'lower',zeros(2,1),'upper',zeros(2,1));
end

function errorSet=localAbsoluteTargetError(model)
    errorSet=model.uncertainty.target;
    if model.uncertainty.relativeFrame
        g=model.uncertainty.egoGenerator;position=sum(vecnorm(g(1:2,:),2,1));yaw=sum(abs(g(3,:)));
        errorSet.positionRadius=errorSet.positionRadius+position ...
            +2*norm(model.target(1:2)-model.initialState(1:2))*sin(min(pi,yaw)/2);
        errorSet.courseRadius=errorSet.courseRadius+yaw;
    end
end

function certificate=localRetainedTargetUpdate(certificate,model)
    % Keep both the old target family and every information refinement that
    % improves a retained support. Their intersection contains the physical
    % target under the declared measurement/model contract. No new terminal
    % claim is inferred from a re-centered target point.
    errorSet=localAbsoluteTargetError(model);shape=[model.cfg.vehicle.length/2; ...
        model.cfg.vehicle.width/2;model.cfg.vehicle.rectangleOffset];
    changed=false;before=certificate.gaps;
    for k=1:numel(certificate.tube.sets)
        normal=certificate.normals(:,k);
        if norm(normal)<.5,continue;end
        target=predictiveSafetyGeometry.targetPoseSet(model.target,errorSet,(k-1)*model.cfg.controller.sampleTime);
        gap=predictiveSafetyGeometry.setSeparation(certificate.tube.sets{k}(1:3), ...
            shape,target,model.target(8:11),normal);
        % The old separator remains available. A new support is admitted
        % only when it proves a stronger bound for the SAME saved ego tube
        % and measurement-consistent target family. Thus the certificate
        % deficit cannot increase; no new passing maneuver is selected.
        [alternative,direction]=predictiveSafetyGeometry.setSeparation( ...
            certificate.tube.sets{k}(1:3),shape,target,model.target(8:11));
        if alternative>gap,gap=alternative;normal=direction;end
        if gap<0
            covered=predictiveSafetyGeometry.targetFamilySeparation( ...
                certificate.tube.sets{k}(1:3),shape,model.target,errorSet,(k-1)*model.cfg.controller.sampleTime);
            gap=max(gap,covered);
        end
        if gap>certificate.gaps(k),certificate.gaps(k)=gap;changed=true;end
        if gap>before(k),certificate.normals(:,k)=normal;end
    end
    if changed
        record=struct('sampleIndex',model.sampleIndex,'target',model.target, ...
            'errorSet',errorSet,'improvedNodes',find(certificate.gaps>before), ...
            'normals',certificate.normals(:,certificate.gaps>before));
        if ~isfield(certificate,'targetRefinements'),certificate.targetRefinements=record;
        else,certificate.targetRefinements(end+1)=record;end
        certificate.slacks=min(certificate.slacks,max(0,-certificate.gaps));
    end
end

function [candidate,checked]=localRetainedAssess(tube,policies,base,model,checked,checkTerminal)
    candidate=base;candidate.valid=false;candidate.failureReason="";
    cfg=model.cfg;n=numel(policies);h=cfg.controller.sampleTime;
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    gaps=Inf(1,n+1);normals=zeros(2,n+1);
    if nargin<5 || isempty(checked)
        checked=struct('gaps',[],'normals',zeros(2,0),'holds',0);
    end
    if nargin<6,checkTerminal=true;end
    % Initial causal construction only appends a hold. Its earlier sets,
    % controls, target epoch and supports are unchanged, so reuse those
    % exact checks. Every new policy proposal starts with an empty cache.
    count=numel(checked.gaps);
    gaps(1:count)=checked.gaps;normals(:,1:count)=checked.normals;
    for k=1:n+1
        if k>count
            state=tube.sets{k}(1:6);
            if ~predictiveSafetyGeometry.stateSetAdmissible(state,cfg)
                [lo,hi]=bounds(state(4:6));
                side=tan(cfg.model.sideslipMaximum);
                [~,left]=bounds(state(5)-side*state(4));[~,right]=bounds(-state(5)-side*state(4));
                candidate.failureReason=sprintf(['stateDomainAtNode%d: vx=[%.4g,%.4g], ' ...
                    'vy=[%.4g,%.4g], r=[%.4g,%.4g], sideslipUpper=%.4g'], ...
                    k,lo(1),hi(1),lo(2),hi(2),lo(3),hi(3),max(left,right));return;
            end
            normal=[];
            if k<=size(base.normals,2) && norm(base.normals(:,k))>.5,normal=base.normals(:,k);end
            [gaps(k),normals(:,k)]=localRetainedTargetGap(state(1:3),shape,base,model,k,normal);
        end
        if k<=n && k>checked.holds
            [lo,hi]=bounds(tube.inputs{k});
            if lo(2)<cfg.actuation.brakingRatioMinimum || hi(2)>cfg.actuation.brakingRatioMaximum
                candidate.failureReason=sprintf('inputDomainAtHold%d: b=[%.17g,%.17g]',k,lo(2),hi(2));return;
            end
            if isfield(tube,'changes')
                [lo,hi]=bounds(tube.changes{k}(2));
            else
                % Legacy stored tubes have different primitive coordinates
                % before and after compression. Independent ranges are a
                % sound fallback; equal column numbers imply no identity.
                [oldLo,oldHi]=bounds(tube.sets{k}(17));
                [newLo,newHi]=bounds(tube.inputs{k}(2));lo=newLo-oldHi;hi=newHi-oldLo;
            end
            if max(abs([lo,hi]))>cfg.model.brakingRatioRateMaximum*h
                candidate.failureReason="inputSlewAtHold"+k;return;
            end
        end
    end
    checked=struct('gaps',gaps,'normals',normals,'holds',n);
    if ~checkTerminal,candidate.failureReason="awaitingInitialEndpoint";return;end
    context=terminalSafeSet.context(model);context.required=0;context.inflation=0;
    context.heldInputSet=tube.sets{end}(16:17);
    lower=zeros(2,n+1);upper=lower;
    for k=1:n+1
        [lower(:,k),upper(:,k)]=bounds(tube.sets{k}(1:2));
    end
    context.encounterHistory=struct('time',base.epochSeconds+(0:n)*h, ...
        'positionLower',lower,'positionUpper',upper);
    context.targetEpoch=base.target;context.targetEpochIndex=0;context.sampleIndex=0;
    [lo,hi]=bounds(tube.sets{end}(17));
    if max(abs(context.brakingRatio-[lo,hi]))>context.brakingChangeMaximum
        candidate.failureReason="terminalInputSlew";return;
    end
    terminal=terminalSafeSet.certifySet(context,tube.sets{end}(1:6),base.target,base.targetError,base.epochSeconds+n*h);
    if ~terminal.valid && isfield(base,'targetRefinements')
        for j=numel(base.targetRefinements):-1:1
            family=base.targetRefinements(j);age=(model.sampleIndex-family.sampleIndex)*h;
            alternative=context;alternative.targetEpoch=family.target;
            alternative.encounterHistory.time=context.encounterHistory.time+age-base.epochSeconds;
            terminal=terminalSafeSet.certifySet(alternative,tube.sets{end}(1:6), ...
                family.target,family.errorSet,age+n*h);
            if terminal.valid,break;end
        end
    end
    if ~terminal.valid,candidate.failureReason="terminal:"+terminal.endReason;return;end
    candidate.valid=true;candidate.tube=tube;candidate.policies=policies;candidate.terminal=terminal;
    candidate.gaps=gaps;candidate.normals=normals;candidate.slacks=max(0,-gaps);
end

function [gap,normal]=localRetainedTargetGap(ego,shape,base,model,node,normal)
    h=model.cfg.controller.sampleTime;time=base.epochSeconds+(node-1)*h;
    target=predictiveSafetyGeometry.targetPoseSet(base.target,base.targetError,time);
    [gap,normal]=predictiveSafetyGeometry.setSeparation(ego,shape,target,base.target(8:11),normal);
    if gap<0 && isfield(base,'targetRefinements')
        % Every recorded family contains the actual target at the same
        % absolute time. The largest lower gap is valid on their intersection.
        % Recompute the supports for the NEW ego image; do not copy old gaps.
        for j=numel(base.targetRefinements):-1:1
            family=base.targetRefinements(j);
            age=(model.sampleIndex-family.sampleIndex+node-1)*h;
            target=predictiveSafetyGeometry.targetPoseSet(family.target,family.errorSet,age);
            [value,direction]=predictiveSafetyGeometry.setSeparation(ego,shape,target,family.target(8:11));
            if value<0 && isfield(base,'refineTargetSupports') && base.refineTargetSupports
                covered=predictiveSafetyGeometry.targetFamilySeparation(ego,shape,family.target,family.errorSet,age);
                value=max(value,covered);
            end
            if value>gap,gap=value;normal=direction;end
            if gap>=0,break;end
        end
    end
    if gap<0 && isfield(base,'refineTargetSupports') && base.refineTargetSupports
        % A union of support certificates is a distance lower bound for ALL
        % target members, not a single half-space for their convex hull.
        value=predictiveSafetyGeometry.targetFamilySeparation(ego,shape,base.target,base.targetError,time);
        gap=max(gap,value);
    end
end

function [p,anchor,inputs]=localRetainedProblem(certificate,model,contract)
    n=certificate.horizonSteps;m=numel(certificate.policies);cfg=model.cfg;
    states=zeros(6,n+1);inputs=zeros(2,n);lower=zeros(2,n);upper=lower;
    for k=1:m+1,states(:,k)=certificate.tube.sets{k}.center(1:6);end
    if m<n
        entry=certificate.terminal.entry.center;speed=hypot(entry(4),entry(5));
        beta=atan2(entry(5),entry(4));curvature=entry(6)/max(speed,eps);
        for j=double(m>0):n-m
            time=certificate.terminalElapsed+j*cfg.controller.sampleTime;
            pose=nrmmJointEnclosure('terminalPose',nrmmSet(entry),certificate.terminal.context,time);
            v=max(0,speed-time*certificate.terminal.context.deceleration);
            states(:,m+j+1)=[pose.center;v*cos(beta);v*sin(beta);v*curvature];
        end
    end
    for k=1:n
        if k<=m
            policy=certificate.policies(k);context=policy.context;
            observed=[certificate.tube.sets{k}.center(7:11);certificate.tube.sets{k}.center(6)];
            if k==1,observed=[contract.point;contract.currentMeasurement.yawRate];end
            delta=observed-context.reference;delta(3)=atan2(sin(delta(3)),cos(delta(3)));
            inputs(:,k)=context.input+double(~context.first)*context.gain*delta;
            lower(:,k)=policy.lower;upper(:,k)=policy.upper;
        else
            inputs(:,k)=[inputs(1,max(1,k-1));certificate.terminal.context.brakingRatio];
        end
    end
    anchor=struct('states',states,'inputs',inputs);ix=reshape(1:6*(n+1),6,[]);
    iu=reshape(ix(end)+(1:2*n),2,[]);is=iu(end)+(1:n+1);ic=is(end)+1;
    equal=sparse(6*(n+1),ic);equal(1:6,ix(:,1))=eye(6);rhs=zeros(6*(n+1),1);
    rhs(1:6)=model.initialState-states(:,1);stages=struct('a',{},'b',{});
    for k=1:n
        if k<=m
            [seedLo,seedHi]=nonlinearBicycleModel.inputBounds(cfg,true);u=min(seedHi,max(seedLo,inputs(:,k)));
            try
                [next,a,b]=nonlinearBicycleModel.sample(states(:,k),u,cfg);next=next+b*(inputs(:,k)-u);
            catch exception
                if ~localCertificateFailure(exception),rethrow(exception);end
                % A performance tangent is optional. Its failure cannot
                % invalidate the independently certified policy family.
                next=states(:,k+1);a=eye(6);b=zeros(6,2);
            end
        else
            next=states(:,k+1);a=eye(6);b=zeros(6,2);
        end
        rows=6*k+(1:6);equal(rows,ix(:,k+1))=eye(6);equal(rows,ix(:,k))=-a;equal(rows,iu(:,k))=-b;
        rhs(rows)=next-states(:,k+1);stages(k)=struct('a',a,'b',b);
    end
    p=struct('a',sparse(0,ic),'b',zeros(0,1),'equal',equal,'rhs',rhs,'cones',[], ...
        'lower',-Inf(ic,1),'upper',Inf(ic,1),'stateIndices',ix,'inputIndices',iu,'slackIndices',is,'clfIndex',ic, ...
        'safetyObjective',zeros(ic,1),'primaryLowerBound',sum(certificate.slacks), ...
        'maximumCollisionTighteningMeters',NaN,'encounterExit',0, ...
        'trustRadii',[Inf(6,n);max(abs(lower),abs(upper))],'trustSampledErrorRatio',zeros(1,n),'stages',stages);
    p.lower(iu)=lower;p.upper(iu)=upper;p.lower(is)=certificate.slacks;p.upper(is)=certificate.slacks;
    p.safetyObjective(is)=1;p=localSelectObjective(p,anchor,model);
end

function expected=localCertificateFailure(exception)
    id=string(exception.identifier);
    expected=startsWith(id,"nrmmJointEnclosure:") || startsWith(id,"nrmmSet:") ...
        || startsWith(id,"nrmmJointPrediction:uncertified") || any(id==[ ...
        "collisionAvoidanceController:nonlinearDomain","collisionAvoidanceController:invalidTireOperatingPoint", ...
        "collisionAvoidanceController:singularTireLinearization"]);
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
