function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl Predictive safety and research comparison solvers.
% The compact output-feedback path conditions the finite physical hypotheses,
% optimizes shared actuator commands with a condensed nonlinear PCBF program,
% and retains a separately checked executable plan. Its fixed backup law is
% propagated to rest and followed by a complete parked-target track check.
% Primary safety deficit precedes performance; an unsafe optimization iterate
% can warm-start further search but cannot supply an executed command.
% State-input, dynamic-memory and retained-tube paths are research comparators.
% The probe operation exposes their reference, formulation and solve routines.
    if (isstring(model) || ischar(model)) && string(model)=="outputFeedback"
        [solution,search,model]=localOutputFeedback(previousState);return;
    end
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
            projection=predictiveSafetyGeometry.project(model.initialState(1:2),model.lane);
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
    projection=predictiveSafetyGeometry.project(model.target(1:2),model.lane);
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
    % Do not change longitudinal encounter timing while the old plan is
    % still reducing a positive safety deficit. Retract only after zero.
    retract=inherited && isfield(previous,'safetyBudget') ...
        && previous.safetyBudget<=cfg.solver.constraintTolerance;
    for index=1:count
        if index<=size(old,2)
            u=old(:,index);
            if retract,u=localWarmStartInput(states(:,index),u,cfg);end
            % Re-anchor the shifted controls to the new measured state.
            % The nonlinear rollout prepares the next linearization; it does
            % not modify or veto the optimizer's subsequently issued command.
            next=nonlinearBicycleModel.sample(states(:,index),u,cfg);
        else
            % This is a numerical linearization seed, not the theorem's
            % certified terminal extension. Use the bounded nonlinear path
            % guidance also when the endpoint belongs to the terminal set.
            % Extrapolating the local cruise gain at a remote endpoint can
            % put front slip near pi/2 before robust tightening is applied.
            % Terminal feasibility remains a constraint of the optimization.
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

function input=localWarmStartInput(x,input,cfg)
    % Retract a saturated steering seed to the small-slip branch when the
    % SAME body-frame front lateral force can be realized there. The signed
    % longitudinal ratio stays fixed. Only the initialization changes: the
    % QP retains its full physical input bounds and original Fiala model.
    if x(4)<=cfg.model.kinematicSpeedRange(2),return;end
    course=atan2(x(5)+cfg.vehicle.lf*x(6),x(4));
    rear=atan2(x(5)-cfg.vehicle.lr*x(6),x(4));
    [force,slope]=nonlinearBicycleModel.tireForces([course-input(1);rear],input(2),cfg);
    if slope(1)<0,return;end
    tire=nonlinearBicycleModel.tireParameters(cfg);fx=input(2)*tire.longitudinalForceScale(1);
    desired=fx*sin(input(1))+force(1)*cos(input(1));
    limit=atan(3*tire.longitudinalForceScale(1)*sqrt(1-input(2)^2)/tire.corneringStiffness(1));
    [lower,upper]=nonlinearBicycleModel.inputBounds(cfg,true);
    low=max(lower(1),course-limit);high=min(upper(1),course+limit);
    steering=min(high,max(low,course));
    for iteration=1:8
        [force,slope]=nonlinearBicycleModel.tireForces([course-steering;rear],input(2),cfg);
        value=fx*sin(steering)+force(1)*cos(steering);
        if abs(value-desired)<=1e-8*max(1,tire.longitudinalForceScale(1))
            if slope(1)<0 && abs(course-steering)<abs(course-input(1)),input(1)=steering;end
            return;
        end
        derivative=(fx-slope(1))*cos(steering)-force(1)*sin(steering);
        if derivative<=eps,return;end
        steering=min(high,max(low,steering+(desired-value)/derivative));
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
        robustRelaxation=robustRelaxation || any(structfun(@(value)isnumeric(value) && any(value(:)>0),model.uncertainty.target));
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
    jointTube=[];enclosureDomain=[];preparedTrust=[];preparedTrustRatio=[];
    if isfield(model,'jointPredictionTube'),model=rmfield(model,'jointPredictionTube');end
    jointRequested=cfg.controller.jointAffinePrediction || cfg.controller.jointNonlinearPrediction;
    if jointRequested && isfield(model.uncertainty,'jointPrediction')
        options=struct();
        if cfg.controller.jointNonlinearPrediction
            enclosureDomain=localEnclosureDomain(anchor,model,stages,durations);
            options=struct('nonlinearEnclosure',true,'decisionLower',enclosureDomain.lower, ...
                'decisionUpper',enclosureDomain.upper,'stages',stages);
        end
        contract=model.uncertainty.jointPrediction;
        coupled=~cfg.controller.jointNonlinearPrediction && ~isempty(model.target) ...
            && ~isempty(model.uncertainty.target) && isfield(contract,'targetObserver') ...
            && contract.currentMeasurement.radarDetectionAvailable;
        if coupled
            root=[];if isfield(model,'coupledPosterior'),root=model.coupledPosterior;end
            if cfg.rti.trustRegion.enabled
                preparedTrust=Inf(8,count);preparedTrustRatio=zeros(1,count);
                for k=1:count
                    stageCfg=cfg;stageCfg.controller.sampleTime=durations(k);
                    [preparedTrust(:,k),info]=nonlinearBicycleModel.trustRegion( ...
                        anchor.states(:,k),anchor.inputs(:,k),stageCfg,stages(k).next,stages(k).a,stages(k).b);
                    preparedTrustRatio(k)=info.sampledErrorRatio;
                end
                options.feedbackInputRadius=preparedTrust(7:8,:);
            end
            jointTube=nrmmJointPrediction('coupledPropagate',anchor,cfg,initialGenerator, ...
                contract,durations,model.target,model.uncertainty.target,root,options);
        else
            jointTube=nrmmJointPrediction('propagate',anchor,cfg,initialGenerator, ...
                contract,durations,options);
        end
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
    if ~isempty(jointTube) || (isfield(model,'retainedProposal') && model.retainedProposal) ...
            || (~isempty(model.uncertainty.target) && isfield(model.uncertainty.target,'family'))
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
        uncertaintyPrediction.futureTargetObserverDynamicsIncluded=isfield(jointTube,'futureTargetObserverDynamicsIncluded') ...
            && jointTube.futureTargetObserverDynamicsIncluded;
        uncertaintyPrediction.targetFeedbackIncluded=isfield(jointTube,'targetFeedbackIncluded') && jointTube.targetFeedbackIncluded;
        if uncertaintyPrediction.futureTargetObserverDynamicsIncluded
            uncertaintyPrediction.targetObserverErrorRadius=jointTube.targetObserverErrorRadius;
        end
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
            if isempty(preparedTrust)
                [radius,trustInfo]=nonlinearBicycleModel.trustRegion( ...
                    anchor.states(:,k),anchor.inputs(:,k),stageCfg,stages(k).next,stages(k).a,stages(k).b);
            else
                radius=preparedTrust(:,k);trustInfo=struct('sampledErrorRatio',preparedTrustRatio(k));
            end
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
    projection=predictiveSafetyGeometry.project(reshape(points,2,[]),model.lane);
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
    if isfield(model,'jointPredictionTube') && isfield(model.jointPredictionTube,'jointPoseGenerator')
        data.jointPoseGenerator=model.jointPredictionTube.jointPoseGenerator{point};
    end
    [values,jacobianFull,tightening]=localPlaneRows(x,witness,data);
    jacobian=jacobianFull(:,1:3);witnessJacobian=jacobianFull(:,4:5);
end

function [values,jacobian,tightening]=localPlaneRows(x,witness,data)
    data.egoYaw=x(3);
    reserve=localPlaneReserve(witness(1),data);
    % The engineering support oracle is piecewise differentiable. Both
    % backends use the same symmetric derivative at support-feature ties.
    % It is not a smoothness or nonlinear-enclosure certificate at those ties.
    step=1e-6;
    derivative=(localPlaneReserve(witness(1)+step,data)-localPlaneReserve(witness(1)-step,data))/(2*step);
    [values,jacobian]=predictiveSafetyGeometry.separationWitnessRows( ...
        x(1:3),data.shape,data.target(1:3),data.target(8:11),witness,reserve,derivative);
    if isfield(data,'jointPoseGenerator')
        plus=data;minus=data;plus.egoYaw=data.egoYaw+step;minus.egoYaw=data.egoYaw-step;
        yawDerivative=(localPlaneReserve(witness(1),plus)-localPlaneReserve(witness(1),minus))/(2*step);
        jacobian(1:4,3)=jacobian(1:4,3)-yawDerivative;
    end
    tightening=reserve;
end

function reserve=localPlaneReserve(angle,data)
    normal=[cos(angle),sin(angle)];q=data.target;shape=data.shape;
    if isfield(data,'jointPoseGenerator')
        reserve=predictiveSafetyGeometry.jointPoseSupport(normal,[0;0;data.egoYaw],shape, ...
            q(1:3),q(8:11),data.jointPoseGenerator);
        return;
    end

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
    if isfield(model,'jointPredictionTube') && isfield(model.jointPredictionTube,'jointPoseGenerator')
        % Joint pairwise rows use their shared ego/target generators directly.
        % The terminal module still uses an absolute ego outer set plus the
        % target family; do not assume compressed columns are initial poses.
        return;
    end
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

function [result,diagnostics,problem] = localOutputFeedback(request)
    % One joint inference/control runtime; the state-input solver above is
    % retained as a research comparator. No NRMM routine is called here.
    cfg=collisionAvoidanceControllerConfig(request.cfg);
    diagnostics=[];problem=[];
    switch string(request.operation)
        case "features"
            result=localOfFeatures(request.samples,cfg);
        case "memory"
            result=localOfMemory(request.policy,request.memory,request.samples,cfg);
        case "sensor"
            result=localOfSensor(request.hypothesis,request.input,cfg,request.time);
        case "advance"
            result=localOfAdvance(request.hypothesis,request.input,cfg,request.duration,request.endOfHold);
        case "condition"
            [result,diagnostics]=localOfCondition(request.hypotheses,request.block,cfg,request.initial);
        case "window"
            result=localOfWindow(localOfValidateHypotheses(request.hypotheses,cfg),0,cfg);
        case "validateInputSequence"
            hypotheses=localOfValidateHypotheses(request.hypotheses,cfg);
            validateattributes(request.inputs,{'double'},{'2d','nrows',2,'nonempty','finite','real'});
            count=size(request.inputs,2);nodes=min(count,cfg.outputFeedback.controlNodes);
            near=min(nodes-1,cfg.outputFeedback.nearControlNodes);
            boundaries=[0:near,near+round((1:nodes-near)*(count-near)/(nodes-near))];
            kernel=localCompactKernel(cfg,diff(boundaries),~isempty(hypotheses(1).target));
            [valid,traces,violation]=localCompactValidate(request.inputs,hypotheses,request.input,kernel,cfg,true);
            result=struct('accepted',valid,'maximumViolation',violation,'trajectories',{traces});
        case "evaluateControlSequence"
            hypotheses=localOfValidateHypotheses(request.hypotheses,cfg);
            controls=request.inputs;holds=request.holds;nominal=ceil(numel(hypotheses)/2);
            validateattributes(holds,{'double'},{'row','integer','positive','numel',size(controls,2)});
            validateattributes(controls,{'double'},{'2d','nrows',2,'nonempty','finite','real'});
            x=hypotheses(nominal).ego;
            stub=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
            reference=[-1000,0;1000,0];if isfield(request,'reference'),reference=request.reference;end
            [~,lane]=readControllerInputs(stub,[],reference,cfg);
            count=max(round(cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod),nonlinearBicycleModel.meshCount(cfg));
            kernel=localCompactKernel(cfg,holds,~isempty(hypotheses(1).target),count);
            model=localCondensedModel(cfg,hypotheses,request.input,holds,lane,nominal,kernel);
            result=localCondensedOracle(controls(:),model);
        case {"predict","linearize"}
            problem=localOfProblem(request.hypotheses,request.memory,request.input, ...
                request.policies,request.reference,cfg);
            if string(request.operation)=="linearize"
                problem.prepareOnly=true;
                [~,result]=localOfOptimize(problem,false,tic);
            else
                result=problem.nominalTrajectories;
            end
        case {"step","linearizeStep"}
            [result,diagnostics,problem]=localOfStep(request,cfg);
        otherwise
            error('collisionAvoidanceController:invalidOutputFeedbackOperation','Unknown joint operation.');
    end
end

function [result,diagnostics,problem] = localOfStep(request,cfg)
    if string(cfg.outputFeedback.method)=="compactNlp" && string(request.operation)=="step"
        [result,diagnostics]=localCompactStep(request,cfg);problem=[];return;
    end
    timer=tic;block=request.block;prior=request.previousState;
    if isempty(request.reference),request.reference=[-1000,0;1000,0];end
    localOfValidateBlock(block,cfg);
    assert(~cfg.controller.enforceRoadBoundaries, ...
        'collisionAvoidanceController:unsupportedJointRoadConstraint', ...
        'The joint deployment does not implement hard road-boundary inequalities.');
    count=cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod;
    assert(abs(count-round(count))<1e-10 && count>=2 && mod(round(count),2)==0, ...
        'collisionAvoidanceController:invalidSensorGrid', ...
        'A control hold must contain an even, positive number of sensor periods.');
    initial=isempty(prior);
    if initial
        assert(isfield(block,'initialHypotheses') && ~isempty(block.initialHypotheses), ...
            'collisionAvoidanceController:missingInitialInformation', ...
            'An explicit joint initial hypothesis contract is required; a single sensor block cannot identify it.');
        hypotheses=localOfValidateHypotheses(block.initialHypotheses,cfg);
        assert(numel(block.samples)==1 && block.samples.time==block.time, ...
            'collisionAvoidanceController:invalidInitialBlock','Initialization uses one current-boundary reading.');
        memory=zeros(cfg.outputFeedback.memoryDimension,1);
        if isfield(block,'initialMemory'),memory=block.initialMemory;end
        validateattributes(memory,{'double'},{'size',[cfg.outputFeedback.memoryDimension,1],'finite','real'});
        assert(all(abs(memory)<=1),'collisionAvoidanceController:invalidMemory','Memory must belong to its invariant cube.');
        policies=[];
        if isfield(block,'initialPolicies'),policies=block.initialPolicies;end
        inheritedCount=numel(policies);
    else
        assert(isstruct(prior) && isfield(prior,'kind') && prior.kind=="jointOutputFeedback-v1", ...
            'collisionAvoidanceController:incompatibleContinuation','Expected a joint output-feedback continuation.');
        assert(isequaln(prior.configuration,cfg) && isequaln(prior.reference,request.reference), ...
            'collisionAvoidanceController:changedContinuationProblem', ...
            'Continuation must retain its model, uncertainty contract, memory coordinates, and reference.');
        assert(~isfield(block,'initialHypotheses') && ~isfield(block,'initialMemory') ...
            && ~isfield(block,'initialPolicies'), ...
            'collisionAvoidanceController:informationReset','A continuation cannot reset its information or memory.');
        times=prior.time+(1:round(count))*cfg.outputFeedback.sensorPeriod;
        assert(numel(block.samples)==round(count) && ...
            max(abs([block.samples.time]-times))<1e-9 && abs(block.time-times(end))<1e-9, ...
            'collisionAvoidanceController:invalidSensorGrid','Receive the complete consecutive sensor block.');
        assert(norm(block.heldInput-prior.appliedInput,Inf)<1e-12, ...
            'collisionAvoidanceController:actuationMismatch','The received block must use the previously committed actuator command.');
        hypotheses=prior.hypotheses;
        memory=localOfMemory(prior.committedPolicy,prior.memory,block.samples,cfg);
        policies=prior.policies(2:end);inheritedCount=numel(prior.policies);
    end
    inferenceTimer=tic;
    [hypotheses,conditioning]=localOfCondition(hypotheses,block,cfg,initial);
    inferenceSeconds=toc(inferenceTimer);
    preparationTimer=tic;
    window=localOfWindow(hypotheses,inheritedCount,cfg);
    if isempty(policies)
        policies=localOfInitialPolicies(cfg,hypotheses,request.reference,block.heldInput,window.horizonSteps);
    else
        policies=[policies,repmat(localOfBackup(cfg),1,window.horizonSteps-numel(policies))];
    end
    capacity=numel(hypotheses);
    if ~initial && isfield(prior,'nlpHypothesisCapacity'),capacity=prior.nlpHypothesisCapacity;end
    problem=localOfProblem(hypotheses,memory,block.heldInput,policies,request.reference,cfg,capacity);
    preparationSeconds=toc(preparationTimer);
    if string(request.operation)=="linearizeStep"
        problem.prepareOnly=true;
        [~,result]=localOfOptimize(problem,initial,timer);
        diagnostics=[];return;
    end
    warm=[];
    if ~initial && isfield(prior,'nlpWarmStart') && ...
            isequal(conditioning.retainedIndices(:),(1:numel(prior.hypotheses))')
        warm=prior.nlpWarmStart;
    end
    if string(cfg.outputFeedback.method)=="ipopt"
        [point,diagnostics,trajectory,warm]=localOfNonlinearOptimize(problem,initial,timer,warm);
    else
        [point,diagnostics,trajectory]=localOfOptimize(problem,initial,timer);
    end
    diagnostics.informationUpdateSeconds=inferenceSeconds;
    diagnostics.predictionPreparationSeconds=preparationSeconds;
    diagnostics.predictionWindow=window;
    accepted=localOfUnpackPlan(point,problem);input=accepted(1).v+accepted(1).K*memory;
    % Commit the full first controller, including its next-block memory map.
    state=struct('kind',"jointOutputFeedback-v1",'time',block.time, ...
        'memory',memory,'hypotheses',hypotheses,'appliedInput',input, ...
        'committedPolicy',accepted(1),'policies',accepted,'configuration',cfg, ...
        'reference',request.reference,'conditioning',conditioning);
    if string(cfg.outputFeedback.method)=="ipopt"
        state.nlpWarmStart=warm;state.nlpHypothesisCapacity=capacity;
    end
    diagnostics.conditioning=conditioning;diagnostics.elapsedSeconds=toc(timer);
    diagnostics.deadlineMissed=diagnostics.elapsedSeconds>diagnostics.deadlineSeconds;
    diagnostics.memoryDimension=numel(memory);diagnostics.hypothesisCount=numel(hypotheses);
    diagnostics.scenarioCount=numel(problem.scenarios);
    diagnostics.activeScenarioCount=numel(hypotheses)*size(cfg.outputFeedback.noiseDirections,2);
    diagnostics.safetyScope="finiteJointScenariosLinearizedRti";
    if string(cfg.outputFeedback.method)=="ipopt"
        diagnostics.safetyScope="finiteJointScenariosNonlinearRolloutValidated";
    end
    diagnostics.robustNonlinearSafetyCertified=false;
    diagnostics.recursiveFeasibilityCertified=false;
    diagnostics.optimizationReturned=true;
    diagnostics.wholeHoldReserveScope="engineeringIntersampleMarginNoCertificate";
    diagnostics.predictionModel="jointDynamicOutputFeedbackSparseMultipleShooting";
    diagnostics.sensorContract="positionOnlyGnssBodyImuGyroRelativeRadar";
    diagnostics.previousInputUsedForMeasurements=block.heldInput;
    command=struct('actuatorInput',input,'steeringAngle',input(1),'brakingRatio',input(2));
    prediction=struct('metadata',diagnostics,'scenarios',{trajectory}, ...
        'policies',accepted,'memory',memory,'information',hypotheses);
    result=struct('command',command,'inputs',trajectory{1}.inputs, ...
        'prediction',prediction,'state',state);
end

function localOfValidateBlock(block,cfg)
    assert(isstruct(block) && isscalar(block) && ...
        all(isfield(block,{'kind','time','heldInput','samples'})), ...
        'collisionAvoidanceController:invalidSensorBlock','A timestamped raw block and previous input are required.');
    validateattributes(block.time,{'double'},{'scalar','real','finite'});
    validateattributes(block.heldInput,{'double'},{'size',[2,1],'real','finite'});
    assert(abs(block.heldInput(2))<=1,'collisionAvoidanceController:invalidSensorBlock','Invalid previous force ratio.');
    assert(isstruct(block.samples) && ~isempty(block.samples), ...
        'collisionAvoidanceController:invalidSensorBlock','The sensor block cannot be empty.');
    for j=1:numel(block.samples)
        y=block.samples(j);
        assert(all(isfield(y,{'time','position','acceleration','yawRate','relativePosition','radarAvailable'})), ...
            'collisionAvoidanceController:invalidSensorBlock','Every raw reading must include its detection flag.');
        validateattributes(y.time,{'double'},{'scalar','real','finite'});
        validateattributes(y.position,{'double'},{'size',[2,1],'real','finite'});
        validateattributes(y.acceleration,{'double'},{'size',[2,1],'real','finite'});
        validateattributes(y.yawRate,{'double'},{'scalar','real','finite'});
        validateattributes(y.radarAvailable,{'logical'},{'scalar'});
        if y.radarAvailable
            validateattributes(y.relativePosition,{'double'},{'size',[2,1],'real','finite'});
        end
    end
    assert(cfg.model.speedMinimum==0,'collisionAvoidanceController:terminalRestExcluded', ...
        'The joint deployment uses a forward-only physical domain with zero minimum speed.');
end

function hypotheses = localOfValidateHypotheses(hypotheses,cfg)
    assert(isstruct(hypotheses) && ~isempty(hypotheses) ...
        && numel(hypotheses)<=cfg.outputFeedback.maximumHypotheses, ...
        'collisionAvoidanceController:invalidInitialInformation','Supply a nonempty bounded-size joint ensemble.');
    hasTarget=false(1,numel(hypotheses));
    for j=1:numel(hypotheses)
        assert(all(isfield(hypotheses(j),{'ego','target','parameters','sensorBias'})), ...
            'collisionAvoidanceController:invalidInitialInformation', ...
            'Each hypothesis needs ego, target, persistent parameter scales, and persistent sensor bias.');
        validateattributes(hypotheses(j).ego,{'double'},{'size',[6,1],'real','finite'});
        validateattributes(hypotheses(j).parameters,{'double'},{'size',[4,1],'real','finite','positive'});
        validateattributes(hypotheses(j).sensorBias,{'double'},{'size',[7,1],'real','finite'});
        q=hypotheses(j).target;hasTarget(j)=~isempty(q);
        if hasTarget(j)
            validateattributes(q,{'double'},{'size',[11,1],'real','finite'});
            assert(q(4)>=0 && q(7)>0 && all(q(8:9)>0), ...
                'collisionAvoidanceController:invalidInitialInformation','Invalid target speed or footprint.');
        end
    end
    assert(all(hasTarget==hasTarget(1)), ...
        'collisionAvoidanceController:unsupportedTargetExistenceMixture', ...
        'This finite transcription requires a common target-existence mode.');
end

function cfg = localOfPhysicalConfiguration(hypothesis,cfg)
    % Scales remain attached to their physical hypothesis for its lifetime.
    cfg.tire.corneringStiffness=cfg.tire.corneringStiffness.*hypothesis.parameters(1:2);
    cfg.tire.frictionCoefficient=cfg.tire.frictionCoefficient.*hypothesis.parameters(3:4);
end

function next = localOfAdvance(hypothesis,input,cfg,duration,endOfHold)
    physical=localOfPhysicalConfiguration(hypothesis,cfg);next=hypothesis;
    if string(cfg.outputFeedback.method)=="compactNlp"
        next.ego=localCompactInferenceFlow(hypothesis.ego,input,hypothesis.parameters,cfg,duration);
        if endOfHold,next.ego=nonlinearBicycleModel.sampleEndState(next.ego,input,cfg);end
    else
        next.ego=nonlinearBicycleModel.sensorFlow(hypothesis.ego,input,physical,duration,endOfHold);
    end
    if ~isempty(hypothesis.target) && hypothesis.target(4)>0
        next.target=predictiveSafetyGeometry.predictTarget(hypothesis.target,duration);
        next.target(5:6)=hypothesis.target(5:6);
    end
    % V=0 is an absorbing mode, including when latent A was nonzero.
end

function next=localCompactInferenceFlow(x,u,parameters,cfg,duration)
    % Use the fine acceptance resolution for the causal sensor update too.
    % A coarse inference integrator can otherwise discard the true member
    % because its acceleration prediction exceeds the declared noise bound.
    persistent keys flows
    key={cfg,duration};index=[];
    for j=1:numel(keys),if isequaln(keys{j},key),index=j;break;end,end
    if isempty(index)
        import casadi.*
        count=max(1,ceil(cfg.outputFeedback.validationSubsteps*duration/cfg.controller.sampleTime));dt=duration/count;
        root=SX.sym('inference_state',6);command=SX.sym('inference_input',2);scale=SX.sym('inference_scale',4);
        k1=nonlinearBicycleModel.symbolicDerivative(root,command,scale,cfg);
        k2=nonlinearBicycleModel.symbolicDerivative(root+dt*k1/2,command,scale,cfg);
        k3=nonlinearBicycleModel.symbolicDerivative(root+dt*k2/2,command,scale,cfg);
        k4=nonlinearBicycleModel.symbolicDerivative(root+dt*k3,command,scale,cfg);
        options=struct;
        if cfg.outputFeedback.nlpJit
            directory=tempname;mkdir(directory);
            options=struct('jit',true,'compiler','shell','jit_options',struct( ...
                'flags',sprintf('-O%d',cfg.outputFeedback.nlpCompilerOptimization), ...
                'compiler',char(cfg.outputFeedback.nlpCompiler),'directory',[directory,filesep]));
        end
        step=Function('pcbf_inference_step',{root,command,scale},{root+dt*(k1+2*k2+2*k3+k4)/6},options);
        mapped=step.mapaccum(count);root=MX.sym('root',6);command=MX.sym('command',2);scale=MX.sym('scale',4);
        states=mapped(root,repmat(command,1,count),repmat(scale,1,count));
        flows{end+1}=Function('pcbf_inference',{root,command,scale},{states(:,end)});keys{end+1}=key;index=numel(keys);
    end
    next=full(flows{index}(x,u,parameters));
end

function y = localOfSensor(hypothesis,input,cfg,time)
    physical=localOfPhysicalConfiguration(hypothesis,cfg);x=hypothesis.ego;
    dx=nonlinearBicycleModel.derivative(x,input,physical);
    a=[dx(4)-x(6)*x(5);dx(5)+x(6)*x(4)];
    relative=zeros(2,1);detected=false;
    if ~isempty(hypothesis.target)
        rotation=[cos(x(3)),sin(x(3));-sin(x(3)),cos(x(3))];
        relative=rotation*(hypothesis.target(1:2)-x(1:2));
        detected=norm(relative)<=cfg.collision.encounterRangeMeters;
    end
    bias=hypothesis.sensorBias;
    y=struct('time',time,'position',x(1:2)+bias(1:2), ...
        'acceleration',a+bias(3:4),'yawRate',x(6)+bias(5), ...
        'relativePosition',relative+bias(6:7),'radarAvailable',detected);
end

function [posterior,info] = localOfCondition(hypotheses,block,cfg,initial)
    hypotheses=localOfValidateHypotheses(hypotheses,cfg);
    keep=false(1,numel(hypotheses));residual=Inf(numel(hypotheses),1);
    next=hypotheses;bounds=cfg.outputFeedback.noiseBounds;
    tolerance=1e-9; % Numerical measurement comparison only; not extra noise.
    for s=1:numel(hypotheses)
        item=hypotheses(s);excess=-Inf;
        for j=1:numel(block.samples)
            if ~initial
                item=localOfAdvance(item,block.heldInput,cfg,cfg.outputFeedback.sensorPeriod,j==numel(block.samples));
            end
            measured=block.samples(j);
            expected=localOfSensor(item,block.heldInput,cfg,measured.time);
            errors=[norm(measured.position-expected.position); ...
                norm(measured.acceleration-expected.acceleration);abs(measured.yawRate-expected.yawRate)];
            excess=max(excess,max(errors-bounds(1:3)));
            if measured.radarAvailable
                if isempty(item.target) || ~expected.radarAvailable,excess=Inf;
                else,excess=max(excess,norm(measured.relativePosition-expected.relativePosition)-bounds(4));end
            end
            % No return is compatible with a dropout: it never deletes a
            % physical target or clears its future collision obligations.
        end
        next(s)=item;residual(s)=excess;keep(s)=excess<=tolerance;
    end
    assert(any(keep),'collisionAvoidanceController:emptyJointInformation', ...
        'No retained physical hypothesis explains the complete raw block. No command or memory reset is authorized.');
    posterior=next(keep);
    info=struct('priorCount',numel(hypotheses),'posteriorCount',nnz(keep), ...
        'retainedIndices',find(keep),'maximumNoiseExcess',residual, ...
        'rawReadingsConditioned',numel(block.samples),'representation',"finiteConsistentHypotheses");
end

function feature = localOfFeatures(samples,cfg)
    values=zeros(8,numel(samples));
    for j=1:numel(samples)
        y=samples(j);relative=zeros(2,1);
        if y.radarAvailable,relative=y.relativePosition;end
        values(:,j)=[[y.position;y.acceleration;y.yawRate;relative]./cfg.outputFeedback.featureScale; ...
            double(y.radarAvailable)];
    end
    feature=mean(values,2);
end

function memory = localOfMemory(policy,memory,samples,cfg)
    memory=tanh(policy.A*memory+policy.B*localOfFeatures(samples,cfg)+policy.c);
end

function policy = localOfBackup(cfg)
    n=cfg.outputFeedback.memoryDimension;
    policy=struct('v',[0;-cfg.outputFeedback.backupBrakingRatio], ...
        'K',zeros(2,n),'A',zeros(n),'B',zeros(n,8),'c',zeros(n,1));
end

function window = localOfWindow(hypotheses,inheritedCount,cfg)
    % Select coverage directly, before optimization; never try increasing
    % lengths until a solve succeeds. This is an engineering time scale,
    % not a certified Fiala stopping-time or feasibility upper bound.
    of=cfg.outputFeedback;h=cfg.controller.sampleTime;
    speed=max(arrayfun(@(s)hypot(s.ego(4),s.ego(5)),hypotheses));
    if isempty(of.horizonSeconds)
        minimum=2*h;
        if string(of.method)=="compactNlp"
            minimum=max(minimum,of.minimumPreviewSeconds);
            % A low-speed preview must span the encounter's spatial scale,
            % not only braking time. This is a performance lookahead, not a
            % stopping reserve or a certificate of feasibility.
            targetRadius=0;
            for s=1:numel(hypotheses)
                q=hypotheses(s).target;
                if ~isempty(q),targetRadius=max(targetRadius,norm(q(8:9))+norm(q(10:11)));end
            end
            if targetRadius>0 && cfg.referenceSpeed>0
                egoRadius=hypot(cfg.vehicle.length/2,cfg.vehicle.width/2)+norm(cfg.vehicle.rectangleOffset);
                minimum=max(minimum,2*(egoRadius+targetRadius)/max(speed,cfg.referenceSpeed));
            end
        end
        requested=max(minimum,speed/of.previewDecelerationMetersPerSecondSquared+of.previewSettlingSeconds);
        allocated=of.horizonResolutionSeconds*ceil(requested/of.horizonResolutionSeconds);
        source="speedBasedPreview";
    else
        requested=max(2*h,of.horizonSeconds);allocated=requested;source="configuredDuration";
    end
    count=max(inheritedCount,ceil(allocated/h));
    assert(count<=cfg.controller.maximumHorizonSteps, ...
        'collisionAvoidanceController:predictionWindowCapacity', ...
        'The selected %.3g s preview needs %d stages, exceeding the configured capacity %g. It was not truncated.', ...
        count*h,count,cfg.controller.maximumHorizonSteps);
    window=struct('source',source,'requestedSeconds',requested,'allocatedSeconds',count*h, ...
        'horizonSteps',count,'speedUpperMetersPerSecond',speed, ...
        'inheritedHorizonSteps',inheritedCount,'feasibilityCertified',false);
end

function policies = localOfInitialPolicies(cfg,hypotheses,reference,previousInput,count)
    % The selected time coverage sizes this numerical initialization only.
    speed=max(arrayfun(@(s)norm(s.ego(4:5)),hypotheses));
    policy=localOfBackup(cfg);n=cfg.outputFeedback.memoryDimension;
    policy.A=.25*eye(n);policy.B=.1*eye(n,8);
    policies=repmat(policy,1,count);
    % A smooth avoidance rollout provides only a starting policy. It uses a
    % representative retained hypothesis and is checked against every finite
    % scenario by the optimizer. Its first local QP must succeed before execution.
    representative=hypotheses(ceil(numel(hypotheses)/2));
    if speed>cfg.model.kinematicSpeedRange(1) && ~isempty(representative.target)
        x=representative.ego;q=representative.target;
        stub=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
        [~,lane,road]=readControllerInputs(stub,[],reference,cfg);
        frame=predictiveSafetyGeometry.roadFrame(lane,road);
        nominal=struct('state',[0;0;0;cfg.referenceSpeed;0;0],'curvature',frame(4));
        model=struct('cfg',cfg,'initialState',x,'previousInput',previousInput, ...
            'target',q,'lane',lane,'road',road,'frame',frame, ...
            'nominalReference',nominal, ...
            'uncertainty',struct('collisionGenerator',zeros(6,0),'target',[]));
        guide=predictiveSafetyGeometry.avoidanceGuide(model,count);
        for i=1:count-1
            policies(i).v=[max(-cfg.outputFeedback.steeringMaximum,min(cfg.outputFeedback.steeringMaximum,guide.inputs(1,i))); ...
                -cfg.outputFeedback.backupBrakingRatio];
        end
    end
    policies(end)=localOfBackup(cfg);
end

function vector = localOfPack(policy)
    vector=[policy.v(:);policy.K(:);policy.A(:);policy.B(:);policy.c(:)];
end

function policy = localOfUnpack(vector,cfg)
    n=cfg.outputFeedback.memoryDimension;a=2;b=a+2*n;c=b+n*n;d=c+8*n;
    policy=struct('v',vector(1:a),'K',reshape(vector(a+1:b),2,n), ...
        'A',reshape(vector(b+1:c),n,n),'B',reshape(vector(c+1:d),n,8),'c',vector(d+1:end));
end

function policies = localOfUnpackPlan(point,problem)
    policies=repmat(localOfBackup(problem.cfg),1,problem.horizon);
    for i=1:problem.horizon
        policies(i)=localOfUnpack(point(problem.policyIndices(:,i)),problem.cfg);
    end
end

function problem = localOfProblem(hypotheses,memory,input,policies,reference,cfg,capacity)
    if nargin<7,capacity=numel(hypotheses);end
    n=numel(policies);dimension=numel(localOfPack(localOfBackup(cfg)));
    assert(isstruct(policies) && n>=2 && n<=cfg.controller.maximumHorizonSteps, ...
        'collisionAvoidanceController:invalidPolicySeed','The seed must have one complete policy per hold.');
    indices=reshape(1:dimension*n,dimension,n);point=zeros(dimension*n,1);
    for i=1:n
        vector=localOfPack(policies(i));
        assert(numel(vector)==dimension && all(isfinite(vector)), ...
            'collisionAvoidanceController:invalidPolicySeed','Invalid policy dimensions or coefficients.');
        point(indices(:,i))=vector;
    end
    scenarios=struct([]);noise=cfg.outputFeedback.noiseDirections;
    radii=cfg.outputFeedback.noiseBounds([1;1;2;2;3;4;4]);
    for s=1:numel(hypotheses)
        for j=1:size(noise,2)
            scenario=struct('initial',hypotheses(s),'noise',radii.*noise(:,j),'rootIndex',s,'weight',1);
            if isempty(scenarios),scenarios=scenario;else,scenarios(end+1)=scenario;end %#ok<AGROW>
        end
    end
    activeCount=numel(scenarios);
    if string(cfg.outputFeedback.method)=="ipopt"
        % Keep the compiled NLP shape through posterior pruning. Duplicate
        % the last surviving trajectory with zero objective weight; its hard
        % rows are redundant and do not add a new uncertainty realization.
        scenarios(activeCount+1:capacity*size(noise,2))=scenarios(end);
    end
    for s=1:numel(scenarios),scenarios(s).weight=double(s<=activeCount)/activeCount;end
    x=hypotheses(1).ego;
    stub=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
    [~,lane]=readControllerInputs(stub,[],reference,cfg);
    hasTarget=~isempty(hypotheses(1).target);policyCount=numel(point);
    witnessIndices=zeros(2,3,n,numel(scenarios));
    if hasTarget
        witnessIndices=reshape(policyCount+(1:numel(witnessIndices)),size(witnessIndices));
        point=[point;zeros(numel(witnessIndices),1)];
    end
    lower=-cfg.outputFeedback.coefficientMaximum*ones(size(point));upper=-lower;
    for i=1:n
        gain=indices(3:2+2*cfg.outputFeedback.memoryDimension,i);
        lower(gain)=-cfg.outputFeedback.gainMaximum;upper(gain)=cfg.outputFeedback.gainMaximum;
    end
    if hasTarget,lower(policyCount+1:end)=-Inf;upper(policyCount+1:end)=Inf;end
    assert(all(point(1:policyCount)>=lower(1:policyCount)) && all(point(1:policyCount)<=upper(1:policyCount)), ...
        'collisionAvoidanceController:invalidPolicySeed','Policy seed exceeds the coefficient contract.');
    problem=struct('cfg',cfg,'scenarios',scenarios,'memory',memory,'previousInput',input, ...
        'horizon',n,'lane',lane,'policyIndices',indices,'policyCount',policyCount, ...
        'witnessIndices',witnessIndices,'hasTarget',hasTarget,'lower',lower,'upper',upper, ...
        'initial',point,'regularizationAnchor',point(1:policyCount),'seedRepaired',false);
    try
        trajectories=localOfRollout(point,problem);
    catch exception
        if string(cfg.outputFeedback.method)~="ipopt" || ...
                ~any(string(exception.identifier)==["collisionAvoidanceController:jointTrialDomain", ...
                "collisionAvoidanceController:nonlinearDomain"]),rethrow(exception);end
        % A changed measured memory can invalidate a shifted numerical seed.
        % Repair only the seed's actuator maps; inference and actual memory
        % are untouched, and the optimizer must still pass nonlinear replay.
        previous=input;
        for i=1:n
            policy=policies(i);policy.K(:)=0;
            policy.v(1)=max(-cfg.outputFeedback.steeringMaximum,min(cfg.outputFeedback.steeringMaximum, ...
                max(previous(1)-cfg.controller.sampleTime*cfg.outputFeedback.steeringRateMaximum, ...
                min(previous(1)+cfg.controller.sampleTime*cfg.outputFeedback.steeringRateMaximum,policy.v(1)))));
            change=cfg.controller.sampleTime*cfg.model.brakingRatioRateMaximum;
            policy.v(2)=max(cfg.actuation.brakingRatioMinimum,min(cfg.actuation.brakingRatioMaximum, ...
                max(previous(2)-change,min(previous(2)+change,policy.v(2)))));
            point(indices(:,i))=localOfPack(policy);previous=policy.v;
        end
        problem.initial=point;problem.regularizationAnchor=point(1:policyCount);problem.seedRepaired=true;
        trajectories=localOfRollout(point,problem);
    end
    if hasTarget
        for s=1:numel(scenarios)
            for i=1:n
                for j=1:3
                    x=trajectories{s}.checks(:,j,i);q=trajectories{s}.targetChecks(:,j,i);
                    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
                    witness=predictiveSafetyGeometry.supportLinearization(x(1:3),shape,q(1:3),q(8:11));
                    normal=witness.normal;rotation=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];
                    offset=normal.'*rotation*q(10:11)+abs(normal.'*rotation)*q(8:9);
                    point(witnessIndices(:,j,i,s))=[atan2(normal(2),normal(1));offset];
                end
            end
        end
        problem.initial=point;
        problem.nominalTrajectories=trajectories;
    else
        problem.nominalTrajectories=trajectories;
    end
end

function trajectories = localOfRollout(point,problem)
    cfg=problem.cfg;n=problem.horizon;l=cfg.outputFeedback.predictionSubsteps;
    dt=cfg.controller.sampleTime/l;
    policies=localOfUnpackPlan(point,problem);trajectories=cell(1,numel(problem.scenarios));
    for s=1:numel(problem.scenarios)
        scenario=problem.scenarios(s);item=scenario.initial;m=problem.memory;
        trace=struct('ego',zeros(6,n+1),'memory',zeros(numel(m),n+1), ...
            'inputs',zeros(2,n),'targets',zeros(11,n+1),'checks',zeros(6,3,n), ...
            'targetChecks',zeros(11,3,n),'physicalNodes',zeros(6,l+1,n), ...
            'sensorBlocks',{cell(1,n)},'rootIndex',scenario.rootIndex);
        trace.ego(:,1)=item.ego;trace.memory(:,1)=m;
        if problem.hasTarget,trace.targets(:,1)=item.target;end
        for i=1:n
            policy=policies(i);u=policy.v+policy.K*m;trace.inputs(:,i)=u;
            assert(abs(u(2))<=1+1e-12,'collisionAvoidanceController:jointTrialDomain', ...
                'A trial force ratio left the physical model domain.');
            trace.checks(:,1,i)=item.ego;trace.physicalNodes(:,1,i)=item.ego;
            if problem.hasTarget,trace.targetChecks(:,1,i)=item.target;end
            readings=struct([]);
            for j=1:l
                item=localOfAdvance(item,u,cfg,dt,j==l);
                y=localOfSensor(item,u,cfg,(i-1)*cfg.controller.sampleTime+j*dt);
                y.position=y.position+scenario.noise(1:2);y.acceleration=y.acceleration+scenario.noise(3:4);
                y.yawRate=y.yawRate+scenario.noise(5);y.relativePosition=y.relativePosition+scenario.noise(6:7);
                if isempty(readings),readings=y;else,readings(end+1)=y;end %#ok<AGROW>
                trace.physicalNodes(:,j+1,i)=item.ego;
                if j==l/2 || j==l
                    slot=1+round(2*j/l);trace.checks(:,slot,i)=item.ego;
                    if problem.hasTarget,trace.targetChecks(:,slot,i)=item.target;end
                end
            end
            m=localOfMemory(policy,m,readings,cfg);
            trace.ego(:,i+1)=item.ego;trace.memory(:,i+1)=m;trace.sensorBlocks{i}=readings;
            if problem.hasTarget,trace.targets(:,i+1)=item.target;end
        end
        trajectories{s}=trace;
    end
end

function [point,info,trajectory] = localOfOptimize(problem,initial,timer)
    % One sparse multiple-shooting QP. The nonlinear model supplies local
    % tangents only; no nonlinear line search or feasibility acceptance gate.
    cfg=problem.cfg;of=cfg.outputFeedback;n=problem.horizon;
    trajectory=problem.nominalTrajectories;point=problem.initial;
    nm=of.memoryDimension;nx=8+nm;ns=numel(trajectory);np=numel(point);
    ix=reshape(np+(1:nx*n*ns),nx,n,ns);nv=np+numel(ix);
    policies=localOfUnpackPlan(point,problem);
    jr=cell(n*ns+1,1);jc=cell(n*ns,1);ae=cell(n*ns,1);
    residual=cell(n*ns+1,1);constraint=cell(n*ns,1);numericalTolerance=cell(n*ns,1);
    lower=[problem.lower-point;-Inf(numel(ix),1)];upper=[problem.upper-point;Inf(numel(ix),1)];
    lower(1:np)=max(lower(1:np),-.25);upper(1:np)=min(upper(1:np),.25);
    % Braking feedforward may use its full coefficient range. Actual commands
    % still obey the input magnitude and configured slew constraints below.
    brake=problem.policyIndices(2,:);
    lower(brake)=problem.lower(brake)-point(brake);
    upper(brake)=problem.upper(brake)-point(brake);
    linearTimer=tic;flowCount=0;
    for s=1:ns
        trace=trajectory{s};item=problem.scenarios(s).initial;
        for i=1:n
            k=(s-1)*n+i;policy=policies(i);m=trace.memory(:,i);u=trace.inputs(:,i);
            previous=problem.previousInput;if i>1,previous=trace.inputs(:,i-1);end
            item.ego=trace.ego(:,i);
            if problem.hasTarget,item.target=trace.targets(:,i);end
            witness=[];wi=[];
            if problem.hasTarget,wi=problem.witnessIndices(:,:,i,s);wi=wi(:);witness=point(wi);end
            % Primitive local variables: x, u, previous u, memory, witnesses.
            z=[item.ego;u;previous;m;witness];nz=numel(z);
            data=localOfStageFlow(z,item,problem.scenarios(s).noise,cfg);flowCount=flowCount+1;
            [r,c,f]=localOfStageValues(z,data,policy,problem);
            dr=zeros(numel(r),nz);dc=zeros(numel(c),nz);df=zeros(nx,nz);
            for j=1:8
                delta=of.finiteDifferenceStep*max(1,abs(z(j)));
                if j==8 && z(j)+delta>=1,delta=-delta;end
                trial=z;trial(j)=trial(j)+delta;d=data;
                if j<=8
                    d=localOfStageFlow(trial,item,problem.scenarios(s).noise,cfg);flowCount=flowCount+1;
                end
                [rr,cc,ff]=localOfStageValues(trial,d,policy,problem);
                dr(:,j)=(rr-r)/delta;dc(:,j)=(cc-c)/delta;df(:,j)=(ff-f)/delta;
            end
            % Exact cheap derivatives avoid re-evaluating physical or
            % geometric functions for memory, prior input and witnesses.
            dr(end-1:end,9:10)=-sqrt(of.inputWeight/ns)*eye(2);
            base=4;
            if isfinite(of.steeringRateMaximum)
                dc(base+(1:2),9)=[-1;1];base=base+2;
            end
            if isfinite(cfg.model.brakingRatioRateMaximum)
                dc(base+(1:2),10)=[-1;1];base=base+2;
            end
            df(7:6+nm,11:10+nm)=diag(1-f(7:6+nm).^2)*policy.A;
            if ~isempty(wi)
                shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
                base=base+6*size(data.states,2);
                for check=1:3
                    q=data.targets(:,check);x=data.checks(:,check);
                    [~,gj]=predictiveSafetyGeometry.separationWitnessRows( ...
                        x(1:3),shape,q(1:3),q(8:11),witness(2*check-1:2*check),0);
                    dc(base+(check-1)*8+(1:8),10+nm+(2*check-1:2*check))=-gj(:,4:5);
                end
            end
            map=sparse(nz,nv);
            if i>1
                old=ix(:,i-1,s);
                map(1:6,old(1:6))=eye(6);
                map(11:10+nm,old(7:6+nm))=eye(nm);
                map(9:10,old(7+nm:8+nm))=eye(2);
                map(7:8,old(7:6+nm))=policy.K;
            end
            direct=sparse(nx,nv);
            pi=problem.policyIndices(:,i);
            map(7:8,pi(1:2+2*nm))=[eye(2),kron(m.',eye(2))];
            gain=1-f(7:6+nm).^2;
            direct(7:6+nm,pi(3+2*nm:end))=diag(gain)* ...
                [kron(m.',eye(nm)),kron(data.feature.',eye(nm)),eye(nm)];
            if ~isempty(wi),map(11+nm:end,wi)=eye(numel(wi));end
            jr{k}=sparse(dr)*map;jc{k}=sparse(dc)*map;
            ae{k}=-sparse(df)*map-direct;ae{k}(:,ix(:,i,s))=speye(nx);
            residual{k}=r;constraint{k}=c;
            numericalTolerance{k}=.5*cfg.solver.constraintTolerance*ones(size(c));
            commandRows=4+2*isfinite(of.steeringRateMaximum)+2*isfinite(cfg.model.brakingRatioRateMaximum);
            numericalTolerance{k}(1:commandRows)=0;
            lower(ix(7:6+nm,i,s))=-1-trace.memory(:,i+1);
            upper(ix(7:6+nm,i,s))=1-trace.memory(:,i+1);
        end
    end
    anchor=sparse(problem.policyCount,nv);anchor(:,1:problem.policyCount)= ...
        sqrt(of.regularization)*speye(problem.policyCount);
    jr{end}=anchor;residual{end}=zeros(problem.policyCount,1);
    j=vertcat(jr{:});c=vertcat(constraint{:});a=vertcat(jc{:});e=vertcat(ae{:});r=vertcat(residual{:});
    % Drop satisfied constant rows, retaining infeasible constants as errors.
    active=any(a,2);assert(all(c(~active)<=cfg.solver.constraintTolerance), ...
        'collisionAvoidanceController:noOutputFeedbackPlan','A fixed local constraint is infeasible.');
    a=a(active,:);c=c(active);
    allowance=vertcat(numericalTolerance{:});allowance=allowance(active);
    hessian=j.'*j+1e-7*speye(nv);gradient=j.'*r;
    linearizationSeconds=toc(linearTimer);
    if isfield(problem,'prepareOnly') && problem.prepareOnly
        freeLower=lower;freeUpper=upper;
        freeLower(1:np)=problem.lower-point;freeUpper(1:np)=problem.upper-point;
        info=struct('equality',e,'inequality',a,'bounds',allowance-c,'hessian',hessian, ...
            'gradient',gradient,'lower',lower,'upper',upper, ...
            'lowerWithoutTrust',freeLower,'upperWithoutTrust',freeUpper, ...
            'activeRows',find(active),'nominalConstraints',vertcat(constraint{:}), ...
            'stateIndices',ix,'policyIndices',problem.policyIndices, ...
            'initialPoint',point,'nominal', {trajectory},'policies',policies,'policyCount',problem.policyCount);
        return;
    end
    qpTimer=tic;
    options=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
        'ConstraintTolerance',min(1e-8,cfg.solver.constraintTolerance), ...
        'MaxIterations',of.qpMaximumIterations);
    [step,~,flag,output]=quadprog(hessian,gradient,a,allowance-c,e,zeros(size(e,1),1),lower,upper,[],options);
    qpSeconds=toc(qpTimer);
    assert(~isempty(step) && flag>0,'collisionAvoidanceController:noOutputFeedbackPlan', ...
        'The single RTI QP failed (exit flag %d). No command was committed.',flag);
    point=point+step(1:np);
    % Return the affine shooting prediction. It is explicitly not a fresh
    % nonlinear simulation of the trial policy and not a safety certificate.
    accepted=localOfUnpackPlan(point,problem);
    for s=1:ns
        for i=1:n
            trajectory{s}.ego(:,i+1)=trajectory{s}.ego(:,i+1)+step(ix(1:6,i,s));
            trajectory{s}.memory(:,i+1)=trajectory{s}.memory(:,i+1)+step(ix(7:6+nm,i,s));
            trajectory{s}.inputs(:,i)=accepted(i).v+accepted(i).K*trajectory{s}.memory(:,i);
        end
    end
    info=struct('sqpIterations',1,'acceptedSteps',1,'qpExitFlag',flag, ...
        'qpIterations',output.iterations,'maximumLinearizedConstraintViolation',max([0;a*step+c]), ...
        'maximumShootingEqualityResidual',norm(e*step,Inf), ...
        'maximumConstraintViolation',NaN,'objective',.5*norm(r+j*step)^2, ...
        'shiftedPlanRevalidated',false,'usedRevalidatedFallback',false, ...
        'nonlinearValidationPerformed',false,'horizonSteps',n, ...
        'hardCollisionConstraints',true,'terminalControllerActive',false, ...
        'linearizationSeconds',linearizationSeconds,'qpSeconds',qpSeconds, ...
        'stageFlowEvaluations',flowCount,'qpVariables',nv,'qpNonzeros',nnz(hessian)+nnz(a)+nnz(e), ...
        'sparseQp',issparse(hessian)&&issparse(a)&&issparse(e), ...
        'predictionIsLinearized',true,'auxiliaryNodesAreNominal',true,'initialization',initial, ...
        'deadlineSeconds',min(of.maximumSolveSeconds,cfg.controller.sampleTime), ...
        'deadlineMissed',toc(timer)>min(of.maximumSolveSeconds,cfg.controller.sampleTime));
end

function data=localOfStageFlow(z,item,noise,cfg)
    item.ego=z(1:6);u=z(7:8);l=cfg.outputFeedback.predictionSubsteps;
    dt=cfg.controller.sampleTime/l;states=zeros(6,l+1);states(:,1)=item.ego;
    readings=struct([]);targets=zeros(11,3);
    if ~isempty(item.target),targets(:,1)=item.target;end
    for j=1:l
        item=localOfAdvance(item,u,cfg,dt,j==l);
        states(:,j+1)=item.ego;y=localOfSensor(item,u,cfg,j*dt);
        y.position=y.position+noise(1:2);y.acceleration=y.acceleration+noise(3:4);
        y.yawRate=y.yawRate+noise(5);y.relativePosition=y.relativePosition+noise(6:7);
        if j==1,readings=y;else,readings(j)=y;end
        if ~isempty(item.target) && (j==l/2 || j==l),targets(:,1+2*j/l)=item.target;end
    end
    data=struct('states',states,'checks',states(:,[1,1+l/2,1+l]), ...
        'targets',targets,'next',item.ego,'feature',localOfFeatures(readings,cfg));
end

function [r,c,f]=localOfStageValues(z,data,policy,problem)
    cfg=problem.cfg;of=cfg.outputFeedback;nm=of.memoryDimension;h=cfg.controller.sampleTime;
    u=z(7:8);previous=z(9:10);m=z(11:10+nm);
    nextMemory=tanh(policy.A*m+policy.B*data.feature+policy.c);
    f=[data.next;nextMemory;u];x=data.next;
    projection=predictiveSafetyGeometry.project(x(1:2),problem.lane);
    error=[projection.lateralPosition;atan2(sin(x(3)-projection.heading),cos(x(3)-projection.heading)); ...
        x(4)-cfg.referenceSpeed;x(5);x(6)];
    r=[sqrt(h*of.trackingWeights).*error;sqrt(of.inputWeight)*(u-previous)]/sqrt(numel(problem.scenarios));
    c=[u(1)-of.steeringMaximum;-u(1)-of.steeringMaximum; ...
        u(2)-cfg.actuation.brakingRatioMaximum;cfg.actuation.brakingRatioMinimum-u(2)];
    if isfinite(of.steeringRateMaximum)
        change=h*of.steeringRateMaximum;c=[c;u(1)-previous(1)-change;previous(1)-u(1)-change];
    end
    if isfinite(cfg.model.brakingRatioRateMaximum)
        change=h*cfg.model.brakingRatioRateMaximum;c=[c;u(2)-previous(2)-change;previous(2)-u(2)-change];
    end
    states=data.states;
    c=[c;reshape([-states(4,:);states(4,:)-cfg.model.speedMaximum; ...
        states(5,:)-cfg.model.lateralVelocityMaximum;-states(5,:)-cfg.model.lateralVelocityMaximum; ...
        states(6,:)-cfg.model.yawRateMaximum;-states(6,:)-cfg.model.yawRateMaximum],[],1)];
    if problem.hasTarget
        shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
        reserve=of.clearanceMeters+of.intersampleMarginMeters;
        witness=reshape(z(11+nm:end),2,3);
        for j=1:3
            q=data.targets(:,j);x=data.checks(:,j);
            c=[c;-predictiveSafetyGeometry.separationWitnessRows(x(1:3),shape,q(1:3),q(8:11),witness(:,j),reserve)]; %#ok<AGROW>
        end
    end
end

function [point,info,trajectory,warm] = localOfNonlinearOptimize(problem,initial,timer,priorWarm)
    % Thompson-style numerical route for OUR finite joint policy problem:
    % cached parameterized NLP, automatic derivatives, IPOPT and warm starts.
    assert(exist('casadi.SX','class')==8,'collisionAvoidanceController:missingCasadi', ...
        'Run scripts/setupJointNlpSolver before selecting outputFeedback.method=ipopt.');
    cfg=problem.cfg;of=cfg.outputFeedback;buildTimer=tic;
    [kernel,cacheHit]=localOfNlpKernel(problem);buildSeconds=toc(buildTimer);
    [parameters,seed]=localOfNlpData(problem,kernel);
    lower=[problem.lower;kernel.stateLower];upper=[problem.upper;kernel.stateUpper];
    seed=min(upper,max(lower,seed));
    solverArguments={'x0',seed,'p',parameters,'lbx',lower,'ubx',upper, ...
        'lbg',kernel.constraintLower,'ubg',kernel.constraintUpper};
    dualWarm=false;
    if ~isempty(priorWarm) && isequal(priorWarm.key,kernel.key)
        lx=zeros(size(seed));lg=zeros(size(kernel.constraintLower));
        pi=problem.policyIndices;wi=problem.witnessIndices;ix=kernel.stateIndices;
        lx(pi(:,1:end-1))=priorWarm.lambdaX(pi(:,2:end));
        if problem.hasTarget,lx(wi(:,:,1:end-1,:))=priorWarm.lambdaX(wi(:,:,2:end,:));end
        lx(ix(:,1:end-1,:))=priorWarm.lambdaX(ix(:,2:end,:));
        gi=kernel.constraintIndices;
        lg(gi(:,1:end-1,:))=priorWarm.lambdaG(gi(:,2:end,:));
        solverArguments=[solverArguments,{'lam_x0',lx,'lam_g0',lg}];dualWarm=true;
    end
    solveTimer=tic;solution=kernel.solver(solverArguments{:});solveSeconds=toc(solveTimer);
    stats=kernel.solver.stats();trial=full(solution.x);
    validationTimer=tic;
    [valid,trajectory,violation,objective]=localOfValidateNlpPolicy(trial(1:numel(problem.initial)),problem);
    trialViolation=violation;
    fallback=false;point=trial(1:numel(problem.initial));
    if ~valid
        [valid,trajectory,violation,objective]=localOfValidateNlpPolicy(problem.initial,problem);
        fallback=valid;point=problem.initial;
    end
    assert(valid,'collisionAvoidanceController:noOutputFeedbackPlan', ...
        'IPOPT returned %s; nonlinear violation: candidate %g, shifted seed %g (limit %g). No command was committed.', ...
        stats.return_status,trialViolation,violation,of.nlpTolerance);
    validationSeconds=toc(validationTimer);warm=[];
    if ~fallback
        warm=struct('key',{kernel.key},'lambdaX',full(solution.lam_x),'lambdaG',full(solution.lam_g));
    end
    residual=full(solution.g);equality=kernel.constraintLower==kernel.constraintUpper;
    info=struct('solver',"casadiIpopt",'nlpStatus',string(stats.return_status), ...
        'nlpConverged',logical(stats.success),'nlpIterations',stats.iter_count, ...
        'sqpIterations',0,'acceptedSteps',double(~fallback),'qpExitFlag',NaN, ...
        'maximumLinearizedConstraintViolation',NaN, ...
        'maximumShootingEqualityResidual',max([0;abs(residual(equality))]), ...
        'maximumConstraintViolation',violation,'objective',objective, ...
        'candidateMaximumConstraintViolation',trialViolation,'candidateObjective',full(solution.f), ...
        'shiftedPlanRevalidated',fallback,'usedRevalidatedFallback',fallback, ...
        'nonlinearValidationPerformed',true,'horizonSteps',problem.horizon, ...
        'hardCollisionConstraints',true,'terminalControllerActive',false, ...
        'linearizationSeconds',0,'qpSeconds',0,'nlpSeconds',solveSeconds, ...
        'nlpBuildSeconds',buildSeconds,'nonlinearValidationSeconds',validationSeconds, ...
        'nlpCacheHit',cacheHit,'nlpDualWarmStart',dualWarm, ...
        'nlpVariables',numel(seed),'stageFlowEvaluations',NaN, ...
        'predictionIsLinearized',false,'auxiliaryNodesAreNominal',false, ...
        'automaticDerivatives',true,'initialization',initial,'numericalSeedRepaired',problem.seedRepaired, ...
        'trackingReferenceScope',"nominalLocalPathTangent", ...
        'deadlineSeconds',min(of.maximumSolveSeconds,cfg.controller.sampleTime), ...
        'deadlineMissed',toc(timer)>min(of.maximumSolveSeconds,cfg.controller.sampleTime));
end

function [valid,traces,violation,objective] = localOfValidateNlpPolicy(point,problem)
    valid=false;traces={};violation=Inf;objective=Inf;
    if any(~isfinite(point)),return;end
    try
        traces=localOfRollout(point,problem);policies=localOfUnpackPlan(point,problem);
        violation=max([0;problem.lower-point;point-problem.upper]);
        objective=.5*problem.cfg.outputFeedback.regularization* ...
            norm(point(1:problem.policyCount)-problem.initial(1:problem.policyCount))^2;
        for s=1:numel(traces)
            trace=traces{s};previous=problem.previousInput;
            for i=1:problem.horizon
                z=[trace.ego(:,i);trace.inputs(:,i);previous;trace.memory(:,i)];
                if problem.hasTarget,z=[z;reshape(point(problem.witnessIndices(:,:,i,s)),[],1)];end %#ok<AGROW>
                data=struct('next',trace.ego(:,i+1),'feature',localOfFeatures(trace.sensorBlocks{i},problem.cfg), ...
                    'states',trace.physicalNodes(:,:,i),'checks',trace.checks(:,:,i),'targets',trace.targetChecks(:,:,i));
                [r,c]=localOfStageValues(z,data,policies(i),problem);
                objective=objective+.5*(r.'*r)*numel(problem.scenarios)*problem.scenarios(s).weight;
                violation=max(violation,max(c));previous=trace.inputs(:,i);
            end
        end
        valid=isfinite(violation) && violation<=problem.cfg.outputFeedback.nlpTolerance;
    catch exception
        if ~startsWith(exception.identifier,'collisionAvoidanceController:'),rethrow(exception);end
    end
end

function [kernel,hit] = localOfNlpKernel(problem)
    persistent keys kernels
    key={problem.cfg,problem.horizon,numel(problem.scenarios),problem.hasTarget};
    hit=false;
    for j=1:numel(keys)
        if isequaln(keys{j},key),kernel=kernels{j};hit=true;return;end
    end
    import casadi.*
    cfg=problem.cfg;of=cfg.outputFeedback;n=problem.horizon;ns=numel(problem.scenarios);
    nm=of.memoryDimension;nx=8+nm;np=numel(problem.initial);
    ix=reshape(np+(1:nx*n*ns),nx,n,ns);w=MX.sym('decision',np+numel(ix));
    stage=localOfSymbolicStage(cfg,problem.hasTarget);
    stageParameterCount=stage.size1_in(3);rootCount=6*ns+nm+2;
    p=MX.sym('parameter',rootCount+stageParameterCount*n*ns+problem.policyCount);
    pi=reshape(rootCount+(1:stageParameterCount*n*ns),stageParameterCount,n,ns);
    memory=p(6*ns+(1:nm));input=p(6*ns+nm+(1:2));
    g=cell(n,ns);cost=0;
    for s=1:ns
        state=[p((s-1)*6+(1:6));memory;input];
        for i=1:n
            witness=MX.zeros(6,1);
            if problem.hasTarget,witness=w(reshape(problem.witnessIndices(:,:,i,s),[],1));end
            [next,c,r]=stage(state,w(problem.policyIndices(:,i)),witness,p(pi(:,i,s)));
            g{i,s}=[w(ix(:,i,s))-next;c];cost=cost+p(pi(end,i,s))*dot(r,r)/2;
            state=w(ix(:,i,s));
        end
    end
    anchor=p(end-problem.policyCount+1:end);
    cost=cost+.5*of.regularization*dot(w(1:problem.policyCount)-anchor,w(1:problem.policyCount)-anchor);
    nc=numel(g{1});constraint=vertcat(g{:});
    gl=repmat([zeros(nx,1);-Inf(nc-nx,1)],n*ns,1);gu=zeros(size(gl));
    options=struct('print_time',false,'error_on_fail',false,'jit',of.nlpJit,'calc_lam_p',false, ...
        'ipopt',struct('print_level',0,'sb','yes','max_iter',of.nlpMaximumIterations, ...
        'tol',of.nlpTolerance,'constr_viol_tol',of.nlpTolerance, ...
        'bound_relax_factor',0,'warm_start_init_point','yes','mu_init',1e-3));
    if isfinite(of.maximumSolveSeconds),options.ipopt.max_wall_time=of.maximumSolveSeconds;end
    if of.nlpJit
        directory=tempname;mkdir(directory);
        options.compiler='shell';options.jit_options=struct( ...
            'flags',sprintf('-O%d',of.nlpCompilerOptimization),'compiler',char(of.nlpCompiler),'directory',[directory,filesep]);
        % CasADi writes the generated source in the current directory even
        % when the compiler output directory is set. Keep both outside Git.
        previousDirectory=pwd;restoreDirectory=onCleanup(@()cd(previousDirectory));
        cd(directory);
    end
    solver=nlpsol('joint_output_feedback','ipopt',struct('x',w,'p',p,'f',cost,'g',constraint),options);
    if of.nlpJit,clear restoreDirectory;end
    sl=[-Inf(3,1);0;-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum; ...
        -ones(nm,1);-of.steeringMaximum;max(-1,cfg.actuation.brakingRatioMinimum)];
    su=[Inf(3,1);cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum; ...
        ones(nm,1);of.steeringMaximum;min(1,cfg.actuation.brakingRatioMaximum)];
    kernel=struct('key',{key},'solver',solver,'stateIndices',ix,'parameterIndices',pi, ...
        'parameterCount',numel(p),'constraintLower',gl,'constraintUpper',gu, ...
        'constraintIndices',reshape(1:numel(gl),nc,n,ns), ...
        'stateLower',repmat(sl,n*ns,1),'stateUpper',repmat(su,n*ns,1));
    if numel(keys)>=8,keys=keys(2:end);kernels=kernels(2:end);end
    keys{end+1}=key;kernels{end+1}=kernel;
end

function [parameters,seed] = localOfNlpData(problem,kernel)
    cfg=problem.cfg;ns=numel(problem.scenarios);nm=cfg.outputFeedback.memoryDimension;
    l=cfg.outputFeedback.predictionSubsteps;dt=cfg.controller.sampleTime/l;
    parameters=zeros(kernel.parameterCount,1);seed=zeros(numel(problem.initial)+numel(kernel.stateIndices),1);
    seed(1:numel(problem.initial))=problem.initial;
    parameters(6*ns+(1:nm+2))=[problem.memory;problem.previousInput];
    parameters(end-problem.policyCount+1:end)=problem.initial(1:problem.policyCount);
    for s=1:ns
        item=problem.scenarios(s).initial;trace=problem.nominalTrajectories{s};
        parameters((s-1)*6+(1:6))=item.ego;
        for i=1:problem.horizon
            targets=zeros(11,l+1);
            if problem.hasTarget
                q=trace.targets(:,i);targets(:,1)=q;
                for j=1:l
                    if q(4)>0,q=predictiveSafetyGeometry.predictTarget(q,dt);end
                    targets(:,j+1)=q;
                end
            end
            projection=predictiveSafetyGeometry.project(trace.ego(1:2,i+1),problem.lane);
            parameters(kernel.parameterIndices(:,i,s))=[item.parameters;item.sensorBias; ...
                problem.scenarios(s).noise;targets(:);projection.point;projection.heading;problem.scenarios(s).weight];
            seed(kernel.stateIndices(:,i,s))=[trace.ego(:,i+1);trace.memory(:,i+1);trace.inputs(:,i)];
        end
    end
end

function stage = localOfSymbolicStage(cfg,hasTarget)
    import casadi.*
    of=cfg.outputFeedback;nm=of.memoryDimension;l=of.predictionSubsteps;
    z=SX.sym('state',8+nm);alpha=SX.sym('policy',2+2*nm+nm^2+9*nm);
    witness=SX.sym('witness',6);p=SX.sym('data',18+11*(l+1)+4);
    policy=localOfUnpack(alpha,cfg);x=z(1:6);m=z(7:6+nm);previous=z(7+nm:8+nm);
    u=policy.v+policy.K*m;h=cfg.controller.sampleTime;dt=h/l;
    states=x;feature=SX.zeros(8,1);q=reshape(p(19:18+11*(l+1)),11,l+1);
    for j=1:l
        x=nonlinearBicycleModel.symbolicSensorFlow(x,u,p(1:4),cfg,dt,j==l);states=[states,x]; %#ok<AGROW>
        dx=nonlinearBicycleModel.symbolicDerivative(x,u,p(1:4),cfg);
        relative=SX.zeros(2,1);detected=SX(0);
        if hasTarget
            relative=[cos(x(3)),sin(x(3));-sin(x(3)),cos(x(3))]*(q(1:2,j+1)-x(1:2));
            detected=dot(relative,relative)<=cfg.collision.encounterRangeMeters^2;
        end
        y=[x(1:2);dx(4)-x(6)*x(5);dx(5)+x(6)*x(4);x(6);relative]+p(5:11)+p(12:18);
        y=[y(1:5);if_else(detected,y(6:7),zeros(2,1))];
        feature=feature+[y./of.featureScale;detected]/l;
    end
    next=[x;tanh(policy.A*m+policy.B*feature+policy.c);u];
    ref=p(end-3:end-1);normal=[-sin(ref(3));cos(ref(3))];
    error=[normal.'*(x(1:2)-ref(1:2));atan2(sin(x(3)-ref(3)),cos(x(3)-ref(3))); ...
        x(4)-cfg.referenceSpeed;x(5);x(6)];
    residual=[sqrt(h*of.trackingWeights).*error;sqrt(of.inputWeight)*(u-previous)];
    c=[u(1)-of.steeringMaximum;-u(1)-of.steeringMaximum; ...
        u(2)-min(1,cfg.actuation.brakingRatioMaximum);max(-1,cfg.actuation.brakingRatioMinimum)-u(2)];
    if isfinite(of.steeringRateMaximum)
        change=h*of.steeringRateMaximum;c=[c;u(1)-previous(1)-change;previous(1)-u(1)-change];
    end
    if isfinite(cfg.model.brakingRatioRateMaximum)
        change=h*cfg.model.brakingRatioRateMaximum;c=[c;u(2)-previous(2)-change;previous(2)-u(2)-change];
    end
    c=[c;reshape([-states(4,:);states(4,:)-cfg.model.speedMaximum; ...
        states(5,:)-cfg.model.lateralVelocityMaximum;-states(5,:)-cfg.model.lateralVelocityMaximum; ...
        states(6,:)-cfg.model.yawRateMaximum;-states(6,:)-cfg.model.yawRateMaximum],[],1)];
    if hasTarget
        signs=[-1,1,1,-1;-1,-1,1,1];body=cfg.vehicle.rectangleOffset ...
            +[cfg.vehicle.length/2;cfg.vehicle.width/2].*signs;
        reserve=of.clearanceMeters+of.intersampleMarginMeters;checks=[1,1+l/2,1+l];
        for j=1:3
            pose=states(1:3,checks(j));target=q(:,checks(j));
            re=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
            rt=[cos(target(3)),-sin(target(3));sin(target(3)),cos(target(3))];
            normal=[cos(witness(2*j-1));sin(witness(2*j-1))];offset=witness(2*j);
            ego=pose(1:2)-target(1:2)+re*body;other=rt*(target(10:11)+target(8:9).*signs);
            c=[c;(offset+reserve-normal.'*ego).';(normal.'*other-offset).']; %#ok<AGROW>
        end
    end
    stage=Function('joint_stage',{z,alpha,witness,p},{next,c,residual});
end

function [result,info] = localCompactStep(request,cfg)
    % Fixed inference and a condensed PCBF solve with exact geometry dual values.
    timer=tic;block=request.block;prior=request.previousState;initial=isempty(prior);
    localOfValidateBlock(block,cfg);
    sensorCount=cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod;
    assert(abs(sensorCount-round(sensorCount))<1e-10 && sensorCount>=2 && mod(round(sensorCount),2)==0, ...
        'collisionAvoidanceController:invalidSensorGrid','A hold needs an even integer number of sensor samples.');
    assert(~cfg.controller.enforceRoadBoundaries, ...
        'collisionAvoidanceController:unsupportedCompactRoadConstraint', ...
        'Hard road-boundary constraints are not implemented by this transcription.');
    if isempty(request.reference),request.reference=[-1000,0;1000,0];end
    if initial
        assert(isfield(block,'initialHypotheses'), ...
            'collisionAvoidanceController:missingInitialInformation','Supply an explicit initial information contract.');
        assert(~isfield(block,'initialMemory') && ~isfield(block,'initialPolicies'), ...
            'collisionAvoidanceController:unsupportedCompactMemory','The compact controller has no optimized memory map.');
        hypotheses=localOfValidateHypotheses(block.initialHypotheses,cfg);
        assert(numel(block.samples)==1 && abs(block.samples.time-block.time)<1e-9, ...
            'collisionAvoidanceController:invalidInitialBlock','Initialization uses the current reading.');
        inherited=0;
    else
        assert(isfield(prior,'kind') && prior.kind=="compactOutputFeedback-v1", ...
            'collisionAvoidanceController:incompatibleContinuation','Expected a compact controller continuation.');
        assert(isequaln(prior.configuration,cfg) && isequaln(prior.reference,request.reference), ...
            'collisionAvoidanceController:changedContinuationProblem','The continuation contract changed.');
        assert(~isfield(block,'initialHypotheses') && ~isfield(block,'initialMemory') && ~isfield(block,'initialPolicies'), ...
            'collisionAvoidanceController:informationReset','Continuation cannot reset its information.');
        sensorCount=round(cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod);
        times=prior.time+(1:sensorCount)*cfg.outputFeedback.sensorPeriod;
        assert(abs(block.time-prior.time-cfg.controller.sampleTime)<1e-9 && ...
            numel(block.samples)==numel(times) && max(abs([block.samples.time]-times))<1e-9, ...
            'collisionAvoidanceController:invalidSensorGrid','Receive the complete consecutive block.');
        assert(norm(block.heldInput-prior.appliedInput,Inf)<1e-12, ...
            'collisionAvoidanceController:actuationMismatch','Measurements must use the committed input.');
        hypotheses=prior.hypotheses;inherited=prior.horizonSteps;
    end
    clock=tic;[hypotheses,conditioning]=localOfCondition(hypotheses,block,cfg,initial);
    inferenceSeconds=toc(clock);window=localOfWindow(hypotheses,inherited,cfg);
    count=window.horizonSteps;nodes=min(count,cfg.outputFeedback.controlNodes);
    if ~initial,nodes=size(prior.controls,2);end
    near=min(nodes-1,cfg.outputFeedback.nearControlNodes);
    boundaries=[0:near,near+round((1:nodes-near)*(count-near)/(nodes-near))];
    holds=diff(boundaries);h=cfg.controller.sampleTime;hasTarget=~isempty(hypotheses(1).target);
    % A representative retained member supplies the nominal dynamics. All
    % retained members are replayed before accepting a control sequence.
    values=[hypotheses.ego];[~,nominal]=min(sum((values-mean(values,2)).^2,1));item=hypotheses(nominal);
    x=item.ego;
    stub=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
    [~,lane]=readControllerInputs(stub,[],request.reference,cfg);
    [result,info]=localCondensedSolve(request,cfg,hypotheses,conditioning,window,holds,lane,nominal,timer,inferenceSeconds);
end

function [result,info] = localCondensedSolve(request,cfg,hypotheses,conditioning,window,holds,lane,nominal,timer,inferenceSeconds)
    persistent qpOptions
    if isempty(qpOptions),qpOptions=optimoptions('quadprog','Algorithm','active-set','Display','off','MaxIterations',200);end
    clock=tic;integrationCount=max(round(cfg.controller.sampleTime/cfg.outputFeedback.sensorPeriod),nonlinearBicycleModel.meshCount(cfg));
    if ~isempty(request.previousState) && isfield(request.previousState,'requiresFineIntegration') && request.previousState.requiresFineIntegration
        integrationCount=cfg.outputFeedback.validationSubsteps;
    end
    [kernel,cacheHit]=localCompactKernel(cfg,holds,~isempty(hypotheses(1).target),integrationCount);
    fineKernel=localCompactKernel(cfg,holds,~isempty(hypotheses(1).target));buildSeconds=toc(clock);
    count=sum(holds);nodes=numel(holds);nu=2*nodes;previous=request.block.heldInput;prior=request.previousState;
    initial=isempty(prior);h=cfg.controller.sampleTime;boundaries=[0,cumsum(holds)];
    model=localCondensedModel(cfg,hypotheses,previous,holds,lane,nominal,kernel);
    map=model.map;offset=model.offset;
    best=[];bestCost=Inf;bestTraces={};bestViolation=Inf;retained=false;validationSeconds=0;
    rootTerminal=false;initializationSource="shiftedFinePlan";
    if initial
        validationClock=tic;rootTerminal=true;
        for s=1:numel(hypotheses)
            item=hypotheses(s);
            [value,~]=localCompactTerminalCheck(item.ego,item.target,previous,item.parameters,fineKernel,cfg);
            if value>1e-6,rootTerminal=false;break;end
        end
        validationSeconds=validationSeconds+toc(validationClock);
        if rootTerminal
            inherited=repmat(terminalSafeSet.compactControl(previous,cfg),1,count);
            validationClock=tic;[valid,bestTraces,bestViolation]=localCompactValidate(inherited,hypotheses,previous,fineKernel,cfg,true);
            validationSeconds=validationSeconds+toc(validationClock);
            if valid,best=inherited;retained=true;end
        end
        seed=localCompactSeeds(hypotheses(nominal),previous,holds,lane,kernel,cfg);controls=seed{1};
        initializationSource="singleCirculationRollout";
    else
        extension=terminalSafeSet.compactControl(prior.fineControls(:,end),cfg);
        inherited=prior.fineControls(:,2:end);inherited=[inherited,repmat(extension,1,count-size(inherited,2))];
        validationClock=tic;[valid,bestTraces,bestViolation]=localCompactValidate(inherited,hypotheses,previous,fineKernel,cfg,true);
        validationSeconds=validationSeconds+toc(validationClock);
        if valid
            best=inherited;
            retained=true;
        end
        search=inherited;
        if isfield(prior,'searchFineControls')
            search=prior.searchFineControls(:,2:end);
            search=[search,repmat(extension,1,count-size(search,2))];
        end
        controls=localCompactRateLimit(search(:,boundaries(2:end)),previous,holds,cfg);
    end
    controls=localCompactReleaseRestingBrakes(controls,hypotheses,previous,kernel,cfg);
    if ~isempty(best)
        bestCost=localCompactPerformance(bestTraces{nominal}.ego(:,2:end),best,model);
    end
    inheritedCost=bestCost;
    [a,b]=localCondensedInputRows(previous,holds,cfg);lower=repmat([-cfg.outputFeedback.steeringMaximum;max(-1,cfg.actuation.brakingRatioMinimum)],nodes,1);
    upper=repmat([cfg.outputFeedback.steeringMaximum;min(1,cfg.actuation.brakingRatioMaximum)],nodes,1);
    if max([a*controls(:)-b;lower-controls(:);controls(:)-upper])>cfg.outputFeedback.nlpTolerance
        % Project the one warm trajectory onto the complete linear actuator
        % contract, including the terminal handoff, before nonlinear search.
        [projected,~,flag]=quadprog(eye(nu),-controls(:),a,b,[],[],lower,upper,controls(:),qpOptions);
        if flag>0,controls=reshape(projected,2,nodes);end
    end
    lastIterate=controls(:);
    cacheU=[];checkedU=[];cache=[];calls=0;phase=1;needsRefinement=false;solveClock=tic;
    budget=cfg.outputFeedback.maximumSolveSeconds;
    if isempty(best)
        budget=cfg.outputFeedback.recoverySolveSeconds;
        if initial,budget=cfg.outputFeedback.initializationSolveSeconds;end
        if rootTerminal,budget=cfg.outputFeedback.maximumSolveSeconds;end
    end
    evaluation=evaluate(controls(:));initialDeficit=sum(max(0,evaluation.stage));initialTerminal=max(evaluation.terminal);
    restoration=isempty(best);iterationCount=0;phaseIterations=0;exitFlag=0;attempts=0;performanceSteps=0;
    if isempty(best)
        options=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
            'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true, ...
            'ConstraintTolerance',max(1e-7,cfg.outputFeedback.nlpTolerance),'OptimalityTolerance',1e-5, ...
            'StepTolerance',1e-9,'MaxIterations',cfg.outputFeedback.restorationMaximumIterations, ...
            'MaxFunctionEvaluations',10000,'OutputFcn',@progress);
    end
    while isempty(best) && (attempts==0 || needsRefinement)
        if needsRefinement,model.kernel=fineKernel;cacheU=[];needsRefinement=false;evaluation=evaluate(controls(:));end
        z=[controls(:);max(0,evaluation.stage)];attempts=attempts+1;phaseIterations=0;
        try
            [z,~,exitFlag,output]=fmincon(@objective,z,[a,zeros(size(a,1),count)],b,[],[], ...
                [lower;zeros(count,1)],[upper;Inf(count,1)],@constraints,options);
            iterationCount=iterationCount+output.iterations;controls=reshape(z(1:nu),2,nodes);lastIterate=z(1:nu);
        catch exception
            if ~strcmp(exception.identifier,'collisionAvoidanceController:condensedDeadline'),rethrow(exception);end
            exitFlag=-1;iterationCount=iterationCount+phaseIterations;controls=reshape(lastIterate,2,nodes);
        end
        if toc(solveClock)>=budget,break;end
    end
    % A checked zero-deficit plan attains the global lower bound of the PCBF
    % primary objective. Only then may performance replace the stored plan.
    while ~isempty(best) && toc(solveClock)<budget && iterationCount<cfg.outputFeedback.nlpMaximumIterations
        if needsRefinement
            model.kernel=fineKernel;cacheU=[];needsRefinement=false;
        end
        phase=2;attempts=attempts+1;phaseIterations=0;
        try
            e=evaluate(lastIterate);c=[e.stage;e.forward;e.terminal];
            jac=[e.stageGradient,e.forwardGradient,e.terminalGradient].';
            penalty=max(100,10*norm(e.costGradient,Inf));
            % Elasticity globalizes the numerical step only. Executable plans
            % still require zero PCBF deficit and hard terminal membership.
            H=blkdiag((e.costHessian+e.costHessian.')/2+1e-4*eye(nu),1);
            rows=[jac,-ones(numel(c),1);a,zeros(size(a,1),1)];rhs=[-c;b-a*lastIterate];
            start=[zeros(nu,1);max([0;c])];
            [step,~,flag]=quadprog(H,[e.costGradient;penalty],rows,rhs,[],[], ...
                [lower-lastIterate;0],[upper-lastIterate;Inf],start,qpOptions);
            if flag<=0,exitFlag=-2;break;end
            direction=step(1:nu);iterationCount=iterationCount+1;
            if norm(direction,Inf)<1e-7
                exitFlag=-3;if max(c)<=cfg.outputFeedback.nlpTolerance,exitFlag=1;end
                break;
            end
            merit=e.cost+penalty*sum(max(0,c));
            decrease=e.costGradient.'*direction+penalty*(sum(max(0,c+jac*direction))-sum(max(0,c)));
            accepted=false;fraction=1;
            while fraction>=1/128 && toc(solveClock)<budget
                trial=lastIterate+fraction*direction;value=evaluate(trial,false);
                trialMerit=value.cost+penalty*sum(max(0,[value.stage;value.forward;value.terminal]));
                if trialMerit<=merit+1e-4*fraction*min(-eps,decrease)
                    lastIterate=trial;accepted=true;performanceSteps=performanceSteps+1;break;
                end
                fraction=fraction/2;
            end
            if ~accepted,exitFlag=-3;break;end
            exitFlag=0;
        catch exception
            if ~strcmp(exception.identifier,'collisionAvoidanceController:condensedDeadline'),rethrow(exception);end
            exitFlag=-1;break;
        end
    end
    if exitFlag==0 && toc(solveClock)>=budget,exitFlag=-1;end
    assert(~isempty(best),'collisionAvoidanceController:compactNoFeasiblePlan', ...
        'No checked feasible PCBF plan; last evaluated clearance deficit %.6g, terminal violation %.6g after %d evaluations. This is not a proof of physical infeasibility.', ...
        sum(max(0,cache.stage)),max(cache.terminal),calls);
    % Reuse the complete replay of the selected plan. Only actuator
    % normalization that actually changes a command needs another replay.
    solveSeconds=toc(solveClock);clipped=localCompactRateLimit(best,previous,ones(1,count),cfg);
    if ~isequal(clipped,best)
        validationClock=tic;[valid,bestTraces,bestViolation]=localCompactValidate(clipped,hypotheses,previous,fineKernel,cfg,true);
        validationSeconds=validationSeconds+toc(validationClock);
        assert(valid,'collisionAvoidanceController:compactNoFeasiblePlan','Final condensed-plan replay failed.');
    end
    best=clipped;traces=bestTraces;violation=bestViolation;
    input=best(:,1);state=struct('kind',"compactOutputFeedback-v1",'time',request.block.time,'memory',zeros(0,1), ...
        'hypotheses',hypotheses,'appliedInput',input,'controls',best(:,boundaries(2:end)), ...
        'fineControls',best,'searchFineControls',reshape(lastIterate,2,nodes)*map.'+offset,'boundaries',boundaries,'horizonSteps',count,'configuration',cfg, ...
        'reference',request.reference,'conditioning',conditioning,'terminal',traces{nominal}.terminal, ...
        'pcbfDeficit',0,'stageDeficits',zeros(count,1), ...
        'requiresFineIntegration',needsRefinement || model.kernel.integrationSubsteps==fineKernel.integrationSubsteps);
    info=struct('solver',"condensedNonlinearPcbfSqp",'nlpStatus',string(exitFlag), ...
        'nlpConverged',exitFlag>0,'nlpIterations',iterationCount,'nlpSeconds',solveSeconds, ...
        'nlpAttempts',attempts,'nlpBuildSeconds',buildSeconds,'nlpCacheHit',cacheHit, ...
        'nlpVariables',nu+count,'nlpConstraints',2*count+numel(hypotheses), ...
        'informationUpdateSeconds',inferenceSeconds,'nonlinearValidationSeconds',validationSeconds, ...
        'elapsedSeconds',toc(timer),'deadlineSeconds',h,'predictionWindow',window, ...
        'controlNodes',nodes,'horizonSteps',count,'initializationTrajectoryCount',1,'initializationSource',initializationSource, ...
        'fallbackUsed',retained,'retainedFinePlanUsed',retained && ~initial,'rootTerminalPlanUsed',retained && initial,'retainedTailRepaired',false, ...
        'restorationAttempted',restoration,'initialRestorationSlack',initialDeficit, ...
        'initialNlpViolation',max(0,initialTerminal),'finalRestorationSlack',0, ...
        'validationSubsteps',kernel.validationSubsteps,'nonlinearValidationPerformed',true, ...
        'maximumConstraintViolation',violation,'scenarioCount',numel(hypotheses), ...
        'activeScenarioCount',numel(hypotheses),'hypothesisCount',numel(hypotheses),'memoryDimension',0, ...
        'conditioning',conditioning,'terminalSetEnforced',true,'terminalSafetyAssumed',true, ...
        'terminalModel',"straighteningBrakingThenParked",'terminalValue',max(cellfun(@(t)t.terminal.value,traces)), ...
        'terminalInput',traces{nominal}.terminal.input,'robustNonlinearSafetyCertified',false, ...
        'recursiveFeasibilityCertified',false,'safetyScope',"finiteHypothesesDenseNonlinearReplay", ...
        'wholeHoldReserveScope',"sampledMotionEstimateAndDenseChecksNoCertificate", ...
        'predictionModel',"condensedSharedInputNonlinearRollout",'previousInputUsedForMeasurements',previous, ...
        'optimizationReturned',true,'pcbfValue',0,'primaryOptimumComputed',true, ...
        'inheritedPerformanceCost',inheritedCost,'acceptedPerformanceCost',bestCost, ...
        'performanceAcceptedSteps',performanceSteps, ...
        'safetyObjective',"sumOfHorizonSafetySlacks",'nonlinearOracleEvaluations',calls);
    info.deadlineMissed=info.elapsedSeconds>h;
    command=struct('actuatorInput',input,'steeringAngle',input(1),'brakingRatio',input(2));
    prediction=struct('metadata',info,'scenarios',{traces},'memory',zeros(0,1),'information',hypotheses);
    result=struct('command',command,'inputs',best,'prediction',prediction,'state',state);

    function e=evaluate(u,derivatives)
        if nargin<2,derivatives=true;end
        if isempty(cacheU)||~isequal(u,cacheU)||(derivatives && ~cache.derivatives)
            if calls>0 && toc(solveClock)>=budget
                error('collisionAvoidanceController:condensedDeadline','The nonlinear evaluation budget elapsed.');
            end
            [cache,rollouts]=localCondensedOracle(u,model,derivatives);cacheU=u;calls=calls+1;
            feasible=max([cache.stage;cache.forward;cache.terminal;a*u-b;lower-u;u-upper])<=cfg.outputFeedback.nlpTolerance;
            promising=feasible && cache.cost<bestCost && ~isequal(u,checkedU);
            if promising
                candidate=reshape(u,2,nodes)*map.'+offset;
                candidate=localCompactRateLimit(candidate,previous,ones(1,count),cfg);
                checkedU=u;
                validationClock=tic;[feasible,candidateTraces,candidateViolation]=localCompactValidate(candidate,hypotheses,previous,fineKernel,cfg,true,rollouts);
                validationSeconds=validationSeconds+toc(validationClock);
                needsRefinement=~feasible && model.kernel.integrationSubsteps<fineKernel.integrationSubsteps;
                if feasible
                    candidateCost=localCompactPerformance(candidateTraces{nominal}.ego(:,2:end),candidate,model);
                    feasible=candidateCost<bestCost;
                end
            end
            if promising && feasible
                if initial && isempty(best)
                    budget=min(budget,toc(solveClock)+cfg.outputFeedback.maximumSolveSeconds);
                end
                best=candidate;bestCost=candidateCost;bestTraces=candidateTraces;bestViolation=candidateViolation;retained=false;
            end
        end
        e=cache;
    end
    function [f,g]=objective(z)
        e=evaluate(z(1:nu));
        if phase==1,f=sum(z(nu+1:end));g=[zeros(nu,1);ones(count,1)];else,f=e.cost;g=e.costGradient;end
    end
    function [c,ceq,gc,gceq]=constraints(z)
        e=evaluate(z(1:nu));ceq=[];gceq=[];
        if phase==1
            c=[e.stage-z(nu+1:end);e.forward;e.terminal];
            gc=[e.stageGradient,e.forwardGradient,e.terminalGradient;-eye(count),zeros(count,count+numel(hypotheses))];
        else,c=[e.stage;e.forward;e.terminal];gc=[e.stageGradient,e.forwardGradient,e.terminalGradient];end
    end
    function stop=progress(z,values,status)
        if strcmp(status,'iter') || strcmp(status,'init'),lastIterate=z(1:nu);phaseIterations=values.iteration;end
        stop=toc(solveClock)>=budget || needsRefinement || (phase==1 && ~isempty(best));
    end
end

function model=localCondensedModel(cfg,hypotheses,previous,holds,lane,nominal,kernel)
    count=sum(holds);nodes=numel(holds);h=cfg.controller.sampleTime;boundaries=[0,cumsum(holds)];
    map=zeros(count,nodes);offset=zeros(2,count);row=0;
    for i=1:nodes
        for j=1:holds(i)
            row=row+1;fraction=j/holds(i);map(row,i)=fraction;
            if i==1,offset(:,row)=(1-fraction)*previous;else,map(row,i-1)=1-fraction;end
        end
    end
    controlMap=kron(map,eye(2));
    model=struct('cfg',cfg,'hypotheses',hypotheses,'previous',previous,'kernel',kernel, ...
        'map',map,'offset',offset,'controlMap',controlMap,'nominal',nominal,'reference',zeros(3,nodes));
    x=hypotheses(nominal).ego;
    for i=1:nodes
        p=x(1:2)+boundaries(i+1)*h*cfg.referenceSpeed*[cos(x(3));sin(x(3))];
        projection=predictiveSafetyGeometry.project(p,lane);model.reference(:,i)=[projection.point;projection.heading];
    end
end

function [a,b]=localCondensedInputRows(previous,holds,cfg)
    n=numel(holds);a=zeros(0,2*n);b=zeros(0,1);rates=[cfg.outputFeedback.steeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    for i=1:n
        for k=1:2
            if ~isfinite(rates(k)),continue;end
            row=zeros(1,2*n);row(2*i-2+k)=1;origin=0;
            if i>1,row(2*i-4+k)=-1;else,origin=previous(k);end
            a=[a;row;-row];b=[b;rates(k)*cfg.controller.sampleTime*holds(i)+origin;rates(k)*cfg.controller.sampleTime*holds(i)-origin];
        end
    end
    terminal=terminalSafeSet.compactControl(previous,cfg);
    for k=1:2
        if ~isfinite(rates(k)),continue;end
        row=zeros(1,2*n);row(2*n-2+k)=1;
        a=[a;row;-row];b=[b;terminal(k)+rates(k)*cfg.controller.sampleTime;-terminal(k)+rates(k)*cfg.controller.sampleTime];
    end

end

function [kernel,hit] = localCompactKernel(cfg,holds,hasTarget,integrationCount)
    persistent keys kernels
    if nargin<4,integrationCount=cfg.outputFeedback.validationSubsteps;end
    integrationCount=min(cfg.outputFeedback.validationSubsteps,integrationCount);
    if mod(cfg.outputFeedback.validationSubsteps,integrationCount)~=0,integrationCount=cfg.outputFeedback.validationSubsteps;end
    key={cfg,numel(holds),integrationCount};hit=false;
    for k=1:numel(keys)
        if isequaln(key,keys{k})&&sum(holds)<=sum(kernels{k}.capacity)
            kernel=kernels{k};kernel.holds=holds;hit=true;return;
        end
    end
    import casadi.*
    of=cfg.outputFeedback;h=cfg.controller.sampleTime;n=numel(holds);
    maximumSpeed=hypot(cfg.model.speedMaximum,cfg.model.lateralVelocityMaximum);
    maximumCount=ceil(max(of.minimumPreviewSeconds,maximumSpeed/of.previewDecelerationMetersPerSecondSquared+of.previewSettlingSeconds)/h);
    capacity=max(sum(holds),maximumCount);K=of.validationSubsteps;
    M=integrationCount;D=K/M;dt=h/M;
    x=SX.sym('x',6);u=SX.sym('u',2);p=SX.sym('p',4);fraction=SX.sym('fraction',5);
    k1=nonlinearBicycleModel.symbolicDerivative(x,u,p,cfg);
    k2=nonlinearBicycleModel.symbolicDerivative(x+dt*k1/2,u,p,cfg);
    k3=nonlinearBicycleModel.symbolicDerivative(x+dt*k2/2,u,p,cfg);
    k4=nonlinearBicycleModel.symbolicDerivative(x+dt*k3,u,p,cfg);
    next=x+dt*(k1+2*k2+2*k3+k4)/6;
    options=struct;
    if of.nlpJit
        directory=tempname;mkdir(directory);options=struct('jit',true,'compiler','shell', ...
            'jit_options',struct('flags',sprintf('-O%d',of.nlpCompilerOptimization), ...
            'compiler',char(of.nlpCompiler),'directory',[directory,filesep]));
    end
    points=cell(1,D);
    for j=1:D
        t=j/D;points{j}=x+dt*((t-1.5*t^2+2/3*t^3)*k1 ...
            +(t^2-2/3*t^3)*(k2+k3)+(-.5*t^2+2/3*t^3)*k4);
    end
    step=Function('pcbf_step',{x,u,p},{next,horzcat(points{:})},options);
    root=MX.sym('root',6);command=MX.sym('command',2);parameters=MX.sym('parameters',4);
    mappedStep=step.mapaccum(M);[ends,points]=mappedStep(root,repmat(command,1,M),repmat(parameters,1,M));
    last=ends(:,end);stopped=(command(2)<=0)&(last(4)<.2*cfg.model.brakeHoldSpeed);
    finish=[last(1:3);if_else(stopped,zeros(3,1),last(4:6))];
    hold=Function('pcbf_hold',{root,command,parameters},{finish,points});
    % RK4 continuous extension supplies dense geometry and its derivatives
    % without a new dynamics integration at every collision sample.
    sensitivity=SX.sym('sensitivity',6,8);
    updated=jacobian(next,x)*sensitivity+[SX.zeros(6,6),jacobian(next,u)];
    index=MX.sym('index',5);initialSensitivity=DM([eye(6),zeros(6,2)]);
    if D==1
        % On the fine mesh, dense samples are RK4 endpoints. Propagate one
        % sensitivity and select the requested samples before the hold reset.
        augmented=[x;sensitivity(:)];
        variation=Function('pcbf_variation',{augmented,u,p},{[next;updated(:)]},options);
        initialVariation=[root;initialSensitivity(:)];
        variationMap=variation.mapaccum(M);
        varying=variationMap(initialVariation,repmat(command,1,M),repmat(parameters,1,M));
        history=densify([initialVariation,varying]);selected=cell(1,5);
        for j=1:5,selected{j}=reshape(history(7:54,index(j)+1),6,8);end
        E=reshape(varying(7:54,end),6,8);D=horzcat(selected{:});
    else
        chosen=SX.sym('chosen',6,40);augmented=[x;sensitivity(:);chosen(:)];selected=cell(1,5);
        for j=1:5
            t=fmin(1,fmax(0,fraction(j)));
            point=x+dt*((t-1.5*t^2+2/3*t^3)*k1+(t^2-2/3*t^3)*(k2+k3)+(-.5*t^2+2/3*t^3)*k4);
            variation=jacobian(point,x)*sensitivity+[SX.zeros(6,6),jacobian(point,u)];
            selected{j}=if_else((fraction(j)>0)&(fraction(j)<=1),variation,chosen(:,8*(j-1)+(1:8)));
        end
        selected=horzcat(selected{:});update=[next;updated(:);selected(:)];
        variation=Function('pcbf_variation',{augmented,u,p,fraction},{update},options);
        variationMap=variation.mapaccum(M);
        varying=variationMap([root;initialSensitivity(:);repmat(initialSensitivity(:),5,1)], ...
            repmat(command,1,M),repmat(parameters,1,M),repmat(index/D,1,M)-repmat(0:M-1,5,1));
        E=reshape(varying(7:54,end),6,8);D=reshape(varying(55:294,end),6,40);
    end
    % At exact rest and neutral force, select the outgoing acceleration
    % branch. The reset value is unchanged; nonlinear replay accepts steps.
    starting=(command(2)==0)&(dot(root(4:6),root(4:6))==0);
    stopped=(command(2)<=0)&(varying(4,end)<.2*cfg.model.brakeHoldSpeed)&~starting;
    E=[E(1:3,:);if_else(stopped,MX.zeros(3,8),E(4:6,:))];
    derivative=Function('pcbf_sensitivity',{root,command,parameters,index},{E,D},options);
    identity=Function('pcbf_identity',{x,u,p},{x,repmat(x,1,K)});
    switchFlow=Function.if_else('pcbf_active',hold,identity);
    root=MX.sym('root',6);input=MX.sym('input',2);scale=MX.sym('scale',4);active=MX.sym('active');
    [endpoint,dense]=switchFlow(active,root,input,scale);
    piece=Function('pcbf_piece',{root,input,scale,active},{endpoint,dense});mapped=piece.mapaccum(capacity);
    inputs=MX.sym('inputs',2,capacity);enabled=MX.sym('enabled',1,capacity);
    [ends,points]=mapped(root,inputs,repmat(scale,1,capacity),enabled);
    rollout=Function('pcbf_rollout',{root,inputs,scale,enabled},{points,ends},options);
    kernel=struct('refinedRollout',rollout,'sensitivity',derivative,'capacity',capacity, ...
        'holds',holds,'validationSubsteps',K,'integrationSubsteps',M,'hasTarget',hasTarget);
    keys{end+1}=key;kernels{end+1}=kernel;
end

function [e,rollouts]=localCondensedOracle(u,model,derivatives)
    if nargin<3,derivatives=true;end
    cfg=model.cfg;kernel=model.kernel;N=sum(kernel.holds);n=numel(kernel.holds);nu=numel(u);K=kernel.validationSubsteps;h=cfg.controller.sampleTime;
    controls=reshape(u,2,n);inputs=controls*model.map.'+model.offset;capacity=sum(kernel.capacity);
    rollouts=[];
    if nargout>1 && kernel.integrationSubsteps==K
        % This cache belongs to one evaluation only. Fine validation checks
        % its complete contract before reusing propagation, never verdicts.
        rollouts=struct('hypotheses',model.hypotheses,'inputs',inputs,'previous',model.previous, ...
            'cfg',cfg,'holds',kernel.holds,'integrationSubsteps',kernel.integrationSubsteps, ...
            'trajectories',{cell(1,numel(model.hypotheses))});
    end
    values=[inputs,zeros(2,capacity-N)];enabled=double((1:capacity)<=N);times=(0:K*N)*h/K;
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    e=struct('stage',-Inf(N,1),'stageGradient',zeros(nu,N),'forward',-Inf(N,1), ...
        'forwardGradient',zeros(nu,N),'terminal',zeros(numel(model.hypotheses),1), ...
        'terminalGradient',zeros(nu,numel(model.hypotheses)),'cost',0,'costGradient',zeros(nu,1), ...
        'costHessian',zeros(nu),'derivatives',derivatives);
    for s=1:numel(model.hypotheses)
        item=model.hypotheses(s);[dense,ends]=kernel.refinedRollout(item.ego,values,item.parameters,enabled);
        ends=full(ends);dense=full(dense);ends=ends(:,1:N);dense=[item.ego,dense(:,1:K*N)];
        [backupDense,backupEnds,backupInputs]=localCompactBackupRollout(kernel,ends(:,end),item.parameters,inputs(:,end),cfg);
        if ~isempty(rollouts)
            rollouts.trajectories{s}=struct('dense',dense,'states',ends(:,cumsum(kernel.holds)), ...
                'backup',struct('dense',backupDense,'states',backupEnds,'inputs',backupInputs));
        end
        ends=[ends,backupEnds];dense=[dense,backupDense(:,2:end)];
        allInputs=[inputs,backupInputs];total=size(allInputs,2);times=(0:K*total)*h/K;
        roots=[item.ego,ends(:,1:end-1)];derivativeModel=model;
        backupMap=zeros(2,nu);
        derivativeModel.controlMap=[model.controlMap;repmat(backupMap,total-N,1)];
        q=[];localStage=-ones(1,total);indices=1+(0:total-1)*K;weights=ones(1,total);travelWeights=zeros(1,total);
        radiusE=norm(shape(1:2))+norm(shape(3:4));
        [speed,speedIndex]=max(hypot(dense(4,:),dense(5,:)));[yaw,yawIndex]=max(abs(dense(6,:)));
        if ~isempty(item.target)
            if item.target(4)>0,q=predictiveSafetyGeometry.predictTarget(item.target,times);else,q=repmat(item.target,1,numel(times));end
            gap=localCompactExactClearance(dense,q,shape);
            radiusT=norm(q(8:9,1))+norm(q(10:11,1));targetSpeed=max(q(4,:));
            L=speed+radiusE*yaw+targetSpeed+radiusT*abs(sin(q(6,1))/q(7,1))*targetSpeed;
            a=gap(1:end-1);b=gap(2:end);travel=L*h/K;
            [lower,branch]=max([a-travel;b-travel;(a+b-travel)/2],[],1);
            [localStage,j]=max(reshape(cfg.outputFeedback.clearanceMeters-lower,K,total),[],1);
            indices=(0:total-1)*K+j;branch=branch(indices);weights=.5*ones(1,total);weights(branch==1)=1;weights(branch==2)=0;
            travelWeights=ones(1,total);travelWeights(branch==3)=.5;
        end
        [forward,fi,domainState,domainInput]=localCompactModelDomain(dense,allInputs,cfg,K);
        if ~derivatives
            e.stage=max(e.stage,localStage(1:N).');e.forward=max(e.forward,forward(1:N).');
            target=[];if ~isempty(q),target=q(:,end);end
            context=terminalSafeSet.compactContext(target,cfg);
            terminal=terminalSafeSet.nonlinearValue(context,ends(:,end),0);
            e.terminal(s)=max([terminal,localStage(N+1:end),forward(N+1:end)]);
            if s==model.nominal,e.cost=localCompactPerformance(ends(:,cumsum(kernel.holds)),inputs,model);end
            continue;
        end
        si=max(1,ceil((speedIndex-1)/K));sj=max(1,ceil((yawIndex-1)/K));
        speedSample=zeros(1,total);speedSample(si)=speedIndex-1-(si-1)*K;
        yawSample=zeros(1,total);yawSample(sj)=yawIndex-1-(sj-1)*K;
        [SC,SE]=localCondensedSensitivities(roots,allInputs,item.parameters, ...
            [mod(indices-1,K);mod(indices-1,K)+1;fi;speedSample;yawSample],derivativeModel);
        S1=SC(1,:);S2=SC(2,:);SF=SC(3,:);speedS=SC{4,si};yawS=SC{5,sj};
        dL=zeros(1,nu);
        if speed>0,dL=dense(4:5,speedIndex).'/speed*speedS(4:5,:);end
        dL=dL+radiusE*sign(dense(6,yawIndex))*yawS(6,:);
        if ~isempty(q)
            grad1=localCompactDistanceGradient(dense(:,indices),q(:,indices),shape);
            grad2=localCompactDistanceGradient(dense(:,indices+1),q(:,indices+1),shape);
        end
        stageGradient=zeros(nu,total);
        for i=1:total
            if ~isempty(q)
                stageGradient(:,i)=(-weights(i)*grad1(:,i).'*S1{i}(1:3,:) ...
                    -(1-weights(i))*grad2(:,i).'*S2{i}(1:3,:)+travelWeights(i)*h/K*dL).';
            end
            domainGradient=(domainState(:,i).'*SF{i}+domainInput(:,i).'*derivativeModel.controlMap(2*i-1:2*i,:)).';
            if i<=N
                if localStage(i)>e.stage(i),e.stage(i)=localStage(i);e.stageGradient(:,i)=stageGradient(:,i);end
                if forward(i)>e.forward(i),e.forward(i)=forward(i);e.forwardGradient(:,i)=domainGradient;end
            end
        end
        target=[];if ~isempty(q),target=q(:,end);end
        context=terminalSafeSet.compactContext(target,cfg);
        [terminal,g]=terminalSafeSet.nonlinearTangent(context,ends(:,end),0);
        e.terminal(s)=terminal;e.terminalGradient(:,s)=(g*SE{end}).';
        if total>N
            [backupValue,index]=max(localStage(N+1:end));
            if backupValue>e.terminal(s),e.terminal(s)=backupValue;e.terminalGradient(:,s)=stageGradient(:,N+index);end
            [backupForward,index]=max(forward(N+1:end));
            if backupForward>e.terminal(s)
                j=N+index;e.terminal(s)=backupForward;
                e.terminalGradient(:,s)=(domainState(:,j).'*SF{j}+domainInput(:,j).'*derivativeModel.controlMap(2*j-1:2*j,:)).';
            end
        end
        if s==model.nominal
            for i=1:n
                state=ends(:,sum(kernel.holds(1:i)));S=SE{sum(kernel.holds(1:i))+1};ref=model.reference(:,i);
                normal=[-sin(ref(3));cos(ref(3))];error=[normal.'*(state(1:2)-ref(1:2));atan2(sin(state(3)-ref(3)),cos(state(3)-ref(3)));state(4)-cfg.referenceSpeed;state(5:6)];
                derivative=[normal.'*S(1:2,:);S(3:6,:)];
                e.cost=e.cost+.5*h*kernel.holds(i)*sum(cfg.outputFeedback.trackingWeights.*error.^2);
                e.costGradient=e.costGradient+h*kernel.holds(i)*derivative.'*(cfg.outputFeedback.trackingWeights.*error);
                e.costHessian=e.costHessian+h*kernel.holds(i)*derivative.'*(cfg.outputFeedback.trackingWeights.*derivative);
            end
        end
    end
    if ~derivatives,return;end
    differences=diff([model.previous,controls],1,2);
    e.cost=e.cost+.5*cfg.outputFeedback.inputWeight*sum(sum(differences.^2)./kernel.holds);
    du=cfg.outputFeedback.inputWeight*differences./kernel.holds;
    e.costGradient=e.costGradient+reshape(du-[du(:,2:end),zeros(2,1)],[],1);
    difference=eye(n)-diag(ones(n-1,1),-1);
    e.costHessian=e.costHessian+cfg.outputFeedback.inputWeight*kron(difference.'*diag(1./kernel.holds)*difference,eye(2));
end

function [selected,ends]=localCondensedSensitivities(roots,inputs,parameters,index,model)
    N=size(inputs,2);nu=size(model.controlMap,2);selected=cell(5,N);ends=cell(1,N+1);ends{1}=zeros(6,nu);
    mapped=model.kernel.sensitivity.map(N);
    [allEnd,allSelected]=mapped(roots,inputs,repmat(parameters,1,N),index);
    allEnd=full(allEnd);allSelected=full(allSelected);
    for i=1:N
        E=allEnd(:,8*(i-1)+(1:8));U=model.controlMap(2*i-1:2*i,:);
        for j=1:5
            D=allSelected(:,40*(i-1)+8*(j-1)+(1:8));
            selected{j,i}=D(:,1:6)*ends{i}+D(:,7:8)*U;
        end
        ends{i+1}=E(:,1:6)*ends{i}+E(:,7:8)*U;
    end
end

function gradient=localCompactDistanceGradient(states,targets,shape)
    [~,gradient]=localCompactExactClearance(states,targets,shape);
end

function value=localCompactPerformance(states,inputs,model)
    cfg=model.cfg;reference=model.reference;
    lateral=-sin(reference(3,:)).*(states(1,:)-reference(1,:)) ...
        +cos(reference(3,:)).*(states(2,:)-reference(2,:));
    angle=states(3,:)-reference(3,:);
    error=[lateral;atan2(sin(angle),cos(angle));states(4,:)-cfg.referenceSpeed;states(5:6,:)];
    value=.5*cfg.controller.sampleTime*sum(model.kernel.holds.*sum(cfg.outputFeedback.trackingWeights.*error.^2,1)) ...
        +.5*cfg.outputFeedback.inputWeight*sum(diff([model.previous,inputs],1,2).^2,'all');
end

function controls=localCompactReleaseRestingBrakes(controls,hypotheses,previous,kernel,cfg)
    % Nonpositive commands are physically identical at absorbing rest. Use
    % neutral braking there to expose the one-sided acceleration derivative;
    % the retained executable plan is kept separately and remains untouched.
    resting=true(1,numel(kernel.holds));allInputs=[];
    for s=1:numel(hypotheses)
        item=hypotheses(s);[~,states,allInputs]=localCompactRollout(kernel,item.ego,previous,controls,item.parameters,false);
        roots=[item.ego,states(:,1:end-1)];resting=resting & all(roots(4:6,:)==0,1);
        if ~any(resting),break;end
    end
    boundary=[0,cumsum(kernel.holds)];
    for i=1:numel(kernel.holds)
        if resting(i) && all(allInputs(2,boundary(i)+1:boundary(i+1))<=0),controls(2,i)=0;end
    end
    controls=localCompactRateLimit(controls,previous,kernel.holds,cfg);
end

function controls = localCompactRateLimit(controls,previous,holds,cfg)
    for i=1:numel(holds)
        span=cfg.controller.sampleTime*holds(i);
        change=[cfg.outputFeedback.steeringRateMaximum;cfg.model.brakingRatioRateMaximum]*span;
        controls(:,i)=max(previous-change,min(previous+change,controls(:,i)));
        controls(:,i)=max([-cfg.outputFeedback.steeringMaximum;max(-1,cfg.actuation.brakingRatioMinimum)], ...
            min([cfg.outputFeedback.steeringMaximum;min(1,cfg.actuation.brakingRatioMaximum)],controls(:,i)));
        previous=controls(:,i);
    end
end

function seeds = localCompactSeeds(item,previous,holds,lane,kernel,cfg)
    % One geometric-planner rollout initializes the optimizer.
    count=sum(holds);x=item.ego;
    frame=predictiveSafetyGeometry.roadFrame(lane,struct('lateralClearance',[]));
    nominal=struct('state',[0;0;0;cfg.referenceSpeed;0;0],'curvature',frame(4));
    model=struct('cfg',cfg,'initialState',x,'previousInput',previous,'target',item.target, ...
        'lane',lane,'road',struct('lateralClearance',[]),'frame',frame,'nominalReference',nominal,'compactInitialization',true, ...
        'uncertainty',struct('collisionGenerator',zeros(6,0),'target',[]));
    if isempty(item.target)
        inputs=repmat([0;min(1,max(-1,.25*(cfg.referenceSpeed-x(4))))],1,count);
    else
        guide=predictiveSafetyGeometry.avoidanceGuide(model,count);inputs=guide.inputs;
    end
    controls=inputs(:,cumsum(holds));
    seeds={localCompactRateLimit(controls,previous,holds,cfg)};
end

function [valid,traces,violation,nodeStates] = localCompactValidate(controls,hypotheses,previous,kernel,cfg,fine,rollouts)
    if nargin<6,fine=false;end
    if nargin<7,rollouts=[];end
    reuse=~isempty(rollouts) && fine && isequal(controls,rollouts.inputs) ...
        && isequal(previous,rollouts.previous) && isequaln(hypotheses,rollouts.hypotheses) ...
        && isequaln(cfg,rollouts.cfg) && isequal(kernel.holds,rollouts.holds) ...
        && kernel.integrationSubsteps==rollouts.integrationSubsteps;
    traces=cell(1,numel(hypotheses));nodeStates=cell(size(traces));violation=0;h=cfg.controller.sampleTime;
    durations=kernel.holds;if fine,durations=ones(1,size(controls,2));end
    clipped=localCompactRateLimit(controls,previous,durations,cfg);
    violation=max(violation,max(abs(controls-clipped),[],'all'));
    stateViolation=violation;
    for s=1:numel(hypotheses)
        item=hypotheses(s);
        if reuse
            flow=rollouts.trajectories{s};dense=flow.dense;states=flow.states;inputs=controls;
        else
            [dense,states,inputs]=localCompactRollout(kernel,item.ego,previous,controls,item.parameters,fine);
        end
        % The accepted trace contains the original control-knot states.
        nodeStates{s}=states;
        if any(~isfinite(dense),'all'),violation=Inf;valid=false;return;end
        [domain,~,~,~,defined]=localCompactModelDomain(dense,inputs,cfg,kernel.validationSubsteps);
        stateViolation=max(stateViolation,max(domain));
        if ~defined,stateViolation=Inf;end
        violation=max(violation,stateViolation);
        times=(0:size(dense,2)-1)*h/kernel.validationSubsteps;q=[];
        if ~isempty(item.target)
            if item.target(4)>0,q=predictiveSafetyGeometry.predictTarget(item.target,times);
            else,q=repmat(item.target,1,numel(times));end
            shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
            separation=localCompactExactClearance(dense,q,shape);
            violation=max(violation,max(cfg.outputFeedback.clearanceMeters-separation));
            violation=max(violation,localCompactIntervalViolation(dense,q,inputs,separation,cfg,kernel.validationSubsteps));
        end
        traces{s}=struct('ego',[item.ego,states],'inputs',inputs,'denseEgo',dense,'targets',q,'validationSubsteps',kernel.validationSubsteps);
    end
    valid=isfinite(violation) && violation<=cfg.outputFeedback.nlpTolerance;
    if isfinite(violation)
        for s=1:numel(hypotheses)
            trace=traces{s};target=[];
            if ~isempty(trace.targets),target=trace.targets(:,end);end
            backup=[];if reuse,backup=rollouts.trajectories{s}.backup;end
            [terminalValue,terminal]=localCompactTerminalCheck(trace.ego(:,end),target,trace.inputs(:,end),hypotheses(s).parameters,kernel,cfg,backup);
            traces{s}.terminal=terminal;violation=max(violation,terminalValue);
        end
        valid=isfinite(violation)&&violation<=cfg.outputFeedback.nlpTolerance;
    end
end

function [dense,states,inputs] = localCompactRollout(kernel,x,previous,controls,parameters,fine)
    N=sum(kernel.holds);inputs=zeros(2,N);k=0;
    if fine,inputs=controls;else
        for i=1:numel(kernel.holds)
            for j=1:kernel.holds(i),k=k+1;inputs(:,k)=previous+(controls(:,i)-previous)*j/kernel.holds(i);end
            previous=controls(:,i);
        end
    end
    capacity=sum(kernel.capacity);
    [dense,ends]=kernel.refinedRollout(x,[inputs,zeros(2,capacity-N)],parameters,double((1:capacity)<=N));
    dense=full(dense);ends=full(ends);
    dense=[x,dense(:,1:kernel.validationSubsteps*N)];states=ends(:,cumsum(kernel.holds));
end
function [terminalValue,terminal]=localCompactTerminalCheck(x,target,previous,parameters,kernel,cfg,backup)
    h=cfg.controller.sampleTime;
    if nargin<7 || isempty(backup)
        [tail,tailEnds,tailInputs]=localCompactBackupRollout(kernel,x,parameters,previous,cfg);
    else
        tail=backup.dense;tailEnds=backup.states;tailInputs=backup.inputs;
    end
    terminalTarget=target;tailViolation=max([0,-tail(4,:)]);
    if ~isempty(tailInputs)
        [domain,~,~,~,defined]=localCompactModelDomain(tail,tailInputs,cfg,kernel.validationSubsteps);
        tailViolation=max(tailViolation,max(domain));
        if ~defined,tailViolation=Inf;end
    end
    if ~isempty(target)
        terminalTarget=predictiveSafetyGeometry.predictTarget(target,(size(tail,2)-1)*h/kernel.validationSubsteps);
        if target(4)==0,tailTarget=repmat(target,1,size(tail,2));terminalTarget=target;
        else,tailTarget=predictiveSafetyGeometry.predictTarget(target,(0:size(tail,2)-1)*h/kernel.validationSubsteps);end
        shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
        gap=localCompactExactClearance(tail,tailTarget,shape);
        if size(tail,2)>1,tailViolation=max(tailViolation,localCompactIntervalViolation(tail,tailTarget,tailInputs,gap,cfg,kernel.validationSubsteps));
        else,tailViolation=max(tailViolation,cfg.outputFeedback.clearanceMeters-gap);end
    end
    parked=tail(:,end);if ~isempty(tailEnds),parked=tailEnds(:,end);end
    [terminalValue,terminal]=terminalSafeSet.compactEvaluate(parked,terminalTarget,previous,cfg);
    terminalValue=max(terminalValue,tailViolation);terminal.value=terminalValue;terminal.member=terminalValue<=1e-6;
    terminal.model="straighteningBrakingThenParked";terminal.stopSeconds=size(tailInputs,2)*h;
    terminal.backupHolds=size(tailInputs,2);
end

function [dense,ends,inputs]=localCompactBackupRollout(kernel,x,parameters,previous,cfg)
    timer=tic;dense=x;ends=zeros(6,0);inputs=zeros(2,0);
    if all(x(4:6)==0),return;end
    command=terminalSafeSet.compactControl(previous,cfg);capacity=kernel.capacity;
    commands=repmat(command,1,capacity);
    while true
        blockCount=min(capacity,max(1,ceil(2*hypot(x(4),x(5))/(cfg.terminal.brakingDecelerationMetersPerSecondSquared*cfg.controller.sampleTime))));
        active=double((1:capacity)<=blockCount);
        [points,states]=kernel.refinedRollout(x,commands,parameters,active);states=full(states);points=full(points);
        states=states(:,1:blockCount);points=points(:,1:blockCount*kernel.validationSubsteps);
        assert(all(isfinite(states),'all'),'collisionAvoidanceController:invalidTerminalFlow','The backup rollout left the numerical model domain.');
        stopped=find(all(states(4:6,:)==0,1),1);count=blockCount;if ~isempty(stopped),count=stopped;end
        dense=[dense,points(:,1:count*kernel.validationSubsteps)];ends=[ends,states(:,1:count)];inputs=[inputs,commands(:,1:count)]; %#ok<AGROW>
        if ~isempty(stopped),return;end
        if toc(timer)>=max([cfg.outputFeedback.initializationSolveSeconds,cfg.outputFeedback.recoverySolveSeconds,cfg.outputFeedback.maximumSolveSeconds])
            error('collisionAvoidanceController:condensedDeadline','The backup did not reach rest within the numerical evaluation budget.');
        end
        x=states(:,end);
    end
end

function [value,index,stateGradient,inputGradient,defined]=localCompactModelDomain(states,inputs,cfg,K)
    % The numerical Fiala guard must not become an accepted model extension.
    % Above the pure kinematic branch, front-wheel longitudinal velocity must
    % be positive (equivalent to |alpha_f|<pi/2 for the steering domain).
    % Check both sides of held-command jumps; no stability speed/rate cap is used.
    count=size(inputs,2);samples=(0:K).'+(0:count-1)*K+1;
    x=states(:,samples(:));u=repelem(inputs,1,K+1);c=cos(u(1,:));s=sin(u(1,:));
    lateral=x(5,:)+cfg.vehicle.lf*x(6,:);
    front=x(4,:).*c+lateral.*s;
    blend=x(4,:)-cfg.model.kinematicSpeedRange(1);
    defined=all(blend<=0 | front>0);
    [v,branch]=max([-x(4,:);min(blend,-front)],[],1);
    [value,j]=max(reshape(v,K+1,count),[],1);index=j-1;
    if nargout<3,return;end
    selected=(0:count-1)*(K+1)+j;x=x(:,selected);u=u(:,selected);
    branch=branch(selected);c=c(selected);s=s(selected);lateral=lateral(selected);
    stateGradient=zeros(6,count);inputGradient=zeros(2,count);stateGradient(4,:)=-1;
    active=branch==2;low=active & blend(selected)<=-front(selected);
    stateGradient(4,low)=1;wheel=active & ~low;
    stateGradient(4,wheel)=-c(wheel);stateGradient(5,wheel)=-s(wheel);
    stateGradient(6,wheel)=-cfg.vehicle.lf*s(wheel);
    inputGradient(1,wheel)=x(4,wheel).*s(wheel)-lateral(wheel).*c(wheel);
end

function violation = localCompactIntervalViolation(states,targets,inputs,separation,cfg,substeps)
    radiusE=hypot(cfg.vehicle.length/2,cfg.vehicle.width/2)+norm(cfg.vehicle.rectangleOffset);
    radiusT=norm(targets(8:9,1))+norm(targets(10:11,1));
    targetSpeed=max(targets(4,:));targetRate=abs(sin(targets(6,1))/targets(7,1))*targetSpeed;
    pointSpeed=max(hypot(states(4,:),states(5,:))) ...
        +targetSpeed+radiusE*max(abs(states(6,:)))+radiusT*targetRate;
    % Exact rest is an equilibrium of the declared model under nonpositive
    % force commands. No residual force or target restart is admitted here.
    if all(states(4:6,:)==0,'all') && all(targets(4,:)==0) && all(inputs(2,:)<=0)
        pointSpeed=0;
    end
    travel=pointSpeed*cfg.controller.sampleTime/substeps;
    a=separation(1:end-1);b=separation(2:end);
    % Minimum of the two endpoint Lipschitz cones on each interval. This is
    % conditional on the continuous speed/rate domain and exact node poses;
    % numerical integration and finite information remain uncertified.
    lower=max([a-travel;b-travel;(a+b-travel)/2],[],1);
    violation=max(cfg.outputFeedback.clearanceMeters-lower);
end

function [clearance,gradient] = localCompactExactClearance(states,targets,shape)
    % Batched closest-feature evaluation of the rectangle distance dual.
    ce=cos(states(3,:));se=sin(states(3,:));ct=cos(targets(3,:));st=sin(targets(3,:));
    signs=[-1,1,1,-1;-1,-1,1,1];ego=cell(1,4);target=cell(1,4);
    for j=1:4
        e=shape(3:4)+shape(1:2).*signs(:,j);
        t=targets(10:11,:)+targets(8:9,:).*signs(:,j);
        ego{j}=states(1:2,:)+[ce*e(1)-se*e(2);se*e(1)+ce*e(2)];
        target{j}=targets(1:2,:)+[ct.*t(1,:)-st.*t(2,:);st.*t(1,:)+ct.*t(2,:)];
    end
    square=Inf(1,size(states,2));closest=zeros(2,size(states,2));difference=closest;
    for j=1:4
        for k=1:4
            next=mod(k,4)+1;
            edge=target{next}-target{k};offset=ego{j}-target{k};
            t=max(0,min(1,sum(offset.*edge,1)./sum(edge.^2,1)));
            delta=offset-edge.*t;distance=sum(delta.^2,1);selected=distance<square;
            square(selected)=distance(selected);
            if nargout>1,closest(:,selected)=ego{j}(:,selected);difference(:,selected)=delta(:,selected);end
            edge=ego{next}-ego{k};offset=target{j}-ego{k};
            t=max(0,min(1,sum(offset.*edge,1)./sum(edge.^2,1)));
            delta=edge.*t-offset;distance=sum(delta.^2,1);selected=distance<square;
            square(selected)=distance(selected);
            if nargout>1
                point=ego{k}+edge.*t;closest(:,selected)=point(:,selected);difference(:,selected)=delta(:,selected);
            end
        end
    end
    clearance=sqrt(square);
    gap=max(predictiveSafetyGeometry.rectangleAxisGaps(states(1:3,:),shape,targets(1:3,:),targets(8:11,:)),[],1);
    overlap=gap<=0;clearance(overlap)=gap(overlap);
    if nargout>1
        normal=difference./max(sqrt(square),eps);arm=closest-states(1:2,:);
        gradient=[normal;normal(2,:).*arm(1,:)-normal(1,:).*arm(2,:)];
        if any(overlap)
            x=states(1:3,overlap);q=targets(:,overlap);step=1e-5;
            for j=1:3
                plus=x;minus=x;plus(j,:)=plus(j,:)+step;minus(j,:)=minus(j,:)-step;
                a=max(predictiveSafetyGeometry.rectangleAxisGaps(plus,shape,q(1:3,:),q(8:11,:)),[],1);
                b=max(predictiveSafetyGeometry.rectangleAxisGaps(minus,shape,q(1:3,:),q(8:11,:)),[],1);
                gradient(j,overlap)=(a-b)/(2*step);
            end
        end
    end
end
