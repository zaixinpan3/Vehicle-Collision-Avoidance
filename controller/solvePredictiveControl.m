function [solution,search,model] = solvePredictiveControl(model,previousState,timer)
%solvePredictiveControl PCBF/CLF RTI with terminal-feasible horizon search.
% Each reference supplies one affine model for both convex objectives. A
% failed shifted reference requests fresh planning. Exceptional bounded
% feasibility restoration improves subsequent references; ordinary RTI
% still uses one model. Only unchanged, nonlinearly checked optimized inputs
% can be returned. Restoration steps themselves are never executed.
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
    cfg=model.cfg;solution=[];model.linearizationBuilds=0;
    model.solveTimer=timer;
    model.solveTimeLimitSeconds=max(0,cfg.solver.timeLimitSeconds-cfg.solver.completionReserveSeconds);
    model.circulationDirection=0;
    if isfield(model,'targetOnset') && ~model.targetOnset && isstruct(previousState) && isfield(previousState,'circulationDirection')
        model.circulationDirection=previousState.circulationDirection;
    end
    baseModel=model;minimumCount=cfg.controller.horizonSteps;originalPrevious=previousState;
    if isfield(model,'referenceProbeOnly') && model.referenceProbeOnly && isfield(model,'probeMinimumHorizonSteps')
        minimumCount=max(minimumCount,model.probeMinimumHorizonSteps);
    end
    history=struct([]);anchor=[];replan=false;restoredAnchor=[];restorations=struct([]);freshTried=false;
    model.referencePenaltyMultiplier=1;
    extensions=0;horizonStop="";bestNonlinearSafety=Inf;
    rollout=struct('available',false,'reason',"noCompletedOptimization", ...
        'fullGap',NaN,'fullCorrection',NaN);
    while true
        source="nominalFeedbackRollout";domainFailure=false;proposed=[];problem=[];
        rollout=struct('available',false,'reason',"noCompletedOptimization", ...
            'fullGap',NaN,'fullCorrection',NaN);
        if isfield(model,'target') && ~isempty(model.target),source="plannedAvoidanceRollout";end
        if isstruct(previousState) && isfield(previousState,'inputTrajectory') ...
                && size(previousState.inputTrajectory,2)>1 && all(isfinite(previousState.inputTrajectory),'all') ...
                && ~(isfield(model,'targetOnset') && model.targetOnset)
            source="shiftedInputRollout";
        end
        search=localEmptySearch(source,"",0);
        try
            localCheckDeadline(model);
            wall=tic;
            if isempty(restoredAnchor)
                [anchor,source]=localRtiInitialize(model,previousState,minimumCount);
            else
                anchor=restoredAnchor;restoredAnchor=[];source="restoredNonlinearRollout";
            end
            search.horizonSteps=size(anchor.inputs,2);
            freshTried=freshTried || source=="plannedAvoidanceRollout";
            search.initializationSeconds=toc(wall);search.initialization=source;
            model.collisionEscapeNormals=zeros(2,2*size(anchor.inputs,2)+1);
            [model.collisionSide,model.retainCollisionSide]=localCollisionSide(model);
            if anchor.planner.used
                if anchor.planner.circulationDirection~=0,model.circulationDirection=anchor.planner.circulationDirection;end
                projection=laneGeometry.project(model.initialState(1:2),model.lane);
                lateral=anchor.guide.direction.'*[-sin(projection.heading);cos(projection.heading)];
                if abs(lateral)>1e-8,model.collisionSide=sign(lateral);end
            end
            if source=="shiftedInputRollout" && ~isempty(model.target) && ~model.targetOnset && isstruct(previousState) ...
                    && isfield(previousState,'collisionSide')
                model.collisionSide=previousState.collisionSide;
                if isfield(previousState,'retainCollisionSide'),model.retainCollisionSide=previousState.retainCollisionSide;end
            end
            anchor.terminalContext.collisionSide=model.collisionSide;
            model.terminalContext=anchor.terminalContext;
            if source=="restoredNonlinearRollout"
                model.fixedCollisionNormals=anchor.safetyNormals;
            end
            if source=="shiftedInputRollout" && ~isempty(model.target) && ~model.targetOnset ...
                    && isfield(previousState,'collisionEscapeNormals') && isfield(previousState,'collisionSide')
                old=previousState.collisionEscapeNormals(:,3:end);
                count=min(size(old,2),size(model.collisionEscapeNormals,2));
                model.collisionEscapeNormals(:,1:count)=old(:,1:count);
            end
            localCheckDeadline(model);
            if isfield(model,'referenceProbeOnly') && model.referenceProbeOnly
                model.linearization=anchor;solution=[];return;
            end
            wall=tic;[problem,model]=localFormulate(anchor,model);
            if ~isfield(model,'numericalStateMargin') && ~isfield(model,'numericalTerminalMargin')
                anchorCheck=localPrimaryResidual(problem);
            else
                anchorCheck=localNonlinearCheck(anchor,model,problem.safetyNormals);
            end
            if anchorCheck.hard<=cfg.solver.feasibilityTolerance
                bestNonlinearSafety=min(bestNonlinearSafety,anchorCheck.safety);
            end
            search.formulationSeconds=toc(wall);search.modelBuilt=true;
            model.linearization=anchor;problem.inheritedAnchor=source=="shiftedInputRollout";
            initializationSeconds=search.initializationSeconds;formulationSeconds=search.formulationSeconds;
            [point,problem,search,model]=localPrimary(anchor,model,source,timer,problem);
            search.modelBuilt=true;search.formulationSeconds=formulationSeconds;
            search.initializationSeconds=initializationSeconds;
            [solution,search,model]=localSecondary(point,problem,anchor,model,search,timer);
            search.nonlinearSlackCap=search.slackCap;
            if isfinite(bestNonlinearSafety)
                search.nonlinearSlackCap=min(search.slackCap,bestNonlinearSafety ...
                    +cfg.solver.lexicographicTieTolerance*max(1,bestNonlinearSafety));
            end
            if ~isempty(solution)
                proposed=solution;
                localCheckDeadline(model);
                [solution,rollout]=localPrediction(solution,anchor,model);
                if ~rollout.available
                    solution=[];search.terminationReason="nonlinearPredictionUnavailable";
                else
                    actual=anchor;actual.states=solution.states;actual.inputs=solution.inputs;
                    [check,~,~]=localNonlinearCheck(actual,model,problem.safetyNormals);
                    rollout.hardResidual=check.hard;rollout.safety=check.safety;
                    tolerance=cfg.solver.feasibilityTolerance;
                    if check.hard>tolerance || check.safety>search.nonlinearSlackCap+tolerance
                        model.numericalStateMargin=localNumericalMargin(solution,anchor,problem,model);
                        model.lastTerminalDefect=check.terminalViolation;
                        solution=[];search.terminationReason="nonlinearConstraintViolation";
                        rollout.reason="nonlinearConstraintViolation";
                    else
                        solution.hard=check.hard;solution.nonlinearValidationPerformed=true;
                        solution.affineSafety=solution.safety;
                        solution.affineStageSlacks=solution.stageSlacks;
                        solution.safety=check.safety;solution.stageSlacks=check.slacks;
                        solution.nonlinearSafety=check.safety;solution.nonlinearStageSlacks=check.slacks;
                        solution.collisionEscapeNormals=model.collisionEscapeNormals;
                        solution.collisionSide=model.collisionSide;
                        solution.circulationDirection=model.circulationDirection;
                        solution.retainCollisionSide=model.retainCollisionSide;
                        solution.linearizationGap=rollout.fullGap;
                        solution=localTerminalMetrics(solution,model);
                        rollout.reason="sampledNonlinearConstraintsPassed";
                    end
                end
            end
        catch exception
            if isfield(model,'referenceProbeOnly') && model.referenceProbeOnly,rethrow(exception);end
            if strcmp(exception.identifier,'collisionAvoidanceController:timeBudgetExceeded')
                solution=[];search.terminationReason="timeLimit";
            elseif any(strcmp(exception.identifier,{'collisionAvoidanceController:invalidTireOperatingPoint', ...
                    'collisionAvoidanceController:singularTireLinearization','collisionAvoidanceController:nonlinearDomain'}))
                % A longer shifted reference may leave the model domain.
                % Try fresh guidance instead of issuing an invalid command.
                solution=[];domainFailure=true;
                search.initialization=source;search.initializationFailure=string(exception.identifier);
                search.terminationReason="referencePredictionUnavailable";
            else
                rethrow(exception);
            end
        end
        history=[history,localAttempt(search)]; %#ok<AGROW> Bounded by the shared deadline.
        infeasible=~isempty(search.stages) && search.stages(end).exitFlag==-2;
        invalid=any(search.terminationReason==["nonlinearPredictionUnavailable","nonlinearConstraintViolation"]);
        % Primal infeasibility of a shifted model first requests fresh
        % planning. A solved but inaccurate model can restore its nonlinear
        % reference first. Both paths disclose every additional model/solve.
        if isempty(solution) && (infeasible || invalid) && (source~="shiftedInputRollout" || invalid) ...
                && ~isempty(problem) && numel(restorations)<cfg.rti.maximumRestorations
            try
                [restoredAnchor,restoration]=localRestore(anchor,model,problem,proposed);
                restorations=[restorations,restoration]; %#ok<AGROW>
                if restoration.timedOut
                    search.terminationReason="timeLimit";break;
                end
                if ~isempty(restoredAnchor)
                    model.numericalStateMargin=restoredAnchor.numericalStateMargin;
                    if isfield(model,'lastTerminalDefect')
                        reserve=1.25*restoration.acceptedStep^2*model.lastTerminalDefect;
                        if isfield(model,'numericalTerminalMargin'),reserve=max(reserve,model.numericalTerminalMargin);end
                        model.numericalTerminalMargin=reserve;
                    end
                    model.referencePenaltyMultiplier=10*model.referencePenaltyMultiplier;
                    replan=true;continue;
                end
            catch exception
                if ~strcmp(exception.identifier,'collisionAvoidanceController:timeBudgetExceeded'),rethrow(exception);end
                search.terminationReason="timeLimit";break;
            end
        end
        % Extending an unrollable fresh guide repeats the same invalid
        % prefix; horizon growth cannot repair that initialization failure.
        % A failed shifted prefix can still be replaced by fresh guidance.
        recoverable=infeasible || (source=="shiftedInputRollout" && (domainFailure || invalid));
        if source=="restoredNonlinearRollout" && numel(restorations)>=cfg.rti.maximumRestorations && ~infeasible
            recoverable=false;
        end
        if isempty(solution) && (infeasible || invalid) && ~freshTried && ~isempty(model.target)
            % A failed restoration of the old maneuver must not consume the
            % request to rebuild a genuinely fresh field reference.
            minimumCount=size(anchor.inputs,2);previousState=[];
            builds=model.linearizationBuilds;model=baseModel;model.linearizationBuilds=builds;
            model.referencePenaltyMultiplier=1;anchor=[];replan=true;continue;
        end
        if ~isempty(solution) || ~recoverable ...
                || toc(timer)>=model.solveTimeLimitSeconds
            break;
        end
        count=minimumCount;
        if ~isempty(anchor),count=size(anchor.inputs,2);
        elseif source=="shiftedInputRollout",count=max(count,size(previousState.inputTrajectory,2)-1);end
        if source=="shiftedInputRollout"
            % Try a newly planned reference at this length before enlarging
            % the problem. Keep the original plan available for extension;
            % a poor fresh guide must not erase a useful old maneuver.
            minimumCount=count;previousState=[];
        else
            % Geometric search has no fixed duration increment or attempt
            % count. Terminal constraints, not a seed-membership test, decide
            % whether a horizon is usable. A finite explicit user cap and
            % the common computation deadline remain operational limits.
            nextCount=min(cfg.controller.maximumHorizonSteps,max(count+1,ceil(1.5*count)));
            if nextCount<=count,horizonStop="horizonLimit";break;end
            minimumCount=nextCount;previousState=originalPrevious;extensions=extensions+1;
            bestNonlinearSafety=Inf;
        end
        builds=model.linearizationBuilds;lastModel=model;model=baseModel;model.linearizationBuilds=builds;
        if isfield(lastModel,'linearization'),model.linearization=lastModel.linearization;end
        model.terminalContext=[];anchor=[];replan=true;
        model.referencePenaltyMultiplier=1;
    end
    search.source="twoStageRealTimeIteration";search.algorithm="realTimeIteration";
    search.initialization=source;search.returned=~isempty(solution);
    % A completed convex subproblem is not nonlinear SQP convergence.
    search.convexSolveCompleted=search.clfStageCompleted;search.converged=false;
    search.linearizationCount=model.linearizationBuilds;
    search.rtiIterations=model.linearizationBuilds;
    search.nominalRestarted=replan;search.restartCount=max(0,model.linearizationBuilds-1);
    search.horizonExtensions=extensions;
    search.stopReason=search.terminationReason;
    if strlength(horizonStop)>0,search.stopReason=horizonStop;end
    search.rollout=rollout;
    search.anchorCertified=false;search.appendedTerminalSteps=0;search.appendedBrakingSteps=0;
    search.planner=localPlannerMetadata();
    if ~isempty(anchor)
        search.appendedTerminalSteps=anchor.appendedTerminalSteps;
        search.appendedBrakingSteps=anchor.appendedBrakingSteps;search.planner=anchor.planner;
    end
    search.potentialFieldRestarted=false;search.initializationFailure="";
    search.attempts=history;search.selectedAttempt=numel(history)*double(search.returned);
    search.solverCalls=sum([history.solverCalls]);
    search.restorations=restorations;search.restorationCount=numel(restorations);
    search.restorationSolverCalls=0;
    if ~isempty(restorations),search.restorationSolverCalls=sum([restorations.solverCalls]);end
    search.solverCalls=search.solverCalls+search.restorationSolverCalls;
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
    % This never decides whether to call the planner: every fresh reference
    % with a target is planned, including every infeasibility-triggered retry.
    % Doubling the geometry grid costs O(N) in its final node count. The same
    % wall-clock deadline applies during this search and the one RTI solve.
    cfg=model.cfg;h=cfg.controller.sampleTime;minimumDuration=h*minimumCount;duration=minimumDuration;
    x=model.initialState;speed=hypot(x(4),x(5));beta=atan2(x(5),x(4));
    % A near-equal-speed target can put the next geometric intersection tens
    % of seconds away. Initial search need only span a physically meaningful
    % stopping/settling maneuver. The seed need not end in the terminal set;
    % RTI enforces endpoint constraints and may retry with a longer horizon.
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

function [anchor,source]=localRtiInitialize(model,previous,minimumCount)
    cfg=model.cfg;count=minimumCount;
    inherited=isstruct(previous) && isfield(previous,'inputTrajectory') ...
        && size(previous.inputTrajectory,2)>1 && all(isfinite(previous.inputTrajectory),'all') ...
        && ~(isfield(model,'targetOnset') && model.targetOnset);
    plan=~isempty(model.target) && ~inherited;
    if plan
        % A bounded planner supplies the route, never an executable fallback.
        count=localPlanningHorizon(model,minimumCount);
        anchor=localPlanReference(model,count);source="plannedAvoidanceRollout";return;
    end
    old=zeros(2,0);source="nominalFeedbackRollout";
    if inherited
        old=previous.inputTrajectory(:,2:end);source="shiftedInputRollout";
        % Preserve the issued command, but build the next tangent at an
        % interior reference and recompute its complete nonlinear rollout.
        [seedLower,seedUpper]=nonlinearBicycleModel.inputBounds(cfg,true);
        old=min(seedUpper,max(seedLower,old));
        % Preserve the previous absolute endpoint unless the minimum prefix
        % or the current encounter preview needs more holds. Disappearance
        % alone must not replace the tail with an arbitrary cruise rollout.
        count=max(minimumCount,size(old,2));
        % A changed target forecast can require a longer encounter horizon
        % even while the previous input sequence remains a useful seed.
        % Choose that length geometrically, without a terminal-state search.
        if ~isempty(model.target),count=max(count,localPlanningHorizon(model,minimumCount));end
    end
    count=min(count,cfg.controller.maximumHorizonSteps);
    inputs=zeros(2,count);states=zeros(6,count+1);states(:,1)=model.initialState;
    context=terminalSafeSet.context(model);
    context.solveTimer=model.solveTimer;context.solveTimeLimitSeconds=model.solveTimeLimitSeconds;
    appended=0;braking=0;
    prior=model.previousInput;nominal=nonlinearBicycleModel.nominalGuidanceParameters(cfg,model.nominalReference.curvature);
    for index=1:count
        if index<=size(old,2)
            [u,next]=predictiveSafetyGeometry.boundaryReferenceInput(states(:,index),old(:,index),prior,model);
        elseif inherited && terminalSafeSet.member(context,states(:,index),index-1)
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

function discrepancy=localPoseDiscrepancy(first,second,cfg)
    % Body-point pose metric of the safety rows: translation plus reach*yaw.
    reach=norm([cfg.vehicle.length;cfg.vehicle.width]/2)+norm(cfg.vehicle.rectangleOffset);
    difference=first-second;
    discrepancy=vecnorm(difference(1:2,:))+reach*abs(atan2(sin(difference(3,:)),cos(difference(3,:))));
end

function [solution,rollout]=localPrediction(solution,anchor,model)
    % Predict unchanged optimized inputs before evaluating their constraints.
    rollout=struct('available',false,'reason',"noCompletedOptimization", ...
        'fullGap',NaN,'fullCorrection',NaN);
    if isempty(solution),return;end
    if any(abs(solution.inputs(2,:))>1)
        rollout.reason="actuatorDomain";return;
    end
    initialSlip=atan2(model.initialState(5)+model.cfg.vehicle.lf*model.initialState(6),model.initialState(4))-solution.inputs(1,1);
    if abs(initialSlip)>=pi/2
        rollout.reason="frontTireDomain";return;
    end
    solution.affineStates=solution.states;
    solution.inputCorrection=solution.inputs-anchor.inputs;
    unit=[model.cfg.clf.certificationSteeringRadians;model.cfg.clf.certificationBrakingRatio];
    rollout.fullCorrection=max(abs(solution.inputCorrection)./unit,[],'all');
    states=zeros(size(solution.states));states(:,1)=model.initialState;
    try
        for index=1:size(solution.inputs,2)
            if mod(index,4)==1,localCheckDeadline(model);end
            states(:,index+1)=nonlinearBicycleModel.sample(states(:,index),solution.inputs(:,index),model.cfg);
            ends=states(:,index:index+1);
            slip=atan2(ends(5,:)+model.cfg.vehicle.lf*ends(6,:),ends(4,:))-solution.inputs(1,index);
            if any(abs(slip)>=pi/2)
                rollout.reason="frontTireDomain";return;
            end
        end
    catch exception
        if ~startsWith(string(exception.identifier),"collisionAvoidanceController:"),rethrow(exception);end
        if strcmp(exception.identifier,'collisionAvoidanceController:timeBudgetExceeded'),rethrow(exception);end
        % A model-domain failure makes the optimized prediction unavailable.
        rollout.reason=string(exception.identifier);return;
    end
    rollout.available=true;rollout.reason="predictionOnly";
    rollout.fullGap=max(localPoseDiscrepancy(states,solution.affineStates,model.cfg));
    solution.states=states;
end

function [check,problem,formed]=localNonlinearCheck(anchor,model,normals)
    % At zero correction the reassembled rows evaluate the actual rollout,
    % including its true yaw, posterior margins and terminal continuation.
    % Keep the solve's separating normals: validation cannot select an easier
    % collision branch. This is a sampled numerical check, not a tube proof.
    model.fixedCollisionNormals=normals;builds=model.linearizationBuilds;
    if isfield(model,'numericalStateMargin'),model=rmfield(model,'numericalStateMargin');end
    if isfield(model,'numericalTerminalMargin'),model=rmfield(model,'numericalTerminalMargin');end
    [problem,formed]=localFormulate(anchor,model);
    formed.linearizationBuilds=builds;
    check=localPrimaryResidual(problem);
end

function check=localPrimaryResidual(problem)
    % Numerical reserves may restrict a new subproblem, but cannot justify
    % losing a better already available nonlinear safety witness.
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
    check.merit=1000*check.hard+check.safety/max(1,numel(slack));
end

function margin=localNumericalMargin(solution,anchor,problem,model)
    % A posteriori model-error feedback for the NEXT numerical attempt.
    % It is separate from observer uncertainty and is not a certified bound.
    % Acceptance always rechecks the true constraints without this reserve.
    count=size(solution.inputs,2);margin=zeros(6,2*count+1);
    margin(:,1:2:end)=abs(solution.states-solution.affineStates);
    for k=1:count
        stage=problem.stages(k);
        affine=stage.middle+stage.am*(solution.affineStates(:,k)-anchor.states(:,k)) ...
            +stage.bm*(solution.inputs(:,k)-anchor.inputs(:,k));
        [middle,am,~,~,a]=nonlinearBicycleModel.hold(solution.states(:,k),solution.inputs(:,k),model.cfg);
        margin(:,2*k)=abs(middle-affine);
        % The fixed posterior is used once. Changing the operating point
        % changes its held-input image too; account for that change instead
        % of confusing it with a second independent source of uncertainty.
        generator=model.uncertainty.egoGenerator;
        middleChange=sum(abs((am-stage.am)*generator),2);
        nextChange=sum(abs((a-stage.a)*generator),2);
        oldMiddle=localRelativeGenerator(stage.middle,stage.am*generator,model,anchor.states(:,k));
        newMiddle=localRelativeGenerator(middle,am*generator,model,solution.states(:,k));
        oldNext=localRelativeGenerator(anchor.states(:,k+1),stage.a*generator,model,anchor.states(:,k));
        newNext=localRelativeGenerator(solution.states(:,k+1),a*generator,model,solution.states(:,k));
        margin(:,2*k)=margin(:,2*k)+max(middleChange,sum(abs(newMiddle-oldMiddle),2));
        margin(:,2*k+1)=margin(:,2*k+1)+max(nextChange,sum(abs(newNext-oldNext),2));
    end
    margin=1.25*margin;
end

function [next,info]=localRestore(anchor,model,problem,proposed)
    % Feasibility restoration only creates the next reference. Its controls
    % are never issued. It softens hard state/collision/terminal rows in one
    % auxiliary solve, retains actuator and Fiala capacity limits, then tests
    % a decreasing sequence of steps through the nonlinear vehicle model.
    next=[];info=struct('solverCalls',0,'acceptedStep',0,'before',Inf,'after',Inf, ...
        'initialFeasible',false,'timedOut',false);
    try
        [before,p]=localNonlinearCheck(anchor,model,problem.safetyNormals);
        info.before=before.merit;cfg=model.cfg;nv=numel(p.lower);
        info.initialFeasible=before.hard<=cfg.solver.feasibilityTolerance && before.safety<=cfg.solver.lexicographicTieTolerance;
        if ~isempty(proposed)
            [next,fraction,merit]=localRestorationStep(anchor,model,p,proposed.inputs-anchor.inputs,before);
            if ~isempty(next)
                info.acceptedStep=fraction;info.after=merit;return;
            end
        end
        units=[cfg.clf.lateralPositionErrorScale;cfg.clf.lateralPositionErrorScale; ...
            cfg.clf.headingErrorScale;cfg.clf.speedErrorScale; ...
            cfg.clf.lateralVelocityErrorScale;cfg.clf.yawRateErrorScale];
        inputUnits=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio];
        diagonal=zeros(nv+1,1);n=size(anchor.inputs,2);
        diagonal(p.stateIndices(:,2:end))=repmat(1./units.^2/n,1,n);
        diagonal(p.inputIndices)=repmat(1./inputUnits.^2/n,1,n);
        quadratic=spdiags(.1*diagonal,0,nv+1,nv+1);
        objective=[p.safetyObjective/max(1,numel(p.slackIndices));1000];
        % A single elastic variable measures the largest hard violation.
        hard=~any(p.a(:,p.slackIndices),2);
        inputOnly=~any(p.a(:,p.stateIndices(:)),2) & any(p.a(:,p.inputIndices(:)),2);
        hard=hard & ~inputOnly;
        a=[p.a,-double(hard)];b=p.b;
        states=p.stateIndices(:);lo=isfinite(p.lower(states));hi=isfinite(p.upper(states));
        eyeState=sparse(1:numel(states),states,1,numel(states),nv);
        a=[a;-eyeState(lo,:),-ones(nnz(lo),1);eyeState(hi,:),-ones(nnz(hi),1)];
        b=[b;-p.lower(states(lo));p.upper(states(hi))];
        lower=[p.lower;0];upper=[p.upper;Inf];lower(states)=-Inf;upper(states)=Inf;
        % The CLF is absent from restoration and cannot demand cruise here.
        lower(p.clfIndex)=0;upper(p.clfIndex)=0;
        cones=p.cones(1:p.primaryConeCount);
        for k=1:numel(cones)
            cones(k).A(:,nv+1)=0;cones(k).D(:,nv+1)=0;
        end
        localCheckDeadline(model);
        remaining=model.solveTimeLimitSeconds-toc(model.solveTimer);
        [step,flag]=localConicSolve(quadratic,objective,cones,a,b, ...
            [p.equal,sparse(size(p.equal,1),1)],p.rhs,lower,upper,cfg,remaining,cfg.solver.optimalityTolerance);
        info.solverCalls=1;
        if isempty(step) || flag<=0,return;end
        direction=reshape(step(p.inputIndices),size(anchor.inputs));
        [next,fraction,merit]=localRestorationStep(anchor,model,p,direction,before);
        if ~isempty(next),info.acceptedStep=fraction;info.after=merit;end
    catch exception
        if ~strcmp(exception.identifier,'collisionAvoidanceController:timeBudgetExceeded'),rethrow(exception);end
        % Preserve the count of an auxiliary solve completed before a later
        % trial ran out of time. An incomplete trial cannot become a seed.
        next=[];info.timedOut=true;
    end
end

function [next,fraction,merit]=localRestorationStep(anchor,model,problem,direction,before)
    next=[];merit=Inf;cfg=model.cfg;
    for fraction=2.^(-(0:8))
        localCheckDeadline(model);
        candidate=anchor;candidate.inputs=anchor.inputs+fraction*direction;
        [lower,upper]=nonlinearBicycleModel.inputBounds(cfg,true);
        % Only an initializer needs an interior tangent. This never changes
        % the command returned by either of the two actual objective solves.
        candidate.inputs=min(upper,max(lower,candidate.inputs));
        affine=anchor.states;delta=model.initialState-anchor.states(:,1);
        for k=1:size(candidate.inputs,2)
            stage=problem.stages(k);
            delta=stage.a*delta+stage.b*(candidate.inputs(:,k)-anchor.inputs(:,k)) ...
                +stage.next-anchor.states(:,k+1);
            affine(:,k+1)=anchor.states(:,k+1)+delta;
        end
        trial=struct('inputs',candidate.inputs,'states',affine);
        [trial,rollout]=localPrediction(trial,anchor,model);
        if ~rollout.available,continue;end
        candidate.states=trial.states;
        [after,~,~]=localNonlinearCheck(candidate,model,problem.safetyNormals);
        improved=after.merit<before.merit-1e-6*max(1,before.merit);
        feasible=after.hard<=cfg.solver.feasibilityTolerance ...
            && after.safety<=cfg.solver.lexicographicTieTolerance;
        if improved || feasible
            candidate.safetyNormals=problem.safetyNormals;
            candidate.numericalStateMargin=localNumericalMargin(trial,anchor,problem,model);
            next=candidate;merit=after.merit;return;
        end
    end
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
    search=struct('solverCalls',0,'source',"twoStageRealTimeIteration",'modelBuilt',false, ...
        'initialization',source,'returned',false,'converged',false,'terminationReason',"anchorUnavailable", ...
        'formulationSeconds',0,'clfConstructionSeconds',0, ...
        'primaryOptimum',NaN,'primaryLowerBound',NaN,'slackCap',NaN,'clfInitialSlack',NaN, ...
        'clfStageAttempted',false,'clfStageCompleted',false,'clfLowerBound',false, ...
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
    search=struct('solverCalls',0,'source',"twoStageRealTimeIteration", ...
        'modelBuilt',formulationSeconds>0, ...
        'initialization',initialization,'returned',false,'converged',false,'terminationReason',"timeLimit", ...
        'formulationSeconds',formulationSeconds,'clfConstructionSeconds',0, ...
        'primaryOptimum',NaN,'primaryLowerBound',problem.primaryLowerBound,'slackCap',NaN,'clfInitialSlack',NaN, ...
        'initializationSeconds',0, ...
        'clfStageAttempted',false,'clfStageCompleted',false,'clfLowerBound',false, ...
        'stages',struct('objective',{},'exitFlag',{},'seconds',{},'value',{},'numericalSolve',{},'solverInfo',{}), ...
        'horizonSteps',size(problem.inputIndices,2));
end

function [point,problem,search,model]=localPrimary(anchor,model,initialization,timer,problem)
    % Formulate the reference's problem and solve its safety stage.
    localCheckDeadline(model);
    cfg=model.cfg;point=[];formulationSeconds=0;
    if nargin<5 || isempty(problem)
        wall=tic;[problem,model]=localFormulate(anchor,model);formulationSeconds=toc(wall);
        model.linearization=anchor;
        % Retain the reference source for research diagnostics.
        problem.inheritedAnchor=initialization=="shiftedInputRollout";
    end
    search=localSearch(problem,initialization,formulationSeconds);
    remaining=model.solveTimeLimitSeconds-toc(timer);if remaining<=0,return;end
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
    search.primaryOptimum=problem.safetyObjective(1:primaryCount).'*point;
    search.stages(1).value=search.primaryOptimum;
    % Match the cap's scale to the numerical primary objective. An absolute
    % micrometer cap on a large slack sum can exclude the computed
    % primary point within the solver's relative optimality tolerance.
    search.slackCap=search.primaryOptimum+cfg.solver.lexicographicTieTolerance*max(1,search.primaryOptimum);
    search.terminationReason="primaryReturned";
end

function [solution,search,model]=localSecondary(point,problem,anchor,model,search,timer)
    cfg=model.cfg;solution=[];if isempty(point),return;end
    localCheckDeadline(model);
    if ~isfield(problem,'clf')
        wall=tic;problem=localAddClf(problem,anchor,model);search.clfConstructionSeconds=toc(wall);
        search.formulationSeconds=search.formulationSeconds+search.clfConstructionSeconds;
    end
    search.clfInitialSlack=problem.initialClfSlack;
    problem.a=[problem.a;problem.safetyObjective.'];problem.b=[problem.b;search.slackCap];
    remaining=model.solveTimeLimitSeconds-toc(timer);
    if remaining<=0,search.terminationReason="timeLimit";return;end
    % Penalize departure from this linearization's state/input reference.
    % This soft cost leaves the feasible set and primary slack cap unchanged.
    objective=problem.performanceObjective;objective(problem.clfIndex)=1;
    search.clfStageAttempted=true;wall=tic;
    [second,flag,solverInfo]=localConicSolve(problem.quadratic,objective,problem.cones,problem.a,problem.b, ...
        problem.equal,problem.rhs,problem.lower,problem.upper,cfg,remaining, ...
        min(cfg.solver.optimalityTolerance,cfg.solver.constraintTolerance));
    search.solverCalls=search.solverCalls+1;
    search.stages(2)=struct('objective',"clfSlackAndReferenceDeviation",'exitFlag',flag,'seconds',toc(wall),'value',NaN,'numericalSolve',true,'solverInfo',solverInfo);
    if isempty(second) || any(~isfinite(second)) || flag<=0,search.terminationReason="clfNoNumericalResult";return;end
    solution=localDecode(second,problem,anchor);
    search.stages(2).value=solution.secondaryObjectiveValue;search.clfStageCompleted=true;search.returned=true;
    search.convexSolveCompleted=all([search.stages([search.stages.numericalSolve]).exitFlag]>0);
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
        'referenceDeviationWeight',problem.referenceDeviationWeight,'clfWorstNextValue',worstNext,'clfCurrentBudget',problem.clfCurrentBudget, ...
        'maximumCollisionTighteningMeters',problem.maximumCollisionTighteningMeters, ...
        'minimumCollisionMargin',NaN, ...
        'encounterExit',problem.encounterExit,'affineValidationPerformed',false,'nonlinearValidationPerformed',false, ...
        'referenceStateDeviation',norm(problem.referenceStateMap*point)^2, ...
        'referenceInputDeviation',norm(problem.referenceInputMap*point)^2);
    solution.referenceDeviationCost=.5*problem.referenceDeviationWeight ...
        *(solution.referenceStateDeviation+solution.referenceInputDeviation);
    solution.secondaryObjectiveValue=solution.clfSlack+solution.referenceDeviationCost;
    if isfield(problem,'durationIndex')
        solution.durationChangeSeconds=point(problem.durationIndex);
        solution.states=solution.states+problem.durationStateDerivative*solution.durationChangeSeconds;
    end
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
    % The reference defines derivatives and collision normals, not a hard
    % neighborhood. Only physical/model-domain bounds restrict corrections.
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
        numericalMiddle=zeros(6,1);numericalNext=zeros(6,1);
        if isfield(model,'numericalStateMargin')
            numericalMiddle=model.numericalStateMargin(:,2*index);
            numericalNext=model.numericalStateMargin(:,2*index+1);
            middleRadius=middleRadius+numericalMiddle(4:6);nextRadius=nextRadius+numericalNext(4:6);
        end
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
            blockBounds{blockCount}=-side*nextReference-sum(abs(side*nextGenerator),2)-abs(side)*numericalNext;
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
    if isfield(model,'numericalStateMargin')
        terminalBounds=terminalBounds-abs(terminalRows)*model.numericalStateMargin(:,end);
    end
    if isfield(model,'numericalTerminalMargin')
        terminalBounds=terminalBounds-model.numericalTerminalMargin;
    end
    localCheckDeadline(model);
    model.terminalContext=context;model.encounterExitRecord=exitRecord;
    if ~isempty(terminalRows)
        blockCount=blockCount+1;blockRows{blockCount}=terminalRows;blockColumns{blockCount}=endColumns;blockBounds{blockCount}=terminalBounds;
    end
    % Prune only inequalities implied by actual physical bounds. There is
    % no pose/input trust box behind this redundancy calculation.
    lower(iu)=max(lower(iu),physicalInputLower-anchor.inputs);
    upper(iu)=min(upper(iu),physicalInputUpper-anchor.inputs);
    reachLower=lower;reachUpper=upper;
    if isfield(model,'durationProbe') && model.durationProbe
        % Keep redundant rows while differentiating: pruning at a bound
        % crossing would change the finite-difference row identities.
        reachLower(:)=-Inf;reachUpper(:)=Inf;
    end
    [inequality,bounds]=localAssemble(blockRows,blockColumns,blockBounds,blockCount,nv,reachLower,reachUpper);
    % Carry the Fiala force capacity into its affine approximation. This
    % uses exactly the model's axle loads and friction: an unconstrained
    % tangent otherwise predicts lateral force even at full braking.
    cones=localTireCapacity(cones,anchor,stages,ix,iu,nv,cfg);
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
    problem.inputIncrementLower=physicalInputLower-anchor.inputs;problem.inputIncrementUpper=physicalInputUpper-anchor.inputs;
end

function cones=localTireCapacity(cones,anchor,stages,ix,iu,nv,cfg)
    scale=modifiedFialaTire.parameters(cfg).longitudinalForceScale;
    count=size(iu,2);maps=cell(1,3*count);values=cell(1,3*count);used=0;
    for k=1:count
        poses=[anchor.states(:,k),stages(k).middle,anchor.states(:,k+1)];
        for point=1:3
            x=poses(:,point);
            if x(4)<=cfg.model.kinematicSpeedRange(1),continue;end
            map=sparse(6,nv);
            if point==1,map(:,ix(:,k))=eye(6);
            elseif point==3,map(:,ix(:,k+1))=eye(6);
            else,map(:,ix(:,k))=stages(k).am;map(:,iu(:,k))=stages(k).bm;end
            u=anchor.inputs(:,k);tire=modifiedFialaTire.affineModel(x,u,cfg);
            force=tire.state*map;force(:,iu(:,k))=force(:,iu(:,k))+tire.input;
            row=sparse(4,nv);row([1,3],iu(2,k))=1;
            row([2,4],:)=force./scale;
            used=used+1;maps{used}=row;values{used}=reshape([repmat(u(2),1,2);(tire.force./scale).'],[],1);
        end
    end
    if used>0
        cones(end+1)=struct('A',vertcat(maps{1:used}),'b',-vertcat(values{1:used}), ...
            'D',sparse(2*used,nv),'gamma',-ones(2*used,1),'size',2);
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

function cone=localConeBlock(A,b,D,gamma,rowsPerCone)
    % A block of second-order cones of equal dimension: cone j is
    % ||A(rows j) z - b(rows j)|| <= D(j,:) z - gamma(j) in the solver's
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
    if isfield(model,'numericalStateMargin')
        remainder=model.numericalStateMargin(:,point);
        values=values-normalNorm*(norm(remainder(1:2))+egoReach*remainder(3));
    end
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


function problem=localAddClf(problem,anchor,model)
    nv=numel(problem.safetyObjective);iu=problem.inputIndices;ic=problem.clfIndex;count=size(iu,2);
    [nominalMap,nominalOffset,~,value,scale,~,clf]=localNominalClf(anchor,model,problem.stateIndices(:,2),nv);
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
    if isfield(model,'referencePenaltyMultiplier')
        problem.referenceDeviationWeight=problem.referenceDeviationWeight*model.referencePenaltyMultiplier;
    end
    problem.referencePenaltyWeight=problem.referenceDeviationWeight/(2*scale);
    problem.quadratic=2*problem.referencePenaltyWeight ...
        *(problem.referenceStateMap.'*problem.referenceStateMap+problem.referenceInputMap.'*problem.referenceInputMap);
    problem.performanceObjective=zeros(nv,1);
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
    % Diagnostics of the already checked nonlinear endpoint.
    cfg=model.cfg;y=solution.states(:,end);count=size(solution.inputs,2);
    solution.terminalValue=nonlinearBicycleModel.nominalValue(y,model.lane,model.nominalReference);
    solution.terminalMarginMeters=NaN;solution.terminalRoadMarginMeters=NaN;
    solution.terminalStopSeconds=hypot(y(4),y(5))/cfg.terminal.brakingDecelerationMetersPerSecondSquared;
    solution.terminalExitSeconds=NaN;solution.terminalEnd="sampledConstraintsValidated";
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
            [~,search,model]=solvePredictiveControl(model,request.previousState,timer);
            model=rmfield(model,'referenceProbeOnly');
            if isfield(model,'probeMinimumHorizonSteps'),model=rmfield(model,'probeMinimumHorizonSteps');end
            assert(isfield(model,'linearization'),'collisionAvoidanceController:noProbeReference', ...
                'No rollable reference was constructed.');
            result=model.linearization;
        case "formulate"
            model.solveTimer=timer;model.solveTimeLimitSeconds=model.cfg.solver.timeLimitSeconds;
            [result,model]=localFormulate(request.anchor,model);
            result.inheritedAnchor=request.inherited;
            result=localAddClf(result,request.anchor,model);
        case "solve"
            model.solveTimer=timer;model.solveTimeLimitSeconds=model.cfg.solver.timeLimitSeconds;
            problem=request.problem;anchor=request.anchor;
            [point,problem,search,model]=localPrimary(anchor,model,"durationExperiment",timer,problem);
            [result,search,model]=localSecondary(point,problem,anchor,model,search,timer);
        case "check"
            model.solveTimer=timer;model.solveTimeLimitSeconds=model.cfg.solver.timeLimitSeconds;
            [result,search,model]=localNonlinearCheck(request.anchor,model,request.problem.safetyNormals);
        case "restore"
            model.solveTimer=timer;model.solveTimeLimitSeconds=model.cfg.solver.timeLimitSeconds;
            [result,search]=localRestore(request.anchor,model,request.problem,request.proposed);
        case "remainder"
            result=localNumericalMargin(request.solution,request.anchor,request.problem,model);
        otherwise
            error('collisionAvoidanceController:invalidProbeOperation','Unknown research operation.');
    end
end
