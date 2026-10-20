function result = exploreFreeTimePredictiveControl(sample,previousState,options)
%exploreFreeTimePredictiveControl Compare fixed and free duration on one RTI reference.
% Input is a controller model or a saved joint-simulation failure snapshot.
% This research probe never issues a command or changes the online defaults.
% The first prefix holds retain the actual sample interval. A scalar change
% in total time is distributed uniformly over the remaining holds. The two
% original PCBF/CLF objectives are retained without a time penalty.
% Future enclosures use the runtime's approximate per-hold convention, with
% the longer hold duration propagated explicitly. Interior measurement
% benefits are not invented. This is not a robust output-feedback certificate.
    arguments
        sample (1,1) struct
        previousState = []
        options.DurationIncreaseFraction (1,1) double {mustBeNonnegative,mustBeFinite} = 1
        options.DifferenceSeconds (1,1) double {mustBePositive,mustBeFinite} = 1e-3
        options.IntegrationStep (1,1) double {mustBePositive,mustBeFinite} = .01
        options.FreshReference (1,1) logical = false
        options.MovingReference (1,1) logical = true
        options.RefinementIterations (1,1) double {mustBeInteger,mustBePositive} = 1
    end
    if isfield(sample,'egoInput')
        previousState=sample.previousState;model=localSnapshotModel(sample);
    else
        model=sample;
    end
    if options.FreshReference,previousState=[];end
    cfg=model.cfg;wall=tic;
    request=struct('operation',"reference",'model',model,'previousState',previousState);
    [anchor,initialization,model]=solvePredictiveControl("probe",request);
    % Obtain guidance with its existing CLF table key. Use a finer, fixed
    % integration mesh for both comparison problems and reroll the same
    % controls; otherwise time perturbations can change the RK4 mesh at h.
    cfg.nonlinear.integrationStep=options.IntegrationStep;model.cfg=cfg;
    anchor.states(:,1)=model.initialState;
    for k=1:size(anchor.inputs,2)
        [~,~,~,anchor.states(:,k+1)]=nonlinearBicycleModel.hold(anchor.states(:,k),anchor.inputs(:,k),cfg);
    end
    setupSeconds=toc(wall);count=size(anchor.inputs,2);prefix=cfg.controller.horizonSteps;
    assert(count>prefix,'exploreFreeTimePredictiveControl:noFreeTail', ...
        'The reference must extend beyond the fixed physical prefix.');
    inherited=initialization.initialization=="shiftedInputRollout";
    h=cfg.controller.sampleTime;weights=[zeros(1,prefix),ones(1,count-prefix)/(count-prefix)];
    durations=repmat(h,1,count);initialDuration=sum(durations);
    maximumIncrease=(count-prefix)*h*options.DurationIncreaseFraction;
    model.durationProbe=true;model.predictionDurations=durations;
    initialAnchor=anchor;history=cell(1,options.RefinementIterations);
    result=struct('status',"notRun",'initialDurationSeconds',initialDuration, ...
        'fixedPrefixSeconds',prefix*h,'stageCount',count,'setupSeconds',setupSeconds, ...
        'initialization',initialization.initialization,'fixed',[],'free',[], ...
        'history',[],'options',options,'scope',"researchAffineTimeLinearization", ...
        'uncertaintyScope',"existingPerHoldApproximationWithoutInteriorMeasurementBenefit");
    for iteration=1:options.RefinementIterations
        wall=tic;
        [base,model]=localProblem(anchor,model,inherited);
        model.probeNormals=model.collisionEscapeNormals;
        step=options.DifferenceSeconds;
        [plus,plusAnchor]=localPerturb(anchor,model,inherited,durations+weights*step,options.MovingReference);
        [minus,minusAnchor]=localPerturb(anchor,model,inherited,durations-weights*step,options.MovingReference);
        [halfPlus,~]=localPerturb(anchor,model,inherited,durations+weights*step/2,options.MovingReference);
        [halfMinus,~]=localPerturb(anchor,model,inherited,durations-weights*step/2,options.MovingReference);
        compatible=localSameRows(base,plus) && localSameRows(base,minus) ...
            && localSameRows(base,halfPlus) && localSameRows(base,halfMinus);
        if ~compatible
            result.status="constraintStructureChanged";break;
        end
        derivative=localDifference(plus,minus,2*step);
        fine=localDifference(halfPlus,halfMinus,step);
        coarseVector=localFiniteVector(derivative);fineVector=localFiniteVector(fine);
        error=max(abs(coarseVector-fineVector)./max(1,abs(fineVector)));
        if error>1e-2 || ~isfinite(error)
            result.status="nonsmoothTimeDerivative";result.derivativeError=error;break;
        end
        elapsed=sum(durations)-initialDuration;
        % The final intervals remain at least one real sample long. The
        % finite difference alone may sample just below h to find a slope.
        free=localAugment(base,derivative,[-elapsed,maximumIncrease-elapsed],model);
        free.durationStateDerivative=(plusAnchor.states-minusAnchor.states)/(2*step);
        result.fixedProblem=base;result.freeProblem=free;
        formulationSeconds=toc(wall);
        if iteration==1
            result.fixed=localSolve(base,anchor,model,inherited,weights,0);
        end
        trial=localSolve(free,anchor,model,inherited,weights,0);
        trial.formulationSeconds=formulationSeconds;trial.derivativeError=error;
        trial.dynamicsTimeDerivativeNorm=norm(derivative.rhs,Inf);
        trial.constraintTimeDerivativeNorm=norm(derivative.b,Inf);
        history{iteration}=trial;result.free=trial;
        if ~trial.returned,result.status="freeTimeSolveFailed";break;end
        result.status="completed";
        if iteration<options.RefinementIterations
            if ~trial.audit.rolloutAvailable,result.status="nonlinearRolloutFailed";break;end
            anchor.states=trial.audit.states;anchor.inputs=trial.solution.inputs;
            durations=trial.durations;model.predictionDurations=durations;
            % Refine around the new physical path. This is explicitly an
            % offline SQP-style experiment, not a one-iteration RTI result.
            model=rmfield(model,'probeNormals');
        end
    end
    result.history=history(~cellfun('isempty',history));
    result.initialAnchor=initialAnchor;result.model=model;
end

function model=localSnapshotModel(snapshot)
    cfg=snapshot.configuration;
    [ego,lane,road,observation]=readControllerInputs(snapshot.egoInput,snapshot.targetEstimate,snapshot.road,cfg);
    q=predictiveSafetyGeometry.target(observation,snapshot.targetEstimate,cfg);
    frame=predictiveSafetyGeometry.roadFrame(lane,road);
    uncertainty=predictiveSafetyGeometry.observerUncertainty(ego,observation,q);
    previous=snapshot.previousState;index=0;epoch=ego.stateTime;input=zeros(2,1);
    if isstruct(previous),index=previous.sampleIndex+1;epoch=previous.epochTime;end
    if isfield(snapshot.egoInput,'heldActuatorInput'),input=snapshot.egoInput.heldActuatorInput;end
    model=struct('cfg',cfg,'initialState',ego.modelState,'previousInput',input, ...
        'target',q,'targetEpoch',q,'targetEpochIndex',index,'targetOnset',isempty(previous), ...
        'sampleIndex',index,'epochTime',epoch,'lane',lane,'road',road,'frame',frame, ...
        'nominalReference',nonlinearBicycleModel.cruise(cfg,frame(4)), ...
        'stateTime',ego.stateTime,'uncertainty',uncertainty);
end

function [problem,anchor]=localPerturb(anchor,model,inherited,durations,moving)
    model.predictionDurations=durations;
    if moving
        for k=1:numel(durations)
            cfg=model.cfg;cfg.controller.sampleTime=durations(k);
            [~,~,~,anchor.states(:,k+1)]=nonlinearBicycleModel.hold(anchor.states(:,k),anchor.inputs(:,k),cfg);
        end
    end
    [problem,~]=localProblem(anchor,model,inherited);
end

function [problem,model]=localProblem(anchor,model,inherited)
    request=struct('operation',"formulate",'model',model,'anchor',anchor,'inherited',inherited);
    [problem,~,model]=solvePredictiveControl("probe",request);
end

function same=localSameRows(a,b)
    same=isequal(size(a.a),size(b.a)) && a.terminalRowCount==b.terminalRowCount ...
        && isequal(size(a.equal),size(b.equal)) && numel(a.cones)==numel(b.cones) ...
        && isequal(isfinite(a.lower),isfinite(b.lower)) && isequal(isfinite(a.upper),isfinite(b.upper));
end

function derivative=localDifference(plus,minus,width)
    derivative=struct('b',(plus.b-minus.b)/width,'rhs',(plus.rhs-minus.rhs)/width, ...
        'lower',zeros(size(plus.lower)),'upper',zeros(size(plus.upper)),'cones',plus.cones, ...
        'trackingOffset',(plus.trackingOffset-minus.trackingOffset)/width);
    low=isfinite(plus.lower);high=isfinite(plus.upper);
    derivative.lower(low)=(plus.lower(low)-minus.lower(low))/width;
    derivative.upper(high)=(plus.upper(high)-minus.upper(high))/width;
    for k=1:numel(plus.cones)
        derivative.cones(k).b=(plus.cones(k).b-minus.cones(k).b)/width;
        derivative.cones(k).gamma=(plus.cones(k).gamma-minus.cones(k).gamma)/width;
    end
end

function value=localFiniteVector(derivative)
    value=[derivative.b;derivative.rhs;derivative.lower;derivative.upper;derivative.trackingOffset];
    for cone=derivative.cones,value=[value;cone.b;cone.gamma(:)];end %#ok<AGROW>
end

function problem=localAugment(base,derivative,limits,model)
    problem=base;n=numel(base.lower);map=[1:n-1,n+1];timeIndex=n;
    problem.clfIndex=n+1;problem.durationIndex=timeIndex;
    for field=["a","equal","trackingMap","inputTieMap","clfMap","nominalClfMap"]
        matrix=sparse(size(base.(field),1),n+1);matrix(:,map)=base.(field);problem.(field)=matrix;
    end
    problem.a(:,timeIndex)=-derivative.b;problem.equal(:,timeIndex)=-derivative.rhs;
    for field=["safetyObjective","performanceObjective","lower","upper"]
        vector=zeros(n+1,1);vector(map)=base.(field);problem.(field)=vector;
    end
    problem.lower(timeIndex)=limits(1);problem.upper(timeIndex)=limits(2);
    problem.quadratic=sparse(n+1,n+1);problem.quadratic(map,map)=base.quadratic;
    problem.trackingMap(:,timeIndex)=derivative.trackingOffset;
    % Include the time displacement in the original bounded tracking tie.
    % Otherwise a long duration step could outweigh the CLF objective.
    span=zeros(n+1,1);
    span(problem.stateIndices(:))=repmat(model.cfg.nonlinear.trustRadius*[5;5;.5;5;3;1.5],size(problem.stateIndices,2),1);
    span(timeIndex)=max(abs(limits));
    trackingBound=max(eps,sum((abs(problem.trackingOffset)+abs(problem.trackingMap)*span).^2));
    problem.trackingTieWeight=base.inputTieWeight/trackingBound;
    problem.quadratic=2*(base.inputTieWeight*(problem.inputTieMap.'*problem.inputTieMap) ...
        +problem.trackingTieWeight*(problem.trackingMap.'*problem.trackingMap));
    problem.performanceObjective=2*problem.trackingTieWeight*problem.trackingMap.'*problem.trackingOffset;
    for k=1:numel(base.cones)
        a=sparse(size(base.cones(k).A,1),n+1);a(:,map)=base.cones(k).A;a(:,timeIndex)=-derivative.cones(k).b;
        d=sparse(size(base.cones(k).D,1),n+1);d(:,map)=base.cones(k).D;d(:,timeIndex)=-derivative.cones(k).gamma;
        problem.cones(k).A=a;problem.cones(k).D=d;
    end
    % Move duration-dependent variable bounds into affine inequalities.
    for k=find(derivative.lower~=0).'
        row=sparse(1,n+1);row(map(k))=-1;row(timeIndex)=derivative.lower(k);
        problem.a=[problem.a;row];problem.b=[problem.b;-base.lower(k)];problem.lower(map(k))=-Inf;
    end
    for k=find(derivative.upper~=0).'
        row=sparse(1,n+1);row(map(k))=1;row(timeIndex)=-derivative.upper(k);
        problem.a=[problem.a;row];problem.b=[problem.b;base.upper(k)];problem.upper(map(k))=Inf;
    end
end

function trial=localSolve(problem,anchor,model,inherited,weights,change)
    wall=tic;
    request=struct('operation',"solve",'model',model,'anchor',anchor,'problem',problem);
    [solution,search,~]=solvePredictiveControl("probe",request);
    seconds=toc(wall);
    trial=struct('returned',~isempty(solution),'solveSeconds',seconds,'search',search, ...
        'durationSeconds',NaN,'durationChangeSeconds',NaN,'durations',[], ...
        'solution',solution,'audit',[]);
    if isempty(solution),return;end
    if isfield(solution,'durationChangeSeconds'),change=solution.durationChangeSeconds;end
    durations=model.predictionDurations+weights*change;
    trial.durationChangeSeconds=change;trial.durationSeconds=sum(durations);trial.durations=durations;
    trial.audit=localAudit(solution,anchor,model,inherited,durations);
end

function audit=localAudit(solution,anchor,model,inherited,durations)
    audit=struct('rolloutAvailable',false,'reason',"notRun",'states',[], ...
        'minimumRectangleGapMeters',NaN,'terminalMember',false,'terminalReason',"", ...
        'poseLinearizationGapMeters',NaN,'relinearizedViolation',NaN, ...
        'firstHoldSeconds',durations(1),'maximumHoldSeconds',max(durations));
    states=zeros(size(anchor.states));states(:,1)=model.initialState;
    middles=zeros(6,numel(durations));
    try
        for k=1:numel(durations)
            cfg=model.cfg;cfg.controller.sampleTime=durations(k);
            [middles(:,k),~,~,states(:,k+1)]=nonlinearBicycleModel.hold(states(:,k),solution.inputs(:,k),cfg);
        end
        audit.rolloutAvailable=true;audit.states=states;
        audit.poseLinearizationGapMeters=max(vecnorm(states(1:2,:)-solution.states(1:2,:),2,1));
        model.predictionDurations=durations;
        replay=anchor;replay.states=states;replay.inputs=solution.inputs;
        [checked,model]=localProblem(replay,model,inherited);
        point=zeros(numel(checked.lower),1);point(checked.slackIndices)=solution.stageSlacks;
        point(checked.clfIndex)=solution.clfSlack/checked.clfScale;
        audit.relinearizedViolation=max([0;checked.a*point-checked.b;checked.lower-point;point-checked.upper;abs(checked.equal*point-checked.rhs)]);
        for cone=checked.cones
            residual=vecnorm(reshape(cone.A*point-cone.b,cone.size,[]),2,1) ...
                -reshape(cone.D*point-cone.gamma,1,[]);
            audit.relinearizedViolation=max([audit.relinearizedViolation,residual]);
        end
        context=model.terminalContext;
        [audit.terminalMember,~,info]=terminalSafeSet.member(context,states(:,end),sum(durations)/model.cfg.controller.sampleTime);
        audit.terminalReason=info.reason;
        times=model.recordTimes;poses=zeros(6,numel(times));poses(:,1:2:end)=states;poses(:,2:2:end)=middles;
        audit.minimumRectangleGapMeters=Inf;
        if ~isempty(model.target)
            q=predictiveSafetyGeometry.predictTarget(model.target,times);
            shape=[model.cfg.vehicle.length/2;model.cfg.vehicle.width/2;model.cfg.vehicle.rectangleOffset];
            gaps=zeros(size(times));
            for k=1:numel(times),gaps(k)=predictiveSafetyGeometry.rectangle(poses(1:3,k),shape,q(1:3,k),q(8:11,k));end
            audit.minimumRectangleGapMeters=min(gaps);
        end
        audit.reason="diagnosticOnly";
    catch exception
        if ~startsWith(string(exception.identifier),"collisionAvoidanceController:"),rethrow(exception);end
        audit.reason=string(exception.identifier);
    end
end
