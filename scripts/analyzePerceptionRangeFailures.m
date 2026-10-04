function analysis=analyzePerceptionRangeFailures(directory)
%analyzePerceptionRangeFailures Reproduce saved failures in a dedicated process.
% Non-stopping conditional breakpoints capture the actual conic formulation.
% All counterfactuals are offline single-frame experiments, never controls
% issued by the scenario driver. No admission or diagnostic layer is added.
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    assignin('base','rangeFailureCapture',@localCapture);
    cleanup=onCleanup(@localClear);
    primaryLines=splitlines(string(fileread(fullfile(root,'controller','solvePredictiveControl.m'))));
    primaryLine=find(startsWith(strtrim(primaryLines),'primaryCount=problem.clfIndex-1;'),1);
    candidateLine=find(startsWith(strtrim(primaryLines),'scale=max(1,candidate.clfInitialValue);'),1);
    entryLines=splitlines(string(fileread(fullfile(root,'controller','collisionAvoidanceController.m'))));
    entryLine=find(strtrim(entryLines)=="if isempty(solution)",1);
    dbstop('in','solvePredictiveControl','at',num2str(primaryLine),'if', ...
        "feval(evalin('base','rangeFailureCapture'),'primary',model,problem)");
    dbstop('in','collisionAvoidanceController','at',num2str(entryLine),'if', ...
        "feval(evalin('base','rangeFailureCapture'),'result',model,search)");
    dbstop('in','solvePredictiveControl','at',num2str(candidateLine),'if', ...
        "feval(evalin('base','rangeFailureCapture'),'candidate',trialModel,struct('solution',candidate,'segment',segment))");
    files=dir(fullfile(directory,'*-failure.mat'));analysis=struct([]);
    for index=1:numel(files)
        file=fullfile(files(index).folder,files(index).name);
        loaded=load(file,'snapshot');s=loaded.snapshot;
        localCapture("reset",[],[]);
        actual=localSolve(s.egoInput,s.targetEstimate,s);
        captured=localCapture("get",[],[]);
        if isempty(s.sensorFrame)
            shape=[s.configuration.vehicle.length/2;s.configuration.vehicle.width/2;s.configuration.vehicle.rectangleOffset];
            targetShape=[s.targetTruth.targetLength/2;s.targetTruth.targetWidth/2;s.targetTruth.targetRectangleOffset];
            distance=predictiveSafetyGeometry.rectangle([s.egoTruth.position;s.egoTruth.yaw],shape, ...
                [s.targetTruth.targetPositionInertial;s.targetTruth.targetYawInertial],targetShape);
            row=struct('scenario',s.scenario,'speed',s.configuration.referenceSpeed,'time',s.time, ...
                'currentTruthClearanceMeters',distance,'actual',actual,'search',captured.result.search, ...
                'currentTrueRangeMeters',norm(s.targetTruth.targetPositionInertial-s.egoTruth.position), ...
                'candidateBases',localCandidateBases(captured.candidate), ...
                'freshSeedHardOverlapTimes',localSeedOverlaps(captured.primary{end}.model), ...
                'freshHardRowContradiction',localHardRow(captured.primary{end}.problem));
            analysis=[analysis,row]; %#ok<AGROW>
            localSave(directory,file,captured,analysis);
            fprintf('ANALYSIS exact %g %s at %.3f s: %s\n',row.speed,row.scenario,row.time,actual.failure);
            continue;
        end
        m=captured.result.model;p=captured.primary{1}.problem;
        if ~isfield(m,'uncertaintyPrediction'),m=captured.primary{end}.model;end
        initialTruth=[s.egoTruth.position;s.egoTruth.yaw;s.egoTruth.speed; ...
            s.egoTruth.lateralVelocity;s.egoTruth.yawRate];
        pointError=m.initialState-initialTruth;
        pointError(3)=atan2(sin(pointError(3)),cos(pointError(3)));
        endpointRadii=cellfun(@(item)item.model.uncertaintyPrediction.terminalReserve,captured.primary);
        coreRadius=m.terminal.radius;
        targetPrediction=m.uncertaintyPrediction.target;
        targetPositionError=norm(m.target(1:2)-s.targetTruth.targetPositionInertial);
        egoOnly=s.egoInput;egoOnly.targetEstimate=[];
        egoOnly.perception=struct('time',s.time,'range',50,'completeWithinRange',true);
        withoutTarget=localSolve(egoOnly,[],s);
        pointEgo=s.egoInput;pointTarget=s.targetEstimate;
        pointEgo=localRemoveBound(pointEgo);pointTarget=localRemoveBound(pointTarget);
        if isfield(pointTarget,'predictionErrorSet'),pointTarget=rmfield(pointTarget,'predictionErrorSet');end
        pointOnly=localSolve(pointEgo,pointTarget,s);
        % Preserve target uncertainty while setting only the ego radius to zero.
        zeroEgo=s.egoInput;zeroEgo.controllerErrorBound.bounds(:)=0;
        if isfield(zeroEgo.controllerErrorBound,'generator'),zeroEgo.controllerErrorBound.generator(:)=0;end
        targetOnly=localSolve(zeroEgo,s.targetEstimate,s);
        targetOnlyTrace=localCapture("get",[],[]);targetOnlyCapture=targetOnlyTrace.primary{end};
        row=struct('scenario',s.scenario,'speed',s.configuration.referenceSpeed, ...
            'time',s.time,'sensorDetected',s.sensorFrame.radarDetectionAvailable, ...
            'trueRangeMeters',norm(s.targetTruth.targetPositionInertial-s.egoTruth.position), ...
            'actual',actual,'egoEstimateError',pointError,'egoInitialRadius',sum(abs(m.uncertainty.egoGenerator),2), ...
            'targetPositionEstimateErrorMeters',targetPositionError, ...
            'targetSpeedEstimate',m.target(4),'targetAccelerationEstimate',m.target(5), ...
            'targetSideslipEstimate',m.target(6),'targetUncertainty',m.uncertainty.target, ...
            'targetHistory',s.targetEstimate.measurementHistoryEnclosure, ...
            'targetPublishedComponentBounds',s.targetEstimate.controllerErrorBound.bounds, ...
            'targetPrediction',targetPrediction,'horizonSeconds',m.uncertaintyPrediction.time(end), ...
            'terminalCoreRadius',coreRadius,'terminalReserves',endpointRadii, ...
            'terminalReserveToRadius',endpointRadii/coreRadius, ...
            'terminalPureSpeedHalfWidth',coreRadius/norm(m.terminal.quotientFactor(:,1)), ...
            'endpointEgoRadius',m.uncertaintyPrediction.egoStateRadius(:,end), ...
            'endpointConeGamma',p.cones(1).gamma,'endpointConeHasVariableRightSide',nnz(p.cones(1).d)>0, ...
            'maximumCollisionTighteningMeters',p.maximumCollisionTighteningMeters, ...
            'search',captured.result.search,'withoutTarget',withoutTarget, ...
            'pointEstimatesOnly',pointOnly,'targetUncertaintyOnly',targetOnly, ...
            'targetOnlyHardRowContradiction',localHardRow(targetOnlyCapture.problem));
        analysis=[analysis,row]; %#ok<AGROW>
        localSave(directory,file,captured,analysis);
        fprintf('ANALYSIS %g %s: terminal reserve/radius %.6g, point-only %d, target-only %d, no-target %d\n', ...
            row.speed,row.scenario,max(row.terminalReserveToRadius),pointOnly.returned,targetOnly.returned,withoutTarget.returned);
    end
end

function times=localSeedOverlaps(model)
    cfg=model.cfg;times=[];anchor=model.linearization;
    shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
    for k=cfg.controller.horizonSteps+1:size(anchor.inputs,2)
        middle=nonlinearBicycleModel.hold(anchor.states(:,k),anchor.inputs(:,k),cfg);
        for half=0:1
            x=anchor.states(:,k);if half==1,x=middle;end
            t=(k-1+half/2)*cfg.controller.sampleTime;
            q=predictiveSafetyGeometry.predictTarget(model.target,t);
            if norm(x(1:2)-q(1:2))<=cfg.collision.encounterRangeMeters ...
                    && predictiveSafetyGeometry.rectangle(x(1:3),shape,q(1:3),q(8:11))==0
                times(end+1)=model.stateTime+t; %#ok<AGROW>
            end
        end
    end
end

function result=localCandidateBases(candidates)
    result=struct([]);
    for index=1:numel(candidates)
        model=candidates{index}.model;value=candidates{index}.value;
        problem=value.segment.problem;base=[value.segment.primary;0];
        residuals=zeros(1,problem.primaryConeCount);
        for k=1:numel(residuals)
            cone=problem.cones(k);
            residuals(k)=norm(cone.A*base-cone.b)-cone.d.'*base+cone.gamma;
        end
        affine=model.linearization.states+reshape(base(problem.stateIndices),size(model.linearization.states));
        inputs=model.linearization.inputs+reshape(base(problem.inputIndices),size(model.linearization.inputs));
        nonlinear=affine;nonlinear(:,1)=model.initialState;
        for k=1:size(inputs,2)
            nonlinear(:,k+1)=nonlinearBicycleModel.sample(nonlinear(:,k),inputs(:,k),model.cfg);
        end
        poseCount=min(size(affine,2),problem.encounterExit+1);
        reach=norm([model.cfg.vehicle.length;model.cfg.vehicle.width]/2)+norm(model.cfg.vehicle.rectangleOffset);
        error=vecnorm(affine(1:2,:)-nonlinear(1:2,:))+reach*abs(affine(3,:)-nonlinear(3,:));
        row=struct('baseTerminalConeResidual',residuals(1), ...
            'baseLargestPrimaryConeResidual',max(residuals), ...
            'baseLargestLinearResidual',max(problem.a*base-problem.b), ...
            'basePoseErrorMeters',max(error(1:poseCount)));
        result=[result,row]; %#ok<AGROW>
    end
end

function localSave(directory,file,captured,analysis)
    captureFile=replace(file,'-failure.mat','-formulation.mat');
    save(captureFile,'captured','-v7.3');
    fid=fopen(fullfile(directory,'failure-analysis.json'),'w');
    fprintf(fid,'%s\n',jsonencode(analysis));fclose(fid);
end

function result=localHardRow(problem)
    lower=problem.lower;upper=problem.upper;
    % The initial-state equality fixes its correction before optimization.
    lower(problem.stateIndices(:,1))=problem.rhs(1:6);
    upper(problem.stateIndices(:,1))=problem.rhs(1:6);
    [row,column,value]=find(problem.a);
    limit=lower(column);negative=value<0;limit(negative)=upper(column(negative));
    minimum=accumarray(row,value.*limit,[size(problem.a,1),1],@sum,0);
    hard=~any(problem.a(:,problem.slackIndices),2);
    gaps=minimum-problem.b;gaps(~hard)=-Inf;
    [gap,index]=max(gaps);
    stage=find(any(reshape(full(problem.a(index,problem.stateIndices(:)))~=0,6,[]),1),1,'last');
    result=struct('minimumUnavoidableViolation',gap,'row',index, ...
        'latestStateStage',stage-1,'rightSide',problem.b(index),'boxMinimumLeftSide',minimum(index), ...
        'nonzeroCoefficients',nnz(problem.a(index,:)));
end

function result=localSolve(ego,target,s)
    timer=tic;
    try
        [command,~,prediction]=collisionAvoidanceController(ego,target,s.road,s.configuration,s.previousState);
        result=struct('returned',true,'failure',"",'seconds',toc(timer), ...
            'input',command.actuatorInput,'search',prediction.metadata.search);
    catch exception
        result=struct('returned',false,'failure',string(exception.identifier)+": "+string(exception.message), ...
            'seconds',toc(timer),'input',[],'search',[]);
    end
end
function data=localRemoveBound(data)
    if isfield(data,'controllerErrorBound'),data=rmfield(data,'controllerErrorBound');end
end
function result=localCapture(kind,model,value)
    persistent trace
    result=false;
    if kind=="reset"
        trace=struct('primary',{{}},'candidate',{{}},'result',[]);
        return;
    elseif kind=="get"
        result=trace;
        return;
    end
    if kind=="primary"
        trace.primary{end+1}=struct('model',model,'problem',value);
    elseif kind=="candidate"
        trace.candidate{end+1}=struct('model',model,'value',value);
    else
        trace.result=struct('model',model,'search',value);
    end
end
function localClear()
    dbclear('in','solvePredictiveControl');
    dbclear('in','collisionAvoidanceController');
    evalin('base','clear rangeFailureCapture');
    localCapture("reset",[],[]);
end
