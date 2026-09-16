function summary = diagnoseOncomingAdmission(options)
%diagnoseOncomingAdmission Replay and isolate first-admission hard constraints.
% Constraint removal is an offline diagnostic, never an executable controller.
    arguments
        options.ReplayReport (1,1) string
        options.OutputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    loaded=load(options.ReplayReport,'report');baseline=loaded.report;
    assert(baseline.scenario=="oncoming" && baseline.failureTime==2.6);
    cfg=baseline.configuration;cfg.solver.frameDeadlineSeconds=Inf;
    localCapture("reset");cfg.solver.jointFunction=@localCapture;
    road=struct('centerline',[-100,0;2000,0]);
    prior=[];problem=[];failure=[];
    for index=1:baseline.executedHolds+1
        time=(index-1)*cfg.controller.sampleTime;
        if isempty(prior)
            ego=struct('position',[0;0],'yaw',0,'speed',8, ...
                'lateralVelocity',0,'yawRate',0);
        else
            x=prior.predictedState(:,2);
            [position,heading]=laneGeometry.fromFrenet(x,problem.model.lane);
            ego=struct('position',position,'yaw',heading,'speed',x(4), ...
                'lateralVelocity',x(5),'yawRate',x(6),'heldActuatorInput',prior.appliedInput);
        end
        ego.stateTime=time;ego.controllerStateErrorBound=zeros(6,1);
        ego.perception=struct('time',time,'range',16,'completeWithinRange',true);
        target=struct('trackId',1,'targetPositionInertial',[60-8*time;0], ...
            'targetVelocityInertial',[-8;0],'targetAccelerationInertial',[0;0], ...
            'targetHeadingInertial',pi,'targetYawRate',0, ...
            'predictionMotion',struct('kind',"finite-sensing-motion-v1", ...
            'jerkBound',[0;0],'yawAccelerationBound',0));
        try
            [~,~,problem,prior]=collisionAvoidanceController(ego,target,road,cfg,prior);
        catch exception
            failure=exception;break;
        end
    end
    failed=localCapture("read");localCapture("reset");
    captured=failed{1};native=failed{2};
    assert(~isempty(failure) && time==baseline.failureTime && ~isempty(captured));
    n=captured.layout.planCount;
    matrix=captured.physicalMatrix(:,1:n);bound=captured.physicalBound;
    labels=captured.physicalLabels;
    collision=startsWith(labels,"collision:");exit=startsWith(labels,"exit:");
    terminal=labels=="terminalEntry";actuator=labels=="actuator";
    names=["allAffine","withoutCollision","withoutExit","withoutTerminalEntry", ...
        "withoutExitAndTerminalEntry","collisionAndActuator", ...
        "withoutModelDomain","withoutTireSlip"];
    masks={true(size(labels)),~collision,~exit,~terminal,~(exit|terminal), ...
        collision|actuator,labels~="modelDomain",labels~="tireSlip"};
    lpOptions=optimoptions('linprog','Display','none');
    trials=cell(size(names));
    for index=1:numel(names)
        selected=masks{index};
        [decision,~,flag,output]=linprog(zeros(n,1),matrix(selected,:),bound(selected), ...
            [],[],[],[],lpOptions);
        residual=NaN;
        if ~isempty(decision),residual=max(matrix(selected,:)*decision-bound(selected));end
        trials{index}=struct('name',names(index),'exitFlag',flag,'maximumResidual',residual, ...
            'message',string(output.message));
    end
    [relaxed,minimumViolation,relaxFlag,~,multipliers]=linprog([zeros(n,1);1], ...
        [matrix,-double(collision)],bound,[],[],[-inf(n,1);0],[],lpOptions);
    assert(relaxFlag==1 && minimumViolation>0);
    dual=multipliers.ineqlin;
    stationarity=matrix.'*dual;
    contradiction=bound.'*dual+abs(stationarity).'*captured.decisionRadius;
    assert(min(dual)>=0 && contradiction<0);
    stage=[captured.geometry.stage;nan(numel(bound)-numel(captured.geometry.stage),1)];
    low=0;high=captured.layout.horizonSteps;
    while high-low>1
        middle=floor((high+low)/2);
        selected=stage<=middle | actuator;
        [~,~,flag]=linprog(zeros(n,1),matrix(selected,:),bound(selected),[],[],[],[],lpOptions);
        assert(any(flag==[1,-2]));
        if flag==1,low=middle;else,high=middle;end
    end
    [~,criticalRow]=max(dual.*double(collision));
    [~,minimumValue,rowFlag]=linprog(matrix(criticalRow,:).',matrix(~collision,:), ...
        bound(~collision),[],[],[],[],lpOptions);
    assert(rowFlag==1);
    critical=localRowDetails(captured,criticalRow,cfg.controller.sampleTime);
    critical.minimumViolation=minimumValue-bound(criticalRow);
    critical.maximumLateralHeadingCombination=-critical.limit-critical.minimumViolation;
    critical.brakingCoefficientMaximum=max(abs(matrix(criticalRow,2:2:n)));
    summary=struct('failureTime',time,'completedCruiseHolds',round(time/cfg.controller.sampleTime), ...
        'centerSeparation',norm(target.targetPositionInertial-ego.position), ...
        'egoSpeed',ego.speed,'targetSpeed',8,'nativeExitFlag',native.exitFlag, ...
        'horizonSteps',captured.layout.horizonSteps,'labels',unique(labels), ...
        'affineAblations',[trials{:}],'collisionRelaxationFlag',relaxFlag, ...
        'minimumCommonCollisionRowViolation',minimumViolation, ...
        'firstInfeasiblePrefixStage',high,'criticalCollisionRow',critical, ...
        'dualBound',bound.'*dual,'dualStationarityMaximum',norm(stationarity,inf), ...
        'dualMinimum',min(dual),'boundedInputContradiction',contradiction, ...
        'dualNonzeroRows',find(dual>1e-10),'dualNonzeroWeights',dual(dual>1e-10), ...
        'scope',"Offline necessary-condition diagnostics; relaxed decisions are not safe commands");
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    save(fullfile(options.OutputDirectory,'admission-diagnosis.mat'), ...
        'summary','captured','native','relaxed','dual','ego','target','problem','prior');
    stream=fopen(fullfile(options.OutputDirectory,'summary.json'),'w');
    cleanup=onCleanup(@() fclose(stream));
    fprintf(stream,'%s\n',jsonencode(summary,PrettyPrint=true));
    disp(summary);

end

function result=localCapture(action,program)
    persistent captured native
    if action=="reset",captured=[];native=[];result=[];return;end
    if action=="read",result={captured,native};return;end
    result=program.defaultSolver();
    if result.exitFlag<0
        captured=rmfield(program,'defaultSolver');native=result;
    end
end

function details=localRowDetails(program,row,sampleTime)
    groups=program.geometry.local;
    ends=cumsum(arrayfun(@(group)numel(group.bound),groups));
    cellIndex=find(ends>=row,1);first=0;
    if cellIndex>1,first=ends(cellIndex-1);end
    localRow=row-first;group=groups(cellIndex);perPoint=size(group.nodeStateRows,1);
    point=ceil(localRow/perPoint);source=mod(localRow-1,perPoint)+1;
    cells=find([groups.stage]==group.stage);
    % The affine predictor divides each held-input stage into equal cells.
    cellStart=(group.stage-1)*sampleTime+(find(cells==cellIndex)-1)*sampleTime/numel(cells);
    details=struct('row',row,'cell',cellIndex,'stage',group.stage,'bernsteinPoint',point, ...
        'cellStartSeconds',cellStart,'stateRow',group.nodeStateRows(source,:,point), ...
        'inputRow',group.nodeInputRows(source,:,point),'limit',group.nodeLimits(source,point), ...
        'normal',program.geometry.normals{cellIndex});
end
