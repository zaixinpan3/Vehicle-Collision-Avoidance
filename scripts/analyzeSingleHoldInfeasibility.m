function analysis = analyzeSingleHoldInfeasibility(campaignFile, outputDirectory)
%analyzeSingleHoldInfeasibility Replay saved measurements and isolate hard rows.
% Offline diagnosis only: no modified program issues a controller command.
% Removing rows or uncertainty below is a counterfactual, not a safety policy.
    arguments
        campaignFile (1,1) string
        outputDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'), ...
        fullfile(root,'solver','bicycle'),fullfile(root,'solver','clarabel','matlab'));
    loaded = load(campaignFile,'campaign');
    campaign = loaded.campaign;
    assert(numel(campaign.trials)==8,'Expected the eight-case straight campaign.');
    names = ["exact-stationary","exact-oncoming","exact-crossing", ...
        "bounded-stationary","bounded-oncoming","bounded-crossing", ...
        "range-exact-oncoming","range-nrmm-oncoming"];
    cases = cell(1,numel(campaign.trials));
    failures = cell(size(cases));
    for caseIndex = 1:numel(cases)
        report = campaign.trials{caseIndex};
        assert(~report.roadBoundariesEnabled,'This diagnostic requires the no-road campaign.');
        measurements = localMeasurements(report,caseIndex);
        count = numel(measurements);
        if report.completed, count = min(count,20); end
        timeline = cell(1,count);
        for sample = 1:count
            data = measurements{sample};
            model = localModel(data,report.configuration);
            [program,prediction,clf] = formulateAvoidanceProblem(model);
            [labels,decayRows,obstacleRows] = localLabels(model,prediction,clf);
            item = localRowAudit(model,program,labels);
            if sample<=size(report.input,2),item.recordedInput=report.input(:,sample);end
            timeline{sample} = item;
        end
        last = timeline{end};
        if ~report.completed
            native = solveHardCbfClf.constrained(program,model.cfg);
            last.nativeExitFlag = native.exitFlag;
            last.nativeMessage = native.message;
            hardRows = (1:program.cones(2)-1).';
            last.hardOnly = localLinearSolve(program,hardRows);
            last.withoutDecay = localLinearSolve(program,setdiff(hardRows,decayRows));
            last.withoutObstacles = localLinearSolve(program,setdiff(hardRows,obstacleRows));
            exactModel = model;
            exactModel.initialFrenetErrorBound(:)=0;
            if ~isempty(exactModel.encounter)
                exactModel.encounter.radius(:)=0;
                exactModel.encounter.parameters=targetPrediction.nrmmParameters(exactModel.encounter.center, ...
                    zeros(8,1),exactModel.encounter.contract,[]);
            end
            exactProgram = formulateAvoidanceProblem(exactModel);
            last.withoutUncertainty = localLinearSolve(exactProgram,(1:exactProgram.cones(2)-1).');
            % Use the common initial error in b(h)-q*b(0) instead of two
            % independent endpoint boxes. Retain all other enclosure reserves.
            paired = program;
            reduction = 0;
            phi = clf.cruise.transition(1:6,1:6);
            q = program.barrier.contraction;
            if ~isempty(model.encounter)
                normal = program.barrier.normal;
                row = [normal.',zeros(1,4)];
                assert(all(abs(model.lane.tangent(:,2))<1e-12), ...
                    'The correlated-error counterfactual requires a straight x-axis chart.');
                reduction = (abs(row)*abs(phi)+q*abs(row) ...
                    -abs(row*phi-q*row))*model.initialFrenetErrorBound ...
                    +2*q*abs(normal).'*model.encounter.radius(1:2);
                paired.b(decayRows)=paired.b(decayRows)+reduction;
            end
            last.pairedInitialErrorSupport = localLinearSolve(paired,hardRows);
            last.pairedSupportReduction = reduction;
            last.failureState = model.initialEgoState;
            last.egoErrorRadius = model.initialFrenetErrorBound;
            last.targetCenters = [model.encounter.center];
            last.targetErrorRadius = [model.encounter.radius];
            last.decayMatrix = full(program.A(decayRows,:));
            last.decayBound = program.b(decayRows);
            last.equilibriumInput = clf.cruise.input;
            last.nextPositionInputMap = clf.cruise.transition(1:2,7:8);
            last.plantGenerator = [clf.cruise.stage.continuousA, ...
                clf.cruise.stage.continuousB,clf.cruise.stage.continuousC];
            failures{caseIndex} = struct('model',model,'program',program, ...
                'decayRows',decayRows,'obstacleRows',obstacleRows,'labels',labels);
            % Confirm reconstructed input matrices against the actual entry
            % point, including its removal of trivial constant rows.
            captured = [];
            cfg = model.cfg; cfg.solver.jointFunction=@capture;
            try
                collisionAvoidanceController(data.ego,data.target,data.road,cfg,[]);
            catch exception
                assert(strcmp(exception.identifier,'collisionAvoidanceController:optimizationFailed'));
            end
            constant = all(program.A(1:program.cones(2),:)==0,2) & program.b(1:program.cones(2))>=0;
            keep = [~constant;true(size(program.A,1)-program.cones(2),1)];
            last.replayMatrixDifference = norm(full(captured.A-program.A(keep,:)),inf);
            last.replayBoundDifference = norm(captured.b-program.b(keep),inf);
            assert(last.replayMatrixDifference<1e-12 && last.replayBoundDifference<1e-12);
            assert(last.minimumRowViolation>1e-6 && last.hardOnly.exitFlag==-2);
            assert(last.withoutDecay.exitFlag>0 && last.withoutDecay.maximumResidual<1e-8);
        end
        cases{caseIndex} = struct('name',names(caseIndex),'timeline',{timeline},'last',last);
        fprintf('%s: t=%.1f, worst row %s, input-box gap %.9g\n', ...
            names(caseIndex),last.time,last.worstLabel,last.minimumRowViolation);
    end
    cfg = campaign.trials{1}.configuration;
    diskDistance = hypot(cfg.vehicle.length/2,cfg.vehicle.width/2) ...
        +hypot(cfg.target.defaultLength/2,cfg.target.defaultWidth/2);
    analysis = struct('sourceCampaign',campaignFile,'cases',{cases}, ...
        'requiredDiskCenterDistance',diskDistance, ...
        'alignedRectangleLateralDistance',(cfg.vehicle.width+cfg.target.defaultWidth)/2, ...
        'scope',"Offline row diagnosis and counterfactual feasibility; no controller modification or new safe policy");
    analysis.stationaryStoppingCounterfactual = localStoppingCounterfactual(failures{1}.model);
    exactMeasurements = localMeasurements(campaign.trials{1},1);
    model = localModel(exactMeasurements{end-1},campaign.trials{1}.configuration);
    analysis.stationaryEarlierBraking = localEarlierBraking(model);
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    save(fullfile(outputDirectory,'analysis.mat'),'analysis','failures');
    file = fopen(fullfile(outputDirectory,'analysis.json'),'w');
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(analysis,PrettyPrint=true));
    function result = capture(~,candidate)
        captured = candidate;
        result = candidate.defaultSolver();
    end
end

function item = localRowAudit(model,program,labels)
    count = program.cones(2)-1;
    matrix = full(program.A(1:count,1:2));
    cfg = model.cfg;
    lower = [-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    upper = [cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    smallest = sum(min(matrix.*lower.',matrix.*upper.'),2);
    [gap,row] = max(smallest-program.b(1:count));
    item = struct('time',model.stateTime,'state',model.initialEgoState, ...
        'barrierLower',program.barrier.initialLower,'barrierUpper',program.barrier.initialUpper, ...
        'normal',program.barrier.normal,'minimumRowViolation',gap, ...
        'worstLabel',labels(row),'worstRow',row,'worstCoefficients',matrix(row,:), ...
        'worstBound',program.b(row),'recordedInput',nan(2,1));
    decayRows=find(labels=="obstacle:decay");
    item.decayMatrix=matrix(decayRows,:);
    item.decayBound=program.b(decayRows);
    item.betaUpperAtZeroSteering=program.b(decayRows)./matrix(decayRows,2);
end

function result = localLinearSolve(program,rows)
    % Slack is absent from every physical row; removing the CLF leaves an LP.
    matrix = full(program.A(rows,1:2));bound = program.b(rows);
    [u,~,flag,output] = linprog(zeros(2,1),matrix,bound,[],[],[],[], ...
        optimoptions('linprog','Display','none'));
    result = struct('exitFlag',flag,'input',u,'message',string(output.message),'maximumResidual',NaN);
    if ~isempty(u),result.maximumResidual=max(matrix*u-bound);end
    if ~isempty(u)
        index=program.cones(2);
        affine=program.b(index+1:end)-program.A(index+1:end,1:2)*u;
        result.requiredClfSlack=max(0,norm(affine(2:end))-affine(1));
    end
end

function result = localEarlierBraking(model)
% A feasible earlier action can preserve feasibility for one more hold.
% This does not claim a complete safe continuation or eventual passing.
    input=[0;-0.95];
    [program,~,clf]=formulateAvoidanceProblem(model);
    currentResidual=max(program.A(1:program.cones(2)-1,1:2)*input-program.b(1:program.cones(2)-1));
    model.initialEgoState=clf.cruise.transition(1:6,:)*[model.initialEgoState;input;1];
    model.previousInput=input;model.stateTime=model.stateTime+model.sampleTime;
    nextProgram=formulateAvoidanceProblem(model);
    nextResult=localLinearSolve(nextProgram,(1:nextProgram.cones(2)-1).');
    assert(currentResidual<0 && nextResult.exitFlag>0);
    result=struct('inputAtPreviousHold',input,'previousHardResidual',currentResidual, ...
        'nextState',model.initialEgoState,'nextHardFeasibility',nextResult);
end

function result = localStoppingCounterfactual(model)
% Brake within the declared model to 1 mm/s at a sample boundary.
% A small positive speed respects the inward numerical speed-domain reserve.
% Audit swept hard rows with ONLY decay omitted. Never execute this policy.
    start=model.initialEgoState;initialTime=model.stateTime;
    states=start;inputs=zeros(2,0);residuals=zeros(1,0);
    terminalSpeed=0.001;
    for sample=1:20
        [program,prediction,clf]=formulateAvoidanceProblem(model);
        [labels,decayRows,~]=localLabels(model,prediction,clf);
        flow=clf.cruise.transition;
        drift=flow(4,:)*[model.initialEgoState;0;0;1];
        beta=max(-0.9999,(terminalSpeed-drift)/flow(4,8));
        input=[0;beta];
        rows=setdiff(1:program.cones(2)-1,decayRows);
        [residual,worst]=max(program.A(rows,1:2)*input-program.b(rows));
        assert(residual<1e-8,'Counterfactual hold %d violates %s by %.12g.',sample,labels(rows(worst)),residual);
        next=flow(1:6,:)*[model.initialEgoState;input;1];
        inputs(:,end+1)=input;states(:,end+1)=next;residuals(end+1)=residual; %#ok<AGROW>
        model.initialEgoState=next;model.previousInput=input;model.stateTime=model.stateTime+model.sampleTime;
        if abs(next(4)-terminalSpeed)<1e-10,break;end
    end
    assert(abs(states(4,end)-terminalSpeed)<1e-10);
    result=struct('startTime',initialTime,'stopTime',model.stateTime,'inputs',inputs,'states',states, ...
        'maximumRetainedHardResidual',max(residuals),'stoppingDistance',states(1,end)-start(1), ...
        'remainingDiskMargin',program.barrier.initialLower-(states(1,end)-states(1,end-1)), ...
        'scope',"A finite braking prefix in the declared affine plant; no executed fallback and no cruise recovery claim");
end

function [labels,decayRows,obstacleRows] = localLabels(model,prediction,clf)
    model.anchorPlan=clf.cruise.input;
    geometryModel=model;geometryModel.encounter=struct('key',{},'radius',{},'contract',{});
    geometry=avoidanceSafetyGeometry.build(geometryModel,prediction);
    labels=geometry.label(~startsWith(geometry.label,'collision:'));
    targetRows=numel(prediction.cells)*(model.cfg.encounter.taylorOrder+2)+2;
    first=numel(labels);
    if ~isempty(model.encounter)
        suffix=repmat("obstacle:swept",targetRows,1);
        suffix(1)="obstacle:initial";suffix(end)="obstacle:decay";
        labels=[labels;suffix];
    end
    obstacleRows=(first+1:numel(labels)).';
    decayRows=[];
    if ~isempty(model.encounter),decayRows=first+targetRows;end
    labels=[labels;"input:steeringUpper";"input:betaUpper";"input:steeringLower";"input:betaLower"];
end

function model = localModel(data,cfg)
    cfg.solver.frameDeadlineSeconds=Inf;
    [ego,lane,road,target]=readPlanningInputs(data.ego,data.target,data.road,cfg);
    projection=laneGeometry.project(ego.position,lane);
    [radius,~]=stateUncertainty.toFrenet(ego.modelState,ego.stateErrorBound,lane);
    previous=zeros(2,1);
    if ~isempty(ego.heldActuatorInput),previous=ego.heldActuatorInput;end
    model=struct('cfg',cfg,'lane',lane,'road',road,'stateTime',ego.stateTime, ...
        'sampleTime',cfg.controller.sampleTime,'horizonSteps',1,'referenceSpeed',cfg.referenceSpeed, ...
        'initialEgoState',[projection.station;projection.lateralPosition; ...
        atan2(sin(ego.yaw-projection.heading),cos(ego.yaw-projection.heading));ego.modelState(4:6)], ...
        'initialFrenetErrorBound',radius,'longitudinalAccelerationBias',ego.longitudinalAccelerationBias, ...
        'previousInput',previous,'requiredMargin',0,'confirmation',[]);
    model.encounter=[];
    if ~isempty(target)
        model.encounter=targetPrediction.admitOnline(target,model.stateTime,lane,cfg);
    end
end

function measurements = localMeasurements(report,caseIndex)
    count=numel(report.time);measurements=cell(1,count);
    road=struct('centerline',[-100,0;2000,0]);
    if caseIndex>=7
        for sample=1:count
            measurements{sample}=struct('ego',report.egoEstimate{sample}, ...
                'target',report.targetEstimate{sample},'road',road);
        end
        return;
    end
    stream=RandStream('mt19937ar',Seed=report.seed);
    for sample=1:count
        time=report.time(sample);x=report.state(:,sample);
        if any(~isfinite(x)),assert(sample==1);x=[100;0;0;8;0;0];end
        noise=report.egoErrorBound.*(2*rand(stream,6,1)-1);
        ego=struct('position',x(1:2)+[-100;0]+noise(1:2),'yaw',x(3)+noise(3), ...
            'speed',x(4)+noise(4),'lateralVelocity',x(5)+noise(5),'yawRate',x(6)+noise(6), ...
            'stateTime',time,'controllerStateErrorBound',report.egoErrorBound);
        if sample>1,ego.heldActuatorInput=report.input(:,sample-1);end
        % The same draws as the recorded run (nrmmTargetMeasurement after the ego noise).
        target=nrmmTargetMeasurement(report.targetMotion,time,report.targetErrorBound,stream);
        measurements{sample}=struct('ego',ego,'target',target,'road',road);
    end
end
