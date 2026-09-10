function analysis = analyzeStraightEncounterFeasibility(inputDirectory, outputDirectory)
%analyzeStraightEncounterFeasibility Audit saved straight-scene admission failures.
% Rebuilds the exact failed hard program and runs diagnostic constraint
% subsets, exact LTI disturbance-support quadrature, nominal endpoint bounds,
% road-update replay and target-free countdown. Diagnostic relaxations are
% never applied to a physical plant or installed in the online controller.
% inputDirectory contains the September 9 rerun MAT exports. Large outputs
% and the standalone uncertainty figure are written outside the repository.
    arguments
        inputDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    baseline = load(fullfile(inputDirectory,'straight-realtime-validation.mat'),'report');
    acquisition = load(fullfile(inputDirectory,'acquisition.mat'),'acquisition');
    captured = load(fullfile(inputDirectory,'captured-program.mat'),'capturedProgram');
    trial = baseline.report.controllerOnly;
    targetTrial = acquisition.acquisition;
    [qp,prediction,model] = localRebuild(trial.failureContext,trial.controllerConfiguration);
    program = captured.capturedProgram;
    matrixError = max(abs(qp.inequalityMatrix-program.A(1:end-2,1:end-1)),[],'all');
    boundError = max(abs(qp.barrier.baseBound-program.b(1:end-2)));
    assert(matrixError==0 && boundError==0,'The diagnostic must reproduce the actual failed LP.');
    opts = optimoptions('linprog','Display','none');
    labels = localLabels(qp);
    names = unique(labels);
    families = struct('name',{},'aloneExitFlag',{},'withoutExitFlag',{});
    for index = 1:numel(names)
        mask = labels==names(index);
        families(index) = struct('name',names(index), ...
            'aloneExitFlag',localFeasible(qp,mask,opts), ...
            'withoutExitFlag',localFeasible(qp,~mask,opts));
    end
    % These original rows are opposite lateral-force inequalities at the
    % end of cell 2. A tiny trigonometric coefficient cannot close their gap.
    pairRows = [740;746];
    pairBound = qp.barrier.baseBound(pairRows);
    pairNormal = sum(qp.inequalityMatrix(pairRows,:),1);
    limits = [repmat([model.cfg.model.frontWheelSteeringAngleMaximum; ...
        max(abs([model.cfg.actuation.brakingRatioMinimum,model.cfg.actuation.brakingRatioMaximum]))], ...
        prediction.stageCount,1);zeros(prediction.stageCount,1)];
    arithmeticSupport = abs(pairNormal)*limits;
    assert(all(labels(pairRows)=="combinedTireForce") && sum(pairBound)<-arithmeticSupport);
    contradiction = struct('rows',pairRows,'upperBounds',pairBound, ...
        'sumOfBounds',sum(pairBound),'maximumCoefficientCancellationError',arithmeticSupport, ...
        'cellEndTime',prediction.cells(2).time(end));

    a = prediction.continuousA(:,:,1);
    assert(max(abs(prediction.continuousA-a),[],'all')==0,'Exact LTI audit requires constant A.');
    rate = prediction.modelErrorRateBound(:,1);
    force = modifiedFialaTire.frictionCirclePolygonRows( ...
        prediction.scheduleCurvature(1),prediction.scheduleSpeedProfile(1), ...
        prediction.scheduleBrakingRatio(1),model.cfg,prediction.tireModels{1});
    directions = force.state([4,16],:);
    % Cell endpoints give a compact, reproducible comparison figure.
    times = [0,reshape(arrayfun(@(cell)cell.time(end),prediction.cells),1,[])];
    exactSupport = zeros(2,numel(times));
    exactBoxSupport = zeros(2,numel(times));
    sweptSupport = zeros(2,numel(times));
    endpointSupport = zeros(2,numel(times));
    for index = 2:numel(times)
        time = times(index);
        box = integral(@(s)abs(expm(a*s))*rate,0,time, ...
            'ArrayValued',true,'AbsTol',1e-10,'RelTol',1e-9);
        exactSupport(:,index) = integral(@(s)abs(directions*expm(a*s))*rate,0,time, ...
            'ArrayValued',true,'AbsTol',1e-10,'RelTol',1e-9);
        exactBoxSupport(:,index) = abs(directions)*box;
        sweptSupport(:,index) = abs(directions)*prediction.cells(index-1).radius(:,end);
        endpointSupport(:,index) = abs(directions)*prediction.cells(index-1).endRadius;
    end
    polygonLimit = cos(pi/model.cfg.model.frictionPolygonSides);
    rearThreshold = fzero(@(time)integral(@(s)abs(directions(2,:)*expm(a*s))*rate, ...
        0,time,'ArrayValued',true,'AbsTol',1e-10,'RelTol',1e-9)-polygonLimit,[0.1,0.4]);
    adverseRate = zeros(6,1);
    adverseRate(5) = -rate(5);
    adverseRate(6) = rate(6);
    adverseFlow = expm(0.4*[a,adverseRate;zeros(1,7)]);
    adverseRearForce = directions(2,:)*adverseFlow(1:6,7);
    assert(adverseRearForce>polygonLimit);
    uncertainty = struct('time',times,'exactDirectionalSupport',exactSupport, ...
        'exactMarginalBoxSupport',exactBoxSupport,'implementedSweptSupport',sweptSupport, ...
        'implementedEndpointSupport',endpointSupport,'polygonLimit',polygonLimit, ...
        'rearExactSupportCrossingTime',rearThreshold,'generator',a,'rate',rate, ...
        'constantAdverseRate',adverseRate,'constantAdverseRearForceAtPoint4Seconds',adverseRearForce);

    zeroCfg = trial.controllerConfiguration;
    zeroCfg.model.plantModelResidualRateBound = zeros(6,1);
    [zeroQp,~,~] = localRebuild(trial.failureContext,zeroCfg);
    zeroNoTargetFlag = localFeasible(zeroQp,true(size(zeroQp.inequalityBound)),opts);
    targetZeroCfg = targetTrial.controllerConfiguration;
    targetZeroCfg.model.plantModelResidualRateBound = zeros(6,1);
    [targetQp,targetPredictionData,targetModel] = localRebuild(targetTrial.failureContext,targetZeroCfg);
    targetLabels = localLabels(targetQp);
    core = ~startsWith(targetLabels,"collision:") & targetLabels~="terminalExit";
    selections = {core,core|startsWith(targetLabels,"collision:"),core|targetLabels=="terminalExit",true(size(core))};
    subsetNames = ["core","coreAndCollision","coreAndExit","all"];
    subsetFlags = zeros(4,1);
    for index = 1:4
        subsetFlags(index) = localFeasible(targetQp,selections{index},opts);
    end
    frame = targetQp.geometry.frames(end);
    world = [frame.tangent,frame.lateral];
    map = [world*targetPredictionData.egoStateMatrix(1:2,:,end),zeros(2,targetPredictionData.stageCount)];
    offset = frame.origin+world*targetPredictionData.egoStateOffset(1:2,end);
    positionLimits = zeros(2,2);
    for axis = 1:2
        for side = 1:2
            sign = 3-2*side;
            [~,value,flag] = linprog(sign*map(axis,:).',targetQp.inequalityMatrix(core,:), ...
                targetQp.barrier.baseBound(core),[],[],[],[],opts);
            assert(flag==1);
            positionLimits(axis,side) = sign*value+offset(axis);
        end
    end
    [targetEnd,~] = targetPrediction.finiteFlow(targetModel.encounters(1), ...
        targetModel.horizonSteps*targetModel.sampleTime);
    distanceUpper = norm(max(abs(positionLimits-targetEnd(1:2)),[],2));
    terminal = struct('zeroResidualNoTargetExitFlag',zeroNoTargetFlag, ...
        'subsetNames',subsetNames,'subsetExitFlags',subsetFlags,'positionBounds',positionLimits, ...
        'targetPosition',targetEnd(1:2),'distanceUpperBound',distanceUpper, ...
        'requiredPerceptionRadius',targetModel.perceptionRange);
    assert(zeroNoTargetFlag==1 && isequal(subsetFlags,[1;1;-2;-2]) && distanceUpper<targetModel.perceptionRange);

    context = trial.failureContext;
    [command,~,problem,certificate] = collisionAvoidanceController(context.controllerState,[], ...
        context.controllerRoadGeometry,zeroCfg,[]);
    ego = localNextEgo(certificate,problem,command,1);
    [~,~,sameRoadProblem] = collisionAvoidanceController(ego,[],context.controllerRoadGeometry,zeroCfg,certificate);
    road = fitPerceivedRoadBoundaries(trial.scenario.centerline,[ego.position;ego.yaw], ...
        PerceptionRange=30,RightOffset=6,LeftOffset=8,ShoulderWidth=2.6);
    changedRoadIdentifier = "";
    try
        collisionAvoidanceController(ego,[],road.roadGeometry,zeroCfg,certificate);
    catch exception
        changedRoadIdentifier = string(exception.identifier);
    end
    assert(changedRoadIdentifier=="collisionAvoidanceController:changedExecutionContract");
    roadUpdate = struct('sameRoadSource',sameRoadProblem.metadata.certificateSource, ...
        'refittedRoadError',changedRoadIdentifier,'oldOrigin',context.controllerRoadGeometry.boundaries(1).origin, ...
        'newOrigin',road.roadGeometry.boundaries(1).origin);
    for stage = 1:model.horizonSteps
        ego = localNextEgo(certificate,problem,command,stage);
        [command,~,problem,certificate] = collisionAvoidanceController(ego,[], ...
            context.controllerRoadGeometry,zeroCfg,certificate);
        if isempty(command),break;end
    end
    countdown = struct('complete',certificate.encounterComplete,'emptyCommand',isempty(command), ...
        'time',certificate.stateTime,'targetDistance',norm(ego.position-[100-10*certificate.stateTime;0.8]));
    assert(countdown.complete && countdown.emptyCommand && countdown.targetDistance>30);
    analysis = struct('scope',"Offline causal diagnosis; diagnostic relaxations do not validate physical safety", ...
        'matrixReproductionError',matrixError,'boundReproductionError',boundError, ...
        'families',families,'contradiction',contradiction,'uncertainty',uncertainty, ...
        'terminal',terminal,'roadUpdate',roadUpdate,'countdown',countdown);
    save(fullfile(outputDirectory,'analysis.mat'),'analysis','-v7.3');
    summary = rmfield(analysis,'uncertainty');
    summary.uncertainty = struct('polygonLimit',polygonLimit,'rearExactCrossingTime',rearThreshold, ...
        'exactAtDeadline',exactSupport(:,end),'boxAtDeadline',exactBoxSupport(:,end), ...
        'sweptAtDeadline',sweptSupport(:,end),'endpointAtDeadline',endpointSupport(:,end), ...
        'constantAdverseRate',adverseRate,'constantAdverseRearForceAtPoint4Seconds',adverseRearForce);
    file = fopen(fullfile(outputDirectory,'summary.json'),'w');
    assert(file>=0);
    cleanup = onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
    localPlot(uncertainty,outputDirectory);
    disp(summary);
end

function labels = localLabels(qp)
    labels = [qp.geometry.label;repmat("inputAndSlack", ...
        size(qp.inequalityMatrix,1)-numel(qp.geometry.label),1)];
    labels(qp.barrier.completionRows) = "terminalExit";
end

function flag = localFeasible(qp,mask,opts)
    [~,~,flag] = linprog(zeros(size(qp.inequalityMatrix,2),1), ...
        qp.inequalityMatrix(mask,:),qp.barrier.baseBound(mask),[],[],[],[],opts);
end

function ego = localNextEgo(certificate,problem,command,stage)
    state = certificate.predictedState(:,2);
    [position,heading] = laneGeometry.fromFrenet(state,problem.model.lane);
    time = stage*problem.model.sampleTime;
    ego = struct('position',position,'yaw',heading,'speed',state(4),'lateralVelocity',state(5), ...
        'yawRate',state(6),'stateTime',time,'heldActuatorInput',command.actuatorInput, ...
        'perception',struct('time',time,'range',problem.model.perceptionRange,'completeWithinRange',true));
end

function localPlot(data,directory)
    figureHandle = figure('Visible','off','Position',[100,100,1000,650],'Color','w');
    cleanup = onCleanup(@()close(figureHandle));
    tiledlayout(2,1,'TileSpacing','compact');
    axleNames = ["Front axle","Rear axle"];
    for axle = 1:2
        nexttile;
        plot(data.time,data.implementedSweptSupport(axle,:),'LineWidth',1.7);
        hold on;
        plot(data.time,data.exactMarginalBoxSupport(axle,:),'LineWidth',1.7);
        plot(data.time,data.exactDirectionalSupport(axle,:),'LineWidth',1.7);
        yline(data.polygonLimit,'k--','LineWidth',1.4);
        title(axleNames(axle));
        ylabel('Normalized force uncertainty');
        grid on;
        xlim([0,data.time(end)]);
    end
    xlabel('Prediction time (s)');
    legend('Implemented swept bound','Exact marginal box support', ...
        'Exact directional support','Polygon limit','Location','eastoutside');
    exportgraphics(figureHandle,fullfile(directory,'force-uncertainty.pdf'),'ContentType','vector');
    exportgraphics(figureHandle,fullfile(directory,'force-uncertainty.png'),'Resolution',160);
end

function [qp,prediction,model,ego] = localRebuild(context,cfg)
    cfg = collisionAvoidanceControllerConfig(cfg);
    [ego,lane,road,observations] = readPlanningInputs(context.controllerState,context.targetEstimate,context.controllerRoadGeometry,cfg);
    projection = laneGeometry.project(ego.position,lane);
    heading = atan2(sin(ego.yaw-projection.heading),cos(ego.yaw-projection.heading));
    [radius,valid] = stateUncertainty.toFrenet(ego.modelState,ego.stateErrorBound,lane);
    assert(valid || ~any(ego.stateErrorBound));
    prior = zeros(2,1);
    if ~isempty(ego.heldActuatorInput), prior = ego.heldActuatorInput; end
    model = struct('cfg',cfg,'lane',lane,'road',road,'stateTime',ego.stateTime, ...
        'sampleTime',cfg.controller.sampleTime,'horizonSteps',cfg.controller.horizonSteps, ...
        'referenceSpeed',cfg.referenceSpeed,'initialEgoState',[projection.station;projection.lateralPosition;heading;ego.modelState(4:6)], ...
        'initialFrenetErrorBound',radius,'longitudinalAccelerationBias',ego.longitudinalAccelerationBias, ...
        'previousInput',prior,'requiredMargin',0,'perceptionRange',ego.perceptionRange);
    encounters = struct('key',{},'contract',{},'center',{},'radius',{},'time',{},'halfLength',{},'halfWidth',{},'nominalCenter',{});
    for index=1:numel(observations)
        encounters(end+1) = targetPrediction.admit(observations(index),model.stateTime,lane,cfg); %#ok<AGROW>
    end
    model.encounters = encounters;
    model.exitMargin = Inf;
    model.exitSteps = repmat(model.horizonSteps,numel(encounters),1);
    prediction = ltvBicycleModel.finitePredict(model,[]);
    input = reshape(prediction.referencePlan,2,[]);
    lower = [-cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMinimum];
    upper = [cfg.model.frontWheelSteeringAngleMaximum;cfg.actuation.brakingRatioMaximum];
    change = model.sampleTime*[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum];
    for stage = 1:size(input,2)
        input(:,stage) = min(max(input(:,stage),max(lower,prior-change)),min(upper,prior+change));
        prior = input(:,stage);
    end
    model.anchorPlan = input(:);
    qp = formulateAvoidanceProblem(model,prediction,model.anchorPlan);
end
