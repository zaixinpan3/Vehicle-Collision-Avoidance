function result = evaluateContinuousSeparation(inputDirectory,diagnosisDirectory,outputDirectory)
%evaluateContinuousSeparation Try continuous normals on the diagnosed encounter.
% The saved feasible affine plan initializes geometry; it is not a global
% first-admission oracle. Road coverage is explicitly supplied on [-200,200] m.
    arguments
        inputDirectory (1,1) string
        diagnosisDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    a=load(fullfile(inputDirectory,'acquisition-after.mat'),'acquisitionAfter');
    d=load(fullfile(diagnosisDirectory,'diagnosis.mat'),'result');
    cfg=a.acquisitionAfter.controllerConfiguration;
    cfg.model.plantModelResidualRateBound=zeros(6,1);
    cfg.controller.horizonSteps=32;
    context=a.acquisitionAfter.failureContext;
    for k=1:numel(context.controllerRoadGeometry.boundaries)
        context.controllerRoadGeometry.boundaries(k).parameterRange=[-200;200];
    end
    [qp,prediction,model]=localRebuild(context,cfg);cfg=model.cfg;
    [baseline,~]=solveHardCbfClf.solve(qp,cfg);
    plan=d.result.anchorStudy(2).inputPlan(:);
    result=struct('scope',"geometry trial on affine inclusion; nonlinear Fiala replay checked separately", ...
        'initialization',"saved feasible affine plan from prior zero-residual extended-road diagnostic", ...
        'baselineFeasible',baseline.feasible,'initialPlan',reshape(plan,2,[]),'iterations',[]);
    prediction.geometryFrames=qp.geometry.frames;
    stored=[];
    for iteration=1:3
        timer=tic;
        [normals,normalInfo]=avoidanceSafetyGeometry.optimizeNormals(model,prediction,plan);
        prediction.separationNormals=normals;
        % Preserve the original station charts while updating the support probe.
        model.anchorPlan=plan;prediction.geometryAnchor=plan;
        prediction.geometryNominal=cell(numel(prediction.cells),1);
        for k=1:numel(prediction.cells)
            tube=prediction.cells(k);
            prediction.geometryNominal{k}=reshape(pagemtimes(tube.map,plan),6,[])+tube.offset;
        end
        candidate=formulateAvoidanceProblem(model,prediction,plan);
        [solve,candidate]=solveHardCbfClf.solve(candidate,cfg);
        check=solveHardCbfClf.certify(candidate,prediction,model,solve.decision);
        result.iterations(iteration).seconds=toc(timer);
        result.iterations(iteration).accepted=solve.feasible && check.accepted;
        result.iterations(iteration).margin=check.margin;
        result.iterations(iteration).normalMargin=min(normalInfo.proposalMargins,[],'all');
        if solve.feasible && check.accepted
            plan=solve.decision(candidate.layout.planIndex);
            stored=struct('qp',candidate,'prediction',prediction,'model',model,'decision',solve.decision,'check',check);
        end
    end
    result.finalAccepted=~isempty(stored);
    result.finalPlan=reshape(plan,2,[]);
    result.fialaReplay=localFialaReplay(model,result.finalPlan);
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    save(fullfile(outputDirectory,'continuous-separation.mat'),'result','stored');
    file=fopen(fullfile(outputDirectory,'continuous-separation.json'),'w');assert(file>=0);
    cleanup=onCleanup(@()fclose(file));fprintf(file,'%s\n',jsonencode(result,PrettyPrint=true));
    disp(result.iterations);disp(result.fialaReplay);
end

function result=localFialaReplay(model,plan)
    cfg=model.cfg;state=model.initialEgoState;
    [position,heading]=laneGeometry.fromFrenet(state,model.lane);
    state=[position;heading;state(4:6)];
    minimum=inf;h=model.sampleTime/100;
    for k=1:size(plan,2)
        for j=1:100
            time=(k-1)*model.sampleTime+(j-1)*h;
            target=targetPrediction.finiteFlow(model.encounters(1),time);
            minimum=min(minimum,avoidanceSafetyGeometry.rectangleDistance(state(1:2),state(3),target(1:2),target(7), ...
                [cfg.vehicle.length/2;cfg.vehicle.width/2;model.encounters(1).halfLength;model.encounters(1).halfWidth])-cfg.collision.clearanceMargin);
            flow=@(x)ltvBicycleModel.fialaWorldDynamics(x,plan(:,k),cfg);
            k1=flow(state);k2=flow(state+h*k1/2);k3=flow(state+h*k2/2);k4=flow(state+h*k3);
            state=state+h*(k1+2*k2+2*k3+k4)/6;
        end
    end
    target=targetPrediction.finiteFlow(model.encounters(1),model.sampleTime*size(plan,2));
    result=struct('minimumSampledClearance',minimum,'finalState',state, ...
        'finalCenterDistance',norm(state(1:2)-target(1:2)), ...
        'scope',"deterministic nonlinear numerical replay, not a validated swept tube");
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
