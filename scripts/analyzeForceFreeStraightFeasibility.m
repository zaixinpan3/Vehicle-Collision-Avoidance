function result = analyzeForceFreeStraightFeasibility(inputDirectory, outputDirectory)
%analyzeForceFreeStraightFeasibility Offline diagnosis of saved version-14 failures.
% Altered residuals and horizons are counterfactual diagnostics, not controller
% configurations or safety certificates. No physical vehicle is advanced.
    arguments
        inputDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    acquisition = load(fullfile(inputDirectory,'acquisition-after.mat'),'acquisitionAfter');
    original = load(fullfile(inputDirectory,'original-functional.mat'),'unboundedRuntime');
    baseline = load(fullfile(inputDirectory,'row-removal-comparison.mat'),'problem','certificate');
    trial = acquisition.acquisitionAfter;
    context = trial.failureContext;
    cfg = trial.controllerConfiguration;
    [qp,prediction,model] = localRebuild(context,cfg);
    [core,collision,completion] = localMasks(qp);
    result = struct('sourceCertificateVersion',14,'configuration',cfg);
    result.baseline = localSubsets(qp,core,collision,completion);
    result.collisionRoadPair = localCollisionRoadPair(qp);
    result.collisionRelaxation = localRelax(qp,core,collision,prediction,model.sampleTime);
    result.exitRelaxation = localRelax(qp,core,completion,prediction,model.sampleTime);
    result.endpointErrorBound = prediction.egoStateErrorBound(:,end);
    result.exactDirectionalEndpointError = localDirectionalRadius(prediction,model);
    result.cellCount = numel(prediction.cells);
    result.hardRowCount = size(qp.inequalityMatrix,1);
    result.collisionNormal = unique(round(cat(2,qp.geometry.normals{:}).',10),'rows');
    result.lateralNormalFirstTime = NaN;
    for k = 1:numel(qp.geometry.normals)
        if abs(qp.geometry.normals{k}(2))>0.5
            result.lateralNormalFirstTime = prediction.cells(k).start;
            break;
        end
    end
    % Keep the saved scenario fixed while changing only diagnostic residual size.
    scales = [0,0.1,0.25,0.5,0.75,1];
    residualStudy = repmat(struct(),numel(scales),1);
    for k = 1:numel(scales)
        altered = cfg;
        altered.model.plantModelResidualRateBound = scales(k)*cfg.model.plantModelResidualRateBound;
        [q,p,m] = localRebuild(context,altered);
        [c,h,e] = localMasks(q);
        residualStudy(k).scale = scales(k);
        residualStudy(k).subsets = localSubsets(q,c,h,e);
        residualStudy(k).endpointBox = localEndpointBox(q,p,m,c);
    end
    result.residualStudy = residualStudy;
    % Longer windows also extend the exact constant-velocity target contract.
    % Failed road coverage is recorded before extending the same straight curbs.
    % This is a stated counterfactual; the actual saved contract ends at 1.6 s.
    horizons = [12,16,18,20,24,30,32];
    horizonStudy = repmat(struct(),2*numel(horizons),1);
    index = 0;
    for scale = [0,1]
        for steps = horizons
            index = index+1;
            altered = cfg; altered.controller.horizonSteps = steps;
            altered.model.plantModelResidualRateBound = scale*cfg.model.plantModelResidualRateBound;
            coverageFailure = "";
            try
                [q,p,m] = localRebuild(context,altered);
            catch exception
                if string(exception.identifier)~="collisionAvoidanceController:roadBoundaryCoverageGap"
                    rethrow(exception);
                end
                coverageFailure = string(exception.identifier);
                extended = context;
                for boundaryIndex=1:numel(extended.controllerRoadGeometry.boundaries)
                    extended.controllerRoadGeometry.boundaries(boundaryIndex).parameterRange=[-200;200];
                end
                [q,p,m] = localRebuild(extended,altered);
            end
            [c,h,e] = localMasks(q);
            horizonStudy(index).originalCoverageFailure = coverageFailure;
            horizonStudy(index).residualScale = scale;
            horizonStudy(index).duration = steps*cfg.controller.sampleTime;
            horizonStudy(index).subsets = localSubsets(q,c,h,e);
            horizonStudy(index).endpointErrorBound = p.egoStateErrorBound(:,end);
            horizonStudy(index).headingPair = localHeadingPair(q,p);
            if scale==1
                horizonStudy(index).exactDirectionalEndpointError=localDirectionalRadius(p,m);
            else
                horizonStudy(index).exactDirectionalEndpointError=zeros(0,1);
            end
            horizonStudy(index).maximumSweptHeadingRadius = max(arrayfun(@(tube)max(tube.radius(3,:)),p.cells));
        end
    end
    result.horizonStudy = horizonStudy;
    result.anchorStudy = repmat(struct(),2,1);
    anchorSteps=[18,32];
    for anchorIndex=1:2
        altered=cfg;altered.model.plantModelResidualRateBound=zeros(6,1);
        altered.controller.horizonSteps=anchorSteps(anchorIndex);
        extended=context;
        for boundaryIndex=1:numel(extended.controllerRoadGeometry.boundaries)
            extended.controllerRoadGeometry.boundaries(boundaryIndex).parameterRange=[-200;200];
        end
        [q,p,m]=localRebuild(extended,altered);
        [c,h,e]=localMasks(q);
        result.anchorStudy(anchorIndex).duration=m.sampleTime*p.stageCount;
        result.anchorStudy(anchorIndex).original=localSubsets(q,c,h,e);
        [alternateMatrix,alternateBound]=localLateralTailRows(m,p,q.geometry.frames);
        opts=optimoptions('linprog','Display','none');
        [plan,~,flag]=linprog(zeros(size(q.inequalityMatrix,2),1), ...
            [q.inequalityMatrix(c|e,:);alternateMatrix], ...
            [q.barrier.baseBound(c|e);alternateBound],[],[],[],[],opts);
        result.anchorStudy(anchorIndex).alternateFullExitFlag=flag;
        if flag==1
            result.anchorStudy(anchorIndex).inputPlan=reshape(plan(1:p.planCount),2,[]);
            result.anchorStudy(anchorIndex).originalViolation=max(q.inequalityMatrix*plan-q.barrier.baseBound);
            result.anchorStudy(anchorIndex).maximumViolation=max( ...
                [q.inequalityMatrix(c|e,:);alternateMatrix]*plan-[q.barrier.baseBound(c|e);alternateBound]);
        else
            result.anchorStudy(anchorIndex).maximumViolation=NaN;
            result.anchorStudy(anchorIndex).inputPlan=[];
            result.anchorStudy(anchorIndex).originalViolation=NaN;
        end
    end

    % Compare the actual next parsed road with the first admitted identity.
    next = original.unboundedRuntime.failureContext;
    [nextEgo,nextLane,nextRoad] = readPlanningInputs(next.controllerState, ...
        next.targetEstimate,next.controllerRoadGeometry,cfg);
    first = baseline.certificate.identity;
    result.roadIdentity = struct('laneEqual',isequaln(first.lane,nextLane), ...
        'roadEqual',isequaln(first.road,nextRoad), ...
        'accelerationBiasEqual',isequaln(first.accelerationBias,nextEgo.longitudinalAccelerationBias));
    result.roadIdentity.changedFields = string(fieldnames(nextRoad));
    keep = false(size(result.roadIdentity.changedFields));
    for k=1:numel(keep)
        key=result.roadIdentity.changedFields(k);
        keep(k)=~isequaln(first.road.(key),nextRoad.(key));
    end
    result.roadIdentity.changedFields=result.roadIdentity.changedFields(keep);
    result.roadIdentity.firstBoundaries=first.road.boundaries;
    result.roadIdentity.nextBoundaries=nextRoad.boundaries;
    result.roadIdentity.maximumCommonWorldGraphDifference=0;
    for k=1:numel(nextRoad.boundaries)
        old=first.road.boundaries(k);new=nextRoad.boundaries(k);
        common=[max(old.origin(1)+old.parameterRange(1),new.origin(1)+new.parameterRange(1)); ...
            min(old.origin(1)+old.parameterRange(2),new.origin(1)+new.parameterRange(2))];
        oldWorld=[old.coefficients(1);old.coefficients(2)-2*old.coefficients(1)*old.origin(1); ...
            polyval(old.coefficients,-old.origin(1))+old.origin(2)];
        newWorld=[new.coefficients(1);new.coefficients(2)-2*new.coefficients(1)*new.origin(1); ...
            polyval(new.coefficients,-new.origin(1))+new.origin(2)];
        difference=oldWorld-newWorld;
        points=common;
        if difference(1)~=0
            vertex=-difference(2)/(2*difference(1));
            if vertex>=common(1) && vertex<=common(2),points=[points;vertex];end %#ok<AGROW>
        end
        result.roadIdentity.maximumCommonWorldGraphDifference=max( ...
            result.roadIdentity.maximumCommonWorldGraphDifference,max(abs(polyval(difference,points))));
    end
    assert(result.roadIdentity.maximumCommonWorldGraphDifference<1e-10);
    assert(result.baseline.core==1 && result.baseline.full==-2);
    assert(result.anchorStudy(2).alternateFullExitFlag==1);
    assert(result.horizonStudy(10).headingPair.summedBound<0);
    assert(result.horizonStudy(10).headingPair.maximumSummedCoefficient==0);

    save(fullfile(outputDirectory,'diagnosis.mat'),'result','qp','prediction','model','-v7.3');
    file = fopen(fullfile(outputDirectory,'summary.json'),'w'); assert(file>=0);
    cleanup = onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(result,PrettyPrint=true));
    disp(result.baseline); disp(result.collisionRelaxation); disp(result.exitRelaxation);
end

function result = localCollisionRoadPair(qp)
% Search the final point for a collision/road pair with exactly opposing rows.
    count=size(qp.geometry.local(end).nodeLimits,1);
    first=size(qp.geometry.matrix,1)-count;
    result=struct('rows',[],'summedBound',inf,'maximumSummedCoefficient',NaN,'labels',strings(0,1));
    for i=1:count
        if ~startsWith(qp.geometry.label(first+i),"collision:"),continue;end
        for j=1:count
            if ~startsWith(qp.geometry.label(first+j),"road:"),continue;end
            rows=first+[i;j];
            coefficient=max(abs(sum(qp.inequalityMatrix(rows,:),1)));
            value=sum(qp.barrier.baseBound(rows));
            if coefficient==0 && value<result.summedBound
                result=struct('rows',rows,'summedBound',value, ...
                    'maximumSummedCoefficient',coefficient,'labels',qp.geometry.label(rows));
            end
        end
    end
    assert(result.summedBound<0 && result.maximumSummedCoefficient==0);
end

function result = localHeadingPair(qp,prediction)
% Opposing +/- heading rows supply a direct contradiction when their sum is negative.
    first=1;minimum=inf;pair=[];
    for k=1:numel(prediction.cells)
        local=qp.geometry.local(k);rowCount=size(local.nodeLimits,1);
        for point=1:size(local.nodeLimits,2)
            rows=first+(point-1)*rowCount+[2;8];
            value=sum(qp.barrier.baseBound(rows));
            if value<minimum,minimum=value;pair=rows;end
        end
        first=first+numel(local.nodeLimits);
    end
    result=struct('rows',pair,'summedBound',minimum, ...
        'maximumSummedCoefficient',max(abs(sum(qp.inequalityMatrix(pair,:),1))));
end

function [matrix,bound] = localLateralTailRows(model,prediction,frames)
% Diagnostic separator probe: preserve original normals before 1.3 s; retain
% a probe at lateral -6 m thereafter (normals may be diagonal). Core and terminal rows are untouched.
    cfg=model.cfg;matrices=cell(numel(prediction.cells),1);bounds=matrices;
    for k=1:numel(prediction.cells)
        tube=prediction.cells(k);frame=frames(k);
        nominal=reshape(pagemtimes(tube.map,model.anchorPlan),6,[])+tube.offset;
        if tube.start>=1.3
            nominal(2,:)=-6;
        end
        encounter=model.encounters(1);
        [center,radius]=targetPrediction.finiteFlow(encounter,tube.start);
        target=struct('center',center,'radius',radius,'contract',encounter.contract, ...
            'halfLength',encounter.halfLength,'halfWidth',encounter.halfWidth);
        data=struct('frame',[frame.origin;frame.tangent;frame.lateral;frame.heading; ...
            frame.positionErrorBound;frame.headingErrorBound;frame.stationLower;frame.stationUpper], ...
            'nominal',nominal,'targets',target,'boundaries',model.road.boundaries([]), ...
            'settings',[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.model.headingDomainRadius; ...
                cfg.model.lateralDomainRadius;cfg.collision.clearanceMargin], ...
            'duration',tube.duration,'degree',cfg.encounter.taylorOrder+1);
        rows=avoidanceSafetyGeometry('cellRows',data);
        % Project the newly chosen geometric support using the original maps.
        values=zeros(size(rows.state,1)*size(tube.offset,2),prediction.planCount);
        limits=zeros(size(values,1),1);
        for point=1:size(tube.offset,2)
            selected=(point-1)*size(rows.state,1)+(1:size(rows.state,1));
            values(selected,:)=rows.state*tube.map(:,:,point);
            limits(selected)=rows.bound(:,point)-rows.state*tube.offset(:,point) ...
                -abs(rows.state)*tube.radius(:,point)-2*cfg.encounter.numericalMargin;
        end
        matrices{k}=[values,zeros(size(values,1),prediction.stageCount)];bounds{k}=limits;
    end
    matrix=vertcat(matrices{:});bound=vertcat(bounds{:});
end

function [core,collision,completion] = localMasks(qp)
    count = size(qp.inequalityMatrix,1);
    collision = false(count,1);
    collision(1:numel(qp.geometry.label)) = startsWith(qp.geometry.label,'collision:');
    completion = false(count,1); completion(qp.barrier.completionRows) = true;
    core = ~collision & ~completion;
end

function result = localSubsets(qp,core,collision,completion)
    masks = {core,core|collision,core|completion,true(size(core))};
    flags = zeros(1,4);
    opts = optimoptions('linprog','Display','none');
    for k=1:4
        [~,~,flags(k)] = linprog(zeros(size(qp.inequalityMatrix,2),1), ...
            qp.inequalityMatrix(masks{k},:),qp.barrier.baseBound(masks{k}),[],[],[],[],opts);
    end
    result = struct('core',flags(1),'coreAndCollision',flags(2),'coreAndExit',flags(3),'full',flags(4));
end

function result = localRelax(qp,core,selected,prediction,sampleTime)
% Minimum uniform relaxation of only the selected physical-distance rows.
    rows = find(core|selected);
    matrix = [qp.inequalityMatrix(rows,:),-double(selected(rows))];
    objective = [zeros(size(matrix,2)-1,1);1];
    opts = optimoptions('linprog','Display','none');
    [decision,value,flag,~,lambda] = linprog(objective,matrix,qp.barrier.baseBound(rows), ...
        [],[],[-inf(size(matrix,2)-1,1);0],[],opts);
    assert(flag==1);
    important = find(lambda.ineqlin>1e-5);
    [~,order] = sort(lambda.ineqlin(important),'descend'); important=important(order);
    important = important(1:min(16,numel(important)));
    allLabels = repmat("inputOrSlack",size(qp.inequalityMatrix,1),1);
    allLabels(1:numel(qp.geometry.label))=qp.geometry.label;
    allLabels(qp.barrier.completionRows)="terminalExit";
    allTimes = nan(size(allLabels)); first=1;
    for k=1:numel(prediction.cells)
        block=qp.geometry.local(k);
        times=repelem(prediction.cells(k).time(:),size(block.nodeLimits,1));
        allTimes(first:first+numel(times)-1)=times;first=first+numel(times);
    end
    allTimes(qp.barrier.completionRows)=prediction.stageCount*sampleTime;
    result = struct('minimumRelaxationMeters',value,'exitFlag',flag, ...
        'maxRelaxedViolation',max(matrix*decision-qp.barrier.baseBound(rows)), ...
        'dualRows',rows(important),'dualWeights',lambda.ineqlin(important), ...
        'dualLabels',allLabels(rows(important)),'dualTimes',allTimes(rows(important)));
end

function result = localEndpointBox(qp,prediction,model,core)
    limits = nan(2,2); opts=optimoptions('linprog','Display','none');
    map=[prediction.egoStateMatrix(1:2,:,end),zeros(2,prediction.stageCount)];
    offset=prediction.egoStateOffset(1:2,end);
    for axis=1:2
        for side=1:2
            signValue=3-2*side;
            [~,value,flag]=linprog(signValue*map(axis,:).',qp.inequalityMatrix(core,:), ...
                qp.barrier.baseBound(core),[],[],[],[],opts);
            if flag==1,limits(axis,side)=signValue*value+offset(axis);end
        end
    end
    frame=qp.geometry.frames(end);
    target=targetPrediction.finiteFlow(model.encounters(1),prediction.stageCount*model.sampleTime);
    corners=[limits(1,[1,1,2,2]);limits(2,[1,2,1,2])];
    world=frame.origin+[frame.tangent,frame.lateral]*corners;
    result=struct('frenetLimits',limits,'targetCenter',target(1:2), ...
        'centerDistanceUpperBound',max(vecnorm(world-target(1:2))));
end

function radius = localDirectionalRadius(prediction,model)
% Numerically integrate exact directional LTV endpoint support, no reboxing.
% This diagnostic is not a certified replacement for the swept enclosure.
    transition=eye(6);radius=zeros(6,1);
    for stage=prediction.stageCount:-1:1
        a=prediction.continuousA(:,:,stage);rate=prediction.modelErrorRateBound(:,stage);
        radius=radius+integral(@(s)abs(transition*expm(a*s))*rate,0,model.sampleTime, ...
            'ArrayValued',true,'AbsTol',1e-11,'RelTol',1e-9);
        transition=transition*expm(a*model.sampleTime);
    end
    radius=radius+abs(transition)*model.initialFrenetErrorBound;
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
