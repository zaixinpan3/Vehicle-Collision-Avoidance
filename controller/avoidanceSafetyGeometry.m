function geometry = avoidanceSafetyGeometry(model, prediction)
% One separating normal per cell; every Bernstein point uses that normal.
% The cellRows entry exposes the shared numeric geometry kernel for codegen.
    if ischar(model) || isstring(model)
        assert(~isempty(prediction));
        if string(model)=="nominalCheck"
            geometry = localNominalCheck(prediction);
        elseif string(model)=="cellRows"
            geometry = localCellRows(prediction(1));
            geometry = repmat(geometry,numel(prediction),1);
            for index = 2:numel(prediction)
                geometry(index) = localCellRows(prediction(index));
            end
        else
            assert(string(model)=="projectRows");
            geometry = localProjectedRows(prediction(1));
            geometry = repmat(geometry,numel(prediction),1);
            for index = 2:numel(prediction)
                geometry(index) = localProjectedRows(prediction(index));
            end
        end
        return;
    end
    cfg = model.cfg;
    degree = cfg.encounter.taylorOrder+1;
    groups = cell(numel(prediction.cells), 1);
    frames = cell(numel(groups), 1);
    normalGroups = cell(numel(groups), 1);
    localGroups = cell(numel(groups),1);
    nominalGroups = cell(numel(groups),1);
    boundaryTemplate = struct("coefficients",zeros(1,3),"safeSideSign",0, ...
        "longitudinalDirection",zeros(2,1),"lateralDirection",zeros(2,1),"origin",zeros(2,1), ...
        "parameterRange",zeros(2,1),"normalDistanceErrorBound",0);
    boundaries = repmat(boundaryTemplate,numel(model.road.boundaries),1);
    boundaryLabels = strings(numel(boundaries),1);
    for index = 1:numel(boundaries)
        boundary = model.road.boundaries(index);
        boundaries(index) = struct("coefficients",boundary.coefficients(:).', ...
            "safeSideSign",boundary.safeSideSign,"longitudinalDirection",boundary.longitudinalDirection(:), ...
            "lateralDirection",boundary.lateralDirection(:),"origin",boundary.origin(:), ...
            "parameterRange",boundary.parameterRange(:),"normalDistanceErrorBound",boundary.normalDistanceErrorBound);
        boundaryLabels(index) = "road:"+boundary.boundaryId;
    end
    targetTemplate = struct("center",zeros(8,1),"radius",zeros(8,1), ...
        "contract",struct("jerkBound",zeros(2,1),"yawAccelerationBound",0), ...
        "nominalEnd",zeros(8,1),"useNominalEnd",false,"anticipate",false, ...
        "nextRadius",zeros(8,1),"halfLength",0,"halfWidth",0);
    nativeGeometry = exist("avoidanceCellRowsKernelMex","file")==3;
    cachedFrames = isfield(prediction,"geometryAnchor") ...
        && isequal(prediction.geometryAnchor,model.anchorPlan);
    if ~cachedFrames
        [computedFrames,computedNominal] = laneGeometry.sweptCellFrames(model,prediction.cells,model.anchorPlan);
    end
    finiteSensing = arrayfun(@targetPrediction.isFiniteSensing,model.encounters);
    targetLookahead = cell(numel(model.encounters),1);
    times = [prediction.cells.start];
    ends = times+[prediction.cells.duration];
    for targetIndex = 1:numel(model.encounters)
        encounter = model.encounters(targetIndex);
        if encounter.discharged || ~finiteSensing(targetIndex),continue;end
        [centers,jerk,yawAcceleration] = targetPrediction.nominalFlow(encounter,[times,ends]);
        centered = encounter;
        displacement = encounter.center-encounter.nominalCenter;
        displacement(7) = atan2(sin(displacement(7)),cos(displacement(7)));
        centered.radius = encounter.radius+abs(displacement);
        [~,nextRadius] = targetPrediction.finiteFlow(centered,model.sampleTime);
        targetLookahead{targetIndex} = struct("center",centers,"jerk",jerk, ...
            "yawAcceleration",yawAcceleration,"nextRadius",nextRadius);
    end
    nominalCells = cell(numel(groups),1);
    cellData = cell(numel(groups),1);
    activeTargets = cell(numel(groups),1);
    sourceLabels = cell(numel(groups),1);
    for cellIndex = 1:numel(groups)
        tube = prediction.cells(cellIndex);
        robust = tube.stage<=cfg.controller.certifiedSteps;
        if cachedFrames
            frame = prediction.geometryFrames(cellIndex);
            nominal = prediction.geometryNominal{cellIndex};
        else
            frame = computedFrames(cellIndex);
            nominal = computedNominal{cellIndex};
        end
        if frame.referenceHeadingErrorBound > 128*eps && ~isfield(model.lane, "referenceCurve")
            error("collisionAvoidanceController:unsupportedReferenceJump", ...
                "The finite CLF certificate requires a continuous reference chart over every cell.");
        end
        frames{cellIndex} = frame;
        active = [];
        if ~isempty(model.encounters)
            active = find(~[model.encounters.discharged] & tube.stage<=model.exitSteps(:).');
        end
        targets = repmat(targetTemplate,numel(active),1);
        targetLabels = strings(numel(active),1);
        for targetIndex = 1:numel(active)
            originalIndex = active(targetIndex);
            encounter = model.encounters(originalIndex);
            target = targetTemplate;
            target.halfLength = encounter.halfLength;target.halfWidth = encounter.halfWidth;
            target.contract.jerkBound = encounter.contract.jerkBound;
            target.contract.yawAccelerationBound = encounter.contract.yawAccelerationBound;
            if ~robust && finiteSensing(originalIndex)
                cached = targetLookahead{originalIndex};
                target.center = cached.center(:,cellIndex);
                target.contract.jerkBound = repmat(cached.jerk(cellIndex),2,1);
                target.contract.yawAccelerationBound = cached.yawAcceleration(cellIndex);
                target.useNominalEnd = true;
                target.nominalEnd = cached.center(:,numel(groups)+cellIndex);
                target.anticipate = any(encounter.radius);
                target.nextRadius = cached.nextRadius;
            else
                [target.center,target.radius] = targetPrediction.finiteFlow(encounter,tube.start);
            end
            targets(targetIndex) = target;
            targetLabels(targetIndex) = "collision:"+encounter.key;
        end
        data = struct("frame",[frame.origin;frame.tangent;frame.lateral;frame.heading; ...
            frame.positionErrorBound;frame.headingErrorBound;frame.stationLower;frame.stationUpper], ...
            "nominal",nominal,"targets",targets,"boundaries",boundaries, ...
            "settings",[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.model.headingDomainRadius; ...
                cfg.model.lateralDomainRadius;cfg.collision.clearanceMargin], ...
            "duration",tube.duration,"degree",degree,"robust",robust);
        nominalCells{cellIndex} = nominal;
        cellData{cellIndex} = data;
        activeTargets{cellIndex} = active;
        sourceLabels{cellIndex} = [targetLabels;boundaryLabels];
    end
    data = vertcat(cellData{:});
    if nativeGeometry
        allGeometricRows = avoidanceCellRowsKernelMex(data);
    else
        allGeometricRows = avoidanceSafetyGeometry('cellRows',data);
    end
    projectionData = cell(numel(groups),1);
    numericCfg = struct("model",struct("lateralDomainRadius",cfg.model.lateralDomainRadius, ...
        "headingDomainRadius",cfg.model.headingDomainRadius,"speedMinimum",cfg.model.speedMinimum, ...
        "speedMaximum",cfg.model.speedMaximum,"lateralVelocityMaximum",cfg.model.lateralVelocityMaximum, ...
        "yawRateMaximum",cfg.model.yawRateMaximum,"scheduleSpeedFloor",cfg.model.scheduleSpeedFloor, ...
        "slipAngleMaximum",repmat(cfg.model.slipAngleMaximum(:),2/numel(cfg.model.slipAngleMaximum),1)), ...
        "vehicle",struct("lf",cfg.vehicle.lf,"lr",cfg.vehicle.lr), ...
        "encounter",struct("corridorOverlap",cfg.encounter.corridorOverlap, ...
        "nominalLinearizationReserve",cfg.encounter.nominalLinearizationReserve));
    maneuver = find(["track","passLeft","passRight","yield"]==model.maneuver)-1;
    for cellIndex = 1:numel(groups)
        tube = prediction.cells(cellIndex);
        robust = tube.stage<=cfg.controller.certifiedSteps;
        stateRadius = tube.numericalRadius;
        if robust,stateRadius = tube.radius;end
        friction = struct("state",zeros(0,6),"input",zeros(0,2),"bound",zeros(0,1));
        if robust || string(cfg.model.linearizationPolicy)=="cruise"
            frictionArguments = {};
            if isfield(prediction,"tireModels") && ~isempty(prediction.tireModels{tube.stage})
                frictionArguments = prediction.tireModels(tube.stage);
            end
            force = modifiedFialaTire.frictionCirclePolygonRows(prediction.scheduleCurvature(tube.stage), ...
                prediction.scheduleSpeedProfile(tube.stage),prediction.scheduleBrakingRatio(tube.stage),cfg,frictionArguments{:});
            friction = struct("state",force.state,"input",force.input,"bound",force.bound);
        end
        frame = frames{cellIndex};
        futureNonlinear = ~robust && string(cfg.model.linearizationPolicy)~="cruise";
        domainError = zeros(6,2);initialError = zeros(6,2);
        if futureNonlinear
            domainError = prediction.domainErrorBound(:,tube.stage:tube.stage+1);
            initialError = prediction.initialErrorBound(:,tube.stage:tube.stage+1);
        end
        numericTube = struct("stage",tube.stage,"map",tube.map,"offset",tube.offset, ...
            "localStateMap",tube.localStateMap,"localInputMap",tube.localInputMap, ...
            "localOffset",tube.localOffset,"numericalRadius",tube.numericalRadius);
        projectionData{cellIndex} = struct("cfg",numericCfg,"tube",numericTube, ...
            "nominal",nominalCells{cellIndex},"stationRange",[frame.stationLower;frame.stationUpper], ...
            "stateRadius",stateRadius,"scheduleSpeed",prediction.scheduleSpeedProfile(tube.stage), ...
            "friction",friction,"maneuver",maneuver,"futureNonlinear",futureNonlinear, ...
            "geometricRows",allGeometricRows(cellIndex),"domainErrorBound",domainError, ...
            "initialErrorBound",initialError,"planCount",prediction.planCount);
        normals = zeros(2,numel(model.encounters));
        normals(:,activeTargets{cellIndex}) = allGeometricRows(cellIndex).normals;
        normalGroups{cellIndex} = normals;
    end
    data = vertcat(projectionData{:});
    if exist("avoidanceProjectedRowsKernelMex","file")==3
        projected = avoidanceProjectedRowsKernelMex(data);
    else
        projected = avoidanceSafetyGeometry('projectRows',data);
    end
    for cellIndex = 1:numel(groups)
        names = ["modelDomain";"tireSlip";"combinedTireForce";"maneuverCorridor";sourceLabels{cellIndex}];
        group = projected(cellIndex);
        local = group.local;
        nominalGroups{cellIndex} = struct("stage",local.stage, ...
            "state",local.nodeStateRows,"startState",local.nodeStartStateRows, ...
            "input",local.nodeInputRows,"limit",local.nodeLimits, ...
            "initialReserve",local.nodeInitialReserve,"slipRows",find(local.nodeLabels==2));
        local.nodeLabels = names(local.nodeLabels);
        localGroups{cellIndex} = local;
        group = rmfield(group,"local");group.label = names(group.label);
        groups{cellIndex} = group;
    end
    groups = vertcat(groups{:});
    geometry = struct("matrix", vertcat(groups.matrix), "physicalBound", vertcat(groups.physicalBound), ...
        "domainReserve",vertcat(groups.domainReserve), ...
        "initialReserve",vertcat(groups.initialReserve), ...
        "anticipationReserve",vertcat(groups.anticipationReserve), ...
        "safety", vertcat(groups.safety), "label", vertcat(groups.label), "stage", vertcat(groups.stage), ...
        "frames", vertcat(frames{:}), "normals", {normalGroups},"local",vertcat(localGroups{:}), ...
        "nominalCheckCells",vertcat(nominalGroups{:}));
end

function result = localNominalCheck(data)
% Shared independent nonlinear lookahead residuals, batched across cells.
    settings = data.settings;states = data.states;inputs = data.inputs;
    total = 0;
    for index = 1:numel(data.cells)
        if data.cells(index).stage>settings(1),total = total+2*size(data.cells(index).state,1);end
    end
    residuals = zeros(total,1);cursor = 0;
    result = struct("violation",0,"cellIndex",0,"rowIndex",0,"residuals",residuals);
    for index = 1:numel(data.cells)
        cellData = data.cells(index);stage = cellData.stage;
        if stage<=settings(1),continue;end
        rowCount = size(cellData.state,1);value = zeros(rowCount,2);
        for point = 1:2
            state = states(:,stage+point-1);input = inputs(:,stage);
            value(:,point) = cellData.state(:,:,point)*state ...
                +cellData.startState(:,:,point)*states(:,stage) ...
                +cellData.input(:,:,point)*input-cellData.limit(:,point);
            lateral = state(5)+[settings(3);-settings(4)]*state(6);
            slip = atan2(lateral,max(state(4),settings(2)))-[input(1);0];
            scheduled = lateral/max(states(4,stage),settings(2))-[input(1);0];
            limit = [settings(5:6);settings(5:6)];
            value(cellData.slipRows,point) = max([scheduled;-scheduled]-limit ...
                +cellData.initialReserve(cellData.slipRows,point),[slip;-slip]-limit)+settings(7);
        end
        residuals(cursor+(1:2*rowCount)) = value(:);cursor = cursor+2*rowCount;
        [peak,row] = max(value(:));
        if peak>result.violation
            result.violation = peak;result.cellIndex = index;result.rowIndex = mod(row-1,rowCount)+1;
        end
    end
    result.residuals = residuals;
end

function rows = localCellRows(data)
% Numeric obstacle/road support construction shared by MATLAB and native code.
    frame = data.frame;origin = frame(1:2);tangent = frame(3:4);lateral = frame(5:6);
    heading = frame(7);positionError = frame(8:9);headingError = frame(10);
    stationRange = frame(11:12).';
    settings = data.settings;halfLength = settings(1);halfWidth = settings(2);
    headingDomain = settings(3);lateralDomain = settings(4);clearanceMargin = settings(5);
    nominal = data.nominal;pointCount = size(nominal,2);
    targetCount = numel(data.targets);boundaryCount = numel(data.boundaries);
    maximumRows = 12*(targetCount+boundaryCount);
    state = zeros(maximumRows,6);bound = zeros(maximumRows,pointCount);
    anticipation = zeros(maximumRows,1);source = zeros(maximumRows,1);
    normals = zeros(2,targetCount);count = 0;
    for index = 1:targetCount
        target = data.targets(index);
        middle = targetPrediction.finiteFlow(target,data.duration/2);
        centerEgo = origin+[tangent,lateral]*mean(nominal(1:2,:),2);
        [~,normal] = rectangleConfigurationDistance(centerEgo,heading+mean(nominal(3,:)), ...
            middle(1:2),middle(7),[halfLength;halfWidth;target.halfLength;target.halfWidth]);
        normals(:,index) = normal;
        degree = data.degree;
        positionPolynomial = [target.center(1:2),target.center(3:4),target.center(5:6)/2,zeros(2,degree-2)];
        radiusPolynomial = [target.radius(1:2),target.radius(3:4),target.radius(5:6)/2, ...
            target.contract.jerkBound/6,zeros(2,degree-3)];
        transform = stateUncertainty.bernsteinTransform(degree,data.duration);
        if pointCount==1,transform = transform(1,:);end
        if ~data.robust,transform = transform([1,end],:);end
        targetPosition = positionPolynomial*transform.';
        targetRadius = radiusPolynomial*transform.';
        [endCenter,endRadius] = targetPrediction.finiteFlow(target,data.duration);
        if target.useNominalEnd
            endCenter = target.nominalEnd;
            targetPosition = [target.center(1:2),endCenter(1:2)];
            acceleration = max(norm(target.center(5:6)),norm(endCenter(5:6)));
            targetRadius = repmat(acceleration*data.duration^2/8,2,2);
        end
        yawCenter = (target.center(7)+endCenter(7))/2;
        yawRadius = abs(endCenter(7)-target.center(7))/2+endRadius(7);
        targetSupport = targetPrediction.rectangleSupport(target.halfLength,target.halfWidth,normal,yawCenter,yawRadius);
        allowance = 0;
        if target.anticipate
            uncertainSupport = targetPrediction.rectangleSupport(target.halfLength,target.halfWidth, ...
                normal,yawCenter,yawRadius+target.nextRadius(7));
            allowance = abs(normal).'*target.nextRadius(1:2)+max(0,uncertainSupport-targetSupport);
        end
        [egoSupport,headingSlope] = targetPrediction.rectangleSupportMajorant(normal,heading, ...
            halfLength,halfWidth,mean(nominal(3,:)),headingDomain+headingError);
        rowCount = numel(egoSupport);selected = count+(1:rowCount);
        state(selected,:) = repmat([-normal.'*[tangent,lateral],zeros(1,4)],rowCount,1);
        state(selected,3) = headingSlope;
        bound(selected,:) = normal.'*origin-normal.'*targetPosition-abs(normal).'*targetRadius ...
            -targetSupport-egoSupport-clearanceMargin-abs(normal).'*positionError-abs(headingSlope)*headingError;
        anticipation(selected) = allowance;source(selected) = index;count = count+rowCount;
    end
    for index = 1:boundaryCount
        boundary = data.boundaries(index);longitudinal = boundary.longitudinalDirection;
        stations = longitudinal.'*tangent*stationRange;
        extent = abs(longitudinal.'*lateral)*lateralDomain+hypot(halfLength,halfWidth)+abs(longitudinal).'*positionError;
        range = [min(stations)-extent,max(stations)+extent]+longitudinal.'*(origin-boundary.origin);
        if range(1)<boundary.parameterRange(1) || range(2)>boundary.parameterRange(2)
            error("collisionAvoidanceController:roadBoundaryCoverageGap", ...
                "Each boundary must cover the complete certified cell.");
        end
        polynomial = boundary.safeSideSign*boundary.coefficients;
        graphSupport = max(polyval(polynomial,range));
        if polynomial(1)<0
            stationary = min(max(-polynomial(2)/(2*polynomial(1)),range(1)),range(2));
            graphSupport = max(graphSupport,polyval(polynomial,stationary));
        end
        normal = boundary.safeSideSign*boundary.lateralDirection;
        slope = max(abs(2*boundary.coefficients(1)*range+boundary.coefficients(2)));
        clearance = (clearanceMargin+boundary.normalDistanceErrorBound)*hypot(1,slope);
        [egoSupport,headingSlope] = targetPrediction.rectangleSupportMajorant(normal,heading, ...
            halfLength,halfWidth,mean(nominal(3,:)),headingDomain+headingError);
        rowCount = numel(egoSupport);selected = count+(1:rowCount);
        state(selected,:) = repmat([-normal.'*[tangent,lateral],zeros(1,4)],rowCount,1);
        state(selected,3) = headingSlope;
        limit = normal.'*(origin-boundary.origin)-graphSupport-egoSupport-clearance ...
            -abs(normal).'*positionError-abs(headingSlope)*headingError;
        bound(selected,:) = repmat(limit,1,pointCount);
        source(selected) = targetCount+index;count = count+rowCount;
    end
    rows = struct("state",state(1:count,:),"bound",bound(1:count,:), ...
        "anticipation",anticipation(1:count),"source",source(1:count),"normals",normals);
end

function result = localProjectedRows(data)
% Apply the same cell rows to condensed and local coordinates in one kernel.
    cfg = data.cfg;tube = data.tube;nominal = data.nominal;
    frame = struct("stationLower",data.stationRange(1),"stationUpper",data.stationRange(2));
    stateRadius = data.stateRadius;pointCount = size(tube.offset,2);
    lower = [frame.stationLower; -cfg.model.lateralDomainRadius; -cfg.model.headingDomainRadius; ...
        cfg.model.speedMinimum; -cfg.model.lateralVelocityMaximum; -cfg.model.yawRateMaximum];
    upper = [frame.stationUpper; cfg.model.lateralDomainRadius; cfg.model.headingDomainRadius; ...
        cfg.model.speedMaximum; cfg.model.lateralVelocityMaximum; cfg.model.yawRateMaximum];
    stateRows = [eye(6); -eye(6)];
    inputRows = zeros(12, 2);
    limits = repmat([upper; -lower], 1, pointCount);
    safety = false(12, 1);
    labels = ones(12,1);
    speed = max(data.scheduleSpeed, cfg.model.scheduleSpeedFloor);
    slips = [0, 0, 0, 0, 1/speed, cfg.vehicle.lf/speed; ...
        0, 0, 0, 0, 1/speed, -cfg.vehicle.lr/speed];
    stateRows = [stateRows; slips; -slips];
    inputRows = [inputRows; -1, 0; 0, 0; 1, 0; 0, 0];
    slipLimit = cfg.model.slipAngleMaximum(:);
    if isscalar(slipLimit), slipLimit = [slipLimit; slipLimit]; end
    limits = [limits; repmat([slipLimit; slipLimit], 1, pointCount)];
    safety = [safety; false(4, 1)];
    labels = [labels; 2*ones(4,1)];
    if ~isempty(data.friction.bound)
        friction = data.friction;
        stateRows = [stateRows;friction.state];
        inputRows = [inputRows;friction.input];
        limits = [limits;repmat(friction.bound,1,pointCount)];
        safety = [safety;false(numel(friction.bound),1)];
        labels = [labels;3*ones(numel(friction.bound),1)];
    end
    % Future trajectory stages use the nonlinear Fiala force law itself:
    % |Fy| <= mu*Fz*sqrt(1-beta^2). An affine inner polygon there adds
    % artificial limits that are not the physical circle. Frozen-model
    % execution stages retain their requested-force polygon.
    corridor = [0, 1, 0, 0, 0, 0];
    switch data.maneuver
        case 0
            corridor = zeros(0, 6);
        case 1
            corridor = -corridor;
        case 3
            corridor = [corridor; -corridor];
    end
    stateRows = [stateRows; corridor];
    inputRows = [inputRows; zeros(size(corridor, 1), 2)];
    limits = [limits; cfg.encounter.corridorOverlap*ones(size(corridor, 1), pointCount)];
    safety = [safety; false(size(corridor, 1), 1)];
    labels = [labels; 4*ones(size(corridor,1),1)];
    geometricRows = data.geometricRows;
    rowCount = size(geometricRows.state,1);
    targetAnticipation = [zeros(size(safety));geometricRows.anticipation];
    stateRows = [stateRows;geometricRows.state];
    inputRows = [inputRows;zeros(rowCount,2)];
    limits = [limits;geometricRows.bound];
    safety = [safety;true(rowCount,1)];
    labels = [labels;4+geometricRows.source];
    pointStateRows = repmat(stateRows,1,1,pointCount);
    pointStartRows = zeros(size(pointStateRows));
    pointInputRows = repmat(inputRows,1,1,pointCount);
    if data.futureNonlinear
        speed = max(nominal(4,1),cfg.model.scheduleSpeedFloor);
        for point = 1:pointCount
            state = nominal(:,point);
            lateral = state(5)+[cfg.vehicle.lf;-cfg.vehicle.lr]*state(6);
            rows = [0,0,0,0,1/speed,cfg.vehicle.lf/speed; ...
                0,0,0,0,1/speed,-cfg.vehicle.lr/speed];
            % Linearize endpoint lateral velocity divided by the
            % interval-start speed, exactly as the executed model's
            % scheduled-slip admission. Its denominator derivative
            % belongs to the start state, including at the endpoint.
            pointStateRows(13:16,:,point) = [rows;-rows];
            if nominal(4,1)>cfg.model.scheduleSpeedFloor
                pointStartRows(13:16,4,point) = [-lateral;lateral]/speed^2;
            end
            pointInputRows(13:16,:,point) = [-1,0;0,0;1,0;0,0];
            constant = lateral/speed;
            if nominal(4,1)<=cfg.model.scheduleSpeedFloor,constant(:)=0;end
            limits(13:16,point) = [slipLimit-constant;slipLimit+constant];
        end
    end
    mapped = pagemtimes(pointStateRows, tube.map)+pagemtimes(pointStartRows,tube.map(:,:,1));
    for point = 1:pointCount
        mapped(:, 2*tube.stage-1:2*tube.stage, point) = ...
            mapped(:, 2*tube.stage-1:2*tube.stage, point)+pointInputRows(:,:,point);
    end
    uncertaintySupport = reshape(pagemtimes(abs(pointStateRows),reshape(stateRadius,6,1,pointCount)),[],pointCount);
    uncertaintySupport = uncertaintySupport+reshape(pagemtimes(abs(pointStartRows),stateRadius(:,1)),[],pointCount);
    domainReserve = zeros(size(uncertaintySupport));
    initialReserve = zeros(size(uncertaintySupport));
    if data.futureNonlinear
        % Prepare future state domains for consecutive uncertain steps.
        % Resetting this allowance at every future stage lets the plan
        % approach a boundary before rate-limited inputs can turn back.
        domainRows = labels==1 | labels==2;
        % Station limits delimit the local coordinate chart, not a
        % vehicle-state operating domain. Its nominal swept frame is
        % constructed separately from the endpoint trajectory.
        domainRows([1,7]) = false;
        executionSupport = reshape(pagemtimes(abs(pointStateRows), ...
            reshape(data.domainErrorBound,6,1,pointCount)),[],pointCount);
        executionSupport = executionSupport+reshape(pagemtimes(abs(pointStartRows),data.domainErrorBound(:,1)),[],pointCount);
        initialSupport = reshape(pagemtimes(abs(pointStateRows), ...
            reshape(data.initialErrorBound,6,1,pointCount)),[],pointCount);
        initialSupport = initialSupport+reshape(pagemtimes(abs(pointStartRows),data.initialErrorBound(:,1)),[],pointCount);
        initialReserve(domainRows,:) = initialSupport(domainRows,:);
        domainReserve(domainRows,:) = max(0,executionSupport(domainRows,:)-initialSupport(domainRows,:));
    end
    % The force polygon limits the nominal linearization's requested
    % force. Actual tire-force variation belongs to the declared plant
    % residual; road and collision rows retain full state tightening.
    forceRows = labels==3;
    uncertaintySupport(forceRows,:) = abs(stateRows(forceRows,:))*tube.numericalRadius;
    physical = limits-reshape(pagemtimes(pointStateRows,reshape(tube.offset,6,1,pointCount)),[],pointCount) ...
        -reshape(pagemtimes(pointStartRows,tube.offset(:,1)),[],pointCount)-uncertaintySupport;
    anticipationReserve = zeros(size(safety));
    if data.futureNonlinear
        support = reshape(pagemtimes(abs(pointStateRows), ...
            reshape(data.domainErrorBound,6,1,pointCount)),[],pointCount);
        anticipationReserve = (max(support,[],2)+cfg.encounter.nominalLinearizationReserve).*double(safety)+targetAnticipation;
    end
    localState = pagemtimes(pointStateRows,tube.localStateMap)+pointStartRows;
    localInput = pagemtimes(pointStateRows,tube.localInputMap)+pointInputRows;
    localBound = limits-reshape(pagemtimes(pointStateRows,reshape(tube.localOffset,6,1,pointCount)),[],pointCount)-uncertaintySupport;
    local = struct("stateMatrix",reshape(permute(localState,[1,3,2]),[],6), ...
        "inputMatrix",reshape(permute(localInput,[1,3,2]),[],2),"bound",localBound(:),"stage",tube.stage, ...
        "nodeStateRows",pointStateRows,"nodeInputRows",pointInputRows,"nodeLimits",limits-uncertaintySupport-initialReserve, ...
        "nodeStartStateRows",pointStartRows, ...
        "nodeInitialReserve",initialReserve, ...
        "nodeLabels",labels);
    result = struct("matrix",reshape(permute(mapped,[1,3,2]),[],data.planCount), ...
        "physicalBound",physical(:),"domainReserve",domainReserve(:),"initialReserve",initialReserve(:), ...
        "anticipationReserve",repmat(anticipationReserve,pointCount,1), ...
        "safety",repmat(safety,pointCount,1),"label",repmat(labels,pointCount,1), ...
        "stage",repmat(tube.stage,numel(physical),1),"local",local);
end
