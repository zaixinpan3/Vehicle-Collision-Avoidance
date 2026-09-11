function geometry = avoidanceSafetyGeometry(model, prediction)
% One separating normal per cell; every Bernstein point uses that normal.
% The cellRows entry exposes the shared numeric geometry kernel for codegen.
    if ischar(model) || isstring(model)
        assert(~isempty(prediction));
        if string(model)=="cellRows"
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
        "halfLength",0,"halfWidth",0);
    nativeGeometry = exist("avoidanceCellRowsKernelMex","file")==3;
    cachedFrames = isfield(prediction,"geometryAnchor") ...
        && isequal(prediction.geometryAnchor,model.anchorPlan);
    if ~cachedFrames
        [computedFrames,computedNominal] = laneGeometry.sweptCellFrames(model,prediction.cells,model.anchorPlan);
    end
    nominalCells = cell(numel(groups),1);
    cellData = cell(numel(groups),1);
    activeTargets = cell(numel(groups),1);
    sourceLabels = cell(numel(groups),1);
    for cellIndex = 1:numel(groups)
        tube = prediction.cells(cellIndex);
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
            active = 1:numel(model.encounters);
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
            [target.center,target.radius] = targetPrediction.finiteFlow(encounter,tube.start);
            targets(targetIndex) = target;
            targetLabels(targetIndex) = "collision:"+encounter.key;
        end
        data = struct("frame",[frame.origin;frame.tangent;frame.lateral;frame.heading; ...
            frame.positionErrorBound;frame.headingErrorBound;frame.stationLower;frame.stationUpper], ...
            "nominal",nominal,"targets",targets,"boundaries",boundaries, ...
            "settings",[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.model.headingDomainRadius; ...
                cfg.model.lateralDomainRadius;cfg.collision.clearanceMargin], ...
            "duration",tube.duration,"degree",degree);
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
    if isfield(prediction,"separationNormals")
        for cellIndex = 1:numel(groups)
            directions = prediction.separationNormals{cellIndex};
            validateattributes(directions,{'double'},{'size',[2,numel(model.encounters)],'finite','real'});
            if any(abs(vecnorm(directions)-1)>1e-10)
                error("collisionAvoidanceController:invalidSeparationNormal","Separation normals must be unit vectors.");
            end
            allGeometricRows(cellIndex) = localCellRows(cellData{cellIndex},directions);
        end
    end
    projectionData = cell(numel(groups),1);
    numericCfg = struct("model",struct("lateralDomainRadius",cfg.model.lateralDomainRadius, ...
        "headingDomainRadius",cfg.model.headingDomainRadius,"speedMinimum",cfg.model.speedMinimum, ...
        "speedMaximum",cfg.model.speedMaximum,"lateralVelocityMaximum",cfg.model.lateralVelocityMaximum, ...
        "yawRateMaximum",cfg.model.yawRateMaximum,"scheduleSpeedFloor",cfg.model.scheduleSpeedFloor, ...
        "slipAngleMaximum",repmat(cfg.model.slipAngleMaximum(:),2/numel(cfg.model.slipAngleMaximum),1)), ...
        "vehicle",struct("lf",cfg.vehicle.lf,"lr",cfg.vehicle.lr));
    for cellIndex = 1:numel(groups)
        tube = prediction.cells(cellIndex);
        stateRadius = tube.radius;
        frame = frames{cellIndex};
        numericTube = struct("stage",tube.stage,"map",tube.map,"offset",tube.offset, ...
            "localStateMap",tube.localStateMap,"localInputMap",tube.localInputMap, ...
            "localOffset",tube.localOffset,"numericalRadius",tube.numericalRadius);
        projectionData{cellIndex} = struct("cfg",numericCfg,"tube",numericTube, ...
            "nominal",nominalCells{cellIndex},"stationRange",[frame.stationLower;frame.stationUpper], ...
            "stateRadius",stateRadius,"scheduleSpeed",prediction.scheduleSpeedProfile(tube.stage), ...
            "geometricRows",allGeometricRows(cellIndex),"planCount",prediction.planCount);
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
        names = ["modelDomain";"tireSlip";sourceLabels{cellIndex}];
        group = projected(cellIndex);
        local = group.local;
        local.nodeLabels = names(local.nodeLabels);
        localGroups{cellIndex} = local;
        group = rmfield(group,"local");group.label = names(group.label);
        groups{cellIndex} = group;
    end
    groups = vertcat(groups{:});
    geometry = struct("matrix", vertcat(groups.matrix), "physicalBound", vertcat(groups.physicalBound), ...
        "safety", vertcat(groups.safety), "label", vertcat(groups.label), "stage", vertcat(groups.stage), ...
        "frames", vertcat(frames{:}), "normals", {normalGroups},"local",vertcat(localGroups{:}));
end

function rows = localCellRows(data,prescribedNormals)
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
    source = zeros(maximumRows,1);
    normals = zeros(2,targetCount);count = 0;
    for index = 1:targetCount
        target = data.targets(index);
        middle = targetPrediction.finiteFlow(target,data.duration/2);
        centerEgo = origin+[tangent,lateral]*mean(nominal(1:2,:),2);
        if nargin > 1
            normal = prescribedNormals(:,index);
        else
            [~,normal] = rectangleConfigurationDistance(centerEgo,heading+mean(nominal(3,:)), ...
                middle(1:2),middle(7),[halfLength;halfWidth;target.halfLength;target.halfWidth]);
        end
        normals(:,index) = normal;
        degree = data.degree;
        positionPolynomial = [target.center(1:2),target.center(3:4),target.center(5:6)/2,zeros(2,degree-2)];
        radiusPolynomial = [target.radius(1:2),target.radius(3:4),target.radius(5:6)/2, ...
            target.contract.jerkBound/6,zeros(2,degree-3)];
        transform = stateUncertainty.bernsteinTransform(degree,data.duration);
        if pointCount==1,transform = transform(1,:);end
        targetPosition = positionPolynomial*transform.';
        targetRadius = radiusPolynomial*transform.';
        [endCenter,endRadius] = targetPrediction.finiteFlow(target,data.duration);
        yawCenter = (target.center(7)+endCenter(7))/2;
        yawRadius = abs(endCenter(7)-target.center(7))/2+endRadius(7);
        targetSupport = targetPrediction.rectangleSupport(target.halfLength,target.halfWidth,normal,yawCenter,yawRadius);
        [egoSupport,headingSlope] = targetPrediction.rectangleSupportMajorant(normal,heading, ...
            halfLength,halfWidth,mean(nominal(3,:)),headingDomain+headingError);
        rowCount = numel(egoSupport);selected = count+(1:rowCount);
        state(selected,:) = repmat([-normal.'*[tangent,lateral],zeros(1,4)],rowCount,1);
        state(selected,3) = headingSlope;
        bound(selected,:) = normal.'*origin-normal.'*targetPosition-abs(normal).'*targetRadius ...
            -targetSupport-egoSupport-clearanceMargin-abs(normal).'*positionError-abs(headingSlope)*headingError;
        source(selected) = index;count = count+rowCount;
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
        "source",source(1:count),"normals",normals);
end

function result = localProjectedRows(data)
% Apply the same cell rows to condensed and local coordinates in one kernel.
    cfg = data.cfg;tube = data.tube;
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
    geometricRows = data.geometricRows;
    rowCount = size(geometricRows.state,1);
    stateRows = [stateRows;geometricRows.state];
    inputRows = [inputRows;zeros(rowCount,2)];
    limits = [limits;geometricRows.bound];
    safety = [safety;true(rowCount,1)];
    labels = [labels;2+geometricRows.source];
    pointStateRows = repmat(stateRows,1,1,pointCount);
    pointStartRows = zeros(size(pointStateRows));
    pointInputRows = repmat(inputRows,1,1,pointCount);
    mapped = pagemtimes(pointStateRows, tube.map)+pagemtimes(pointStartRows,tube.map(:,:,1));
    for point = 1:pointCount
        mapped(:, 2*tube.stage-1:2*tube.stage, point) = ...
            mapped(:, 2*tube.stage-1:2*tube.stage, point)+pointInputRows(:,:,point);
    end
    uncertaintySupport = reshape(pagemtimes(abs(pointStateRows),reshape(stateRadius,6,1,pointCount)),[],pointCount);
    uncertaintySupport = uncertaintySupport+reshape(pagemtimes(abs(pointStartRows),stateRadius(:,1)),[],pointCount);
    physical = limits-reshape(pagemtimes(pointStateRows,reshape(tube.offset,6,1,pointCount)),[],pointCount) ...
        -reshape(pagemtimes(pointStartRows,tube.offset(:,1)),[],pointCount)-uncertaintySupport;
    localState = pagemtimes(pointStateRows,tube.localStateMap)+pointStartRows;
    localInput = pagemtimes(pointStateRows,tube.localInputMap)+pointInputRows;
    localBound = limits-reshape(pagemtimes(pointStateRows,reshape(tube.localOffset,6,1,pointCount)),[],pointCount)-uncertaintySupport;
    local = struct("stateMatrix",reshape(permute(localState,[1,3,2]),[],6), ...
        "inputMatrix",reshape(permute(localInput,[1,3,2]),[],2),"bound",localBound(:),"stage",tube.stage, ...
        "nodeStateRows",pointStateRows,"nodeInputRows",pointInputRows,"nodeLimits",limits-uncertaintySupport, ...
        "nodeStartStateRows",pointStartRows, ...
        "nodeLabels",labels);
    result = struct("matrix",reshape(permute(mapped,[1,3,2]),[],data.planCount), ...
        "physicalBound",physical(:), ...
        "safety",repmat(safety,pointCount,1),"label",repmat(labels,pointCount,1), ...
        "stage",repmat(tube.stage,numel(physical),1),"local",local);
end
