classdef avoidanceSafetyGeometry
    %avoidanceSafetyGeometry Swept separation, normal optimization and footprint distances.

    methods (Static)
        function rows = cellRows(data)
            % Shared numeric geometry kernel used by MATLAB and codegen.
            assert(~isempty(data));
            rows = localCellRows(data(1));
            rows = repmat(rows,numel(data),1);
            for index = 2:numel(data)
                rows(index) = localCellRows(data(index));
            end
        end

        function rows = projectRows(data)
            % Shared numeric geometry kernel used by MATLAB and codegen.
            assert(~isempty(data));
            rows = localProjectedRows(data(1));
            rows = repmat(rows,numel(data),1);
            for index = 2:numel(data)
                rows(index) = localProjectedRows(data(index));
            end
        end

        function geometry = build(model, prediction)
        % One separating normal per cell; every Bernstein point uses that normal.
        % The cellRows entry exposes the shared numeric geometry kernel for codegen.
            cfg = model.cfg;
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
                if isfield(prediction,"fixedInputs")
                    anchorCells = prediction.cells;
                    for index = 1:numel(anchorCells)
                        tube = anchorCells(index);
                        input = prediction.fixedInputs(:,tube.stage);
                        anchorCells(index).offset = tube.offset+reshape( ...
                            reshape(permute(tube.map,[1,3,2]),[],2)*input,6,[]);
                        anchorCells(index).map(:) = 0;
                    end
                    [computedFrames,computedNominal] = laneGeometry.sweptCellFrames(model,anchorCells,zeros(2,1));
                else
                    [computedFrames,computedNominal] = laneGeometry.sweptCellFrames(model,prediction.cells,model.anchorPlan);
                end
            end
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
                reach = repmat([cfg.model.frontWheelSteeringAngleMaximum; ...
                    max(abs([cfg.actuation.brakingRatioMinimum,cfg.actuation.brakingRatioMaximum]))],size(tube.map,2)/2,1);
                support = reshape(pagemtimes(abs(tube.map),reach),6,[]);
                envelope = max(abs(tube.offset)+support+tube.radius,[],2);
                data = struct("frame",[frame.origin;frame.tangent;frame.lateral;frame.heading; ...
                    frame.positionErrorBound;frame.headingErrorBound;frame.stationLower;frame.stationUpper], ...
                    "nominal",nominal,"targets",targets,"boundaries",boundaries, ...
                    "settings",[cfg.vehicle.length/2;cfg.vehicle.width/2;envelope(3); ...
                        envelope(2);cfg.collision.clearanceMargin], ...
                    "duration",tube.duration,"degree",size(tube.offset,2)-1, ...
                    "normals",zeros(2,0));
                if isfield(prediction,"separationNormals")
                    data.normals=prediction.separationNormals{cellIndex};
                    validateattributes(data.normals,{'double'},{'size',[2,numel(model.encounters)],'finite','real'});
                    if any(abs(vecnorm(data.normals)-1)>1e-10)
                        error("collisionAvoidanceController:invalidSeparationNormal","Separation normals must be unit vectors.");
                    end
                end
                cellData{cellIndex} = data;
                activeTargets{cellIndex} = active;
                sourceLabels{cellIndex} = [targetLabels;boundaryLabels];
            end
            data = vertcat(cellData{:});
            if nativeGeometry
                allGeometricRows = avoidanceCellRowsKernelMex(data);
            else
                allGeometricRows = avoidanceSafetyGeometry.cellRows(data);
            end
            projectionData = cell(numel(groups),1);
            for cellIndex = 1:numel(groups)
                tube = prediction.cells(cellIndex);
                stateRadius = tube.radius;
                numericTube = struct("stage",tube.stage,"map",tube.map,"offset",tube.offset, ...
                    "localStateMap",tube.localStateMap,"localInputMap",tube.localInputMap, ...
                    "localOffset",tube.localOffset,"numericalRadius",tube.numericalRadius);
                if isfield(prediction,"fixedInputs") || isfield(prediction,"parametricInputs")
                    numericTube.stage = 1;
                end
                projectionData{cellIndex} = struct("tube",numericTube, ...
                    "stateRadius",stateRadius, ...
                    "geometricRows",allGeometricRows(cellIndex),"planCount",prediction.planCount);
                normals = zeros(2,numel(model.encounters));
                normals(:,activeTargets{cellIndex}) = allGeometricRows(cellIndex).normals;
                normalGroups{cellIndex} = normals;
            end
            data = vertcat(projectionData{:});
            if exist("avoidanceProjectedRowsKernelMex","file")==3
                projected = avoidanceProjectedRowsKernelMex(data);
            else
                projected = avoidanceSafetyGeometry.projectRows(data);
            end
            for cellIndex = 1:numel(groups)
                names = sourceLabels{cellIndex};
                group = projected(cellIndex);
                if isfield(prediction,"parametricInputs")
                    stage = prediction.cells(cellIndex).stage;
                    direct = reshape(permute(group.local.nodeInputRows,[1,3,2]),[],2);
                    group.matrix = group.matrix+direct*(prediction.inputSensitivity(:,:,stage)-eye(2));
                    group.physicalBound = group.physicalBound-direct*prediction.parametricInputs(:,stage);
                    group.stage(:) = stage;
                end
                if isfield(prediction,"fixedInputs")
                    stage = prediction.cells(cellIndex).stage;
                    input=prediction.fixedInputs(:,stage);
                    allowance=8*eps*(abs(group.physicalBound)+abs(group.matrix)*abs(input));
                    group.physicalBound = group.physicalBound-group.matrix*input-allowance;
                    group.matrix = zeros(size(group.matrix,1),0);
                    group.stage(:) = stage;
                end
                local = group.local;
                local.nodeLabels = names(local.nodeLabels);
                localGroups{cellIndex} = local;
                group = rmfield(group,"local");group.label = names(group.label);
                group.cellIndex = repmat(cellIndex,numel(group.physicalBound),1);
                groups{cellIndex} = group;
            end
            groups = vertcat(groups{:});
            geometry = struct("matrix", vertcat(groups.matrix), "physicalBound", vertcat(groups.physicalBound), ...
                "safety", vertcat(groups.safety), "label", vertcat(groups.label), "stage", vertcat(groups.stage), ...
                "frames", vertcat(frames{:}), "normals", {normalGroups},"local",vertcat(localGroups{:}), ...
                "cellIndex",vertcat(groups.cellIndex),"cellData",vertcat(cellData{:}));
        end

        function [normal,information] = supportDirection(egoPosition,egoYaw,targetPosition,targetYaw,halfDimensions)
        % A support direction remains meaningful at overlap and contact.
            [vertices,faces,bounds]=localConfigurationObstacle(egoYaw,targetPosition,targetYaw,halfDimensions);
            [distance,normal,outside]=localPointPolygonSignedDistance(egoPosition,vertices,faces,bounds);
            support=max(normal.'*vertices);
            gaps=faces*egoPosition-bounds;
            tied=gaps>=max(gaps)-1e-9*(1+max(abs(bounds)));
            information=struct('available',all(isfinite(normal)) && abs(norm(normal)-1)<1e-10, ...
                'signedDistance',distance,'anchorSeparated',outside,'support',support, ...
                'alternatives',faces(tied,:).');
        end

        function [normals,information] = supportNormals(model,prediction,plan,frames,geometry)
        % Score whole-hold directions; a side sector selects a constraint family,
        % never a prescribed trajectory or a passing time.
            if nargin<5,geometry=[];end
            cells=prediction.cells;targets=numel(model.encounters);
            normals=cell(numel(cells),1);available=true;overlaps=0;minimum=Inf;
            switches=0;cfg=model.cfg;previous=zeros(2,targets);
            sectors=zeros(1,targets);
            if isfield(model,'supportSectors'),sectors=model.supportSectors;end
            degree=size(cells(1).offset,2)-1;
            weights=arrayfun(@(k)nchoosek(degree,k),0:degree).'/2^degree;
            native=exist('avoidanceSupportKernelMex','file')==3;
            for index=1:numel(cells)
                tube=cells(index);frame=frames(index);
                if size(tube.offset,2)~=numel(weights)
                    degree=size(tube.offset,2)-1;
                    weights=arrayfun(@(k)nchoosek(degree,k),0:degree).'/2^degree;
                end
                points=reshape(pagemtimes(tube.map,plan),6,[])+tube.offset;
                state=points*weights;position=frame.origin+[frame.tangent,frame.lateral]*state(1:2);
                yaw=frame.heading+state(3);normals{index}=zeros(2,targets);
                for targetIndex=1:targets
                    target=model.encounters(targetIndex);
                    center=targetPrediction.finiteFlow(target,tube.start+tube.duration/2);
                    dimensions=[cfg.vehicle.length/2;cfg.vehicle.width/2;target.halfLength;target.halfWidth];
                    if native
                        [normal,query]=avoidanceSupportKernelMex(position,yaw,center(1:2),center(7),dimensions);
                    else
                        [normal,query]=avoidanceSafetyGeometry.supportDirection(position,yaw,center(1:2),center(7),dimensions);
                    end
                    candidates=[normal,query.alternatives,previous(:,targetIndex),frame.tangent,-frame.tangent];
                    if sectors(targetIndex)~=0,candidates=[candidates,sectors(targetIndex)*frame.lateral];end
                    if ~isempty(geometry)
                        candidates=[candidates,geometry.normals{index}(:,targetIndex)];
                    end
                    valid=vecnorm(candidates)>.5;
                    if sectors(targetIndex)~=0
                        valid=valid & sectors(targetIndex)*(frame.lateral.'*candidates)>=-1e-10;
                    end
                    candidates=candidates(:,valid);
                    [~,uniqueIndex]=unique(round(candidates.',12),'rows','stable');candidates=candidates(:,uniqueIndex);
                    score=zeros(1,size(candidates,2));
                    if ~isempty(geometry)
                        data=geometry.cellData(index);data.nominal=points;
                        data.targets=data.targets(targetIndex);data.boundaries=data.boundaries([]);
                        batch=repmat(data,size(candidates,2),1);
                        for option=1:numel(batch),batch(option).normals=candidates(:,option);end
                        if exist('avoidanceCellRowsKernelMex','file')==3
                            queries=avoidanceCellRowsKernelMex(batch);
                        else
                            queries=avoidanceSafetyGeometry.cellRows(batch);
                        end
                    end
                    for option=1:size(candidates,2)
                        direction=candidates(:,option);
                        if isempty(geometry)
                            support=targetPrediction.rectangleSupport(target.halfLength,target.halfWidth,direction,center(7),0) ...
                                +targetPrediction.rectangleSupport(cfg.vehicle.length/2,cfg.vehicle.width/2,direction,yaw,0);
                            score(option)=direction.'*(position-center(1:2))-support;
                        else
                            rows=queries(option);
                            margin=rows.bound-rows.state*points-abs(rows.state)*tube.radius;
                            reach=repmat([cfg.model.frontWheelSteeringAngleMaximum; ...
                                max(abs([cfg.actuation.brakingRatioMinimum,cfg.actuation.brakingRatioMaximum]))],prediction.stageCount,1);
                            support=reshape(pagemtimes(abs(tube.map),reach),6,[]);
                            offset=rows.bound-rows.state*tube.offset-abs(rows.state)*tube.radius;
                            scale=1+abs(offset)+abs(rows.state)*support;
                            reserve=4*max(cfg.encounter.numericalMargin,cfg.solver.constraintTolerance)*scale;
                            score(option)=min(margin-reserve,[],'all');
                        end
                    end
                    best=max(score);tied=find(score>=best-1e-8*(1+abs(best)));
                    preference=previous(:,targetIndex);
                    if norm(preference)<.5,preference=frame.lateral;if sectors(targetIndex)<0,preference=-preference;end,end
                    [~,choice]=max(preference.'*candidates(:,tied));normal=candidates(:,tied(choice));
                    if index>1,switches=switches+double(norm(normal-previous(:,targetIndex))>1e-6);end
                    normals{index}(:,targetIndex)=normal;previous(:,targetIndex)=normal;
                    available=available && query.available;overlaps=overlaps+double(query.signedDistance<=0);
                    minimum=min(minimum,query.signedDistance);
                end
            end
            information=struct('available',available,'overlappingMidpoints',overlaps, ...
                'minimumAnchorDistance',minimum,'normalSwitchCount',switches, ...
                'used',false,'witnessPreserved',false);
        end

        function [signedDistance, normal, supportValue, outside] = ...
                rectangleDistance( ...
                egoPosition, egoYaw, targetPosition, targetYaw, halfDimensions)
        % avoidanceSafetyGeometry.rectangleDistance The configuration obstacle of two rectangles.
        %
        % THE geometry of the collision problem, and the whole of stage 1. The
        % Minkowski sum of the target rectangle and the ego rectangle rotated
        % to the ego yaw is the exact configuration obstacle of the ego CENTRE:
        % the two rectangles intersect if and only if the ego centre lies in
        % it. Both being centrally symmetric, the sum is the zonotope of their
        % four half-edge generators - a convex polygon of at most eight
        % vertices whose edge normals are the two rectangles' own edge
        % normals. No bounding box is taken of either rectangle anywhere.
        %
        % halfDimensions is [egoHalfLength; egoHalfWidth; targetHalfLength;
        % targetHalfWidth]. Returns the SIGNED distance from the ego position
        % to that polygon (positive outside by the separation, negative inside
        % by the least penetration), the unit NORMAL of the supporting
        % half-plane the position is separated by - the direction from the
        % closest boundary point to the position outside, and the outward
        % normal of the least-penetrated edge inside, which is the unified
        % minimum-penetration answer stage 1 needs for a probe that has run
        % into the obstacle - the SUPPORT VALUE of the polygon in that
        % direction (so that normal'*z <= supportValue for every z of the
        % obstacle, with equality at the closest point), and whether the
        % position was outside.
        %
        % Called in PATH COORDINATES by stage 1 (formulateAvoidanceProblem), with the
        % heading errors as the yaws and the target centre as the origin, and
        % in Cartesian coordinates by the harness for the physical clearance
        % readout.
        %
            [vertices, faceNormal, faceBound] = localConfigurationObstacle( ...
                egoYaw, targetPosition, targetYaw, halfDimensions);
            [signedDistance, normal, outside] = localPointPolygonSignedDistance( ...
                egoPosition, vertices, faceNormal, faceBound);
            supportValue = max(normal.' * vertices);
        end
    end
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
        elseif isfield(data,'normals') && ~isempty(data.normals)
            normal=data.normals(:,index);
        else
            [~,normal] = avoidanceSafetyGeometry.rectangleDistance(centerEgo,heading+mean(nominal(3,:)), ...
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
    tube = data.tube;
    stateRadius = data.stateRadius;pointCount = size(tube.offset,2);
    geometricRows = data.geometricRows;
    rowCount = size(geometricRows.state,1);
    stateRows = geometricRows.state;
    inputRows = zeros(rowCount,2);
    limits = geometricRows.bound;
    labels = geometricRows.source;
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
        "safety",true(numel(physical),1),"label",repmat(labels,pointCount,1), ...
        "stage",repmat(tube.stage,numel(physical),1),"local",local);
end

function [vertices, faceNormal, faceBound] = ...
        localConfigurationObstacle( ...
        egoYaw, targetPosition, targetYaw, halfDimensions)
% The Minkowski sum of the two oriented rectangles, as the zonotope of
% their four half-edge generators: sort the generators by direction and
% walk them, which traces the eight vertices in order. Written without
% loops - stage 1 evaluates this at every node of every program, and
% the loop form measured 15 ms of a sample against 2 ms here.
    egoCosine = cos(egoYaw);
    egoSine = sin(egoYaw);
    targetCosine = cos(targetYaw);
    targetSine = sin(targetYaw);
    generator = [ ...
        halfDimensions(1)*egoCosine, -halfDimensions(2)*egoSine, ...
        halfDimensions(3)*targetCosine, -halfDimensions(4)*targetSine; ...
        halfDimensions(1)*egoSine, halfDimensions(2)*egoCosine, ...
        halfDimensions(3)*targetSine, halfDimensions(4)*targetCosine];
    % Canonical half-plane, then sorted by direction.
    generatorAngle = atan2(generator(2, :), generator(1, :));
    reverse = generatorAngle < 0.0 | generatorAngle >= pi;
    generator(:, reverse) = -generator(:, reverse);
    generatorAngle(reverse) = mod(generatorAngle(reverse), pi);
    [~, generatorOrder] = sort(generatorAngle);
    step = 2.0 * generator(:, generatorOrder);

    walk = [step, -step(:, 1:3)];
    vertices = targetPosition - sum(generator, 2) ...
        + cumsum([zeros(2, 1), walk], 2);
    edge = vertices(:, [2:8, 1]) - vertices;
    edgeLength = max(sqrt(sum(edge.^2, 1)), realmin);
    faceNormal = [edge(2, :); -edge(1, :)] ./ edgeLength;
    faceBound = sum(faceNormal .* vertices, 1).';
    faceNormal = faceNormal.';
end

function [signedDistance, normal, outside] = ...
        localPointPolygonSignedDistance( ...
        point, vertices, faceNormal, faceBound)
    violation = faceNormal * point - faceBound;
    tolerance = 100.0 * eps(max( ...
        [1.0; abs(faceBound); abs(point)]));
    outside = any(violation > tolerance);
    if ~outside
        % Inside: the least-penetrated edge is the minimum-norm way
        % out, and its outward normal is the supporting direction.
        [signedDistance, faceIdx] = max(violation);
        normal = faceNormal(faceIdx, :).';
        if abs(signedDistance) <= tolerance
            signedDistance = 0.0;
        end
        return;
    end
    edge = vertices(:, [2:8, 1]) - vertices;
    offset = point - vertices;
    parameter = sum(offset .* edge, 1) ./ max(sum(edge.^2, 1), realmin);
    parameter = min(max(parameter, 0.0), 1.0);
    difference = offset - parameter .* edge;
    distanceSquared = sum(difference.^2, 1);
    [minimumSquared, edgeIdx] = min(distanceSquared);
    signedDistance = sqrt(max(minimumSquared, 0.0));
    if signedDistance > tolerance
        normal = difference(:, edgeIdx) / signedDistance;
    else
        [~, faceIdx] = max(violation);
        normal = faceNormal(faceIdx, :).';
    end
end
