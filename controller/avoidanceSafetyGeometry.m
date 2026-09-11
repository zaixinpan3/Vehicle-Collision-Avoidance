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
                allGeometricRows = avoidanceSafetyGeometry.cellRows(data);
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
                projected = avoidanceSafetyGeometry.projectRows(data);
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

        function [normals,information] = optimizeNormals(model,prediction,plan)
        %avoidanceSafetyGeometry.optimizeNormals Continuous maximum-margin cell-normal proposals.
        % A small SOCP separates synchronous relative Bernstein footprint enclosures.
        % Returned normals are proposals; complete trajectory checking remains required.
            cfg=model.cfg;
            if exist("solveAvoidanceSocpMex","file")~=3
                addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'solver','clarabel','matlab'));
            end
            cells=prediction.cells;
            if isfield(prediction,'geometryFrames')
                frames=prediction.geometryFrames;
            else
                frames=laneGeometry.sweptCellFrames(model,cells,plan);
            end
            normals=cell(numel(cells),1);margins=zeros(numel(cells),numel(model.encounters));
            flags=zeros(size(margins));
            signs=[1,1,-1,-1;1,-1,1,-1];
            egoBody=[cfg.vehicle.length/2;cfg.vehicle.width/2].*signs;
            for index=1:numel(cells)
                tube=cells(index);frame=frames(index);
                nominal=reshape(pagemtimes(tube.map,plan),6,[])+tube.offset;
                map=[frame.tangent,frame.lateral];
                egoPosition=frame.origin+map*nominal(1:2,:);
                egoRadius=abs(map)*tube.radius(1:2,:)+frame.positionErrorBound;
                egoHeading=frame.heading+mean(nominal(3,:));
                headingRadius=max(abs(frame.heading+nominal(3,:)-egoHeading)+tube.radius(3,:))+frame.headingErrorBound;
                egoCorners=localRotation(egoHeading)*egoBody;
                normals{index}=zeros(2,numel(model.encounters));
                for targetIndex=1:numel(model.encounters)
                    target=model.encounters(targetIndex);
                    [center,radius]=targetPrediction.finiteFlow(target,tube.start);
                    [last,lastRadius]=targetPrediction.finiteFlow(target,tube.start+tube.duration);
                    degree=size(tube.offset,2)-1;
                    transform=stateUncertainty.bernsteinTransform(degree,tube.duration);
                    targetPosition=[center(1:2),center(3:4),center(5:6)/2,zeros(2,degree-2)]*transform.';
                    targetRadius=[radius(1:2),radius(3:4),radius(5:6)/2, ...
                        target.contract.jerkBound/6,zeros(2,degree-3)]*transform.';
                    targetHeading=(center(7)+last(7))/2;
                    targetHeadingRadius=abs(last(7)-center(7))/2+lastRadius(7);
                    targetCorners=localRotation(targetHeading)*([target.halfLength;target.halfWidth].*signs);
                    rotationRadius=hypot(cfg.vehicle.length/2,cfg.vehicle.width/2)*headingRadius ...
                        +hypot(target.halfLength,target.halfWidth)*targetHeadingRadius;
                    points=zeros(2,16*size(nominal,2));radii=points;
                    cursor=0;
                    for point=1:size(nominal,2)
                        for a=1:4
                            selected=cursor+(1:4);
                            points(:,selected)=egoPosition(:,point)-targetPosition(:,point)+egoCorners(:,a)-targetCorners;
                            radii(:,selected)=repmat(egoRadius(:,point)+targetRadius(:,point),1,4);
                            cursor=cursor+4;
                        end
                    end
                    matrix=[-points.',radii.',ones(cursor,1);eye(2),-eye(2),zeros(2,1); ...
                        -eye(2),-eye(2),zeros(2,1)];
                    bound=[-(cfg.collision.clearanceMargin+rotationRadius)*ones(cursor,1);zeros(4,1)];
                    cone=[zeros(1,5);-eye(2),zeros(2,3)];
                    [decision,status]=solveAvoidanceSocpMex(sparse(5,5),[0;0;0;0;-1], ...
                        sparse([matrix;cone]),[bound;1;0;0],[0;numel(bound);3], ...
                        [cfg.solver.constraintTolerance,cfg.solver.optimalityTolerance,cfg.solver.maxIterations]);
                    flags(index,targetIndex)=status.status;
                    if any(status.status==[1,4]) && norm(decision(1:2))>1e-8
                        normals{index}(:,targetIndex)=decision(1:2)/norm(decision(1:2));
                        margins(index,targetIndex)=decision(5);
                    else
                        [~,normals{index}(:,targetIndex)]=avoidanceSafetyGeometry.rectangleDistance(mean(egoPosition,2),egoHeading, ...
                            mean(targetPosition,2),targetHeading,[cfg.vehicle.length/2;cfg.vehicle.width/2;target.halfLength;target.halfWidth]);
                        margins(index,targetIndex)=-inf;
                    end
                end
            end
            information=struct('proposalMargins',margins,'nativeStatus',flags, ...
                'scope',"normal proposals only; full original swept certificate must be checked");
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

function rotation=localRotation(angle)
    rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];
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
