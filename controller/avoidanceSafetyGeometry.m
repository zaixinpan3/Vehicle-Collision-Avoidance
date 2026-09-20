classdef avoidanceSafetyGeometry
    %avoidanceSafetyGeometry Node separation, nominal support normals and footprint distances.

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
            % Bounded target flows at every cell start, one batch per encounter.
            encounterCount = numel(model.encounters);
            cellStarts = reshape([prediction.cells.start],1,[]);
            flowCenters = cell(encounterCount,1);flowRadii = cell(encounterCount,1);
            baseTargets = repmat(targetTemplate,encounterCount,1);
            targetLabels = strings(encounterCount,1);
            for encounterIndex = 1:encounterCount
                encounter = model.encounters(encounterIndex);
                [flowCenters{encounterIndex},flowRadii{encounterIndex}] = ...
                    targetPrediction.finiteFlow(encounter,cellStarts);
                baseTargets(encounterIndex).halfLength = encounter.halfLength;
                baseTargets(encounterIndex).halfWidth = encounter.halfWidth;
                baseTargets(encounterIndex).contract.jerkBound = encounter.contract.jerkBound;
                baseTargets(encounterIndex).contract.yawAccelerationBound = encounter.contract.yawAccelerationBound;
                targetLabels(encounterIndex) = "collision:"+encounter.key;
            end
            activeAll = 1:encounterCount;
            labelsPerCell = [targetLabels;boundaryLabels;"poseDomain";"referencePhaseDomain"];
            reach = zeros(0,1);
            if ~isempty(prediction.cells)
                reach = repmat([cfg.model.frontWheelSteeringAngleMaximum; ...
                    max(abs([cfg.actuation.brakingRatioMinimum,cfg.actuation.brakingRatioMaximum]))], ...
                    size(prediction.cells(1).map,2)/2,1);
            end
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
                active = activeAll;
                targets = baseTargets;
                for targetIndex = 1:encounterCount
                    targets(targetIndex).center = flowCenters{targetIndex}(:,cellIndex);
                    targets(targetIndex).radius = flowRadii{targetIndex}(:,cellIndex);
                end
                support = reshape(pagemtimes(abs(tube.map),reach),6,[]);
                envelope = max(abs(tube.offset)+support+tube.radius,[],2);
                [pose,domain]=laneGeometry.poseData(frame);
                data = struct("frame",[frame.origin;frame.tangent;frame.lateral;frame.heading; ...
                    frame.positionErrorBound;frame.headingErrorBound;frame.stationLower;frame.stationUpper], ...
                    "pose",pose,"domain",domain, ...
                    "nominal",nominal,"targets",targets,"boundaries",boundaries, ...
                    "settings",[cfg.vehicle.length/2;cfg.vehicle.width/2;envelope(3); ...
                        envelope(2)], ...
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
                sourceLabels{cellIndex} = labelsPerCell;
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
                if isfield(prediction,'referencePhaseIndex')
                    % Stage-local phase rows share the swept projection kernel,
                    % so the lifted SOCP retains its sparse temporal structure.
                    phase=linspace(prediction.referenceStates(1,tube.stage), ...
                        prediction.referenceStates(1,tube.stage+1),size(tube.offset,2));
                    geometric=allGeometricRows(cellIndex);
                    geometric.state=[geometric.state;1,zeros(1,5);-1,zeros(1,5)];
                    geometric.bound=[geometric.bound;cfg.encounter.referencePhaseRadius+phase; ...
                        cfg.encounter.referencePhaseRadius-phase];
                    geometric.source=[geometric.source;repmat(numel(sourceLabels{cellIndex}),2,1)];
                    lateralLimit=model.cruiseCertificate.lateralRegularityRadius;
                    if isfinite(lateralLimit)
                        geometric.state=[geometric.state;0,1,zeros(1,4);0,-1,zeros(1,4)];
                        geometric.bound=[geometric.bound;repmat(lateralLimit,2,numel(phase))];
                        geometric.source=[geometric.source;repmat(numel(sourceLabels{cellIndex}),2,1)];
                    end
                    allGeometricRows(cellIndex)=geometric;
                end
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

        function program = jointProgram(program,model)
        % Keep the convex base and carry the occupied sets with their angles.
        % These independent position/yaw enclosures are fixed at admission;
        % conditioning never replaces them with a larger product enclosure.
            inherited=program.inheritedPredictionFamily;
            if inherited
                certificate=model.carriedWitness.program.jointCertificate;
                keep=[certificate.records.stage]>1 & ...
                    ~ismember(string({certificate.records.key}),model.dischargedTargetKeys);
                certificate.records=certificate.records(keep);
                certificate.angles=certificate.angles(keep);
                certificate.upperBound=certificate.upperBound(keep);
                for index=1:numel(certificate.records)
                    certificate.records(index).stage=certificate.records(index).stage-1;
                end
            else
                records=cell(0,1);angles=zeros(0,1);
                for index=1:numel(program.prediction.cells)
                    tube=program.prediction.cells(index);
                    assert(tube.duration==0 && size(tube.offset,2)==1, ...
                        'avoidanceSafetyGeometry:nodeCertificate','Joint certificates require hold nodes.');
                    data=program.geometry.cellData(index);
                    for target=1:numel(data.targets)
                        records{end+1,1}=localJointRecord(data.targets(target), ...
                            model.encounters(target).key,tube.stage,program.geometry.frames(index), ...
                            tube.radius,[model.cfg.vehicle.length;model.cfg.vehicle.width]/2, ...
                            0,false,model.cfg.solver.constraintTolerance); %#ok<AGROW>
                        normal=program.geometry.normals{index}(:,target);
                        angles(end+1,1)=atan2(normal(2),normal(1)); %#ok<AGROW>
                    end
                end
                for target=1:numel(model.encounters)
                    item=model.encounters(target);
                    [item.center,item.radius]=targetPrediction.finiteFlow(item, ...
                        program.prediction.stageCount*model.sampleTime);
                    records{end+1,1}=localJointRecord(item,item.key,program.prediction.stageCount, ...
                        program.completion.frame,program.prediction.initialErrorBound(:,end), ...
                        zeros(2,1),model.confirmation.range+model.cfg.encounter.numericalMargin, ...
                        true,model.cfg.solver.constraintTolerance); %#ok<AGROW>
                    normal=-program.completion.direction(:,target);
                    angles(end+1,1)=atan2(normal(2),normal(1)); %#ok<AGROW>
                end
                records=vertcat(records{:});
                reserve=zeros(numel(records),1);
                states=localJointStates(program,program.anchorPlan);
                for index=1:numel(records)
                    record=records(index);
                    relative=record.positionOffset+record.positionMap*states(:,record.stage+1);
                    reserve(index)=8*max(model.cfg.encounter.numericalMargin,model.cfg.solver.constraintTolerance) ...
                        *(1+record.clearance+norm(relative)+sum(record.egoHalfSize)+sum(record.targetHalfSize) ...
                        +sum(vecnorm(record.generators)));
                end
                certificate=struct('records',records,'angles',angles,'upperBound',-reserve);
            end
            % Remove only collision and exit slices, keeping all actuator,
            % slew, chart, reference-phase, CLF and terminal constraints.
            remove=startsWith(program.physicalLabels,"collision:") | startsWith(program.physicalLabels,"exit:");
            if any(remove)
                count=numel(remove);keep=[~remove;true(size(program.A,1)-count,1)];
                program.A=program.A(keep,:);program.b=program.b(keep);
                program.cones(2)=program.cones(2)-nnz(remove);
                program.physicalMatrix=program.physicalMatrix(~remove,:);
                program.physicalBound=program.physicalBound(~remove);
                program.safetyBound=program.safetyBound(~remove);
                program.physicalLabels=program.physicalLabels(~remove);
            end
            geometry=program.geometry;keep=~startsWith(geometry.label,"collision:");
            if ~all(keep)
                for name={'matrix','physicalBound','safety','label','stage','cellIndex'}
                    geometry.(name{1})=geometry.(name{1})(keep,:);
                end
            end
            for index=1:numel(geometry.local)
                item=geometry.local(index);keep=~startsWith(item.nodeLabels,"collision:");
                if all(keep),continue;end
                for name={'stateMatrix','inputMatrix','bound','nodeStateRows','nodeInputRows', ...
                        'nodeLimits','nodeStartStateRows','nodeLabels'}
                    item.(name{1})=item.(name{1})(keep,:,:);
                end
                geometry.local(index)=item;
            end
            program.geometry=geometry;program.jointCertificate=certificate;
            program.obstacleCbfRowCount=nnz(~[certificate.records.isExit]);
            program.supportGeometry.used=true;program.supportGeometry.available=true;
            program.supportGeometry.witnessPreserved=inherited;
            program.replacementContainsWitness=inherited;
        end

        function values = jointResidual(program,decision,angles)
        % Independent nonlinear support evaluation; no conic epigraph values.
            states=localJointStates(program,decision(program.layout.planIndex));
            records=program.jointCertificate.records;values=zeros(numel(records),1);
            for index=1:numel(records)
                values(index)=avoidanceSafetyGeometry.jointValue(records(index), ...
                    states(:,records(index).stage+1),angles(index));
            end
        end

        function allowance = jointAllowance(program,decision)
        % Same arithmetic reserve for MATLAB and standalone verification.
            allowance=localJointAllowance(program,decision);
        end

        function value = jointValue(record,state,angle)
            normal=[cos(angle);sin(angle)];
            yaw=record.yawOffset+record.yawRow*state;
            value=record.clearance+record.positionBall ...
                +avoidanceSafetyGeometry.supportValue(record.egoHalfSize,record.egoYawRadius, ...
                    [cos(angle-yaw);sin(angle-yaw)]) ...
                +avoidanceSafetyGeometry.supportValue(record.targetHalfSize,record.targetYawRadius, ...
                    [cos(angle-record.targetYaw);sin(angle-record.targetYaw)]) ...
                +sum(abs(record.generators.'*normal)) ...
                -normal.'*(record.positionOffset+record.positionMap*state);
        end

        function value = jointMajorant(record,state0,angle0,state,coordinate,positionScale)
        % Global convex bound on sqrt(1+coordinate^2) times the unit residual.
        % The physical angle is angle0+atan(coordinate); the coordinate itself
        % is not an angle. Position scale selects a bound, not a step domain.
            dr=record.positionMap*(state-state0);
            r=record.positionOffset+record.positionMap*state0;
            dyaw=record.yawRow*(state-state0);yaw=record.yawOffset+record.yawRow*state0;
            n=[cos(angle0);sin(angle0)];t=[-n(2);n(1)];
            egoAngle=angle0-yaw;targetAngle=angle0-record.targetYaw;
            ae=[cos(egoAngle);sin(egoAngle)]+[-sin(egoAngle);cos(egoAngle)]*(coordinate-dyaw);
            ao=[cos(targetAngle);sin(targetAngle)]+[-sin(targetAngle);cos(targetAngle)]*coordinate;
            eta=1/positionScale;
            value=(record.clearance+record.positionBall)*hypot(1,coordinate) ...
                +avoidanceSafetyGeometry.supportValue(record.egoHalfSize,record.egoYawRadius,ae) ...
                +avoidanceSafetyGeometry.supportValue(record.targetHalfSize,record.targetYawRadius,ao) ...
                +sum(abs(record.generators.'*(n+t*coordinate)))-n.'*(r+dr)-t.'*r*coordinate ...
                +.5*norm(record.egoHalfSize)*(coordinate^2+2*dyaw^2) ...
                +.25*(sqrt(eta)*(t.'*dr)-coordinate/sqrt(eta))^2;
        end

        function value = supportValue(halfSize,yawRadius,vector)
        % Positively homogeneous support, including the zero vector.
            value=norm(vector)*targetPrediction.rectangleSupport( ...
                halfSize(1),halfSize(2),vector,0,yawRadius);
        end

        function [directions,bounds,radius] = yawHull(halfSize,yawRadius)
        % Exact hull of the four vertex arcs: disk and uncovered-gap chords.
            radius=norm(halfSize);
            phase=atan2(halfSize(2),halfSize(1));
            vertices=[phase;pi-phase;pi+phase;2*pi-phase];
            next=[vertices(2:end);vertices(1)+2*pi];
            gaps=next-vertices-2*yawRadius;keep=gaps>0;
            middle=(vertices+next)/2;
            directions=[cos(middle(keep)),sin(middle(keep))];
            bounds=radius*cos(gaps(keep)/2);
        end

        function program = certifyJoint(program,decision)
        % Numerical reserves belong to the accepted certificate and shift once.
            cert=program.jointCertificate;
            values=avoidanceSafetyGeometry.jointResidual(program,decision,cert.angles);
            allowance=localJointAllowance(program,decision);
            if any(~isfinite(values)) || any(values+allowance>0)
                error('collisionAvoidanceController:optimizationFailed', ...
                    'The independently evaluated joint separation certificate is unsafe. No command was issued.');
            end
            cert.upperBound=max(cert.upperBound,values+allowance);
            program.jointCertificate=cert;
            keys=program.completion.keys;
            for index=1:numel(cert.records)
                item=cert.records(index);normal=[cos(cert.angles(index));sin(cert.angles(index))];
                target=find(keys==string(item.key),1);
                if item.isExit
                    program.completion.direction(:,target)=-normal;
                    program.completion.stateRow(target,:)=-normal.'*item.positionMap;
                    program.completion.stateBound(target)=normal.'*item.positionOffset-item.clearance-item.positionBall ...
                        -sum(abs(item.generators.'*normal))-avoidanceSafetyGeometry.supportValue( ...
                        item.targetHalfSize,item.targetYawRadius, ...
                        [cos(cert.angles(index)-item.targetYaw);sin(cert.angles(index)-item.targetYaw)]);
                else
                    cellIndex=find([program.prediction.cells.stage]==item.stage,1);
                    program.geometry.normals{cellIndex}(:,target)=normal;
                    program.geometry.cellData(cellIndex).normals(:,target)=normal;
                end
            end
            program.prediction.separationNormals=program.geometry.normals;
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

        function [normals,information] = supportNormals(model,prediction,plan)
        % One analytic signed-distance direction at each nominal hold node.
        % No sector, alternate direction scoring, or trajectory modification.
        % At overlap the oracle returns an outward support normal, not a
        % claim that the nominal itself already separates the rectangles.
            cells=prediction.cells;targets=numel(model.encounters);
            count=numel(cells);normals=cell(count,1);
            available=true;overlaps=0;minimum=Inf;switches=0;
            previous=zeros(2,targets);states=zeros(6,count);
            times=reshape([cells.start],1,[]);
            for index=1:count
                tube=cells(index);
                assert(tube.duration==0 && size(tube.offset,2)==1, ...
                    'collisionAvoidanceController:invalidNormalNode', ...
                    'The online nominal requires one state per hold node.');
                states(:,index)=tube.map*plan+tube.offset;
            end
            [positions,yaws]=laneGeometry.fromFrenet(states,model.lane);
            centers=cell(1,targets);
            for targetIndex=1:targets
                centers{targetIndex}=targetPrediction.finiteFlow(model.encounters(targetIndex),times);
            end
            native=exist('avoidanceSupportKernelMex','file')==3;
            for index=1:count
                normals{index}=zeros(2,targets);
                for targetIndex=1:targets
                    target=model.encounters(targetIndex);center=centers{targetIndex}(:,index);
                    dimensions=[model.cfg.vehicle.length/2;model.cfg.vehicle.width/2; ...
                        target.halfLength;target.halfWidth];
                    if native
                        [normal,query]=avoidanceSupportKernelMex(positions(:,index),yaws(index),center(1:2),center(7),dimensions);
                    else
                        [normal,query]=avoidanceSafetyGeometry.supportDirection(positions(:,index),yaws(index),center(1:2),center(7),dimensions);
                    end
                    normals{index}(:,targetIndex)=normal;
                    if index>1,switches=switches+double(norm(normal-previous(:,targetIndex))>1e-6);end
                    previous(:,targetIndex)=normal;
                    available=available && query.available;
                    overlaps=overlaps+double(query.signedDistance<=0);
                    minimum=min(minimum,query.signedDistance);
                end
            end
            information=struct('available',available,'overlappingNodes',overlaps, ...
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
        % Cartesian positions and physical yaws are used for geometric queries
        % and the harness's independent physical-clearance readout.
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
% One nominal column is a certified node evaluated at the target's node-time
% set; several columns are Bernstein coefficients of a whole-hold enclosure.
    frame = data.frame;origin = frame(1:2);tangent = frame(3:4);lateral = frame(5:6);
    heading = frame(7);positionError = frame(8:9);headingError = frame(10);
    stationRange = frame(11:12).';
    pose=data.pose;positionMap=reshape(pose(3:14),2,6);positionOffset=pose(1:2);
    yawOffset=pose(15);yawRow=pose(16:21).';
    localDomain=data.domain(1)>0;
    settings = data.settings;halfLength = settings(1);halfWidth = settings(2);
    headingDomain = settings(3);lateralDomain = settings(4);
    nominal = data.nominal;pointCount = size(nominal,2);
    targetCount = numel(data.targets);boundaryCount = numel(data.boundaries);
    maximumRows = 12*(targetCount+boundaryCount)+6;
    state = zeros(maximumRows,6);bound = zeros(maximumRows,pointCount);
    source = zeros(maximumRows,1);
    normals = zeros(2,targetCount);count = 0;
    for index = 1:targetCount
        target = data.targets(index);
        middle = targetPrediction.finiteFlow(target,data.duration/2);
        centerEgo = positionOffset+positionMap*mean(nominal,2);
        if nargin > 1
            normal = prescribedNormals(:,index);
        elseif isfield(data,'normals') && ~isempty(data.normals)
            normal=data.normals(:,index);
        else
            [~,normal] = avoidanceSafetyGeometry.rectangleDistance(centerEgo,yawOffset+yawRow*mean(nominal,2), ...
                middle(1:2),middle(7),[halfLength;halfWidth;target.halfLength;target.halfWidth]);
        end
        normals(:,index) = normal;
        degree = data.degree;
        if pointCount==1
            % Node certificate: the target's bounded set at the node time.
            targetPosition = target.center(1:2);
            targetRadius = target.radius(1:2);
            yawCenter = target.center(7);
            yawRadius = target.radius(7);
        else
            % Whole-hold enclosure: Bernstein coefficients of the bounded flow.
            positionPolynomial = [target.center(1:2),target.center(3:4),target.center(5:6)/2,zeros(2,degree-2)];
            radiusPolynomial = [target.radius(1:2),target.radius(3:4),target.radius(5:6)/2, ...
                target.contract.jerkBound/6,zeros(2,degree-3)];
            transform = stateUncertainty.bernsteinTransform(degree,data.duration);
            targetPosition = positionPolynomial*transform.';
            targetRadius = radiusPolynomial*transform.';
            [endCenter,endRadius] = targetPrediction.finiteFlow(target,data.duration);
            yawCenter = (target.center(7)+endCenter(7))/2;
            yawRadius = abs(endCenter(7)-target.center(7))/2+endRadius(7);
        end
        targetSupport = targetPrediction.rectangleSupport(target.halfLength,target.halfWidth,normal,yawCenter,yawRadius);
        yawCenter=heading;yawExtent=headingDomain+headingError;
        positionCharge=abs(normal).'*positionError;
        if localDomain
            yawCenter=yawOffset+yawRow(1:3)*data.domain(2:4);
            yawExtent=abs(yawRow(1:3))*data.domain(5:7)+headingError;
            positionCharge=norm(normal)*pose(22);
        end
        yawAnchor=yawOffset+yawRow*mean(nominal,2)-yawCenter;
        [egoSupport,headingSlope] = targetPrediction.rectangleSupportMajorant(normal,yawCenter, ...
            halfLength,halfWidth,yawAnchor,yawExtent);
        rowCount = numel(egoSupport);selected = count+(1:rowCount);
        state(selected,:) = repmat(-normal.'*positionMap,rowCount,1)+headingSlope*yawRow;
        bound(selected,:) = normal.'*positionOffset-normal.'*targetPosition-abs(normal).'*targetRadius ...
            -targetSupport-egoSupport-headingSlope*(yawOffset-yawCenter) ...
            -positionCharge-abs(headingSlope)*headingError;
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
        clearance = boundary.normalDistanceErrorBound*hypot(1,slope);
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
    if localDomain
        selected=count+(1:6);
        directions=[eye(3);-eye(3)];
        state(selected,:)=[directions,zeros(6,3)];
        bound(selected,:)=repmat([data.domain(5:7);data.domain(5:7)]+directions*data.domain(2:4),1,pointCount);
        source(selected)=targetCount+boundaryCount+1;count=count+6;
    end
    rows = struct("state",state(1:count,:),"bound",bound(1:count,:), ...
        "source",source(1:count),"normals",normals);
end

function result = localProjectedRows(data)
% Apply the same cell rows to condensed and local coordinates in one kernel.
% Geometric rows carry no direct input or stage-start coefficients, so those
% zero blocks are reported but never multiplied. Only plan columns the held
% cell can depend on are multiplied; every other condensed column is exactly
% zero because the cell map itself is zero there.
    tube = data.tube;
    stateRadius = data.stateRadius;pointCount = size(tube.offset,2);
    geometricRows = data.geometricRows;
    rowCount = size(geometricRows.state,1);
    stateRows = geometricRows.state;
    limits = geometricRows.bound;
    labels = geometricRows.source;
    pointStateRows = repmat(stateRows,1,1,pointCount);
    pointStartRows = zeros(rowCount,6,pointCount);
    pointInputRows = zeros(rowCount,2,pointCount);
    mapped = zeros(rowCount,size(tube.map,2),pointCount);
    active = find(any(any(tube.map~=0,1),3));
    if ~isempty(active)
        mapped(:,active,:) = pagemtimes(pointStateRows,tube.map(:,active,:));
    end
    uncertaintySupport = reshape(pagemtimes(abs(pointStateRows),reshape(stateRadius,6,1,pointCount)),[],pointCount);
    physical = limits-reshape(pagemtimes(pointStateRows,reshape(tube.offset,6,1,pointCount)),[],pointCount) ...
        -uncertaintySupport;
    localState = pagemtimes(pointStateRows,tube.localStateMap);
    localInput = pagemtimes(pointStateRows,tube.localInputMap);
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

function record=localJointRecord(target,key,stage,frame,rho,egoHalfSize,clearance,isExit,numericalTolerance)
    [pose,domain]=laneGeometry.poseData(frame);
    positionMap=reshape(pose(3:14),2,6);yawRow=pose(16:21).';
    generators=[positionMap*diag(rho),diag(target.radius(1:2))];
    positionBall=0;
    if domain(1)>0
        positionBall=pose(22);
    else
        generators=[generators,diag(frame.positionErrorBound)];
    end
    generators=generators(:,any(generators~=0,1));
    record=struct('key',string(key),'stage',stage,'isExit',isExit, ...
        'positionMap',positionMap,'positionOffset',pose(1:2)-target.center(1:2), ...
        'yawRow',yawRow,'yawOffset',pose(15), ...
        'egoHalfSize',egoHalfSize,'egoYawRadius',abs(yawRow)*rho+frame.headingErrorBound, ...
        'targetHalfSize',[target.halfLength;target.halfWidth], ...
        'targetYaw',target.center(7),'targetYawRadius',target.radius(7), ...
        'generators',generators,'positionBall',positionBall,'clearance',clearance);
    % Preserve the complete tiny numerical enclosure as one outward disk.
    radius=sum(vecnorm(record.generators));
    if radius>0 && radius<=numericalTolerance
        record.positionBall=record.positionBall+radius+16*eps(max(1,radius));
        record.generators=zeros(2,0);
    end
    radius=norm(record.egoHalfSize)*min(2,record.egoYawRadius);
    if radius>0 && radius<=numericalTolerance
        record.positionBall=record.positionBall+radius+16*eps(max(1,radius));record.egoYawRadius=0;
    end
    radius=norm(record.targetHalfSize)*min(2,record.targetYawRadius);
    if radius>0 && radius<=numericalTolerance
        record.positionBall=record.positionBall+radius+16*eps(max(1,radius));record.targetYawRadius=0;
    end
end

function states=localJointStates(program,plan)
    prediction=program.prediction;
    states=prediction.egoStateOffset+reshape(pagemtimes(prediction.egoStateMatrix,plan),6,[]);
end

function allowance=localJointAllowance(program,decision)
    states=localJointStates(program,decision(program.layout.planIndex));
    records=program.jointCertificate.records;allowance=zeros(numel(records),1);
    for index=1:numel(records)
        record=records(index);
        scale=1+record.clearance+record.positionBall+sum(record.egoHalfSize)+sum(record.targetHalfSize) ...
            +sum(vecnorm(record.generators))+norm(record.positionOffset) ...
            +norm(abs(record.positionMap)*abs(states(:,record.stage+1)));
        allowance(index)=512*(1+program.layout.planCount)*eps*scale;
    end
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
