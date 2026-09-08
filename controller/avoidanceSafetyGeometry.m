function geometry = avoidanceSafetyGeometry(model, prediction)
% One separating normal per cell; every Bernstein point uses that normal.
    cfg = model.cfg;
    degree = cfg.encounter.taylorOrder+1;
    groups = cell(numel(prediction.cells), 1);
    frames = cell(numel(groups), 1);
    normalGroups = cell(numel(groups), 1);
    localGroups = cell(numel(groups),1);
    egoRadius = hypot(cfg.vehicle.length/2, cfg.vehicle.width/2);
    for cellIndex = 1:numel(groups)
        tube = prediction.cells(cellIndex);
        pointCount = size(tube.offset,2);
        robust = tube.stage<=cfg.controller.certifiedSteps;
        stateRadius = tube.numericalRadius;
        if robust, stateRadius = tube.radius; end
        [frame,nominal] = laneGeometry.sweptCellFrame(model,tube,model.anchorPlan);
        if frame.referenceHeadingErrorBound > 128*eps && ~isfield(model.lane, "referenceCurve")
            error("collisionAvoidanceController:unsupportedReferenceJump", ...
                "The finite CLF certificate requires a continuous reference chart over every cell.");
        end
        frames{cellIndex} = frame;
        lower = [frame.stationLower; -cfg.model.lateralDomainRadius; -cfg.model.headingDomainRadius; ...
            cfg.model.speedMinimum; -cfg.model.lateralVelocityMaximum; -cfg.model.yawRateMaximum];
        upper = [frame.stationUpper; cfg.model.lateralDomainRadius; cfg.model.headingDomainRadius; ...
            cfg.model.speedMaximum; cfg.model.lateralVelocityMaximum; cfg.model.yawRateMaximum];
        stateRows = [eye(6); -eye(6)];
        inputRows = zeros(12, 2);
        limits = repmat([upper; -lower], 1, pointCount);
        safety = false(12, 1);
        labels = repmat("modelDomain", 12, 1);
        speed = max(prediction.scheduleSpeedProfile(tube.stage), cfg.model.scheduleSpeedFloor);
        slips = [0, 0, 0, 0, 1/speed, cfg.vehicle.lf/speed; ...
            0, 0, 0, 0, 1/speed, -cfg.vehicle.lr/speed];
        stateRows = [stateRows; slips; -slips]; %#ok<AGROW>
        inputRows = [inputRows; -1, 0; 0, 0; 1, 0; 0, 0]; %#ok<AGROW>
        slipLimit = cfg.model.slipAngleMaximum(:);
        if isscalar(slipLimit), slipLimit = [slipLimit; slipLimit]; end %#ok<AGROW>
        limits = [limits; repmat([slipLimit; slipLimit], 1, pointCount)]; %#ok<AGROW>
        safety = [safety; false(4, 1)]; %#ok<AGROW>
        labels = [labels; repmat("tireSlip", 4, 1)]; %#ok<AGROW>
        if robust || string(cfg.model.linearizationPolicy)=="cruise"
            frictionArguments = {};
            if isfield(prediction,"tireModels") && ~isempty(prediction.tireModels{tube.stage})
                frictionArguments = prediction.tireModels(tube.stage);
            end
            friction = modifiedFialaTire.frictionCirclePolygonRows(prediction.scheduleCurvature(tube.stage), ...
                prediction.scheduleSpeedProfile(tube.stage),prediction.scheduleBrakingRatio(tube.stage),cfg,frictionArguments{:});
            stateRows = [stateRows;friction.state]; %#ok<AGROW>
            inputRows = [inputRows;friction.input]; %#ok<AGROW>
            limits = [limits;repmat(friction.bound,1,pointCount)]; %#ok<AGROW>
            safety = [safety;false(numel(friction.bound),1)]; %#ok<AGROW>
            labels = [labels;repmat("combinedTireForce",numel(friction.bound),1)]; %#ok<AGROW>
        end
        % Future trajectory stages use the nonlinear Fiala force law itself:
        % |Fy| <= mu*Fz*sqrt(1-beta^2). An affine inner polygon there adds
        % artificial limits that are not the physical circle. Frozen-model
        % execution stages retain their requested-force polygon.
        corridor = [0, 1, 0, 0, 0, 0];
        switch model.maneuver
            case "track"
                corridor = zeros(0, 6);
            case "passLeft"
                corridor = -corridor;
            case "yield"
                corridor = [corridor; -corridor]; %#ok<AGROW>
        end
        stateRows = [stateRows; corridor]; %#ok<AGROW>
        inputRows = [inputRows; zeros(size(corridor, 1), 2)]; %#ok<AGROW>
        limits = [limits; cfg.encounter.corridorOverlap*ones(size(corridor, 1), pointCount)]; %#ok<AGROW>
        safety = [safety; false(size(corridor, 1), 1)]; %#ok<AGROW>
        labels = [labels; repmat("maneuverCorridor", size(corridor, 1), 1)]; %#ok<AGROW>
        targetAnticipation = zeros(size(safety));
        normals = zeros(2, numel(model.encounters));
        for targetIndex = 1:numel(model.encounters)
            encounter = model.encounters(targetIndex);
            if encounter.discharged || tube.stage > model.exitSteps(targetIndex), continue; end
            shifted = encounter;
            [shifted.center, shifted.radius] = targetPrediction.finiteFlow(encounter, tube.start);
            if ~robust && targetPrediction.isFiniteSensing(encounter)
                [shifted.center,jerk,yawAcceleration] = targetPrediction.nominalFlow(encounter,tube.start);
                shifted.radius(:) = 0;
                shifted.contract.jerkBound = repmat(jerk,2,1);
                shifted.contract.yawAccelerationBound = yawAcceleration;
            end
            [middle, ~] = targetPrediction.finiteFlow(shifted, tube.duration/2);
            centerEgo = frame.origin+[frame.tangent, frame.lateral]*mean(nominal(1:2, :), 2);
            [~, normal] = rectangleConfigurationDistance(centerEgo, frame.heading+mean(nominal(3, :)), ...
                middle(1:2), middle(7), [cfg.vehicle.length/2; cfg.vehicle.width/2; ...
                encounter.halfLength; encounter.halfWidth]);
            normals(:, targetIndex) = normal;
            positionPolynomial = [shifted.center(1:2), shifted.center(3:4), shifted.center(5:6)/2, zeros(2, degree-2)];
            radiusPolynomial = [shifted.radius(1:2), shifted.radius(3:4), shifted.radius(5:6)/2, ...
                shifted.contract.jerkBound/6, zeros(2, degree-3)];
            transform = stateUncertainty.bernsteinTransform(degree, tube.duration);
            if pointCount==1, transform = transform(1,:); end
            if ~robust,transform = transform([1,end],:);end
            targetPosition = positionPolynomial*transform.';
            targetRadius = radiusPolynomial*transform.';
            [endCenter, endRadius] = targetPrediction.finiteFlow(shifted, tube.duration);
            if ~robust && targetPrediction.isFiniteSensing(encounter)
                endCenter = targetPrediction.nominalFlow(encounter,tube.start+tube.duration);
                targetPosition = [shifted.center(1:2),endCenter(1:2)];
                targetAcceleration = max(norm(shifted.center(5:6)),norm(endCenter(5:6)));
                targetRadius = repmat(targetAcceleration*tube.duration^2/8,2,2);
            end
            yawCenter = (shifted.center(7)+endCenter(7))/2;
            yawRadius = abs(endCenter(7)-shifted.center(7))/2+endRadius(7);
            targetSupport = targetPrediction.rectangleSupport(encounter.halfLength, ...
                encounter.halfWidth, normal, yawCenter, yawRadius);
            targetAllowance = 0;
            if ~robust && targetPrediction.isFiniteSensing(encounter) && any(encounter.radius)
                % Future nominal footprints must also anticipate the target
                % enclosure that the next measurement will make relevant.
                % This is an allocated planning reserve, not a certificate
                % of the unknown future observer errors.
                centered = encounter;
                displacement = encounter.center-encounter.nominalCenter;
                displacement(7) = atan2(sin(displacement(7)),cos(displacement(7)));
                centered.radius = encounter.radius+abs(displacement);
                [~,nextRadius] = targetPrediction.finiteFlow(centered,model.sampleTime);
                uncertainSupport = targetPrediction.rectangleSupport(encounter.halfLength, ...
                    encounter.halfWidth,normal,yawCenter,yawRadius+nextRadius(7));
                targetAllowance = abs(normal).'*nextRadius(1:2) ...
                    +max(0,uncertainSupport-targetSupport);
            end
            [egoSupport,headingSlope] = targetPrediction.rectangleSupportMajorant(normal,frame.heading, ...
                cfg.vehicle.length/2,cfg.vehicle.width/2,mean(nominal(3,:)), ...
                cfg.model.headingDomainRadius+frame.headingErrorBound);
            rowCount = numel(egoSupport);
            row = repmat([-normal.'*[frame.tangent,frame.lateral],zeros(1,4)],rowCount,1);
            row(:,3) = headingSlope;
            limit = normal.'*frame.origin-normal.'*targetPosition-abs(normal).'*targetRadius ...
                -targetSupport-egoSupport-cfg.collision.clearanceMargin ...
                -abs(normal).'*frame.positionErrorBound-abs(headingSlope)*frame.headingErrorBound;
            stateRows = [stateRows; row]; %#ok<AGROW>
            inputRows = [inputRows; zeros(rowCount, 2)]; %#ok<AGROW>
            limits = [limits; limit]; %#ok<AGROW>
            safety = [safety; true(rowCount,1)]; %#ok<AGROW>
            labels = [labels; repmat("collision:"+encounter.key,rowCount,1)]; %#ok<AGROW>
            targetAnticipation = [targetAnticipation;repmat(targetAllowance,rowCount,1)]; %#ok<AGROW>
        end
        normalGroups{cellIndex} = normals;
        for boundaryIndex = 1:numel(model.road.boundaries)
            boundary = model.road.boundaries(boundaryIndex);
            longitudinal = boundary.longitudinalDirection;
            stations = longitudinal.'*frame.tangent*[frame.stationLower, frame.stationUpper];
            extent = abs(longitudinal.'*frame.lateral)*cfg.model.lateralDomainRadius ...
                +egoRadius+abs(longitudinal).'*frame.positionErrorBound;
            range = [min(stations)-extent, max(stations)+extent] ...
                +longitudinal.'*(frame.origin-boundary.origin);
            if range(1) < boundary.parameterRange(1) || range(2) > boundary.parameterRange(2)
                error("collisionAvoidanceController:roadBoundaryCoverageGap", ...
                    "Boundary %s must cover the complete certified cell.", boundary.boundaryId);
            end
            polynomial = boundary.safeSideSign*boundary.coefficients;
            candidates = range;
            if polynomial(1) < 0
                candidates(3) = min(max(-polynomial(2)/(2*polynomial(1)), range(1)), range(2));
            end
            graphSupport = max(polyval(polynomial, candidates));
            normal = boundary.safeSideSign*boundary.lateralDirection;
            slope = max(abs(2*boundary.coefficients(1)*range+boundary.coefficients(2)));
            clearance = (cfg.collision.clearanceMargin+boundary.normalDistanceErrorBound)*hypot(1, slope);
            [egoSupport,headingSlope] = targetPrediction.rectangleSupportMajorant(normal,frame.heading, ...
                cfg.vehicle.length/2,cfg.vehicle.width/2,mean(nominal(3,:)), ...
                cfg.model.headingDomainRadius+frame.headingErrorBound);
            rowCount = numel(egoSupport);
            row = repmat([-normal.'*[frame.tangent,frame.lateral],zeros(1,4)],rowCount,1);
            row(:,3) = headingSlope;
            limit = normal.'*(frame.origin-boundary.origin)-graphSupport-egoSupport-clearance ...
                -abs(normal).'*frame.positionErrorBound-abs(headingSlope)*frame.headingErrorBound;
            stateRows = [stateRows; row]; %#ok<AGROW>
            inputRows = [inputRows; zeros(rowCount, 2)]; %#ok<AGROW>
            limits = [limits; repmat(limit, 1, pointCount)]; %#ok<AGROW>
            safety = [safety; true(rowCount,1)]; %#ok<AGROW>
            labels = [labels; repmat("road:"+boundary.boundaryId,rowCount,1)]; %#ok<AGROW>
            targetAnticipation = [targetAnticipation;zeros(rowCount,1)]; %#ok<AGROW>
        end
        pointStateRows = repmat(stateRows,1,1,pointCount);
        pointStartRows = zeros(size(pointStateRows));
        pointInputRows = repmat(inputRows,1,1,pointCount);
        if ~robust && string(cfg.model.linearizationPolicy)~="cruise"
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
        if ~robust && string(cfg.model.linearizationPolicy)~="cruise"
            % Prepare future state domains for consecutive uncertain steps.
            % Resetting this allowance at every future stage lets the plan
            % approach a boundary before rate-limited inputs can turn back.
            domainRows = labels=="modelDomain" | labels=="tireSlip";
            % Station limits delimit the local coordinate chart, not a
            % vehicle-state operating domain. Its nominal swept frame is
            % constructed separately from the endpoint trajectory.
            domainRows([1,7]) = false;
            executionSupport = reshape(pagemtimes(abs(pointStateRows), ...
                reshape(prediction.domainErrorBound(:,tube.stage:tube.stage+1),6,1,pointCount)),[],pointCount);
            executionSupport = executionSupport+reshape(pagemtimes(abs(pointStartRows),prediction.domainErrorBound(:,tube.stage)),[],pointCount);
            initialSupport = reshape(pagemtimes(abs(pointStateRows), ...
                reshape(prediction.initialErrorBound(:,tube.stage:tube.stage+1),6,1,pointCount)),[],pointCount);
            initialSupport = initialSupport+reshape(pagemtimes(abs(pointStartRows),prediction.initialErrorBound(:,tube.stage)),[],pointCount);
            initialReserve(domainRows,:) = initialSupport(domainRows,:);
            domainReserve(domainRows,:) = max(0,executionSupport(domainRows,:)-initialSupport(domainRows,:));
        end
        % The force polygon limits the nominal linearization's requested
        % force. Actual tire-force variation belongs to the declared plant
        % residual; road and collision rows retain full state tightening.
        forceRows = labels=="combinedTireForce";
        uncertaintySupport(forceRows,:) = abs(stateRows(forceRows,:))*tube.numericalRadius;
        physical = limits-reshape(pagemtimes(pointStateRows,reshape(tube.offset,6,1,pointCount)),[],pointCount) ...
            -reshape(pagemtimes(pointStartRows,tube.offset(:,1)),[],pointCount)-uncertaintySupport;
        anticipationReserve = zeros(size(safety));
        if ~robust && string(cfg.model.linearizationPolicy)~="cruise"
            support = reshape(pagemtimes(abs(pointStateRows), ...
                reshape(prediction.domainErrorBound(:,tube.stage:tube.stage+1),6,1,pointCount)),[],pointCount);
            anticipationReserve = (max(support,[],2)+cfg.encounter.nominalLinearizationReserve).*double(safety)+targetAnticipation;
        end
        localState = pagemtimes(pointStateRows,tube.localStateMap)+pointStartRows;
        localInput = pagemtimes(pointStateRows,tube.localInputMap)+pointInputRows;
        localBound = limits-reshape(pagemtimes(pointStateRows,reshape(tube.localOffset,6,1,pointCount)),[],pointCount)-uncertaintySupport;
        localGroups{cellIndex} = struct("stateMatrix",reshape(permute(localState,[1,3,2]),[],6), ...
            "inputMatrix",reshape(permute(localInput,[1,3,2]),[],2),"bound",localBound(:),"stage",tube.stage, ...
            "nodeStateRows",pointStateRows,"nodeInputRows",pointInputRows,"nodeLimits",limits-uncertaintySupport-initialReserve, ...
            "nodeStartStateRows",pointStartRows, ...
            "nodeInitialReserve",initialReserve, ...
            "nodeLabels",labels);
        groups{cellIndex} = struct("matrix", reshape(permute(mapped, [1, 3, 2]), [], prediction.planCount), ...
            "physicalBound", physical(:), "domainReserve",domainReserve(:),"initialReserve",initialReserve(:), ...
            "anticipationReserve",repmat(anticipationReserve,pointCount,1), ...
            "safety", repmat(safety, pointCount, 1), ...
            "label", repmat(labels, pointCount, 1), "stage", repmat(tube.stage, numel(physical), 1));
    end
    groups = vertcat(groups{:});
    geometry = struct("matrix", vertcat(groups.matrix), "physicalBound", vertcat(groups.physicalBound), ...
        "domainReserve",vertcat(groups.domainReserve), ...
        "initialReserve",vertcat(groups.initialReserve), ...
        "anticipationReserve",vertcat(groups.anticipationReserve), ...
        "safety", vertcat(groups.safety), "label", vertcat(groups.label), "stage", vertcat(groups.stage), ...
        "frames", vertcat(frames{:}), "normals", {normalGroups},"local",vertcat(localGroups{:}));
end
