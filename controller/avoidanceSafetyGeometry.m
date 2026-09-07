function geometry = avoidanceSafetyGeometry(model, prediction)
% One separating normal per cell; every Bernstein point uses that normal.
    cfg = model.cfg;
    degree = cfg.encounter.taylorOrder+1;
    groups = cell(numel(prediction.cells), 1);
    frames = cell(numel(groups), 1);
    normalGroups = cell(numel(groups), 1);
    egoRadius = hypot(cfg.vehicle.length/2, cfg.vehicle.width/2);
    for cellIndex = 1:numel(groups)
        tube = prediction.cells(cellIndex);
        nominal = reshape(pagemtimes(tube.map, model.anchorPlan), 6, [])+tube.offset;
        station = (min(nominal(1, :))+max(nominal(1, :)))/2;
        stationRadius = cfg.controller.stationTrustRadius ...
            +(max(nominal(1, :))-min(nominal(1, :)))/2+max(tube.radius(1, :));
        frame = laneGeometry.frameBounds(model.lane, station, stationRadius, cfg.model.lateralDomainRadius);
        if frame.headingErrorBound > 128*eps
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
        limits = repmat([upper; -lower], 1, degree+1);
        safety = false(12, 1);
        labels = repmat("modelDomain", 12, 1);
        speed = max(prediction.scheduleSpeedProfile(tube.stage), cfg.model.scheduleSpeedFloor);
        slips = [0, 0, 0, 0, 1/speed, cfg.vehicle.lf/speed; ...
            0, 0, 0, 0, 1/speed, -cfg.vehicle.lr/speed];
        stateRows = [stateRows; slips; -slips]; %#ok<AGROW>
        inputRows = [inputRows; -1, 0; 0, 0; 1, 0; 0, 0]; %#ok<AGROW>
        slipLimit = cfg.model.slipAngleMaximum(:);
        if isscalar(slipLimit), slipLimit = [slipLimit; slipLimit]; end %#ok<AGROW>
        limits = [limits; repmat([slipLimit; slipLimit], 1, degree+1)]; %#ok<AGROW>
        safety = [safety; false(4, 1)]; %#ok<AGROW>
        labels = [labels; repmat("tireSlip", 4, 1)]; %#ok<AGROW>
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
        limits = [limits; cfg.encounter.corridorOverlap*ones(size(corridor, 1), degree+1)]; %#ok<AGROW>
        safety = [safety; false(size(corridor, 1), 1)]; %#ok<AGROW>
        labels = [labels; repmat("maneuverCorridor", size(corridor, 1), 1)]; %#ok<AGROW>
        normals = zeros(2, numel(model.encounters));
        for targetIndex = 1:numel(model.encounters)
            encounter = model.encounters(targetIndex);
            if encounter.discharged || tube.stage > model.exitSteps(targetIndex), continue; end
            shifted = encounter;
            [shifted.center, shifted.radius] = targetPrediction.finiteFlow(encounter, tube.start);
            [middle, ~] = targetPrediction.finiteFlow(shifted, tube.duration/2);
            centerEgo = frame.origin+[frame.tangent, frame.lateral]*mean(nominal(1:2, :), 2);
            [~, normal] = rectangleConfigurationDistance(centerEgo, frame.heading+mean(nominal(3, :)), ...
                middle(1:2), middle(7), [cfg.vehicle.length/2; cfg.vehicle.width/2; ...
                encounter.halfLength; encounter.halfWidth]);
            normals(:, targetIndex) = normal;
            positionPolynomial = [shifted.center(1:2), shifted.center(3:4), shifted.center(5:6)/2, zeros(2, degree-2)];
            radiusPolynomial = [shifted.radius(1:2), shifted.radius(3:4), shifted.radius(5:6)/2, ...
                encounter.contract.jerkBound/6, zeros(2, degree-3)];
            transform = stateUncertainty.bernsteinTransform(degree, tube.duration);
            targetPosition = positionPolynomial*transform.';
            targetRadius = radiusPolynomial*transform.';
            [endCenter, endRadius] = targetPrediction.finiteFlow(shifted, tube.duration);
            yawCenter = (shifted.center(7)+endCenter(7))/2;
            yawRadius = abs(endCenter(7)-shifted.center(7))/2+endRadius(7);
            targetSupport = targetPrediction.rectangleSupport(encounter.halfLength, ...
                encounter.halfWidth, normal, yawCenter, yawRadius);
            egoSupport = targetPrediction.rectangleSupport(cfg.vehicle.length/2, cfg.vehicle.width/2, ...
                normal, frame.heading, cfg.model.headingDomainRadius+frame.headingErrorBound);
            row = [-normal.'*[frame.tangent, frame.lateral], zeros(1, 4)];
            limit = normal.'*frame.origin-normal.'*targetPosition-abs(normal).'*targetRadius ...
                -targetSupport-egoSupport-cfg.collision.clearanceMargin ...
                -abs(normal).'*frame.positionErrorBound;
            stateRows = [stateRows; row]; %#ok<AGROW>
            inputRows = [inputRows; zeros(1, 2)]; %#ok<AGROW>
            limits = [limits; limit]; %#ok<AGROW>
            safety = [safety; true]; %#ok<AGROW>
            labels = [labels; "collision:"+encounter.key]; %#ok<AGROW>
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
            egoSupport = targetPrediction.rectangleSupport(cfg.vehicle.length/2, cfg.vehicle.width/2, ...
                normal, frame.heading, cfg.model.headingDomainRadius+frame.headingErrorBound);
            row = [-normal.'*[frame.tangent, frame.lateral], zeros(1, 4)];
            limit = normal.'*(frame.origin-boundary.origin)-graphSupport-egoSupport-clearance ...
                -abs(normal).'*frame.positionErrorBound;
            stateRows = [stateRows; row]; %#ok<AGROW>
            inputRows = [inputRows; zeros(1, 2)]; %#ok<AGROW>
            limits = [limits; repmat(limit, 1, degree+1)]; %#ok<AGROW>
            safety = [safety; true]; %#ok<AGROW>
            labels = [labels; "road:"+boundary.boundaryId]; %#ok<AGROW>
        end
        mapped = pagemtimes(stateRows, tube.map);
        for point = 1:degree+1
            mapped(:, 2*tube.stage-1:2*tube.stage, point) = ...
                mapped(:, 2*tube.stage-1:2*tube.stage, point)+inputRows;
        end
        physical = limits-stateRows*tube.offset-abs(stateRows)*tube.radius;
        groups{cellIndex} = struct("matrix", reshape(permute(mapped, [1, 3, 2]), [], prediction.planCount), ...
            "physicalBound", physical(:), "safety", repmat(safety, degree+1, 1), ...
            "label", repmat(labels, degree+1, 1), "stage", repmat(tube.stage, numel(physical), 1));
    end
    groups = vertcat(groups{:});
    geometry = struct("matrix", vertcat(groups.matrix), "physicalBound", vertcat(groups.physicalBound), ...
        "safety", vertcat(groups.safety), "label", vertcat(groups.label), "stage", vertcat(groups.stage), ...
        "frames", vertcat(frames{:}), "normals", {normalGroups});
end
