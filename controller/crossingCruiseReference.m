function reference = crossingCruiseReference(model)
%crossingCruiseReference Prefer arrival after crossing traffic clears the path.
% This is a deterministic performance reference, not a safety certificate.
% A target must leave the nominal path corridor within the prediction window
% before this rule applies. Parallel traffic and targets without such an exit
% retain the requested cruise speed. The predictive SOCP still certifies the
% complete input continuation and may use steering as required.

    reference = struct("speed", model.referenceSpeed, "yielding", false, ...
        "clearTime", 0.0, "stopStation", inf);
    if ~model.hasTarget
        return;
    end
    count = model.horizonSteps+model.tailSteps+1;
    occupied = false(1, count);
    stopStation = zeros(1, count);
    clearance = model.cfg.collision.clearanceMargin;
    for nodeIdx = 1:count
        projection = laneProjection( ...
            model.targetHorizon.targetPosition(:, nodeIdx), model.lane);
        relativeYaw = model.targetHorizon.targetYaw(nodeIdx)-projection.heading;
        lateralSupport = model.targetHalfLength*abs(sin(relativeYaw)) ...
            + model.targetHalfWidth*abs(cos(relativeYaw));
        longitudinalSupport = model.targetHalfLength*abs(cos(relativeYaw)) ...
            + model.targetHalfWidth*abs(sin(relativeYaw));
        occupied(nodeIdx) = abs(projection.lateralPosition) ...
            <= model.egoHalfWidth+lateralSupport+clearance;
        stopStation(nodeIdx) = projection.station-longitudinalSupport ...
            - model.egoHalfLength-clearance;
    end
    lastOccupied = find(occupied, 1, "last");
    if isempty(lastOccupied) || lastOccupied == count
        return;
    end
    reference.stopStation = min(stopStation(occupied));
    distance = reference.stopStation-model.initialEgoState(1);
    if distance <= 0.0
        return;
    end
    % Node one is time zero. The node following the last occupied node
    % occurs at lastOccupied*Ts. The time gap is a performance preference;
    % it is not presented as a bound on plant or prediction error.
    reference.clearTime = lastOccupied*model.sampleTime;
    reference.speed = min(reference.speed, distance ...
        / (reference.clearTime+model.cfg.performance.crossingTimeGap));
    reference.yielding = reference.speed < model.referenceSpeed;
end
