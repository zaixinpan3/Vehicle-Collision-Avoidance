function baseline = givenPathCollisionBaseline(road, target, cfg, durationSeconds)
%givenPathCollisionBaseline Audit ideal constant-speed given-path cruise.
% Strict rectangle overlap, rather than proximity or touching, admits a threat.
% This kinematic counterfactual does not call the avoidance controller.
    arguments
        road (1,1) struct
        target double
        cfg (1,1) struct
        durationSeconds (1,1) double {mustBePositive,mustBeFinite}
    end
    if isfield(road,'referenceCurve')
        curve = road.referenceCurve;
    else
        tangent = road.centerline(end,:)-road.centerline(1,:);
        curve = struct('origin',road.centerline(1,:).', ...
            'heading',atan2(tangent(2),tangent(1)),'curvature',0,'length',norm(tangent));
    end
    start = laneGeometry.projectReferenceCurve([0;0],curve);
    sampleIntervalSeconds = cfg.controller.sampleTime/30;
    times = unique([0:sampleIntervalSeconds:durationSeconds,durationSeconds]);
    [position,heading] = laneGeometry.referencePose(start.station+cfg.referenceSpeed*times,0,curve);
    egoPoses = [position;heading];
    overlapDepth = zeros(size(times));
    signedGap = inf(size(times));
    targetPoses = zeros(3,0);
    initialClearance = Inf;
    if ~isempty(target)
        arc = target(4)*times+.5*target(5)*times.^2;
        angle = sin(target(6))/target(7)*arc/2;
        scale = ones(size(angle));
        nonzero = angle~=0;
        scale(nonzero) = sin(angle(nonzero))./angle(nonzero);
        course = target(3)+target(6)+angle;
        targetPoses = [target(1:2)+arc.*scale.*[cos(course);sin(course)];target(3)+2*angle];
        ce = cos(heading);se = sin(heading);
        ct = cos(targetPoses(3,:));st = sin(targetPoses(3,:));
        egoCenter = position+[ce*cfg.vehicle.rectangleOffset(1)-se*cfg.vehicle.rectangleOffset(2); ...
            se*cfg.vehicle.rectangleOffset(1)+ce*cfg.vehicle.rectangleOffset(2)];
        targetCenter = targetPoses(1:2,:)+[ct*target(10)-st*target(11);st*target(10)+ct*target(11)];
        delta = targetCenter-egoCenter;
        axesX = [ce;-se;ct;-st];axesY = [se;ce;st;ct];
        egoSupport = cfg.vehicle.length/2*abs(axesX.*ce+axesY.*se) ...
            +cfg.vehicle.width/2*abs(-axesX.*se+axesY.*ce);
        targetSupport = target(8)*abs(axesX.*ct+axesY.*st) ...
            +target(9)*abs(-axesX.*st+axesY.*ct);
        signedGap = max(abs(axesX.*delta(1,:)+axesY.*delta(2,:))-egoSupport-targetSupport,[],1);
        overlapDepth = max(0,-signedGap);
        shape = [cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
        initialClearance = predictiveSafetyGeometry.rectangle(egoPoses(:,1),shape,target(1:3),target(8:11));
    end
    collision = overlapDepth>1e-9;
    firstCollision = NaN;lastCollision = NaN;
    if any(collision)
        firstCollision = times(find(collision,1));lastCollision = times(find(collision,1,'last'));
    end
    baseline = struct('policy',"ideal given-path constant-speed cruise without avoidance", ...
        'referenceSpeedMetersPerSecond',cfg.referenceSpeed,'durationSeconds',durationSeconds, ...
        'sampleIntervalSeconds',sampleIntervalSeconds,'collisionDetected',any(collision), ...
        'strictOverlapSamples',nnz(collision),'firstCollisionSeconds',firstCollision, ...
        'lastCollisionSeconds',lastCollision,'maximumOverlapDepthMeters',max(overlapDepth), ...
        'initialClearanceMeters',initialClearance,'minimumSatGapMeters',min(signedGap), ...
        'referenceCurve',curve,'initialStationMeters',start.station, ...
        'times',times.','egoPoses',egoPoses.','targetPoses',targetPoses.');
end
