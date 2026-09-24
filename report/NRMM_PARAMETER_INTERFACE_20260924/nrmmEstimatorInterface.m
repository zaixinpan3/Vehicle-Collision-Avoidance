function table_ = nrmmEstimatorInterface(outputFile)
%nrmmEstimatorInterface Target box from the estimator's inertial box alone and
% with its published NRMM parameter error bounds.
% Offline diagnostic, run from the repository root. It rebuilds the target
% reconstruction fixture of tests/nrmmControllerErrorBoundsTest.m (rotated,
% angle-branch, velocity-peaking and near-rest geometries; the tracking
% configuration's design) and publishes the estimate through
% nrmmControllerErrorBounds. For each geometry it admits the published record
% into the controller's NRMM prediction twice: from the inertial box alone
% (the published parameter bounds removed) and with them. It reports the NRMM
% parameter half-widths and the position half-width per axis of the reachable
% box at 1, 2, 3, 4 and 4.8 s.
    addpath("controller","config","estimator");
    design = synthesizeNrmmObserverGains(nrmmTrackingConfig());
    geometries = struct("rotated",[1.2;0.03;0],"angleBranch",[3.13;-0.04;0], ...
        "velocityPeaking",[-0.7;0.2;1],"nearZeroVelocity",[0.6;-0.2;2]);
    cfg = collisionAvoidanceControllerConfig();
    times = [1,2,3,4,4.8];rows = {};
    for name = string(fieldnames(geometries)).'
        published = localReconstruction(design,geometries.(name));
        record = published.targetEstimate;record.trackId = 1;record.stateTime = 0;
        for variant = ["box-only","with published bounds"]
            data = record;
            if variant=="box-only"
                data = rmfield(data,["targetSpeedErrorBound","targetCourseErrorBound", ...
                    "targetSpeedRateErrorBound","targetCurvatureInterval"]);
            end
            ego = struct("position",published.egoPositionInertial,"yaw",0,"speed",12,"stateTime",0, ...
                "controllerStateErrorBound",zeros(6,1), ...
                "perception",struct("time",0,"range",60,"completeWithinRange",true));
            [~,lane,~,parsed] = readPlanningInputs(ego,data,[-100,0;2000,0],cfg);
            encounter = targetPrediction.admit(parsed,0,lane,cfg);
            widths = structfun(@(interval) diff(interval)/2,encounter.parameters).';
            [~,radius] = targetPrediction.finiteFlow(encounter,times);
            rows(end+1,:) = [{name,variant},num2cell(widths),num2cell(max(radius(1:2,:),[],1))]; %#ok<AGROW>
            fprintf("%-16s %-22s | half-widths V %.3f m/s, course %.3f rad, A %.3f m/s^2, kappa %.4f 1/m" ...
                +" | position box (m) at 1/2/3/4/4.8 s: %s\n", ...
                name,variant,widths,sprintf("%6.2f ",max(radius(1:2,:),[],1)));
        end
    end
    table_ = cell2table(rows,"VariableNames",["geometry","variant","speedHalfWidth","courseHalfWidth", ...
        "speedRateHalfWidth","curvatureHalfWidth","box1s","box2s","box3s","box4s","box4p8s"]);
    if nargin>0,writetable(table_,outputFile);end
end

function published = localReconstruction(design, geometry)
% The fixture of tests/nrmmControllerErrorBoundsTest.m.
    yaw = geometry(1);
    estimatedYaw = yaw+geometry(2);
    rotation = localRotation(yaw);
    estimatedRotation = localRotation(estimatedYaw);
    position = [4; -2];
    estimatedPosition = position+[0.03; -0.02];
    egoVelocity = [12; 0.1];
    estimatedEgoVelocity = egoVelocity+[0.04; -0.08];
    targetPosition = [25; 5];
    speed = 0.5*(design.target.domain.speedMinimum+design.target.domain.speedMaximum);
    course = -3.13;
    domain = design.target.domain;
    yawRate = min(0.25*domain.yawRateMaximum, ...
        0.5*sin(domain.sideslipMaximum)*speed/domain.rearAxleDistance);
    scalarAcceleration = 0.2*domain.scalarAccelerationMaximum;
    direction = [cos(course); sin(course)];
    targetVelocity = speed*direction;
    targetAcceleration = scalarAcceleration*direction+yawRate*speed*[-direction(2); direction(1)];
    actual = [rotation.'*(targetPosition-position); ...
        rotation.'*targetVelocity; rotation.'*targetAcceleration];
    estimated = actual+[0.1; -0.15; 0.08; -0.1; 0.04; 0.06];
    switch geometry(3)
        case 1
            estimated(3:4) = [100; -120];
            estimated(5:6) = [-30; 50];
        case 2
            estimated(3:4) = [1e-6; -1e-6];
    end
    [~, target] = nrmmTargetTrackerDerivative(estimated, ...
        struct("bodyVelocity", estimatedEgoVelocity, "yawRate", 0), domain);
    target.targetPositionInertial = estimatedPosition+estimatedRotation*estimated(1:2);
    target.targetVelocityInertial = estimatedRotation*estimated(3:4);
    target.targetAccelerationInertial = estimatedRotation*estimated(5:6);
    target.targetHeadingInertial = estimatedYaw+target.targetCourseAngleEgoFrame-target.targetSideslip;
    output = struct("stateTime", 0, "egoPositionInertial", estimatedPosition, ...
        "targetEstimate", target);
    components = vecnorm(reshape(actual-estimated, 2, 3)).';
    bound = struct("yaw", abs(geometry(2)), "bodyVelocity", norm(egoVelocity-estimatedEgoVelocity), ...
        "targetComponents", components, "trueRangeMaximum", norm(actual(1:2)), ...
        "egoValid", true, "valid", true, "scope", "declared test enclosure", ...
        "holdBounds", struct("yawAcceleration", 0), ...
        "orientationSet",nrmmYawSet("initialize",estimatedYaw,abs(geometry(2))));
    input = struct("time", 0, "yawRate", 0, "gnssPosition", ...
        position+design.sensors.positionNoiseMaximum*[1; 1]/sqrt(2));
    published = nrmmControllerErrorBounds(output, bound, input, design);
end

function rotation = localRotation(yaw)
    rotation = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
end
