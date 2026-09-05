function information = nrmmBodyVelocitySet(velocityGnss, yawRate, cfg)
% nrmmBodyVelocitySet Yaw-independent body velocity and an outer yaw arc.
% Intersect the GNSS speed annulus, single-track lateral strip, and forward
% sideslip domain. The representative is feasible even if the central inverse
% is inadmissible. Empty information is reported without clipping a square root.

    arguments
        velocityGnss (2, 1) double {mustBeFinite}
        yawRate (1, 1) double {mustBeFinite}
        cfg (1, 1) struct
    end
    speedMeasured = norm(velocityGnss);
    noise = cfg.measurement.gps.velocityNoiseMaximum;
    betaMax = cfg.ego.yaw.sideslipDomainMaximum;
    lateral = cfg.ego.yaw.rearAxleDistance*yawRate;
    epsilon = cfg.ego.yaw.rearAxleDistance ...
        *(cfg.measurement.gyroscope.noiseMaximum ...
        +cfg.ego.yaw.singleTrackYawRateMismatchMaximum);
    speedLow = max(cfg.ego.domain.speedMinimum, speedMeasured-noise);
    speedHigh = min(cfg.ego.domain.speedMaximum, speedMeasured+noise);
    yLow = max(lateral-epsilon, -speedHigh*sin(betaMax));
    yHigh = min(lateral+epsilon, speedHigh*sin(betaMax));
    information = struct("nonempty", false, "representative", [NaN; NaN], ...
        "lower", [Inf; Inf], "upper", [-Inf; -Inf], "radius", Inf, ...
        "yawArcs", zeros(0, 2), "nominalInverseAdmissible", false, ...
        "speedInterval", [speedLow, speedHigh], "sideslipInterval", [NaN, NaN]);
    if speedLow > speedHigh || yLow > yHigh
        return
    end
    y = min(max(lateral, yLow), yHigh);
    if betaMax == 0
        minimumFeasibleSpeed = speedLow;
    else
        minimumFeasibleSpeed = max(speedLow, abs(y)/sin(betaMax));
    end
    speed = min(max(speedMeasured, minimumFeasibleSpeed), speedHigh);
    if speed < abs(y)
        return
    end
    representative = [sqrt(speed^2-y^2); y];
    maximumY = max(abs([yLow, yHigh]));
    minimumY = max([yLow, -yHigh, 0]);
    lower = [max(speedLow*cos(betaMax), ...
        sqrt(max(0, speedLow^2-maximumY^2))); yLow];
    upper = [sqrt(max(0, speedHigh^2-minimumY^2)); yHigh];
    padding = cfg.window.numericalAllowance;
    lower = lower-padding;
    upper = upper+padding;
    sineLimits = [yLow, yHigh]./[speedLow; speedHigh];
    betaLow = max(-betaMax, asin(max(-1, min(sineLimits, [], "all"))));
    betaHigh = min(betaMax, asin(min(1, max(sineLimits, [], "all"))));
    if speedMeasured > noise
        courseRadius = asin(min(1, noise/speedMeasured));
    else
        courseRadius = pi;
    end
    course = atan2(velocityGnss(2), velocityGnss(1));
    yawCentre = course-(betaLow+betaHigh)/2;
    yawRadius = courseRadius+(betaHigh-betaLow)/2+padding;
    information.nonempty = true;
    information.representative = representative;
    information.lower = lower;
    information.upper = upper;
    information.radius = norm(max(abs(representative-lower), abs(upper-representative)));
    information.yawArcs = nrmmCircularSet("arc", yawCentre, yawRadius);
    information.nominalInverseAdmissible = speedMeasured >= speedLow ...
        && speedMeasured <= speedHigh && abs(lateral) <= speedMeasured*sin(betaMax);
    information.sideslipInterval = [betaLow, betaHigh];
end
