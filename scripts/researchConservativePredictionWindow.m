function window = researchConservativePredictionWindow(cfg, speedUpper, prefixSteps, frictionUpper, decelerationLower)
%researchConservativePredictionWindow Allocate the declared analytic stopping tail.
% Research helper, not wired into the controller. speedUpper bounds current
% hypot(vx,vy); frictionUpper bounds both actual axle friction coefficients.
% decelerationLower is a positive bound for the DECLARED terminal model, not
% a braking-force ratio or a deceleration inferred from the Fiala plant.
% The prefix must stay in the configured forward speed domain with passive
% road load and the existing combined-slip tire model. See the research report
% for the real-arithmetic proof and the distinction from prefix reachability.
    arguments
        cfg (1,1) struct
        speedUpper (1,1) double {mustBeReal, mustBeFinite, mustBeNonnegative}
        prefixSteps (1,1) double {mustBeReal, mustBeFinite, mustBeInteger, mustBeNonnegative}
        frictionUpper (2,1) double {mustBeReal, mustBeFinite, mustBePositive}
        decelerationLower (1,1) double {mustBeReal, mustBeFinite, mustBePositive}
    end
    h = cfg.controller.sampleTime;
    vxMaximum = cfg.model.speedMaximum;
    vyMaximum = cfg.model.lateralVelocityMaximum;
    blendMaximum = cfg.model.kinematicSpeedRange(2);
    roadLoad = cfg.roadLoad;
    passive = [roadLoad.airDensity, roadLoad.dragCoefficient, ...
        roadLoad.frontalArea, roadLoad.rollingCoefficient, ...
        roadLoad.rollingSpeedCoefficient, roadLoad.rollingQuarticCoefficient];
    assert(cfg.model.speedMinimum >= 0 && all(passive >= 0), ...
        'collisionAvoidanceResearch:windowContract', ...
        'The speed envelope requires forward travel and passive road load.');
    validateattributes([h, vxMaximum, vyMaximum, blendMaximum, ...
        cfg.vehicle.gravity, cfg.vehicle.lf, cfg.vehicle.lr, ...
        roadLoad.rollingTransitionSpeed], {'double'}, {'positive', 'finite'});
    speedCap = hypot(vxMaximum, vyMaximum);
    blendSpeedCap = hypot(blendMaximum, vyMaximum);
    accelerationUpper = cfg.vehicle.gravity * ...
        (frictionUpper(1)*cfg.vehicle.lr + frictionUpper(2)*cfg.vehicle.lf) / ...
        (cfg.vehicle.lf + cfg.vehicle.lr);
    prefixSeconds = prefixSteps*h;
    if prefixSteps == 0
        entrySpeedUpper = min(speedCap, speedUpper);
    else
        entrySpeedUpper = min(speedCap, ...
            max(speedUpper, blendSpeedCap) + accelerationUpper*prefixSeconds);
    end
    tailSteps = localRoundUp(entrySpeedUpper, decelerationLower, h);
    domainTailSteps = localRoundUp(speedCap, decelerationLower, h);
    window = struct('prefixSteps', prefixSteps, 'prefixSeconds', prefixSeconds, ...
        'entrySpeedUpperMetersPerSecond', entrySpeedUpper, ...
        'domainSpeedUpperMetersPerSecond', speedCap, ...
        'blendSpeedUpperMetersPerSecond', blendSpeedCap, ...
        'accelerationUpperMetersPerSecondSquared', accelerationUpper, ...
        'decelerationLowerMetersPerSecondSquared', decelerationLower, ...
        'tailSteps', tailSteps, 'tailSeconds', tailSteps*h, ...
        'totalSteps', prefixSteps + tailSteps, ...
        'totalSeconds', (prefixSteps + tailSteps)*h, ...
        'domainTailSteps', domainTailSteps, ...
        'domainTotalSeconds', (prefixSteps + domainTailSteps)*h);
end

function steps = localRoundUp(speed, deceleration, sampleTime)
    steps = ceil(speed/(deceleration*sampleTime));
    % Guard the final product against ordinary floating-point roundoff.
    % This is not directed-rounding interval certification of the inputs.
    if steps*sampleTime*deceleration < speed
        steps = steps + 1;
    end
end
