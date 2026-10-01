function runs = runNrmmOracleTargetComparison(result, estimators)
% runNrmmOracleTargetComparison Isolate target filtering with exact ego inputs.
% Diagnostic only: all filters receive the same exact, constant ego body
% velocity, acceleration and yaw rate. Use only constant-body-velocity cases.
% Radar draws, [rho;q;s] initialization, predictor and RK4 grid are paired.
% No GNSS channel or estimated yaw enters this experiment.

    arguments
        result (1, 1) struct
        estimators (1, :) struct
    end
    truth = result.truth;
    time = truth.time;
    rotation = [cos(truth.egoYaw(1)), -sin(truth.egoYaw(1)); ...
        sin(truth.egoYaw(1)), cos(truth.egoYaw(1))];
    ego = struct("bodyVelocity", rotation.'*truth.egoVelocity(1, :).', ...
        "bodyAcceleration", truth.egoBodyAcceleration(1, :).', ...
        "yawRate", truth.egoYawRate(1));
    if max(abs(truth.egoYawAcceleration)) > 1.0e-10 ...
            || max(vecnorm(truth.egoBodyJerk, 2, 2)) > 1.0e-8 ...
            || max(abs(truth.egoSpeedDot)) > 1.0e-10
        error("runNrmmOracleTargetComparison:nonconstantEgo", ...
            "The oracle diagnostic requires constant body velocity and yaw rate.");
    end
    samplePeriod = result.config.runtime.samplePeriod;
    substeps = ceil(samplePeriod/result.config.runtime.integrationStepMaximum);
    step = samplePeriod/substeps;
    runs = struct([]);
    for estimator = estimators
        isStructured = isequal(estimator.runtime, @onlineNrmmTrackingRuntime);
        initial = result.initialEstimate.targetState;
        if isStructured
            state = initial;
        else
            state = localToCompanion(initial, ego);
        end
        estimate = NaN(numel(time), 6);
        estimate(1, :) = initial.';
        for sample = 1:numel(time)-1
            state = [state; result.measurements.radarRelativePosition(sample, :).']; %#ok<AGROW>
            available = result.measurements.radarAvailable(sample);
            for substep = 1:substeps
                k1 = localDerivative(state, ego, estimator, isStructured, available);
                k2 = localDerivative(state + 0.5*step*k1, ego, estimator, isStructured, available);
                k3 = localDerivative(state + 0.5*step*k2, ego, estimator, isStructured, available);
                k4 = localDerivative(state + step*k3, ego, estimator, isStructured, available);
                state = state + step*(k1 + 2*k2 + 2*k3 + k4)/6;
            end
            state = state(1:6);
            if ~all(isfinite(state)) || norm(state) > 1.0e9
                break
            end
            if isStructured
                estimate(sample+1, :) = state.';
            else
                [~, mapped] = sharmaNrmmCompanionDerivative( ...
                    state, ego, estimator.design.variant, estimator.design.target.speedFloor);
                estimate(sample+1, :) = [state([1, 4]); mapped.targetVelocity; mapped.zeta].';
            end
        end
        selected = time >= result.options.TransientDuration;
        stateError = estimate - truth.targetTransformedState;
        speedError = vecnorm(estimate(:, 3:4), 2, 2) - truth.targetSpeed;
        course = atan2(estimate(:, 4), estimate(:, 3)) + truth.egoYaw - truth.targetCourse;
        course = rad2deg(mod(course + pi, 2*pi) - pi);
        metrics = struct("relativePositionRmse", localRms(vecnorm(stateError(:, 1:2), 2, 2), selected), ...
            "targetSpeedRmse", localRms(speedError, selected), ...
            "targetVelocityRmse", localRms(vecnorm(stateError(:, 3:4), 2, 2), selected), ...
            "courseRmseDeg", localRms(course, selected), ...
            "diverged", any(~isfinite(estimate), "all"));
        run = struct("name", estimator.name, "metrics", metrics, "targetState", estimate);
        runs = [runs, run]; %#ok<AGROW>
    end
end

function derivative = localDerivative(state, ego, estimator, isStructured, available)
    design = estimator.design;
    if isStructured
        process = nrmmTargetTrackerDerivative(state(1:6), ego, design.target.domain);
        innovation = state(7:8) - state(1:2);
        gains = design.target.innovationGains;
        correction = [gains(1)*innovation; gains(2)*innovation; gains(3)*innovation];
        predictor = process(1:2);
    else
        process = sharmaNrmmCompanionDerivative( ...
            state(1:6), ego, design.variant, design.target.speedFloor);
        innovation = state(7:8) - state([1, 4]);
        correction = design.target.gain*innovation;
        predictor = state([2, 5]);
    end
    derivative = [process + double(available)*correction; predictor];
end

function state = localToCompanion(transformed, ego)
    cross = [0, -1; 1, 0];
    rho = transformed(1:2);
    q = transformed(3:4);
    s = transformed(5:6);
    first = q - ego.bodyVelocity - ego.yawRate*cross*rho;
    velocityDot = ego.bodyAcceleration - ego.yawRate*cross*ego.bodyVelocity;
    second = s - ego.yawRate*cross*q - velocityDot - ego.yawRate*cross*first;
    state = [rho(1); first(1); second(1); rho(2); first(2); second(2)];
end

function value = localRms(error, selected)
    value = sqrt(mean(error(selected).^2));
    if ~isfinite(value)
        value = Inf;
    end
end
