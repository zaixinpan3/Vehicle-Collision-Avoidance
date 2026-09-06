function [positionBound, yawBound] = targetPredictionErrorEnvelope(time, model)
%targetPredictionErrorEnvelope Propagate current estimation error into prediction.
% The estimator's B(t) concerns today's estimated state. Future truth is
% enclosed around the controller's fixed nominal trajectory using declared
% motion bounds, without assuming future measurement corrections.

    time = max(0.0, double(time(:).'));
    positionBound = zeros(2, numel(time));
    yawBound = zeros(1, numel(time));
    if ~model.hasTarget
        return;
    end
    if ~isfield(model, "targetPredictionMotionBounds") ...
            || isempty(model.targetPredictionMotionBounds)
        positionBound = model.targetPositionErrorBound ...
            + model.targetVelocityErrorBound*time ...
            + 0.5*model.targetPredictionAccelerationErrorBound*time.^2;
        yawBound = model.targetYawErrorBound ...
            + model.targetYawRateErrorBound*time ...
            + 0.5*model.targetPredictionYawAccelerationErrorBound*time.^2;
        return;
    end
    motion = model.targetPredictionMotionBounds;
    propagationTime = min(time, model.targetStopTime);
    nominalSpeed = max(model.targetSpeed, ...
        max(0.0, model.targetSpeed+model.targetTangentialAcceleration*propagationTime));
    nominalAcceleration = abs(model.targetTangentialAcceleration) ...
        + abs(model.targetCurvature)*nominalSpeed.^2;
    accelerationEnvelope = 0.5*(motion.accelerationNormMaximum ...
        + nominalAcceleration).*time.^2;
    nominalJerk = model.targetCurvature^2*nominalSpeed.^3 ...
        + 3*abs(model.targetCurvature*model.targetTangentialAcceleration)*nominalSpeed;
    initialAccelerationError = norm(model.targetAccelerationErrorBound) ...
        + norm(model.targetPredictionAccelerationErrorBound);
    jerkEnvelope = 0.5*initialAccelerationError*time.^2 ...
        + (motion.jerkNormMaximum+nominalJerk).*time.^3/6.0;
    % The nominal stopping rule has an acceleration jump. The acceleration
    % enclosure still holds there; a smooth-jerk remainder does not.
    jerkEnvelope(time > model.targetStopTime) = inf;
    velocityEnvelope = model.targetVelocityErrorBound*time ...
        + min(accelerationEnvelope, jerkEnvelope);
    speedEnvelope = (motion.speedMaximum+nominalSpeed).*time;
    positionBound = model.targetPositionErrorBound+min(velocityEnvelope, speedEnvelope);
    yawBound = min(pi, model.targetYawErrorBound ...
        + (motion.yawRateMaximum+abs(model.targetCurvature)*nominalSpeed).*time);
end
