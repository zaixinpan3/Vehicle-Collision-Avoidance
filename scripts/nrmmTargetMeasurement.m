function target = nrmmTargetMeasurement(truth, time, bound, stream)
%nrmmTargetMeasurement Noisy measurement of an NRMM truth target with its contract.
% The measurement is the truth (nrmmTargetTruth) plus independent noise drawn
% from the stream: uniform in the box for the position, yaw and yaw rate, and
% uniform in discs of the declared radii for the velocity and acceleration (the
% NRMM estimator certifies component norms, so equal per-axis bounds are
% required). It carries the nrmm-motion-v1 contract with truth.curvatureMaximum,
% truth.accelerationMaximum when finite and, when truth.publishParameterBounds
% is set, the NRMM parameter error bounds the estimator would publish from the
% disc radii (nrmmTargetParameterErrorBounds with no ego yaw error).
    arguments
        truth (1,1) struct
        time (1,1) double {mustBeNonnegative}
        bound (8,1) double {mustBeNonnegative,mustBeFinite}
        stream (1,1) RandStream
    end
    state = nrmmTargetTruth(truth,time);
    noise = bound.*(2*rand(stream,8,1)-1);
    assert(bound(3)==bound(4) && bound(5)==bound(6), ...
        "nrmmTargetMeasurement:discNoise", ...
        "NRMM truths need equal per-axis velocity and acceleration bounds.");
    for rows = {3:4,5:6}
        direction = 2*pi*rand(stream);
        fraction = sqrt(rand(stream));
        noise(rows{1}) = bound(rows{1}(1))*fraction*[cos(direction);sin(direction)];
    end
    target = struct("trackId",1,"targetPositionInertial",state(1:2)+noise(1:2), ...
        "targetVelocityInertial",state(3:4)+noise(3:4),"targetAccelerationInertial",state(5:6)+noise(5:6), ...
        "targetHeadingInertial",state(7)+noise(7),"targetYawRate",state(8)+noise(8), ...
        "targetPositionInertialErrorBound",bound(1:2),"targetVelocityInertialErrorBound",bound(3:4), ...
        "targetAccelerationInertialErrorBound",bound(5:6),"targetYawErrorBound",bound(7), ...
        "targetYawRateErrorBound",bound(8), ...
        "predictionMotion",struct("kind","nrmm-motion-v1","curvatureMaximum",truth.curvatureMaximum));
    if isfinite(truth.accelerationMaximum)
        target.predictionMotion.scalarAccelerationMaximum = truth.accelerationMaximum;
    end
    if truth.publishParameterBounds
        parameters = nrmmTargetParameterErrorBounds(target.targetVelocityInertial, ...
            target.targetAccelerationInertial,bound(3),bound(5),0);
        target.targetSpeedErrorBound = parameters.speedErrorBound;
        target.targetCourseErrorBound = parameters.courseErrorBound;
        target.targetSpeedRateErrorBound = parameters.speedRateErrorBound;
        target.targetCurvatureInterval = parameters.curvatureInterval;
    end
end
