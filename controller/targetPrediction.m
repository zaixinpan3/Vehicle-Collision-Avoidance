classdef targetPrediction
    %targetPrediction Finite-horizon target flow from uncertain initial states.

    methods (Static)
        function enclosure = initialSet(model)
        %initialSet Map current errors into a family of fixed prediction laws.
        % Each member keeps its initial curvature and tangential acceleration.
        % The estimator's operating domain is not a future maneuver disturbance.

            prediction = model.targetPrediction;
            speed = prediction.initialSpeed;
            velocityRadius = norm(model.targetVelocityErrorBound);
            accelerationRadius = norm(model.targetAccelerationErrorBound);
            speedRange = [max(0.0, speed-velocityRadius), speed+velocityRadius];
            courseRadius = localDirectionRadius(speed, velocityRadius);
            if speedRange(2) == 0.0
                courseRadius = localDirectionRadius( ...
                    norm(model.targetAcceleration), accelerationRadius);
            end
            accelerationError = accelerationRadius ...
                + 2*norm(model.targetAcceleration)*sin(courseRadius/2);
            accelerationRange = prediction.tangentialAcceleration ...
                + [-accelerationError, accelerationError];
            yawRateRange = model.targetYawRate ...
                + [-model.targetYawRateErrorBound, model.targetYawRateErrorBound];
            if all(yawRateRange == 0.0) || speedRange(2) == 0.0
                curvatureRange = [0.0, 0.0];
            elseif speedRange(1) > 0.0
                quotients = yawRateRange(:)./speedRange;
                curvatureRange = [min(quotients, [], "all"), max(quotients, [], "all")];
            else
                curvatureRange = [-inf, inf];
            end
            enclosure = struct("kind", "fixed-prediction-initial-set-v1", ...
                "speedRange", speedRange, "courseRadius", courseRadius, ...
                "accelerationRange", accelerationRange, "curvatureRange", curvatureRange, ...
                "positionRadius", model.targetPositionErrorBound, ...
                "yawRadius", model.targetYawErrorBound);
        end

        function [positionBound, yawBound] = errorEnvelope(time, model)
        %targetPrediction.errorEnvelope Propagate current estimation error into prediction.
        % Enclose the same constant-curvature/constant-tangential-acceleration
        % flow from all currently possible initial states. No future observer
        % correction or arbitrary change of maneuver parameters is assumed.

            time = max(0.0, double(time(:).'));
            positionBound = zeros(2, numel(time));
            yawBound = zeros(1, numel(time));
            if ~model.hasTarget
                return;
            end
            enclosure = model.targetPredictionSet;
            nominal = model.targetPrediction;
            nominalArc = localArc(time, nominal.initialSpeed, nominal.tangentialAcceleration);
            arcMinimum = localArc(time, enclosure.speedRange(1), enclosure.accelerationRange(1));
            arcMaximum = localArc(time, enclosure.speedRange(2), enclosure.accelerationRange(2));
            arcError = max(abs(arcMinimum-nominalArc), abs(arcMaximum-nominalArc));
            curvatureError = max(abs(enclosure.curvatureRange-nominal.curvature));
            commonArc = min(nominalArc, arcMaximum);
            directionError = 2*commonArc;
            turnError = pi*ones(size(time));
            if isfinite(curvatureError)
                directionError = min(directionError, ...
                    enclosure.courseRadius*commonArc+0.5*curvatureError*commonArc.^2);
                turnError = curvatureError*arcMaximum+abs(nominal.curvature)*arcError;
            end
            turnError(arcMaximum == 0.0) = 0.0;
            positionBound = enclosure.positionRadius+arcError+directionError;
            yawBound = min(pi, enclosure.yawRadius+turnError);
        end

    end
end

function radius = localDirectionRadius(magnitude, errorRadius)
    radius = 0.0;
    if errorRadius > 0.0
        radius = pi;
        if errorRadius < magnitude
            radius = asin(min(1.0, errorRadius/magnitude));
        end
    end
end

function distance = localArc(time, speed, acceleration)
    if acceleration < 0.0
        time = min(time, speed/-acceleration);
    end
    distance = speed*time+0.5*acceleration*time.^2;
end
