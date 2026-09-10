classdef targetPrediction
    %targetPrediction Finite-horizon target flow from uncertain initial states.

    methods (Static)
        function [offset,slope] = rectangleSupportMajorant(normal,referenceHeading,halfLength,halfWidth,anchor,errorMaximum)
        % Touching convex majorant of rectangle support over a yaw interval.
        % Every vertex projection is concave wherever it is nonnegative.
        % Tangents there cover every possible support-maximizing vertex.
            rotation = [cos(referenceHeading),-sin(referenceHeading); ...
                sin(referenceHeading),cos(referenceHeading)];
            localNormal = rotation.'*normal;
            vertices = [halfLength,halfLength,-halfLength,-halfLength; ...
                halfWidth,-halfWidth,halfWidth,-halfWidth];
            a = localNormal.'*vertices;
            b = localNormal.'*[-vertices(2,:);vertices(1,:)];
            phase = atan2(b,a);
            periods = 2*pi*(-1:1).';
            lower = max(-errorMaximum,phase-pi/2+periods);
            upper = min(errorMaximum,phase+pi/2+periods);
            point = min(max(anchor,lower),upper);
            derivative = -a.*sin(point)+b.*cos(point);
            value = a.*cos(point)+b.*sin(point);
            intercept = value-derivative.*point+64*eps*(1+abs(a)+abs(b));
            retained = lower<=upper;
            offset = intercept(retained);slope = derivative(retained);
        end
        function finite = isFiniteSensing(encounter)
            finite = string(encounter.contract.kind) == "finite-sensing-motion-v1";
        end

        function encounter = admit(target, time, ~, cfg)
        %admit Validate the finite motion bounds used by every certificate.
            if isfield(target,"predictionMotion") && ~isempty(target.predictionMotion)
                motion = target.predictionMotion;
                if ~isstruct(motion) || ~isscalar(motion) ...
                        || ~all(isfield(motion,["kind","jerkBound","yawAccelerationBound"])) ...
                        || string(motion.kind) ~= "finite-sensing-motion-v1"
                    error("collisionAvoidanceController:invalidEncounterContract","Invalid finite motion bounds.");
                end
                contract = struct("kind","finite-sensing-motion-v1","id",target.key, ...
                    "validFrom",time,"validityScope","whileEncounterActive", ...
                    "jerkBound",motion.jerkBound,"yawAccelerationBound",motion.yawAccelerationBound, ...
                    "predictionSampleTime",cfg.controller.sampleTime);
                if isfield(motion,"scalarAccelerationMaximum")
                    validateattributes(motion.scalarAccelerationMaximum,{'double'},{'scalar','finite','nonnegative'});
                    contract.scalarAccelerationMaximum = motion.scalarAccelerationMaximum;
                end
                validateattributes(contract.jerkBound,{'double'},{'real','finite','nonnegative','numel',2});
                validateattributes(contract.yawAccelerationBound,{'double'},{'real','finite','nonnegative','scalar'});
                validateattributes(time,{'double'},{'real','finite','scalar'});
                if target.predictionYawAccelerationErrorBound>contract.yawAccelerationBound
                    error("collisionAvoidanceController:invalidEncounterContract","Yaw acceleration exceeds the motion bound.");
                end
                if startsWith(target.key,"anonymousTarget:")
                    error("collisionAvoidanceController:invalidEncounterContract","Finite encounters need stable track identity.");
                end
                contract.jerkBound = contract.jerkBound(:);
                encounter = localEncounter(target,time,contract);
                return;
            end
            error("collisionAvoidanceController:missingPredictionMotion", ...
                "Every target requires identified finite-sensing motion bounds.");
        end

        function [center, radius] = finiteFlow(encounter, duration)
        %finiteFlow Positive Cartesian reachability; no speed division.
            duration = double(duration(:).');
            if any(~isfinite(duration) | duration < 0)
                error("collisionAvoidanceController:invalidPredictionTime", "Prediction times must be finite and nonnegative.");
            end
            x = encounter.center;
            r = encounter.radius;
            jerk = encounter.contract.jerkBound;
            yawAcceleration = encounter.contract.yawAccelerationBound;
            center = [x(1:2)+x(3:4)*duration+x(5:6)*(duration.^2/2); ...
                x(3:4)+x(5:6)*duration; repmat(x(5:6), 1, numel(duration)); ...
                x(7)+x(8)*duration; repmat(x(8), 1, numel(duration))];
            if nargout<2,return;end
            radius = [r(1:2)+r(3:4)*duration+r(5:6)*(duration.^2/2)+jerk*(duration.^3/6); ...
                r(3:4)+r(5:6)*duration+jerk*(duration.^2/2); ...
                r(5:6)+jerk*duration; r(7)+r(8)*duration+yawAcceleration*(duration.^2/2); ...
                r(8)+yawAcceleration*duration];
            % Charge arithmetic in the prediction, rather than accepting an
            % empty measurement intersection with a physical tolerance.
            arithmetic = [abs(x(1:2))+abs(x(3:4))*duration+abs(x(5:6))*(duration.^2/2); ...
                abs(x(3:4))+abs(x(5:6))*duration;repmat(abs(x(5:6)),1,numel(duration)); ...
                abs(x(7))+abs(x(8))*duration;repmat(abs(x(8)),1,numel(duration))];
            radius = radius+64*eps*(1+arithmetic+radius);
        end

        function [center, jerk, yawAcceleration] = nominalFlow(encounter, duration)
        % Constant curvature and tangential acceleration for objective anchors.
        % Safety uses the uncertain flow on every held interval.
            x = encounter.center;
            duration = double(duration(:).');
            if any(~isfinite(duration) | duration<0)
                error("collisionAvoidanceController:invalidPredictionTime","Prediction times must be finite and nonnegative.");
            end
            if isfield(encounter,"nominalCenter"), x = encounter.nominalCenter; end
            speed = norm(x(3:4));
            if speed<=sqrt(eps)
                center = repmat(x,1,numel(duration));center(3:6,:) = 0;center(8,:) = 0;
                jerk = zeros(size(duration));yawAcceleration = jerk;
                return;
            end
            acceleration = dot(x(3:4),x(5:6))/speed;
            if isfield(encounter.contract,"scalarAccelerationMaximum")
                limit = encounter.contract.scalarAccelerationMaximum;
                acceleration = min(max(acceleration,-limit),limit);
            end
            curvature = x(8)/speed;
            stopped = acceleration<0 & duration>=speed/-acceleration;
            if acceleration<0, duration = min(duration,speed/-acceleration); end
            arc = speed*duration+acceleration*duration.^2/2;
            initialCourse = atan2(x(4),x(3));
            course = initialCourse+curvature*arc;
            if abs(curvature)<sqrt(eps)
                position = x(1:2)+[cos(initialCourse);sin(initialCourse)]*arc;
            else
                position = x(1:2)+[sin(course)-sin(initialCourse);cos(initialCourse)-cos(course)]/curvature;
            end
            speed = max(0,speed+acceleration*duration);
            acceleration = acceleration+zeros(size(duration));
            acceleration(stopped) = 0;
            direction = [cos(course);sin(course)];
            center = [position;speed.*direction; ...
                acceleration.*direction+speed.^2*curvature.*[-direction(2,:);direction(1,:)]; ...
                x(7)+curvature*arc;curvature*speed];
            % Uniform nominal derivatives over a complete controller interval.
            interval = 0;
            if isfield(encounter.contract,"predictionSampleTime"), interval = encounter.contract.predictionSampleTime; end
            maximumSpeed = speed+abs(acceleration)*interval;
            jerk = hypot(maximumSpeed.^3*curvature^2,3*maximumSpeed.*abs(acceleration*curvature));
            yawAcceleration = abs(acceleration*curvature);
        end

        function next = advance(encounter, duration, observation, lane, cfg)
        %advance Condition the carried set; observation replacement is not renewal.
            next = encounter;
            [next.center, next.radius] = targetPrediction.finiteFlow(encounter, duration);
            next.time = encounter.time+duration;
            if ~isempty(observation)
                measured = targetPrediction.admit(observation,next.time,lane,cfg);
                if ~targetPrediction.isFiniteSensing(measured) ...
                        || ~isequal(measured.contract.jerkBound,encounter.contract.jerkBound) ...
                        || measured.contract.yawAccelerationBound ~= encounter.contract.yawAccelerationBound ...
                        || measured.halfLength ~= encounter.halfLength || measured.halfWidth ~= encounter.halfWidth
                    error("collisionAvoidanceController:changedEncounterContract","Physical target motion bounds changed.");
                end
                next.center(7) = measured.center(7)+atan2(sin(next.center(7)-measured.center(7)), ...
                    cos(next.center(7)-measured.center(7)));
                lower = max(measured.center-measured.radius,next.center-next.radius);
                upper = min(measured.center+measured.radius,next.center+next.radius);
                if any(lower>upper)
                    error("collisionAvoidanceController:inconsistentObservation","Target measurements contradict the motion enclosure.");
                end
                measured.center = lower+(upper-lower)/2;
                measured.radius = max(upper-measured.center,measured.center-lower);
                nominal = targetPrediction.nominalFlow(encounter,duration);
                difference = nominal-measured.center;
                difference(7) = atan2(sin(difference(7)),cos(difference(7)));
                % Correct only coordinates excluded by the new enclosure.
                % A tiny yaw-rate or position correction must not replace
                % every still-compatible motion parameter with a noisy point
                % estimate and reverse the complete future trajectory.
                measured.nominalCenter = measured.center ...
                    +min(max(difference,-measured.radius),measured.radius);
                next = measured;
                return;
            end
        end

        function support = rectangleSupport(halfLength, halfWidth, normal, yaw, radius)
        %rectangleSupport Exact directional support maximized over a yaw interval.
            angle = atan2(normal(2), normal(1))-yaw;
            lower = angle-radius;
            upper = angle+radius;
            support = max(halfLength*abs(cos(lower))+halfWidth*abs(sin(lower)), ...
                halfLength*abs(cos(upper))+halfWidth*abs(sin(upper)));
            phase = atan2(halfWidth, halfLength);
            for candidate = [phase, -phase, pi-phase, pi+phase]
                reachesPeak = ceil((lower-candidate)/(2*pi)) <= floor((upper-candidate)/(2*pi));
                support(reachesPeak) = hypot(halfLength, halfWidth);
            end
        end

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

function encounter = localEncounter(target,time,contract)
    encounter = struct("key",target.key,"contract",contract, ...
        "center",[target.position;target.velocity;target.acceleration;target.yaw;target.yawRate], ...
        "radius",[target.positionErrorBound;target.velocityErrorBound; ...
            target.accelerationErrorBound;target.yawErrorBound;target.yawRateErrorBound], ...
        "time",time,"halfLength",target.length/2,"halfWidth",target.width/2);
    encounter.nominalCenter = encounter.center;
    if string(contract.kind)=="finite-sensing-motion-v1"
        % A position-only acquisition does not measure motion derivatives.
        % Seed unresolved acceleration and turning from the constant-velocity
        % hypothesis, projected into the published enclosure. Hard execution
        % still uses the complete uncertain state, including nonzero turns.
        derivativeRows = [5,6,8];
        lower = encounter.center(derivativeRows)-encounter.radius(derivativeRows);
        upper = encounter.center(derivativeRows)+encounter.radius(derivativeRows);
        encounter.nominalCenter(derivativeRows) = min(max(0,lower),upper);
    end
end

function distance = localArc(time, speed, acceleration)
    if acceleration < 0.0
        time = min(time, speed/-acceleration);
    end
    distance = speed*time+0.5*acceleration*time.^2;
end
