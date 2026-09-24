classdef targetPrediction
    %targetPrediction Bounded online target motion and footprint propagation.

    methods (Static)
        function [center,jerk] = nrmmFlow(initial,rearAxleDistance,time,stopPolicy)
        % Exact fixed-frame NRMM, initial=[X;Y;V;A;psi;beta], in SI units.
        % Output rows are [X;Y;vx;vy;ax;ay;psi;yawRate]. An explicit 'hold'
        % policy freezes the pose after braking to rest; the default rejects
        % times beyond that stop. Derivatives at a held stop are right sided.
            if nargin<4,stopPolicy="reject";end
            validateattributes(initial,{'double'},{'real','finite','size',[6,1]});
            validateattributes(rearAxleDistance,{'double'},{'scalar','real','finite','positive'});
            validateattributes(time,{'double'},{'real','finite','nonnegative','vector'});
            assert(initial(3)>=0 && isscalar(string(stopPolicy)) ...
                && any(string(stopPolicy)==["reject","hold"]), ...
                'collisionAvoidanceController:invalidNrmmMotion','Use forward speed and a reject or hold stop policy.');
            time=reshape(time,1,[]);speed=initial(3);acceleration=initial(4);
            stopped=false(size(time));
            if acceleration<0
                stopTime=speed/-acceleration;
                if any(time>stopTime) && string(stopPolicy)=="reject"
                    error('collisionAvoidanceController:nrmmPastStop','Specify hold explicitly for times beyond the NRMM braking stop.');
                end
                if string(stopPolicy)=="hold",stopped=time>=stopTime;time=min(time,stopTime);end
            end
            curvature=sin(initial(6))/rearAxleDistance;
            arc=speed*time+acceleration*time.^2/2;
            course=initial(5)+initial(6)+curvature*arc;
            position=initial(1:2)+localArcDisplacement(arc,curvature,initial(5)+initial(6));
            speed=speed+acceleration*time;acceleration=acceleration+zeros(size(time));
            speed(stopped)=0;acceleration(stopped)=0;
            tangent=[cos(course);sin(course)];normal=[-sin(course);cos(course)];
            center=[position;speed.*tangent;acceleration.*tangent+curvature*speed.^2.*normal; ...
                initial(5)+curvature*arc;curvature*speed];
            jerk=-curvature^2*speed.^3.*tangent+3*curvature*acceleration.*speed.*normal;
        end

        function [offset,slope] = rectangleSupportMajorant(normal,referenceHeading,halfLength,halfWidth,anchor,errorMaximum)
        % Touching convex majorant of rectangle support over a yaw interval.
        % Every vertex projection is concave wherever it is nonnegative.
        % Tangents there cover every possible support-maximizing vertex.
            if errorMaximum>=pi
                offset=hypot(halfLength,halfWidth)+64*eps*(1+halfLength+halfWidth);
                slope=0;
                return;
            end
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
        function encounter = admitOnline(target, time, lane, cfg)
        % Admit a nominal Cartesian flow with bounded jerk and yaw acceleration.
        % Nonzero motion bounds tighten reachable sets, not input eligibility.
            if isempty(target)
                encounter = struct("key",{},"radius",{},"contract",{});
                return;
            end
            motion = target.predictionMotion;
            if isempty(motion)
                motion = struct("kind","finite-sensing-motion-v1", ...
                    "jerkBound",zeros(2,1), ...
                    "yawAccelerationBound",target.predictionYawAccelerationErrorBound);
            else
                if ~isstruct(motion) || ~isscalar(motion) || ~isfield(motion,"kind") ...
                        || ~isscalar(string(motion.kind)) ...
                        || ~any(string(motion.kind)==["exact-motion-v1","finite-sensing-motion-v1","nrmm-motion-v1"])
                    error("collisionAvoidanceController:invalidEncounterContract", ...
                        "Use identified Cartesian motion bounds.");
                end
                if string(motion.kind)=="exact-motion-v1"
                    if ~isfield(motion,"jerkBound"), motion.jerkBound = zeros(2,1); end
                    if ~isfield(motion,"yawAccelerationBound"), motion.yawAccelerationBound = 0; end
                    motion.kind = "finite-sensing-motion-v1";
                end
            end
            if target.key=="anonymousTarget", target.key = "singleTarget:1"; end
            target.predictionMotion = motion;
            encounter = targetPrediction.admit(target,time,lane,cfg);
        end

        function next = condition(carried, duration, measured)
        %condition Intersect the bounded reachable box with a new measurement.
        % The true target state lies in the propagated carried box and in the
        % measurement box, so the interval hull of their intersection contains
        % it and stays inside the propagated box. An NRMM encounter also
        % propagates its parameter intervals over the hold and intersects them
        % with the measurement's. An empty intersection contradicts the declared
        % motion bound or measurement contract.
            [center, radius] = targetPrediction.finiteFlow(carried, duration);
            measuredCenter = measured.center;
            measuredCenter(7) = center(7)+atan2(sin(measuredCenter(7)-center(7)), ...
                cos(measuredCenter(7)-center(7)));
            allowance = 256*eps*(1+abs(center)+abs(measuredCenter)+radius+measured.radius);
            lower = max(center-radius, measuredCenter-measured.radius);
            upper = min(center+radius, measuredCenter+measured.radius);
            if any(lower > upper+allowance)
                error("collisionAvoidanceController:inconsistentObservation", ...
                    "The target measurement box does not intersect its bounded reachable set.");
            end
            middle = (lower+upper)/2;
            lower = min(lower, middle);
            upper = max(upper, middle);
            next = carried;
            next.center = middle;
            next.radius = (upper-lower)/2;
            next.time = carried.time+duration;
            next.nominalCenter = next.center;
            if localIsNrmm(carried.contract) && localIsNrmm(measured.contract)
                next.parameters = localConditionParameters(carried.parameters,duration,measured);
            end
        end

        function finite = isFiniteSensing(encounter)
            finite = any(string(encounter.contract.kind) == ["finite-sensing-motion-v1","nrmm-motion-v1"]);
        end

        function encounter = admit(target, time, ~, cfg)
        %admit Validate the finite motion bounds used by every certificate.
        % finite-sensing-motion-v1: any Cartesian motion with |jerk| <= jerkBound
        % from the estimated state. nrmm-motion-v1: exact NRMM motion, a constant
        % speed-rate, constant-curvature (constant-sideslip) path through the
        % estimated state with |curvature| <= curvatureMaximum and, when
        % declared, |speed-rate| <= speedRateMaximum; its jerkBound and
        % yawAccelerationBound must be zero. A varying speed-rate or curvature
        % leaves every NRMM path through a later estimate, so it is declared
        % with a finite-sensing contract instead. The encounter carries the
        % NRMM parameter intervals (targetPrediction.nrmmParameters).
            if isfield(target,"predictionMotion") && ~isempty(target.predictionMotion)
                motion = target.predictionMotion;
                if ~isstruct(motion) || ~isscalar(motion) ...
                        || ~all(isfield(motion,["kind","jerkBound","yawAccelerationBound"])) ...
                        || ~any(string(motion.kind) == ["finite-sensing-motion-v1","nrmm-motion-v1"])
                    error("collisionAvoidanceController:invalidEncounterContract","Invalid finite motion bounds.");
                end
                contract = struct("kind",string(motion.kind),"id",target.key, ...
                    "validFrom",time,"validityScope","whileEncounterActive", ...
                    "jerkBound",motion.jerkBound,"yawAccelerationBound",motion.yawAccelerationBound, ...
                    "predictionSampleTime",cfg.controller.sampleTime);
                if contract.kind=="nrmm-motion-v1"
                    if ~isfield(motion,"curvatureMaximum")
                        error("collisionAvoidanceController:invalidEncounterContract", ...
                            "An NRMM motion contract requires curvatureMaximum.");
                    end
                    validateattributes(motion.curvatureMaximum,{'double'},{'scalar','real','finite','positive'});
                    if any(motion.jerkBound(:)~=0) || any(motion.yawAccelerationBound(:)~=0)
                        error("collisionAvoidanceController:invalidEncounterContract", ...
                            "An NRMM motion contract describes exact NRMM motion; " ...
                            +"use a finite-sensing contract for a model error.");
                    end
                    contract.curvatureMaximum = motion.curvatureMaximum;
                    if isfield(motion,"speedRateMaximum")
                        validateattributes(motion.speedRateMaximum,{'double'},{'scalar','real','finite','nonnegative'});
                        contract.speedRateMaximum = motion.speedRateMaximum;
                    end
                end
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
                if target.key=="anonymousTarget"
                    error("collisionAvoidanceController:invalidEncounterContract","Finite encounters need stable track identity.");
                end
                contract.jerkBound = contract.jerkBound(:);
                encounter = localEncounter(target,time,contract);
                if contract.kind=="nrmm-motion-v1"
                    published = [];
                    if isfield(target,"parameterErrorBounds"),published = target.parameterErrorBounds;end
                    encounter.parameters = targetPrediction.nrmmParameters(encounter.center, ...
                        encounter.radius,contract,published);
                end
                return;
            end
            error("collisionAvoidanceController:missingPredictionMotion", ...
                "Every target requires identified finite-sensing motion bounds.");
        end

        function parameters = nrmmParameters(center,radius,contract,published)
        %nrmmParameters NRMM parameter intervals of an estimate box.
        % Intervals [lo; hi] for the speed V, course, speed-rate A and curvature
        % kappa of every NRMM state in the box (center, radius). From the box:
        %   V = |v| +- |r_v|; course = atan2(v) +- asin(|r_v|/|v|);
        %   A = v'a/|v| +- (|r_a| + 2|a| sin(r_course/2));
        %   kappa in omega/V and in a_N/V^2 over the yaw-rate, normal-acceleration
        %   and speed intervals.
        % These are intersected with the bounds the declarer publishes about the
        % same estimate (readPlanningInputs: parameterErrorBounds, e.g. the NRMM
        % estimator's frame-free component balls) and with the contract's
        % curvatureMaximum, speedRateMaximum and acceleration magnitude maximum.
        % An empty intersection is an inconsistent observation.
            x = center;r = radius;
            velocityRadius = norm(r(3:4));accelerationRadius = norm(r(5:6));
            speed = norm(x(3:4));course = atan2(x(4),x(3));
            courseRadius = pi;
            if velocityRadius<speed,courseRadius = asin(velocityRadius/speed);end
            tangential = 0;
            if speed>0,tangential = dot(x(3:4),x(5:6))/speed;end
            % Tangential and normal acceleration components err by at most this much.
            componentRadius = accelerationRadius+2*norm(x(5:6))*sin(courseRadius/2);
            parameters = struct('speed',[max(0,speed-velocityRadius);speed+velocityRadius], ...
                'course',course+[-courseRadius;courseRadius], ...
                'speedRate',tangential+[-componentRadius;componentRadius],'curvature',[-Inf;Inf]);
            if speed-velocityRadius>0
                % On an NRMM path the yaw rate is kappa*V and the normal
                % acceleration is kappa*V^2; the true state satisfies both.
                speedLow = speed-velocityRadius;speedHigh = speed+velocityRadius;
                rates = x(8)+[-r(8);r(8)];
                normalAcceleration = (x(3)*x(6)-x(4)*x(5))/speed+[-componentRadius;componentRadius];
                quotients = [rates./[speedLow,speedHigh],normalAcceleration./[speedLow^2,speedHigh^2]];
                parameters.curvature = [max(min(quotients(:,1:2),[],'all'),min(quotients(:,3:4),[],'all')); ...
                    min(max(quotients(:,1:2),[],'all'),max(quotients(:,3:4),[],'all'))];
            end
            if ~isempty(published)
                parameters.speed = localIntersect(parameters.speed,speed+[-published.speed;published.speed]);
                parameters.course = localIntersect(parameters.course,course+[-published.course;published.course]);
                parameters.speedRate = localIntersect(parameters.speedRate, ...
                    tangential+[-published.speedRate;published.speedRate]);
                parameters.curvature = localIntersect(parameters.curvature,published.curvature);
            end
            parameters.curvature = localIntersect(parameters.curvature, ...
                contract.curvatureMaximum*[-1;1]);
            limit = Inf;
            if isfield(contract,'speedRateMaximum'),limit = contract.speedRateMaximum;end
            if isfield(contract,'scalarAccelerationMaximum')
                % |A| <= |a| <= the declared acceleration magnitude bound.
                limit = min(limit,contract.scalarAccelerationMaximum);
            end
            parameters.speedRate = localIntersect(parameters.speedRate,limit*[-1;1]);
        end

        function [center, radius] = finiteFlow(encounter, duration)
        %finiteFlow Positive Cartesian reachability; no speed division.
        % For an nrmm-motion-v1 contract the box encloses every NRMM path
        % through the estimate box: the NRMM parameter enclosure
        % (targetPrediction.deviationModel) intersected with the Cartesian
        % enclosure of the same paths.
            duration = double(duration(:).');
            if any(~isfinite(duration) | duration < 0)
                error("collisionAvoidanceController:invalidPredictionTime", "Prediction times must be finite and nonnegative.");
            end
            if localIsNrmm(encounter.contract)
                [center,radius] = localNrmmFlow(encounter,duration);
                return;
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
            if isfield(encounter.contract,"scalarAccelerationMaximum")
                % A declared |acceleration| <= amax caps each axis deviation from
                % the constant nominal acceleration at amax + |nominal|; the
                % velocity and position radii integrate the capped deviation.
                [radius(1:2,:),radius(3:4,:),radius(5:6,:)] = localCappedDeviation(r(1:6),jerk, ...
                    encounter.contract.scalarAccelerationMaximum+abs(x(5:6)),duration);
            end
            % Charge arithmetic in the prediction, rather than accepting an
            % empty measurement intersection with a physical tolerance. Only
            % terms that were actually summed are charged: exactly copied
            % acceleration or yaw-rate constants need no integration reserve.
            arithmetic = [abs(x(1:2))+abs(x(3:4))*duration+abs(x(5:6))*(duration.^2/2); ...
                abs(x(3:4))+abs(x(5:6))*duration;repmat(abs(x(5:6)),1,numel(duration)); ...
                abs(x(7))+abs(x(8))*duration;repmat(abs(x(8)),1,numel(duration))];
            radius = radius+64*eps*(arithmetic+radius);
        end

        function motion = deviationModel(encounter,duration)
        %deviationModel Target deviation from its nominal flow, for reactive tubes.
        % At each time: center (8 x n) is the nominal state; parameterGenerators
        % (6 x g x n) map g error sources fixed at admission to the deviation of
        % [position; velocity; acceleration]; remainder (6 x n) bounds the rest
        % of that deviation; holdBound (2 x n) bounds, per axis, the mean
        % acceleration deviation over the hold ending at that time that the
        % parameter generators do not describe; holdJerk (2 x 1) is its jerk.
        % finite-sensing-motion-v1: the sources are the initial position and
        % velocity box (p = p0 + v0 t) and holdBound is the capped jerk growth.
        % nrmm-motion-v1: the sources are the NRMM parameter errors [p0; V; A;
        % course; curvature] with sensitivities of the NRMM path, the remainder
        % is their second-order term (or a path-length ball where the linear
        % model does not apply), and holdBound is zero (exact NRMM).
            duration = double(duration(:).');
            jerk = encounter.contract.jerkBound(:);
            if localIsNrmm(encounter.contract)
                state = localNrmmState(encounter,duration,true);
                generators = pagemtimes(state.sensitivity,diag(state.parameterRadius));
                remainder = state.remainder+64*eps*(1+abs(state.center(1:6,:)) ...
                    +reshape(sum(abs(generators),2),6,[]));
                motion = struct('center',state.center,'parameterGenerators',generators, ...
                    'remainder',remainder,'holdBound',jerk*duration,'holdJerk',jerk);
                return;
            end
            center = targetPrediction.finiteFlow(encounter,duration);
            radius = encounter.radius(1:4);
            generators = zeros(6,4,numel(duration));
            for index = 1:numel(duration)
                generators(1:4,:,index) = [eye(2),duration(index)*eye(2);zeros(2),eye(2)]*diag(radius);
            end
            motion = struct('center',center,'parameterGenerators',generators, ...
                'remainder',zeros(6,numel(duration)), ...
                'holdBound',targetPrediction.accelerationDeviationBound(encounter,duration),'holdJerk',jerk);
        end

        function bound = accelerationDeviationBound(encounter,duration)
        %accelerationDeviationBound Per-axis bound on |a(t) - a_nominal| at the
        % given prediction times: the jerk growth from the current acceleration
        % radius, capped at amax + |a_nominal| when a scalar acceleration
        % maximum is declared. Nondecreasing in time.
            duration = double(duration(:).');
            r = encounter.radius;jerk = encounter.contract.jerkBound(:);
            bound = r(5:6)+jerk*duration;
            if isfield(encounter.contract,"scalarAccelerationMaximum")
                bound = min(bound,encounter.contract.scalarAccelerationMaximum+abs(encounter.center(5:6)));
            end
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
            if speed==0
                % The Cartesian interface cannot identify sideslip/curvature at
                % rest. Use the acceleration direction and a straight launch.
                acceleration = norm(x(5:6));
                initialCourse = x(7);
                if acceleration>0,initialCourse=atan2(x(6),x(5));end
                curvature = 0;
            else
                acceleration = dot(x(3:4),x(5:6))/speed;
                initialCourse = atan2(x(4),x(3));
                curvature = x(8)/speed;
            end
            if isfield(encounter.contract,"scalarAccelerationMaximum")
                limit = encounter.contract.scalarAccelerationMaximum;
                acceleration = min(max(acceleration,-limit),limit);
            end
            stopped = acceleration<0 & duration>=speed/-acceleration;
            if acceleration<0, duration = min(duration,speed/-acceleration); end
            arc = speed*duration+acceleration*duration.^2/2;
            course = initialCourse+curvature*arc;
            position = x(1:2)+localArcDisplacement(arc,curvature,initialCourse);
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
                        || string(measured.contract.kind)~=string(encounter.contract.kind) ...
                        || ~isequaln(localCurvatureMaximum(measured.contract), ...
                            localCurvatureMaximum(encounter.contract)) ...
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
                if localIsNrmm(encounter.contract) && localIsNrmm(measured.contract)
                    measured.parameters = localConditionParameters(encounter.parameters,duration,measured);
                end
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

function displacement=localArcDisplacement(arc,curvature,course)
    halfTurn=curvature*arc/2;scale=ones(size(halfTurn));
    regular=abs(halfTurn)>1e-4;
    scale(regular)=sin(halfTurn(regular))./halfTurn(regular);
    small=halfTurn(~regular);scale(~regular)=1-small.^2/6+small.^4/120;
    displacement=arc.*scale.*[cos(course+halfTurn);sin(course+halfTurn)];
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

function [position,velocity,acceleration] = localCappedDeviation(r,jerk,cap,t)
% Radii under |jerk| <= J and a per-axis acceleration deviation cap c:
% b(t) = min(r_a + J t, c), velocity r_v + int b, position r_p + r_v t + int int b.
    rp = r(1:2);rv = r(3:4);ra = min(r(5:6),cap);
    switchTime = zeros(2,1);
    for axis = 1:2
        if jerk(axis)>0
            switchTime(axis) = max(0,(cap(axis)-ra(axis))/jerk(axis));
        elseif ra(axis)<cap(axis)
            switchTime(axis) = Inf;
        end
    end
    early = min(t,switchTime);late = max(0,t-switchTime);
    acceleration = min(ra+jerk.*t,cap);
    velocity = rv+ra.*early+jerk.*early.^2/2+cap.*late;
    position = rp+rv.*t+ra.*early.^2/2+jerk.*early.^3/6 ...
        +(ra.*early+jerk.*early.^2/2).*late+cap.*late.^2/2;
    position(isnan(position)) = Inf;velocity(isnan(velocity)) = Inf;
end

function nrmm = localIsNrmm(contract)
% Node snapshots of the geometry kernel carry no kind: their zero-duration flow
% is the node state itself under either contract.
    nrmm = isfield(contract,'kind') && string(contract.kind)=="nrmm-motion-v1";
end

function maximum = localCurvatureMaximum(contract)
% Declared NRMM curvature maximum; NaN for a Cartesian contract.
    maximum = NaN;
    if isfield(contract,'curvatureMaximum'),maximum = contract.curvatureMaximum;end
end

function [center,radius] = localNrmmFlow(encounter,duration)
% Box enclosure of every NRMM path through the estimate box: the NRMM parameter
% enclosure intersected with the Cartesian enclosure of the same paths, whose
% jerk is at most hypot(kappa^2 V^3, 3 A kappa V) and whose yaw acceleration is
% at most |A kappa| over the parameter intervals. A stop sets the acceleration
% to zero: per axis |a - a0| <= r_a + J t before it and |a0| after, so where a
% path may have stopped the acceleration deviation also covers max(0,|a0|-r_a).
    state = localNrmmState(encounter,duration,true);
    center = state.center;count = numel(duration);
    radius = zeros(8,count);
    radius(1:6,:) = reshape(pagemtimes(abs(state.sensitivity),state.parameterRadius),6,[])+state.remainder;
    radius(7,:) = state.yawRadius;radius(8,:) = state.yawRateRadius;
    radius = radius+64*eps*(1+abs(center)+radius);
    x = encounter.center;r = encounter.radius;t = duration;jerk = state.jerkBound;
    yawAcceleration = state.yawAccelerationBound;
    cartesian = [x(1:2)+x(3:4)*t+x(5:6)*(t.^2/2);x(3:4)+x(5:6)*t;repmat(x(5:6),1,count); ...
        x(7)+x(8)*t;repmat(x(8),1,count)];
    jump = max(0,abs(x(5:6))-r(5:6)).*~state.noStop;
    spread = [r(1:2)+r(3:4)*t+(r(5:6)+jump).*(t.^2/2)+jerk.*t.^3/6; ...
        r(3:4)+(r(5:6)+jump).*t+jerk.*t.^2/2;r(5:6)+jump+jerk.*t; ...
        r(7)+r(8)*t+yawAcceleration*t.^2/2;r(8)+yawAcceleration*t];
    spread = spread+64*eps*(1+abs(cartesian)+spread);
    lower = max(center-radius,cartesian-spread);upper = min(center+radius,cartesian+spread);
    allowance = 256*eps*(1+abs(center)+abs(cartesian)+radius+spread);
    if any(lower>upper+allowance,'all')
        error("collisionAvoidanceController:inconsistentObservation", ...
            "The target estimate admits no NRMM path within the declared bounds.");
    end
    middle = (lower+upper)/2;lower = min(lower,middle);upper = max(upper,middle);
    center = middle;radius = (upper-lower)/2;
end

function state = localNrmmState(encounter,duration,uncertain)
% NRMM (constant speed-rate A, constant curvature kappa) path of the estimate
% and, when uncertain, its parameter sensitivities and second-order remainder.
% The initial position is the estimate box; V, A, course and kappa are the
% encounter's parameter intervals (targetPrediction.nrmmParameters), taken as
% their midpoints plus or minus their half-widths.
% sensitivity(:,:,k) maps [dp0x; dp0y; dV; dA; dcourse; dkappa] to the change
% of [position; velocity; acceleration] at time k; remainder(:,k) bounds the
% rest (Lagrange second-order bounds). Where the linear model does not apply
% (course radius above 0.5 rad, speed interval touching zero or a possible stop
% by that time) only the p0 columns are kept and the remainder is a
% path-length ball bound.
    x = encounter.center;r = encounter.radius;
    if ~isfield(encounter,'parameters') || isempty(encounter.parameters)
        error("collisionAvoidanceController:invalidEncounterContract", ...
            "An NRMM encounter carries parameter intervals (targetPrediction.nrmmParameters).");
    end
    intervals = encounter.parameters;
    duration = double(duration(:).');count = numel(duration);
    speedLow = intervals.speed(1);speedHigh = intervals.speed(2);
    speed = (speedLow+speedHigh)/2;velocityRadius = (speedHigh-speedLow)/2;
    course = mean(intervals.course);courseRadius = min(pi,(intervals.course(2)-intervals.course(1))/2);
    tangential = mean(intervals.speedRate);tangentialRadius = (intervals.speedRate(2)-intervals.speedRate(1))/2;
    kappa = mean(intervals.curvature);kappaRadius = (intervals.curvature(2)-intervals.curvature(1))/2;
    stopTime = Inf;
    if tangential<0,stopTime = speed/-tangential;end
    moving = min(duration,stopTime);
    arc = speed*moving+tangential*moving.^2/2;
    rate = max(0,speed+tangential*moving);
    stopped = duration>=stopTime;
    heading = course+kappa*arc;
    tangent = [cos(heading);sin(heading)];normal = [-tangent(2,:);tangent(1,:)];
    displacement = localArcDisplacement(arc,kappa,course);
    acceleration = tangential*tangent+kappa*rate.^2.*normal;
    acceleration(:,stopped) = 0;
    state = struct('center',[x(1:2)+displacement;rate.*tangent;acceleration;x(7)+kappa*arc;kappa*rate]);
    if ~uncertain,return;end
    t = duration;
    dV = velocityRadius;dA = tangentialRadius;dCourse = courseRadius;dK = kappaRadius;
    ds = t*dV+t.^2/2*dA;dRate = dV+t*dA;
    sMax = arc+ds;rateMax = rate+dRate;kMax = abs(kappa)+dK;aMax = abs(tangential)+dA;
    dPhi = dCourse+abs(kappa)*ds+arc*dK+dK*ds;
    noStop = speedLow>0 & speedLow+min(0,tangential-dA)*t>0;
    linear = noStop & courseRadius<=0.5;
    % Cartesian jerk and yaw acceleration of every path, nondecreasing in t.
    speedBound = speedHigh+max(tangential+dA,0)*t;
    state.jerkBound = hypot(kMax^2*speedBound.^3,3*aMax*kMax*speedBound);
    state.yawAccelerationBound = aMax*kMax;
    state.noStop = noStop;
    sensitivity = zeros(6,6,count);
    sensitivity(1,1,:) = 1;sensitivity(2,2,:) = 1;
    curvatureColumn = localCurvatureSensitivity(arc,kappa,course);
    for k = find(linear)
        T = tangent(:,k);N = normal(:,k);v = rate(k);s = arc(k);
        sensitivity(1:2,3:6,k) = [T*t(k),T*t(k)^2/2,[-displacement(2,k);displacement(1,k)],curvatureColumn(:,k)];
        sensitivity(3:4,3:6,k) = [T+v*kappa*t(k)*N,t(k)*T+v*kappa*t(k)^2/2*N,v*N,v*s*N];
        sensitivity(5:6,3:6,k) = [(tangential*kappa*t(k)+2*v*kappa)*N-v^2*kappa^2*t(k)*T, ...
            T+(tangential*kappa*t(k)^2/2+2*v*t(k)*kappa)*N-v^2*kappa^2*t(k)^2/2*T, ...
            tangential*N-v^2*kappa*T,(tangential*s+v^2)*N-v^2*kappa*s*T];
    end
    positionRemainder = (kMax*ds.^2+2*ds*dCourse+2*sMax.*ds*dK+sMax*dCourse^2 ...
        +sMax.^2*dCourse*dK+sMax.^3/3*dK^2)/2;
    velocityRemainder = dRate.*dPhi+rateMax.*dPhi.^2/2+rateMax*dK.*ds;
    phiSlope = aMax+rateMax.^2*kMax;
    accelerationRemainder = kMax*dRate.^2+phiSlope.*dPhi.^2/2+dA*dPhi+2*rateMax.*dRate*dK ...
        +2*rateMax*kMax.*dRate.*dPhi+rateMax.^2*dK.*dPhi+phiSlope*dK.*ds;
    remainder = [repmat(positionRemainder,2,1);repmat(velocityRemainder,2,1);repmat(accelerationRemainder,2,1)];
    yawRadius = r(7)+abs(kappa)*ds+arc*dK+dK*ds;
    yawRateRadius = abs(kappa)*dRate+rate*dK+dK*dRate;
    % Path-length ball where the linear model does not apply.
    aHigh = tangential+dA;highStop = Inf;
    if aHigh<0,highStop = speedHigh/-aHigh;end
    highMoving = min(t,highStop);
    sHigh = speedHigh*highMoving+aHigh*highMoving.^2/2;
    rateHigh = max(0,speedHigh+aHigh*highMoving);
    ball = ~linear;
    remainder(1:2,ball) = repmat(sHigh(ball)+vecnorm(displacement(:,ball),2,1),2,1);
    remainder(3:4,ball) = repmat(rateHigh(ball)+rate(ball),2,1);
    remainder(5:6,ball) = repmat(hypot(aMax,rateHigh(ball).^2*kMax)+vecnorm(acceleration(:,ball),2,1),2,1);
    yawRadius(ball) = r(7)+kMax*sHigh(ball)+abs(kappa)*arc(ball);
    yawRateRadius(ball) = kMax*rateHigh(ball)+abs(kappa)*rate(ball);
    state.sensitivity = sensitivity;
    state.parameterRadius = [r(1:2);dV;dA;dCourse;dK];
    state.remainder = remainder;state.yawRadius = yawRadius;state.yawRateRadius = yawRateRadius;
    state.linear = linear;
end

function interval = localIntersect(first,second)
% Intersection of two intervals [lo; hi]; empty within roundoff is inconsistent.
    interval = [max(first(1),second(1));min(first(2),second(2))];
    values = [first(:);second(:)];values = abs(values(isfinite(values)));
    allowance = 256*eps*(1+max([0;values]));
    if interval(1)>interval(2)+allowance
        error("collisionAvoidanceController:inconsistentObservation", ...
            "The target estimate admits no NRMM parameter within the declared bounds.");
    end
    middle = (interval(1)+interval(2))/2;
    interval = [min(interval(1),middle);max(interval(2),middle)];
end

function parameters = localConditionParameters(carried,duration,measured)
% Propagate the carried NRMM parameter intervals over the hold and intersect
% them with the measured encounter's. The speed-rate and curvature are
% constant; the speed max(0, V + A t) and the arc length are nondecreasing in
% both, and the course advances by kappa times the arc.
    speedRate = carried.speedRate;curvature = carried.curvature;
    speed = max(0,carried.speed+speedRate*duration);
    arc = [localArc(duration,carried.speed(1),speedRate(1));localArc(duration,carried.speed(2),speedRate(2))];
    products = curvature*arc.';
    course = carried.course+[min(products,[],'all');max(products,[],'all')];
    measuredCourse = measured.parameters.course;
    measuredCourse = measuredCourse+2*pi*round((mean(course)-mean(measuredCourse))/(2*pi));
    parameters = struct('speed',localIntersect(speed,measured.parameters.speed), ...
        'course',localIntersect(course,measuredCourse), ...
        'speedRate',localIntersect(speedRate,measured.parameters.speedRate), ...
        'curvature',localIntersect(curvature,measured.parameters.curvature));
end

function column = localCurvatureSensitivity(arc,kappa,course)
% d(displacement)/d(kappa) = int_0^s sigma * N(course + kappa*sigma) d sigma,
% by 12-point Gauss-Legendre quadrature (the integrand is analytic).
    persistent nodes weights
    if isempty(nodes)
        order = 12;beta = (1:order-1)./sqrt(4*(1:order-1).^2-1);
        [vectors,values] = eig(diag(beta,1)+diag(beta,-1));
        nodes = diag(values).';weights = 2*vectors(1,:).^2;
    end
    sigma = arc(:)*(1+nodes)/2;
    phase = course+kappa*sigma;
    scale = (arc(:)/2).*sigma.*weights;
    column = [-sum(scale.*sin(phase),2).';sum(scale.*cos(phase),2).'];
end

