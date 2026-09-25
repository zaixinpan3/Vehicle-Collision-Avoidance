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
        % Admit the target under its NRMM motion contract (targetPrediction.admit).
        % A sole anonymous target receives the stable key singleTarget:1.
            if isempty(target)
                encounter = struct("key",{},"radius",{},"contract",{});
                return;
            end
            if target.key=="anonymousTarget", target.key = "singleTarget:1"; end
            encounter = targetPrediction.admit(target,time,lane,cfg);
        end

        function next = condition(carried, duration, measured)
        %condition Intersect the bounded reachable box with a new measurement.
        % The true target state lies in the propagated carried box and in the
        % measurement box, so the interval hull of their intersection contains
        % it and stays inside the propagated box. The NRMM parameter intervals
        % are propagated over the hold and intersected with the measurement's.
        % An empty intersection contradicts the declared motion bound or
        % measurement contract.
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
            next.parameters = localConditionParameters(carried.parameters,duration,measured);
        end

        function encounter = admit(target, time, ~, cfg)
        %admit Validate the target's motion contract used by every certificate.
        % nrmm-motion-v1 is the only contract: exact NRMM motion, a constant
        % speed-rate, constant-curvature (constant-sideslip) path through the
        % estimated state with |curvature| <= curvatureMaximum and, when
        % declared, |speed-rate| <= speedRateMaximum and |acceleration| <=
        % scalarAccelerationMaximum. A varying speed-rate or curvature has no
        % contract. The encounter carries the NRMM parameter intervals
        % (targetPrediction.nrmmParameters).
            if ~isfield(target,"predictionMotion") || isempty(target.predictionMotion)
                error("collisionAvoidanceController:missingPredictionMotion", ...
                    "Every target requires an nrmm-motion-v1 contract.");
            end
            motion = target.predictionMotion;
            if ~isstruct(motion) || ~isscalar(motion) || ~isfield(motion,"kind") ...
                    || ~isscalar(string(motion.kind)) || string(motion.kind)~="nrmm-motion-v1"
                error("collisionAvoidanceController:invalidEncounterContract","Invalid target motion contract.");
            end
            if ~isfield(motion,"curvatureMaximum")
                error("collisionAvoidanceController:invalidEncounterContract", ...
                    "An NRMM motion contract requires curvatureMaximum.");
            end
            validateattributes(motion.curvatureMaximum,{'double'},{'scalar','real','finite','positive'});
            for name = ["jerkBound","yawAccelerationBound"]
                if isfield(motion,name) && any(motion.(name)(:)~=0)
                    error("collisionAvoidanceController:invalidEncounterContract", ...
                        "An NRMM motion contract describes exact NRMM motion; a model error has no contract.");
                end
            end
            contract = struct("kind","nrmm-motion-v1","id",target.key, ...
                "validFrom",time,"validityScope","whileEncounterActive", ...
                "curvatureMaximum",motion.curvatureMaximum, ...
                "predictionSampleTime",cfg.controller.sampleTime);
            if isfield(motion,"speedRateMaximum")
                validateattributes(motion.speedRateMaximum,{'double'},{'scalar','real','finite','nonnegative'});
                contract.speedRateMaximum = motion.speedRateMaximum;
            end
            if isfield(motion,"scalarAccelerationMaximum")
                validateattributes(motion.scalarAccelerationMaximum,{'double'},{'scalar','finite','nonnegative'});
                contract.scalarAccelerationMaximum = motion.scalarAccelerationMaximum;
            end
            validateattributes(time,{'double'},{'real','finite','scalar'});
            if target.key=="anonymousTarget"
                error("collisionAvoidanceController:invalidEncounterContract","Finite encounters need stable track identity.");
            end
            encounter = localEncounter(target,time,contract);
            published = [];
            if isfield(target,"parameterErrorBounds"),published = target.parameterErrorBounds;end
            encounter.parameters = targetPrediction.nrmmParameters(encounter.center,encounter.radius,contract,published);
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
        %finiteFlow Box enclosing every NRMM path of the encounter's parameter
        % intervals through its position box at the prediction times
        % (targetPrediction.deviationModel).
            duration = double(duration(:).');
            if any(~isfinite(duration) | duration < 0)
                error("collisionAvoidanceController:invalidPredictionTime", "Prediction times must be finite and nonnegative.");
            end
            [center,radius] = localNrmmFlow(encounter,duration);
        end

        function motion = deviationModel(encounter,duration)
        %deviationModel Target deviation from its nominal NRMM flow, for reactive tubes.
        % At each time: center (8 x n) is the nominal state; parameterGenerators
        % (6 x g x n) map the NRMM parameter errors [p0; V; A; course; curvature],
        % fixed at admission, to the deviation of [position; velocity;
        % acceleration] through the sensitivities of the NRMM path; remainder
        % (6 x n) bounds their second-order term, or the path-length ball where
        % the linear model does not apply.
            duration = double(duration(:).');
            state = localNrmmState(encounter,duration,true);
            generators = pagemtimes(state.sensitivity,diag(state.parameterRadius));
            remainder = state.remainder+64*eps*(1+abs(state.center(1:6,:)) ...
                +reshape(sum(abs(generators),2),6,[]));
            motion = struct('center',state.center,'parameterGenerators',generators,'remainder',remainder);
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
                % A tighter or equal contract is adopted; a larger declared
                % maximum (hardEncounterBarrier: motion bounds increased) or a
                % changed footprint is not a continuation of this encounter.
                names = ["curvatureMaximum","speedRateMaximum","scalarAccelerationMaximum"];
                increased = arrayfun(@(name) localCap(measured.contract,name)>localCap(encounter.contract,name),names);
                if any(increased) || measured.halfLength ~= encounter.halfLength ...
                        || measured.halfWidth ~= encounter.halfWidth
                    error("collisionAvoidanceController:changedEncounterContract", ...
                        "Physical target motion bounds changed.");
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
                measured.parameters = localConditionParameters(encounter.parameters,duration,measured);
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
end

function distance = localArc(time, speed, acceleration)
    if acceleration < 0.0
        time = min(time, speed/-acceleration);
    end
    distance = speed*time+0.5*acceleration*time.^2;
end

function value = localCap(contract,name)
% A declared contract maximum; Inf when it is not declared.
    value = Inf;
    if isfield(contract,name),value = contract.(name);end
end

function [center,radius] = localNrmmFlow(encounter,duration)
% Box enclosure of every NRMM path of the parameter intervals through the
% position box: the linearization of the path in the parameters with its
% Lagrange second-order remainder, or the path-length ball where the
% linearization does not apply (targetPrediction.deviationModel).
    state = localNrmmState(encounter,duration,true);
    center = state.center;count = numel(duration);
    radius = zeros(8,count);
    radius(1:6,:) = reshape(pagemtimes(abs(state.sensitivity),state.parameterRadius),6,[])+state.remainder;
    radius(7,:) = state.yawRadius;radius(8,:) = state.yawRateRadius;
    radius = radius+64*eps*(1+abs(center)+radius);
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
    linear = speedLow>0 & courseRadius<=0.5 & speedLow+min(0,tangential-dA)*t>0;
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

