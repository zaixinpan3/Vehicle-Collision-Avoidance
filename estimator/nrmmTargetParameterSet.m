function set = nrmmTargetParameterSet(history, time, center, domain, prior, previous, options)
%nrmmTargetParameterSet Certified constant-parameter set of an NRMM target.
% Under the target contract (constant tangential acceleration A and constant
% sideslip beta) the target's whole path is fixed by six constants. This
% function encloses them at `time`, in the true ego body frame at that time
% (the frame of the tracker's relative state):
%   theta = [x; y; chi; V; A; kappa],  kappa = sin(beta)/lr,
% with p(tau) = [x; y] + s*sinc(kappa*s/2)*[cos; sin](chi + kappa*s/2) and
% s = V*tau + A*tau^2/2 (the controller's predictTarget family).
%
% Every recorded radar detection j of the history window is transported into
% that frame. Two certified transports exist, and each detection uses the one
% with the smaller radius:
%   gyro:     z_j = R(psih)'(g_j - g_k) + R(alpha_j)*rho_j, alpha_j the gyro
%             integral of psi_j - psi (trapezoid; the history's increment
%             radius gyroNoise*dt + yawAccelerationMaximum*dt^2/4), radius
%             radar + gnss + 2 sin(gyro radius/2)|rho_j|;
%   heading:  z_j = R(psih)'(g_j + R(psih_j)*rho_j - g_k), psih_j the
%             detection's own certified heading (radius b_j), radius
%             radar + gnss + 2 sin(b_j/2)|rho_j|;
% g the GNSS ego positions, rho_j the radar relative positions and psih the
% current heading estimate. The current heading error phi (|phi| <= b, the
% last record's headingRadius) and the current GNSS error c (|c| <= gnssNoise)
% are common to all past detections and enter the linear program as nuisance
% variables. Their second-order effect adds |D_j|(b^2/2 + b^3/6) + b(r_j + gnss)
% to the radius, D_j the part rotated by phi and r_j the radius above.
% The family is linearized at `center`; its second-order remainder is bounded
% over the current domain box with the Taylor bound of
% predictiveSafetyGeometry.targetPositionSupport, the arc length inflated by
% its own error. The Euclidean residual balls are replaced by circumscribed
% octagons. A linear program that fails numerically (after a second
% algorithm) keeps its whole slice. The course and curvature ranges are split into slices of at most
% options.courseSliceWidth and options.curvatureSliceWidth (at most
% options.maximumSlices in all), each linearized about its own values; a slice
% whose linear program is empty cannot explain the detections and is
% discarded. Twelve linear programs per remaining slice give its box of theta,
% and the union of these boxes is intersected with the domain box. The
% domain box is
%   the observer's current set (`prior`, certified),
%   intersect the contract (speed, |A|, |kappa| <= sin(betaMax)/lr),
%   intersect the previous set's frame-invariant constants carried forward
%   (A and kappa unchanged, V widened by the A interval times the elapsed time).
% Because the remainder bound is valid on the domain and the truth lies in
% it, the result again contains the truth; the computation is repeated on the
% smaller box (options.iterations). An infeasible program means that a
% premise (contract, noise bounds, transport bound) is violated; the set is
% then reported unavailable and nothing is intersected.
%
% Inputs
%   history  nrmmTargetHistory state with records (time, relativePosition,
%            egoPosition, yawRate, heading, headingRadius), gyroscopeNoise
%            and yawAccelerationMaximum.
%   center   struct relativePosition (2x1), course (ego frame), speed,
%            acceleration, curvature: the tracker's estimate at `time`.
%   domain   target contract: speedMinimum, speedMaximum,
%            scalarAccelerationMaximum, sideslipMaximum, rearAxleDistance.
%   prior    struct positionRadius, courseRadius, speedInterval,
%            accelerationInterval, curvatureInterval (absolute intervals).
%   previous an earlier output of this function, or [].
%   options  radarNoise, gnssNoise, maximumMeasurements, iterations,
%            courseSliceWidth, curvatureSliceWidth, maximumSlices.
% Output fields: available, reason, time, center, lower and upper (offsets
% of theta from the center), speedInterval, accelerationInterval,
% curvatureInterval (absolute), measurements, iterations, slices, transport,
% maximumRemainder.
% The set is conditional on the stated noise, transport and contract
% premises and on the linear-program solutions; it is not floating-point
% verified.
    arguments
        history (1,1) struct
        time (1,1) double {mustBeReal,mustBeFinite}
        center (1,1) struct
        domain (1,1) struct
        prior (1,1) struct
        previous
        options (1,1) struct
    end
    names = ["x","y","course","speed","acceleration","curvature"];
    theta = [center.relativePosition(:);center.course;center.speed;center.acceleration;center.curvature];
    set = struct('kind',"nrmm-constant-parameter-membership-v1",'available',false,'reason',"", ...
        'time',time,'center',theta,'names',names,'lower',-inf(6,1),'upper',inf(6,1), ...
        'speedInterval',[-Inf;Inf],'accelerationInterval',[-Inf;Inf],'curvatureInterval',[-Inf;Inf], ...
        'measurements',0,'iterations',0,'slices',0,'solverFailures',0,'transport',strings(1,0),'maximumRemainder',NaN, ...
        'scope',"conditional on the noise, gyro-transport and constant-parameter premises");
    records = history.records;
    if isempty(records),set.reason = "noDetections";return;end
    last = records(end);
    if abs(last.time-time) > 1e-9*max(1,abs(time)),set.reason = "noCurrentDetection";return;end
    if ~isfinite(history.yawAccelerationMaximum) || ~isfinite(last.headingRadius) || last.headingRadius >= pi/4
        set.reason = "noTransportBound";return;
    end
    if any(~isfinite(theta)),set.reason = "invalidCenter";return;end
    % Domain box (absolute), then relative to the center.
    curvatureLimit = sin(domain.sideslipMaximum)/domain.rearAxleDistance;
    lowerAbsolute = [theta(1:2)-prior.positionRadius;theta(3)-prior.courseRadius; ...
        max(domain.speedMinimum,prior.speedInterval(1)); ...
        max(-domain.scalarAccelerationMaximum,prior.accelerationInterval(1)); ...
        max(-curvatureLimit,prior.curvatureInterval(1))];
    upperAbsolute = [theta(1:2)+prior.positionRadius;theta(3)+prior.courseRadius; ...
        min(domain.speedMaximum,prior.speedInterval(2)); ...
        min(domain.scalarAccelerationMaximum,prior.accelerationInterval(2)); ...
        min(curvatureLimit,prior.curvatureInterval(2))];
    if isstruct(previous) && isfield(previous,'available') && isequal(previous.available,true) ...
            && previous.time <= time
        elapsed = time-previous.time;
        a = previous.accelerationInterval;
        lowerAbsolute(4:6) = max(lowerAbsolute(4:6),[previous.speedInterval(1)+min(a*elapsed);a(1); ...
            previous.curvatureInterval(1)]);
        upperAbsolute(4:6) = min(upperAbsolute(4:6),[previous.speedInterval(2)+max(a*elapsed);a(2); ...
            previous.curvatureInterval(2)]);
    end
    if any(~isfinite([lowerAbsolute;upperAbsolute])) || prior.courseRadius >= pi/2
        set.reason = "unboundedDomain";return;
    end
    if any(lowerAbsolute > upperAbsolute),set.reason = "emptyDomain";return;end
    lower = lowerAbsolute-theta;upper = upperAbsolute-theta;
    % Detections transported into the current ego frame.
    times = [records.time];count = numel(records);
    gyro = [records.yawRate];dt = diff(times);
    increments = (gyro(1:end-1)+gyro(2:end)).*dt/2;
    angle = -[fliplr(cumsum(fliplr(increments))),0];
    angleRadius = [fliplr(cumsum(fliplr(history.gyroscopeNoise*dt ...
        +history.yawAccelerationMaximum*dt.^2/4))),0];
    selected = unique(round(linspace(1,count,min(count,options.maximumMeasurements))));
    b = last.headingRadius;rotation = localRotation(last.heading);
    m = numel(selected);
    tau = times(selected)-time;z = zeros(2,m);w = zeros(2,m);radius = zeros(1,m);common = false(1,m);
    transport = strings(1,m);
    for index = 1:m
        j = selected(index);
        rho = records(j).relativePosition(:);
        common(index) = j < count;
        if ~common(index)
            % The current detection is already in the current body frame.
            z(:,index) = rho;radius(index) = options.radarNoise;transport(index) = "current";
            continue;
        end
        d = rotation.'*(records(j).egoPosition(:)-last.egoPosition(:));
        gyroRadius = options.radarNoise+options.gnssNoise+2*sin(min(pi,angleRadius(j))/2)*norm(rho);
        headingRadius = options.radarNoise+options.gnssNoise+2*sin(min(pi,records(j).headingRadius)/2)*norm(rho);
        if gyroRadius <= headingRadius
            rotated = d;z(:,index) = d+localRotation(angle(j))*rho;
            individual = gyroRadius;transport(index) = "gyro";
        else
            z(:,index) = rotation.'*(records(j).egoPosition(:)+localRotation(records(j).heading)*rho ...
                -last.egoPosition(:));
            rotated = z(:,index);individual = headingRadius;transport(index) = "heading";
        end
        w(:,index) = [-rotated(2);rotated(1)];
        % phi also rotates the individual and common errors: b*(r_j + gnss).
        radius(index) = individual*(1+b)+b*options.gnssNoise+norm(rotated)*(b^2/2+b^3/6);
    end
    set.transport = transport;
    set.measurements = m;
    octagon = [cos((0:7)*pi/4);sin((0:7)*pi/4)];
    opts = localLinprogOptions();
    remainderMaximum = NaN;slices = 0;failures = 0;
    for iteration = 1:options.iterations
        % The course and curvature nonlinearities dominate the remainder.
        % Split both ranges into slices, linearize each slice about its own
        % course and curvature, discard the slices that no parameter in them
        % can explain, and take the union of the others. Each slice is a
        % sound enclosure of the truth restricted to it, so the union
        % contains the truth.
        courseCount = max(1,ceil((upper(3)-lower(3))/options.courseSliceWidth));
        curvatureCount = max(1,ceil((upper(6)-lower(6))/options.curvatureSliceWidth));
        scale = sqrt(courseCount*curvatureCount/options.maximumSlices);
        if scale > 1
            courseCount = max(1,floor(courseCount/scale));curvatureCount = max(1,floor(curvatureCount/scale));
        end
        courseEdges = lower(3)+(upper(3)-lower(3))*(0:courseCount)/courseCount;
        curvatureEdges = lower(6)+(upper(6)-lower(6))*(0:curvatureCount)/curvatureCount;
        unionLower = inf(6,1);unionUpper = -inf(6,1);
        for courseSlice = 1:courseCount
            for curvatureSlice = 1:curvatureCount
                sliceLower = lower;sliceUpper = upper;
                sliceLower([3,6]) = [courseEdges(courseSlice);curvatureEdges(curvatureSlice)];
                sliceUpper([3,6]) = [courseEdges(courseSlice+1);curvatureEdges(curvatureSlice+1)];
                offset = zeros(6,1);offset([3,6]) = (sliceLower([3,6])+sliceUpper([3,6]))/2;
                local = theta+offset;
                [points,gradients] = localFamily(local,tau);
                remainder = localRemainder(local,tau,sliceLower-offset,sliceUpper-offset);
                remainderMaximum = max([remainderMaximum,remainder],[],'omitnan');
                A = zeros(8*m,9);rhs = zeros(8*m,1);
                for index = 1:m
                    rows = 8*index-7:8*index;
                    A(rows,1:6) = octagon.'*gradients(:,:,index);
                    A(rows,7) = -octagon.'*w(:,index);
                    if common(index),A(rows,8:9) = -octagon.';end
                    rhs(rows) = radius(index)+remainder(index)-octagon.'*(points(:,index)-z(:,index));
                end
                boundsLower = [sliceLower-offset;-b;-options.gnssNoise*[1;1]];
                boundsUpper = [sliceUpper-offset;b;options.gnssNoise*[1;1]];
                [low,high,flag] = localBox(A,rhs,boundsLower,boundsUpper,opts);
                slices = slices+1;
                if flag == -2,continue;end
                if flag ~= 1
                    % A numerical solver failure proves nothing about the
                    % slice; keep all of it, which remains a sound union.
                    failures = failures+1;
                    low = sliceLower-offset;high = sliceUpper-offset;
                end
                % A small outward guard covers the solver's feasibility tolerance.
                guard = 1e-7*max(1,abs(local));
                unionLower = min(unionLower,offset+low-guard);
                unionUpper = max(unionUpper,offset+high+guard);
            end
        end
        set.iterations = iteration;set.maximumRemainder = remainderMaximum;
        set.slices = slices;set.solverFailures = failures;
        newLower = max(lower,unionLower);newUpper = min(upper,unionUpper);
        if any(newLower > newUpper)
            set.reason = "inconsistentMeasurements";return;
        end
        % Continue while some component still shrinks appreciably.
        shrink = min((newUpper-newLower)./max(upper-lower,eps));
        lower = newLower;upper = newUpper;
        if shrink > 0.9,break;end
    end
    set.available = true;set.reason = "solved";
    set.lower = lower;set.upper = upper;set.maximumRemainder = remainderMaximum;
    set.speedInterval = theta(4)+[lower(4);upper(4)];
    set.accelerationInterval = theta(5)+[lower(5);upper(5)];
    set.curvatureInterval = theta(6)+[lower(6);upper(6)];
end

function [low,high,flag] = localBox(A,rhs,lower,upper,opts)
% Componentwise extent of the first six variables over the polytope.
% flag: 1 solved, -2 empty (the slice cannot explain the detections; only
% a certificate of the default solver is accepted), 0 solver failure.
    low = zeros(6,1);high = zeros(6,1);flag = 1;
    for component = 1:6
        objective = zeros(numel(lower),1);objective(component) = 1;
        [value,exitLow] = localSolve(objective,A,rhs,lower,upper,opts);
        if exitLow == -2,flag = -2;return;end
        [negative,exitHigh] = localSolve(-objective,A,rhs,lower,upper,opts);
        if exitLow ~= 1 || exitHigh ~= 1,flag = 0;return;end
        low(component) = value;high(component) = -negative;
    end
end

function [value,exitFlag] = localSolve(objective,A,rhs,lower,upper,opts)
    [~,value,exitFlag] = linprog(objective,A,rhs,[],[],lower,upper,opts.primary);
    if exitFlag ~= 1 && exitFlag ~= -2
        [~,value,exitFlag] = linprog(objective,A,rhs,[],[],lower,upper,opts.fallback);
        if exitFlag == -2,exitFlag = 0;end
    end
end

function [points,gradients] = localFamily(theta,tau)
% Positions and analytic first derivatives of the constant-parameter family.
    count = numel(tau);points = zeros(2,count);gradients = zeros(2,6,count);
    chi = theta(3);kappa = theta(6);
    for index = 1:count
        t = tau(index);s = theta(4)*t+.5*theta(5)*t^2;a = kappa*s/2;
        [sincValue,sincSlope] = localSinc(a);
        f = s*sincValue;u = chi+a;
        e = [cos(u);sin(u)];normal = [-sin(u);cos(u)];
        points(:,index) = theta(1:2)+f*e;
        tangent = [cos(chi+2*a);sin(chi+2*a)];
        dfdk = s*sincSlope*s/2;
        gradients(:,:,index) = [[1;0],[0;1],f*normal,t*tangent,.5*t^2*tangent, ...
            dfdk*e+f*(s/2)*normal];
    end
end

function [value,slope] = localSinc(a)
    if abs(a) < 1e-4
        value = 1-a^2/6+a^4/120;slope = -a/3+a^3/30;
    else
        value = sin(a)/a;slope = (a*cos(a)-sin(a))/a^2;
    end
end

function remainder = localRemainder(theta,tau,lower,upper)
% Taylor remainder of the family over the box, as in
% predictiveSafetyGeometry.targetPositionSupport with the arc length
% inflated by its own error.
    dg = max(abs([lower(3),upper(3)]));dk = max(abs([lower(6),upper(6)]));
    dv = max(abs([lower(4),upper(4)]));da = max(abs([lower(5),upper(5)]));
    s = theta(4)*tau+.5*theta(5)*tau.^2;
    ds = dv*abs(tau)+.5*da*tau.^2;
    length = abs(s)+ds;
    remainder = .5*length*dg^2+.5*length.^2*dg*dk+length.^3*dk^2/6 ...
        +ds.*(dg+length*dk)+.5*(abs(theta(6))+dk)*ds.^2;
end

function rotation = localRotation(angle)
    rotation = [cos(angle),-sin(angle);sin(angle),cos(angle)];
end

function opts = localLinprogOptions()
    persistent cached
    if isempty(cached)
        cached = struct('primary',optimoptions('linprog','Display','off'), ...
            'fallback',optimoptions('linprog','Display','off','Algorithm','interior-point'));
    end
    opts = cached;
end
