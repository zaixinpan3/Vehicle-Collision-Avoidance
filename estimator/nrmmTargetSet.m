function result = nrmmTargetSet(action, varargin)
% nrmmTargetSet Analytic outer boxes for physical [rho;q;s] information.
% All approximations use explicit bounded-jerk Taylor remainders. A nonempty
% box does NOT establish existence of an exactly measurement-consistent CCA
% trajectory. A local optimizer is never used to prune the hard set.

    switch string(action)
        case "initialize"
            domain = varargin{1};
            bound = repelem([domain.relativePositionMaximum; ...
                domain.speedMaximum; domain.accelerationNormBound], 2);
            result = struct("lower", -bound, "upper", bound);
        case "intersect"
            result = varargin{1};
            result.lower = max(result.lower, varargin{2}.lower);
            result.upper = min(result.upper, varargin{2}.upper);
        case "predict"
            result = localPredict(varargin{:});
        case "window"
            result = localWindow(varargin{:});
        case "radius"
            box = varargin{1};
            point = varargin{2};
            if any(box.lower > box.upper)
                result = Inf(3, 1);
            else
                componentError = max(abs(box.lower-point), abs(box.upper-point));
                result = vecnorm(reshape(componentError, 2, 3), 2, 1).';
            end
        otherwise
            error("nrmmTargetSet:invalidAction", "Unknown target-set action.");
    end
end

function box = localPredict(box, duration, motion, design)
    if any(box.lower > box.upper)
        return
    end
    domain = design.target.domain;
    chain = kron([1, duration, duration^2/2; 0, 1, duration; 0, 0, 1], eye(2));
    centre = chain*((box.lower+box.upper)/2);
    radius = abs(chain)*((box.upper-box.lower)/2) ...
        +repelem(domain.jerkNormBound*[duration^3/6; duration^2/2; duration], 2);
    centre(1:2) = centre(1:2)-motion.translation;
    radius(1:2) = radius(1:2)+motion.translationErrorMaximum;
    angle = motion.yawIncrement;
    rotation = [cos(angle), sin(angle); -sin(angle), cos(angle)];
    transform = kron(eye(3), rotation);
    centre = transform*centre;
    radius = abs(transform)*radius+repelem( ...
        2*sin(min(pi, motion.yawErrorMaximum)/2)*[domain.relativePositionMaximum; ...
        domain.speedMaximum; domain.accelerationNormBound], 2) ...
        +design.configuration.window.numericalAllowance;
    box = struct("lower", centre-radius, "upper", centre+radius);
    box = nrmmTargetSet("intersect", box, nrmmTargetSet("initialize", domain));
end

function box = localWindow(time, points, errorRadius, queryTime, pose, design)
% Position Taylor equations about the query time: z_j = p-d_j*q+d_j^2*s/2
% plus a radius epsilon_j+Jmax*d_j^3/6. Multiplying by the inverse of the
% three-time matrix gives conservative component bounds on p, q, and s.
    domain = design.target.domain;
    speed = domain.speedMaximum;
    acceleration = domain.accelerationNormBound;
    jerk = domain.jerkNormBound;
    pad = design.configuration.window.numericalAllowance;
    box = struct("lower", [-Inf; -Inf; -speed; -speed; -acceleration; -acceleration], ...
        "upper", [Inf; Inf; speed; speed; acceleration; acceleration]);
    count = numel(time);
    if count == 0
        box = nrmmTargetSet("initialize", domain);
        return
    end
    delay = queryTime-time(:);
    errorRadius = errorRadius(:);
    for start = unique(max(1, [1, floor(count/2), floor(3*count/4)]))
        indices = unique([start, floor((start+count)/2), count]);
        if numel(indices) == 3
            lag = delay(indices);
            matrix = [ones(3, 1), -lag, lag.^2/2];
            if rcond(matrix) > 1.0e-10
                weights = matrix\eye(3);
                centre = points(:, indices)*weights.';
                radius = abs(weights)*(errorRadius(indices)+jerk*lag.^3/6)+pad;
                candidate = struct("lower", centre(:)-repelem(radius, 2), ...
                    "upper", centre(:)+repelem(radius, 2));
                box = nrmmTargetSet("intersect", box, candidate);
            end
        end
        if start < count
            interval = time(end)-time(start);
            centre = (points(:, end)-points(:, start))/interval;
            radius = (errorRadius(end)+errorRadius(start))/interval ...
                +acceleration*(delay(start)+delay(end))/2+pad;
            box.lower(3:4) = max(box.lower(3:4), centre-radius);
            box.upper(3:4) = min(box.upper(3:4), centre+radius);
        end
    end
    % Every radar point supplies a position constraint at the query time.
    for index = 1:count
        lag = delay(index);
        centre = points(:, index);
        radius = errorRadius(index)+speed*lag+pad;
        box.lower(1:2) = max(box.lower(1:2), centre-radius);
        box.upper(1:2) = min(box.upper(1:2), centre+radius);
    end
    lag = delay(end);
    box.lower(1:2) = max(box.lower(1:2), points(:, end) ...
        +lag*box.lower(3:4)-lag^2*box.upper(5:6)/2 ...
        -errorRadius(end)-jerk*lag^3/6-pad);
    box.upper(1:2) = min(box.upper(1:2), points(:, end) ...
        +lag*box.upper(3:4)-lag^2*box.lower(5:6)/2 ...
        +errorRadius(end)+jerk*lag^3/6+pad);
    if any(box.lower > box.upper)
        return
    end
    % Transform from the fixed window frame to the uncertain query ego frame.
    centre = (box.lower+box.upper)/2;
    radius = (box.upper-box.lower)/2;
    centre(1:2) = centre(1:2)-pose.translation;
    radius(1:2) = radius(1:2)+pose.translationErrorMaximum;
    rotation = [cos(pose.yawIncrement), sin(pose.yawIncrement); ...
        -sin(pose.yawIncrement), cos(pose.yawIncrement)];
    transform = kron(eye(3), rotation);
    centre = transform*centre;
    radius = abs(transform)*radius+repelem( ...
        2*sin(min(pi, pose.yawErrorMaximum)/2)*[domain.relativePositionMaximum; ...
        speed; acceleration], 2)+pad;
    box = struct("lower", centre-radius, "upper", centre+radius);
    box = nrmmTargetSet("intersect", box, nrmmTargetSet("initialize", domain));
end
