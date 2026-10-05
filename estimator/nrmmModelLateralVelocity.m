function lateral = nrmmModelLateralVelocity(gnssVelocity, yawRate, bodyAcceleration, heldInput, model, sensors, sideslipDomainMaximum)
% nrmmModelLateralVelocity Certified ego lateral body velocity from the force balance.
%
% The ego plant obeys, at the centre of mass,
%
%   m*a_y = Fx_f(b)*sin(d) + Fy_f(alpha_f)*cos(d) + Fy_r(alpha_r),
%   alpha_f = atan2(v_y + lf*r, v_x) - d,   alpha_r = atan2(v_y - lr*r, v_x),
%
% with the same modified Fiala tires that the controller predicts with and
% the held steering d and braking ratio b that the controller applied. The
% accelerometer measures a = R(psi)'*pddot, whose lateral component is a_y.
% Each axle force is non-increasing in v_y, so the right-hand side G(v_y) is
% non-increasing, and the measured a_y determines v_y. This replaces the
% kinematic no-slip relation v_y = l*r, which fails whenever the rear tire
% slips.
%
% Certified interval. The true quantities satisfy |a_y - am_y| <= epsA,
% |r - u| <= epsW, ||v|| in [M - epsG, M + epsG], and the sideslip cone
% |v_y| <= ||v|| sin(b); every cornering stiffness and friction coefficient
% lies within the relative parameterUncertainty of the model value. Each
% axle force is monotone in r, in ||v|| (through v_x), in its cornering
% stiffness and in its friction, so its extremes over that box are attained
% at the box corners. With Gmin, Gmax the resulting per-axle sums (both
% non-increasing in v_y), every admissible v_y satisfies
%
%   Gmin(v_y) <= m*(am_y + epsA)  and  Gmax(v_y) >= m*(am_y - epsA),
%
% which is an interval [lower, upper] found by bisection. It widens near
% tire saturation, where the force no longer determines the slip, and it is
% empty when the measurements contradict the model or the cone.
%
% Point value. The returned center solves the force balance with the
% nominal parameters and the measured values; it lies in [lower, upper],
% and radius = max(center-lower, upper-center) certifies it. The interval
% midpoint is not used as the estimate: near saturation the parameter
% uncertainty makes the interval wide and asymmetric, and its midpoint is
% biased although the nominal solution is not.
%
% Inputs: gnssVelocity (inertial, 2x1), yawRate (gyro), bodyAcceleration
% (2x1), heldInput [d; b], model (mass, lf, lr, gravity, corneringStiffness,
% frictionCoefficient, parameterUncertainty), sensors (velocityNoiseMaximum,
% gyroscopeNoiseMaximum, accelerometerNoiseMaximum), sideslipDomainMaximum.

    speed = norm(gnssVelocity);
    speedLow = speed-sensors.velocityNoiseMaximum;
    speedHigh = speed+sensors.velocityNoiseMaximum;
    sineDomain = sin(sideslipDomainMaximum);
    lateral = struct("center",NaN,"lower",-Inf,"upper",Inf,"radius",Inf, ...
        "consistent",true,"informative",false,"source","force-balance", ...
        "measuredSpeed",speed,"trueSpeedLowerBound",speedLow);
    if ~(speedLow > 0) || numel(heldInput) ~= 2 || any(~isfinite(heldInput)) || abs(heldInput(2)) > 1
        lateral.lower = -speedHigh*sineDomain;lateral.upper = speedHigh*sineDomain;
        lateral.center = 0;lateral.radius = speedHigh*sineDomain;
        return;
    end
    steering = heldInput(1);braking = heldInput(2);
    accelerationLow = model.mass*(bodyAcceleration(2)-sensors.accelerometerNoiseMaximum);
    accelerationHigh = model.mass*(bodyAcceleration(2)+sensors.accelerometerNoiseMaximum);
    rates = yawRate+[-1,1]*sensors.gyroscopeNoiseMaximum;
    spread = model.parameterUncertainty*[-1,1]+1;
    wheelbase = model.lf+model.lr;
    load = model.mass*model.gravity/wheelbase*[model.lr;model.lf];
    eta = sqrt(max(0,1-braking^2));
    % Corner grids: rate x speed x stiffness x friction (2x2x2x2 per axle).
    [rateGrid,speedIndex,stiffnessGrid,frictionGrid] = ndgrid(rates,[1,2],spread,spread);
    rateGrid = rateGrid(:).';speedIndex = speedIndex(:).';
    stiffnessGrid = stiffnessGrid(:).';frictionGrid = frictionGrid(:).';
    longitudinal = braking*model.frictionCoefficient(1)*spread*load(1)*sin(steering);
    longitudinalRange = [min(longitudinal),max(longitudinal)];

    left = -speedHigh*sineDomain;right = speedHigh*sineDomain;
    [lowAtLeft,highAtLeft] = localRange(left);[lowAtRight,highAtRight] = localRange(right);
    % lower: smallest v_y with Gmin(v_y) <= m*(a_y + epsA).
    if lowAtLeft <= accelerationHigh
        lower = left;
    elseif lowAtRight > accelerationHigh
        lower = Inf;
    else
        lower = localBisect(left,right,@(v) localMinimum(v) <= accelerationHigh);
    end
    % upper: largest v_y with Gmax(v_y) >= m*(a_y - epsA).
    if highAtRight >= accelerationLow
        upper = right;
    elseif highAtLeft < accelerationLow
        upper = -Inf;
    else
        upper = localBisect(left,right,@(v) localMaximum(v) < accelerationLow);
    end
    lateral.lower = lower;lateral.upper = upper;
    lateral.consistent = lower <= upper;
    if lateral.consistent
        nominal = localNominal(left,right);
        if ~(isfinite(nominal) && nominal >= lower && nominal <= upper)
            nominal = 0.5*(lower+upper);
        end
        lateral.center = nominal;
        lateral.radius = max(nominal-lower,upper-nominal);
        lateral.informative = isfinite(lateral.radius);
    else
        lateral.center = NaN;lateral.radius = Inf;
    end

    function value = localNominal(leftEnd,rightEnd)
        % Nominal parameters and measured values: G0(v) = m*am_y.
        target = model.mass*bodyAcceleration(2);
        g = @(v) localNominalForce(v);
        if g(leftEnd) < target || g(rightEnd) > target
            value = NaN;return;
        end
        value = localBisect(leftEnd,rightEnd,@(v) g(v) < target);
    end
    function force = localNominalForce(v)
        longitudinalSpeed = sqrt(max(speed^2-v^2,(speed*cos(sideslipDomainMaximum))^2));
        slips = atan2([v+model.lf*yawRate;v-model.lr*yawRate],longitudinalSpeed)-[steering;0];
        forces = localFiala(slips,model.corneringStiffness(:), ...
            model.frictionCoefficient(:).*load*eta);
        force = braking*model.frictionCoefficient(1)*load(1)*sin(steering) ...
            +forces(1)*cos(steering)+forces(2);
    end

    function value = localMinimum(v)
        [value,~] = localRange(v);
    end
    function value = localMaximum(v)
        [~,value] = localRange(v);
    end
    function [low,high] = localRange(v)
        % Per-axle extremes over the corner grid at lateral velocity v.
        % Admissible speeds keep v inside the cone; the search interval ends
        % exactly on the cone boundary at the largest speed.
        speeds = [min(speedHigh,max(speedLow,abs(v)/max(sineDomain,eps))),speedHigh];
        longitudinalSpeed = sqrt(max(speeds(speedIndex).^2-v^2,0));
        frontSlip = atan2(v+model.lf*rateGrid,longitudinalSpeed)-steering;
        rearSlip = atan2(v-model.lr*rateGrid,longitudinalSpeed);
        front = localFiala(frontSlip,model.corneringStiffness(1)*stiffnessGrid, ...
            model.frictionCoefficient(1)*frictionGrid*load(1)*eta)*cos(steering);
        rear = localFiala(rearSlip,model.corneringStiffness(2)*stiffnessGrid, ...
            model.frictionCoefficient(2)*frictionGrid*load(2)*eta);
        low = longitudinalRange(1)+min(front)+min(rear);
        high = longitudinalRange(2)+max(front)+max(rear);
    end
end

function force = localFiala(slip,stiffness,capacity)
% Modified Fiala lateral force, identical to modifiedFialaTire.evaluate.
    tangent = tan(slip);
    force = -capacity.*sign(slip);
    adhesion = capacity > 0 & abs(tangent) < 3*capacity./stiffness;
    q = stiffness(adhesion).*abs(tangent(adhesion))./(3*capacity(adhesion));
    force(adhesion) = -stiffness(adhesion).*tangent(adhesion).*(1-q+q.^2/3);
end

function value = localBisect(left,right,predicate)
% First point of [left,right] where the monotone predicate becomes true,
% to 1e-5 m/s; predicate(left) is false and predicate(right) is true.
    while right-left > 1e-5
        middle = 0.5*(left+right);
        if predicate(middle),right = middle;else,left = middle;end
    end
    value = right;
end
