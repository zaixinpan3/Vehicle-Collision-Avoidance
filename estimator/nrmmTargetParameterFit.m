function value = nrmmTargetParameterFit(action, varargin)
%nrmmTargetParameterFit Contract-consistent window fit of the target motion parameters.
% The target contract (NRMM) keeps the tangential acceleration A and the
% sideslip beta constant, so the path has constant curvature kappa =
% sin(beta)/lr and the closed-form trajectory of
% predictiveSafetyGeometry.predictTarget. The high-gain tracker does not
% use this constancy: its A and beta follow from its acceleration state,
% which differentiates radar noise twice and has a transient after every
% initialization. This fit uses it.
%
% Each radar detection is kept with the ego position estimated at its time
% and the gyro rate. At a solve the ego heading at every detection is the
% current yaw estimate minus the integrated gyro (trapezoidal) back to that
% detection. Within the window these relative headings are accurate to the
% gyro noise. A constant yaw error e rotates the target path and adds e
% times the ego displacement, so A and kappa move by e times the ego
% acceleration only; the fitted course moves by about e. (Headings
% estimated at each detection would let the yaw observer's corrections,
% times a range of 40 m, bend the track into a false curvature.) Gauss-Newton fits theta =
% [X; Y; course; V; A; kappa] at the solve time to the detections of the
% last `duration` seconds, started from a quadratic polynomial fit. Model
% selection then keeps A and kappa only where they exceed three standard
% errors of the fit (from its residual variance); otherwise the constant-
% speed or straight member of the contract is refitted with that parameter
% at zero. A straight target thus keeps a straight forecast while the window
% is short, and the test passes any parameter once the window has made it
% significant.
%
%   fit = nrmmTargetParameterFit("initialize", duration, minimumDuration, domain)
%   fit = nrmmTargetParameterFit("append", fit, time, egoPosition, radarRelative, yawRate)
%   fit = nrmmTargetParameterFit("reset", fit)
%   solution = nrmmTargetParameterFit("solve", fit, time, yaw, yawRate)
%
% solve takes the yaw estimate and gyro rate at time (not before the last
% detection). The solution holds the fitted state at time (position,
% course, speed), A, kappa and beta clipped to the target domain, the
% residual root mean square, the standard errors, the selected parameters,
% and available = the window spans at least minimumDuration. It is an
% estimate, not an enclosure; the observer's certified sets are unchanged.

    switch string(action)
        case "initialize"
            [duration,minimumDuration,domain] = varargin{:};
            validateattributes(duration,{'double'},{'scalar','real','finite','positive'});
            validateattributes(minimumDuration,{'double'},{'scalar','real','finite','positive','<=',duration});
            value = struct("duration",duration,"minimumDuration",minimumDuration, ...
                "time",zeros(1,0),"egoPosition",zeros(2,0),"radar",zeros(2,0),"yawRate",zeros(1,0), ...
                "rearAxleDistance",domain.rearAxleDistance,"sideslipMaximum",domain.sideslipMaximum, ...
                "speedMinimum",domain.speedMinimum,"speedMaximum",domain.speedMaximum, ...
                "scalarAccelerationMaximum",domain.scalarAccelerationMaximum);
        case "append"
            [value,time,egoPosition,radar,yawRate] = varargin{:};
            if ~isempty(value.time) && time <= value.time(end)
                error("nrmmTargetParameterFit:timeOrder","Detections must arrive in increasing time.");
            end
            if ~isfinite(time) || numel(egoPosition)~=2 || numel(radar)~=2 ...
                    || any(~isfinite([egoPosition(:);radar(:);yawRate]))
                error("nrmmTargetParameterFit:invalidDetection","A detection needs finite data.");
            end
            keep = value.time >= time-value.duration-1e-9;
            value.time = [value.time(keep),time];
            value.egoPosition = [value.egoPosition(:,keep),egoPosition(:)];
            value.radar = [value.radar(:,keep),radar(:)];
            value.yawRate = [value.yawRate(keep),yawRate];
        case "reset"
            value = varargin{1};
            value.time = zeros(1,0);value.egoPosition = zeros(2,0);
            value.radar = zeros(2,0);value.yawRate = zeros(1,0);
        case "solve"
            value = localSolve(varargin{:});
        otherwise
            error("nrmmTargetParameterFit:invalidAction","Unknown action.");
    end
end

function solution = localSolve(fit,time,yaw,yawRate)
    solution = struct("available",false,"time",time,"position",NaN(2,1),"course",NaN, ...
        "speed",NaN,"acceleration",NaN,"curvature",NaN,"sideslip",NaN, ...
        "residualRms",NaN,"residualVariance",NaN,"standardError",NaN(6,1), ...
        "samples",numel(fit.time),"span",0,"iterations",0,"selected",true(6,1));
    keep = fit.time >= time-fit.duration-1e-9 & fit.time <= time+1e-9;
    relative = fit.time(keep)-time;count = numel(relative);
    if count < 4,return;end
    solution.span = relative(end)-relative(1);
    if solution.span < fit.minimumDuration-1e-9,return;end
    % Headings by the gyro, integrated back from the current yaw estimate.
    rates = [fit.yawRate(keep),yawRate];times = [relative,0];
    increments = (rates(1:end-1)+rates(2:end)).*diff(times)/2;
    heading = yaw-fliplr(cumsum(fliplr(increments)));
    radar = fit.radar(:,keep);
    positions = fit.egoPosition(:,keep)+[cos(heading).*radar(1,:)-sin(heading).*radar(2,:); ...
        sin(heading).*radar(1,:)+cos(heading).*radar(2,:)];
    % Quadratic polynomial start: velocity and acceleration at the window end.
    basis = [ones(count,1),relative(:),relative(:).^2/2];
    coefficients = basis\positions.';
    velocity = coefficients(2,:).';acceleration = coefficients(3,:).';
    speed = max(norm(velocity),fit.speedMinimum);
    theta = [coefficients(1,:).';atan2(velocity(2),velocity(1));speed; ...
        acceleration.'*velocity/speed;(velocity(1)*acceleration(2)-velocity(2)*acceleration(1))/speed^3];
    theta = localClip(theta,fit);
    target = positions(:);
    free = true(6,1);
    [theta,residual,cost,iterations] = localGaussNewton(theta,free,relative,target,fit);
    standardError = localStandardError(theta,free,residual,cost,relative,target,fit);
    % Model selection inside the contract: A and kappa (entries 5 and 6)
    % stay free only where they exceed three standard errors.
    selected = free;selected(5:6) = abs(theta(5:6)) > 3*standardError(5:6);
    if ~all(selected)
        reduced = theta;reduced(~selected) = 0;
        [theta,residual,cost,more] = localGaussNewton(reduced,selected,relative,target,fit);
        iterations = iterations+more;
        standardError = localStandardError(theta,selected,residual,cost,relative,target,fit);
    end
    solution.iterations = iterations;solution.selected = selected;
    solution.available = true;
    solution.position = theta(1:2);solution.course = theta(3);solution.speed = theta(4);
    solution.acceleration = theta(5);solution.curvature = theta(6);
    solution.sideslip = asin(theta(6)*fit.rearAxleDistance);
    solution.residualRms = sqrt(cost/numel(residual));
    solution.residualVariance = cost/max(1,numel(residual)-nnz(selected));
    solution.standardError = standardError;
end

function [theta,residual,cost,iterations] = localGaussNewton(theta,free,relative,target,fit)
% Levenberg-damped Gauss-Newton over the free entries of theta.
    residual = localResidual(theta,relative,target,fit);cost = residual.'*residual;
    damping = 1e-6;iterations = 0;
    for iteration = 1:12
        jacobian = localJacobian(theta,free,residual,relative,target,fit);
        normal = jacobian.'*jacobian;gradient = jacobian.'*residual;
        step = zeros(6,1);
        step(free) = -(normal+damping*diag(max(diag(normal),eps)))\gradient;
        candidate = localClip(theta+step,fit);
        candidateResidual = localResidual(candidate,relative,target,fit);
        candidateCost = candidateResidual.'*candidateResidual;
        iterations = iteration;
        if candidateCost <= cost
            converged = abs(cost-candidateCost) <= 1e-12*max(1,cost);
            theta = candidate;residual = candidateResidual;cost = candidateCost;
            damping = max(damping/10,1e-9);
            if converged || norm(step) < 1e-10,break;end
        else
            damping = damping*10;
            if damping > 1e6,break;end
        end
    end
end

function standardError = localStandardError(theta,free,residual,cost,relative,target,fit)
% Standard errors of the free entries from the residual variance; zero for
% entries held at zero.
    variance = cost/max(1,numel(residual)-nnz(free));
    jacobian = localJacobian(theta,free,residual,relative,target,fit);
    covariance = variance*pinv(jacobian.'*jacobian);
    standardError = zeros(6,1);
    standardError(free) = sqrt(max(diag(covariance),0));
end

function jacobian = localJacobian(theta,free,residual,relative,target,fit)
    columns = find(free);
    jacobian = zeros(numel(residual),numel(columns));
    for index = 1:numel(columns)
        column = columns(index);
        delta = zeros(6,1);delta(column) = 1e-6*max(1,abs(theta(column)));
        jacobian(:,index) = (localResidual(theta+delta,relative,target,fit)-residual)/delta(column);
    end
end

function theta = localClip(theta,fit)
    theta(4) = min(max(theta(4),fit.speedMinimum),fit.speedMaximum);
    theta(5) = min(max(theta(5),-fit.scalarAccelerationMaximum),fit.scalarAccelerationMaximum);
    limit = sin(fit.sideslipMaximum)/fit.rearAxleDistance;
    theta(6) = min(max(theta(6),-limit),limit);
end

function residual = localResidual(theta,relative,target,fit)
    beta = asin(theta(6)*fit.rearAxleDistance);
    q = [theta(1:2);theta(3)-beta;theta(4);theta(5);beta;fit.rearAxleDistance];
    predicted = predictiveSafetyGeometry.predictTarget(q,relative);
    residual = reshape(predicted(1:2,:),[],1)-target;
end
