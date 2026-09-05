function fit = fitNrmmTrajectoryWindow(time, position, queryTime, prior, design)
% fitNrmmTrajectoryWindow Three-parameter isotropic least-squares trajectory fit.
% Positions (2-by-N) and prior physical state use one FIXED window frame.
% Translation and initial course are eliminated by planar Procrustes alignment.
% This is a nominal fit, never a feasible-set or convergence certificate.

    time = double(time(:));
    cfg = design.configuration;
    domain = design.target.domain;
    fit = struct("available", false, "status", "insufficientWindow", ...
        "state", prior, "parameters", NaN(6, 1), ...
        "residual", NaN(size(time)), "exitFlag", 0, ...
        "objective", Inf, "conditionNumber", Inf, "sampleCount", numel(time));
    if size(position, 1) ~= 2 || size(position, 2) ~= numel(time) ...
            || any(~isfinite([time; position(:); queryTime; prior(:)])) ...
            || any(diff(time) <= 0) || (~isempty(time) && queryTime < time(end))
        error("fitNrmmTrajectoryWindow:invalidWindow", ...
            "Use finite positions at strictly increasing times before the query.");
    end
    if numel(time) < 3 || time(end)-time(1) < cfg.window.minimumFitSpan
        return
    end
    tau = time-time(1);
    query = queryTime-time(1);
    centredPosition = position-mean(position, 2);
    if norm(centredPosition, "fro") <= cfg.measurement.radar.noiseMaximum
        fit.status = "degeneratePositions";
        return
    end
    % A quadratic seed is used only to initialize the nonlinear fit.
    polynomial = [ones(size(tau)), tau, 0.5*tau.^2]\position.';
    q = polynomial(2, :).';
    s = polynomial(3, :).';
    speed = max(norm(q), domain.speedMinimum);
    acceleration = dot(q, s)/speed;
    curvature = dot([-q(2); q(1)], s)/speed^3;
    scale = [domain.speedMaximum; max(domain.scalarAccelerationMaximum, 1); ...
        max(domain.curvatureMaximum, 1.0e-3)];
    lower = [domain.speedMinimum; -domain.scalarAccelerationMaximum; ...
        -domain.curvatureMaximum]./scale;
    upper = [domain.speedMaximum; domain.scalarAccelerationMaximum; ...
        domain.curvatureMaximum]./scale;
    seed = min(max([speed; acceleration; curvature]./scale, lower), upper);
    seed(2) = min(max(seed(2), (domain.speedMinimum-seed(1)*scale(1)) ...
        /(query*scale(2))), (domain.speedMaximum-seed(1)*scale(1))/(query*scale(2)));
    speedRows = [scale(1), query*scale(2), 0];
    matrix = [speedRows; -speedRows];
    bound = [domain.speedMaximum; -domain.speedMinimum];
    options = optimoptions("fmincon", "Algorithm", "sqp", "Display", "off", ...
        "MaxIterations", cfg.window.maximumIterations, ...
        "OptimalityTolerance", 1.0e-8, "StepTolerance", 1.0e-9, ...
        "ConstraintTolerance", 1.0e-9, "SpecifyObjectiveGradient", true);
    [solution, objective, exitFlag] = fmincon(@(value) ...
        localObjective(value, scale, tau, centredPosition), seed, ...
        matrix, bound, [], [], lower, upper, [], options);
    parameters = solution.*scale;
    [~, ~, theta, origin, residual] = localObjective( ...
        solution, scale, tau, centredPosition);
    origin = origin+mean(position, 2);
    fit.exitFlag = exitFlag;
    fit.objective = objective;
    if exitFlag <= 0 || any(matrix*solution > bound+1.0e-7) ...
            || any(solution < lower-1.0e-7 | solution > upper+1.0e-7)
        fit.status = "optimizerFailed";
        return
    end
    chart = [origin; theta; parameters];
    [~, physical] = nrmmExactFlow(chart, query);
    fit.available = true;
    fit.status = "nominalFit";
    fit.state = physical;
    fit.parameters = chart;
    fit.residual = vecnorm(residual, 2, 1).';
    % A dimensionless local Jacobian diagnostic, not a global uniqueness test.
    jacobian = zeros(2*numel(tau), 6);
    step = 1.0e-5;
    chartScale = [1; 1; 1; scale];
    for index = 1:6
        delta = zeros(6, 1);
        delta(index) = step*chartScale(index);
        plus = localPositions(chart+delta, tau);
        minus = localPositions(chart-delta, tau);
        jacobian(:, index) = (plus(:)-minus(:))/(2*step);
    end
    singular = svd(jacobian, 0);
    fit.conditionNumber = singular(1)/max(singular(end), realmin);
end

function [objective, gradient, theta, origin, residual] = ...
        localObjective(value, scale, time, centredPosition)
    parameters = value.*scale;
    [shape, derivatives] = localShape(parameters, time);
    centredShape = shape-mean(shape, 2);
    sine = sum(centredShape(1, :).*centredPosition(2, :) ...
        -centredShape(2, :).*centredPosition(1, :));
    cosine = sum(centredShape.*centredPosition, "all");
    theta = atan2(sine, cosine);
    rotation = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    residual = rotation*centredShape-centredPosition;
    objective = sum(residual.^2, "all");
    origin = -rotation*mean(shape, 2);
    gradient = zeros(3, 1);
    % Envelope theorem: at the aligned optimum the eliminated translation
    % and rotation contribute no additional first-order objective terms.
    for index = 1:3
        derivative = derivatives(:, :, index);
        derivative = derivative-mean(derivative, 2);
        gradient(index) = 2*scale(index)*sum(residual.*(rotation*derivative), "all");
    end
end

function points = localPositions(chart, time)
    shape = localShape(chart(4:6), time);
    rotation = [cos(chart(3)), -sin(chart(3)); sin(chart(3)), cos(chart(3))];
    points = chart(1:2)+rotation*shape;
end

function [shape, derivatives] = localShape(parameters, time)
    distance = (parameters(1)*time+0.5*parameters(2)*time.^2).';
    angle = parameters(3)*distance;
    half = angle/2;
    sincValue = ones(size(half));
    active = abs(half) > 1.0e-4;
    sincValue(active) = sin(half(active))./half(active);
    sincValue(~active) = 1-half(~active).^2/6+half(~active).^4/120;
    shape = distance.*sincValue.*[cos(half); sin(half)];
    if nargout < 2
        return
    end
    derivatives = zeros(2, numel(time), 3);
    tangent = [cos(angle); sin(angle)];
    derivatives(:, :, 1) = tangent.*time.';
    derivatives(:, :, 2) = tangent.*(0.5*time.'.^2);
    % Integrals of l*J*e(kappa*l); series avoids cancellation at kappa=0.
    small = abs(angle) < 1.0e-3;
    kappa = parameters(3);
    derivative = zeros(size(shape));
    arc = distance(small);
    derivative(1, small) = -kappa*arc.^3/3+kappa^3*arc.^5/30;
    derivative(2, small) = arc.^2/2-kappa^2*arc.^4/8+kappa^4*arc.^6/144;
    turn = angle(~small);
    derivative(1, ~small) = (turn.*cos(turn)-sin(turn))/kappa^2;
    derivative(2, ~small) = (turn.*sin(turn)+cos(turn)-1)/kappa^2;
    derivatives(:, :, 3) = derivative;
end
