function result = verifyUnifiedObserverSafety(outputFile)
%verifyUnifiedObserverSafety Execute the physical information-set safety algorithm.
%   Deterministic longitudinal examples use bounded observations, a moving
%   target, held braking, absorbing rest and exact whole-hold gap minima.
%   A finite input alphabet permits exhaustive lexicographic optimization.
%   This is a research specialization, not an NRMM/Fiala runtime certificate.
    if nargin < 1, outputFile = ""; end
    cfg = struct('sampleTimeSeconds', .2, 'horizonSteps', 4, ...
        'slackPrefixSteps', 2, 'brakingLevels', [0, 2, 4], ...
        'brakingMaximum', 4, 'brakingSlewMaximum', 10, ...
        'bodySpanMeters', 4.8, 'clearanceMeters', 0, ...
        'referenceSpeedMetersPerSecond', 8);
    result.configuration = cfg;
    % Rows are ego position/speed and target position/speed. The observer
    % supplies current boxes only; no future observation is used by a plan.
    safe = [-.2, .2; 7.8, 8.2; 9.8, 10.2; 5.8, 6.2];
    recovery = [-.1, .1; 7.9, 8.1; 4.5, 4.7; 11.9, 12.1];
    result.safe = localRun(safe, [0; 8; 10; 6], 0, cfg, 12, [4, 5]);
    result.recovery = localRun(recovery, [0; 8; 4.6; 12], 0, cfg, 6, []);
    short = cfg;
    short.horizonSteps = 1;
    short.slackPrefixSteps = 1;
    result.oneHold = localRun(safe, [0; 8; 10; 6], 0, short, 4, 2);
    parked = [0, .2; 0, 0; 8, 8.2; 0, 0];
    result.parked = localRun(parked, [.1; 0; 8.1; 0], 0, short, 4, [1, 2]);

    % Start, midpoint and endpoint are separated, but a collision occurs at
    % h/4. The minimum is found analytically, without a denser sampling grid.
    between = [0, 0; 1.2, 1.2; 4.804, 4.804; 1, 1];
    times = [0, cfg.sampleTimeSeconds/2, cfg.sampleTimeSeconds];
    sampled = arrayfun(@(t) localGap(between, 4, t, cfg), times);
    result.betweenSamples = struct('sampledMinimumGapMeters', min(sampled), ...
        'wholeHoldMinimumGapMeters', localHoldMargin(between, 4, cfg));

    unsafe = [0, 0; 25, 25; 8, 8; 0, 0];
    result.inputLimitedPlanFound = ~isempty(localPlan(unsafe, 0, cfg));
    overlap = [0, 0; 0, 0; 4, 4; 0, 0];
    result.overlapGapMeters = localHoldMargin(overlap, 0, cfg);
    result.emptyIntersectionRejected = false;
    try
        localCondition(safe, safe + 100);
    catch exception
        if exception.identifier ~= "verifyUnifiedObserverSafety:inconsistentInformation"
            rethrow(exception);
        end
        result.emptyIntersectionRejected = true;
    end

    % A bounded-real LMI supplies a quadratic dissipation inequality, not
    % the unqualified Euclidean ball radius lambda*wbar.
    matrix = [-1, -20; 5, -1];
    metric = diag([1, 4]);
    time = pi/20;
    error = exp(-time)*[-2*sin(10*time); cos(10*time)];
    result.energyVsPointwise = struct( ...
        'dissipationMaximumEigenvalue', max(eig(matrix.'*metric + metric*matrix + eye(2))), ...
        'initialErrorNorm', 1, 'claimedEuclideanRadius', 1, ...
        'laterErrorNorm', norm(error), 'validMetricComparisonRadius', 2);
    result.scope = "Finite-input longitudinal research specialization with analytic hold extrema";
    result.fullVehicleNonlinearCertificate = false;
    if strlength(string(outputFile)) > 0
        file = fopen(outputFile, 'w');
        assert(file >= 0, 'Cannot open result file.');
        cleaner = onCleanup(@() fclose(file));
        fprintf(file, '%s\n', jsonencode(result, PrettyPrint=true));
    end
end

function result = localRun(initial, truth, previousInput, cfg, count, dropouts)
    information = initial;
    previousPlan = [];
    values = zeros(1, count);
    deficits = values;
    applied = values;
    actualMargins = values;
    inherited = nan(1, count);
    posteriorWidths = values;
    predictionsGrow = false;
    rawNonNestedCount = 0;
    dropoutCount = 0;
    shiftIncluded = true;
    for k = 1:count
        assert(all(truth >= information(:, 1) & truth <= information(:, 2)));
        plan = localPlan(information, previousInput, cfg);
        assert(~isempty(plan), 'The research example requires a feasible plan.');
        if ~isempty(previousPlan)
            shiftedInputs = [previousPlan.inputs(2:end), previousPlan.terminalInput];
            shifted = localEvaluate(information, shiftedInputs, previousInput, cfg);
            assert(shifted.feasible);
            inherited(k) = previousPlan.primary - previousPlan.deficits(1);
            assert(shifted.primary <= inherited(k) + 1e-10);
            assert(plan.primary <= shifted.primary + 1e-10);
            for j = 1:cfg.horizonSteps
                shiftIncluded = shiftIncluded && localSubset( ...
                    shifted.sets(:, :, j), previousPlan.sets(:, :, j+1));
            end
            assert(shiftIncluded);
        end
        values(k) = plan.primary;
        deficits(k) = plan.deficits(1);
        applied(k) = plan.inputs(1);
        actualMargins(k) = localHoldMargin([truth, truth], applied(k), cfg);
        assert(actualMargins(k) >= -deficits(k) - 1e-10);
        predictionsGrow = predictionsGrow || ...
            diff(plan.sets(1, :, end)) > diff(information(1, :)) + 1e-10;
        exact = localAdvance([truth, truth], applied(k), cfg.sampleTimeSeconds);
        truth = exact(:, 1);
        prediction = plan.sets(:, :, 2);
        if ismember(k, dropouts)
            observation = repmat([-Inf, Inf], 4, 1);
            dropoutCount = dropoutCount + 1;
        else
            noise = (-1)^(k+1)*[.2; -.15; -.2; .15];
            radius = [.25; .2; .25; .2];
            observation = truth + noise + [-radius, radius];
            rawNonNestedCount = rawNonNestedCount + ~localSubset(observation, prediction);
        end
        information = localCondition(prediction, observation);
        assert(localSubset(information, prediction));
        assert(all(truth >= information(:, 1)-1e-12 & truth <= information(:, 2)+1e-12));
        posteriorWidths(k) = diff(information(1, :));
        previousPlan = plan;
        previousInput = applied(k);
    end
    result = struct('primaryValuesMeters', values, 'firstHoldDeficitsMeters', deficits, ...
        'inheritedBudgetsMeters', inherited, 'appliedBrakingMetersPerSecondSquared', applied, ...
        'minimumTrueGapMeters', min(actualMargins), 'posteriorPositionWidthsMeters', posteriorWidths, ...
        'rawNonNestedObservationCount', rawNonNestedCount, 'dropoutCount', dropoutCount, ...
        'shiftedSetsIncluded', shiftIncluded, 'predictionWidthGrows', predictionsGrow, ...
        'horizonSteps', cfg.horizonSteps, 'sampleTimeSeconds', cfg.sampleTimeSeconds, ...
        'maximumInputChange', max(abs(diff([0, applied]))));
end

function plan = localPlan(information, previousInput, cfg)
    % Exhaust both priorities over the same finite admissible plan class.
    % The observer's unknown future software state is absent from this search.
    levels = cfg.brakingLevels;
    count = numel(levels);
    plan = [];
    for index = 0:count^cfg.horizonSteps-1
        digits = mod(floor(index./count.^(0:cfg.horizonSteps-1)), count) + 1;
        candidate = localEvaluate(information, levels(digits), previousInput, cfg);
        if ~candidate.feasible, continue; end
        if isempty(plan) || candidate.primary < plan.primary-1e-12 ...
                || (abs(candidate.primary-plan.primary) <= 1e-12 && candidate.secondary < plan.secondary)
            plan = candidate;
        end
    end
end

function plan = localEvaluate(information, inputs, previousInput, cfg)
    count = numel(inputs);
    sets = zeros(4, 2, count+1);
    sets(:, :, 1) = information;
    margins = zeros(1, count);
    for i = 1:count
        margins(i) = localHoldMargin(sets(:, :, i), inputs(i), cfg);
        sets(:, :, i+1) = localAdvance(sets(:, :, i), inputs(i), cfg.sampleTimeSeconds);
    end
    deficits = max(0, -margins);
    terminalInput = cfg.brakingMaximum;
    if sets(2, 2, end) == 0, terminalInput = inputs(end); end
    stopTime = sets(2, 2, end)/cfg.brakingMaximum;
    terminalMargin = localMinimumGap(sets(:, :, end), cfg.brakingMaximum, stopTime, cfg);
    slew = cfg.brakingSlewMaximum*cfg.sampleTimeSeconds;
    feasible = all(inputs >= 0 & inputs <= cfg.brakingMaximum) ...
        && all(abs(diff([previousInput, inputs, terminalInput])) <= slew+1e-12) ...
        && all(deficits(cfg.slackPrefixSteps+1:end) <= 1e-12) ...
        && terminalMargin >= -1e-12;
    speeds = reshape(mean(sets(2, :, 2:end), 2), 1, []);
    plan = struct('inputs', inputs, 'sets', sets, 'deficits', deficits, ...
        'primary', sum(deficits(1:cfg.slackPrefixSteps)), ...
        'secondary', sum((speeds-cfg.referenceSpeedMetersPerSecond).^2) + .01*sum(inputs.^2), ...
        'terminalInput', terminalInput, 'terminalMargin', terminalMargin, 'feasible', feasible);
end

function posterior = localCondition(prediction, observation)
    posterior = [max(prediction(:, 1), observation(:, 1)), ...
        min(prediction(:, 2), observation(:, 2))];
    if any(posterior(:, 1) > posterior(:, 2))
        error('verifyUnifiedObserverSafety:inconsistentInformation', ...
            'Prediction and observation are inconsistent; no containment certificate exists.');
    end
end

function next = localAdvance(set, braking, time)
    duration = time + zeros(1, 2);
    if braking > 0, duration = min(duration, set(2, :)/braking); end
    next = set;
    next(1, :) = set(1, :) + set(2, :).*duration - braking*duration.^2/2;
    next(2, :) = max(0, set(2, :) - braking*time);
    next(3, :) = set(3, :) + set(4, :)*time;
end

function margin = localHoldMargin(set, braking, cfg)
    margin = localMinimumGap(set, braking, cfg.sampleTimeSeconds, cfg);
end

function margin = localMinimumGap(set, braking, duration, cfg)
    times = [0, duration];
    if braking > 0
        crossing = (set(2, 2)-set(4, 1))/braking;
        times(end+1) = max(0, min(duration, crossing));
    end
    margin = min(arrayfun(@(time) localGap(set, braking, time, cfg), times));
end

function gap = localGap(set, braking, time, cfg)
    % For the declared nonnegative target speed, the gap increases after
    % ego rest. Before rest its only interior minimum is the speed crossing.
    movingTime = time;
    if braking > 0, movingTime = min(time, set(2, 2)/braking); end
    egoMaximum = set(1, 2) + set(2, 2)*movingTime - braking*movingTime^2/2;
    targetMinimum = set(3, 1) + set(4, 1)*time;
    gap = targetMinimum - egoMaximum - cfg.bodySpanMeters - cfg.clearanceMeters;
end

function value = localSubset(first, second)
    value = all(first(:, 1) >= second(:, 1)-1e-12 & first(:, 2) <= second(:, 2)+1e-12);
end
