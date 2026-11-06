function result = runConservativePredictionWindowStudy(outputDirectory)
%runConservativePredictionWindowStudy Reproduce the conservative tail study.
% No QP is solved and no production configuration is changed. ODE samples
% diagnose the speed envelope; the report gives its conditional analytic proof.
    arguments
        outputDirectory (1,1) string = "report/CONSERVATIVE_HORIZON_BOUND_20261105"
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'config'), fullfile(root, 'controller'));
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    cfg = collisionAvoidanceControllerConfig();
    frictionUpper = 1.005*cfg.tire.frictionCoefficient;
    speeds = [0, .2, 2.5, 8, 15, 25];
    prefixes = [0, 4, 16];
    cases = {};
    for speed = speeds
        caseCfg = cfg;
        caseCfg.model.speedMaximum = max(18, 1.2*speed);
        for prefix = prefixes
            window = researchConservativePredictionWindow(caseCfg, speed, prefix, frictionUpper, 4);
            row = window;
            row.currentSpeedUpperMetersPerSecond = speed;
            row.longitudinalSpeedCapMetersPerSecond = caseCfg.model.speedMaximum;
            cases{end+1} = row; %#ok<AGROW>
        end
    end
    writetable(struct2table([cases{:}]), fullfile(outputDirectory, 'window-bounds.csv'));

    % The naive global Vdot <= mu*g bound fails in the declared low-speed
    % extension. The barrier envelope does not make that assumption there.
    probe = [0; 0; 0; 2.4; .6; 5];
    rate = nonlinearBicycleModel.derivative(probe, [0; 1], cfg);
    naiveRate = probe(4:5)'*rate(4:5)/norm(probe(4:5));
    forceLimit = nonlinearBicycleModel.tireAccelerationGain(cfg);
    assert(naiveRate > forceLimit);

    % Deterministic held-input prefixes, including both blend boundaries.
    % Integration is independent ode45 with dense requested output. A domain
    % exit is reported as such, never treated as evidence for the bound.
    odeCfg = cfg;
    odeCfg.model.speedMaximum = 40;
    odeCfg.tire.frictionCoefficient = frictionUpper;
    initialSpeeds = [.2, 2.5, 3.5, 4, 8, 15, 25];
    inputs = [0, .04, -.04; .95, .2, -.2];
    trajectories = {};
    for speed = initialSpeeds
        for inputIndex = 1:size(inputs, 2)
            x = [0; 0; .2; speed; 0; 0];
            u = inputs(:, inputIndex);
            times = linspace(0, .8, 801);
            [time, states] = ode45(@(~, state)nonlinearBicycleModel.derivative(state, u, odeCfg), ...
                times, x, odeset('RelTol', 1e-9, 'AbsTol', 1e-11));
            window = researchConservativePredictionWindow(odeCfg, speed, 16, frictionUpper, 4);
            actualSpeed = hypot(states(:, 4), states(:, 5));
            upper = min(window.domainSpeedUpperMetersPerSecond, ...
                max(speed, window.blendSpeedUpperMetersPerSecond) + ...
                window.accelerationUpperMetersPerSecondSquared*time);
            domain = states(:, 4) >= 0 & states(:, 4) <= odeCfg.model.speedMaximum & ...
                abs(states(:, 5)) <= odeCfg.model.lateralVelocityMaximum & ...
                abs(states(:, 6)) <= odeCfg.model.yawRateMaximum;
            trajectories{end+1} = struct('initialSpeed', speed, ...
                'steeringRadians', u(1), 'forceRatio', u(2), ...
                'sampledDomainSatisfied', all(domain), ...
                'samples', numel(time), ...
                'maximumSpeedMinusBound', max(actualSpeed-upper), ...
                'terminalSpeed', actualSpeed(end)); %#ok<AGROW>
        end
    end
    trajectoryTable = struct2table([trajectories{:}]);
    writetable(trajectoryTable, fullfile(outputDirectory, 'prefix-ode-checks.csv'));
    assert(all(trajectoryTable.sampledDomainSatisfied));
    assert(all(trajectoryTable.maximumSpeedMinusBound <= 1e-8));

    window = researchConservativePredictionWindow(cfg, 8, 4, frictionUpper, 4);
    context = struct('sampleTime', cfg.controller.sampleTime, 'deceleration', 4);
    entry = [0; 0; 0; 8; .1; .02];
    selector = @()researchConservativePredictionWindow(cfg, 8, 4, frictionUpper, 4);
    continuation = @()terminalSafeSet.continuation(context, entry, window.tailSteps);
    terminalContext = localContext(cfg, entry);
    value = @()terminalSafeSet.nonlinearValue(terminalContext, entry, 4);
    tangent = @()terminalSafeSet.nonlinearTangent(terminalContext, entry, 4);
    selectorSeconds = timeit(selector);
    continuationSeconds = timeit(continuation);
    valueSeconds = timeit(value);
    tangentSeconds = timeit(tangent);
    [terminalValue, terminalActive] = terminalSafeSet.nonlinearValue(terminalContext, entry, 4);
    result = struct('matlabVersion', version, 'computer', computer, ...
        'sampleTimeSeconds', cfg.controller.sampleTime, 'frictionUpper', frictionUpper, ...
        'terminalDecelerationLower', 4, 'odePrefixSeconds', .8, ...
        'odeLongitudinalSpeedCap', odeCfg.model.speedMaximum, ...
        'odeTrajectories', height(trajectoryTable), ...
        'odeSamples', sum(trajectoryTable.samples), ...
        'odeMaximumSpeedMinusBound', max(trajectoryTable.maximumSpeedMinusBound), ...
        'lowSpeedProbe', probe, 'lowSpeedProbeSpeedDerivative', naiveRate, ...
        'nominalTireAccelerationLimit', forceLimit, ...
        'selectorMicroseconds', selectorSeconds*1e6, ...
        'fixedTailPoseMicroseconds', continuationSeconds*1e6, ...
        'fixedTailSteps', window.tailSteps, ...
        'existingTerminalValueMilliseconds', valueSeconds*1e3, ...
        'existingTerminalTangentMilliseconds', tangentSeconds*1e3, ...
        'existingTerminalValue', terminalValue, 'existingTerminalActiveKind', terminalActive.kind, ...
        'existingTerminalActualSteps', ceil(hypot(entry(4), entry(5))/(4*cfg.controller.sampleTime)), ...
        'targetAcceleration', .5, 'targetSideslip', 0, ...
        'productionRuntimeChanged', false, 'closedLoopCampaignRun', false);
    file = fopen(fullfile(outputDirectory, 'measurements.json'), 'w');
    cleanup = onCleanup(@()fclose(file));
    fprintf(file, '%s\n', jsonencode(result, PrettyPrint=true));
    disp(result);
end

function context = localContext(cfg, x)
    road = struct('centerline', [-100, 0; 1000, 0], 'lateralClearance', [20; 20]);
    ego = struct('position', x(1:2), 'yaw', x(3), 'speed', x(4), ...
        'lateralVelocity', x(5), 'yawRate', x(6), 'longitudinalVelocity', x(4), ...
        'stateTime', 0, 'heldActuatorInput', [0; 0]);
    [~, lane, roadOut] = readControllerInputs(ego, [], road, cfg);
    frame = predictiveSafetyGeometry.roadFrame(lane, roadOut);
    reference = nonlinearBicycleModel.cruise(cfg, frame(4));
    target = [20; 2.4; pi; 8; .5; 0; 1.6; 2.4; .95; 0; 0];
    model = struct('cfg', cfg, 'initialState', x, 'target', target, ...
        'targetEpoch', target, 'sampleIndex', 0, 'lane', lane, ...
        'road', roadOut, 'nominalReference', reference);
    context = terminalSafeSet.context(model);
end
