function comparison = verifyNrmmGeometricDesign(baselineDesign)
% verifyNrmmGeometricDesign Audit certificates and paired synthetic trajectories.
% An optional previously synthesized design enables a gain-only comparison.
% Both designs run through the current sampled runtime with identical truth
% and random draws. Continuous certificates are those stored in each design.
% No files or figures are generated; callers choose the report destination.

    if nargin < 1
        baselineDesign = struct();
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"config"),fullfile(root,"estimator"));
    cfg = nrmmTrackingConfig();
    design = synthesizeNrmmObserverGains(cfg);
    comparison = struct("configuration",cfg, ...
        "method","paired gains in the same current runtime; deterministic bounded uniform noise", ...
        "geometric",localDesignSummary(design),"baseline",struct(), ...
        "scenarios",struct([]));
    designs = {design};
    labels = "geometric";
    if ~isempty(fieldnames(baselineDesign))
        comparison.baseline = localDesignSummary(baselineDesign);
        designs = {baselineDesign,design};
        labels = ["baseline","geometric"];
    end
    names = ["retained7","retained17","retained23","slower23","dropout23"];
    seeds = [7,17,23,23,23];
    scenarios = cell(numel(names),1);
    for scenario = 1:numel(names)
        current = cfg;
        dropout = zeros(0,2);
        if names(scenario) == "slower23"
            current.runtime.samplePeriod = 0.04;
        elseif names(scenario) == "dropout23"
            dropout = [4,4.4];
        end
        entry = struct("name",names(scenario),"seed",seeds(scenario), ...
            "samplePeriod",current.runtime.samplePeriod,"dropoutIntervals",dropout);
        for index = 1:numel(designs)
            selected = designs{index};
            result = runOnlineNrmmComplexManeuverScenario("Plot",false,"Report",false, ...
                "Config",current,"Seed",seeds(scenario),"NoiseModel","boundedUniform", ...
                "DropoutIntervals",dropout,"DesignFunction",@(~) selected);
            entry.(labels(index)) = result.metrics;
            assert(result.metrics.allSamplesFinite && result.metrics.truthOperatingDomainValid, ...
                "verifyNrmmGeometricDesign:invalidScenario","A comparison trajectory left its declared domain.");
            assert(result.metrics.minimumPositionBoundSlack >= -1e-10, ...
                "verifyNrmmGeometricDesign:containmentFailure","A sampled position enclosure missed truth.");
        end
        scenarios{scenario} = entry;
    end
    comparison.scenarios = vertcat(scenarios{:});
end

function summary = localDesignSummary(design)
    target = design.target;
    gyro = nrmmObserverCertificate("targetGyro",target.lyapunovMatrix, ...
        target.bandwidth,target.domain,target.domain.relativePositionMaximum);
    summary = struct("gains",target.innovationGains,"bandwidth",target.bandwidth, ...
        "decayRate",target.lambda,"metric",target.lyapunovMatrix, ...
        "phiVelocity",target.lipschitzCertificate.phiVelocity, ...
        "phiAcceleration",target.lipschitzCertificate.phiAcceleration, ...
        "ultimateBounds",[design.ultimateBounds.relativePosition; ...
            design.ultimateBounds.targetVelocity;design.ultimateBounds.targetAcceleration], ...
        "residualMargin",target.lipschitzResidualMargin, ...
        "gyroCoefficient",target.disturbanceCoefficients.gyro, ...
        "sameMetricGeometricGyro",gyro.coefficient, ...
        "sameMetricSeparatedGyro",gyro.separatedCoefficient);
end
