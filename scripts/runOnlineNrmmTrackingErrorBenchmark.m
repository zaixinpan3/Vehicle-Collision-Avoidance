function benchmark = runOnlineNrmmTrackingErrorBenchmark(varargin)
% runOnlineNrmmTrackingErrorBenchmark Compare multistage high-gain designs.
% Paired trials use identical truth, sensor draws, and initial conditions.
% Cases include bounded noise, 25 Hz sensing, model variation, and dropout,
% plus campaign cases: aggressive ego weaving, a straight-road oncoming target
% (8 s, 100 m domain), a target lane change, noise at twice the declared
% bounds, three-times initial offsets, and intermittent 0.2 s radar gaps.
% Cases may select a subset; continuous-observation studies can omit dropout.
% BaselineRuntime and BaselineDesign can point to versioned source exports;
% no historical implementation is kept in the active repository. Continuous
% Lyapunov bounds and empirical sampled errors are reported separately.

    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"config"),fullfile(root,"estimator"));
    parser = inputParser;
    availableCases = ["retained-noise-free","retained-noise","varying-noise", ...
        "dropout-noise","retained-noise-25Hz","aggressive-ego-noise","oncoming-noise", ...
        "lane-change-noise","noise-2x","large-offset-noise","intermittent-dropout-noise"];
    addParameter(parser,"Cases",availableCases,@(x) isstring(x) && isvector(x) ...
        && ~isempty(x) && all(ismember(x,availableCases)) && numel(unique(x)) == numel(x));
    addParameter(parser,"Report",true,@(x) islogical(x) && isscalar(x));
    addParameter(parser,"Seed",7,@(x) isscalar(x) && isfinite(x) && x == fix(x) && x >= 0);
    addParameter(parser,"Duration",12,@(x) isscalar(x) && isfinite(x) && x > 4);
    addParameter(parser,"MonteCarloRuns",5,@(x) isscalar(x) && x >= 1 && x == fix(x));
    addParameter(parser,"BaselineRuntime",[],@(x) isempty(x) || isa(x,"function_handle"));
    addParameter(parser,"BaselineDesign",[],@(x) isempty(x) || isa(x,"function_handle"));
    parse(parser,varargin{:});
    options = parser.Results;
    if xor(isempty(options.BaselineRuntime),isempty(options.BaselineDesign))
        error("runOnlineNrmmTrackingErrorBenchmark:incompleteBaseline", ...
            "Supply both baseline runtime and design functions.");
    end
    cases = options.Cases(:).';
    labels = "structured-high-gain";
    if ~isempty(options.BaselineRuntime)
        labels = ["baseline-high-gain",labels];
    end
    trials = struct([]);
    records = {};
    for caseName = cases
        cfg = nrmmTrackingConfig();
        noise = "boundedUniform";
        count = options.MonteCarloRuns;
        motion = "retained";
        dropouts = zeros(0,2);
        duration = options.Duration;
        scenario = struct("EgoManeuver","retained","TargetInitialPosition",[25,4], ...
            "TargetInitialHeading",0.10,"TargetInitialSpeed",12.5, ...
            "NoiseScale",1,"InitialOffsetScale",1);
        if caseName == "retained-noise-free"
            noise = "none";
            count = 1;
        elseif caseName == "varying-noise"
            motion = "varying";
            cfg.target.domain.relativePositionMaximum = 55;
            cfg.target.model.scalarAccelerationRateMaximum = 1.5;
            cfg.target.model.curvatureRateMaximum = 0.0175;
        elseif caseName == "dropout-noise"
            dropouts = [options.Duration/3,options.Duration/3+1];
        elseif caseName == "retained-noise-25Hz"
            cfg.runtime.samplePeriod = 0.04;
        elseif caseName == "aggressive-ego-noise"
            scenario.EgoManeuver = "aggressive";
            cfg.target.domain.relativePositionMaximum = 80;
        elseif caseName == "oncoming-noise"
            % Head-on target on a straight road; the pass happens near 4 s.
            scenario.EgoManeuver = "straight";
            scenario.TargetInitialPosition = [95,3.5];
            scenario.TargetInitialHeading = pi;
            scenario.TargetInitialSpeed = 11;
            cfg.target.domain.relativePositionMaximum = 100;
            duration = min(options.Duration,8);
        elseif caseName == "lane-change-noise"
            motion = "laneChange";
            cfg.target.domain.relativePositionMaximum = 55;
            cfg.target.model.curvatureRateMaximum = 0.008;
        elseif caseName == "noise-2x"
            scenario.NoiseScale = 2;
        elseif caseName == "large-offset-noise"
            scenario.InitialOffsetScale = 3;
        elseif caseName == "intermittent-dropout-noise"
            starts = (2.5:2:duration-1.5).';
            dropouts = [starts,starts+0.2];
        end
        for label = labels
            runtimeFunction = @onlineNrmmTrackingRuntime;
            designFunction = @synthesizeNrmmObserverGains;
            if label == "baseline-high-gain"
                runtimeFunction = options.BaselineRuntime;
                designFunction = options.BaselineDesign;
            end
            design = designFunction(cfg);
            for run = 1:count
                result = runOnlineNrmmComplexManeuverScenario("Plot",false,"Report",false, ...
                    "Duration",duration,"Seed",options.Seed+run-1,"NoiseModel",noise, ...
                    "Config",cfg,"TargetMotion",motion,"DropoutIntervals",dropouts, ...
                    "EgoManeuver",scenario.EgoManeuver, ...
                    "TargetInitialPosition",scenario.TargetInitialPosition, ...
                    "TargetInitialHeading",scenario.TargetInitialHeading, ...
                    "TargetInitialSpeed",scenario.TargetInitialSpeed, ...
                    "NoiseScale",scenario.NoiseScale, ...
                    "InitialOffsetScale",scenario.InitialOffsetScale, ...
                    "RuntimeFunction",runtimeFunction,"DesignFunction",@(~) design);
                metrics = result.metrics;
                assert(metrics.truthOperatingDomainValid, ...
                    "Benchmark truth violates its physical domain or model-rate bounds.");
                trial = struct("caseName",caseName,"estimator",label, ...
                    "seed",options.Seed+run-1,"samplePeriod",cfg.runtime.samplePeriod, ...
                    "metrics",metrics,"result",result);
                if isempty(trials)
                    trials = trial;
                else
                    trials(end+1) = trial; %#ok<AGROW>
                end
                records(end+1,:) = {caseName,label,options.Seed+run-1, ...
                    metrics.relativePositionRmse,metrics.targetVelocityRmse, ...
                    metrics.targetAccelerationRmse,metrics.curvatureLagSeconds, ...
                    metrics.continuousUltimateBounds(1),metrics.continuousUltimateBounds(2), ...
                    metrics.continuousUltimateBounds(3),metrics.continuousTargetDecayRate, ...
                    metrics.targetBandwidth,metrics.peakInitialAccelerationError, ...
                    metrics.maximumPositionError,metrics.maximumAccelerationError, ...
                    metrics.estimatedDomainValidFraction,metrics.truthOperatingDomainValid, ...
                    metrics.meanStepMilliseconds,metrics.maximumStepMilliseconds, ...
                    metrics.declaredModelJerkCovered,metrics.hasRadarDropout, ...
                    metrics.digitalErrorBoundCertified}; %#ok<AGROW>
            end
        end
        if options.Report
            fprintf("Completed %s (%d seeds, %d high-gain designs).\n",caseName,count,numel(labels));
        end
    end
    table = cell2table(records,"VariableNames",["Case","Estimator","Seed", ...
        "PositionRmseM","VelocityRmseMps","AccelerationRmseMps2","CurvatureLagS", ...
        "ContinuousPositionBoundM","ContinuousVelocityBoundMps","ContinuousAccelerationBoundMps2", ...
        "ContinuousDecayRatePerS","BandwidthPerS","InitialPeakAccelerationErrorMps2", ...
        "MaximumPositionErrorM","MaximumAccelerationErrorMps2", ...
        "DomainValidFraction","TruthDomainValid","MeanStepMs","MaxStepMs", ...
        "DeclaredModelJerkCovered","HasRadarDropout","DigitalCertified"]);
    summary = groupsummary(table,["Case","Estimator"],"mean", ...
        ["PositionRmseM","VelocityRmseMps","AccelerationRmseMps2","CurvatureLagS", ...
        "ContinuousPositionBoundM","ContinuousDecayRatePerS","BandwidthPerS", ...
        "InitialPeakAccelerationErrorMps2","MaximumPositionErrorM","DomainValidFraction","MeanStepMs"]);
    benchmark = struct("options",options,"trials",trials,"table",table,"summary",summary, ...
        "burnInSeconds",2,"timingContract","assimilate-at-t-predict-to-t-plus-sample", ...
        "certificateQualification","continuous Lyapunov/Lipschitz ISS; sampled implementation and dropouts are not certified");
    if options.Report
        disp(summary);
    end
end
