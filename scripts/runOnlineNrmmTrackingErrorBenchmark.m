function benchmark = runOnlineNrmmTrackingErrorBenchmark(varargin)
% runOnlineNrmmTrackingErrorBenchmark Paired accuracy, lag, dropout, and radius study.
% All estimators receive identical truth, samples, initialization, and seeds.
% WindowDurations exposes the information/response tradeoff. BaselineRuntime
% and BaselineDesign optionally accept an independently versioned comparator;
% no historical implementation is stored in this repository. Reported timing
% includes each step's fit, enclosures, and output reconstruction on this host.
% Tight ego increment bounds here are NEW, EXPLICIT scenario assumptions:
% |a_E| <= 3 m/s^2 and |yawAcceleration| <= 0.05 rad/s^2 throughout intervals.
% The analytic ego profile has |yawAcceleration| <= 0.042 and |a_E| < 1.8.

    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"config"),fullfile(root,"estimator"));
    parser = inputParser;
    addParameter(parser,"Report",true,@(x) islogical(x) && isscalar(x));
    addParameter(parser,"Seed",7,@(x) isscalar(x) && isfinite(x) && x == fix(x) && x >= 0);
    addParameter(parser,"Duration",12,@(x) isscalar(x) && isfinite(x) && x > 4);
    addParameter(parser,"MonteCarloRuns",5,@(x) isscalar(x) && x >= 1 && x == fix(x));
    addParameter(parser,"WindowDurations",[0.4,0.8,1.2], ...
        @(x) isvector(x) && all(isfinite(x) & x >= 0.12));
    addParameter(parser,"BaselineRuntime",[],@(x) isempty(x) || isa(x,"function_handle"));
    addParameter(parser,"BaselineDesign",[],@(x) isempty(x) || isa(x,"function_handle"));
    parse(parser,varargin{:});
    options = parser.Results;
    if xor(isempty(options.BaselineRuntime),isempty(options.BaselineDesign))
        error("runOnlineNrmmTrackingErrorBenchmark:incompleteBaseline", ...
            "Supply both baseline runtime and design functions.");
    end
    definitions = ["retained-noise-free","retained-noise","varying-noise", ...
        "dropout-noise","retained-noise-25Hz"];
    trials = struct([]);
    records = {};
    for definition = definitions
        cfg = nrmmTrackingConfig();
        cfg.ego.intersample.accelerationMaximum = 3;
        cfg.ego.intersample.yawAccelerationMaximum = 0.05;
        noise = "boundedUniform";
        count = options.MonteCarloRuns;
        motion = "retained";
        dropouts = zeros(0,2);
        if definition == "retained-noise-free"
            noise = "none";
            count = 1;
        elseif definition == "varying-noise"
            motion = "varying";
            cfg.target.domain.relativePositionMaximum = 55;
            cfg.target.model.scalarAccelerationRateMaximum = 1.5;
            cfg.target.model.curvatureRateMaximum = 0.0175;
        elseif definition == "dropout-noise"
            dropouts = [options.Duration/3,options.Duration/3+1];
        elseif definition == "retained-noise-25Hz"
            cfg.runtime.samplePeriod = 0.04;
        end
        windows = options.WindowDurations;
        labels = "window-"+string(windows);
        if ~isempty(options.BaselineRuntime)
            windows = [NaN,windows]; %#ok<AGROW>
            labels = ["continuous-baseline",labels]; %#ok<AGROW>
        end
        for variant = 1:numel(windows)
            for run = 1:count
                runtime = @onlineNrmmTrackingRuntime;
                design = @nrmmWindowEstimatorDesign;
                if isnan(windows(variant))
                    runtime = options.BaselineRuntime;
                    design = options.BaselineDesign;
                else
                    cfg.window.duration = windows(variant);
                end
                result = runOnlineNrmmComplexManeuverScenario("Plot",false,"Report",false, ...
                    "Duration",options.Duration,"Seed",options.Seed+run-1,"NoiseModel",noise, ...
                    "Config",cfg,"TargetMotion",motion,"DropoutIntervals",dropouts, ...
                    "RuntimeFunction",runtime,"DesignFunction",design);
                metrics = result.metrics;
                assert(metrics.truthOperatingDomainValid, ...
                    "Benchmark truth violates its declared target domain or model-rate bounds.");
                trial = struct("caseName",definition,"estimator",labels(variant), ...
                    "seed",options.Seed+run-1,"windowDuration",windows(variant), ...
                    "samplePeriod",cfg.runtime.samplePeriod,"metrics",metrics,"result",result);
                if isempty(trials)
                    trials = trial;
                else
                    trials(end+1) = trial; %#ok<AGROW>
                end
                records(end+1,:) = {definition,labels(variant),options.Seed+run-1, ...
                    metrics.relativePositionRmse,metrics.targetVelocityRmse, ...
                    metrics.targetAccelerationRmse,metrics.curvatureLagSeconds, ...
                    metrics.meanErrorRadius(1),metrics.meanErrorRadius(2),metrics.meanErrorRadius(3), ...
                    metrics.radiusCoverageFraction,metrics.outerNonemptyFraction, ...
                    metrics.operatingDomainValidFinalInterval,metrics.estimatedDomainValidFraction, ...
                    metrics.truthOperatingDomainValid,metrics.meanStepMilliseconds, ...
                    metrics.maximumStepMilliseconds}; %#ok<AGROW>
            end
        end
        if options.Report
            fprintf("Completed %s (%d seed(s), %d estimators).\n",definition,count,numel(windows));
        end
    end
    table = cell2table(records,"VariableNames",["Case","Estimator","Seed", ...
        "PositionRmseM","VelocityRmseMps","AccelerationRmseMps2","CurvatureLagS", ...
        "PositionRadiusM","VelocityRadiusMps","AccelerationRadiusMps2", ...
        "RadiusCoverage","OuterNonempty","FinalDomainValid","DomainValidFraction","TruthDomainValid","MeanStepMs","MaxStepMs"]);
    summary = groupsummary(table,["Case","Estimator"],"mean", ...
        ["PositionRmseM","VelocityRmseMps","AccelerationRmseMps2","CurvatureLagS", ...
        "PositionRadiusM","RadiusCoverage","DomainValidFraction","MeanStepMs"]);
    benchmark = struct("options",options,"trials",trials,"table",table,"summary",summary, ...
        "burnInSeconds",2,"timingContract","assimilate-at-t-predict-to-t-plus-sample", ...
        "certificateQualification","analytic-conditional-enclosures-not-machine-verified; no MHE convergence theorem claimed");
    if options.Report
        disp(summary);
    end
end
