function benchmark = runNrmmPositionBoundBenchmark(varargin)
% runNrmmPositionBoundBenchmark Evaluate online containment and conservatism.
% Bounds-only variants share bit-identical point estimates and sensor draws.
% The finite ego envelopes here describe this analytic experiment, not an
% automatically calibrated vehicle. Defaults in nrmmTrackingConfig remain Inf.

    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"estimator"),fullfile(root,"config"));
    parser = inputParser;
    addParameter(parser,"Duration",12,@(x) isscalar(x) && isfinite(x) && x > 4);
    addParameter(parser,"Seeds",[71,72,73],@(x) isvector(x) && all(isfinite(x)) ...
        && all(x >= 0) && all(x == fix(x)));
    addParameter(parser,"Report",true,@(x) islogical(x) && isscalar(x));
    parse(parser,varargin{:});
    options = parser.Results;
    cases = ["retained-noise","varying-noise","dropout-noise", ...
        "coarse-integration","25Hz-noise","retained-noise-free"];
    trials = struct([]);
    records = {};
    for caseName = cases
        cfg = nrmmTrackingConfig();
        motion = "retained";
        noise = "boundedUniform";
        dropout = zeros(0,2);
        seeds = options.Seeds;
        if caseName == "varying-noise"
            motion = "varying";
            cfg.target.domain.relativePositionMaximum = 55;
            cfg.target.model.scalarAccelerationRateMaximum = 1.5;
            cfg.target.model.curvatureRateMaximum = 0.0175;
        elseif caseName == "dropout-noise"
            dropout = [options.Duration/3,options.Duration/3+1];
        elseif caseName == "coarse-integration"
            cfg.runtime.integrationStepMaximum = cfg.runtime.samplePeriod;
        elseif caseName == "25Hz-noise"
            cfg.runtime.samplePeriod = 0.04;
        elseif caseName == "retained-noise-free"
            noise = "none";
            seeds = seeds(1);
        end
        design = synthesizeNrmmObserverGains(cfg);
        for seed = seeds(:).'
            previous = struct();
            for envelope = ["domain-only","declared-ego-rates"]
                trialConfig = cfg;
                if envelope == "declared-ego-rates"
                    trialConfig.ego.domain.accelerationNormMaximum = 5;
                    trialConfig.ego.domain.bodyAccelerationRateMaximum = 5;
                    trialConfig.ego.domain.yawAccelerationMaximum = 0.1;
                end
                result = runOnlineNrmmComplexManeuverScenario("Plot",false,"Report",false, ...
                    "Duration",options.Duration,"Seed",seed,"NoiseModel",noise, ...
                    "Config",trialConfig,"TargetMotion",motion,"DropoutIntervals",dropout, ...
                    "DesignFunction",@(~) design);
                if ~isempty(fieldnames(previous))
                    assert(isequaln(previous.truth,result.truth) ...
                        && isequaln(previous.measurements,result.measurements) ...
                        && isequaln(previous.estimate.targetState,result.estimate.targetState) ...
                        && isequaln(previous.estimate.egoState,result.estimate.egoState), ...
                        "Bounds must not modify the estimates or paired inputs.");
                end
                previous = result;
                acceleration = result.truth.egoBodyAcceleration;
                yaw = result.truth.egoYaw;
                jerk = result.truth.egoJerk;
                bodyJerk = [cos(yaw).*jerk(:,1)+sin(yaw).*jerk(:,2), ...
                    -sin(yaw).*jerk(:,1)+cos(yaw).*jerk(:,2)] ...
                    -result.truth.egoYawRate.*[-acceleration(:,2),acceleration(:,1)];
                assert(max(vecnorm(acceleration,2,2)) <= 5 ...
                    && max(vecnorm(bodyJerk,2,2)) <= 5 ...
                    && max(abs(result.truth.egoYawAcceleration)) <= 0.1);
                metrics = result.metrics;
                assert(metrics.truthOperatingDomainValid);
                assert(all(result.estimate.positionErrorBoundAvailable));
                assert(metrics.minimumPositionBoundSlack >= -1e-10, ...
                    "A sampled true position escaped its online enclosure.");
                trial = struct("caseName",caseName,"envelope",envelope,"seed",seed,"result",result);
                trials = [trials;trial]; %#ok<AGROW>
                records(end+1,:) = {caseName,envelope,seed,metrics.relativePositionRmse, ...
                    metrics.meanPositionErrorBound,metrics.maximumPositionErrorBound, ...
                    metrics.minimumPositionBoundSlack,metrics.positionBoundContainmentFraction, ...
                    metrics.positionBoundAvailableFraction,metrics.meanStepMilliseconds}; %#ok<AGROW>
            end
        end
        if options.Report
            fprintf("Position-bound benchmark completed %s (%d paired seeds).\n",caseName,numel(seeds));
        end
    end
    table = cell2table(records,"VariableNames",["Case","Envelope","Seed","PositionRmseM", ...
        "MeanBoundM","MaximumBoundM","MinimumSlackM","ContainmentFraction", ...
        "AvailableFraction","MeanStepMs"]);
    summary = groupsummary(table,["Case","Envelope"],"mean", ...
        ["PositionRmseM","MeanBoundM","MaximumBoundM","MinimumSlackM", ...
        "ContainmentFraction","AvailableFraction","MeanStepMs"]);
    benchmark = struct("options",options,"trials",trials,"table",table,"summary",summary, ...
        "qualification","conditional real-arithmetic enclosure; no interval-arithmetic or closed-loop safety claim");
    if options.Report
        disp(summary);
    end
end
