function audit = runSharmaFairnessAudit(options)
% runSharmaFairnessAudit Training-separated comparison and target-stage ablation.
% Fixed protocol: select ONE Sharma gain candidate on seeds 101:102 across
% all six scenarios, then evaluate seeds 201:220. No tuning uses test errors.
% Both normalized LMI and repeated-pole gain shapes are considered. Empirical
% tuning keeps the paper's measurements/model and does not assert Theorem 1.
% The objective averages rhoRMSE/0.1 + speedRMSE/0.5 + courseRMSE/5, equally
% across training seeds and scenarios. Any failed trial gets infinite loss.
% The target oracle removes all upstream sensor differences on four cases.

    arguments
        options.OutputDirectory (1, 1) string
        options.TrainingSeeds (1, :) double = 101:102
        options.TestSeeds (1, :) double = 201:220
        options.StressSeeds (1, :) double = 201:205
    end
    assert(isempty(intersect(options.TrainingSeeds, ...
        [options.TestSeeds, options.StressSeeds])), "Training and test seeds must be disjoint.");
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root, "config"), fullfile(root, "estimator"));
    destination = options.OutputDirectory;
    if ~isfolder(destination)
        mkdir(destination);
    end
    scenarios = ["straightOncoming", "circularCrossing", "circularFollowing", ...
        "avoidanceSwerve", "lowRelativeVelocity", "aggressiveEgo"];
    protocol = struct("options", options, "scenarios", scenarios, ...
        "selection", "One global minimum training mean of position/0.1 + speed/0.5 + course/5", ...
        "gainShapes", ["lmi", "repeated"], "egoRates", [1.5, 3, 6], ...
        "targetRates", [1.5, 3, 6], "yawGains", [0.1, 1, 3], ...
        "primaryWindow", "t >= 2 seconds; also record final-half and full-run errors", ...
        "noise", "bounded uniform, common draws; cfg sensor bounds", ...
        "createdAt", string(datetime("now", TimeZone="America/Chicago")), ...
        "matlabVersion", string(version));
    localJson(fullfile(destination, "protocol.json"), protocol);
    start = tic;
    templates = cell(1, numel(scenarios));
    candidates = cell(size(templates));
    trainingRows = struct([]);
    for scenarioIndex = 1:numel(scenarios)
        name = scenarios(scenarioIndex);
        preview = runObserverComparisonScenario(Scenario=name, ...
            Estimators=["structured", "sharmaMatchedPredictor", ...
                "sharmaCertifiedHold", "sharmaCertifiedPredictor"], ...
            Duration=0.1, TransientDuration=0, Report=false);
        templates{scenarioIndex} = localPrepared(preview.runs);
        candidates{scenarioIndex} = prepareSharmaGainCandidates(templates{scenarioIndex}(2));
        for seed = options.TrainingSeeds
            result = runObserverComparisonScenario(Scenario=name, Seed=seed, ...
                Estimators=candidates{scenarioIndex}, Report=false);
            assert(result.truthDomainValid);
            for run = result.runs
                row = localRow(name, "training", seed, run);
                row.loss = row.relativePositionRmse/0.1 + row.targetSpeedRmse/0.5 ...
                    + row.courseRmseDeg/5.0;
                if row.diverged
                    row.loss = Inf;
                end
                trainingRows = [trainingRows; row]; %#ok<AGROW>
            end
        end
        writetable(struct2table(trainingRows), fullfile(destination, "training.csv"));
        fprintf("[%6.1f s] training %s complete\n", toc(start), name);
    end
    candidateNames = string({candidates{1}.name});
    training = struct2table(trainingRows);
    losses = arrayfun(@(name) mean(training.loss(training.Estimator == name)), candidateNames);
    [~, winner] = min(losses);
    selection = table(candidateNames.', losses.', VariableNames=["Candidate", "TrainingLoss"]);
    writetable(selection, fullfile(destination, "selection.csv"));
    fprintf("Selected before test evaluation: %s, loss %.6g\n", candidateNames(winner), losses(winner));

    rows = struct([]);
    oracleRows = struct([]);
    numericalRows = struct([]);
    traces = struct();
    variants = ["nominal", "noiseFree", "sample20Hz", "sample100Hz", ...
        "noise2x", "largeOffset", "radarDropout", "kinematicMismatch"];
    for scenarioIndex = 1:numel(scenarios)
        name = scenarios(scenarioIndex);
        base = templates{scenarioIndex};
        tuned = candidates{scenarioIndex}(winner);
        tuned.name = "sharmaTunedPredictor";
        corrected = tuned;
        corrected.name = "sharmaTunedCorrectedPredictor";
        corrected.design.variant = "corrected";
        full = [base, tuned, corrected];
        for variant = variants
            seeds = options.StressSeeds;
            if variant == "nominal"
                seeds = options.TestSeeds;
                estimators = full;
            else
                estimators = [base(1), tuned, corrected];
            end
            if variant == "noiseFree"
                seeds = options.TestSeeds(1);
            end
            args = localVariant(variant, name);
            for seed = seeds
                result = runObserverComparisonScenario("Scenario", name, "Seed", seed, ...
                    "Estimators", estimators, "Report", false, args{:});
                assert(result.truthDomainValid);
                for run = result.runs
                    rows = [rows; localRow(name, variant, seed, run)]; %#ok<AGROW>
                end
                if variant == "nominal" && ismember(scenarioIndex, [1, 2, 3, 5])
                    sameGain = corrected;
                    sameGain.name = "sharmaSameInjection";
                    sameGain.design.target.gain = zeros(6, 2);
                    sameGain.design.target.gain(1:3, 1) = base(1).design.target.innovationGains;
                    sameGain.design.target.gain(4:6, 2) = base(1).design.target.innovationGains;
                    oracle = runNrmmOracleTargetComparison(result, [base(1:2), corrected, sameGain]);
                    for run = oracle
                        row = struct("Scenario", name, "Seed", seed, "Estimator", run.name);
                        fields = fieldnames(run.metrics);
                        for fieldIndex = 1:numel(fields)
                            field = fields{fieldIndex};
                            row.(field) = run.metrics.(field);
                        end
                        oracleRows = [oracleRows; row]; %#ok<AGROW>
                    end
                end
                if variant == "nominal" && seed == options.TestSeeds(1)
                    traces.(name) = result;
                    refined = runObserverComparisonScenario(Scenario=name, Seed=seed, ...
                        Estimators=full, Report=false, IntegrationStepMaximum=0.0025);
                    for run = refined.runs
                        numericalRows = [numericalRows; localRow(name, "halfRk4Step", seed, run)]; %#ok<AGROW>
                    end
                end
            end
            writetable(struct2table(rows), fullfile(destination, "trials.csv"));
            fprintf("[%6.1f s] test %s / %s complete\n", toc(start), name, variant);
        end
        writetable(struct2table(oracleRows), fullfile(destination, "oracle.csv"));
        writetable(struct2table(numericalRows), fullfile(destination, "numerical.csv"));
        save(fullfile(destination, "traces.mat"), "traces", "-v7.3");
    end
    audit = struct("protocol", protocol, "selectedCandidate", candidateNames(winner), ...
        "selection", selection, "elapsedSeconds", toc(start), ...
        "trainingTrialCount", height(training), "testTrialCount", numel(rows), ...
        "oracleTrialCount", numel(oracleRows), "numericalTrialCount", numel(numericalRows));
    localJson(fullfile(destination, "completion.json"), audit);
    fprintf("Audit complete: %d training, %d test, %d oracle trials in %.1f s\n", ...
        height(training), numel(rows), numel(oracleRows), toc(start));
end

function prepared = localPrepared(runs)
    prepared = rmfield(runs, ["estimate", "metrics"]);
    prepared = orderfields(prepared, ["name", "runtime", "design", "description"]);
end

function row = localRow(scenario, variant, seed, run)
    row = struct("Scenario", scenario, "Variant", variant, "Seed", seed, "Estimator", run.name);
    fields = fieldnames(run.metrics);
    for index = 1:numel(fields)
        field = fields{index};
        row.(field) = run.metrics.(field);
    end
end

function args = localVariant(variant, name)
    args = {};
    switch variant
        case "noiseFree"
            args = {"NoiseModel", "none"};
        case "sample20Hz"
            args = {"SamplePeriod", 0.05};
        case "sample100Hz"
            args = {"SamplePeriod", 0.01};
        case "noise2x"
            args = {"NoiseScale", 2.0};
        case "largeOffset"
            args = {"InitialOffsetScale", 3.0};
        case "radarDropout"
            duration = struct("straightOncoming", 6, "circularCrossing", 6, ...
                "circularFollowing", 12, "avoidanceSwerve", 6.5, ...
                "lowRelativeVelocity", 12, "aggressiveEgo", 12);
            start = duration.(name)/3;
            args = {"DropoutIntervals", [start, start+0.5]};
        case "kinematicMismatch"
            args = {"SingleTrackMismatch", 0.02};
    end
end

function localJson(path, value)
    file = fopen(path, "w");
    assert(file ~= -1, "Cannot write audit output.");
    closeFile = onCleanup(@() fclose(file));
    fprintf(file, "%s\n", jsonencode(value, PrettyPrint=true));
end
