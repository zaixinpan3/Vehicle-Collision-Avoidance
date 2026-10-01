function runSharmaScenarioTunedAudit(outputDirectory)
% runSharmaScenarioTunedAudit Sensitivity to scenario-specific baseline tuning.
% Uses the existing training rows only, with the same objective. Each scenario
% selects its own candidate before any supplemental test trial is executed.
% Test seeds and all other settings match the prior nominal campaign. This
% deliberately gives the baseline knowledge of which scenario it will face.
    arguments
        outputDirectory (1, 1) string
    end
    training = readtable(fullfile(outputDirectory, "training.csv"), TextType="string");
    loaded = load(fullfile(outputDirectory, "traces.mat"), "traces");
    protocol = jsondecode(fileread(fullfile(outputDirectory, "protocol.json")));
    scenarios = string(fieldnames(loaded.traces)).';
    selections = struct([]);
    prepared = cell(size(scenarios));
    for index = 1:numel(scenarios)
        name = scenarios(index);
        reference = loaded.traces.(name).runs(2);
        reference = rmfield(reference, ["estimate", "metrics"]);
        reference = orderfields(reference, ["name", "runtime", "design", "description"]);
        candidates = prepareSharmaGainCandidates(reference);
        names = string({candidates.name});
        losses = arrayfun(@(candidate) mean(training.loss( ...
            training.Scenario == name & training.Estimator == candidate)), names);
        [loss, chosen] = min(losses);
        prepared{index} = candidates(chosen);
        selections = [selections; struct("Scenario", name, "Candidate", names(chosen), ...
            "TrainingLoss", loss)]; %#ok<AGROW>
    end
    writetable(struct2table(selections), fullfile(outputDirectory, "scenario-selection.csv"));
    rows = struct([]);
    noiseFreeRows = struct([]);
    for index = 1:numel(scenarios)
        estimator = prepared{index};
        estimator.name = "sharmaScenarioTunedPredictor";
        for seed = protocol.options.TestSeeds(:).'
            result = runObserverComparisonScenario(Scenario=scenarios(index), ...
                Seed=seed, Estimators=estimator, Report=false);
            run = result.runs;
            row = struct("Scenario", scenarios(index), "Variant", "nominal", ...
                "Seed", seed, "Estimator", estimator.name);
            fields = fieldnames(run.metrics);
            for fieldIndex = 1:numel(fields)
                row.(fields{fieldIndex}) = run.metrics.(fields{fieldIndex});
            end
            rows = [rows; row]; %#ok<AGROW>
        end
        writetable(struct2table(rows), fullfile(outputDirectory, "scenario-tuned.csv"));
        oldMatched = loaded.traces.(scenarios(index)).runs(2);
        oldMatched = rmfield(oldMatched, ["estimate", "metrics"]);
        oldMatched = orderfields(oldMatched, ["name", "runtime", "design", "description"]);
        result = runObserverComparisonScenario(Scenario=scenarios(index), ...
            Seed=protocol.options.TestSeeds(1), NoiseModel="none", ...
            Estimators=[oldMatched, estimator], Report=false);
        for run = result.runs
            row = struct("Scenario", scenarios(index), "Variant", "noiseFree", ...
                "Seed", protocol.options.TestSeeds(1), "Estimator", run.name);
            fields = fieldnames(run.metrics);
            for fieldIndex = 1:numel(fields)
                row.(fields{fieldIndex}) = run.metrics.(fields{fieldIndex});
            end
            noiseFreeRows = [noiseFreeRows; row]; %#ok<AGROW>
        end
        writetable(struct2table(noiseFreeRows), fullfile(outputDirectory, "noisefree-supplement.csv"));
        fprintf("Scenario-tuned sensitivity complete: %s\n", scenarios(index));
    end
end
