function summary = runDeclaredPlantFailureSweep(options)
% runDeclaredPlantFailureSweep Sweep declared-plant scenarios for non-timeout failures.
%
% Every case runs runExactStateRecursiveFeasibilityScenario with the frame
% deadline disabled, so a run ends only on a controller error or completes.
% The sweep records the controller failure identifier, the executed holds
% and the node and sampled body gaps of every case. A negative sampled gap
% with a positive node gap is an inter-node overlap that the node
% certificate does not cover; it is recorded, not repaired.

    arguments
        options.OutputDirectory (1,1) string
        options.SampleCount (1,1) double {mustBePositive, mustBeInteger} = 240
        options.SearchTimeLimitSeconds (1,1) double {mustBePositive} = 30
        options.FeedbackPrediction (1,1) struct = struct()
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root, "scripts"), fullfile(root, "controller"), fullfile(root, "config"));
    if ~isfolder(options.OutputDirectory), mkdir(options.OutputDirectory); end
    cases = localCases();
    columns = ["group", "scenario", "curvature", "uncertaintyScale", "targetJerk", "targetYawAcc", ...
        "initialLateral", "initialHeading", "initialSpeed", "roadBoundaries", "horizonSeconds", ...
        "confirmationRange", "completed", "passed", "executedHolds", "failureIdentifier", ...
        "failureMessage", "minNodeGap", "minSampledGap", "minDomainMargin", "minTerminalMargin", ...
        "maxClfResidual", "recursiveFeasibilityGuaranteed", "maxFrameMs"];
    rows = cell(numel(cases), numel(columns));
    for index = 1:numel(cases)
        c = cases{index};
        caseDirectory = fullfile(options.OutputDirectory, sprintf("case%03d", index));
        arguments_ = [{"Scenario", c.Scenario, "RoadCurvature", c.RoadCurvature, ...
            "SampleCount", options.SampleCount, "DeadlineSeconds", Inf, ...
            "SearchTimeLimitSeconds", options.SearchTimeLimitSeconds, ...
            "OutputDirectory", caseDirectory, "FeedbackPrediction", options.FeedbackPrediction}, c.extra];
        failureIdentifier = ""; failureMessage = "";
        try
            report = runExactStateRecursiveFeasibilityScenario(arguments_{:});
        catch exception
            % The scenario saves its report before rethrowing a controller error.
            failureIdentifier = string(exception.identifier);
            failureMessage = string(exception.message);
            report = localLoadReport(caseDirectory, c.Scenario);
        end
        rows(index, :) = {c.group, c.Scenario, c.RoadCurvature, c.uncertaintyScale, c.targetJerk, ...
            c.targetYawAcc, c.initialLateral, c.initialHeading, c.initialSpeed, c.roadBoundaries, ...
            c.horizonSeconds, c.confirmationRange, localField(report, "completed", false), ...
            localField(report, "passed", false), localField(report, "executedHolds", 0), ...
            failureIdentifier, failureMessage, localField(report, "minimumNodeBodyGap", NaN), ...
            localField(report, "minimumSampledBodyGap", NaN), ...
            localField(report, "minimumSampledModelDomainMargin", NaN), ...
            localMin(report, "terminalMembershipMargin"), localMax(report, "clfDissipationResidual"), ...
            localField(report, "recursiveFeasibilityGuaranteed", false), localMaxFrame(report)};
        fprintf("[%3d/%3d] %s %s k=%g -> completed=%d passed=%d holds=%d %s\n", index, numel(cases), ...
            c.group, c.Scenario, c.RoadCurvature, rows{index, 13}, rows{index, 14}, rows{index, 15}, ...
            failureIdentifier);
    end
    results = cell2table(rows, "VariableNames", columns);
    writetable(results, fullfile(options.OutputDirectory, "declaredPlantFailureSweep.csv"));
    summary = struct("matlabVersion", string(version), "sampleCount", options.SampleCount, ...
        "caseCount", numel(cases), "passedCount", nnz(results.passed), ...
        "controllerErrorCount", nnz(strlength(results.failureIdentifier) > 0), ...
        "silentOverlapCount", nnz(results.completed & ~results.passed), "results", results, ...
        "scope", "Declared held affine plant; deadlines disabled; numerical observation, not a proof");
    save(fullfile(options.OutputDirectory, "declaredPlantFailureSweep.mat"), "summary");
end

function cases = localCases()
    egoBase = [.01; .01; .001; .01; .01; .001];
    targetBase = [.1; .1; .05; .05; .01; .01; .01; .01];
    cases = {};
    for scenario = ["stationary", "oncoming", "crossing", "cruise"]
        for curvature = [0, .005, -.005, .01, -.01, .015, -.015, .02, -.02]
            cases{end+1} = localCase("A-geometry", scenario, curvature, {}); %#ok<AGROW>
        end
    end
    for scenario = ["stationary", "oncoming", "crossing"]
        for curvature = [0, .01]
            for scale = [1, 3, 10]
                item = localCase("B-uncertainty", scenario, curvature, {"EgoErrorBound", scale*egoBase, ...
                    "TargetErrorBound", scale*targetBase, "TargetJerkAmplitude", scale*[.02; .02]});
                item.uncertaintyScale = scale; item.targetJerk = scale*.02;
                cases{end+1} = item; %#ok<AGROW>
            end
        end
    end
    for scenario = ["oncoming", "crossing"]
        for jerk = [.1, .5, 1.0]
            item = localCase("C-targetJerk", scenario, 0, {"TargetErrorBound", targetBase, ...
                "TargetJerkAmplitude", jerk*[1; 1]});
            item.targetJerk = jerk; cases{end+1} = item; %#ok<AGROW>
        end
        for yawAcc = [.1, .5]
            item = localCase("C-targetYawAcc", scenario, 0, {"TargetErrorBound", targetBase, ...
                "TargetYawAccelerationAmplitude", yawAcc});
            item.targetYawAcc = yawAcc; cases{end+1} = item; %#ok<AGROW>
        end
    end
    for scenario = ["cruise", "stationary"]
        for curvature = [0, .01]
            for lateral = [.5, 1, 2, 3]
                item = localCase("D-initLateral", scenario, curvature, {"InitialTrackingError", [lateral; 0; 0; 0; 0]});
                item.initialLateral = lateral; cases{end+1} = item; %#ok<AGROW>
            end
            for heading = [.1, .2, .3]
                item = localCase("D-initHeading", scenario, curvature, {"InitialTrackingError", [0; heading; 0; 0; 0]});
                item.initialHeading = heading; cases{end+1} = item; %#ok<AGROW>
            end
            for speed = [-2, 2]
                item = localCase("D-initSpeed", scenario, curvature, {"InitialTrackingError", [0; 0; speed; 0; 0]});
                item.initialSpeed = speed; cases{end+1} = item; %#ok<AGROW>
            end
        end
    end
    for scenario = ["stationary", "oncoming", "crossing", "cruise"]
        item = localCase("E-road", scenario, 0, {"UseRoadBoundaries", true});
        item.roadBoundaries = true; cases{end+1} = item; %#ok<AGROW>
    end
    for scenario = ["oncoming", "crossing"]
        for horizon = [.8, 1.2, 2.4, 3.2]
            item = localCase("F-horizon", scenario, 0, {"HorizonSeconds", horizon});
            item.horizonSeconds = horizon; cases{end+1} = item; %#ok<AGROW>
        end
    end
    for scenario = ["oncoming", "crossing"]
        for range = [10, 24, 32]
            item = localCase("G-confirm", scenario, 0, {"ConfirmationRange", range});
            item.confirmationRange = range; cases{end+1} = item; %#ok<AGROW>
        end
    end
end

function item = localCase(group, scenario, curvature, extra)
    item = struct("group", group, "Scenario", scenario, "RoadCurvature", curvature, "extra", {extra}, ...
        "uncertaintyScale", 0, "targetJerk", 0, "targetYawAcc", 0, "initialLateral", 0, ...
        "initialHeading", 0, "initialSpeed", 0, "roadBoundaries", false, "horizonSeconds", 1.6, ...
        "confirmationRange", 16);
end

function report = localLoadReport(directory, scenario)
    file = fullfile(directory, scenario + "-exact-state.mat");
    report = struct();
    if isfile(file), loaded = load(file, "report"); report = loaded.report; end
end

function value = localField(report, name, default)
    value = default;
    if isfield(report, name), value = report.(name); end
end

function value = localMin(report, name)
    value = NaN;
    if isfield(report, name) && ~isempty(report.(name)), value = min(report.(name)); end
end

function value = localMax(report, name)
    value = NaN;
    if isfield(report, name) && ~isempty(report.(name)), value = max(report.(name)); end
end

function value = localMaxFrame(report)
    value = NaN;
    if isfield(report, "runtime") && isfield(report.runtime, "frameSeconds") ...
            && ~isempty(report.runtime.frameSeconds)
        value = 1000*max(report.runtime.frameSeconds, [], "omitnan");
    end
end
