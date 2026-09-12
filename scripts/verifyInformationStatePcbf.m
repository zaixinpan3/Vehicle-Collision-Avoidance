function summary = verifyInformationStatePcbf(outputDirectory,options)
%verifyInformationStatePcbf Run the information-state PCBF validation campaign.
% Exact and noisy-estimate straight-road trials for the three scenes, plus a
% forced fresh-solve failure that consumes the carried witness down to the
% terminal law. Each trial executes the accepted plan's first-stage affine
% generator from the true state; measurements carry bounded noise. Results,
% per-frame value functions and timings are saved for the results record.
    arguments
        outputDirectory (1,1) string
        options.SampleCount (1,1) double {mustBeInteger,mustBePositive} = 300
        options.EgoErrorBound (6,1) double {mustBeNonnegative,mustBeFinite} = [0.05;0.05;0.005;0.05;0.02;0.005]
        options.TargetErrorBound (8,1) double {mustBeNonnegative,mustBeFinite} = [0.1;0.1;0;0;0;0;0.01;0]
        options.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 20260912
        options.Scenes (1,:) string {mustBeMember(options.Scenes,["oncoming","stationary","crossing"])} = ["oncoming","stationary","crossing"]
        options.Estimations (1,:) string {mustBeMember(options.Estimations,["exact","noisy"])} = ["exact","noisy"]
        options.ForcedFailure (1,1) logical = true
    end
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"scripts"),fullfile(root,"controller"),fullfile(root,"config"));
    scenes = options.Scenes;
    summary = struct("scene",{},"estimation",{},"passed",{},"completed",{},"executedHolds",{}, ...
        "carriedWitnessCommands",{},"allCandidatesVerified",{},"maximumValue",{},"maximumDescentResidual",{}, ...
        "minimumSeparationMargin",{},"minimumRoadMargin",{},"finalSpeed",{},"finalLateralError",{}, ...
        "medianFrameSeconds",{},"maximumFrameSeconds",{},"medianWitnessSeconds",{},"failureIdentifier",{}, ...
        "terminalLawFrames",{});
    for scene = scenes
        for estimation = options.Estimations
            ego = zeros(6,1); target = zeros(8,1);
            if estimation=="noisy", ego = options.EgoErrorBound; target = options.TargetErrorBound; end
            report = runExactStateRecursiveFeasibilityScenario(Scenario=scene,SampleCount=options.SampleCount, ...
                EgoErrorBound=ego,TargetErrorBound=target,Seed=options.Seed, ...
                OutputDirectory=fullfile(outputDirectory,scene+"-"+estimation));
            summary(end+1) = localRow(scene,estimation,report); %#ok<AGROW>
        end
    end
    if options.ForcedFailure
        report = runExactStateRecursiveFeasibilityScenario(Scenario="crossing",SampleCount=40, ...
            FailAfterAdmission=true,EgoErrorBound=options.EgoErrorBound,TargetErrorBound=options.TargetErrorBound, ...
            Seed=options.Seed,OutputDirectory=fullfile(outputDirectory,"crossing-forced-failure"));
        summary(end+1) = localRow("crossing","noisyForcedFreshFailure",report);
        summary(end).terminalLawFrames = nnz(report.terminalActive);
    end
    % A partial rerun replaces the matching rows of an earlier summary and
    % keeps the others, so the saved summary always covers every trial run.
    previous = fullfile(outputDirectory,"summary.mat");
    if isfile(previous)
        earlier = load(previous).summary;
        for index = 1:numel(summary)
            earlier([earlier.scene]==summary(index).scene & [earlier.estimation]==summary(index).estimation) = [];
        end
        summary = [earlier,summary];
        [~,order] = sortrows([arrayfun(@(row) find(["oncoming","stationary","crossing"]==row.scene),summary).', ...
            arrayfun(@(row) find(["exact","noisy","noisyForcedFreshFailure"]==row.estimation),summary).']);
        summary = summary(order);
    end
    save(fullfile(outputDirectory,"summary.mat"),"summary");
    file = fopen(fullfile(outputDirectory,"summary.json"),'w');
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
    disp(struct2table(summary));
end

function row = localRow(scene,estimation,report)
    row = struct("scene",scene,"estimation",estimation,"passed",report.passed,"completed",report.completed, ...
        "executedHolds",report.executedHolds,"carriedWitnessCommands",NaN,"allCandidatesVerified",NaN, ...
        "maximumValue",NaN,"maximumDescentResidual",NaN,"minimumSeparationMargin",NaN,"minimumRoadMargin",NaN, ...
        "finalSpeed",NaN,"finalLateralError",NaN,"medianFrameSeconds",NaN,"maximumFrameSeconds",NaN, ...
        "medianWitnessSeconds",NaN,"failureIdentifier",string(report.failureIdentifier));
    if isfield(report,"candidateExecuted")
        row.carriedWitnessCommands = nnz(report.candidateExecuted);
        row.allCandidatesVerified = report.allCandidatesVerified;
        row.maximumValue = max(report.pcbfValue);
        row.maximumDescentResidual = report.maximumDescentResidual;
        row.minimumSeparationMargin = report.minimumSampledSeparationMargin;
        row.minimumRoadMargin = report.minimumSampledRoadMargin;
        row.finalSpeed = report.state(4,end);
        row.finalLateralError = report.state(2,end);
        row.medianFrameSeconds = median(report.runtime.frameSeconds(2:end));
        row.maximumFrameSeconds = max(report.runtime.frameSeconds);
        row.medianWitnessSeconds = median(report.witnessSeconds(2:end));
    end
    row.terminalLawFrames = 0;
end
