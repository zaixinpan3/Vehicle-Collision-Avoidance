function summary = runEstimatorBoundCampaign(options)
%runEstimatorBoundCampaign Declared-plant scenarios with the estimator's ego bound.
% Every case runs runExactStateRecursiveFeasibilityScenario with the recorded
% NRMM estimator bound as the ego measurement box: the ego measurement is the
% declared-plant truth plus independent uniform noise inside that box at every
% frame. The frame deadline is disabled, so a run ends only on a controller
% error or after SampleCount holds. Measured outcomes only.
    arguments
        options.OutputDirectory (1,1) string
        options.SampleCount (1,1) double {mustBePositive,mustBeInteger} = 240
        options.EgoErrorBound (6,1) double {mustBeNonnegative,mustBeFinite} = [0.076;0.076;0.048;0.089;0.497;0.0015]
        options.SearchTimeLimitSeconds (1,1) double {mustBePositive} = 30
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"scripts"),fullfile(root,"controller"),fullfile(root,"config"));
    if ~isfolder(options.OutputDirectory), mkdir(options.OutputDirectory); end
    scenarios = ["stationary","oncoming","crossing","cruise"];
    curvatures = [0,0.01];
    columns = ["scenario","curvature","completed","passed","executedHolds","failureIdentifier", ...
        "failureMessage","minNodeGap","minSampledGap","minTerminalMargin","maxClfResidual","maxFrameMs"];
    rows = cell(numel(scenarios)*numel(curvatures),numel(columns));
    index = 0;
    for curvature = curvatures
        for scenario = scenarios
            index = index+1;
            directory = fullfile(options.OutputDirectory,sprintf("%s_k%g",scenario,curvature));
            identifier = "";message = "";
            try
                report = runExactStateRecursiveFeasibilityScenario(Scenario=scenario,RoadCurvature=curvature, ...
                    SampleCount=options.SampleCount,DeadlineSeconds=Inf, ...
                    SearchTimeLimitSeconds=options.SearchTimeLimitSeconds,OutputDirectory=directory, ...
                    EgoErrorBound=options.EgoErrorBound);
            catch exception
                identifier = string(exception.identifier);message = string(exception.message);
                loaded = load(fullfile(directory,scenario+"-exact-state.mat"),"report");
                report = loaded.report;
            end
            rows(index,:) = {scenario,curvature,report.completed,report.passed,report.executedHolds, ...
                identifier,message,localField(report,"minimumNodeBodyGap"),localField(report,"minimumSampledBodyGap"), ...
                min([report.terminalMembershipMargin(:);Inf]),max([report.clfDissipationResidual(:);-Inf]), ...
                1000*max([report.runtime.frameSeconds(:);NaN])};
            fprintf("[%d/%d] %s k=%g -> completed=%d passed=%d holds=%d %s\n",index,size(rows,1), ...
                scenario,curvature,report.completed,report.passed,report.executedHolds,identifier);
        end
    end
    results = cell2table(rows,"VariableNames",columns);
    writetable(results,fullfile(options.OutputDirectory,"estimatorBoundCampaign.csv"));
    summary = struct("egoErrorBound",options.EgoErrorBound,"sampleCount",options.SampleCount, ...
        "results",results,"scope","Declared held affine plant with estimator-sized ego measurement noise");
    save(fullfile(options.OutputDirectory,"estimatorBoundCampaign.mat"),"summary");
end

function value = localField(report,name)
    value = NaN;
    if isfield(report,name),value = report.(name);end
end
