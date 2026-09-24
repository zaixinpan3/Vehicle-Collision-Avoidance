function summary = runTargetMotionCampaign(options)
%runTargetMotionCampaign Declared-plant scenarios with maneuvering targets.
% Each case runs runExactStateRecursiveFeasibilityScenario with the recorded
% NRMM estimator ego bound, nominal target sensing bounds, target jerk J and
% four prediction variants: the target's reachable set with or without a
% declared scalar acceleration maximum, each with the ego-only feedback tube
% or with the target-reactive policy tried first. The truth target's jerk is
% J cos(t), so its acceleration never exceeds sqrt(2) J and the declared
% maximum (2 m/s^2 for J = 1, 3 m/s^2 for J = 2) holds. Measured outcomes only.
    arguments
        options.OutputDirectory (1,1) string
        options.SampleCount (1,1) double {mustBePositive,mustBeInteger} = 240
        options.SearchTimeLimitSeconds (1,1) double {mustBePositive} = 60
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"scripts"),fullfile(root,"controller"),fullfile(root,"config"));
    if ~isfolder(options.OutputDirectory), mkdir(options.OutputDirectory); end
    egoBound = [0.076;0.076;0.048;0.089;0.497;0.0015];
    targetBound = [.1;.1;.05;.05;.01;.01;.01;.01];
    variants = struct("name",["jerkOnly-egoOnly","jerkOnly-reactive","capped-egoOnly","capped-reactive"], ...
        "capped",[false,false,true,true],"order",{{Inf,[30,100,Inf],Inf,[30,100,Inf]}});
    columns = ["scenario","curvature","jerk","accelerationMaximum","variant","completed","passed", ...
        "executedHolds","failureIdentifier","failureMessage","minNodeGap","minSampledGap", ...
        "minTerminalMargin","maxClfResidual","reactiveAdmissions","maxFrameMs"];
    rows = cell(0,numel(columns));
    for scenario = ["stationary","oncoming","crossing"]
        for curvature = [0,0.01]
            for jerk = [1,2]
                maximum = 2+(jerk>1);
                for v = 1:numel(variants.name)
                    name = variants.name(v);
                    cap = Inf;if variants.capped(v),cap = maximum;end
                    directory = fullfile(options.OutputDirectory,sprintf("%s_k%g_J%g_%s",scenario,curvature,jerk,name));
                    identifier = "";message = "";
                    collisionAvoidanceController("resetNominalTrajectory");
                    try
                        report = runExactStateRecursiveFeasibilityScenario(Scenario=scenario,RoadCurvature=curvature, ...
                            SampleCount=options.SampleCount,DeadlineSeconds=Inf, ...
                            SearchTimeLimitSeconds=options.SearchTimeLimitSeconds,OutputDirectory=directory, ...
                            EgoErrorBound=egoBound,TargetErrorBound=targetBound,TargetJerkAmplitude=jerk*[1;1], ...
                            TargetAccelerationMaximum=cap, ...
                            FeedbackPrediction=struct("targetReaction",struct("inputWeightScales",variants.order{v})));
                    catch exception
                        identifier = string(exception.identifier);message = string(exception.message);
                        loaded = load(fullfile(directory,scenario+"-exact-state.mat"),"report");
                        report = loaded.report;
                    end
                    strengths = cellfun(@localStrength,report.admissionSearch);
                    rows(end+1,:) = {scenario,curvature,jerk,cap,name,report.completed,report.passed, ...
                        report.executedHolds,identifier,message,localField(report,"minimumNodeBodyGap"), ...
                        localField(report,"minimumSampledBodyGap"),min([report.terminalMembershipMargin(:);Inf]), ...
                        max([report.clfDissipationResidual(:);-Inf]),nnz(isfinite(strengths)), ...
                        1000*max([report.runtime.frameSeconds(:);NaN])}; %#ok<AGROW>
                    fprintf("%s k=%g J=%g %-18s -> completed=%d passed=%d holds=%d reactive=%d %s\n",scenario,curvature, ...
                        jerk,name,report.completed,report.passed,report.executedHolds,nnz(isfinite(strengths)),identifier);
                end
            end
        end
    end
    results = cell2table(rows,"VariableNames",columns);
    writetable(results,fullfile(options.OutputDirectory,"targetMotionCampaign.csv"));
    summary = struct("egoErrorBound",egoBound,"targetErrorBound",targetBound,"sampleCount",options.SampleCount, ...
        "results",results,"scope","Declared held affine plant; maneuvering targets; deadline disabled");
    save(fullfile(options.OutputDirectory,"targetMotionCampaign.mat"),"summary");
end

function strength = localStrength(search)
% Reaction strength of an accepted fresh admission; Inf for ego-only or inherited frames.
    strength = Inf;
    if isfield(search,"reactionStrength") && isfield(search,"reactionAttempts") && ~isempty(search.reactionAttempts)
        strength = search.reactionStrength;
    end
end

function value = localField(report,name)
    value = NaN;
    if isfield(report,name),value = report.(name);end
end
