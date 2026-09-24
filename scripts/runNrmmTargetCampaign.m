function summary = runNrmmTargetCampaign(options)
%runNrmmTargetCampaign Declared-plant scenarios with NRMM (constant speed-rate
% and curvature) targets under four target predictions.
% The truth target keeps its speed-rate and path curvature; its measurement is
% the truth plus independent noise at every frame: uniform in the scaled
% nominal box for the position, yaw and yaw rate, uniform in discs of the
% scaled nominal radii for the velocity and acceleration (the NRMM estimator
% certifies component norms). The ego measurement box is the recorded NRMM
% estimator bound. Variants:
%   cartesian     - finite-sensing contract whose jerk bound covers the
%                   turning of an NRMM target of this speed and the declared
%                   curvature maximum, hypot(kappa^2 V^3, 3 A kappa V) (the form
%                   the estimator publishes when it declares a model error),
%                   ego-only feedback tube;
%   nrmm-boxOnly  - nrmm-motion-v1 contract from the measurement box alone,
%                   ego-only feedback tube;
%   nrmm-egoOnly  - nrmm-motion-v1 contract with the published NRMM parameter
%                   error bounds (nrmmTargetParameterErrorBounds), ego-only
%                   feedback tube;
%   nrmm-reactive - as nrmm-egoOnly with the target-reactive policy tried first.
% Measured outcomes only; deadline disabled.
    arguments
        options.OutputDirectory (1,1) string
        options.SampleCount (1,1) double {mustBePositive,mustBeInteger} = 240
        options.SearchTimeLimitSeconds (1,1) double {mustBePositive} = 60
        options.Scenarios (1,:) string = ["crossing","oncoming","stationary"]
        options.RoadCurvatures (1,:) double = [0,0.01]
        options.TargetCurvatures (1,:) double = [0,0.02]
        options.UncertaintyScales (1,:) double = [1,3,10]
        options.Variants (1,:) string = ["cartesian","nrmm-boxOnly","nrmm-egoOnly","nrmm-reactive"]
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"scripts"),fullfile(root,"controller"),fullfile(root,"config"));
    if ~isfolder(options.OutputDirectory), mkdir(options.OutputDirectory); end
    egoBound = [0.076;0.076;0.048;0.089;0.497;0.0015];
    targetBase = [.1;.1;.05;.05;.01;.01;.01;.01];
    curvatureMaximum = 0.03;
    columns = ["scenario","roadCurvature","targetCurvature","uncertaintyScale","variant","completed","passed", ...
        "executedHolds","failureIdentifier","failureMessage","minNodeGap","minSampledGap", ...
        "minTerminalMargin","maxClfResidual","reactiveAdmissions","cartesianJerk","maxFrameMs"];
    rows = cell(0,numel(columns));
    for scenario = options.Scenarios
        targetCurvatures = options.TargetCurvatures;
        if scenario=="stationary",targetCurvatures = 0;end
        for roadCurvature = options.RoadCurvatures
            for targetCurvature = targetCurvatures
                for scale = options.UncertaintyScales
                    for variant = options.Variants
                        arguments_ = {"Scenario",scenario,"RoadCurvature",roadCurvature, ...
                            "SampleCount",options.SampleCount,"DeadlineSeconds",Inf, ...
                            "SearchTimeLimitSeconds",options.SearchTimeLimitSeconds, ...
                            "EgoErrorBound",egoBound,"TargetErrorBound",scale*targetBase, ...
                            "TargetMotionModel","nrmm","TargetCurvature",targetCurvature, ...
                            "TargetCurvatureMaximum",curvatureMaximum};
                        order = Inf;
                        if variant=="cartesian"
                            % Same NRMM truth, Cartesian contract covering its turning.
                            arguments_ = [arguments_,{"NrmmContract","cartesian"}]; %#ok<AGROW>
                        elseif variant=="nrmm-boxOnly"
                            arguments_ = [arguments_,{"NrmmParameterBounds",false}]; %#ok<AGROW>
                        elseif variant=="nrmm-reactive"
                            order = [30,100,Inf];
                        end
                        arguments_ = [arguments_,{"FeedbackPrediction", ...
                            struct("targetReaction",struct("inputWeightScales",order))}]; %#ok<AGROW>
                        directory = fullfile(options.OutputDirectory,sprintf("%s_k%g_c%g_x%g_%s", ...
                            scenario,roadCurvature,targetCurvature,scale,variant));
                        arguments_ = [arguments_,{"OutputDirectory",directory}]; %#ok<AGROW>
                        identifier = "";message = "";
                        collisionAvoidanceController("resetNominalTrajectory");
                        try
                            report = runExactStateRecursiveFeasibilityScenario(arguments_{:});
                        catch exception
                            identifier = string(exception.identifier);message = string(exception.message);
                            loaded = load(fullfile(directory,scenario+"-exact-state.mat"),"report");
                            report = loaded.report;
                        end
                        strengths = cellfun(@localStrength,report.admissionSearch);
                        rows(end+1,:) = {scenario,roadCurvature,targetCurvature,scale,variant, ...
                            report.completed,report.passed,report.executedHolds,identifier,message, ...
                            localField(report,"minimumNodeBodyGap"),localField(report,"minimumSampledBodyGap"), ...
                            min([report.terminalMembershipMargin(:);Inf]), ...
                            max([report.clfDissipationResidual(:);-Inf]), ...
                            nnz(isfinite(strengths)),report.targetMotion.cartesianJerkBound, ...
                            1000*max([report.runtime.frameSeconds(:);NaN])}; %#ok<AGROW>
                        fprintf("%s k=%g kT=%g x%g %-14s -> completed=%d passed=%d holds=%d reactive=%d %s\n", ...
                            scenario,roadCurvature,targetCurvature,scale,variant,report.completed,report.passed, ...
                            report.executedHolds,nnz(isfinite(strengths)),identifier);
                    end
                end
            end
        end
    end
    results = cell2table(rows,"VariableNames",columns);
    writetable(results,fullfile(options.OutputDirectory,"nrmmTargetCampaign.csv"));
    summary = struct("egoErrorBound",egoBound,"targetErrorBase",targetBase,"curvatureMaximum",curvatureMaximum, ...
        "sampleCount",options.SampleCount,"results",results, ...
        "scope","Declared held affine plant; NRMM truth targets; deadline disabled");
    save(fullfile(options.OutputDirectory,"nrmmTargetCampaign.mat"),"summary");
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
