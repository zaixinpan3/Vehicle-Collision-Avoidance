function study = runControllerDesignExperiments(options)
% runControllerDesignExperiments Test cruise, late detection and avoidance.
% Runs straight and circular paths with the actual PassVeh14DOF template.
% Each path has a nominal counterfactual and a 50 m range-gated trial.
% Results include stopped/failed trials; an incomplete run cannot pass.
% Avoidance acceptance and final-window tracking observations are separate.
% These finite trials do not establish eventual convergence to nominal cruise.
%
%   study = runControllerDesignExperiments(OutputDirectory="/tmp/designStudy");
%
% Configuration contains physical experiment parameters, not control modes.
% OutputDirectory is optional. MAT checkpoints are written after each trial
% so evidence survives a later infrastructure error. A summary CSV and a
% figure per scene are written when both trials have returned.

    arguments
        options.Configuration (1,1) struct = struct()
        options.OutputDirectory (1,1) string = ""
        options.Plot (1,1) logical = false
        options.Progress (1,1) logical = true
    end
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(repositoryRoot, "config"));
    experiment = options.Configuration;
    if isempty(fieldnames(experiment))
        experiment = controllerDesignExperimentConfig();
    end
    if strlength(options.OutputDirectory) > 0 && ~isfolder(options.OutputDirectory)
        mkdir(options.OutputDirectory);
    end
    study = struct("configuration", experiment, "results", ...
        repmat(struct("scene", struct(), "nominal", struct(), ...
        "avoidance", struct(), "assessment", struct()), 1, numel(experiment.scenes)));
    for sceneIdx = 1:numel(experiment.scenes)
        scene = experiment.scenes(sceneIdx);
        station = (-200.0:experiment.centerlineSpacing:400.0).';
        if scene.curvature == 0.0
            centerline = [station, zeros(size(station))];
        else
            angle = scene.curvature*station;
            centerline = [sin(angle), 1.0-cos(angle)]/scene.curvature;
        end
        common = {"Centerline", centerline, ...
            "InitialYawRate", scene.initialYawRate, ...
            "Duration", experiment.duration, ...
            "ReferenceSpeed", experiment.referenceSpeed, ...
            "ControllerConfiguration", experiment.controllerConfiguration, ...
            "PerceptionRange", experiment.perceptionRange, ...
            "RoadBoundaryOffsets", experiment.roadBoundaryOffsets, ...
            "RoadShoulderWidth", experiment.shoulderWidth, ...
            "EgoCollisionSize", experiment.egoSize, ...
            "TargetCollisionSize", experiment.targetSize, ...
            "UseStateEstimator", false, "Plot", false, "Report", false, ...
            "Progress", options.Progress};
        fprintf("\n%s: nominal counterfactual\n", scene.name);
        nominal = runCenterlineCruiseScenario(common{:}, ...
            Description=scene.name+" nominal counterfactual");
        localCheckpoint(options.OutputDirectory, scene.name+"_nominal", nominal);
        fprintf("\n%s: target observations within %.1f m\n", ...
            scene.name, experiment.perceptionRange);
        avoidance = runCenterlineCruiseScenario(common{:}, ...
            Description=scene.name+" crossing-target avoidance", ...
            TargetStateFunction=@(time, ego) localTarget(time, scene, experiment));
        localCheckpoint(options.OutputDirectory, scene.name+"_avoidance", avoidance);
        assessment = evaluateControllerDesignExperiment( ...
            nominal, avoidance, scene, experiment);
        study.results(sceneIdx) = struct( ...
            "scene", scene, "nominal", nominal, "avoidance", avoidance, ...
            "assessment", assessment);
        disp(assessment.summary);
        if options.Plot || strlength(options.OutputDirectory) > 0
            plotControllerDesignExperiment(study.results(sceneIdx), experiment, ...
                OutputDirectory=options.OutputDirectory, Visible=options.Plot);
        end
    end
    study.summary = vertcat(study.results.assessment);
    study.summary = vertcat(study.summary.summary);
    study.avoidanceRequirementsMet = all(study.summary.avoidancePass);
    study.convergenceStatus = "notEstablishedByFiniteRun";
    study.realTimeRequirementMet = all(study.summary.deadlinePass);
    if strlength(options.OutputDirectory) > 0
        writetable(study.summary, fullfile(options.OutputDirectory, "summary.csv"));
        save(fullfile(options.OutputDirectory, "study.mat"), "study", "-v7.3");
    end
end

function target = localTarget(time, scene, experiment)
    target = struct( ...
        "targetPositionInertial", scene.targetInitialPosition+time*scene.targetVelocity, ...
        "targetVelocityInertial", scene.targetVelocity, ...
        "targetAccelerationInertial", [0.0; 0.0], ...
        "targetYawInertial", scene.targetYaw, "targetYawRate", 0.0, ...
        "targetLength", experiment.targetSize(1), ...
        "targetWidth", experiment.targetSize(2));
end

function localCheckpoint(directory, name, result)
    if strlength(directory) > 0
        save(fullfile(directory, name+".mat"), "result", "-v7.3");
    end
end
