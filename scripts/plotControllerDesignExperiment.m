function plotControllerDesignExperiment(result, experiment, options)
% plotControllerDesignExperiment Plot one saved nominal/avoidance pair.
% Uses recorded control-step states only. Interrupted traces end at failure.

    arguments
        result (1,1) struct
        experiment (1,1) struct
        options.OutputDirectory (1,1) string = ""
        options.Visible (1,1) logical = false
    end
    figureHandle = figure(Visible=options.Visible, Color="white", ...
        Position=[100, 100, 1400, 900]);
    if ~options.Visible
        cleanup = onCleanup(@() close(figureHandle));
    end
    tiledlayout(figureHandle, 2, 2, TileSpacing="loose", Padding="loose");
    nominal = result.nominal;
    avoidance = result.avoidance;
    assessment = result.assessment;
    nexttile;
    plot(nominal.controlState(:,1), nominal.controlState(:,2), "--", ...
        avoidance.controlState(:,1), avoidance.controlState(:,2), "-", ...
        assessment.nominalTargetPosition(:,1), ...
        assessment.nominalTargetPosition(:,2), ":", LineWidth=1.5);
    axis equal; grid on;
    xlabel("x [m]"); ylabel("y [m]");
    legend("Nominal", "Avoidance", "Target", Location="best", Box="off");
    title(sprintf("%s path; target crossing at t = %.1f s", ...
        result.scene.name, experiment.nominalCollisionTime));
    nexttile;
    plot(nominal.controlTime, assessment.nominalSeparationMargin, "--", ...
        avoidance.controlTime, assessment.avoidanceSeparationMargin, LineWidth=1.5);
    yline(0.0, ":r", "Contact"); grid on;
    xlabel("Time [s]"); ylabel("Rectangle SAT margin [m]");
    legend("Nominal", "Avoidance", Location="best", Box="off");
    nexttile;
    speedLines = plot(nominal.controlTime, nominal.controlState(:,4), "--", ...
        avoidance.controlTime, avoidance.controlState(:,4), LineWidth=1.5);
    speedLines(1).DisplayName = "Nominal";
    speedLines(2).DisplayName = "Avoidance";
    hold on;
    reference = nan(size(avoidance.attempts.time));
    for idx = 1:numel(reference)
        metadata = avoidance.attempts.metadata{idx};
        if isstruct(metadata) && isfield(metadata, "performanceReferenceSpeed")
            reference(idx) = metadata.performanceReferenceSpeed;
        end
    end
    if any(isfinite(reference))
        stairs(avoidance.attempts.time, reference, "-.", ...
            Color=[0.35, 0.35, 0.35], LineWidth=1.0, ...
            DisplayName="Performance reference");
    end
    yline(experiment.referenceSpeed, ":", DisplayName="Cruise request");
    legend(Location="best", Box="off"); grid on;
    xlabel("Time [s]"); ylabel("Longitudinal speed [m/s]");
    nexttile;
    plot(avoidance.controlTime, avoidance.controlTracking.lateralError, LineWidth=1.5);
    grid on; xlabel("Time [s]"); ylabel("Lateral tracking error [m]");
    if avoidance.failure.occurred
        title(sprintf("Controller stopped at %.2f s", avoidance.failure.time));
    else
        title("Avoidance and recovery");
    end
    set(findall(figureHandle, "-property", "Interpreter"), "Interpreter", "none");
    set(findall(figureHandle, "-property", "TickLabelInterpreter"), ...
        "TickLabelInterpreter", "none");
    set(findall(figureHandle, "-property", "FontName"), "FontName", "Arial");
    set(findall(figureHandle, "-property", "FontSize"), "FontSize", 11);
    axesHandles = findall(figureHandle, "Type", "axes");
    for idx = 1:numel(axesHandles)
        axesHandles(idx).YLabel.Units = "normalized";
        axesHandles(idx).YLabel.Position = [-0.12, 0.5, 0.0];
    end
    drawnow;
    if strlength(options.OutputDirectory) > 0
        set(figureHandle, PaperUnits="inches", PaperPosition=[0, 0, 14, 9], ...
            PaperSize=[14, 9]);
        print(figureHandle, fullfile( ...
            options.OutputDirectory, result.scene.name+".png"), "-dpng", "-r150");
    end
end
