function figureHandle = plotObserverComparison(result, options)
% plotObserverComparison Plot truth geometry and per-estimator error histories.
%
% result is the output of runObserverComparisonScenario. The figure shows the
% truth trajectories, the relative-position error norm, the target speed error
% and the ego yaw error for every estimator in the result; the transient window
% excluded from the metrics is shaded. Set Visible=false for batch export.

    arguments
        result (1, 1) struct
        options.Visible (1, 1) logical = true
        options.YLimitPosition (1, 1) double {mustBePositive} = 5.0
        options.YLimitSpeed (1, 1) double {mustBePositive} = 5.0
        options.YLimitYawDeg (1, 1) double {mustBePositive} = 10.0
    end

    figureHandle = figure(Name="Observer comparison: " + result.scenario.name, ...
        Visible=options.Visible, Position=[100, 100, 1200, 800], Theme="light");
    layout = tiledlayout(figureHandle, 2, 2, TileSpacing="compact", Padding="compact");
    title(layout, sprintf("%s: %s", result.scenario.name, result.scenario.description), ...
        Interpreter="none");
    time = result.truth.time;
    truth = result.truth;
    transient = result.options.TransientDuration;

    axesHandle = nexttile(layout);
    plot(axesHandle, truth.egoPosition(:, 1), truth.egoPosition(:, 2), LineWidth=1.5);
    hold(axesHandle, "on");
    plot(axesHandle, truth.targetPosition(:, 1), truth.targetPosition(:, 2), LineWidth=1.5);
    plot(axesHandle, truth.egoPosition(1, 1), truth.egoPosition(1, 2), "ko", MarkerFaceColor="k");
    plot(axesHandle, truth.targetPosition(1, 1), truth.targetPosition(1, 2), "ks", MarkerFaceColor="k");
    axis(axesHandle, "equal");
    grid(axesHandle, "on");
    legend(axesHandle, "ego", "target", "ego start", "target start", Location="best");
    title(axesHandle, "Truth trajectories (m)");

    positionAxes = nexttile(layout);
    speedAxes = nexttile(layout);
    yawAxes = nexttile(layout);
    hold(positionAxes, "on");
    hold(speedAxes, "on");
    hold(yawAxes, "on");
    for run = result.runs
        positionError = vecnorm(run.estimate.targetState(:, 1:2) ...
            - truth.targetTransformedState(:, 1:2), 2, 2);
        speedError = run.estimate.targetSpeed - truth.targetSpeed;
        yawError = rad2deg(mod(run.estimate.egoYaw - truth.egoYaw + pi, 2.0*pi) - pi);
        plot(positionAxes, time, min(positionError, options.YLimitPosition), ...
            DisplayName=run.name, LineWidth=1.0);
        plot(speedAxes, time, max(min(speedError, options.YLimitSpeed), -options.YLimitSpeed), ...
            DisplayName=run.name, LineWidth=1.0);
        plot(yawAxes, time, max(min(yawError, options.YLimitYawDeg), -options.YLimitYawDeg), ...
            DisplayName=run.name, LineWidth=1.0);
    end
    for axesHandle = [positionAxes, speedAxes, yawAxes]
        limits = ylim(axesHandle);
        patch(axesHandle, [0, transient, transient, 0], [limits(1), limits(1), limits(2), limits(2)], ...
            [0.85, 0.85, 0.85], FaceAlpha=0.4, EdgeColor="none", HandleVisibility="off");
        set(axesHandle, "Children", flipud(get(axesHandle, "Children")));
        grid(axesHandle, "on");
        xlabel(axesHandle, "time (s)");
    end
    title(positionAxes, sprintf("Relative-position error norm (m, clipped at %.3g)", ...
        options.YLimitPosition));
    title(speedAxes, sprintf("Target speed error (m/s, clipped at %.3g)", options.YLimitSpeed));
    title(yawAxes, sprintf("Ego yaw error (deg, clipped at %.3g)", options.YLimitYawDeg));
    legend(positionAxes, Location="northeast", Interpreter="none");
end
