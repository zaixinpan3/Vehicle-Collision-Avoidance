function result = generateCurbRingFeatureFigure(options)
%GENERATECURBRINGFEATUREFIGURE Plot the four curb features along one LiDAR ring.
%   RESULT = GENERATECURBRINGFEATUREFIGURE() builds a straight two-lane road
%   bounded by raised curbs in a drivingScenario, places the ego vehicle on the
%   road centerline with a 64-ring LiDAR, and scans one sweep with ray tracing.
%   The sweep is organized onto the shared 64-by-1000 grid by
%   organizeLidarSweepGrid, the ring features are computed by
%   computeLidarRingCurbFeatures, and the four features reported in the
%   manuscript are plotted along the forward half of one representative ring.
%
%   The plotted features are the adjacent-ring compression
%   (hataCompressionNormalized), the vertical step (heightStep), the radial
%   jump (radialSecondDifference), and the local shape (huangTangentialStrength).
%   Dashed markers give the azimuth bins at which the ring crosses each curb on
%   flat ground, computed from the ring elevation and the road half width alone.
%
%   Name-value options:
%       OutputDirectory     folder receiving the figure, default paper/figures
%       FigureBaseName      file base name, default "curb_ring_features_v01"
%       TargetRingRadiusM   flat-ground ring radius used to pick the ring, 8 m
%       SensorHeightM       LiDAR mounting height, default 2.35 m
%       RoadWidthM          road width, default 7 m
%       CurbHeightM         curb face height, default 0.15 m
%       SaveFigure          write the figure files, default true
%
%   RESULT reports the selected ring, its geometry, the predicted crossing
%   bins, the plotted feature traces, and the written file paths.

    arguments
        options.OutputDirectory (1,1) string = localDefaultOutputDirectory()
        options.FigureBaseName (1,1) string = "curb_ring_features_v01"
        options.TargetRingRadiusM (1,1) double {mustBePositive} = 8.0
        options.SensorHeightM (1,1) double {mustBePositive} = 2.35
        options.RoadWidthM (1,1) double {mustBePositive} = 7.0
        options.CurbHeightM (1,1) double {mustBePositive} = 0.15
        options.SaveFigure (1,1) logical = true
    end

    numberOfRings = 64;
    numberOfAzimuthBins = 1000;
    upperElevationDegrees = 10.0;
    lowerElevationDegrees = -30.0;
    ringElevationDegrees = linspace( ...
        upperElevationDegrees, lowerElevationDegrees, numberOfRings);
    azimuthStepDegrees = 360.0 / numberOfAzimuthBins;

    scenario = localBuildRoadScenario(options);
    frame = localScanOrganizedFrame(scenario, options, azimuthStepDegrees);
    [features, featureValidity] = computeLidarRingCurbFeatures(frame, ...
        struct("ringFeatureSensorHeightM", options.SensorHeightM));

    [ringIndex, ringRadiusM] = localSelectRing( ...
        ringElevationDegrees, options.SensorHeightM, ...
        options.TargetRingRadiusM);
    crossingAzimuthDegrees = asind( ...
        min(1.0, 0.5 * options.RoadWidthM / ringRadiusM));
    crossingBins = crossingAzimuthDegrees / azimuthStepDegrees * [-1, 1];

    forwardBins = [(numberOfAzimuthBins * 3 / 4 + 1):numberOfAzimuthBins, ...
        1:(numberOfAzimuthBins / 4 + 1)];
    signedBinIndex = [ ...
        (numberOfAzimuthBins * 3 / 4 + 1):numberOfAzimuthBins, ...
        1:(numberOfAzimuthBins / 4 + 1)] - 1;
    signedBinIndex(signedBinIndex > numberOfAzimuthBins / 2) = ...
        signedBinIndex(signedBinIndex > numberOfAzimuthBins / 2) ...
        - numberOfAzimuthBins;

    traces = localCollectTraces(features, featureValidity, ringIndex, ...
        forwardBins);
    figureHandle = localPlotTraces(traces, signedBinIndex, crossingBins);

    writtenFiles = strings(0, 1);
    if options.SaveFigure
        writtenFiles = localWriteFigure(figureHandle, ...
            options.OutputDirectory, options.FigureBaseName);
    end

    result = struct( ...
        "ringIndex", ringIndex, ...
        "ringElevationDegrees", ringElevationDegrees(ringIndex), ...
        "ringRadiusM", ringRadiusM, ...
        "crossingAzimuthDegrees", crossingAzimuthDegrees, ...
        "crossingBins", crossingBins, ...
        "signedBinIndex", signedBinIndex, ...
        "traces", traces, ...
        "writtenFiles", writtenFiles, ...
        "figureHandle", figureHandle);
end

function outputDirectory = localDefaultOutputDirectory()
    experimentDirectory = fileparts(mfilename("fullpath"));
    repositoryRoot = fileparts(fileparts(experimentDirectory));
    outputDirectory = string(fullfile(repositoryRoot, "paper", "figures"));
end

function scenario = localBuildRoadScenario(options)
%LOCALBUILDROADSCENARIO Straight road bounded by two raised curbs.
%   Each curb is a long cuboid actor whose top face is the raised walkway and
%   whose inner face is the curb face at the road edge.
    roadLengthM = 120.0;
    walkwayWidthM = 3.0;

    scenario = drivingScenario("SampleTime", 0.1, "StopTime", 0.1);
    road(scenario, ...
        [-roadLengthM / 2, 0, 0; roadLengthM / 2, 0, 0], ...
        options.RoadWidthM);
    vehicle(scenario, "ClassID", 1, "Position", [0, 0, 0]);
    for side = [-1, 1]
        actor(scenario, ...
            "ClassID", 5, ...
            "Length", roadLengthM, ...
            "Width", walkwayWidthM, ...
            "Height", options.CurbHeightM, ...
            "Position", [0, ...
                side * (0.5 * options.RoadWidthM + 0.5 * walkwayWidthM), 0]);
    end
end

function frame = localScanOrganizedFrame( ...
        scenario, options, azimuthStepDegrees)
%LOCALSCANORGANIZEDFRAME Ray trace one sweep and organize it onto the grid.
%   The generator places floor(span/resolution) samples inclusively across each
%   angular span, so the requested resolutions are the ones whose sample counts
%   reproduce the grid elevations exactly and the grid azimuths on both ends.
    numberOfRings = 64;
    elevationResolutionDegrees = 40.0 / numberOfRings;
    azimuthResolutionDegrees = 0.999 * azimuthStepDegrees;
    egoActor = scenario.Actors(1);
    lidar = lidarPointCloudGenerator( ...
        "SensorLocation", [0, 0], ...
        "Height", options.SensorHeightM, ...
        "MaxRange", 60.0, ...
        "AzimuthResolution", azimuthResolutionDegrees, ...
        "ElevationResolution", elevationResolutionDegrees, ...
        "AzimuthLimits", [-180.0, 180.0], ...
        "ElevationLimits", [-30.0, 10.0], ...
        "HasNoise", true, ...
        "HasEgoVehicle", false, ...
        "HasRoadsInputPort", true, ...
        "DetectionCoordinates", "Sensor Cartesian", ...
        "ActorProfiles", actorProfiles(scenario));

    rng(0, "twister");
    advance(scenario);
    pointCloud = lidar(targetPoses(egoActor), roadMesh(egoActor), ...
        scenario.SimulationTime);
    rawPoints = reshape(pointCloud.Location, [], 3);
    rawPoints = rawPoints(all(isfinite(rawPoints), 2), :);
    frame = organizeLidarSweepGrid(double(rawPoints));
end

function [ringIndex, ringRadiusM] = localSelectRing( ...
        ringElevationDegrees, sensorHeightM, targetRingRadiusM)
%LOCALSELECTRING Ring whose flat-ground radius is closest to the target.
    downwardRings = find(ringElevationDegrees < -eps);
    radii = sensorHeightM ./ tand(-ringElevationDegrees(downwardRings));
    [~, closest] = min(abs(radii - targetRingRadiusM));
    ringIndex = downwardRings(closest);
    ringRadiusM = radii(closest);
end

function traces = localCollectTraces( ...
        features, featureValidity, ringIndex, forwardBins)
%LOCALCOLLECTTRACES Gather the four manuscript features along one ring.
    traceNames = [ ...
        "hataCompressionNormalized", ...
        "heightStep", ...
        "radialSecondDifference", ...
        "huangTangentialStrength"];
    traces = struct();
    for traceName = traceNames
        values = features.(traceName)(ringIndex, forwardBins);
        values(~featureValidity.(traceName)(ringIndex, forwardBins)) = NaN;
        traces.(traceName) = values;
    end
end

function figureHandle = localPlotTraces(traces, signedBinIndex, crossingBins)
%LOCALPLOTTRACES Stacked feature traces sharing the azimuth-bin axis.
    panels = { ...
        "hataCompressionNormalized", "$\gamma_{i,j}$", "(a) compression"; ...
        "heightStep", "$\sigma_{i,j}$ (m)", "(b) vertical step"; ...
        "radialSecondDifference", "$\zeta_{i,j}$ (m)", "(c) radial jump"; ...
        "huangTangentialStrength", "$\tau_{i,j}$", "(d) local shape"};

    observed = false(size(signedBinIndex));
    for panelIndex = 1:size(panels, 1)
        observed = observed | isfinite(traces.(panels{panelIndex, 1}));
    end
    observedBins = signedBinIndex(observed);
    binLimits = [min(observedBins) - 5, max(observedBins) + 5];

    figureHandle = figure("Units", "inches", ...
        "Position", [1, 1, 3.5, 5.0], "Color", "w", "Theme", "light");
    tiles = tiledlayout(figureHandle, size(panels, 1), 1, ...
        "TileSpacing", "compact", "Padding", "compact");
    for panelIndex = 1:size(panels, 1)
        axesHandle = nexttile(tiles);
        hold(axesHandle, "on");
        for crossingBin = crossingBins
            xline(axesHandle, crossingBin, "--", ...
                "Color", [0.65, 0.65, 0.65], "LineWidth", 0.8, ...
                "HandleVisibility", "off");
        end
        plot(axesHandle, signedBinIndex, traces.(panels{panelIndex, 1}), ...
            "-", "Color", [0.00, 0.30, 0.65], "LineWidth", 0.9);
        hold(axesHandle, "off");
        grid(axesHandle, "on");
        box(axesHandle, "on");
        xlim(axesHandle, binLimits);
        ylabel(axesHandle, panels{panelIndex, 2}, "Interpreter", "latex");
        title(axesHandle, panels{panelIndex, 3}, ...
            "FontWeight", "normal", "FontSize", 8);
        set(axesHandle, "FontSize", 8, "TickLabelInterpreter", "latex");
        if panelIndex < size(panels, 1)
            set(axesHandle, "XTickLabel", []);
        end
    end
    xlabel(tiles, "azimuth bin index along the ring", ...
        "Interpreter", "latex", "FontSize", 8);
end

function writtenFiles = localWriteFigure( ...
        figureHandle, outputDirectory, figureBaseName)
%LOCALWRITEFIGURE Export vector and raster copies of the figure.
    if ~isfolder(outputDirectory)
        mkdir(outputDirectory);
    end
    pdfPath = fullfile(outputDirectory, figureBaseName + ".pdf");
    pngPath = fullfile(outputDirectory, figureBaseName + ".png");
    exportgraphics(figureHandle, pdfPath, "ContentType", "vector");
    exportgraphics(figureHandle, pngPath, "Resolution", 300);
    writtenFiles = [string(pdfPath); string(pngPath)];
end
