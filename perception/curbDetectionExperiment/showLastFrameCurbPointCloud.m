curbFrameIndex = 200; % Zero-based sample index; change this first line to inspect another frame.

%% Show one outer-loop CARLA LiDAR frame with ring-feature curb candidates
% These candidates come only from local geometric features measured along
% the LiDAR rings and their family-consensus fusion. No road frame is
% estimated, no boundary quadratic is fitted, and no tracking runs: the
% final curb quadratic for a frame is produced downstream by the temporal
% tracker, not here.
%
% A candidate must carry adjacent-ring compression evidence, which is the
% feature family that responds to a raised curb face rather than to flat
% pavement structure. On the development labels that requirement admits none
% of the 71 hard negatives, against 4.2 percent without it.

scriptPath = mfilename("fullpath");
repositoryRoot = fileparts(fileparts(fileparts(scriptPath)));
datasetFolder = fullfile( ...
    repositoryRoot, ...
    "simulation", ...
    "data", ...
    "carla_outer_loop_lap_10hz_20260802_epic_no_rt_auto");
lidarFolder = fullfile(datasetFolder, "lidar");
curbDetectorFolder = fileparts(scriptPath);

if ~isfolder(lidarFolder)
    error("Unable to find the LiDAR folder: %s", lidarFolder);
end
if ~isfile(fullfile(curbDetectorFolder, "computeLidarRingCurbFeatures.m"))
    error("Unable to find computeLidarRingCurbFeatures.m in: %s", ...
        curbDetectorFolder);
end
addpath(curbDetectorFolder);

lidarFiles = dir(fullfile(lidarFolder, "*.npy"));
if isempty(lidarFiles)
    error("No .npy LiDAR frames were found in: %s", lidarFolder);
end
[~, fileOrder] = sort({lidarFiles.name});
lidarFiles = lidarFiles(fileOrder);
if exist("curbFrameIndex", "var") && ~isempty(curbFrameIndex)
    validateattributes( ...
        curbFrameIndex, ...
        {'numeric'}, ...
        {'scalar', 'integer', 'nonnegative', '<', numel(lidarFiles)});
    selectedFrameIndex = double(curbFrameIndex);
else
    selectedFrameIndex = numel(lidarFiles) - 1;
end
selectedLidarFile = lidarFiles(selectedFrameIndex + 1);
selectedFramePath = fullfile(lidarFolder, selectedLidarFile.name);

pythonEnvironment = pyenv;
if pythonEnvironment.Status == "NotLoaded"
    pythonExecutable = ...
        "/home/zai/.cache/carla-yolo-range-venv/bin/python";
    if isfile(pythonExecutable)
        pyenv( ...
            Version=pythonExecutable, ...
            ExecutionMode="OutOfProcess");
    end
end

[rawPoints, organizedFrame, rawPointIndexGrid] = ...
    loadOrganizedFrame(selectedFramePath);
lidarPoints = rawPoints(:, 1:3);

[candidateCellMask, candidateDiagnostics] = ...
    extractRingFeatureCurbCandidates(organizedFrame, struct());

curbPointIndices = localCellsToRawIndices( ...
    candidateCellMask, rawPointIndexGrid);
curbMask = false(size(lidarPoints, 1), 1);
curbMask(curbPointIndices) = true;
curbPointsXyz = lidarPoints(curbMask, :);

pointColors = repmat( ...
    uint8([145, 145, 145]), size(lidarPoints, 1), 1);
pointColors(curbMask, :) = repmat( ...
    uint8([255, 0, 0]), nnz(curbMask), 1);
coloredCloud = pointCloud(lidarPoints, Color=pointColors);

fprintf( ...
    "Frame %d: %d ring-feature curb candidates " + ...
    "(fusion %d, after company %d), road level %.3f m.\n", ...
    selectedFrameIndex, nnz(curbMask), ...
    candidateDiagnostics.fusionCellCount, ...
    candidateDiagnostics.companyCellCount, ...
    candidateDiagnostics.roadLevelZ);
fprintf( ...
    "curbPointIndices holds their one-based raw-cloud indices; " + ...
    "curbPointsXyz holds their XYZ coordinates.\n");

figureName = sprintf( ...
    "CARLA LiDAR %s - Ring Feature Curb Candidates", ...
    erase(selectedLidarFile.name, ".npy"));
oldFigures = findall( ...
    groot, "Type", "figure", "Name", figureName);
close(oldFigures);

figureHandle = figure( ...
    "Name", figureName, ...
    "NumberTitle", "off", ...
    "Visible", "on");
figureHandle.WindowState = "maximized";
pcshow(coloredCloud, "MarkerSize", 24);
axis equal;
% CARLA's sensor frame is left-handed with +Y to the vehicle's right, while
% MATLAB draws a right-handed triad, so plotting these coordinates as they
% are plots the mirror image: a curb on the vehicle's right appears on the
% left of the figure and disagrees with the rendered video. Reversing the
% Y axis direction restores the real sense. It changes only how the axes
% are drawn, never the data, so the data cursor below still reports true
% CARLA coordinates and points read off this figure remain directly usable.
axesHandle = gca;
axesHandle.YDir = "reverse";
% Open on a bird's eye with the vehicle's forward axis up the screen, which
% is the orientation in which a curb's side is unambiguous and the one the
% rendered chase video shows. Rotating the axes still works normally.
view(axesHandle, -90, 90);
xlabel("X (m), forward");
ylabel("Y (m), +Y to the vehicle's right (CARLA)");
zlabel("Z (m), up");
title(sprintf( ...
    "%s: ring-feature curb candidates (red) = %d, other (gray) = %d", ...
    erase(selectedLidarFile.name, ".npy"), ...
    nnz(curbMask), ...
    nnz(~curbMask)));

dataCursor = datacursormode(figureHandle);
dataCursor.Enable = "on";
dataCursor.SnapToDataVertex = "on";
dataCursor.DisplayStyle = "datatip";
dataCursor.UpdateFcn = @showPointCoordinates;

shg;

function dataTipText = showPointCoordinates(~, eventData)
    point = eventData.Position;
    fprintf( ...
        "[%.6f,%.6f,%.6f]\n", ...
        point(1), point(2), point(3));
    dataTipText = {sprintf( ...
        "X: %.6f m\nY: %.6f m\nZ: %.6f m", ...
        point(1), point(2), point(3))};
end

function rawIndices = localCellsToRawIndices( ...
        organizedMask, rawPointIndexGrid)
%LOCALCELLSTORAWINDICES Map organized-grid cells to raw point indices.
    gridIndices = rawPointIndexGrid(organizedMask);
    rawIndices = unique(gridIndices(gridIndices > 0), "stable");
end

function [rawPoints, organizedFrame, rawPointIndexGrid] = ...
        loadOrganizedFrame(framePath)
%LOADORGANIZEDFRAME Organize one raw .npy sweep into the 64x1000 grid.
    rawPoints = double(py.numpy.load(framePath));
    if size(rawPoints, 2) < 3
        error( ...
            "LiDAR frame must contain at least XYZ columns: %s", ...
            framePath);
    end
    [organizedFrame, rawPointIndexGrid] = ...
        organizeLidarSweepGrid(rawPoints);
end
