function margin = rectangleSeparationMargin( ...
        egoPosition, egoYaw, targetPosition, targetYaw, ...
        egoLength, egoWidth, targetLength, targetWidth)
% rectangleSeparationMargin Separating-axis margin between two rectangles.
%
% Per-sample separating-axis-theorem margin between the oriented ego and
% target rectangles: the largest axis gap over the four face normals,
% positive when a separating axis exists and negative on overlap.
% Positions are N-by-2, yaws N-by-1, dimensions scalar.
%
% This is the scenario-acceptance geometry, deliberately independent of
% the controller's own avoidanceSafetyGeometry.rectangleDistance so that acceptance
% checks do not certify the controller with its own code.

    sampleCount = size(egoPosition, 1);
    margin = NaN(sampleCount, 1);
    egoHalfExtent = 0.5 * [egoLength, egoWidth];
    targetHalfExtent = 0.5 * [targetLength, targetWidth];
    for sampleIdx = 1:sampleCount
        egoAxes = [ ...
            cos(egoYaw(sampleIdx)), -sin(egoYaw(sampleIdx)); ...
            sin(egoYaw(sampleIdx)), cos(egoYaw(sampleIdx))];
        targetAxes = [ ...
            cos(targetYaw(sampleIdx)), -sin(targetYaw(sampleIdx)); ...
            sin(targetYaw(sampleIdx)), cos(targetYaw(sampleIdx))];
        axes = [egoAxes, targetAxes];
        centerDifference = ...
            targetPosition(sampleIdx, :) - egoPosition(sampleIdx, :);
        axisGap = NaN(4, 1);
        for axisIdx = 1:4
            axis = axes(:, axisIdx);
            egoRadius = sum(egoHalfExtent ...
                .* abs(egoAxes.' * axis).');
            targetRadius = sum(targetHalfExtent ...
                .* abs(targetAxes.' * axis).');
            axisGap(axisIdx) = abs(centerDifference * axis) ...
                - egoRadius - targetRadius;
        end
        margin(sampleIdx) = max(axisGap);
    end
end
