classdef groundPointsProcessingTest < matlab.unittest.TestCase
% groundPointsProcessingTest Behavior of sparse-ring curb recovery.

    methods (TestClassSetup)
        function addPerceptionPath(testCase)
            repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(fullfile( ...
                    repositoryRoot, ...
                    "perception", ...
                    "curbDetectionExperiment")));
        end
    end

    methods (Test)
        function recoversBothBoundariesAcrossMissingRings(testCase)
            frame = localSyntheticFrame(true);

            result = groundPointsProcessing(frame, struct());

            testCase.verifyTrue(result.curbModels.leftForward.found);
            testCase.verifyTrue(result.curbModels.rightForward.found);
            testCase.verifyGreaterThanOrEqual(size(result.left, 1), 80);
            testCase.verifyGreaterThanOrEqual(size(result.right, 1), 80);
            testCase.verifyEqual( ...
                result.curbModels.leftForward.intercept, ...
                -6.0, AbsTol=0.05);
            testCase.verifyEqual( ...
                result.curbModels.rightForward.intercept, ...
                5.0, AbsTol=0.05);
        end

        function recoversSameCurbsAfterQuarterTurn(testCase)
            referenceFrame = localSyntheticFrame(true);
            rotatedFrame = localRotateFrame(referenceFrame, 90.0);
            referenceResult = groundPointsProcessing( ...
                referenceFrame, struct());

            rotatedResult = groundPointsProcessing(rotatedFrame, struct());

            testCase.verifyEqual( ...
                rotatedResult.curbs, referenceResult.curbs);
            testCase.verifyLessThanOrEqual( ...
                localLineHeadingError( ...
                    rotatedResult.curbModels.roadFrame.headingDegrees, ...
                    90.0), ...
                2.0);
        end

        function retainsCurvedCurbsAfterObliqueRotation(testCase)
            referenceFrame = localSyntheticCurvedFrame();
            rotatedFrame = localRotateFrame(referenceFrame, 37.0);
            referenceResult = groundPointsProcessing( ...
                referenceFrame, struct());

            rotatedResult = groundPointsProcessing(rotatedFrame, struct());
            overlapRate = localRowOverlapRate( ...
                referenceResult.curbs, rotatedResult.curbs);

            testCase.verifyGreaterThanOrEqual(overlapRate, 0.95);
            testCase.verifyTrue( ...
                rotatedResult.curbModels.leftForward.found);
            testCase.verifyTrue( ...
                rotatedResult.curbModels.rightForward.found);
            testCase.verifyLessThanOrEqual( ...
                localLineHeadingError( ...
                    rotatedResult.curbModels.roadFrame.headingDegrees, ...
                    37.0), ...
                2.0);
        end

        function rejectsIsolatedHeightDiscontinuities(testCase)
            frame = localSyntheticFrame(false);

            result = groundPointsProcessing(frame, struct());

            testCase.verifyEmpty(result.curbs);
            testCase.verifyFalse(result.curbModels.left.found);
            testCase.verifyFalse(result.curbModels.right.found);
        end

        function rejectsRoadGradeAcrossLargeRadialGap(testCase)
            frame = localSyntheticRoadGradeFrame();

            result = groundPointsProcessing(frame, struct());

            testCase.verifyEmpty(result.curbs);
            testCase.verifyFalse(result.curbModels.left.found);
            testCase.verifyFalse(result.curbModels.leftForward.found);
        end

        function recoversForwardCurbsOnQuadraticRoad(testCase)
            frame = localSyntheticCurvedFrame();

            result = groundPointsProcessing(frame, struct());

            testCase.verifyTrue(result.curbModels.leftForward.found);
            testCase.verifyTrue(result.curbModels.rightForward.found);
            testCase.verifyEqual( ...
                result.curbModels.leftForward.curvature, ...
                0.006, AbsTol=2.0e-3);
            testCase.verifyEqual( ...
                result.curbModels.rightForward.curvature, ...
                -0.010, AbsTol=2.0e-3);
            testCase.verifyGreaterThanOrEqual(size(result.left, 1), 80);
            testCase.verifyGreaterThanOrEqual(size(result.right, 1), 80);
        end

        function forwardCurveSupersedesLineInSharedDomain(testCase)
            frame = localSyntheticCurvedFrame();
            [frame, distractorIndex] = ...
                localAddLineOnlyForwardDistractor(frame);

            result = groundPointsProcessing(frame, struct());

            testCase.verifyFalse(ismember( ...
                distractorIndex, result.curbs, "rows"));
        end

        function recoversElevatedOuterCurbSurface(testCase)
            frame = localSyntheticCurvedFrame();
            [frame, outerSurfaceIndex] = ...
                localAddElevatedOuterSurfacePoint(frame);

            result = groundPointsProcessing(frame, struct());

            testCase.verifyTrue(ismember( ...
                outerSurfaceIndex, result.left, "rows"));
        end

        function recoversBoundaryWithFifteenAzimuths(testCase)
            frame = localSyntheticSparseBoundaryFrame();

            result = groundPointsProcessing(frame, struct());

            testCase.verifyTrue(result.curbModels.leftForward.found);
            testCase.verifyTrue(result.curbModels.rightForward.found);
            testCase.verifyGreaterThanOrEqual( ...
                result.curbModels.leftForward.azimuthSupport, 15);
            testCase.verifyGreaterThanOrEqual( ...
                result.curbModels.rightForward.azimuthSupport, 15);
        end
    end
end

function frame = localSyntheticSparseBoundaryFrame()
    numRings = 7;
    numAzimuths = 30;
    frame.x = nan(numRings, numAzimuths);
    frame.y = nan(numRings, numAzimuths);
    frame.z = nan(numRings, numAzimuths);
    frame.range = nan(numRings, numAzimuths);
    frame.valid = false(numRings, numAzimuths);
    frame = localAddBoundary(frame, 1:15, 5.0);
    frame = localAddBoundary(frame, 16:30, -6.0);
end

function frame = localSyntheticRoadGradeFrame()
    numRings = 7;
    numAzimuths = 60;
    frame.x = nan(numRings, numAzimuths);
    frame.y = nan(numRings, numAzimuths);
    frame.z = nan(numRings, numAzimuths);
    frame.range = nan(numRings, numAzimuths);
    frame.valid = false(numRings, numAzimuths);

    positionX = linspace(-12.0, 12.0, numAzimuths);
    boundaryY = -6.0;
    radialGapM = 1.2;
    for column = 1:numAzimuths
        lowerRangeM = hypot(positionX(column), boundaryY);
        upperScale = (lowerRangeM + radialGapM) / lowerRangeM;
        frame.x(2, column) = positionX(column) * upperScale;
        frame.y(2, column) = boundaryY * upperScale;
        frame.z(2, column) = -2.70;
        frame.range(2, column) = lowerRangeM + radialGapM;
        frame.valid(2, column) = true;

        frame.x(4, column) = positionX(column);
        frame.y(4, column) = boundaryY;
        frame.z(4, column) = -2.82;
        frame.range(4, column) = lowerRangeM;
        frame.valid(4, column) = true;
    end
end

function frame = localSyntheticFrame(includeContinuousCurbs)
    numRings = 7;
    numAzimuths = 120;
    frame.x = nan(numRings, numAzimuths);
    frame.y = nan(numRings, numAzimuths);
    frame.z = nan(numRings, numAzimuths);
    frame.range = nan(numRings, numAzimuths);
    frame.valid = false(numRings, numAzimuths);

    if includeContinuousCurbs
        frame = localAddBoundary(frame, 1:60, 5.0);
        frame = localAddBoundary(frame, 61:120, -6.0);
    else
        columns = [5, 19, 37, 64, 88, 113];
        boundaryY = [4.0, 7.0, 10.0, -3.5, -8.0, -13.0];
        for pointIndex = 1:numel(columns)
            column = columns(pointIndex);
            positionX = -10.0 + 20.0 * ...
                (pointIndex - 1) / (numel(columns) - 1);
            frame = localWriteTransition( ...
                frame, column, positionX, boundaryY(pointIndex));
        end
    end
end

function frame = localSyntheticCurvedFrame()
    numRings = 7;
    numAzimuths = 120;
    frame.x = nan(numRings, numAzimuths);
    frame.y = nan(numRings, numAzimuths);
    frame.z = nan(numRings, numAzimuths);
    frame.range = nan(numRings, numAzimuths);
    frame.valid = false(numRings, numAzimuths);

    positionX = linspace(0.0, 20.0, 60);
    rightBoundaryY = 7.7 - 0.010 * positionX.^2;
    leftBoundaryY = -11.2 + 0.006 * positionX.^2;
    for pointIndex = 1:numel(positionX)
        frame = localWriteTransition( ...
            frame, pointIndex, ...
            positionX(pointIndex), rightBoundaryY(pointIndex));
        frame = localWriteTransition( ...
            frame, pointIndex + 60, ...
            positionX(pointIndex), leftBoundaryY(pointIndex));
    end
end

function [frame, distractorIndex] = ...
        localAddLineOnlyForwardDistractor(frame)
    initialResult = groundPointsProcessing(frame, struct());
    lineModel = initialResult.curbModels.left;
    positionX = 20.3;
    positionY = ...
        lineModel.slope * positionX + lineModel.intercept;
    positionZ = 0.5 * (lineModel.minimumZ + lineModel.maximumZ);
    ring = 1;
    azimuth = 90;
    frame.x(ring, azimuth) = positionX;
    frame.y(ring, azimuth) = positionY;
    frame.z(ring, azimuth) = positionZ;
    frame.range(ring, azimuth) = hypot(positionX, positionY);
    frame.valid(ring, azimuth) = true;
    distractorIndex = [azimuth, ring];
end

function [frame, outerSurfaceIndex] = ...
        localAddElevatedOuterSurfacePoint(frame)
    positionX = 10.0;
    centerlineY = -11.2 + 0.006 * positionX^2;
    positionY = centerlineY - 0.42;
    positionZ = -2.70;
    ring = 1;
    azimuth = 90;
    frame.x(ring, azimuth) = positionX;
    frame.y(ring, azimuth) = positionY;
    frame.z(ring, azimuth) = positionZ;
    frame.range(ring, azimuth) = hypot(positionX, positionY);
    frame.valid(ring, azimuth) = true;
    outerSurfaceIndex = [azimuth, ring];
end

function frame = localAddBoundary(frame, columns, boundaryY)
    positionX = linspace(-12.0, 12.0, numel(columns));
    for columnIndex = 1:numel(columns)
        frame = localWriteTransition( ...
            frame, columns(columnIndex), ...
            positionX(columnIndex), boundaryY);
    end
end

function frame = localWriteTransition(frame, column, positionX, boundaryY)
    % Ring 3 is intentionally absent between rings 2 and 4.
    frame.x(2, column) = positionX;
    frame.y(2, column) = boundaryY;
    frame.z(2, column) = -2.70;
    frame.range(2, column) = hypot(positionX, boundaryY);
    frame.valid(2, column) = true;

    frame.x(4, column) = positionX;
    frame.y(4, column) = boundaryY;
    frame.z(4, column) = -2.82;
    frame.range(4, column) = hypot(positionX, boundaryY);
    frame.valid(4, column) = true;

    % A second curb-face return is recovered by fitted-boundary expansion.
    frame.x(5, column) = positionX;
    frame.y(5, column) = boundaryY + 0.18 * sign(boundaryY);
    frame.z(5, column) = -2.84;
    frame.range(5, column) = hypot( ...
        frame.x(5, column), frame.y(5, column));
    frame.valid(5, column) = true;
end

function rotatedFrame = localRotateFrame(frame, headingDegrees)
    rotatedFrame = frame;
    cosine = cosd(headingDegrees);
    sine = sind(headingDegrees);
    rotatedFrame.x = cosine * frame.x - sine * frame.y;
    rotatedFrame.y = sine * frame.x + cosine * frame.y;
end

function errorDegrees = localLineHeadingError( ...
        actualDegrees, expectedDegrees)
    errorDegrees = abs( ...
        mod(actualDegrees - expectedDegrees + 90.0, 180.0) - 90.0);
end

function overlapRate = localRowOverlapRate(referenceRows, actualRows)
    commonRows = intersect(referenceRows, actualRows, "rows");
    overlapRate = size(commonRows, 1) / size(referenceRows, 1);
end
