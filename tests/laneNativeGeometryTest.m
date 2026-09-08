classdef laneNativeGeometryTest < matlab.unittest.TestCase
%laneNativeGeometryTest Batched geometry retains endpoints and vertex bounds.

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "solver", "clarabel", "matlab")));
        end
    end

    methods (Test)
        function batchedProjectionRetainsEndpointAndFirstTieSemantics(testCase)
            lane = localLane([0, 0; 1, 0; 1, 1]);

            projection = laneGeometry.project([-1, 1, 2, 0.5; 0, 0, 2, 0.5], lane);

            testCase.verifyEqual(projection.point, [0, 1, 1, 0.5; 0, 0, 1, 0], AbsTol=1e-14);
            testCase.verifyEqual(projection.heading, [0, 0, pi/2, 0], AbsTol=1e-14);
            testCase.verifyEqual(projection.station, [0, 1, 2, 0.5], AbsTol=1e-14);
            testCase.verifyEqual(projection.lateralPosition, [0, 0, -1, 0.5], AbsTol=1e-14);
        end

        function batchedFramesIncludeBothSidesOfAVertex(testCase)
            lane = localLane([0, 0; 1, 0; 1, 1]);

            frames = laneGeometry.frameBounds(lane, [0.5, 1.0], 0.5, 2.0);

            testCase.verifyEqual([frames.segmentIndex], [1, 2]);
            testCase.verifyEqual([frames.stationLower], [0, 0.5]);
            testCase.verifyEqual([frames.stationUpper], [1, 1.5]);
            testCase.verifyEqual([frames.headingErrorBound], [pi/2, pi/2], AbsTol=1e-14);
            testCase.verifyEqual([frames.positionErrorBound], [2, 2.5; 2, 2.5], AbsTol=1e-14);
        end

        function nativeAndMatlabGeometryAgreeOnACurvedPolyline(testCase)
            station = linspace(-20, 40, 501).';
            lane = localLane([sin(station/30), 1-cos(station/30)]*30);
            queries = [linspace(-24, 44, 101); 3*cos(linspace(0, 2*pi, 101))];
            nativeProjection = laneGeometry.project(queries, lane);
            nativeFrames = laneGeometry.frameBounds(lane, [0, 0.1, 5, 25, 59, 60], 2, 12);

            [referenceProjection, referenceFrames] = localWithoutNative(queries, lane);

            testCase.verifyEqual(nativeProjection, referenceProjection, AbsTol=2e-12);
            testCase.verifyEqual(nativeFrames, referenceFrames, AbsTol=2e-12);
        end

        function individualRadiiRetainEveryPolylineCornerBound(testCase)
            lane = localLane([0,0;1,0;1,1;2,1]);
            station = [0.5,1,1.5,2.5];radii = [0.1,0.5,1,0.25];
            frames = laneGeometry.frameBounds(lane,station,radii,2);
            for index = 1:numel(station)
                expected = laneGeometry.frameBounds(lane,station(index),radii(index),2);
                testCase.verifyEqual(frames(index),expected,AbsTol=1e-14);
            end
            testCase.verifyEqual([frames.stationLower],[0.4,0.5,0.5,2.25],AbsTol=1e-14);
            testCase.verifyEqual([frames.stationUpper],[0.6,1.5,2.5,2.75],AbsTol=1e-14);
            testCase.verifyEqual([frames.headingErrorBound],[0,pi/2,pi/2,0],AbsTol=1e-14);
        end

        function nativeProjectionRejectsZeroLengthSegments(testCase)
            testCase.verifyError(@localZeroLengthProjection, "projectLanePolylineMex:length");
        end

        function nativeFrameSearchRejectsUnsortedStations(testCase)
            testCase.verifyError(@localUnsortedFrames, "laneFrameBoundsMex:order");
        end
    end
end

function localZeroLengthProjection()
    result = projectLanePolylineMex([0; 0], [0, 0], [0, 0], 0, 0, [1, 0]); %#ok<NASGU>
end

function localUnsortedFrames()
    result = laneFrameBoundsMex(0, 1, 1, [0, 0; 1, 0], [1; 1], [1; 0], [1, 0; 1, 0]); %#ok<NASGU>
end

function lane = localLane(points)
    cfg = collisionAvoidanceControllerConfig();
    [~, lane] = readPlanningInputs(struct("position", [0; 0], "yaw", 0, "speed", 1), [], points, cfg);
end

function [projection, frames] = localWithoutNative(queries, lane)
    directory = fileparts(which("projectLanePolylineMex"));
    restore = onCleanup(@() localRestore(directory));
    rmpath(directory);
    clear laneGeometry
    assert(exist("projectLanePolylineMex", "file") ~= 3);
    projection = laneGeometry.project(queries, lane);
    frames = laneGeometry.frameBounds(lane, [0, 0.1, 5, 25, 59, 60], 2, 12);
end

function localRestore(directory)
    addpath(directory);
    clear laneGeometry
end
