classdef avoidanceSafetyGeometryTest < matlab.unittest.TestCase
    % Regression for a readmission reference crossing a route vertex at rest.

    methods (TestClassSetup)
        function addControllerPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
        end
    end

    methods (Test)
        function theChartBoundContainsPosesAcrossSegmentBoundaries(testCase)
            cfg = collisionAvoidanceControllerConfig();
            ego = struct("position", [0.0; 0.0], "yaw", 0.0, "speed", 1.0);
            [~, lane] = readPlanningInputs(ego, [], ...
                [0.0, 0.0; 5.0, 0.0; 6.0, 0.03; 7.0, 0.08; 20.0, 0.9], cfg);
            frame = laneGeometry.frameBounds(lane, 6.0, 2.0, 12.0);
            stations = linspace(frame.stationLower, frame.stationUpper, 401);
            state = zeros(3, 3*numel(stations));
            state(1, :) = repmat(stations, 1, 3);
            state(2, :) = repelem([-12.0, 0.0, 12.0], numel(stations));

            [position, heading] = laneGeometry.fromFrenet(state, lane);
            affinePosition = frame.origin+[frame.tangent, frame.lateral]*state(1:2, :);
            error = max(abs(position-affinePosition), [], 2);

            testCase.verifyLessThanOrEqual(error, frame.positionErrorBound+1.0e-12);
            testCase.verifyLessThanOrEqual(max(abs(heading-frame.heading)), ...
                frame.headingErrorBound+1.0e-12);
            testCase.verifyGreaterThan(frame.stationUpper-frame.stationLower, 3.9);
        end

        function aStationaryEndpointCanOccupyAPolylineVertex(testCase)
            cfg = collisionAvoidanceControllerConfig();
            ego = struct("position", [9.0; 0.0], "yaw", 0.0, ...
                "speed", 1.0);
            [~, lane] = readPlanningInputs( ...
                ego, [], [0.0, 0.0; 10.0, 0.0; 20.0, 0.2], cfg);
            model = struct("cfg", cfg, "lane", lane, "hasTarget", false, ...
                "targetKey", "", "road", struct("boundaries", struct([])));
            prediction = struct("nodeCount", 3, "planCount", 4, ...
                "scheduleSpeedProfile", [1.0, 0.0, 0.0]);
            anchor = zeros(6, 3);
            anchor(1, :) = [9.0, 10.0+2.0e-5, 10.0-2.0e-5];

            geometry = avoidanceSafetyGeometry(model, prediction, anchor);

            last = geometry.frames(end-1:end);
            testCase.verifyLessThanOrEqual(max([last.stationLower]), 10.0);
            testCase.verifyGreaterThanOrEqual(min([last.stationUpper]), 10.0);
            restPosition = [10.0; 0.4];
            firstPose = last(1).origin ...
                + [last(1).tangent, last(1).lateral]*restPosition;
            secondPose = last(2).origin ...
                + [last(2).tangent, last(2).lateral]*restPosition;
            testCase.verifyEqual(firstPose, secondPose, AbsTol=1.0e-14);
        end
    end
end
