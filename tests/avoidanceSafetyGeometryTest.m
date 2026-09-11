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
        function directionalSupportsBoundAllAdmittedRectangleHeadings(testCase)
            worst = localSupportResidual();
            testCase.verifyLessThanOrEqual(worst, 1e-12);
        end

        function theChartBoundContainsPosesAcrossSegmentBoundaries(testCase)
            cfg = collisionAvoidanceControllerConfig();
            ego = struct("position", [0.0; 0.0], "yaw", 0.0, "speed", 1.0);
            ego.stateTime = 0;
            ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
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

        function aReferenceVertexRequiresAnExplicitJumpCertificate(testCase)
            cfg = collisionAvoidanceControllerConfig(struct("controller",struct("horizonSteps",4)));
            ego = struct("position",[9;0],"yaw",0,"speed",1);
            ego.stateTime = 0;
            ego.perception = struct("time",0,"range",30,"completeWithinRange",true);
            testCase.verifyError(@() collisionAvoidanceController(ego,encounterTestFixture.stationaryTarget(),[0,0;10,0;20,0.2],cfg,[]), ...
                "collisionAvoidanceController:unsupportedReferenceJump");
        end
    end
end

function worst = localSupportResidual()
    worst = -inf;
    for angle = linspace(-pi,pi,25)
        normal = [cos(angle);sin(angle)];
        for center = linspace(-pi,pi,13)
            radius = 0.47;
            support = targetPrediction.rectangleSupport(2.4,0.95,normal,center,radius);
            yaw = linspace(center-radius,center+radius,501);
            actual = 2.4*abs(cos(angle-yaw))+0.95*abs(sin(angle-yaw));
            worst = max(worst,max(actual-support));
        end
    end
end
