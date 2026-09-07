classdef uncertaintyGeometryTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "config")));
        end
    end
    methods (Test)
        function aChangedClosestSegmentEnlargesTheHeadingBound(testCase)
            [state, radius, lane, samples] = localCornerInputs();
            [bound, chartValid] = stateUncertainty.toFrenet(state, radius, lane);
            errors = localProjectionErrors(state, samples, lane);

            testCase.verifyGreaterThan(bound(3), radius(3));
            testCase.verifyFalse(chartValid);
            testCase.verifyLessThanOrEqual(max(errors, [], 2), bound+1e-12);
        end

        function aStraightRoadRetainsTheDirectionalPositionBounds(testCase)
            cfg = collisionAvoidanceControllerConfig();
            ego = struct("position", [10; 0], "yawAngle", 0, ...
                "longitudinalVelocity", 5, "lateralVelocity", 0, "yawRate", 0);
            [~, lane] = readPlanningInputs(ego, [], [0, 0; 100, 0], cfg);
            bound = stateUncertainty.toFrenet([10; 0; 0; 5; 0; 0], ...
                [0.1; 0.2; 0.01; 0.03; 0.04; 0.005], lane);

            testCase.verifyEqual(bound, [0.1; 0.2; 0.01; 0.03; 0.04; 0.005], AbsTol=1e-13);
        end

        function continuousForcingIncludesCrossChannelTransport(testCase)
            bound = stateUncertainty.heldDisturbance([0, 1; 0, 0], [0; 2], 0.1);
            testCase.verifyEqual(bound, [0.01; 0.2], AbsTol=1e-14);
        end

        function aDecayingChannelUsesItsActualDiagonal(testCase)
            bound = stateUncertainty.heldDisturbance(-2, 0.3, 0.2);
            testCase.verifyEqual(bound, 0.15*(1-exp(-0.4)), AbsTol=1e-14);
        end

        function yawWrappingIsCoveredEvenWithoutPositionError(testCase)
            cfg = collisionAvoidanceControllerConfig();
            ego = struct("position", [10; 0], "yawAngle", pi-0.001, "speed", 0);
            [~, lane] = readPlanningInputs(ego, [], [0, 0; 100, 0], cfg);
            radius = stateUncertainty.toFrenet([10; 0; pi-0.001; 0; 0; 0], ...
                [0; 0; 0.01; 0; 0; 0], lane);
            testCase.verifyEqual(radius(3), 2*pi);
        end
    end
end

function [state, radius, lane, samples] = localCornerInputs()
    cfg = collisionAvoidanceControllerConfig();
    ego = struct("position", [10; 0], "yawAngle", 0, ...
        "longitudinalVelocity", 5, "lateralVelocity", 0, "yawRate", 0);
    [~, lane] = readPlanningInputs(ego, [], [0, 0; 10, 0; 20, 1], cfg);
    state = [10; 0; 0; 5; 0; 0];
    radius = [0.2; 0.2; 0.01; 0; 0; 0];
    [x, y, yaw] = ndgrid(linspace(-0.2, 0.2, 9), linspace(-0.2, 0.2, 9), [-0.01, 0.01]);
    samples = state+[x(:).'; y(:).'; yaw(:).'; zeros(3, numel(x))];
end

function errors = localProjectionErrors(state, samples, lane)
    centre = laneGeometry.project(state(1:2), lane);
    projected = laneGeometry.project(samples(1:2, :), lane);
    errors = abs([projected.station-centre.station; ...
        projected.lateralPosition-centre.lateralPosition; ...
        samples(3, :)-projected.heading-(state(3)-centre.heading); ...
        samples(4:6, :)-state(4:6)]);
end
