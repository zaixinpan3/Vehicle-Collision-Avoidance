classdef collisionRectangleEnvelopeTest < matlab.unittest.TestCase
    % collisionRectangleEnvelopeTest Public oriented-rectangle geometry.

    properties (TestParameter)
        separation = struct("separated", 0.25, "tangent", 0.0, ...
            "overlapping", -0.20)
    end

    methods (TestClassSetup)
        function addGeometryPath(testCase)
            repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repositoryRoot, "controller")));
        end
    end

    methods (Test)
        function signedDistanceMeasuresGapOrPenetration(testCase, separation)
            dimensions = [2.4; 0.95; 2.4; 0.95];

            distance = rectangleConfigurationDistance( ...
                [0.0; 0.0], 0.0, [4.8+separation; 0.0], 0.0, dimensions);

            testCase.verifyEqual(distance, separation, AbsTol=1.0e-12);
        end

        function targetOrientationChangesClearance(testCase)
            dimensions = [2.4; 0.95; 2.4; 0.95];

            aligned = rectangleConfigurationDistance( ...
                [0.0; 0.0], 0.0, [4.8; 0.0], 0.0, dimensions);
            perpendicular = rectangleConfigurationDistance( ...
                [0.0; 0.0], 0.0, [4.8; 0.0], pi/2.0, dimensions);

            testCase.verifyEqual(aligned, 0.0, AbsTol=1.0e-12);
            testCase.verifyEqual(perpendicular, 1.45, AbsTol=1.0e-12);
        end

        function crossingRectanglesUsePenetrationDepth(testCase)
            dimensions = [2.4; 0.95; 2.4; 0.95];

            [distance, ~, ~, outside] = rectangleConfigurationDistance( ...
                [0.0; 0.0], 0.0, [0.0; 0.0], pi/2.0, dimensions);

            testCase.verifyEqual(distance, -3.35, AbsTol=1.0e-12);
            testCase.verifyFalse(outside);
        end

        function cornerProjectionReturnsTheSupportingNormal(testCase)
            dimensions = [2.4; 0.95; 2.4; 0.95];

            [distance, normal, support, outside] = ...
                rectangleConfigurationDistance( ...
                    [0.0; 0.0], 0.0, [7.8; 5.9], 0.0, dimensions);

            testCase.verifyEqual(distance, 5.0, AbsTol=1.0e-12);
            testCase.verifyEqual(normal, [-0.6; -0.8], AbsTol=1.0e-12);
            testCase.verifyEqual(-support, distance, AbsTol=1.0e-12);
            testCase.verifyTrue(outside);
        end

        function rigidFrameChangePreservesDistanceAndRotatesNormal(testCase)
            dimensions = [2.4; 0.95; 2.0; 0.8];
            ego = [0.0; 0.0];
            target = [9.0; 3.0];
            angle = 0.73;
            rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
            translation = [110.0; -80.0];
            [originalDistance, originalNormal] = rectangleConfigurationDistance( ...
                ego, 0.2, target, -0.4, dimensions);

            [distance, normal] = rectangleConfigurationDistance( ...
                rotation*ego+translation, 0.2+angle, ...
                rotation*target+translation, -0.4+angle, dimensions);

            testCase.verifyEqual(distance, originalDistance, AbsTol=1.0e-12);
            testCase.verifyEqual(normal, rotation*originalNormal, AbsTol=1.0e-12);
        end
    end
end
