classdef estimateRoadHeadingFromFeatureSeedsTest ...
        < matlab.unittest.TestCase
%ESTIMATEROADHEADINGFROMFEATURESEEDSTEST Orientation-estimator checks.

    methods (TestClassSetup)
        function addDetectorFolder(testCase)
            repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
            detectorFolder = fullfile( ...
                repositoryRoot, "perception", ...
                "curbDetectionExperiment");
            originalPath = path();
            testCase.addTeardown(@() path(originalPath));
            addpath(detectorFolder);
        end
    end

    methods (Test)
        function dominantRoadAlignedStructureRejectsCrosswiseDistractor( ...
                testCase)
            [x, y, azimuth, weight] = localSeedGeometry();
            estimate = estimateRoadHeadingFromFeatureSeeds( ...
                x, y, azimuth, weight);

            testCase.verifyTrue(estimate.found);
            testCase.verifyEqual( ...
                estimate.headingDegrees, 0.0, AbsTol=1.0);
            testCase.verifyGreaterThan(estimate.scoreMargin, 0.0);
            testCase.verifyGreaterThanOrEqual( ...
                estimate.azimuthSupport, 90);
        end

        function rotatingEverySeedRotatesEstimatedLine(testCase)
            [x, y, azimuth, weight] = localSeedGeometry();
            rotationDegrees = 37.0;
            [rotatedX, rotatedY] = localRotatePoints( ...
                x, y, rotationDegrees);
            estimate = estimateRoadHeadingFromFeatureSeeds( ...
                rotatedX, rotatedY, azimuth, weight);

            testCase.verifyTrue(estimate.found);
            testCase.verifyEqual( ...
                estimate.headingDegrees, ...
                rotationDegrees, AbsTol=1.0);
        end

        function lateralReflectionChangesOnlyHeadingAndOffsetSigns( ...
                testCase)
            [x, y, azimuth, weight] = localSeedGeometry();
            rotationDegrees = 31.0;
            [rotatedX, rotatedY] = localRotatePoints( ...
                x, y, rotationDegrees);
            original = estimateRoadHeadingFromFeatureSeeds( ...
                rotatedX, rotatedY, azimuth, weight);
            reflected = estimateRoadHeadingFromFeatureSeeds( ...
                rotatedX, -rotatedY, azimuth, weight);

            testCase.verifyEqual( ...
                reflected.headingDegrees, ...
                -original.headingDegrees, AbsTol=1.0);
            testCase.verifyEqual( ...
                reflected.offsetM, -original.offsetM, AbsTol=0.05);
            testCase.verifyEqual( ...
                reflected.score, original.score, RelTol=1.0e-12);
        end

        function duplicateAzimuthCannotDominateUniqueSupport(testCase)
            x = linspace(-20.0, 20.0, 80).';
            y = 4.0 * ones(size(x));
            azimuth = (1:numel(x)).';
            weight = 2.0 * ones(size(x));
            duplicateCount = 500;
            x = [x; 6.0 * ones(duplicateCount, 1)];
            y = [y; linspace(-12.0, 12.0, duplicateCount).'];
            azimuth = [azimuth; ...
                1000 * ones(duplicateCount, 1)];
            weight = [weight; 100.0 * ones(duplicateCount, 1)];

            estimate = estimateRoadHeadingFromFeatureSeeds( ...
                x, y, azimuth, weight);

            testCase.verifyTrue(estimate.found);
            testCase.verifyEqual( ...
                estimate.headingDegrees, 0.0, AbsTol=1.0);
        end

        function insufficientUniqueSupportReturnsFallback(testCase)
            x = linspace(-5.0, 5.0, 10).';
            y = 4.0 * ones(size(x));
            azimuth = (1:numel(x)).';
            weight = ones(size(x));

            estimate = estimateRoadHeadingFromFeatureSeeds( ...
                x, y, azimuth, weight);

            testCase.verifyFalse(estimate.found);
            testCase.verifyEqual(estimate.headingDegrees, 0.0);
        end
    end
end


function [x, y, azimuth, weight] = localSeedGeometry()
    roadX = linspace(-30.0, 30.0, 120).';
    roadY = 4.0 + 0.03 * sin(roadX);
    crossY = linspace(-12.0, 12.0, 35).';
    crossX = 7.0 + 0.02 * cos(crossY);
    x = [roadX; crossX];
    y = [roadY; crossY];
    azimuth = (1:numel(x)).';
    weight = [ ...
        2.5 * ones(size(roadX)); ...
        3.0 * ones(size(crossX))];
end


function [rotatedX, rotatedY] = localRotatePoints(x, y, angleDegrees)
    cosine = cosd(angleDegrees);
    sine = sind(angleDegrees);
    rotatedX = cosine * x - sine * y;
    rotatedY = sine * x + cosine * y;
end
