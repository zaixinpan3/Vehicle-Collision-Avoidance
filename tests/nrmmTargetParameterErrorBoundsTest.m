classdef nrmmTargetParameterErrorBoundsTest < matlab.unittest.TestCase
    %nrmmTargetParameterErrorBoundsTest NRMM parameter bounds of a target estimate.
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "estimator")));
        end
    end
    methods (Test)
        function sampledTruthsStayInsideTheBounds(testCase)
            % Estimates in a frame that errs by up to the rotation bound; the
            % true velocity and acceleration lie in discs around them.
            stream = RandStream("mt19937ar", Seed=3);
            excess = -Inf;
            for trial = 1:400
                speed = 0.3+12*rand(stream);course = 2*pi*rand(stream)-pi;
                velocity = speed*[cos(course); sin(course)];
                acceleration = 2*(2*rand(stream, 2, 1)-1);
                velocityRadius = 1.5*rand(stream);accelerationRadius = 0.5*rand(stream);
                rotationRadius = 0.2*rand(stream);
                bounds = nrmmTargetParameterErrorBounds(velocity, acceleration, ...
                    velocityRadius, accelerationRadius, rotationRadius);
                for sample = 1:50
                    rotation = rotationRadius*(2*rand(stream)-1);
                    frame = [cos(rotation), -sin(rotation); sin(rotation), cos(rotation)];
                    direction = 2*pi*rand(stream);
                    trueVelocity = frame*(velocity ...
                        +velocityRadius*sqrt(rand(stream))*[cos(direction); sin(direction)]);
                    direction = 2*pi*rand(stream);
                    trueAcceleration = frame*(acceleration ...
                        +accelerationRadius*sqrt(rand(stream))*[cos(direction); sin(direction)]);
                    trueSpeed = norm(trueVelocity);
                    if trueSpeed == 0, continue; end
                    tangent = trueVelocity/trueSpeed;normal = [-tangent(2); tangent(1)];
                    courseError = atan2(sin(atan2(trueVelocity(2), trueVelocity(1))-course), ...
                        cos(atan2(trueVelocity(2), trueVelocity(1))-course));
                    curvature = dot(trueAcceleration, normal)/trueSpeed^2;
                    excess = max([excess; ...
                        abs(trueSpeed-speed)-bounds.speedErrorBound; ...
                        abs(courseError)-bounds.courseErrorBound; ...
                        abs(dot(trueAcceleration, tangent)-dot(velocity, acceleration)/speed)- ...
                            bounds.speedRateErrorBound; ...
                        bounds.curvatureInterval(1)-curvature; curvature-bounds.curvatureInterval(2)]);
                end
            end
            testCase.verifyLessThanOrEqual(excess, 1e-9);
        end
        function aBallContainingRestLeavesCourseAndCurvatureOpen(testCase)
            bounds = nrmmTargetParameterErrorBounds([0.2; 0.1], [0.5; -0.2], 0.3, 0.1, 0.05);
            testCase.verifyEqual(bounds.speedErrorBound, 0.3);
            testCase.verifyEqual(bounds.courseErrorBound, pi);
            testCase.verifyEqual(bounds.speedRateErrorBound, 0.1+2*norm([0.5; -0.2]));
            testCase.verifyEqual(bounds.curvatureInterval, [-Inf; Inf]);
        end
    end
end
