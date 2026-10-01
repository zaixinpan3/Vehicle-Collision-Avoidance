classdef sharmaBaselineObserverTest < matlab.unittest.TestCase
% sharmaBaselineObserverTest Tests for the Sharma (2026) comparator observer.
%
% The comparator reproduces the multistage high-gain observer of Sharma, Alai
% and Rajamani (Transp. Res. Part C 182, 2026). These tests check the companion
% form against an exact third derivative of the relative position under the
% paper's Assumption 1, the Theorem 1 gain design, and the sampled runtime.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            repoRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repoRoot, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repoRoot, "estimator"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repoRoot, "scripts")));
        end
    end

    methods (Test)
        function correctedCompanionFormMatchesExactThirdDerivative(testCase)
            % Ego with constant body acceleration and constant yaw rate satisfies
            % Assumption 1 exactly while its body velocity still changes, so the
            % omitted term psiEdot*J*vDot is nonzero. The target is an exact
            % Sharma-model target (constant A, constant sideslip).
            [state, input, thirdDerivative] = localAssumptionOneTruth(1.7);
            corrected = sharmaNrmmCompanionDerivative(state, input, "corrected", 1.0);
            published = sharmaNrmmCompanionDerivative(state, input, "published", 1.0);
            testCase.verifyEqual(corrected([3, 6]), thirdDerivative, ...
                "The corrected companion map must reproduce the exact third derivative.", AbsTol=2.0e-4);
            vDot = input.bodyAcceleration - input.yawRate*[0.0, -1.0; 1.0, 0.0]*input.bodyVelocity;
            omittedTerm = input.yawRate*[0.0, -1.0; 1.0, 0.0]*vDot;
            testCase.verifyEqual(corrected([3, 6]) - published([3, 6]), omittedTerm, ...
                "The variants must differ exactly by psiEdot*J*vDot.", AbsTol=1.0e-12);
            testCase.verifyGreaterThan(norm(published([3, 6]) - thirdDerivative), 0.05, ...
                "The printed equations must be measurably inconsistent when vDot is nonzero.");
            testCase.verifyEqual(corrected([1, 2, 4, 5]), state([2, 3, 5, 6]), AbsTol=0.0);
        end

        function publishedCompanionFormIsExactWhenBodyVelocityIsConstant(testCase)
            % Constant-speed circular ego motion has vDot = 0, so the printed
            % equations are exact there; both variants must agree with the truth.
            [state, input, thirdDerivative] = localAssumptionOneTruth(0.0);
            published = sharmaNrmmCompanionDerivative(state, input, "published", 1.0);
            testCase.verifyEqual(published([3, 6]), thirdDerivative, AbsTol=2.0e-4);
        end

        function designSatisfiesTheoremOne(testCase)
            cfg = nrmmTrackingConfig();
            design = sharmaMultistageObserverDesign(cfg);
            lmi = design.lmi;
            closedLoop = lmi.systemMatrix - lmi.gainSeed*lmi.outputMatrix;
            residual = closedLoop.'*lmi.lyapunovMatrix + lmi.lyapunovMatrix*closedLoop ...
                + lmi.lambda*eye(6);
            testCase.verifyLessThanOrEqual(max(eig((residual + residual.')/2.0)), 1.0e-6, ...
                "The recovered (P, K) must satisfy the Theorem 1 Lyapunov inequality.");
            testCase.verifyGreaterThan(min(eig(lmi.lyapunovMatrix)), 0.0);
            testCase.verifyGreaterThan(design.ego.theta, design.ego.thetaThreshold);
            testCase.verifyGreaterThan(design.target.theta, design.target.thetaThreshold);
            testCase.verifyEqual(design.target.thetaThreshold, ...
                2.0*design.target.lipschitz*lmi.lyapunovMaximumEigenvalue/lmi.lambda, RelTol=1.0e-12);
            scaling = blkdiag(diag([design.target.theta, design.target.theta^2, design.target.theta^3]), ...
                diag([design.target.theta, design.target.theta^2, design.target.theta^3]));
            testCase.verifyEqual(design.target.gain, scaling*lmi.gainSeed, RelTol=1.0e-12);
            testCase.verifyTrue(design.yaw.hInfinity.feasible);

            reference = synthesizeNrmmObserverGains(cfg);
            matched = sharmaMultistageObserverDesign(cfg, Theta="matched", ReferenceDesign=reference);
            expectedRate = reference.target.bandwidth*min(abs(real(reference.target.closedLoopPoles)));
            testCase.verifyEqual(matched.target.slowestPhysicalRate, expectedRate, RelTol=1.0e-9);
            testCase.verifyEqual(matched.ego.slowestPhysicalRate, reference.velocity.gain, RelTol=1.0e-9);
        end

        function targetLipschitzMaximizesEquation73OverSigns(testCase)
            cfg = nrmmTrackingConfig();
            design = sharmaMultistageObserverDesign(cfg);
            c1 = design.target.domain.yawRateMaximum;
            c2 = cfg.ego.domain.yawRateMaximum;
            expected = 0.0;
            for s1 = [-1.0, 1.0]
                for s2 = [-1.0, 1.0]
                    a = s1*c1;
                    b = s2*c2;
                    expected = max(expected, sqrt(2.0*(2.0*a^2 - 6.0*a*b + 3.0*b^2)^2 ...
                        + 18.0*(a - b)^2 + 2.0*b^2*(2.0*a^2 - 3.0*a*b + b^2)^2));
                end
            end
            testCase.verifyEqual(design.target.lipschitz, expected, RelTol=1.0e-12);
        end

        function noiseFreeStraightRoadConverges(testCase)
            result = runObserverComparisonScenario(Scenario="straightOncoming", NoiseModel="none", ...
                Duration=4.0, Estimators=["structured", "sharmaMatchedPredictor"], Report=false);
            testCase.verifyTrue(result.truthDomainValid);
            for run = result.runs
                finalError = norm(run.estimate.targetState(end, 1:2) ...
                    - result.truth.targetTransformedState(end, 1:2));
                testCase.verifyLessThan(finalError, 0.02, run.name + " must converge without noise.");
                testCase.verifyFalse(run.metrics.diverged);
                testCase.verifyTrue(run.metrics.completed);
            end
        end

        function runtimeOutputCarriesTheComparisonContract(testCase)
            cfg = nrmmTrackingConfig();
            design = sharmaMultistageObserverDesign(cfg);
            options = struct("initialTime", 0.0, "egoInitialPosition", [0.0; 0.0], ...
                "egoInitialYaw", 0.0, "egoInitialBodyVelocity", [15.0; 0.0], ...
                "targetInitialState", [30.0; 0.0; 15.3; 0.0; 0.0; 0.0]);
            runtime = sharmaMultistageObserverRuntime("initialize", cfg, options, design);
            frame = struct("time", 0.0, "xGps", 0.0, "yGps", 0.0, "vxGps", 15.0, "vyGps", 0.0, ...
                "longitudinalAcceleration", 0.0, "lateralAcceleration", 0.0, "yawRateMeasured", 0.0, ...
                "radarRelativePosition", [30.0, 0.0], "radarDetectionAvailable", true);
            initialOutput = sharmaMultistageObserverRuntime("output", runtime, frame);
            testCase.verifyEqual(initialOutput.targetState, options.targetInitialState, ...
                "The companion mapping must invert exactly at the initial state.", AbsTol=1.0e-9);
            testCase.verifyEqual(initialOutput.targetEstimate.targetSpeed, 15.3, AbsTol=1.0e-9);
            [runtime, output] = sharmaMultistageObserverRuntime("step", runtime, frame);
            testCase.verifyEqual(runtime.currentTime, cfg.runtime.samplePeriod, AbsTol=1.0e-12);
            for field = ["egoState", "egoYaw", "egoYawRate", "egoBodyVelocity", "targetState", ...
                    "targetEstimate", "integrationStep", "diverged"]
                testCase.verifyTrue(isfield(output, field), "Missing output field " + field);
            end
            for field = ["relativeVelocity", "targetVelocityInertial", "targetCourseAngleInertial", ...
                    "targetSpeed", "targetYawRate", "targetHeadingInertial"]
                testCase.verifyTrue(isfield(output.targetEstimate, field), ...
                    "Missing target field " + field);
            end
            testCase.verifyEqual(output.targetState(1:2), [30.0 + 0.3*cfg.runtime.samplePeriod; 0.0], ...
                AbsTol=1.0e-3);
            testCase.verifyEqual(output.targetEstimate.targetSpeed, 15.3, AbsTol=1.0e-3);
            frame.time = cfg.runtime.samplePeriod;
            frame.radarRelativePosition(:) = NaN;
            frame.radarDetectionAvailable = false;
            [~, coasting] = sharmaMultistageObserverRuntime("step", runtime, frame);
            testCase.verifyFalse(coasting.radarDetectionAvailable);
            testCase.verifyTrue(all(isfinite(coasting.targetState)));
        end
    end
end

function [state, input, thirdDerivative] = localAssumptionOneTruth(longitudinalAcceleration)
% localAssumptionOneTruth Exact Sharma companion state and third derivative.
%
% Ego: constant body acceleration [ax; ay] and constant yaw rate omega, so
% Assumption 1 holds exactly; inertial velocity and position are closed-form.
% Target: constant scalar acceleration A and constant sideslip betaC; its
% position is obtained by adaptive quadrature. The relative position rate
% rDot = q - v - omega*J*r is exact kinematics; its second time derivative
% (the third derivative of r) is taken with a five-point stencil.
    omega = 0.15;
    bodyAcceleration = [longitudinalAcceleration; 2.25];
    initialInertialVelocity = [15.0; 0.0];
    planarCross = [0.0, -1.0; 1.0, 0.0];
    rotation = @(angle) [cos(angle), -sin(angle); sin(angle), cos(angle)];
    velocityKernel = @(t) (1.0/omega)*[sin(omega*t), -(1.0 - cos(omega*t)); ...
        1.0 - cos(omega*t), sin(omega*t)];
    positionKernel = @(t) (1.0/omega^2)*[1.0 - cos(omega*t), -(omega*t - sin(omega*t)); ...
        omega*t - sin(omega*t), 1.0 - cos(omega*t)];
    egoVelocity = @(t) initialInertialVelocity + velocityKernel(t)*bodyAcceleration;
    egoPosition = @(t) initialInertialVelocity*t + positionKernel(t)*bodyAcceleration;

    targetSpeed0 = 12.5;
    targetAcceleration = 0.4;
    sideslip = 0.01;
    rearAxle = 1.6;
    curvature = sin(sideslip)/rearAxle;
    course = @(t) 0.35 + curvature*(targetSpeed0*t + 0.5*targetAcceleration*t.^2);
    targetSpeed = @(t) targetSpeed0 + targetAcceleration*t;
    targetVelocity = @(t) targetSpeed(t).*[cos(course(t)); sin(course(t))];
    targetPosition = @(t) [25.0; 4.0] + integral(targetVelocity, 0.0, t, ...
        ArrayValued=true, AbsTol=1.0e-13, RelTol=1.0e-13);
    targetAccelerationVector = @(t) targetAcceleration*[cos(course(t)); sin(course(t))] ...
        + targetSpeed(t).^2*curvature*[-sin(course(t)); cos(course(t))];

    relativePosition = @(t) rotation(omega*t).'*(targetPosition(t) - egoPosition(t));
    relativeRate = @(t) rotation(omega*t).'*targetVelocity(t) - rotation(omega*t).'*egoVelocity(t) ...
        - omega*planarCross*relativePosition(t);

    t0 = 1.3;
    h = 2.0e-3;
    samples = zeros(2, 5);
    for k = -2:2
        samples(:, k + 3) = relativeRate(t0 + k*h);
    end
    relativeAcceleration = (samples(:, 1) - 8.0*samples(:, 2) + 8.0*samples(:, 4) - samples(:, 5))/(12.0*h);
    thirdDerivative = (-samples(:, 1) + 16.0*samples(:, 2) - 30.0*samples(:, 3) ...
        + 16.0*samples(:, 4) - samples(:, 5))/(12.0*h^2);

    r = relativePosition(t0);
    rDot = relativeRate(t0);
    state = [r(1); rDot(1); relativeAcceleration(1); r(2); rDot(2); relativeAcceleration(2)];
    input = struct("bodyVelocity", rotation(omega*t0).'*egoVelocity(t0), ...
        "bodyAcceleration", bodyAcceleration, "yawRate", omega);
    % The target acceleration is unused by the companion form itself but kept
    % here to document that the truth is an exact Sharma-model target.
    assert(all(isfinite(targetAccelerationVector(t0))));
end
