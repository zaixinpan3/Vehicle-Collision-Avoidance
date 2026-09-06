classdef nrmmModelFormulationTest < matlab.unittest.TestCase
    % nrmmModelFormulationTest Verify the measured-input NRMM target realization.
    %
    % The target state is x_T = [rho; q; s] in ego-frame coordinates. These
    % tests check that the coordinate change from the Sharma target model is
    % exact, that the Lipschitz saturation extension Phi_e agrees with the
    % exact Phi on the certified domain while remaining finite and globally
    % Lipschitz everywhere, and that only measured or estimated ego inputs
    % (body velocity and yaw rate) enter the dynamics.

    properties
        Domain
        PlanarCross
        PhiLipschitz
    end

    properties (TestParameter)
        derivativeState = struct( ...
            "moving", [3; 1; 12; 1; 0.2; 0.4], ...
            "zeroSpeed", [3; 1; 0; 0; 2; -1], ...
            "highSpeed", [3; 1; 100; -40; 2; 1], ...
            "highAcceleration", [3; 1; 12; 1; 30; -40]);
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            repoRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repoRoot, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repoRoot, "estimator")));
        end
    end

    methods (TestMethodSetup)
        function loadCertifiedDomain(testCase)
            design = synthesizeNrmmObserverGains(nrmmTrackingConfig());
            testCase.Domain = design.target.domain;
            testCase.PhiLipschitz = design.target.lipschitzCertificate.phi;
            testCase.PlanarCross = [0.0, -1.0; 1.0, 0.0];
        end
    end

    methods (Test)
        function requestingDiagnosticsPreservesTargetDynamics(testCase, derivativeState)
            ego = struct("bodyVelocity", [10; 0.5], "yawRate", 0.12);

            derivativeOnly = nrmmTargetTrackerDerivative( ...
                derivativeState, ego, testCase.Domain);
            [derivative, estimate] = nrmmTargetTrackerDerivative( ...
                derivativeState, ego, testCase.Domain);

            testCase.verifyEqual(derivativeOnly, derivative, AbsTol=0.0);
            testCase.verifyEqual(estimate.relativeVelocity, derivative(1:2), AbsTol=0.0);
            testCase.verifyTrue(all(isfinite(derivativeOnly)));
        end

        function transformedTargetMapIsSo2Equivariant(testCase)
            angle = 0.63;
            rotation = [cos(angle), -sin(angle); ...
                sin(angle), cos(angle)];
            state = [5.0; -2.0; 12.0; 2.0; 0.2; 0.5];
            ego = struct("bodyVelocity", [8.0; 0.4], ...
                "yawRate", 0.12);
            rotatedState = [rotation*state(1:2); ...
                rotation*state(3:4); rotation*state(5:6)];
            rotatedEgo = struct( ...
                "bodyVelocity", rotation*ego.bodyVelocity, ...
                "yawRate", ego.yawRate);
            derivative = nrmmTargetTrackerDerivative( ...
                state, ego, testCase.Domain);
            rotatedDerivative = nrmmTargetTrackerDerivative( ...
                rotatedState, rotatedEgo, testCase.Domain);
            expected = [rotation*derivative(1:2); ...
                rotation*derivative(3:4); ...
                rotation*derivative(5:6)];

            testCase.verifyEqual(rotatedDerivative, expected, ...
                AbsTol=1.0e-12);
        end

        function transformedDynamicsMatchSharmaModel(testCase)
            % The exact Sharma target (constant scalar acceleration, constant
            % sideslip) and a maneuvering analytic ego with nonzero yaw rate,
            % jerk, and yaw acceleration are propagated in inertial
            % coordinates. The central difference of x_T(t) = [rho; q; s]
            % must match nrmmTargetTrackerDerivative even though the ego
            % struct carries only the measured body velocity and yaw rate.
            domain = testCase.Domain;
            trajectory = localAnalyticTrajectories(domain.rearAxleDistance);
            sampleTime = 1.7;
            stepSize = 1.0e-4;

            stateNow = localTransformedState(trajectory, sampleTime);
            statePlus = localTransformedState(trajectory, ...
                sampleTime+stepSize);
            stateMinus = localTransformedState(trajectory, ...
                sampleTime-stepSize);
            centralDifference = (statePlus-stateMinus)/(2.0*stepSize);

            ego = struct( ...
                "bodyVelocity", trajectory.egoBodyVelocity(sampleTime), ...
                "yawRate", trajectory.egoYawRate(sampleTime));
            [stateDerivative, estimate] = nrmmTargetTrackerDerivative( ...
                stateNow, ego, domain);

            testCase.verifyFalse(estimate.saturationActive, ...
                "The analytic target must lie inside the certified domain.");
            testCase.verifyTrue(estimate.operatingDomainValid);
            testCase.verifyEqual(stateDerivative, centralDifference, ...
                AbsTol=1.0e-6);
        end

        function extendedPhiMatchesExactPhiOnDomain(testCase)
            % On the certified operating set the saturation extension must
            % reproduce the raw formula Phi = -Omega^2*q + 3*A*Omega*J*q/|q|.
            domain = testCase.Domain;
            planarCross = testCase.PlanarCross;
            restingEgo = struct("bodyVelocity", zeros(2, 1), "yawRate", 0.0);
            velocitySamples = [12.0, 15.0, -10.5, 0.0; ...
                1.0, -3.0, 4.0, 18.0];
            accelerationSamples = [0.4, -1.5, 1.0, 2.0; ...
                0.3, 0.8, -1.0, 0.5];

            for sampleIdx = 1:size(velocitySamples, 2)
                q = velocitySamples(:, sampleIdx);
                s = accelerationSamples(:, sampleIdx);
                state = [5.0; -2.0; q; s];

                [stateDerivative, estimate] = nrmmTargetTrackerDerivative( ...
                    state, restingEgo, domain);

                speed = norm(q);
                scalarAcceleration = (q.'*s)/speed;
                yawRate = ((planarCross*q).'*s)/speed^2;
                phiExact = -yawRate^2*q ...
                    + 3.0*scalarAcceleration*yawRate*planarCross*q/speed;

                testCase.verifyFalse(estimate.saturationActive);
                testCase.verifyEqual(estimate.phiValue, phiExact, ...
                    AbsTol=1.0e-12);
                testCase.verifyEqual(stateDerivative(5:6), phiExact, ...
                    AbsTol=1.0e-12);
            end
        end

        function extendedPhiIsFiniteBelowSpeedMinimum(testCase)
            % Phi_e must evaluate without error at q = 0 and for any speed
            % below the certified minimum, flagging saturation and an
            % invalid speed domain instead of dividing by |q|.
            domain = testCase.Domain;
            restingEgo = struct("bodyVelocity", zeros(2, 1), "yawRate", 0.0);
            lowSpeedVelocities = [zeros(2, 1), [1.0e-9; 0.0], ...
                0.5*domain.speedMinimum*[cos(0.4); sin(0.4)]];

            for sampleIdx = 1:size(lowSpeedVelocities, 2)
                state = [3.0; 1.0; lowSpeedVelocities(:, sampleIdx); ...
                    2.0; -1.0];

                [stateDerivative, estimate] = nrmmTargetTrackerDerivative( ...
                    state, restingEgo, domain);

                testCase.verifyTrue(all(isfinite(stateDerivative)));
                testCase.verifyTrue(all(isfinite(estimate.phiValue)));
                testCase.verifyTrue(estimate.saturationActive);
                testCase.verifyFalse(estimate.speedDomainValid);
                testCase.verifyFalse(estimate.operatingDomainValid);
            end
        end

        function extensionSatisfiesSampledLipschitzBound(testCase)
            % Sampled difference quotients of Phi_e over (q, s) pairs drawn
            % inside and far outside the certified domain, including tiny
            % |q|, must stay below a fixed multiple of the certified global
            % Lipschitz constant L_Phi.
            domain = testCase.Domain;
            lipschitzCeiling = 3.0*testCase.PhiLipschitz;
            rng(42, "twister");
            numberOfPairs = 300;
            velocityScales = [0.001, 0.1, 1.0, 3.0];
            maximumQuotient = 0.0;

            for pairIdx = 1:numberOfPairs
                scaleFirst = velocityScales(mod(pairIdx, 4)+1);
                scaleSecond = velocityScales(mod(pairIdx+1, 4)+1);
                firstPoint = [scaleFirst*10.0*randn(2, 1); 4.0*randn(2, 1)];
                if mod(pairIdx, 2) == 0
                    % Distant pairs probe the global bound.
                    secondPoint = [scaleSecond*10.0*randn(2, 1); ...
                        4.0*randn(2, 1)];
                else
                    % Nearby pairs probe the local slope.
                    secondPoint = firstPoint+1.0e-4*randn(4, 1);
                end
                separation = norm(firstPoint-secondPoint);
                if separation == 0.0
                    continue
                end
                phiDifference = norm( ...
                    localPhiExtension(firstPoint, domain) ...
                    - localPhiExtension(secondPoint, domain));
                maximumQuotient = max(maximumQuotient, ...
                    phiDifference/separation);
            end

            testCase.verifyLessThanOrEqual(maximumQuotient, ...
                lipschitzCeiling);
        end

        function inverseTransformReconstructsSharmaStates(testCase)
            % For an in-domain state the estimate must return the exact
            % inverse-transform reconstructions of the Sharma target states.
            domain = testCase.Domain;
            planarCross = testCase.PlanarCross;
            restingEgo = struct("bodyVelocity", zeros(2, 1), "yawRate", 0.0);
            q = [11.0; 2.5];
            s = [0.5; 0.9];
            state = [8.0; -4.0; q; s];

            [~, estimate] = nrmmTargetTrackerDerivative( ...
                state, restingEgo, domain);

            speed = norm(q);
            scalarAcceleration = (q.'*s)/speed;
            yawRate = ((planarCross*q).'*s)/speed^2;
            sideslip = asin(domain.rearAxleDistance*yawRate/speed);
            courseAngle = atan2(q(2), q(1));

            testCase.verifyFalse(estimate.saturationActive);
            testCase.verifyLessThan(abs(sideslip), domain.sideslipMaximum);
            testCase.verifyEqual(estimate.targetSpeed, speed, ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(estimate.targetScalarAcceleration, ...
                scalarAcceleration, AbsTol=1.0e-12);
            testCase.verifyEqual(estimate.targetYawRate, yawRate, ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(estimate.targetSideslip, sideslip, ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(estimate.targetCourseAngleEgoFrame, ...
                courseAngle, AbsTol=1.0e-12);
            testCase.verifyEqual(estimate.targetRelativeHeading, ...
                courseAngle-sideslip, AbsTol=1.0e-12);
        end

        function onlyMeasuredEgoInputsAreRead(testCase)
            % The ego struct with only bodyVelocity and yawRate must
            % suffice, and hypothetical higher-order fields (ego yaw
            % acceleration, ego body-acceleration derivative) must be
            % ignored: the outputs are bitwise identical with and without
            % them.
            domain = testCase.Domain;
            state = [18.0; 1.3; 12.0; 1.0; 0.4; 0.3];
            minimalEgo = struct( ...
                "bodyVelocity", [7.5; 0.4], "yawRate", 0.12);
            augmentedEgo = minimalEgo;
            augmentedEgo.yawAcceleration = 0.5;
            augmentedEgo.bodyAccelerationDerivative = [3.0; -2.0];

            [minimalDerivative, minimalEstimate] = ...
                nrmmTargetTrackerDerivative(state, minimalEgo, domain);
            [augmentedDerivative, augmentedEstimate] = ...
                nrmmTargetTrackerDerivative(state, augmentedEgo, domain);

            testCase.verifyEqual(augmentedDerivative, minimalDerivative);
            testCase.verifyEqual(augmentedEstimate, minimalEstimate);
        end

        function malformedDomainRaisesInvalidDomainError(testCase)
            state = [18.0; 1.3; 12.0; 1.0; 0.4; 0.3];
            ego = struct("bodyVelocity", [7.5; 0.4], "yawRate", 0.12);
            expectedError = "nrmmTargetTrackerDerivative:invalidDomain";

            missingField = rmfield(testCase.Domain, "speedMinimum");
            testCase.verifyError(@() nrmmTargetTrackerDerivative( ...
                state, ego, missingField), expectedError);

            nonpositiveSpeed = testCase.Domain;
            nonpositiveSpeed.speedMinimum = 0.0;
            testCase.verifyError(@() nrmmTargetTrackerDerivative( ...
                state, ego, nonpositiveSpeed), expectedError);

            nonfiniteBound = testCase.Domain;
            nonfiniteBound.yawRateMaximum = NaN;
            testCase.verifyError(@() nrmmTargetTrackerDerivative( ...
                state, ego, nonfiniteBound), expectedError);
        end
    end
end

function phiValue = localPhiExtension(point, domain)
% localPhiExtension Evaluate Phi_e at (q, s) through the tracker derivative.
%
% With a resting ego the acceleration-row derivative reduces to Phi_e.
    restingEgo = struct("bodyVelocity", zeros(2, 1), "yawRate", 0.0);
    stateDerivative = nrmmTargetTrackerDerivative( ...
        [0.0; 0.0; point], restingEgo, domain);
    phiValue = stateDerivative(5:6);
end

function trajectory = localAnalyticTrajectories(rearAxleDistance)
% localAnalyticTrajectories Closed-form Sharma target and maneuvering ego.
%
% Target: VC(t) = VC0 + AC*t, psiC(t) = psiC0 + sin(betaC)/lr*(VC0*t +
% AC*t^2/2), course theta = psiC + betaC, so vC = VC*[cos(theta);
% sin(theta)] and aC = AC*[cos(theta); sin(theta)] + VC*Omega*[-sin(theta);
% cos(theta)] with Omega = VC*sin(betaC)/lr. Ego: smooth inertial velocity
% and yaw angle with nonzero yaw rate, yaw acceleration, and jerk.
% Positions are recovered by adaptive quadrature of the velocities.
    initialSpeed = 12.0;
    scalarAcceleration = 0.4;
    sideslip = 0.012;
    initialHeading = 0.3;
    curvatureRate = sin(sideslip)/rearAxleDistance;

    targetSpeed = @(t) initialSpeed+scalarAcceleration*t;
    targetCourse = @(t) initialHeading+sideslip ...
        + curvatureRate*(initialSpeed*t+0.5*scalarAcceleration*t.^2);
    targetYawRate = @(t) targetSpeed(t)*curvatureRate;
    targetVelocity = @(t) targetSpeed(t) ...
        * [cos(targetCourse(t)); sin(targetCourse(t))];
    targetAcceleration = @(t) scalarAcceleration ...
        * [cos(targetCourse(t)); sin(targetCourse(t))] ...
        + targetSpeed(t)*targetYawRate(t) ...
        * [-sin(targetCourse(t)); cos(targetCourse(t))];
    targetPosition = @(t) [40.0; -6.0]+integral(targetVelocity, ...
        0.0, t, ArrayValued=true, AbsTol=1.0e-13, RelTol=1.0e-13);

    egoYaw = @(t) 0.1+0.2*sin(0.5*t);
    egoYawRate = @(t) 0.1*cos(0.5*t);
    egoInertialVelocity = @(t) [8.0+0.5*sin(0.7*t); 1.2*cos(0.3*t)];
    egoPosition = @(t) [5.0; 2.0]+integral(egoInertialVelocity, ...
        0.0, t, ArrayValued=true, AbsTol=1.0e-13, RelTol=1.0e-13);
    egoBodyVelocity = @(t) localRotation(egoYaw(t)).' ...
        * egoInertialVelocity(t);

    trajectory = struct( ...
        "targetVelocity", targetVelocity, ...
        "targetAcceleration", targetAcceleration, ...
        "targetPosition", targetPosition, ...
        "egoYaw", egoYaw, ...
        "egoYawRate", egoYawRate, ...
        "egoPosition", egoPosition, ...
        "egoBodyVelocity", egoBodyVelocity);
end

function state = localTransformedState(trajectory, t)
% localTransformedState Evaluate x_T(t) = [rho; q; s] in ego coordinates.
    rotation = localRotation(trajectory.egoYaw(t));
    relativePosition = rotation.' ...
        * (trajectory.targetPosition(t)-trajectory.egoPosition(t));
    targetVelocity = rotation.'*trajectory.targetVelocity(t);
    targetAcceleration = rotation.'*trajectory.targetAcceleration(t);
    state = [relativePosition; targetVelocity; targetAcceleration];
end

function rotation = localRotation(angle)
    rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
end
