classdef nrmmGeometricCertificateTest < matlab.unittest.TestCase
    % Geometry, incremental bounds and the common target disturbance column.
    properties
        Domain
    end
    properties (TestParameter)
        radialRegion = struct('inverseFloor',1,'physicalSpeed',2,'speedSaturation',3)
        speedMinimum = struct('moving',10,'includingRest',0)
    end
    methods (TestClassSetup)
        function prepareDomain(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"estimator")));
            testCase.Domain = struct("speedMinimum",10,"speedMaximum",20, ...
                "scalarAccelerationMaximum",2,"yawRateMaximum",0.2, ...
                "accelerationNormBound",hypot(2,4),"rearAxleDistance",1.6, ...
                "sideslipMaximum",0.016,"relativePositionMaximum",50);
        end
    end
    methods (Test)
        function regionalEnvelopesCoverCoupledVectorJacobians(testCase,radialRegion)
            excess = localJacobianExcess(testCase.Domain,radialRegion);
            testCase.verifyLessThanOrEqual(max(excess),2e-8);
        end

        function finiteIncrementsAcrossSaturationBoundariesAreCovered(testCase)
            excess = localIncrementExcess(testCase.Domain);
            testCase.verifyLessThanOrEqual(max(excess),1e-10);
        end

        function coupledDerivativesImproveTheSeparatedProductBound(testCase)
            certificate = nrmmObserverCertificate("targetLipschitz",testCase.Domain);
            testCase.verifyLessThan(certificate.phiVelocity,0.7*certificate.separatedPhiVelocity);
        end

        function commonGyroColumnCoversTheWholeMotionBox(testCase,speedMinimum)
            domain = testCase.Domain;
            domain.speedMinimum = speedMinimum;
            metric = [2,0.4,0.3;0.4,2,-0.6;0.3,-0.6,2];
            certificate = nrmmObserverCertificate("targetGyro",metric,2,domain,50);
            norms = localGyroSamples(metric,2,domain,50);
            testCase.verifyLessThanOrEqual(max(norms),certificate.coefficient+1e-10);
            testCase.verifyLessThan(certificate.coefficient,certificate.separatedCoefficient);
        end

        function commonGyroMaximumIsAttainedAndRotationInvariant(testCase)
            metric = [2,0.4,0.3;0.4,2,-0.6;0.3,-0.6,2];
            certificate = nrmmObserverCertificate("targetGyro",metric,2,testCase.Domain,50);
            rotation = [cos(0.73),-sin(0.73);sin(0.73),cos(0.73)];
            state = kron(eye(3),rotation)*certificate.maximizerState;
            column = kron(diag([1,1/2,1/4]),[0,-1;1,0])*state;
            actual = sqrt(column.'*kron(metric,eye(2))*column);
            testCase.verifyEqual(actual,certificate.coefficient,AbsTol=1e-11);
        end

        function changingStateScalingPreservesThePhysicalGyroBound(testCase)
            metric = [2,0.4,0.3;0.4,2,-0.6;0.3,-0.6,2];
            ratio = 3/2;
            transform = diag([1,ratio,ratio^2]);
            first = nrmmObserverCertificate("targetGyro",metric,2,testCase.Domain,50);
            second = nrmmObserverCertificate("targetGyro", ...
                transform*metric*transform,3,testCase.Domain,50);
            testCase.verifyEqual(first.coefficient,second.coefficient,AbsTol=1e-11);
        end

        function independentAccelerationCapRemainsAnOuterBound(testCase)
            domain = testCase.Domain;
            domain.accelerationNormBound = 0.1;
            certificate = nrmmObserverCertificate("targetGyro",eye(3),2,domain,0);
            testCase.verifyEqual(certificate.coefficient,hypot(10,0.1/4),AbsTol=1e-12);
        end

        function indefiniteMetricsCannotCertifyAColumn(testCase)
            testCase.verifyError(@() nrmmObserverCertificate("targetGyro", ...
                diag([1,1,-1]),2,testCase.Domain,50), ...
                "nrmmObserverCertificate:invalidTargetMetric");
        end
    end
end

function excess = localJacobianExcess(domain,region)
    certificate = nrmmObserverCertificate("targetLipschitz",domain);
    stream = RandStream("mt19937ar","Seed",113+region);
    radii = [0.3,0.8]*domain.speedMinimum;
    if region == 2
        radii = domain.speedMinimum+[0.2,0.8]*(domain.speedMaximum-domain.speedMinimum);
    elseif region == 3
        radii = [1.1,3]*domain.speedMaximum;
    end
    excess = zeros(800,1);
    index = 0;
    for radius = radii
        for sample = 1:100
            angle = 2*pi*rand(stream);
            rotation = [cos(angle),-sin(angle);sin(angle),cos(angle)];
            q = radius*rotation(:,1);
            s = 2*domain.accelerationNormBound*randn(stream,2,1);
            jacobian = zeros(2);
            step = 1e-5;
            for axis = 1:2
                delta = step*rotation(:,axis);
                jacobian(:,axis) = rotation.'*(localPhi(q+delta,s,domain) ...
                    -localPhi(q-delta,s,domain))/(2*step);
            end
            rows = index+(1:4);
            excess(rows) = reshape(abs(jacobian)-certificate.velocityJacobianEnvelopes(:,:,region),4,1);
            index = index+4;
        end
    end
end

function excess = localIncrementExcess(domain)
    certificate = nrmmObserverCertificate("targetLipschitz",domain);
    stream = RandStream("mt19937ar","Seed",127);
    excess = zeros(500,1);
    for sample = 1:numel(excess)
        q = 30*randn(stream,2,1);
        s = 8*randn(stream,2,1);
        dq = 30*randn(stream,2,1);
        ds = 8*randn(stream,2,1);
        excess(sample) = norm(localPhi(q+dq,s+ds,domain)-localPhi(q,s,domain)) ...
            -certificate.phiVelocity*norm(dq)-certificate.phiAcceleration*norm(ds);
    end
end

function norms = localGyroSamples(metric,bandwidth,domain,range)
    stream = RandStream("mt19937ar","Seed",131);
    norms = zeros(1000,1);
    scaledMetric = kron(diag([1,1/bandwidth,1/bandwidth^2]) ...
        *metric*diag([1,1/bandwidth,1/bandwidth^2]),eye(2));
    for sample = 1:numel(norms)
        angle = 2*pi*rand(stream);
        direction = [cos(angle);sin(angle)];
        speed = domain.speedMinimum+rand(stream)*(domain.speedMaximum-domain.speedMinimum);
        acceleration = (2*rand(stream)-1)*domain.scalarAccelerationMaximum;
        yaw = (2*rand(stream)-1)*domain.yawRateMaximum;
        rho = randn(stream,2,1);
        rho = range*rand(stream)*rho/norm(rho);
        state = [rho;speed*direction;acceleration*direction+speed*yaw*[-direction(2);direction(1)]];
        norms(sample) = sqrt(state.'*scaledMetric*state);
    end
end

function phi = localPhi(q,s,domain)
    derivative = nrmmTargetTrackerDerivative([zeros(2,1);q;s], ...
        struct("bodyVelocity",zeros(2,1),"yawRate",0),domain);
    phi = derivative(5:6);
end
